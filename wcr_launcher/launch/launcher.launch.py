import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from wcr_launcher.yaml_loader import LauncherConfigurator

def generate_launch_description():

    # Paths and configuration
    pkg_share = get_package_share_path('wcr_description')
    controller_share = get_package_share_path('wcr_control')
    config = LauncherConfigurator()
    rviz_config_path = os.path.join(pkg_share, 'rviz', config.rviz_config)

    # Robot State Publisher (CENTRAL)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=config.namespace,
        output="both",
        parameters=[{
            'robot_description': config.build_robot_description(),
            'use_sim_time': config.use_sim_time,
            "namespace" : config.namespace,
        }]
    )

    # Include Gazebo launch (conditional)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ),
        condition=IfCondition(str(config.sim).lower())
    )

    # Include controllers launch, started once the robot is spawned and the sim is running
    # (Gazebo blocks while loading meshes and the Humble spawner gives up after 10 s)
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, 'launch', 'controller.launch.py')
        )
    )
    wait_for_sim = Node(
        package="wcr_climb",
        executable="wait_for_sim",
        name="wait_for_sim",
        namespace=config.namespace,
        output="screen",
        parameters=[{'use_sim_time': config.use_sim_time}],
    )
    start_controllers = RegisterEventHandler(
        OnProcessExit(target_action=wait_for_sim, on_exit=[control_launch])
    )

    controller_launch = Node(
        package="wcr_controllers",
        executable="inv_kin_controller",
        name="inv_kin_controller",
        namespace=config.namespace,
        parameters=[{
            'use_sim_time': config.use_sim_time,
            # the node builds absolute topic names from this: <ns>/cmd_vel, <ns>/driving_velocity_controller/...
            'robot_namespace': f'/{config.namespace}',
        }],
    )

    # ROSBridge server (for Visualization)
    rosbridge_server = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    # Odom node
    odometry = Node(
        package="wcr_odometry",
        executable="odometry",
        name="odometry",
        namespace=config.namespace,
        parameters=[{
            'use_sim_time': config.use_sim_time
        }],
    )

    # EDF adhesion force (thrust + skirt suction) from the surface angle
    adhesion_mode = str(config.get_launch_param('adhesion', 'auto'))
    adhesion = Node(
        package="wcr_climb",
        executable="adhesion_controller",
        name="adhesion_controller",
        namespace=config.namespace,
        output="screen",
        parameters=[{
            'use_sim_time': config.use_sim_time,
            'mode': adhesion_mode,
            'max_force': float(config.get_robot_param('max_thrust_cmd_edf', 45.0)),
            'fixed_force': float(config.get_launch_param('adhesion_force', 30.0)),
            'startup_force': float(config.get_launch_param('adhesion_force', 30.0)),
        }],
        condition=IfCondition(str(adhesion_mode != 'off').lower())
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=config.namespace,
        output="screen",
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': config.use_sim_time}],
        condition=IfCondition(str(config.rviz).lower())
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_launch,
        wait_for_sim,
        start_controllers,
        controller_launch,
        rosbridge_server,
        odometry,
        adhesion,
        rviz_node
    ])