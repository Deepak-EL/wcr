from launch import LaunchDescription
from launch_ros.actions import Node
from wcr_launcher.yaml_loader import LauncherConfigurator # type: ignore

def generate_launch_description():

    config = LauncherConfigurator()
    namespace = config.namespace
    controller_manager_name = f"/{config.namespace}/controller_manager"

    #control_node = Node(
    #    package="controller_manager",
    #    executable="ros2_control_node",
    #    namespace=namespace,
    #    output="both",
    #    parameters=[config.controllers_yaml],
    #    remappings=[
    #        ("~/robot_description", f"/{config.namespace}/robot_description")
    #    ]
    #)

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", "--controller-manager", controller_manager_name, "--switch-timeout", "60.0",
                   # big visual meshes can block the controller manager >10 s at startup
                   "--service-call-timeout", "60.0"],
    )

    steering_position_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["steering_position_controller", "--controller-manager", controller_manager_name, "--switch-timeout", "60.0",
                   # big visual meshes can block the controller manager >10 s at startup
                   "--service-call-timeout", "60.0"],
    )

    driving_velocity_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["driving_velocity_controller", "--controller-manager", controller_manager_name, "--switch-timeout", "60.0",
                   # big visual meshes can block the controller manager >10 s at startup
                   "--service-call-timeout", "60.0"],
    )

    propeller_effort_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["propeller_effort_controller", "--controller-manager", controller_manager_name, "--switch-timeout", "60.0",
                   # big visual meshes can block the controller manager >10 s at startup
                   "--service-call-timeout", "60.0"],
    )

    return LaunchDescription([
        joint_state_broadcaster_node,
        steering_position_controller_node,
        driving_velocity_controller_node,
        propeller_effort_controller_node,
    ])