import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from wcr_launcher.yaml_loader import LauncherConfigurator # type: ignore


def generate_launch_description():
    
    # Paths
    pkg_share = get_package_share_path('wcr_description')
    launcher_share = get_package_share_path('wcr_launcher')
    
    # Extract launch parameters with defaults
    config = LauncherConfigurator()
    namespace = config.namespace
    #edf_thrust_coefficient = config.thrust_coefficient_edf
    #propeller_thrust_coefficient = config.thrust_coefficient_propeller
    #edf_max_thrust_cmd = config.max_thrust_cmd_edf
    #propeller_max_thrust_cmd = config.max_thrust_cmd_propeller
    world_file = config.world_file
    spawn_x = config.spawn_x
    spawn_y = config.spawn_y
    spawn_roll = config.spawn_roll
    spawn_pitch = config.spawn_pitch
    spawn_yaw = config.spawn_yaw
    spawn_z_height = config.spawn_z_height
    gz_args = config.gz_args
    
    # Build paths
    world_path = os.path.join(pkg_share, 'worlds', world_file)

    # Server and GUI run as separate processes: the GUI occasionally segfaults at startup
    # on WSLg (Qt/GLX context teardown), and it must not take the simulation down with it.
    args = gz_args.split()
    headless = '-s' in args
    gui_args = []
    if '--render-engine-gui' in args:
        i = args.index('--render-engine-gui')
        gui_args = args[i:i + 2]
        del args[i:i + 2]
    verbosity = [a for a in args if a.startswith('-v')]
    server_args = ' '.join(args + ([] if headless else ['-s']))

    # Include Gazebo server (ros_gz_sim)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [f'"{world_path}"', TextSubstitution(text=f' {server_args}')],
            'on_exit_shutdown': 'true',
            'use_sim_time': str(config.use_sim_time).lower(),
        }.items()
    )

    # gui_software_gl: render the GUI window with Mesa's CPU renderer. On WSLg the GPU
    # path (D3D12) sometimes presents half-drawn frames (flicker); the server and its
    # sensors keep using the GPU either way.
    gui_env = {'LIBGL_ALWAYS_SOFTWARE': '1'} if config.get_launch_param('gui_software_gl', False) else {}
    # gui_adapter: D3D12 adapter for the GUI window only (e.g. "Intel", the GPU WSLg itself uses,
    # avoiding cross-GPU frame hand-off); empty = same as the server (run_sim.sh: NVIDIA)
    if config.get_launch_param('gui_adapter', ''):
        gui_env['MESA_D3D12_DEFAULT_ADAPTER_NAME'] = str(config.get_launch_param('gui_adapter'))
    gazebo_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g', *verbosity, *gui_args],
        additional_env=gui_env,
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        condition=UnlessCondition(str(headless).lower()),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=namespace,
        arguments=[
            '-topic', f'/{namespace}/robot_description',
            '-name', namespace,
            '-x', str(spawn_x),
            '-y', str(spawn_y),
            '-z', str(spawn_z_height),
            '-R', str(spawn_roll),
            '-P', str(spawn_pitch),
            '-Y', str(spawn_yaw),
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[f'/world/{config.world_name}/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        remappings=[(f'/world/{config.world_name}/clock', '/clock')],
    )

    # Bridge (TODO: ako bude potrebe)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(launcher_share, 'config', 'bridge.yaml')}],
        namespace=namespace
    )

    return LaunchDescription([
        gazebo,
        gazebo_gui,
        spawn,
        clock_bridge,
        bridge,
    ])
