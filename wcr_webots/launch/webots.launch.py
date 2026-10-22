"""WCR in Webots (Windows) + ROS 2 (WSL): sensors, drive and RViz.

  ros2 launch wcr_webots webots.launch.py [world:=winter_dam|gravity_dam|goldeneye_dam|goldeneye_gorge]
                                          [rviz:=true] [start_webots:=true] [layout:=fixed|random seed:=N]
                                          [foxglove:=false]   (rviz:=false foxglove:=true if WSLg flickers)
                                          [nav:=false] [mission:=false area:=demo]   (Phase 4)
                                          [model:=detailed|simple]   (simple = the simplified CAD look)

- Copies webots/worlds/<world>.wbt to webots/worlds/_ros_<world>.wbt with the robot's controller
  set to <extern> and starts Webots on Windows with it (start_webots:=false if it is already
  open with that world).
- webots_ros2_driver connects to the robot over TCP (WSL -> Windows) and loads
  resource/wcr_webots_driver.urdf: /scan, /wall_camera, /front_camera, /imu and the WcrDriver
  plugin (/cmd_vel, /odom, /joint_states, /ground_truth, /adhesion, TF).
- robot_state_publisher (resource/wcr.urdf) and RViz (resource/wcr.rviz).
- inspection_detector (Phase 3): defects of the layout in the wall camera's view -> detections,
  RViz markers, annotated wall camera image, results/inspection_log_<world>.csv. layout:=random
  re-places the defects and obstacles (tools/inspection.py --seed) in the world copy.
- wall_frames: world -> map (the face) -> odom (start pose).
- nav:=true (Phase 4): robot_localization EKF (odom -> base_link), map_server with the wall map
  (tools/wall_map.py), Nav2 (config/nav2_params.yaml). mission:=true adds coverage_mission:
  autonomous lawnmower coverage of the area, /coverage/map, report in results/.
Drive: ros2 run teleop_twist_keyboard teleop_twist_keyboard  (i = up the face, J/L = crab)
"""
import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WEBOTS_HOME_WIN = '/mnt/c/Users/DEEPAK E L/AppData/Local/Programs/Webots'


def setup(context):
    share = Path(get_package_share_directory('wcr_webots'))
    repo = Path(os.path.realpath(share / 'resource' / 'wcr.urdf')).parents[2]   # symlink-install -> checkout
    world = LaunchConfiguration('world').perform(context)
    src = repo / 'webots' / 'worlds' / f'{world}.wbt'
    dst = src.with_name(f'_ros_{world}.wbt')
    results = repo / 'results'
    if LaunchConfiguration('layout').perform(context) == 'random':
        seed = LaunchConfiguration('seed').perform(context)
        layout = results / f'inspection_layout_{world}_seed{seed}.yaml'
        wall_map = results / 'maps' / f'{world}_seed{seed}.yaml'
        subprocess.check_call(['python3', str(repo / 'tools' / 'inspection.py'), '--world', world, '--seed', seed,
                               '--wbt', str(src), '--out', str(dst), '--layout', str(layout),
                               '--map', str(wall_map.with_suffix(''))])
        text = dst.read_text()
    else:
        layout = repo / 'wcr_description' / 'worlds' / 'inspection' / f'{world}.yaml'
        wall_map = repo / 'wcr_description' / 'worlds' / 'maps' / f'{world}.yaml'
        text = src.read_text()
    mission = LaunchConfiguration('mission').perform(context) == 'true'
    nav = mission or LaunchConfiguration('nav').perform(context) == 'true'
    # with Nav2 the robot_localization EKF publishes odom -> base_link instead of the driver
    os.environ['WCR_ODOM_TF'] = '0' if nav else '1'
    speed = float(LaunchConfiguration('speed').perform(context))
    os.environ['WCR_MAX_SPEED'] = str(speed)        # driver: wheel speed limit (0.128 = real motors)
    if 'controllerArgs [ "keyboard" ]' not in text:
        raise RuntimeError(f'{src}: robot controllerArgs not found (regenerate with tools/build_webots.py)')
    text = text.replace('controllerArgs [ "keyboard" ]', 'controller "<extern>"')
    simple = LaunchConfiguration('model').perform(context) == 'simple'
    if simple:              # the same robot drawn with the simplified CAD (tools/build_webots.py --simple)
        text = text.replace('EXTERNPROTO "../protos/Wcr.proto"', 'EXTERNPROTO "../protos/WcrSimple.proto"')
        text = text.replace('\nWcr {', '\nWcrSimple {')
    dst.write_text(text)

    os.environ.setdefault('WEBOTS_HOME', os.environ.get('ROS2_WEBOTS_HOME', WEBOTS_HOME_WIN))
    import webots_ros2_driver.utils as wr_utils
    from webots_ros2_driver.webots_controller import WebotsController     # needs WEBOTS_HOME
    # webots_ros2 takes the Windows host from the resolv.conf nameserver, which is 10.255.255.254
    # with WSL DNS tunnelling; the host that runs Webots is the default gateway.
    gateway = subprocess.check_output(['ip', 'route']).decode().split('default via ')[1].split()[0]
    wr_utils.get_wsl_ip_address = lambda: gateway

    actions = []
    if LaunchConfiguration('start_webots').perform(context) == 'true':
        exe = Path(os.environ['WEBOTS_HOME']) / 'msys64' / 'mingw64' / 'bin' / 'webots.exe'
        win_world = subprocess.check_output(['wslpath', '-w', str(dst)]).decode().strip()
        mode = 'fast' if LaunchConfiguration('fast').perform(context) == 'true' else 'realtime'
        actions.append(ExecuteProcess(cmd=[str(exe), '--port=1234', f'--mode={mode}', win_world],
                                      output='screen', name='webots'))
    actions.append(WebotsController(
        robot_name='wcr', port='1234',
        parameters=[{'robot_description': str(share / 'resource' / 'wcr_webots_driver.urdf'),
                     'use_sim_time': True}],
        respawn=True))
    actions.append(Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': (share / 'resource' / ('wcr_simple.urdf' if simple else 'wcr.urdf')).read_text(),
                     'use_sim_time': True}]))
    actions.append(Node(
        package='wcr_webots', executable='inspection_detector', output='screen',
        parameters=[{'layout': str(layout), 'log_file': str(results / f'inspection_log_{world}.csv'),
                     'use_sim_time': True}]))
    actions.append(Node(                                             # world -> map (face) -> odom (start)
        package='wcr_webots', executable='wall_frames', output='screen',
        parameters=[{'layout': str(layout), 'use_sim_time': True}]))
    if nav:
        from nav2_common.launch import RewrittenYaml
        cfg = share / 'config'
        nav2_params = RewrittenYaml(
            source_file=str(cfg / 'nav2_params.yaml'), convert_types=True,
            param_rewrites={'vx_max': str(speed), 'vx_min': str(-speed), 'vy_max': str(speed),
                            'default_nav_through_poses_bt_xml': str(cfg / 'navigate_through_poses_coverage.xml')}
        ).perform(context)
        actions.append(Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
            parameters=[str(cfg / 'ekf.yaml')]))
        # map server a few seconds after the start-up rush: under load its lifecycle "configure"
        # reply was lost (DDS timeout), it never activated and the global costmap had no map
        actions.append(TimerAction(period=6.0, actions=[
            Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                 parameters=[nav2_params, {'yaml_filename': str(wall_map), 'use_sim_time': True}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
                 output='screen', parameters=[{'use_sim_time': True, 'autostart': True,
                                               'node_names': ['map_server']}])]))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory('nav2_bringup'))
                                              / 'launch' / 'navigation_launch.py')),
            launch_arguments={'params_file': nav2_params, 'use_sim_time': 'true',
                              'autostart': 'true'}.items()))
    if mission:
        actions.append(Node(
            package='wcr_webots', executable='coverage_mission', output='screen',
            parameters=[{'layout': str(layout), 'area': LaunchConfiguration('area').perform(context),
                         'speed': speed, 'report_dir': str(results), 'use_sim_time': True}]))
    actions.append(Node(
        package='rviz2', executable='rviz2', output='screen', condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', str(share / 'resource' / 'wcr.rviz')], parameters=[{'use_sim_time': True}]))
    # Foxglove (Windows app or app.foxglove.dev in a browser) -> ws://localhost:8765: a native
    # Windows viewer, unaffected by WSLg "copy mode" flicker. Layout: resource/wcr_foxglove.json
    actions.append(Node(
        package='foxglove_bridge', executable='foxglove_bridge', output='screen',
        condition=IfCondition(LaunchConfiguration('foxglove')),
        parameters=[{'port': 8765, 'use_sim_time': True, 'send_buffer_limit': 50000000,
                     'capabilities': ['clientPublish', 'parameters', 'services', 'assets']}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='winter_dam',
                              description='winter_dam | gravity_dam | goldeneye_dam | goldeneye_gorge'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='false',
                              description='start foxglove_bridge on ws://localhost:8765 (use with rviz:=false)'),
        DeclareLaunchArgument('layout', default_value='fixed',
                              description='fixed (wcr_description/worlds/inspection/<world>.yaml) | random'),
        DeclareLaunchArgument('seed', default_value='1', description='seed for layout:=random'),
        DeclareLaunchArgument('start_webots', default_value='true'),
        DeclareLaunchArgument('speed', default_value='0.12',
                              description='max driving speed [m/s]; 0.128 is the real motors, more = faster demo'),
        DeclareLaunchArgument('model', default_value='detailed',
                              description='robot look: detailed (original CAD) or simple (cad/WCR_Simple); '
                                          'physics, sensors and control are the same'),
        DeclareLaunchArgument('fast', default_value='false',
                              description='run Webots as fast as possible (ROS follows /clock), e.g. for missions'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='Phase 4: EKF + wall map + Nav2 (goals from RViz / Foxglove)'),
        DeclareLaunchArgument('mission', default_value='false',
                              description='Phase 4: nav + autonomous coverage of the area, report in results/'),
        DeclareLaunchArgument('area', default_value='demo',
                              description="coverage area: 'demo' (layout demo_area) or 's0,s1,t0,t1' in map metres"),
        OpaqueFunction(function=setup),
    ])
