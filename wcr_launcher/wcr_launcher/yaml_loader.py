import os
import re
import yaml
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


class LauncherConfigurator:
    
    def __init__(self):
        self.launcher_share = get_package_share_path('wcr_launcher')
        self._load_configs()
    
    def _load_configs(self):
        launch_params_yaml = os.path.join(self.launcher_share, 'config', 'launch_params.yaml')
        with open(launch_params_yaml, 'r') as f:
            self.launch_params = yaml.safe_load(f) or {}

        robot_params_yaml = os.path.join(self.launcher_share, 'config', 'robot_params.yaml')
        with open(robot_params_yaml, 'r') as f:
            self.robot_params = yaml.safe_load(f) or {}
    
    def get_launch_param(self, key, default=None):
        # WCR_<KEY> environment variables override launch_params.yaml, e.g.
        #   WCR_RVIZ=false WCR_GZ_ARGS="-r -s" ros2 launch wcr_launcher launcher.launch.py
        env = os.environ.get(f'WCR_{key.upper()}')
        if env is not None:
            return yaml.safe_load(env)
        return self.launch_params.get(key, default)
    
    def get_robot_param(self, key, default=None):
        return self.robot_params.get(key, default)
    
    def get_all_robot_params(self):
        return self.robot_params
    
    def build_robot_description(self):
        """Build robot_description from xacro with all parameters"""
        pkg_share = get_package_share_path('wcr_description')
        urdf_path = os.path.join(pkg_share, 'urdf', self.urdf_file)
        
        xacro_args = [
            "xacro", " ", f'"{urdf_path}"',
            " ", f"variant:={self.variant}",
            " ", f"sim:={self.sim}",
            " ", f"propeller_control:={self.propeller_control}",
            " ", f"namespace:={self.namespace}",
        ]
        
        for key, value in self.get_all_robot_params().items():
            if isinstance(value, bool):
                value = str(value).lower()
            xacro_args.extend([" ", f"{key}:={value}"])
      
        return ParameterValue(
            Command(xacro_args),
            value_type=str
        )
    
    @property
    def namespace(self):
        return self.get_launch_param('namespace', 'wcr')
    
    @property
    def variant(self):
        return self.get_launch_param('variant', 'mock')
    
    @property
    def sim(self):
        return self.get_launch_param('sim', True)
    
    @property
    def propeller_control(self):
        return self.get_launch_param('propeller_control', False)
    
    @property
    def urdf_file(self):
        return self.get_launch_param('urdf_file', 'wcr.urdf.xacro')
    
    @property
    def use_sim_time(self):
        return self.get_launch_param('use_sim_time', True)
    
    @property
    def use_controllers(self):
        return self.get_launch_param('controller', True)

    @property
    def rviz(self):
        return self.get_launch_param('rviz', True)

    @property
    def rviz_config(self):
        return self.get_launch_param('rviz_config', 'wcr_sensors.rviz')
    
    @property
    def world_file(self):
        return self.get_launch_param('world_file', 'robot_world.sdf')

    @property
    def world_name(self):
        """<world name="..."> from the SDF (the /clock topic depends on it)."""
        path = os.path.join(get_package_share_path('wcr_description'), 'worlds', self.world_file)
        with open(path, 'r') as f:
            match = re.search(r'<world\s+name\s*=\s*"([^"]+)"', f.read())
        return match.group(1) if match else 'default'

    def _world_spawn(self):
        """Spawn pose from the world's generated geometry file (e.g. dam_geometry.yaml), if set."""
        name = self.get_launch_param('world_geometry')
        if not name:
            return {}
        path = os.path.join(get_package_share_path('wcr_description'), 'worlds', name)
        with open(path, 'r') as f:
            return (yaml.safe_load(f) or {}).get('spawn', {})

    def _spawn(self, key, launch_key, default):
        spawn = self._world_spawn()
        return spawn[key] if key in spawn else self.get_launch_param(launch_key, default)

    @property
    def spawn_z_height(self):
        return self._spawn('z', 'spawn_z_height', 0.2)

    @property
    def spawn_x(self):
        return self._spawn('x', 'spawn_x', 0.0)

    @property
    def spawn_y(self):
        return self._spawn('y', 'spawn_y', 0.0)

    @property
    def spawn_roll(self):
        return self._spawn('roll', 'spawn_roll', 0.0)

    @property
    def spawn_pitch(self):
        return self._spawn('pitch', 'spawn_pitch', 0.0)

    @property
    def spawn_yaw(self):
        return self._spawn('yaw', 'spawn_yaw', 0.0)
    
    @property
    def gz_args(self):
        return self.get_launch_param('gz_args', '-r -v4')
    
    @property
    def controllers_yaml(self):
        return os.path.join(self.launcher_share, 'config', 'wcr_controller.yaml')
