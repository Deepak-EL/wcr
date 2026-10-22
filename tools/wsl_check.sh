#!/usr/bin/env bash
# Quick environment check for the WSL sim host.
ls -la ~/wcr_ws/src 2>&1 | head -20
source /opt/ros/humble/setup.bash
which ign xacro 2>&1
ign gazebo --versions 2>&1 | head -3
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>&1 | tail -1
python3 -c "import trimesh; print('trimesh', trimesh.__version__)" 2>&1 | tail -1
dpkg -l | grep -E "ros-humble-(ros-gz|ros-ign|gz-ros2-control|ign-ros2-control|robot-localization|nav2-bringup|slam-toolbox|xacro|teleop-twist)" | awk '{print $2}'
nproc; free -g | head -2
