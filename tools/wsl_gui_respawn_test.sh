#!/usr/bin/env bash
# Launch with GUI, kill the GUI, check the sim survives and the GUI respawns.
source /opt/ros/humble/setup.bash; source ~/wcr_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$HOME/wcr_ws/install/wcr_description/share:${IGN_GAZEBO_RESOURCE_PATH}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA WCR_RVIZ=false
mkdir -p ~/wcr_sim_logs
ros2 launch wcr_launcher launcher.launch.py > ~/wcr_sim_logs/launch_gui.log 2>&1 &
sleep 25
echo "server: $(pgrep -fc 'gazebo.*-s')  gui: $(pgrep -fc 'ign gazebo -g')"
pkill -9 -f "gazebo -g"; echo "killed GUI"; sleep 8
echo "server: $(pgrep -fc 'gazebo.*-s')  gui: $(pgrep -fc 'ign gazebo -g')"
timeout 5 ros2 topic echo --once /wcr/ground_truth/odom | grep -A3 "position:"
pkill -INT -f "ros2 launch wcr_launcher"; sleep 3
pkill -9 -f "ign gazebo"; pkill -9 -f parameter_bridge; pkill -9 -f rosbridge; pkill -9 -f "ros2 launch"; true
grep -E "Segmentation|respawn|process has died" ~/wcr_sim_logs/launch_gui.log | head
