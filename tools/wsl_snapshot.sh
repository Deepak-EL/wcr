#!/usr/bin/env bash
# Headless sim, drive up the face for DRIVE_S seconds, save overview + robot camera frames.
LOG=$HOME/wcr_sim_logs; mkdir -p $LOG
DRIVE_S=${DRIVE_S:-8}
TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkill -9 -f "ign gazebo" 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null
source /opt/ros/humble/setup.bash; source ~/wcr_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$HOME/wcr_ws/install/wcr_description/share:${IGN_GAZEBO_RESOURCE_PATH}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA"
export WCR_RVIZ=false WCR_GZ_ARGS="-r -s -v3"
ros2 launch wcr_launcher launcher.launch.py > $LOG/launch_snap.log 2>&1 &
ros2 run ros_gz_bridge parameter_bridge "/overview/image@sensor_msgs/msg/Image[ignition.msgs.Image" > /dev/null 2>&1 &
for i in $(seq 1 90); do
  ros2 control list_controllers -c /wcr/controller_manager 2>/dev/null | grep -q "driving_velocity_controller.*active" && break; sleep 1; done
timeout $DRIVE_S ros2 topic pub -r 20 /wcr/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.12}}" > /dev/null
python3 "$TOOLS/grab_image.py" /overview/image $LOG/overview.ppm 20
python3 "$TOOLS/grab_image.py" /wcr/camera $LOG/robot_camera.ppm 20
pkill -INT -f "ros2 launch wcr_launcher"; sleep 2
pkill -9 -f "ign gazebo"; pkill -9 -f parameter_bridge; pkill -9 -f rosbridge; pkill -9 -f "ros2 launch"
grep -m5 -E "\[Err\]|Unable to find|Failed to load" $LOG/launch_snap.log
true
