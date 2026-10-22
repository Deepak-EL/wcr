#!/usr/bin/env bash
# Record the robot climbing the gravity dam with two GPU world cameras (no GUI, no WSLg).
# Frames: ~/wcr_sim_logs/video/{wide,close}/fNNNNN.ppm  (5 fps sim time)
TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
V=~/wcr_sim_logs/video; rm -rf $V; mkdir -p $V
SRC=~/wcr_ws/install/wcr_description/share/wcr_description/worlds/gravity_dam_world.sdf
CAM='<model name="NAME"><static>true</static><pose>POSE</pose><link name="link"><sensor name="NAME" type="camera"><update_rate>5</update_rate><always_on>true</always_on><topic>/video/NAME</topic><camera><horizontal_fov>FOV</horizontal_fov><image><width>1280</width><height>720</height></image><clip><near>0.05</near><far>300</far></clip></camera></sensor></link></model>'
WIDE=$(echo "$CAM" | sed -e 's/NAME/wide/g' -e 's/POSE/7 -13 5.5 0 0.034 2.064/' -e 's/FOV/0.9/')
CLOSE=$(echo "$CAM" | sed -e 's/NAME/close/g' -e 's/POSE/1.6 -2.6 1.7 0 0.03 2.12/' -e 's/FOV/1.2/')
python3 - "$SRC" "$V/world.sdf" "$WIDE$CLOSE" <<'EOF'
import sys
t = open(sys.argv[1]).read()
open(sys.argv[2], 'w').write(t.replace('    <gui fullscreen="0">', sys.argv[3] + '\n    <gui fullscreen="0">', 1))
EOF
source /opt/ros/humble/setup.bash; source ~/wcr_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$HOME/wcr_ws/install/wcr_description/share:${IGN_GAZEBO_RESOURCE_PATH}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA WCR_RVIZ=false WCR_GZ_ARGS="-r -s -v2"
export WCR_WORLD_FILE=$V/world.sdf WCR_WORLD_GEOMETRY=gravity_dam_geometry.yaml
pkill -9 -f "ign gazebo" 2>/dev/null
ros2 launch wcr_launcher launcher.launch.py > $V/launch.log 2>&1 &
ros2 run ros_gz_bridge parameter_bridge /video/wide@sensor_msgs/msg/Image[ignition.msgs.Image \
  /video/close@sensor_msgs/msg/Image[ignition.msgs.Image > /dev/null 2>&1 &
for i in $(seq 1 90); do
  ros2 control list_controllers -c /wcr/controller_manager 2>/dev/null | grep -q "driving_velocity_controller.*active" && break; sleep 1; done
python3 "$TOOLS/record_frames.py" /video/wide $V/wide & R1=$!
python3 "$TOOLS/record_frames.py" /video/close $V/close & R2=$!
timeout 300 ros2 run wcr_climb climb_test --ros-args -r __ns:=/wcr -p use_sim_time:=true \
  -p distance:=${DIST:-8.0} -p speed:=0.12 -p descend:=false -p hold:=3.0 -p settle:=3.0 -p csv:=$V/climb.csv | grep -A8 "climb test:"
kill -INT $R1 $R2; sleep 2
pkill -INT -f "ros2 launch wcr_launcher"; sleep 2
pkill -9 -f "ign gazebo"; pkill -9 -f parameter_bridge; pkill -9 -f rosbridge; pkill -9 -f "ros2 launch"; true
