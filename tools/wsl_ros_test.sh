#!/usr/bin/env bash
# End-to-end check of Webots (Windows) + ROS 2 (WSL): launches wcr_webots without RViz, lists
# the topics, measures rates, drives 5 s up the face with /cmd_vel and compares wheel odometry
# with ground truth. Writes results/ros_webots_test.txt.
#   bash tools/wsl_ros_test.sh [world]
WORLD=${1:-winter_dam}
START=${2:-true}          # false: Webots already open with webots/worlds/_ros_<world>.wbt
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$REPO/results/ros_webots_test.txt"
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # Nav2-recommended DDS (FastDDS lost service replies here)
LOG=/tmp/wcr_webots_launch.log
ros2 launch wcr_webots webots.launch.py world:=$WORLD rviz:=false start_webots:=$START > $LOG 2>&1 &
LPID=$!
for i in $(seq 1 90); do
  ros2 topic list 2>/dev/null | grep -q '^/odom$' && break
  sleep 2
done
sleep 5
{
echo "== ROS-Webots test ($WORLD) $(date)"
echo "-- topics"; ros2 topic list
for t in /scan /wall_camera/image_color /front_camera/image_color /imu /odom /joint_states /clock; do
  r=$(timeout 6 ros2 topic hz $t 2>/dev/null | grep 'average rate' | tail -1)
  echo "rate $t: ${r:-none}"
done
echo "-- start"
timeout 5 ros2 topic echo --once /odom --field pose.pose.position
timeout 5 ros2 topic echo --once /ground_truth --field pose.pose.position
timeout 5 ros2 topic echo --once /adhesion
echo "-- drive 5 s at 0.12 m/s up the face"
timeout 5 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}}' > /dev/null
sleep 1
timeout 5 ros2 topic echo --once /odom --field pose.pose.position
timeout 5 ros2 topic echo --once /ground_truth --field pose.pose.position
timeout 5 ros2 topic echo --once /scan --field ranges | tr ',' '\n' | grep -vc inf | sed 's/^/scan finite beams: /'
} > "$OUT" 2>&1
kill -INT $LPID; sleep 5; kill -9 $LPID 2>/dev/null
echo "-- launch log (tail)" >> "$OUT"; grep -v '^\s*$' $LOG | tail -25 >> "$OUT"
cat "$OUT"
