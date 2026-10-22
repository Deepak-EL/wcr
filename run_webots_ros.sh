#!/usr/bin/env bash
# WCR in Webots (Windows) with ROS 2 + RViz (WSL).
#   ./run_webots_ros.sh [winter_dam|gravity_dam|goldeneye_dam|goldeneye_gorge] [extra launch args]
# extra args:  rviz:=false foxglove:=true      viewer on Windows instead of RViz (WSLg flicker)
#              layout:=random seed:=N           re-place defects + obstacles
#              nav:=true                        Nav2 on the wall: send goals from RViz / Foxglove
#              mission:=true [area:=demo]       autonomous coverage + report in results/
# Drive it from a second WSL terminal:
#   source ~/wcr_ws/install/setup.bash
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp      (same DDS as the simulation)
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.12 -p turn:=0.5
#   (i = up the face, , = down, j/l = turn, Shift+J / Shift+L = crab sideways, k = stop)
WORLD=${1:-winter_dam}
shift || true
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # Nav2-recommended DDS (FastDDS lost service replies here)
ros2 launch wcr_webots webots.launch.py world:="$WORLD" "$@"
