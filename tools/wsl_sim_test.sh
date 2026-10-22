#!/usr/bin/env bash
# Headless end-to-end test: launch the dam sim (server only), run the climb test, shut down.
#   tools/wsl_sim_test.sh [climb_test ros args...]   e.g.  -p distance:=3.0 -p speed:=0.1
LOG=${LOG:-$HOME/wcr_sim_logs}
mkdir -p "$LOG"
pkill -9 -f "ign gazebo" 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f rosbridge 2>/dev/null
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$HOME/wcr_ws/install/wcr_description/share:${IGN_GAZEBO_RESOURCE_PATH}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA"
export WCR_RVIZ=false
export WCR_GZ_ARGS="${WCR_GZ_ARGS:--r -s -v3}"

ros2 launch wcr_launcher launcher.launch.py > "$LOG/launch.log" 2>&1 &
LPID=$!
for i in $(seq 1 90); do
  ros2 topic list 2>/dev/null | grep -q /wcr/ground_truth/odom && \
  ros2 control list_controllers -c /wcr/controller_manager 2>/dev/null | grep -q "driving_velocity_controller.*active" && break
  sleep 1
done
echo "sim up after ${i}s"
ros2 control list_controllers -c /wcr/controller_manager 2>/dev/null
ros2 topic list | grep -qx /clock && echo "/clock ok" || echo "/clock MISSING"
grep -m3 "\[Err\]" "$LOG/launch.log"
timeout ${TEST_TIMEOUT:-240} ros2 run wcr_climb climb_test --ros-args -r __ns:=/wcr -p use_sim_time:=true -p csv:="$LOG/climb.csv" "$@"
RC=$?
kill -INT $LPID 2>/dev/null; sleep 3
pkill -9 -f "ign gazebo" 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f rosbridge 2>/dev/null
pkill -9 -f "ros2 launch wcr_launcher" 2>/dev/null
echo "--- launch log (errors/warnings) ---"
grep -iE "error|exception|fail|warn" "$LOG/launch.log" | grep -viE "rosbridge|deprecat" | sort | uniq -c | sort -rn | head -25
exit $RC
