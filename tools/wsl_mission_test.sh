#!/usr/bin/env bash
# Phase 4 check: launch Webots (fast mode) + ROS + Nav2 + coverage mission (no viewer), wait for the
# report, keep the launch log in results/mission_launch_<world>.log.
#   bash tools/wsl_mission_test.sh [world] [area: demo | s0,s1,t0,t1] [timeout_s] [speed m/s] [model: detailed | simple]
WORLD=${1:-winter_dam}; AREA=${2:-demo}; TMO=${3:-1500}; SPEED=${4:-0.12}; MODEL=${5:-detailed}
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPORT="$REPO/results/mission_report_$WORLD.txt"
rm -f "$REPORT"
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # Nav2-recommended DDS (FastDDS lost service replies here)
LOG=/tmp/wcr_mission_launch.log
ros2 launch wcr_webots webots.launch.py world:=$WORLD rviz:=false mission:=true fast:=true area:=$AREA speed:=$SPEED model:=$MODEL > $LOG 2>&1 &
LPID=$!
t0=$(date +%s)
while [ ! -f "$REPORT" ] && [ $(( $(date +%s) - t0 )) -lt $TMO ]; do
  sleep 15
  echo "[$(( $(date +%s) - t0 )) s] $(grep -E 'coverage_mission.*(poses|started|footprint|cannot|Nav2|restart)' $LOG | tail -1 | sed 's/.*\]: //')"
done
echo "== Nav2 / mission warnings"
grep -E '\[(planner_server|controller_server|bt_navigator)-[0-9]+\] \[(WARN|ERROR)\]' $LOG \
  | sed -E 's/\[[0-9]+\.[0-9]+\]//' | sort | uniq -c | sort -rn | head -12
echo "== report"
cat "$REPORT" 2>/dev/null || echo "no report after $TMO s"
kill -INT $LPID; sleep 8; kill -9 $LPID 2>/dev/null
pkill -f webots_controller 2>/dev/null
cp $LOG "$REPO/results/mission_launch_$WORLD.log"; true
