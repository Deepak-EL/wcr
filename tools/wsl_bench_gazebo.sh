#!/usr/bin/env bash
# Gazebo simulation speed (real-time factor) on the gravity dam: headless vs with the GUI window.
source /opt/ros/humble/setup.bash; source ~/wcr_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$HOME/wcr_ws/install/wcr_description/share:${IGN_GAZEBO_RESOURCE_PATH}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA WCR_RVIZ=false
export WCR_WORLD_FILE=gravity_dam_world.sdf WCR_WORLD_GEOMETRY=gravity_dam_geometry.yaml
mkdir -p ~/wcr_sim_logs
cleanup() {   # every ROS/Gazebo process from earlier runs, plus stale Fast DDS shared-memory locks
  pkill -9 -f "ign gazebo"; pkill -9 -f "ros2 launch"; pkill -9 -f "/opt/ros/humble/(lib|bin)/"; pkill -9 -f "wcr_ws/install/"
  sleep 1; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null; true
}
for mode in headless gui; do
  cleanup; sleep 1
  if [ $mode = headless ]; then export WCR_GZ_ARGS="-r -s -v2"; else export WCR_GZ_ARGS="-r -v2 --render-engine-gui ogre"; fi
  ros2 launch wcr_launcher launcher.launch.py > ~/wcr_sim_logs/bench_$mode.log 2>&1 &
  for i in $(seq 1 90); do   # wait until the controllers are up (robot spawned, sim running)
    ros2 control list_controllers -c /wcr/controller_manager 2>/dev/null | grep -q "driving_velocity_controller.*active" && break; sleep 1; done
  sleep 5
  rtf=$(timeout 25 ign topic -e -t /world/gravity_dam_world/stats -n 40 2>/dev/null \
        | grep real_time_factor | awk '{s+=$2; n++} END {if (n) printf "%.3f (n=%d)", s/n, n}')
  cpu=$(top -bn2 -d1 | awk '/^top -/{blk++} blk==2 && /ruby|ign/ {s+=$9} END {printf "%.0f", s}')
  echo "$mode: real-time factor $rtf, gazebo CPU ${cpu}% (12 threads = 1200%)"
  pkill -INT -f "ros2 launch wcr_launcher"; sleep 3
done
cleanup
