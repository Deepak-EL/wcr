#!/usr/bin/env bash
# Start the Gazebo GUI (with RViz opening at the same time, like run_sim.sh) N times;
# report how often it segfaults within 25 s.
source /opt/ros/humble/setup.bash; source ~/wcr_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$HOME/wcr_ws/install/wcr_description/share:${IGN_GAZEBO_RESOURCE_PATH}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
W=~/wcr_ws/install/wcr_description/share/wcr_description/worlds
mkdir -p ~/wcr_sim_logs
for i in $(seq 1 ${N:-4}); do
  pkill -9 -f "ign gazebo" 2>/dev/null; pkill -9 rviz2 2>/dev/null; sleep 1
  timeout 25 rviz2 > /dev/null 2>&1 &
  timeout 25 ign gazebo -v3 --render-engine-gui ogre "$W/dam_world.sdf" > ~/wcr_sim_logs/gui_$i.log 2>&1
  grep -q "Segmentation fault" ~/wcr_sim_logs/gui_$i.log && echo "run $i: CRASH" || echo "run $i: ok"
done
pkill -9 -f "ign gazebo" 2>/dev/null; pkill -9 rviz2 2>/dev/null; true
