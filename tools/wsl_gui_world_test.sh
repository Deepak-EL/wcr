#!/usr/bin/env bash
# Open a world's Gazebo GUI (a window appears), check it is still alive after 20 s,
# then close it. Crashes printed only after the close are shutdown noise.
#   tools/wsl_gui_world_test.sh gravity_dam_world.sdf
source /opt/ros/humble/setup.bash; source ~/wcr_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$HOME/wcr_ws/install/wcr_description/share:${IGN_GAZEBO_RESOURCE_PATH}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
W=${1:-gravity_dam_world.sdf}; [ -f "$W" ] || W=~/wcr_ws/install/wcr_description/share/wcr_description/worlds/$W
mkdir -p ~/wcr_sim_logs; LOG=~/wcr_sim_logs/gui_world.log
pkill -9 -f "ign gazebo" 2>/dev/null; sleep 1
ign gazebo -v4 --render-engine-gui ogre "$W" > $LOG 2>&1 &
sleep 20
if grep -q "Segmentation" $LOG; then echo "CRASHED while running"; else echo "alive after 20 s"; fi
sed 's/\x1b\[[0-9;]*m//g' $LOG | grep -E "Loaded plugin \[(GzScene3D|MinimalScene)\]|\[Err\]" | cut -c1-140 | sort | uniq -c
pkill -INT -f "ign gazebo"; sleep 3; pkill -9 -f "ign gazebo" 2>/dev/null; true
