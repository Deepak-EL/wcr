#!/usr/bin/env bash
# Start the Gazebo GUI on a given D3D12 adapter for ~40 s in the background (for checking the
# WSLg window title for [WARN:COPY MODE]).   tools/wsl_gui_adapter_test.sh NVIDIA|Intel
source /opt/ros/humble/setup.bash; source ~/wcr_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$HOME/wcr_ws/install/wcr_description/share:${IGN_GAZEBO_RESOURCE_PATH}"
pkill -9 -f "ign gazebo" 2>/dev/null; sleep 1
W=~/wcr_ws/install/wcr_description/share/wcr_description/worlds/gravity_dam_world.sdf
[ "$1" = software ] && export LIBGL_ALWAYS_SOFTWARE=1
MESA_D3D12_DEFAULT_ADAPTER_NAME="$1" timeout ${SECS:-45} ign gazebo -v2 --render-engine-gui ogre "$W" \
  > ~/wcr_sim_logs/gui_adapter_$1.log 2>&1
pkill -9 -f "ign gazebo" 2>/dev/null; echo done
