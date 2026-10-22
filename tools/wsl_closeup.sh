#!/usr/bin/env bash
# Close-up render of the robot at its spawn pose (temporary copy of the dam world with the
# overview camera moved next to the robot). Output: ~/wcr_sim_logs/overview.ppm
mkdir -p ~/wcr_sim_logs
pkill -9 -f "ign gazebo" 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f "ros2 launch" 2>/dev/null
sed -e 's#<pose>2.6 -2.2 2.6 0 0.27 2.27</pose>#<pose>0.38 0.30 1.40 0 0.16 2.107</pose>#' \
    -e 's#<horizontal_fov>1.0</horizontal_fov>#<horizontal_fov>0.75</horizontal_fov>#' \
    ~/wcr_ws/install/wcr_description/share/wcr_description/worlds/dam_world.sdf > ~/wcr_sim_logs/closeup_world.sdf
export WCR_WORLD_FILE="$HOME/wcr_sim_logs/closeup_world.sdf" DRIVE_S=0.5
bash "$(dirname "${BASH_SOURCE[0]}")/wsl_snapshot.sh" 2>&1 | grep -v Killed
