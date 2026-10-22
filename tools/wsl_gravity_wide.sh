#!/usr/bin/env bash
# Wide shot of the gravity dam with the robot on the face (temporary world copy, camera moved back).
TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p ~/wcr_sim_logs
sed -e 's#<pose>2.6 -3.4 2.4 0 0.22 2.2</pose>#<pose>7 -15 5.5 0 0.17 1.98</pose>#' \
    -e 's#<horizontal_fov>1.0</horizontal_fov>#<horizontal_fov>1.25</horizontal_fov>#' \
    ~/wcr_ws/install/wcr_description/share/wcr_description/worlds/gravity_dam_world.sdf > ~/wcr_sim_logs/gravity_wide.sdf
export WCR_WORLD_FILE="$HOME/wcr_sim_logs/gravity_wide.sdf" WCR_WORLD_GEOMETRY=gravity_dam_geometry.yaml DRIVE_S=${DRIVE_S:-25}
bash "$TOOLS/wsl_snapshot.sh" 2>&1 | grep saved
