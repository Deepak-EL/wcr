#!/usr/bin/env bash
# Climb test + snapshots on the USACE gravity dam world (the default world is unchanged).
TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p ~/wcr_sim_logs
export WCR_WORLD_FILE=gravity_dam_world.sdf WCR_WORLD_GEOMETRY=gravity_dam_geometry.yaml
export TEST_TIMEOUT=400
bash "$TOOLS/wsl_sim_test.sh" -p distance:=${DIST:-9.0} -p speed:=0.12 -p descend:=false -p timeout:=200.0 -p hold:=3.0 \
  > ~/wcr_sim_logs/run_gravity.txt 2>&1
grep -E 'sim up' ~/wcr_sim_logs/run_gravity.txt
grep -A9 'climb test:' ~/wcr_sim_logs/run_gravity.txt
python3 "$TOOLS/climb_log_summary.py" ~/wcr_sim_logs/climb.csv 10
cp ~/wcr_sim_logs/climb.csv ~/wcr_sim_logs/climb_gravity.csv
# snapshots after driving 1 m up the face
DRIVE_S=8 bash "$TOOLS/wsl_snapshot.sh" 2>&1 | grep saved
