#!/usr/bin/env bash
# Build the workspace (~/wcr_ws/src holds symlinks to this repo checkout).
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # repo root (works in WSL and native Linux)
mkdir -p ~/wcr_ws/src
for p in wcr_control wcr_controllers wcr_description wcr_launcher wcr_odometry wcr_climb wcr_webots; do
  [ -e ~/wcr_ws/src/$p ] || ln -s "$REPO/$p" ~/wcr_ws/src/$p
done
source /opt/ros/humble/setup.bash
cd ~/wcr_ws
colcon build --symlink-install --packages-select "$@" 2>&1 | tail -25
