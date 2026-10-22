#!/usr/bin/env bash
# Point ~/wcr_ws/src at THIS folder (replacing old symlinks) and do a clean rebuild.
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKGS="wcr_control wcr_controllers wcr_description wcr_launcher wcr_odometry wcr_climb"
mkdir -p ~/wcr_ws/src
cd ~/wcr_ws
for p in $PKGS; do
  [ -L src/$p ] && rm src/$p
  [ -e src/$p ] && { echo "src/$p is not a symlink - not touching it"; exit 1; }
  ln -s "$REPO/$p" src/$p
  rm -rf build/$p install/$p
done
ls -l src | awk '{print $9, $10, $11}'
source /opt/ros/humble/setup.bash
colcon build --symlink-install 2>&1 | tail -12
