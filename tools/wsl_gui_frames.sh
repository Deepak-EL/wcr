#!/usr/bin/env bash
# Open a world's GUI, move the view camera, and save N GUI screenshots in a row
# (to see flicker/incomplete frames). Frames: ~/wcr_sim_logs/gui_frames/
#   tools/wsl_gui_frames.sh <world.sdf> "<x y z roll pitch yaw>" [N]
source /opt/ros/humble/setup.bash; source ~/wcr_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$HOME/wcr_ws/install/wcr_description/share:${IGN_GAZEBO_RESOURCE_PATH}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
W=${1:-gravity_dam_world.sdf}; [ -f "$W" ] || W=~/wcr_ws/install/wcr_description/share/wcr_description/worlds/$W
POSE=(${2:-60 -110 45 0 0.35 2.0}); N=${3:-8}
OUT=~/wcr_sim_logs/gui_frames; rm -rf $OUT ~/.ignition/gui/pictures; mkdir -p $OUT
# temporary copy of the world with the Screenshot GUI plugin added
sed 's#</gui>#  <plugin filename="Screenshot" name="Screenshot"><ignition-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property></ignition-gui></plugin>\n    </gui>#' \
  "$W" > ~/wcr_sim_logs/frames_world.sdf
pkill -9 -f "ign gazebo" 2>/dev/null; sleep 1
ign gazebo -v3 --render-engine-gui ogre ~/wcr_sim_logs/frames_world.sdf > ~/wcr_sim_logs/gui_frames.log 2>&1 &
sleep 18
ign service -l | grep -E "^/gui/(screenshot|move_to)"
read -r x y z r p yw <<< "${POSE[*]}"
q=$(python3 -c "import math;r,p,y=$r,$p,$yw;cr,sr,cp,sp,cy,sy=math.cos(r/2),math.sin(r/2),math.cos(p/2),math.sin(p/2),math.cos(y/2),math.sin(y/2);print(f'x: {sr*cp*cy-cr*sp*sy} y: {cr*sp*cy+sr*cp*sy} z: {cr*cp*sy-sr*sp*cy} w: {cr*cp*cy+sr*sp*sy}')")
ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera --reptype ignition.msgs.Boolean --timeout 3000 \
  --req "pose: {position: {x: $x, y: $y, z: $z}, orientation: {$q}}" > /dev/null
sleep 3
for i in $(seq 1 $N); do
  ign service -s /gui/screenshot --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 3000 --req 'data: ""' > /dev/null
  sleep 0.4
done
sleep 1
cp ~/.ignition/gui/pictures/*.png $OUT/ 2>/dev/null; ls $OUT | wc -l
[ -n "$4" ] && cp "$(ls $OUT/*.png | head -1)" "$4"   # optional: copy the first frame here
pkill -INT -f "ign gazebo"; sleep 2; pkill -9 -f "ign gazebo" 2>/dev/null; true
