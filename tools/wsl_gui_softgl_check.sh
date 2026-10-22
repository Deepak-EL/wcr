#!/usr/bin/env bash
# Launch with gui_software_gl, confirm the GUI process runs on Mesa's CPU renderer.
source /opt/ros/humble/setup.bash; source ~/wcr_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$HOME/wcr_ws/install/wcr_description/share:${IGN_GAZEBO_RESOURCE_PATH}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA WCR_RVIZ=false WCR_GUI_SOFTWARE_GL=true
export WCR_WORLD_FILE=${WORLD:-gravity_dam_world.sdf} WCR_WORLD_GEOMETRY=${GEOM:-gravity_dam_geometry.yaml}
mkdir -p ~/wcr_sim_logs; rm -f ~/.ignition/rendering/ogre.log
ros2 launch wcr_launcher launcher.launch.py > ~/wcr_sim_logs/launch_softgl.log 2>&1 &
sleep 30
GUI=$(pgrep -f "ign gazebo -g" | head -1); SRV=$(pgrep -f "gazebo.*-s" | head -1)
echo "gui pid $GUI env: $(tr '\0' '\n' < /proc/$GUI/environ 2>/dev/null | grep LIBGL)"
echo "server pid $SRV env: $(tr '\0' '\n' < /proc/$SRV/environ 2>/dev/null | grep -c LIBGL_ALWAYS_SOFTWARE) LIBGL vars"
grep -hE "GL_RENDERER|GL_VERSION" ~/.ignition/rendering/ogre.log 2>/dev/null | sort | uniq -c
grep -c "Segmentation" ~/wcr_sim_logs/launch_softgl.log
pkill -INT -f "ros2 launch wcr_launcher"; sleep 3
pkill -9 -f "ign gazebo"; pkill -9 -f parameter_bridge; pkill -9 -f rosbridge; pkill -9 -f "ros2 launch"; true
