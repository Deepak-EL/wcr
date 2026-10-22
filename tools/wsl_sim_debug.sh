#!/usr/bin/env bash
# Start the headless sim, drive forward for a few seconds and dump the command chain.
LOG=$HOME/wcr_sim_logs; mkdir -p $LOG
source /opt/ros/humble/setup.bash; source ~/wcr_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$HOME/wcr_ws/install/wcr_description/share:${IGN_GAZEBO_RESOURCE_PATH}"
export WCR_RVIZ=false WCR_GZ_ARGS="-r -s -v3"
ros2 launch wcr_launcher launcher.launch.py > $LOG/launch.log 2>&1 &
for i in $(seq 1 90); do
  ros2 control list_controllers -c /wcr/controller_manager 2>/dev/null | grep -q "driving_velocity_controller.*active" && break; sleep 1; done
echo "up after $i s"
ros2 topic pub -r 20 /wcr/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" > /dev/null &
PUB=$!
sleep 4
echo "--- cmd_vel subscribers"; ros2 topic info /wcr/cmd_vel
echo "--- driving commands"; timeout 5 ros2 topic echo --once /wcr/driving_velocity_controller/commands
echo "--- steering commands"; timeout 5 ros2 topic echo --once /wcr/steering_position_controller/commands
echo "--- joint states"; timeout 5 ros2 topic echo --once /wcr/joint_states | grep -A14 -E "^(name|velocity|effort):"
echo "--- odom"; timeout 5 ros2 topic echo --once /wcr/ground_truth/odom | grep -A3 -E "position:|linear:" | head -12
echo "--- thrust"; timeout 5 ros2 topic echo --once /wcr/thrust/edf
echo "--- surface angle from IMU"; timeout 5 ros2 topic echo --once /wcr/adhesion/surface_angle_deg
echo "--- imu orientation"; timeout 5 ros2 topic echo --once /wcr/imu | grep -A4 orientation: | head -5
python3 - <<'EOF'
import subprocess, yaml, math
o = yaml.safe_load(subprocess.run(['ros2', 'topic', 'echo', '--once', '/wcr/ground_truth/odom'], capture_output=True, text=True, timeout=10).stdout.split('---')[0])
p = o['pose']['pose']['position']; g = yaml.safe_load(open('/home/' + __import__('os').environ['USER'] + '/wcr_ws/install/wcr_description/share/wcr_description/worlds/dam_geometry.yaml'))
n, u = g['slope']['normal'], g['slope']['up']
pv = [p['x'], p['y'], p['z']]
print('base_link off slope plane [mm]: %.1f   up-slope position [m]: %.4f' % (1000 * sum(a * b for a, b in zip(pv, n)), sum(a * b for a, b in zip(pv, u))))
EOF
kill $PUB
pkill -INT -f "ros2 launch wcr_launcher"; sleep 2
pkill -9 -f "ign gazebo"; pkill -9 -f parameter_bridge; pkill -9 -f rosbridge; pkill -9 -f "ros2 launch"
true
