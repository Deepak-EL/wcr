#!/usr/bin/env bash
# Phase 3 check: launch Webots + ROS (no RViz), climb 3.6 m (odometry) up the climb line and
# record what inspection_detector reports (the fixed layouts put a crack 1.3 m and a wet patch
# 3.0 m above the spawn on that line). Saves one annotated wall-camera frame.
#   bash tools/wsl_inspection_test.sh [world] [layout fixed|random] [seed]
WORLD=${1:-winter_dam}; LAYOUT=${2:-fixed}; SEED=${3:-1}
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$REPO/results/inspection_test_$WORLD.txt"
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # Nav2-recommended DDS (FastDDS lost service replies here)
LOG=/tmp/wcr_inspection_launch.log
ros2 launch wcr_webots webots.launch.py world:=$WORLD rviz:=false layout:=$LAYOUT seed:=$SEED > $LOG 2>&1 &
LPID=$!
for i in $(seq 1 90); do ros2 topic list 2>/dev/null | grep -q '^/inspection/markers$' && ros2 topic list | grep -q '^/odom$' && break; sleep 2; done
sleep 8
ros2 topic echo /inspection/detections --field data > /tmp/wcr_detections.txt 2>/dev/null &
EPID=$!
python3 - "$REPO/results/inspection_annotated_$WORLD" <<'EOF' &
# saves the annotated wall-camera frame ~0.3 s after each detection: <prefix>_<id>.ppm
import json, sys, time, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
rclpy.init(); n = Node('grab'); state = {'want': None, 'at': 0.0}
def on_det(m):
    state['want'], state['at'] = json.loads(m.data)['id'], time.time() + 0.3
def on_img(m):
    if state['want'] is None or time.time() < state['at']: return
    ch = 4 if m.encoding.endswith('a8') else 3
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step // ch, ch)[:, :m.width, :3]
    if m.encoding.startswith('bgr'): a = a[..., ::-1]
    open(f"{sys.argv[1]}_{state['want']}.ppm", 'wb').write(b'P6 %d %d 255\n' % (a.shape[1], a.shape[0]) + a.tobytes())
    state['want'] = None
n.create_subscription(String, '/inspection/detections', on_det, 10)
n.create_subscription(Image, '/inspection/wall_camera_annotated', on_img, 1)
t0 = time.time()
while time.time() - t0 < 150: rclpy.spin_once(n, timeout_sec=0.05)
EOF
GPID=$!
python3 - <<'EOF'
# drive up the face at 0.12 m/s until the wheel odometry reads 3.6 m (or 150 s wall time)
import time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
rclpy.init(); n = Node('climb_cmd'); st = {'x': 0.0}
n.create_subscription(Odometry, '/odom', lambda m: st.update(x=m.pose.pose.position.x), 10)
pub = n.create_publisher(Twist, '/cmd_vel', 10); cmd = Twist(); cmd.linear.x = 0.12
t0 = time.time()
while st['x'] < 3.6 and time.time() - t0 < 150:
    pub.publish(cmd); rclpy.spin_once(n, timeout_sec=0.05)
pub.publish(Twist()); print(f'climbed {st["x"]:.2f} m by odometry in {time.time() - t0:.0f} s')
EOF
sleep 2
{
echo "== inspection test ($WORLD, layout $LAYOUT, seed $SEED) $(date)"
echo "-- robot after the climb (ground truth / odom)"
timeout 5 ros2 topic echo --once /ground_truth --field pose.pose.position
timeout 5 ros2 topic echo --once /odom --field pose.pose.position
echo "-- detections"
cat /tmp/wcr_detections.txt | grep -v '^---'
} > "$OUT" 2>&1
kill $EPID $GPID 2>/dev/null
kill -INT $LPID; sleep 6; kill -9 $LPID 2>/dev/null
echo "-- detector log" >> "$OUT"; grep -E 'inspection_detector|DETECTED|found|Error|error' $LOG | tail -25 >> "$OUT"
cat "$OUT"
