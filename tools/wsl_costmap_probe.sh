#!/usr/bin/env bash
# Nav2 costmap probe: launch with nav:=true (no mission), then print the global costmap around the
# robot / demo area and the cost at given map points.   bash tools/wsl_costmap_probe.sh [world] [s,t ...]
WORLD=${1:-winter_dam}; shift
POINTS="${*:-1.5,0 3.78,-0.85 3.78,-0.71 2.0,-0.85 1.73,-0.85}"
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # Nav2-recommended DDS (FastDDS lost service replies here)
LOG=/tmp/wcr_costmap_probe.log
ros2 launch wcr_webots webots.launch.py world:=$WORLD rviz:=false nav:=true > $LOG 2>&1 &
LPID=$!
for i in $(seq 1 90); do ros2 topic list 2>/dev/null | grep -q '^/global_costmap/costmap$' && break; sleep 2; done
sleep 20
python3 - $POINTS <<'EOF'
import sys, time, numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node('cprobe'); got = {}
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: got.__setitem__('c', m), 1)
buf = Buffer(); TransformListener(buf, n)
t0 = time.time()
while 'c' not in got and time.time() - t0 < 20: rclpy.spin_once(n, timeout_sec=0.1)
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.05)
g = got['c']; res = g.info.resolution; ox, oy = g.info.origin.position.x, g.info.origin.position.y
d = np.array(g.data).reshape(g.info.height, g.info.width)
print(f'costmap {g.info.width}x{g.info.height} res {res} origin ({ox:.2f},{oy:.2f}) frame {g.header.frame_id}')
try:
    tf = buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform.translation
    print(f'robot (map): s {tf.x:.3f} t {tf.y:.3f}')
except Exception as e:
    print('no TF map->base_link', e)
def cost(s, t):
    j, i = int((s - ox) / res), int((t - oy) / res)
    return d[i, j] if 0 <= i < d.shape[0] and 0 <= j < d.shape[1] else None
for p in sys.argv[1:]:
    s, t = map(float, p.split(','))
    print(f'cost at s {s:.2f} t {t:+.2f}: {cost(s, t)}')
print('cost along the lane t -0.85 (s 1.5..4.2, step 0.1):', [cost(s, -0.85) for s in np.arange(1.5, 4.21, 0.1)])
print('cost along s 3.78 (t -1.3..1.0, step 0.1):', [cost(3.78, t) for t in np.arange(-1.3, 1.01, 0.1)])
EOF
kill -INT $LPID; sleep 6; kill -9 $LPID 2>/dev/null; true
