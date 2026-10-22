#!/usr/bin/env bash
# Lidar probe: launch the stack with Nav2 (no mission, no viewer), then print what /scan returns
# within 1 m of the robot - at rest and while driving up the face - to debug false obstacles.
#   bash tools/wsl_scan_probe.sh [world]
WORLD=${1:-winter_dam}
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # Nav2-recommended DDS (FastDDS lost service replies here)
LOG=/tmp/wcr_probe_launch.log
ros2 launch wcr_webots webots.launch.py world:=$WORLD rviz:=false nav:=true > $LOG 2>&1 &
LPID=$!
for i in $(seq 1 90); do ros2 topic list 2>/dev/null | grep -q '^/scan$' && break; sleep 2; done
sleep 10
python3 - <<'EOF'
import math, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
rclpy.init(); n = Node('probe'); st = {}
n.create_subscription(LaserScan, '/scan', lambda m: st.__setitem__('scan', m), qos_profile_sensor_data)
pub = n.create_publisher(Twist, '/cmd_vel', 10)

def report(tag):
    m = st.get('scan')
    if m is None:
        print(tag, 'no scan'); return
    r = np.array(m.ranges); a = m.angle_min + np.arange(len(r)) * m.angle_increment
    ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
    print(f'{tag}: frame {m.header.frame_id}, {len(r)} beams, angle_min {math.degrees(m.angle_min):.1f} '
          f'max {math.degrees(m.angle_max):.1f} inc {math.degrees(m.angle_increment):.2f} deg, '
          f'range {m.range_min}..{m.range_max}, finite {ok.sum()}')
    near = ok & (r < LIM)
    for ang, rr in zip(a[near], r[near]):
        print(f'   angle {math.degrees(ang):7.1f} deg  r {rr:.3f}  x {rr*math.cos(ang):+.3f} y {rr*math.sin(ang)-0.04:+.3f} (base)')

t0 = time.time()
while time.time() - t0 < 3: rclpy.spin_once(n, timeout_sec=0.05)
LIM = 2.6
report('AT REST (expect bush ~(+0.9, -0.8), rocks ~(+2.1, +0.8))')
LIM = 1.0
cmd = Twist(); cmd.linear.x = 0.12
for k in range(0):
    t0 = time.time()
    while time.time() - t0 < 4: pub.publish(cmd); rclpy.spin_once(n, timeout_sec=0.05)
    report(f'DRIVING UP, {4 * (k + 1)} s')
pub.publish(Twist())

# global costmap: lethal cells inside the map's free area, grouped into blobs (map frame, metres)
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
got = {}
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: got.__setitem__('c', m), 1)
n.create_subscription(OccupancyGrid, '/map', lambda m: got.__setitem__('m', m),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0 = time.time()
while ('c' not in got or 'm' not in got) and time.time() - t0 < 20: rclpy.spin_once(n, timeout_sec=0.1)
for key, name in (('m', 'static map'), ('c', 'global costmap')):
    g = got.get(key)
    if g is None:
        print(name, 'not received'); continue
    d = np.array(g.data).reshape(g.info.height, g.info.width)
    ys, xs = np.nonzero(d >= (100 if key == 'm' else 99))
    X = g.info.origin.position.x + (xs + 0.5) * g.info.resolution
    Y = g.info.origin.position.y + (ys + 0.5) * g.info.resolution
    inner = (X > 1.2) & (X < 5.5) & (Y > -1.5) & (Y < 1.5)          # around the demo area
    print(f'{name}: {inner.sum()} lethal cells in s 1.2..5.5, t -1.5..1.5')
    for s0 in np.arange(1.2, 5.5, 0.5):
        row = inner & (X >= s0) & (X < s0 + 0.5)
        if row.any():
            print(f'   s {s0:.1f}-{s0 + 0.5:.1f}: t {Y[row].min():+.2f}..{Y[row].max():+.2f} ({row.sum()} cells)')
EOF
kill -INT $LPID; sleep 6; kill -9 $LPID 2>/dev/null
