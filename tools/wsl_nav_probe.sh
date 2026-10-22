#!/usr/bin/env bash
# Nav2 drive probe: launch with nav:=true (no mission), send one NavigateToPose goal 0.5 m up the
# face and print cmd_vel / odom / EKF / ground truth every 2 s.   bash tools/wsl_nav_probe.sh [world] [speed]
WORLD=${1:-winter_dam}; SPEED=${2:-0.25}
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # Nav2-recommended DDS (FastDDS lost service replies here)
LOG=/tmp/wcr_nav_probe.log
ros2 launch wcr_webots webots.launch.py world:=$WORLD rviz:=false nav:=true fast:=true speed:=$SPEED > $LOG 2>&1 &
LPID=$!
for i in $(seq 1 90); do ros2 action list 2>/dev/null | grep -q navigate_to_pose && break; sleep 2; done
sleep 15
python3 - <<'EOF'
import time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node('navprobe'); st = {}
for topic, typ in (('/cmd_vel', Twist), ('/cmd_vel_nav', Twist), ('/odom', Odometry), ('/odometry/filtered', Odometry), ('/ground_truth', Odometry)):
    n.create_subscription(typ, topic, lambda m, t=topic: st.__setitem__(t, (m, time.time())), 10)
buf = Buffer(); TransformListener(buf, n)
cli = ActionClient(n, NavigateToPose, 'navigate_to_pose'); cli.wait_for_server(timeout_sec=20)
t0 = time.time()
while time.time() - t0 < 3: rclpy.spin_once(n, timeout_sec=0.05)
try:
    p = buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform.translation
    print(f'start (map): s {p.x:.3f} t {p.y:.3f}')
except Exception as e:
    print('TF map->base_link missing:', e); p = None
x0, y0 = (p.x, p.y) if p else (1.5, 0.0)
for dx, dy in ((0.5, 0.0), (0.5, -0.85), (0.0, -0.85), (0.0, 0.0)):        # up, sideways, down, back
  g = PoseStamped(); g.header.frame_id = 'map'; g.pose.orientation.w = 1.0
  g.pose.position.x, g.pose.position.y = x0 + dx, y0 + dy
  print(f'-- goal s {x0 + dx:.2f} t {y0 + dy:+.2f}')
  res = {}
  cli.send_goal_async(NavigateToPose.Goal(pose=g)).add_done_callback(
      lambda f: f.result().get_result_async().add_done_callback(lambda r: res.__setitem__('s', r.result().status)))
  t0 = time.time(); last = 0
  while time.time() - t0 < 40 and 's' not in res:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - last >= 2:
        last = time.time()
        def v(t, f):
            if t not in st: return f'{t}: none'
            m, ts = st[t]; age = time.time() - ts
            return f'{t}: {f(m)} ({age:.1f}s old)'
        tw = lambda m: f'vx {m.linear.x:+.3f} vy {m.linear.y:+.3f} wz {m.angular.z:+.2f}'
        od = lambda m: f'x {m.pose.pose.position.x:+.3f} y {m.pose.pose.position.y:+.3f} vx {m.twist.twist.linear.x:+.3f}'
        try:
            q = buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform.translation
            tfs = f'map->base s {q.x:.3f} t {q.y:.3f}'
        except Exception:
            tfs = 'map->base missing'
        # true displacement since the start, expressed in the map frame (x up the face, y across)
        gt = st.get('/ground_truth')
        if gt and 'gt0' not in st:
            st['gt0'] = gt[0].pose.pose.position
        true = '(no ground truth)'
        try:
            q = buf.lookup_transform('world', 'map', rclpy.time.Time()).transform.rotation
            import numpy as np
            x, y, z, w = q.x, q.y, q.z, q.w
            R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                          [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            a, b0 = gt[0].pose.pose.position, st['gt0']
            d = R.T @ np.array([a.x - b0.x, a.y - b0.y, a.z - b0.z])
            true = f'TRUE x {d[0]:+.3f} y {d[1]:+.3f}'
        except Exception:
            pass
        od_m = st.get('/odom')
        odp = od_m[0].pose.pose.position if od_m else None
        print(f'[{time.time() - t0:4.1f}s] cmd {v("/cmd_vel", tw)} | odom x {odp.x:+.3f} y {odp.y:+.3f} | {true}')
  print('goal status', res.get('s'))
EOF
grep -E 'ekf|EKF' $LOG | grep -iE 'warn|error' | head -5
kill -INT $LPID; sleep 6; kill -9 $LPID 2>/dev/null; true
