"""Autonomous coverage mission (Phase 4): lawnmower lanes over an area of the dam face, driven by Nav2.

The robot keeps facing up the face and sweeps vertical lanes (up, crab across, down, ...) so that
the wall camera's footprint covers the area. Lanes are cut into segments that keep clear of the
obstacles on the known map. As in Nav2's coverage server (opennav_coverage), each lane segment is
a straight path handed to Nav2's controller (FollowPath action, MPPI omnidirectional), which also
avoids anything the lidar marks in the local costmap. The short moves between lanes are FollowPath
too, unless a mapped obstacle is in the way: then NavigateToPose (Smac planner + MPPI); the way
back to the start is a NavigateToPose. A leg that fails or overruns its time is logged and skipped.

  params  layout          inspection layout YAML: map frame (face), demo_area, items
          area            'demo' (layout demo_area) or 's0,s1,t0,t1' in map metres
          lane_overlap    fraction of the wall camera footprint width shared by adjacent lanes (0.2)
          clearance       distance kept from mapped obstacles when cutting lanes [m] (0.12)
          start_delay     s of sim time to wait after Nav2 is up (3)
          speed           driving speed [m/s] (sets the time limit per leg)
          report_dir      where mission_report_<world>.txt / coverage_<world>.png go
  sub     /inspection/detections, /global_costmap/costmap, /wall_camera/camera_info, TF
  pub     /coverage/map (OccupancyGrid, 100 = seen by the wall camera), /coverage/percent,
          /coverage/lanes (nav_msgs/Path, the planned route), /coverage/status
  action  follow_path (nav2_msgs/FollowPath), navigate_to_pose (nav2_msgs/NavigateToPose)
"""
import json
import math
import struct
import zlib
from pathlib import Path

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener

CELL = 0.02          # coverage grid resolution [m]


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def write_png(path, rgb):
    """Minimal PNG writer (no PIL in the ROS environment)."""
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[r].astype(np.uint8).tobytes() for r in range(h))

    def chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
    Path(path).write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                           + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


class CoverageMission(Node):
    def __init__(self):
        super().__init__('coverage_mission')
        self.doc = yaml.safe_load(Path(self.declare_parameter('layout', '').value).read_text())
        area = self.declare_parameter('area', 'demo').value
        if area == 'demo':
            (self.s0, self.s1), (self.t0, self.t1) = self.doc['demo_area']['s'], self.doc['demo_area']['t']
        else:
            self.s0, self.s1, self.t0, self.t1 = (float(v) for v in area.split(','))
        self.overlap = self.declare_parameter('lane_overlap', 0.2).value    # covers MPPI's ~1-2 cm tracking error
        self.clearance = self.declare_parameter('clearance', 0.12).value
        self.start_delay = self.declare_parameter('start_delay', 3.0).value
        self.speed = float(self.declare_parameter('speed', 0.12).value)
        self.report_dir = Path(self.declare_parameter('report_dir', '/tmp').value)
        self.world = self.doc['world']
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow = ActionClient(self, FollowPath, 'follow_path')
        self.K = self.img_size = None
        self.create_subscription(CameraInfo, '/wall_camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.found = {}
        self.create_subscription(String, '/inspection/detections', self.on_detection, 10)
        self.costmap, self.seen = None, set()
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.have_map = False
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'have_map', True), latched)
        self.map_mgr = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_map/manage_nodes')
        self.t_map_retry = None
        self.pub_cov = self.create_publisher(OccupancyGrid, '/coverage/map', latched)
        self.pub_pct = self.create_publisher(Float32, '/coverage/percent', 10)
        self.pub_lanes = self.create_publisher(PathMsg, '/coverage/lanes', latched)
        self.pub_status = self.create_publisher(String, '/coverage/status', 10)
        # coverage grid over the area (rows = s, cols = t)
        self.ns = int(round((self.s1 - self.s0) / CELL))
        self.nt = int(round((self.t1 - self.t0) / CELL))
        self.covered = np.zeros((self.ns, self.nt), bool)
        cs = self.s0 + (np.arange(self.ns) + 0.5) * CELL
        ct = self.t0 + (np.arange(self.nt) + 0.5) * CELL
        self.S, self.T = np.meshgrid(cs, ct, indexing='ij')
        self.blocked = np.zeros_like(self.covered)                 # obstacle footprints (not inspectable)
        for it in self.doc['items']:
            if it['type'] == 'bush':                                # canopy leans up the face
                self.blocked |= np.hypot(self.S - it['s'] - 0.2 * it['size'][0], self.T - it['t']) < 0.5 * it['size'][0]
            elif it['type'] == 'debris':
                self.blocked |= np.hypot(self.S - it['s'], self.T - it['t']) < 0.45 * it['size'][0]
            elif it['type'] in ('pipe', 'ledge'):
                self.blocked |= (np.abs(self.S - it['s']) < it['size'][0] / 2) & (np.abs(self.T - it['t']) < it['size'][1] / 2)
        self.state = 'waiting for Nav2'
        self.t_start = None
        self.path_len, self.last_xy = 0.0, None
        self.route, self.legs, self.leg_i = [], [], 0              # lane poses / legs to drive / current leg
        self.failed, self.events = [], []
        self.goal_handle, self.cancelling = None, False
        self.t_leg, self.leg_limit = 0.0, 1e9
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.publish_coverage)
        self.get_logger().info(f'{self.world}: area s {self.s0:.2f}..{self.s1:.2f} m, t {self.t0:.2f}..{self.t1:.2f} m')

    # ---------------------------------------------------------------- inputs / geometry
    def on_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.img_size = (msg.width, msg.height)

    def on_detection(self, msg):
        d = json.loads(msg.data)
        self.found.setdefault(d['id'], d)

    def on_costmap(self, msg):
        self.costmap = msg
        for it in self.doc['items']:
            if it['type'] in ('bush', 'pipe', 'ledge', 'debris') and it['id'] not in self.seen and \
                    self.lethal_near(it['s'], it['t'], 0.45 * it['size'][0]):
                self.seen.add(it['id'])

    def lethal_near(self, s, t, r):
        m = self.costmap
        if m is None:
            return False
        res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
        data = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
        j0, j1 = int((s - r - ox) / res), int((s + r - ox) / res) + 1
        i0, i1 = int((t - r - oy) / res), int((t + r - oy) / res) + 1
        win = data[max(i0, 0):max(i1, 0), max(j0, 0):max(j1, 0)]
        return bool(win.size and (win >= 99).any())

    def lookup(self, child):
        t = self.tf_buffer.lookup_transform('map', child, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), quat_to_R(q.x, q.y, q.z, q.w)

    def footprint(self):
        """Wall camera footprint on the face (map z = 0) as a polygon of 4 (s, t) points."""
        c, R = self.lookup('wall_camera_optical')
        w, h = self.img_size
        Kinv = np.linalg.inv(self.K)
        pts = []
        for u, v in ((0, 0), (w, 0), (w, h), (0, h)):
            d = R @ (Kinv @ np.array([u, v, 1.0]))
            if d[2] >= -1e-6:
                return None
            pts.append((c + d * (-c[2] / d[2]))[:2])
        return np.array(pts)

    def robot_blocked(self, s, t):
        """Would the robot base at (s, t), facing up the face, come within `clearance` of a mapped
        obstacle? (robot: 0.17 m ahead, 0.16 m behind, 0.15 m to each side of the base)"""
        for it in self.doc['items']:
            if not it.get('mapped'):
                continue
            ls, lt = it['size']
            if it['type'] == 'bush':                                 # canopy: centre 0.2 size up the face
                cs = it['s'] + 0.2 * ls
                ds = np.clip(cs - s, -0.16, 0.17)
                dt = np.clip(it['t'] - t, -0.15, 0.15)
                if math.hypot(cs - s - ds, it['t'] - t - dt) < 0.5 * ls + self.clearance:
                    return True
            else:                                                     # pipe (0.2 m wide) / ledge
                half_t = 0.1 if it['type'] == 'pipe' else lt / 2
                if abs(it['s'] - s) < ls / 2 + 0.17 + self.clearance and abs(it['t'] - t) < half_t + 0.15 + self.clearance:
                    return True
        return False

    # ---------------------------------------------------------------- planning
    def plan_route(self):
        """Lane segments for the robot base so the camera footprint sweeps the area -> poses."""
        fp = self.footprint()
        b, _ = self.lookup('base_link')
        off = fp.mean(0) - b[:2]                                    # footprint centre vs base (yaw ~0)
        width, height = np.ptp(fp[:, 1]), np.ptp(fp[:, 0])
        spacing = width * (1 - self.overlap)
        ts = list(np.arange(self.t0 + width / 2, self.t1 - width / 2 + 1e-6, spacing))
        if not ts or ts[-1] < self.t1 - width / 2 - 1e-3:
            ts.append(self.t1 - width / 2)
        s_lo, s_hi = self.s0 + height / 2 - off[0], self.s1 - height / 2 - off[0]
        ss = np.arange(s_lo, s_hi + 1e-6, 0.05)
        route, n_seg, skipped = [], 0, 0.0
        for k, t in enumerate(ts):
            tr = t - off[1]                                          # robot base across the face
            free = np.array([not self.robot_blocked(s, tr) for s in ss])
            skipped += (~free).sum() * 0.05
            lane, start = [], None
            for i, f in enumerate(np.r_[free, False]):
                if f and start is None:
                    start = i
                elif not f and start is not None:
                    if ss[i - 1] - ss[start] >= 0.3:                # skip stubs (near-duplicate poses)
                        lane.append((ss[start], ss[i - 1]))
                    start = None
            if k % 2:
                lane = [(b_, a_) for a_, b_ in reversed(lane)]
            for a_, b_ in lane:
                route += [(a_, tr), (b_, tr)]
                n_seg += 1
        # leave the start straight up first; the way back is a separate goal afterwards (as the last
        # pose of this route, where the robot already stands, Nav2 would consider it done at once),
        # to a point 0.35 m above the start, clear of where the deployment cradle was
        route.insert(0, (b[0] + 0.3, b[1]))
        self.home = (b[0] + 0.35, b[1])
        self.route = route
        self.build_legs()
        self.get_logger().info(f'wall camera footprint {height * 100:.0f} x {width * 100:.0f} cm, centre '
                               f'{off[0]:.2f} m ahead / {off[1]:+.2f} m across of the base -> {len(ts)} lanes '
                               f'{spacing * 100:.0f} cm apart, {n_seg} segments, {len(route)} poses '
                               f'({skipped:.1f} m of lanes skipped next to mapped obstacles)')
        path = PathMsg()
        path.header.frame_id = 'map'
        path.poses = [self.pose(*p) for p in route]
        self.pub_lanes.publish(path)

    def pose(self, x, y):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.w = 1.0                                  # facing up the face
        return p

    # ---------------------------------------------------------------- loop
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        try:
            b, _ = self.lookup('base_link')
        except Exception:
            return
        now = self.now()
        if self.state == 'waiting for Nav2':
            if self.K is None or not self.nav.server_is_ready() or not self.follow.server_is_ready():
                return
            if not self.have_map:
                # the map server's lifecycle start can get lost under start-up load: restart it
                # through Nav2's lifecycle manager (reset, then startup)
                if self.t_map_retry is None:
                    self.t_map_retry = now
                elif now - self.t_map_retry > 15 and self.map_mgr.service_is_ready():
                    self.get_logger().warn('no /map yet: restarting the map server via its lifecycle manager')
                    self.map_mgr.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.RESET)) \
                        .add_done_callback(lambda f: self.map_mgr.call_async(
                            ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP)))
                    self.t_map_retry = now
                return
            if self.costmap is None or self.costmap.info.width * self.costmap.info.resolution < 6.0:
                return                                              # global costmap not on the map yet
            self.state, self.t_ready = 'starting', now
            return
        if self.state == 'starting':
            if now - self.t_ready < self.start_delay:
                return
            try:
                self.plan_route()
            except Exception as e:                                  # TF not complete yet
                self.get_logger().warn(f'cannot plan yet: {e}')
                return
            self.state, self.t_start = 'running', now
            self.get_logger().info('mission started')
            self.send()
        if self.state != 'running':
            return
        if self.last_xy is not None:
            self.path_len += float(np.hypot(*(b[:2] - self.last_xy)))
        self.last_xy = b[:2]
        fp = self.footprint()
        if fp is not None:
            self.paint(fp)
        if self.goal_handle is not None and not self.cancelling and now - self.t_leg > self.leg_limit:
            self.log_event(f'leg {self.leg_i + 1} took over {self.leg_limit:.0f} s at s {b[0]:.2f} t {b[1]:.2f}: '
                           f'cancelled')
            self.cancelling = True
            self.goal_handle.cancel_goal_async()

    # ---------------------------------------------------------------- Nav2
    # Each lane is a straight path followed by Nav2's controller (FollowPath, MPPI); the short moves
    # between lanes too, unless a mapped obstacle is in the way: then NavigateToPose (planner +
    # controller). The way back to the start is a NavigateToPose. (The same split as Nav2's coverage
    # server: a precomputed coverage path to the controller, the planner only to get somewhere.)
    def build_legs(self):
        """route[0] = straight up off the start, then (lane start, lane end) pairs -> legs."""
        b, _ = self.lookup('base_link')
        legs, cur = [('move', (b[0], b[1]), self.route[0])], self.route[0]
        pts = self.route[1:]
        for k in range(0, len(pts) - 1, 2):
            a, e = pts[k], pts[k + 1]
            legs.append(('move', cur, a))
            legs.append(('lane', a, e))
            cur = e
        legs.append(('home', cur, self.home))
        self.legs, self.leg_i = legs, 0

    def straight_path(self, a, e):
        n = max(2, int(np.hypot(e[0] - a[0], e[1] - a[1]) / 0.05) + 1)
        path = PathMsg()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self.pose(a[0] + (e[0] - a[0]) * i / (n - 1), a[1] + (e[1] - a[1]) * i / (n - 1)) for i in range(n)]
        return path

    def send(self):
        if self.leg_i >= len(self.legs):
            self.finish()
            return
        kind, a, e = self.legs[self.leg_i]
        b, _ = self.lookup('base_link')
        here = (b[0], b[1])
        length = float(np.hypot(e[0] - here[0], e[1] - here[1]))
        self.leg_limit = 25.0 + 4.0 * length / max(self.speed, 0.05)
        self.t_leg = self.now()
        if kind == 'lane' or (kind == 'move' and length < 0.8 and self.line_free(here, e)):
            goal = FollowPath.Goal(path=self.straight_path(here, e), controller_id='FollowPath',
                                   goal_checker_id='general_goal_checker')
            self.follow.send_goal_async(goal).add_done_callback(self.on_accepted)
        else:                                                       # around a mapped obstacle / home
            self.nav.send_goal_async(NavigateToPose.Goal(pose=self.pose(*e))).add_done_callback(self.on_accepted)

    def line_free(self, a, e):
        n = max(2, int(np.hypot(e[0] - a[0], e[1] - a[1]) / 0.05))
        return not any(self.robot_blocked(a[0] + (e[0] - a[0]) * k / n, a[1] + (e[1] - a[1]) * k / n)
                       for k in range(n + 1))

    def on_accepted(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.log_event(f'Nav2 rejected leg {self.leg_i + 1}')
            self.next_leg()
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        status = fut.result().status
        self.goal_handle = None
        self.cancelling = False
        kind, a, e = self.legs[self.leg_i]
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.failed.append(self.leg_i)
            self.log_event(f'{kind} leg {self.leg_i + 1} to s {e[0]:.2f} t {e[1]:.2f} not completed '
                           f'({"aborted" if status == GoalStatus.STATUS_ABORTED else "cancelled"})')
            self.save_costmap(f'costmap_{self.world}_leg{self.leg_i + 1}.png')
        self.next_leg()

    def next_leg(self):
        self.leg_i += 1
        if self.leg_i < len(self.legs):
            kind = self.legs[self.leg_i][0]
            if kind == 'lane':
                lanes_done = sum(1 for k in range(self.leg_i) if self.legs[k][0] == 'lane')
                lanes = sum(1 for leg in self.legs if leg[0] == 'lane')
                self.get_logger().info(f'lane {lanes_done + 1}/{lanes}, coverage {self.percent()[0]:.1f} %')
            elif kind == 'home':
                self.get_logger().info(f'all lanes done, coverage {self.percent()[0]:.1f} %: returning to the start')
        self.send()

    def save_costmap(self, name):
        """Debug picture of the global costmap around the area: grey = cost, red = lethal,
        blue dots = remaining leg ends, yellow = failed leg end, green square = robot (up = up the face)."""
        m = self.costmap
        if m is None:
            return
        res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
        data = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)    # [t, s]
        s_lo, s_hi, t_lo, t_hi = self.s0 - 1.0, self.s1 + 1.0, self.t0 - 1.0, self.t1 + 1.0
        j0, j1 = int((s_lo - ox) / res), int((s_hi - ox) / res)
        i0, i1 = int((t_lo - oy) / res), int((t_hi - oy) / res)
        win = data[max(i0, 0):i1, max(j0, 0):j1]
        img = np.stack([255 - np.clip(win, 0, 100) * 2.2] * 3, -1).astype(np.uint8)
        img[win >= 99] = (220, 40, 40)
        img[win < 0] = (90, 90, 140)

        def dot(s, t, col, r=1):
            j, i = int((s - ox) / res) - max(j0, 0), int((t - oy) / res) - max(i0, 0)
            img[max(i - r, 0):i + r + 1, max(j - r, 0):j + r + 1] = col
        for leg in self.legs[self.leg_i + 1:]:
            dot(*leg[2], (40, 90, 230))
        dot(*self.legs[self.leg_i][2], (240, 200, 0))
        try:
            b, _ = self.lookup('base_link')
            dot(b[0], b[1], (30, 180, 60), 2)
        except Exception:
            pass
        img = img.transpose(1, 0, 2)[::-1, ::-1]                  # rows = s (up at the top), cols = -t
        write_png(self.report_dir / name, np.repeat(np.repeat(img, 6, 0), 6, 1))

    def log_event(self, text):
        self.events.append(text)
        self.get_logger().info(text)

    # ---------------------------------------------------------------- coverage
    def paint(self, fp):
        """Mark the coverage cells inside the (convex) footprint polygon."""
        lo, hi = fp.min(0), fp.max(0)
        i0, i1 = np.clip(((lo[0] - self.s0) / CELL, (hi[0] - self.s0) / CELL + 1), 0, self.ns).astype(int)
        j0, j1 = np.clip(((lo[1] - self.t0) / CELL, (hi[1] - self.t0) / CELL + 1), 0, self.nt).astype(int)
        if i1 <= i0 or j1 <= j0:
            return
        S, T = self.S[i0:i1, j0:j1], self.T[i0:i1, j0:j1]
        inside = np.ones_like(S, bool)
        for k in range(4):                                         # same side of each edge as the polygon
            a, c, e = fp[k], fp[(k + 1) % 4], fp[(k + 2) % 4]
            cross = (c[0] - a[0]) * (T - a[1]) - (c[1] - a[1]) * (S - a[0])
            inside &= cross * np.sign((c[0] - a[0]) * (e[1] - a[1]) - (c[1] - a[1]) * (e[0] - a[0])) >= 0
        self.covered[i0:i1, j0:j1] |= inside

    def percent(self):
        free = ~self.blocked
        return 100.0 * self.covered[free].mean(), 100.0 * self.covered.mean()

    def publish_coverage(self):
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = CELL
        g.info.origin.position.x, g.info.origin.position.y = self.s0, self.t0
        # grid x = map x = s (width), y = t (height); data rows run along y -> transpose (s, t)
        g.info.width, g.info.height = self.ns, self.nt
        g.data = np.where(self.covered, 100, np.where(self.blocked, -1, 0)).astype(np.int8).T.ravel().tolist()
        self.pub_cov.publish(g)
        pct, _ = self.percent()
        self.pub_pct.publish(Float32(data=float(pct)))
        self.pub_status.publish(String(data=f'{self.state}: {pct:.1f} % covered'))

    # ---------------------------------------------------------------- report
    def finish(self):
        if self.state == 'done':
            return
        self.state = 'done'
        self.publish_coverage()
        now = self.now()
        pct, raw = self.percent()
        b, _ = self.lookup('base_link')
        home = getattr(self, 'home', (b[0], b[1]))
        in_area = [it for it in self.doc['items']
                   if self.s0 - 0.5 < it['s'] < self.s1 + 0.5 and self.t0 - 0.5 < it['t'] < self.t1 + 0.5]
        lanes = sum(1 for leg in self.legs if leg[0] == 'lane')
        lanes_failed = sum(1 for k in self.failed if self.legs[k][0] == 'lane')
        lines = [f'WCR coverage mission (Nav2 FollowPath / NavigateToPose): {self.world}',
                 f'  area            s {self.s0:.2f}..{self.s1:.2f} m, t {self.t0:.2f}..{self.t1:.2f} m '
                 f'({(self.s1 - self.s0) * (self.t1 - self.t0):.1f} m2)',
                 f'  coverage        {pct:.1f} % of the inspectable area ({raw:.1f} % incl. obstacle footprints)',
                 f'  route           {lanes} lane segments ({lanes_failed} not completed), {len(self.legs)} legs '
                 f'({len(self.failed)} not completed), '
                 f'back at the start: {"yes" if np.hypot(*(b[:2] - np.array(home))) < 0.15 else "no"}',
                 f'  path length     {self.path_len:.1f} m in {now - self.t_start:.0f} s sim time',
                 f'  defects found   {len(self.found)}: ' + ', '.join(self.found)]
        for it in in_area:
            if it['type'] in ('bush', 'pipe', 'ledge', 'debris'):
                lines.append(f'  obstacle        {it["id"]:10s} {"on the map" if it.get("mapped") else "NOT on the map"}, '
                             f'{"in the costmap" if it["id"] in self.seen else "never in the costmap"}')
        lines += [f'  event           {e}' for e in self.events]
        text = '\n'.join(lines)
        self.get_logger().info('\n' + text)
        self.report_dir.mkdir(parents=True, exist_ok=True)
        (self.report_dir / f'mission_report_{self.world}.txt').write_text(text + '\n')
        img = np.full(self.covered.shape + (3,), 235, np.uint8)            # rows = s, cols = t
        img[self.covered] = (60, 170, 90)
        img[self.blocked & ~self.covered] = (120, 120, 120)
        img = np.repeat(np.repeat(img[::-1, ::-1], 4, axis=0), 4, axis=1)   # up = up the face, left = +t
        write_png(self.report_dir / f'coverage_{self.world}.png', img)
        self.pub_status.publish(String(data='done: ' + text.replace('\n', ' | ')))


def main():
    rclpy.init()
    node = CoverageMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
