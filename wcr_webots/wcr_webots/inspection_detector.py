"""Inspection detector (Phase 3): reports a known defect when it is inside the wall camera's view.

No image processing: defects (cracks, wet patches, weep holes) come from the layout YAML written by
tools/inspection.py, each with 'mark' points: pixels of the defect itself (crack line, damp area)
placed on the wall. They are projected into the wall camera image (pinhole model from
/wall_camera/camera_info, camera pose = /ground_truth pose of the robot x the base_link ->
wall_camera_optical transform from TF); a defect is detected when at least `min_points` of its
mark points are inside the image (the camera sees only ~12 x 9 cm of the wall).

  params  layout        path to the layout YAML (wcr_description/worlds/inspection/<world>.yaml)
          log_file      CSV of first detections (default: results/inspection_log_<world>.csv)
          min_points    mark points that must be in view (4)
  sub     /ground_truth, /wall_camera/camera_info, /wall_camera/image_color
  pub     /inspection/detections        std_msgs/String  JSON per first detection
          /inspection/markers           visualization_msgs/MarkerArray  (frame world)
              defects: grey = not yet found, red crack / blue wet patch / orange weep hole = found;
              obstacles on the map (bush, pipe, ledge); the wall camera footprint (yellow)
          /inspection/wall_camera_annotated  sensor_msgs/Image  boxes around defects in view
"""
import csv
import json
import math
import os
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

DEFECTS = ('crack', 'wet_patch', 'weep_hole')
FOUND_RGB = {'crack': (0.95, 0.15, 0.1), 'wet_patch': (0.15, 0.45, 1.0), 'weep_hole': (1.0, 0.6, 0.1)}
BOX_BGR = {'crack': (40, 40, 240), 'wet_patch': (255, 120, 40), 'weep_hole': (30, 150, 255)}


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class InspectionDetector(Node):
    def __init__(self):
        super().__init__('inspection_detector')
        layout_path = self.declare_parameter('layout', '').value
        doc = yaml.safe_load(Path(layout_path).read_text())
        self.world = doc['world']
        self.face = {k: np.array(v) for k, v in doc['face'].items()}
        self.items = doc['items']
        self.defects = [it for it in self.items if it['type'] in DEFECTS]
        repo = Path(os.path.realpath(layout_path)).parents[3]
        default_log = repo / 'results' / f'inspection_log_{self.world}.csv'
        self.log_file = Path(self.declare_parameter('log_file', str(default_log)).value)
        self.min_points = self.declare_parameter('min_points', 4).value
        self.found = {}
        self.K = None
        self.size = None
        self.T_base_cam = None
        self.pose = None
        self.history = []
        self.in_view = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/ground_truth', self.on_pose, 10)
        self.create_subscription(CameraInfo, '/wall_camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/wall_camera/image_color', self.on_image, qos_profile_sensor_data)
        self.pub_det = self.create_publisher(String, '/inspection/detections', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/inspection/markers', 10)
        self.pub_img = self.create_publisher(Image, '/inspection/wall_camera_annotated', 10)
        self.create_timer(0.1, self.check)
        self.create_timer(1.0, self.publish_markers)
        self.create_timer(5.0, self.status)
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        with open(self.log_file, 'w', newline='') as f:
            csv.writer(f).writerow(('sim_time_s', 'id', 'type', 's_m', 't_m', 'x', 'y', 'z',
                                    'robot_x', 'robot_y', 'robot_z', 'points_in_view', 'visible_fraction'))
        self.get_logger().info(f'{self.world}: {len(self.defects)} defects to find, log -> {self.log_file}')

    # -- inputs
    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (np.array([p.x, p.y, p.z]), quat_to_R(q.x, q.y, q.z, q.w), msg.header.stamp)
        self.history.append(self.pose)
        del self.history[:-100]                          # ~2 s at 50 Hz, for matching image stamps

    def on_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.size = (msg.width, msg.height)

    def camera_in_world(self, pose=None):
        if self.T_base_cam is None:
            try:
                t = self.tf_buffer.lookup_transform('base_link', 'wall_camera_optical', rclpy.time.Time())
            except Exception:
                return None
            tr, q = t.transform.translation, t.transform.rotation
            self.T_base_cam = (np.array([tr.x, tr.y, tr.z]), quat_to_R(q.x, q.y, q.z, q.w))
        pb, Rb, _ = pose or self.pose
        tc, Rc = self.T_base_cam
        return pb + Rb @ tc, Rb @ Rc

    def project(self, P, cam):
        """World points -> pixel coordinates (NaN behind the camera)."""
        c, R = cam
        pc = (P - c) @ R                                    # R columns = camera axes in world
        z = pc[:, 2]
        uv = (self.K @ pc.T).T
        with np.errstate(divide='ignore', invalid='ignore'):
            uv = uv[:, :2] / uv[:, 2:3]
        uv[z <= 0.01] = np.nan
        return uv

    def visible_fraction(self, it, cam):
        """(number of the defect's mark points in the image, fraction of them, their pixels).
        Mark points are pixels of the defect itself (crack line, damp area) placed on the wall
        by tools/inspection.py; only those count, not the empty part of the decal."""
        if self.T_base_cam is None:
            return 0, 0.0, None
        P = np.array(it['mark'])
        uv = self.project(P, cam)
        w, h = self.size
        m = 0.02 * w
        ok = ~np.isnan(uv).any(axis=1)
        ok[ok] = (uv[ok, 0] >= m) & (uv[ok, 0] < w - m) & (uv[ok, 1] >= m) & (uv[ok, 1] < h - m)
        n = int(ok.sum())
        return n, n / len(P), (uv[ok] if n else None)

    # -- detection
    def status(self):
        """Once every 5 s: which inputs are missing, or the closest defect and how much is in view."""
        if self.pose is None or self.K is None or self.camera_in_world() is None:
            self.get_logger().info(f'waiting for: {"ground truth " if self.pose is None else ""}'
                                   f'{"camera_info " if self.K is None else ""}'
                                   f'{"TF base_link->wall_camera_optical" if self.T_base_cam is None else ""}')
            return
        cam = self.camera_in_world()
        best = min(self.defects, key=lambda it: np.linalg.norm(np.array(it['centre']) - cam[0]))
        n, frac, _ = self.visible_fraction(best, cam)
        self.get_logger().info(f'camera at {np.round(cam[0], 2)}, closest defect {best["id"]} '
                               f'{np.linalg.norm(np.array(best["centre"]) - cam[0]):.2f} m away, '
                               f'{n} of its points in view (found {len(self.found)}/{len(self.defects)})')

    def check(self):
        if self.pose is None or self.K is None:
            return
        cam = self.camera_in_world()
        if cam is None:
            return
        self.in_view = []
        t = self.pose[2].sec + self.pose[2].nanosec * 1e-9
        for it in self.defects:
            n, frac, uv = self.visible_fraction(it, cam)
            if uv is not None:
                self.in_view.append((it, uv))
            if n >= self.min_points and it['id'] not in self.found:
                self.found[it['id']] = t
                rp = self.pose[0]
                rec = dict(time=round(t, 2), id=it['id'], type=it['type'], s=round(it['s'], 3), t=round(it['t'], 3),
                           position=it['centre'], robot=[round(float(v), 3) for v in rp], points_in_view=n,
                           visible=round(frac, 2))
                self.pub_det.publish(String(data=json.dumps(rec)))
                with open(self.log_file, 'a', newline='') as f:
                    csv.writer(f).writerow((rec['time'], it['id'], it['type'], rec['s'], rec['t'], *it['centre'], *rec['robot'],
                                            n, rec['visible']))
                self.get_logger().info(f'DETECTED {it["type"]} {it["id"]} at s {it["s"]:.2f} m, t {it["t"]:.2f} m '
                                       f'({len(self.found)}/{len(self.defects)})')
                self.publish_markers()

    def on_image(self, msg):
        if self.pub_img.get_subscription_count() == 0 or msg.encoding not in ('bgra8', 'bgr8', 'rgba8', 'rgb8'):
            return
        ch = 4 if msg.encoding.endswith('a8') else 3
        img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step // ch, ch)[:, :msg.width].copy()
        # project with the robot pose closest to the image's capture time: at 0.12 m/s the robot
        # moves ~1 cm between frames, tens of pixels this close to the wall
        t_img = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = min(self.history, key=lambda p: abs(p[2].sec + p[2].nanosec * 1e-9 - t_img), default=None)
        cam = self.camera_in_world(pose) if pose is not None and self.K is not None else None
        boxes = []
        if cam is not None:
            for it, _ in self.in_view:
                _, _, uv = self.visible_fraction(it, cam)
                if uv is not None:
                    boxes.append((it, uv))
        for it, uv in boxes:
            x0, y0 = np.clip(uv.min(0), 0, [msg.width - 1, msg.height - 1]).astype(int)
            x1, y1 = np.clip(uv.max(0), 0, [msg.width - 1, msg.height - 1]).astype(int)
            col = BOX_BGR[it['type']] if it['id'] in self.found else (160, 160, 160)
            if msg.encoding.startswith('rgb'):
                col = col[::-1]
            for th in range(3):
                img[max(y0 - th, 0):y1 + th + 1, max(x0 - th, 0)] = col[:ch] if ch == 3 else (*col, 255)
                img[max(y0 - th, 0):y1 + th + 1, min(x1 + th, msg.width - 1)] = col[:ch] if ch == 3 else (*col, 255)
                img[max(y0 - th, 0), max(x0 - th, 0):x1 + th + 1] = col[:ch] if ch == 3 else (*col, 255)
                img[min(y1 + th, msg.height - 1), max(x0 - th, 0):x1 + th + 1] = col[:ch] if ch == 3 else (*col, 255)
        out = Image(header=msg.header, height=msg.height, width=msg.width, encoding=msg.encoding,
                    is_bigendian=msg.is_bigendian, step=msg.width * ch, data=img.tobytes())
        self.pub_img.publish(out)

    # -- RViz
    def publish_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        n = self.face['normal']

        def marker(mid, ns, kind):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'world', stamp
            m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
            m.pose.orientation.w = 1.0
            return m
        for i, it in enumerate(self.items):
            found = it['id'] in self.found
            if it['type'] in DEFECTS:
                m = marker(i, 'defects', Marker.LINE_STRIP)
                m.scale.x = 0.025 if found else 0.012
                r, g, b = FOUND_RGB[it['type']] if found else (0.6, 0.6, 0.6)
                m.color = ColorRGBA(r=r, g=g, b=b, a=1.0 if found else 0.6)
                for p in it['corners'] + [it['corners'][0]]:
                    q = np.array(p) + n * 0.02
                    m.points.append(Point(x=q[0], y=q[1], z=q[2]))
                ma.markers.append(m)
                if found:
                    tm = marker(i, 'labels', Marker.TEXT_VIEW_FACING)
                    q = np.array(it['centre']) + n * 0.15
                    tm.pose.position = Point(x=q[0], y=q[1], z=q[2])
                    tm.scale.z = 0.12
                    tm.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
                    tm.text = it['id']
                    ma.markers.append(tm)
            elif it.get('mapped', True):                    # obstacles on the known map
                m = marker(i, 'obstacles', Marker.SPHERE if it['type'] in ('bush', 'debris') else Marker.CUBE)
                q = np.array(it['centre']) + n * 0.15
                m.pose.position = Point(x=q[0], y=q[1], z=q[2])
                ls, lt = it['size']
                m.scale.x = m.scale.y = m.scale.z = max(ls, lt) if it['type'] == 'bush' else 0.3
                if it['type'] in ('pipe', 'ledge'):
                    up, ac = self.face['up'], self.face['across']
                    # axis-aligned approximation in world; good enough as a map hint
                    ext = np.abs(up) * ls + np.abs(ac) * lt + np.abs(n) * 0.3
                    m.scale.x, m.scale.y, m.scale.z = (float(max(v, 0.05)) for v in ext)
                m.color = ColorRGBA(r=0.2, g=0.75, b=0.3, a=0.5) if it['type'] == 'bush' else ColorRGBA(r=0.7, g=0.7, b=0.75, a=0.5)
                ma.markers.append(m)
        if self.pose is not None and self.K is not None:
            cam = self.camera_in_world()
            if cam is not None:                              # footprint of the wall camera on the face
                c, R = cam
                w, h = self.size
                fp = marker(0, 'camera_footprint', Marker.LINE_STRIP)
                fp.scale.x = 0.01
                fp.color = ColorRGBA(r=1.0, g=0.9, b=0.1, a=1.0)
                Kinv = np.linalg.inv(self.K)
                base = self.face['base']
                for u, v in ((0, 0), (w, 0), (w, h), (0, h), (0, 0)):
                    d = R @ (Kinv @ np.array([u, v, 1.0]))
                    s = ((base - c) @ n) / (d @ n) if abs(d @ n) > 1e-6 else 0
                    q = c + d * s + n * 0.02
                    fp.points.append(Point(x=q[0], y=q[1], z=q[2]))
                ma.markers.append(fp)
        self.pub_mk.publish(ma)

    def summary(self):
        lines = [f'inspection {self.world}: found {len(self.found)} of {len(self.defects)} defects']
        lines += [f'  {it["id"]:12s} {"found at t=%.1f s" % self.found[it["id"]] if it["id"] in self.found else "-"}'
                  for it in self.defects]
        return '\n'.join(lines)


def main():
    rclpy.init()
    node = InspectionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(node.summary(), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
