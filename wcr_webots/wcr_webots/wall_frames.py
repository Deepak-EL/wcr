"""Static frames of the wall (Phase 4).

  world -> map    the dam face: origin = the layout's face base point, x up the face, y across
                  (robot's left when facing up), z out of the wall. Nav2 plans in this plane.
  map -> odom     where the robot started: its first /ground_truth pose, expressed in the map
                  frame and flattened onto the plane (x, y, yaw). Odometry (odom -> base_link,
                  from the EKF or the driver) is relative to it.

  param  layout   inspection layout YAML (its 'face' block defines the map frame)
"""
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def quat_from_R(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2 * math.sqrt(tr + 1)
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
    q[i], q[j], q[k] = s / 4, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    return tuple(q)


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class WallFrames(Node):
    def __init__(self):
        super().__init__('wall_frames')
        doc = yaml.safe_load(Path(self.declare_parameter('layout', '').value).read_text())
        f = doc['face']
        self.base = np.array(f['base'])
        self.R_wm = np.c_[f['up'], f['across'], f['normal']]          # map axes in world
        self.static = StaticTransformBroadcaster(self)
        self.tf_world_map = self.make('world', 'map', self.base, self.R_wm)
        self.static.sendTransform(self.tf_world_map)
        self.done = False
        self.create_subscription(Odometry, '/ground_truth', self.on_pose, 10)
        self.get_logger().info(f'{doc["world"]}: map frame on the face at {np.round(self.base, 3)}')

    def make(self, parent, child, p, R):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in p)
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = quat_from_R(R)
        return t

    def on_pose(self, msg):
        if self.done:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        R_wb = quat_to_R(q.x, q.y, q.z, q.w)
        pm = self.R_wm.T @ (np.array([p.x, p.y, p.z]) - self.base)    # start position in the map
        xb = self.R_wm.T @ R_wb[:, 0]                                   # robot x axis in the map
        yaw = math.atan2(xb[1], xb[0])
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        self.static.sendTransform([self.tf_world_map, self.make('map', 'odom', (pm[0], pm[1], 0.0), Rz)])
        self.done = True
        self.get_logger().info(f'map -> odom: start at x {pm[0]:.3f} m (up the face), y {pm[1]:.3f} m, '
                               f'yaw {math.degrees(yaw):.1f} deg ({pm[2] * 1000:.1f} mm off the face plane)')


def main():
    rclpy.init()
    node = WallFrames()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
