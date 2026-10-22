#!/usr/bin/env python3
"""Exit once the robot is spawned and the simulation is really running.

Gazebo blocks while it loads large visual meshes; controller spawners that call the
controller manager during that time hit Humble's hard-coded 10 s service timeout and
fail. The launcher starts the spawners when this node exits: after the first
ground-truth odometry message, plus `settle` seconds of advancing sim time.
"""
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class WaitForSim(Node):

    def __init__(self):
        super().__init__('wait_for_sim')
        self.settle = self.declare_parameter('settle', 2.0).value
        self.timeout = self.declare_parameter('timeout', 300.0).value
        self.t_first = None
        self.done = False
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_odom, 10)

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_first is None:
            self.t_first = t
            self.get_logger().info('robot is up, waiting for the simulation to settle')
        elif t - self.t_first >= self.settle:
            self.done = True


def main():
    rclpy.init()
    node = WaitForSim()
    t0 = time.time()
    while rclpy.ok() and not node.done and time.time() - t0 < node.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.done:
        node.get_logger().info(f'simulation running after {time.time() - t0:.1f} s; starting controllers')
    else:
        node.get_logger().warn('timed out waiting for the simulation; starting controllers anyway')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
