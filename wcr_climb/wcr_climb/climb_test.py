#!/usr/bin/env python3
"""Scripted climb test: settle -> climb -> hold -> descend, logged to CSV.

Uses the ground-truth odometry from Gazebo (ground_truth/odom, world frame) and
drives through cmd_vel (inv_kin_controller). Prints a PASS/FAIL summary:
  * climb speed >= min_speed_ratio * commanded (wheels do not slip),
  * drift while holding (after 0.5 s of braking) <= max_hold_drift,
  * the robot stays on the face (never ends up below its start height).
"""
import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

from wcr_climb.adhesion import surface_angle


class ClimbTest(Node):

    def __init__(self):
        super().__init__('climb_test')
        p = self.declare_parameter
        self.speed = p('speed', 0.10).value
        self.distance = p('distance', 2.0).value
        self.settle = p('settle', 4.0).value
        self.hold = p('hold', 5.0).value
        self.descend = p('descend', True).value
        self.timeout = p('timeout', 120.0).value
        self.min_speed_ratio = p('min_speed_ratio', 0.8).value
        self.max_hold_drift = p('max_hold_drift', 0.005).value
        self.csv_path = p('csv', 'climb_test.csv').value

        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_odom, 20)
        self.create_subscription(Float64, 'thrust/edf', self.on_force, 10)
        self.pose = None
        self.force = float('nan')
        self.rows = []
        self.phase = 'wait'
        self.t_phase = None
        self.p_phase = None
        self.travel = 0.0
        self.last_p = None
        self.result = {}
        self.create_timer(0.05, self.step)
        self.t0 = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = ((p.x, p.y, p.z), surface_angle(q.x, q.y, q.z, q.w))

    def on_force(self, msg):
        self.force = msg.data

    def go(self, phase):
        self.get_logger().info(f'phase {self.phase} -> {phase}')
        self.phase, self.t_phase, self.p_phase = phase, self.now(), self.pose[0]
        self.travel = 0.0

    def drive(self, v):
        t = Twist()
        t.linear.x = float(v)
        self.cmd.publish(t)

    def step(self):
        if self.pose is None:
            return
        t = self.now()
        if self.t0 is None:
            self.t0 = t
            self.start = self.pose[0]
            self.go('settle')
        p, beta = self.pose
        if self.last_p is not None:
            self.travel += math.dist(p, self.last_p)
        self.last_p = p
        el = t - self.t_phase
        self.rows.append((round(t - self.t0, 3), self.phase, *[round(a, 4) for a in p],
                          round(math.degrees(beta), 2), round(self.force, 2)))

        if t - self.t0 > self.timeout:
            self.finish('timeout')
        elif self.phase == 'settle':
            self.drive(0.0)
            if el >= self.settle:
                self.result['settle_slide_m'] = math.dist(p, self.start)
                self.go('climb')
        elif self.phase == 'climb':
            self.drive(self.speed)
            if self.travel >= self.distance:
                self.result['climb_speed'] = self.travel / el
                self.result['climb_height_m'] = p[2] - self.start[2]
                self.result['beta_deg'] = math.degrees(beta)
                self.go('hold')
            elif el > 3 * self.distance / self.speed:
                self.result['climb_speed'] = self.travel / el
                self.finish('climb too slow (slipping?)')
        elif self.phase == 'hold':
            self.drive(0.0)
            if el < 0.5:                      # still braking from climb speed
                self.p_hold = p
            elif el >= self.hold:
                self.result['stop_dist_m'] = math.dist(self.p_hold, self.p_phase)
                self.result['hold_drift_m'] = math.dist(p, self.p_hold)
                if self.descend:
                    self.go('descend')
                else:
                    self.finish('done')
        elif self.phase == 'descend':
            self.drive(-self.speed)
            if self.travel >= self.distance * 0.9 or el > 3 * self.distance / self.speed:
                self.result['descend_speed'] = self.travel / el
                self.drive(0.0)
                self.finish('done')
        if p[2] < self.start[2] - 0.05 and self.phase in ('climb', 'hold'):
            self.finish('fell / slid below start height')

    def finish(self, why):
        if self.phase == 'end':
            return
        self.drive(0.0)
        self.phase = 'end'
        r = self.result
        ok = (why == 'done'
              and r.get('climb_speed', 0) >= self.min_speed_ratio * self.speed
              and r.get('hold_drift_m', 1) <= self.max_hold_drift)
        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'phase', 'x', 'y', 'z', 'surface_deg', 'adhesion_N'])
            w.writerows(self.rows)
        lines = [f'climb test: {why} -> {"PASS" if ok else "FAIL"}']
        for k, v in r.items():
            lines.append(f'  {k:16s} {v:.4f}')
        lines.append(f'  commanded speed  {self.speed:.4f}   log: {self.csv_path}')
        print('\n'.join(lines), flush=True)
        self.ok = ok
        raise SystemExit(0 if ok else 1)


def main():
    rclpy.init()
    node = ClimbTest()
    code = 1
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = e.code
    except KeyboardInterrupt:
        node.finish('interrupted')
    finally:
        node.drive(0.0)
        time.sleep(0.2)
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
