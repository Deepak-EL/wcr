#!/usr/bin/env python3
"""EDF adhesion controller.

Publishes the adhesion force command [N] for the EDF Thruster (thrust/edf).

mode 'auto' : F = safety * required_force(surface angle from the IMU orientation),
              clamped to [min_force, max_force] and rate-limited (EDF spool-up).
mode 'fixed': F = fixed_force.
Until the first IMU message arrives, startup_force is held so a robot spawned on
the face is pressed on immediately.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from wcr_climb.adhesion import required_force, surface_angle


class AdhesionController(Node):

    def __init__(self):
        super().__init__('adhesion_controller')
        p = self.declare_parameter
        self.mode = p('mode', 'auto').value
        self.mass = p('mass', 2.01).value               # incl. hanging tether
        self.mu = p('mu', 0.7).value                    # assumed wheel/concrete friction
        self.com_height = p('com_height', 0.083).value
        self.half_wheelbase = p('half_wheelbase', 0.1125).value
        self.safety = p('safety', 1.5).value
        self.min_force = p('min_force', 0.0).value
        self.max_force = p('max_force', 45.0).value
        self.fixed_force = p('fixed_force', 30.0).value
        self.startup_force = p('startup_force', 30.0).value
        self.rate_limit = p('rate_limit', 60.0).value   # N/s
        rate = p('rate', 50.0).value

        self.beta = None
        self.force = self.startup_force
        self.pub = self.create_publisher(Float64, 'thrust/edf', 10)
        self.dbg = self.create_publisher(Float64, 'adhesion/surface_angle_deg', 10)
        self.create_subscription(Imu, 'imu', self.on_imu, 10)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f'mode={self.mode} m={self.mass} kg mu={self.mu} safety={self.safety} '
            f'F in [{self.min_force}, {self.max_force}] N, startup {self.startup_force} N')

    def on_imu(self, msg):
        q = msg.orientation
        self.beta = surface_angle(q.x, q.y, q.z, q.w)

    def target(self):
        if self.mode == 'fixed':
            return self.fixed_force
        if self.beta is None:
            return self.startup_force
        f = self.safety * required_force(self.beta, self.mass, self.mu, self.com_height, self.half_wheelbase)
        return min(self.max_force, max(self.min_force, f))

    def step(self):
        target = self.target()
        if self.beta is None:
            self.force = target           # no ramp before the first IMU message
        else:
            step = self.rate_limit * self.dt
            self.force += max(-step, min(step, target - self.force))
        self.pub.publish(Float64(data=float(self.force)))
        if self.beta is not None:
            self.dbg.publish(Float64(data=float(self.beta * 57.29578)))


def main():
    rclpy.init()
    node = AdhesionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
