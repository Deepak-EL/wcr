"""webots_ros2_driver Python plugin for the WCR (robot controller "<extern>").

Runs the same adhesion and 4WIS drive as the Webots controller (wcr_webots.model) and bridges
the robot to ROS 2. The sensors themselves (lidar, cameras, IMU) are published by the standard
webots_ros2 device interfaces declared in resource/wcr_webots_driver.urdf.

  sub  /cmd_vel        geometry_msgs/Twist   body twist on the wall: x up the face, y across, z yaw
                                             (stops if no command for 0.5 s)
  pub  /odom           nav_msgs/Odometry     wheel odometry in the wall plane (frame odom)
       /joint_states   sensor_msgs/JointState  steering + wheel angles (encoders)
       /ground_truth   nav_msgs/Odometry     true 3D pose in the Webots world (frame world)
       /adhesion       std_msgs/Float32      applied adhesion force [N]
       /clock          rosgraph_msgs/Clock   simulation time (every step)
  tf   odom -> base_link (wheel odometry), unless WCR_ODOM_TF=0 in the environment (when the
       robot_localization EKF publishes it). world -> map -> odom come from wall_frames.
"""
import math
import os

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from wcr_webots import model
from wcr_webots.model import WHEELS, Adhesion, edf_speed, forward_kinematics, inverse_kinematics

CMD_TIMEOUT = 0.5          # s
CRADLE_RELEASE = 4.0       # s of sim time: the adhesion is fully on, the deployment cradle retracts


def diag6(*v):
    """Row-major 6x6 covariance (36 floats) with the given diagonal."""
    return [float(v[i]) if i == j else 0.0 for i in range(6) for j in range(6)]


# planar odometry: x, y, yaw meaningful; z, roll, pitch fixed (large variance)
POSE_COV = diag6(1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2)
TWIST_COV = diag6(1e-4, 1e-4, 1e6, 1e6, 1e6, 1e-3)


def quat_from_matrix(R):
    """(x, y, z, w) from a row-major 3x3 rotation matrix (list of 9)."""
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = R
    tr = m00 + m11 + m22
    if tr > 0:
        s = 2 * math.sqrt(tr + 1)
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, s / 4)
    if m00 > m11 and m00 > m22:
        s = 2 * math.sqrt(1 + m00 - m11 - m22)
        return (s / 4, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = 2 * math.sqrt(1 + m11 - m00 - m22)
        return ((m01 + m10) / s, s / 4, (m12 + m21) / s, (m02 - m20) / s)
    s = 2 * math.sqrt(1 + m22 - m00 - m11)
    return ((m02 + m20) / s, (m12 + m21) / s, s / 4, (m10 - m01) / s)


class WcrDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot                    # a Supervisor (adhesion uses addForce)
        self.dt = self.robot.getBasicTimeStep() / 1000.0
        self.self_node = self.robot.getSelf()
        ms = int(self.robot.getBasicTimeStep())
        self.wheel = {n: self.robot.getDevice(f'{n}_wheel') for n in WHEELS}
        self.steer = {n: self.robot.getDevice(f'{n}_steering') for n in WHEELS}
        self.wheel_enc = {n: self.robot.getDevice(f'{n}_wheel_sensor') for n in WHEELS}
        self.steer_enc = {n: self.robot.getDevice(f'{n}_steering_sensor') for n in WHEELS}
        for m in self.wheel.values():
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
        for s in [*self.wheel_enc.values(), *self.steer_enc.values()]:
            s.enable(ms)
        self.imu = self.robot.getDevice('imu')
        self.imu.enable(ms)
        self.edf = self.robot.getDevice('edf_motor')                # visual EDF rotor
        self.edf.setPosition(float('inf'))
        self.edf_enc = self.robot.getDevice('edf_sensor')
        self.edf_enc.enable(ms)
        self.adhesion = Adhesion()
        self.period = float(properties.get('publishPeriodMs', '20')) / 1000.0
        # WCR_MAX_SPEED (m/s, from the launch file's speed:=) above 0.128 = faster-than-real demo
        model.set_max_speed(float(os.environ.get('WCR_MAX_SPEED', model.MAX_SPEED)))

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node('wcr_driver')
        self.node.create_subscription(Twist, 'cmd_vel', self._on_cmd, 1)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_odom = self.node.create_publisher(Odometry, 'odom', qos)
        self.pub_gt = self.node.create_publisher(Odometry, 'ground_truth', qos)
        self.pub_js = self.node.create_publisher(JointState, 'joint_states', qos)
        self.pub_adh = self.node.create_publisher(Float32, 'adhesion', qos)
        # simulation clock for every node with use_sim_time (webots_ros2 only publishes it from its
        # optional Ros2Supervisor, which this setup does not run)
        self.pub_clock = self.node.create_publisher(Clock, '/clock', 10)
        self.tf = TransformBroadcaster(self.node) if os.environ.get('WCR_ODOM_TF', '1') != '0' else None
        self.cmd, self.t_cmd = (0.0, 0.0, 0.0), -1.0
        self.x = self.y = self.yaw = 0.0
        self.prev_wheel = None
        self.t_pub = -1.0
        self.cradle_removed = False

    def _on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.t_cmd = self.robot.getTime()

    def _remove_cradle(self):
        """The deployment cradle (a 6 cm ledge under the rear wheels) only holds the robot until the
        adhesion is on. It is too low for the lidar and not on the map, so once the robot drives off
        it caught the wheels when they came back near the start (and the odometry ran away);
        it is removed from the world, like a cradle that retracts after deployment."""
        children = self.robot.getRoot().getField('children')
        for i in reversed(range(children.getCount())):
            node = children.getMFNode(i)
            name = node.getField('name') if node.getTypeName() == 'Solid' else None
            if name is not None and name.getSFString().startswith('cradle'):
                node.remove()
        self.cradle_removed = True

    def _stamp(self, t):
        return Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        t = self.robot.getTime()
        self.pub_clock.publish(Clock(clock=self._stamp(t)))
        vx, vy, wz = self.cmd if t - self.t_cmd < CMD_TIMEOUT else (0.0, 0.0, 0.0)
        cur = {n: s.getValue() for n, s in self.steer_enc.items()}       # current steering angles
        cur = cur if all(math.isfinite(v) for v in cur.values()) else None
        for n, (om, d) in inverse_kinematics(vx, vy, wz, cur).items():
            self.steer[n].setPosition(d)
            self.wheel[n].setVelocity(om)
        f = self.adhesion.update(self.imu.getQuaternion(), self.dt)
        self.self_node.addForce([0.0, 0.0, -f], True)       # into the surface, robot frame
        if not self.cradle_removed and t > CRADLE_RELEASE:
            self._remove_cradle()
        self.edf.setVelocity(edf_speed(f))

        # wheel odometry in the wall plane
        wheel = {n: s.getValue() for n, s in self.wheel_enc.items()}
        steer = {n: s.getValue() for n, s in self.steer_enc.items()}
        if not all(math.isfinite(v) for v in [*wheel.values(), *steer.values()]):
            return
        if self.prev_wheel is None:
            self.prev_wheel = wheel
        speed = {n: (wheel[n] - self.prev_wheel[n]) / self.dt for n in WHEELS}
        self.prev_wheel = wheel
        bvx, bvy, bwz = forward_kinematics(speed, steer)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * bvx - s * bvy) * self.dt
        self.y += (s * bvx + c * bvy) * self.dt
        self.yaw += bwz * self.dt

        if t - self.t_pub < self.period - 1e-9:
            return
        self.t_pub = t
        stamp = self._stamp(t)
        q = (0.0, 0.0, math.sin(self.yaw / 2), math.cos(self.yaw / 2))
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
        p = odom.pose.pose
        p.position.x, p.position.y = self.x, self.y
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = bvx, bvy, bwz
        # planar odometry: x, y, yaw meaningful; z, roll, pitch fixed (large variance)
        odom.pose.covariance = POSE_COV
        odom.twist.covariance = TWIST_COV
        self.pub_odom.publish(odom)
        if self.tf is not None:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
            r = tf.transform.rotation
            r.x, r.y, r.z, r.w = q
            self.tf.sendTransform(tf)

        js = JointState()
        js.header.stamp = stamp
        edf = self.edf_enc.getValue()
        js.name = [f'{n}_{j}_joint' for n in WHEELS for j in ('steering', 'wheel')] + ['edf_joint']
        js.position = [float(v) for n in WHEELS for v in (steer[n], wheel[n])] + [float(edf if math.isfinite(edf) else 0.0)]
        js.velocity = [float(v) for n in WHEELS for v in (0.0, speed[n])] + [float(edf_speed(f))]
        self.pub_js.publish(js)

        gt = Odometry()
        gt.header.stamp, gt.header.frame_id, gt.child_frame_id = stamp, 'world', 'base_link'
        pos, R = self.self_node.getPosition(), self.self_node.getOrientation()
        gp = gt.pose.pose
        gp.position.x, gp.position.y, gp.position.z = pos
        gp.orientation.x, gp.orientation.y, gp.orientation.z, gp.orientation.w = quat_from_matrix(R)
        self.pub_gt.publish(gt)
        self.pub_adh.publish(Float32(data=float(f)))
