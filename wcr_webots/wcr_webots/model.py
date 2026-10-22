"""WCR models shared by the Webots controller (webots/controllers/wcr_climb) and the ROS 2 driver
plugin (wcr_webots.wcr_driver). Pure Python, no ROS or Webots imports.

  adhesion   F = safety * max(m g (sin b / mu - cos b), m g sin b h / (L/2)), b = surface angle
             from the IMU, clamped to [0, F_MAX], rate-limited; applied along -z of the robot
             (same as wcr_climb/adhesion.py on the Gazebo side)
  drive      4-wheel independent steering: body twist -> per-wheel speed + steering angle
             (wcr_controllers/inv_kin_controller) and back (wheel odometry)
"""
import math

G = 9.81
MASS, MU, COM_H, HALF_WB = 2.01, 0.7, 0.083, 0.1125
SAFETY, F_MAX, F_START, F_RATE = 1.5, 45.0, 30.0, 60.0          # N, N, N, N/s
WHEELS = {'FL': (0.1125, 0.1125), 'BL': (-0.1125, 0.1125), 'BR': (-0.1125, -0.1125), 'FR': (0.1125, -0.1125)}
WHEEL_R = 0.0254
MAX_W = 210 * 0.229 * 2 * math.pi / 60                            # rad/s (Dynamixel units, as inv_kin)
MAX_SPEED = MAX_W * WHEEL_R                                         # 0.128 m/s, the real robot's limit
speed_scale = 1.0          # > 1 only for faster-than-real simulation demos (set_max_speed)


def set_max_speed(v):
    """Allow wheel speeds up to v m/s (simulation only; default = the real motors' MAX_SPEED)."""
    global speed_scale
    speed_scale = max(1.0, v / MAX_SPEED)


def edf_speed(force):
    """Visual EDF rotor speed [rad/s] for an adhesion force [N] (idle 15, full 60)."""
    return 15.0 + 45.0 * max(0.0, min(1.0, force / F_MAX))


def required_force(beta):
    s, c = math.sin(beta), math.cos(beta)
    return max(0.0, MASS * G * (s / MU - c), MASS * G * abs(s) * COM_H / HALF_WB)


def surface_angle(q):
    """Angle between the robot's z axis and world up, from an (x, y, z, w) quaternion; None if
    the reading is invalid (Webots returns garbage before the first step)."""
    if not q or not all(math.isfinite(v) and abs(v) <= 1.01 for v in q):
        return None
    return math.acos(max(-1.0, min(1.0, 1 - 2 * (q[0] ** 2 + q[1] ** 2))))


class Adhesion:
    """Rate-limited EDF + skirt adhesion force [N] from the surface angle."""

    def __init__(self):
        self.force = F_START
        self.beta = None

    def update(self, quaternion, dt):
        beta = surface_angle(quaternion)
        if beta is not None:
            self.beta = beta
            target = min(F_MAX, SAFETY * required_force(beta))
            lim = F_RATE * dt
            self.force += max(-lim, min(lim, target - self.force))
        return self.force


def inverse_kinematics(vx, vy, wz, prev=None):
    """Body twist -> {wheel: (wheel speed rad/s, steering angle rad)}, steering within +-90 deg,
    speeds scaled together so none exceeds MAX_W.

    Each wheel direction has two equivalent solutions (angle, speed) and (angle -+ 180 deg, -speed).
    With `prev` ({wheel: current steering angle}) the one closest to the current angle is used and
    a stopped wheel keeps its angle; without it, crabbing straight sideways (90 deg) flips between
    +90 and -90 deg on tiny forward components and the wheels never settle."""
    out, scale = {}, 1.0
    for n, (x, y) in WHEELS.items():
        vxi, vyi = vx - y * wz, vy + x * wz
        v = math.hypot(vxi, vyi)
        keep = prev[n] if prev is not None else 0.0
        out[n] = [v / WHEEL_R, math.atan2(vyi, vxi) if v > 1e-6 else keep]
        scale = max(scale, abs(out[n][0]) / (MAX_W * speed_scale))
    lim, slack = math.pi / 2, 0.15             # steering joint limit; a few degrees past it is
    for n in out:                               # clamped (error <= 8.6 deg) rather than flipped
        om, d = out[n][0] / scale, out[n][1]
        cands = [c for c in ((om, d), (-om, d - math.pi), (-om, d + math.pi)) if abs(c[1]) <= lim + slack]
        ref = prev[n] if prev is not None else 0.0
        om, d = min(cands, key=lambda c: abs(c[1] - ref))
        out[n] = (om, max(-lim, min(lim, d)))
    return out


def forward_kinematics(wheel_speed, steering):
    """Least-squares body twist (vx, vy, wz) from {wheel: rad/s} and {wheel: steering rad}."""
    sx = sy = sw = 0.0
    for n, (x, y) in WHEELS.items():
        v = wheel_speed[n] * WHEEL_R
        vxi, vyi = v * math.cos(steering[n]), v * math.sin(steering[n])
        sx += vxi
        sy += vyi
        sw += x * vyi - y * vxi
    k = len(WHEELS)
    r2 = sum(x * x + y * y for x, y in WHEELS.values())
    return sx / k, sy / k, sw / r2
