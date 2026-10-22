"""Adhesion force needed to hold and drive on an inclined surface (see tools/design_math.py)."""
import math

G = 9.81


def surface_angle(qx, qy, qz, qw):
    """Angle [rad] between the body z axis (surface normal) and world up.
    0 = floor, pi/2 = vertical wall, > pi/2 = overhang."""
    # z component of R(q) * (0, 0, 1)
    zz = 1.0 - 2.0 * (qx * qx + qy * qy)
    return math.acos(max(-1.0, min(1.0, zz)))


def required_force(beta, mass, mu, com_height, half_wheelbase):
    """Minimum normal force F [N] so the wheels neither slip nor peel off.

    slip:  mu (F + m g cos b) >= m g sin b
    peel:  F * L/2 >= m g sin b * h   (upper wheels stay loaded)
    """
    w = mass * G
    s, c = math.sin(beta), math.cos(beta)
    slip = w * (s / mu - c)
    peel = w * abs(s) * com_height / half_wheelbase
    return max(0.0, slip, peel)
