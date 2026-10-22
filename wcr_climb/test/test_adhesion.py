import math

from wcr_climb.adhesion import required_force, surface_angle

M, MU, H, L2 = 2.01, 0.7, 0.083, 0.1125


def test_floor_needs_nothing():
    assert required_force(0.0, M, MU, H, L2) == 0.0


def test_matches_design_math_table():
    # tools/design_math.py: slope 53.55 deg -> 11.7 N (peel), wall 89.6 deg -> 28.0 N (slip)
    assert math.isclose(required_force(math.radians(53.55), M, MU, H, L2), 11.7, abs_tol=0.1)
    assert math.isclose(required_force(math.radians(89.6), M, MU, H, L2), 28.0, abs_tol=0.1)


def test_surface_angle_from_quaternion():
    assert math.isclose(surface_angle(0, 0, 0, 1), 0.0, abs_tol=1e-9)
    # pitched by -53.55 deg about y (nose up on the slope) and yawed: angle is still 53.55
    b = math.radians(53.55)
    qy = math.sin(-b / 2)
    qw = math.cos(-b / 2)
    assert math.isclose(math.degrees(surface_angle(0, qy, 0, qw)), 53.55, abs_tol=1e-6)
