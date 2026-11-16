import math

from rover_core.kinematics import (WheelOdometry, limit_twist, sides_to_twist,
                                   twist_to_sides, wrap_angle)
from rover_core.params import DEFAULT, MotorParams


def test_twist_round_trip():
    vl, vr = twist_to_sides(0.3, 0.5, 0.87)
    v, w = sides_to_twist(vl, vr, 0.87)
    assert math.isclose(v, 0.3) and math.isclose(w, 0.5)


def test_limit_twist_preserves_curvature():
    v, w = limit_twist(10.0, 5.0)
    assert abs(v) <= DEFAULT.max_speed + 1e-9 and abs(w) <= DEFAULT.max_yaw_rate + 1e-9


def test_motor_model_consistency():
    m = MotorParams()
    assert math.isclose(m.torque_at(m.rated_rpm * 2 * math.pi / 60), m.rated_torque, rel_tol=1e-9)
    assert m.torque_at(m.no_load_omega) == 0.0
    assert 18 < m.rated_output_power < 21  # datasheet: 20 W


def _ticks(dl, dr):
    c = 1.0 / DEFAULT.metres_per_count
    return [int(round(dl * c))] * 3 + [int(round(dr * c))] * 3


def test_straight_line():
    odo = WheelOdometry()
    odo.update([0] * 6, 0.0)
    odo.update(_ticks(1.0, 1.0), 1.0)
    assert math.isclose(odo.pose.x, 1.0, abs_tol=2e-3) and abs(odo.pose.y) < 1e-6


def test_rotate_in_place():
    odo = WheelOdometry()
    odo.update([0] * 6, 0.0)
    arc = math.pi / 2 * DEFAULT.b_eff / 2
    odo.update(_ticks(-arc, arc), 1.0)
    assert math.isclose(odo.pose.theta, math.pi / 2, abs_tol=5e-3)
    assert math.hypot(odo.pose.x, odo.pose.y) < 1e-3


def test_median_rejects_single_slipping_wheel():
    odo = WheelOdometry()
    odo.update([0] * 6, 0.0)
    t = _ticks(1.0, 1.0)
    t[1] += 5000  # middle-left wheel spins freely
    odo.update(t, 1.0)
    assert abs(odo.pose.theta) < 1e-6 and math.isclose(odo.pose.x, 1.0, abs_tol=2e-3)


def test_counter_wrap():
    odo = WheelOdometry()
    start = [2 ** 31 - 10] * 6
    odo.update(start, 0.0)
    odo.update([-(2 ** 31) + 10] * 6, 1.0)  # +20 counts across the wrap
    assert math.isclose(odo.pose.x, 20 * DEFAULT.metres_per_count, rel_tol=1e-6)


def test_wrap_angle():
    assert math.isclose(wrap_angle(3 * math.pi), -math.pi) or math.isclose(wrap_angle(3 * math.pi), math.pi)
