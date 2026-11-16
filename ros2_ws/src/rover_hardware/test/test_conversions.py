import math

import numpy as np
import pytest

from rover_core.params import DEFAULT
from rover_core.protocol import (NO_ECHO, PT, ArmSetJoints, ArmState, Drive, EnvTlm, Imu,
                                 SetVelocity, TlmV2, decode_tlm)
from rover_hardware import conversions as cv


# --------------------------------------------------------------- geometry
@pytest.mark.parametrize("rpy", [(0, 0, 0), (0.1, -0.2, 0.3), (-0.5, 0.4, -2.9), (0, 0, math.pi / 2)])
def test_quaternion_round_trip(rpy):
    q = cv.quaternion_from_rpy(*rpy)
    assert math.isclose(sum(c * c for c in q), 1.0, rel_tol=1e-12)
    back = cv.rpy_from_quaternion(*q)
    for a, b in zip(rpy, back):
        assert math.isclose(math.remainder(a - b, 2 * math.pi), 0.0, abs_tol=1e-9)


def test_rover_params_override_keeps_defaults():
    p = cv.rover_params(track=0.60, skid_chi=None)
    assert p.track == 0.60 and p.skid_chi == DEFAULT.skid_chi and p.wheel_radius == DEFAULT.wheel_radius


# --------------------------------------------------------------- mobility
def test_set_velocity_is_limited_and_packed():
    sv = cv.twist_to_set_velocity(10.0, 0.0, DEFAULT, 500)
    assert sv.v_mm_s == round(DEFAULT.max_speed * 1000) and sv.w_mrad_s == 0
    assert sv.timeout_ms == 500
    assert len(sv.pack()) == 6 and SetVelocity.unpack(sv.pack()) == sv


def test_set_velocity_preserves_curvature_and_rejects_nan():
    # within the per-axis limits, but the outer wheels would exceed the motor
    # speed -> v and w are scaled together (curvature preserved)
    v, w = DEFAULT.max_speed, DEFAULT.max_yaw_rate
    sv = cv.twist_to_set_velocity(v, w)
    assert sv.v_mm_s < v * 1000 and math.isclose(sv.w_mrad_s / sv.v_mm_s, w / v, rel_tol=0.01)
    assert cv.twist_to_set_velocity(float("nan"), 1.0) == SetVelocity(0, 0, 500)


def test_odom_covariances_shape_and_growth():
    p0, t0 = cv.odom_covariances(0.0, 0.0)
    p1, t1 = cv.odom_covariances(0.4, 0.8)
    for c in (p0, t0, p1, t1):
        assert len(c) == 36
        m = np.array(c).reshape(6, 6)
        assert np.allclose(m, m.T) and np.all(np.diag(m) > 0)
    assert t1[0] > t0[0] and t1[35] > t0[35]       # vx, vyaw variances grow with speed


def test_ranges_to_meters():
    vals = cv.ranges_to_meters([1234, NO_ECHO, 5000, 20, 4000, 0], 0.02, 4.0)
    assert vals[0] == pytest.approx(1.234)
    assert math.isinf(vals[1]) and math.isinf(vals[2])
    assert vals[3] == pytest.approx(0.02) and vals[4] == pytest.approx(4.0) and vals[5] == 0.0
    assert cv.ranges_to_meters([NO_ECHO], 0.02, 4.0, no_echo_as_max=True) == [4.0]
    assert cv.normalise_range(math.inf, 0.02, 4.0, True) == 4.0
    assert math.isinf(cv.normalise_range(float("nan"), 0.02, 4.0))


def test_wheel_angles():
    a = cv.wheel_angles([840, -420, 0, 0, 0, 0], 840)
    assert a[0] == pytest.approx(2 * math.pi) and a[1] == pytest.approx(-math.pi)


# --------------------------------------------------------------- payload
def test_imu_to_si():
    s = cv.imu_to_si(Imu(t_ms=1, ax=100, ay=-200, az=9810, gx=10, gy=-20, gz=1000,
                         roll_cdeg=4500, pitch_cdeg=-1000))
    assert s.roll == pytest.approx(math.pi / 4) and s.pitch == pytest.approx(math.radians(-10))
    assert (s.ax, s.ay, s.az) == pytest.approx((0.1, -0.2, 9.81))
    assert s.gz == pytest.approx(1.0) and s.gy == pytest.approx(-0.02)


def test_env_to_si_status_bits_and_fallback():
    e = EnvTlm(dht_temp_cC=2512, dht_rh_cP=4550, bmp_pressure_Pa=101325, bmp_temp_cC=2490,
               mq2_raw=300, mq4_raw=200, soil_raw=512, rain_raw=1023, proximity=1, status=0x0F)
    s = cv.env_to_si(e)
    assert s.temperature_c == pytest.approx(25.12) and s.humidity_pct == pytest.approx(45.5)
    assert s.pressure_pa == 101325 and s.metal_detected and s.dht_ok and s.gas_warm
    s2 = cv.env_to_si(EnvTlm(dht_temp_cC=0, bmp_temp_cC=2100, bmp_pressure_Pa=99000, status=0x02))
    assert s2.temperature_c == pytest.approx(21.0)          # BMP fallback
    assert math.isnan(s2.humidity_pct) and not s2.dht_ok and s2.bmp_ok
    s3 = cv.env_to_si(EnvTlm(status=0))
    assert math.isnan(s3.temperature_c) and math.isnan(s3.pressure_pa)


# --------------------------------------------------------------- arm
def test_clamp_arm_deg():
    assert cv.clamp_arm_deg([-5, 0, 200, 90.4, 90.6, 100]) == [0, 15, 180, 90, 91, 80]
    assert cv.clamp_arm_deg([float("nan")] * 6) == list(cv.ARM_HOME_DEG)
    with pytest.raises(ValueError):
        cv.clamp_arm_deg([90] * 5)


def test_arm_command_payload():
    assert cv.arm_command_payload(True, 3, [0] * 6, 0) == (PT.ARM_SET_POSE, b"\x03")
    t, data = cv.arm_command_payload(False, 0, [10, 20, 30, 40, 50, 60], 1500)
    assert t == PT.ARM_SET_JOINTS and len(data) == 8
    j = ArmSetJoints.unpack(data)
    assert j.deg == [10, 20, 30, 40, 50, 60] and j.duration_ms == 1500
    with pytest.raises(ValueError):
        cv.arm_command_payload(True, 42, [0] * 6, 0)
    rad = cv.arm_state_to_radians(ArmState([0, 90, 180, 45, 90, 10], 0))
    assert rad[1] == pytest.approx(math.pi / 2) and rad[2] == pytest.approx(math.pi)


# --------------------------------------------------------------- ground station
def test_drive_to_twist():
    assert cv.drive_to_twist(Drive(50, -25, 1000), 0.4, 0.8) == pytest.approx((0.2, -0.2, 1.0))
    assert cv.drive_to_twist(Drive(127, -128, 0), 0.4, 0.8) == pytest.approx((0.4, -0.8, 0.0))


def test_build_tlm_v2_layout():
    s = cv.TelemetrySnapshot(mode=2, fault_flags=0x05, battery_v=12.34, uptime_s=3600.7,
                             temperature_c=25.5, humidity_pct=40.25, pressure_pa=101325.4,
                             roll=math.radians(1.5), pitch=math.radians(-2.0),
                             ax=0.1, ay=-0.2, az=9.81, gx=0.01, gy=0.02, gz=-0.5,
                             mq2_raw=300, mq4_raw=200, soil_raw=500, rain_raw=1000,
                             metal_detected=True, x=1.2344, y=-2.5, heading=math.radians(190),
                             ranges=[math.inf, 1.5, 0.75, math.inf, 3.0, math.inf])
    raw = cv.build_tlm_v2(s).pack()
    assert len(raw) == 56 and raw[0] == 2
    t = decode_tlm(raw)
    assert isinstance(t, TlmV2)
    assert (t.mode, t.fault_flags, t.battery_mV, t.uptime_s) == (2, 5, 12340, 3600)
    assert (t.temperature_cC, t.humidity_cP, t.pressure_Pa) == (2550, 4025, 101325)
    assert (t.roll_cdeg, t.pitch_cdeg) == (150, -200)
    assert (t.ax, t.ay, t.az, t.gz) == (100, -200, 9810, -500)
    assert t.proximity == 1 and (t.x_mm, t.y_mm) == (1234, -2500)
    assert t.heading_cdeg == t.yaw_cdeg == -17000            # 190 deg wrapped to -170 deg
    assert t.range_min_mm == 750


def test_build_tlm_v2_unavailable_fields():
    t = TlmV2.unpack(cv.build_tlm_v2(cv.TelemetrySnapshot()).pack())
    assert t.battery_mV == 0 and t.temperature_cC == 0 and t.pressure_Pa == 0
    assert t.range_min_mm == NO_ECHO
    t2 = TlmV2.unpack(cv.build_tlm_v2(cv.TelemetrySnapshot(x=1e9, temperature_c=1e6)).pack())
    assert t2.x_mm == 2 ** 31 - 1 and t2.temperature_cC == 32767     # clamped, no struct error


# --------------------------------------------------------------- images
def _rgb_image(w=8, h=4):
    img = np.zeros((h, w, 3), np.uint8)
    img[..., 0] = 200   # R
    img[..., 1] = 100   # G
    img[..., 2] = 50    # B
    return img


def test_gray_from_raw_encodings():
    rgb = _rgb_image()
    expect = int(0.299 * 200 + 0.587 * 100 + 0.114 * 50 + 0.5)
    g = cv.gray_from_raw(rgb.tobytes(), 8, 4, "rgb8")
    assert g.shape == (4, 8) and int(g[0, 0]) == expect
    g2 = cv.gray_from_raw(rgb[..., ::-1].copy().tobytes(), 8, 4, "bgr8")
    assert np.array_equal(g, g2)
    mono = np.arange(32, dtype=np.uint8).reshape(4, 8)
    padded = np.zeros((4, 12), np.uint8)
    padded[:, :8] = mono                                   # step > width
    assert np.array_equal(cv.gray_from_raw(padded.tobytes(), 8, 4, "mono8", step=12), mono)
    yuyv = np.zeros((4, 8, 2), np.uint8)
    yuyv[..., 0] = 77
    assert np.all(cv.gray_from_raw(yuyv.tobytes(), 8, 4, "yuyv") == 77)
    with pytest.raises(ValueError):
        cv.gray_from_raw(b"\x00" * 10, 2, 2, "bayer_rggb8")


def test_resize_and_placeholder():
    img = np.arange(200 * 300, dtype=np.uint32).reshape(200, 300).astype(np.uint8)
    assert cv.resize_nearest(img, 100, 100).shape == (100, 100)
    p = cv.placeholder_image(100, 100)
    assert p.shape == (100, 100) and p.dtype == np.uint8 and p[50, 50] == 0


def test_legacy_image_chunks():
    img = (np.arange(10000) % 256).astype(np.uint8).reshape(100, 100)
    chunks = cv.legacy_image_chunks(img)
    assert len(chunks) == 50 and all(len(c) == 200 for c in chunks)
    assert b"".join(chunks) == img.tobytes()
    assert len(cv.legacy_image_chunks(np.zeros((240, 320), np.uint8))) == 50


def test_v2_image_chunks_round_trip():
    payload = bytes(range(256)) * 3 + b"tail"               # 772 B -> 4 chunks
    chunks = cv.v2_image_chunks(payload, 160, 120, 1)
    assert len(chunks) == 4 and all(len(c) <= 209 for c in chunks)
    hdr, data = cv.reassemble_v2_chunks(list(reversed(chunks)))
    assert data == payload and (hdr.width, hdr.height, hdr.fmt, hdr.total) == (160, 120, 1, 4)
    with pytest.raises(ValueError):
        cv.reassemble_v2_chunks(chunks[:2])
