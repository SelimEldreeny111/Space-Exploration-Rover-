"""Physical parameters of the MISSION X rover (ICD v2.0 §9).

Values marked *estimated* were derived from photographs / overall dimensions
and should be replaced by measured values when available (they are exposed as
ROS parameters and in the URDF xacro arguments).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Tuple

G_EARTH = 9.81
G_MARS = 3.71
G_MOON = 1.62


@dataclass(frozen=True)
class MotorParams:
    name: str = "SG775125000-60K"
    voltage: float = 12.0
    gear_ratio: float = 60.0
    no_load_rpm: float = 83.0
    no_load_current: float = 0.5
    rated_rpm: float = 63.0
    rated_torque: float = 2.97          # N m at the gearbox output (28.5 kgf cm)
    rated_current: float = 4.0
    stall_current: float = 14.0
    encoder_ppr_motor: int = 7          # pulses per motor revolution per channel

    @property
    def no_load_omega(self) -> float:
        return self.no_load_rpm * 2 * math.pi / 60.0

    @property
    def stall_torque(self) -> float:
        """Linear torque-speed line through the rated and no-load points."""
        return self.rated_torque / (1.0 - self.rated_rpm / self.no_load_rpm)

    def torque_at(self, omega: float, duty: float = 1.0) -> float:
        """Available output torque [N m] at wheel speed *omega* [rad/s] for PWM duty."""
        w0 = self.no_load_omega * duty
        if w0 <= 0:
            return 0.0
        return max(0.0, self.stall_torque * duty * (1.0 - omega / w0))

    @property
    def rated_output_power(self) -> float:
        return self.rated_torque * self.rated_rpm * 2 * math.pi / 60.0

    @property
    def rated_efficiency(self) -> float:
        return self.rated_output_power / (self.voltage * self.rated_current)


@dataclass(frozen=True)
class RoverParams:
    mass: float = 36.5
    length: float = 1.02
    width: float = 0.73
    height: float = 0.79
    wheel_radius: float = 0.100
    wheel_width: float = 0.05            # estimated
    wheelbase: float = 0.82              # estimated (front-rear axle distance)
    track: float = 0.58                  # estimated (left-right wheel-pair centres)
    skid_chi: float = 1.5                # effective track factor (calibrate on site)
    lidar_height: float = 0.62           # estimated
    ultrasonic_height: float = 0.25      # estimated
    counts_per_wheel_rev: int = 840      # 7 PPR x 60 x 2 (A-channel CHANGE)
    max_speed: float = 0.45              # m/s software limit (no-load 0.87 m/s)
    max_yaw_rate: float = 1.0            # rad/s software limit
    motor: MotorParams = field(default_factory=MotorParams)
    #: ultrasonic mounting poses (x, y, yaw_deg) in base_link, ICD order
    ultrasonic_poses: Tuple[Tuple[float, float, float], ...] = (
        (0.50, 0.18, 30.0), (0.52, 0.00, 0.0), (0.50, -0.18, -30.0),
        (0.10, 0.33, 90.0), (0.10, -0.33, -90.0), (-0.50, 0.00, 180.0))
    ultrasonic_fov_deg: float = 15.0
    ultrasonic_range: Tuple[float, float] = (0.02, 4.0)
    #: LDS-01 (HLS-LFCD2) characteristics
    lidar_range: Tuple[float, float] = (0.12, 3.5)
    lidar_beams: int = 360
    lidar_rate_hz: float = 5.0

    @property
    def b_eff(self) -> float:
        return self.track * self.skid_chi

    @property
    def metres_per_count(self) -> float:
        return 2 * math.pi * self.wheel_radius / self.counts_per_wheel_rev

    @property
    def weight_earth(self) -> float:
        return self.mass * G_EARTH

    def wheel_positions(self) -> Dict[str, Tuple[float, float]]:
        hb, ht = self.wheelbase / 2, self.track / 2
        return {"left_front": (hb, ht), "left_mid": (0.0, ht), "left_rear": (-hb, ht),
                "right_front": (hb, -ht), "right_mid": (0.0, -ht), "right_rear": (-hb, -ht)}


DEFAULT = RoverParams()
