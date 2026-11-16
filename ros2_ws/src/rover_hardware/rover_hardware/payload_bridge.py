"""payload_bridge: ROS 2 <-> Arduino Uno payload MCU (SSP address 0x03, ICD v2.0).

* ENV_TLM (1 Hz) -> ``/environment`` (rover_interfaces/EnvironmentTelemetry)
  and the standard ``/environment/temperature`` (sensor_msgs/Temperature),
  ``/environment/humidity`` (RelativeHumidity, fraction 0..1) and
  ``/environment/pressure`` (FluidPressure) - the standard messages are only
  published while the corresponding sensor status bit is set.
* IMU (25 Hz) -> ``/imu/data`` (sensor_msgs/Imu, frame imu_link): orientation
  from the MCU complementary-filter roll/pitch, yaw unknown (large variance),
  angular velocity and linear acceleration.
* HEARTBEAT -> ``/rover/status`` (source = 0x03).
"""
from __future__ import annotations

from sensor_msgs.msg import FluidPressure, Imu, RelativeHumidity, Temperature

from rover_core.protocol import PT, Addr, EnvTlm, Heartbeat, Imu as ImuPayload
from rover_interfaces.msg import EnvironmentTelemetry, RoverStatus

from .conversions import env_to_si, imu_to_si
from .ros_common import RELIABLE_QOS, McuBridgeNode, make_imu, run_node


class PayloadBridge(McuBridgeNode):
    def __init__(self) -> None:
        super().__init__("payload_bridge", "/dev/rover_payload", "payload", Addr.PAYLOAD)
        dp = self.declare_parameter
        self.env_frame = dp("env_frame", "base_link").value
        self.imu_frame = dp("imu_frame", "imu_link").value
        # 1-sigma: roll/pitch 1 deg, gyro 0.02 rad/s, accel 0.1 m/s^2 (MPU-6050, filtered)
        self.rp_var = float(dp("orientation_variance", 3.0e-4).value)
        self.yaw_var = float(dp("yaw_variance", 1.0e3).value)
        self.gyro_var = float(dp("angular_velocity_variance", 4.0e-4).value)
        self.accel_var = float(dp("linear_acceleration_variance", 1.0e-2).value)
        self.temp_var = float(dp("temperature_variance", 4.0).value)        # DHT11 +-2 degC
        self.rh_var = float(dp("humidity_variance", 2.5e-3).value)          # +-5 %RH (fraction^2)
        self.press_var = float(dp("pressure_variance", 1.0e4).value)        # BMP180 +-100 Pa

        self.env_pub = self.create_publisher(EnvironmentTelemetry, "environment", RELIABLE_QOS)
        self.temp_pub = self.create_publisher(Temperature, "environment/temperature", RELIABLE_QOS)
        self.rh_pub = self.create_publisher(RelativeHumidity, "environment/humidity", RELIABLE_QOS)
        self.press_pub = self.create_publisher(FluidPressure, "environment/pressure", RELIABLE_QOS)
        self.imu_pub = self.create_publisher(Imu, "imu/data", RELIABLE_QOS)
        self.status_pub = self.create_publisher(RoverStatus, "rover/status", RELIABLE_QOS)

    def handle_frame(self, fr) -> None:
        if fr.type == PT.ENV_TLM:
            self._on_env(EnvTlm.unpack(fr.data))
        elif fr.type == PT.IMU:
            self._on_imu(ImuPayload.unpack(fr.data))
        elif fr.type == PT.HEARTBEAT:
            hb = Heartbeat.unpack(fr.data)
            msg = RoverStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.source, msg.mode, msg.fault_flags = int(fr.src), hb.mode, hb.fault_flags
            msg.battery_v = float("nan")
            msg.uptime_s = hb.uptime_ms / 1000.0
            self.status_pub.publish(msg)

    def _on_env(self, e: EnvTlm) -> None:
        s = env_to_si(e)
        stamp = self.get_clock().now().to_msg()
        msg = EnvironmentTelemetry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.env_frame
        msg.temperature_c = s.temperature_c
        msg.humidity_pct = s.humidity_pct
        msg.pressure_pa = s.pressure_pa
        msg.mq2_raw, msg.mq4_raw, msg.soil_raw, msg.rain_raw = (
            s.mq2_raw, s.mq4_raw, s.soil_raw, s.rain_raw)
        msg.metal_detected = s.metal_detected
        msg.status = s.status
        self.env_pub.publish(msg)
        if s.dht_ok or s.bmp_ok:
            t = Temperature()
            t.header = msg.header
            t.temperature = float(s.temperature_c)
            t.variance = self.temp_var
            self.temp_pub.publish(t)
        if s.dht_ok:
            h = RelativeHumidity()
            h.header = msg.header
            h.relative_humidity = s.humidity_pct / 100.0
            h.variance = self.rh_var
            self.rh_pub.publish(h)
        if s.bmp_ok:
            p = FluidPressure()
            p.header = msg.header
            p.fluid_pressure = float(s.pressure_pa)
            p.variance = self.press_var
            self.press_pub.publish(p)

    def _on_imu(self, m: ImuPayload) -> None:
        s = imu_to_si(m)
        self.imu_pub.publish(make_imu(self.get_clock().now().to_msg(), self.imu_frame, s.roll,
                                      s.pitch, s.ax, s.ay, s.az, s.gx, s.gy, s.gz, self.rp_var,
                                      self.yaw_var, self.gyro_var, self.accel_var))


def main(args=None) -> None:
    run_node(PayloadBridge, args)


if __name__ == "__main__":
    main()
