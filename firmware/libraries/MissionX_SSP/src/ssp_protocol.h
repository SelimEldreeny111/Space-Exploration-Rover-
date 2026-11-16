/*
 * ssp_protocol.h - MISSION X SSP v2.0: addresses, packet types, payload
 * layouts (ICD v2.0 sections 4, 6, 7). Mirrors rover_core/protocol.py.
 * Platform independent (no Arduino.h). All multi-byte fields little-endian.
 */
#ifndef MISSIONX_SSP_PROTOCOL_H
#define MISSIONX_SSP_PROTOCOL_H

#include "ssp_core.h"

/* ------------------------------------------------------ addresses (ICD 4) */
enum SspAddr {
  SSP_ADDR_ROVER = 0x01,    /* Brain (and legacy single-MCU rover) */
  SSP_ADDR_MOBILITY = 0x02, /* Arduino Mega 2560                   */
  SSP_ADDR_PAYLOAD = 0x03,  /* Arduino Uno, sensors + IMU           */
  SSP_ADDR_ARM = 0x04,      /* Arduino Uno + PCA9685                */
  SSP_ADDR_GROUND = 0x0B,   /* ground station                       */
  SSP_ADDR_BROADCAST = 0xFF
};

/* ------------------------------------------------- packet types (ICD 6) */
enum SspType {
  SSP_PING = 0x00,
  SSP_ACK = 0x01,
  SSP_NACK = 0x02,
  SSP_GET_IMAGE = 0x08,
  SSP_IMAGE_CHUNK = 0x09,
  SSP_CAP_IMAGE = 0x0C,
  SSP_GET_TLM = 0x0F,
  SSP_START_SCAN = 0x16,
  SSP_STOP_SCAN = 0x1C,
  SSP_COLLECT_OBJECT = 0x20,
  SSP_SET_VELOCITY = 0x30,
  SSP_WHEEL_ODOM = 0x31,
  SSP_RANGES = 0x32,
  SSP_ENV_TLM = 0x33,
  SSP_IMU = 0x34,
  SSP_ARM_SET_JOINTS = 0x40,
  SSP_ARM_SET_POSE = 0x41,
  SSP_ARM_STATE = 0x42,
  SSP_HEARTBEAT = 0x50,
  SSP_ESTOP = 0x51,
  SSP_CLEAR_ESTOP = 0x52,
  SSP_SET_MODE = 0x53,
  SSP_DRIVE = 0x60
};

enum SspMode { SSP_MODE_IDLE = 0, SSP_MODE_TELEOP = 1, SSP_MODE_AUTONOMOUS = 2, SSP_MODE_SAFE = 3 };

enum SspNackReason {
  SSP_NACK_BAD_LENGTH = 1,
  SSP_NACK_UNKNOWN_TYPE = 2,
  SSP_NACK_BUSY = 3,
  SSP_NACK_NOT_PERMITTED = 4, /* also: e-stop latched */
  SSP_NACK_HW_FAULT = 5
};

/* fault_flags (Mobility, ICD 6.2) */
enum SspFault {
  SSP_FAULT_ESTOP = 1u << 0,
  SSP_FAULT_CMD_TIMEOUT = 1u << 1,
  SSP_FAULT_LOW_BATTERY = 1u << 2,
  SSP_FAULT_ENCODER = 1u << 3,
  SSP_FAULT_SAFE_MODE = 1u << 4
};

enum SspArmPose {
  SSP_POSE_HOME = 0,
  SSP_POSE_STOW = 1,
  SSP_POSE_READY = 2,
  SSP_POSE_REACH_GROUND = 3,
  SSP_POSE_GRIP_OPEN = 4,
  SSP_POSE_GRIP_CLOSE = 5,
  SSP_POSE_LIFT_SAMPLE = 6,
  SSP_POSE_DEPOSIT = 7,
  SSP_POSE_COUNT = 8
};

/* ENV_TLM status bits */
enum SspEnvStatus {
  SSP_ENV_DHT_OK = 1u << 0,
  SSP_ENV_BMP_OK = 1u << 1,
  SSP_ENV_MPU_OK = 1u << 2,
  SSP_ENV_GAS_WARM = 1u << 3
};

#define SSP_NO_ECHO 0xFFFFu
#define SSP_WHEELS 6
#define SSP_ULTRASONICS 6
#define SSP_ARM_JOINTS 6

/* ------------------------------------------------------- payload lengths */
#define SSP_LEN_NACK 2u
#define SSP_LEN_SET_VELOCITY 6u
#define SSP_LEN_WHEEL_ODOM 28u
#define SSP_LEN_RANGES(n) (1u + 2u * (n)) /* 13 for n = 6 */
#define SSP_LEN_ENV_TLM 20u
#define SSP_LEN_IMU 20u
#define SSP_LEN_ARM_SET_JOINTS 8u
#define SSP_LEN_ARM_SET_POSE 1u
#define SSP_LEN_ARM_STATE 7u
#define SSP_LEN_HEARTBEAT 6u
#define SSP_LEN_SET_MODE 1u
#define SSP_LEN_DRIVE 4u
#define SSP_LEN_TLM_LEGACY 7u
#define SSP_LEN_TLM_V2 56u

/*
 * Length plausibility (ICD 3 step 5a) = rover_core.protocol.plausible_length
 * with the same _ALLOWED_LEN table (union of both directions, legacy
 * included). Unknown types are accepted with any length (non-strict) so that
 * the node can still answer them with NACK(UNKNOWN_TYPE); the strict
 * variant also rejects unknown types.
 */
bool ssp_plausible_length(uint8_t type, uint8_t len);
bool ssp_plausible_length_strict(uint8_t type, uint8_t len);
bool ssp_known_type(uint8_t type);
/* Answers and periodic reports (ACK, NACK, IMAGE_CHUNK, WHEEL_ODOM, RANGES,
 * ENV_TLM, IMU, ARM_STATE, HEARTBEAT). A node never replies to these, not
 * even with a NACK, so two nodes can never ping-pong error frames. */
bool ssp_is_report_type(uint8_t type);

/* ------------------------------------------------------ payload structs */
struct SspSetVelocity { int16_t v_mm_s; int16_t w_mrad_s; uint16_t timeout_ms; };
struct SspWheelOdom { uint32_t t_ms; int32_t ticks[SSP_WHEELS]; };
struct SspRanges { uint8_t n; uint16_t range_mm[SSP_ULTRASONICS]; };
struct SspEnvTlm {
  int16_t dht_temp_cC;
  uint16_t dht_rh_cP;
  uint32_t bmp_pressure_Pa;
  int16_t bmp_temp_cC;
  uint16_t mq2_raw, mq4_raw, soil_raw, rain_raw;
  uint8_t proximity, status;
};
struct SspImu {
  uint32_t t_ms;
  int16_t ax, ay, az; /* mm/s^2 */
  int16_t gx, gy, gz; /* mrad/s */
  int16_t roll_cdeg, pitch_cdeg;
};
struct SspArmSetJoints { uint8_t deg[SSP_ARM_JOINTS]; uint16_t duration_ms; };
struct SspArmState { uint8_t deg[SSP_ARM_JOINTS]; uint8_t moving; };
struct SspHeartbeat { uint32_t uptime_ms; uint8_t mode; uint8_t fault_flags; };
struct SspDrive { int8_t v_pct; int8_t w_pct; uint16_t duration_ms; };
struct SspNack { uint8_t rejected_type; uint8_t reason; };
struct SspTlmLegacy { uint8_t ir, gas_hi, gas_lo, us1_cm, us2_cm, temp_c, rh_pct; };
struct SspTlmV2 {
  uint8_t tlm_version, mode, fault_flags;
  uint16_t battery_mV;
  uint32_t uptime_s;
  int16_t temperature_cC;
  uint16_t humidity_cP;
  uint32_t pressure_Pa;
  int16_t roll_cdeg, pitch_cdeg, yaw_cdeg;
  int16_t ax, ay, az, gx, gy, gz;
  uint16_t mq2_raw, mq4_raw, soil_raw, rain_raw;
  uint8_t proximity;
  int32_t x_mm, y_mm;
  int16_t heading_cdeg;
  uint16_t range_min_mm;
};

/* pack: writes the payload into out (caller provides >= SSP_LEN_x bytes),
 *       returns the number of bytes written.
 * unpack: returns false if len does not match the layout. */
uint8_t ssp_pack_set_velocity(uint8_t *out, const SspSetVelocity &v);
bool ssp_unpack_set_velocity(const uint8_t *d, uint8_t len, SspSetVelocity *v);
uint8_t ssp_pack_wheel_odom(uint8_t *out, const SspWheelOdom &v);
bool ssp_unpack_wheel_odom(const uint8_t *d, uint8_t len, SspWheelOdom *v);
uint8_t ssp_pack_ranges(uint8_t *out, const SspRanges &v);
bool ssp_unpack_ranges(const uint8_t *d, uint8_t len, SspRanges *v);
uint8_t ssp_pack_env_tlm(uint8_t *out, const SspEnvTlm &v);
bool ssp_unpack_env_tlm(const uint8_t *d, uint8_t len, SspEnvTlm *v);
uint8_t ssp_pack_imu(uint8_t *out, const SspImu &v);
bool ssp_unpack_imu(const uint8_t *d, uint8_t len, SspImu *v);
uint8_t ssp_pack_arm_set_joints(uint8_t *out, const SspArmSetJoints &v);
bool ssp_unpack_arm_set_joints(const uint8_t *d, uint8_t len, SspArmSetJoints *v);
uint8_t ssp_pack_arm_state(uint8_t *out, const SspArmState &v);
bool ssp_unpack_arm_state(const uint8_t *d, uint8_t len, SspArmState *v);
uint8_t ssp_pack_heartbeat(uint8_t *out, const SspHeartbeat &v);
bool ssp_unpack_heartbeat(const uint8_t *d, uint8_t len, SspHeartbeat *v);
uint8_t ssp_pack_drive(uint8_t *out, const SspDrive &v);
bool ssp_unpack_drive(const uint8_t *d, uint8_t len, SspDrive *v);
uint8_t ssp_pack_nack(uint8_t *out, const SspNack &v);
bool ssp_unpack_nack(const uint8_t *d, uint8_t len, SspNack *v);
uint8_t ssp_pack_tlm_legacy(uint8_t *out, const SspTlmLegacy &v);
bool ssp_unpack_tlm_legacy(const uint8_t *d, uint8_t len, SspTlmLegacy *v);
uint8_t ssp_pack_tlm_v2(uint8_t *out, const SspTlmV2 &v);
bool ssp_unpack_tlm_v2(const uint8_t *d, uint8_t len, SspTlmV2 *v);

#endif /* MISSIONX_SSP_PROTOCOL_H */
