/*
 * ssp_protocol.cpp - packet-type table and payload (un)packing (ICD 6, 7).
 */
#include "ssp_protocol.h"

/* ------------------------------------------------ length plausibility */
/* Encodes rover_core.protocol._ALLOWED_LEN. Returns:
 *   -1  unknown type, 0 = LEN not allowed, 1 = LEN allowed. */
static int8_t allowedLen(uint8_t type, uint8_t len) {
  switch (type) {
  case SSP_PING: return len == 0;
  case SSP_ACK: return len <= 1; /* {0, 1} */
  case SSP_NACK: return len == 2;
  case SSP_GET_IMAGE: return len == 0 || len == 1 || len == 200;
  case SSP_IMAGE_CHUNK: return len >= 9 && len <= 209; /* range(9, 210) */
  case SSP_CAP_IMAGE: return len == 0;
  case SSP_GET_TLM: return 1; /* None: legacy brains sent 7/17/68 B */
  case SSP_START_SCAN: return len == 0;
  case SSP_STOP_SCAN: return len == 0;
  case SSP_COLLECT_OBJECT: return len == 0;
  case SSP_SET_VELOCITY: return len == 6;
  case SSP_WHEEL_ODOM: return len == 28;
  case SSP_RANGES: return (len & 1u) && len <= 13; /* 1 + 2n, n = 0..6 */
  case SSP_ENV_TLM: return len == 20;
  case SSP_IMU: return len == 20;
  case SSP_ARM_SET_JOINTS: return len == 8;
  case SSP_ARM_SET_POSE: return len == 1;
  case SSP_ARM_STATE: return len == 7;
  case SSP_HEARTBEAT: return len == 6;
  case SSP_ESTOP: return len == 0;
  case SSP_CLEAR_ESTOP: return len == 0;
  case SSP_SET_MODE: return len == 1;
  case SSP_DRIVE: return len == 4;
  default: return -1;
  }
}

bool ssp_plausible_length(uint8_t type, uint8_t len) { return allowedLen(type, len) != 0; }
bool ssp_plausible_length_strict(uint8_t type, uint8_t len) { return allowedLen(type, len) == 1; }
bool ssp_known_type(uint8_t type) { return allowedLen(type, 0) >= 0; }

bool ssp_is_report_type(uint8_t type) {
  switch (type) {
  case SSP_ACK:
  case SSP_NACK:
  case SSP_IMAGE_CHUNK:
  case SSP_WHEEL_ODOM:
  case SSP_RANGES:
  case SSP_ENV_TLM:
  case SSP_IMU:
  case SSP_ARM_STATE:
  case SSP_HEARTBEAT: return true;
  default: return false;
  }
}

/* ------------------------------------------------------------ payloads */
uint8_t ssp_pack_set_velocity(uint8_t *out, const SspSetVelocity &v) {
  SspWriter w(out, SSP_LEN_SET_VELOCITY);
  w.i16(v.v_mm_s).i16(v.w_mrad_s).u16(v.timeout_ms);
  return (uint8_t)w.len();
}
bool ssp_unpack_set_velocity(const uint8_t *d, uint8_t len, SspSetVelocity *v) {
  if (len != SSP_LEN_SET_VELOCITY) return false;
  SspReader r(d, len);
  v->v_mm_s = r.i16();
  v->w_mrad_s = r.i16();
  v->timeout_ms = r.u16();
  return r.atEnd();
}

uint8_t ssp_pack_wheel_odom(uint8_t *out, const SspWheelOdom &v) {
  SspWriter w(out, SSP_LEN_WHEEL_ODOM);
  w.u32(v.t_ms);
  for (uint8_t i = 0; i < SSP_WHEELS; i++) w.i32(v.ticks[i]);
  return (uint8_t)w.len();
}
bool ssp_unpack_wheel_odom(const uint8_t *d, uint8_t len, SspWheelOdom *v) {
  if (len != SSP_LEN_WHEEL_ODOM) return false;
  SspReader r(d, len);
  v->t_ms = r.u32();
  for (uint8_t i = 0; i < SSP_WHEELS; i++) v->ticks[i] = r.i32();
  return r.atEnd();
}

uint8_t ssp_pack_ranges(uint8_t *out, const SspRanges &v) {
  const uint8_t n = v.n > SSP_ULTRASONICS ? (uint8_t)SSP_ULTRASONICS : v.n;
  SspWriter w(out, (uint16_t)SSP_LEN_RANGES(n));
  w.u8(n);
  for (uint8_t i = 0; i < n; i++) w.u16(v.range_mm[i]);
  return (uint8_t)w.len();
}
bool ssp_unpack_ranges(const uint8_t *d, uint8_t len, SspRanges *v) {
  if (len < 1) return false;
  const uint8_t n = d[0];
  if (n > SSP_ULTRASONICS || len != SSP_LEN_RANGES(n)) return false;
  SspReader r(d, len);
  v->n = r.u8();
  for (uint8_t i = 0; i < n; i++) v->range_mm[i] = r.u16();
  for (uint8_t i = n; i < SSP_ULTRASONICS; i++) v->range_mm[i] = SSP_NO_ECHO;
  return r.atEnd();
}

uint8_t ssp_pack_env_tlm(uint8_t *out, const SspEnvTlm &v) {
  SspWriter w(out, SSP_LEN_ENV_TLM);
  w.i16(v.dht_temp_cC).u16(v.dht_rh_cP).u32(v.bmp_pressure_Pa).i16(v.bmp_temp_cC);
  w.u16(v.mq2_raw).u16(v.mq4_raw).u16(v.soil_raw).u16(v.rain_raw);
  w.u8(v.proximity).u8(v.status);
  return (uint8_t)w.len();
}
bool ssp_unpack_env_tlm(const uint8_t *d, uint8_t len, SspEnvTlm *v) {
  if (len != SSP_LEN_ENV_TLM) return false;
  SspReader r(d, len);
  v->dht_temp_cC = r.i16();
  v->dht_rh_cP = r.u16();
  v->bmp_pressure_Pa = r.u32();
  v->bmp_temp_cC = r.i16();
  v->mq2_raw = r.u16();
  v->mq4_raw = r.u16();
  v->soil_raw = r.u16();
  v->rain_raw = r.u16();
  v->proximity = r.u8();
  v->status = r.u8();
  return r.atEnd();
}

uint8_t ssp_pack_imu(uint8_t *out, const SspImu &v) {
  SspWriter w(out, SSP_LEN_IMU);
  w.u32(v.t_ms).i16(v.ax).i16(v.ay).i16(v.az).i16(v.gx).i16(v.gy).i16(v.gz);
  w.i16(v.roll_cdeg).i16(v.pitch_cdeg);
  return (uint8_t)w.len();
}
bool ssp_unpack_imu(const uint8_t *d, uint8_t len, SspImu *v) {
  if (len != SSP_LEN_IMU) return false;
  SspReader r(d, len);
  v->t_ms = r.u32();
  v->ax = r.i16();
  v->ay = r.i16();
  v->az = r.i16();
  v->gx = r.i16();
  v->gy = r.i16();
  v->gz = r.i16();
  v->roll_cdeg = r.i16();
  v->pitch_cdeg = r.i16();
  return r.atEnd();
}

uint8_t ssp_pack_arm_set_joints(uint8_t *out, const SspArmSetJoints &v) {
  SspWriter w(out, SSP_LEN_ARM_SET_JOINTS);
  for (uint8_t i = 0; i < SSP_ARM_JOINTS; i++) w.u8(v.deg[i]);
  w.u16(v.duration_ms);
  return (uint8_t)w.len();
}
bool ssp_unpack_arm_set_joints(const uint8_t *d, uint8_t len, SspArmSetJoints *v) {
  if (len != SSP_LEN_ARM_SET_JOINTS) return false;
  SspReader r(d, len);
  for (uint8_t i = 0; i < SSP_ARM_JOINTS; i++) v->deg[i] = r.u8();
  v->duration_ms = r.u16();
  return r.atEnd();
}

uint8_t ssp_pack_arm_state(uint8_t *out, const SspArmState &v) {
  SspWriter w(out, SSP_LEN_ARM_STATE);
  for (uint8_t i = 0; i < SSP_ARM_JOINTS; i++) w.u8(v.deg[i]);
  w.u8(v.moving);
  return (uint8_t)w.len();
}
bool ssp_unpack_arm_state(const uint8_t *d, uint8_t len, SspArmState *v) {
  if (len != SSP_LEN_ARM_STATE) return false;
  SspReader r(d, len);
  for (uint8_t i = 0; i < SSP_ARM_JOINTS; i++) v->deg[i] = r.u8();
  v->moving = r.u8();
  return r.atEnd();
}

uint8_t ssp_pack_heartbeat(uint8_t *out, const SspHeartbeat &v) {
  SspWriter w(out, SSP_LEN_HEARTBEAT);
  w.u32(v.uptime_ms).u8(v.mode).u8(v.fault_flags);
  return (uint8_t)w.len();
}
bool ssp_unpack_heartbeat(const uint8_t *d, uint8_t len, SspHeartbeat *v) {
  if (len != SSP_LEN_HEARTBEAT) return false;
  SspReader r(d, len);
  v->uptime_ms = r.u32();
  v->mode = r.u8();
  v->fault_flags = r.u8();
  return r.atEnd();
}

uint8_t ssp_pack_drive(uint8_t *out, const SspDrive &v) {
  SspWriter w(out, SSP_LEN_DRIVE);
  w.i8(v.v_pct).i8(v.w_pct).u16(v.duration_ms);
  return (uint8_t)w.len();
}
bool ssp_unpack_drive(const uint8_t *d, uint8_t len, SspDrive *v) {
  if (len != SSP_LEN_DRIVE) return false;
  SspReader r(d, len);
  v->v_pct = r.i8();
  v->w_pct = r.i8();
  v->duration_ms = r.u16();
  return r.atEnd();
}

uint8_t ssp_pack_nack(uint8_t *out, const SspNack &v) {
  out[0] = v.rejected_type;
  out[1] = v.reason;
  return SSP_LEN_NACK;
}
bool ssp_unpack_nack(const uint8_t *d, uint8_t len, SspNack *v) {
  if (len != SSP_LEN_NACK) return false;
  v->rejected_type = d[0];
  v->reason = d[1];
  return true;
}

uint8_t ssp_pack_tlm_legacy(uint8_t *out, const SspTlmLegacy &v) {
  out[0] = v.ir;
  out[1] = v.gas_hi;
  out[2] = v.gas_lo;
  out[3] = v.us1_cm;
  out[4] = v.us2_cm;
  out[5] = v.temp_c;
  out[6] = v.rh_pct;
  return SSP_LEN_TLM_LEGACY;
}
bool ssp_unpack_tlm_legacy(const uint8_t *d, uint8_t len, SspTlmLegacy *v) {
  if (len != SSP_LEN_TLM_LEGACY) return false;
  v->ir = d[0];
  v->gas_hi = d[1];
  v->gas_lo = d[2];
  v->us1_cm = d[3];
  v->us2_cm = d[4];
  v->temp_c = d[5];
  v->rh_pct = d[6];
  return true;
}

uint8_t ssp_pack_tlm_v2(uint8_t *out, const SspTlmV2 &v) {
  SspWriter w(out, SSP_LEN_TLM_V2);
  w.u8(v.tlm_version).u8(v.mode).u8(v.fault_flags).u16(v.battery_mV).u32(v.uptime_s);
  w.i16(v.temperature_cC).u16(v.humidity_cP).u32(v.pressure_Pa);
  w.i16(v.roll_cdeg).i16(v.pitch_cdeg).i16(v.yaw_cdeg);
  w.i16(v.ax).i16(v.ay).i16(v.az).i16(v.gx).i16(v.gy).i16(v.gz);
  w.u16(v.mq2_raw).u16(v.mq4_raw).u16(v.soil_raw).u16(v.rain_raw).u8(v.proximity);
  w.i32(v.x_mm).i32(v.y_mm).i16(v.heading_cdeg).u16(v.range_min_mm);
  return (uint8_t)w.len();
}
bool ssp_unpack_tlm_v2(const uint8_t *d, uint8_t len, SspTlmV2 *v) {
  if (len != SSP_LEN_TLM_V2) return false;
  SspReader r(d, len);
  v->tlm_version = r.u8();
  v->mode = r.u8();
  v->fault_flags = r.u8();
  v->battery_mV = r.u16();
  v->uptime_s = r.u32();
  v->temperature_cC = r.i16();
  v->humidity_cP = r.u16();
  v->pressure_Pa = r.u32();
  v->roll_cdeg = r.i16();
  v->pitch_cdeg = r.i16();
  v->yaw_cdeg = r.i16();
  v->ax = r.i16();
  v->ay = r.i16();
  v->az = r.i16();
  v->gx = r.i16();
  v->gy = r.i16();
  v->gz = r.i16();
  v->mq2_raw = r.u16();
  v->mq4_raw = r.u16();
  v->soil_raw = r.u16();
  v->rain_raw = r.u16();
  v->proximity = r.u8();
  v->x_mm = r.i32();
  v->y_mm = r.i32();
  v->heading_cdeg = r.i16();
  v->range_min_mm = r.u16();
  return r.atEnd();
}
