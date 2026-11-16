/*
 * drive.cpp - see drive.h.
 *
 * Speed measurement: at 0.45 m/s a wheel gives ~12 counts per 20 ms period,
 * i.e. a 37 mm/s quantisation step. Averaging the three wheels of a side and
 * a light IIR filter (beta 0.35, ~37 ms) bring this to a few mm/s without
 * adding much lag compared with the mechanical time constant of a 36 kg
 * rover. A wheel flagged with an encoder fault is excluded from the mean;
 * if all three encoders of a side are faulty that side runs open loop
 * (feed-forward only) instead of winding the integrator up.
 */
#include <Arduino.h>

#include "battery.h"
#include "config.h"
#include "drive.h"
#include "encoders.h"
#include "motors.h"

namespace {

PiParams s_pi_params;
PiState s_pi[2];
float s_target[2]; /* side speed targets after mixing/limits   */
float s_sp[2];     /* slew-limited setpoints                   */
float s_meas[2];   /* filtered measured side speeds            */
float s_cmd_v = 0.0f, s_cmd_w = 0.0f;
drive::Source s_src = drive::SRC_NONE;
uint32_t s_cmd_ms = 0;
uint16_t s_cmd_window_ms = 0;
bool s_fast_stop = false;
bool s_timeout_fault = false;
bool s_enabled = true;

int32_t s_prev_cnt[NUM_WHEELS];
uint32_t s_last_tick_ms[NUM_WHEELS];
uint8_t s_enc_fault = 0;
uint32_t s_last_ctrl_ms = 0;
uint32_t s_last_ctrl_us = 0;
Pose2D s_pose = {0.0f, 0.0f, 0.0f};

void setTwist(float v, float w) {
  s_cmd_v = clampf(v, -MAX_SPEED, MAX_SPEED);
  s_cmd_w = clampf(w, -MAX_YAW_RATE, MAX_YAW_RATE);
}

void resetControl() {
  for (uint8_t s = 0; s < 2; s++) {
    pi_reset(&s_pi[s]);
    s_target[s] = 0.0f;
    s_sp[s] = 0.0f;
  }
  s_cmd_v = s_cmd_w = 0.0f;
}

void controlStep(uint32_t now_ms) {
  const uint32_t now_us = micros();
  float dt = (float)(now_us - s_last_ctrl_us) * 1e-6f;
  s_last_ctrl_us = now_us;
  if (dt <= 0.0f || dt > 0.2f) dt = CONTROL_PERIOD_MS * 1e-3f; /* first call / stall */

  /* ---- encoders: per-wheel deltas, encoder-fault detection */
  int32_t cnt[NUM_WHEELS];
  encoders::read(cnt);
  int32_t delta[NUM_WHEELS];
  for (uint8_t i = 0; i < NUM_WHEELS; i++) {
    delta[i] = cnt[i] - s_prev_cnt[i];
    s_prev_cnt[i] = cnt[i];
    const bool commanded = fabs(motors::duty(i)) >= ENC_FAULT_DUTY;
    if (delta[i] != 0 || !commanded) {
      s_last_tick_ms[i] = now_ms; /* ticking, or not asked to move: restart window */
      if (delta[i] != 0) s_enc_fault &= (uint8_t)~(1u << i);
    } else if ((uint32_t)(now_ms - s_last_tick_ms[i]) > ENC_FAULT_MS) {
      s_enc_fault |= (uint8_t)(1u << i);
    }
  }

  /* ---- side speeds from healthy encoders; odometry */
  bool closed[2];
  float dist[2];
  for (uint8_t s = 0; s < 2; s++) {
    int32_t sum = 0;
    uint8_t n = 0;
    for (uint8_t k = 0; k < 3; k++) {
      const uint8_t i = (uint8_t)(3 * s + k);
      if (!(s_enc_fault & (1u << i))) {
        sum += delta[i];
        n++;
      }
    }
    closed[s] = n > 0;
    dist[s] = n ? (float)sum / (float)n * M_PER_COUNT : 0.0f;
    const float v_raw = dist[s] / dt;
    s_meas[s] += SPEED_FILTER_BETA * (v_raw - s_meas[s]);
  }
  odom_integrate(&s_pose, dist[0], dist[1], B_EFF_M);

  /* ---- command watchdog (SET_VELOCITY) and DRIVE duration */
  if ((s_src == drive::SRC_VELOCITY || s_src == drive::SRC_DRIVE) &&
      (uint32_t)(now_ms - s_cmd_ms) > s_cmd_window_ms) {
    if (s_src == drive::SRC_VELOCITY) s_timeout_fault = true; /* link lost */
    s_src = drive::SRC_NONE;
    s_cmd_v = s_cmd_w = 0.0f;
    s_fast_stop = true;
  }

  /* ---- mixing, limits, slew */
  skid_steer(s_cmd_v, s_cmd_w, B_EFF_M, &s_target[0], &s_target[1]);
  limit_sides(&s_target[0], &s_target[1], MAX_SIDE_SPEED);
  const float step = (s_fast_stop ? DECEL_STOP : ACCEL_MAX) * dt;
  float rate[2];
  for (uint8_t s = 0; s < 2; s++) {
    const float prev = s_sp[s];
    s_sp[s] = slew_toward(s_sp[s], s_target[s], step);
    rate[s] = (s_sp[s] - prev) / dt;
  }

  /* ---- battery compensation of the feed-forward */
  const uint16_t mv = battery::millivolts();
  const float batt_scale = mv > 6000 ? clampf(NOMINAL_BATT_MV / (float)mv, 0.8f, 1.3f) : 1.0f;

  /* ---- PI per side, outputs */
  const bool open_loop_cmd = (s_src == drive::SRC_DRIVE);
  for (uint8_t s = 0; s < 2; s++) {
    const float u = pi_step(&s_pi[s], s_pi_params, s_sp[s], rate[s], s_meas[s], dt, batt_scale,
                            closed[s] && !open_loop_cmd);
    for (uint8_t k = 0; k < 3; k++) motors::set((uint8_t)(3 * s + k), s_enabled ? u : 0.0f);
  }
}

} // namespace

namespace drive {

void begin() {
  s_pi_params.kp = KP_DUTY_PER_MPS;
  s_pi_params.ki = KI_DUTY_PER_M;
  s_pi_params.i_max = I_MAX_DUTY;
  s_pi_params.u_max = DUTY_MAX;
  s_pi_params.kff = KFF_DUTY_PER_MPS;
  s_pi_params.ka = KA_DUTY_PER_MPS2;
  s_pi_params.u_static = FF_STATIC_DUTY;
  s_pi_params.v_dead = V_DEADBAND;
  resetControl();
  s_meas[0] = s_meas[1] = 0.0f;
  encoders::read(s_prev_cnt);
  const uint32_t now = millis();
  for (uint8_t i = 0; i < NUM_WHEELS; i++) s_last_tick_ms[i] = now;
  s_last_ctrl_ms = now;
  s_last_ctrl_us = micros();
}

void setVelocity(float v, float w, uint16_t timeout_ms, uint32_t now_ms) {
  if (!s_enabled) return;
  setTwist(v, w);
  s_src = SRC_VELOCITY;
  s_cmd_ms = now_ms;
  s_cmd_window_ms = timeout_ms ? timeout_ms : CMD_TIMEOUT_DEFAULT_MS;
  s_fast_stop = false;
  s_timeout_fault = false;
}

void driveOpenLoop(float v, float w, uint16_t duration_ms, uint32_t now_ms) {
  if (!s_enabled) return;
  setTwist(v, w);
  s_src = SRC_DRIVE;
  s_cmd_ms = now_ms;
  if (duration_ms == 0) duration_ms = DRIVE_DEFAULT_MS;
  s_cmd_window_ms = duration_ms > DRIVE_MAX_MS ? DRIVE_MAX_MS : duration_ms;
  s_fast_stop = false;
  s_timeout_fault = false;
}

void setScanTwist(float v, float w) {
  if (!s_enabled) return;
  setTwist(v, w);
  s_src = SRC_SCAN;
  s_fast_stop = false;
}

void stop() {
  s_src = SRC_NONE;
  s_cmd_v = s_cmd_w = 0.0f;
  s_fast_stop = true;
}

void setEnabled(bool enabled) {
  if (!enabled) {
    motors::stopAll(); /* immediately, not at the next control tick */
    resetControl();
    s_src = SRC_NONE;
  }
  s_enabled = enabled;
}

void update(uint32_t now_ms) {
  if ((uint32_t)(now_ms - s_last_ctrl_ms) < CONTROL_PERIOD_MS) return;
  /* keep a fixed 20 ms grid; after a stall re-align instead of catching up */
  s_last_ctrl_ms += CONTROL_PERIOD_MS;
  if ((uint32_t)(now_ms - s_last_ctrl_ms) >= CONTROL_PERIOD_MS) s_last_ctrl_ms = now_ms;
  controlStep(now_ms);
}

Source source() { return s_src; }
bool timeoutFault() { return s_timeout_fault; }
void clearTimeoutFault() { s_timeout_fault = false; }
bool encoderFault() { return s_enc_fault != 0; }
uint8_t encoderFaultMask() { return s_enc_fault; }
bool moving() {
  return fabs(s_sp[0]) > 0.0f || fabs(s_sp[1]) > 0.0f || fabs(s_meas[0]) > 0.02f ||
         fabs(s_meas[1]) > 0.02f;
}
const Pose2D &pose() { return s_pose; }
float sideSpeed(uint8_t side) { return side < 2 ? s_meas[side] : 0.0f; }
float sideSetpoint(uint8_t side) { return side < 2 ? s_sp[side] : 0.0f; }

} // namespace drive
