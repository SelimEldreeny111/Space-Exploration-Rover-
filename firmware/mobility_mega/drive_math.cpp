/*
 * drive_math.cpp - see drive_math.h (no Arduino dependencies).
 */
#include "drive_math.h"

#include <math.h>

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

void skid_steer(float v, float w, float b_eff, float *v_left, float *v_right) {
  const float half = 0.5f * w * b_eff;
  *v_left = v - half;
  *v_right = v + half;
}

void limit_sides(float *v_left, float *v_right, float vmax) {
  const float m = fmaxf(fabsf(*v_left), fabsf(*v_right));
  if (m > vmax && m > 0.0f) {
    const float k = vmax / m;
    *v_left *= k;
    *v_right *= k;
  }
}

float slew_toward(float current, float target, float max_step) {
  if (target > current + max_step) return current + max_step;
  if (target < current - max_step) return current - max_step;
  return target;
}

void pi_reset(PiState *s) {
  s->integ = 0.0f;
  s->last_u = 0.0f;
}

float feed_forward(const PiParams &p, float setpoint, float setpoint_rate, float batt_scale) {
  if (fabsf(setpoint) < p.v_dead) return 0.0f;
  const float mag = p.u_static + fabsf(setpoint) * p.kff * batt_scale;
  return (setpoint > 0.0f ? mag : -mag) + p.ka * setpoint_rate * batt_scale;
}

float pi_step(PiState *s, const PiParams &p, float setpoint, float setpoint_rate, float measured,
              float dt, float batt_scale, bool closed_loop) {
  if (fabsf(setpoint) < p.v_dead) {
    pi_reset(s); /* brake: MDD20A with PWM = 0 shorts the motor (dynamic brake) */
    return 0.0f;
  }
  const float ff = feed_forward(p, setpoint, setpoint_rate, batt_scale);
  if (!closed_loop) {
    s->last_u = clampf(ff, -p.u_max, p.u_max);
    return s->last_u;
  }
  const float e = setpoint - measured;
  const float u_unsat = ff + p.kp * e + s->integ;
  const bool sat_hi = u_unsat >= p.u_max;
  const bool sat_lo = u_unsat <= -p.u_max;
  const bool ramping = fabsf(setpoint_rate) > 1e-3f;
  /* conditional integration: not into saturation, not during a ramp */
  if (!ramping && !((sat_hi && e > 0.0f) || (sat_lo && e < 0.0f))) {
    s->integ = clampf(s->integ + p.ki * e * dt, -p.i_max, p.i_max);
  }
  s->last_u = clampf(ff + p.kp * e + s->integ, -p.u_max, p.u_max);
  return s->last_u;
}

float wrap_pi(float a) {
  const float two_pi = 6.28318531f;
  while (a > 3.14159265f) a -= two_pi;
  while (a < -3.14159265f) a += two_pi;
  return a;
}

void odom_integrate(Pose2D *pose, float d_left, float d_right, float b_eff) {
  const float ds = 0.5f * (d_left + d_right);
  const float dth = (d_right - d_left) / b_eff;
  const float mid = pose->theta + 0.5f * dth; /* midpoint (2nd order) integration */
  pose->x += ds * cosf(mid);
  pose->y += ds * sinf(mid);
  pose->theta = wrap_pi(pose->theta + dth);
}
