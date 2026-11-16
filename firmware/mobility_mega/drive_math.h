/*
 * drive_math.h - pure (Arduino-free) drive math for the Mobility MCU:
 * skid-steer mixing, setpoint slew limiting, feed-forward + PI wheel-speed
 * controller with anti-windup. Unit-tested on the PC (firmware/tests).
 */
#ifndef MOBILITY_DRIVE_MATH_H
#define MOBILITY_DRIVE_MATH_H

#include <stdint.h>

/* Skid steer (ICD 9.1): v_L = v - w*b/2, v_R = v + w*b/2 (w > 0 = turn left). */
void skid_steer(float v, float w, float b_eff, float *v_left, float *v_right);

/* Scale both sides by the same factor so that |v_left|,|v_right| <= vmax
 * (keeps the path curvature when a command exceeds the side speed limit). */
void limit_sides(float *v_left, float *v_right, float vmax);

/* Move `current` towards `target` by at most `max_step`. */
float slew_toward(float current, float target, float max_step);

float clampf(float x, float lo, float hi);

struct PiParams {
  float kp;       /* duty / (m/s)                         */
  float ki;       /* duty / m                             */
  float i_max;    /* |integrator| <= i_max                */
  float u_max;    /* |duty| <= u_max                      */
  float kff;      /* feed-forward duty per m/s at nominal V */
  float ka;       /* acceleration feed-forward, duty per m/s^2 */
  float u_static; /* static friction offset (duty)        */
  float v_dead;   /* |setpoint| < v_dead -> 0 output      */
};

struct PiState {
  float integ;
  float last_u;
};

void pi_reset(PiState *s);

/* Feed-forward duty for a side speed setpoint and its rate of change
 * (m/s^2, from the slew limiter); batt_scale = V_nominal/V_batt compensates
 * the supply voltage (1.0 if unknown). */
float feed_forward(const PiParams &p, float setpoint, float setpoint_rate, float batt_scale);

/*
 * One controller step. closed_loop = false -> feed-forward only (open-loop
 * DRIVE, or all encoders of the side faulty); the integrator is then frozen.
 * Anti-windup: conditional integration (the integrator only grows while the
 * output is not saturated in the direction of the error, and not while the
 * setpoint is still ramping - the ramp lag is the job of the acceleration
 * feed-forward) plus a clamp. A setpoint inside the dead band brakes
 * (0 duty) and clears the integrator.
 */
float pi_step(PiState *s, const PiParams &p, float setpoint, float setpoint_rate, float measured,
              float dt, float batt_scale, bool closed_loop);

/* Planar dead reckoning from left/right travelled distance (m). */
struct Pose2D {
  float x, y, theta;
};
void odom_integrate(Pose2D *pose, float d_left, float d_right, float b_eff);
float wrap_pi(float a);

#endif /* MOBILITY_DRIVE_MATH_H */
