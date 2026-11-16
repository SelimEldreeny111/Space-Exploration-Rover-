/*
 * arm_math.h - pure arm logic (no Arduino): joint table (ICD 9.5), named
 * poses, servo pulse mapping, minimum-jerk trajectory. Unit-tested on the PC.
 */
#ifndef ARM_MATH_H
#define ARM_MATH_H

#include <stdint.h>

#define ARM_NJ 6
#define POSE_KEEP 0xFF /* in a pose: keep this joint where it is */

struct JointSpec {
  uint8_t min_deg, max_deg, home_deg;
};
/* ch0 base yaw, ch1 shoulder, ch2 elbow, ch3 wrist pitch, ch4 wrist roll,
 * ch5 gripper (10 = open) */
extern const JointSpec kJoints[ARM_NJ];

/* Named poses, index = ICD pose_id (0 HOME .. 7 DEPOSIT). */
struct PoseSpec {
  uint8_t deg[ARM_NJ]; /* POSE_KEEP = unchanged */
  uint16_t duration_ms;
};
#define ARM_NUM_POSES 8
extern const PoseSpec kPoses[ARM_NUM_POSES];
#define GRIPPER_OPEN_DEG 10
#define GRIPPER_CLOSE_DEG 75

/* Clamp to the joint limits; returns true if the value had to be changed. */
bool arm_clamp(uint8_t joint, uint8_t deg_in, uint8_t *deg_out);

/* 0 deg -> 102 counts (0.5 ms), 180 deg -> 512 counts (2.5 ms) at 50 Hz. */
uint16_t arm_deg_to_counts(float deg);

/* Minimum-jerk profile s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5, tau in [0,1]. */
float min_jerk(float tau);

/* Duration actually used: max(requested, time that keeps the min-jerk
 * peak speed 1.875*|dq|/T below max_dps). */
uint16_t arm_motion_duration(const float from[ARM_NJ], const uint8_t to[ARM_NJ],
                             uint16_t requested_ms, float max_dps);

/* Interpolated joint angles at time t_ms of a motion of length T_ms. */
void arm_interpolate(const float from[ARM_NJ], const uint8_t to[ARM_NJ], uint32_t t_ms,
                     uint16_t T_ms, float out[ARM_NJ]);

#endif
