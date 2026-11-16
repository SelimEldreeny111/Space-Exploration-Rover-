/*
 * arm_math.cpp - see arm_math.h.
 *
 * Named poses (degrees, K = keep current angle). Convention assumed for the
 * 2023 arm: shoulder 90 = upper arm vertical, < 90 leans forward/down;
 * elbow 90 = forearm at right angle, > 90 folds the forearm down; wrist
 * pitch 90 = in line with the forearm. THESE ARE STARTING VALUES - tune
 * them on the real arm (README, "arm pose tuning") and keep this table and
 * the README in sync.
 *
 *  id pose          base sh  elbow wristP wristR grip  T[ms]  purpose
 *  0  HOME           90  90   90    90     90     10   1500   ICD home, gripper open
 *  1  STOW           90 160   15    30     90     60   2000   folded back over the deck for driving
 *  2  READY          90 110   60   120     90      K   1500   raised in front, camera view of the ground
 *  3  REACH_GROUND   90  40  140    50     90      K   2000   gripper at ground level in front
 *  4  GRIP_OPEN       K   K    K     K      K     10    600   open gripper only
 *  5  GRIP_CLOSE      K   K    K     K      K     75    800   close on the sample (75, not 80: no stall)
 *  6  LIFT_SAMPLE    90 110   70   110     90      K   1500   lift the sample clear of the ground
 *  7  DEPOSIT       170 120   60    60     90      K   2000   over the sample box (then gripper opens)
 */
#include "arm_math.h"

#include <math.h>

const JointSpec kJoints[ARM_NJ] = {
    {0, 180, 90}, /* 0 base yaw    */
    {15, 165, 90}, /* 1 shoulder   */
    {0, 180, 90}, /* 2 elbow       */
    {0, 180, 90}, /* 3 wrist pitch */
    {0, 180, 90}, /* 4 wrist roll  */
    {10, 80, 10}, /* 5 gripper     */
};

#define K POSE_KEEP
const PoseSpec kPoses[ARM_NUM_POSES] = {
    {{90, 90, 90, 90, 90, GRIPPER_OPEN_DEG}, 1500},  /* HOME         */
    {{90, 160, 15, 30, 90, 60}, 2000},               /* STOW         */
    {{90, 110, 60, 120, 90, K}, 1500},               /* READY        */
    {{90, 40, 140, 50, 90, K}, 2000},                /* REACH_GROUND */
    {{K, K, K, K, K, GRIPPER_OPEN_DEG}, 600},        /* GRIP_OPEN    */
    {{K, K, K, K, K, GRIPPER_CLOSE_DEG}, 800},       /* GRIP_CLOSE   */
    {{90, 110, 70, 110, 90, K}, 1500},               /* LIFT_SAMPLE  */
    {{170, 120, 60, 60, 90, K}, 2000},               /* DEPOSIT      */
};
#undef K

bool arm_clamp(uint8_t j, uint8_t in, uint8_t *out) {
  uint8_t v = in;
  if (v < kJoints[j].min_deg) v = kJoints[j].min_deg;
  if (v > kJoints[j].max_deg) v = kJoints[j].max_deg;
  *out = v;
  return v != in;
}

uint16_t arm_deg_to_counts(float deg) {
  if (deg < 0.0f) deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;
  return (uint16_t)(102.0f + deg * (410.0f / 180.0f) + 0.5f);
}

float min_jerk(float tau) {
  if (tau <= 0.0f) return 0.0f;
  if (tau >= 1.0f) return 1.0f;
  const float t3 = tau * tau * tau;
  return t3 * (10.0f + tau * (-15.0f + 6.0f * tau));
}

uint16_t arm_motion_duration(const float from[ARM_NJ], const uint8_t to[ARM_NJ],
                             uint16_t requested_ms, float max_dps) {
  float dq = 0.0f;
  for (uint8_t j = 0; j < ARM_NJ; j++) {
    const float d = fabsf((float)to[j] - from[j]);
    if (d > dq) dq = d;
  }
  const float t_min_ms = 1.875f * dq / max_dps * 1000.0f;
  const float t = (float)requested_ms > t_min_ms ? (float)requested_ms : t_min_ms;
  return t > 65535.0f ? (uint16_t)65535u : (uint16_t)(t + 0.5f);
}

void arm_interpolate(const float from[ARM_NJ], const uint8_t to[ARM_NJ], uint32_t t_ms,
                     uint16_t T_ms, float out[ARM_NJ]) {
  const float s = T_ms == 0 ? 1.0f : min_jerk((float)t_ms / (float)T_ms);
  for (uint8_t j = 0; j < ARM_NJ; j++) out[j] = from[j] + ((float)to[j] - from[j]) * s;
}
