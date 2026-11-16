/*
 * imu_math.cpp - see imu_math.h.
 */
#include "imu_math.h"

#include <math.h>

static int16_t sat16(float x) {
  if (x > 32767.0f) return 32767;
  if (x < -32768.0f) return -32768;
  return (int16_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

int16_t mpu_acc_mm_s2(int16_t raw) { return sat16((float)raw * (9806.65f / MPU_ACC_LSB_PER_G)); }

float mpu_gyro_rad_s(int32_t raw) {
  return (float)raw * (3.14159265f / 180.0f / MPU_GYRO_LSB_PER_DPS);
}

int16_t mpu_gyro_mrad_s(int32_t raw) { return sat16(mpu_gyro_rad_s(raw) * 1000.0f); }

void cf_init(CompFilter *f, float alpha) {
  f->alpha = alpha;
  f->roll = 0.0f;
  f->pitch = 0.0f;
  f->init = false;
}

void cf_update(CompFilter *f, float ax, float ay, float az, float gx, float gy, float dt) {
  const float roll_acc = atan2f(ay, az);
  const float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));
  if (!f->init) { /* start from the gravity estimate, no slow convergence */
    f->roll = roll_acc;
    f->pitch = pitch_acc;
    f->init = true;
    return;
  }
  f->roll = f->alpha * (f->roll + gx * dt) + (1.0f - f->alpha) * roll_acc;
  f->pitch = f->alpha * (f->pitch + gy * dt) + (1.0f - f->alpha) * pitch_acc;
}

int16_t rad_to_cdeg(float rad) { return sat16(rad * (18000.0f / 3.14159265f)); }
