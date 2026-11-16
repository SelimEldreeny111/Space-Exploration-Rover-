/*
 * imu_math.h - pure IMU math (no Arduino): unit conversion for the MPU-6050
 * at +-4 g / +-500 deg/s and a complementary roll/pitch filter.
 */
#ifndef PAYLOAD_IMU_MATH_H
#define PAYLOAD_IMU_MATH_H

#include <stdint.h>

#define MPU_ACC_LSB_PER_G 8192.0f   /* AFS_SEL = 1 (+-4 g)      */
#define MPU_GYRO_LSB_PER_DPS 65.5f  /* FS_SEL = 1 (+-500 deg/s) */

/* raw -> mm/s^2, saturated to int16 (+-32.767 m/s^2 = +-3.34 g, the ICD
 * field cannot hold the full +-4 g range) */
int16_t mpu_acc_mm_s2(int16_t raw);
/* raw (bias already removed) -> mrad/s */
int16_t mpu_gyro_mrad_s(int32_t raw);
float mpu_gyro_rad_s(int32_t raw);

/*
 * roll  = atan2(ay, az), pitch = atan2(-ax, sqrt(ay^2 + az^2)) from gravity,
 * fused with the integrated gyro rates:  angle = a*(angle + rate*dt) + (1-a)*acc
 */
struct CompFilter {
  float alpha;
  float roll, pitch; /* rad */
  bool init;
};
void cf_init(CompFilter *f, float alpha);
void cf_update(CompFilter *f, float ax, float ay, float az, float gx_rad_s, float gy_rad_s,
               float dt);
int16_t rad_to_cdeg(float rad);

#endif
