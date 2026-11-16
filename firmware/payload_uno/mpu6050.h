/*
 * mpu6050.h - minimal MPU-6050 driver (I2C 0x68): wake-up, +-4 g,
 * +-500 deg/s, 10 Hz digital low-pass (no aliasing at the 25 Hz IMU rate),
 * burst read of accel + gyro.
 */
#ifndef PAYLOAD_MPU6050_H
#define PAYLOAD_MPU6050_H

#include <stdint.h>

namespace mpu6050 {
bool begin();
/* raw sensor axes; returns false on an I2C error */
bool read(int16_t acc[3], int16_t gyro[3]);
}

#endif
