/*
 * mpu6050.cpp - see mpu6050.h.
 */
#include <Arduino.h>

#include "config.h"
#include "i2c_util.h"
#include "mpu6050.h"

namespace {
const uint8_t REG_SMPLRT_DIV = 0x19;
const uint8_t REG_CONFIG = 0x1A;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_WHO_AM_I = 0x75;

int16_t be16(const uint8_t *p) { return (int16_t)(((uint16_t)p[0] << 8) | p[1]); }
} // namespace

namespace mpu6050 {

bool begin() {
  uint8_t who = 0;
  if (!i2c_read(MPU_ADDR, REG_WHO_AM_I, &who, 1)) return false;
  /* 0x68 = MPU-6050; 0x70/0x72/0x98 = register-compatible clones (6500/...) */
  if (who != 0x68 && who != 0x70 && who != 0x72 && who != 0x98) return false;
  return i2c_write8(MPU_ADDR, REG_PWR_MGMT_1, 0x01) &&   /* wake, clock = PLL gyro X */
         i2c_write8(MPU_ADDR, REG_CONFIG, 0x05) &&       /* DLPF 10 Hz (acc & gyro)  */
         i2c_write8(MPU_ADDR, REG_SMPLRT_DIV, 39) &&     /* 1 kHz / 40 = 25 Hz       */
         i2c_write8(MPU_ADDR, REG_GYRO_CONFIG, 0x08) &&  /* FS_SEL 1: +-500 deg/s    */
         i2c_write8(MPU_ADDR, REG_ACCEL_CONFIG, 0x08);   /* AFS_SEL 1: +-4 g         */
}

bool read(int16_t acc[3], int16_t gyro[3]) {
  uint8_t b[14]; /* ax ay az temp gx gy gz, big-endian */
  if (!i2c_read(MPU_ADDR, REG_ACCEL_XOUT_H, b, sizeof b)) return false;
  for (uint8_t i = 0; i < 3; i++) {
    acc[i] = be16(b + 2 * i);
    gyro[i] = be16(b + 8 + 2 * i);
  }
  return true;
}

} // namespace mpu6050
