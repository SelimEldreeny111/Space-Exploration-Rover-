/*
 * bmp180.cpp - see bmp180.h.
 */
#include <Arduino.h>

#include "bmp180.h"
#include "config.h"
#include "i2c_util.h"

namespace {
const uint8_t REG_CALIB = 0xAA;
const uint8_t REG_CHIP_ID = 0xD0;
const uint8_t REG_CTRL = 0xF4;
const uint8_t REG_OUT = 0xF6;
const uint8_t CMD_TEMP = 0x2E;
const uint8_t CMD_PRES = 0x34;
const uint8_t CHIP_ID = 0x55;
} // namespace

bool Bmp180::begin(uint8_t oss) {
  oss_ = oss > 3 ? 3 : oss;
  ok_ = false;
  state_ = S_OFF;
  uint8_t id = 0;
  if (!i2c_read(BMP180_ADDR, REG_CHIP_ID, &id, 1) || id != CHIP_ID) return false;
  uint8_t raw[22];
  if (!i2c_read(BMP180_ADDR, REG_CALIB, raw, sizeof raw)) return false;
  if (!bmp180_parse_calib(raw, &cal_)) return false;
  state_ = S_IDLE;
  last_start_ = millis() - 100000UL; /* measure right away */
  return true;
}

void Bmp180::fail() {
  ok_ = false;
  state_ = S_OFF;
}

void Bmp180::update(uint32_t now, uint16_t period_ms) {
  switch (state_) {
  case S_IDLE:
    if ((uint32_t)(now - last_start_) < period_ms) return;
    if (!i2c_write8(BMP180_ADDR, REG_CTRL, CMD_TEMP)) return fail();
    last_start_ = now;
    t0_ = now;
    state_ = S_WAIT_T;
    return;
  case S_WAIT_T: {
    if ((uint32_t)(now - t0_) < 6) return; /* 4.5 ms max conversion */
    uint8_t b[2];
    if (!i2c_read(BMP180_ADDR, REG_OUT, b, 2)) return fail();
    ut_ = ((int32_t)b[0] << 8) | b[1];
    if (!i2c_write8(BMP180_ADDR, REG_CTRL, (uint8_t)(CMD_PRES | (oss_ << 6)))) return fail();
    t0_ = now;
    state_ = S_WAIT_P;
    return;
  }
  case S_WAIT_P: {
    if ((uint32_t)(now - t0_) < bmp180_pressure_wait_ms(oss_)) return;
    uint8_t b[3];
    if (!i2c_read(BMP180_ADDR, REG_OUT, b, 3)) return fail();
    const int32_t up =
        (int32_t)((((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2]) >> (8 - oss_));
    int32_t b5;
    t_dc_ = bmp180_temperature_dC(cal_, ut_, &b5);
    p_pa_ = bmp180_pressure_Pa(cal_, up, oss_, b5);
    /* plausibility: 300..1100 hPa, -40..85 degC (sensor range) */
    ok_ = p_pa_ > 30000L && p_pa_ < 110000L && t_dc_ > -400 && t_dc_ < 850;
    state_ = S_IDLE;
    return;
  }
  case S_OFF:
  default:
    return;
  }
}
