/*
 * bmp180.h - minimal non-blocking BMP180 driver (I2C 0x77).
 * One temperature + pressure measurement per period, driven by update():
 *   start T conversion -> wait 5 ms -> read UT, start P conversion ->
 *   wait (oss) -> read UP, compensate (Bosch integer formulas).
 * Waiting is done by returning from update(), never by delay().
 */
#ifndef PAYLOAD_BMP180_H
#define PAYLOAD_BMP180_H

#include <stdint.h>

#include "bmp180_comp.h"

class Bmp180 {
public:
  Bmp180() : ok_(false), state_(S_OFF), oss_(0), t0_(0), last_start_(0), ut_(0), p_pa_(0), t_dc_(0) {}
  /* Check the chip id (0x55) and read the calibration EEPROM. */
  bool begin(uint8_t oss);
  void update(uint32_t now_ms, uint16_t period_ms);
  bool ok() const { return ok_; }
  bool running() const { return state_ != S_OFF; } /* false: needs begin() */
  int32_t pressurePa() const { return p_pa_; }
  int16_t temperature_cC() const { return (int16_t)(t_dc_ * 10); }

private:
  enum State : uint8_t { S_OFF, S_IDLE, S_WAIT_T, S_WAIT_P };
  void fail();

  Bmp180Calib cal_;
  bool ok_;
  State state_;
  uint8_t oss_;
  uint32_t t0_;
  uint32_t last_start_;
  int32_t ut_;
  int32_t p_pa_;
  int32_t t_dc_;
};

#endif
