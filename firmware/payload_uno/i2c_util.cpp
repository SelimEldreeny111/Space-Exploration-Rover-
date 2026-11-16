/*
 * i2c_util.cpp - see i2c_util.h.
 */
#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "i2c_util.h"

void i2c_begin() {
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  /* reset_with_timeout = true: the TWI hardware is re-initialised after a
   * timeout so the next transaction can succeed (bus glitch, sensor reset) */
  Wire.setWireTimeout(WIRE_TIMEOUT_US, true);
}

bool i2c_write8(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  const bool ok = Wire.endTransmission() == 0; /* 5 = timeout */
  Wire.clearWireTimeoutFlag();
  return ok;
}

bool i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { /* repeated start */
    Wire.clearWireTimeoutFlag();
    return false;
  }
  const uint8_t got = Wire.requestFrom(addr, n);
  if (got != n || Wire.getWireTimeoutFlag()) {
    while (Wire.available()) Wire.read();
    Wire.clearWireTimeoutFlag();
    return false;
  }
  for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}
