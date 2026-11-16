/*
 * i2c_util.h - small register helpers on top of Wire with error checks.
 * Every call is bounded by Wire.setWireTimeout() (AVR core >= 1.8.3), so a
 * missing or hung sensor returns false instead of blocking loop().
 */
#ifndef PAYLOAD_I2C_UTIL_H
#define PAYLOAD_I2C_UTIL_H

#include <stdint.h>

void i2c_begin();
bool i2c_write8(uint8_t addr, uint8_t reg, uint8_t value);
bool i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n);

#endif
