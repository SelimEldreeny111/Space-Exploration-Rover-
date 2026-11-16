/*
 * bmp180_comp.h - Bosch BMP180 integer compensation (datasheet BST-BMP180-DS000
 * section 3.5), pure C++ (no Arduino) so it is unit-tested on the PC with
 * the datasheet example (UT 27898, UP 23843, oss 0 -> 15.0 degC, 69964 Pa).
 */
#ifndef PAYLOAD_BMP180_COMP_H
#define PAYLOAD_BMP180_COMP_H

#include <stdint.h>

struct Bmp180Calib {
  int16_t ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t b1, b2, mb, mc, md;
};

/* Parse the 22 calibration bytes read from 0xAA..0xBF (big-endian words).
 * Returns false if any word is 0x0000 or 0xFFFF (bus/sensor failure). */
bool bmp180_parse_calib(const uint8_t raw[22], Bmp180Calib *c);

/* Temperature in 0.1 degC; *b5 is needed by the pressure compensation. */
int32_t bmp180_temperature_dC(const Bmp180Calib &c, int32_t ut, int32_t *b5);

/* Pressure in Pa from the raw UP (already shifted by 8 - oss). */
int32_t bmp180_pressure_Pa(const Bmp180Calib &c, int32_t up, uint8_t oss, int32_t b5);

/* Wait in ms (millis() ticks) before reading a pressure result at oss. */
uint8_t bmp180_pressure_wait_ms(uint8_t oss);

#endif
