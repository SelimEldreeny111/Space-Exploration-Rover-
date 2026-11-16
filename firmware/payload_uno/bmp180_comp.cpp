/*
 * bmp180_comp.cpp - see bmp180_comp.h. All arithmetic in explicit 32-bit
 * types, exactly as on the AVR (long = 32 bit), following the Bosch
 * reference driver (shifts for the 2^n divisions).
 */
#include "bmp180_comp.h"

static int16_t be16s(const uint8_t *p) { return (int16_t)(((uint16_t)p[0] << 8) | p[1]); }
static uint16_t be16u(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0] << 8) | p[1]); }

bool bmp180_parse_calib(const uint8_t raw[22], Bmp180Calib *c) {
  for (uint8_t i = 0; i < 22; i += 2) {
    const uint16_t w = be16u(raw + i);
    if (w == 0x0000u || w == 0xFFFFu) return false;
  }
  c->ac1 = be16s(raw + 0);
  c->ac2 = be16s(raw + 2);
  c->ac3 = be16s(raw + 4);
  c->ac4 = be16u(raw + 6);
  c->ac5 = be16u(raw + 8);
  c->ac6 = be16u(raw + 10);
  c->b1 = be16s(raw + 12);
  c->b2 = be16s(raw + 14);
  c->mb = be16s(raw + 16);
  c->mc = be16s(raw + 18);
  c->md = be16s(raw + 20);
  return true;
}

int32_t bmp180_temperature_dC(const Bmp180Calib &c, int32_t ut, int32_t *b5) {
  const int32_t x1 = ((ut - (int32_t)c.ac6) * (int32_t)c.ac5) >> 15;
  const int32_t den = x1 + (int32_t)c.md;
  const int32_t x2 = den != 0 ? ((int32_t)c.mc * 2048) / den : 0;
  *b5 = x1 + x2;
  return (*b5 + 8) >> 4;
}

int32_t bmp180_pressure_Pa(const Bmp180Calib &c, int32_t up, uint8_t oss, int32_t b5) {
  if (oss > 3) oss = 3;
  const int32_t b6 = b5 - 4000;
  int32_t x1 = ((int32_t)c.b2 * ((b6 * b6) >> 12)) >> 11;
  int32_t x2 = ((int32_t)c.ac2 * b6) >> 11;
  int32_t x3 = x1 + x2;
  const int32_t b3 = ((((int32_t)c.ac1 * 4 + x3) * ((int32_t)1 << oss)) + 2) >> 2;
  x1 = ((int32_t)c.ac3 * b6) >> 13;
  x2 = ((int32_t)c.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  const uint32_t b4 = ((uint32_t)c.ac4 * (uint32_t)(x3 + 32768)) >> 15;
  if (b4 == 0) return 0;
  const uint32_t b7 = ((uint32_t)up - (uint32_t)b3) * (uint32_t)(50000UL >> oss);
  int32_t p = (b7 < 0x80000000UL) ? (int32_t)((b7 << 1) / b4) : (int32_t)((b7 / b4) << 1);
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;
  return p;
}

uint8_t bmp180_pressure_wait_ms(uint8_t oss) {
  /* datasheet max 4.5 / 7.5 / 13.5 / 25.5 ms; +1 ms because millis() can
   * advance by one right after the conversion was started */
  static const uint8_t kWait[4] = {6, 9, 15, 27};
  return kWait[oss > 3 ? 3 : oss];
}
