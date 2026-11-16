/*
 * legacy_tlm.cpp - see legacy_tlm.h.
 */
#include "legacy_tlm.h"

uint8_t legacy_cm8(uint16_t mm) {
  if (mm == SSP_NO_ECHO) return 255;
  const uint16_t cm = (uint16_t)((mm + 5u) / 10u);
  return cm > 255u ? (uint8_t)255 : (uint8_t)cm;
}

void legacy_tlm_from_ranges(const uint16_t r[6], SspTlmLegacy *t) {
  t->ir = 0;
  t->gas_hi = 0;
  t->gas_lo = 0;
  t->us1_cm = legacy_cm8(r[1]);                   /* front-centre          */
  t->us2_cm = legacy_cm8(r[0] < r[2] ? r[0] : r[2]); /* closer front corner (no echo = 0xFFFF = far) */
  t->temp_c = 0;
  t->rh_pct = 0;
}
