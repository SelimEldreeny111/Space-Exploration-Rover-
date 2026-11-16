/*
 * legacy_tlm.h - legacy 7-byte GET_TLM (ICD 7.1) built by the Mega when the
 * 2023 LabVIEW GS calls it as 0x01 over the HC-12 link. Pure (no Arduino),
 * unit-tested on the PC.
 *
 *   ir = 0, gas_hi = gas_lo = 0, temp_C = 0, rh_pct = 0 (not on this MCU)
 *   us1_cm = front-centre range, us2_cm = min(front-left, front-right)
 *   ranges in cm, rounded, clamped to 255; no echo (0xFFFF) -> 255
 */
#ifndef MOBILITY_LEGACY_TLM_H
#define MOBILITY_LEGACY_TLM_H

#include <stdint.h>

#include "ssp_protocol.h"

uint8_t legacy_cm8(uint16_t mm);
/* range_mm in ICD 9.3 order: FL, FC, FR, L, R, rear */
void legacy_tlm_from_ranges(const uint16_t range_mm[6], SspTlmLegacy *t);

#endif
