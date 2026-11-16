/*
 * encoders.cpp - wheel encoder ISRs.
 *
 * Timing: at the 83 rpm no-load speed one wheel produces 840*83/60 = 1162
 * A-edges/s, all six together < 7 k interrupts/s. Each ISR reads both
 * channels through cached port-register pointers (no digitalRead(): ~0.1 us
 * instead of ~4 us), so the total interrupt load stays around 1-2 % CPU.
 *
 * Direction (x2 decoding on A): right after an A edge, A != B means A leads
 * B (one direction), A == B means B leads A (the other). The per-wheel
 * mounting sign ENC_SIGN is applied when the counters are read, so the ISRs
 * stay identical.
 */
#include <Arduino.h>
#include <util/atomic.h>

#include "config.h"
#include "encoders.h"

namespace {
volatile int32_t s_count[NUM_WHEELS];
volatile uint8_t *s_a_reg[NUM_WHEELS];
volatile uint8_t *s_b_reg[NUM_WHEELS];
uint8_t s_a_mask[NUM_WHEELS];
uint8_t s_b_mask[NUM_WHEELS];

template <uint8_t I> void encIsr() {
  const bool a = (*s_a_reg[I] & s_a_mask[I]) != 0;
  const bool b = (*s_b_reg[I] & s_b_mask[I]) != 0;
  if (a != b) {
    s_count[I]++;
  } else {
    s_count[I]--;
  }
}

typedef void (*IsrFn)();
const IsrFn kIsr[NUM_WHEELS] = {encIsr<0>, encIsr<1>, encIsr<2>,
                                encIsr<3>, encIsr<4>, encIsr<5>};
} // namespace

namespace encoders {

void begin() {
  for (uint8_t i = 0; i < NUM_WHEELS; i++) {
    /* Hall sensors are usually open collector: enable the pull-ups (the
     * Mega R3 also has 10k pull-ups on pins 20/21 = SDA/SCL). */
    pinMode(ENC_A_PIN[i], INPUT_PULLUP);
    pinMode(ENC_B_PIN[i], INPUT_PULLUP);
    s_a_reg[i] = portInputRegister(digitalPinToPort(ENC_A_PIN[i]));
    s_b_reg[i] = portInputRegister(digitalPinToPort(ENC_B_PIN[i]));
    s_a_mask[i] = digitalPinToBitMask(ENC_A_PIN[i]);
    s_b_mask[i] = digitalPinToBitMask(ENC_B_PIN[i]);
    s_count[i] = 0;
  }
  for (uint8_t i = 0; i < NUM_WHEELS; i++) {
    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[i]), kIsr[i], CHANGE);
  }
}

void read(int32_t out[6]) {
  int32_t raw[NUM_WHEELS];
  /* 32-bit counters are updated by ISRs: copy them with interrupts off so
   * no value is torn between its bytes (about 3 us). */
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = 0; i < NUM_WHEELS; i++) raw[i] = s_count[i];
  }
  for (uint8_t i = 0; i < NUM_WHEELS; i++) out[i] = ENC_SIGN[i] > 0 ? raw[i] : -raw[i];
}

} // namespace encoders
