/*
 * battery.cpp - see battery.h. analogRead() takes ~112 us; it is called at
 * 10 Hz only. Motor current surges sag the voltage for a few hundred ms, so
 * the low-battery decision needs the condition to persist for 2 s.
 */
#include <Arduino.h>

#include "battery.h"
#include "config.h"

namespace {
uint32_t s_last_ms = 0;
uint32_t s_cond_since = 0;
float s_mv = 0.0f;
bool s_low = false;
bool s_init = false;
}

namespace battery {

void begin() {
  pinMode(BATT_PIN, INPUT);
  s_init = false;
}

void update(uint32_t now_ms) {
  if (s_init && (uint32_t)(now_ms - s_last_ms) < BATT_SAMPLE_MS) return;
  s_last_ms = now_ms;
  const float mv = (float)((uint32_t)analogRead(BATT_PIN) * BATT_UV_PER_COUNT / 1000UL);
  if (!s_init) {
    s_mv = mv;
    s_init = true;
    s_cond_since = now_ms;
    s_low = mv < BATT_LOW_MV;
    return;
  }
  s_mv += 0.2f * (mv - s_mv); /* ~0.5 s low-pass at 10 Hz */
  /* the "other" state must hold continuously for BATT_FILTER_MS to switch */
  const bool cond = s_low ? (s_mv > BATT_OK_MV) : (s_mv < BATT_LOW_MV);
  if (!cond) {
    s_cond_since = now_ms;
  } else if ((uint32_t)(now_ms - s_cond_since) >= BATT_FILTER_MS) {
    s_low = !s_low;
    s_cond_since = now_ms;
  }
}

uint16_t millivolts() { return (uint16_t)(s_mv + 0.5f); }
bool low() { return s_low; }

} // namespace battery
