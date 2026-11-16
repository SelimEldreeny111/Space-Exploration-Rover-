/*
 * ultrasonic.cpp - non-blocking HC-SR04 round-robin.
 *
 * Only one sensor is active at a time and only its echo pin is enabled in
 * PCMSK2, so the PCINT2 ISR knows which pulse it is timing. A sensor gets a
 * fixed 25 ms slot (= echo timeout = 4.3 m round trip): the next sensor is
 * not fired while echoes of the previous burst may still be travelling,
 * which avoids cross-talk between sensors. Full cycle 6 x 25 ms = 150 ms
 * (each sensor ~6.7 Hz); RANGES (10 Hz) always carries the latest values.
 *
 * micros() is safe inside the ISR (it accounts for a pending Timer0
 * overflow); 4 us resolution = 0.7 mm.
 */
#include <Arduino.h>
#include <util/atomic.h>

#include "config.h"
#include "ultrasonic.h"

namespace {
enum EchoState : uint8_t { ECHO_IDLE, ECHO_WAIT_RISE, ECHO_WAIT_FALL, ECHO_DONE };

volatile uint8_t s_echo_state = ECHO_IDLE;
volatile uint32_t s_rise_us = 0;
volatile uint32_t s_fall_us = 0;
volatile uint8_t s_mask = 0; /* PINK bit of the active sensor */

uint8_t s_active = NUM_US - 1;
uint32_t s_trig_us = 0;
uint32_t s_trig_ms = 0;
bool s_busy = false;
uint16_t s_range[NUM_US];
uint32_t s_cycles = 0;
} // namespace

ISR(PCINT2_vect) {
  const uint8_t pins = PINK;
  const uint32_t t = micros();
  if (pins & s_mask) { /* rising edge of the active echo pin */
    if (s_echo_state == ECHO_WAIT_RISE) {
      s_rise_us = t;
      s_echo_state = ECHO_WAIT_FALL;
    }
  } else if (s_echo_state == ECHO_WAIT_FALL) { /* falling edge */
    s_fall_us = t;
    s_echo_state = ECHO_DONE;
  }
}

namespace ultrasonic {

void begin() {
  for (uint8_t i = 0; i < NUM_US; i++) {
    pinMode(US_TRIG_PIN[i], OUTPUT);
    digitalWrite(US_TRIG_PIN[i], LOW);
    pinMode(US_ECHO_PIN_FIRST + i, INPUT); /* HC-SR04 echo is push-pull */
    s_range[i] = 0xFFFF;
  }
  PCMSK2 = 0;
  PCIFR = _BV(PCIF2);
  PCICR |= _BV(PCIE2);
  s_trig_ms = millis() - US_SLOT_MS;
}

static void storeResult(uint16_t mm) {
  s_range[s_active] = mm;
  s_cycles++;
  s_busy = false;
}

static uint16_t widthToMm(uint32_t width_us) {
  /* speed of sound 343 m/s (20 degC), round trip: mm = us * 0.1715 */
  const uint32_t mm = (width_us * 343UL + 1000UL) / 2000UL;
  if (mm > US_MAX_MM) return 0xFFFF;
  if (mm < US_MIN_MM) return US_MIN_MM; /* something is touching the sensor */
  return (uint16_t)mm;
}

void update() {
  const uint32_t now_ms = millis();
  if (s_busy) {
    uint8_t st;
    uint32_t rise, fall;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      st = s_echo_state;
      rise = s_rise_us;
      fall = s_fall_us;
    }
    if (st == ECHO_DONE) {
      storeResult(widthToMm(fall - rise));
    } else if ((uint32_t)(micros() - s_trig_us) > US_TIMEOUT_US) {
      storeResult(0xFFFF); /* 25 ms without a complete echo */
    }
    if (!s_busy) {
      PCMSK2 = 0;
      s_echo_state = ECHO_IDLE;
    }
  }
  /* fire the next sensor at the start of its slot */
  if (!s_busy && (uint32_t)(now_ms - s_trig_ms) >= US_SLOT_MS) {
    s_active = (uint8_t)((s_active + 1) % NUM_US);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      s_mask = (uint8_t)_BV(s_active); /* A8+i = PK(i) */
      s_echo_state = ECHO_WAIT_RISE;
      PCMSK2 = s_mask;
      PCIFR = _BV(PCIF2); /* discard edges of the previous sensor */
    }
    /* 10 us trigger pulse: the only busy-wait in the firmware (10 us) */
    digitalWrite(US_TRIG_PIN[s_active], HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG_PIN[s_active], LOW);
    s_trig_us = micros();
    s_trig_ms = now_ms;
    s_busy = true;
  }
}

uint16_t range(uint8_t i) { return i < NUM_US ? s_range[i] : 0xFFFF; }

void ranges(uint16_t out[6]) {
  for (uint8_t i = 0; i < NUM_US; i++) out[i] = s_range[i];
}

uint16_t minRange() {
  uint16_t m = 0xFFFF;
  for (uint8_t i = 0; i < NUM_US; i++)
    if (s_range[i] < m) m = s_range[i];
  return m;
}

uint32_t cycles() { return s_cycles; }

} // namespace ultrasonic
