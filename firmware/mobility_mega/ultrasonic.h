/*
 * ultrasonic.h - six HC-SR04, fired one at a time in round-robin; the echo
 * pulse is timed by the pin-change interrupt of port K (A8..A13), so the
 * main loop never waits (replaces the blocking pulseIn() of 2023, D8).
 */
#ifndef MOBILITY_ULTRASONIC_H
#define MOBILITY_ULTRASONIC_H

#include <stdint.h>

namespace ultrasonic {
void begin();
void update(); /* call every loop() pass */
/* Latest range in mm (ICD order), 0xFFFF = no echo / out of range. */
uint16_t range(uint8_t i);
void ranges(uint16_t out[6]);
uint16_t minRange(); /* closest valid range, 0xFFFF if none */
uint32_t cycles();   /* completed measurements (all sensors) */
}

#endif
