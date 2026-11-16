/*
 * battery.h - battery voltage on A0 (100k/22k divider), low-battery fault
 * (< 10.8 V for 2 s, cleared above 11.1 V for 2 s).
 */
#ifndef MOBILITY_BATTERY_H
#define MOBILITY_BATTERY_H

#include <stdint.h>

namespace battery {
void begin();
void update(uint32_t now_ms); /* samples every BATT_SAMPLE_MS */
uint16_t millivolts();
bool low();
}

#endif
