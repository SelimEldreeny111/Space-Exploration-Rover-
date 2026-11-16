/*
 * encoders.h - six Hall wheel encoders, channel A on external interrupts
 * (CHANGE -> 840 counts / wheel rev), channel B read in the ISR for the
 * direction. Counts are signed so that + = wheel moving the rover forward.
 */
#ifndef MOBILITY_ENCODERS_H
#define MOBILITY_ENCODERS_H

#include <stdint.h>

namespace encoders {
void begin();
/* Atomic snapshot of all six counters (sign-corrected, ICD wheel order). */
void read(int32_t out[6]);
}

#endif
