/*
 * motors.h - six DC gear-motors on 3 x Cytron MDD20A (sign-magnitude:
 * PWM pin = magnitude, DIR pin = direction).
 */
#ifndef MOBILITY_MOTORS_H
#define MOBILITY_MOTORS_H

#include <stdint.h>

namespace motors {
void begin();
/* duty in [-1, 1], + = drive the rover forward (MOTOR_SIGN applied). */
void set(uint8_t wheel, float duty);
void stopAll(); /* PWM = 0 on all channels (MDD20A: both outputs low = brake) */
float duty(uint8_t wheel);
}

#endif
