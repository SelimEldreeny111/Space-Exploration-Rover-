/*
 * drive.h - wheel-speed control of the six-wheel skid-steer drive.
 *
 * Body twist (v, w) -> skid-steer side speeds -> setpoint slew limit ->
 * per-side feed-forward + PI on the mean speed of the healthy encoders of
 * that side (50 Hz) -> the same duty to the three motors of the side.
 * Also: SET_VELOCITY watchdog, DRIVE duration, encoder-fault detection and
 * wheel-odometry dead reckoning.
 */
#ifndef MOBILITY_DRIVE_H
#define MOBILITY_DRIVE_H

#include <stdint.h>

#include "drive_math.h"

namespace drive {

enum Source : uint8_t { SRC_NONE = 0, SRC_VELOCITY, SRC_DRIVE, SRC_SCAN };

void begin();
/* SET_VELOCITY: closed loop; stops (and sets the timeout fault) if not
 * refreshed within timeout_ms (0 -> 500 ms). */
void setVelocity(float v, float w, uint16_t timeout_ms, uint32_t now_ms);
/* DRIVE: open loop (feed-forward only) for duration_ms. */
void driveOpenLoop(float v, float w, uint16_t duration_ms, uint32_t now_ms);
/* START_SCAN behaviour output: closed loop, no timeout. */
void setScanTwist(float v, float w);
/* Ramp down to standstill (STOP_SCAN, end of autonomy). */
void stop();
/* Enable/disable the outputs. Disabling (e-stop) zeroes every PWM output
 * immediately, clears the setpoints, integrators and the command source. */
void setEnabled(bool enabled);
void update(uint32_t now_ms); /* call every loop(); runs the 50 Hz controller */

Source source();
bool timeoutFault();
void clearTimeoutFault();
bool encoderFault();       /* any wheel commanded without ticks for 1 s */
uint8_t encoderFaultMask(); /* bit i = wheel i */
bool moving();
const Pose2D &pose();
float sideSpeed(uint8_t side); /* filtered measured speed, 0 left 1 right */
float sideSetpoint(uint8_t side);

} // namespace drive

#endif
