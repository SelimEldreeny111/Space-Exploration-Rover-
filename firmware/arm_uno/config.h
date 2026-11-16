/*
 * config.h - MISSION X Arm MCU (Arduino Uno + PCA9685, SSP address 0x04).
 * ICD v2.0 section 9.5.
 */
#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

#define LINK_BAUD 115200UL
#define RX_TIMEOUT_MS 20 /* ICD 3 step 6 (L2-L4) */

#define PCA9685_ADDR 0x40
#define SERVO_FREQ_HZ 50
/* PCA9685 internal oscillator: nominally 25 MHz, real chips 23-27 MHz.
 * Calibrate (README): measure the pulse of a channel at 90 deg (should be
 * 1.50 ms) and set PCA_OSC_HZ = 27e6 * 1.50 / measured_ms. 27 MHz is the
 * value Adafruit recommends for most boards. */
#define PCA_OSC_HZ 27000000UL
#define I2C_CLOCK_HZ 400000UL
#define WIRE_TIMEOUT_US 5000UL

#define SERVO_UPDATE_MS 20   /* trajectory sampling = servo frame (50 Hz) */
#define STATE_PERIOD_MS 200  /* ARM_STATE 5 Hz (+ on change)               */
#define HEARTBEAT_PERIOD_MS 1000
#define PCA_RETRY_MS 5000
/* A trajectory is stretched if its min-jerk peak joint speed
 * (1.875 * |dq| / T) would exceed this (protects gears and rover balance). */
#define MAX_JOINT_SPEED_DPS 120.0f
#define POSE_DWELL_MS 300    /* pause between the steps of COLLECT_OBJECT */

#endif
