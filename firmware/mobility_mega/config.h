/*
 * config.h - MISSION X Mobility MCU (Arduino Mega 2560, SSP address 0x02).
 * Pin map ICD v2.0 section 9.3, drive train section 9.2, vehicle section 9.1.
 * Everything a technician may have to change on the bench is in this file.
 */
#ifndef MOBILITY_CONFIG_H
#define MOBILITY_CONFIG_H

#include <stdint.h>

/* ----------------------------------------------------------------- links */
#define BRAIN_BAUD 115200UL   /* L2: USB CDC to the Raspberry Pi (Serial)    */
#define SAFE_BAUD 9600UL      /* L1-safe: HC-12 on Serial2 (pins 16/17)      */
#define BRAIN_RX_TIMEOUT_MS 20 /* ICD 3 step 6: 20 ms on L2-L4               */
#define SAFE_RX_TIMEOUT_MS 50  /* ICD 3 step 6: 50 ms on L1                  */
/* fault bit4 "HC-12 safe-mode active" stays set this long after the last
 * valid frame received on Serial2 */
#define SAFE_LINK_ACTIVE_MS 60000UL

/* ------------------------------------------------- pins (ICD 9.3), wheel
 * order everywhere: L-front, L-mid, L-rear, R-front, R-mid, R-rear */
#define NUM_WHEELS 6
static const uint8_t ENC_A_PIN[NUM_WHEELS] = {2, 3, 18, 19, 20, 21}; /* INT4,5,3,2,1,0 */
static const uint8_t ENC_B_PIN[NUM_WHEELS] = {22, 23, 24, 25, 26, 27};
static const uint8_t MOTOR_PWM_PIN[NUM_WHEELS] = {5, 6, 7, 8, 11, 12};
static const uint8_t MOTOR_DIR_PIN[NUM_WHEELS] = {28, 29, 30, 31, 32, 33};

/* Ultrasonic order: front-left, front-centre, front-right, left, right, rear */
#define NUM_US 6
static const uint8_t US_TRIG_PIN[NUM_US] = {34, 36, 38, 40, 42, 44};
/* ECHO pins A8..A13 = PK0..PK5 = PCINT16..21 (pin-change interrupt bank 2) */
static const uint8_t US_ECHO_PIN_FIRST = 62; /* A8 on the Mega */

#define HC12_SET_PIN 48 /* HIGH = transparent mode (LOW = AT commands)        */
#define ESTOP_PIN 49    /* NC switch to GND, INPUT_PULLUP: HIGH = stop        */
#define BATT_PIN A0     /* 100k / 22k divider                                 */
#define LED_PIN 13

/* ---------------------------------------------------- sign conventions
 * +ticks / +duty must mean "wheel moves the rover forward". Left and right
 * motors are mirror images, so the right side is inverted by default.
 * VERIFY on the rover (README: "encoder sign check") and edit here. */
static const int8_t ENC_SIGN[NUM_WHEELS] = {+1, +1, +1, -1, -1, -1};
static const int8_t MOTOR_SIGN[NUM_WHEELS] = {+1, +1, +1, -1, -1, -1};

/* ------------------------------------------------- vehicle (ICD 9.1/9.2) */
#define WHEEL_RADIUS_M 0.100f
#define COUNTS_PER_REV 840.0f /* 7 PPR x 60:1 x 2 (A-channel CHANGE)      */
#define M_PER_COUNT (2.0f * 3.14159265f * WHEEL_RADIUS_M / COUNTS_PER_REV) /* 0.748 mm */
#define TRACK_M 0.58f
#define SKID_CHI 1.5f /* effective-track factor, calibrate (README)         */
#define B_EFF_M (SKID_CHI * TRACK_M)
#define V_NOLOAD_12V 0.869f /* 83 rpm x 2 pi x 0.1 m / 60 at 12 V          */
#define MAX_SPEED 0.45f     /* m/s   software limit (rover_core params.py) */
#define MAX_YAW_RATE 1.0f   /* rad/s software limit                        */
#define MAX_SIDE_SPEED 0.70f /* m/s per side after skid-steer mixing       */

/* ----------------------------------------------------- wheel-speed PI */
#define CONTROL_PERIOD_MS 20 /* 50 Hz                                    */
/* Initial gains from a simulation (first-order wheel, tau 0.2 s, friction,
 * encoder quantisation): ~2 % overshoot unloaded, none loaded. Tune on the
 * rover (README). */
#define KP_DUTY_PER_MPS 1.0f   /* duty / (m/s)                            */
#define KI_DUTY_PER_M 3.0f     /* duty / m                                */
#define KA_DUTY_PER_MPS2 0.15f /* acceleration feed-forward               */
#define I_MAX_DUTY 0.5f      /* integrator clamp (anti-windup)            */
#define DUTY_MAX 1.0f
#define KFF_DUTY_PER_MPS (1.0f / V_NOLOAD_12V) /* feed-forward at 12 V   */
#define FF_STATIC_DUTY 0.06f /* static-friction offset                    */
#define V_DEADBAND 0.01f     /* |setpoint| below this -> brake            */
#define SPEED_FILTER_BETA 0.35f /* IIR on measured side speed (~37 ms)    */
#define ACCEL_MAX 0.8f       /* m/s^2 setpoint slew, normal                */
#define DECEL_STOP 1.5f      /* m/s^2 setpoint slew on timeout / stop      */
#define NOMINAL_BATT_MV 12000.0f

/* ------------------------------------------------- commands / safety */
#define CMD_TIMEOUT_DEFAULT_MS 500 /* SET_VELOCITY timeout_ms == 0         */
#define DRIVE_DEFAULT_MS 500       /* DRIVE duration_ms == 0               */
#define DRIVE_MAX_MS 5000          /* cap on DRIVE duration (blind radio driving) */
#define ESTOP_DEBOUNCE_MS 2        /* hardware input must read HIGH this long */
#define NACK_RATE_LIMIT_MS 1000    /* SET_VELOCITY NACK while e-stopped    */

/* --------------------------------------------------- encoder fault */
#define ENC_FAULT_DUTY 0.20f  /* "wheel commanded" = |duty| >= this       */
#define ENC_FAULT_MS 1000     /* no ticks for this long -> fault bit3     */

/* ---------------------------------------------------------- battery */
/* V_batt = V_A0 * (100k + 22k) / 22k ; 5 V / 1023 counts -> 27.106 mV/count.
 * Calibrate with a multimeter: BATT_UV_PER_COUNT = 1000 * V_meas / ADC. */
#define BATT_UV_PER_COUNT 27106UL
#define BATT_LOW_MV 10800
#define BATT_OK_MV 11100     /* hysteresis */
#define BATT_FILTER_MS 2000  /* must stay below / above this long         */
#define BATT_SAMPLE_MS 100

/* ------------------------------------------------------- ultrasonics */
#define US_TIMEOUT_US 25000UL /* 25 ms echo timeout -> 0xFFFF           */
#define US_SLOT_MS 25         /* one sensor at a time, fixed slot        */
#define US_MIN_MM 20
#define US_MAX_MM 4000

/* ---------------------------------------------------------- telemetry */
#define ODOM_PERIOD_MS 50      /* WHEEL_ODOM 20 Hz */
#define RANGES_PERIOD_MS 100   /* RANGES 10 Hz     */
#define HEARTBEAT_PERIOD_MS 1000
#define SCAN_PERIOD_MS 50      /* obstacle-avoidance update 20 Hz */

#endif /* MOBILITY_CONFIG_H */
