/*
 * motors.cpp - MDD20A sign-magnitude outputs.
 *
 * PWM frequency: the PWM pins 5 (Timer3), 6/7/8 (Timer4) and 11/12 (Timer1)
 * default to ~490 Hz phase-correct PWM, which gives large current ripple and
 * heating in the motors. The prescaler of these three timers is changed
 * from 64 to 8 -> 3.92 kHz (MDD20A accepts up to 20 kHz). Timer0 (millis,
 * micros, delay) is NOT touched. Do not use Servo/tone on this board.
 */
#include <Arduino.h>

#include "config.h"
#include "motors.h"

namespace {
float s_duty[NUM_WHEELS];
}

namespace motors {

void begin() {
  for (uint8_t i = 0; i < NUM_WHEELS; i++) {
    digitalWrite(MOTOR_PWM_PIN[i], LOW);
    pinMode(MOTOR_PWM_PIN[i], OUTPUT);
    pinMode(MOTOR_DIR_PIN[i], OUTPUT);
    digitalWrite(MOTOR_DIR_PIN[i], LOW);
    s_duty[i] = 0.0f;
  }
#if defined(__AVR_ATmega2560__)
  TCCR1B = (uint8_t)((TCCR1B & 0xF8u) | 0x02u); /* clk/8 -> 3.92 kHz on pins 11, 12   */
  TCCR3B = (uint8_t)((TCCR3B & 0xF8u) | 0x02u); /* pin 5                            */
  TCCR4B = (uint8_t)((TCCR4B & 0xF8u) | 0x02u); /* pins 6, 7, 8                     */
#endif
}

void set(uint8_t wheel, float duty) {
  if (wheel >= NUM_WHEELS) return;
  if (duty > 1.0f) duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;
  s_duty[wheel] = duty;
  const float d = (MOTOR_SIGN[wheel] > 0) ? duty : -duty;
  const uint8_t pwm = (uint8_t)(fabs(d) * 255.0f + 0.5f);
  digitalWrite(MOTOR_DIR_PIN[wheel], d >= 0.0f ? HIGH : LOW);
  analogWrite(MOTOR_PWM_PIN[wheel], pwm);
}

void stopAll() {
  for (uint8_t i = 0; i < NUM_WHEELS; i++) {
    analogWrite(MOTOR_PWM_PIN[i], 0);
    s_duty[i] = 0.0f;
  }
}

float duty(uint8_t wheel) { return wheel < NUM_WHEELS ? s_duty[wheel] : 0.0f; }

} // namespace motors
