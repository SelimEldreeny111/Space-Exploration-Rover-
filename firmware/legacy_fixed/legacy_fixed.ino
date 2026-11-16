/*
 * legacy_fixed.ino - the 2023 single-MCU MISSION X rover (Arduino Mega 2560)
 * with the SAME WIRING as "Rover Final Code" (RF.ino / Rover_F1.cpp), but
 * with the defects D1-D12 of ICD v2.0 section 8 fixed and the MissionX_SSP
 * library, so the 2023 LabVIEW ground station keeps working.
 * Board: arduino:avr:mega. Library: DFRobot_DHT11.
 *
 *   L298N  IN1..IN4 = 2,3,4,5 (motor 1 = left side, motor 2 = right side)
 *          ENA = 8, ENB = 9
 *   IR obstacle sensor 53, LED 13 (on while the IR output is LOW)
 *   gas sensor A0, ultrasonic 1 echo/trig 22/23, ultrasonic 2 echo/trig 24/25
 *   DHT11 26, HC-12 on Serial1 9600 baud
 *   SSP: rover 0x01, ground station 0x0B. The same command set is also
 *   accepted on the USB port (Serial, 115200) for bench tests.
 *
 * Commands (ICD 6.1 legacy set + v2 safety commands):
 *   PING -> ACK                GET_TLM -> legacy 7-byte telemetry (ICD 7.1)
 *   CAP_IMAGE, GET_IMAGE -> echo (no camera on this MCU, as in 2023)
 *   START_SCAN / STOP_SCAN -> echo; 2023 behaviour: forward while the front
 *       ultrasonic (1) sees more than 80 cm, otherwise rotate right in place
 *   SET_VELOCITY (watchdog), DRIVE (duration limited), ESTOP, CLEAR_ESTOP
 *   anything else -> NACK(UNKNOWN_TYPE)
 *
 * Fixes:  D1 CRC low byte first (library)   D2 only CRC-valid frames run
 *   D3 streaming length-driven parser with resync, no readBytes()/delay()
 *   D4 replies swap addresses   D5 no malloc, static buffers only
 *   D6 GET_TLM from live sensor data   D7 typed status codes (enum/bool)
 *   D8 16-bit mm ranges, 25 ms echo timeout, non-blocking (polled) echo timing
 *   D9 one telemetry slot per ultrasonic   D10 both sides set explicitly
 *   D11 SET_VELOCITY timeout + ESTOP; the autonomous scan also stops if the
 *       GS has been silent for SCAN_LINK_TIMEOUT_MS   D12 (Python, n/a here)
 */
#include <DFRobot_DHT11.h>
#include <MissionX_SSP.h>

/* ---------------------------------------------------- 2023 wiring */
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
#define ENA 8
#define ENB 9
#define IR_PIN 53
#define LED_PIN 13
#define GAS_PIN A0
#define US1_ECHO 22
#define US1_TRIG 23
#define US2_ECHO 24
#define US2_TRIG 25
#define DHT_PIN 26

/* ---------------------------------------------------------- tuning */
#define OWN_ADDR SSP_ADDR_ROVER
#define RADIO_BAUD 9600UL
#define USB_BAUD 115200UL
#define RADIO_RX_TIMEOUT_MS 50 /* ICD 3 step 6, L1 */
#define USB_RX_TIMEOUT_MS 20
#define SCAN_SPEED 255         /* PWM used by the 2023 code (controll_speed) */
#define SCAN_THRESHOLD_MM 800  /* 2023: TLM[3] > 80 cm -> forward */
#define SCAN_PERIOD_MS 50
#define SCAN_LINK_TIMEOUT_MS 60000UL /* 0 disables (D11 for the scan) */
#define CMD_TIMEOUT_DEFAULT_MS 500
#define DRIVE_DEFAULT_MS 500
#define DRIVE_MAX_MS 5000
#define V_FULL_PWM 0.87f       /* m/s at PWM 255 (approx., open loop) */
#define B_EFF_M 0.87f          /* effective track for SET_VELOCITY mixing */
#define MAX_SPEED 0.45f
#define MAX_YAW_RATE 1.0f
#define US_TIMEOUT_US 25000UL
#define US_SLOT_MS 30
#define US_MAX_POLL_GAP_US 400UL /* loop stalled during a pulse -> discard */
#define DHT_PERIOD_MS 5000UL
#define DHT_RETRY_MS 30000UL
#define DHT_MAX_STALE_MS 30000UL /* while driving, read at most this often */

static SspLink g_radio(Serial1, OWN_ADDR);
static SspLink g_usb(Serial, OWN_ADDR);
static DFRobot_DHT11 g_dht;

/* ------------------------------------------------------- motors */
enum MotionSource : uint8_t { MOT_NONE, MOT_SCAN, MOT_VELOCITY, MOT_DRIVE };
static MotionSource g_src = MOT_NONE;
static bool g_estop = false;
static uint32_t g_cmd_ms = 0;
static uint16_t g_cmd_window = 0;
static int16_t g_pwm_l = 0, g_pwm_r = 0;
static uint32_t g_last_gs_ms = 0;
static uint32_t g_sv_nack_ms = 0;
static bool g_sv_nack_sent = false;

/* signed PWM per side, -255..255; BOTH sides are always written (D10) */
static void setSides(int16_t left, int16_t right) {
  if (g_estop) left = right = 0;
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  g_pwm_l = left;
  g_pwm_r = right;
  digitalWrite(IN1, left > 0 ? HIGH : LOW);
  digitalWrite(IN2, left < 0 ? HIGH : LOW);
  digitalWrite(IN3, right > 0 ? HIGH : LOW);
  digitalWrite(IN4, right < 0 ? HIGH : LOW);
  analogWrite(ENA, (uint8_t)abs(left));
  analogWrite(ENB, (uint8_t)abs(right));
}

static void motorsStop() {
  g_src = MOT_NONE;
  setSides(0, 0);
}

static int16_t speedToPwm(float v) {
  float p = v / V_FULL_PWM * 255.0f;
  if (p > 255.0f) p = 255.0f;
  if (p < -255.0f) p = -255.0f;
  return (int16_t)(p >= 0.0f ? p + 0.5f : p - 0.5f);
}

static void setTwistOpenLoop(float v, float w) {
  v = constrain(v, -MAX_SPEED, MAX_SPEED);
  w = constrain(w, -MAX_YAW_RATE, MAX_YAW_RATE);
  setSides(speedToPwm(v - 0.5f * w * B_EFF_M), speedToPwm(v + 0.5f * w * B_EFF_M));
}

/* ------------------------------------------------ ultrasonics (D8, D9)
 * Echo pins 22/24 are on port A, which has no pin-change interrupt, so the
 * echo is timed by polling from loop() (one pass takes ~20-50 us). If loop()
 * was stalled while a pulse was being timed, the reading is discarded. */
struct Ultrasonic {
  uint8_t trig, echo;
  uint16_t mm; /* 0xFFFF = no echo */
};
static Ultrasonic g_us[2] = {{US1_TRIG, US1_ECHO, 0xFFFF}, {US2_TRIG, US2_ECHO, 0xFFFF}};
enum UsState : uint8_t { US_IDLE, US_WAIT_RISE, US_WAIT_FALL };
static UsState g_us_state = US_IDLE;
static uint8_t g_us_k = 1;
static uint32_t g_us_trig_us = 0, g_us_rise_us = 0, g_us_last_poll = 0, g_us_slot_ms = 0;
static bool g_us_rise_late = false;

static void usFinish(uint16_t mm, bool valid) {
  if (valid) g_us[g_us_k].mm = mm;
  g_us_state = US_IDLE;
}

static void usUpdate() {
  /* an edge is time-stamped when it is seen: its error is at most the gap
   * since the previous poll, so a reading is kept only if neither edge was
   * detected after a long gap */
  const uint32_t t = micros();
  const bool late = (uint32_t)(t - g_us_last_poll) > US_MAX_POLL_GAP_US;
  g_us_last_poll = t;
  switch (g_us_state) {
  case US_IDLE:
    if ((uint32_t)(millis() - g_us_slot_ms) < US_SLOT_MS) return;
    g_us_slot_ms = millis();
    g_us_k ^= 1; /* alternate between the two sensors */
    digitalWrite(g_us[g_us_k].trig, HIGH);
    delayMicroseconds(10); /* 10 us trigger pulse */
    digitalWrite(g_us[g_us_k].trig, LOW);
    g_us_trig_us = micros();
    g_us_last_poll = g_us_trig_us;
    g_us_state = US_WAIT_RISE;
    return;
  case US_WAIT_RISE:
    if (digitalRead(g_us[g_us_k].echo) == HIGH) {
      g_us_rise_us = t;
      g_us_rise_late = late;
      g_us_state = US_WAIT_FALL;
    } else if ((uint32_t)(t - g_us_trig_us) > US_TIMEOUT_US) {
      usFinish(0xFFFF, true);
    }
    return;
  case US_WAIT_FALL:
    if (digitalRead(g_us[g_us_k].echo) == LOW) {
      const uint32_t mm = ((t - g_us_rise_us) * 343UL + 1000UL) / 2000UL;
      usFinish(mm > 4000 ? 0xFFFF : (uint16_t)(mm < 20 ? 20 : mm), !g_us_rise_late && !late);
    } else if ((uint32_t)(t - g_us_trig_us) > US_TIMEOUT_US) {
      usFinish(0xFFFF, true);
    }
    return;
  }
}

static uint8_t mmToCm8(uint16_t mm) {
  if (mm == 0xFFFF) return 255; /* no echo -> "far" (2023 sent 0) */
  const uint16_t cm = (uint16_t)((mm + 5) / 10);
  return cm > 255 ? 255 : (uint8_t)cm;
}

/* ------------------------------------------------------------ DHT11 */
static bool g_dht_ok = false;
static uint8_t g_temp_c = 0, g_rh = 0;
static uint32_t g_dht_next = 2000, g_dht_try_ms = 0;

static void dhtUpdate(uint32_t now) {
  if ((int32_t)(now - g_dht_next) < 0) return;
  /* DFRobot_DHT11::read() blocks ~225 ms (up to ~2 s without a sensor):
   * while the motors run, read at most every DHT_MAX_STALE_MS */
  const bool driving = g_pwm_l != 0 || g_pwm_r != 0;
  if (driving && (uint32_t)(now - g_dht_try_ms) < DHT_MAX_STALE_MS) return;
  g_dht_try_ms = now;
  g_radio.poll();
  g_usb.poll();
  const uint32_t t0 = millis();
  g_dht.read(DHT_PIN);
  const bool valid = (millis() - t0) < 600 && g_dht.humidity >= 1 && g_dht.humidity <= 100 &&
                     g_dht.temperature >= 0 && g_dht.temperature <= 60;
  if (valid) {
    g_temp_c = (uint8_t)g_dht.temperature;
    g_rh = (uint8_t)g_dht.humidity;
  }
  g_dht_ok = valid;
  g_dht_next = millis() + (valid ? DHT_PERIOD_MS : DHT_RETRY_MS);
}

/* ------------------------------------------------- telemetry (D6) */
static void sendLegacyTlm(SspLink &link, const SspFrame &req) {
  const uint16_t gas = (uint16_t)analogRead(GAS_PIN);
  SspTlmLegacy t;
  t.ir = (uint8_t)digitalRead(IR_PIN);
  t.gas_hi = (uint8_t)(gas >> 8);
  t.gas_lo = (uint8_t)(gas & 0xFF);
  t.us1_cm = mmToCm8(g_us[0].mm);
  t.us2_cm = mmToCm8(g_us[1].mm); /* own slot (D9) */
  t.temp_c = g_dht_ok ? g_temp_c : 0;
  t.rh_pct = g_dht_ok ? g_rh : 0;
  uint8_t buf[SSP_LEN_TLM_LEGACY];
  link.reply(req, SSP_GET_TLM, buf, ssp_pack_tlm_legacy(buf, t)); /* swapped addresses (D4) */
}

/* -------------------------------------------- command handler (D2, D7) */
static void onFrame(const SspFrame &f, void *ctx) {
  SspLink &link = *static_cast<SspLink *>(ctx);
  const uint32_t now = millis();
  const bool bcast = f.dest == SSP_ADDR_BROADCAST;
  g_last_gs_ms = now;
  switch (f.type) {
  case SSP_PING:
    link.ack(f, false);
    return;
  case SSP_GET_TLM:
    if (f.len != 0) break;
    sendLegacyTlm(link, f);
    return;
  case SSP_CAP_IMAGE:
  case SSP_GET_IMAGE:
    link.echo(f); /* no camera on the legacy MCU (2023 behaviour) */
    return;
  case SSP_START_SCAN:
    if (f.len != 0) break;
    if (g_estop) {
      link.nack(f, SSP_NACK_NOT_PERMITTED);
      return;
    }
    g_src = MOT_SCAN;
    link.echo(f);
    return;
  case SSP_STOP_SCAN:
    if (f.len != 0) break;
    motorsStop();
    link.echo(f);
    return;
  case SSP_SET_VELOCITY: {
    SspSetVelocity sv;
    if (!ssp_unpack_set_velocity(f.data, f.len, &sv)) break;
    if (g_estop) { /* SET_VELOCITY has no response: NACK at most once per second */
      if (!g_sv_nack_sent || (uint32_t)(now - g_sv_nack_ms) >= 1000) {
        link.nack(f, SSP_NACK_NOT_PERMITTED);
        g_sv_nack_ms = now;
        g_sv_nack_sent = true;
      }
      return;
    }
    g_src = MOT_VELOCITY;
    g_cmd_ms = now;
    g_cmd_window = sv.timeout_ms ? sv.timeout_ms : CMD_TIMEOUT_DEFAULT_MS;
    setTwistOpenLoop(sv.v_mm_s * 0.001f, sv.w_mrad_s * 0.001f);
    return;
  }
  case SSP_DRIVE: {
    SspDrive d;
    if (!ssp_unpack_drive(f.data, f.len, &d)) break;
    if (g_estop) {
      link.nack(f, SSP_NACK_NOT_PERMITTED);
      return;
    }
    g_src = MOT_DRIVE;
    g_cmd_ms = now;
    g_cmd_window = d.duration_ms ? (d.duration_ms > DRIVE_MAX_MS ? DRIVE_MAX_MS : d.duration_ms)
                                 : DRIVE_DEFAULT_MS;
    setTwistOpenLoop(constrain(d.v_pct, -100, 100) * 0.01f * MAX_SPEED,
                     constrain(d.w_pct, -100, 100) * 0.01f * MAX_YAW_RATE);
    link.ack(f, true);
    return;
  }
  case SSP_ESTOP:
    if (f.len != 0) break;
    motorsStop();
    g_estop = true;
    link.ack(f, true);
    return;
  case SSP_CLEAR_ESTOP:
    if (f.len != 0) break;
    g_estop = false;
    g_sv_nack_sent = false;
    link.ack(f, true);
    return;
  default:
    if (ssp_is_report_type(f.type) || bcast) return;
    link.nack(f, SSP_NACK_UNKNOWN_TYPE);
    return;
  }
  if (!bcast) link.nack(f, SSP_NACK_BAD_LENGTH);
}

/* ------------------------------------------------- motion supervisor */
static uint32_t g_scan_next = 0;

static void motionUpdate(uint32_t now) {
  if (g_estop) return;
  if ((g_src == MOT_VELOCITY || g_src == MOT_DRIVE) && (uint32_t)(now - g_cmd_ms) > g_cmd_window) {
    motorsStop(); /* D11: command watchdog / DRIVE duration */
  }
  if (g_src == MOT_SCAN) {
#if SCAN_LINK_TIMEOUT_MS > 0
    if ((uint32_t)(now - g_last_gs_ms) > SCAN_LINK_TIMEOUT_MS) {
      motorsStop(); /* ground station silent: do not wander off */
      return;
    }
#endif
    if ((int32_t)(now - g_scan_next) < 0) return;
    g_scan_next = now + SCAN_PERIOD_MS;
    const uint16_t front = g_us[0].mm;
    if (front == 0xFFFF || front > SCAN_THRESHOLD_MM) {
      setSides(SCAN_SPEED, SCAN_SPEED); /* forward */
    } else {
      setSides(SCAN_SPEED, -SCAN_SPEED); /* rotate right in place (D10) */
    }
  }
}

/* ------------------------------------------------------ setup / loop */
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  setSides(0, 0);
  pinMode(IR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(GAS_PIN, INPUT);
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(g_us[i].echo, INPUT);
    pinMode(g_us[i].trig, OUTPUT);
    digitalWrite(g_us[i].trig, LOW);
  }
  Serial.begin(USB_BAUD);
  Serial1.begin(RADIO_BAUD);
  g_radio.begin(onFrame, &g_radio, RADIO_RX_TIMEOUT_MS);
  g_usb.begin(onFrame, &g_usb, USB_RX_TIMEOUT_MS);
}

void loop() {
  const uint32_t now = millis();
  g_radio.poll();
  g_usb.poll();
  usUpdate();
  motionUpdate(now);
  digitalWrite(LED_PIN, digitalRead(IR_PIN) == LOW ? HIGH : LOW); /* 2023 IR indicator */
  dhtUpdate(now);
}
