/*
 * mobility_mega.ino - MISSION X Mobility MCU (Arduino Mega 2560, SSP 0x02).
 * ICD v2.0. Board: arduino:avr:mega (ATmega2560).
 *
 * Links
 *   Serial  115200  L2  Brain (Raspberry Pi), USB CDC: SET_VELOCITY in,
 *                       WHEEL_ODOM 20 Hz, RANGES 10 Hz, HEARTBEAT 1 Hz out.
 *   Serial2   9600  L1-safe  HC-12 radio wired to the Mega when the Brain is
 *                       absent ("safe mode"): replies only (PING, GET_TLM v2,
 *                       DRIVE, START_SCAN, STOP_SCAN, ESTOP, CLEAR_ESTOP ...).
 *                       Accepts DEST 0x02, 0xFF and 0x01 (legacy LabVIEW GS);
 *                       frames to 0x01 are answered with SRC 0x01 and a
 *                       GET_TLM to 0x01 gets the legacy 7-byte layout.
 * Both links accept the same command set; every reply goes back on the link
 * the request came from with swapped addresses (ICD 4).
 *
 * Main loop: everything is non-blocking (no delay(), no pulseIn()); one pass
 * takes well under 1 ms, so the 64-byte UART RX buffers (5.6 ms at 115200
 * baud) never overflow. Periodic jobs run on fixed time grids with
 * different phases so their frames do not pile up in the 64-byte TX buffer.
 */
#include <MissionX_SSP.h>

#include "battery.h"
#include "config.h"
#include "drive.h"
#include "encoders.h"
#include "legacy_tlm.h"
#include "motors.h"
#include "scan_logic.h"
#include "ultrasonic.h"

#ifndef ESTOP_INPUT_ENABLED
#define ESTOP_INPUT_ENABLED 1 /* 0 only for bench tests without the switch */
#endif

static SspLink g_brain(Serial, SSP_ADDR_MOBILITY);
static SspLink g_safe(Serial2, SSP_ADDR_MOBILITY);
static ScanLogic g_scan;

static bool g_estop = false;      /* latched e-stop (command or input)   */
static bool g_hw_estop = false;   /* debounced state of the input pin    */
static bool g_hw_hi = false;
static uint32_t g_hw_hi_since = 0;
static uint32_t g_last_sv_nack_ms = 0;
static bool g_sv_nack_sent = false;

/* ----------------------------------------------------------- status */
static bool safeLinkActive(uint32_t now) {
  return g_safe.everReceived() && (uint32_t)(now - g_safe.lastRxMs()) < SAFE_LINK_ACTIVE_MS;
}

/* Mode of this node: AUTONOMOUS while START_SCAN runs, TELEOP while a
 * SET_VELOCITY / DRIVE command is active, otherwise IDLE (ICD 6.1: STOP_SCAN
 * -> IDLE). Safe mode (radio on Serial2) is reported by fault bit4 only. */
static uint8_t currentMode() {
  if (g_scan.active()) return SSP_MODE_AUTONOMOUS;
  const drive::Source src = drive::source();
  if (src == drive::SRC_VELOCITY || src == drive::SRC_DRIVE) return SSP_MODE_TELEOP;
  return SSP_MODE_IDLE;
}

static uint8_t faultFlags(uint32_t now) {
  uint8_t f = 0;
  if (g_estop) f |= SSP_FAULT_ESTOP;
  if (drive::timeoutFault()) f |= SSP_FAULT_CMD_TIMEOUT;
  if (battery::low()) f |= SSP_FAULT_LOW_BATTERY;
  if (drive::encoderFault()) f |= SSP_FAULT_ENCODER;
  if (safeLinkActive(now)) f |= SSP_FAULT_SAFE_MODE;
  return f;
}

static int16_t radToCdeg(float rad) {
  float cd = rad * (18000.0f / 3.14159265f);
  if (cd > 32767.0f) cd = 32767.0f;
  if (cd < -32768.0f) cd = -32768.0f;
  return (int16_t)(cd >= 0.0f ? cd + 0.5f : cd - 0.5f);
}

static int32_t mToMm(float m) {
  const float mm = m * 1000.0f;
  if (mm > 2.0e9f) return 2000000000L;
  if (mm < -2.0e9f) return -2000000000L;
  return (int32_t)(mm >= 0.0f ? mm + 0.5f : mm - 0.5f);
}

/* --------------------------------------------------------- e-stop */
static void latchEstop() {
  g_estop = true;
  g_scan.stop();
  drive::setEnabled(false); /* PWM = 0 on all six channels right now */
}

static void pollEstopInput(uint32_t now) {
#if ESTOP_INPUT_ENABLED
  /* NC switch to GND with pull-up: HIGH = pressed or wire broken (fail-safe).
   * A 2 ms filter rejects motor-noise spikes; the latch happens at once. */
  const bool hi = digitalRead(ESTOP_PIN) == HIGH;
  if (hi) {
    if (!g_hw_hi) {
      g_hw_hi = true;
      g_hw_hi_since = now;
    }
    if ((uint32_t)(now - g_hw_hi_since) >= ESTOP_DEBOUNCE_MS) {
      g_hw_estop = true;
      if (!g_estop) latchEstop();
    }
  } else {
    g_hw_hi = false;
    g_hw_estop = false; /* the latch stays until CLEAR_ESTOP */
  }
#else
  (void)now;
#endif
}

/* ------------------------------------------------------ telemetry */
static void sendTlmV2(SspLink &link, const SspFrame &req, uint32_t now) {
  SspTlmV2 t;
  memset(&t, 0, sizeof t); /* fields the Mega does not have stay 0 (ICD 7.1) */
  t.tlm_version = 2;
  t.mode = currentMode();
  t.fault_flags = faultFlags(now);
  t.battery_mV = battery::millivolts();
  t.uptime_s = now / 1000UL;
  const Pose2D &p = drive::pose(); /* wheel-odometry dead reckoning since boot */
  t.yaw_cdeg = radToCdeg(p.theta);
  t.x_mm = mToMm(p.x);
  t.y_mm = mToMm(p.y);
  t.heading_cdeg = t.yaw_cdeg;
  t.range_min_mm = ultrasonic::minRange();
  uint8_t buf[SSP_LEN_TLM_V2];
  const uint8_t n = ssp_pack_tlm_v2(buf, t);
  link.reply(req, SSP_GET_TLM, buf, n);
}

/* Legacy 7-byte TLM (ICD 7.1) for the 2023 LabVIEW GS, which calls the rover
 * 0x01 over the HC-12: ultrasonic ranges only, other fields 0. */
static void sendTlmLegacy(SspLink &link, const SspFrame &req) {
  uint16_t r[NUM_US];
  ultrasonic::ranges(r);
  SspTlmLegacy t;
  legacy_tlm_from_ranges(r, &t);
  uint8_t buf[SSP_LEN_TLM_LEGACY];
  link.reply(req, SSP_GET_TLM, buf, ssp_pack_tlm_legacy(buf, t)); /* SRC = 0x01 */
}

static void sendOdom(uint32_t now) {
  SspWheelOdom o;
  o.t_ms = now;
  encoders::read(o.ticks);
  uint8_t buf[SSP_LEN_WHEEL_ODOM];
  g_brain.send(SSP_ADDR_ROVER, SSP_WHEEL_ODOM, buf, ssp_pack_wheel_odom(buf, o), true);
}

static void sendRanges() {
  SspRanges r;
  r.n = NUM_US;
  ultrasonic::ranges(r.range_mm);
  uint8_t buf[SSP_LEN_RANGES(NUM_US)];
  g_brain.send(SSP_ADDR_ROVER, SSP_RANGES, buf, ssp_pack_ranges(buf, r), true);
}

static void sendHeartbeat(uint32_t now) {
  SspHeartbeat h;
  h.uptime_ms = now;
  h.mode = currentMode();
  h.fault_flags = faultFlags(now);
  uint8_t buf[SSP_LEN_HEARTBEAT];
  g_brain.send(SSP_ADDR_ROVER, SSP_HEARTBEAT, buf, ssp_pack_heartbeat(buf, h), true);
}

/* Fixed-rate scheduler slot: true once per period, re-aligned after stalls. */
struct Every {
  uint32_t next;
  uint16_t period;
  bool due(uint32_t now) {
    if ((int32_t)(now - next) < 0) return false;
    next += period;
    if ((int32_t)(now - next) >= 0) next = now + period; /* do not burst */
    return true;
  }
};

static Every g_odom_t = {0, ODOM_PERIOD_MS};
static Every g_ranges_t = {25, RANGES_PERIOD_MS};
static Every g_hb_t = {10, HEARTBEAT_PERIOD_MS};
static Every g_scan_t = {5, SCAN_PERIOD_MS};

/* ------------------------------------------------------ commands */
static bool motionRefused(SspLink &link, const SspFrame &f) {
  if (!g_estop) return false;
  link.nack(f, SSP_NACK_NOT_PERMITTED); /* e-stop latched */
  return true;
}

static void onFrame(const SspFrame &f, void *ctx) {
  SspLink &link = *static_cast<SspLink *>(ctx);
  const uint32_t now = millis();
  const bool bcast = f.dest == SSP_ADDR_BROADCAST;

  switch (f.type) {
  case SSP_PING:
    link.ack(f, false); /* ACK, LEN 0 (ICD 6.1) */
    return;

  case SSP_GET_TLM:
    if (f.len != 0) break; /* bad length -> NACK below */
    if (&link == &g_safe && f.dest == SSP_ADDR_ROVER)
      sendTlmLegacy(link, f); /* LabVIEW GS: 7-byte layout */
    else
      sendTlmV2(link, f, now); /* to 0x02 / 0xFF: v2 56-byte layout */
    return;

  case SSP_SET_VELOCITY: {
    SspSetVelocity sv;
    if (!ssp_unpack_set_velocity(f.data, f.len, &sv)) break;
    if (g_estop) {
      /* SET_VELOCITY has no response; while e-stopped a NACK is sent at most
       * once per second so a 20 Hz command stream does not flood the link */
      if (!g_sv_nack_sent || (uint32_t)(now - g_last_sv_nack_ms) >= NACK_RATE_LIMIT_MS) {
        link.nack(f, SSP_NACK_NOT_PERMITTED);
        g_last_sv_nack_ms = now;
        g_sv_nack_sent = true;
      }
      return;
    }
    g_scan.stop();
    drive::setVelocity(sv.v_mm_s * 0.001f, sv.w_mrad_s * 0.001f, sv.timeout_ms, now);
    return;
  }

  case SSP_DRIVE: {
    SspDrive d;
    if (!ssp_unpack_drive(f.data, f.len, &d)) break;
    if (motionRefused(link, f)) return;
    const int8_t vp = d.v_pct > 100 ? 100 : (d.v_pct < -100 ? -100 : d.v_pct);
    const int8_t wp = d.w_pct > 100 ? 100 : (d.w_pct < -100 ? -100 : d.w_pct);
    g_scan.stop();
    drive::driveOpenLoop(vp * 0.01f * MAX_SPEED, wp * 0.01f * MAX_YAW_RATE, d.duration_ms, now);
    link.ack(f, true); /* ACK(0x60) */
    return;
  }

  case SSP_START_SCAN:
    if (f.len != 0) break;
    if (motionRefused(link, f)) return;
    drive::clearTimeoutFault();
    g_scan.start(now);
    link.echo(f);
    return;

  case SSP_STOP_SCAN:
    if (f.len != 0) break;
    g_scan.stop();
    drive::stop();
    drive::clearTimeoutFault();
    link.echo(f);
    return;

  case SSP_ESTOP:
    if (f.len != 0) break;
    latchEstop();
    link.ack(f, true); /* ACK(0x51) */
    return;

  case SSP_CLEAR_ESTOP:
    if (f.len != 0) break;
    if (g_hw_estop) {
      link.nack(f, SSP_NACK_NOT_PERMITTED); /* hardware switch still pressed */
      return;
    }
    g_estop = false;
    g_sv_nack_sent = false;
    drive::clearTimeoutFault();
    drive::setEnabled(true);
    link.ack(f, true); /* ACK(0x52) */
    return;

  default:
    if (ssp_is_report_type(f.type)) return; /* never answer answers/reports */
    if (!bcast) link.nack(f, SSP_NACK_UNKNOWN_TYPE);
    return;
  }
  /* a known command with a LEN this node cannot use */
  if (!bcast) link.nack(f, SSP_NACK_BAD_LENGTH);
}

/* ------------------------------------------------------------ LED */
static void updateLed(uint32_t now, uint8_t faults) {
  bool on;
  if (faults & SSP_FAULT_ESTOP) on = true;                     /* solid      */
  else if (faults & (uint8_t)~SSP_FAULT_SAFE_MODE) on = (now / 125) & 1; /* 4 Hz  */
  else on = (now % 1000) < 50;                                  /* heartbeat */
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

/* ---------------------------------------------------- setup / loop */
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(HC12_SET_PIN, OUTPUT);
  digitalWrite(HC12_SET_PIN, HIGH); /* HC-12 transparent mode */

  motors::begin(); /* outputs low before anything else */
  encoders::begin();
  ultrasonic::begin();
  battery::begin();
  drive::begin();

  Serial.begin(BRAIN_BAUD);
  Serial2.begin(SAFE_BAUD);
  g_brain.begin(onFrame, &g_brain, BRAIN_RX_TIMEOUT_MS); /* + ICD 3 step 5a check */
  g_safe.begin(onFrame, &g_safe, SAFE_RX_TIMEOUT_MS);
  /* The legacy LabVIEW GS always addresses "the rover" as 0x01: on the HC-12
   * link accept DEST 0x01 too and answer those frames with SRC 0x01 (frames
   * to 0x02 / 0xFF are answered with SRC 0x02). The USB link stays 0x02. */
  g_safe.setAlias(SSP_ADDR_ROVER);

  const uint32_t now = millis();
  g_odom_t.next += now;
  g_ranges_t.next += now;
  g_hb_t.next += now;
  g_scan_t.next += now;
}

void loop() {
  const uint32_t now = millis();

  g_brain.poll();
  g_safe.poll();
  pollEstopInput(now);
  ultrasonic::update();
  battery::update(now);

  if (g_scan.active() && g_scan_t.due(now)) {
    uint16_t r[NUM_US];
    ultrasonic::ranges(r);
    float v, w;
    g_scan.update(r, now, &v, &w);
    if (!g_estop) drive::setScanTwist(v, w);
  }

  drive::update(now);

  if (g_odom_t.due(now)) sendOdom(now);
  if (g_ranges_t.due(now)) sendRanges();
  if (g_hb_t.due(now)) sendHeartbeat(now);

  updateLed(now, faultFlags(now));
}
