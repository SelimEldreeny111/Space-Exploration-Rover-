/*
 * arm_uno.ino - MISSION X Arm MCU (Arduino Uno + PCA9685, SSP address 0x04).
 * ICD v2.0. Board: arduino:avr:uno. Library: Adafruit PWM Servo Driver.
 *
 * Serial 115200 (L4, USB CDC to the Brain):
 *   ARM_SET_JOINTS (0x40) -> ACK(0x40), or NACK(0x40, 4) if a joint was
 *                            outside its limits: the target is CLAMPED to the
 *                            limits and the clamped motion IS executed; the
 *                            NACK tells the Brain it did not get exactly what
 *                            it asked for (ARM_STATE shows the result).
 *   ARM_SET_POSE   (0x41) -> ACK(0x41); NACK(0x41, 4) for an unknown pose id
 *   COLLECT_OBJECT (0x20) -> echo; READY, GRIP_OPEN, REACH_GROUND, GRIP_CLOSE,
 *                            LIFT_SAMPLE, DEPOSIT (move, then open), STOW.
 *                            NACK(0x20, 3) if a sequence is already running.
 *   ESTOP / CLEAR_ESTOP    -> freeze in place (servos keep holding) / resume.
 *   PING -> ACK.     out: ARM_STATE 5 Hz + on change, HEARTBEAT 1 Hz.
 * A new ARM_SET_JOINTS / ARM_SET_POSE aborts a running sequence (operator
 * override). Motion: minimum-jerk interpolation from the current commanded
 * angles, sampled every 20 ms (servo frame); durations are stretched so the
 * peak joint speed stays below MAX_JOINT_SPEED_DPS. NACK(5) while the
 * PCA9685 does not answer on I2C.
 */
#include <Adafruit_PWMServoDriver.h>
#include <MissionX_SSP.h>
#include <Wire.h>

#include "arm_math.h"
#include "config.h"

/* HEARTBEAT.fault_flags of the Arm node: bit0 e-stop latched (as Mobility),
 * bit1 PCA9685 not answering on I2C. */
#define ARM_FAULT_ESTOP 0x01
#define ARM_FAULT_PCA 0x02

static SspLink g_link(Serial, SSP_ADDR_ARM);
static Adafruit_PWMServoDriver g_pwm(PCA9685_ADDR);

static bool g_pca_ok = false;
static uint32_t g_pca_retry = 0;
static uint16_t g_sent[ARM_NJ]; /* counts last written, 0 = unknown */

static float g_cur[ARM_NJ];  /* commanded angles now (deg)            */
static float g_from[ARM_NJ]; /* start of the current motion           */
static uint8_t g_to[ARM_NJ]; /* target of the current motion          */
static uint16_t g_T = 0;
static uint32_t g_t0 = 0;
static bool g_moving = false;
static uint32_t g_motion_end = 0;
static bool g_estop = false;
static bool g_state_dirty = true; /* send ARM_STATE now ("on change") */

/* sequence engine (COLLECT_OBJECT, multi-phase poses) */
static const uint8_t kCollectSeq[] = {SSP_POSE_READY,       SSP_POSE_GRIP_OPEN,
                                      SSP_POSE_REACH_GROUND, SSP_POSE_GRIP_CLOSE,
                                      SSP_POSE_LIFT_SAMPLE,  SSP_POSE_DEPOSIT,
                                      SSP_POSE_STOW};
static uint8_t g_single_pose = 0;
static const uint8_t *g_seq = NULL;
static uint8_t g_seq_len = 0, g_seq_idx = 0, g_seq_sub = 0;
static bool g_seq_collect = false;

/* --------------------------------------------------------- servo out */
static bool pcaInit() {
  if (!g_pwm.begin()) return false; /* Adafruit: probes the device first */
  g_pwm.setOscillatorFrequency(PCA_OSC_HZ);
  g_pwm.setPWMFreq(SERVO_FREQ_HZ);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(WIRE_TIMEOUT_US, true); /* begin() re-ran Wire.begin() */
  for (uint8_t j = 0; j < ARM_NJ; j++) g_sent[j] = 0;
  return true;
}

static void writeServos() {
  if (!g_pca_ok) return;
  for (uint8_t j = 0; j < ARM_NJ; j++) {
    const uint16_t c = arm_deg_to_counts(g_cur[j]);
    if (c == g_sent[j]) continue; /* only changed channels: ~0.2 ms each */
    if (g_pwm.setPWM(j, 0, c) != 0) {
      g_pca_ok = false;
      g_pca_retry = millis() + PCA_RETRY_MS;
      Wire.clearWireTimeoutFlag();
      return;
    }
    g_sent[j] = c;
  }
}

/* ------------------------------------------------------------ motion */
static void startMotion(const uint8_t to[ARM_NJ], uint16_t duration_ms, uint32_t now) {
  bool differs = false;
  for (uint8_t j = 0; j < ARM_NJ; j++) {
    arm_clamp(j, to[j], &g_to[j]);
    g_from[j] = g_cur[j];
    if (fabs((float)g_to[j] - g_cur[j]) > 0.01f) differs = true;
  }
  g_T = arm_motion_duration(g_from, g_to, duration_ms, MAX_JOINT_SPEED_DPS);
  g_t0 = now;
  g_moving = differs;
  if (!differs) g_motion_end = now;
  g_state_dirty = true;
}

static void moveToPose(uint8_t pose, uint32_t now) {
  uint8_t to[ARM_NJ];
  for (uint8_t j = 0; j < ARM_NJ; j++) {
    const uint8_t d = kPoses[pose].deg[j];
    to[j] = d == POSE_KEEP ? (uint8_t)(g_cur[j] + 0.5f) : d;
  }
  startMotion(to, kPoses[pose].duration_ms, now);
}

static void abortSequence() {
  g_seq = NULL;
  g_seq_len = g_seq_idx = g_seq_sub = 0;
  g_seq_collect = false;
}

static void startSequence(const uint8_t *steps, uint8_t n, bool collect, uint32_t now) {
  g_seq = steps;
  g_seq_len = n;
  g_seq_idx = 0;
  g_seq_sub = 0;
  g_seq_collect = collect;
  g_moving = false;                   /* pre-empt: start from the current angles */
  g_motion_end = now - POSE_DWELL_MS; /* first step without dwell */
}

static void sequenceUpdate(uint32_t now) {
  if (g_seq == NULL || g_moving || g_estop) return;
  if ((uint32_t)(now - g_motion_end) < POSE_DWELL_MS) return;
  if (g_seq_idx >= g_seq_len) {
    abortSequence(); /* finished */
    g_state_dirty = true;
    return;
  }
  const uint8_t pose = g_seq[g_seq_idx];
  if (pose == SSP_POSE_DEPOSIT && g_seq_sub == 0) {
    moveToPose(SSP_POSE_DEPOSIT, now); /* over the box, gripper still closed */
    g_seq_sub = 1;
  } else if (pose == SSP_POSE_DEPOSIT) {
    moveToPose(SSP_POSE_GRIP_OPEN, now); /* release the sample */
    g_seq_sub = 0;
    g_seq_idx++;
  } else {
    moveToPose(pose, now);
    g_seq_idx++;
  }
}

static void servoUpdate(uint32_t now) {
  if (g_moving) {
    const uint32_t t = now - g_t0;
    if (t >= g_T) {
      for (uint8_t j = 0; j < ARM_NJ; j++) g_cur[j] = g_to[j];
      g_moving = false;
      g_motion_end = now;
      g_state_dirty = true;
    } else {
      arm_interpolate(g_from, g_to, t, g_T, g_cur);
    }
  }
  writeServos();
}

static void freeze(uint32_t now) {
  abortSequence();
  g_moving = false; /* hold the current commanded angles */
  g_motion_end = now;
  g_state_dirty = true;
}

/* --------------------------------------------------------- telemetry */
static bool busy() { return g_moving || g_seq != NULL; }

static void sendState() {
  SspArmState s;
  for (uint8_t j = 0; j < ARM_NJ; j++) {
    const float d = g_cur[j] + 0.5f;
    s.deg[j] = d < 0.0f ? 0 : (d > 180.0f ? 180 : (uint8_t)d);
  }
  s.moving = busy() ? 1 : 0;
  uint8_t buf[SSP_LEN_ARM_STATE];
  g_link.send(SSP_ADDR_ROVER, SSP_ARM_STATE, buf, ssp_pack_arm_state(buf, s), true);
}

static void sendHeartbeat(uint32_t now) {
  SspHeartbeat h;
  h.uptime_ms = now;
  h.mode = g_seq_collect ? SSP_MODE_AUTONOMOUS : (busy() ? SSP_MODE_TELEOP : SSP_MODE_IDLE);
  h.fault_flags = (uint8_t)((g_estop ? ARM_FAULT_ESTOP : 0) | (g_pca_ok ? 0 : ARM_FAULT_PCA));
  uint8_t buf[SSP_LEN_HEARTBEAT];
  g_link.send(SSP_ADDR_ROVER, SSP_HEARTBEAT, buf, ssp_pack_heartbeat(buf, h), true);
}

/* ---------------------------------------------------------- commands */
static bool motionAllowed(const SspFrame &f) {
  if (g_estop) {
    g_link.nack(f, SSP_NACK_NOT_PERMITTED);
    return false;
  }
  if (!g_pca_ok) {
    g_link.nack(f, SSP_NACK_HW_FAULT);
    return false;
  }
  return true;
}

static void onFrame(const SspFrame &f, void *) {
  const uint32_t now = millis();
  const bool bcast = f.dest == SSP_ADDR_BROADCAST;
  switch (f.type) {
  case SSP_PING:
    g_link.ack(f, false);
    return;

  case SSP_ARM_SET_JOINTS: {
    SspArmSetJoints c;
    if (!ssp_unpack_arm_set_joints(f.data, f.len, &c)) break;
    if (!motionAllowed(f)) return;
    bool clamped = false;
    for (uint8_t j = 0; j < ARM_NJ; j++) clamped |= arm_clamp(j, c.deg[j], &c.deg[j]);
    abortSequence();
    startMotion(c.deg, c.duration_ms, now);
    if (clamped) g_link.nack(f, SSP_NACK_NOT_PERMITTED); /* limits: clamped */
    else g_link.ack(f, true);
    return;
  }

  case SSP_ARM_SET_POSE: {
    if (f.len != 1) break;
    if (!motionAllowed(f)) return;
    const uint8_t pose = f.data[0];
    if (pose >= ARM_NUM_POSES) {
      g_link.nack(f, SSP_NACK_NOT_PERMITTED);
      return;
    }
    abortSequence();
    g_single_pose = pose; /* DEPOSIT runs as two phases through the engine */
    startSequence(&g_single_pose, 1, false, now);
    sequenceUpdate(now);
    g_link.ack(f, true);
    return;
  }

  case SSP_COLLECT_OBJECT:
    if (f.len != 0) break;
    if (!motionAllowed(f)) return;
    if (g_seq_collect) {
      g_link.nack(f, SSP_NACK_BUSY);
      return;
    }
    abortSequence();
    startSequence(kCollectSeq, sizeof kCollectSeq, true, now);
    g_link.echo(f);
    return;

  case SSP_ESTOP:
    if (f.len != 0) break;
    g_estop = true;
    freeze(now);
    g_link.ack(f, true);
    return;

  case SSP_CLEAR_ESTOP:
    if (f.len != 0) break;
    g_estop = false;
    g_link.ack(f, true);
    return;

  default:
    if (ssp_is_report_type(f.type) || bcast) return;
    g_link.nack(f, SSP_NACK_UNKNOWN_TYPE);
    return;
  }
  if (!bcast) g_link.nack(f, SSP_NACK_BAD_LENGTH);
}

/* ------------------------------------------------------ setup / loop */
struct Every {
  uint32_t next;
  uint16_t period;
  bool due(uint32_t now) {
    if ((int32_t)(now - next) < 0) return false;
    next += period;
    if ((int32_t)(now - next) >= 0) next = now + period;
    return true;
  }
};
static Every g_servo_t = {0, SERVO_UPDATE_MS};
static Every g_state_t = {0, STATE_PERIOD_MS};
static Every g_hb_t = {100, HEARTBEAT_PERIOD_MS};

void setup() {
  Serial.begin(LINK_BAUD);
  g_link.begin(onFrame, NULL, RX_TIMEOUT_MS);
  for (uint8_t j = 0; j < ARM_NJ; j++) {
    g_cur[j] = g_from[j] = kJoints[j].home_deg; /* power-up: servos jump to HOME */
    g_to[j] = kJoints[j].home_deg;
  }
  g_pca_ok = pcaInit();
  if (!g_pca_ok) {
    Wire.setWireTimeout(WIRE_TIMEOUT_US, true);
    g_pca_retry = millis() + PCA_RETRY_MS;
  }
  writeServos();
}

void loop() {
  const uint32_t now = millis();
  g_link.poll();
  if (!g_pca_ok && (int32_t)(now - g_pca_retry) >= 0) {
    g_pca_retry = now + PCA_RETRY_MS;
    g_pca_ok = pcaInit();
  }
  sequenceUpdate(now);
  if (g_servo_t.due(now)) servoUpdate(now);
  if (g_state_dirty || g_state_t.due(now)) {
    g_state_dirty = false;
    sendState();
  }
  if (g_hb_t.due(now)) sendHeartbeat(now);
}
