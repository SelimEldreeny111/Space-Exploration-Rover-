// Host tests for the Arduino-free logic shared with the sketches:
//   mobility_mega/drive_math, mobility_mega/scan_logic,
//   payload_uno/bmp180_comp, payload_uno/imu_math, arm_uno/arm_math.
#include <cmath>
#include <cstdio>
#include <cstring>

#include "arm_math.h"
#include "bmp180_comp.h"
#include "drive_math.h"
#include "imu_math.h"
#include "legacy_tlm.h"
#include "scan_logic.h"
#include "ssp_core.h"

static int g_pass = 0, g_fail = 0;
static const char *g_ctx = "";
#define CHECK(cond)                                                                 \
  do {                                                                              \
    if (cond) g_pass++;                                                             \
    else {                                                                          \
      g_fail++;                                                                     \
      std::printf("FAIL %s:%d [%s] %s\n", __FILE__, __LINE__, g_ctx, #cond);        \
    }                                                                               \
  } while (0)
#define NEAR(a, b, tol) (std::fabs((double)(a) - (double)(b)) <= (tol))

// ------------------------------------------------------------ drive_math
// Same values as mobility_mega/config.h
static PiParams piDefaults() {
  PiParams p;
  p.kp = 1.0f;
  p.ki = 3.0f;
  p.i_max = 0.5f;
  p.u_max = 1.0f;
  p.kff = 1.0f / 0.869f;
  p.ka = 0.15f;
  p.u_static = 0.06f;
  p.v_dead = 0.01f;
  return p;
}

// Closed loop as in drive.cpp: slew-limited setpoint (0.8 m/s^2), side speed
// measured from quantised encoder counts (3 wheels, 0.748 mm/count, 20 ms)
// and an IIR filter (beta 0.35). Plant: first-order wheel with Coulomb
// friction, v' = (gain * (u - friction) - v) / tau, gain = speed at full duty.
struct Sim {
  float v = 0, pos = 0, sp = 0, meas = 0;
  long cnt_prev = 0;
};
static float simulate(Sim &s, float target, float gain, float seconds, PiState *st,
                      const PiParams &p, float *max_v = nullptr) {
  const float dt = 0.02f, tau = 0.2f, fric = 0.06f, mpc = 2.0f * 3.14159265f * 0.1f / 840.0f;
  for (int k = 0; k < (int)(seconds / dt + 0.5f); k++) {
    const float prev = s.sp;
    s.sp = slew_toward(s.sp, target, 0.8f * dt);
    const float rate = (s.sp - prev) / dt;
    const long cnt = (long)std::floor(s.pos / mpc * 3.0f);
    s.meas += 0.35f * ((float)(cnt - s.cnt_prev) / 3.0f * mpc / dt - s.meas);
    s.cnt_prev = cnt;
    const float u = pi_step(st, p, s.sp, rate, s.meas, dt, 1.0f, true);
    const float ueff = std::fabs(u) < fric ? 0.0f : u - std::copysign(fric, u);
    for (int i = 0; i < 10; i++) {
      s.v += dt / 10 / tau * (gain * ueff - s.v);
      s.pos += s.v * dt / 10;
    }
    if (max_v && s.v > *max_v) *max_v = s.v;
  }
  return s.v;
}

static void test_drive_math() {
  g_ctx = "drive_math";
  float l, r;
  skid_steer(0.3f, 0.0f, 0.87f, &l, &r);
  CHECK(NEAR(l, 0.3, 1e-6) && NEAR(r, 0.3, 1e-6));
  skid_steer(0.0f, 1.0f, 0.87f, &l, &r); // w > 0 = turn left: right side faster
  CHECK(NEAR(l, -0.435, 1e-6) && NEAR(r, 0.435, 1e-6));
  l = 0.9f;
  r = 0.3f;
  limit_sides(&l, &r, 0.7f);
  CHECK(NEAR(l, 0.7, 1e-6) && NEAR(r, 0.7 / 3.0, 1e-6));
  CHECK(NEAR(slew_toward(0.0f, 1.0f, 0.016f), 0.016, 1e-7));
  CHECK(NEAR(slew_toward(0.5f, 0.0f, 0.03f), 0.47, 1e-6));
  CHECK(NEAR(slew_toward(0.5f, 0.49f, 0.03f), 0.49, 1e-7));

  PiParams p = piDefaults();
  CHECK(feed_forward(p, 0.005f, 0.0f, 1.0f) == 0.0f);
  CHECK(feed_forward(p, -0.3f, 0.0f, 1.0f) < -0.3f);
  CHECK(feed_forward(p, 0.3f, 0.0f, 1.1f) > feed_forward(p, 0.3f, 0.0f, 1.0f)); // low battery
  CHECK(feed_forward(p, 0.3f, 0.8f, 1.0f) > feed_forward(p, 0.3f, 0.0f, 1.0f)); // accelerating

  // nominal plant: converges, overshoot < 10 %
  PiState st;
  pi_reset(&st);
  Sim a;
  float vmax = 0.0f;
  simulate(a, 0.30f, 0.869f, 3.0f, &st, p, &vmax);
  CHECK(NEAR(a.v, 0.30, 0.009));
  CHECK(vmax < 0.33f);
  // loaded plant (0.6 m/s at full duty): integral action removes the error
  pi_reset(&st);
  Sim b;
  simulate(b, 0.30f, 0.6f, 3.0f, &st, p);
  CHECK(NEAR(b.v, 0.30, 0.009));
  // anti-windup: unreachable setpoint (0.7 > 0.6 - friction) for 4 s: the
  // output saturates, the integrator must not wind up; afterwards 0.3 is
  // reached without overshoot like a normal step on the loaded plant
  pi_reset(&st);
  Sim c;
  simulate(c, 0.70f, 0.6f, 4.0f, &st, p);
  CHECK(std::fabs(st.integ) < 0.05f);
  float vmax_after = 0.0f;
  Sim c2 = c;
  simulate(c2, 0.30f, 0.6f, 0.6f, &st, p); // ramp down (0.5 s)
  simulate(c2, 0.30f, 0.6f, 3.0f, &st, p, &vmax_after);
  CHECK(NEAR(c2.v, 0.30, 0.009));
  CHECK(vmax_after < 0.33f);
  // the integrator does not run while the setpoint ramps
  pi_reset(&st);
  pi_step(&st, p, 0.2f, 0.8f, 0.0f, 0.02f, 1.0f, true);
  CHECK(st.integ == 0.0f);
  pi_step(&st, p, 0.2f, 0.0f, 0.0f, 0.02f, 1.0f, true);
  CHECK(st.integ > 0.0f);
  // dead band brakes and clears the integrator
  CHECK(pi_step(&st, p, 0.0f, 0.0f, 0.1f, 0.02f, 1.0f, true) == 0.0f && st.integ == 0.0f);
  // open loop: feed-forward only, integrator untouched
  pi_reset(&st);
  CHECK(NEAR(pi_step(&st, p, 0.3f, 0.0f, 0.0f, 0.02f, 1.0f, false), feed_forward(p, 0.3f, 0.0f, 1.0f), 1e-6));
  CHECK(st.integ == 0.0f);

  // odometry
  Pose2D pose = {0, 0, 0};
  for (int i = 0; i < 100; i++) odom_integrate(&pose, 0.01f, 0.01f, 0.87f);
  CHECK(NEAR(pose.x, 1.0, 1e-4) && NEAR(pose.y, 0.0, 1e-6) && NEAR(pose.theta, 0.0, 1e-6));
  pose.x = pose.y = pose.theta = 0;
  const float arc = 3.14159265f * 0.87f / 2.0f; // half turn in place
  for (int i = 0; i < 100; i++) odom_integrate(&pose, -arc / 100, arc / 100, 0.87f);
  CHECK(NEAR(std::fabs(pose.theta), 3.14159265, 1e-4) && NEAR(pose.x, 0.0, 1e-5));
  CHECK(NEAR(wrap_pi(3.5f), 3.5 - 6.28318531, 1e-5));
}

// ------------------------------------------------------------ scan_logic
static void ranges(uint16_t r[6], uint16_t fl, uint16_t fc, uint16_t fr, uint16_t le,
                   uint16_t ri, uint16_t re) {
  r[0] = fl; r[1] = fc; r[2] = fr; r[3] = le; r[4] = ri; r[5] = re;
}

static void test_scan_logic() {
  g_ctx = "scan_logic";
  const uint16_t F = 0xFFFF; // no echo = free
  ScanLogic s;
  float v, w;
  uint16_t r[6];
  uint32_t t = 1000;
  ranges(r, F, F, F, F, F, F);
  s.update(r, t, &v, &w);
  CHECK(!s.active() && v == 0.0f && w == 0.0f);
  s.start(t);
  s.update(r, t, &v, &w);
  CHECK(s.state() == SCAN_FORWARD && v > 0.0f && w == 0.0f);

  ranges(r, 400, 2000, 2000, F, F, F); // obstacle front-left -> rotate right
  s.update(r, t += 50, &v, &w);
  CHECK(s.state() == SCAN_ROTATE_RIGHT && v == 0.0f && w < 0.0f);
  ranges(r, 650, 2000, 2000, F, F, F); // hysteresis: 650 is not "clear" (700)
  s.update(r, t += 500, &v, &w);
  CHECK(s.state() == SCAN_ROTATE_RIGHT);
  ranges(r, 900, 2000, 2000, F, F, F);
  s.update(r, t += 50, &v, &w);
  CHECK(s.state() == SCAN_FORWARD && v > 0.0f);

  ranges(r, 2000, 2000, 300, F, F, F); // front-right -> rotate left
  s.update(r, t += 50, &v, &w);
  CHECK(s.state() == SCAN_ROTATE_LEFT && w > 0.0f);
  ranges(r, F, F, F, F, F, F);
  s.update(r, t += 100, &v, &w); // dwell not over yet
  CHECK(s.state() == SCAN_ROTATE_LEFT);
  s.update(r, t += 300, &v, &w);
  CHECK(s.state() == SCAN_FORWARD);

  // front-centre, rear free -> back up, then turn toward the freer side (right)
  ranges(r, 1500, 500, 3000, 800, F, F);
  s.update(r, t += 50, &v, &w);
  CHECK(s.state() == SCAN_BACKUP && v < 0.0f && w == 0.0f);
  s.update(r, t += 600, &v, &w);
  CHECK(s.state() == SCAN_BACKUP);
  s.update(r, t += 700, &v, &w); // > 1.2 s
  CHECK(s.state() == SCAN_ROTATE_RIGHT && w < 0.0f);

  // both front corners blocked, rear blocked -> no back-up, rotate to freer side (left)
  s.start(t += 50);
  ranges(r, 500, 900, 550, 3000, 700, 300);
  s.update(r, t += 50, &v, &w);
  CHECK(s.state() == SCAN_ROTATE_LEFT);

  // everything in front and on the sides blocked -> stop
  s.start(t += 50);
  ranges(r, 300, 300, 300, 300, 300, F);
  s.update(r, t += 50, &v, &w);
  CHECK(s.state() == SCAN_BLOCKED && v == 0.0f && w == 0.0f);
  ranges(r, 300, 300, 300, 2000, 300, F); // left side opens
  s.update(r, t += 400, &v, &w);
  CHECK(s.state() == SCAN_BACKUP || s.state() == SCAN_ROTATE_LEFT);

  // rotating too long without finding a gap -> blocked
  s.start(t += 50);
  ranges(r, 400, 2000, 2000, F, F, F);
  s.update(r, t += 50, &v, &w);
  CHECK(s.state() == SCAN_ROTATE_RIGHT);
  s.update(r, t += 13000, &v, &w);
  CHECK(s.state() == SCAN_BLOCKED);

  s.stop();
  s.update(r, t += 50, &v, &w);
  CHECK(!s.active() && v == 0.0f && w == 0.0f);
}

// ----------------------------------------------------------- bmp180_comp
static void test_bmp180() {
  g_ctx = "bmp180";
  // Bosch datasheet example (BST-BMP180-DS000, 3.5)
  Bmp180Calib c;
  c.ac1 = 408; c.ac2 = -72; c.ac3 = -14383; c.ac4 = 32741; c.ac5 = 32757; c.ac6 = 23153;
  c.b1 = 6190; c.b2 = 4; c.mb = -32768; c.mc = -8711; c.md = 2868;
  int32_t b5 = 0;
  const int32_t t = bmp180_temperature_dC(c, 27898, &b5);
  CHECK(t == 150); // 15.0 degC
  const int32_t p = bmp180_pressure_Pa(c, 23843, 0, b5);
  std::printf("  bmp180 datasheet example: T = %ld (0.1 degC), p = %ld Pa (datasheet: 150, 69964)\n",
              (long)t, (long)p);
  CHECK(NEAR(p, 69964, 1)); // integer path differences in X2 rounding allowed: +-1 Pa
  // calibration parser: big-endian words, 0x0000/0xFFFF rejected
  uint8_t raw[22];
  const int16_t words[11] = {408, -72, -14383, (int16_t)32741, (int16_t)32757, (int16_t)23153,
                             6190, 4, -32768, -8711, 2868};
  for (int i = 0; i < 11; i++) {
    raw[2 * i] = (uint8_t)((uint16_t)words[i] >> 8);
    raw[2 * i + 1] = (uint8_t)((uint16_t)words[i] & 0xFF);
  }
  Bmp180Calib d;
  CHECK(bmp180_parse_calib(raw, &d) && d.ac1 == 408 && d.ac4 == 32741 && d.mc == -8711 && d.mb == -32768);
  raw[4] = raw[5] = 0xFF;
  CHECK(!bmp180_parse_calib(raw, &d));
  CHECK(bmp180_pressure_wait_ms(0) >= 5 && bmp180_pressure_wait_ms(3) >= 26);
}

// --------------------------------------------------------------- imu_math
static void test_imu() {
  g_ctx = "imu";
  CHECK(mpu_acc_mm_s2(8192) == 9807);
  CHECK(mpu_acc_mm_s2(32767) == 32767); // saturates (+-3.34 g max in an i16 mm/s^2 field)
  CHECK(mpu_acc_mm_s2(-32768) == -32768);
  CHECK(mpu_gyro_mrad_s(655) == 175); // 10 deg/s = 174.5 mrad/s
  CompFilter f;
  cf_init(&f, 0.98f);
  const float g = 8192.0f, a = 30.0f * 3.14159265f / 180.0f;
  cf_update(&f, 0.0f, g * std::sin(a), g * std::cos(a), 0.0f, 0.0f, 0.04f);
  CHECK(rad_to_cdeg(f.roll) == 3000 && rad_to_cdeg(f.pitch) == 0);
  // gyro integration dominates short-term: 10 deg/s for 1 s with a level accel
  cf_init(&f, 0.98f);
  cf_update(&f, 0.0f, 0.0f, g, 0.0f, 0.0f, 0.04f);
  for (int i = 0; i < 25; i++) cf_update(&f, 0.0f, 0.0f, g, 10.0f * 3.14159265f / 180.0f, 0.0f, 0.04f);
  const float roll_deg = f.roll * 180.0f / 3.14159265f;
  CHECK(roll_deg > 5.0f && roll_deg < 10.0f); // pulled back toward 0 by the accel term
}

// ---------------------------------------------------------------- arm_math
static void test_arm() {
  g_ctx = "arm";
  CHECK(min_jerk(0.0f) == 0.0f && min_jerk(1.0f) == 1.0f && NEAR(min_jerk(0.5f), 0.5, 1e-6));
  CHECK(min_jerk(-1.0f) == 0.0f && min_jerk(2.0f) == 1.0f);
  bool mono = true;
  for (int i = 1; i <= 100; i++) mono = mono && min_jerk(i / 100.0f) >= min_jerk((i - 1) / 100.0f);
  CHECK(mono);
  CHECK(arm_deg_to_counts(0) == 102 && arm_deg_to_counts(180) == 512 && arm_deg_to_counts(90) == 307);
  uint8_t o;
  CHECK(arm_clamp(1, 10, &o) && o == 15);   // shoulder 15..165
  CHECK(arm_clamp(5, 90, &o) && o == 80);   // gripper 10..80
  CHECK(!arm_clamp(0, 180, &o) && o == 180);
  // every named pose respects the joint limits; home matches ICD 9.5
  bool ok = true;
  for (int p = 0; p < ARM_NUM_POSES; p++)
    for (int j = 0; j < ARM_NJ; j++) {
      const uint8_t d = kPoses[p].deg[j];
      if (d != POSE_KEEP && (d < kJoints[j].min_deg || d > kJoints[j].max_deg)) ok = false;
    }
  CHECK(ok);
  const uint8_t home[ARM_NJ] = {90, 90, 90, 90, 90, 10};
  CHECK(std::memcmp(kPoses[0].deg, home, ARM_NJ) == 0);
  for (int j = 0; j < ARM_NJ; j++) CHECK(kJoints[j].home_deg == home[j]);
  // duration stretching: 180 deg at 120 deg/s peak -> >= 2.8125 s
  const float from[ARM_NJ] = {0, 90, 90, 90, 90, 10};
  const uint8_t to[ARM_NJ] = {180, 90, 90, 90, 90, 10};
  CHECK(arm_motion_duration(from, to, 500, 120.0f) == 2813);
  CHECK(arm_motion_duration(from, to, 4000, 120.0f) == 4000);
  float out[ARM_NJ];
  arm_interpolate(from, to, 0, 1000, out);
  CHECK(out[0] == 0.0f);
  arm_interpolate(from, to, 500, 1000, out);
  CHECK(NEAR(out[0], 90.0, 1e-4));
  arm_interpolate(from, to, 1000, 1000, out);
  CHECK(out[0] == 180.0f && out[5] == 10.0f);
}

// --------------------------------------------- Mega legacy TLM (LabVIEW, 0x01)
static void test_legacy_tlm() {
  g_ctx = "legacy_tlm";
  CHECK(legacy_cm8(0xFFFF) == 255);  // no echo
  CHECK(legacy_cm8(2550) == 255 && legacy_cm8(2544) == 254 && legacy_cm8(4000) == 255);
  CHECK(legacy_cm8(704) == 70 && legacy_cm8(705) == 71 && legacy_cm8(20) == 2);
  // FL 900, FC 455, FR 1234, L/R/rear arbitrary
  const uint16_t r[6] = {900, 455, 1234, 300, 300, 300};
  SspTlmLegacy t;
  legacy_tlm_from_ranges(r, &t);
  CHECK(t.ir == 0 && t.gas_hi == 0 && t.gas_lo == 0 && t.temp_c == 0 && t.rh_pct == 0);
  CHECK(t.us1_cm == 46 && t.us2_cm == 90);
  const uint16_t far[6] = {0xFFFF, 0xFFFF, 3000, 1, 1, 1}; // FC none, FR 3 m (> 255 cm)
  legacy_tlm_from_ranges(far, &t);
  CHECK(t.us1_cm == 255 && t.us2_cm == 255);
  const uint16_t none[6] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
  legacy_tlm_from_ranges(none, &t);
  CHECK(t.us1_cm == 255 && t.us2_cm == 255);
  // full reply frame: 7-byte layout, SRC 0x01 -> GS 0x0B, decodable as legacy TLM
  legacy_tlm_from_ranges(r, &t);
  uint8_t payload[SSP_LEN_TLM_LEGACY], wire[SSP_MAX_FRAME];
  const uint8_t n = ssp_pack_tlm_legacy(payload, t);
  CHECK(n == 7);
  const size_t len = ssp_encode(wire, sizeof wire, SSP_ADDR_GROUND, SSP_ADDR_ROVER, SSP_GET_TLM, payload, n);
  CHECK(len == 15 && wire[1] == 0x0B && wire[2] == 0x01 && wire[3] == 0x0F && wire[4] == 7);
  CHECK(wire[5] == 0 && wire[6] == 0 && wire[7] == 0 && wire[8] == 46 && wire[9] == 90 && wire[10] == 0 &&
        wire[11] == 0);
}

int main() {
  test_drive_math();
  test_legacy_tlm();
  test_scan_logic();
  test_bmp180();
  test_imu();
  test_arm();
  std::printf("%s: firmware logic tests %d passed, %d failed\n", g_fail ? "FAILED" : "OK", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
