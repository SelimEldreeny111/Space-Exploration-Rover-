/*
 * payload_uno.ino - MISSION X Payload MCU (Arduino Uno, SSP address 0x03).
 * ICD v2.0. Board: arduino:avr:uno.
 *
 * Serial 115200 (L3, USB CDC to the Brain):
 *   out: ENV_TLM 1 Hz, IMU 25 Hz, HEARTBEAT 1 Hz     in: PING -> ACK
 * Sensors: DHT11 (D2, DFRobot_DHT11 library), BMP180 (0x77) and MPU-6050
 * (0x68) on I2C, MQ-2 A0, MQ-4 A1, soil A2, rain A3 (x8 oversampling),
 * inductive proximity D3 (LOW = metal).
 *
 * Robustness: every I2C transaction is bounded by Wire.setWireTimeout(), a
 * failed sensor is flagged in ENV_TLM.status / HEARTBEAT.fault_flags and
 * re-initialised every 5 s. The only blocking call is DFRobot_DHT11::read()
 * (library, ~225 ms), made at most every 5 s (30 s while the DHT11 fails).
 * RAM: no dynamic memory, no String, no Serial.print.
 */
#include <DFRobot_DHT11.h>
#include <MissionX_SSP.h>
#include <Wire.h>

#include "bmp180.h"
#include "config.h"
#include "i2c_util.h"
#include "imu_math.h"
#include "mpu6050.h"

/* HEARTBEAT.fault_flags of the Payload node (ICD 6.2 defines the bits for
 * Mobility only): bit0 DHT11, bit1 BMP180, bit2 MPU-6050 not working. */
#define PAYLOAD_FAULT_DHT 0x01
#define PAYLOAD_FAULT_BMP 0x02
#define PAYLOAD_FAULT_MPU 0x04

static SspLink g_link(Serial, SSP_ADDR_PAYLOAD);
static DFRobot_DHT11 g_dht;
static Bmp180 g_bmp;
static CompFilter g_cf;

/* DHT11 */
static bool g_dht_ok = false;
static int16_t g_dht_t_cC = 0;
static uint16_t g_dht_rh_cP = 0;
static uint32_t g_dht_next = DHT_FIRST_MS;

/* MPU-6050 */
static bool g_mpu_ok = false;      /* initialised and calibrated   */
static bool g_mpu_up = false;      /* initialised                  */
static uint32_t g_mpu_retry = 0;
static int32_t g_gyro_sum[3];
static int16_t g_gyro_min[3], g_gyro_max[3];
static int16_t g_gyro_bias[3];
static uint8_t g_cal_n = 0, g_cal_attempts = 0;
static uint32_t g_imu_last_us = 0;

/* BMP180 */
static uint32_t g_bmp_retry = 0;

/* analog channels (x8 oversampling, one channel per ANALOG_SAMPLE_MS) */
static const uint8_t kAnalogPin[4] = {MQ2_PIN, MQ4_PIN, SOIL_PIN, RAIN_PIN};
static uint16_t g_an_sum[4];
static uint8_t g_an_n[4];
static uint16_t g_an_val[4];
static uint8_t g_an_ch = 0;

static bool g_prox_seen = false; /* metal seen since the last ENV_TLM */

/* ---------------------------------------------------------- scheduler */
struct Every {
  uint32_t next;
  uint16_t period;
  bool due(uint32_t now) {
    if ((int32_t)(now - next) < 0) return false;
    next += period;
    if ((int32_t)(now - next) >= 0) next = now + period; /* re-align, no burst */
    return true;
  }
};
static Every g_env_t = {500, ENV_PERIOD_MS};
static Every g_imu_t = {0, IMU_PERIOD_MS};
static Every g_hb_t = {250, HEARTBEAT_PERIOD_MS};
static Every g_an_t = {0, ANALOG_SAMPLE_MS};

/* ------------------------------------------------------------ sensors */
static void dhtUpdate(uint32_t now) {
  if ((int32_t)(now - g_dht_next) < 0) return;
  g_link.poll(); /* empty the UART buffer before the blocking read */
  const uint32_t t0 = millis();
  g_dht.read(DHT_PIN); /* BLOCKING ~225 ms (DFRobot library) */
  const uint32_t took = millis() - t0;
  /* the library exposes no checksum: accept only plausible DHT11 values
   * (20-90 %RH, 0-50 degC) from a read that did not run into its timeouts */
  const bool valid = took < DHT_MAX_READ_MS && g_dht.humidity >= 1 && g_dht.humidity <= 100 &&
                     g_dht.temperature >= 0 && g_dht.temperature <= 60;
  g_dht_ok = valid;
  if (valid) {
    g_dht_t_cC = (int16_t)(g_dht.temperature * 100);
    g_dht_rh_cP = (uint16_t)(g_dht.humidity * 100);
  }
  g_dht_next = millis() + (valid ? DHT_PERIOD_MS : DHT_RETRY_MS);
}

static void analogUpdate() {
  const uint8_t ch = g_an_ch;
  analogRead(kAnalogPin[ch]); /* discard: S/H settles after the mux switch */
  g_an_sum[ch] += (uint16_t)analogRead(kAnalogPin[ch]);
  if (++g_an_n[ch] >= ANALOG_OVERSAMPLE) {
    g_an_val[ch] = (uint16_t)((g_an_sum[ch] + ANALOG_OVERSAMPLE / 2) / ANALOG_OVERSAMPLE);
    g_an_sum[ch] = 0;
    g_an_n[ch] = 0;
  }
  g_an_ch = (uint8_t)((ch + 1) & 3);
}

static void mpuStartCalibration() {
  g_cal_n = 0;
  for (uint8_t i = 0; i < 3; i++) {
    g_gyro_sum[i] = 0;
    g_gyro_min[i] = 32767;
    g_gyro_max[i] = -32768;
  }
}

static void mpuInit(uint32_t now) {
  g_mpu_up = mpu6050::begin();
  g_mpu_ok = false;
  g_cal_attempts = 0;
  mpuStartCalibration();
  cf_init(&g_cf, COMP_ALPHA);
  g_imu_last_us = micros();
  if (!g_mpu_up) g_mpu_retry = now + SENSOR_RETRY_MS;
}

static void imuUpdate(uint32_t now) {
  if (!g_mpu_up) {
    if ((int32_t)(now - g_mpu_retry) >= 0) mpuInit(now);
    return;
  }
  int16_t acc[3], gyro[3];
  if (!mpu6050::read(acc, gyro)) {
    g_mpu_up = g_mpu_ok = false;
    g_mpu_retry = now + SENSOR_RETRY_MS;
    return;
  }
  const uint32_t now_us = micros();
  float dt = (float)(now_us - g_imu_last_us) * 1e-6f;
  g_imu_last_us = now_us;
  if (dt <= 0.0f || dt > 0.5f) dt = IMU_PERIOD_MS * 1e-3f;

  if (!g_mpu_ok) {
    /* start-up gyro bias calibration: the rover must stand still for 2 s */
    for (uint8_t i = 0; i < 3; i++) {
      g_gyro_sum[i] += gyro[i];
      if (gyro[i] < g_gyro_min[i]) g_gyro_min[i] = gyro[i];
      if (gyro[i] > g_gyro_max[i]) g_gyro_max[i] = gyro[i];
    }
    if (++g_cal_n < GYRO_CAL_SAMPLES) return;
    bool still = true;
    for (uint8_t i = 0; i < 3; i++)
      if ((int32_t)g_gyro_max[i] - g_gyro_min[i] > GYRO_CAL_MAX_SPREAD) still = false;
    if (!still && ++g_cal_attempts < GYRO_CAL_ATTEMPTS) {
      mpuStartCalibration(); /* it moved: try again */
      return;
    }
    for (uint8_t i = 0; i < 3; i++)
      g_gyro_bias[i] = (int16_t)((g_gyro_sum[i] + (g_gyro_sum[i] >= 0 ? 1 : -1) * (GYRO_CAL_SAMPLES / 2)) /
                                 GYRO_CAL_SAMPLES);
    g_mpu_ok = true;
    return;
  }

  const int32_t gx = (int32_t)gyro[0] - g_gyro_bias[0];
  const int32_t gy = (int32_t)gyro[1] - g_gyro_bias[1];
  const int32_t gz = (int32_t)gyro[2] - g_gyro_bias[2];
  cf_update(&g_cf, acc[0], acc[1], acc[2], mpu_gyro_rad_s(gx), mpu_gyro_rad_s(gy), dt);

  SspImu m;
  m.t_ms = now;
  m.ax = mpu_acc_mm_s2(acc[0]);
  m.ay = mpu_acc_mm_s2(acc[1]);
  m.az = mpu_acc_mm_s2(acc[2]);
  m.gx = mpu_gyro_mrad_s(gx);
  m.gy = mpu_gyro_mrad_s(gy);
  m.gz = mpu_gyro_mrad_s(gz);
  m.roll_cdeg = rad_to_cdeg(g_cf.roll);
  m.pitch_cdeg = rad_to_cdeg(g_cf.pitch);
  uint8_t buf[SSP_LEN_IMU];
  g_link.send(SSP_ADDR_ROVER, SSP_IMU, buf, ssp_pack_imu(buf, m), true);
}

static void bmpUpdate(uint32_t now) {
  g_bmp.update(now, BMP_PERIOD_MS);
  /* the state machine stops after an I2C error: re-initialise every 5 s */
  if (!g_bmp.running() && (int32_t)(now - g_bmp_retry) >= 0) {
    g_bmp_retry = now + SENSOR_RETRY_MS;
    g_bmp.begin(BMP180_OSS);
  }
}

static uint8_t envStatus(uint32_t now) {
  static bool warm = false; /* latched: millis() wraps after 49 days */
  if (!warm && now >= GAS_WARMUP_MS) warm = true;
  uint8_t s = 0;
  if (g_dht_ok) s |= SSP_ENV_DHT_OK;
  if (g_bmp.ok()) s |= SSP_ENV_BMP_OK;
  if (g_mpu_ok) s |= SSP_ENV_MPU_OK;
  if (warm) s |= SSP_ENV_GAS_WARM;
  return s;
}

static void sendEnv(uint32_t now) {
  SspEnvTlm e;
  e.dht_temp_cC = g_dht_ok ? g_dht_t_cC : 0;
  e.dht_rh_cP = g_dht_ok ? g_dht_rh_cP : 0;
  e.bmp_pressure_Pa = g_bmp.ok() ? (uint32_t)g_bmp.pressurePa() : 0;
  e.bmp_temp_cC = g_bmp.ok() ? g_bmp.temperature_cC() : 0;
  e.mq2_raw = g_an_val[0];
  e.mq4_raw = g_an_val[1];
  e.soil_raw = g_an_val[2];
  e.rain_raw = g_an_val[3];
  e.proximity = (g_prox_seen || digitalRead(PROX_PIN) == LOW) ? 1 : 0;
  g_prox_seen = false;
  e.status = envStatus(now);
  uint8_t buf[SSP_LEN_ENV_TLM];
  g_link.send(SSP_ADDR_ROVER, SSP_ENV_TLM, buf, ssp_pack_env_tlm(buf, e), true);
}

static void sendHeartbeat(uint32_t now) {
  SspHeartbeat h;
  h.uptime_ms = now;
  h.mode = SSP_MODE_IDLE;
  h.fault_flags = (uint8_t)((g_dht_ok ? 0 : PAYLOAD_FAULT_DHT) | (g_bmp.ok() ? 0 : PAYLOAD_FAULT_BMP) |
                            (g_mpu_ok ? 0 : PAYLOAD_FAULT_MPU));
  uint8_t buf[SSP_LEN_HEARTBEAT];
  g_link.send(SSP_ADDR_ROVER, SSP_HEARTBEAT, buf, ssp_pack_heartbeat(buf, h), true);
}

/* ----------------------------------------------------------- commands */
static void onFrame(const SspFrame &f, void *) {
  if (f.type == SSP_PING) {
    g_link.ack(f, false); /* ACK, LEN 0 */
    return;
  }
  if (ssp_is_report_type(f.type) || f.dest == SSP_ADDR_BROADCAST) return;
  g_link.nack(f, SSP_NACK_UNKNOWN_TYPE);
}

/* -------------------------------------------------------- setup / loop */
void setup() {
  pinMode(PROX_PIN, INPUT_PULLUP);
  Serial.begin(LINK_BAUD);
  g_link.begin(onFrame, NULL, RX_TIMEOUT_MS);
  i2c_begin();
  const uint32_t now = millis();
  g_bmp.begin(BMP180_OSS);
  g_bmp_retry = now + SENSOR_RETRY_MS;
  mpuInit(now);
}

void loop() {
  const uint32_t now = millis();
  g_link.poll();
  if (digitalRead(PROX_PIN) == LOW) g_prox_seen = true;
  if (g_an_t.due(now)) analogUpdate();
  bmpUpdate(now);
  if (g_imu_t.due(now)) imuUpdate(now);
  if (g_env_t.due(now)) sendEnv(now);
  if (g_hb_t.due(now)) sendHeartbeat(now);
  dhtUpdate(now); /* blocking, rarely: keep it last */
}
