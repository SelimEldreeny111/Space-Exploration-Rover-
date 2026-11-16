/*
 * config.h - MISSION X Payload MCU (Arduino Uno, SSP address 0x03).
 * Pin map ICD v2.0 section 9.4.
 */
#ifndef PAYLOAD_CONFIG_H
#define PAYLOAD_CONFIG_H

#define LINK_BAUD 115200UL /* L3: USB CDC to the Brain                 */
#define RX_TIMEOUT_MS 20   /* ICD 3 step 6 (L2-L4)                      */

/* ---------------------------------------------------------------- pins */
#define DHT_PIN 2   /* DHT11 data                                        */
#define PROX_PIN 3  /* PR08-2DN NPN NO: INPUT_PULLUP, LOW = metal        */
#define MQ2_PIN A0
#define MQ4_PIN A1
#define SOIL_PIN A2
#define RAIN_PIN A3
/* I2C: A4 SDA, A5 SCL */

/* ---------------------------------------------------------------- I2C */
#define I2C_CLOCK_HZ 100000UL  /* 100 kHz: robust with long wires near motors */
#define WIRE_TIMEOUT_US 5000UL /* a stuck bus is reset instead of hanging loop() */
#define BMP180_ADDR 0x77
#define BMP180_OSS 3           /* 0..3 (ultra low power .. ultra high res)  */
#define MPU_ADDR 0x68

/* ------------------------------------------------------------- timing */
#define ENV_PERIOD_MS 1000      /* ENV_TLM 1 Hz              */
#define IMU_PERIOD_MS 40        /* IMU 25 Hz                 */
#define HEARTBEAT_PERIOD_MS 1000
#define BMP_PERIOD_MS 1000      /* one T+P measurement per second */
/* DFRobot_DHT11::read() is BLOCKING (~225 ms; up to ~2 s if the sensor does
 * not answer). It is therefore called rarely: every DHT_PERIOD_MS while the
 * sensor answers, every DHT_RETRY_MS after a failed read. The DHT11 itself
 * has a 6-30 s response time, so 5 s loses no information. */
#define DHT_FIRST_MS 2000       /* DHT11 needs > 1 s after power-up */
#define DHT_PERIOD_MS 5000
#define DHT_RETRY_MS 30000
#define DHT_MAX_READ_MS 600     /* a read that took longer has timed out */
#define SENSOR_RETRY_MS 5000    /* re-initialise a failed BMP180 / MPU-6050 */
#define GAS_WARMUP_MS 60000UL   /* status bit3 after 60 s (ICD 6.2) */
#define ANALOG_SAMPLE_MS 25     /* one channel per 25 ms, x8 oversampling */
#define ANALOG_OVERSAMPLE 8

/* ---------------------------------------------------------------- IMU */
#define GYRO_CAL_SAMPLES 50     /* 2 s at 25 Hz, rover must stand still   */
#define GYRO_CAL_MAX_SPREAD 300 /* raw LSB (4.6 deg/s): rover moved -> retry */
#define GYRO_CAL_ATTEMPTS 5
#define COMP_ALPHA 0.98f        /* complementary filter (ICD 6.2)          */

#endif
