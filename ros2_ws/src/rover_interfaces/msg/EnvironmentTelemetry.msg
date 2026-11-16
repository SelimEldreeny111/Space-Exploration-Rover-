# Payload MCU environment telemetry (SSP ENV_TLM 0x33, ICD v2.0 section 6.2), 1 Hz.
# Published by rover_hardware/payload_bridge on /environment.

# bits of `status`
uint8 STATUS_DHT_OK=1        # bit0 DHT11 read ok
uint8 STATUS_BMP_OK=2        # bit1 BMP180 read ok
uint8 STATUS_MPU_OK=4        # bit2 MPU-6050 ok
uint8 STATUS_GAS_WARM=8      # bit3 MQ-2 / MQ-4 warmed up (> 60 s)

std_msgs/Header header

float32 temperature_c        # [degC] DHT11 (BMP180 fallback); NaN if no valid sensor
float32 humidity_pct         # [%RH] DHT11; NaN if invalid
float32 pressure_pa          # [Pa] BMP180; NaN if invalid
uint16 mq2_raw               # ADC counts (H2 / LPG / propane / smoke)
uint16 mq4_raw               # ADC counts (CH4)
uint16 soil_raw              # ADC counts (soil moisture, LM393 AO)
uint16 rain_raw              # ADC counts (rain sensor AO)
bool metal_detected          # inductive proximity PR08-2DN
uint8 status                 # STATUS_* bit field
