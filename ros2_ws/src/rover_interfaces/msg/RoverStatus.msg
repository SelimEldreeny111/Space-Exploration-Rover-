# Rover / MCU status, built from SSP HEARTBEAT (0x50) frames (ICD v2.0 section 6.2).
# Published on /rover/status by the MCU bridges (field `source` tells which MCU).

uint8 MODE_IDLE=0
uint8 MODE_TELEOP=1
uint8 MODE_AUTONOMOUS=2
uint8 MODE_SAFE=3

# fault_flags bits (mobility MCU)
uint8 FAULT_ESTOP=1          # bit0 e-stop latched
uint8 FAULT_CMD_TIMEOUT=2    # bit1 SET_VELOCITY timeout
uint8 FAULT_LOW_BATTERY=4    # bit2 battery < 10.8 V
uint8 FAULT_ENCODER=8        # bit3 wheel commanded but no ticks for 1 s
uint8 FAULT_SAFE_MODE=16     # bit4 HC-12 safe mode active

std_msgs/Header header
uint8 source                 # SSP address of the reporting node (2 mobility, 3 payload, 4 arm)
uint8 mode                   # MODE_*
uint8 fault_flags            # FAULT_* bit field
float32 battery_v            # [V]; NaN when not reported
float64 uptime_s             # MCU uptime [s]
