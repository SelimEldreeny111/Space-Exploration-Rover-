# MISSION X Rover — Interface Control Document (ICD) v2.0

This document is the single source of truth for every interface between the
rover's computers (Raspberry Pi 4 "Brain", Arduino Mega 2560 "Mobility",
Arduino Uno "Payload", Arduino Uno "Arm") and the Ground Station (LabVIEW GS,
Python GS, Flutter app bridge). Firmware, ROS 2 nodes and ground-station tools
MUST implement exactly what is written here. Changes require bumping the ICD
version and updating `rover_core/protocol.py` (Python reference implementation)
and `firmware/libraries/MissionX_SSP` (C reference implementation) together.

v2.0 is **wire-compatible with the legacy 2023 LabVIEW ground station**
(same flags, header order, CRC algorithm and CRC byte order). See §8 for the
defects in the 2023 implementation that v2.0 fixes.

---------------------------------------------------------------------------

## 1. Physical links

| Link | Endpoints | Medium | Settings |
|------|-----------|--------|----------|
| L1 Space link | Ground Station ↔ Brain | HC-12 433.4 MHz (channel 001, FU3, 20 dBm) via USB-UART on the Pi | 9600 baud, 8N1 |
| L1-safe (fallback) | Ground Station ↔ Mobility MCU | Same HC-12 radio wired to Mega `Serial2` (pins 16/17) | 9600 baud, 8N1 |
| L2 | Brain ↔ Mobility (Mega) | USB CDC (`/dev/rover_mobility`) | 115200 baud, 8N1 |
| L3 | Brain ↔ Payload (Uno) | USB CDC (`/dev/rover_payload`) | 115200 baud, 8N1 |
| L4 | Brain ↔ Arm (Uno) | USB CDC (`/dev/rover_arm`) | 115200 baud, 8N1 |
| L5 | Brain ↔ LiDAR | LDS-01 via USB2LDS (`/dev/rover_lidar`) | 230400 baud |

Only one HC-12 is fitted. In the nominal configuration it is connected to the
Brain (L1). If the Brain is not available the radio may be re-wired to the Mega
(L1-safe); the Mega firmware then answers PING / GET_TLM / STOP_SCAN /
ESTOP directly ("safe mode").

udev rules that create the `/dev/rover_*` names are in
`ros2_ws/src/rover_bringup/config/99-missionx-rover.rules`.

## 2. SSP frame format (wire level)

```
+------+------+-----+------+-----+-----------+-------+-------+------+
| FEND | DEST | SRC | TYPE | LEN | DATA[LEN] | CRC_L | CRC_H | FEND |
| 0xC0 |  1B  | 1B  |  1B  | 1B  | 0..255 B  |  1B   |  1B   | 0xC0 |
+------+------+-----+------+-----+-----------+-------+-------+------+
```

* `FEND` = `0xC0` opens and closes every frame. Back-to-back frames therefore
  appear as `... C0 C0 ...` on the wire; an empty frame (`C0 C0`) is ignored.
* `LEN` is the number of DATA bytes (0–255). Maximum frame = 263 bytes.
* CRC = **CRC-16/XMODEM** (a.k.a. CRC-16/CCITT-zero): polynomial `0x1021`,
  init `0x0000`, no input/output reflection, xorout `0x0000`.
  Check value: CRC("123456789") = `0x31C3`.
* CRC covers `DEST..DATA` (4 + LEN bytes) and is transmitted **low byte first**
  (little-endian) — this is the convention of the 2023 LabVIEW GS
  (verified from captured frame `C0 01 0B 0F 00 7B 96 C0`, CRC = 0x967B).
* All multi-byte fields inside DATA are **little-endian**.
* **Transparency (optional, per link):** when `transparency=ON`, any `0xC0`
  between the flags is sent as `0xDB 0xDC` and any `0xDB` as `0xDB 0xDD`
  (SLIP / KISS escaping, RFC 1055). Default is **OFF** on every link so that
  the legacy LabVIEW GS keeps working. With transparency OFF the receiver is
  *length-driven* (see §3), so 0xC0 bytes inside DATA/CRC are harmless.

### 2.1 Reference test vectors (transparency OFF)

| Description | Bytes (hex) |
|-------------|-------------|
| GS→Rover PING | `C0 01 0B 00 00 45 86 C0` |
| GS→Rover GET_TLM | `C0 01 0B 0F 00 7B 96 C0` (identical to the frame captured from the 2023 LabVIEW GS) |
| Rover→GS ACK | `C0 0B 01 01 00 1E 1A C0` |
| Rover→GS legacy 7-byte TLM `00 87 00 46 2E 08 16` | `C0 0B 01 0F 07 00 87 00 46 2E 08 16 80 D9 C0` |

(The test-vector table is regenerated and checked by
`ros2_ws/src/rover_core/test/test_ssp.py`; if this table and the code ever
disagree, the code's test is authoritative and this table must be fixed.)

## 3. Receiver state machine (normative)

Receivers MUST be length-driven and MUST validate the CRC before acting:

1. `HUNT`: discard bytes until `FEND`.
2. `HEADER`: collect 4 bytes (DEST, SRC, TYPE, LEN). If a `FEND` arrives
   while the header is empty, stay in `HEADER` (idle flag / back-to-back
   frames). If a `FEND` arrives inside a non-empty header → restart `HEADER`.
3. `BODY`: collect `LEN + 2` bytes (DATA + CRC).
4. `TAIL`: the next byte MUST be `FEND`; otherwise the frame is a framing
   error.
5. On framing error or CRC mismatch the receiver re-scans the bytes it has
   buffered **after the opening FEND** for the next `FEND` and resumes in
   `HEADER` from there (no byte that could start a valid frame is lost).
5a. *Length plausibility (recommended, mandatory on MCUs):* as soon as the
   4-byte header is complete, a receiver that knows the packet type checks
   that LEN is one of the lengths allowed for that type (table
   `_ALLOWED_LEN` in `rover_core/protocol.py`; e.g. PING → 0, SET_VELOCITY →
   6). An implausible LEN is a framing error handled as in step 5. Without
   this check a false start may hold the decision until LEN + 2 bytes have
   arrived (≤ 257 B); with it most false starts are rejected at the header.
6. Inter-byte timeout: if more than **50 ms** (L1) / **20 ms** (L2–L4) pass
   inside a frame, the partial frame is dropped and the receiver returns to
   `HUNT`.
7. Frames whose `DEST` is neither the receiver's own address nor `0xFF`
   (broadcast) are ignored (the Brain forwards them, see §5).

## 4. Addresses

| Addr | Node |
|------|------|
| `0x01` | ROVER — Brain / on-board computer (also the legacy single-MCU rover) |
| `0x02` | MOBILITY — Arduino Mega 2560 |
| `0x03` | PAYLOAD — Arduino Uno (environment sensors + IMU) |
| `0x04` | ARM — Arduino Uno + PCA9685 (6-DOF arm) |
| `0x0B` | GROUND — ground station (LabVIEW, Python GS, Firebase bridge) |
| `0xFF` | BROADCAST |

A response always swaps addresses: `resp.DEST = req.SRC`, `resp.SRC = own address`.

## 5. Routing

The Brain is the only router. A frame from the GS addressed to `0x02/0x03/0x04`
is forwarded unchanged to that MCU and the MCU's reply is forwarded back to the
GS. Frames addressed to `0x01` are executed by the Brain.

## 6. Packet types

### 6.1 Legacy command set (unchanged values, used by the LabVIEW GS)

| Type | Name | Request DATA | Response |
|------|------|--------------|----------|
| `0x00` | PING | — | `ACK` (0x01), LEN 0 |
| `0x01` | ACK | (v2: optional 1 B = acknowledged type) | — |
| `0x08` | GET_IMAGE | LEN 0: legacy (100×100, 8-bit gray, 50 chunks × 200 B, each sent as type 0x08). LEN 1: v2, DATA = `u8 quality` → chunks sent as `IMAGE_CHUNK` (0x09) | image chunks |
| `0x0C` | CAP_IMAGE | — | echo (type 0x0C, LEN 0) after the picture is stored |
| `0x0F` | GET_TLM | — | type 0x0F + telemetry (§7.1). Legacy MCU answers 7 B, Brain answers v2 56 B |
| `0x16` | START_SCAN | — | echo (0x16, LEN 0); starts autonomous exploration (mode AUTONOMOUS) |
| `0x1C` | STOP_SCAN | — | echo (0x1C, LEN 0); cancels navigation, stops motors (mode IDLE) |
| `0x20` | COLLECT_OBJECT | — | echo (0x20, LEN 0); runs the arm sample-collection sequence |

### 6.2 v2 additions

| Type | Name | Direction | DATA layout (little-endian) |
|------|------|-----------|-----------------------------|
| `0x02` | NACK | any | `u8 rejected_type, u8 reason` (1 bad length, 2 unknown type, 3 busy, 4 not permitted / e-stop latched, 5 hardware fault) |
| `0x09` | IMAGE_CHUNK | Brain→GS | `u16 seq, u16 total, u16 width, u16 height, u8 fmt(0=gray8,1=jpeg)`, then ≤ 200 B payload |
| `0x30` | SET_VELOCITY | Brain→Mobility | `i16 v_mm_s, i16 w_mrad_s, u16 timeout_ms` — body twist; MCU stops if not refreshed within `timeout_ms` (0 → default 500 ms). No response. |
| `0x31` | WHEEL_ODOM | Mobility→Brain (20 Hz) | `u32 t_ms, i32 ticks[6]` (order L-front, L-mid, L-rear, R-front, R-mid, R-rear; +ticks = wheel moving the rover forward) — 28 B |
| `0x32` | RANGES | Mobility→Brain (10 Hz) | `u8 n(=6), u16 range_mm[6]` (order §9.3; `0xFFFF` = no echo/out of range) — 13 B |
| `0x33` | ENV_TLM | Payload→Brain (1 Hz) | `i16 dht_temp_cC, u16 dht_rh_cP, u32 bmp_pressure_Pa, i16 bmp_temp_cC, u16 mq2_raw, u16 mq4_raw, u16 soil_raw, u16 rain_raw, u8 proximity, u8 status` — 20 B. `status` bit0 DHT ok, bit1 BMP ok, bit2 MPU ok, bit3 gas sensors warmed up (> 60 s) |
| `0x34` | IMU | Payload→Brain (25 Hz) | `u32 t_ms, i16 ax, ay, az [mm/s²], i16 gx, gy, gz [mrad/s], i16 roll_cdeg, pitch_cdeg` — 20 B (roll/pitch from complementary filter, α = 0.98) |
| `0x40` | ARM_SET_JOINTS | Brain→Arm | `u8 deg[6], u16 duration_ms` — 8 B; response ACK(0x40) or NACK |
| `0x41` | ARM_SET_POSE | Brain→Arm | `u8 pose_id` (0 HOME, 1 STOW, 2 READY, 3 REACH_GROUND, 4 GRIP_OPEN, 5 GRIP_CLOSE, 6 LIFT_SAMPLE, 7 DEPOSIT); response ACK(0x41) |
| `0x42` | ARM_STATE | Arm→Brain (5 Hz, and on change) | `u8 deg[6], u8 moving` — 7 B |
| `0x50` | HEARTBEAT | any MCU→Brain (1 Hz) | `u32 uptime_ms, u8 mode, u8 fault_flags` — 6 B |
| `0x51` | ESTOP | GS/Brain→Mobility | — ; motors stop immediately, latched; response ACK(0x51) |
| `0x52` | CLEAR_ESTOP | GS/Brain→Mobility | — ; response ACK(0x52) |
| `0x53` | SET_MODE | GS→Brain | `u8 mode` (0 IDLE, 1 TELEOP, 2 AUTONOMOUS, 3 SAFE); response ACK(0x53) |
| `0x60` | DRIVE | GS→Brain/Mobility | `i8 v_pct, i8 w_pct, u16 duration_ms` — open-loop teleop over the radio (±100 % of the configured max speed); response ACK(0x60) |

`fault_flags` (Mobility): bit0 e-stop latched, bit1 command timeout, bit2 low
battery (< 10.8 V), bit3 encoder fault (wheel commanded but no ticks for 1 s),
bit4 HC-12 safe-mode active.

## 7. Telemetry layouts

### 7.1 GET_TLM response

**Legacy (7 B, 2023 Mega firmware):** `u8 ir, u8 gas_hi, u8 gas_lo, u8 us1_cm, u8 us2_cm, u8 temp_C, u8 rh_pct`.

**v2 (56 B, Brain; Mega in safe mode fills unavailable fields with 0 and sets `fault_flags` bit4):**

| Off | Type | Field | Unit |
|-----|------|-------|------|
| 0 | u8 | tlm_version (= 2) | — |
| 1 | u8 | mode | 0 IDLE 1 TELEOP 2 AUTONOMOUS 3 SAFE |
| 2 | u8 | fault_flags | §6.2 |
| 3 | u16 | battery_mV | mV |
| 5 | u32 | uptime_s | s |
| 9 | i16 | temperature | 0.01 °C (DHT11) |
| 11 | u16 | humidity | 0.01 %RH (DHT11) |
| 13 | u32 | pressure | Pa (BMP180) |
| 17 | i16 | roll | 0.01 deg |
| 19 | i16 | pitch | 0.01 deg |
| 21 | i16 | yaw | 0.01 deg (from SLAM/odometry heading) |
| 23 | i16 ×3 | ax, ay, az | mm/s² |
| 29 | i16 ×3 | gx, gy, gz | mrad/s |
| 35 | u16 | mq2_raw | ADC counts (H₂/LPG/propane/smoke) |
| 37 | u16 | mq4_raw | ADC counts (CH₄) |
| 39 | u16 | soil_raw | ADC counts |
| 41 | u16 | rain_raw | ADC counts |
| 43 | u8 | proximity | 1 = metal detected |
| 44 | i32 | x | mm (map frame) |
| 48 | i32 | y | mm (map frame) |
| 52 | i16 | heading | 0.01 deg |
| 54 | u16 | range_min | mm (closest ultrasonic) |

## 8. Defects in the 2023 implementation fixed by v2.0

| # | 2023 behaviour | Effect | v2.0 rule |
|---|----------------|--------|-----------|
| D1 | Arduino `SSP_P_Con` appended the CRC high byte first, LabVIEW and the Python brain low byte first | Arduino-generated frames fail the GS CRC check and vice-versa | CRC low byte first everywhere (§2) |
| D2 | Arduino `loop()` ignored the return value of `Exetreaction()` and dispatched on the raw buffer | Corrupted or misaligned frames executed as commands | Dispatch only CRC-valid frames (§3) |
| D3 | Fixed 8-byte `readBytes()` after `available()>5` and a 100 ms delay; buffer flushed with `Serial1.begin()` | A single lost byte misaligns all following frames | Streaming length-driven parser with resync (§3) |
| D4 | Response built with `SSP_P_Con(src=req.src, dest=req.dest)` | Replies addressed to the rover itself | Addresses swapped in every response (§4) |
| D5 | `malloc()` on every command, never `free()`d (8 KB SRAM) | Heap exhaustion after a few hundred commands | Static buffers only |
| D6 | GET_TLM answered with a hard-coded 15-byte array | Telemetry did not reflect sensors | TLM built from live data (§7) |
| D7 | `Exetreaction` returns `-1` from an `unsigned char` function and has a path with no return | Undefined behaviour | Typed status codes |
| D8 | `distance=(unsigned char)(duration*0.034)/2` and `pulseIn()` without timeout | Distances > 255 cm wrap; loop blocks up to 1 s with no echo | 16-bit mm ranges, 25 ms echo timeout, non-blocking scheduler |
| D9 | `UlTRASONIC1_start()` wrote both `TLM[3]` and `TLM[4]` | Sensor 2 reading overwritten | One slot per sensor |
| D10 | `MOTORS_rotate_*()` changed only one side's direction pins | Turn behaviour depended on previous state | Explicit per-side commands, velocity interface |
| D11 | No command watchdog | Rover keeps driving if the link drops | `SET_VELOCITY` timeout (default 500 ms) + e-stop |
| D12 | Python `packetconst()` did not reset loop index `i` | Worked only because `list.insert` past the end appends | Re-implemented (`rover_core.ssp`) with tests |

## 9. Physical parameters (shared by firmware, URDF, simulator and analysis)

### 9.1 Vehicle

| Parameter | Value | Source |
|-----------|-------|--------|
| Overall L × W × H | 1.02 × 0.73 × 0.79 m | final presentation |
| Mass | 36.5 kg | mass budget (report Table 1) |
| Drive wheels | 6 (+6 passive twin wheels) | report §Challenges |
| Wheel diameter | 0.200 m (solid rubber, plastic rim) | report |
| Wheelbase (front–rear axle) | ≈ 0.82 m | estimated: L − wheel Ø |
| Track width (wheel-pair centres) | ≈ 0.58 m | estimated from photographs |
| Effective skid-steer track `b_eff` | χ · track, χ ≈ 1.5 (calibrate) | Mandow et al. 2007 |
| LiDAR height | ≈ 0.62 m (mast) | navigation video |
| Ultrasonic height | ≈ 0.25 m | report (arm-elongation brackets) |

### 9.2 Drive train

| Parameter | Value |
|-----------|-------|
| Motor | SG775125000-60K brushed DC gear-motor, 12 V |
| Gear ratio | 60 : 1 |
| No-load speed / current | 83 rpm / 0.5 A |
| Rated (loaded) speed / torque / current | 63 rpm / 2.97 N·m (28.5 kgf·cm) / 4 A |
| Stall current | 14 A |
| Encoder | 2-channel Hall, 7 PPR on the motor shaft → 420 pulses per wheel rev per channel |
| Counting mode (v2 FW) | channel A CHANGE interrupts → **840 counts / wheel rev** → 0.748 mm / count |
| Motor driver | 3 × Cytron MDD20A (dual channel, 6–30 V, 20 A cont., sign-magnitude PWM+DIR) |

### 9.3 Mobility MCU pin map (Arduino Mega 2560)

| Function | Pins |
|----------|------|
| Encoder A (interrupt) L-front, L-mid, L-rear, R-front, R-mid, R-rear | 2, 3, 18, 19, 20, 21 |
| Encoder B (direction) same order | 22, 23, 24, 25, 26, 27 |
| Motor PWM same order | 5, 6, 7, 8, 11, 12 |
| Motor DIR same order | 28, 29, 30, 31, 32, 33 |
| Ultrasonic TRIG: front-left, front-centre, front-right, left, right, rear | 34, 36, 38, 40, 42, 44 |
| Ultrasonic ECHO (PCINT16–21, port K) same order | A8, A9, A10, A11, A12, A13 |
| HC-12 (safe-mode link) TX2/RX2, SET | 16, 17, 48 |
| E-stop input (NC switch to GND, INPUT_PULLUP; HIGH = stop) | 49 |
| Battery sense (100 kΩ / 22 kΩ divider) | A0 |
| Status LED | 13 |

Ultrasonic mounting (x forward, y left, yaw, in `base_link`):
front-left (0.50, 0.18, +30°), front-centre (0.52, 0, 0°), front-right
(0.50, −0.18, −30°), left (0.10, 0.33, +90°), right (0.10, −0.33, −90°),
rear (−0.50, 0, 180°). Field of view 15° (HC-SR04), range 0.02–4.0 m.

### 9.4 Payload MCU pin map (Arduino Uno)

| Sensor | Interface | Pin |
|--------|-----------|-----|
| DHT11 temperature / humidity | 1-wire | D2 |
| Inductive proximity PR08-2DN (NPN, NO) | digital, INPUT_PULLUP, LOW = metal | D3 |
| MQ-2 (H₂, LPG, propane, smoke) | analog | A0 |
| MQ-4 (CH₄) | analog | A1 |
| Soil moisture (LM393 module, AO) | analog | A2 |
| Rain sensor (AO) | analog | A3 |
| BMP180 (0x77), MPU-6050 (0x68) | I²C | A4 SDA / A5 SCL |

### 9.5 Arm MCU (Arduino Uno + PCA9685 @ 0x40, 50 Hz)

| Channel | Joint | Limits (deg) | Home (deg) |
|---------|-------|--------------|------------|
| 0 | base yaw | 0–180 | 90 |
| 1 | shoulder | 15–165 | 90 |
| 2 | elbow | 0–180 | 90 |
| 3 | wrist pitch | 0–180 | 90 |
| 4 | wrist roll | 0–180 | 90 |
| 5 | gripper | 10–80 (10 = open) | 10 |

Pulse mapping: 0° → 102 counts (0.5 ms), 180° → 512 counts (2.5 ms) at 50 Hz
(the 2023 code used 125/575 counts at 60 Hz, i.e. 0.51/2.34 ms — equivalent).
MG996R servos, 6 V supply from a dedicated 5 A buck converter.

## 10. ROS 2 interface (Jazzy)

| Topic / action | Type | Producer → consumer |
|----------------|------|---------------------|
| `/cmd_vel` | geometry_msgs/Twist (TwistStamped when `use_stamped:=true`) | Nav2 / teleop → `mobility_bridge` |
| `/wheel/odom` | nav_msgs/Odometry | `mobility_bridge` → EKF |
| `/odometry/filtered` + TF `odom→base_footprint` | nav_msgs/Odometry | `robot_localization` EKF |
| `/imu/data` | sensor_msgs/Imu | `payload_bridge` |
| `/ultrasonic/{front_left,front_center,front_right,left,right,rear}` | sensor_msgs/Range | `mobility_bridge` → costmap `range_layer` |
| `/scan` | sensor_msgs/LaserScan | `hls_lfcd_lds_driver` (real) or `rover_sim` |
| `/environment` | rover_interfaces/EnvironmentTelemetry | `payload_bridge` |
| `/arm/joint_states`, `/arm/command` | sensor_msgs/JointState, rover_interfaces/ArmCommand | `arm_bridge` |
| `/rover/mode` | std_msgs/UInt8 | `ground_station_bridge` |
| `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | `explorer` → Nav2 |

Frames: `map → odom → base_footprint → base_link → {laser_link, imu_link,
camera_link, us_*_link, wheel links}`.
