# MISSION X rover firmware (ICD v2.0)

Embedded firmware for the MISSION X six-wheel rocker-bogie rover (Beni-Suef
University / Egyptian Space Agency). The normative interface is
[`docs/ICD.md`](../docs/ICD.md); the Python reference implementation of the
protocol is `ros2_ws/src/rover_core` (`crc.py`, `ssp.py`, `protocol.py`). For
operating the rover, see the [operator manual](../docs/OPERATOR_MANUAL.md).

```
firmware/
├── libraries/MissionX_SSP/   SSP v2.0 Arduino library (CRC, encoder, decoder, payloads)
├── mobility_mega/            Arduino Mega 2560, address 0x02 (drive, odometry, ultrasonics, safe mode)
├── payload_uno/              Arduino Uno, address 0x03 (DHT11, BMP180, MPU-6050, gas, soil, rain, metal)
├── arm_uno/                  Arduino Uno + PCA9685, address 0x04 (6-DOF arm)
├── legacy_fixed/             2023 single-MCU rover, same wiring, defects D1-D12 fixed (LabVIEW GS)
├── tests/                    host unit tests (g++) + cross-implementation vectors (Python oracle)
├── build_all.sh              compiles all sketches with arduino-cli, prints flash/RAM
└── build/                    build output (git-ignored)
```

## 1. Architecture

```
 Ground station ──HC-12 433 MHz (L1, 9600)──> Raspberry Pi "Brain" (0x01, ROS 2)
                                                 │ USB 115200 (L2)   │ USB (L3)      │ USB (L4)
                                                 v                   v               v
                                         Mobility Mega (0x02)   Payload Uno (0x03)  Arm Uno (0x04)
 Ground station ──HC-12 (L1-safe, Brain absent)──> Mega Serial2 ("safe mode")
```

Every link carries SSP frames
`C0 | DEST | SRC | TYPE | LEN | DATA | CRC_L | CRC_H | C0` (CRC-16/XMODEM over
DEST..DATA, low byte first, transparency OFF).

**MissionX_SSP** (`libraries/MissionX_SSP/src`)

| File | Contents |
|------|----------|
| `ssp_core.h/.cpp` | CRC-16/XMODEM (PROGMEM table, or bitwise with `-DSSP_CRC_BITWISE`), `ssp_encode()` into a caller buffer / `ssp_encode_to()` into a byte sink (optional SLIP transparency), `SspDecoder` (streaming, length-driven, ICD §3 incl. step 5a length check, inter-byte timeout, re-scan of the buffered bytes after a failed opening flag, address filter with optional alias address, statistics), little-endian pack/unpack helpers and `SspWriter`/`SspReader`. No `Arduino.h`, no heap. |
| `ssp_protocol.h/.cpp` | Addresses, packet types, NACK reasons, fault bits, poses, payload lengths, `ssp_plausible_length()` (= `_ALLOWED_LEN` of `protocol.py`), pack/unpack of every ICD payload (TLM v2 56 B, legacy 7 B, ENV_TLM, IMU, ...). |
| `MissionX_SSP.h/.cpp` | Umbrella header + `SspLink` (Arduino only): decoder bound to a `Stream`, replies with swapped addresses, ACK/NACK/echo helpers, frames streamed into the UART without a TX buffer, non-blocking `send(..., may_drop=true)` for periodic telemetry. |

The decoder is a line-by-line port of `rover_core.ssp.StreamDecoder`; the
cross-implementation test proves identical output **and identical
statistics** on 192 corrupted streams. The bytes that have to be re-scanned
after a framing/CRC error are replayed in place inside the single 262-byte
receive buffer (no extra RAM). `SspLink` evaluates the inter-byte timeout with
the *lower bound* of the real gap (bytes are only seen when `poll()` runs), so a
slow `loop()` pass can never make it drop a valid frame.

**Common rules in all sketches.** Static memory only; no `delay()`/`pulseIn()`
in the main loops (exception: the DFRobot DHT11 library, see §6); periodic
jobs on fixed time grids with different phases; every reply swaps addresses;
NACK reasons per ICD §6.2; no reply (not even a NACK) to ACK/NACK/telemetry
frames or to broadcast frames of unknown type; MCU decoders use the
non-strict length check so unknown types still get `NACK(2)`.

## 2. Boards, wiring and behaviour

### 2.1 Mobility — Arduino Mega 2560, address 0x02 (`mobility_mega/`)

| Function | Pins (order L-front, L-mid, L-rear, R-front, R-mid, R-rear) |
|----------|------|
| Encoder A (CHANGE interrupt, 840 counts/wheel rev) | 2, 3, 18, 19, 20, 21 |
| Encoder B (direction) | 22, 23, 24, 25, 26, 27 |
| Motor PWM (3 x Cytron MDD20A, sign-magnitude, 3.9 kHz) | 5, 6, 7, 8, 11, 12 |
| Motor DIR | 28, 29, 30, 31, 32, 33 |
| Ultrasonic TRIG (FL, FC, FR, L, R, rear) | 34, 36, 38, 40, 42, 44 |
| Ultrasonic ECHO (PCINT16-21, port K) | A8, A9, A10, A11, A12, A13 |
| HC-12 TX2/RX2, SET (driven HIGH = transparent mode) | 16, 17, 48 |
| E-stop input: NC switch to GND, INPUT_PULLUP, HIGH = stop | 49 |
| Battery sense, 100 kΩ / 22 kΩ divider | A0 |
| Status LED (solid = e-stop, 4 Hz = fault, 1 Hz blip = OK) | 13 |

* `Serial` 115200 = Brain (L2): SET_VELOCITY in; WHEEL_ODOM 20 Hz, RANGES 10 Hz,
  HEARTBEAT 1 Hz out (DEST 0x01). `Serial2` 9600 = HC-12 safe-mode link (L1-safe):
  replies only. Both links accept the same commands, replies go back on the
  link the request came from. On the HC-12 link the Mega accepts DEST
  **0x02, 0xFF and 0x01** (the legacy LabVIEW GS always addresses the rover as
  0x01); frames sent to 0x01 are answered with **SRC 0x01**, all others with
  SRC 0x02 (`SspLink::setAlias()`). The USB link accepts only 0x02 / 0xFF.

  | GET_TLM received on | DEST | Reply (SRC) | Layout |
  |---------------------|------|-------------|--------|
  | Serial2 (HC-12) | 0x01 (LabVIEW GS) | 0x01 | **legacy 7 B** (ICD §7.1): ir 0, gas 0/0, us1_cm = front-centre, us2_cm = min(front-left, front-right), temp 0, rh 0; cm rounded, > 255 cm or no echo → 255 (`legacy_tlm.cpp`) |
  | Serial2 (HC-12) | 0x02 / 0xFF | 0x02 | v2 56 B, fault bit4 set |
  | Serial (USB, Brain) | 0x02 / 0xFF | 0x02 | v2 56 B |
* **Drive control (50 Hz).** Body twist → skid steer `v_L = v − ω·b_eff/2`,
  `v_R = v + ω·b_eff/2` (`b_eff = χ·0.58 m`, χ = 1.5) → |v| ≤ 0.45 m/s,
  |ω| ≤ 1 rad/s, |side| ≤ 0.70 m/s (scaled, curvature kept) → slew limit
  0.8 m/s² (1.5 m/s² when stopping) → per side: feed-forward
  (static offset + speed/V_no-load, battery-voltage compensated, plus
  acceleration term) + PI on the mean speed of the healthy encoders of that
  side; anti-windup = clamp + conditional integration (not into saturation,
  not while the setpoint ramps). The same duty drives the three motors of a side.
* **SET_VELOCITY** watchdog: stop if not refreshed within `timeout_ms`
  (0 → 500 ms) and set fault bit1. **DRIVE**: open loop (feed-forward only),
  `±100 %` of 0.45 m/s / 1 rad/s, `duration_ms` (0 → 500 ms, capped at 5 s),
  ACK(0x60).
* **ESTOP** (ACK 0x51) or hardware input → PWM 0 on all channels at once,
  latched (bit0). **CLEAR_ESTOP** → ACK(0x52), or NACK(0x52, 4) while the
  switch is still open. Motion commands while latched → NACK(4)
  (SET_VELOCITY: at most one NACK per second).
* **Encoder fault** (bit3): a wheel driven with |duty| ≥ 20 % and no tick for
  1 s. It is excluded from the side average; if a whole side is faulty that
  side runs open loop.
* **Battery**: low-battery bit2 below 10.8 V for 2 s (cleared above 11.1 V).
* **Ultrasonics**: one sensor at a time, round robin, fixed 25 ms slot = echo
  timeout (no cross-talk, each sensor ≈ 6.7 Hz), echo timed by the PCINT2 ISR;
  no echo / > 4 m → 0xFFFF; RANGES (10 Hz) carries the latest values.
* **GET_TLM** to 0x02/0xFF (either link) → v2 56-byte layout: version, mode, faults, battery,
  uptime, **x/y/heading/yaw from wheel-odometry dead reckoning since power-up**
  (not map frame), range_min = closest ultrasonic; every other field 0.
  Fault bit4 is set while a valid frame was received on Serial2 in the last 60 s.
* **START_SCAN** (echo; mode AUTONOMOUS) — the 2023 obstacle avoidance as a
  non-blocking state machine (`scan_logic.cpp`, threshold 0.6 m, hysteresis
  0.7 m): forward 0.25 m/s when clear; obstacle front-left → rotate right;
  front-right → rotate left; front-centre or both corners → back up 1.2 s
  (skipped if the rear is blocked) then rotate toward the freer side
  (larger diagonal + side range); all three front + both side sensors blocked →
  stop and keep watching; > 12 s of rotation without a gap → stop.
  SET_VELOCITY / DRIVE / STOP_SCAN / ESTOP cancel it. **STOP_SCAN** → echo,
  ramped stop, mode IDLE.
* Mode in HEARTBEAT/TLM: AUTONOMOUS (scan), TELEOP (SET_VELOCITY/DRIVE active), else IDLE.

### 2.2 Payload — Arduino Uno, address 0x03 (`payload_uno/`)

| Sensor | Interface | Pin |
|--------|-----------|-----|
| DHT11 (DFRobot_DHT11 library) | 1-wire | D2 |
| Inductive proximity PR08-2DN (NPN NO) | INPUT_PULLUP, LOW = metal | D3 |
| MQ-2 / MQ-4 / soil / rain (×8 oversampling) | analog | A0 / A1 / A2 / A3 |
| BMP180 (0x77, oss 3) and MPU-6050 (0x68) | I²C 100 kHz | A4 SDA / A5 SCL |

* Out (DEST 0x01): ENV_TLM 1 Hz, IMU 25 Hz, HEARTBEAT 1 Hz. In: PING → ACK.
* ENV_TLM `status`: bit0 DHT ok, bit1 BMP ok, bit2 MPU ok (after gyro
  calibration), bit3 60 s warm-up done. Values of a failed sensor are sent as 0.
  `proximity` = 1 if metal was seen at any time since the previous ENV_TLM.
* BMP180: chip-id check, calibration EEPROM, Bosch integer compensation
  (`bmp180_comp.cpp`, verified against the datasheet example: 15.0 °C,
  69964 Pa), non-blocking T→P state machine, re-initialised every 5 s after an error.
* MPU-6050: wake (PLL), ±4 g, ±500 °/s, 10 Hz DLPF (no aliasing at 25 Hz),
  gyro bias = mean of 50 samples (2 s) at start-up — **keep the rover still for
  2 s after power-up** (repeated if it moved, up to 5 times); complementary
  filter α = 0.98 for roll/pitch. Raw chip axes are reported (the URDF
  `imu_link` must match the mounting). The ICD field `i16 mm/s²` saturates at
  ±3.34 g although the sensor range is ±4 g.
* All I²C calls are bounded by `Wire.setWireTimeout(5 ms, reset)`.
* HEARTBEAT `fault_flags` (Payload): bit0 DHT11, bit1 BMP180, bit2 MPU-6050 not working.

### 2.3 Arm — Arduino Uno + PCA9685 @ 0x40, 50 Hz, address 0x04 (`arm_uno/`)

| Ch | Joint | Limits (deg) | Home |
|----|-------|--------------|------|
| 0 | base yaw | 0–180 | 90 |
| 1 | shoulder | 15–165 | 90 |
| 2 | elbow | 0–180 | 90 |
| 3 | wrist pitch | 0–180 | 90 |
| 4 | wrist roll | 0–180 | 90 |
| 5 | gripper | 10–80 (10 = open) | 10 |

Pulse mapping 0° → 102 counts (0.5 ms), 180° → 512 counts (2.5 ms). MG996R
servos on a dedicated 6 V / 5 A supply (common GND with the Uno).

* **ARM_SET_JOINTS** → ACK(0x40). Policy for limit violations: the target is
  **clamped and executed, and the reply is NACK(0x40, 4)** so the Brain knows
  it did not get exactly what it asked for (ARM_STATE shows the result).
* Motion = minimum-jerk interpolation (`10τ³ − 15τ⁴ + 6τ⁵`) from the current
  commanded angles, sampled every 20 ms; `duration_ms` is stretched if the
  peak joint speed (1.875·Δq/T) would exceed 120 °/s. A new command pre-empts
  the running motion/sequence.
* **ARM_SET_POSE** → ACK(0x41) (NACK(4) for an id > 7). **COLLECT_OBJECT** →
  echo, then READY → GRIP_OPEN → REACH_GROUND → GRIP_CLOSE → LIFT_SAMPLE →
  DEPOSIT → STOW with 300 ms pauses (NACK(3) if already running). DEPOSIT moves
  over the sample box with the gripper closed, then opens it.
* ESTOP/CLEAR_ESTOP (e.g. broadcast): freeze in place (servos keep holding),
  motion commands → NACK(4) until cleared. PCA9685 not answering → NACK(5),
  re-initialised every 5 s. At power-up the servos jump to HOME — keep clear.
* Out: ARM_STATE 5 Hz + immediately on start/end of a motion (`moving` = 1
  while a motion or sequence runs), HEARTBEAT 1 Hz (`fault_flags`: bit0 e-stop,
  bit1 PCA9685 fault; mode AUTONOMOUS during COLLECT_OBJECT).

Named poses (`arm_math.cpp`; K = joint unchanged). **Starting values — tune on
the real arm** (assumed convention: shoulder 90 = upper arm vertical,
< 90 leans forward; elbow 90 = right angle; wrist pitch 90 = in line):

| id | Pose | base | shoulder | elbow | wrist P | wrist R | gripper | T [ms] | Purpose |
|----|------|------|----------|-------|---------|---------|---------|--------|---------|
| 0 | HOME | 90 | 90 | 90 | 90 | 90 | 10 | 1500 | ICD home, gripper open |
| 1 | STOW | 90 | 160 | 15 | 30 | 90 | 60 | 2000 | folded back over the deck for driving |
| 2 | READY | 90 | 110 | 60 | 120 | 90 | K | 1500 | raised in front of the rover |
| 3 | REACH_GROUND | 90 | 40 | 140 | 50 | 90 | K | 2000 | gripper at ground level in front |
| 4 | GRIP_OPEN | K | K | K | K | K | 10 | 600 | open gripper |
| 5 | GRIP_CLOSE | K | K | K | K | K | 75 | 800 | close on the sample (75, not 80, to limit stall current) |
| 6 | LIFT_SAMPLE | 90 | 110 | 70 | 110 | 90 | K | 1500 | lift the sample clear of the ground |
| 7 | DEPOSIT | 170 | 120 | 60 | 60 | 90 | K → 10 | 2000 | over the sample box, then release |

### 2.4 Legacy single-MCU rover — Arduino Mega (`legacy_fixed/`)

Same wiring as the 2023 "Rover Final Code": L298N IN1–IN4 = 2, 3, 4, 5
(motor 1 = left, motor 2 = right), ENA 8, ENB 9; IR 53 + LED 13; gas A0;
ultrasonic 1 echo/trig 22/23, ultrasonic 2 echo/trig 24/25; DHT11 26;
HC-12 on `Serial1` 9600; rover 0x01, GS 0x0B. The same commands are also
accepted on the USB port (115200) for bench tests.

* GET_TLM → legacy 7-byte layout from live data: IR, gas (hi, lo), US1 cm,
  US2 cm (each in its own slot; no echo or > 255 cm → 255), DHT11 °C, %RH.
* PING → ACK; CAP_IMAGE/GET_IMAGE → echo (no camera on this MCU, as in 2023);
  START_SCAN/STOP_SCAN → echo; scan = 2023 behaviour (forward while ultrasonic 1
  sees > 80 cm, otherwise rotate right in place). v2 additions: SET_VELOCITY
  (open loop, watchdog), DRIVE, ESTOP, CLEAR_ESTOP. The scan stops if no frame
  has been received from the GS for 60 s (`SCAN_LINK_TIMEOUT_MS`).
* Echo pins 22/24 have no pin-change interrupt: the echo is timed by polling
  from `loop()`, and a reading is discarded if `loop()` was stalled when an
  edge was detected. The DHT11 is read every 5 s (at most every 30 s while driving).

## 3. Build and flash

Prerequisites (already installed on the team PC): Arduino AVR core 1.8.8
(`Wire.setWireTimeout` needs ≥ 1.8.3), libraries `DFRobot_DHT11` and
`Adafruit PWM Servo Driver Library` (+ `Adafruit BusIO`).

**arduino-cli** (from `firmware/`, Git Bash):

```bash
CLI="/d/Library/3-Programs/arduino-cli/arduino-cli_1.4.1_Windows_64bit/arduino-cli.exe"
FW="$(pwd -W)"                                # absolute path with forward slashes
./build_all.sh                                # all four sketches + size table (--clean = full rebuild)
"$CLI" compile --fqbn arduino:avr:mega --library "$FW/libraries/MissionX_SSP" \
    --build-path "$FW/build/mobility_mega" "$FW/mobility_mega"
"$CLI" board list                             # find the COM port
"$CLI" upload -p COM5 --fqbn arduino:avr:mega --input-dir "$FW/build/mobility_mega" "$FW/mobility_mega"
```

Uno sketches: `--fqbn arduino:avr:uno` (`payload_uno`, `arm_uno`). `legacy_fixed`: `arduino:avr:mega`.

**Arduino IDE**: copy (or link) `firmware/libraries/MissionX_SSP` into the
sketchbook `libraries` folder (`Documents/Arduino/libraries/`), restart the IDE,
open e.g. `mobility_mega/mobility_mega.ino` (all `.h/.cpp` files of the folder
open as tabs), select *Arduino Mega or Mega 2560* / *Arduino Uno*, the port, Upload.

**Host tests** (g++ and Python 3 on PATH): `tests/run_tests.sh`.
`gen_vectors.py` imports `ros2_ws/src/rover_core` and writes CRCs, encoded
frames, the length table, 192 random streams with byte drops / bit flips /
garbage / truncations (+ the frames and statistics the Python decoder
produces) and 480 random payloads; `test_ssp` (built with the CRC table and
with `-DSSP_CRC_BITWISE`) must reproduce all of it. `test_logic` checks the
drive math and PI (simulated wheel with friction and encoder quantisation),
the obstacle-avoidance state machine, the Mega's legacy 7-byte TLM for the
LabVIEW GS, the BMP180 compensation, IMU math and the arm trajectory/pose table.

## 4. Measured size (arduino-cli 1.4.1, AVR core 1.8.8, `--warnings all`: 0 warnings in our code)

| Sketch | Board | Flash | RAM (globals) | Free RAM for stack |
|--------|-------|-------|---------------|--------------------|
| mobility_mega | Mega 2560 | 22 122 B (8 %) | 1 469 B (17 %) | 6 723 B |
| payload_uno | Uno | 16 454 B (51 %) | 926 B (45 %) | 1 122 B |
| arm_uno | Uno | 14 684 B (45 %) | 971 B (47 %) | 1 077 B |
| legacy_fixed | Mega 2560 | 12 558 B (4 %) | 1 117 B (13 %) | 7 075 B |

(One `SspLink` costs ≈ 350 B RAM, mostly the 262-byte receive buffer and
32 bytes of statistics; the CRC table is in flash. The rest of the Uno RAM is
the core's serial (≈ 160 B) and Wire (≈ 200 B) buffers.)

## 5. Bench-test checklist

Python snippet for any link (needs `pyserial`; the board resets when the port opens):

```python
import sys, time, serial
sys.path.insert(0, "ros2_ws/src/rover_core")
from rover_core.ssp import StreamDecoder, encode
from rover_core.protocol import PT, Addr, SetVelocity, plausible_length
ser = serial.Serial("COM5", 115200, timeout=0.05); time.sleep(2.0)
dec = StreamDecoder(length_check=plausible_length)
ser.write(encode(Addr.MOBILITY, Addr.ROVER, PT.PING))
t0 = time.time()
while time.time() - t0 < 2:
    for f in dec.feed(ser.read(256)): print(f)
```

Mobility (wheels off the ground, e-stop switch wired or pin 49 jumpered to GND):
- [ ] LED blips at 1 Hz; HEARTBEAT 1 Hz, WHEEL_ODOM 20 Hz, RANGES 10 Hz arrive; PING → ACK (LEN 0).
- [ ] A frame with a corrupted CRC gets no reply; an unknown type gets NACK(type, 2); GET_TLM with LEN ≠ 0 gets NACK(1).
- [ ] Encoder sign check (§6.1) passes for all six wheels.
- [ ] SET_VELOCITY v = 0.2 m/s every 100 ms: all wheels turn forward; stop sending → wheels stop within ~0.5 s and fault bit1 is set; next command clears it.
- [ ] ESTOP → ACK(0x51 in DATA), wheels stop instantly, bit0 set, SET_VELOCITY → NACK(4) (≤ 1/s); CLEAR_ESTOP → ACK(0x52).
- [ ] Open the e-stop switch → instant stop + bit0; CLEAR_ESTOP while open → NACK(4).
- [ ] Hold one wheel while commanding motion for > 1 s → bit3; release → clears.
- [ ] Battery voltage in GET_TLM matches a multimeter (±0.1 V); below 10.8 V → bit2 after 2 s.
- [ ] Each ultrasonic (§6.3); START_SCAN on the floor: avoids a box, STOP_SCAN stops.
- [ ] HC-12 on Serial2: PING to 0x02 → ACK from 0x02; PING to 0x01 (LabVIEW) → ACK from 0x01 (`C0 0B 01 01 00 1E 1A C0`); LabVIEW GET_TLM (`C0 01 0B 0F 00 7B 96 C0`) → 7-byte reply from 0x01 showing the front ultrasonic distances in cm; GET_TLM to 0x02 → 56-byte reply with fault bit4; DRIVE v=30 % for 1000 ms → drives 1 s, ACK(0x60). The same PING to 0x01 on USB gets no reply.

Payload: ENV_TLM 1 Hz with status bits 0-2 set within ~5 s and bit3 after 60 s;
IMU 25 Hz after the 2 s calibration (roll/pitch ≈ 0 when level, follow a tilt);
unplug the BMP180/MPU-6050 → status bit clears, IMU stops, loop keeps running,
re-plug → recovers within 5 s; metal in front of the proximity sensor → `proximity` = 1.

Arm (servo power on, arm clear): power-up → HOME; ARM_SET_POSE 1..7 each
reach the pose smoothly (ACK); ARM_SET_JOINTS with shoulder = 5° → NACK(4) and
the arm goes to 15°; COLLECT_OBJECT → echo + full sequence, ARM_STATE `moving` = 1
until STOW; second COLLECT_OBJECT meanwhile → NACK(3); ESTOP freezes the arm.

Legacy: LabVIEW GS PING / GET_TLM (7 B, live values) / START_SCAN / STOP_SCAN work over the HC-12.

## 6. Calibration procedures

### 6.1 Encoder and motor sign check
1. Rover on blocks, wheels free. Watch WHEEL_ODOM.
2. Turn each wheel by hand in the direction that would move the rover
   **forward**: its tick count must increase. If it decreases, flip that
   wheel's entry in `ENC_SIGN` (`mobility_mega/config.h`).
3. Send `DRIVE v_pct=+20, w_pct=0, duration_ms=1000`: every wheel must turn
   forward. If one turns backwards, flip its `MOTOR_SIGN` entry (or swap the
   motor leads). Ticks must increase on all six wheels.
4. One wheel revolution must give ≈ 840 counts (mark the tyre, turn by hand).

### 6.2 Effective track χ (skid-steer calibration by a commanded 360° spin)
1. Hard, flat ground (repeat later on the target terrain - χ depends on it).
   Put a tape mark on the floor aligned with the rover's nose.
2. Record WHEEL_ODOM ticks, command a pure rotation
   (`SET_VELOCITY v=0, w=400 mrad/s`, refreshed every 100 ms) and stop exactly
   when the rover has turned **N = 1** (better 2–3) full turns back onto the mark.
3. With the mean tick change per side `ΔL`, `ΔR` (counts, signed) and
   0.748 mm/count: `d_L = ΔL·0.000748`, `d_R = ΔR·0.000748`,
   `b_eff = (d_R − d_L) / (2π·N)`, `χ = b_eff / 0.58`.
4. Put χ into `SKID_CHI` (`mobility_mega/config.h`) and into the Brain's
   `skid_chi` parameter (`rover_core/params.py` / ROS parameter) - both must agree.
5. Check: GET_TLM `heading` after a commanded 360° spin returns to ≈ 0.

### 6.3 Ultrasonic check
For every sensor: flat board at 0.30, 0.60, 1.00 and 2.00 m on its axis →
RANGES within ±2 cm (sound speed fixed at 343 m/s, ≈ +0.17 %/°C error).
Nothing within 4 m → 0xFFFF. Two sensors facing the same wall must not
disturb each other (one sensor at a time, 25 ms slots).

### 6.4 MQ-2 / MQ-4 warm-up
New sensors need 24–48 h burn-in powered (datasheet). After every power-up
the heaters need time: ENV_TLM status bit3 is set after 60 s (ICD), but wait
3–5 min before trusting absolute values. Record the clean-air baseline
(`mq2_raw`, `mq4_raw`) after warm-up at each site; report gas as change from baseline.

### 6.5 Other
* Battery: `BATT_UV_PER_COUNT = 1000·V_multimeter / ADC_count` (default 27106 µV/count).
* PCA9685 oscillator: measure the pulse of a servo at 90° (should be 1.500 ms);
  `PCA_OSC_HZ = 27e6 · 1.500 / measured_ms` (`arm_uno/config.h`).
* Arm poses: move joint by joint with ARM_SET_JOINTS (long durations), note the
  angles, update `kPoses` in `arm_uno/arm_math.cpp` and the table in §2.3.
* Wheel-speed PI (`config.h`): start with the defaults (Kp 1.0, Ki 3.0,
  Ka 0.15, FF static 0.06); log WHEEL_ODOM during 0 → 0.3 m/s steps on the
  ground; raise `FF_STATIC_DUTY` if small speeds stall, lower Kp if the wheels oscillate.

## 7. 2023 defect → fix

| # | 2023 behaviour | Fix in this firmware |
|---|----------------|----------------------|
| D1 | CRC high byte first in `SSP_P_Con` | `ssp_encode*()` sends CRC low byte first; ICD §2.1 vectors in `test_ssp` |
| D2 | Return value of `Exetreaction()` ignored, raw buffer dispatched | Handlers are only called by `SspDecoder` for CRC-valid frames |
| D3 | `readBytes(8)` after `available()>5`, 100 ms delay, `Serial1.begin()` flush | Streaming length-driven decoder with re-scan resync, timeout, length check; no delays |
| D4 | Reply built with src/dest of the request | `SspLink::reply()` sets DEST = req.SRC, SRC = own address |
| D5 | `malloc()` per command, never freed | No heap use in our code (Adafruit's PCA9685 driver allocates one small object in `begin()`) |
| D6 | GET_TLM answered with a hard-coded array | TLM built from live sensors (legacy 7 B / v2 56 B) |
| D7 | `unsigned char` function returning −1 / missing return | Typed status (`bool`, enums), `-Wall` clean |
| D8 | `(unsigned char)(duration*0.034)/2`, `pulseIn()` without timeout | 16-bit mm ranges, 25 ms timeout, PCINT (Mega) or polled (legacy) echo timing, non-blocking |
| D9 | `UlTRASONIC1_start()` wrote TLM[3] and TLM[4] | One slot per sensor |
| D10 | Rotate functions changed one side only | `setSides()`/per-side control always drives both sides explicitly |
| D11 | No command watchdog | SET_VELOCITY timeout (default 500 ms), DRIVE duration, ESTOP latch + hardware input; legacy scan stops after 60 s without GS traffic |
| D12 | Python `packetconst()` loop index bug | Python side re-implemented in `rover_core.ssp`; C side cross-tested against it |

## 8. Interpretations where the ICD is silent

* ACK: PING → ACK with LEN 0 (ICD §6.1); every other ACK carries the acknowledged type (LEN 1).
* Mega HC-12 link: DEST 0x01 is accepted as an alias of 0x02 because the
  legacy LabVIEW GS cannot address 0x02; replies keep the address the GS used
  (SRC 0x01) and a GET_TLM to 0x01 returns the legacy 7-byte layout (only the
  ultrasonic fields carry data; the Mega has no IR, gas or DHT11 sensor).
* `fault_flags` bits are defined for Mobility only; Payload and Arm use the
  node-specific bits listed in §2.2/§2.3.
* SET_VELOCITY has no response; while the e-stop is latched it is answered
  with NACK(4) at most once per second.
* DRIVE `duration_ms = 0` → 500 ms; durations are capped at 5 s (blind radio driving).
* The Mega starts with the e-stop *not* latched unless the hardware input is open.
* Mega GET_TLM: wheel-odometry pose and closest ultrasonic range are filled in
  (they are available on the Mega); fields of other nodes are 0.
* DFRobot_DHT11 (required library) blocks ≈ 225 ms per read (≈ 2 s if the
  sensor does not answer) and exposes no checksum: it is read every 5 s
  (30 s after a failure), only plausible values are accepted, and `SspLink`'s
  stall-aware timeout prevents frame loss caused by the stall.

## 9. To be verified on the real rover (hardware team)

1. `ENC_SIGN` / `MOTOR_SIGN` (§6.1); encoder outputs are 5 V (pull-ups enabled; pins 20/21 also have the Mega's 10 kΩ I²C pull-ups).
2. E-stop wiring: NC switch between pin 49 and GND. **Without it the Mega latches the e-stop at boot by design.**
3. MDD20A: PWM 3.9 kHz accepted, PWM = 0 brakes; motor current within the driver rating at stall.
4. Battery divider values and `BATT_UV_PER_COUNT`; 10.8 V threshold suits the battery chemistry.
5. HC-12 configured 9600 baud / FU3 / channel 001 and its SET pin on 48.
6. HC-SR04 ECHO on A8–A13, TRIG on 34–44 (even pins), mounting angles (§9.3 ICD).
7. MPU-6050 orientation vs. URDF `imu_link`; BMP180 module address 0x77; DHT11 data pull-up present.
8. PCA9685 oscillator, servo zero/direction per joint, the arm pose table, servo supply ≥ 5 A.
9. χ (§6.2) and the PI gains on the real terrain.
10. Brain side: `mobility_bridge` addresses the Mega as 0x02 and refreshes SET_VELOCITY faster than its `timeout_ms`.
