# MISSION X Rover — Operator Manual

This manual tells an operator how to run a complete session with the MISSION X
rover: charging, checks, power-on, the ground station, driving, mapping,
navigation, autonomous exploration, science measurements, sample collection,
emergencies and shutdown. It also gives a troubleshooting guide and a quick
reference.

It does not repeat the engineering details. Those are in:

| Topic | Document |
|-------|----------|
| Interfaces, packet types, pin maps, physical parameters | [`ICD.md`](ICD.md) |
| Flashing the Arduinos, bench tests, calibration | [`../firmware/README.md`](../firmware/README.md) |
| Ground-station software in detail | [`../ground_station/README.md`](../ground_station/README.md) |
| Installing and running ROS 2 on the Raspberry Pi | [`ROS2.md`](ROS2.md) |

> **Status.** The procedures follow the firmware and ROS 2 software in this
> repository. At the time of writing, the new firmware has not yet been
> flashed to the rover and the ROS 2 stack has not yet run on it. Before the
> first field session, complete the one-time setup in §3 and the bench tests
> in `firmware/README.md` §5. Then replace the estimated values in this
> manual with measured ones.

## Contents

1. [Safety](#1-safety)
2. [The rover at a glance](#2-the-rover-at-a-glance)
3. [One-time setup](#3-one-time-setup)
4. [Batteries and charging](#4-batteries-and-charging)
5. [Before every session](#5-before-every-session)
6. [Power-on](#6-power-on)
7. [Ground station: connect and check](#7-ground-station-connect-and-check)
8. [Driving](#8-driving)
9. [Mapping (SLAM)](#9-mapping-slam)
10. [Navigating on a saved map](#10-navigating-on-a-saved-map)
11. [Autonomous exploration](#11-autonomous-exploration)
12. [Environment sensors and camera](#12-environment-sensors-and-camera)
13. [Sample collection and the arm](#13-sample-collection-and-the-arm)
14. [Emergency stop and link loss](#14-emergency-stop-and-link-loss)
15. [Safe mode: driving without the Raspberry Pi](#15-safe-mode-driving-without-the-raspberry-pi)
16. [Shutdown](#16-shutdown)
17. [Troubleshooting](#17-troubleshooting)
18. [Maintenance and calibration schedule](#18-maintenance-and-calibration-schedule)
19. [Quick reference](#19-quick-reference)
- [Appendix A: start ROS automatically at boot](#appendix-a-start-ros-automatically-at-boot)
- [Appendix B: recording data](#appendix-b-recording-data)

---

## 1. Safety

Read this section before every session. At least two people should be
present: an **operator** at the ground station and a **spotter** next to the
rover who can reach the e-stop switch.

**Lifting and handling**
- The rover weighs 36.5 kg. Lift it with two people.
- Lift by the chassis or the electronics box. Never lift by the rocker–bogie
  legs, the arm, the LiDAR mast or the solar panel.
- Avoid drops, jumps off steps and hard impacts. The 2023 analysis showed
  stresses in the legs above the strength of their aluminium, and the legs
  bent during integration.

**Emergency stop**
- There are two e-stops: the **hardware switch** on the rover and the
  **E-STOP button** in the ground station (also the `Space` key).
- The hardware switch is wired fail-safe. Pressing it, or a broken wire,
  stops all six motors at once.
- Before you power on, make sure both people know where the switch is.
- §14 explains how the e-stop works and how to clear it.

**Moving parts**
- The arm servos jump to their HOME position whenever the arm board
  starts. This happens at power-on and again when ROS starts, because the
  Arduinos reset when their USB ports open. Keep hands and faces away from
  the arm at those moments.
- The sample-collection sequence moves the arm on its own for about 15 s.
  Any e-stop stops it.
- The wheels can trap fingers between the wheels and the motor boxes. For
  bench tests, put the rover on blocks so that all wheels are off the
  ground.
- One motor draws up to 14 A when stalled, and all six together about 84 A.
  Do not hold the wheels or drive against a wall at full power.

**Batteries**
- The rover runs on three 12 V Li-ion packs. Charge them only with the solar
  charge controller or the packs' own charger.
- Never short the terminals. Do not use a pack that is swollen, hot or
  damaged.
- Stop driving and recharge when the ground station shows **low battery**:
  below 10.8 V for 2 s.

**Driving area**
- Keep people and animals out of the driving area, and keep the area clear
  during autonomous runs.
- The LiDAR scans a single plane 0.62 m above the ground and cannot see
  anything lower. The six ultrasonic sensors at 0.25 m cover low obstacles
  in front, at the sides and behind. Small or soft objects can still be
  missed, so never leave an autonomous run unwatched.
- **In autonomous mode the rover keeps exploring if the radio link drops**
  (§14). Stay within radio range and in sight of the rover.

**Terrain**
- The analysis gives 26.9° gradeability and 55 mm continuous step climbing
  at rated motor torque.
- These are calculated values; slopes and steps have not yet been tested
  on the real rover. Start with gentle slopes and small steps, and watch the
  battery voltage and the wheel behaviour.

---

## 2. The rover at a glance

```
 Ground station (laptop, Chrome/Edge)
   └─ USB-UART ─ HC-12 433 MHz ~~~~~~ radio ~~~~~~ HC-12 ─┐
                                                          │ USB
                                   Raspberry Pi 4 "Brain" (Ubuntu 24.04, ROS 2 Jazzy), address 0x01
                                    ├─ USB ─ Arduino Mega 2560 "Mobility" 0x02 ─ 6 motors, 6 encoders, 6 ultrasonics, e-stop
                                    ├─ USB ─ Arduino Uno "Payload" 0x03 ─ DHT11, BMP180, MPU-6050, MQ-2, MQ-4, soil, rain, metal
                                    ├─ USB ─ Arduino Uno "Arm" 0x04 ─ PCA9685 ─ 6 servos
                                    ├─ USB ─ LDS-01 LiDAR (on the mast)
                                    └─ camera
 Power: 3 × 12 V 15 Ah Li-ion ─ 12 V bus (motor drivers) ─ LM2596 bucks: 5 V logic, 6 V servos
        50 W solar panel ─ 10 A PWM charge controller ─ batteries
```

### 2.1 Operating modes

The rover reports its mode in every telemetry packet. The ground station
shows it in the header.

| Mode | Value | Meaning | How it is entered |
|------|-------|---------|-------------------|
| IDLE | 0 | No motion task active | At start-up, after STOP_SCAN, after an e-stop, and when exploration finishes |
| TELEOP | 1 | Driven by the operator | Automatically with the first drive command, or with *Set mode* |
| AUTONOMOUS | 2 | Frontier exploration (on the Pi), or the simple obstacle-avoidance scan (on the Mega in safe mode) | *Start scan*, or *Set mode* AUTONOMOUS; refused while the e-stop is latched |
| SAFE | 3 | Defined in the ICD. The current software gives it no special behaviour; selecting it stops exploration as IDLE does. | *Set mode* |

A drive command during exploration cancels the exploration and switches to
TELEOP. This is the quickest way to take over.

### 2.2 Radio configurations

| Configuration | HC-12 connected to | Use |
|---------------|--------------------|-----|
| **Normal** | Raspberry Pi (`/dev/rover_radio`) | All functions: telemetry, driving, SLAM, navigation, exploration, camera, arm |
| **Safe mode** | Mega `Serial2` (pins 16/17, SET 48) | The Pi is broken or absent: driving, simple obstacle-avoidance scan, basic telemetry (§15) |

### 2.3 Key numbers

| Item | Value |
|------|-------|
| Mass, size (L × W × H) | 36.5 kg, 1.02 × 0.73 × 0.79 m |
| Maximum speed | 0.45 m/s and 1.0 rad/s (Mega hard limit); radio driving and Nav2 use 0.40 m/s and 0.8 rad/s |
| Battery | 3 × 12 V 15 Ah Li-ion = 540 Wh (432 Wh usable at 80 % depth of discharge) |
| Low-battery warning | below 10.8 V for 2 s (clears above 11.1 V) |
| Endurance (calculated) | about 6.7 h on a typical sortie (65 W average); §4 has the figures by activity |
| Solar recharge (calculated) | about 186 Wh per sunny day in Beni-Suef, so about 2.3 days from empty |
| Radio | HC-12, 433.4 MHz (channel 001), FU3, 20 dBm, 9600 baud 8N1; range about 0.9 km with line of sight (calculated) |
| LiDAR | LDS-01, 0.12–3.5 m, 5 Hz, scan plane 0.62 m above the ground |
| Ultrasonic sensors | 6 × HC-SR04 at 0.25 m: front-left, front-centre, front-right, left, right, rear; 0.02–4 m, 15° beam |
| Image download | 100 × 100 grey (legacy) in about 11 s at 9600 baud; v2 JPEG 160 × 120 |

---

## 3. One-time setup

Do these once, and again after any hardware or software change. Each item
points to its detailed guide.

| # | Task | Guide |
|---|------|-------|
| 1 | Configure both HC-12 modules: channel 001, FU3, 20 dBm, 9600 baud | `ground_station/README.md` §1 ("HC-12 radio on a USB-UART adapter") |
| 2 | Flash the three Arduinos (`mobility_mega`, `payload_uno`, `arm_uno`) | `firmware/README.md` §3 |
| 3 | Run the bench-test checklist, with the rover on blocks | `firmware/README.md` §5 |
| 4 | Calibrate: wheel and motor directions, effective track χ, ultrasonic sensors, battery reading, servo oscillator, arm poses | `firmware/README.md` §6 |
| 5 | Install Ubuntu 24.04 and ROS 2 Jazzy on the Pi, install the udev rules (fill in the boards' serial numbers), build the workspace | `docs/ROS2.md` §3 |
| 6 | Run the first-test sequence on the Pi | `docs/ROS2.md` §9 |
| 7 | Ground-station laptop: Chrome or Edge, the USB-UART driver (CH340 or CP210x), a copy of this repository | `ground_station/README.md` §1 |
| 8 | Optional: start ROS automatically at boot | Appendix A |
| 9 | Practise with the virtual rover in the web ground station (no hardware needed) | §7.4 |

The firmware deliberately **latches the e-stop at boot if the e-stop switch
is missing or open**. The rover will not drive until the switch is fitted
and closed (§14).

---

## 4. Batteries and charging

### 4.1 System

- **Packs:** three 12 V 15 Ah Li-ion packs, 540 Wh in total. Use at most
  80 % of that (432 Wh) to protect the cells.
- **Solar charging:** a 50 W panel, tilted 30°, charges the packs through a
  10 A PWM charge controller.
- **Supply rails:** the motor drivers run directly from the 12 V bus.
  LM2596 buck converters supply the 5 V logic (Pi, Arduinos, LiDAR, sensors)
  and the 6 V servo rail.

### 4.2 Charging

- **Solar:** park the rover in full sun with the panel facing the sun. A
  sunny day in Beni-Suef gives about 186 Wh through the PWM controller
  (about 240 Wh with an MPPT controller). A full recharge from the 80 % limit
  therefore takes about 2–2.5 sunny days.
- **Mains:** for quick turnaround, charge the packs with their own Li-ion
  charger. Follow the pack manufacturer's instructions.
- **Check the voltage** with the battery gauge in the ground station (live
  telemetry) or with a multimeter on the 12 V bus. Compare the two once, as
  part of calibration (`firmware/README.md` §6.5).

### 4.3 How long a charge lasts (calculated)

These figures come from the power budget in the paper: 432 Wh usable, 85 %
efficient converters. Measure the real consumption and update this table.

| Activity | Power | Endurance |
|----------|-------|-----------|
| Standby (Pi idle, sensors on, not driving) | 8.8 W | about 49 h |
| Science stop (Pi busy, LiDAR + camera, arm working) | 25.7 W | about 17 h |
| Driving on flat ground | 78.6 W | about 5.5 h |
| Rough ground with skid turns | 204.5 W | about 2.1 h |
| Worst case (all motors at rated load, arm moving) | 326.4 W | about 1.3 h |
| **Typical sortie** (mixed) | **64.7 W average** | **about 6.7 h** |

### 4.4 Storage

- Disconnect the packs when the rover is stored.
- For storage longer than a few weeks, leave them partly charged, not full
  and not empty, unless the pack instructions say otherwise.
- Keep them in a cool, dry place away from flammable material.

---

## 5. Before every session

Tick every item. If one fails, fix it before powering on.

**Rover**
- [ ] Legs, pivots and bolts: no new bending, nothing loose. Check the
      differential pivot and the leg bolts in particular.
- [ ] All 6 driven wheels and 6 twin wheels turn freely; no cracks.
- [ ] Cables secured, connectors seated, nothing rubbing on the wheels.
- [ ] LiDAR mast, solar panel and camera firmly attached; LiDAR window clean.
- [ ] Ultrasonic sensors clean and pointing in their directions.
- [ ] Arm free to move, gripper empty, sample box in place.
- [ ] Battery charged (§4); low-battery warning not expected during the
      session.
- [ ] E-stop switch fitted and **released** (circuit closed).

**Ground station**
- [ ] Laptop charged; Chrome or Edge.
- [ ] HC-12 radio on its USB-UART adapter; antenna fitted.
- [ ] Copy of `ground_station/web/` on the laptop. It works offline.
- [ ] Optional: a laptop with ROS 2 Jazzy and RViz on the same Wi-Fi as the Pi.

**Area**
- [ ] Driving area clear; spotter in position.
- [ ] Starting position chosen. For mapping, the map's origin is where the
      rover starts. Mark it on the ground (tape) so you can start there again.

---

## 6. Power-on

1. Put the rover at its starting position on level ground (or on blocks for
   a bench test). Keep clear of the arm.
2. Check that the e-stop switch is released.
3. Switch on the main power.
4. **Keep the rover completely still for at least 10 s.** The payload board
   measures the gyro bias during the first 2 s and repeats the measurement
   if the rover moves. The attitude filter then needs a few more seconds to
   settle.
5. Watch the status LED on the Mega (pin 13):

   | LED | Meaning |
   |-----|---------|
   | short blip every second | OK |
   | solid | e-stop latched (switch open or not yet cleared) |
   | fast blinking (4 Hz) | fault: see the fault flags in the ground station (§17.2) |

6. The arm moves to HOME.
7. The gas sensors need to warm up. The ground station shows them as ready
   after 60 s, but wait 3–5 minutes before trusting the values (§12.2).
8. The Raspberry Pi needs about a minute to boot. If ROS does not start
   automatically (Appendix A), connect with SSH and start it:

   ```bash
   ssh <user>@<rover-ip>
   source /opt/ros/jazzy/setup.bash && source ~/MissionX-Rover/ros2_ws/install/setup.bash
   ros2 launch rover_bringup robot.launch.py            # drivers, bridges, EKF, ground-station bridge
   ```

   Add the options for the task of the session:

   | Task | Launch options |
   |------|----------------|
   | Driving, telemetry, sample collection | (none) |
   | Camera images | `camera:=true` |
   | Making a map | `slam:=true` |
   | Navigating on a saved map | `nav:=true map:=$HOME/maps/<name>.yaml` |
   | Autonomous exploration | `slam:=true nav:=true explore:=true` |

9. **Starting ROS resets the Arduinos**, because they reset when their USB
   ports open. The arm returns to HOME and the gyro is measured again, so
   keep the rover still and clear for another 10 s.

---

## 7. Ground station: connect and check

### 7.1 Start

1. Plug the HC-12 USB-UART adapter into the laptop.
2. Open `ground_station/web/index.html` in Chrome or Edge (double-click).
3. Go to **Link → Web Serial**, set the baud rate to **9600**, then click
   **Connect**. Choose the adapter's port in the browser's list. The browser
   remembers the permission.
4. The link pill in the header turns *live* when frames arrive.

### 7.2 First contact

| Step | Key | Expected result |
|------|-----|-----------------|
| Ping | `P` | ACK, with the round-trip time shown |
| Get telemetry | `T` | Telemetry panel fills: battery, mode, fault flags, temperature, humidity, pressure, attitude, gas, closest obstacle |
| Auto-poll | `R` | Telemetry refreshes every 0.5–10 s; 1–2 s is a good start |

Before you drive, check the following:
- the battery is above 10.8 V with margin;
- the mode is IDLE;
- no fault flags are shown;
- roll and pitch are close to 0 on level ground;
- the closest-obstacle value is plausible.

### 7.3 What else is on screen

- **Header:** link status, mode, fault flags, UTC clock and the red E-STOP.
- **Commands:** Ping, Get TLM, Capture image, Get image, Start scan, Stop
  scan, Collect object, Clear E-stop, Set mode; the last reply is decoded.
- **Drive pad**, **telemetry gauges**, **history charts**, **camera** and
  **frame log**. The log shows every frame in hex with its meaning, plus any
  CRC or framing errors. *Export TLM CSV* and *Save session* save the data.
- `?` shows all keyboard shortcuts (also listed in §19).

### 7.4 Practice without the rover

Choose **Link → Virtual rover → Connect**. A simulated rover with sensors,
battery and a simulated radio link runs inside the page. Use it to train new
operators. It is not connected to the real rover.

### 7.5 Command-line ground station

The same functions are available from a terminal, which is useful for
scripts and logging:

```bash
cd ground_station/python
python gs_cli.py --port COM7 ping
python gs_cli.py --port COM7 tlm --poll 1 --count 600 --csv tlm.csv
python gs_cli.py --port COM7 drive 40 0 1500        # 40 % forward for 1.5 s
python gs_cli.py --port COM7 estop                  # also clear-estop, scan-start, scan-stop, collect, cap
python gs_cli.py --port COM7 shell                  # interactive
```

### 7.6 The 2023 LabVIEW ground station

`original_2023/Space Exploration Rover/Ground Station Subsystem/Ground
Station project.vi` still works for the legacy commands: PING, START_SCAN,
STOP_SCAN, CAP_IMAGE, GET_IMAGE and COLLECT_OBJECT. It cannot display the
new 56-byte telemetry and has no e-stop, mode or drive commands. **Use the
new ground station for normal operation.**

---

## 8. Driving

### 8.1 From the ground station (radio)

1. Make sure the e-stop is not latched and the mode is IDLE or TELEOP.
2. **Hold** `W`/`S` (forward/back) and `A`/`D` (turn), or the arrow keys or
   the on-screen pad. Set the speed and turn sliders low (20–40 %) at first.
3. **Release** to stop. The ground station sends a stop command, and each
   drive command expires after 0.6 s anyway. The rover therefore stops by
   itself if the laptop or the radio fails.

Notes:
- 100 % on the sliders = 0.40 m/s and 0.8 rad/s.
- The rover turns by skidding (skid steering). Turns on the spot load the
  motors heavily (§4.3), so prefer gentle curves.
- The radio is half-duplex at 9600 baud. Keep telemetry polling at 1 s or
  slower while driving, so that drive commands are not delayed.

### 8.2 From ROS (keyboard on the Pi or a ROS laptop)

```bash
ros2 launch rover_bringup teleop.launch.py           # speed 0.3 m/s, turn 0.6 rad/s
ros2 launch rover_bringup teleop.launch.py speed:=0.15 turn:=0.4
```

Keys: `i` forward, `,` back, `j`/`l` turn, `k` stop, `q`/`z` faster/slower.
Run it in an SSH terminal on the Pi, or on a laptop with the same
`ROS_DOMAIN_ID`.

---

## 9. Mapping (SLAM)

1. Start ROS with `slam:=true` (§6). Optionally also open RViz on a ROS laptop:

   ```bash
   rviz2 -d $(ros2 pkg prefix --share rover_bringup)/rviz/rover_nav.rviz
   ```

2. Drive slowly, at 0.2 m/s or less, with smooth turns. Mapping works best
   when the LiDAR sees walls and furniture at 0.12–3.5 m.
3. Cover the whole area, then **drive back to the start** so that SLAM can
   close the loop and remove accumulated drift.
4. Save the map before you stop ROS:

   ```bash
   mkdir -p ~/maps
   ros2 run nav2_map_server map_saver_cli -f ~/maps/<name>     # writes <name>.yaml and <name>.pgm
   ```

The map's origin is the rover's starting pose (§5).

---

## 10. Navigating on a saved map

1. Put the rover at the starting mark used when the map was made, facing the
   same way.
2. Start ROS with `nav:=true map:=$HOME/maps/<name>.yaml` (§6).
3. In RViz, check that the laser points line up with the map. If they do
   not, use **2D Pose Estimate** to set the rover's true position.
4. Send a goal with **2D Goal Pose**. The rover plans a path, follows it and
   avoids obstacles seen by the LiDAR and the ultrasonic sensors.
5. Leave enough room. The rover is 0.73 m wide and the planner keeps a safety
   margin, so passages should be about 1.2 m wide or more.

To stop, press `H` (Stop scan) in the ground station, which cancels all
navigation goals, or use the e-stop.

---

## 11. Autonomous exploration

The rover maps an unknown area by itself. It repeatedly drives to the
boundary between known and unknown space (a *frontier*). When no frontiers
are left, it returns to its start and switches to IDLE.

1. Start ROS with `slam:=true nav:=true explore:=true` (§6).
2. Clear the area and post the spotter at the e-stop switch.
3. Press `G` (**Start scan**) in the ground station. The mode changes to
   AUTONOMOUS. If the e-stop is latched, the command is refused (NACK 4).
4. Watch the rover, the telemetry and, if available, RViz (map, goals,
   frontiers).
5. To stop or take over at any time:
   - `H` (**Stop scan**) cancels navigation and stops the rover (mode IDLE);
   - a drive key takes over manually (mode TELEOP);
   - `Space` triggers the e-stop.
6. When it is finished, save the map (§9 step 4).

Without the ground-station bridge you can start exploration from ROS:

```bash
ros2 service call /rover/set_mode rover_interfaces/srv/SetMode "{mode: 2}"
```

---

## 12. Environment sensors and camera

### 12.1 Readings

| Measurement | Sensor | Shown as |
|-------------|--------|----------|
| Temperature, humidity | DHT11 | °C, %RH |
| Pressure | BMP180 | hPa |
| Roll, pitch | MPU-6050 | degrees (+pitch = nose up, +roll = right side down) |
| Flammable gas / smoke, methane | MQ-2, MQ-4 | raw ADC counts (0–1023) |
| Soil moisture, rain | resistive modules | raw ADC counts |
| Metal | inductive proximity sensor | metal detected yes or no (latched until the next reading) |

### 12.2 Taking good gas readings

1. Wait 3–5 minutes after power-on for the MQ heaters to warm up.
   Brand-new sensors need 24–48 h of burn-in before their first use.
2. In clean air at each site, write down the baseline readings of MQ-2 and
   MQ-4.
3. Report gas as the **change from the baseline**. The raw counts are not
   concentrations.

### 12.3 Pictures

Start ROS with `camera:=true` (§6).

1. Stop the rover.
2. Press `C` (**Capture image**). The rover stores a picture.
3. Press `I` (**Get image**) to download it.
   - The legacy 100 × 100 grey picture takes about 11 s at 9600 baud.
   - Choosing a JPEG quality in the camera panel downloads a v2 JPEG
     instead.
   - Missing chunks are shown in red. Press `I` again to fetch the picture
     again.
4. Click **Save PNG**.

Telemetry polling pauses automatically during a download.

---

## 13. Sample collection and the arm

### 13.1 Automatic collection

1. Drive so that the sample is on the ground in front of the rover, within
   the arm's reach, and stop.
2. Press `O` (**Collect object**). The rover stops and the arm runs the
   sequence:

   READY → GRIP_OPEN → REACH_GROUND → GRIP_CLOSE → LIFT_SAMPLE → DEPOSIT
   (over the sample box) → GRIP_OPEN → STOW

   The sequence takes about 15 s. A second *Collect* while it runs is refused
   (NACK 3), and so is a *Collect* while the e-stop is latched (NACK 4).
3. Any e-stop stops the sequence and freezes the arm where it is:
   - the ground-station E-STOP does this at once;
   - the hardware switch does it within about a second, when the Pi
     receives the Mega's status.

   After clearing the e-stop, send the arm to STOW (§13.2).

The pose angles are starting values. Tune them on the real arm before field
use (`firmware/README.md` §2.3 and §6.5).

### 13.2 Single poses

The command-line ground station can send the arm to a named pose (routed
through the Pi to the arm board):

```bash
python gs_cli.py --port COM7 arm-pose stow     # home, stow, ready, reach_ground, grip_open, grip_close, lift_sample, deposit
```

**Stow the arm before driving**, so it is folded back over the deck.

---

## 14. Emergency stop and link loss

### 14.1 Stopping

| Method | Effect |
|--------|--------|
| **Hardware e-stop switch** on the rover | All motors stop at once and the e-stop latches. Within about a second the Pi also freezes the arm, cancels navigation and any sample collection, and sets mode IDLE |
| **E-STOP** button or `Space` in the ground station | Sent ahead of every other queued command and retried up to 3 times until the rover acknowledges it. Motors stop, the arm freezes, navigation and sample collection are cancelled, the e-stop latches and the mode becomes IDLE |
| ROS topic | `ros2 topic pub --once /rover/estop std_msgs/msg/Bool "{data: true}"`: same effect as the ground-station E-STOP |

While the e-stop is latched:
- the Mega LED is solid;
- fault bit 0 is set;
- drive, start-scan and collect commands, and mode changes to TELEOP or
  AUTONOMOUS, are refused with NACK 4;
- the arm holds its position.

### 14.2 Clearing

1. Find and remove the cause.
2. Release the hardware switch, if it was pressed.
3. Press `Shift+C` (**Clear E-stop**). The rover acknowledges. If the
   switch is still open, the e-stop latches again.
4. Clearing never restarts anything: the rover stays in IDLE until you
   drive or start a task again. If the arm stopped part-way through a
   movement, send it to STOW (§13.2).

From ROS: `ros2 topic pub --once /rover/estop std_msgs/msg/Bool "{data: false}"`.

### 14.3 What happens when something fails

| Failure | Rover behaviour |
|---------|-----------------|
| Radio link lost while **driving** | Each drive command expires after 0.6 s, so the rover stops |
| Radio link lost during **autonomous exploration** | **The rover keeps exploring.** It returns to its start when it finishes. Use the hardware e-stop if needed, and stay in range |
| Hardware e-stop pressed during exploration or sample collection | Motors stop at once; within about a second the Pi cancels the task and sets IDLE. Clearing the e-stop does not restart the task |
| Pi or ROS stops | The Mega stops the motors if it gets no velocity command for 0.5 s, and sets fault bit 1 |
| A wheel encoder fails | Fault bit 3. That wheel is left out of the speed control; if a whole side fails, that side runs without feedback |
| Battery low | Fault bit 2 below 10.8 V. The rover keeps working; end the session and recharge |

---

## 15. Safe mode: driving without the Raspberry Pi

If the Pi is broken or not fitted, the radio can be wired directly to the
Mega:

1. Power off.
2. Connect the HC-12 to the Mega: HC-12 TXD → pin 17 (RX2), HC-12 RXD →
   pin 16 (TX2), SET → pin 48, VCC and GND.
3. Power on. The Mega answers on the radio link.
4. In the ground station, open **Link → Advanced** and set the
   **destination address to 0x02**. Telemetry is then in the full 56-byte
   format, with battery, fault flags, odometry position and closest
   obstacle. With address 0x01 the Mega answers in the 2023 format that the
   LabVIEW ground station understands.
5. Fault bit 4 shows that safe mode is active.

Available commands: Ping, Get TLM, drive (100 % = 0.45 m/s and 1.0 rad/s,
at most 5 s per command), Start scan, Stop scan, E-stop and Clear E-stop.

Start scan runs the 2023-style obstacle avoidance:
- the rover drives forward at 0.25 m/s;
- it turns away from obstacles closer than 0.6 m;
- it backs up when the front is blocked;
- it stops if it is boxed in or cannot find a gap within 12 s of turning.

There is no mapping, navigation, camera, environment data or arm in safe mode.

---

## 16. Shutdown

1. Press `H` (**Stop scan**) to stop any task. The mode becomes IDLE.
2. Stow the arm (§13.2).
3. Save your data:
   - the map (§9 step 4);
   - in the ground station, **Export TLM CSV** and **Save session**;
   - any rosbag recordings (Appendix B).
4. Shut the Pi down cleanly, so that its SD card is not corrupted:
   `sudo shutdown -h now` over SSH. Wait until its green LED stops blinking
   (about 20 s).
5. Press the hardware e-stop.
6. Switch off the main power.
7. Disconnect the ground-station radio and close the browser tab.
8. Recharge the batteries (§4) and note any problems in the log book.

---

## 17. Troubleshooting

### 17.1 NACK reasons

The ground station shows the reason when the rover rejects a command:

| NACK | Meaning | Typical cause | What to do |
|------|---------|---------------|------------|
| 1 | Bad length | Wrong command data (custom or `raw` commands) | Check the command |
| 2 | Unknown type | Command not supported by the board it was sent to | Check the destination address (§15 step 4) |
| 3 | Busy | Sample collection already running | Wait until it finishes |
| 4 | Not permitted | E-stop latched (drive, scan, collect, mode change); invalid mode value; arm angle outside its limits (the arm moves to the nearest allowed angle) | Clear the e-stop (§14.2) |
| 5 | Hardware fault | Servo driver not answering, or the Pi's bridge to that board not running | Check power and cables; check `ros2 node list` on the Pi |

### 17.2 Fault flags (Mobility board)

| Bit | Flag | Meaning | What to do |
|-----|------|---------|------------|
| 0 | E-stop latched | Hardware switch open, or E-STOP received | §14.2 |
| 1 | Command timeout | No velocity command for 0.5 s; the rover stopped by itself | Normal after a link or Pi loss; the next command clears it |
| 2 | Low battery | Below 10.8 V for 2 s | End the session and recharge |
| 3 | Encoder fault | A driven wheel showed no encoder ticks for 1 s | Check that the wheel is not jammed; check the encoder cable |
| 4 | Safe mode | The radio is wired to the Mega (§15) | Expected in safe mode |

The payload board reports DHT11, BMP180 and MPU-6050 failures; the arm board
reports a latched e-stop and servo-driver (PCA9685) failures.

### 17.3 Symptoms

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| No reply to Ping | Wrong COM port or baud rate; radios on different channels or settings; rover off; ROS or the ground-station bridge not running (normal configuration); out of range | Check the port and 9600 baud. Re-check the HC-12 settings (§3 item 1). On the Pi: `ros2 node list` should include `ground_station_bridge`. Move closer. Try safe mode (§15) |
| Many CRC or framing errors in the frame log | Weak or disturbed radio link | Move closer, raise the antennas, keep line of sight, poll less often |
| Rover does not drive | E-stop latched (LED solid); low battery; no 12 V supply to the motor drivers | §14.2; check the battery and the 12 V wiring to the three motor drivers |
| Rover drives backwards or turns the wrong way | Encoder or motor direction settings wrong | `firmware/README.md` §6.1 |
| Rover drifts while driving straight, or turns more or less than commanded | Track factor χ not calibrated for this ground | `firmware/README.md` §6.2 and `docs/ROS2.md` §6 |
| Map smeared or doubled | Driving too fast; rover moved during power-on; χ not calibrated; no loop closure | Drive at 0.2 m/s or less; keep still at power-on; calibrate; return to the start before saving |
| No LiDAR data (`ros2 topic hz /scan` shows nothing) | LiDAR not spinning; `/dev/rover_lidar` missing | Check the LiDAR cable and power; check the udev rules (`ls -l /dev/rover_*`) |
| Phantom obstacles close in front on rough ground | Ultrasonic sensors echo off the ground | Lower `ultrasonic_max_range` (for example 1.5 m), see `docs/ROS2.md` §6 |
| Roll and pitch not 0 on level ground, or drifting | Rover moved during the gyro calibration | Power-cycle (or restart ROS) with the rover still |
| Gas values jump or drift | Heaters not warmed up | Wait 3–5 min; use the baseline method (§12.2) |
| Arm does not move | Servo power (6 V, 5 A) off; e-stop latched (NACK 4); servo driver fault (NACK 5) | Check the servo supply and the I²C cable to the PCA9685 |
| Picture incomplete (red areas) | Chunks lost on the radio link | Press `I` again; get closer |
| `/dev/rover_*` names missing on the Pi | udev rules not installed, or serial numbers not filled in | `docs/ROS2.md` §3 step 3 |

---

## 18. Maintenance and calibration schedule

| When | Task |
|------|------|
| Every session | Checklist §5; afterwards, clean dust and sand from the wheels, encoders, LiDAR window and ultrasonic sensors |
| Every 10 sessions or monthly | Check all bolts and the leg geometry; check cables for wear; ultrasonic check (`firmware/README.md` §6.3) |
| New type of ground | Re-calibrate χ (`firmware/README.md` §6.2) |
| Every new site | Record gas baselines (§12.2) |
| After any mechanical change to the arm | Re-tune the arm poses (`firmware/README.md` §6.5) |
| After any change to the wiring or a board swap | Bench checklist (`firmware/README.md` §5); update the udev serial numbers |
| Battery packs | Check for swelling and damage before every charge; store partly charged (§4.4) |

---

## 19. Quick reference

### 19.1 Web ground-station keys

| Key | Action | Key | Action |
|-----|--------|-----|--------|
| `Space` | **E-STOP** | `Shift+C` | Clear e-stop |
| `W A S D` / arrows | Drive (hold) | `M` | Mode selector |
| `P` | Ping | `T` | Get telemetry |
| `R` | Auto-poll on/off | `C` | Capture image |
| `I` | Get image | `O` | Collect object |
| `G` | Start scan (exploration) | `H` | Stop scan |
| `/` | Filter the frame log | `?` | Help |

### 19.2 Commands

For the command-line and LabVIEW ground stations:

| Command | Type | Reply |
|---------|------|-------|
| PING | 0x00 | ACK |
| GET_TLM | 0x0F | Telemetry (56 B from the Pi or the Mega at 0x02; 7 B in the 2023 format) |
| CAP_IMAGE / GET_IMAGE | 0x0C / 0x08 | Echo / picture chunks |
| START_SCAN / STOP_SCAN | 0x16 / 0x1C | Echo |
| COLLECT_OBJECT | 0x20 | Echo |
| ESTOP / CLEAR_ESTOP | 0x51 / 0x52 | ACK |
| SET_MODE | 0x53 | ACK (0 IDLE, 1 TELEOP, 2 AUTONOMOUS, 3 SAFE) |
| DRIVE | 0x60 | ACK (speed %, turn %, duration ms) |

Addresses: 0x01 rover (Pi), 0x02 mobility (Mega), 0x03 payload, 0x04 arm,
0x0B ground station, 0xFF broadcast. Full details: [`ICD.md`](ICD.md).

### 19.3 ROS commands on the Pi

```bash
ros2 launch rover_bringup robot.launch.py [camera:=true] [slam:=true] [nav:=true map:=...] [explore:=true]
ros2 launch rover_bringup teleop.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/maps/<name>
ros2 service call /rover/set_mode rover_interfaces/srv/SetMode "{mode: 2}"     # 0 idle, 1 teleop, 2 explore
ros2 topic pub --once /rover/estop std_msgs/msg/Bool "{data: true}"    # e-stop (false = clear)
ros2 topic hz /scan                     # LiDAR about 5 Hz
ros2 topic echo /rover/status           # heartbeats of the three boards
sudo shutdown -h now
```

---

## Appendix A: start ROS automatically at boot

Run `robot.launch.py` by hand once and confirm that it works. Then create
`/etc/systemd/system/missionx.service`, replacing `<user>` and adding the
launch options you always want:

```ini
[Unit]
Description=MISSION X rover bring-up (ROS 2)
After=network-online.target
Wants=network-online.target

[Service]
User=<user>
Environment=ROS_DOMAIN_ID=42
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/<user>/MissionX-Rover/ros2_ws/install/setup.bash && exec ros2 launch rover_bringup robot.launch.py'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now missionx.service
journalctl -u missionx.service -f        # watch the log
```

The Arduinos reset when the service starts. Follow §6 step 9.

## Appendix B: recording data

**Ground station:** *Export TLM CSV* saves all telemetry samples. *Save
session* saves the telemetry history, frame log, pictures and settings in
one JSON file that can be loaded again later.

**ROS bag on the Pi:** record the sensor data of a run, for example for
later analysis or a paper:

```bash
mkdir -p ~/bags
ros2 bag record -o ~/bags/$(date +%Y%m%d_%H%M) /scan /tf /tf_static /wheel/odom /odometry/filtered \
  /imu/data /ultrasonic/front_left /ultrasonic/front_center /ultrasonic/front_right \
  /ultrasonic/left /ultrasonic/right /ultrasonic/rear /environment /rover/status
```

Stop with `Ctrl+C`. Copy the bag to a laptop with
`scp -r <user>@<rover-ip>:~/bags .`.
