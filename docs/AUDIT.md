# Technical audit of the 2023 MISSION X project

This audit reviewed every artefact in `Space Exploration Rover/` (the team's
own files from that folder are in this repository under `original_2023/`): the final
report (`space rover.docx`, 166 figures), the subsystem reports, the
presentations, the Arduino code, the Python SSP script, the LabVIEW VIs, the
videos and the CAD inventory. Each finding below states the evidence, its impact
and what was done about it in `MissionX-Rover/`. The original folder was **not
modified**.

Severity: **H** = wrong result or unsafe behaviour; **M** = misleading or
fragile; **L** = cosmetic or documentation.

## 1. Engineering calculations

| # | Sev | Finding | Evidence | Resolution |
|---|-----|---------|----------|------------|
| E1 | H | The FEA conclusion contradicts its own result. The report says "maximum stress is less than yield strength", but the legend shows a peak von Mises stress of **44.1 MPa** against a library yield of **27.6 MPa** (AL-1050-O), a minimum factor of safety of **0.62**. The report text also says "yield 250 MPa", while the presentation says 27.5 MPa. | Report fig. 156 (legend), §4.1; presentation slide 6 | Paper §IV-C reports the correct reading. The peaks are local and agree with the bending seen in the legs. A strain-hardened temper (H14/H24), a 5xxx/6xxx alloy, or doubler plates at the joints would raise FoS above 2. |
| E2 | M | The motor sizing criterion `T = (mg/6)·r = 5.97 N·m` describes lifting one wheel vertically. It is not a slope or step requirement. The chosen motor gives 2.97 N·m, half of that figure, yet the report calls it "very good". | Report §2.2.2.2 | `analysis/mobility.py`: gradeability 26.9° on Earth at rated torque; continuous step climbing 55 mm; 100 mm (= wheel radius) needs short stall-torque bursts and μ ≥ 0.74. |
| E3 | M | Two incompatible mass/torque sets are used: 30 kg on Mars with μ = 0.8 (mobility report, xlsx) and 36.5 kg on Earth (final report). | `motor calculation0.xlsx`, report §2.2 | A single parameter set lives in `rover_core/params.py` (ICD §9). |
| E4 | H | The power budget adds currents from 3.3 V, 5 V and 12 V rails ("35.3 A"). It uses 0.2 A per Hall encoder (datasheet 14–20 mA, given in the same report) and per ultrasonic sensor (15 mA). It counts a Kinect and a YDLIDAR G4 that were never fitted (as-built: LDS-01 + Pi Camera v2). It assumes all six motors at rated load all the time. | Report Table 6 / fig. image29 | `analysis/power.py` rebuilds the budget per rail and per mode. Typical sortie: 64.7 W and 6.7 h endurance; flat traverse 79 W. |
| E5 | M | "Charging time of the batteries from the solar panel would be 12 hours" assumes the panel gives 50 W continuously (540 Wh / 50 W = 10.8 h). | Report §2.3 | About 186 Wh/day at Beni-Suef through the PWM controller (about 240 Wh/day with MPPT), so a full recharge takes about **2 sunny days**. |
| E6 | L | Battery capacity is written as "12 V & 15 Amp" (an amp-hour rating was presumably meant). The 2022 design was 2 × 12 V 24 Ah. | Report §2.2.2.3, power report | Assumed 3 × 12 V 15 Ah = 540 Wh. **The team should confirm this.** |
| E7 | L | Six stalled motors draw 6 × 14 A = 84 A from the battery packs, but battery and fuse ratings are not documented. | Motor datasheet | Recorded in the paper and the firmware README. The firmware limits PWM ramps. |

## 2. Interfaces and code (SSP link)

See `docs/ICD.md` §8 for D1–D12. The ones with safety impact:

| # | Sev | Finding | Evidence |
|---|-----|---------|----------|
| D1 | H | CRC byte order differs between implementations. LabVIEW and Python send the low byte first (captured frame `C0 01 0B 0F 00 7B 96 C0`, CRC 0x967B). The Arduino `SSP_P_Con` appends the high byte first. | `SSP.cpp`, `full capture.py`, report fig. 185 |
| D2 | H | `loop()` ignores `Exetreaction()`'s CRC result and dispatches on the raw buffer, so corrupted frames are executed. | `RF.ino` |
| D3 | H | The receiver reads a fixed 8 bytes 100 ms after `available()>5` and flushes by calling `Serial1.begin()`. A single lost byte shifts the length field into the type position. | `RF.ino` |
| D6 | M | The GET_TLM reply is a hard-coded 15-byte array, not sensor data. | `SSP.cpp` (Final Code) |
| D5 | M | `malloc()` on every command, never freed, on an 8 KB SRAM MCU. | `SSP.cpp` |
| D8 | M | `pulseIn()` without a timeout blocks up to 1 s; distances above 255 cm wrap in an 8-bit cast. | `Rover_F1.cpp` |
| D11 | H | No command watchdog, so the rover keeps driving if the link drops. | `RF.ino` |

Quantified impact: `analysis/comms.py` ran a Monte Carlo of 520 000 commands
per receiver. The 2023 receiver executes the **wrong command** for 0.09 % of
commands at BER 1e-3, and for 3.1 % / 8.9 % at 1 % / 3 % byte loss. The v2
receiver executed **none**.

Resolution: `rover_core/ssp.py` (Python reference), `firmware/libraries/MissionX_SSP`
(C++, cross-checked frame-by-frame against the Python on 192 corrupted streams)
and `ground_station/web/js/ssp.js` (cross-checked the same way). The wire format
is unchanged, so **the LabVIEW ground station keeps working**:
`firmware/legacy_fixed` keeps the 2023 wiring and 7-byte telemetry, and the v2
Mega answers LabVIEW in safe mode.

## 3. Documentation consistency

| # | Sev | Finding |
|---|-----|---------|
| C1 | M | Report figs. 163/164 (IMU results) are tutorial screenshots, with ROS timestamps from May 2017. Figs. 161/162 (map saving, navigation) show the TurtleBot3 Gazebo world, not the rover. The real results are in the videos and in figs. 159/177 (lab map). The paper uses only the team's own data. |
| C2 | M | The text says the payload sends "17 bytes for 7 sensors", while the firmware sends 7 bytes. The Python brain sent 7 or 68 bytes. ICD §7.1 now defines both the legacy and v2 layouts. |
| C3 | L | Names differ between documents: "Khaled Youssef" (report) vs "Khaled Y. Kamel" (presentation), and "Ahmed Mamdouh Ibrahim" (report) vs "Ahmed Mamdouh Mekky" (presentation). **The team must confirm both before submitting the paper.** |
| C4 | L | The LiDAR model is LDS-01 in the text, but YDLIDAR G4 in the power table and "laser camera"/Kinect in the conclusion. The as-built rover uses the LDS-01, as the photos and video show. |
| C5 | L | The obstacle-avoidance threshold is 80 cm in the code and slides but "10 cm" in the mobility report. The v2 fallback behaviour uses 0.6 m, which is configurable. |
| C6 | L | The ROS source code (Hector/AMCL/move_base launch files, rosserial sketches, IMU publisher), the Flutter app and the payload/arm Arduino code are not in the project folder. Only report figures show them. The ROS stack and all firmware were rebuilt (`ros2_ws/`, `firmware/`). |

## 4. Recommendations for the next team

1. Write the ICD first, with test vectors, and run every implementation against it on every build. The repository now does this.
2. Read FEA legends, and put the factor of safety in the report rather than a sentence.
3. Build power budgets per rail and per operating mode, referred to the battery bus.
4. Calibrate the skid-steer effective track (χ) by a 360° spin, and take heading from the gyro. In simulation this cuts odometry drift from 11.4 % to 0.5 % of distance.
5. Keep ultrasonic detections persistent (costmap range layer), tilt the sensors slightly downward, or add a depth camera. A LiDAR on a mast cannot see anything below about 0.6 m.
6. Test on loose soil, slopes and steps, and log ground truth (tape marks, or a phone camera on a tripod) so results are quantitative.
