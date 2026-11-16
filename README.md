# MISSION X — Space Exploration Rover (2026 upgrade)

Graduation project of the Faculty of Navigation Science and Space Technology,
Beni-Suef University, built with the Egyptian Space Agency (EgSA). It is a
36.5 kg six-wheel rocker–bogie rover with LiDAR SLAM navigation, an
environmental payload, a 6-DOF sampling arm, solar power and a 433 MHz ground
link.

This repository holds everything produced in the 2026 upgrade. The original
2023 material (reports, CAD, presentation videos) is not in this repository:
it is about 1.8 GB and several files exceed GitHub's 100 MB limit. The scripts
expect it in a sibling folder, `../Space Exploration Rover/`, and they never
modify it.

| Deliverable | Where | Status |
|-------------|-------|--------|
| **Research paper** (IEEE two-column, 10 pages) | `paper/MissionX_Rover_Paper.pdf`, source `paper/main.tex` + `refs.bib` | Compiles cleanly; author order and names to be confirmed by the team |
| **Final demo video** (1080p, ~3 min) in English and Arabic | `video/MissionX_Rover_Demo.mp4` and `video/MissionX_Rover_Demo_AR.mp4` (right-to-left captions); generator `video/make_video.py` (`--ar` for Arabic); voice-over script `video/NARRATION.md` | Rendered |
| **ROS 2 Jazzy stack** (9 packages) + simulator | `ros2_ws/`, guide `docs/ROS2.md`, Docker `docker/` | Unit-tested; not yet run on a real ROS 2 install (see below) |
| **Firmware** (Mega mobility, Uno payload, Uno arm, legacy fix) | `firmware/`, guide `firmware/README.md` | All 4 sketches compile; 5 696 + 82 host tests pass; not yet flashed to hardware |
| **Ground station** (web app with Web Serial + Python CLI/virtual rover) | `ground_station/`, guide `ground_station/README.md` | 66 JS + 47 Python tests pass; exercised in the browser |
| **Interface Control Document v2.0** | `docs/ICD.md` | Wire-compatible with the 2023 LabVIEW ground station |
| **Audit of the 2023 project** | `docs/AUDIT.md` | — |
| **Analysis scripts** behind every number and figure in the paper | `analysis/` | Reproducible (see below) |

## Key results (paper §IV–VIII)

- **Mobility:** 26.9° gradeability on Earth at rated torque; 55 mm continuous
  step climbing (100 mm needs short stall-torque bursts and μ ≥ 0.74).
- **Structure:** the 2023 FEA peak (44.1 MPa) **exceeds** the AL-1050-O yield
  strength (27.6 MPa), which is consistent with the leg bending seen during
  integration.
- **Power:** 6.7 h on a typical sortie (65 W average). A solar recharge takes
  about 2 sunny days, not the 12 hours stated in 2023.
- **Radio:** HC-12 range about 0.9 km (two-ray model, 10 dB fade margin).
- **Protocol safety:** the 2023 receiver executes wrong commands (up to 8.9 %
  under byte loss); the v2 receiver executed 0 wrong commands in 520 000
  trials.
- **Simulation:** SLAM trajectory error 0.11 ± 0.01 m vs 0.93 m for odometry
  alone; odometry drift falls from 11.4 % to 0.5 % of distance (calibrated χ
  plus gyro); a persistent ultrasonic layer cuts collisions with obstacles
  below the LiDAR plane from 231 to 0.

## Folder map

```
MissionX-Rover/
├── paper/            main.tex, refs.bib, figures/, MissionX_Rover_Paper.pdf
├── video/            make_video.py, NARRATION.md, MissionX_Rover_Demo.mp4, assets/
├── ros2_ws/src/      rover_core (shared Python library + simulator), rover_interfaces,
│                     rover_description, rover_hardware, rover_sim, rover_slam,
│                     rover_navigation, rover_bringup, rover_mission
├── firmware/         libraries/MissionX_SSP, mobility_mega, payload_uno, arm_uno,
│                     legacy_fixed, tests/, build_all.sh
├── ground_station/   web/ (open index.html in Chrome/Edge), python/, tools/
├── analysis/         mobility, power, comms, sim_* studies, compose_photos, style
├── docker/           headless simulation + evaluation (ROS 2 Jazzy)
└── docs/             ICD.md, ROS2.md, AUDIT.md
```

## Reproduce

Requirements: Python ≥ 3.10 with numpy, scipy, matplotlib, pillow, pyyaml,
pyserial and pytest. You also need ffmpeg (for the video), a LaTeX distribution
with latexmk (for the paper), Node ≥ 18 (for the ground-station JavaScript
tests) and arduino-cli with the AVR core (for the firmware).

```bash
# shared library + simulator tests
cd ros2_ws/src/rover_core && python -m pytest -q
```

```bash
# every figure and number in the paper (writes paper/figures/*.pdf and analysis/out/*.json)
cd analysis && python mobility.py && python power.py && python comms.py && python sim_odometry.py && python sim_slam.py && python sim_slam_seeds.py && python sim_obstacle_detection.py && python compose_photos.py
```

```bash
# paper
cd paper && latexmk -pdf -outdir=build main.tex
```

```bash
# video: first extract the raw media from the 2023 folder, then render
# (about 15 min at 1080p; --ar for the Arabic edition, --preview for a quick 540p version)
cd video && python extract_assets.py && python make_video.py
```

```bash
# firmware: build all sketches, then run the host tests
cd firmware && ./build_all.sh && ./tests/run_tests.sh
```

```bash
# ground station tests
cd ground_station && python -m pytest -q
```

```bash
cd ground_station/web && node --test
```

ROS 2 on the rover or a PC with Ubuntu 24.04 + Jazzy: see `docs/ROS2.md`. In
short, `colcon build`, then `ros2 launch rover_bringup sim.launch.py slam:=true`
in simulation or `ros2 launch rover_bringup robot.launch.py` on the rover. With
Docker: `docker compose -f docker/docker-compose.yml up sim-slam`.

## Verification status and open items

- The **ROS 2 stack has not been built or run on a real ROS 2 install**. The
  team will run that test. Validated here:
  - 103 unit tests of the ROS-free modules;
  - every node driven through a fake rclpy layer with real SSP frames;
  - static validation of all YAML, XML and launch files, and the URDF expanded
    with xacro;
  - a second, independent review against the Jazzy APIs, which found and fixed
    3 simulation-path bugs.

  `docs/ROS2.md` §9 gives a step-by-step first-test order, and §8 lists what
  remains to verify.
- **Firmware has not been flashed to the rover.** Wheel and motor signs,
  e-stop wiring, battery divider, PCA9685 oscillator and arm poses need bench
  calibration (`firmware/README.md` §5–6, §9).
- **Real-world results in the paper are the 2023 qualitative trials.** All
  quantitative evaluation is analytical or simulated, as the paper states.
- **For the team to confirm:** the author list and order, the supervisor name
  form ("Khaled Youssef" vs "Khaled Y. Kamel"), "Ahmed Mamdouh Ibrahim" vs
  "Mekky", battery capacity (15 Ah assumed), and the estimated geometry
  (track 0.58 m, wheelbase 0.82 m, LiDAR height 0.62 m in `rover_core/params.py`).
