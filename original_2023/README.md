# Original 2023 project files

These are the files of the 2023 graduation project *Design and Implementation
of a Space Exploration Rover Using an Autonomous Navigation System* (Faculty
of Navigation Science and Space Technology, Beni-Suef University, with the
Egyptian Space Agency). The folder layout is the team's own. Every file is
byte-for-byte the original, except for the five files listed under
[What was changed](#what-was-changed).

The 2026 upgrade in the rest of this repository builds on these files:

- `docs/AUDIT.md` reviews them;
- `firmware/legacy_fixed/` is the corrected version of the Arduino code in
  `Space Exploration Rover/Mobility sub system/`;
- `docs/ICD.md` stays wire-compatible with the LabVIEW ground station.

## Contents

| Folder | What it holds |
|--------|---------------|
| `Documentation/final documentation/` | The full project book, `space rover.docx`, and a PDF version, `space exploration rover.pdf` |
| `Documentation/final presentation/` | The final presentation, `All in One.pptx`, as PDF and as a video of the slides (`All in One.mp4`); the demo videos `mapping.mp4`, `navigation.mp4` and `sensor.mp4` |
| `Documentation/show/` | Exhibition material: photos (`1.jpg`–`3.jpg`), a newspaper clipping (`66.jpg`), a TV news report (`نشر - الموجز.mp4`) and the project description in Arabic and English (`Rover.txt`) |
| `Documentation/Banner/`, `Documentation/name plate/` | Poster template and name plate |
| `Space Exploration Rover/final documentation/` | The subsystem reports in `Report/` (1 introduction, 2 structure, 3 communication and ground station, 4 brain, 5 and 6 power), the presentation as PDF (`peresentation/`) and the group report (`التقرير/`) |
| `Space Exploration Rover/Mobility sub system/` | Two versions of the Arduino Mega rover code (`RF/` and `Rover Final Code/`, the latter also as `.rar`), the mobility report and presentation, and the motor sizing sheet `motor calculation0.xlsx` |
| `Space Exploration Rover/Ground Station Subsystem/` | The LabVIEW ground station, `Ground Station project.vi`, and `Crc calculator.vi` |
| `Space Exploration Rover/SSP protocol python code/` | `full capture.py`: SSP packet building and parsing, with camera image capture |
| `Space Exploration Rover/power subsystem/` | Altium Designer project of the LM2596S buck-converter board (schematic, PCB, libraries, BOM). `History/` and `__Previews/` are Altium's own backup and preview folders |
| `Space Exploration Rover/Structure Subsystem/` | SolidWorks CAD: `Rocker Bogie mechanism/` (the rover's parts and assemblies), `robot arms/robot arm files/` (the sampling arm) and `motor Design/` (gear-motor model and the vendor's drawing) |

## What was changed

GitHub rejects files larger than 100 MB, and five files were larger. They were
re-encoded with ffmpeg (H.264, x264 preset slow). Audio was copied unchanged,
and the frame counts and timing match the originals. The full-size originals
remain in the team's archive.

| File in `Documentation/final presentation/` | Original | In this repository |
|------|----------|--------------------|
| `navigation.mp4` | 423 MB, 3840×2160, 50 Mb/s | 51.3 MB, 1920×1080 (CRF 22) |
| `sensor.mp4` | 403 MB, 3840×2160, 50 Mb/s | 47.4 MB, 1920×1080 (CRF 22) |
| `mapping.mp4` | 111 MB, 1080×1920, 20 Mb/s | 41.7 MB, same resolution (CRF 22) |
| `All in One.gif` | 168 MB animated GIF, 853×480 | Replaced by `All in One.mp4`, 21.6 MB, with the same 1475 frames and timing (1 Mb/s) |
| `All in One.pptx` | 144 MB | 94.7 MB. Only the embedded rover-run video, `ppt/media/media3.mp4` (1080p, 66 MB), was re-encoded, to 16.8 MB (4.9 Mb/s, still 1080p). The 43 slides, the images and the other videos are unchanged. |

## What was left out

| Not included | Size | Why |
|--------------|------|-----|
| `Space Exploration Rover/Structure Subsystem/Components/` | 265 MB | CAD models of off-the-shelf parts (Arduino, DHT22, sensors, a drone model) downloaded from GrabCAD. They are not the team's work and stay under their authors' terms. One file is 109 MB. |
| `Structure Subsystem/robot arms/0-pdf/`, `robot arms/other arms/`, `robot arms/Robot Arm/` | 264 MB | Other designers' robot-arm models (for example a FANUC M-20iA), downloaded for comparison |
| `Structure Subsystem/0-pdf/reference rover/MER_static.glb` | 12 MB | NASA's public 3D model of the Mars Exploration Rover, used as a reference and available from NASA |
| `~$…` files | — | Lock files that Word and PowerPoint leave behind while a document is open |

## Third-party material inside the team's files

Some of the team's files contain material by others. It is kept because it is
part of the files as submitted, and it remains the property of its owners:

- the final presentation embeds a NASA/JPL animation of the Perseverance rover
  (`ppt/media/media1.mp4`) and images from the web;
- `Documentation/show/66.jpg` is a newspaper clipping, and `نشر - الموجز.mp4` is a TV
  news report about the project;
- `motor Design/0-pdf/Transmotec-Drawing-SDS80120.pdf` is the motor vendor's
  drawing.
