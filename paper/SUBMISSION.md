# Submitting the MISSION X paper

`MissionX_Rover_Paper.pdf` is a 10-page IEEE conference-format manuscript
(`main.tex` + `refs.bib`, figures in `figures/`). It builds with
`latexmk -pdf -outdir=build main.tex`. Every number in it comes from a script
in `../analysis/` (see the top-level README).

## Before submitting (team to do)

- [ ] **Authors.** Confirm the author list and order. The order is currently
  alphabetical, as on the 2023 final presentation. Confirm the supervisor's
  name form ("Khaled Youssef" in the report vs "Khaled Y. Kamel" in the
  presentation) and "Ahmed Mamdouh Ibrahim" vs "Mekky". Add e-mail addresses
  and ORCID iDs if the venue requires them, and choose the corresponding
  author.
- [ ] **Facts only the team can check.** Battery capacity (3 × 12 V 15 Ah
  assumed); the estimated geometry in `rover_core/params.py` (track 0.58 m,
  wheelbase 0.82 m, LiDAR height 0.62 m); dates of the laboratory trials.
- [ ] **Photos.** Fig. 1 and Fig. 9 show only the rover and the laboratory. The
  video shows team members, so get everyone's consent before publishing it.
- [ ] **Code availability.** If you publish the repository, add its URL to the
  abstract's last sentence and to a "Data and code availability" footnote.
- [ ] **Venue length.** Many IEEE conferences allow 6 pages plus paid extra
  pages. To shorten the paper, merge Tables I–II, move Fig. 6 (SSP frame) and
  Table III into the text, and drop Fig. 4 (FEA) or Fig. 11 (odometry).
- [ ] **Venue format.** Re-run the venue's template checks (IEEE PDF eXpress
  for IEEE conferences). The paper uses only standard packages.

## Candidate venues (check current calls and deadlines)

| Type | Venue | Why it fits |
|------|-------|-------------|
| Conference | IEEE Aerospace Conference | Rover systems engineering, lessons learned; accepts long papers |
| Conference | International Astronautical Congress (IAC), space education or exploration symposia | Student and education rover projects |
| Conference | NILES (Novel Intelligent and Leading Emerging Sciences Conference), Egypt | Regional IEEE conference, robotics/embedded tracks |
| Journal | *Journal of Engineering and Applied Science* (Springer, Cairo University) | Open-access engineering journal |
| Journal | *Aerospace* or *Robotics* (MDPI) | Open access, accepts system design + evaluation papers |

## Cover letter (draft)

> Dear Editor,
>
> We submit the manuscript "Design, Implementation and Verification of a
> Low-Cost Six-Wheel Rocker–Bogie Exploration Rover with LiDAR-Based
> Autonomous Navigation" for consideration in [venue].
>
> The paper reports a complete student-built planetary-rover prototype
> developed with the Egyptian Space Agency, and adds a post-hoc engineering
> verification that student projects rarely contain: mobility, structural,
> power and radio-link analyses; an interface audit of the ground link with a
> Monte-Carlo fault-injection study (zero wrong commands in 520 000 trials
> after correction, versus up to 8.9 % before); and a digital-twin study of
> SLAM, odometry and low-obstacle safety. We also discuss transferable lessons
> learned and release the firmware, a ROS 2 stack and analysis scripts.
>
> The manuscript is original, has not been published elsewhere, and is not
> under consideration by another venue. All authors approved the submission.
>
> Sincerely,
> [corresponding author], on behalf of the authors

## Highlights (for journals that ask for them)

- A 36.5 kg rocker–bogie rover with LiDAR SLAM, a sampling arm and a 433 MHz ground link, built by undergraduates.
- An interface audit found a CRC byte-order mismatch and a disabled CRC check; the corrected receiver executed zero wrong commands in 520 000 trials.
- Re-analysis gives 26.9° gradeability, about 0.9 km link range and 6.7 h endurance, and corrects the original FEA and power conclusions.
- The digital twin gives 0.11 m SLAM trajectory error and 0.5 % odometry drift with gyro fusion.
- A persistent ultrasonic layer removes collisions with obstacles below the LiDAR plane (231 → 0).
