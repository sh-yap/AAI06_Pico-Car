# Requirements baseline — Stage 0

Status: extracted from the provided material; **team sign-off pending**.

Sources:

- [Project Write-up](../private/Project%20Write-up.pdf), pages 1–11.
- [Project Briefing](../private/Project%20Briefing.pdf), pages 1–10.
- Local `AGENTS.md` for supplied sensor models and development-board arrangement.
- [Stage 0](../autonomous_robot_project_roadmap_latest/01_STAGE_0_REQUIREMENTS.md)
  and [open questions](../autonomous_robot_project_roadmap_latest/OPEN_QUESTIONS.md).

Evidence below is a proposed verification plan. No mission function has been
demonstrated by this review. Agree numerical acceptance limits before testing;
the briefs do not give enough tolerances to invent pass thresholds.

## Required platform and development constraints

- Raspberry Pi Pico / RP2040 with Robo-Pico for final integration.
- Lecturer-provided micro T-Kernel RTOS base, C, and Pico C SDK.
- Follow the supplied GitHub template and Barr-C as the project coding guideline.
- Autonomous onboard decisions; normal operation must not depend on remote control.
- One Robo-Pico integration platform and five Maker Pi Pico bench platforms.
- Explicit hardware ownership and a central Vehicle Controller.

The repository uses GNU Make and selected SDK sources, rather than the usual
Pico SDK CMake runtime. Preserve that integration. The kernel supports single
core and SMP; agree the assessed/final execution profile with the lecturer and
team. Use `SMP=1` for the shared SMP bring-up demonstration.

## Mission and evidence matrix

| ID | Required behaviour | Responsible subsystem(s) | Brief reference | Proposed evidence |
|---|---|---|---|---|
| R01 | Follow a black line autonomously | Buddy 3, controller, Buddy 2 | Write-up pp. 2–3, 7 | Straight/curve runs, tracking success, lighting and speed variations |
| R02 | Detect and decode navigation barcodes | Buddy 3 | Write-up pp. 2–3, 7 | Edge/width traces and recognition results at multiple speeds |
| R03 | Execute Left, Right, Straight, and U-turn | Controller, Buddy 2; intent from Buddy 3 | Write-up pp. 2–3, 6 | Encoder-based manoeuvres, repeated angle/distance errors, resumed tracking |
| R04 | Detect humps using IMU data | Buddy 4 | Write-up pp. 2–3, 8 | Calibrated IMU traces with labelled entry, peak, and exit |
| R05 | Estimate hump height and report the highest peak of the run | Buddy 4, Buddy 1 | Write-up pp. 2–3, 8 | Estimates against measured hump heights; persistent run maximum in telemetry |
| R06 | Detect obstacles on or near the path | Buddy 5 | Write-up pp. 2–3, 9 | Known-distance tests, invalid echo/timeout handling |
| R07 | Coarse/fine scan and estimate obstacle position, width, nearest point, and clearance | Buddy 5 | Write-up pp. 9–10; Briefing p. 8 | Angle/distance logs and profiles compared with measured objects |
| R08 | Choose and execute a safe bypass with limited route deviation | Buddy 5 recommends; controller decides; Buddy 2 moves | Write-up pp. 2–3, 9–10 | Left/centre/right obstacle trials and collision-free bypass results |
| R09 | Reacquire the original line and continue | Buddies 3/5, controller, Buddy 2 | Write-up pp. 2–3, 9–10 | Full stop/scan/bypass/search/reacquire sequence |
| R10 | Provide WiFi/MQTT telemetry and command subscription | Buddy 1; commands go through controller | Write-up p. 5; Briefing p. 4 | Broker messages, subscription test, documented fields and units |
| R11 | Provide heartbeat and connection recovery | Buddy 1 | Write-up p. 5 | AP/broker interruption and recovery logs; continued safe robot operation |
| R12 | Classify motion, detect tilt/collision, and report turn-rate/motion events | Buddy 4; controller handles events | Write-up p. 8; Briefing p. 7 | Calibration/classification results; lecturer-approved turn-rate interpretation |
| R13 | Demonstrate dependable integrated operation and resource efficiency | All buddies | Write-up pp. 1, 11; Briefing pp. 9–10 | Completion rate/time, fault trials, timing/stack/memory results, final rehearsal |

## Confirmed scope and remaining ambiguities

**U-turn stays in scope.** Write-up page 2 explicitly requires it and page 3
includes it in the example mapping. Buddy 3's shorter list on page 7 omits it;
that omission does not justify removing a mission requirement. Ask whether
there are any assessment-specific restrictions.

**A/B/C/D is an example, not a frozen barcode specification.** Write-up page 3
shows A→Left, B→Right, C→Straight, D→U-turn. Confirm final symbols, symbology,
bar/space widths, orientation, and placement before implementing the decoder.
The roadmap mentions A/Z samples; no separately identifiable A/Z sample files
were found in this workspace inventory.

**Hump height needs a defined method and units.** The brief asks for height
estimated using IMU data. Maximum acceleration or maximum tilt can support
detection, but those quantities alone do not satisfy a physical height estimate.
Clarify acceptable use of encoder distance, calibration geometry, expected
accuracy, and the definition of the reported peak before choosing an algorithm.

**Turn rate needs clarification.** `AGENTS.md` identifies LSM303DLHC as an
accelerometer plus magnetometer without a gyroscope. Keep the turn-rate
requirement visible and ask which estimation inputs/methods are acceptable.

**WiFi hardware must be identified.** Record Pico/Pico W and any external
WiFi module on every development and final board. The existing CYW43 service
targets Pico W hardware; board names alone do not establish compatibility.

**Course, broker, and template details are still pending.** Record line widths,
junction/barcode geometry, hump/obstacle bounds, start/finish behaviour, broker
settings, final template identity, and any required accuracy/runtime limits.
Briefing page 10 mentions a Week 10 demonstration; confirm its calendar date.

## Stage 0 approval record

| Item | Current status |
|---|---|
| Written requirements | This document exists |
| All five members reviewed the mission | Pending |
| Named subsystem owners and controller coordinator | Five Buddy owners recorded in STARTING_CHECKLIST.md; controller coordinator UNKNOWN |
| Hardware inventory measured/photographed | Pending |
| Lecturer responses recorded | Pending; see open-question register |
| Requirements freeze agreed | Pending |
