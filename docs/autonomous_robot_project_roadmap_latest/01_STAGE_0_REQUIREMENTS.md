# Stage 0 — Requirements Freeze and Team Organisation

The [requirements baseline](../project/REQUIREMENTS.md) documents the
briefs. Mission boxes below require team agreement on scope; they are not
implementation pass results. Named owners and hardware records can be filled
in the [starting checklist](../project/STARTING_CHECKLIST.md).

## Goal
Make sure all five team members agree on what the robot must do before implementation starts.

## Mission checklist
- [ ] Follow the line autonomously
- [ ] Detect and decode barcodes
- [ ] Execute left turn
- [ ] Execute right turn
- [ ] Execute straight
- [ ] Execute U-turn
- [ ] Detect humps
- [ ] Measure/report the highest hump peak
- [ ] Detect obstacles
- [ ] Profile obstacle shape/position
- [ ] Bypass obstacles safely
- [ ] Reacquire the original line
- [ ] Report telemetry over WiFi

## Team organisation
- [x] Assign Buddy 1 — Skyler: WiFi / MQTT / telemetry
- [x] Assign Buddy 2 — Suen Hong: Motion control / motors / encoders
- [x] Assign Buddy 3 — Wynn: IR line following / barcode
- [x] Assign Buddy 4 — Zhi Hong: IMU / terrain / hump
- [x] Assign Buddy 5 — Nicholas: Ultrasonic / servo / obstacle profiling
- [ ] Assign Vehicle Controller / integration coordinator — UNKNOWN
- [ ] Give each buddy one Maker Pi Pico
- [ ] Reserve Robo-Pico for integration/final robot

## Hardware audit
- [ ] Record motor model and electrical ratings — PH2.0-6P motor/encoder interface is documented, but the model, voltage, current, and gear ratio remain unknown
- [x] Record motor/encoder connector pinout — `PicoCar.pptx` slide 1: pin 1 encoder ground, 2 A phase, 3 B phase, 4 encoder power, 5 motor+, 6 motor-
- [ ] Record encoder model / resolution / electrical levels — two labelled phases are documented, but PPR/CPR, voltage, output type, and count convention remain unknown
- [x] Record wheel diameter — user reports 60 mm; rolling-diameter calibration pending
- [ ] Record wheel separation / track width
- [x] Record servo model — T-SG90 (user report); electrical/pulse limits pending
- [x] Record battery voltage / capacity — powerbank labelled 20000 mAh; outputs 1 and 2 each 5 V / 2.1 A; combined current/usable capacity pending
- [x] Record IR sensor module model — MH-Sensor-Series (user report/image); exact variant/rating pending
- [x] Record IMU model — supplied material identifies GY-511 / LSM303DLHC; physical board revision still needs confirmation
- [ ] Record ultrasonic module
- [ ] Photograph all hardware
- [ ] Identify Pico vs Pico W on each board

## Questions to clarify
- [ ] Confirm barcode A/B/C/D mapping
- [ ] Clarify supplied A/Z barcode samples
- [ ] Confirm U-turn is assessed
- [ ] Clarify barcode symbology / width encoding
- [ ] Clarify hump-height method / expected accuracy
- [ ] Clarify turn-rate expectation with LSM303DLHC
- [ ] Confirm MQTT broker/settings
- [ ] Confirm supplied GitHub template
- [ ] Confirm any missing course dimensions

## Exit condition
- [x] Written requirements exist — docs/project/REQUIREMENTS.md; team sign-off pending
- [ ] Buddy ownership is clear
- [x] Open questions are documented — OPEN_QUESTIONS.md
