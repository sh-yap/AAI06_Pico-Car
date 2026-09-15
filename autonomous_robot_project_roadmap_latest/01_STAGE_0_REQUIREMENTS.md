# Stage 0 — Requirements Freeze and Team Organisation

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
- [ ] Assign Buddy 1 — WiFi / MQTT / telemetry
- [ ] Assign Buddy 2 — Motion control / motors / encoders
- [ ] Assign Buddy 3 — IR line following / barcode
- [ ] Assign Buddy 4 — IMU / terrain / hump
- [ ] Assign Buddy 5 — Ultrasonic / servo / obstacle profiling
- [ ] Give each buddy one Maker Pi Pico
- [ ] Reserve Robo-Pico for integration/final robot

## Hardware audit
- [ ] Record motor model
- [ ] Record encoder model / resolution
- [ ] Record wheel diameter
- [ ] Record wheel separation / track width
- [ ] Record servo model
- [ ] Record battery voltage / capacity
- [ ] Record IR sensor module model
- [ ] Record IMU model
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
- [ ] Written requirements exist
- [ ] Buddy ownership is clear
- [ ] Open questions are documented
