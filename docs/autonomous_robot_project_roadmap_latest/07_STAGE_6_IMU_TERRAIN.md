# Stage 6 — Buddy 4: IMU Motion and Terrain Monitoring

## Goal
Use LSM303DLHC data to classify motion and detect/measure humps.

## 6A — IMU driver
- [ ] Detect device on I2C
- [ ] Configure known sample rate
- [ ] Read X/Y/Z acceleration
- [ ] Read magnetic data if required
- [ ] Convert raw values to physical units
- [ ] Implement communication timeout/error reporting

## 6B — Calibration
- [ ] Stationary-flat readings recorded
- [ ] Sensor orientation documented
- [ ] Zero offsets measured
- [ ] Noise level measured
- [ ] Calibration values stored/documented

## 6C — Tilt validation
- [ ] Forward
- [ ] Backward
- [ ] Left
- [ ] Right
- [ ] Verify expected axis response

## 6D — Motion classification
Build:
```text
STATIONARY
ACCELERATING
TURNING
CLIMBING_HUMP
DESCENDING_HUMP
IMPACT
```

## 6E — Hump detection
- [ ] Drive over one controlled hump
- [ ] Log full acceleration trace
- [ ] Identify hump entry
- [ ] Identify peak
- [ ] Identify exit
- [ ] Repeat at different speeds
- [ ] Test multiple humps

## 6F — Highest hump peak
Track:
```text
current_hump_peak
highest_hump_peak
```

- [ ] Highest peak persists for the run
- [ ] Telemetry can report it

## Deliverables
- [ ] IMU-processing module
- [ ] Hump-detection algorithm
- [ ] Terrain-analysis report

## Important unresolved point
- [ ] Clarify expected "turn-rate monitoring" because LSM303DLHC has no gyroscope
- [ ] Clarify hump-height estimation method/accuracy
