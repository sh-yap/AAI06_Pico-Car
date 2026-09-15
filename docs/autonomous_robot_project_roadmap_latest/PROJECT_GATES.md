# Project Gates — Go / No-Go Checklist

Current evidence and limitations: [gate status](../project/PROJECT_GATES.md).
Local builds do not establish everyone-can-build/flash or hardware success.

## G0 — Requirements
- [ ] Mission understood by all members
- [x] Buddy roles assigned — Skyler, Suen Hong, Wynn, Zhi Hong, Nicholas; see starting checklist
- [x] Open questions documented — see OPEN_QUESTIONS.md

## G1 — Platform
- [ ] Everyone can build
- [ ] Everyone can flash
- [ ] Fresh clone works

## G2 — Architecture
- [ ] APIs agreed
- [ ] GPIO/resource map agreed
- [ ] RTOS task ownership agreed
- [ ] Units/data contracts agreed

## G3 — Drivers
- [ ] Motors work
- [ ] Encoders work
- [ ] IR sensors work
- [ ] IMU works
- [ ] Ultrasonic works
- [ ] Servo works
- [ ] WiFi works

## G4 — Motion
- [ ] Stable speed control
- [ ] Accurate distance
- [ ] Repeatable turns
- [ ] Straight-line correction

## G5 — Perception
- [ ] Line following works
- [ ] Barcode decoding works
- [ ] Hump detection works
- [ ] Obstacle scanning/profiling works

## G6 — Controller
- [ ] State machine works with simulated inputs
- [ ] Arbitration rules defined
- [ ] Fault transitions work

## G7 — Integration
- [ ] Pairwise integration tests pass
- [ ] Regression tests exist

## G8 — Mission
- [ ] Full autonomous sequence works

## G9 — Robustness
- [ ] Repeated runs pass
- [ ] Fault/variation tests completed
- [ ] Timing/memory checked

## G10 — Demo Ready
- [ ] Evidence organised
- [ ] Final run rehearsed
- [ ] Team can explain interfaces/design choices

# Team rule
**Do not integrate an untested subsystem.**

Each Buddy should arrive at integration with:
- stable API
- test results
- calibration data
- known failure modes
- clear definition of done
