# Stage 7 — Buddy 5: Ultrasonic Obstacle Profiling and Avoidance

## Goal
Detect obstacles, scan/profile them, recommend a bypass, and support line recovery.

## 7A — Basic ranging
- [ ] HC-SR04 electrical interface is safe for Pico logic
- [ ] Trigger works
- [ ] Echo timing works
- [ ] Timeout implemented
- [ ] Known-distance calibration recorded
- [ ] Invalid readings handled

## 7B — Servo calibration
- [ ] Safe minimum angle
- [ ] Safe maximum angle
- [ ] Verify 90° centre
- [ ] Measure settling time
- [ ] Prevent mechanical overtravel

## 7C — Coarse scan
Suggested angles:
```text
30°
60°
90°
120°
150°
```

Record at each angle:
- [ ] angle
- [ ] distance
- [ ] valid/invalid
- [ ] timestamp

## 7D — Fine scan
- [ ] Return to detected obstacle region
- [ ] Use smaller angular steps
- [ ] Improve localisation

## 7E — Obstacle profile
Estimate:
- [ ] location
- [ ] width
- [ ] closest point
- [ ] left clearance
- [ ] right clearance

## 7F — Avoidance recommendation
Return:
```text
STOP
CONTINUE
TURN_LEFT
TURN_RIGHT
REVERSE_RETRY
```

- [ ] Do not directly fight line/motion modules for motor control

## 7G — Line recovery
```text
search for line
→ detect line
→ centre vehicle
→ return control to line following
```

## Deliverables
- [ ] Scanning subsystem
- [ ] Obstacle profile generator
- [ ] Avoidance algorithm
- [ ] Recovery algorithm

## Definition of done
- [ ] Obstacle detection repeatable
- [ ] Coarse scan works
- [ ] Fine scan refines position
- [ ] Bypass direction can be selected
- [ ] Line reacquisition is reliable
