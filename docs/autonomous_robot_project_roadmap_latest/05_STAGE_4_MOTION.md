# Stage 4 — Buddy 2: Motion Control System

## Goal
Build a repeatable motion layer that every other subsystem can trust.

## Known motor/encoder interface

The supplied `docs/project_datasheets/PicoCar.pptx` slide 1 shows an integrated
geared motor/encoder assembly using a PH2.0-6P connector:

| Pin | Documented function |
|---:|---|
| 1 | Encoder ground |
| 2 | Encoder output A phase |
| 3 | Encoder output B phase |
| 4 | Encoder power supply |
| 5 | Motor line+ |
| 6 | Motor line- |

This confirms two labelled encoder phase outputs and their connector positions.
It does **not** specify the motor/encoder model, motor voltage/current/gear ratio,
encoder supply/output voltage, output type, required pull-ups, PPR/CPR, or
whether resolution is stated at the motor or wheel shaft. Obtain those values
and verify the physical harness orientation/continuity before powered testing.
Do not energise encoder power or connect A/B to RP2040 GPIO until their voltage
and output levels are confirmed; the Pico/Pico W GPIO domain is 3.3 V.

## 4A — Motor control
- [ ] Verify left motor polarity
- [ ] Verify right motor polarity
- [ ] Measure motor deadband
- [ ] Determine useful PWM range
- [ ] Implement stop
- [ ] Implement forward
- [ ] Implement reverse
- [ ] Decide brake vs coast behaviour

## 4B — Encoder calibration
- [ ] Identify encoder resolution
- [ ] Count pulses for one wheel revolution
- [ ] Repeat at least 10 times
- [ ] Measure wheel diameter
- [ ] Measure wheel separation
- [ ] Calculate distance per encoder count
- [ ] Verify direction/sign handling

## 4C — Speed estimation
- [ ] Calculate wheel speed
- [ ] Choose fixed update period
- [ ] Reject impossible/glitch readings
- [ ] Verify multiple motor speeds

## 4D — PID speed control
Suggested target tests:
```text
100 mm/s
200 mm/s
300 mm/s
```

Record:
- [ ] target speed
- [ ] measured speed
- [ ] rise time
- [ ] overshoot
- [ ] steady-state error
- [ ] PID gains

## 4E — Straight-line correction
Test:
- [ ] 500 mm
- [ ] 1000 mm
- [ ] 1500 mm
- [ ] Record distance error
- [ ] Record lateral drift

## 4F — Encoder-based turning
Required API behaviour:
```text
moveForward(distance)
moveBackward(distance)
turnLeft(angle)
turnRight(angle)
stop()
```

Test:
- [ ] 90° left
- [ ] 90° right
- [ ] 180°
- [ ] At least 10 repeats each

## Deliverables
- [ ] Motion-control library
- [ ] PID tuning report
- [ ] Motion accuracy evaluation

## Definition of done
- [ ] Mean error measured
- [ ] Maximum error measured
- [ ] Standard deviation / spread measured
- [ ] Repeatability demonstrated
