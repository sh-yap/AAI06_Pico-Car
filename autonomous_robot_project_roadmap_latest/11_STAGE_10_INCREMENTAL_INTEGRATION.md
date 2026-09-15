# Stage 10 — Incremental Integration

## Goal
Integrate in controlled pairs, not all at once.

## 10.1 Motion + encoders
- [ ] Stable

## 10.2 Motion + line following
- [ ] Normal line tracking works

## 10.3 Motion + barcode
Test:
```text
detect command
→ controller requests manoeuvre
→ motion executes
→ line following resumes
```

- [ ] LEFT
- [ ] RIGHT
- [ ] STRAIGHT
- [ ] UTURN

## 10.4 Motion + IMU
- [ ] IMU data remains stable with motor noise/vibration

## 10.5 Servo + ultrasonic + controller
- [ ] Stationary obstacle scan works

## 10.6 Motion + obstacle
- [ ] Bypass works in open space

## 10.7 Obstacle + line
```text
follow line
→ obstacle
→ stop
→ scan
→ bypass
→ search
→ reacquire
```

- [ ] Entire sequence passes

## 10.8 Add WiFi
- [ ] Network activity does not disturb control timing

## At every integration step
- [ ] Repeat 5–10 times
- [ ] Log failures
- [ ] Fix before adding next subsystem
- [ ] Keep regression tests
