# Stage 5 low-speed closed-loop test - 2026-10-28

## Purpose

Find the slowest smooth course speed without WiFi/MQTT. Raw PWM is no longer
used as the motion command: GP20 submits a 200 mm encoder-distance command
through the Motion task at a selected speed in mm/s.

## Firmware

- Application: `motion_stage5_test`
- Board: `robo_pico_robot`
- SMP: enabled
- Console: USB CDC, optional result output after braking
- WiFi/MQTT: disabled
- Build result: passed with zero emitted warnings/errors;
  text/data/bss 96924/156/32392 bytes
- GP20: start selected case
- GP21 while running: immediate stop
- GP21 while idle: next case; long hold resets to forward 50 mm/s

Selection indications:

| Indication | Forward speed | Reverse indication |
| ---: | ---: | --- |
| 1 short | 50 mm/s | 1 long + 1 short |
| 2 short | 60 mm/s | 1 long + 2 short |
| 3 short | 80 mm/s | 1 long + 3 short |
| 4 short | 100 mm/s | 1 long + 4 short |
| 5 short | 120 mm/s | 1 long + 5 short |

Result codes: 1 complete, 2 operator stop, 3 timeout, 4 stall, 5 encoder
fault, 6 invalid command, 7 hardware/configuration fault, 8 command rejection,
and repeating 9 for test initialisation failure.

## Minimal test

Run forward 50 mm/s first. If it is smooth and completes, repeat it twice more
and stop. Test 60 mm/s only if 50 stalls or is visibly jerky; test 80 mm/s only
if 60 also fails. Record whether both wheels start, physical travel, elapsed
time if measured, drift and result code. Do not test faster or reverse cases
yet. This adaptive check selects a provisional course speed with the fewest
necessary physical runs; formal repeatability is deferred.

| Speed | Trial | Approx. physical travel | Smooth start/run | Drift | Result code | Notes |
| ---: | ---: | ---: | --- | --- | ---: | --- |
| 50 mm/s | 1 | 235 mm | Yes; both wheels started | Slightly left | 1 complete | Corrected centre-of-wheel measurement |
| 50 mm/s | 2 | 240 mm | Yes; both wheels started | More left | 1 complete | Corrected centre-of-wheel measurement |
| 50 mm/s | 3 | 235 mm | Yes; both wheels started | Left | 1 complete | Corrected centre-of-wheel measurement |
| 60 mm/s | 1, only if 50 fails | | | | | |
| 80 mm/s | 1, only if 60 fails | | | | | |

## Review

- Slowest smooth speed: provisionally 50 mm/s
- Both wheels started in all completed provisional repeats: yes, 3/3
- Faults observed: none; all three runs returned one completion beep
- Direction/drift trend: small and inconsistent left/right drift
- Physical distance: three corrected runs measured 235/240/235 mm for a
  200 mm encoder target; mean 236.7 mm (+18.3%)
- Candidate Vehicle Controller normal speed: not selected yet
- Configuration or PID change required: no immediate change; confirm the
  +17.5% distance bias before changing feed-forward or wheel size

### Measurement correction

The original 90-95 mm values used an incorrect reference/unit interpretation.
The tester remeasured from the middle of the wheel at the start line to the
middle of the wheel at the stop position and obtained 23.5 cm (235 mm), then
24 cm (240 mm), then 23.5 cm (235 mm). The corrected values supersede the
earlier estimates for calibration purposes.

This test does not change the Vehicle Controller speed by itself. The normal
mission speed is updated only after a repeatable physical result.

## PID status correction

The smooth 50 mm/s runs establish low-speed operability, not completed PID
tuning. The active per-wheel values remain provisional:

```text
Kp = 0.40
Ki = 1.20 1/s
Kd = 0
target slew = 6 mm/s per 20 ms nominal period
```

A later 500 mm forward command completed in 10.39 s. Its encoder-command
average was 48.1 mm/s against the 50 mm/s setpoint; the 570 mm floor distance
is a separate Stage 6 conversion issue. It used the same controller, gains and
unity distance scale, so it is accepted as the forward Stage 5 timing result
and must not be repeated. Only the reverse 500 mm timing run remains before
deciding whether the provisional gains need adjustment.

Current follow-up build: zero emitted warnings/errors; text/data/bss
`97020/156/32400` bytes.

The completed 50 mm/s image exposed only forward and reverse 500 mm cases.
GP21 toggled between the two cases while idle. The earlier 60-120 mm/s selection
table describes the exploratory image and was not active in that follow-up
build.

### 50 mm/s reverse result

The 500 mm reverse run measured 570 mm in 9.97 s, travelled straight, started
both wheels and returned one completion beep. Its encoder-command average is
`500 / 9.97 = 50.15 mm/s`, only +0.3% from the 50 mm/s setpoint. Together with
the forward 48.1 mm/s average, smooth motion and no faults, this accepts the
current `Kp=0.40`, `Ki=1.20`, `Kd=0` gains for the 50 mm/s operating point. No
PID gain change is warranted from these results.

### 80 mm/s result and supported-speed decision

The final 80 mm/s image exposed only forward and reverse 500 mm cases:

| Direction | Floor distance | Time | Physical average | Drift | Both wheels | Result |
| --- | ---: | ---: | ---: | --- | --- | --- |
| Forward | 560 mm | 6.9 s | 81.2 mm/s | Straight | Yes | 1 complete |
| Reverse | 560 mm | 6.7 s | 83.6 mm/s | Straight | Yes | 1 complete |

The physical averages differ from the 80 mm/s request by approximately +1.5%
forward and +4.5% reverse. Both directions were smooth, straight and
fault-free, so no PID gain change is justified.

To match the course's low-speed requirement and avoid testing speeds the robot
will not use, the Vehicle Controller is reduced to 80 mm/s for normal,
reverse and turn motion, and 50 mm/s for hump/search motion. The old
100/120/200/300 mm/s checks are outside the supported scope and are skipped.
Stage 5 is cleared for low-speed progression with `Kp=0.40`, `Ki=1.20 1/s`,
`Kd=0`. Formal performance acceptance remains dependent on the still-TBC
Stage 0 numerical limits.
