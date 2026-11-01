# Stage 4 loaded motor results - 2026-10-28

## Setup

- Firmware: `motion_stage4_test` (case/result feedback is also sent to the
  Robo-Pico buzzer on GP22; GP19 is optional external LED output)
- Board: Robo-Pico with complete 0.665 kg car
- Power: powerbank only; computer USB disconnected during movement
- Surface:
- Clear lane length:
- Tester: Suen Hong

The original image requested one second by summing RTOS delays, so its earlier
travel measurements must not be treated as exact mm/s. The rebuilt image now
uses the RP2040 microsecond clock as its deadline and optionally prints the
actual elapsed microseconds and wheel counts after braking. Earlier results
below remain approximate historical evidence.
Measure from the same point on the car each time. Record right drift as `R`,
left drift as `L`, or `straight`. Record whether each wheel visibly starts.

## Minimum clearance run - do this next

The earlier sweep is enough to skip unused 275-350 permille cases. Select
forward 200 permille and complete five trials, then select reverse 200 permille
and complete five trials. If either direction has any failed start, switch
only that direction to 225 permille and restart its five-trial set.

| Direction | PWM permille | Trial | Travel mm | Drift | Both wheels start | Result code | Notes |
| --- | ---: | ---: | ---: | --- | --- | ---: | --- |
| Forward | 200 | 1 | | | | | |
| Forward | 200 | 2 | | | | | |
| Forward | 200 | 3 | | | | | |
| Forward | 200 | 4 | | | | | |
| Forward | 200 | 5 | | | | | |
| Reverse | 200 | 1 | | | | | |
| Reverse | 200 | 2 | | | | | |
| Reverse | 200 | 3 | | | | | |
| Reverse | 200 | 4 | | | | | |
| Reverse | 200 | 5 | | | | | |

Clearance criteria: both wheels start in all five trials in each direction;
no result code 6 or 7; test setup fields are completed. High/nominal/minimum
battery testing, WiFi/MQTT loading and unused high-duty cases are deferred.

### User run reported after the software audit

The tester completed five forward trials believed to be at 200 permille. The
three reverse trials were initially reported as 200 permille, then corrected:
the tester believes reverse 150 permille was selected. A separate forward
150-permille check also travelled about 90 mm. Result beep codes and explicit
per-wheel start observations were not recorded.

| Direction | Selected PWM | Trial | Approx. travel | Drift/heading | Result code / wheel start |
| --- | ---: | ---: | ---: | --- | --- |
| Forward | 200 permille (believed) | 1 | 110 mm | Left | Not recorded; vehicle moved |
| Forward | 200 permille (believed) | 2 | 130 mm | Straight | Not recorded; vehicle moved |
| Forward | 200 permille (believed) | 3 | 130 mm | Straight | Not recorded; vehicle moved |
| Forward | 200 permille (believed) | 4 | 130 mm | Straight | Not recorded; vehicle moved |
| Forward | 200 permille (believed) | 5 | 130 mm | Straight | Not recorded; vehicle moved |
| Reverse | 150 permille (corrected, believed) | 1 | 90 mm | Straight | Not recorded; vehicle moved |
| Reverse | 150 permille (corrected, believed) | 2 | 90 mm | Slightly faced right | Not recorded; vehicle moved |
| Reverse | 150 permille (corrected, believed) | 3 | 90 mm | Straight | Not recorded; vehicle moved |
| Forward | 150 permille | extra check | 90 mm | Not recorded | Not recorded; vehicle moved |

The user elected to skip the remaining two reverse repetitions and proceed to
low-speed closed-loop work. Therefore Stage 4 is not formally cleared against
the five-repeat criteria. The results are still sufficient to show that raw
200 permille is faster than desired for the course and that raw PWM percentage
is not an appropriate mission-speed command.

## Initial sweep

### User-reported runs

These observations were made with the one-second raw-PWM firmware. Distances
are the user's approximate floor measurements; a second value means a repeat
of the same case.

| Direction | PWM permille | Approx. travel | Observation |
| --- | ---: | ---: | --- |
| Forward | 150 | 18 cm; repeat pivoted about 95 degrees | Marginal; one wheel moved on the repeat |
| Forward | 175 | 39 cm; 39 cm | First relatively straight; second strong right swerve |
| Forward | 200 | 41 cm; 41 cm | Similar travel on both runs |
| Forward | 225 | 53 cm; 56 cm | Both runs moved; rightward bias remains |
| Forward | 250 | 64 cm; 63 cm | Strong right swerve |
| Forward | 350 | 91 cm | Strong right swerve; high-duty case is optional for this short course |
| Reverse | 150 | 28 cm | Single run; low-duty case remains to be repeated |
| Reverse | 200 | about 43 cm | Single run |
| Reverse | 225 | about 50 cm | Single run |

The forward 150‰ repeat is a failed start/repeatability point. The forward
175‰ point is marginal because heading changes substantially between repeats.
The rightward drift agrees with the earlier loaded encoder runs: the left
wheel generally travels farther than the right at equal raw PWM. Do not alter
the calibrated wheel diameters from this result; the remaining correction
belongs in loaded per-wheel speed/feed-forward tuning.

| Case | Direction | PWM permille | Travel mm | Drift | Left starts | Right starts | GP19 result code | Notes |
| ---: | --- | ---: | ---: | --- | --- | --- | ---: | --- |
| 1 | Forward | 150 | | | | | | |
| 2 | Forward | 175 | | | | | | |
| 3 | Forward | 200 | | | | | | |
| 4 | Forward | 225 | | | | | | |
| 5 | Forward | 250 | | | | | | |
| 6 | Forward | 275 | | | | | | |
| 7 | Forward | 300 | | | | | | |
| 8 | Forward | 325 | | | | | | |
| 9 | Forward | 350 | | | | | | |
| 10 | Reverse | 150 | | | | | | |
| 11 | Reverse | 175 | | | | | | |
| 12 | Reverse | 200 | | | | | | |
| 13 | Reverse | 225 | | | | | | |
| 14 | Reverse | 250 | | | | | | |
| 15 | Reverse | 275 | | | | | | |
| 16 | Reverse | 300 | | | | | | |
| 17 | Reverse | 325 | | | | | | |
| 18 | Reverse | 350 | | | | | | |

## Repeatability around the start threshold

After the initial sweep, use GP21 to select the lowest case that started both
wheels. Press GP20 five times without changing the case and fill this table.
Repeat for forward and reverse.

| Direction | PWM permille | Trial | Travel mm | Drift | Left starts | Right starts | GP19 code |
| --- | ---: | ---: | ---: | --- | --- | --- | ---: |
| Forward | | 1 | | | | | |
| Forward | | 2 | | | | | |
| Forward | | 3 | | | | | |
| Forward | | 4 | | | | | |
| Forward | | 5 | | | | | |
| Reverse | | 1 | | | | | |
| Reverse | | 2 | | | | | |
| Reverse | | 3 | | | | | |
| Reverse | | 4 | | | | | |
| Reverse | | 5 | | | | | |

## Review (complete after physical testing)

- Lowest repeatable forward start PWM:
- Lowest repeatable reverse start PWM:
- First PWM with acceptable straightness:
- Any result code 6 or 7:
- Configuration changes selected from these measurements:
- Five-repeat forward start passed:
- Five-repeat reverse start passed:
- Stage 4 clearance decision:
- Tester decision: remaining reverse repeats intentionally skipped; proceed
  provisionally to closed-loop low-speed testing.
