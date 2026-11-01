# Stage 6 distance-calibration preparation - 2026-10-28

## Status

The first three 50 mm/s Stage 5 repeats measured 235, 240 and 235 mm for a
200 mm encoder-distance command. The mean physical result is 236.7 mm, or
approximately +18.3%. This is repeatable enough to justify a calibration
investigation, but not enough to choose a new wheel diameter or scale: the
result could include proportional conversion error, stopping behaviour, floor
slip or a measurement-reference error.

## Firmware prepared

- Application: `motion_stage6_test`
- Board: `robo_pico_robot`
- Powerbank-only; WiFi/MQTT disabled
- GP20 starts the selected move
- GP21 stops a running move; while idle it selects the next case
- GP21 long hold resets to the first case
- Speed: 50 mm/s for every case
- Current minimal cases: 500 mm forward, then 100 mm reverse
- One completion beep; fault codes match the Stage 5 image
- Serial report is emitted after braking and includes target, encoder-derived
  travel, left/right counts, invalid transitions, sequence ID, state and fault
- Current build: passed with zero emitted warnings/errors; text/data/bss
  `97256/156/32400` bytes

## Calibration parameter prepared

`motion_config_t.distance_scale_q10` is now active at `1167`, meaning 1.140.
The distance conversion multiplies both wheel conversions by this scale and
validates it in the range 512-2048. It was applied only after the supported
Stage 5 speed points were accepted.

## First physical pass

Run the 500 mm forward case once from a fresh boot, then select and run the
100 mm reverse case once. Record:

```text
case, target_mm, result_beeps, both_wheels_started, physical_mm, drift,
serial_line, battery/source, surface
```

Do not run the ten-repeat gate, WiFi/MQTT, extra distances or faster speeds.
Repeat or add a correction only if either minimum run is clearly inaccurate,
drifts, faults or fails to start.

## Earlier uncorrected-scale results

All five cases completed without a reported error, both wheels moved, and the
vehicle remained straight:

| Direction | Target | Measured floor distance | Approx. error | Result |
| --- | ---: | ---: | ---: | --- |
| Forward | 100 mm | 100 mm | 0% | complete |
| Forward | 200 mm | 230 mm | +15% | complete |
| Forward | 300 mm | 340 mm | +13.3% | complete |
| Forward | 500 mm | 570 mm | +14% | complete |
| Reverse | 100 mm | 125 mm | +25% | complete |

The 200-500 mm forward results show a fairly consistent proportional positive
bias, while the 100 mm forward result is near target and the 100 mm reverse
result has a larger directional bias. No calibration constant has been changed
yet. The Stage 6 firmware is being extended to print encoder-measured elapsed
time and left/right speed statistics after each brake, so physical travel can
be compared against both encoder distance and measured velocity.

## Speed telemetry follow-up

The serial report contains `elapsed_ms`, signed `mean_speed` and `peak_speed`
when a console is available. GP19 is only an optional external LED output; it
is not a visible onboard Robo-Pico LED, and no external LED is fitted in the
reported powerbank setup. Therefore the reported one-light events are not
speed evidence and the LED test must not be repeated. If physical speed is
needed, use one timed floor run and calculate distance divided by elapsed time.

## Timed physical-speed result

The 500 mm case travelled 570 mm in 10.39 s:

```text
physical average speed = 570 mm / 10.39 s = 54.9 mm/s
```

This is approximately 9.7% above the 50 mm/s command. The 14% distance ratio
(`570 / 500`) agrees with the longer forward-distance runs and confirms a
systematic conversion bias rather than a fixed stopping offset.

## Provisional calibration candidate

The measured 570/500 ratio gives the active common loaded distance scale of
Q10 1167 (1.140). Because the same conversion feeds distance and speed
estimation, this should make the previous physically fast 50 mm/s motion and
long 500 mm travel agree more closely with their commands.

The remaining minimum verification set is one 500 mm forward run and one
100 mm reverse run. The reverse case previously measured 125 mm; measure it
after the common correction before considering any reverse-only compensation.

## Corrected-scale verification

The first run with the active Q10 1167 scale was the required 500 mm forward
case:

| Direction | Target | Measured floor distance | Time | Physical average | Drift | Wheels | Result |
| --- | ---: | ---: | ---: | ---: | --- | --- | --- |
| Forward | 500 mm | 500 mm | 10.04 s | 49.8 mm/s | Straight | Both started | 1 complete |

This confirms the distance correction on the forward path: the command reached
the requested floor distance without drift or a start/fault problem.

The follow-up reverse case also completed exactly:

| Direction | Target | Measured floor distance | Time | Physical average | Drift | Wheels | Result |
| --- | ---: | ---: | ---: | ---: | --- | --- | --- |
| Reverse | 100 mm | 100 mm | 2.00 s | 50.0 mm/s | Straight | Both started | 1 complete |

The minimal Stage 6 forward/reverse progression set is therefore complete.
No reverse-only correction is indicated by these results.

## Optional long straight-run observation

The rebuilt Stage 6 image adds one 1500 mm forward case at 50 mm/s. This is
not required to reopen Stage 6 calibration; it is a single practical check for
cumulative lateral drift over a longer distance. Record the start/end wheel
centre offset and stop the run if the vehicle approaches the edge of the safe
area.

Observed result: the 500 mm run travelled approximately 500 mm, first drifting
slightly left and then correcting slightly right. The 1500 mm run travelled
approximately 1510 mm and swerved right without an obvious correction. The
distance result is accurate, but the current encoder correction only compares
normalised left/right wheel travel; it cannot observe absolute chassis heading
or side-slip when both wheel encoders remain similar. No gain or wheel-trim
change is made from this single approximate long-run observation. A repeat
1500 mm run was relatively straight with a slight left drift, so the earlier
rightward drift was not repeatable. Starting heading/placement is therefore a
likely contributor, along with normal floor and traction variation.
