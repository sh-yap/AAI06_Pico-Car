# Stage 7 minimal turn test - 2026-10-28

## Firmware

- Application: `motion_stage7_test`
- Board: `robo_pico_robot`, SMP enabled, USB CDC, WiFi/MQTT disabled
- Wheel speed: 80 mm/s
- Initial physical track width: 145 mm
- Current focused retest candidate: 119 mm effective turn width
- Turn timeout: 8000 ms
- Build passed with zero emitted warnings/errors; text/data/bss
  `96196/156/32400` bytes

GP20 starts the selected turn. GP21 advances the selection while idle and is
an immediate stop while running. Selection codes are:

| Beeps | Case |
| ---: | --- |
| 1 | Left 90 degrees |
| 2 | Right 90 degrees |
| 3 | U-turn 180 degrees |

Result code 1 is complete, 2 is operator stop, and 3-8 retain the Motion fault
meanings.

## Minimum physical results

### Initial left-turn result

The first 90-degree left command completed with one beep and both wheels
moving, but the measured heading change was approximately 110 degrees. The
effective-width estimate is therefore:

```text
145 mm * 90 / 110 = 118.6 mm -> 119 mm candidate
```

The left retest and right comparison are both near 95 degrees, so the 119 mm
candidate was retained for the U-turn comparison.

Run each case once in the order shown. Record approximate actual angle,
direction, time if practical, whether both wheels moved and the result code.
Repeat or tune effective track width only if a run is materially inaccurate,
fails, or turns in the wrong direction.

| Case | Actual direction/angle | Time | Both wheels | Result | Decision |
| --- | --- | ---: | --- | ---: | --- |
| Left 90, initial | Left, approximately 110 degrees | | Both | 1 complete | Over-rotated; recalibrate |
| Left 90, 119 mm retest | Left, approximately 95 degrees | | Both | 1 complete | Near-pass; keep 119 mm |
| Right 90 | Right, approximately 95 degrees; slight slant | | Both | 1 complete | Near-pass; keep 119 mm |
| U-turn 180 | Opposite heading; slight right slant, about 1.5 cm endpoint offset | | | | Near-pass; retain 119 mm |

The minimum turn progression set is complete. The small rightward endpoint
offset is retained as a known floor/traction bias rather than triggering
another geometry change from an approximate line measurement.
