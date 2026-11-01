# Buddy 2 raised-wheel closed-loop check - 2026-10-27

Branch: `buddy2-motion-catchup`  
Firmware: `motion_closed_loop_test`, Robo-Pico, SMP, USB CDC  
Setup: tester held the vehicle clear of the surface so both wheels were
unloaded. Results came from the tester's pasted serial log. Supply voltage was
not measured. These results must not be treated as loaded-floor calibration.

## Results

The reported means below use the settled portion of each run (samples 8
onward, excluding a final stopped sample where present).

| Key / command | Settled left speed | Settled right speed | Result |
| --- | ---: | ---: | --- |
| `d`: 100 mm/s, +30000 mdeg/s | 131.0 mm/s | 69.9 mm/s | Positive angular rate produces a right curve: left wheel faster |
| `a`: 100 mm/s, -30000 mdeg/s | 67.9 mm/s | 134.6 mm/s | Negative angular rate produces a left curve: right wheel faster |
| `r`: -100 mm/s | -102.4 mm/s | -102.5 mm/s | Reverse signs and unloaded tracking pass |
| `1`: +100 mm/s | 100.8 mm/s | 100.8 mm/s | Forward signs and unloaded tracking pass |

For the curve commands, the configured geometry produces nominal wheel
targets of approximately 137/63 mm/s for `d` and 63/137 mm/s for `a`.
Every captured sample reported `fault=0` and encoder invalid-transition counts
of `0/0`.

The final `d` and forward samples showed `state=STOPPED` and `pwm=0/0`. The
left-curve and reverse dumps raced the asynchronous stop request and ended on
a `RUNNING` sample even though their headings claimed the motors were braked.
This exposed a diagnostic-only defect, not a Motion control fault. The test
application was changed afterward to wait for the Motion task to report
`STOPPED`, `COMPLETE`, or `FAULT` before dumping the capture. The corrected
firmware rebuilt without warnings.

## Conclusion

- Forward/reverse encoder signs are correct.
- Positive angular rate = vehicle-right and negative = vehicle-left are
  physically confirmed with the wheels raised.
- Unloaded closed-loop speed tracking is balanced at 100 mm/s.
- Discrete `-90`, `+90`, `180` turns and 100 mm distance completion still need
  raised-wheel checks with the corrected diagnostic firmware.
- Loaded start-up, straightness, speed tuning, braking distance and floor-turn
  accuracy remain unverified.

