# Buddy 4 terrain

Buddy 4 owns LSM303DLHC access and terrain processing.
`b4_cfg.c` configures I2C1 at 400 kHz on GP2 SDA and GP3 SCL, accelerometer
address 0x19 and magnetometer address 0x1E.
The device contains an accelerometer and magnetometer, with no gyroscope.

`b4_task.c` samples acceleration at 100 Hz and sends a terrain word to the VC
every second sample (50 Hz). `b4_proc.c` implements calibration, filtering,
motion classification, ramp detection, height estimation and the sharp-change
cue. Height uses the configured lateral axis and 115000 um lever.

The 32-bit terrain word contains health (2 bits), turn (2), current height
(10), maximum height (10), height-valid (1), hump count (4), lift cue (1)
and reserved bits (2). Height units are 0.1 mm. During a hump the live height
is positive and not yet valid; completion sets a valid final height or clears
it if measurement failed. A nonzero estimate is not itself proof of validity.

The VC sends a drive word containing driving/moving flags, left/right wheel
speeds, sequence and target-step size. This supports movement-aware noise
learning. START requests level reference, noise measurement and calibration
completion through `b4_task.h`. A run-reset request clears the run maximum.

An absent IMU does not prevent task creation: the task retries and reports
FAULT health. VC freshness is tracked separately from the terrain word.
Terrain telemetry includes motion class, tilt in milli-g, current/maximum
height and estimated turn rate. Turn-rate output is an estimate, not direct
gyroscope measurement.

Run `make -C tests/buddy4_proc test` for processing and packed-status checks.
`tests/buddy4/` contains car-test C sources, not a Make-based host suite.
The current processing suite fails seven of 39 checks involving clean
crossings and vibration. Its separate packed-status suite passes 34 checks.
See [software verification](../../evidence/SOFTWARE_VERIFICATION.md).
Host results do not establish physical hump height accuracy.
