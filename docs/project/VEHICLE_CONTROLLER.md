# Vehicle Controller

`vc.c` contains hardware-independent mission logic. `vc_task.c` gathers
subsystem data, calls `vc_step()` every 20 ms and executes its outputs.
`vc_car_main.c` initializes all five buddies and starts the controller.
Motion remains the drive-motor owner.

## Mission states

| State | Behaviour |
| --- | --- |
| INITIALISE | Evaluate initialization; failure leads to EMERGENCY_STOP |
| READY | Stopped; START begins calibration |
| CALIBRATE | Still reference, line sweep, threshold/noise calibration and return; failure returns to READY |
| FOLLOW_LINE | Apply line steering/speed, retain mapped barcode intent and watch obstacles |
| HANDLE_JUNCTION | Transient selection of straight or turn |
| TURNING | Encoder manoeuvre with junction advance and branch-catching logic |
| OBSTACLE_SCAN | Stop and request scan; choose continue, bypass, reverse/rescan or emergency stop |
| BYPASS | Execute planned turns/legs, path check, edge watch and line detection |
| REACQUIRE_LINE | Search using line inputs and bounded pivots |
| EMERGENCY_STOP | Brake and latch; operator STOP plus cleared serious faults and stationary wheels returns to READY |
| FINISHED | Stopped; supports START calibration or STOP to READY |

Serious faults have priority over normal navigation. STOP aborts active
operation. Pickup detection can finish a run. Line-loss search is bounded to
90 degrees per side and a 9000 ms backstop; the post-bypass bound is 45 degrees.

## Interfaces and configured timing

- Motion: `motion_submit()`, `motion_stop()` and status snapshots.
- Buddy 3: packed line/barcode words and START calibration interface.
- Buddy 4: terrain words, calibration requests and a drive-feedback word.
- Buddy 5: status, scan/profile, path-check and edge-watch interfaces.
- Comms: command queue, state/fault publication, telemetry and run summary.

The VC task has priority 5 and a 4096-byte stack. It runs on processor 2 in
SMP builds. Buddy 3 is pinned to processor 1 for ADC-driver ownership.

Current source constants are 100 mm/s normal, 50 mm/s at full curve,
100 mm/s hump, 120 mm/s search setting and 80 mm/s encoder-turn wheel speed.
The bypass leg setting is 150 mm/s. Turn timeout is 8000 ms, scan 3000 ms,
bypass 25000 ms and normal reacquisition 4000 ms.

Drive steering is scaled by 350 millidegrees/s per steer unit and capped
against inner-wheel geometry. Changes of at least one steer unit or 10 mm/s
trigger resubmission; transitions to/from zero are always sent.
Velocity commands use `timeout_ms = 0`. A two-step (40 ms) stop hold separates
manoeuvres. Submit failures have bounded retries and become command faults.

## Fault word

Positions come from `vc_task.h`; the transmitted mask contains ten bits.

| Bit | Meaning |
| --- | --- |
| 0 | Motion stall |
| 1 | Encoder fault |
| 2 | Motion command/hardware/status failure |
| 3 | IMU unavailable or unhealthy (published, excluded from serious-stop mask) |
| 4 | Reserved |
| 5 | Ultrasonic fault |
| 6 | Calibration contrast failure |
| 7 | Calibration line not found |
| 8 | Calibration Buddy 4 failure |
| 9 | Calibration sensor gap |

Line data becomes stale after 100 ms; terrain data after 100 ms is treated as
not ready. IMU unavailability prevents successful START calibration, while
during driving its status controls terrain handling without alone causing a
serious emergency stop.

VC state codes are internal. Comms maps CALIBRATE to READY, TURNING and
HANDLE_JUNCTION to EXECUTE_NAV, REACQUIRE_LINE to LINE_RECOVERY and
EMERGENCY_STOP to SAFE_STOP.

Run `make -C tests/vc test` for the pure state machine.
The [operation guide](../TEST_INTEGRATE.md) covers building and MQTT commands.
