# Buddy 2 motion

Buddy 2 owns drive motors and wheel encoders. The Vehicle Controller submits
velocity, distance and turn commands; sensor tasks do not write motor PWM.

## Source and API

| File | Responsibility |
| --- | --- |
| `motion_task.c/.h` | 20 ms RTOS control loop, queue, stop event and status |
| `motion_motor.c/.h` | Motor PWM, brake/coast and direction changes |
| `motion_encoder.c/.h` | Quadrature ISR and sequence-checked snapshots |
| `motion_control.c/.h` | Speed, geometry, validation, PID and control calculations |
| `motion_config.c/.h` | Wiring, calibration and output limits |
| `motion_pack.c/.h` | Packed command/status storage |
| `motion_clock.h` | Microsecond timing access |

`motion_init()` initializes the subsystem and `motion_start_task()` starts it.
`motion_submit()` validates and queues a command. The queue send uses polling,
but acquiring its mutex can wait up to 30 ms: the whole call is not strictly
non-blocking. Queue depth is four. Rejection returns INVALID, UNINITIALISED,
BUSY, QUEUE_FULL or HARDWARE as applicable.

`motion_stop()` takes the command mutex with `TMO_FEVR`, posts a stop event,
and discards commands queued before that stop. The Motion task performs the
brake; successful return is not confirmation that physical braking has finished.
`motion_get_status()` returns a mutex-protected decoded status.

Public units are mm, mm/s, millidegrees, millidegrees/s and ms.
Forward is positive; positive angular rate/turn is vehicle-right.
Distance and turn progress use per-command encoder origins.

## Wiring and configuration

Values below come from `motion_config_default()` and `motion_task.c`.

| Item | Value |
| --- | --- |
| Left motor A/B | GP8 / GP9 |
| Right motor A/B | GP10 / GP11 |
| Left harness encoder phases | GP16 / GP17; logical decoder A/B is GP17 / GP16 |
| Right decoder A/B | GP4 / GP5 |
| Counts per wheel revolution, L/R | 2536 / 2529 |
| Effective wheel diameter, L/R | 57.7 / 57.6 mm |
| Distance scale | Q10 1167 (1167/1024) |
| Effective turn track | 119 mm |
| Command speed limit | 300 mm/s |
| Encoder physical plausibility bound | 1000 mm/s |
| PWM maximum / deadband | 900 / 80 permille |
| Startup PWM / boost duration | 200 permille / 250 ms |
| PID, both wheels | Kp 0.60, Ki 1.20, Kd 0 |
| Feed-forward offset / slope, both wheels | 140 permille / Q10 512 |
| Additional slope above 100 mm/s | Q10 16 |
| Reverse boost, L/R | 6 / 0 permille |
| Target slew | 6 mm/s per nominal 20 ms |
| PWM slew | 25 permille per nominal 20 ms |
| Straight correction gain / cap | 0.10 / 30 mm/s |
| Steered inner-wheel brake threshold | Below 35 mm/s |

The Motion task uses priority 4, a static 4096-byte stack and processor 1
affinity. Calibration constants do not establish accuracy across all floors,
loads or battery conditions. Encoder correction compares wheel travel; it
cannot independently observe chassis heading or tyre slip.

The BSP explicitly initializes the RP2040 watchdog TICK after CLK_REF setup.
This supplies the timing reference used for encoder speed and plausibility.
It is not a task-health watchdog. See the
[encoder incident](../../evidence/motion/ENCODER_FAULT_INCIDENT.md).

## Fault and recovery behaviour

| Condition | Existing response |
| --- | --- |
| Impossible count delta, speed calculation failure or more than eight invalid transitions in a sample | Encoder fault; brake both motors and retain diagnostic detail |
| A wheel target is at least 50 mm/s but its speed remains below 20 mm/s for 1000 ms | Stall fault; brake both motors |
| Nonzero command timeout expires | Timeout fault; brake both motors |
| PWM/brake write failure | Latch hardware/configuration fault |
| Target reached | Brake both wheels before publishing COMPLETE |
| Stop event | Brake, clear earlier queued work and acknowledge recoverable latched faults |

Timeout, stall and encoder faults reject retries until explicit stop
acknowledgement. Hardware/configuration faults require reinitialization.
Detecting a failed brake write does not prove the physical motor has stopped.
Continuous VC velocity commands set timeout to zero, so this interface does
not expire unchanged drive commands.

## Build and verification

Use the [integrated build guide](../TEST_INTEGRATE.md) for full firmware.
Motion-only applications include `motion_motor_test`, `motion_encoder_test`,
`motion_closed_loop_test`, `motion_closed_loop_button_test`,
`motion_stage4_test` through `motion_stage7_test`, `motion_route_test`,
`motion_clock_test` and `motion_sim_test`; the build file selects their entry
points. Robot hardware tests use `BOARD=robo_pico_robot CONSOLE=usb_cdc`.

Run `make -C tests/motion test` for calculations, real-loop mocks, encoder
snapshot/IRQ behaviour, USB isolation, packing and motor error paths.
See [test scope](../../tests/motion/README.md) and
[recorded measurements](../../evidence/motion/FINAL_MOTION_VERIFICATION.md).
