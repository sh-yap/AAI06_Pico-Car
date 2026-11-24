# Buddy 5 obstacle sensing

Buddy 5 owns the HC-SR04 and servo. It publishes range health, scan results,
profiles, path checks and bypass recommendations. The Vehicle Controller
chooses movement and Buddy 2 executes it.

## Wiring and limits

The current `obstacle_config.h` assigns trigger GP0, echo GP1 and servo GP12.
The robot profile disables the UART0 mirror on GP0/GP1. Echo uses PWM
level-gated timing; its PWM slice must not conflict with motor outputs.
The servo uses a 20000 us period, centre 1540 us, safe pulse bounds
1100-1900 us and angle bounds 60-120 degrees.

The ping interval is 60 ms; the trigger pulse is 12 us and echo wait is 40 ms.
Conversion uses 343 m/s. Echo bounds are 117-23324 us. Status distinguishes
OK, NO_ECHO, TOO_CLOSE, TOO_FAR, TIMEOUT and NOT_READY.

## Processing and controller interface

| Source | Responsibility |
| --- | --- |
| `obstacle_ultrasonic.c` / `obstacle_servo.c` | Peripheral drivers |
| `obstacle_monitor.c` | Ahead/blind/lost-obstacle and sensor-health decisions |
| `obstacle_scan.c` / `obstacle_calc.c` | Scan samples, geometry and profile |
| `obstacle_plan.c` | Bypass geometry and edge-width calculation |
| `obstacle_task.c/.h` | Task, freshness, event flags and request/result interfaces |

MONITOR ranges forward. The configured stop threshold is 200 mm with two
readings, blind latch 100 mm, fault threshold five readings and recovery
three readings. Status older than 200 ms is unavailable, not a clear path.

Scanning uses 15-degree increments over the configured servo range, three
pings per angle and at least two valid samples. The profile carries distance,
width, clearance and a recommendation: STOP, CONTINUE, TURN_LEFT, TURN_RIGHT
or REVERSE_RETRY. Invalid readings retain their status.

The controller requests a scan while stopped. After a bypass pivot it requests
a path check. During bypass leg 1 an edge watch looks at the obstacle face;
those side-looking readings are not forward collision guards. Width uses the
planned geometry and a centred-obstacle assumption; the result distinguishes
FOUND, AT_LEAST and NO_FACE.

Read `obstacle_task.h` for request/result contracts and event bits
AHEAD, FAULT, SCAN_DONE, PATH_DONE and EDGE_DONE. The task alone owns the
event bits. It uses priority 3 and a 1024-byte stack.

## Build and tests

The integrated application is `vc_car`. Dedicated applications are
`ultrasonic_test`, `servo_test` and `obstacle_task_test`, selected in the
build Makefile with `BOARD=robo_pico_robot CONSOLE=usb_cdc`.

Run `make -C tests/obstacle test` for host calculation, monitor, scan,
planning and packed-data tests. The 60-120 degree clamp is an implemented
range limit, not proof of coverage outside that range.
