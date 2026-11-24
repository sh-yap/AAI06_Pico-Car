# Implemented robot behaviour

This is a source-based description of the submitted application, not a
certificate of measured mission performance. The executable constants and
interfaces in the files below define its current configuration.

| Function | Implementation |
| --- | --- |
| Autonomous line following | `buddy3_task.c` samples IR inputs; `line_follow.c` estimates line position; `vc.c` chooses steering and speed |
| START calibration | VC stops, takes an IMU level reference and pivots over the line to obtain contrast, thresholds and IMU noise; failed calibration returns to READY |
| Barcode navigation | `barcode.c` decodes Code 39 using encoder distance; `nav_map.c` applies the mapping received through MQTT |
| Junction movement | VC handles left, right, straight and U-turn intent; Buddy 2 executes encoder-based motion |
| Wheel control | `motion_task.c` runs every 20 ms with separate wheel PID, feed-forward, slew limits and straight correction |
| Hump detection and maximum | `b4_proc.c` processes IMU samples; `b4_task.c` reports current/maximum height and validity |
| Obstacle detection | `obstacle_monitor.c` evaluates valid and invalid range readings |
| Obstacle scan and bypass | `obstacle_scan.c`, `obstacle_plan.c` and VC implement scan, recommendation, path check, bypass and line recovery |
| Pickup finish | VC combines line-sensor spells and the terrain lift cue, with an uncued moving-car bound, to finish a run |
| Wireless operation | `comms.c` accepts START/STOP/STATUS/MAP and publishes snapshots, events, heartbeat and summary |
| Network recovery | CYW43 and MQTT services reconnect independently of mission decisions |
| Fault handling | Motion brakes and latches faults; serious VC faults enter EMERGENCY_STOP with operator acknowledgement |

## Execution and ownership

The platform is Pico W/RP2040, Robo-Pico, C, GNU Make and micro T-Kernel SMP.
The integrated entry point is `vc_car_main.c`. Motion owns drive PWM and encoder
acquisition, Buddy 3 owns IR acquisition, Buddy 4 owns IMU access, Buddy 5 owns
ultrasonic/servo access, and the radio service owns WiFi/lwIP/MQTT.
The VC coordinates movement through Motion's public API.

Messages and snapshots use fixed-size storage, packed words, mutexes or
sequence checks as defined by each module. Motion and VC use 20 ms periods;
Buddy 3 uses a 10 ms cyclic wake; Buddy 4 samples at 100 Hz and sends terrain
words at 50 Hz. These are configured rates, not measured worst-case deadlines.

## Current limits

- Line following uses centre and right IR sensors; a third IR input reads barcodes.
- Barcode characters have no navigation meaning until a valid MAP command is received.
- Terrain height is an estimate with a validity flag; a detected hump does not
  automatically imply a valid height. LSM303DLHC has no gyroscope.
- Servo scanning is clamped to 60-120 degrees.
- Normal VC speed is 100 mm/s, full-steer curve speed 50 mm/s, hump speed
  100 mm/s and bypass-leg speed 150 mm/s in the current source.
- Continuous drive commands use a zero timeout. There is no command-refresh
  expiry in that path. Motion stop waits on its command mutex without a timeout.
- The BSP initializes the watchdog tick as a timer reference; that write is
  not a software-health reset watchdog.
- Host tests and historical measurements are scoped evidence; they do not
  prove a universal 20 ms brake bound, final stopping distance or full-course accuracy.
- The current Buddy 4 processing host suite fails seven checks; see
  [software verification](../../evidence/SOFTWARE_VERIFICATION.md).
