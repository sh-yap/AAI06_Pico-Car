# AAI06 project documentation

The integrated application is `APP=vc_car BOARD=robo_pico_robot SMP=1`.
It runs the robot subsystems on one Pico W using the supplied micro T-Kernel
RP2040 port. The Maker Pi Pico boards are development platforms.

| Document | Purpose |
| --- | --- |
| [Implemented behaviour](REQUIREMENTS.md) | Source references for the robot's functions and current limits |
| [Motion](BUDDY2_MOTION.md) | Buddy 2 API, wiring, control constants and fault handling |
| [Line and barcode](BUDDY3_LINE_BARCODE.md) | Buddy 3 acquisition, calibration and navigation mapping |
| [Terrain](BUDDY4_TERRAIN.md) | Buddy 4 IMU processing and terrain-word interface |
| [Obstacle](BUDDY5_OBSTACLE.md) | Buddy 5 ranging, scanning and avoidance interface |
| [Vehicle Controller](VEHICLE_CONTROLLER.md) | Mission decisions, integration and fault policy |
| [Communications](../COMMS_GUIDE.md) | MQTT ownership, commands and telemetry scheduling |
| [Binary format](../BINARY_FORMAT.md) | On-wire fields and decoder |
| [Build and operation](../TEST_INTEGRATE.md) | Full-car build, flash, operation and host checks |

[Current software verification](../../evidence/SOFTWARE_VERIFICATION.md) records
the passing host suites and the existing Buddy 4 processing failures.
[Motion verification](../../evidence/motion/FINAL_MOTION_VERIFICATION.md)
contains historical measurements with their original scope. Source behaviour
and host tests do not establish physical accuracy or full-course completion.
The root README and platform guides retain the supplied kernel's attribution,
licensing and platform qualification information.
