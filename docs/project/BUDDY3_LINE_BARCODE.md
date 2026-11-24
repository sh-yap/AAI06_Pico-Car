# Buddy 3 line and barcode

`buddy3_task.c` samples the IR subsystem on a 10 ms cyclic wake and publishes
packed line/barcode data through a four-slot message buffer.
The integrated entry point pins the task to processor 1 with priority 5 and
a static 4096-byte stack to match ADC ownership.

`ir_sensor.c` assigns barcode GP28, centre GP27 and right GP26.
The centre/right sensors feed `line_follow.c`; the barcode sensor feeds
`barcode.c`. Line state codes are LEFT, CENTRE, RIGHT, LOST and JUNCTION;
signed line error uses the range -127 to +127. Motion and VC execute steering.

START calibration is requested by the VC. It records line contrast,
thresholds, centre full-black level and sensor overlap, and reports calibration
failure rather than driving with an accepted bad sweep. Default thresholds
in `line_follow.c` are centre 1563 and right 1675 ADC counts.

Barcode widths are derived from wheel encoder travel, with sensor-offset
geometry in `buddy3_task.c`. `barcode.c` recognizes Code 39 characters;
`nav_map.c` translates a decoded character into navigation intent.
The active mapping is one aligned 32-bit word. A rejected MAP command leaves
the previous mapping intact. Before a mapping arrives, decoded barcodes have
no turn command. See [commands](../COMMS_GUIDE.md).

`make -C tests/buddy3 test` runs the host checks. These checks do not measure
recognition accuracy under physical lighting, speed or track conditions.
