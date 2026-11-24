# MQTT binary format

The wire implementation is `app_program/buddy1_vehicle_comms/comms.c` with
widths in `comms_codec.h`; `tools/robot_decode.ps1` provides the matching
laptop table.

## Encoding

Binary version 2 starts with a 2-bit version and a 32-bit millisecond timestamp.
Fields follow in the order below, least-significant bit first. Signed fields
use two's complement. The final byte is padded to a whole byte; message
length is ceil((34 + field bits)/8). Numeric writers clamp to field range.
JSON builds use version 1 and named keys/enums. Internal packed records are
separate from the wire layout; their word alignment must not be used to infer
message byte offsets.

The following tables transcribe the executable decoder field table.
`s` means signed, `u` unsigned and named fields contain enum indices.

## robot/telemetry/motion

| Field | Type | Bits |
| --- | --- | ---: |
| spd_l_mm_s | s | 10 |
| spd_r_mm_s | s | 10 |
| enc_l | s | 19 |
| enc_r | s | 19 |
| dist_mm | s | 16 |
| tgt_l_mm_s | s | 10 |
| tgt_r_mm_s | s | 10 |
| slew_l_mm_s | s | 10 |
| slew_r_mm_s | s | 10 |
| pwm_l | s | 11 |
| pwm_r | s | 11 |

Payload: 22 bytes, including header.

## robot/telemetry/line

| Field | Type | Bits |
| --- | --- | ---: |
| pos | enum Line | 3 |
| err | s | 8 |
| sensors | u | 2 |
| centre_adc | u | 12 |
| right_adc | u | 12 |

Payload: 9 bytes, including header.

## robot/telemetry/terrain

| Field | Type | Bits |
| --- | --- | ---: |
| motion | enum Motion | 3 |
| tilt_mg | s | 11 |
| hump_dmm | u | 10 |
| max_hump_dmm | u | 10 |
| turn_dps | s | 11 |

Payload: 10 bytes, including header.

## robot/telemetry/obstacle

| Field | Type | Bits |
| --- | --- | ---: |
| dist_mm | u | 12 |
| status | enum Range | 3 |
| state | enum Sonar | 3 |
| silent | u | 8 |

Payload: 8 bytes, including header.

## robot/heartbeat

| Field | Type | Bits |
| --- | --- | ---: |
| seq | u | 16 |
| state | enum States | 4 |
| fault | u | 10 |
| ev_drop | u | 8 |

Payload: 9 bytes, including header.

## robot/state

| Field | Type | Bits |
| --- | --- | ---: |
| state | enum States | 4 |
| fault | u | 10 |

Payload: 6 bytes, including header.

## robot/event/barcode

| Field | Type | Bits |
| --- | --- | ---: |
| code | enum Barcode | 6 |
| command | enum Nav | 3 |

Payload: 6 bytes, including header.

## robot/event/hump

| Field | Type | Bits |
| --- | --- | ---: |
| peak_dmm | u | 10 |
| max_dmm | u | 10 |

Payload: 7 bytes, including header.

## robot/event/calibration

| Field | Type | Bits |
| --- | --- | ---: |
| result | enum CalResult | 3 |
| centre_low | u | 12 |
| centre_high | u | 12 |
| centre_thr | u | 12 |
| centre_full | u | 12 |
| right_low | u | 12 |
| right_high | u | 12 |
| right_thr | u | 12 |
| barcode_low | u | 12 |
| barcode_high | u | 12 |
| barcode_thr | u | 12 |
| barcode_learned | u | 1 |
| first_right | u | 1 |
| overlap | s | 9 |
| b4_floor_mg | u | 9 |
| b4_samples | u | 15 |
| sweep_deg | u | 8 |

Payload: 25 bytes, including header.

## robot/event/obstacle

| Field | Type | Bits |
| --- | --- | ---: |
| dist_mm | u | 12 |
| angle_deg | u | 8 |
| width_mm | u | 10 |
| clear_l_mm | u | 12 |
| clear_r_mm | u | 12 |
| action | enum Avoid | 3 |

Payload: 12 bytes, including header.

## robot/event/obstacle_width

| Field | Type | Bits |
| --- | --- | ---: |
| result | enum Edge | 2 |
| side | enum Side | 1 |
| edge_mm | u | 10 |
| width_mm | u | 11 |
| face_mm | u | 12 |
| heard_mm | u | 11 |
| turn_deg | u | 6 |

Payload: 11 bytes, including header.

## robot/event/motion

| Field | Type | Bits |
| --- | --- | ---: |
| motion | enum Motion | 3 |

Payload: 5 bytes, including header.

## robot/event/run

| Field | Type | Bits |
| --- | --- | ---: |
| run | enum Run | 1 |

Payload: 5 bytes, including header.

## robot/summary

| Field | Type | Bits |
| --- | --- | ---: |
| run_ms | u | 20 |
| dist_mm | s | 16 |
| max_hump_dmm | u | 10 |
| humps | u | 8 |
| barcodes | u | 8 |
| obstacles | u | 8 |

Payload: 13 bytes, including header.

## robot/event/command

| Field | Type | Bits |
| --- | --- | ---: |
| cmd | enum Cmd | 3 |
| ok | u | 1 |

Payload: 5 bytes, including header.

## Enum codes and units

Codes are zero-based in this order:

| Name | Values |
| --- | --- |
| States | INIT, READY, LINE_FOLLOW, EXECUTE_NAV, OBSTACLE_STOP, OBSTACLE_SCAN, BYPASS, LINE_RECOVERY, FINISHED, SAFE_STOP |
| Nav | NONE, LEFT, RIGHT, STRAIGHT, UTURN |
| Line | LEFT, CENTRE, RIGHT, LOST, JUNCTION |
| Motion | QUIET, ACCELERATING, CLIMBING_HUMP, DESCENDING_HUMP, LIFTED, TURNING |
| Avoid | STOP, CONTINUE, TURN_LEFT, TURN_RIGHT, REVERSE_RETRY |
| Cmd | unknown, start, stop, status, map |
| CalResult | OK, CONTRAST, LINE_NOT_FOUND, BUDDY4, SENSOR_GAP |
| Range | OK, NO_ECHO, TOO_CLOSE, TOO_FAR, TIMEOUT, NOT_READY |
| Sonar | INIT, MONITOR, SCANNING, SIDE_WATCH, LINE_RECOVERY, SENSOR_FAULT |
| Edge | FOUND, AT_LEAST, NO_FACE |
| Side | LEFT, RIGHT |

Barcode code 0 is NONE, 1-43 follows Code 39 table order (0-9, A-Z,
hyphen, period, space, dollar, slash, plus, percent); 44 is INVALID.
Run code 1 is BEGIN; run end is a summary.

Speed and distance fields use mm/s and mm; encoder fields are counts.
PWM is signed permille; hump fields ending in dmm use 0.1 mm; tilt uses
milli-g and turn rate degrees/s. ADC readings use raw 12-bit counts.
Calibration overlap -256 means not measured. Width FOUND is an estimate,
AT_LEAST a lower bound and NO_FACE provides no estimate.
Fault bits are documented in [Vehicle Controller](project/VEHICLE_CONTROLLER.md).

## Delivery and commands

Telemetry and heartbeat use QoS 0. Events use QoS 1, with state and summary
retained. `robot/info`, `robot/nav_map` and `robot/status` remain text in
both builds. `robot/debug` is also text when debug publication is enabled.

`robot/command` accepts text START/STOP/STATUS commands as documented in
[COMMS_GUIDE.md](COMMS_GUIDE.md), or one-byte codes 1/2/3.
MAP is text only, for example `map A=LEFT F=RIGHT Z=FORWARD X=UTURN`.
The mapping is configurable, not a hard-coded course alphabet.

Run `make -C tests/comms test` and
`powershell -File tools/test_robot_decode.ps1` for the golden-message checks.
