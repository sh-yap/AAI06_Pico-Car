# Binary MQTT Format (CR1)
## DRAFT — Buddy 1, 26 Oct 2026

The robot publishes the same topics and the same values as the JSON format,
but each value is packed into the **fewest bits its agreed range needs**
(CR1: minimum-bit representation).  Bit widths come from
`docs/DATA_REPRESENTATION.md`; anything not yet confirmed there is marked TBC.

The firmware builds **binary by default**.  `COMMS_FORMAT=json` builds the
readable JSON fallback instead (only one format is compiled in).  The laptop
tools read both.

## 1. Packing rules

- Fields are written in the order listed, **least significant bit first**,
  into consecutive bytes (bit 0 of byte 0 first).  The last byte is padded
  with zeros.
- **Signed** fields are two's complement in their width.
- A value outside its field's range is **clamped** to the nearest end
  (e.g. 600 mm/s in a 10-bit signed field is sent as 511), never wrapped.
- Every binary message starts with the **header**:

| Field | Bits | Meaning |
|---|---:|---|
| `v` | 2 | Format version: **2** = binary (JSON messages say `"v":1`) |
| `t` | 32 | Robot time, ms since power-on (wraps after 49 days) |

  Header = 34 bits.  A laptop tool tells the formats apart by the first
  byte: JSON always starts with `{` (0x7B); a binary message starts with
  a byte whose low 2 bits are 2 (the version).

## 2. Messages (robot → laptop)

Bits shown are **after** the 34-bit header.  "Bytes" = whole message,
header included.  JSON size = average from the CR1 baseline
(`evidence/buddy1/20261026_cr1_baseline_json.md`).

### robot/telemetry/motion  (Buddy 2, up to 5 per second)

All 11 fields the firmware sends, in this order:

| Field | Bits | Range / meaning | Status |
|---|---:|---|---|
| `spd_l_mm_s` | 10 s | −512 … +511 mm/s (max speed 300) | Proposed |
| `spd_r_mm_s` | 10 s | as above | Proposed |
| `enc_l` | 19 s | encoder counts since power-on | TBC (reset rule, Buddy 2) |
| `enc_r` | 19 s | as above | TBC |
| `dist_mm` | 16 s | −32 768 … +32 767 mm since power-on | TBC |
| `tgt_l_mm_s` | 10 s | commanded wheel target, mm/s | Proposed |
| `tgt_r_mm_s` | 10 s | as above | Proposed |
| `slew_l_mm_s` | 10 s | target after the slew limit, mm/s | Proposed |
| `slew_r_mm_s` | 10 s | as above | Proposed |
| `pwm_l` | 11 s | applied drive, permille, −1000 … +1000 | Proposed |
| `pwm_r` | 11 s | as above | Proposed |

136 + 34 = 170 bits → **22 bytes** (JSON ≈ 100 B).

### robot/telemetry/line  (Buddy 3)

| Field | Bits | Range / meaning | Status |
|---|---:|---|---|
| `pos` | 3 | 0 LEFT, 1 CENTRE, 2 RIGHT, 3 LOST, 4 JUNCTION | Code + Tribunal |
| `err` | 8 s | −127 (left) … +127 (right) | Proposed |
| `sensors` | 2 | bit 0 centre (GP27), bit 1 right (GP26); 1 = black | Corrected by Buddy 3 (2 line sensors) |

13 + 34 = 47 bits → **6 bytes** (JSON 54 B).

### robot/telemetry/terrain  (Buddy 4)

| Field | Bits | Range / meaning | Status |
|---|---:|---|---|
| `motion` | 3 | 0 QUIET, 1 ACCELERATING, 2 CLIMBING_HUMP, 3 DESCENDING_HUMP, 4 LIFTED (Buddy 4's sharp-change cue is latched), 5 TURNING | Confirmed (Buddy 4) |
| `tilt_mg` | 11 s | pitch, −1000 … +1000 milli-g (clamped) | Confirmed (Buddy 4) |
| `hump_dmm` | 10 | current hump height, 0 … 800 = 0 … 80.0 mm | Confirmed (Buddy 4) |
| `max_hump_dmm` | 10 | highest hump this run, same scale | Confirmed (Buddy 4) |
| `turn_dps` | 11 s | yaw rate from wheel encoders, deg/s, + = left | Proposed (Buddy 4) |

45 + 34 = 79 bits → **10 bytes** (JSON ≈ 81 B).

### robot/telemetry/obstacle  (Buddy 5, up to 5 per second)

The latest forward ultrasonic reading.  Added 2026-11-20 so a sensor fault
(fault bit 5) can be traced in the laptop log.

| Field | Bits | Range / meaning | Status |
|---|---:|---|---|
| `dist_mm` | 12 | 0 … 4095 mm; 0 = no valid reading | Buddy 5 |
| `status` | 3 | 0 OK, 1 NO_ECHO (sensor did not answer), 2 TOO_CLOSE, 3 TOO_FAR, 4 TIMEOUT (nothing reflected), 5 NOT_READY | Buddy 5 |
| `state` | 3 | 0 INIT, 1 MONITOR, 2 SCANNING, 3 SIDE_WATCH, 4 LINE_RECOVERY, 5 SENSOR_FAULT | Buddy 5 |
| `silent` | 8 | NO_ECHO + NOT_READY readings since boot, wraps at 256.  A running count, so readings between two snapshots are not lost: compare consecutive values. | Buddy 5 |

26 + 34 = 60 bits → **8 bytes** (JSON ≈ 70 B).

### robot/heartbeat  (Buddy 1, once per second)

| Field | Bits | Range / meaning | Status |
|---|---:|---|---|
| `seq` | 16 | 0 … 65 535, wraps | Proposed |
| `state` | 4 | robot state, see §4 | Proposed |
| `fault` | 10 | fault bits 0-9; bit 4 is reserved.  Names and set/clear rules: `VC_FAULT_*` in `vc_task.h`, `docs/DATA_REPRESENTATION.md` §9 | Proposed |
| `ev_drop` | 8 | events dropped, 0 … 255, stops at 255 | Proposed |

38 + 34 = 72 bits → **9 bytes** (JSON 74 B).

### robot/state  (retained)

| Field | Bits | Status |
|---|---:|---|
| `state` | 4 | Proposed |
| `fault` | 10 | Proposed (bits 0-9, bit 4 reserved) |

14 + 34 = 48 bits → **6 bytes** (JSON 51 B).

### robot/event/barcode

| Field | Bits | Range / meaning | Status |
|---|---:|---|---|
| `code` | 6 | 0 NONE, 1…43 Code 39 in the decoder's table order, 44 INVALID | Any Code 39 character is reported (18 Nov): the index is the character's position in `c39_table` (`'0'`…`'9'`, `'A'`…`'Z'`, `- . space $ / + %`) + 1; 44 = decode failed, 45-63 reserved |
| `command` | 3 | 0 NONE, 1 LEFT, 2 RIGHT, 3 STRAIGHT, 4 UTURN | Code + Tribunal; a non-course character sends NONE |

6 + 3 + 34 = 43 bits → **6 bytes**.

### robot/event/hump

| Field | Bits | Range | Status |
|---|---:|---|---|
| `peak_dmm` | 10 | 0 … 800 (0.1 mm) | Confirmed (Buddy 4) |
| `max_dmm` | 10 | highest this run | Confirmed (Buddy 4) |

20 + 34 = 54 bits → **7 bytes** (JSON 44 B).

### robot/event/obstacle  (Buddy 5)

| Field | Bits | Range / meaning | Status |
|---|---:|---|---|
| `dist_mm` | 12 | 0 … 4095 mm; 0 = no valid reading | Tribunal |
| `angle_deg` | 8 | 0 … 255°; 90 = straight ahead | Proposed |
| `width_mm` | 10 | 0 … 1023 mm | TBC (widest obstacle) |
| `clear_l_mm` | 12 | 0 … 4095 mm; 0 = unknown | Proposed |
| `clear_r_mm` | 12 | as above | Proposed |
| `action` | 3 | 0 STOP, 1 CONTINUE, 2 TURN_LEFT, 3 TURN_RIGHT, 4 REVERSE_RETRY | Code (Buddy 5 + comms agree) |

57 + 34 = 91 bits → **12 bytes** (JSON 117 B).

### robot/event/obstacle_width  (Buddy 5, once per bypass)

The obstacle width estimate from the edge watch on bypass leg 1. The servo looks
square at the obstacle's face while the diagonal leg moves the car sideways. The
face stops echoing once the sensor has passed its edge on the bypass side.
`width_mm` = 2 × `edge_mm` assumes the obstacle is centred on the car's centre line
where it stopped (the bypass planner assumes the same). Sent once leg 1 ends, or
sooner if the edge is found. On the robot it shares the obstacle event's queue
slot (`CODEC_EVENT_OBSTACLE`, kind 1), so the event type list is unchanged.

| Field | Bits | Range / meaning | Status |
|---|---:|---|---|
| `result` | 2 | 0 FOUND (an estimate), 1 AT_LEAST (still echoing at the end of leg 1: a lower bound), 2 NO_FACE (no estimate) | Code (Buddy 5) |
| `side` | 1 | 0 LEFT, 1 RIGHT (bypass side) | Code |
| `edge_mm` | 10 | 0 … 1023 mm, edge from the car's centre line | Code |
| `width_mm` | 11 | 0 … 2047 mm, 2 × `edge_mm` | Code |
| `face_mm` | 12 | 0 … 4095 mm, last face reading | Code |
| `heard_mm` | 11 | 0 … 2047 mm, leg 1 distance at that reading | Code |
| `turn_deg` | 6 | 0 … 63°, the plan's first pivot | Code |

53 + 34 = 87 bits → **11 bytes**. Golden message (`tests/comms/test_codec.c`):
`52670700600c63f082432e` = t 121300, FOUND, LEFT, edge 99, width 198, face 188,
heard 270, turn 23.

### robot/event/calibration  (Vehicle Controller, once per START)

The START sweep result: the raw 12-bit ADC readings of the three line
sensors, and Buddy 4's noise floor.  `result` says why a sweep failed.

| Field | Bits | Range / meaning | Status |
|---|---:|---|---|
| `result` | 3 | 0 OK, 1 CONTRAST, 2 LINE_NOT_FOUND, 3 BUDDY4, 4 SENSOR_GAP | Proposed |
| `centre_low` | 12 | raw ADC, 0 … 4095 | Proposed |
| `centre_high` | 12 | as above | Proposed |
| `centre_thr` | 12 | threshold in use, raw ADC | Proposed |
| `centre_full` | 12 | full-black reading, raw ADC | Proposed |
| `right_low` | 12 | raw ADC | Proposed |
| `right_high` | 12 | as above | Proposed |
| `right_thr` | 12 | as above | Proposed |
| `barcode_low` | 12 | raw ADC | Proposed |
| `barcode_high` | 12 | as above | Proposed |
| `barcode_thr` | 12 | the one in use, learned or fallback | Proposed |
| `barcode_learned` | 1 | 1 = learned this START, 0 = fallback constant | Proposed |
| `first_right` | 1 | 1 = the sweep pivoted right first | Proposed |
| `overlap` | 9 s | samples both line sensors were black at once; negative = a white gap of that many samples; −256 = not measured | Proposed |
| `b4_floor_mg` | 9 | Buddy 4 noise floor, 1 … 500; 0 = Buddy 4 failed | Proposed |
| `b4_samples` | 15 | noise samples averaged, 0 … 20 000 | Proposed |
| `sweep_deg` | 8 | whole degrees turned; the limit is 90 | Proposed |

166 + 34 = 200 bits → **25 bytes**.

### robot/event/motion  (Buddy 4)

| Field | Bits | Status |
|---|---:|---|
| `motion` | 3 | as terrain `motion` |

3 + 34 = 37 bits → **5 bytes**.

### robot/event/run

| Field | Bits | Meaning |
|---|---:|---|
| `run` | 1 | 1 = BEGIN (the end of a run is `robot/summary`) |

1 + 34 = 35 bits → **5 bytes** (JSON 33 B).

### robot/summary  (retained, end of each run)

| Field | Bits | Range | Status |
|---|---:|---|---|
| `run_ms` | 20 | 0 … 1 048 575 ms (≈ 17.5 min) | Proposed; longest run TBC |
| `dist_mm` | 16 s | as motion | TBC |
| `max_hump_dmm` | 10 | 0 … 800 | Confirmed (Buddy 4) |
| `humps` | 8 | 0 … 255 | Proposed |
| `barcodes` | 8 | 0 … 255 | Proposed |
| `obstacles` | 8 | 0 … 255 | Proposed |

70 + 34 = 104 bits → **13 bytes** (JSON 110 B).

### robot/event/command  (reply to a laptop command)

| Field | Bits | Meaning |
|---|---:|---|
| `cmd` | 3 | 0 UNKNOWN, 1 START, 2 STOP, 3 STATUS, 4 MAP |
| `ok` | 1 | 1 = accepted |

4 + 34 = 38 bits → **5 bytes** (JSON 40 B).

### Kept as text (both formats)

| Topic | Why |
|---|---|
| `robot/info` | Build name, sent once per connection; text is the information |
| `robot/nav_map` | Barcode mapping in force, e.g. `A=LEFT F=RIGHT Z=FORWARD X=UTURN`, or `none`; retained, sent on each connection and after every `map` command |
| `robot/status` | `online` / `offline`: the MQTT broker itself publishes `offline` (last will), so it must stay a fixed text |

## 3. Laptop → robot: robot/command

The robot accepts **both** forms in both builds:

| Command | Text | Binary (1 byte) |
|---|---|---|
| START | `start` | 0x01 |
| STOP | `stop` | 0x02 |
| STATUS (resend everything) | `status` | 0x03 |
| MAP (barcode character → movement) | `map A=LEFT F=RIGHT Z=FORWARD X=UTURN` | - (text only) |

`map` sets which four Code 39 characters mean LEFT, RIGHT, FORWARD and
U-TURN at the next cross-junction; the format and rules are in
`app_program/nav_map.h`.  Nothing is hard-coded: until a mapping arrives,
barcodes are still decoded and reported but none makes the car turn.  The
reply is `cmd` MAP with `ok` 0 if the mapping was rejected (the previous
one stays), and `robot/nav_map` always shows the one in force.  Publish it
**retained** so a car that reboots gets it again.  The payload must fit in
64 bytes.

## 4. Shared codes

**Robot state** (4 bits): 0 INIT, 1 READY, 2 LINE_FOLLOW, 3 EXECUTE_NAV,
4 OBSTACLE_STOP, 5 OBSTACLE_SCAN, 6 BYPASS, 7 LINE_RECOVERY, 8 FINISHED,
9 SAFE_STOP.  (To be replaced by the Vehicle Controller's states when the
VC branch is merged; still 4 bits.)

## 5. Changes to the comms API (`comms.h`) this needs

Fixes found while checking each owner's own numbering:

| Item | Today in `comms.h` | Becomes (owner's code) |
|---|---|---|
| Line position | 0 LOST, 1 LEFT, 2 CENTRE, 3 RIGHT, 4 JUNCTION | 0 LEFT, 1 CENTRE, 2 RIGHT, 3 LOST, 4 JUNCTION (Buddy 3) |
| Motion class | 0 STATIONARY, 1 ACCELERATING, 2 TURNING, 3 CLIMBING, 4 DESCENDING, 5 IMPACT | 0 QUIET, 1 ACCELERATING, 2 CLIMBING_HUMP, 3 DESCENDING_HUMP, 4 LIFTED, 5 TURNING (Buddy 4) |
| Terrain | `pitch_deg`, `roll_deg`, `hump_height_mm` | `tilt_mg` (pitch), `hump_dmm` (Buddy 4's real data) |
| Hump event | `comms_event_hump(peak_height_mm)` | `comms_event_hump(peak_dmm)` |
| Line error | "owner's scale" | −127 … +127 |

Only `comms_demo.c` calls these today, so no teammate's code breaks.

## 6. How to use it

**Build** (from `build_make`; binary is the default, add `COMMS_FORMAT=json` for the JSON fallback):

```
make APP=demo SMP=1 BOARD=pico CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1 COMMS_DEMO=1 PICO_SDK_PATH=<sdk> -j8
```

JSON images are named `..._json_...uf2`; binary images have no suffix.

**Laptop:** `tools\mqtt_log.ps1` decodes both formats into the same JSON
lines and also saves the raw bytes in `<log>.hex`; `tools\mqtt_check.ps1`
reports sizes from the `.hex` file (bytes actually sent).  Commands still
work as text: `mosquitto_pub -t robot/command -m start`.

**Code:** encoder `app_program/comms_codec.c` (one description per
message in `comms.c`, written as either format); decoder
`tools/robot_decode.ps1` (field table must match `comms.c`).

**Tests:** `make -C tests/comms test` (encoder, incl. locked golden bytes,
and the packed-record round trips) and `tools\test_robot_decode.ps1`
(decoder, same golden bytes).

## 7. Expected saving (motion, the busiest topic)

JSON ≈ 100 B → binary 22 B per message (−78 %).  Measured "after" numbers go
in a new evidence file next to the baseline.
