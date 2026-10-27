# Binary MQTT Format (CR1)
## DRAFT — Buddy 1, 26 Oct 2026

The robot publishes the same topics and the same values as the JSON format,
but each value is packed into the **fewest bits its agreed range needs**
(CR1: minimum-bit representation).  Bit widths come from
`docs/DATA_DEFINITIONS.md`; anything not yet confirmed there is marked TBC.

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

| Field | Bits | Range / meaning | Status |
|---|---:|---|---|
| `spd_l_mm_s` | 10 s | −512 … +511 mm/s (max speed 300) | Proposed |
| `spd_r_mm_s` | 10 s | as above | Proposed |
| `enc_l` | 19 s | encoder counts since power-on | TBC (reset rule, Buddy 2) |
| `enc_r` | 19 s | as above | TBC |
| `dist_mm` | 16 s | −32 768 … +32 767 mm since power-on | TBC |

74 + 34 = 108 bits → **14 bytes** (JSON ≈ 100 B).

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
| `motion` | 3 | 0 QUIET, 1 ACCELERATING, 2 CLIMBING_HUMP, 3 DESCENDING_HUMP, 4 IMPACT | Confirmed (Buddy 4) |
| `tilt_mg` | 11 s | pitch, −1000 … +1000 milli-g (clamped) | Confirmed (Buddy 4) |
| `hump_dmm` | 10 | current hump height, 0 … 800 = 0 … 80.0 mm | Confirmed (Buddy 4) |
| `max_hump_dmm` | 10 | highest hump this run, same scale | Confirmed (Buddy 4) |

34 + 34 = 68 bits → **9 bytes** (JSON 99 B).

### robot/heartbeat  (Buddy 1, once per second)

| Field | Bits | Range / meaning | Status |
|---|---:|---|---|
| `seq` | 16 | 0 … 65 535, wraps | Proposed |
| `state` | 4 | robot state, see §4 | Proposed |
| `fault` | 9 | fault bits 0-8, see `DATA_DEFINITIONS.md` §6 | Proposed |
| `ev_drop` | 8 | events dropped, 0 … 255, stops at 255 | Proposed |

37 + 34 = 71 bits → **9 bytes** (JSON 74 B).

### robot/state  (retained)

| Field | Bits | Status |
|---|---:|---|
| `state` | 4 | Proposed |
| `fault` | 9 | Proposed |

13 + 34 = 47 bits → **6 bytes** (JSON 51 B).

### robot/event/barcode

| Field | Bits | Range / meaning | Status |
|---|---:|---|---|
| `code` | 3 | 0 NONE, 1 A, 2 B, 3 C, 4 D, 5 F, 6 Z, 7 INVALID | Proposed: covers the design (A–D) and the course slide (A, F, Z) until settled |
| `command` | 3 | 0 NONE, 1 LEFT, 2 RIGHT, 3 STRAIGHT, 4 UTURN | Code + Tribunal |

6 + 34 = 40 bits → **5 bytes** (JSON 49 B).

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
| `cmd` | 2 | 0 UNKNOWN, 1 START, 2 STOP, 3 STATUS |
| `ok` | 1 | 1 = accepted |

3 + 34 = 37 bits → **5 bytes** (JSON 40 B).

### Kept as text (both formats)

| Topic | Why |
|---|---|
| `robot/info` | Build name, sent once per connection; text is the information |
| `robot/status` | `online` / `offline`: the MQTT broker itself publishes `offline` (last will), so it must stay a fixed text |

## 3. Laptop → robot: robot/command

The robot accepts **both** forms in both builds:

| Command | Text | Binary (1 byte) |
|---|---|---|
| START | `start` | 0x01 |
| STOP | `stop` | 0x02 |
| STATUS (resend everything) | `status` | 0x03 |

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
| Motion class | 0 STATIONARY, 1 ACCELERATING, 2 TURNING, 3 CLIMBING, 4 DESCENDING, 5 IMPACT | 0 QUIET, 1 ACCELERATING, 2 CLIMBING_HUMP, 3 DESCENDING_HUMP, 4 IMPACT (Buddy 4) |
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

**Tests:** `make -C tests/comms test` (encoder, incl. locked golden bytes)
and `tools\test_robot_decode.ps1` (decoder, same golden bytes).

## 7. Expected saving (motion, the busiest topic)

JSON ≈ 100 B → binary 14 B per message (−86 %).  Measured "after" numbers go
in a new evidence file next to the baseline.
