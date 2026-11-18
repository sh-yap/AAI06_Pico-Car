# Data Representation (CR1) - minimum-bit design

Week 13 deliverable 1 for CR1 ("every application data field uses only the
number of bits its valid range needs") *and* the design that the later code
phases implement.  Branch `cr1-min-bit`, from `main` 410a8b6.  Documentation
only: no source file is changed by this document.

## 0. Rules, and how to read the tables

**Packing rule.**  Shared, queued, ring-buffered or transmitted data is stored
in fixed-width unsigned words (`uint32_t`, `uint16_t`, `uint8_t`) with explicit
masks and shifts.  No C bit-fields: their order and padding are
implementation-defined (BARR-C:2018 5.5.b, CERT EXP11-C).  Bit 0 is the LSB.
A field never straddles a word boundary.

**Signed rule.**  A signed field is two's complement in its own width and is
sign-extended on unpack (`(int32_t)(field << (32 - n)) >> (32 - n)`).

**Calculation rule.**  PID, filters, estimators, integrators and timing
arithmetic stay in full-width locals.  Only state that is stored, shared,
queued, ring-buffered or sent is packed.

**Race rule.**  Each shared packed word has exactly one writer.  It is written
with one aligned 32-bit store (single-copy atomic on ARMv6-M); the reader
copies the word once and unpacks.  Multi-writer data (the fault bits) is
serialised: one owner task or a uT-Kernel mutex / event flag.  Values larger
than one word use the existing seqlock (`comms.c` slots, `obstacle_task.c`
counter pairs) or are split so that each word is single-writer.

**Out-of-range rule.**  Measured values saturate (clamp) at their field ends;
counters and sequence numbers wrap; an unknown enum code is rejected as "no
data", never guessed.

**Column legend.**

| Column | Meaning |
|---|---|
| Field | member path inside the struct, or the symbol name for a global |
| Where | `file:type.member` or `file:symbol` |
| Valid range and unit | the range from the code (constant named), the datasheet, a measurement, or `TBC` with the question.  `[code]` = derived from code constants, `[ds]` = datasheet, `[meas]` = measured, `[TBC]` = not established |
| Bits | bits the valid range needs: `U` unsiged, `s` signed two's complement |
| Current | current storage: C type, bytes (whole struct for the struct's first row) |
| Proposed | packed word/bit positions, or `keep: reason` |
| Shared? | writer -> readers, ISR/task/core |

Words are numbered per packed record: `W0[0:1]` is word 0, bits 0-1.  `N words`
in the Proposed column means `N * 4` bytes.  "Static bytes saved" ignores
stack (task stacks are excluded, section 12).

Sizes quoted as "current" were checked with `arm-none-eabi-gcc 15.2.1`,
`-mcpu=cortex-m0plus -O2`, this tree's headers (this toolchain's default
`enum` width makes `buddy3_status_t` 7 B and `obstacle_status_t` 12 B, so the
probe and the team's measured numbers agree).

---

## 1. Motion control and encoder data

Source of ranges: `motion_config.c` (max_speed_mm_s 300, max_pwm_permille 900,
max_track 119 mm, 2536/2529 counts per wheel revolution, 181 mm wheel
circumference = 14.0 counts/mm), `motion_task.c` (MOTION_MAX_TURN_ANGLE_MDEG
360000), `vc_task.c` (turn speeds 80, legs 150, reverse 100 mm at 80 mm/s).

### 1.1 `motion_command_t` - queue message, 4 slots (20 B each)

| Field | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| `type` | motion_types.h:motion_command_t.type | 0..2 `[code]` `motion_command_type_t` | 2 U | enum, 4 B | `W0[0:1]` | VC task -> Motion task (mbf) |
| `sequence_id` | .sequence_id | 1..65535; the VC producer wraps at 16 bits and skips 0 (`vc_task.c:next_seq()`), so Motion echoes the same value; 0 = invalid `[code]` | 16 U | uint32, 4 B | `W0[2:17]`; producer wraps at 16 bits | same |
| `timeout_ms` | .timeout_ms | 0 (none) .. 31 586 ms = measured longest bypass leg `[meas]`: `(2294*2000)/150 + 1000`; other VC values 0, 3000, 8000 ms; VC state bounds 25 000 ms | 14 U in 10 ms units = 0..163 830 ms | uint32, 4 B | `W0[18:31]`, unit 10 ms, round up | same |
| `value.velocity.linear_mm_s` | .value.velocity.linear_mm_s | +-300 mm/s `[code]` max_speed_mm_s | 10 s | int32, 4 B | `W1[0:9]` | same |
| `value.velocity.angular_mdeg_s` | .value.velocity.angular_mdeg_s | +-288 800 mdeg/s = 2*300/119 mm `[code]`, 20 s (+-524 288) | 20 s | int32, 4 B | `W1[10:29]` | same |
| `value.distance.distance_mm` | .value.distance.distance_mm | +-4096 mm; the bypass planner reaches 2294 mm out at a 2225 mm obstacle `[meas]`, so 12 s (+-2047) is not enough | 13 s | int32, 4 B | `W1[0:12]` | same |
| `value.distance.speed_mm_s` | .value.distance.speed_mm_s | +-300 mm/s `[code]` | 10 s | int32, 4 B | `W1[13:22]` | same |
| `value.turn.angle_mdeg` | .value.turn.angle_mdeg | +-360 000 mdeg `[code]` MOTION_MAX_TURN_ANGLE_MDEG | 20 s | int32, 4 B | `W1[0:19]` | same |
| `value.turn.wheel_speed_mm_s` | .value.turn.wheel_speed_mm_s | +-300 mm/s `[code]`; VC uses 80 | 10 s | int32, 4 B | `W1[20:29]` | same |

Whole message: 20 B -> `motion_command_packed_t` = **2 words, 8 B**.  Bits
30-31 of W1 are spare.  The union is type-tagged, so three payloads share one
word.  As built in Phase 3 (`motion_pack.c`): packed in `motion_submit()` and
in VC `queue_manoeuvre()`, unpacked on receive.  The distance payload grew
from the first 12-bit estimate to 13 bits after measuring the planner's
largest leg; bits 23-31 of W1 remain spare.

### 1.2 `motion_status_t` - shared snapshot under `status_mtx` (160 B -> 104 B)

Split into a live record (10 words) plus a diagnostic record (16 words).  The
Motion task keeps a full-width working copy for one cycle and stores the
packed record under `status_mtx`; `motion_get_status()` unpacks it.  The table
below is the Phase 3 as-built layout.  It differs from the first proposal in
five places, each forced by a real reader:

* `left_encoder_counts` and `right_encoder_counts` are read by the VC and were
  missing from the first table, so they add live words W8 and W9.  They are
  boot-relative counters that are never reset, so each keeps its own full
  32-bit word: 24 s clamps at +-8 388 607 counts, only +-599 m of travel
  (`motion_task.c:462-463`, `vc_task.c:359-363`).
* `travelled_mm` stays 24 s: relative to the command origin for
  MOVE_DISTANCE/TURN_ANGLE (`motion_task.c:329-345`, a 13-bit distance or a
  turn of at most 360 deg = ~374 mm wheel travel), and boot-relative only for
  SET_VELOCITY (`motion_task.c:340-345`), where 24 s covers +-8.4 km, far
  beyond any course run.  `target_mm` is set per command
  (`motion_task.c:236,250-253`) from a 13-bit distance or the turn wheel
  travel; `target_angle_mdeg` is the 20-bit command angle
  (`motion_task.c:238,257-258`).  All three stay inside their widths.
* Measured wheel speed can exceed the 300 mm/s command limit while a sample is
  being validated (the regression replay reaches 544 mm/s; the physical bound
  is 1000 mm/s), so the two measured speeds use 12 signed bits, not 10.
* `encoder_snapshot_duration_us` can exceed 65 535 us when acquisition is
  preempted (the task regression uses 100 000 us), so it stays 32 bits.
* `timestamp_us` and `state_timestamp_us` still have readers that compare an
  absolute stored value with a 64-bit start time (`motion_stage6_test_main.c`),
  so both stay 64 bits.  `encoder_sample_time_us` and
  `encoder_fault_previous_time_us` are only printed after a `uint32_t` cast
  (`motion_closed_loop_button_test_main.c:62-63`) and are narrowed.
* `left_invalid_encoder_transitions` is documented as 0..65 535 but the first
  table gave it 12 bits; it uses 16 bits here.

The diagnostic record is kept in all builds in this phase.

| Field | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| `active_sequence_id` | motion_types.h:motion_status_t.active_sequence_id | 1..65535, wraps `[code]` | 16 U | uint32, 4 B | `L W0[0:15]` | Motion task -> VC task, radio |
| `state` | .state | 0..5 `[code]` `motion_state_t` | 3 U | enum, 4 B | `L W0[16:18]` | same |
| `fault` | .fault | 0..5 `[code]` `motion_fault_t` | 3 U | enum, 4 B | `L W0[19:21]` | same |
| `encoder_fault` | .encoder_fault | 0..3 `[code]` | 2 U | enum, 4 B | `L W0[22:23]` | same |
| `encoder_fault_wheel_mask` | .encoder_fault_wheel_mask | 0..3, bits 0 left, 1 right `[code]` | 2 U | uint8, 1 B | `L W0[24:25]` | same |
| `valid` | .valid | 0/1 | 1 U | bool, 1 B | `L W0[26]` | same |
| `left_speed_mm_s` | .left_speed_mm_s | +-2048 mm/s; measured, physical bound 1000 `[code]` | 12 s | int32, 4 B | `L W1[0:11]` | same |
| `right_speed_mm_s` | .right_speed_mm_s | +-2048 mm/s; physical bound 1000 | 12 s | int32, 4 B | `L W1[12:23]` | same |
| `left_target_mm_s` | .left_target_mm_s | +-300 mm/s | 10 s | int32, 4 B | `L W2[0:9]` | same |
| `right_target_mm_s` | .right_target_mm_s | +-300 mm/s | 10 s | int32, 4 B | `L W2[10:19]` | same |
| `left_slewed_target_mm_s` | .left_slewed_target_mm_s | +-300 mm/s | 10 s | int32, 4 B | `L W2[20:29]` | same |
| `right_slewed_target_mm_s` | .right_slewed_target_mm_s | +-300 mm/s | 10 s | int32, 4 B | `L W3[0:9]` | same |
| `left_drive_permille` | .left_drive_permille | +-1000 `[code]` (max_pwm_permille 900) | 11 s | int32, 4 B | `L W3[10:20]` | same |
| `right_drive_permille` | .right_drive_permille | +-1000 | 11 s | int32, 4 B | `L W3[21:31]` | same |
| `left_pwm_permille` | .left_pwm_permille | 0..1000 `[code]` | 10 U | uint16, 2 B | `L W4[0:9]` | same |
| `right_pwm_permille` | .right_pwm_permille | 0..1000 | 10 U | uint16, 2 B | `L W4[10:19]` | same |
| `target_speed_mm_s` | .target_speed_mm_s | +-300 mm/s | 10 s | int32, 4 B | `L W4[20:29]` | same |
| `travelled_mm` | .travelled_mm | TBC (longest course leg); 24 s covers +-8388 m | 24 s | int32, 4 B | `L W5[0:23]` | same |
| `target_mm` | .target_mm | TBC; same as distance command +-2047 mm, 12 s enough today | 24 s (kept aligned with travelled) | int32, 4 B | `L W6[0:23]` | same |
| `target_angle_mdeg` | .target_angle_mdeg | +-360 000 mdeg `[code]` | 20 s | int32, 4 B | `L W7[0:19]` | same |
| `left_encoder_counts` | .left_encoder_counts | counts since boot, never reset (open Q1); full width, a 24-bit field would clamp at +-599 m | 32 s | int32, 4 B | `L W8[0:31]` | Motion -> VC, radio |
| `right_encoder_counts` | .right_encoder_counts | as above | 32 s | int32, 4 B | `L W9[0:31]` | same |
| `encoder_fault_elapsed_ms` | .encoder_fault_elapsed_ms | 0..65 535 ms `[code]` MOTION_STALL_MS 1000 | 16 U | uint32, 4 B | `D W0[0:15]` | Motion -> recorder/console only |
| `encoder_fault_left_delta` | .encoder_fault_left_delta | +-32767 counts per cycle; 14 counts/mm at 1000 mm/s = 280/20 ms `[code]` | 16 s | int32, 4 B | `D W0[16:31]` | same |
| `encoder_fault_right_delta` | .encoder_fault_right_delta | +-32767 counts per cycle | 16 s | int32, 4 B | `D W1[0:15]` | same |
| `encoder_snapshot_duration_us` | .encoder_snapshot_duration_us | 0..4 294 967 295 us; one cycle is 20 ms, but preemption can delay acquisition (regression: 100 ms) | 32 U | uint32, 4 B | `D W2[0:31]` | same |
| `encoder_fault_previous_left` | .encoder_fault_previous_left | counts since boot, TBC (see open Q1) | 24 s | int32, 4 B | `D W3[0:23]` | same |
| `encoder_fault_previous_right` | .encoder_fault_previous_right | as above | 24 s | int32, 4 B | `D W4[0:23]` | same |
| `encoder_fault_previous_irq` | .encoder_fault_previous_irq | 4 IRQ per count = 56 IRQ/mm, wraps `[code]` | 24 U | uint32, 4 B | `D W5[0:23]` | same |
| `encoder_fault_previous_time_us` | .encoder_fault_previous_time_us | us since boot; only printed as low 32 bits | 32 U | uint64, 8 B | `D W6[0:31]`, wraps | same |
| `encoder_sample_time_us` | .encoder_sample_time_us | as above; only printed as low 32 bits | 32 U | uint64, 8 B | `D W7[0:31]`, wraps | same |
| `timestamp_us` | .timestamp_us | us since boot; test readers compare an absolute start time | 64 U | uint64, 8 B | `D W8[0:31]` + `D W9[0:31]` | same |
| `state_timestamp_us` | .state_timestamp_us | as above; test readers keep it as a 64-bit start time | 64 U | uint64, 8 B | `D W10[0:31]` + `D W11[0:31]` | same |
| `encoder_fault_previous_left_pwm` | .encoder_fault_previous_left_pwm | 0..1000 `[code]` | 10 U | uint16, 2 B | `D W12[0:9]` | same |
| `encoder_fault_previous_right_pwm` | .encoder_fault_previous_right_pwm | 0..1000 | 10 U | uint16, 2 B | `D W12[10:19]` | same |
| `left_invalid_encoder_transitions` | .left_invalid_encoder_transitions | 0..65535, wraps (diagnostic) | 16 U | uint32, 4 B | `D W13[0:15]` | same |
| `right_invalid_encoder_transitions` | .right_invalid_encoder_transitions | 0..65535, wraps | 16 U | uint32, 4 B | `D W13[16:31]` | same |
| `invalid_encoder_transitions` | .invalid_encoder_transitions | 0..65535, wraps | 16 U | uint32, 4 B | `D W14[0:15]` | same |
| `encoder_irq_count` | .encoder_irq_count | 56 IRQ/mm, wraps `[code]` | 24 U | uint32, 4 B | `D W15[0:23]` | same |

Whole record: 160 B -> live **10 words (40 B)** + diagnostic **16 words
(64 B)** = **104 B**, saving **56 B**; the task also keeps a 4 B pointer to
its full-width working copy, so the net static saving is **52 B**.  A
non-recorder build can drop the diagnostic record (`VC_RECORDER == 0`): 40 B,
saving 120 B.

### 1.3 `motion_encoder_snapshot_t` - snapshot copied by the Motion task (32 B, kept)

As built in Phase 3 this stays at 32 B.  Every use is a local or stack value
inside the Motion task (or another caller that immediately consumes it):
`motion_encoder_get_snapshot()` fills the caller's copy from the ISR counters;
there is no stored global snapshot to pack.  Section 11 row 17 therefore saves
no static RAM in this phase.

| Field | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| `left_counts` | motion_encoder.h:motion_encoder_snapshot_t.left_counts | counts since boot, TBC (open Q1); 24 s = +-599 m | 24 s | int32, 4 B | `W0[0:23]` | Motion task local (from 32-bit ISR counters) |
| `right_counts` | .right_counts | as above | 24 s | int32, 4 B | `W1[0:23]` | same |
| `left_invalid_transitions` | .left_invalid_transitions | 0..65535, wraps | 16 U | uint32, 4 B | `W2[0:15]` | same |
| `right_invalid_transitions` | .right_invalid_transitions | 0..65535, wraps | 16 U | uint32, 4 B | `W2[16:31]` | same |
| `invalid_transitions` | .invalid_transitions | = left + right, do not store | 0 | uint32, 4 B | derived on unpack | same |
| `irq_count` | .irq_count | 56 IRQ/mm, wraps | 24 U | uint32, 4 B | `W3[0:23]` | ISR -> Motion task |
| `sampled_at_us` | .sampled_at_us | us since boot, 32-bit wrap | 32 U | uint64, 8 B | `W4[0:31]` | same |

32 B -> **5 words, 20 B**.

---

## 2. Line and barcode sensing

Sources: `line_follow.h` (LINE_ERROR_MAX 127, 2 sensor bits), `buddy3_status.h`
(already a packed 29-bit word), `barcode.h`/`barcode.c` (64 edges, 480 raw
samples, 128 histogram bins, 63 elements), `obstacle_config.h` (servo 60..120).

### 2.1 `buddy3_status_t` and the Buddy 3 words

| Field | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| whole status | buddy3_status.h:buddy3_packed_t | 29 bits used, 3 reserved `[code]` | 29 | uint32 word, 4 B | keep: already minimum-bit, one writer, one aligned store | Buddy 3 task -> VC (mbf), radio |
| `line_state` | buddy3_status.h:buddy3_status_t.line_state | 0..4 `[code]` LINE_* | 3 U | same word `B3_LINE_STATE` | keep (`W[0:2]`) | decode view only |
| `line_error` | .line_error | -127..+127 `[code]` LINE_ERROR_MAX | 8 s | `B3_LINE_ERROR` | keep (`W[3:10]`) | decode view |
| `line_sensors` | .line_sensors | 0..3, 2 sensors `[code]` | 2 U | `B3_LINE_SENSORS` | keep (`W[11:12]`) | decode view |
| `barcode` | .barcode | 0..6 `[code]` `barcode_t` | 3 U | `B3_BARCODE` | keep (`W[13:15]`) | decode view |
| `nav_command` | .nav_command | 0..4 `[code]` `nav_command_t` | 3 U | `B3_NAV` | keep (`W[16:18]`) | decode view |
| `cal_state` | .cal_state | 0..5 `[code]` `line_cal_state_t` | 3 U | `B3_CAL` | keep (`W[19:21]`) | decode view |
| `b_barcode_capture` | .b_barcode_capture | 0/1 | 1 U | `B3_CAPTURE` | keep (`W[22]`) | decode view |
| `barcode_char` | .barcode_char | 0 none, 1..43 Code 39 in `c39_table` order, 44 failed `[code]` | 6 U | `B3_CHAR` | new (`W[23:28]`), reserved `W[29:31]` | decode view |
| `buddy3_reading_word` | buddy3_task.c:buddy3_reading_word | centre ADC 12, right ADC 12 `[code]` | 24 | uint32 volatile, 4 B | keep: already 12+12, one writer | Buddy 3 task -> VC task |
| `buddy3_cal_word` | buddy3_task.c:buddy3_cal_word | request 3 bits + 29-bit counter `[code]` | 32 | uint32 volatile, 4 B | keep | VC task -> Buddy 3 task |
| `ir_readings_t` | ir_sensor.h:ir_readings_t | 3 x 12-bit ADC 0..4095 `[ds/code]` | 36 | struct, 6 B | keep: transient by-value, never stored | driver -> Buddy 3 task (local) |

### 2.2 Line calibration result and history

As built in Phase 4: `cal_history[]` is packed to three bytes per sample
(768 B, 256 B saved).  `line_cal_result_t` stays 24 B: its fields need 108
(9 x 12) + 12 (`centre_full_black`) + 1 (`gap_measured`) + 9 (`gap_overlap`)
= **130 bits**, which does not fit five 32-bit words (128 bits) without
dropping a field, and the natural struct is already 24 B with only two bytes
of padding, so no alignment win exists.  Row 21 of section 11 is therefore
"no win" as built.

| Field | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| `centre/low/high/threshold`, `right/*`, `barcode/*` | line_follow.h:line_cal_result_t (3 x line_cal_sensor_t) | each 0..4095, 12-bit ADC `[code]` | 9 x 12 = 108 | 3 x 6 B = 18 B | keep (see above): 5 words do not hold 130 bits | Buddy 3 task -> VC (calibration event) |
| `centre_full_black` | .centre_full_black | 0..4095 `[code]` | 12 U | uint16, 2 B | keep | same |
| `gap_measured` | .gap_measured | 0/1 | 1 U | bool, 1 B | keep | same |
| `gap_overlap` | .gap_overlap | -255..+255 `[code]` LINE_CAL_OVERLAP_LIMIT | 9 s | int16, 2 B | keep | same |
| whole result | line_follow.h:line_cal_result_t | as above | 130 | struct, 24 B | keep: 24 B, no win after alignment | same |
| `cal_history[]` | line_follow.c:cal_history | 256 x cal_sample_t{centre, right}, each 0..4095 `[code]` | 24 per sample | 256 x 4 B = 1024 B | as built: 256 x 3 B (24-bit little-endian byte triplet, `line_cal_sample_pack()`) = 768 B | Buddy 3 task local (ring) |
| `ref_bucket[]` | line_follow.c:ref_bucket | 6 x {centre_max, right_max} 0..4095 | 24 per bucket | 6 x 4 B = 24 B | keep: 24 B total, no win | Buddy 3 task local |
| `cal_track_t` x3 | line_follow.c:cal_centre/right/barcode | filtered 0..4095, low/high 12-bit ADC | 12 + 12 + 12 + 8 | 3 x 12 B = 36 B | keep: sweep scratch, task-local | Buddy 3 task local |

### 2.3 Barcode capture and decode buffers

As built in Phase 4 (`barcode.c`/`barcode.h`): the packed forms are the
storage only; `barcode_edge_t` is still the value type of
`barcode_get_edge()`, so `buddy3_task.c` and the recorder are unchanged.
Every in-range value reads back exactly as written (round-trip tests in
`tests/buddy3`), and out-of-range values saturate.

| Field | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| `barcode_edges[]` | barcode.c:barcode_edges | 64 edges `[code]` BARCODE_MAX_EDGES | 64 x 2 words | 64 x 12 B = 768 B | as built: 64 x 2 words = 512 B (`barcode_edge_word()`) | Buddy 3 task local |
| `.is_black` | barcode.h:barcode_edge_t.is_black | 0/1 | 1 U | bool, 1 B | `W0[0]` | same |
| `.timestamp_ms` | .timestamp_ms | ms since boot, wraps 49 days `[code]` | 32 U | uint32, 4 B | `W1[0:31]` | same |
| `.encoder_position` | .encoder_position | counts since boot, TBC (open Q1) | 31 s | int32, 4 B | `W0[1:31]` (bit 0 = is_black), clamped to -1073741824..1073741823 | same |
| `raw_adc[]` | barcode.c:raw_adc | 480 samples x 0..4095 `[code]` | 12 per sample | 480 x 2 B = 960 B | as built: 480 x 12 bits, two per three bytes = 720 B (`raw_adc_get()`/`raw_adc_set()`) | Buddy 3 task local |
| `raw_encoder[]` | barcode.c:raw_encoder | 480 x counts since boot, TBC | 16 per sample | 480 x 4 B = 1920 B | as built: 480 x int16 offset from `raw_encoder_base` (the capture's first sample) = 960 B (`raw_encoder_get()`/`raw_encoder_set()`); an offset leaving int16 clamps and sets `raw_overflow` | Buddy 3 task local |
| `raw_encoder_base` | barcode.c:raw_encoder_base | counts since boot, TBC | 32 | - | 4 B, set at the capture's first sample; the raw buffer is appended, never overwritten, so it stays valid for every stored sample | same |
| `raw_time_ms[]` | barcode.c:raw_time_ms | 480 x ms since capture start; 16 bits = 65.5 s, capture ~2.5 s `[TBC]` | 16 per sample | 480 x 2 B = 960 B | keep: already 16 bits (12 would fit if the capture bound is proven) | Buddy 3 task local |
| `raw_hist[]` | barcode.c:raw_hist | 128 bins x 0..480 samples `[code]` | 9 per bin | 128 x 2 B = 256 B | keep: 9 bits does not fit a byte | Buddy 3 task local |
| `raw_widths[]` | barcode.c:raw_widths | 63 elements x width in counts; widest element <= guard 2460 `[code]` | 12 per element | 63 x 4 B = 252 B | as built: 63 x uint16 = 126 B (`barcode_width_counts()`, saturating; the sums stay 32-bit) | Buddy 3 task local |
| `variant_widths[]` | barcode.c:variant_widths | 63 x width in counts, same range | 12 per element | 63 x 4 B = 252 B | as built: 63 x uint16 = 126 B | Buddy 3 task local |
| `glitch_choice[]` | barcode.c:glitch_choice | 63 x 0..3 `[code]` GLITCH_* | 2 per element | 63 B | keep: 63 B total, byte array already | Buddy 3 task local |
| decode scalars | barcode.c:edge_count, decoded_code, guard_*, raw_count, raw_levels | 6..255, 3-bit codes, 0..4095 levels | 1..16 | 1..4 B each | keep: scalars total < 40 B | Buddy 3 task local |

---

## 3. Obstacle detection

Sources: `obstacle_config.h` (HC-SR04 20..4000 mm `[ds]`, echo 117..23 324 us
`[ds]`, servo 60..120 deg, scan step 15 deg, 13 scan points, 3 path points,
plan max deg 40), `obstacle_calc.h` (status word 32 bits, profile word 64
bits, both already packed).

### 3.1 One ping, status and profile

| Field | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| `echo_us` | obstacle_types.h:obstacle_range_t.echo_us | 117..23 324 us `[ds]` OBSTACLE_ECHO_MAX_US | 15 U | uint32, 4 B | `W0[0:14]` | driver -> obstacle task (by value) |
| `distance_mm` | .distance_mm | 0..4000 mm `[ds]`; 0 = invalid | 12 U (clamp 4095) | uint32, 4 B | `W0[15:26]` | same |
| `timestamp_us` | .timestamp_us | us since boot, 32-bit wrap | 32 U | uint32, 4 B | `W1[0:31]` | same |
| `status` | .status | 0..5 `[code]` `obstacle_range_status_t` | 3 U | enum, 4 B | `W0[27:29]` | same |
| whole ping | obstacle_types.h:obstacle_range_t | as above | 62 | struct, 16 B | keep: 16 B, only ever a by-value result in the obstacle task and the driver (no stored or shared copy), so nothing is packed here | same |
| `obstacle_status_t` | obstacle_types.h:obstacle_status_t | decode view of the 32-bit word | 32 | struct, 12 B | keep: decode view; the stored form is the packed word | obstacle task -> VC, radio |
| status word | obstacle_calc.c / obstacle_task.c:status_word | distance 12, status 3, state 3, seq 8, 6 reserved `[code]` | 26 | uint32 volatile, 4 B | keep: already minimum-bit; 6 reserve bits stay 0 | obstacle task -> any reader |
| `obstacle_profile_t` | obstacle_types.h:obstacle_profile_t | decode view of the 64-bit profile word | 57 | struct, 24 B | keep: decode view | obstacle task -> VC, radio |
| profile word | obstacle_calc.c:obstacle_calc_pack_profile | dist 12, angle 8, width 10, clear 12+12, action 3, 7 reserved `[code]` | 57 | uint64, 8 B | keep: already minimum-bit; 7 reserve bits | same |

### 3.2 Scan result (300 B -> 68 B)

As built (Phase 5a, `obstacle_pack.c`): the task's full-width
`obstacle_scan_result_t` is a stack working copy; the stored copy is the
packed record below.  Angles are stored **absolute** (7 bits, 60..120), not
as `angle - 60`: the offset saves one bit per point but open question Q15
(offset conventions elsewhere) is not settled, and 7 spare bits per word
make the saving worthless.

| Field | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| `obstacle_scan_point_t` | obstacle_types.h:obstacle_scan_point_t | decode view of one packed point | 29 | struct, 20 B | keep: decode view | obstacle task -> VC |
| `.angle_deg` | .angle_deg | 60..120 `[code]` OBSTACLE_SERVO_ANGLE_*, step 15 | 7 U, absolute | uint32, 4 B | point word `Wn[0:6]` | same |
| `.distance_mm` | .distance_mm | 0..4000 mm, 0 = not measured | 12 U (clamp 4095) | uint32, 4 B | `Wn[7:18]` | same |
| `.valid_count` | .valid_count | 0..8 `[code]`: OBSTACLE_SCAN_PINGS 3, but SCAN_PINGS_MAX 8 in obstacle_scan.c | 4 U (was 2) | uint32, 4 B | `Wn[19:22]` | same |
| `.silent_count` | .silent_count | 0..8, same rule | 4 U (was 2) | uint32, 4 B | `Wn[23:26]` | same |
| `.point_class` | .point_class | 0..2 `[code]` | 2 U | enum, 4 B | `Wn[27:28]` | same |
| `points[]` | obstacle_types.h:obstacle_scan_result_t.points[13] | 13 points `[code]` OBSTACLE_SCAN_MAX_POINTS | 29 each | 13 x 20 B = 260 B | 13 x 1 word = 52 B | obstacle task (seqlock) -> VC |
| `.count` | .count | 0..13 | 4 U | uint32, 4 B | `W13[0:3]` | same |
| `.profile` | .profile | 64-bit packed profile word | 64 | 24 B | `W15` low, `W16` high | same |
| `.right_edge_deg` | .right_edge_deg | 0..120, 0 = none | 7 U | uint32, 4 B | `W13[4:10]` | same |
| `.left_edge_deg` | .left_edge_deg | 0..120, 0 = none | 7 U | uint32, 4 B | `W13[11:17]` | same |
| `.duration_ms` | .duration_ms | TBC (scan ~2 s: 13 x settle 160..400 ms + pings); 16 bits = 65.5 s | 16 U | uint32, 4 B | `W14[0:15]` | same |
| whole result | obstacle_types.h:obstacle_scan_result_t | as above | 29*13 + 4 + 64 + 7 + 7 + 16 = 475 | struct, 300 B | **17 words, 68 B, as built** | same |

### 3.3 Path check (52 B -> 12 B) and monitor

As built (Phase 5a): 95 field bits do not fit three words when each point
keeps its three fields together (a 22-bit point cannot share a word with
another point), so `points[0].verdict` sits in W0 and the other point fields
are split between W1 and W2.  Every field still lies inside one word.

| Field | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| `verdict` | obstacle_types.h:obstacle_path_t.verdict | 0..4 `[code]` | 3 U | enum, 4 B | `W0[0:2]` | obstacle task (seqlock) -> VC |
| `need_mm` | .need_mm | plan check_mm = out_mm + 100, out_mm <= ~311 mm `[code]`; 12 bits = 4095 mm | 12 U | uint32, 4 B | `W0[3:14]` | VC -> obstacle task (volatile write) |
| `known_mm` | .known_mm | 0..4000 mm, 0 = none | 12 U | uint32, 4 B | `W0[15:26]` | VC -> obstacle task |
| `count` | .count | 0..3 `[code]` OBSTACLE_PATH_MAX_POINTS | 2 U | uint32, 4 B | `W0[27:28]` | obstacle task -> VC |
| `points[].angle_deg` | .points[3].angle_deg | 70..115 `[code]`; 7 bits absolute | 7 U | uint32, 4 B | `W1[0:6]`, `W2[12:18]`, `W2[19:25]` | obstacle task -> VC |
| `points[].distance_mm` | .points[].distance_mm | 0..4000 mm, 0 = not measured | 12 U | uint32, 4 B | `W1[7:18]`, `W1[19:30]`, `W2[0:11]` | same |
| `points[].verdict` | .points[].verdict | 0..4 `[code]` | 3 U | enum, 4 B | `W0[29:31]`, `W2[26:28]`, `W2[29:31]` | same |
| whole path | obstacle_types.h:obstacle_path_t | as above | 3+12+12+2+3*22 = 95 | struct, 52 B | **3 words, 12 B, as built** | same |
| `obstacle_monitor_t` | obstacle_monitor.h:obstacle_monitor_t | 3 flags, close/lost counts 0..255, near_mm 0..4000, silent/alive 0..255 | 47 | struct, 24 B | keep: task-private decision state, never shared, never queued | obstacle task only |
| `obstacle_bypass_plan_t` | obstacle_types.h:obstacle_bypass_plan_t | side 0..4, turn/back_deg 0..60 `[code]`, out/side/along/back/check 0..4095 | 77 | struct, 32 B | keep: VC-task private; pack only if it is ever sent (see 11.4) | VC task only |
| `path_need_mm`, `path_known_mm`, `path_side` | obstacle_task.c | 0..4095, 0..4000, 0..4 | 12+12+3 | 3 x 4 B = 12 B | one word, as built: need `[0:11]`, known `[12:23]`, side `[24:26]`, 5 bits spare.  One writer (VC), one aligned store, read once by the task | VC -> obstacle task |
| `scan_counter`, `path_counter` | obstacle_task.c | odd/even seqlock sequence | 32 | uint32, 4 B each | keep: sequence numbers wrap | obstacle task -> VC |

---

## 4. Hump estimation (Buddy 4)

Sources: `b4_cfg.c` (accel range +-2 g = +-2000 mg `[code]` B4_CFG_MAX_SAMPLE_MG,
noise limit 500 mg for a valid seed, lever 300 mm max, thresholds 6000 mg max),
`b4_task.h` (height 0..800 dmm `[code]` B4_TASK_HEIGHT_MAX_DMM, tilt +-1000 mg).

### 4.1 `b4_types_status_t` - shared snapshot (116 B -> 68 B)

| Field | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| `health` | b4_types.h:b4_types_status_t.health | 0..3 `[code]` | 2 U | enum, 4 B | `W0[0:1]` | Buddy 4 task -> VC, radio (mbf copy) |
| `fault` | .fault | 0..8 `[code]` | 4 U | enum, 4 B | `W0[2:5]` | same |
| `tilt` | .tilt | 0..6 `[code]` | 3 U | enum, 4 B | `W0[6:8]` | same |
| `motion` | .motion | 0..4 `[code]` | 3 U | enum, 4 B | `W0[9:11]` | same |
| `hump_state` | .hump_state | 0..5 `[code]` | 3 U | enum, 4 B | `W0[12:14]` | same |
| `turn` | .turn | 0..3 `[code]` | 2 U | enum, 4 B | `W0[15:16]` | same |
| `sample.timestamp_ms` | .sample.timestamp_ms | ms since boot, wraps | 32 U | uint32, 4 B | `W1[0:31]` | same |
| `sample.accel_x/y/z_mg` | .sample.accel_x_mg etc. | +-4000 mg `[code]` b4_proc.c:99 B4_PROC_MAX_SAMPLE_MG (raw +-2048 mg minus bias <= 2000 mg) | 3 x 13 s | 3 x int32, 12 B | `W2[0:12]`, `W2[13:25]`, `W3[0:12]` | same |
| `sample.b_valid` | .sample.b_valid | 0/1 | 1 U | bool, 1 B | `W2[26]` | same |
| `mag_sample.timestamp_ms` | .mag_sample.timestamp_ms | ms since boot | 32 U | uint32, 4 B | `W4[0:31]` | same |
| `mag_sample.mag_x/y/z` | .mag_sample.mag_x etc. | +-4096 counts `[ds]` B4_IMU_MAG_MAX_COUNTS | 3 x 13 s | 3 x int16, 6 B | `W5[0:12]`, `W5[13:25]`, `W6[0:12]` | same |
| `mag_sample.b_valid` | .mag_sample.b_valid | 0/1 | 1 U | bool, 1 B | `W6[13]` | same |
| `filtered_x/y/z_mg` | .filtered_x_mg etc. | +-4000 mg `[code]` (same validated range) | 3 x 13 s | 3 x int32, 12 B | `W7[0:12]`, `W7[13:25]`, `W8[0:12]` | same |
| `vertical_deviation_mg` | .vertical_deviation_mg | -6000..3999 mg `[code]`: filtered axis +-4000 minus gravity_reference_mg 1..2000 (b4_cfg.c:247) | 14 s | int32, 4 B | `W9[0:13]` | same |
| `roll_mg` | .roll_mg | +-4000 mg (filtered hump axis) | 13 s | int32, 4 B | `W9[14:26]` | same |
| `roll_baseline_mg` | .roll_baseline_mg | +-4000 mg (mean of the window, then 1 mg steps) | 13 s | int32, 4 B | `W10[0:12]` | same |
| `incline_delta_mg` | .incline_delta_mg | +-8000 mg `[code]`: difference of two +-4000 mg means | 14 s | int32, 4 B | `W10[13:26]` | same |
| `turn_cross` | .turn_cross | TBC (counts^2, threshold 20000); 25 s = +-16.7 M; publish-only (read at tests/buddy4/b4_car_test.c) | 25 s | int32, 4 B | `W11[0:24]` | same |
| `drive_speed_mm_s` | .drive_speed_mm_s | 0..1000 mm/s `[code]` encoder_physical_max | 11 U | int32, 4 B | `W12[0:10]` | same |
| `noise_mg` | .noise_mg | 0..500 mg `[code]` valid seed limit; publish-only (read at b4_task.c:896, 1183) | 9 U | uint32, 4 B | `W12[11:19]` | same |
| `learn_count` | .learn_count | 1 per mm of driving, saturating; TBC longest run; 24 U = 16 777 m | 24 U | uint32, 4 B | `W12[20:31]` + `W13[0:11]` | same |
| `detectable_height_um` | .detectable_height_um | TBC (threshold x lever); 20 U = 1.05 m; publish-only (read at b4_task.c:899, 926) | 20 U | uint32, 4 B | `W13[12:31]` | same |
| `current_hump_peak_mg` | .current_hump_peak_mg | 0..8000 mg `[code]`: amplitude is the difference of two +-4000 mg means; read back as a decision input (b4_proc.c:2807, 2838, 2863, 2894, 2951) | 13 U | uint32, 4 B | `W14[0:12]` | same |
| `highest_hump_peak_mg` | .highest_hump_peak_mg | 0..8000 mg; read back at b4_proc.c:2952 | 13 U | uint32, 4 B | `W14[13:25]` | same |
| `hump_peak_height_um` | .hump_peak_height_um | 0..80 000 um = 80 mm `[code]` B4_TASK_HEIGHT_MAX_DMM (the terrain word saturates there); publish-only, read at b4_task.c:801 | 17 U | uint32, 4 B | `W15[0:16]` | same |
| `highest_hump_height_um` | .highest_hump_height_um | 0..2 400 000 000 um `[code]`: 300000 um lever x 8000 mg / 1 mg (b4_cfg.c:33, 247); read back as a comparison bound at b4_proc.c:2961 | 32 U | uint32, 4 B | `W16[0:31]` | same |
| `active_hump_height_um` | .active_hump_height_um | 0..80 000 um `[code]` B4_TASK_HEIGHT_MAX_DMM; publish-only, read at b4_task.c:793 | 17 U | uint32, 4 B | `W15[17:31]` + `W14[26:27]` | same |
| `b_hump_height_valid` ... `b_jolt_window_active` | .b_* (8 flags) | 0/1 each | 8 U | 8 x bool, 8 B | `W0[17:24]` | same |

Whole snapshot: 116 B -> **17 words, 68 B**.

As built in Phase 5b (`b4_pack.h`, `b4_pack.c`): `b4_status_packed_t` is
the stored form, and the only long-lived copy is
`b4_proc.c:g_b4_proc_state.status`. The processor unpacks it into a
full-width `b4_types_status_t` local at the start of each call that
changes it, updates that working copy (rule 2), and packs it back before
returning; `b4_proc_get_status()` unpacks it straight into its caller's
snapshot. `b4_task.c` keeps the full-width snapshot only in a one-cycle
stack local, and the Vehicle Controller never sees the record at all: it
reads the terrain word, which is already minimum-bit (section 4.2).

Two details changed from the first proposal, each to keep the section 0
rules: the flags are 8, not 7 (`b_jolt_window_active` is the eighth, so
W0 uses bits 17-24), and the magnetometer Z field stays inside one word
(`W6[0:12]` with its valid flag in `W6[13]`) instead of straddling W5/W6.
`learn_count` (24 bits) is wider than the room left in its first word, so
it is stored as a low and a high part, both masked inside their own word
and recombined on unpack.  `active_hump_height_um` (17 bits) likewise
keeps its low 15 bits in W15 and its high 2 bits in W14.

A review of the reachable ranges widened every field that carries state
across calls or feeds a decision, so no reachable value is clamped
silently: the calibration and validity check bound a sample at +-4000 mg
(`b4_proc.c:99`, raw +-2048 mg minus a bias of at most 2000 mg,
`b4_cfg.c:25,250-255`), so `sample.accel_*`, `filtered_*`, `roll_mg` and
`roll_baseline_mg` use 13 s; the deviation subtracts a 1..2000 mg gravity
reference, so it uses 14 s; `incline_delta_mg` and the two peaks use the
+-8000 mg difference of two means (14 s and 13 U); and
`highest_hump_height_um` keeps a full 32 U word because it is read back as
a comparison bound and a config-legal lever and offset reach
300000 x 8000 / 1 um.  `hump_peak_height_um` and `active_hump_height_um`
are publish-only (read at `b4_task.c:801` and `b4_task.c:793`), so they
keep 17 U, and `turn_cross` stays 25 s (publish-only, read at
`tests/buddy4/b4_car_test.c`).  The record is still **17 words**.

### 4.2 Terrain word and Buddy 4 private state

| Field | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| `b4_task_terrain_t` | b4_task.h:b4_task_terrain_t | decode view of the 32-bit terrain word | 31 | struct, 10 B | keep: decode view | VC task (local) |
| terrain word | b4_task.h:B4_TASK_* macros | health 2, turn 2, height 10, highest 10, valid 1, count 4, cue 1, 2 reserved `[code]` = 30 used | 30 | uint32 in mbf, 4 B | keep: already minimum-bit; 2 reserve bits | Buddy 4 task -> VC (mbf) |
| drive word | b4_task.h:B4_TASK_DRIVE_* | driving 1, moving 1, speeds 2 x 11 s, seq 4, step 4 `[code]` | 32 | uint32 volatile, 4 B | keep: already minimum-bit | VC -> Buddy 4 task |
| `b4_task_cal_word` | b4_task.c:g_b4_task_cal_word | cal state 3, floor 9 `[code]` | 12 | uint32 volatile, 4 B | keep: 12 of 32 bits used, one writer; still one word | Buddy 4 task -> VC task |
| `g_b4_proc_state` | b4_proc.c:g_b4_proc_state | filter/estimator history: incline window 100 x int16 mg, sharp ring 3 x 5 x uint16, raw medians, config 96 B | n/a | 952 B | as built 704 B: status stored packed (17 words, 48 B less) and `incline_window_mg[100]` int16 (200 B instead of 400 B); every other array kept (rule 2) | Buddy 4 task only |
| `g_b4_imu_state` | b4_imu.c:g_b4_imu_state | config copy 96 B + IDs, health, fault, counters | n/a | 108 B | keep: driver state, config copy is read-only after init | Buddy 4 task only |
| `b4_cfg_config_t` | b4_cfg.h:b4_cfg_config_t | configuration constants | n/a | 96 B | keep: written once before tasks start, read-only afterwards | init -> Buddy 4 task |

As built in Phase 5b, `g_b4_proc_state` is **704 B**.  Its arrays, one
decision each (rule 2: filters, estimators and accumulators keep full
width):

| Array | Current | Decision | Range source and reason |
|---|---|---|---|
| `incline_window_mg[100]` | 100 x int32, 400 B | **int16, 200 B** (saved 200 B) | milli-g from the calibrated sample, whose validity check allows +-4000 mg (`b4_proc.c` B4_PROC_MAX_SAMPLE_MG) and whose sensor range is +-2 g = +-2000 mg (`b4_cfg.c` B4_CFG_ACCEL_RANGE_G).  Either fits int16.  The store clamps through `b4_pack_mg_i16()`; the spread and mean read into int32_t locals |
| `sharp_hp_x256[3]` | 3 x int32, 12 B | keep | High-pass filter state, scaled by 256: a filter accumulator, not a stored value |
| `sharp_ring_mg[3][5]` | 15 x uint16, 30 B | keep | Already 16-bit milli-g |
| `sharp_raw_mg[3][50]` | 150 x int16, 300 B | keep | Already 16-bit milli-g |
| `sharp_median_mg[3]` | 3 x int16, 6 B | keep | Already 16-bit milli-g |
| `hump_raw_mg[50]` | 50 x int16, 100 B | keep | Already 16-bit milli-g |
| `config` | `b4_cfg_config_t`, 96 B | keep | Tuning record, read-only after init |

The status record is the rest of the saving: the stored copy is
`b4_status_packed_t` (17 words, 68 B) instead of a 116 B
`b4_types_status_t`, and the full-width working copy lives in the stack
frame of the call that updates it.

---

## 5. Vehicle and controller states

Sources: `vc.h` (VC_SEARCH_LIMIT_MDEG 90 000, VC_CAL_LIMIT_MDEG 45 000,
VC_SPEED_* 50..150, VC_LIFT_SPELL_CAP_CYCLES 25, VC_MAX_RESCANS 2),
`vc_task.h` (10 fault bits).

### 5.1 `vc_t` - controller state (88 B -> 40 B)

As built in Phase 5c (`vc_pack.c`): the VC task keeps `g_vc` as
`vc_packed_t` (10 words, 40 B), unpacks it into a stack `vc_t` working copy
at the start of each 20 ms step, runs `vc_step()` on that copy, and packs it
back.  `vc_t` stays the full-width value type that `vc.c` operates on and
that `tests/vc` drives.  Only the VC task reads or writes either form, so
no lock is needed.  A stored record whose enum code or reserved bit is not
a value the packer can write is never run: the working copy restarts at
INITIALISE and the step sees a serious fault, which the VC's safety rule
turns into EMERGENCY_STOP (the shared fault mask has no VC-internal bit, so
VC_FAULT_MOTION_CMD is reused).  The table below is the as-built layout; it
differs from the first proposal in two widths, both because the value is
read back in a later step and must never clamp:

* `search_origin_mdeg` needs 32 s, not 18 s.  It is an absolute encoder
  angle, not a bounded offset: `vc_task.c:359-361` computes the input as
  (left-right counts since the sweep origin) x 34 mdeg, and the encoder
  counts are stored full width (section 1.2), so a sweep difference can
  exceed 18 s.  `vc.c:1010` reads it back as
  `cal_angle_mdeg - search_origin_mdeg`, so an 18-bit field would clamp a
  reachable value and move the second pivot.
* `search_limit_mdeg` needs 17 U, not 18: the only values assigned are
  VC_SEARCH_LIMIT_MDEG (90 000) and VC_SEARCH_LIMIT_POST_BYPASS_MDEG
  (45 000), `vc.c:582-586`.

| Field | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| `state` | vc.h:vc_t.state | 0..10 `[code]` vc_state_t | 4 U | enum, 4 B | `W0[0:3]` | VC task only |
| `entry_ms` | .entry_ms | ms since boot, wraps | 32 U | uint32, 4 B | `W1[0:31]` | VC task only |
| `stored_nav` | .stored_nav | 0..4 `[code]` vc_nav_t | 3 U | enum, 4 B | `W0[4:6]` | same |
| `rescans` | .rescans | 0..2 `[code]` VC_MAX_RESCANS | 2 U | uint8, 1 B | `W0[7:8]` | same |
| `b_obstacle_active`, `b_reversing` | .b_obstacle_active, .b_reversing | 0/1 | 2 U | 2 x bool, 2 B | `W0[9:10]` | same |
| `prev_line` | .prev_line | 0..4 `[code]` vc_line_t | 3 U | enum, 4 B | `W0[11:13]` | same |
| `last_side` | .last_side | -1..+1 `[code]` | 2 s | int8, 1 B | `W0[14:15]` | same |
| `search` | .search | 0..2 `[code]` | 2 U | enum, 4 B | `W0[16:17]` | same |
| `search_dir` | .search_dir | -1/+1 `[code]` | 2 s | int32, 4 B | `W0[18:19]` | same |
| `search_origin_mdeg` | .search_origin_mdeg | encoder-count difference x 34 mdeg (vc_task.c:359-361) `[code]` | 32 s | int32, 4 B | `W2[0:31]` (widened from 18 s) | same |
| `search_limit_mdeg` | .search_limit_mdeg | 45 000 or 90 000 mdeg `[code]` | 17 U | int32, 4 B | `W5[0:16]` (narrowed from 18 U) | same |
| `b_after_bypass` | .b_after_bypass | 0/1 | 1 U | bool, 1 B | `W0[20]` | same |
| `bypass_end_ms` | .bypass_end_ms | ms since boot, wraps | 32 U | uint32, 4 B | `W3[0:31]` | same |
| `cal_phase` | .cal_phase | 0..5 `[code]` | 3 U | enum, 4 B | `W0[21:23]` | same |
| `cal_phase_ms` | .cal_phase_ms | ms since boot | 32 U | uint32, 4 B | `W4[0:31]` | same |
| `cal_out_mdeg`, `cal_far_mdeg` | .cal_out_mdeg, .cal_far_mdeg | 0..90 000 mdeg = CAL_LIMIT plus the other side, and the 20 000 mdeg back margin `[code]` | 2 x 18 U | 2 x int32, 8 B | `W6[0:17]`, `W7[0:17]` | same |
| `cal_first_dir`, `cal_dir` | .cal_first_dir, .cal_dir | -1..+1 | 2 x 2 s | 2 x int32, 8 B | `W6[18:19]`, `W6[20:21]` | same |
| `b_junction_hold`, `b_junction_track` | .b_junction_hold, .b_junction_track | 0/1 | 2 U | 2 x bool, 2 B | `W0[24:25]` | same |
| `junction_origin_mm`, `junction_seen_mm` | .junction_origin_mm, .junction_seen_mm | TBC travel range (24 s) | 2 x 24 s | 2 x int32, 8 B | `W8[0:23]`, `W9[0:23]` | same |
| `b_junction_seen` | .b_junction_seen | 0/1 | 1 U | bool, 1 B | `W0[26]` | same |
| `pending_turn` | .pending_turn | 0..4 `[code]` vc_nav_t | 3 U | enum, 4 B | `W0[27:29]` | same |
| `steer_side` | .steer_side | -1..+1 | 2 s | int8, 1 B | `W5[17:18]` | same |
| `b_right_exit` | .b_right_exit | 0/1 | 1 U | bool, 1 B | `W0[30]` | same |
| `steer_mag` | .steer_mag | 0..127 `[code]` VC_SEARCH_STEER | 7 U | int16, 2 B | `W5[19:25]` | same |
| `lift_spell_cycles` | .lift_spell_cycles | 0..25 `[code]` CAP | 5 U | uint8, 1 B | `W5[26:30]` | same |
| `lift_spell_max_cycles` | .lift_spell_max_cycles | 0..25 | 5 U | uint8, 1 B | `W7[18:22]` | same |
| `b_lift_spell_cued` | .b_lift_spell_cued | 0/1 | 1 U | bool, 1 B | `W5[31]` | same |
| whole state | vc.h:vc_t | as above | 284 | struct, 88 B | **10 words, 40 B** | VC task only |
| whole instance | vc_task.c:g_vc | one `vc_packed_t`, the controller's live state | 284 | struct, 88 B | same as `vc_t`: 40 B | VC task only (the file states it is never shared) |

36 bits of the ten words are spare (W0 bit 31, W6 bits 22-31, W7 bits 23-31,
W8/W9 bits 24-31).  All are written as 0, and a stored record with any of
them set is rejected, like an out-of-range enum code.  `vc_pack.h` has the
field-by-field layout and the range sources; `tests/vc/test_vc_pack.c`
checks the size, the field ends, the rejected records, a round trip of a
`vc_t` captured mid-run from the same scenarios `test_vc.c` uses, and the
bypass plan record.

### 5.2 `vc_inputs_t` and `vc_outputs_t` - one 20 ms step

Both are locals filled and consumed inside the VC task each cycle (stack, not
static data), so packing them changes no static RAM.  **As built: kept
full width, transient on the stack** - the per-field proposal below is not
implemented, because the VC task builds `in` and `out` fresh every step and
no copy is stored, queued or shared (neither type has a static or global
instance in `vc_task.c`, `vc.c` or the tests).  They are listed because
they carry every shared input and output of the controller.

| Field | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| `now_ms` | vc.h:vc_inputs_t.now_ms | ms since boot, wraps | 32 U | uint32, 4 B | keep 32 (timing) | VC local |
| `b_init_ok`, `b_init_failed` | .b_init_ok, .b_init_failed | 0/1 | 2 U | 2 x bool, 2 B | `W0[0:1]` | VC local |
| `cmd` | .cmd | 0..2 `[code]` | 2 U | enum, 4 B | `W0[2:3]` | radio -> VC (queue) |
| `line`, `line_error`, `line_sensors` | .line, .line_error, .line_sensors | 0..4, -127..127, 0..3 | 3+8 s+2 | 4+4+1 B | `W0[4:6]`, `W0[7:14]`, `W0[15:16]` | Buddy 3 -> VC |
| `b_barcode_capture`, `barcode` | .b_barcode_capture, .barcode | 0/1, 0..4 | 1+3 | 1+4 B | `W0[17]`, `W0[18:20]` | Buddy 3 -> VC |
| motion flags | .b_motion_done, .b_motion_failed, .b_stationary | 0/1 | 3 U | 3 B | `W0[21:23]` | Motion -> VC |
| terrain flags | .b_hump, .b_ramp_active, .b_imu_cue, .b_terrain_ready | 0/1 | 4 U | 4 B | `W0[24:27]` | Buddy 4 -> VC |
| obstacle flags | .b_obstacle_ahead, .b_scan_done, .b_obstacle_passed, `recommendation` | 0/1 x3, 0..4 | 3+3 | 3+4 B | `W0[28:30]`, `W1[0:2]` | Buddy 5 -> VC |
| `serious_faults` | .serious_faults | 10 fault bits (section 9) | 10 U | uint16, 2 B | `W1[3:12]` | VC task (own mask) |
| `b_finish` | .b_finish | 0/1 | 1 U | bool, 1 B | `W1[13]` | VC local |
| `cal_b3`, `cal_b4` | .cal_b3, .cal_b4 | 0..5, 0..4 | 3+3 | 4+4 B | `W1[14:16]`, `W1[17:19]` | Buddy 3/4 -> VC |
| `cal_angle_mdeg` | .cal_angle_mdeg | +-65 000 mdeg `[code]` | 18 s | int32, 4 B | `W2[0:17]` | Motion -> VC |
| `travel_mm` | .travel_mm | TBC; 24 s | 24 s | int32, 4 B | `W3[0:23]` | Motion -> VC |
| whole inputs | vc.h:vc_inputs_t | as above | 125 | struct, 40 B | kept: transient, stack | VC local |
| `state`, `b_state_changed`, `motion` | vc.h:vc_outputs_t | 0..10, 0/1, 0..6 | 4+1+3 | 4+1+4 B | `W0[0:3]`, `W0[4]`, `W0[5:7]` | VC -> subsystems |
| `speed_mm_s` | .speed_mm_s | +-300 mm/s | 10 s | int16, 2 B | `W0[8:17]` | Motion |
| `steer` | .steer | -127..+127 `[code]` | 8 s | int8, 1 B | `W0[18:25]` | Motion |
| `turn`, `bypass_side`, `obstacle` | .turn, .bypass_side, .obstacle | 0..4, 0..4, 0..3 | 3+3+2 | 3 enums, 12 B | `W0[26:28]`, `W0[29:31]`+`W1[0]`, `W1[1:2]` | Motion, obstacle |
| `pivot_mdeg` | .pivot_mdeg | +-360 000 mdeg `[code]` | 20 s | int32, 4 B | `W1[3:22]` | Motion |
| `cal_req`, `cal_fail`, `b_lift_confirmed` | .cal_req, .cal_fail, .b_lift_confirmed | 0..5, 0..4, 0/1 | 3+3+1 | 4+4+1 B | `W1[23:25]`, `W1[26:28]`, `W1[29]` | Buddy 3/4, radio |
| whole outputs | vc.h:vc_outputs_t | as above | 61 | struct, 20 B | kept: transient, stack | VC local |

---

## 6. Data shared between tasks, or between an ISR and a task

| Data | Where | Valid range and unit | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| encoder IRQ counter | motion_encoder.c:motion_encoder_irq_count | 4 IRQ per count x 14 counts/mm, wraps | 32 U | volatile uint32, 4 B | keep 32: ISR increments it, a wrap must be detectable | encoder ISR -> Motion task |
| encoder snapshot | motion_encoder.c:motion_encoder_get_snapshot | see 1.3 | 128 | 32 B (6-word copy inside) | 5 words, 20 B, seqlock unchanged | ISR -> Motion task |
| Buddy 3 status word | buddy3_status.h:buddy3_packed_t | see 2.1 | 23 | uint32, 4 B per mbf slot | keep | Buddy 3 -> VC, radio |
| Buddy 3 reading word | buddy3_task.c:buddy3_reading_word | centre/right ADC 12+12 | 24 | uint32 volatile, 4 B | keep | Buddy 3 -> VC |
| Buddy 3 cal word | buddy3_task.c:buddy3_cal_word | request 3 + counter 29 | 32 | uint32 volatile, 4 B | keep | VC -> Buddy 3 |
| Buddy 4 drive word | b4_task.c:g_b4_task_drive_word | see 4.2 | 32 | uint32 volatile, 4 B | keep | VC -> Buddy 4 |
| Buddy 4 cal word | b4_task.c:g_b4_task_cal_word | state 3 + floor 9 | 12 | uint32 volatile, 4 B | keep | Buddy 4 -> VC |
| Buddy 4 cal samples | b4_task.c:g_b4_task_cal_samples | 0..20000 `[code]` | 15 U | volatile uint32, 4 B | keep: one word, written before the state | Buddy 4 -> VC |
| terrain word | b4_task.c:g_b4_task_word_buf / mbf | see 4.2 | 32 | 4 B per message | keep | Buddy 4 -> VC |
| obstacle status word | obstacle_task.c:status_word | see 3.1 | 26 | uint32 volatile, 4 B | keep | obstacle task -> VC |
| obstacle scan result | obstacle_task.c:scan_result | see 3.2 | 410 | 300 B + 4 B counter | 17 words, 68 B, seqlock unchanged | obstacle task -> VC |
| obstacle path result | obstacle_task.c:path_result | see 3.3 | 95 | 52 B + 4 B counter | 3 words, 12 B, seqlock unchanged | obstacle task -> VC |
| path request words | obstacle_task.c:path_need_mm, path_known_mm, path_side | 0..4095, 0..4 | 12+12+3 | 3 x 4 B = 12 B | one word, 27 bits, single VC write | VC -> obstacle task |
| obstacle event flag | obstacle_task.c:flag_id | OBSTACLE_EVT_* bits 0..3 | 4 | T_CFLG, kernel | keep: event-flag bits, multi-writer = task only | obstacle task -> VC |
| comms state word | comms.c:g_state | 0..9 `[code]` comms_state_t | 4 | volatile uint32, 4 B | keep one word: replaced whole, one writer | VC -> radio task |
| comms fault word | comms.c:g_fault | 10 fault bits (section 9) | 10 | volatile uint32, 4 B | keep one word: replaced whole, one writer | VC -> radio task |
| comms counters | comms.c:g_events_sent, g_events_dropped, g_telemetry_sent, g_commands_received | 0..2^32-1, wrap; dropped saturates at 255 on the wire | 32 each | 4 x volatile uint32, 16 B | keep: one writer each, read for diagnostics | callers -> radio task |
| `comms_stats_t` | comms.h:comms_stats_t | `b_connected` 1 bit + 4 counters 0..2^32-1, wrap | 129 | struct, 20 B | keep: transient API copy, one writer per field | radio task -> any caller |
| motion snapshot slot | comms.c:g_motion_slot | 4 B seq + see 8.1 | 32 + 146 | 48 B | seq 4 B + 6 words = 28 B | VC -> radio task (seqlock) |
| line snapshot slot | comms.c:g_line_slot | 4 B seq + see 8.3 | 32 + 37 | 20 B | seq 4 B + 2 words = 12 B | Buddy 3 -> radio task (seqlock) |
| terrain snapshot slot | comms.c:g_terrain_slot | 4 B seq + see 8.4 | 32 + 35 | 20 B | seq 4 B + 2 words = 12 B | Buddy 4 -> radio task (seqlock) |
| VC fault history | vc_task.c:g_faults_seen | 10 fault bits | 10 | uint16, 2 B | keep: one writer, read at run end | VC task only |
| VC pending command | vc_task.c:g_pending | motion_command_t, see 1.1 | 61 | 20 B | 2 words, 8 B | VC task only |
| VC scan/profile copies | vc_task.c:g_profile, g_scan_req, g_plan, g_line_watch | profile word 64, request 2, plan 77, watch 33 | 176 | 24 + 1 + 32 + 8 B | profile: 2 words (8 B); plan keep or 3 words (12 B) | VC task only |

---

## 7. Inter-task queue messages

Message buffers (mbf) and other IPC objects created in `app_program`.
`COMMS_MBF_OVERHEAD` is 8 B per message in `comms.c`; the kernel adds a small
header (4 B is what `buddy3_task.c` and `b4_task.c` reserve).  Buffer bytes
below are the static backing arrays as declared.

| Queue / IPC object | Created in | Message now | Slots | Buffer now | Message proposed | Buffer proposed |
|---|---|---|---|---|---|---|
| events to the radio task, `g_event_mbf` | comms.c:comms_init | `comms_event_t` 76 B | 16 `[code]` COMMS_EVENT_QUEUE_LEN | 1344 B (`g_event_buf`) | 28 B (7 words) | 576 B |
| laptop commands, `g_cmd_mbf` | comms.c:comms_init | uint32 4 B | 4 `[code]` | 48 B (`g_cmd_buf`) | keep 4 B (kernel word) | 48 B |
| debug text, `g_debug_mbf` | comms.c:comms_init | char[200] 200 B | 4 `[code]` | 832 B (`g_debug_buf`) | keep 200 B: TEMPORARY diagnosis code, deleted with COMMS_DEBUG_TEXT | 832 B |
| motion commands, `command_mbf` | motion_task.c:motion_start_task | `motion_command_t` 20 B | 4 `[code]` MOTION_QUEUE_DEPTH | 112 B (`command_buffer`, staging array `command_queue` 80 B + 32 B header) | 8 B | 64 B |
| Buddy 3 status, `buddy3_output_mbfid` | buddy3_task.c | `buddy3_packed_t` 4 B | 4 `[code]` | 80 B (`buddy3_output_mbf_buf`) | keep: 4 B word | 80 B |
| Buddy 4 terrain word, `g_b4_task_mbf_id` | b4_task.c:b4_task_start | uint32 4 B | 2 `[code]` B4_TASK_WORD_SLOTS | 16 B (`g_b4_task_word_buf`) | keep: 4 B word | 16 B |
| obstacle requests / events, `flag_id` | obstacle_task.c:obstacle_task_init | event flags, 4 bits used | - | no buffer | keep | no buffer |
| motion stop, `stop_flg` | motion_task.c | event flag bit 0 | - | no buffer | keep | no buffer |
| Buddy 4 request, `g_b4_task_flag_id` | b4_task.c | event flag bits | - | no buffer | keep | no buffer |
| shared packed words (no queue) | see section 6 | 1 word each | - | 4 B each | keep: single writer, single store | 4 B each |

The `comms_event_t` message is the one real queue win, because it must hold the
largest payload (the calibration event, 166 bits after the header).  Proposed
`comms_event_packed_t` = 7 words = 28 B:

| Word | Layout |
|---|---|
| W0 | `time_ms`, 32 U (ms since boot, wraps) |
| W1 | bits 0-2 event type; bits 3-31 payload (barcode: code 8 + command 3; hump: peak 10; motion: class 3; calibration: result 3, centre_low 12, centre_high 12, learned 1, first_right 1) |
| W2 | obstacle: dist 12, angle 8, width 10; or calibration: centre_thr 12, centre_full 12, sweep 8 |
| W3 | obstacle: clear_l 12, clear_r 12, action 3; or calibration: right_low 12, right_high 12 |
| W4-W6 | calibration only: right_thr, barcode_low; barcode_high, b4_samples 15; barcode_thr, overlap 9 s, b4_floor 9 |

As built in Phase 2 (`comms_codec.c`). The calibration payload (166 bits) does not fit W2-W6 alone, so it also uses W1 bits 3-31.

---

## 8. MQTT telemetry and command payloads

The wire format is already minimum-bit binary (`comms_codec.c`,
`docs/BINARY_FORMAT.md`); this section records the widths in
`comms.c` lines 65-99 (`BITS_*`) against the ranges in this document, and flags
mismatches.  Header is always 2 version bits + 32 time bits = 34 bits.

### 8.1 `robot/telemetry/motion` (Buddy 2)

| Field | `BITS_*` | bits | Range source | Status |
|---|---|---|---|---|
| `spd_l_mm_s`, `spd_r_mm_s` | BITS_SPEED | 10 s | +-300 mm/s `[code]` | ok |
| `enc_l`, `enc_r` | BITS_ENCODER | 19 s | counts since boot `[TBC]`; 19 s = +-262 143 counts = **+-18.7 m**, too small for a full run | **wire mismatch: needs 22 s (+-150 m) or a documented reset rule**.  The stored slot keeps the full 32 s value (`comms_codec.c:M_W_ENC_L/M_W_ENC_R`), so the run summary and wheel difference no longer see a clamped copy; the wire still clamps at 19 s |
| `dist_mm` | BITS_DISTANCE | 16 s | +-32 767 mm `[TBC]` (longest run) | open Q2.  The stored slot keeps the full 32 s value (`comms_codec.c:M_W_DIST`), so a summary subtraction is exact; the wire still clamps at 16 s |
| `tgt_l_mm_s`, `tgt_r_mm_s`, `slew_l_mm_s`, `slew_r_mm_s` | BITS_SPEED | 10 s each | +-300 mm/s | ok |
| `pwm_l`, `pwm_r` | BITS_DRIVE | 11 s each | +-1000 permille | ok |
| message | | 136 + 34 = 170 | 22 B | `BINARY_FORMAT.md` documents only 5 fields (14 B) |

### 8.2 `robot/heartbeat` and `robot/state`

| Field | `BITS_*` | bits | Range source | Status |
|---|---|---|---|---|
| `seq` | BITS_SEQ | 16 U | wraps | ok |
| `state` | BITS_STATE | 4 U | 0..9 `comms_state_t`, 0..10 `vc_state_t` | ok size, mapping still open (open Q12) |
| `fault` | BITS_FAULT | 10 U | bits 0-9 of the VC mask | **mismatch: `BINARY_FORMAT.md` says 9 bits; `comms.h` says bits 0-15; code sends 10 (`FAULT_MASK` 0x3FF)** |
| `ev_drop` | BITS_DROPS | 8 U | saturates at 255 | ok |
| state message | | 14 + 34 = 48 | 6 B | ok |
| heartbeat | | 38 + 34 = 72 | 9 B | ok (size unchanged, bit count differs) |

### 8.3 `robot/telemetry/line` (Buddy 3)

| Field | `BITS_*` | bits | Range source | Status |
|---|---|---|---|---|
| `pos` | BITS_LINE_POS | 3 U | 0..4 LINE_* | ok |
| `err` | BITS_LINE_ERROR | 8 s | -127..+127 | ok |
| `sensors` | BITS_SENSORS | 2 U | 2 sensors | ok |
| `centre_adc` | BITS_ADC | 12 U | 0..4095 | **missing from `BINARY_FORMAT.md`** |
| `right_adc` | BITS_ADC | 12 U | 0..4095 | **missing from `BINARY_FORMAT.md`** |
| message | | 37 + 34 = 71 | 9 B | doc says 6 B (3 fields) |

### 8.4 `robot/telemetry/terrain` and `robot/event/motion` (Buddy 4)

| Field | `BITS_*` | bits | Range source | Status |
|---|---|---|---|---|
| `motion` | BITS_MOTION_CLASS | 3 U | 0..5 | ok |
| `tilt_mg` | BITS_TILT | 11 s | +-1000 mg | ok |
| `hump_dmm`, `max_hump_dmm` | BITS_HUMP | 10 U | 0..800 | ok |
| `turn_dps` | BITS_TURN | 11 s | +-288.8 deg/s `[code]` 2*300/119 mm | 10 s (+-511) is enough; 11 s is safe but one bit more than needed |
| terrain message | | 45 + 34 = 79 | 10 B | ok |
| motion event | | 3 + 34 = 37 | 5 B | ok |

### 8.5 Obstacle, run, command, calibration events

| Field | `BITS_*` | bits | Range source | Status |
|---|---|---|---|---|
| `dist_mm` | BITS_OBST_DIST | 12 U | 0..4000 mm | ok |
| `angle_deg` | BITS_ANGLE | 8 U | 0..180 (bearing), 7 bits would need an offset | ok |
| `width_mm` | BITS_WIDTH | 10 U | 0..1023, saturating; max obstacle TBC | open Q5 |
| `clear_l_mm`, `clear_r_mm` | BITS_CLEARANCE | 12 U | 0..4095 | ok |
| `action` / `bypass_side` | BITS_ACTION | 3 U | 0..4 | ok |
| obstacle message | | 57 + 34 = 91 | 12 B | ok |
| `run` | BITS_RUN | 1 U | 0/1 | ok |
| `run_ms` | BITS_RUN_MS | 20 U | 0..1 048 575 ms = 17.5 min `[TBC]` | open Q3 |
| `humps`, `barcodes`, `obstacles` | BITS_COUNT | 8 U each | 0..255 | ok |
| summary | | 70 + 34 = 104 | 13 B | ok |
| `cmd`, `ok` | BITS_CMD, BITS_OK | 2 + 1 | 0..3, 0/1 | ok |
| command reply | | 3 + 34 = 37 | 5 B | ok |
| calibration: `result` | BITS_CAL_RESULT | 3 U | 0..4 | ok |
| calibration: 10 ADC fields | BITS_ADC | 12 U each | 0..4095 | ok |
| `barcode_learned`, `first_right` | BITS_CAL_LEARNED, BITS_CAL_FIRST_RIGHT | 1 + 1 | 0/1 | ok |
| `overlap` | BITS_CAL_OVERLAP | 9 s | -255..+255, -256 = not measured `[code]` | ok |
| `b4_floor_mg` | BITS_CAL_FLOOR | 9 U | 1..500 | ok |
| `b4_samples` | BITS_CAL_SAMPLES | 15 U | 0..20000 | ok |
| `sweep_deg` | BITS_CAL_ANGLE | 8 U | 0..90 `[code]` | ok |
| calibration event | | 166 + 34 = 200 | 25 B | **missing from `BINARY_FORMAT.md`** |

The two payload structs, with their own storage where a copy is kept:

| Struct | Where | Fields and ranges | Bits | Current | Proposed | Shared? |
|---|---|---|---|---|---|---|
| `comms_obstacle_t` | comms.h:comms_obstacle_t | dist 12, angle 8, width 10, clear_l 12, clear_r 12, action 3 | 57 | struct, 24 B | keep source view; a stored copy is the 64-bit profile word (8 B) | Buddy 5 / VC -> radio task |
| `comms_calibration_t` | comms.h:comms_calibration_t | result 3, 10 x ADC 12, learned 1, first_right 1, overlap 9 s, floor 9, samples 15, sweep 8 | 166 | struct, 40 B | **6 words, 24 B**, single writer (VC) -> event queue | VC -> radio task |

### 8.6 Laptop -> robot `robot/command`

| Form | Payload | Bits needed | Status |
|---|---|---|---|
| binary | 1 byte: 0x01 START, 0x02 STOP, 0x03 STATUS | 2 | keep 1 byte: MQTT payloads are byte strings, sub-byte packing buys nothing |
| text | `start`, `stop`, `status` | - | keep: human interface, both formats accepted |

### 8.7 Documentation mismatches found (flag only, not fixed here)

1. `BINARY_FORMAT.md` heartbeat and state say `fault` is 9 bits; `BITS_FAULT`
   is 10 and `FAULT_MASK` 0x3FF sends bits 0-9.
2. `comms.h` says "bits 0-15 are sent" for `comms_set_fault()`; only 10 are.
3. `BINARY_FORMAT.md` has no `robot/event/calibration` message (17 fields,
   25 B) although `comms.c` sends it.
4. `BINARY_FORMAT.md` line telemetry lists 3 fields (6 B); the code sends 5
   (9 B, with `centre_adc` and `right_adc`).
5. `BINARY_FORMAT.md` motion telemetry lists 5 fields (14 B); the code sends 11
   (22 B).
6. `DATA_DEFINITIONS.md` is referenced by `comms.h`, `vc.h`, `vc_task.h` and
   `BINARY_FORMAT.md` but does not exist in `docs/`.
7. `COMMS_MBF_OVERHEAD` is 8 B per message in `comms.c` but 4 B is reserved in
   `buddy3_task.c` and `b4_task.c`.
8. `BINARY_FORMAT.md` §4 state codes (0..9) are the comms states, while the VC
   has 11 states and maps them in `vc_task.c`; the mapping is not written down.

---

## 9. Shared fault bitmask

One shared bitmask; each bit is an individual fault.  Held in `uint16_t`
(`vc_inputs_t.serious_faults`, `vc_task.c:g_faults_seen`) and in the 32-bit
`comms.c:g_fault`, then sent in 10 bits.

| Bit | Name | Owner / writer | Set rule | Clear rule |
|---:|---|---|---|---|
| 0 | `VC_FAULT_STALL` | VC task (`vc_task.h`) | Motion status fault == STALL, or travel command with no encoder progress | Next run start / `vc_init()` |
| 1 | `VC_FAULT_ENCODER` | VC task | Motion status fault == ENCODER, or encoder fault detail != NONE | same |
| 2 | `VC_FAULT_MOTION_CMD` | VC task | Motion refused the command (rejected, invalid, hardware, timeout, status unreadable) | same |
| 3 | `VC_FAULT_IMU` | VC task | Buddy 4 not ready, faulty or silent; published only, never a stop cause | Buddy 4 healthy and fresh |
| 4 | (unused) | - | always 0 | - |
| 5 | `VC_FAULT_ULTRASONIC` | VC task | Buddy 5 SENSOR_FAULT | Buddy 5 healthy |
| 6 | `VC_FAULT_CAL_CONTRAST` | VC task | START calibration failed with CONTRAST | Next START |
| 7 | `VC_FAULT_CAL_LINE` | VC task | START calibration failed with LINE_NOT_FOUND | Next START |
| 8 | `VC_FAULT_CAL_BUDDY4` | VC task | START calibration failed with BUDDY4 | Next START |
| 9 | `VC_FAULT_CAL_GAP` | VC task | START calibration failed with SENSOR_GAP | Next START |

**Proposed width: 10 bits (bits 0-9)**, which is what the code already sends.
Minimum for the current set is 10 because bit 9 is the highest used bit; the
gap at bit 4 does not reduce the width.  Alternatives: (a) keep bit 4 reserved
for a future fault (recommended: no renumbering of published bits); (b) move
`VC_FAULT_ULTRASONIC` to bit 4 and shrink the field to 9 bits, which only saves
a bit in a 4-byte word and breaks existing decoders - not recommended.

**Race safety of the mask.**  Today the *whole word* has one writer (the VC
task), so a single aligned store is enough (`comms_set_fault()` replaces the
word; `g_faults_seen` is OR-ed by the same task).  If another task must set a
bit later, it does not write the mask: it sets a uT-Kernel event flag bit
(`tk_set_flg`, one bit per writer, no read-modify-write) and the VC task, the
mask's only writer, folds it in with one read-modify-write while no other
writer exists.  A mutex is the fallback if two tasks must both change bits.
Bit assignment must keep each bit's meaning stable: the mask is transmitted.

---

## 10. Queue and message sizes

This repeats section 7 as the CR1 "before / after" message-size table, adding
the wire message sizes.

| Message / buffer | Where | Before | After (proposed) | Notes |
|---|---|---|---|---|
| `comms_event_t` (queue element) | comms.c:g_event_mbf | 76 B | 28 B | largest event is calibration: 166 payload bits |
| event queue backing store | comms.c:g_event_buf | 1344 B | 576 B | 16 slots x (msg + 8 B header) |
| `motion_command_t` (queue element) | motion_task.c:command_mbf | 20 B | 8 B | type-tagged union in one payload word |
| motion command buffer | motion_task.c:command_buffer | 112 B | 64 B | 4 slots x (8 + 8) |
| `motion_status_t` (snapshot copy) | motion_task.c:motion_status_stored + `status_mtx` | 160 B | 104 B (40 live + 64 diagnostic) | mutex copy, not a queue; see 1.2 for the as-built layout |
| `comms_motion_t` (snapshot slot) | comms.c:g_motion_slot | 44 B (+4 seq) | 24 B (+4 seq) | seqlock |
| `comms_line_t` (snapshot slot) | comms.c:g_line_slot | 16 B (+4 seq) | 8 B (+4 seq) | seqlock |
| `comms_terrain_t` (snapshot slot) | comms.c:g_terrain_slot | 16 B (+4 seq) | 8 B (+4 seq) | seqlock |
| `buddy3_packed_t` | buddy3_task.c:buddy3_output_mbf | 4 B | 4 B | already packed |
| terrain word | b4_task.c:g_b4_task_mbf | 4 B | 4 B | already packed |
| `b4_types_status_t` (stored copy) | b4_proc.c:g_b4_proc_state.status | 116 B | 68 B | `b4_status_packed_t`, 17 words; packed on publish and unpacked in `b4_proc_get_status()` |
| `obstacle_scan_result_t` | obstacle_task.c:scan_result | 300 B | 68 B | seqlock, not a queue; as built (packed points, header, profile word) |
| `obstacle_path_t` | obstacle_task.c:path_result | 52 B | 12 B | seqlock, not a queue; as built |
| `vc_t` (stored copy) | vc_task.c:g_vc | 88 B | 40 B | VC-private, not a queue; `vc_packed_t` 10 words, unpacked into a stack working copy per step (section 5.1) |
| VC bypass plan (stored copy) | vc_task.c:g_plan | 32 B | 12 B | VC-private; `vc_plan_packed_t` 3 words, unpacked once per use (section 11 row 15) |
| VC obstacle profile (stored copy) | vc_task.c:g_profile_word | 24 B | 8 B | VC-private; the existing 64-bit profile word, `obstacle_calc.h` (section 11 row 16) |
| path-check request word | obstacle_task.c:path_request | 12 B (3 volatiles) | 4 B | as built: one word, one store, read once |
| MQTT motion telemetry | comms.c send | 22 B binary (JSON ~100 B) | 22 B (23 B if encoder becomes 22 bits) | section 8.1 |
| MQTT line telemetry | comms.c send | 9 B | 9 B | section 8.3 |
| MQTT terrain telemetry | comms.c send | 10 B | 10 B | section 8.4 |
| MQTT heartbeat | comms.c send | 9 B | 9 B | section 8.2 |
| MQTT obstacle event | comms.c send | 12 B | 12 B | section 8.5 |
| MQTT calibration event | comms.c send | 25 B | 25 B | not yet in BINARY_FORMAT.md |

---

## 11. Proposed changes, ranked by RAM saved

Static RAM estimates only (stack excluded).  Risk is the chance of changing
timing, precision or a race.  Phases: 2 = comms + fault mask, 3 = motion,
4 = line + barcode, 5 = Buddy 4 + obstacle + VC.

| # | Change | Files | Static bytes saved | Risk | Phase |
|---:|---|---|---:|---|---|
| 1 | `comms_event_t` 76 -> 28 B packed (`comms_event_packed_t`), event queue 1344 -> 576 B | comms.c, comms.h, tests/comms, tools/robot_decode.ps1 | 768 | Low: codec tests have golden bytes, encoder only | 2 |
| 2 | `raw_encoder[480]` int32 -> int16 offset from a per-capture base (`raw_encoder_base`), as built | barcode.c | 960 | Medium: decode still uses absolute positions (base + offset); an offset leaving int16 clamps and sets `raw_overflow`, which reports the capture invalid | 4 |
| 3 | `cal_history[256]` 4 B -> 3 B per sample (12+12 bits), as built | line_follow.c, line_follow.h | 256 | Medium: unaligned byte triplets, sweep timing unchanged (decode output byte-identical in the host tests) | 4 |
| 4 | `barcode_edges[64]` 12 B -> 8 B (1 + 31 + 32 bits: is_black, position, timestamp), as built | barcode.c, barcode.h | 256 | Low: same values, `barcode_get_edge()` API unchanged | 4 |
| 5 | `raw_adc[480]` uint16 -> 12-bit byte triplets, as built | barcode.c, barcode.h | 240 | Medium: unaligned, decode reads through helpers | 4 |
| 6 | `obstacle_scan_result_t` 300 -> 68 B (packed points + profile word), as built | obstacle_types.h, obstacle_scan.c, obstacle_task.c, obstacle_pack.c, tests/obstacle | 232 | Medium: seqlock copy size changes, VC unpack needed | 5 |
| 7 | `raw_widths[]`, `variant_widths[]` uint32[63] -> uint16[63], as built | barcode.c, barcode.h | 252 | Low: width <= 2460 counts `[code]`; saturating store, sums in 32-bit locals | 4 |
| 8 | `g_b4_proc_state.incline_window_mg[100]` int32 -> int16 (mg, +-2000) | b4_proc.c | 200 | Low: values are milli-g from a +-2000 mg sensor | 5 |
| 9 | `motion_status_t` 160 -> 104 B (40 live + 64 diagnostic) plus a 4 B working pointer, as built | motion_types.h, motion_pack.c, motion_task.c, motion_task.h, vc_task.c, tests/motion | 52 | Medium: mutex copy, recorder reads new layout; measured speed, encoder counts, snapshot duration and two 64-bit timestamps keep more bits than the first estimate | 3 |
| 10 | `b4_types_status_t` 116 -> 68 B packed | b4_types.h, b4_proc.c, b4_task.c, vc_task.c | 48 | Medium: many fields, one writer | 5 |
| 11 | `vc_t` 88 -> 40 B packed, as built | vc.h, vc.c, vc_task.c, vc_pack.c, vc_pack.h, tests/vc | 48 | Medium: pure logic is host-tested; keep all values bit-exact | 5 |
| 12 | motion `command_queue` + `command_buffer` 112 -> 64 B | motion_task.c, motion_pack.c, motion_types.h, vc_task.c, tests/motion | 48 | Medium: `sequence_id` and `timeout_ms` widths change (16 bits, 10 ms units) | 3 |
| 13 | obstacle `path_result` 52 -> 12 B, as built | obstacle_types.h, obstacle_task.c, obstacle_pack.c, vc_task.c | 40 | Low: one seqlock copy | 5 |
| 14 | `comms_motion_t` 44 -> 24 B + slot | comms.h, comms.c, vc_task.c | 20 | Low: one writer, one seqlock | 2 |
| 15 | VC `g_plan` 32 -> 12 B, as built | vc_pack.c, vc_pack.h, vc_task.c | 20 | Low: VC-private | 5 |
| 16 | VC `g_profile` 24 -> 8 B (keep the 64-bit profile word), as built | vc_task.c | 16 | Low | 5 |
| 17 | `motion_encoder_snapshot_t` kept at 32 B: every copy is task-local/stack, never stored | motion_encoder.h, motion_encoder.c, motion_task.c | 0 | None in this phase: nothing is stored to pack | 3 (deferred) |
| 18 | VC `g_pending` 20 -> 8 B | vc_task.c, motion_pack.c | 12 | Low: same command layout | 3 |
| 19 | comms line slot 16 -> 8 B | comms.c | 8 | Low | 2 |
| 20 | comms terrain slot 16 -> 8 B | comms.c | 8 | Low | 2 |
| 21 | `line_cal_result_t` 24 -> 20 B: **no win, kept at 24 B** (130 field bits do not fit 5 words; the struct already has only 2 pad bytes) | line_follow.h, line_follow.c | 0 | None: representation unchanged | 4 (dropped) |
| 22 | fault mask: fix docs to 10 bits, reserve bit 4 | comms.h, vc_task.h, docs/BINARY_FORMAT.md | 0 | None (documentation) | 2 |
| 23 | telemetry widths: encoder 19 -> 22 s, turn 11 -> 10 s, add line ADC + calibration to BINARY_FORMAT.md | comms.c, docs/BINARY_FORMAT.md, tools/robot_decode.ps1 | 0 | Low: decoder must match | 2 |

| Phase | Bytes |
|---|---:|
| 2 (comms + fault mask) | 804 |
| 3 (motion) | 112 (52 status + 48 command queue + 12 `g_pending`; snapshot kept) |
| 4 (line + barcode) | 1964 (960 `raw_encoder` + 256 `cal_history` + 256 `barcode_edges` + 240 `raw_adc` + 252 widths; `line_cal_result_t` kept) |
| 5 (Buddy 4 + obstacle + VC) | 604 |
| **Total** | **3484 (about 3.4 KiB of static RAM)** |

The Phase 3 car image moved from 83388 B to 83276 B of `.bss` (112 B), with
`.text` growing by 2960 B for the pack/unpack helpers.  The Phase 4 car image
moved from 83276 B to 81324 B of `.bss` (1952 B; the packed symbols account for
1964 B saved minus the new 4 B `raw_encoder_base`, the remaining 8 B being the
alignment of the neighbouring statics), with `.text` growing by 652 B for the
pack/unpack helpers.

The Phase 5a car image (obstacle only) moved from 81324 B to 81048 B of `.bss`
(276 B: rows 6 and 13 plus the 8 B path-check request word, minus 4 B of
alignment), with `.text` growing by 916 B for `obstacle_pack.c`.  The scan
working copy (300 B, `obstacle_scan_result_t`) that the analysis needs now
lives in the obstacle task's frame instead of in `.bss`, so the 1 KiB task
stack is the place to watch; the stored copy is the 68 B packed record.

The Phase 5b car image (Buddy 4 status and incline window) moved from
81048 B to 80808 B of `.bss` (240 B: rows 8 and 10 give 248 B, the remaining
8 B being alignment of the neighbouring statics), with `.text` growing by
1316 B for `b4_pack.c` and the unpack/pack calls in `b4_proc.c`.  The
`g_b4_proc_state` symbol is 704 B (was 952 B).  Widening every cross-call
field to its reachable range (section 4.1) kept the stored record at 17
words: the six spare bits of W14 hold the two high bits of
`active_hump_height_um`.  The full-width status working copy (116 B) now
lives in the frame of the `b4_proc` call that updates it, which raises the
measured `b4_proc_process` frame from 148 B to 264 B (`b4_pack_status` and
`b4_unpack_status` add 28 B each); the Buddy 4 task stack is 2048 B.

The Phase 5c car image (VC state, bypass plan and obstacle profile) moved
from 80808 B to 80724 B of `.bss` (84 B: row 11 gives 48 B, rows 15 and 16
20 and 16 B), with `.text` growing by 1580 B for `vc_pack.c` and the
unpack/pack calls in `vc_task.c`.  The symbols are `g_vc` 40 B (was 88 B),
`g_plan` 12 B (was 32 B) and `g_profile_word` 8 B (was 24 B).  Budget: the
`vc_t` profile is 284 field bits, which is why the ten words keep 36 spare
bits - `search_origin_mdeg` needed 32 s (section 5.1).  The stack cost is
+128 B on the VC task's own frame: `vc_task_entry` measures 904 B with
`-fstack-usage` (was 776 B); it holds the 88 B `vc_t` working copy, and the
inlined bypass sequence keeps the unpacked 32 B plan and the 24 B profile
in the same frame.  `vc_step` itself is unchanged at 56 B (vc.c is not
modified), so the 4096 B VC task stack has about 3.1 kB of headroom.

Not counted: stack (task work buffers, `vc_inputs_t`/`vc_outputs_t` packing), the
832 B debug text buffer (temporary), and the 80 B `Buddy 3` / 16 B `Buddy 4`
message buffers (already packed).

---

## 12. Excluded

| Item | Why excluded |
|---|---|
| Kernel objects and control blocks: `T_CMBF`, `T_CMTX`, `T_CFLG`, `T_CTSK`, `T_CSEM`, `ID`s | Owned and laid out by uT-Kernel; not application data |
| Task stacks: `motion_stack` 4096 B, `vc_task.c:g_stack` 4096 B, Buddy 3 `g_buddy3_stack` 4096 B, `g_b4_task_stack` 2048 B, `obstacle_task.c:task_stack` 1024 B | Kernel-scheduled stack, sized for the SMP kernel path; not a data field |
| `app_main.c` and its demo objects (`mpf_buf`, `mbf_buf`, `RECORD`) | Kernel demo application, not part of the car build's application data |
| Test-only mains: `*_test_main.c` (motion, servo, ultrasonic, obstacle, b4), `motion_stage*.c/_test_main.c`, `demo_tasks.c`, `comms_demo.c` | Bench/test images only |
| Recorder-only code: `run_rec.c`/`.h` and every block compiled only with `VC_RECORDER=1` or `B4_RECORDER=1` | Evidence path, not the production image |
| Bench/tooling structs: `obstacle_probe_t`, `obstacle_pin_test_t`, `obstacle_servo_cal_t` | Used only by `ultrasonic_test` / `servo_test` |
| SDK / lwIP / cyw43 / pico-sdk buffers and structures (`comms_link_msg_t` 260 B payload buffers, `pbuf` pools, `cyw43_utk`) | Third-party code, no application field ranges |
| Hardware registers, SIO, ADC FIFO, PWM slices | Memory-mapped hardware, fixed layout |
| `static const` tables: `c39_table`, `course_codes`, `sin_milli`, `g_*_names` (JSON builds), `line_state_name` | ROM (or dropped from binary builds), not RAM data fields |
| Configuration records: `motion_config_t` 64 B, `b4_cfg_config_t` 96 B, `motion_pid_t`, float gains/slopes | Written once during init, read-only afterwards; a field range here is a tuning constant, not measured data |
| `motion_learn_t` 124 B (fits, sums, PWM history), `b4_proc` filter arrays other than the incline window | Full-width calculation history (rule 2); packing would cost precision and gain little |

---

## 13. Open questions (TBC ranges for the team)

1. **Encoder counts since boot**: what is the longest run in counts, and is
   there a reset rule?  19 s (the current wire width) covers only +-18.7 m at
   14.0 counts/mm.  Proposal: 22 s (+-150 m) on the wire, full 32 s in RAM
   (as built).
   (Buddy 2 / Buddy 1.)
2. **`dist_mm` 16 s** (+-32 767 mm) - longest course length and whether a run
   can exceed 32.7 m.  (Buddy 1 / VC.)
3. **`run_ms` 20 bits** (17.5 min) - longest mission, including bypass
   rescans.  (Buddy 1 / VC.)
4. **`turn_dps` 11 s**: is 10 s (+-511 deg/s) enough?  The car's maximum is
   about 289 deg/s at 300 mm/s on the 119 mm track.  (Buddy 4.)
5. **`width_mm` 10 bits**: widest obstacle the scan may report
   (`OBSTACLE_MAX_WIDTH_MM` is 200, the field saturates at 1023 mm).  (Buddy 5.)
6. **Motion command `timeout_ms`**: resolved in Phase 3.  The design uses
   14 bits in 10 ms units (163 830 ms); checking every VC producer gives
   0, 3000, 8000 and at most 31 586 ms for a bypass leg (2294 mm at 150 mm/s
   plus 1 s).  The Motion loop is 20 ms, so 10 ms rounding-up cannot change
   behaviour.
7. **Barcode raw capture limits**: maximum element width in counts (proposed
   16 bits), maximum capture duration (`raw_time_ms` 16 bits = 65.5 s, capture
   is about 2.5 s), and whether `raw_encoder` may store a delta instead of an
   absolute position.  (Buddy 3.)  Resolved in Phase 4: `raw_encoder` stores
   an int16 offset from a per-capture base (960 B).  The raw buffer is
   appended, never overwritten, and only resets with the capture, so the base
   stays valid for every stored sample; 4.8 s at 300 mm/s x 14 counts/mm is
   about 20 000 counts, inside int16 from the capture's first sample.  An
   offset that leaves int16 clamps and sets `raw_overflow`, so that capture is
   reported invalid exactly like an over-long one.
8. **`turn_cross` range** (counts^2, threshold 20 000) and whether the
   magnetometer is used in the car build at all.  (Buddy 4.)
9. **`learn_count` saturation value** (1 step per mm of driving) and the
   largest `detectable_height_um`.  (Buddy 4.)
10. **`sweep_deg` 8 bits** and `cal_out_mdeg`/`cal_far_mdeg` 18 bits: is
    65 000 mdeg (VC_CAL_LIMIT_MDEG 45 000 + BACK_MARGIN 20 000) the true
    bound?  (VC.)
11. **`travelled_mm`, `target_mm`, `junction_origin_mm`, `junction_seen_mm`**:
    same longest-distance question as Q2, at 24 bits in RAM.
12. **State codes**: which state set is published in `robot/state` and
    `robot/heartbeat` - `comms_state_t` (10 values) or `vc_state_t` (11)?
    Both fit 4 bits, but the decoder table must match.  (Buddy 1 / VC.)
13. **Fault bit 4**: reserve it, or assign a new fault (recommended: reserve,
    never renumber published bits)?  (VC / Buddy 1.)
14. **`line_cal_history` depth**: is 256 samples needed, or would 128 samples
    (512 B less) still cover the sweep?  It is a tunable, not a representation
    change, so it is not in the ranked list.  (Buddy 3.)
15. **Obstacle scan angle offset**: may the packed scan point store
    `angle_deg - 60` (6 bits) instead of the absolute 7-bit angle?  Only valid
    while the servo range stays 60..120 deg.  (Buddy 5.)
16. **Debug text buffer** (832 B, `COMMS_DEBUG_TEXT`): keep for the assessment
    or remove before the final RAM figure?  (Buddy 1.)
