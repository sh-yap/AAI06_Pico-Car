/** @file b4_pack.h
 *
 * @brief Packed stored copy of the Buddy 4 status snapshot (CR1).
 *
 * Only the copy that the processor stores in static RAM is packed: see
 * docs/DATA_REPRESENTATION.md sections 4.1 and 11 row 10.  The value type
 * b4_types_status_t (116 B) stays full width for callers and for the
 * calculations; b4_proc unpacks the stored record into a full-width working
 * copy for the call, updates it, and packs it back before returning, and
 * b4_proc_get_status() unpacks it for its caller.
 *
 * Rules (docs/DATA_REPRESENTATION.md section 0): unsigned words, explicit
 * masks and shifts, no C bit-fields, signed fields two's complement and
 * sign-extended, a field never straddles a word boundary, measured values
 * saturate at their field ends, and an enum code that is not a value the
 * code produces is read back as the enum's no-data member rather than
 * guessed.
 *
 * Status record: B4_STATUS_PACKED_WORDS = 17 words (68 B).
 *
 * Widths are the full reachable range from code (rule: never clamp a
 * reachable value).  Signed fields that carry filter state across calls or
 * feed a decision use 13 s (+-4095) for values bounded by the +-4000 mg
 * sample validity check, and 14 s (+-8191) for a difference of two such
 * values.  Fields that are publish-only keep the width of their documented
 * range; the comments below name their readers.
 *
 *   W0      bits  0..1   health                      2 U
 *           bits  2..5   fault                       4 U (0..8)
 *           bits  6..8   tilt                        3 U (0..6)
 *           bits  9..11  motion                      3 U (0..4)
 *           bits 12..14  hump_state                  3 U (0..5)
 *           bits 15..16  turn                        2 U (0..3)
 *           bit  17      b_hump_height_valid
 *           bit  18      b_mag_valid
 *           bit  19      b_sample_stale
 *           bit  20      b_sharp_change_event
 *           bit  21      b_lift_cue
 *           bit  22      b_hump_started_event
 *           bit  23      b_hump_completed_event
 *           bit  24      b_jolt_window_active
 *           bits 25..31  reserved, 0
 *   W1      bits  0..31  sample.timestamp_ms      32 U, wraps
 *   W2      bits  0..12  sample.accel_x_mg        13 s (+-4000 mg)
 *           bits 13..25  sample.accel_y_mg        13 s
 *           bit  26      sample.b_valid
 *           bits 27..31  reserved, 0
 *   W3      bits  0..12  sample.accel_z_mg        13 s
 *           bits 13..31  reserved, 0
 *   W4      bits  0..31  mag_sample.timestamp_ms  32 U
 *   W5      bits  0..12  mag_sample.mag_x         13 s (B4_IMU_MAG_MAX_COUNTS)
 *           bits 13..25  mag_sample.mag_y         13 s
 *           bits 26..31  reserved, 0
 *   W6      bits  0..12  mag_sample.mag_z         13 s
 *           bit  13      mag_sample.b_valid
 *           bits 14..31  reserved, 0
 *   W7      bits  0..12  filtered_x_mg            13 s (+-4000 mg)
 *           bits 13..25  filtered_y_mg            13 s
 *           bits 26..31  reserved, 0
 *   W8      bits  0..12  filtered_z_mg            13 s
 *           bits 13..31  reserved, 0
 *   W9      bits  0..13  vertical_deviation_mg     14 s (+-2000 mg gravity
 *                                                    reference: -6000..3999)
 *           bits 14..26  roll_mg                  13 s
 *           bits 27..31  reserved, 0
 *   W10     bits  0..12  roll_baseline_mg         13 s
 *           bits 13..26  incline_delta_mg         14 s (difference of two
 *                                                    +-4000 mg means)
 *           bits 27..31  reserved, 0
 *   W11     bits  0..24  turn_cross               25 s, publish-only
 *                                                    (read at
 *                                                    tests/buddy4/b4_car_test.c)
 *           bits 25..31  reserved, 0
 *   W12     bits  0..10  drive_speed_mm_s         11 U (0..1023 mm/s)
 *           bits 11..19  noise_mg                 9 U, publish-only
 *                                                    (b4_task.c)
 *           bits 20..31  learn_count              12 U (low part)
 *   W13     bits  0..11  learn_count              24 U in total, saturates
 *           bits 12..31  detectable_height_um     20 U, publish-only
 *                                                    (b4_task.c)
 *   W14     bits  0..12  current_hump_peak_mg     13 U (0..8000 mg)
 *           bits 13..25  highest_hump_peak_mg     13 U
 *           bits 26..27  active_hump_height_um    2 U (high part)
 *           bits 28..31  reserved, 0
 *   W15     bits  0..16  hump_peak_height_um      17 U, publish-only
 *                                                    (b4_task.c)
 *           bits 17..31  active_hump_height_um    15 U (low part)
 *   W16     bits  0..31  highest_hump_height_um   32 U: the run maximum is
 *                                                    read back as a
 *                                                    comparison bound
 *                                                    (b4_proc.c), and one
 *                                                    config-legal lever and
 *                                                    offset reaches
 *                                                    300000 * 8000 / 1 um
 *
 * The magnetometer Z field is the one place where the first proposal split a
 * field over two words (W5 bits 26..31 plus W6 bits 0..6).  That breaks the
 * section 0 rule, so the as-built record keeps mag_x and mag_y together in
 * W5 and puts mag_z with its valid flag in W6.
 *
 * Widened in the Phase 5b review: accel, filtered, deviation, roll and
 * baseline, the two hump peaks, and the run-maximum height.  The other
 * height fields stay 17 U because nothing reads them back (publish-only),
 * and the record still fits 17 words because W14 had six spare bits for the
 * two high bits of active_hump_height_um.
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h>, <stdbool.h> and
 * "b4_types.h" before this header.
 */

#ifndef B4_PACK_H
#define B4_PACK_H

/* Words in the stored status record. */
#define B4_STATUS_PACKED_WORDS  (17U)

/* The stored status record: b4_proc's static copy of b4_types_status_t. */
typedef struct
{
    uint32_t word[B4_STATUS_PACKED_WORDS];
} b4_status_packed_t;

/* Copy a status snapshot into its stored form.  Measured values outside a
 * field saturate at the field's end; an enum code that is not a value of its
 * enum is stored as the no-data member (code 0). */
void
b4_pack_status (b4_types_status_t const * p_status,
                b4_status_packed_t *      p_packed);

/* Decode a stored status record.  false when an enum code is not a value of
 * its enum; the affected fields then hold the enum's no-data member, so a
 * damaged record never presents a guessed state. */
bool
b4_unpack_status (b4_status_packed_t const * p_packed,
                  b4_types_status_t *        p_status);

/* Clamp a milli-g value into the int16_t range of a stored incline window
 * entry.  The processor's filtered incline is inside the validated sample
 * range, so the clamp only guards the narrower store. */
int16_t
b4_pack_mg_i16 (int32_t value_mg);

#endif /* B4_PACK_H */
