/** @file b4_pack.c
 *
 * @brief Packed stored copy of the Buddy 4 status snapshot (CR1).
 *
 * The bit layout is documented in b4_pack.h.  Only the stored copy is
 * packed; the value type and the processor's working copy stay full width.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "b4_types.h"
#include "b4_pack.h"

/* Enum codes that the code can produce (b4_types.h). */
#define ST_HEALTH_COUNT  (4U)   /* NOT_INITIALIZED, READY, DEGRADED, FAULT. */
#define ST_FAULT_COUNT   (9U)   /* NONE .. MAGNETOMETER.                   */
#define ST_TILT_COUNT    (7U)   /* UNKNOWN .. COMPOUND.                    */
#define ST_MOTION_COUNT  (5U)   /* QUIET .. LIFTED.                        */
#define ST_HUMP_COUNT    (6U)   /* NORMAL .. COMPLETE.                     */
#define ST_TURN_COUNT    (4U)   /* UNKNOWN .. RIGHT.                       */

/* Status words (b4_pack.h). */
#define ST_W_STATE       (0U)
#define ST_W_SAMPLE_TIME (1U)
#define ST_W_SAMPLE_XY   (2U)
#define ST_W_SAMPLE_Z    (3U)
#define ST_W_MAG_TIME    (4U)
#define ST_W_MAG_XY      (5U)
#define ST_W_MAG_Z       (6U)
#define ST_W_FILTER_XY   (7U)
#define ST_W_FILTER_Z    (8U)
#define ST_W_DEVIATION   (9U)
#define ST_W_BASELINE    (10U)
#define ST_W_TURN_CROSS  (11U)
#define ST_W_NOISE       (12U)
#define ST_W_DETECTABLE  (13U)
#define ST_W_PEAKS       (14U)
#define ST_W_HEIGHT_PEAK (15U)
#define ST_W_HIGHEST_HEIGHT (16U)

/* W0: state codes, enum codes and event flags. */
#define ST_HEALTH_SHIFT  (0U)
#define ST_HEALTH_BITS   (2U)
#define ST_FAULT_SHIFT   (2U)
#define ST_FAULT_BITS    (4U)
#define ST_TILT_SHIFT    (6U)
#define ST_TILT_BITS     (3U)
#define ST_MOTION_SHIFT  (9U)
#define ST_MOTION_BITS   (3U)
#define ST_HUMP_SHIFT    (12U)
#define ST_HUMP_BITS     (3U)
#define ST_TURN_SHIFT    (15U)
#define ST_TURN_BITS     (2U)
#define ST_FLAG_SHIFT    (17U)
#define ST_FLAG_HEIGHT   (0U)
#define ST_FLAG_MAG      (1U)
#define ST_FLAG_STALE    (2U)
#define ST_FLAG_SHARP    (3U)
#define ST_FLAG_CUE      (4U)
#define ST_FLAG_STARTED  (5U)
#define ST_FLAG_DONE     (6U)
#define ST_FLAG_JOLT     (7U)

/* W2: the calibrated sample.  The validity check accepts +-4000 mg
 * (b4_proc.c B4_PROC_MAX_SAMPLE_MG), so a 13 s field is needed. */
#define ST_SAMPLE_X_SHIFT (0U)
#define ST_SAMPLE_Y_SHIFT (13U)
#define ST_SAMPLE_XY_BITS (13U)
#define ST_SAMPLE_VALID_SHIFT (26U)
#define ST_SAMPLE_VALID_BITS  (1U)
#define ST_SAMPLE_Z_SHIFT (0U)
#define ST_SAMPLE_Z_BITS  (13U)

/* W5, W6: the magnetometer sample. */
#define ST_MAG_X_SHIFT    (0U)
#define ST_MAG_Y_SHIFT    (13U)
#define ST_MAG_XY_BITS    (13U)
#define ST_MAG_Z_SHIFT    (0U)
#define ST_MAG_Z_BITS     (13U)
#define ST_MAG_VALID_SHIFT (13U)
#define ST_MAG_VALID_BITS  (1U)

/* W7, W8: the filtered axes, bounded by the validated sample range. */
#define ST_FILTER_X_SHIFT (0U)
#define ST_FILTER_Y_SHIFT (13U)
#define ST_FILTER_XY_BITS (13U)
#define ST_FILTER_Z_SHIFT (0U)
#define ST_FILTER_Z_BITS  (13U)

/* W9, W10: deviation, incline and baseline.  The deviation subtracts the
 * config's gravity reference (1..2000 mg, b4_cfg.c) from a filtered axis,
 * so it reaches -6000..3999 and needs 14 s. */
#define ST_DEVIATION_SHIFT (0U)
#define ST_ROLL_SHIFT     (14U)
#define ST_DEVIATION_BITS (14U)
#define ST_ROLL_BITS      (13U)
#define ST_BASELINE_SHIFT (0U)
#define ST_BASELINE_BITS  (13U)
#define ST_DELTA_SHIFT    (13U)
#define ST_DELTA_BITS     (14U)

/* W11: turn cross product. */
#define ST_TURN_CROSS_SHIFT (0U)
#define ST_TURN_CROSS_BITS  (25U)

/* W12, W13: speed, noise, learning count and detectable height. */
#define ST_SPEED_SHIFT    (0U)
#define ST_SPEED_BITS     (11U)
#define ST_NOISE_SHIFT    (11U)
#define ST_NOISE_BITS     (9U)
#define ST_LEARN_LO_SHIFT (20U)
#define ST_LEARN_LO_BITS  (12U)
#define ST_LEARN_HI_SHIFT (0U)
#define ST_LEARN_HI_BITS  (12U)
#define ST_LEARN_BITS     (24U)
#define ST_DETECTABLE_SHIFT (12U)
#define ST_DETECTABLE_BITS  (20U)

/* W14, W15, W16: hump peaks and hump heights.  The peaks hold an
 * amplitude, the difference of two +-4000 mg means, so they reach 8000 mg
 * and need 13 U.  The run-maximum height is read back as a comparison
 * bound, and a config-legal lever and offset reach 300000 * 8000 / 1 um, so
 * it keeps a full word. */
#define ST_PEAK_SHIFT     (0U)
#define ST_HIGHEST_PEAK_SHIFT (13U)
#define ST_PEAK_BITS      (13U)
#define ST_HEIGHT_SHIFT   (0U)
#define ST_HEIGHT_BITS    (17U)
#define ST_ACTIVE_LO_SHIFT (17U)
#define ST_ACTIVE_LO_BITS  (15U)
#define ST_ACTIVE_HI_SHIFT (26U)
#define ST_ACTIVE_HI_BITS  (2U)
#define ST_ACTIVE_BITS    (17U)

/* Field ends of the signed fields (b4_pack.h). */
#define ST_ACCEL_MIN      (-4096)
#define ST_ACCEL_MAX      (4095)
#define ST_MAG_MIN        (-4096)
#define ST_MAG_MAX        (4095)
#define ST_DELTA_MIN      (-8192)
#define ST_DELTA_MAX      (8191)
#define ST_DEVIATION_MIN  (-8192)
#define ST_DEVIATION_MAX  (8191)
#define ST_CROSS_MIN      (-16777216)
#define ST_CROSS_MAX      (16777215)


/*!
 * @brief Mask for a field of 1..31 bits.
 *
 * @param[in] bits Field width.
 *
 * @return The mask, bit 0 = LSB.
 */
static uint32_t
pack_mask (uint32_t bits)
{
    return ((32U <= bits) ? 0xFFFFFFFFU : ((1U << bits) - 1U));
}

/*!
 * @brief A value placed in its field, saturated at the field's end.
 *
 * @param[in] value Field value.
 * @param[in] bits  Field width.
 *
 * @return value, or the field's largest value if value is too big.
 */
static uint32_t
pack_field (uint32_t value, uint32_t bits)
{
    uint32_t mask = pack_mask(bits);

    return ((value > mask) ? mask : value);
}

/*!
 * @brief A signed value in its field, saturated and two's complement.
 *
 * @param[in] value   Field value.
 * @param[in] minimum Smallest value the field may hold.
 * @param[in] maximum Largest value the field may hold.
 * @param[in] bits    Field width.
 *
 * @return The field bits.
 */
static uint32_t
pack_signed (int32_t value, int32_t minimum, int32_t maximum, uint32_t bits)
{
    int32_t clamped = value;

    if (clamped < minimum)
    {
        clamped = minimum;
    }
    else if (clamped > maximum)
    {
        clamped = maximum;
    }
    else
    {
        /* Already inside the field. */
    }

    return ((uint32_t)clamped & pack_mask(bits));
}

/*!
 * @brief An unsigned field from a signed value, 0 when it is negative.
 *
 * @param[in] value Field value.
 * @param[in] bits  Field width.
 *
 * @return The field bits.
 */
static uint32_t
pack_non_negative (int32_t value, uint32_t bits)
{
    uint32_t field = 0U;

    if (value > 0)
    {
        field = pack_field((uint32_t)value, bits);
    }

    return (field);
}

/*!
 * @brief An enum placed in its field: 0 (the no-data code) if it is not one.
 *
 * @param[in] value Enum value.
 * @param[in] count Number of codes in the enum.
 *
 * @return value, or 0 if value is not a code of that enum.
 */
static uint32_t
pack_enum (uint32_t value, uint32_t count)
{
    return ((value < count) ? value : 0U);
}

/*!
 * @brief One flag bit.
 *
 * @param[in] b_value The flag.
 *
 * @return 1 when set, 0 when clear.
 */
static uint32_t
pack_flag (bool b_value)
{
    return ((true == b_value) ? 1U : 0U);
}

/*!
 * @brief Read a field back out of a word.
 *
 * @param[in] word  Packed word.
 * @param[in] shift Position of the field's LSB.
 * @param[in] bits  Field width.
 *
 * @return The field, unsigned.
 */
static uint32_t
pack_get (uint32_t word, uint32_t shift, uint32_t bits)
{
    return ((word >> shift) & pack_mask(bits));
}

/*!
 * @brief Read a signed field back out of a word, sign-extended.
 *
 * @param[in] word  Packed word.
 * @param[in] shift Position of the field's LSB.
 * @param[in] bits  Field width.
 *
 * @return The field, sign-extended to int32_t.
 */
static int32_t
pack_signed_get (uint32_t word, uint32_t shift, uint32_t bits)
{
    uint32_t mask = pack_mask(bits);
    uint32_t value = (word >> shift) & mask;
    uint32_t sign = (1U << (bits - 1U));

    if (0U != (value & sign))
    {
        return (-(int32_t)((~value & mask) + 1U));
    }

    return ((int32_t)value);
}

/*!
 * @brief Copy a status snapshot into its stored form.
 *
 * @param[in]  p_status Status snapshot (or NULL).
 * @param[out] p_packed Stored record (or NULL).
 */
void
b4_pack_status (b4_types_status_t const * p_status,
                b4_status_packed_t *      p_packed)
{
    uint32_t flags;
    uint32_t learn_count;
    uint32_t active_height;

    if ((NULL != p_status) && (NULL != p_packed))
    {
        flags = (pack_flag(p_status->b_hump_height_valid) << ST_FLAG_HEIGHT)
                | (pack_flag(p_status->b_mag_valid) << ST_FLAG_MAG)
                | (pack_flag(p_status->b_sample_stale) << ST_FLAG_STALE)
                | (pack_flag(p_status->b_sharp_change_event) << ST_FLAG_SHARP)
                | (pack_flag(p_status->b_lift_cue) << ST_FLAG_CUE)
                | (pack_flag(p_status->b_hump_started_event) << ST_FLAG_STARTED)
                | (pack_flag(p_status->b_hump_completed_event) << ST_FLAG_DONE)
                | (pack_flag(p_status->b_jolt_window_active) << ST_FLAG_JOLT);

        p_packed->word[ST_W_STATE] =
            (pack_enum((uint32_t)p_status->health, ST_HEALTH_COUNT)
             << ST_HEALTH_SHIFT)
            | (pack_enum((uint32_t)p_status->fault, ST_FAULT_COUNT)
               << ST_FAULT_SHIFT)
            | (pack_enum((uint32_t)p_status->tilt, ST_TILT_COUNT)
               << ST_TILT_SHIFT)
            | (pack_enum((uint32_t)p_status->motion, ST_MOTION_COUNT)
               << ST_MOTION_SHIFT)
            | (pack_enum((uint32_t)p_status->hump_state, ST_HUMP_COUNT)
               << ST_HUMP_SHIFT)
            | (pack_enum((uint32_t)p_status->turn, ST_TURN_COUNT)
               << ST_TURN_SHIFT)
            | (flags << ST_FLAG_SHIFT);

        p_packed->word[ST_W_SAMPLE_TIME] = p_status->sample.timestamp_ms;
        p_packed->word[ST_W_SAMPLE_XY] =
            (pack_signed(p_status->sample.accel_x_mg, ST_ACCEL_MIN,
                         ST_ACCEL_MAX, ST_SAMPLE_XY_BITS)
             << ST_SAMPLE_X_SHIFT)
            | (pack_signed(p_status->sample.accel_y_mg, ST_ACCEL_MIN,
                           ST_ACCEL_MAX, ST_SAMPLE_XY_BITS)
               << ST_SAMPLE_Y_SHIFT)
            | (pack_flag(p_status->sample.b_valid) << ST_SAMPLE_VALID_SHIFT);
        p_packed->word[ST_W_SAMPLE_Z] =
            (pack_signed(p_status->sample.accel_z_mg, ST_ACCEL_MIN,
                         ST_ACCEL_MAX, ST_SAMPLE_Z_BITS)
             << ST_SAMPLE_Z_SHIFT);

        p_packed->word[ST_W_MAG_TIME] = p_status->mag_sample.timestamp_ms;
        p_packed->word[ST_W_MAG_XY] =
            (pack_signed((int32_t)p_status->mag_sample.mag_x, ST_MAG_MIN,
                         ST_MAG_MAX, ST_MAG_XY_BITS)
             << ST_MAG_X_SHIFT)
            | (pack_signed((int32_t)p_status->mag_sample.mag_y, ST_MAG_MIN,
                           ST_MAG_MAX, ST_MAG_XY_BITS)
               << ST_MAG_Y_SHIFT);
        p_packed->word[ST_W_MAG_Z] =
            (pack_signed((int32_t)p_status->mag_sample.mag_z, ST_MAG_MIN,
                         ST_MAG_MAX, ST_MAG_Z_BITS)
             << ST_MAG_Z_SHIFT)
            | (pack_flag(p_status->mag_sample.b_valid) << ST_MAG_VALID_SHIFT);

        p_packed->word[ST_W_FILTER_XY] =
            (pack_signed(p_status->filtered_x_mg, ST_ACCEL_MIN, ST_ACCEL_MAX,
                         ST_FILTER_XY_BITS)
             << ST_FILTER_X_SHIFT)
            | (pack_signed(p_status->filtered_y_mg, ST_ACCEL_MIN, ST_ACCEL_MAX,
                           ST_FILTER_XY_BITS)
               << ST_FILTER_Y_SHIFT);
        p_packed->word[ST_W_FILTER_Z] =
            (pack_signed(p_status->filtered_z_mg, ST_ACCEL_MIN, ST_ACCEL_MAX,
                         ST_FILTER_Z_BITS)
             << ST_FILTER_Z_SHIFT);

        p_packed->word[ST_W_DEVIATION] =
            (pack_signed(p_status->vertical_deviation_mg, ST_DEVIATION_MIN,
                         ST_DEVIATION_MAX, ST_DEVIATION_BITS)
             << ST_DEVIATION_SHIFT)
            | (pack_signed(p_status->roll_mg, ST_ACCEL_MIN, ST_ACCEL_MAX,
                           ST_ROLL_BITS)
               << ST_ROLL_SHIFT);
        p_packed->word[ST_W_BASELINE] =
            (pack_signed(p_status->roll_baseline_mg, ST_ACCEL_MIN,
                         ST_ACCEL_MAX, ST_BASELINE_BITS)
             << ST_BASELINE_SHIFT)
            | (pack_signed(p_status->incline_delta_mg, ST_DELTA_MIN,
                           ST_DELTA_MAX, ST_DELTA_BITS)
               << ST_DELTA_SHIFT);
        p_packed->word[ST_W_TURN_CROSS] =
            (pack_signed(p_status->turn_cross, ST_CROSS_MIN, ST_CROSS_MAX,
                         ST_TURN_CROSS_BITS)
             << ST_TURN_CROSS_SHIFT);

        learn_count = pack_field(p_status->learn_count, ST_LEARN_BITS);
        p_packed->word[ST_W_NOISE] =
            (pack_non_negative(p_status->drive_speed_mm_s, ST_SPEED_BITS)
             << ST_SPEED_SHIFT)
            | (pack_field(p_status->noise_mg, ST_NOISE_BITS)
               << ST_NOISE_SHIFT)
            | (pack_get(learn_count, 0U, ST_LEARN_LO_BITS)
               << ST_LEARN_LO_SHIFT);
        p_packed->word[ST_W_DETECTABLE] =
            (pack_get(learn_count, ST_LEARN_LO_BITS, ST_LEARN_HI_BITS)
             << ST_LEARN_HI_SHIFT)
            | (pack_field(p_status->detectable_height_um, ST_DETECTABLE_BITS)
               << ST_DETECTABLE_SHIFT);

        active_height = pack_field(p_status->active_hump_height_um,
                                   ST_ACTIVE_BITS);
        p_packed->word[ST_W_PEAKS] =
            (pack_field(p_status->current_hump_peak_mg, ST_PEAK_BITS)
             << ST_PEAK_SHIFT)
            | (pack_field(p_status->highest_hump_peak_mg, ST_PEAK_BITS)
               << ST_HIGHEST_PEAK_SHIFT)
            | (pack_get(active_height, ST_ACTIVE_LO_BITS, ST_ACTIVE_HI_BITS)
               << ST_ACTIVE_HI_SHIFT);
        p_packed->word[ST_W_HEIGHT_PEAK] =
            (pack_field(p_status->hump_peak_height_um, ST_HEIGHT_BITS)
             << ST_HEIGHT_SHIFT)
            | (pack_get(active_height, 0U, ST_ACTIVE_LO_BITS)
               << ST_ACTIVE_LO_SHIFT);
        /* The run maximum is read back as a comparison bound, so it keeps
         * its full 32-bit range and needs no mask. */
        p_packed->word[ST_W_HIGHEST_HEIGHT] =
            p_status->highest_hump_height_um;
    }
}

/*!
 * @brief Decode a stored status record.
 *
 * @param[in]  p_packed Stored record (or NULL).
 * @param[out] p_status Destination snapshot (or NULL).
 *
 * @return false when an enum code is not a value of its enum.
 */
bool
b4_unpack_status (b4_status_packed_t const * p_packed,
                  b4_types_status_t *        p_status)
{
    uint32_t word;
    uint32_t health;
    uint32_t fault;
    uint32_t tilt;
    uint32_t motion;
    uint32_t hump_state;
    uint32_t turn;
    uint32_t learn_count;
    uint32_t active_height;
    bool b_enums_valid;

    if ((NULL == p_packed) || (NULL == p_status))
    {
        return (false);
    }

    word = p_packed->word[ST_W_STATE];
    health = pack_get(word, ST_HEALTH_SHIFT, ST_HEALTH_BITS);
    fault = pack_get(word, ST_FAULT_SHIFT, ST_FAULT_BITS);
    tilt = pack_get(word, ST_TILT_SHIFT, ST_TILT_BITS);
    motion = pack_get(word, ST_MOTION_SHIFT, ST_MOTION_BITS);
    hump_state = pack_get(word, ST_HUMP_SHIFT, ST_HUMP_BITS);
    turn = pack_get(word, ST_TURN_SHIFT, ST_TURN_BITS);

    b_enums_valid = (health < ST_HEALTH_COUNT) && (fault < ST_FAULT_COUNT) &&
                    (tilt < ST_TILT_COUNT) && (motion < ST_MOTION_COUNT) &&
                    (hump_state < ST_HUMP_COUNT) && (turn < ST_TURN_COUNT);

    p_status->health = b_enums_valid ? (b4_types_health_t)health
                                     : B4_HEALTH_NOT_INITIALIZED;
    p_status->fault = b_enums_valid ? (b4_types_fault_t)fault
                                    : B4_FAULT_NONE;
    p_status->tilt = b_enums_valid ? (b4_types_tilt_t)tilt : B4_TILT_UNKNOWN;
    p_status->motion = b_enums_valid ? (b4_types_motion_t)motion
                                     : B4_MOTION_QUIET;
    p_status->hump_state = b_enums_valid ? (b4_types_hump_state_t)hump_state
                                         : B4_HUMP_NORMAL;
    p_status->turn = b_enums_valid ? (b4_types_turn_t)turn : B4_TURN_UNKNOWN;

    p_status->b_hump_height_valid =
        (0U != pack_get(word, ST_FLAG_SHIFT + ST_FLAG_HEIGHT, 1U));
    p_status->b_mag_valid =
        (0U != pack_get(word, ST_FLAG_SHIFT + ST_FLAG_MAG, 1U));
    p_status->b_sample_stale =
        (0U != pack_get(word, ST_FLAG_SHIFT + ST_FLAG_STALE, 1U));
    p_status->b_sharp_change_event =
        (0U != pack_get(word, ST_FLAG_SHIFT + ST_FLAG_SHARP, 1U));
    p_status->b_lift_cue =
        (0U != pack_get(word, ST_FLAG_SHIFT + ST_FLAG_CUE, 1U));
    p_status->b_hump_started_event =
        (0U != pack_get(word, ST_FLAG_SHIFT + ST_FLAG_STARTED, 1U));
    p_status->b_hump_completed_event =
        (0U != pack_get(word, ST_FLAG_SHIFT + ST_FLAG_DONE, 1U));
    p_status->b_jolt_window_active =
        (0U != pack_get(word, ST_FLAG_SHIFT + ST_FLAG_JOLT, 1U));

    p_status->sample.timestamp_ms = p_packed->word[ST_W_SAMPLE_TIME];
    word = p_packed->word[ST_W_SAMPLE_XY];
    p_status->sample.accel_x_mg = pack_signed_get(word, ST_SAMPLE_X_SHIFT,
                                                  ST_SAMPLE_XY_BITS);
    p_status->sample.accel_y_mg = pack_signed_get(word, ST_SAMPLE_Y_SHIFT,
                                                  ST_SAMPLE_XY_BITS);
    p_status->sample.b_valid =
        (0U != pack_get(word, ST_SAMPLE_VALID_SHIFT, ST_SAMPLE_VALID_BITS));
    p_status->sample.accel_z_mg =
        pack_signed_get(p_packed->word[ST_W_SAMPLE_Z], ST_SAMPLE_Z_SHIFT,
                        ST_SAMPLE_Z_BITS);

    p_status->mag_sample.timestamp_ms = p_packed->word[ST_W_MAG_TIME];
    word = p_packed->word[ST_W_MAG_XY];
    p_status->mag_sample.mag_x = (int16_t)pack_signed_get(word, ST_MAG_X_SHIFT,
                                                          ST_MAG_XY_BITS);
    p_status->mag_sample.mag_y = (int16_t)pack_signed_get(word, ST_MAG_Y_SHIFT,
                                                          ST_MAG_XY_BITS);
    word = p_packed->word[ST_W_MAG_Z];
    p_status->mag_sample.mag_z = (int16_t)pack_signed_get(word, ST_MAG_Z_SHIFT,
                                                          ST_MAG_Z_BITS);
    p_status->mag_sample.b_valid =
        (0U != pack_get(word, ST_MAG_VALID_SHIFT, ST_MAG_VALID_BITS));

    word = p_packed->word[ST_W_FILTER_XY];
    p_status->filtered_x_mg = pack_signed_get(word, ST_FILTER_X_SHIFT,
                                              ST_FILTER_XY_BITS);
    p_status->filtered_y_mg = pack_signed_get(word, ST_FILTER_Y_SHIFT,
                                              ST_FILTER_XY_BITS);
    p_status->filtered_z_mg =
        pack_signed_get(p_packed->word[ST_W_FILTER_Z], ST_FILTER_Z_SHIFT,
                        ST_FILTER_Z_BITS);

    word = p_packed->word[ST_W_DEVIATION];
    p_status->vertical_deviation_mg =
        pack_signed_get(word, ST_DEVIATION_SHIFT, ST_DEVIATION_BITS);
    p_status->roll_mg = pack_signed_get(word, ST_ROLL_SHIFT, ST_ROLL_BITS);

    word = p_packed->word[ST_W_BASELINE];
    p_status->roll_baseline_mg = pack_signed_get(word, ST_BASELINE_SHIFT,
                                                 ST_BASELINE_BITS);
    p_status->incline_delta_mg = pack_signed_get(word, ST_DELTA_SHIFT,
                                                 ST_DELTA_BITS);

    p_status->turn_cross =
        pack_signed_get(p_packed->word[ST_W_TURN_CROSS], ST_TURN_CROSS_SHIFT,
                        ST_TURN_CROSS_BITS);

    word = p_packed->word[ST_W_NOISE];
    p_status->drive_speed_mm_s = (int32_t)pack_get(word, ST_SPEED_SHIFT,
                                                   ST_SPEED_BITS);
    p_status->noise_mg = pack_get(word, ST_NOISE_SHIFT, ST_NOISE_BITS);
    learn_count = pack_get(word, ST_LEARN_LO_SHIFT, ST_LEARN_LO_BITS);
    learn_count |= (pack_get(p_packed->word[ST_W_DETECTABLE], ST_LEARN_HI_SHIFT,
                             ST_LEARN_HI_BITS) << ST_LEARN_LO_BITS);
    p_status->learn_count = learn_count;
    p_status->detectable_height_um =
        pack_get(p_packed->word[ST_W_DETECTABLE], ST_DETECTABLE_SHIFT,
                 ST_DETECTABLE_BITS);

    word = p_packed->word[ST_W_PEAKS];
    p_status->current_hump_peak_mg = pack_get(word, ST_PEAK_SHIFT,
                                              ST_PEAK_BITS);
    p_status->highest_hump_peak_mg = pack_get(word, ST_HIGHEST_PEAK_SHIFT,
                                              ST_PEAK_BITS);
    active_height = pack_get(word, ST_ACTIVE_HI_SHIFT, ST_ACTIVE_HI_BITS)
                    << ST_ACTIVE_LO_BITS;
    active_height |= pack_get(p_packed->word[ST_W_HEIGHT_PEAK],
                              ST_ACTIVE_LO_SHIFT, ST_ACTIVE_LO_BITS);
    p_status->active_hump_height_um = active_height;

    p_status->hump_peak_height_um =
        pack_get(p_packed->word[ST_W_HEIGHT_PEAK], ST_HEIGHT_SHIFT,
                 ST_HEIGHT_BITS);
    p_status->highest_hump_height_um =
        p_packed->word[ST_W_HIGHEST_HEIGHT];

    return (b_enums_valid);
}

/*!
 * @brief Clamp a milli-g value into the int16_t range.
 *
 * @param[in] value_mg The value in milli-g.
 *
 * @return The value, limited to -32768..32767.
 */
int16_t
b4_pack_mg_i16 (int32_t value_mg)
{
    int32_t clamped = value_mg;

    if (clamped > (int32_t)INT16_MAX)
    {
        clamped = (int32_t)INT16_MAX;
    }
    else if (clamped < (int32_t)INT16_MIN)
    {
        clamped = (int32_t)INT16_MIN;
    }
    else
    {
        /* Already inside int16_t. */
    }

    return ((int16_t)clamped);
}

/*** end of file ***/
