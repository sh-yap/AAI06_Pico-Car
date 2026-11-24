/** @file obstacle_calc.c
 *
 * @brief Hardware-independent Buddy 5 calculations.
 *
 * Everything here is deterministic integer arithmetic so it behaves the
 * same on the Pico and in the host unit tests (tests/obstacle).
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "obstacle_config.h"
#include "obstacle_types.h"
#include "obstacle_calc.h"

/* The echo counter is 16 bits wide, so no valid echo exceeds this. */
#define CALC_ECHO_LIMIT_US      (65535U)

/* distance_mm = echo_us * speed_m_s / 2000: 1e-6 s/us * 1000 mm/m, and
 * halved because the sound travels out and back. */
#define CALC_ECHO_DIVISOR       (2000U)
#define CALC_ECHO_ROUNDING      (CALC_ECHO_DIVISOR / 2U)

#define CALC_HALF_TURN_DEG      (180U)
#define CALC_QUARTER_TURN_DEG   (90U)

#define CALC_PWM_PINS_PER_SLICE (2U)
#define CALC_PWM_SLICE_COUNT    (8U)

/* Packed status word layout (see obstacle_calc.h). */
#define CALC_ST_DIST_SHIFT      (0U)
#define CALC_ST_DIST_MASK       (0xFFFU)
#define CALC_ST_RANGE_SHIFT     (12U)
#define CALC_ST_RANGE_MASK      (0x7U)
#define CALC_ST_STATE_SHIFT     (15U)
#define CALC_ST_STATE_MASK      (0x7U)
#define CALC_ST_SEQ_SHIFT       (18U)
#define CALC_ST_SEQ_MASK        (0xFFU)
#define CALC_ST_RESERVED        (0xFC000000UL)

/* Packed profile word layout (see obstacle_calc.h). */
#define CALC_PR_DIST_SHIFT      (0U)
#define CALC_PR_ANGLE_SHIFT     (12U)
#define CALC_PR_WIDTH_SHIFT     (20U)
#define CALC_PR_LEFT_SHIFT      (30U)
#define CALC_PR_RIGHT_SHIFT     (42U)
#define CALC_PR_ACTION_SHIFT    (54U)
#define CALC_PR_MM_MASK         (0xFFFU)   /* Every 12-bit distance field. */
#define CALC_PR_ANGLE_MASK      (0xFFU)
#define CALC_PR_WIDTH_MASK      (0x3FFU)
#define CALC_PR_ACTION_MASK     (0x7U)
#define CALC_PR_RESERVED        (0xFE00000000000000ULL)


/*!
 * @brief Classify a raw echo pulse width.
 *
 * @param[in] echo_us     Echo width counted by the PWM slice.
 * @param[in] b_echo_high True if the echo pin was still high when read.
 *
 * @return OBSTACLE_RANGE_OK, or the reason the reading is unusable.
 */
obstacle_range_status_t
obstacle_calc_classify_echo(uint32_t echo_us, bool b_echo_high)
{
    obstacle_range_status_t status = OBSTACLE_RANGE_OK;

    if (b_echo_high)
    {
        /* The HC-SR04 holds echo high for ~38 ms when nothing returns. */
        status = OBSTACLE_RANGE_TIMEOUT;
    }
    else if (0U == echo_us)
    {
        status = OBSTACLE_RANGE_NO_ECHO;
    }
    else if (echo_us < OBSTACLE_ECHO_MIN_US)
    {
        status = OBSTACLE_RANGE_TOO_CLOSE;
    }
    else if (echo_us > OBSTACLE_ECHO_MAX_US)
    {
        status = OBSTACLE_RANGE_TOO_FAR;
    }
    else
    {
        status = OBSTACLE_RANGE_OK;
    }

    return (status);
}

/*!
 * @brief Convert an echo pulse width to distance.
 *
 * @param[in] echo_us Echo width in microseconds (clamped to 16 bits).
 *
 * @return One-way distance in millimetres, rounded to nearest.
 */
uint32_t
obstacle_calc_echo_to_mm(uint32_t echo_us)
{
    uint32_t limited_us = echo_us;

    if (limited_us > CALC_ECHO_LIMIT_US)
    {
        limited_us = CALC_ECHO_LIMIT_US;
    }

    /* 65535 * 400 < 2^32, so the product cannot overflow. */
    return (((limited_us * OBSTACLE_SOUND_SPEED_M_S) + CALC_ECHO_ROUNDING)
            / CALC_ECHO_DIVISOR);
}

/*!
 * @brief Limit a pulse width to the absolute servo limits.
 *
 * @param[in] pulse_us Requested pulse width.
 *
 * @return pulse_us, clamped to the servo pulse limits.
 */
uint32_t
obstacle_calc_clamp_pulse(uint32_t pulse_us)
{
    uint32_t result_us = pulse_us;

    if (result_us < OBSTACLE_SERVO_PULSE_MIN_US)
    {
        result_us = OBSTACLE_SERVO_PULSE_MIN_US;
    }
    else if (result_us > OBSTACLE_SERVO_PULSE_MAX_US)
    {
        result_us = OBSTACLE_SERVO_PULSE_MAX_US;
    }
    else
    {
        /* Already within limits. */
    }

    return (result_us);
}

/*!
 * @brief Map a servo angle to a pulse width.
 *
 * Linear between the 0/90 and 90/180 degree calibration points, so a
 * servo that is not symmetric about its centre is still handled.  Works
 * for a reversed servo (pulse_0_us > pulse_180_us) as well.
 *
 * @param[in] angle_deg Horn angle, 0..180 (larger values are clamped).
 * @param[in] p_cal     Calibration points; NULL selects the centre pulse.
 *
 * @return Pulse width in microseconds, within the absolute limits.
 */
uint32_t
obstacle_calc_angle_to_pulse(uint32_t                     angle_deg,
                             obstacle_servo_cal_t const * p_cal)
{
    uint32_t result_us = OBSTACLE_SERVO_PULSE_90_US;
    uint32_t limited_deg = angle_deg;
    int32_t  start_us;
    int32_t  end_us;
    int32_t  offset_deg;
    int32_t  pulse_us;

    if (limited_deg > CALC_HALF_TURN_DEG)
    {
        limited_deg = CALC_HALF_TURN_DEG;
    }

    if (NULL != p_cal)
    {
        /* Casts to int32_t: pulses are at most a few thousand and the
         * angle at most 180, so every value fits; signed arithmetic lets
         * the slope be negative for a reversed servo. */
        if (limited_deg <= CALC_QUARTER_TURN_DEG)
        {
            start_us   = (int32_t)p_cal->pulse_0_us;
            end_us     = (int32_t)p_cal->pulse_90_us;
            offset_deg = (int32_t)limited_deg;
        }
        else
        {
            start_us   = (int32_t)p_cal->pulse_90_us;
            end_us     = (int32_t)p_cal->pulse_180_us;
            offset_deg = (int32_t)(limited_deg - CALC_QUARTER_TURN_DEG);
        }

        pulse_us = start_us + (((end_us - start_us) * offset_deg)
                               / (int32_t)CALC_QUARTER_TURN_DEG);

        if (pulse_us < 0)
        {
            pulse_us = 0;
        }

        /* Cast to uint32_t: pulse_us was just made non-negative. */
        result_us = (uint32_t)pulse_us;
    }

    return (obstacle_calc_clamp_pulse(result_us));
}

/*!
 * @brief Estimate servo travel plus settling time.
 *
 * @param[in] from_deg Current angle.
 * @param[in] to_deg   Target angle.
 *
 * @return Milliseconds to wait before trusting a reading at to_deg.
 */
uint32_t
obstacle_calc_settle_ms(uint32_t from_deg, uint32_t to_deg)
{
    uint32_t travel_deg;

    if (from_deg > to_deg)
    {
        travel_deg = from_deg - to_deg;
    }
    else
    {
        travel_deg = to_deg - from_deg;
    }

    return ((travel_deg * OBSTACLE_SERVO_MS_PER_DEG)
            + OBSTACLE_SERVO_SETTLE_MARGIN_MS);
}

/*!
 * @brief Median of a small array (insertion sort, in place).
 *
 * @param[in,out] p_values Values; sorted ascending on return.
 * @param[in]     count    Number of values.
 *
 * @return The median (mean of the middle pair when count is even), or 0
 *         when p_values is NULL or count is 0.
 */
uint32_t
obstacle_calc_median(uint32_t * p_values, uint32_t count)
{
    uint32_t result = 0U;
    uint32_t idx;
    uint32_t pos;
    uint32_t value;

    if ((NULL != p_values) && (count > 0U))
    {
        for (idx = 1U; idx < count; idx++)
        {
            value = p_values[idx];
            pos = idx;

            while ((pos > 0U) && (p_values[pos - 1U] > value))
            {
                p_values[pos] = p_values[pos - 1U];
                pos--;
            }

            p_values[pos] = value;
        }

        if (0U == (count % 2U))
        {
            result = (p_values[(count / 2U) - 1U]
                      + p_values[count / 2U]) / 2U;
        }
        else
        {
            result = p_values[count / 2U];
        }
    }

    return (result);
}

/*!
 * @brief RP2040 PWM slice that drives a GPIO pin.
 *
 * @param[in] gpio_pin GPIO number.
 *
 * @return Slice number 0..7 (GP0/1 -> 0, ..., GP14/15 -> 7, GP16/17 -> 0).
 */
uint32_t
obstacle_calc_pwm_slice(uint32_t gpio_pin)
{
    return ((gpio_pin / CALC_PWM_PINS_PER_SLICE) % CALC_PWM_SLICE_COUNT);
}

/*!
 * @brief Limit a value to the largest number a packed field can hold.
 *
 * @param[in] value     Value to pack.
 * @param[in] field_max Largest value of the field (all bits set).
 *
 * @return value, or field_max if value is larger.
 */
static uint32_t
calc_saturate(uint32_t value, uint32_t field_max)
{
    uint32_t result = value;

    if (result > field_max)
    {
        result = field_max;
    }

    return (result);
}

/*!
 * @brief Pack the latest reading and task state into one 32-bit word.
 *
 * One aligned 32-bit store is atomic on the Cortex-M0+, so a reader on the
 * other core never sees half an update.  The distance is forced to 0
 * unless the range status is OK, so an invalid reading can never be
 * decoded as a distance.
 *
 * @param[in] p_status Values to pack.
 *
 * @return The packed word.  A NULL pointer or an out-of-range state or
 *         status packs as NOT_READY with every other field 0.
 */
uint32_t
obstacle_calc_pack_status(obstacle_status_t const * p_status)
{
    uint32_t word = ((uint32_t)OBSTACLE_RANGE_NOT_READY
                     << CALC_ST_RANGE_SHIFT);  /* Cast: enum to field. */
    uint32_t distance_mm = 0U;

    if ((NULL != p_status) &&
        (p_status->range_status <= OBSTACLE_RANGE_NOT_READY) &&
        (p_status->state < OBSTACLE_STATE_COUNT))
    {
        if (OBSTACLE_RANGE_OK == p_status->range_status)
        {
            distance_mm = calc_saturate(p_status->distance_mm,
                                        CALC_ST_DIST_MASK);
        }

        /* Casts to uint32_t: both enums were range-checked above. */
        word = (distance_mm << CALC_ST_DIST_SHIFT)
               | ((uint32_t)p_status->range_status << CALC_ST_RANGE_SHIFT)
               | ((uint32_t)p_status->state << CALC_ST_STATE_SHIFT)
               | ((p_status->seq & CALC_ST_SEQ_MASK) << CALC_ST_SEQ_SHIFT);
    }

    return (word);
}

/*!
 * @brief Decode a packed status word.
 *
 * @param[in]  word     Word from obstacle_calc_pack_status().
 * @param[out] p_status Decoded values; written only when the word is valid.
 *
 * @return false if p_status is NULL or the word is malformed: reserved
 *         bits set, an unknown state or status code, or a distance with a
 *         status other than OK.
 */
bool
obstacle_calc_unpack_status(uint32_t word, obstacle_status_t * p_status)
{
    bool     b_valid = false;
    uint32_t distance_mm = (word >> CALC_ST_DIST_SHIFT) & CALC_ST_DIST_MASK;
    uint32_t range = (word >> CALC_ST_RANGE_SHIFT) & CALC_ST_RANGE_MASK;
    uint32_t state = (word >> CALC_ST_STATE_SHIFT) & CALC_ST_STATE_MASK;

    if ((NULL != p_status) &&
        (0U == (word & CALC_ST_RESERVED)) &&
        (range <= (uint32_t)OBSTACLE_RANGE_NOT_READY) &&  /* Cast: enum. */
        (state < (uint32_t)OBSTACLE_STATE_COUNT) &&       /* Cast: enum. */
        ((0U == distance_mm) ||
         ((uint32_t)OBSTACLE_RANGE_OK == range)))         /* Cast: enum. */
    {
        /* Casts to the enums: both codes were range-checked above. */
        p_status->distance_mm  = distance_mm;
        p_status->range_status = (obstacle_range_status_t)range;
        p_status->state        = (obstacle_state_t)state;
        p_status->seq          = (word >> CALC_ST_SEQ_SHIFT)
                                 & CALC_ST_SEQ_MASK;
        b_valid = true;
    }

    return (b_valid);
}

/*!
 * @brief Pack a scan result into one 64-bit word.
 *
 * A 64-bit store is not atomic on the Cortex-M0+: the obstacle task must
 * hand this word over through a message or under a lock, not as a plain
 * shared variable.
 *
 * @param[in] p_profile Values to pack.  Distances and the width saturate
 *                      at their field maximum; the angle is limited to 180.
 *
 * @return The packed word.  A NULL pointer or an unknown action packs as
 *         0: no valid distance and action STOP, the safe default.
 */
uint64_t
obstacle_calc_pack_profile(obstacle_profile_t const * p_profile)
{
    uint64_t word = 0U;

    if ((NULL != p_profile) && (p_profile->action < OBSTACLE_ACTION_COUNT))
    {
        /* Casts to uint64_t: each field is shifted past bit 31. */
        word = ((uint64_t)calc_saturate(p_profile->distance_mm,
                                        CALC_PR_MM_MASK)
                << CALC_PR_DIST_SHIFT)
               | ((uint64_t)calc_saturate(p_profile->angle_deg,
                                          CALC_HALF_TURN_DEG)
                  << CALC_PR_ANGLE_SHIFT)
               | ((uint64_t)calc_saturate(p_profile->width_mm,
                                          CALC_PR_WIDTH_MASK)
                  << CALC_PR_WIDTH_SHIFT)
               | ((uint64_t)calc_saturate(p_profile->clear_left_mm,
                                          CALC_PR_MM_MASK)
                  << CALC_PR_LEFT_SHIFT)
               | ((uint64_t)calc_saturate(p_profile->clear_right_mm,
                                          CALC_PR_MM_MASK)
                  << CALC_PR_RIGHT_SHIFT)
               | ((uint64_t)p_profile->action << CALC_PR_ACTION_SHIFT);
    }

    return (word);
}

/*!
 * @brief Decode a packed profile word.
 *
 * @param[in]  word      Word from obstacle_calc_pack_profile().
 * @param[out] p_profile Decoded values; written only when the word is
 *                       valid.
 *
 * @return false if p_profile is NULL or the word is malformed: reserved
 *         bits set, an angle above 180 or an unknown action.
 */
bool
obstacle_calc_unpack_profile(uint64_t word, obstacle_profile_t * p_profile)
{
    bool     b_valid = false;

    /* Casts to uint32_t: each value is masked to 12 bits or fewer. */
    uint32_t angle_deg = (uint32_t)((word >> CALC_PR_ANGLE_SHIFT)
                                    & CALC_PR_ANGLE_MASK);
    uint32_t action = (uint32_t)((word >> CALC_PR_ACTION_SHIFT)
                                 & CALC_PR_ACTION_MASK);

    if ((NULL != p_profile) &&
        (0U == (word & CALC_PR_RESERVED)) &&
        (angle_deg <= CALC_HALF_TURN_DEG) &&
        (action < (uint32_t)OBSTACLE_ACTION_COUNT))       /* Cast: enum. */
    {
        /* Casts as above; the action code was range-checked. */
        p_profile->distance_mm    = (uint32_t)((word >> CALC_PR_DIST_SHIFT)
                                               & CALC_PR_MM_MASK);
        p_profile->angle_deg      = angle_deg;
        p_profile->width_mm       = (uint32_t)((word >> CALC_PR_WIDTH_SHIFT)
                                               & CALC_PR_WIDTH_MASK);
        p_profile->clear_left_mm  = (uint32_t)((word >> CALC_PR_LEFT_SHIFT)
                                               & CALC_PR_MM_MASK);
        p_profile->clear_right_mm = (uint32_t)((word >> CALC_PR_RIGHT_SHIFT)
                                               & CALC_PR_MM_MASK);
        p_profile->action         = (obstacle_action_t)action;
        b_valid = true;
    }

    return (b_valid);
}

/*** end of file ***/
