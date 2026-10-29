/*
 * line_follow.c
 *
 * Buddy 3 - IR line-following logic
 *
 * Converts the two line-sensor readings (centre and right)
 * into line position, continuous line error, and junction
 * detection. The third IR sensor (GP28) is the dedicated
 * barcode sensor and is not used here.
 *
 * With only centre and right sensors, "line to the left" cannot
 * be seen directly. When both sensors see white, the line is
 * assumed to be on the side it was last seen on, until it has
 * been missing for LINE_LOST_COUNT cycles.
 */

#include "line_follow.h"

#include <stdbool.h>
#include <stdint.h>


/*
 * Calibrated 2026-10-27 on white paper and real track tape, room light
 * (midpoint of highest white and lowest black per sensor):
 *   centre GP27: white <= 216, black >= 2910
 *   right  GP26: white <= 200, black >= 3150
 * Lighting variation not yet tested.
 */
#define IR_CENTRE_THRESHOLD    1563U
#define IR_RIGHT_THRESHOLD     1675U

/*
 * Centre reading with the sensor over the middle of the line
 * (2026-10-29: ~2290).  Between this and IR_CENTRE_THRESHOLD the line
 * is sliding off the centre sensor to the left (the right sensor sees
 * it first if it moves right), so the error ramps 0 .. -LINE_ERROR_MAX
 * instead of jumping to full left once the centre turns white.
 * Re-measure if the sensor is moved; readings above it count as fully
 * on the line.
 */
#define IR_CENTRE_FULL_BLACK   2300U


/* Junction = both line sensors black for this many consecutive cycles. */
#define JUNCTION_DEBOUNCE_COUNT    3U

/* Both sensors white for this many cycles (10 ms each) = LINE_LOST. */
#define LINE_LOST_COUNT            30U

static uint8_t junction_count = 0U;
static bool junction_latched = false;

static uint8_t white_count = 0U;

/* Side the line was last seen on; used while both sensors see white. */
static line_state_t last_seen = LINE_CENTRE;


void line_follow_init(void)
{
    junction_count = 0U;
    junction_latched = false;
    white_count = 0U;
    last_seen = LINE_CENTRE;
}


/*
 * Debounced junction detection. Reports each junction once.
 * Internal only - called from line_get_position() so junction
 * state advances exactly once per cycle.
 * May be refined after track testing (a slow drift across the
 * line also briefly puts both sensors on black).
 */
static bool line_detect_junction(bool centre_black, bool right_black)
{
    if (centre_black && right_black)
    {
        if (junction_count < JUNCTION_DEBOUNCE_COUNT)
        {
            junction_count++;
        }

        if ((junction_count >= JUNCTION_DEBOUNCE_COUNT) &&
            (junction_latched == false))
        {
            junction_latched = true;
            return true;
        }
    }
    else
    {
        junction_count = 0U;
        junction_latched = false;
    }

    return false;
}


line_state_t line_get_position(ir_readings_t readings)
{
    bool centre_black;
    bool right_black;

    centre_black =
        ir_sensor_is_black(readings.centre, IR_CENTRE_THRESHOLD);

    right_black =
        ir_sensor_is_black(readings.right, IR_RIGHT_THRESHOLD);

    if (line_detect_junction(centre_black, right_black))
    {
        white_count = 0U;
        return LINE_JUNCTION;
    }

    if (centre_black && !right_black)
    {
        white_count = 0U;
        last_seen = LINE_CENTRE;
        return LINE_CENTRE;
    }

    if (right_black)
    {
        white_count = 0U;
        last_seen = LINE_RIGHT;
        return LINE_RIGHT;
    }

    /* Both white: line has moved off the left of centre, or off the right sensor. */
    if (white_count < LINE_LOST_COUNT)
    {
        white_count++;
    }

    if (white_count >= LINE_LOST_COUNT)
    {
        return LINE_LOST;
    }

    return (last_seen == LINE_RIGHT) ? LINE_RIGHT : LINE_LEFT;
}


/*
 * Continuous line error: -127 = left, 0 = centre, +127 = right.
 * Only the amount above threshold contributes.
 * Integer maths only - the RP2040 has no FPU.
 * Uses the state from line_get_position(), so call that first
 * in the same cycle.
 * NOTE: assumes higher ADC value = black, matching ir_sensor_is_black().
 */
int8_t line_get_error(ir_readings_t readings)
{
    uint32_t centre_strength;
    uint32_t right_strength;
    uint32_t total_strength;

    if (readings.centre > IR_CENTRE_THRESHOLD)
    {
        centre_strength =
            (uint32_t)(readings.centre - IR_CENTRE_THRESHOLD);
    }
    else
    {
        centre_strength = 0U;
    }

    if (readings.right > IR_RIGHT_THRESHOLD)
    {
        right_strength =
            (uint32_t)(readings.right - IR_RIGHT_THRESHOLD);
    }
    else
    {
        right_strength = 0U;
    }

    total_strength = centre_strength + right_strength;

    /* Neither sensor on the line: full correction towards the last-seen side. */
    if (total_strength == 0U)
    {
        if (white_count >= LINE_LOST_COUNT)
        {
            return 0;
        }

        return (last_seen == LINE_RIGHT) ? LINE_ERROR_MAX : -LINE_ERROR_MAX;
    }

    /*
     * Only the centre sensor on the line: steer left in proportion to
     * how far its reading has dropped below fully-on-the-line.
     */
    if (right_strength == 0U)
    {
        if (readings.centre >= IR_CENTRE_FULL_BLACK)
        {
            return 0;
        }

        return (int8_t)(-(int32_t)(
            (((uint32_t)(IR_CENTRE_FULL_BLACK - readings.centre)
              * (uint32_t)LINE_ERROR_MAX)
             + ((IR_CENTRE_FULL_BLACK - IR_CENTRE_THRESHOLD) / 2U))
            / (IR_CENTRE_FULL_BLACK - IR_CENTRE_THRESHOLD)));
    }

    /* Rounded; result is 0..LINE_ERROR_MAX since right <= total. */
    return (int8_t)(((right_strength * (uint32_t)LINE_ERROR_MAX)
                     + (total_strength / 2U)) / total_strength);
}


uint8_t line_get_sensor_bits(ir_readings_t readings)
{
    uint8_t bits = 0U;

    if (ir_sensor_is_black(readings.centre, IR_CENTRE_THRESHOLD))
    {
        bits |= LINE_SENSOR_CENTRE;
    }

    if (ir_sensor_is_black(readings.right, IR_RIGHT_THRESHOLD))
    {
        bits |= LINE_SENSOR_RIGHT;
    }

    return bits;
}
