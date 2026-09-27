/*
 * line_follow.c
 *
 * Buddy 3 - IR line-following logic
 *
 * Converts IR sensor readings into line position,
 * continuous line error, and junction detection.
 */

#include "line_follow.h"

#include <stdbool.h>
#include <stdint.h>


/* TEMPORARY thresholds */
#define IR_LEFT_THRESHOLD      2048U
#define IR_CENTRE_THRESHOLD    2048U
#define IR_RIGHT_THRESHOLD     2048U


#define JUNCTION_DEBOUNCE_COUNT    3U

static uint8_t junction_count = 0U;
static bool junction_latched = false;


void line_follow_init(void)
{
    junction_count = 0U;
    junction_latched = false;
}


/* Junction = all three sensors seeing black. May be refined after testing. */
static bool line_is_junction_pattern(ir_readings_t readings)
{
    bool left_black;
    bool centre_black;
    bool right_black;

    left_black =
        ir_sensor_is_black(readings.left, IR_LEFT_THRESHOLD);

    centre_black =
        ir_sensor_is_black(readings.centre, IR_CENTRE_THRESHOLD);

    right_black =
        ir_sensor_is_black(readings.right, IR_RIGHT_THRESHOLD);

    return (left_black && centre_black && right_black);
}


/*
 * Debounced junction detection. Reports each junction once.
 * Internal only - called from line_get_position() so junction
 * state advances exactly once per cycle.
 */
static bool line_detect_junction(ir_readings_t readings)
{
    bool junction_pattern;

    junction_pattern = line_is_junction_pattern(readings);

    if (junction_pattern)
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
    bool left_black;
    bool centre_black;
    bool right_black;

    if (line_detect_junction(readings))
    {
        return LINE_JUNCTION;
    }

    left_black =
        ir_sensor_is_black(readings.left, IR_LEFT_THRESHOLD);

    centre_black =
        ir_sensor_is_black(readings.centre, IR_CENTRE_THRESHOLD);

    right_black =
        ir_sensor_is_black(readings.right, IR_RIGHT_THRESHOLD);

    if (centre_black && !left_black && !right_black)
    {
        return LINE_CENTRE;
    }

    if (left_black && !right_black)
    {
        return LINE_LEFT;
    }

    if (right_black && !left_black)
    {
        return LINE_RIGHT;
    }

    if (!left_black && !centre_black && !right_black)
    {
        return LINE_LOST;
    }


    if (centre_black)
    {
        return LINE_CENTRE;
    }

    return LINE_LOST;
}


/*
 * Continuous line error: -1.0 = left, 0.0 = centre, +1.0 = right.
 * Only the amount above threshold contributes.
 * NOTE: assumes higher ADC value = black, matching ir_sensor_is_black().
 */
float line_get_error(ir_readings_t readings)
{
    float left_strength;
    float centre_strength;
    float right_strength;

    float total_strength;
    float error;

    if (readings.left > IR_LEFT_THRESHOLD)
    {
        left_strength =
            (float)(readings.left - IR_LEFT_THRESHOLD);
    }
    else
    {
        left_strength = 0.0f;
    }

    if (readings.centre > IR_CENTRE_THRESHOLD)
    {
        centre_strength =
            (float)(readings.centre - IR_CENTRE_THRESHOLD);
    }
    else
    {
        centre_strength = 0.0f;
    }

    if (readings.right > IR_RIGHT_THRESHOLD)
    {
        right_strength =
            (float)(readings.right - IR_RIGHT_THRESHOLD);
    }
    else
    {
        right_strength = 0.0f;
    }

    total_strength =
        left_strength +
        centre_strength +
        right_strength;

    /* No line detected - check LINE_LOST via line_get_position() separately. */
    if (total_strength <= 0.0f)
    {
        return 0.0f;
    }

    error =
        ((-1.0f * left_strength) +
         ( 0.0f * centre_strength) +
         ( 1.0f * right_strength))
        / total_strength;

    return error;
}