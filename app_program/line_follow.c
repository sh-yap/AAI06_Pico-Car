/*
 * line_follow.c
 *
 * Buddy 3 - IR line-following logic
 *
 * Converts the two line-sensor readings (centre and right)
 * into line position, continuous line error, and junction
 * detection. The third IR sensor (GP28) is the dedicated
 * barcode sensor and is not used here, except that the START
 * calibration records its range too.
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
 * Thresholds in use. Every START learns them in the pivot sweep (see
 * line_cal_*). The defaults below are the 2026-10-27/29 values, measured
 * with the sensors in one position: they are stale, serve only until the
 * first sweep, and a run never starts without a fresh one.
 *
 * Higher ADC = black. A line sensor is black at or above its threshold,
 * the midpoint of the lowest and highest reading it saw. Between
 * centre_full_black and the centre threshold the line is sliding off the
 * centre sensor to the left (the right sensor sees it first if it moves
 * right), so the error ramps 0 .. -LINE_ERROR_MAX instead of jumping to
 * full left once the centre turns white. Readings at or above
 * centre_full_black count as fully on the line.
 */
#define IR_CENTRE_THRESHOLD_DEFAULT    1563U
#define IR_RIGHT_THRESHOLD_DEFAULT     1675U
#define IR_CENTRE_FULL_BLACK_DEFAULT   2300U

static uint16_t centre_threshold = IR_CENTRE_THRESHOLD_DEFAULT;
static uint16_t right_threshold = IR_RIGHT_THRESHOLD_DEFAULT;
static uint16_t centre_full_black = IR_CENTRE_FULL_BLACK_DEFAULT;


/*
 * Smallest black-to-white span a sensor must show for the sweep to count.
 * The 12-bit ADC (4096 counts) is good to a few counts of noise, and a
 * bare floor varies by tens of counts with distance and ambient light.
 * One sixteenth of full scale is far above both, yet far below any real
 * line, so a span under it means no line was seen, not a weak line. It
 * also keeps the margins below (a thirty-second and an eighth of the span)
 * larger than the noise.
 */
#define LINE_CAL_MIN_SPAN          256U

/* Filter each reading by one quarter of the step before recording it. */
#define LINE_CAL_FILTER_DIV        4

/*
 * A sensor is at rest when its lowest and highest values have not moved
 * by more than a thirty-second of the span for this many samples (10 ms
 * each): the pivot has carried the sensor's spot clear of an edge.
 */
#define LINE_CAL_REST_SAMPLES      8U
#define LINE_CAL_REST_DIV          32U

/* The full-black level sits this fraction (1/8) of the span below the top. */
#define LINE_CAL_FULL_BLACK_DIV    8U


/* Junction = both line sensors black for this many consecutive cycles. */
#define JUNCTION_DEBOUNCE_COUNT    3U

/* Both sensors white for this many cycles (10 ms each) = LINE_LOST. */
#define LINE_LOST_COUNT            30U

static uint8_t junction_count = 0U;
static bool junction_latched = false;

static uint8_t white_count = 0U;

/* Side the line was last seen on; used while both sensors see white. */
static line_state_t last_seen = LINE_CENTRE;


/* One sensor's record of the sweep. */
typedef struct
{
    int32_t filtered;
    uint16_t low;
    uint16_t high;
    uint8_t rest;          /* Samples since low or high last moved. */
} cal_track_t;

static line_cal_state_t cal_state = LINE_CAL_IDLE;
static bool cal_first_sample = true;
static cal_track_t cal_centre;
static cal_track_t cal_right;
static cal_track_t cal_barcode;
static line_cal_result_t cal_result;


/*
 * Gap check history: the last LINE_CAL_HISTORY pairs of centre and right
 * readings of the sweep, kept in a ring because the thresholds that say
 * which readings are black are only known when both sensors are done.
 * 256 samples of 4 bytes is 1 KB, at the 10 ms Buddy 3 cycle 2.56 s. The
 * sweep is at most 45 degrees out and 90 degrees across at about 77
 * degrees per second (under 2 s), so the pass that finds the line fits.
 * Older readings (the still phase) hold no black stretch worth keeping.
 */
#define LINE_CAL_HISTORY           256U
#define LINE_CAL_HISTORY_MASK      (LINE_CAL_HISTORY - 1U)
#define LINE_CAL_OVERLAP_LIMIT     255

typedef struct
{
    uint16_t centre;
    uint16_t right;
} cal_sample_t;

static cal_sample_t cal_history[LINE_CAL_HISTORY];
static uint32_t cal_history_count;      /* Samples recorded since sweep.   */
static bool cal_sweeping;               /* The pivot has started.          */


static void clear_line_state(void)
{
    junction_count = 0U;
    junction_latched = false;
    white_count = 0U;
    last_seen = LINE_CENTRE;
}


void line_follow_init(void)
{
    clear_line_state();
    centre_threshold = IR_CENTRE_THRESHOLD_DEFAULT;
    right_threshold = IR_RIGHT_THRESHOLD_DEFAULT;
    centre_full_black = IR_CENTRE_FULL_BLACK_DEFAULT;
    cal_state = LINE_CAL_IDLE;
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
        ir_sensor_is_black(readings.centre, centre_threshold);

    right_black =
        ir_sensor_is_black(readings.right, right_threshold);

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

    if (readings.centre > centre_threshold)
    {
        centre_strength =
            (uint32_t)(readings.centre - centre_threshold);
    }
    else
    {
        centre_strength = 0U;
    }

    if (readings.right > right_threshold)
    {
        right_strength =
            (uint32_t)(readings.right - right_threshold);
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
        if (readings.centre >= centre_full_black)
        {
            return 0;
        }

        return (int8_t)(-(int32_t)(
            (((uint32_t)(centre_full_black - readings.centre)
              * (uint32_t)LINE_ERROR_MAX)
             + ((uint32_t)(centre_full_black - centre_threshold) / 2U))
            / (uint32_t)(centre_full_black - centre_threshold)));
    }

    /* Rounded; result is 0..LINE_ERROR_MAX since right <= total. */
    return (int8_t)(((right_strength * (uint32_t)LINE_ERROR_MAX)
                     + (total_strength / 2U)) / total_strength);
}


uint8_t line_get_sensor_bits(ir_readings_t readings)
{
    uint8_t bits = 0U;

    if (ir_sensor_is_black(readings.centre, centre_threshold))
    {
        bits |= LINE_SENSOR_CENTRE;
    }

    if (ir_sensor_is_black(readings.right, right_threshold))
    {
        bits |= LINE_SENSOR_RIGHT;
    }

    return bits;
}


/* Span (highest minus lowest) a sensor has seen. */
static uint32_t cal_span(cal_track_t const *p_track)
{
    return (uint32_t)(p_track->high - p_track->low);
}


/*
 * Record one reading: filter it, widen the range, and count how long the
 * range has been still. The first reading of a sweep starts the range.
 */
static void cal_track_update(cal_track_t *p_track, uint16_t reading)
{
    uint32_t margin;
    uint16_t value;
    bool moved = false;

    if (cal_first_sample)
    {
        p_track->filtered = (int32_t)reading;
        p_track->low = reading;
        p_track->high = reading;
        p_track->rest = 0U;
        return;
    }

    p_track->filtered +=
        ((int32_t)reading - p_track->filtered) / LINE_CAL_FILTER_DIV;
    value = (uint16_t)p_track->filtered;
    margin = cal_span(p_track) / LINE_CAL_REST_DIV;

    if (value < p_track->low)
    {
        moved = ((uint32_t)(p_track->low - value) > margin);
        p_track->low = value;
    }
    else if (value > p_track->high)
    {
        moved = ((uint32_t)(value - p_track->high) > margin);
        p_track->high = value;
    }
    else
    {
        /* Inside the range already seen. */
    }

    if (moved)
    {
        p_track->rest = 0U;
    }
    else if (p_track->rest < UINT8_MAX)
    {
        p_track->rest++;
    }
    else
    {
        /* Saturated: plenty of rest. */
    }
}


/*
 * A line sensor has done its part when it has seen enough contrast, is
 * back in the lower quarter of its range (on white) and its range has
 * stopped growing. The rest test stops a reading that is only passing
 * the lowest value on the way down from counting as white.
 */
static bool cal_sensor_done(cal_track_t const *p_track)
{
    uint32_t span = cal_span(p_track);

    return (span >= LINE_CAL_MIN_SPAN)
           && ((uint32_t)p_track->filtered
               <= ((uint32_t)p_track->low + (span / 4U)))
           && (p_track->rest >= LINE_CAL_REST_SAMPLES);
}


/* Threshold = midpoint of the lowest and highest reading. */
static uint16_t cal_midpoint(cal_track_t const *p_track)
{
    return (uint16_t)((uint32_t)p_track->low + (cal_span(p_track) / 2U));
}


static void cal_copy_range(line_cal_sensor_t *p_out,
                           cal_track_t const *p_track)
{
    p_out->low = p_track->low;
    p_out->high = p_track->high;
    p_out->threshold = 0U;
}


static void cal_record_ranges(void)
{
    cal_copy_range(&cal_result.centre, &cal_centre);
    cal_copy_range(&cal_result.right, &cal_right);
    cal_copy_range(&cal_result.barcode, &cal_barcode);
}


void line_cal_begin(void)
{
    cal_state = LINE_CAL_RUNNING;
    cal_first_sample = true;
    cal_history_count = 0U;
    cal_sweeping = false;
    cal_result = (line_cal_result_t){0};
}


/*
 * The pivot has started. Only readings from here on set the lowest and
 * highest levels (and so the thresholds) and go into the gap history. While
 * the car stands still a sensor may rest on the line and read higher than
 * it ever does moving, which would raise its threshold above the level the
 * sweep reaches and split the two black stretches apart.
 */
void line_cal_sweep(void)
{
    if (cal_state == LINE_CAL_RUNNING)
    {
        cal_sweeping = true;
        cal_first_sample = true;
        cal_history_count = 0U;
    }
}


/*
 * Is there a heading where the line lies between the two line sensors with
 * both reading white? In one pass the sensors cross the line in a fixed
 * order, so the line covers both at once exactly when their black
 * stretches overlap. Each sensor's stretch is taken as first to last
 * sample above its threshold in the history, and
 *
 *     overlap = min(last_c, last_r) - max(first_c, first_r) + 1
 *
 * counts the samples both were black (zero = the stretches touch, with no
 * sample of both white between them; negative = that many samples of both
 * white, the gap). Samples are in time order and the pivot speed is
 * constant, so within a pass the count stands for angle. A sensor under
 * the line before the sweep, or a sweep that turned back across it, only
 * stretches the envelope towards more overlap, never less, so such data
 * cannot make a gap appear.
 *
 * Returns false (not measured) when the data cannot show a gap: a sensor
 * never above its threshold, or a negative overlap while the ring has
 * dropped older samples and a stretch starts at its oldest sample, which
 * could have started earlier and overlapped.
 */
static bool cal_measure_overlap(uint16_t c_thr, uint16_t r_thr,
                                int16_t *p_overlap)
{
    uint32_t count = cal_history_count;
    uint32_t used = (count < LINE_CAL_HISTORY) ? count : LINE_CAL_HISTORY;
    uint32_t oldest = count - used;
    uint32_t k;
    cal_sample_t sample;
    int32_t c_first = -1;
    int32_t c_last = -1;
    int32_t r_first = -1;
    int32_t r_last = -1;
    int32_t overlap;
    int32_t first;
    int32_t last;
    bool measured = false;

    for (k = 0U; k < used; k++)
    {
        sample = cal_history[(oldest + k) & LINE_CAL_HISTORY_MASK];

        if (sample.centre > c_thr)
        {
            c_first = (c_first < 0) ? (int32_t)k : c_first;
            c_last = (int32_t)k;
        }

        if (sample.right > r_thr)
        {
            r_first = (r_first < 0) ? (int32_t)k : r_first;
            r_last = (int32_t)k;
        }
    }

    if ((c_first >= 0) && (r_first >= 0))
    {
        last = (c_last < r_last) ? c_last : r_last;
        first = (c_first > r_first) ? c_first : r_first;
        overlap = last - first + 1;
        measured = true;

        if ((overlap < 0) && (count > LINE_CAL_HISTORY)
            && ((c_first == 0) || (r_first == 0)))
        {
            measured = false;
        }

        if (overlap > LINE_CAL_OVERLAP_LIMIT)
        {
            overlap = LINE_CAL_OVERLAP_LIMIT;
        }

        *p_overlap = (int16_t)overlap;
    }

    return measured;
}


void line_cal_update(ir_readings_t readings)
{
    if ((cal_state != LINE_CAL_RUNNING) || !cal_sweeping)
    {
        return;
    }

    cal_history[cal_history_count & LINE_CAL_HISTORY_MASK].centre =
        readings.centre;
    cal_history[cal_history_count & LINE_CAL_HISTORY_MASK].right =
        readings.right;
    cal_history_count++;

    cal_track_update(&cal_centre, readings.centre);
    cal_track_update(&cal_right, readings.right);
    cal_track_update(&cal_barcode, readings.barcode);
    cal_first_sample = false;

    if (cal_sensor_done(&cal_centre) && cal_sensor_done(&cal_right))
    {
        uint16_t new_centre = cal_midpoint(&cal_centre);
        uint16_t new_right = cal_midpoint(&cal_right);
        int16_t overlap = 0;

        cal_record_ranges();
        cal_result.centre.threshold = new_centre;
        cal_result.right.threshold = new_right;

        /* Each span is at least LINE_CAL_MIN_SPAN, so the full-black level
         * stays above the centre threshold. */
        cal_result.centre_full_black = (uint16_t)((uint32_t)cal_centre.high
                                                  - (cal_span(&cal_centre)
                                                     / LINE_CAL_FULL_BLACK_DIV));

        cal_result.gap_measured = cal_measure_overlap(new_centre, new_right,
                                                      &overlap);
        cal_result.gap_overlap = overlap;

        /* The barcode sensor may sit beside the line and never see black.
         * Without enough contrast it keeps the caller's fallback. */
        if (cal_span(&cal_barcode) >= LINE_CAL_MIN_SPAN)
        {
            cal_result.barcode.threshold = cal_midpoint(&cal_barcode);
        }

        if (cal_result.gap_measured && (overlap < 0))
        {
            /* A white gap between the sensors: the thresholds stay as
             * they were, the result keeps what was measured. */
            cal_state = LINE_CAL_FAIL_GAP;
            return;
        }

        centre_threshold = new_centre;
        right_threshold = new_right;
        centre_full_black = cal_result.centre_full_black;
        clear_line_state();
        cal_state = LINE_CAL_OK;
    }
}


line_cal_state_t line_cal_end(void)
{
    if (cal_state == LINE_CAL_RUNNING)
    {
        cal_record_ranges();

        if ((cal_span(&cal_centre) < LINE_CAL_MIN_SPAN)
            || (cal_span(&cal_right) < LINE_CAL_MIN_SPAN))
        {
            cal_state = LINE_CAL_FAIL_CONTRAST;
        }
        else
        {
            cal_state = LINE_CAL_FAIL_LINE;
        }
    }

    return cal_state;
}


void line_cal_abort(void)
{
    cal_state = LINE_CAL_IDLE;
}


line_cal_state_t line_cal_state(void)
{
    return cal_state;
}


void line_cal_get_result(line_cal_result_t *p_result)
{
    *p_result = cal_result;
}


uint16_t line_cal_barcode_threshold(uint16_t fallback)
{
    uint16_t threshold = fallback;

    if ((cal_state == LINE_CAL_OK) && (cal_result.barcode.threshold != 0U))
    {
        threshold = cal_result.barcode.threshold;
    }

    return threshold;
}
