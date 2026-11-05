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
#include "motion_config.h"

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


/*
 * Distance-windowed references. The START sweep sets the thresholds once,
 * but the tape's black level varies along the track. While driving, the
 * highest reading of each line sensor over the last LINE_REF_WINDOW_MM of
 * forward wheel travel is kept in a ring of buckets, and the black level
 * follows it. White stays at the START low level: a centred car never sees
 * white, so the window minimum would be an edge reading. The window is
 * distance, not time, so a stopped or pivoting car never forgets its
 * reference.
 *
 * NOTE: the window length is a starting value to be confirmed by a car run.
 */
#define LINE_REF_WINDOW_MM         150U
#define LINE_REF_BUCKETS           6U
#define LINE_REF_BUCKET_MM         (LINE_REF_WINDOW_MM / LINE_REF_BUCKETS)

/* pi in Q16, as motion_task.c uses it for the same count conversion. */
#define LINE_REF_PI_Q16            205887LL
#define LINE_REF_SCALE_ONE         1024LL

typedef struct
{
    uint16_t centre_max;
    uint16_t right_max;
} ref_bucket_t;

static ref_bucket_t ref_bucket[LINE_REF_BUCKETS];
static uint32_t ref_head;            /* Bucket now collecting.            */
static uint32_t ref_travel;          /* Counts travelled in that bucket.  */
static uint32_t ref_advances;        /* Buckets started since the seed.   */
static uint32_t ref_bucket_counts;   /* Encoder counts per bucket.        */
static bool ref_active;              /* Seeded from a successful START.   */


static void clear_line_state(void)
{
    junction_count = 0U;
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
    ref_active = false;
    ref_bucket_counts = 0U;
}


/*
 * Debounced junction detection: true for as long as both sensors stay
 * black, once they have been for JUNCTION_DEBOUNCE_COUNT cycles.
 * Reported every cycle, not once: the VC reads only the newest of the
 * two statuses sent per 20 ms step, so a one-cycle JUNCTION could be
 * overwritten and missed.  The VC acts on each junction once, and only
 * when it lasts long enough (a drift across the line on a curve also
 * puts both sensors on black, briefly) - see VC_JUNCTION_CONFIRM_MM.
 * Internal only - called from line_get_position() so junction
 * state advances exactly once per cycle.
 */
static bool line_detect_junction(bool centre_black, bool right_black)
{
    if (centre_black && right_black)
    {
        if (junction_count < JUNCTION_DEBOUNCE_COUNT)
        {
            junction_count++;
        }

        return (junction_count >= JUNCTION_DEBOUNCE_COUNT);
    }

    junction_count = 0U;
    return false;
}


/*
 * A reading as a distance above the sensor's learned low, held within the
 * learned span (so a reading below the low is zero, not a wrap).
 */
static uint32_t cal_level(uint16_t reading, uint16_t low, uint32_t span)
{
    uint32_t level = 0U;

    if (reading > low)
    {
        level = (uint32_t)(reading - low);
        level = (level < span) ? level : span;
    }

    return level;
}


/*
 * Is a reading above its sensor's learned low by more than a thirty-second
 * of that sensor's span? Then the sensor sees some of the line. The ranges
 * are those just learned, so the caller decides whether they can be trusted.
 */
static bool sees_some_line(uint16_t reading, uint16_t low, uint16_t high)
{
    uint32_t span = (uint32_t)high - low;
    bool sees = false;

    if (span > 0U)
    {
        sees = ((cal_level(reading, low, span) * LINE_CAL_REST_DIV) > span);
    }

    return sees;
}


/* Does the right sensor see some of the line? */
static bool right_sees_line(uint16_t reading)
{
    return sees_some_line(reading, cal_result.right.low,
                          cal_result.right.high);
}


/*
 * Which side does the line favour while both sensors read white? The right
 * sensor sits right of the centre sensor, so if it sees any of the line the
 * line lies between the two or beyond the right sensor, and RIGHT is always
 * right. A partial centre reading alone is ambiguous (just left or just
 * right of centre). After RIGHT with the right sensor blind, though, it
 * shows the line is no longer beyond the right sensor, so it is reported as
 * CENTRE (line_get_position() turns that into LEFT, as for any exit from
 * centre). If the line is really going right, the right sensor sets RIGHT
 * again.
 *
 * Returns LINE_RIGHT if the right sensor sees the line, LINE_CENTRE if
 * last-seen is RIGHT and the centre sensor sees some of it, else the
 * unchanged last-seen side. Without a good sweep it always returns
 * last-seen.
 */
static line_state_t both_white_side(ir_readings_t readings)
{
    line_state_t side = last_seen;

    if (cal_state == LINE_CAL_OK)
    {
        if (right_sees_line(readings.right))
        {
            side = LINE_RIGHT;
        }
        else if ((LINE_RIGHT == last_seen)
                 && sees_some_line(readings.centre, cal_result.centre.low,
                                   cal_result.centre.high))
        {
            side = LINE_CENTRE;
        }
        else
        {
            /* Nothing new: keep the last-seen side. */
        }
    }

    return side;
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

    last_seen = both_white_side(readings);

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


/*
 * Take a sensor's low and high from the raw readings in the history: the
 * second-lowest and second-highest, so one stray sample is rejected. The
 * smoothed track cannot reach the height of a peak only a few samples
 * long. If the raw range is too narrow to use, the smoothed one stays.
 */
static void cal_use_raw_range(cal_track_t *p_track, bool centre)
{
    uint32_t count = cal_history_count;
    uint32_t used = (count < LINE_CAL_HISTORY) ? count : LINE_CAL_HISTORY;
    uint32_t k;
    uint16_t value;
    uint16_t low1 = UINT16_MAX;
    uint16_t low2 = UINT16_MAX;
    uint16_t high1 = 0U;
    uint16_t high2 = 0U;

    for (k = 0U; k < used; k++)
    {
        value = centre ? cal_history[k].centre : cal_history[k].right;

        if (value < low1)
        {
            low2 = low1;
            low1 = value;
        }
        else if (value < low2)
        {
            low2 = value;
        }
        else
        {
            /* Not among the two lowest. */
        }

        if (value > high1)
        {
            high2 = high1;
            high1 = value;
        }
        else if (value > high2)
        {
            high2 = value;
        }
        else
        {
            /* Not among the two highest. */
        }
    }

    if ((used >= 2U) && (high2 > low2)
        && ((uint32_t)(high2 - low2) >= LINE_CAL_MIN_SPAN))
    {
        p_track->low = low2;
        p_track->high = high2;
    }
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


/*
 * Encoder counts for mm of travel on one wheel, the inverse of the
 * mm-per-count conversion in motion_task.c.
 */
static int64_t ref_wheel_counts(uint32_t mm, int32_t counts_per_rev,
                                int32_t diameter_um, uint16_t scale_q10)
{
    int64_t denom = LINE_REF_PI_Q16 * diameter_um * scale_q10;
    int64_t num = (int64_t)mm * counts_per_rev * 1000LL
                  * LINE_REF_SCALE_ONE * 65536LL;

    return (denom > 0) ? (num / denom) : 0;
}


/* Counts per bucket: the mean of both wheels, computed once. */
static uint32_t ref_get_bucket_counts(void)
{
    motion_config_t config;
    int64_t left;
    int64_t right;
    int64_t mean;

    if (0U == ref_bucket_counts)
    {
        motion_config_default(&config);
        left = ref_wheel_counts(LINE_REF_BUCKET_MM,
                                config.left_counts_per_wheel_rev,
                                config.left_wheel_diameter_um,
                                config.distance_scale_q10);
        right = ref_wheel_counts(LINE_REF_BUCKET_MM,
                                 config.right_counts_per_wheel_rev,
                                 config.right_wheel_diameter_um,
                                 config.distance_scale_q10);
        mean = (left + right) / 2;

        /* A bad config must not make the bucket length zero. */
        ref_bucket_counts = (mean > 0) ? (uint32_t)mean : 1U;
    }

    return ref_bucket_counts;
}


static void ref_clear_bucket(ref_bucket_t *p_bucket)
{
    p_bucket->centre_max = 0U;
    p_bucket->right_max = 0U;
}


/* Start the window from the START result (the sweep's own raw range). */
static void ref_seed(void)
{
    uint32_t k;

    for (k = 0U; k < LINE_REF_BUCKETS; k++)
    {
        ref_clear_bucket(&ref_bucket[k]);
    }

    ref_bucket[0].centre_max = cal_result.centre.high;
    ref_bucket[0].right_max = cal_result.right.high;
    ref_head = 0U;
    ref_travel = 0U;
    ref_advances = 0U;
    ref_active = true;
}


/*
 * True if the window maximum of one sensor is usable as its black level:
 * the sensor saw tape (the maximum is above its current threshold) and the
 * span above the START low level is at least LINE_CAL_MIN_SPAN. Then the
 * threshold is the midpoint of the START low level and the maximum.
 */
static bool ref_threshold(uint16_t black, uint16_t start_low,
                          uint16_t current, uint16_t *p_threshold)
{
    bool ok = (black > current) && (black >= start_low)
              && (((uint32_t)black - start_low) >= LINE_CAL_MIN_SPAN);

    if (ok)
    {
        *p_threshold = (uint16_t)((uint32_t)start_low
                                  + (((uint32_t)black - start_low) / 2U));
    }

    return ok;
}


void line_cal_begin(void)
{
    cal_state = LINE_CAL_RUNNING;
    ref_active = false;
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
 * Is any history sample from index begin up to (not including) end blind?
 * These samples lie between the two black stretches, so both sensors are
 * below their thresholds. A sample is blind when the right sensor does not
 * see the line (see right_sees_line), whatever the centre reads: the same
 * test both_white_side() needs to tell which side the line left by.
 */
static bool cal_gap_blind(uint32_t begin, uint32_t end, uint32_t oldest)
{
    uint32_t k;
    cal_sample_t sample;
    bool blind = false;

    for (k = begin; k < end; k++)
    {
        sample = cal_history[(oldest + k) & LINE_CAL_HISTORY_MASK];

        if (!right_sees_line(sample.right))
        {
            blind = true;
        }
    }

    return blind;
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
 * A gap only matters if it is blind (see cal_gap_blind). *p_blind is set
 * when the samples between the stretches include a blind one.
 *
 * Returns false (not measured) when the data cannot show a gap: a sensor
 * never above its threshold, or a negative overlap while the ring has
 * dropped older samples and a stretch starts at its oldest sample, which
 * could have started earlier and overlapped.
 */
static bool cal_measure_overlap(uint16_t c_thr, uint16_t r_thr,
                                int16_t *p_overlap, bool *p_blind)
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

    *p_blind = false;

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

        if (measured && (overlap < 0))
        {
            *p_blind = cal_gap_blind((uint32_t)last + 1U, (uint32_t)first,
                                     oldest);
        }
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
        uint16_t new_centre;
        uint16_t new_right;
        int16_t overlap = 0;
        bool blind = false;

        /* The sweep ends on the smoothed values; the levels are the raw
         * ones. */
        cal_use_raw_range(&cal_centre, true);
        cal_use_raw_range(&cal_right, false);
        new_centre = cal_midpoint(&cal_centre);
        new_right = cal_midpoint(&cal_right);

        cal_record_ranges();
        cal_result.centre.threshold = new_centre;
        cal_result.right.threshold = new_right;

        /* Each span is at least LINE_CAL_MIN_SPAN, so the full-black level
         * stays above the centre threshold. */
        cal_result.centre_full_black = (uint16_t)((uint32_t)cal_centre.high
                                                  - (cal_span(&cal_centre)
                                                     / LINE_CAL_FULL_BLACK_DIV));

        cal_result.gap_measured = cal_measure_overlap(new_centre, new_right,
                                                      &overlap, &blind);
        cal_result.gap_overlap = overlap;

        /* The barcode sensor may sit beside the line and never see black.
         * Without enough contrast it keeps the caller's fallback. */
        if (cal_span(&cal_barcode) >= LINE_CAL_MIN_SPAN)
        {
            cal_result.barcode.threshold = cal_midpoint(&cal_barcode);
        }

        if (cal_result.gap_measured && (overlap < 0) && blind)
        {
            /* A blind gap between the sensors: the thresholds stay as
             * they were, the result keeps what was measured. */
            cal_state = LINE_CAL_FAIL_GAP;
            return;
        }

        centre_threshold = new_centre;
        right_threshold = new_right;
        centre_full_black = cal_result.centre_full_black;
        clear_line_state();
        cal_state = LINE_CAL_OK;
        ref_seed();
    }
}


void line_ref_update(ir_readings_t readings, int32_t travel_counts)
{
    uint32_t bucket_counts;
    uint32_t travel;
    uint32_t k;
    uint16_t c_max = 0U;
    uint16_t r_max = 0U;
    uint16_t threshold;
    uint16_t full_black;
    ref_bucket_t *p_bucket;

    if ((cal_state != LINE_CAL_OK) || !ref_active)
    {
        return;
    }

    bucket_counts = ref_get_bucket_counts();
    travel = (travel_counts > 0) ? (uint32_t)travel_counts : 0U;

    /* A jump longer than the window forgets the whole window anyway. */
    if (travel > (bucket_counts * LINE_REF_BUCKETS))
    {
        travel = bucket_counts * LINE_REF_BUCKETS;
    }

    /* Move to a new bucket for each bucket length of forward travel. */
    ref_travel += travel;

    while (ref_travel >= bucket_counts)
    {
        ref_travel -= bucket_counts;
        ref_head = (ref_head + 1U) % LINE_REF_BUCKETS;
        ref_clear_bucket(&ref_bucket[ref_head]);
        ref_advances++;
    }

    /* Standing or pivoting adds samples here but never advances. */
    p_bucket = &ref_bucket[ref_head];

    if (readings.centre > p_bucket->centre_max)
    {
        p_bucket->centre_max = readings.centre;
    }

    if (readings.right > p_bucket->right_max)
    {
        p_bucket->right_max = readings.right;
    }

    /* Until the seed bucket has rolled out, the START levels stay. */
    if (ref_advances < LINE_REF_BUCKETS)
    {
        return;
    }

    for (k = 0U; k < LINE_REF_BUCKETS; k++)
    {
        if (ref_bucket[k].centre_max > c_max)
        {
            c_max = ref_bucket[k].centre_max;
        }

        if (ref_bucket[k].right_max > r_max)
        {
            r_max = ref_bucket[k].right_max;
        }
    }

    if (ref_threshold(c_max, cal_result.centre.low, centre_threshold,
                      &threshold))
    {
        /* The error division relies on full black above the threshold. */
        full_black = (uint16_t)((uint32_t)c_max
                                - (((uint32_t)c_max - cal_result.centre.low)
                                   / LINE_CAL_FULL_BLACK_DIV));

        if (full_black > threshold)
        {
            centre_threshold = threshold;
            centre_full_black = full_black;
        }
    }

    if (ref_threshold(r_max, cal_result.right.low, right_threshold,
                      &threshold))
    {
        right_threshold = threshold;
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
