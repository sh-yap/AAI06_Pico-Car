/** @file test_line_follow.c
 *
 * @brief Host tests for app_program/line_follow.c (Buddy 3).
 *
 * Run with:  make -C tests/buddy3 test
 *
 * Readings use the 2026-10-29 car values: white ~200, centre on the
 * line ~2300, right on the line ~3800.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "line_follow.h"

#define WHITE          200U
#define CENTRE_ON      2300U    /* IR_CENTRE_FULL_BLACK */
#define CENTRE_EDGE    1600U    /* just above IR_CENTRE_THRESHOLD */
#define RIGHT_ON       3800U

static uint32_t failures = 0U;
static uint32_t checks = 0U;


/* ir_sensor.c needs the ADC; line_follow.c only uses this. */
bool
ir_sensor_is_black(uint16_t reading, uint16_t threshold)
{
    return (reading >= threshold);
}


static void
check(char const * p_name, bool ok)
{
    checks++;

    if (!ok)
    {
        failures++;
        (void)printf("FAIL %s\n", p_name);
    }
}


/* One 10 ms Buddy 3 cycle: state first, then error (as buddy3_task.c). */
static int8_t
step(uint16_t centre, uint16_t right, line_state_t * p_state)
{
    ir_readings_t r = { centre, right, WHITE };

    *p_state = line_get_position(r);
    return line_get_error(r);
}


static void
test_centred_and_left_drift(void)
{
    line_state_t state;
    int8_t prev;
    int8_t err;
    uint16_t c;

    line_follow_init();

    err = step(CENTRE_ON, WHITE, &state);
    check("centred: CENTRE", state == LINE_CENTRE);
    check("centred: error 0", err == 0);

    err = step(3000U, WHITE, &state);
    check("darker than full: error 0", err == 0);

    /* Line sliding off the centre sensor to the left: gradual, not -127. */
    err = step(2100U, WHITE, &state);
    check("slight left drift: CENTRE", state == LINE_CENTRE);
    check("slight left drift: small left error", (err < 0) && (err > -60));

    prev = 0;
    for (c = CENTRE_ON; c > CENTRE_EDGE; c -= 50U)
    {
        err = step(c, WHITE, &state);
        check("left error grows as centre drops", err <= prev);
        prev = err;
    }

    err = step(CENTRE_EDGE, WHITE, &state);
    check("centre at its edge: near full left", err < -110);

    /* Both white after centre: line went left, full left until lost. */
    err = step(WHITE, WHITE, &state);
    check("both white after centre: LEFT", state == LINE_LEFT);
    check("both white after centre: -127", err == -LINE_ERROR_MAX);
}


static void
test_right_drift(void)
{
    line_state_t state;
    int8_t err;

    line_follow_init();

    err = step(CENTRE_ON, 1800U, &state);
    check("right just sees the line: small right error",
          (err > 0) && (err < 40));

    err = step(WHITE, RIGHT_ON, &state);
    check("only right on the line: RIGHT", state == LINE_RIGHT);
    check("only right on the line: +127", err == LINE_ERROR_MAX);

    err = step(WHITE, WHITE, &state);
    check("both white after right: +127", err == LINE_ERROR_MAX);
}


static void
test_lost(void)
{
    line_state_t state = LINE_CENTRE;
    int8_t err = 0;
    uint32_t i;

    line_follow_init();
    (void)step(CENTRE_ON, WHITE, &state);

    for (i = 0U; i < 30U; i++)
    {
        err = step(WHITE, WHITE, &state);
    }

    check("white for 30 cycles: LOST", state == LINE_LOST);
    check("LOST: error 0", err == 0);
}


static void
test_junction(void)
{
    line_state_t state = LINE_CENTRE;
    uint32_t i;

    line_follow_init();
    (void)step(CENTRE_ON, WHITE, &state);

    (void)step(CENTRE_ON, RIGHT_ON, &state);
    (void)step(CENTRE_ON, RIGHT_ON, &state);
    check("both black, 2 cycles: not yet a junction",
          state != LINE_JUNCTION);

    (void)step(CENTRE_ON, RIGHT_ON, &state);
    check("both black, 3 cycles: JUNCTION", state == LINE_JUNCTION);

    /* Every cycle while over it, so the VC never misses it. */
    for (i = 0U; i < 10U; i++)
    {
        (void)step(CENTRE_ON, RIGHT_ON, &state);
    }
    check("still both black: still JUNCTION", state == LINE_JUNCTION);

    (void)step(CENTRE_ON, WHITE, &state);
    check("right off the crossing: CENTRE", state == LINE_CENTRE);

    (void)step(CENTRE_ON, RIGHT_ON, &state);
    check("both black again: debounced afresh", state != LINE_JUNCTION);
}


/*
 * START calibration gap check. A synthetic sweep: both sensors read WHITE,
 * except BLACK_LEVEL inside their own stretch of samples. The sweep starts
 * moving at sample 0 (line_cal_sweep() is called after line_cal_begin()).
 */
#define BLACK_LEVEL    3800U
#define CAL_TAIL       40U

typedef struct
{
    uint32_t centre_first;
    uint32_t centre_end;       /* One past the last black sample. */
    uint32_t right_first;
    uint32_t right_end;
} stretches_t;


static bool
in_stretch(uint32_t i, uint32_t first, uint32_t end)
{
    return (i >= first) && (i < end);
}


/* Run a sweep through line_cal_update until it stops. Returns the state. */
static line_cal_state_t
run_sweep(stretches_t const *p_s)
{
    uint32_t last = (p_s->centre_end > p_s->right_end) ? p_s->centre_end
                                                       : p_s->right_end;
    uint32_t total = last + CAL_TAIL;
    uint32_t i;
    ir_readings_t readings;

    line_cal_begin();
    line_cal_sweep();

    for (i = 0U; (i < total) && (line_cal_state() == LINE_CAL_RUNNING); i++)
    {
        readings.centre = in_stretch(i, p_s->centre_first, p_s->centre_end)
                          ? BLACK_LEVEL : WHITE;
        readings.right = in_stretch(i, p_s->right_first, p_s->right_end)
                         ? BLACK_LEVEL : WHITE;
        readings.barcode = WHITE;
        line_cal_update(readings);
    }

    return line_cal_state();
}


static void
test_calibration_gap(void)
{
    line_cal_result_t result;
    line_cal_state_t state;
    ir_readings_t between = { 1800U, 1800U, WHITE };
    stretches_t s;

    /* Right crosses first, the centre sensor overlaps it by 10 samples. */
    line_follow_init();
    s = (stretches_t){ 40U, 60U, 30U, 50U };
    state = run_sweep(&s);
    line_cal_get_result(&result);
    check("overlap: calibration OK", state == LINE_CAL_OK);
    check("overlap: measured", result.gap_measured);
    check("overlap: 10 samples", result.gap_overlap == 10);
    check("overlap: learned threshold used (1800 is white)",
          (line_get_sensor_bits(between) & LINE_SENSOR_CENTRE) == 0U);

    /* Stretches that only touch: no sample with both white between. */
    line_follow_init();
    s = (stretches_t){ 50U, 70U, 30U, 50U };
    state = run_sweep(&s);
    line_cal_get_result(&result);
    check("touching: calibration OK", state == LINE_CAL_OK);
    check("touching: overlap 0", result.gap_measured
                                 && (result.gap_overlap == 0));

    /* One white sample between them is already a gap. */
    line_follow_init();
    s = (stretches_t){ 51U, 71U, 30U, 50U };
    state = run_sweep(&s);
    line_cal_get_result(&result);
    check("1 sample gap: FAIL_GAP", state == LINE_CAL_FAIL_GAP);
    check("1 sample gap: reported as -1", result.gap_measured
                                          && (result.gap_overlap == -1));

    /* Right first, wide gap. Thresholds stay as they were: 1800 is still
     * black for the default centre threshold. */
    line_follow_init();
    s = (stretches_t){ 56U, 76U, 30U, 50U };
    state = run_sweep(&s);
    line_cal_get_result(&result);
    check("right first, gap: FAIL_GAP", state == LINE_CAL_FAIL_GAP);
    check("right first, gap: -6", result.gap_measured
                                  && (result.gap_overlap == -6));
    check("gap: live thresholds unchanged",
          (line_get_sensor_bits(between) & LINE_SENSOR_CENTRE) != 0U);

    /* Centre first (the other way round): same answer. */
    line_follow_init();
    s = (stretches_t){ 30U, 50U, 56U, 76U };
    state = run_sweep(&s);
    line_cal_get_result(&result);
    check("centre first, gap: FAIL_GAP", state == LINE_CAL_FAIL_GAP);
    check("centre first, gap: -6", result.gap_measured
                                   && (result.gap_overlap == -6));

    line_follow_init();
    s = (stretches_t){ 30U, 50U, 40U, 60U };
    state = run_sweep(&s);
    line_cal_get_result(&result);
    check("centre first, overlap: OK with 10",
          (state == LINE_CAL_OK) && (result.gap_overlap == 10));

    /* Second side: the first pass saw only a short edge of the right
     * sensor's stretch (cut by the bound), the sweep came back and crossed
     * the line the other way. */
    line_follow_init();
    s = (stretches_t){ 106U, 126U, 30U, 100U };
    state = run_sweep(&s);
    line_cal_get_result(&result);
    check("second side, gap: FAIL_GAP", state == LINE_CAL_FAIL_GAP);
    check("second side, gap: measured", result.gap_measured);

    line_follow_init();
    s = (stretches_t){ 90U, 110U, 30U, 100U };
    state = run_sweep(&s);
    line_cal_get_result(&result);
    check("second side, overlap: OK", state == LINE_CAL_OK);
}


static void
test_calibration_not_measured(void)
{
    line_cal_result_t result;
    line_cal_state_t state;
    stretches_t s;

    /* The centre stretch is black from the oldest sample the ring still
     * holds, so it may have started earlier and overlapped the right one.
     * The gap cannot be shown, so the sweep must not fail for it. */
    line_follow_init();
    s = (stretches_t){ 10U, 280U, 300U, 320U };
    state = run_sweep(&s);
    line_cal_get_result(&result);
    check("cut off at the window start: not failed for a gap",
          state == LINE_CAL_OK);
    check("cut off at the window start: not measured",
          !result.gap_measured);

    /* The same shape that fits in the ring is measured and fails. */
    line_follow_init();
    s = (stretches_t){ 10U, 100U, 120U, 140U };
    state = run_sweep(&s);
    line_cal_get_result(&result);
    check("same gap, nothing dropped: FAIL_GAP", state == LINE_CAL_FAIL_GAP);
    check("same gap, nothing dropped: -20", result.gap_measured
                                           && (result.gap_overlap == -20));
}


/*
 * Learned levels and gaps from raw readings. A sweep of 120 samples built
 * from per-sensor levels, then run until the calibration stops.
 */
#define RAW_SAMPLES    120U
#define RAW_LOW        170U

static line_cal_state_t
run_raw(uint16_t const *p_centre, uint16_t const *p_right)
{
    uint32_t i;
    ir_readings_t r;

    line_follow_init();
    line_cal_begin();
    line_cal_sweep();

    for (i = 0U; (i < RAW_SAMPLES) && (line_cal_state() == LINE_CAL_RUNNING);
         i++)
    {
        r = (ir_readings_t){ p_centre[i], p_right[i], WHITE };
        line_cal_update(r);
    }

    return line_cal_state();
}


static void
raw_fill(uint16_t *p_levels, uint16_t value)
{
    uint32_t i;

    for (i = 0U; i < RAW_SAMPLES; i++)
    {
        p_levels[i] = value;
    }
}


/* A short centre peak: the smoothed value reaches only 60 percent of it. */
static void
test_calibration_raw_peak(void)
{
    line_cal_result_t result;
    uint16_t centre[RAW_SAMPLES];
    uint16_t right[RAW_SAMPLES];
    uint32_t i;

    raw_fill(centre, RAW_LOW);
    raw_fill(right, RAW_LOW);

    for (i = 30U; i < 33U; i++)
    {
        centre[i] = 2000U;
    }

    for (i = 28U; i < 45U; i++)
    {
        right[i] = 3000U;
    }

    check("raw peak: OK", run_raw(centre, right) == LINE_CAL_OK);
    line_cal_get_result(&result);
    check("raw peak: high within 20 of the 3-sample peak (old code ~1230)",
          (result.centre.high >= 1980U) && (result.centre.high <= 2020U));
    check("raw peak: low is the white level",
          result.centre.low == RAW_LOW);
    check("raw peak: threshold is the midpoint",
          result.centre.threshold == (RAW_LOW + ((2000U - RAW_LOW) / 2U)));
    check("raw peak: full black below the high",
          (result.centre_full_black < 2000U)
          && (result.centre_full_black > result.centre.threshold));

    /* One spike far above the line does not set the high. */
    raw_fill(centre, RAW_LOW);

    for (i = 30U; i < 34U; i++)
    {
        centre[i] = 1500U;
    }

    centre[10] = 3000U;
    check("spike: OK", run_raw(centre, right) == LINE_CAL_OK);
    line_cal_get_result(&result);
    check("spike: high is the line, not the spike",
          (result.centre.high >= 1480U) && (result.centre.high <= 1520U));

    /* A single stray low sample does not set the low either. */
    raw_fill(centre, 500U);
    centre[3] = 100U;

    for (i = 30U; i < 34U; i++)
    {
        centre[i] = 2000U;
    }

    check("low spike: OK", run_raw(centre, right) == LINE_CAL_OK);
    line_cal_get_result(&result);
    check("low spike: low is the white level",
          result.centre.low == 500U);
}


/*
 * Centre black 30..49, right black 56..75, a gap of 6 samples (50..55)
 * with both below their thresholds. BLIND_NONE means no sample of the gap
 * is made blind.
 */
#define BLIND_NONE    0U

static line_cal_state_t
run_gap_levels(uint16_t centre_gap, uint16_t right_gap, uint32_t blind_at,
               line_cal_result_t *p_result)
{
    uint16_t centre[RAW_SAMPLES];
    uint16_t right[RAW_SAMPLES];
    line_cal_state_t state;
    uint32_t i;

    raw_fill(centre, RAW_LOW);
    raw_fill(right, RAW_LOW);

    for (i = 30U; i < 50U; i++)
    {
        centre[i] = 3800U;
    }

    for (i = 56U; i < 76U; i++)
    {
        right[i] = 3800U;
    }

    for (i = 50U; i < 56U; i++)
    {
        centre[i] = centre_gap;
        right[i] = right_gap;
    }

    if (blind_at != BLIND_NONE)
    {
        centre[blind_at] = RAW_LOW;
        right[blind_at] = RAW_LOW;
    }

    state = run_raw(centre, right);
    line_cal_get_result(p_result);
    return state;
}


static void
test_calibration_blind_gap(void)
{
    line_cal_result_t result;
    line_cal_state_t state;

    /* The right sensor stays at 1000, well above span/32 (about 113),
     * through every gap sample but still below its threshold (1985). */
    state = run_gap_levels(RAW_LOW, 1000U, BLIND_NONE, &result);
    check("partial gap, right: OK", state == LINE_CAL_OK);
    check("partial gap, right: measured as -6", result.gap_measured
                                                && (result.gap_overlap == -6));

    /* Only the centre stays above its margin: the right is at its low, so
     * the side is not known and the gap is blind (centre does not count). */
    state = run_gap_levels(1000U, RAW_LOW, BLIND_NONE, &result);
    check("partial gap, centre only: FAIL_GAP", state == LINE_CAL_FAIL_GAP);
    check("partial gap, centre only: measured as -6", result.gap_measured
                                                      && (result.gap_overlap == -6));

    /* A rise just inside the margin (span/32 = 113) is still blind. */
    state = run_gap_levels(RAW_LOW + 100U, RAW_LOW + 100U, BLIND_NONE,
                           &result);
    check("gap inside the margin: FAIL_GAP", state == LINE_CAL_FAIL_GAP);

    /* Both at their lows for the whole gap. */
    state = run_gap_levels(RAW_LOW, RAW_LOW, BLIND_NONE, &result);
    check("blind gap: FAIL_GAP", state == LINE_CAL_FAIL_GAP);
    check("blind gap: measured as -6", result.gap_measured
                                       && (result.gap_overlap == -6));

    /* One blind sample (right at its low) among partial ones is enough. */
    state = run_gap_levels(1000U, 1000U, 53U, &result);
    check("one blind sample in the gap: FAIL_GAP",
          state == LINE_CAL_FAIL_GAP);
}


/* Stretches that overlap, and a sweep with no contrast, behave as before. */
static void
test_calibration_overlap_and_contrast(void)
{
    uint16_t centre[RAW_SAMPLES];
    uint16_t right[RAW_SAMPLES];
    uint32_t i;

    raw_fill(centre, RAW_LOW);
    raw_fill(right, RAW_LOW);

    for (i = 30U; i < 50U; i++)
    {
        centre[i] = 3800U;
    }

    for (i = 40U; i < 60U; i++)
    {
        right[i] = 3800U;
    }

    check("overlap: OK", run_raw(centre, right) == LINE_CAL_OK);

    raw_fill(centre, RAW_LOW);
    raw_fill(right, RAW_LOW);
    (void)run_raw(centre, right);
    check("no contrast: still running", line_cal_state() == LINE_CAL_RUNNING);
    check("no contrast: FAIL_CONTRAST",
          line_cal_end() == LINE_CAL_FAIL_CONTRAST);
}


/*
 * Levels come from moving samples only. The centre sensor rests on the
 * line while the car is still and reads 3800, higher than the 2300 it
 * reads moving. The threshold must be the midpoint of the moving
 * readings (1250), not of the still peak (2000).
 */
static void
test_calibration_moving_only(void)
{
    line_cal_state_t state;
    ir_readings_t r;
    ir_readings_t probe_a = { 1500U, 200U, WHITE };
    ir_readings_t probe_b = { 200U, 3000U, WHITE };
    uint32_t i;

    line_follow_init();
    line_cal_begin();

    for (i = 0U; i < 40U; i++)           /* Still, centre on the line. */
    {
        r = (ir_readings_t){ 3800U, WHITE, WHITE };
        line_cal_update(r);
    }

    check("still: nothing recorded", line_cal_state() == LINE_CAL_RUNNING);
    line_cal_sweep();

    for (i = 0U; (i < 200U) && (line_cal_state() == LINE_CAL_RUNNING); i++)
    {
        r.centre  = ((i >= 20U) && (i < 60U)) ? CENTRE_ON : WHITE;
        r.right   = ((i >= 40U) && (i < 80U)) ? RIGHT_ON : WHITE;
        r.barcode = WHITE;
        line_cal_update(r);
    }

    state = line_cal_state();
    check("moving only: OK", state == LINE_CAL_OK);
    check("moving only: centre threshold from moving levels",
          (line_get_sensor_bits(probe_a) & LINE_SENSOR_CENTRE) != 0U);
    check("moving only: right threshold",
          (line_get_sensor_bits(probe_b) & LINE_SENSOR_RIGHT) != 0U);

    /* The gap history starts with the sweep too: the 40 still black
     * centre samples must not count as an overlap with the right stretch
     * (centre black 20..60 and right 40..80 overlap by 20 moving). */
    {
        line_cal_result_t result;

        line_cal_get_result(&result);
        check("moving only: overlap 20", result.gap_measured
                                         && (result.gap_overlap == 20));
    }

    /* Right first, and the right sensor starts the sweep on the line:
     * its black level is still captured by the first moving sample. */
    line_follow_init();
    line_cal_begin();
    line_cal_sweep();

    for (i = 0U; (i < 200U) && (line_cal_state() == LINE_CAL_RUNNING); i++)
    {
        r.right   = (i < 40U) ? RIGHT_ON : WHITE;
        r.centre  = ((i >= 30U) && (i < 70U)) ? CENTRE_ON : WHITE;
        r.barcode = WHITE;
        line_cal_update(r);
    }

    check("right first, starts black: OK", line_cal_state() == LINE_CAL_OK);
    check("right first, starts black: right threshold",
          (line_get_sensor_bits(probe_b) & LINE_SENSOR_RIGHT) != 0U);
}


/*
 * Both sensors white: if the right sensor is above its learned low by more
 * than span/32 the side is RIGHT, otherwise the last-seen side stays. The
 * centre reading decides nothing. The sweep uses the 2026-11-02
 * bracket-height levels: centre 171 to 1270, right 240 to 2981, black zones
 * overlapping by 10 samples.
 */
#define GAP_CENTRE_LOW     171U
#define GAP_CENTRE_HIGH    1270U
#define GAP_RIGHT_LOW      240U
#define GAP_RIGHT_HIGH     2981U

static void
learn_levels(uint16_t centre_low, uint16_t centre_high, uint16_t right_low,
             uint16_t right_high)
{
    ir_readings_t r;
    uint32_t i;

    line_follow_init();
    line_cal_begin();
    line_cal_sweep();

    for (i = 0U; (i < 200U) && (line_cal_state() == LINE_CAL_RUNNING); i++)
    {
        r.right = in_stretch(i, 30U, 60U) ? right_high : right_low;
        r.centre = in_stretch(i, 50U, 80U) ? centre_high : centre_low;
        r.barcode = WHITE;
        line_cal_update(r);
    }
}


static void
learn_bracket_height(void)
{
    learn_levels(GAP_CENTRE_LOW, GAP_CENTRE_HIGH, GAP_RIGHT_LOW,
                 GAP_RIGHT_HIGH);
}


static void
test_both_white_side(void)
{
    line_state_t state;
    line_cal_result_t result;
    int8_t err;
    uint32_t i;

    learn_bracket_height();
    line_cal_get_result(&result);
    check("gap side: calibration OK", line_cal_state() == LINE_CAL_OK);
    check("gap side: learned centre range",
          (result.centre.low <= (GAP_CENTRE_LOW + 4U))
          && (result.centre.high >= (GAP_CENTRE_HIGH - 4U)));
    check("gap side: learned right range",
          (result.right.low <= (GAP_RIGHT_LOW + 4U))
          && (result.right.high >= (GAP_RIGHT_HIGH - 4U)));

    /* Centre only, then the line moves right through the gap. */
    (void)step(GAP_CENTRE_HIGH, GAP_RIGHT_LOW, &state);
    check("gap side: centre only reads CENTRE", state == LINE_CENTRE);
    err = step(538U, 1416U, &state);
    check("gap side: right at half range: RIGHT", state == LINE_RIGHT);
    check("gap side: right at half range: error > 0", err > 0);
    err = step(557U, 1453U, &state);
    check("gap side: right slightly higher: RIGHT", state == LINE_RIGHT);
    check("gap side: right slightly higher: error > 0", err > 0);

    /* Right only, then the centre part on with the right at its low: the
     * right was last seen, and the centre alone does not change that. */
    (void)step(GAP_CENTRE_LOW, GAP_RIGHT_HIGH, &state);
    check("gap side: right only reads RIGHT", state == LINE_RIGHT);
    err = step(392U, 209U, &state);
    check("gap side: centre at a quarter after right: RIGHT",
          state == LINE_RIGHT);
    check("gap side: centre at a quarter after right: error > 0", err > 0);
    err = step(445U, 201U, &state);
    check("gap side: centre at a third after right: RIGHT",
          state == LINE_RIGHT);
    check("gap side: centre at a third after right: error > 0", err > 0);

    /* Centre only, then centre part on with the right at its low: the
     * centre alone does not choose, so last-seen (centre, reported LEFT)
     * stays. */
    (void)step(GAP_CENTRE_HIGH, GAP_RIGHT_LOW, &state);
    err = step(445U, 201U, &state);
    check("gap side: centre partial after centre: LEFT",
          state == LINE_LEFT);
    check("gap side: centre partial after centre: error < 0", err < 0);

    /* Both at their learned lows: neither leads, keep the last-seen side. */
    (void)step(GAP_CENTRE_HIGH, GAP_RIGHT_LOW, &state);
    err = step(GAP_CENTRE_LOW, GAP_RIGHT_LOW, &state);
    check("gap side: both low after centre: LEFT", state == LINE_LEFT);
    check("gap side: both low after centre: error < 0", err < 0);
    (void)step(GAP_CENTRE_LOW, GAP_RIGHT_HIGH, &state);
    err = step(GAP_CENTRE_LOW, GAP_RIGHT_LOW, &state);
    check("gap side: both low after right: RIGHT", state == LINE_RIGHT);
    check("gap side: both low after right: error > 0", err > 0);

    /* The chosen side sticks while the readings stay level. */
    (void)step(GAP_CENTRE_HIGH, GAP_RIGHT_LOW, &state);
    (void)step(538U, 1416U, &state);
    err = step(GAP_CENTRE_LOW, GAP_RIGHT_LOW, &state);
    check("gap side: chosen RIGHT is kept", (state == LINE_RIGHT)
                                            && (err > 0));

    /* A reading below the learned low counts as zero, not a wrap. */
    (void)step(GAP_CENTRE_HIGH, GAP_RIGHT_LOW, &state);
    (void)step(100U, 100U, &state);
    check("gap side: below both lows: CENTRE side kept (LEFT)",
          state == LINE_LEFT);
    (void)step(GAP_CENTRE_LOW, GAP_RIGHT_HIGH, &state);
    (void)step(100U, 100U, &state);
    check("gap side: below both lows: RIGHT side kept", state == LINE_RIGHT);

    /* LOST still comes after 30 white samples. */
    (void)step(GAP_CENTRE_HIGH, GAP_RIGHT_LOW, &state);
    for (i = 0U; i < 30U; i++)
    {
        err = step(538U, 1416U, &state);
    }

    check("gap side: 30 white samples: LOST", state == LINE_LOST);
    check("gap side: LOST: error 0", err == 0);

    /* Ranges from the car_cal1 hardware log: the right sensor sees part of
     * the line while both are below threshold. */
    learn_levels(182U, 2318U, 224U, 3572U);
    check("gap side: car_cal1 calibration OK", line_cal_state() == LINE_CAL_OK);
    (void)step(2318U, 224U, &state);
    err = step(1176U, 717U, &state);
    check("gap side: car_cal1 (1176, 717): RIGHT",
          (state == LINE_RIGHT) && (err > 0));
    (void)step(2318U, 224U, &state);
    err = step(965U, 1045U, &state);
    check("gap side: car_cal1 (965, 1045): RIGHT",
          (state == LINE_RIGHT) && (err > 0));
}


/* Before a good sweep the last-seen side rules, whatever the readings. */
static void
test_both_white_side_uncalibrated(void)
{
    line_state_t state;
    int8_t err;

    line_follow_init();
    (void)step(CENTRE_ON, WHITE, &state);
    err = step(300U, 1500U, &state);
    check("uncalibrated: centre then lower right: LEFT",
          (state == LINE_LEFT) && (err == -LINE_ERROR_MAX));

    (void)step(WHITE, RIGHT_ON, &state);
    err = step(1500U, 300U, &state);
    check("uncalibrated: right then lower centre: RIGHT",
          (state == LINE_RIGHT) && (err == LINE_ERROR_MAX));

    /* A failed sweep (white gap between the sensors) learns nothing. */
    line_follow_init();
    {
        stretches_t s = { 56U, 76U, 30U, 50U };

        check("failed sweep: FAIL_GAP", run_sweep(&s) == LINE_CAL_FAIL_GAP);
    }

    (void)step(1700U, WHITE, &state);
    err = step(WHITE, 1500U, &state);
    check("failed sweep: last-seen side kept (LEFT)",
          (state == LINE_LEFT) && (err == -LINE_ERROR_MAX));
}


/*
 * Distance-windowed references. A calibrated START (both sensors 200 to
 * 3800), then the car drives in steps of REF_STEP counts. A bucket is about
 * 306 counts and the window six of them, so REF_WINDOW_CYCLES cycles cover
 * more than one whole window.
 */
#define REF_STEP             10
#define REF_WINDOW_CYCLES    200U
#define REF_PROBE            2600U    /* Between threshold and full black. */

static void
ref_calibrate(void)
{
    stretches_t s = { 40U, 60U, 30U, 50U };

    line_follow_init();
    (void)run_sweep(&s);
}


/* One driven cycle: references first, then the error as the task does. */
static int8_t
ref_cycle(uint16_t centre, int32_t travel)
{
    ir_readings_t r = { centre, WHITE, WHITE };
    line_state_t state;

    line_ref_update(r, travel);
    return step(centre, WHITE, &state);
}


/* The error for a fixed reading, without feeding it to the references. */
static int8_t
ref_probe(void)
{
    line_state_t state;

    return step(REF_PROBE, WHITE, &state);
}


/* One window of weaving: the centre sensor reaches peak half the time. */
static void
ref_drive_window(uint16_t peak, int32_t travel)
{
    uint32_t k;

    for (k = 0U; k < REF_WINDOW_CYCLES; k++)
    {
        (void)ref_cycle(((k % 20U) < 10U) ? peak : WHITE, travel);
    }
}


static void
test_ref_slow_fade(void)
{
    uint32_t w;
    uint16_t peak = BLACK_LEVEL;
    int8_t err = 0;
    line_state_t state = LINE_LOST;
    ir_readings_t at_peak;

    ref_calibrate();
    check("ref: calibrated", line_cal_state() == LINE_CAL_OK);

    for (w = 0U; w < 14U; w++)
    {
        peak = (uint16_t)(BLACK_LEVEL - ((w * (BLACK_LEVEL * 3U / 10U)) / 13U));
        ref_drive_window(peak, REF_STEP);
        err = ref_cycle(peak, REF_STEP);
    }

    at_peak = (ir_readings_t){ peak, WHITE, WHITE };
    state = line_get_position(at_peak);
    check("ref: faded to 70 percent", peak <= ((BLACK_LEVEL * 71U) / 100U));
    check("ref: slow fade, error 0 at local peak", err == 0);
    check("ref: slow fade, still on the line", state == LINE_CENTRE);
}


static void
test_ref_quick_slide_off(void)
{
    uint32_t k;
    uint16_t centre;
    int8_t before;
    int8_t after;
    bool all_left = true;

    ref_calibrate();
    ref_drive_window(BLACK_LEVEL, REF_STEP);
    ref_drive_window(BLACK_LEVEL, REF_STEP);
    before = ref_probe();
    check("ref: slide, probe reads left of centre", before < 0);

    /* Within about one bucket the centre slides from the peak to white. */
    for (k = 0U; k < 40U; k++)
    {
        centre = (uint16_t)(BLACK_LEVEL - ((k * (BLACK_LEVEL - WHITE)) / 39U));

        if (ref_cycle(centre, 5) > 0)
        {
            all_left = false;
        }
    }

    after = ref_probe();
    check("ref: slide, error never turns right", all_left);
    check("ref: slide, reference not lowered", after == before);
}


static void
test_ref_centred_no_white(void)
{
    uint32_t k;
    uint32_t w;
    uint16_t level = 2800U;
    uint16_t low_on = (uint16_t)((level * 7U) / 10U);
    ir_readings_t r;
    bool all_on = true;

    ref_calibrate();

    /* The first window only rolls out the START seed. Centred for several
     * windows: readings only 70 to 100 percent of the
     * tape's black, never near white. */
    for (w = 0U; w < 5U; w++)
    {
        for (k = 0U; k < REF_WINDOW_CYCLES; k++)
        {
            r.centre = ((k % 20U) < 10U) ? level : low_on;
            r.right = WHITE;
            r.barcode = WHITE;
            line_ref_update(r, REF_STEP);

            if ((w > 0U)
                && ((line_get_sensor_bits(r) & LINE_SENSOR_CENTRE) == 0U))
            {
                all_on = false;
            }
        }
    }

    check("ref: no white in window, on-tape readings stay on the line",
          all_on);

    /* The threshold is the midpoint of the START low (200) and the window
     * maximum, so a reading just above it is black and just below is not. */
    r = (ir_readings_t){ 1500U, WHITE, WHITE };
    check("ref: threshold follows window max (1500 is black)",
          (line_get_sensor_bits(r) & LINE_SENSOR_CENTRE) != 0U);
    r = (ir_readings_t){ 1499U, WHITE, WHITE };
    check("ref: threshold not above half of window max (1499 is white)",
          (line_get_sensor_bits(r) & LINE_SENSOR_CENTRE) == 0U);
}


static void
test_ref_no_tape_or_stationary(void)
{
    uint32_t k;
    int8_t before;

    ref_calibrate();
    ref_drive_window(BLACK_LEVEL, REF_STEP);
    ref_drive_window(BLACK_LEVEL, REF_STEP);
    before = ref_probe();

    /* Standing still on white forever: the window never moves. */
    for (k = 0U; k < 5000U; k++)
    {
        (void)ref_cycle(WHITE, 0);
    }

    check("ref: stationary on white keeps thresholds",
          ref_probe() == before);

    /* A pivot in place with the line seen now and then: same. */
    for (k = 0U; k < 5000U; k++)
    {
        (void)ref_cycle(((k % 50U) < 25U) ? BLACK_LEVEL : WHITE, 0);
    }

    check("ref: pivoting keeps thresholds",
          ref_probe() == before);

    /* Driving over white for many windows: no span, nothing accepted. */
    for (k = 0U; k < 8U; k++)
    {
        ref_drive_window(WHITE, REF_STEP);
    }

    check("ref: no tape in window keeps thresholds",
          ref_probe() == before);

    /* Negative travel is ignored, not counted as distance. */
    for (k = 0U; k < 5000U; k++)
    {
        (void)ref_cycle(WHITE, -50);
    }

    check("ref: negative travel keeps thresholds",
          ref_probe() == before);
}


static void
test_ref_reset_on_recalibration(void)
{
    stretches_t s = { 40U, 60U, 30U, 50U };
    uint32_t w;
    int8_t fresh;
    int8_t faded;

    ref_calibrate();
    fresh = ref_probe();

    for (w = 0U; w < 6U; w++)
    {
        ref_drive_window(3000U, REF_STEP);
    }

    faded = ref_probe();
    check("ref: thresholds followed the weaker tape", faded != fresh);

    (void)run_sweep(&s);
    check("ref: recalibrated", line_cal_state() == LINE_CAL_OK);
    check("ref: recalibration restores the START result",
          ref_probe() == fresh);

    /* Not running: no effect while a sweep is in progress. */
    line_cal_begin();
    ref_drive_window(3000U, REF_STEP);
    line_cal_abort();
    check("ref: ignored unless calibrated", line_cal_state() == LINE_CAL_IDLE);
}


int
main(void)
{
    test_centred_and_left_drift();
    test_right_drift();
    test_lost();
    test_junction();
    test_calibration_gap();
    test_calibration_not_measured();
    test_calibration_raw_peak();
    test_calibration_blind_gap();
    test_calibration_overlap_and_contrast();
    test_calibration_moving_only();
    test_both_white_side();
    test_both_white_side_uncalibrated();
    test_ref_slow_fade();
    test_ref_quick_slide_off();
    test_ref_centred_no_white();
    test_ref_no_tape_or_stationary();
    test_ref_reset_on_recalibration();

    (void)printf("%u checks, %u failures\n",
                 (unsigned int)checks, (unsigned int)failures);

    return (failures == 0U) ? 0 : 1;
}
