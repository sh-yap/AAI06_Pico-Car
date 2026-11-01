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


int
main(void)
{
    test_centred_and_left_drift();
    test_right_drift();
    test_lost();
    test_calibration_gap();
    test_calibration_not_measured();
    test_calibration_moving_only();

    (void)printf("%u checks, %u failures\n",
                 (unsigned int)checks, (unsigned int)failures);

    return (failures == 0U) ? 0 : 1;
}
