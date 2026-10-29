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


int
main(void)
{
    test_centred_and_left_drift();
    test_right_drift();
    test_lost();

    (void)printf("%u checks, %u failures\n",
                 (unsigned int)checks, (unsigned int)failures);

    return (failures == 0U) ? 0 : 1;
}
