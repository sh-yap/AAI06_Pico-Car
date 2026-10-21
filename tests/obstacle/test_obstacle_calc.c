/** @file test_obstacle_calc.c
 *
 * @brief Host unit tests for app_program/obstacle_calc.c (Buddy 5).
 *
 * Run with:  make -C tests/obstacle test
 *
 * Expected values assume 343 m/s and the settle constants in
 * obstacle_config.h.  Servo checks use their own calibration and compare
 * clamped results with the configured pulse limits, so recalibrating the
 * servo does not break them.
 */

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "obstacle_config.h"
#include "obstacle_types.h"
#include "obstacle_calc.h"

static uint32_t failures = 0U;
static uint32_t checks = 0U;


/*!
 * @brief Compare two values and report a mismatch.
 *
 * @param[in] p_name   Description of the check.
 * @param[in] actual   Value produced by the code under test.
 * @param[in] expected Value the test expects.
 */
static void
check_u32(char const * p_name, uint32_t actual, uint32_t expected)
{
    checks++;

    if (actual != expected)
    {
        failures++;
        (void)printf("FAIL %s: got %u, expected %u\n",
                     p_name, (unsigned int)actual, (unsigned int)expected);
    }
}

static void
test_classify_echo(void)
{
    check_u32("classify 0 us", obstacle_calc_classify_echo(0U, false),
              OBSTACLE_RANGE_NO_ECHO);
    check_u32("classify 116 us", obstacle_calc_classify_echo(116U, false),
              OBSTACLE_RANGE_TOO_CLOSE);
    check_u32("classify min", obstacle_calc_classify_echo(117U, false),
              OBSTACLE_RANGE_OK);
    check_u32("classify max", obstacle_calc_classify_echo(23324U, false),
              OBSTACLE_RANGE_OK);
    check_u32("classify max+1", obstacle_calc_classify_echo(23325U, false),
              OBSTACLE_RANGE_TOO_FAR);
    check_u32("classify high", obstacle_calc_classify_echo(5000U, true),
              OBSTACLE_RANGE_TIMEOUT);
}

static void
test_echo_to_mm(void)
{
    check_u32("mm 0 us", obstacle_calc_echo_to_mm(0U), 0U);
    check_u32("mm 100 mm", obstacle_calc_echo_to_mm(583U), 100U);
    check_u32("mm 1000 mm", obstacle_calc_echo_to_mm(5831U), 1000U);
    check_u32("mm 4 m", obstacle_calc_echo_to_mm(23324U), 4000U);
    check_u32("mm clamp", obstacle_calc_echo_to_mm(100000U), 11239U);
}

/* The interpolation checks below expect 1200..1800 us to be unclamped. */
#if ((OBSTACLE_SERVO_PULSE_MIN_US > 1200U) || \
     (OBSTACLE_SERVO_PULSE_MAX_US < 1800U))
#error "Pulse limits too narrow for these tests; adjust the test points"
#endif

static void
test_angle_to_pulse(void)
{
    obstacle_servo_cal_t const normal = { 600U, 1500U, 2400U };
    obstacle_servo_cal_t const reversed = { 2400U, 1500U, 600U };
    obstacle_servo_cal_t const extreme = { 0U, 1500U, 3000U };

    check_u32("pulse 60", obstacle_calc_angle_to_pulse(60U, &normal),
              1200U);
    check_u32("pulse 90", obstacle_calc_angle_to_pulse(90U, &normal), 1500U);
    check_u32("pulse 120", obstacle_calc_angle_to_pulse(120U, &normal),
              1800U);
    check_u32("pulse 0 clamps", obstacle_calc_angle_to_pulse(0U, &normal),
              OBSTACLE_SERVO_PULSE_MIN_US);
    check_u32("pulse 200 clamps", obstacle_calc_angle_to_pulse(200U, &normal),
              OBSTACLE_SERVO_PULSE_MAX_US);
    check_u32("pulse NULL cal", obstacle_calc_angle_to_pulse(30U, NULL),
              OBSTACLE_SERVO_PULSE_90_US);
    check_u32("pulse reversed 60",
              obstacle_calc_angle_to_pulse(60U, &reversed), 1800U);
    check_u32("pulse reversed 120",
              obstacle_calc_angle_to_pulse(120U, &reversed), 1200U);
    check_u32("pulse limit low",
              obstacle_calc_angle_to_pulse(0U, &extreme),
              OBSTACLE_SERVO_PULSE_MIN_US);
    check_u32("pulse limit high",
              obstacle_calc_angle_to_pulse(180U, &extreme),
              OBSTACLE_SERVO_PULSE_MAX_US);
}

static void
test_clamp_pulse(void)
{
    check_u32("clamp low", obstacle_calc_clamp_pulse(100U),
              OBSTACLE_SERVO_PULSE_MIN_US);
    check_u32("clamp high", obstacle_calc_clamp_pulse(3000U),
              OBSTACLE_SERVO_PULSE_MAX_US);
    check_u32("clamp pass", obstacle_calc_clamp_pulse(1500U), 1500U);
}

static void
test_settle_ms(void)
{
    uint32_t const swing_ms = (120U * OBSTACLE_SERVO_MS_PER_DEG)
                              + OBSTACLE_SERVO_SETTLE_MARGIN_MS;

    check_u32("settle 30->150", obstacle_calc_settle_ms(30U, 150U),
              swing_ms);
    check_u32("settle 150->30", obstacle_calc_settle_ms(150U, 30U),
              swing_ms);
    check_u32("settle none", obstacle_calc_settle_ms(90U, 90U),
              OBSTACLE_SERVO_SETTLE_MARGIN_MS);
}

static void
test_median(void)
{
    uint32_t odd[] = { 5U, 1U, 3U };
    uint32_t even[] = { 4U, 1U, 3U, 2U };

    check_u32("median odd", obstacle_calc_median(odd, 3U), 3U);
    check_u32("median sorts min", odd[0], 1U);
    check_u32("median sorts max", odd[2], 5U);
    check_u32("median even", obstacle_calc_median(even, 4U), 2U);
    check_u32("median NULL", obstacle_calc_median(NULL, 3U), 0U);
    check_u32("median empty", obstacle_calc_median(odd, 0U), 0U);
}

static void
test_pwm_slice(void)
{
    check_u32("slice GP8", obstacle_calc_pwm_slice(8U), 4U);
    check_u32("slice GP12", obstacle_calc_pwm_slice(12U), 6U);
    check_u32("slice GP15", obstacle_calc_pwm_slice(15U), 7U);
    check_u32("slice GP16", obstacle_calc_pwm_slice(16U), 0U);
    check_u32("slice GP17", obstacle_calc_pwm_slice(17U), 0U);
}

int
main(void)
{
    test_classify_echo();
    test_echo_to_mm();
    test_angle_to_pulse();
    test_clamp_pulse();
    test_settle_ms();
    test_median();
    test_pwm_slice();

    (void)printf("%u checks, %u failures\n",
                 (unsigned int)checks, (unsigned int)failures);

    return ((0U == failures) ? 0 : 1);
}

/*** end of file ***/
