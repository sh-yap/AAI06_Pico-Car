/** @file test_obstacle_calc.c
 *
 * @brief Host unit tests for obstacle_calc.c and obstacle_monitor.c
 *        (Buddy 5).
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
#include "obstacle_monitor.h"

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

/*!
 * @brief Compare two 64-bit values and report a mismatch.
 *
 * @param[in] p_name   Description of the check.
 * @param[in] actual   Value produced by the code under test.
 * @param[in] expected Value the test expects.
 */
static void
check_u64(char const * p_name, uint64_t actual, uint64_t expected)
{
    checks++;

    if (actual != expected)
    {
        failures++;
        (void)printf("FAIL %s: got %llu, expected %llu\n", p_name,
                     (unsigned long long)actual,
                     (unsigned long long)expected);
    }
}

static void
test_pack_status(void)
{
    obstacle_status_t in = { 250U, OBSTACLE_RANGE_OK,
                             OBSTACLE_STATE_MONITOR, 7U };
    obstacle_status_t out = { 0U, OBSTACLE_RANGE_NOT_READY,
                              OBSTACLE_STATE_INIT, 0U };
    uint32_t          word;

    /* Pins the layout: 250 | OK << 12 | MONITOR << 15 | 7 << 18. */
    word = obstacle_calc_pack_status(&in);
    check_u32("status known word", word, 1868026U);
    check_u32("status round trip ok",
              obstacle_calc_unpack_status(word, &out), true);
    check_u32("status rt distance", out.distance_mm, 250U);
    check_u32("status rt range", out.range_status, OBSTACLE_RANGE_OK);
    check_u32("status rt state", out.state, OBSTACLE_STATE_MONITOR);
    check_u32("status rt seq", out.seq, 7U);

    /* Every field at its maximum still fits in 26 bits. */
    in.distance_mm = 4095U;
    in.state = OBSTACLE_STATE_SENSOR_FAULT;
    in.seq = 255U;
    word = obstacle_calc_pack_status(&in);
    check_u32("status all max", word, 67014655U);
    check_u32("status reserved clear", word >> 26, 0U);

    in.distance_mm = 4000U;
    (void)obstacle_calc_unpack_status(obstacle_calc_pack_status(&in), &out);
    check_u32("status 4000 mm", out.distance_mm, 4000U);

    in.distance_mm = 5000U;
    (void)obstacle_calc_unpack_status(obstacle_calc_pack_status(&in), &out);
    check_u32("status saturates", out.distance_mm, 4095U);

    in.seq = 256U;
    (void)obstacle_calc_unpack_status(obstacle_calc_pack_status(&in), &out);
    check_u32("status seq wraps", out.seq, 0U);

    /* A distance with a bad status is dropped: TIMEOUT never reads as mm. */
    in.distance_mm = 300U;
    in.range_status = OBSTACLE_RANGE_TIMEOUT;
    in.state = OBSTACLE_STATE_MONITOR;
    in.seq = 0U;
    word = obstacle_calc_pack_status(&in);
    check_u32("status timeout word", word, 49152U);
    (void)obstacle_calc_unpack_status(word, &out);
    check_u32("status timeout 0 mm", out.distance_mm, 0U);
    check_u32("status timeout code", out.range_status,
              OBSTACLE_RANGE_TIMEOUT);

    /* Bad input packs as NOT_READY with everything else 0. */
    check_u32("status NULL", obstacle_calc_pack_status(NULL), 20480U);
    in.state = OBSTACLE_STATE_COUNT;
    check_u32("status bad state", obstacle_calc_pack_status(&in), 20480U);

    /* Malformed words are rejected. */
    check_u32("unpack NULL", obstacle_calc_unpack_status(0U, NULL), false);
    check_u32("unpack reserved",
              obstacle_calc_unpack_status(1UL << 26, &out), false);
    check_u32("unpack range 6",
              obstacle_calc_unpack_status(6UL << 12, &out), false);
    check_u32("unpack state 6",
              obstacle_calc_unpack_status(6UL << 15, &out), false);
    check_u32("unpack mm with timeout",
              obstacle_calc_unpack_status((4UL << 12) | 100U, &out), false);
}

static void
test_pack_profile(void)
{
    obstacle_profile_t in = { 240U, 75U, 300U, 500U, 0U,
                              OBSTACLE_ACTION_TURN_LEFT };
    obstacle_profile_t out = { 0U, 0U, 0U, 0U, 0U, OBSTACLE_ACTION_STOP };
    uint64_t           word;

    /* Pins the layout: 240 | 75 << 12 | 300 << 20 | 500 << 30 | 0 << 42
     * | TURN_LEFT << 54. */
    word = obstacle_calc_pack_profile(&in);
    check_u64("profile known word", word, 36029334204756208ULL);
    check_u32("profile round trip ok",
              obstacle_calc_unpack_profile(word, &out), true);
    check_u32("profile rt distance", out.distance_mm, 240U);
    check_u32("profile rt angle", out.angle_deg, 75U);
    check_u32("profile rt width", out.width_mm, 300U);
    check_u32("profile rt left", out.clear_left_mm, 500U);
    check_u32("profile rt right", out.clear_right_mm, 0U);
    check_u32("profile rt action", out.action, OBSTACLE_ACTION_TURN_LEFT);

    /* Saturation and limits. */
    in.distance_mm = 9000U;
    in.angle_deg = 200U;
    in.width_mm = 2000U;
    in.clear_left_mm = 5000U;
    in.clear_right_mm = 4095U;
    in.action = OBSTACLE_ACTION_REVERSE_RETRY;
    word = obstacle_calc_pack_profile(&in);
    check_u64("profile reserved clear", word >> 57, 0U);
    (void)obstacle_calc_unpack_profile(word, &out);
    check_u32("profile mm saturates", out.distance_mm, 4095U);
    check_u32("profile angle limit", out.angle_deg, 180U);
    check_u32("profile width saturates", out.width_mm, 1023U);
    check_u32("profile left saturates", out.clear_left_mm, 4095U);
    check_u32("profile right max", out.clear_right_mm, 4095U);
    check_u32("profile action max", out.action,
              OBSTACLE_ACTION_REVERSE_RETRY);

    /* Bad input packs as 0: no distance, STOP. */
    check_u64("profile NULL", obstacle_calc_pack_profile(NULL), 0U);
    in.action = OBSTACLE_ACTION_COUNT;
    check_u64("profile bad action", obstacle_calc_pack_profile(&in), 0U);
    (void)obstacle_calc_unpack_profile(0U, &out);
    check_u32("profile zero is STOP", out.action, OBSTACLE_ACTION_STOP);

    /* Malformed words are rejected. */
    check_u32("unpack profile NULL",
              obstacle_calc_unpack_profile(0U, NULL), false);
    check_u32("unpack profile reserved",
              obstacle_calc_unpack_profile(1ULL << 57, &out), false);
    check_u32("unpack profile action 5",
              obstacle_calc_unpack_profile(5ULL << 54, &out), false);
    check_u32("unpack profile angle 181",
              obstacle_calc_unpack_profile(181ULL << 12, &out), false);
}

/*!
 * @brief Feed one reading to the monitor.
 *
 * @param[in,out] p_mon       Monitor under test.
 * @param[in]     status      Reading status.
 * @param[in]     distance_mm Reading distance (0 unless status is OK).
 * @param[out]    p_report    What the monitor would publish.
 */
static void
feed(obstacle_monitor_t *    p_mon,
     obstacle_range_status_t status,
     uint32_t                distance_mm,
     obstacle_status_t *     p_report)
{
    obstacle_range_t range = { 0U, 0U, 0U, OBSTACLE_RANGE_OK };

    range.status = status;
    range.distance_mm = distance_mm;
    obstacle_monitor_step(p_mon, &range, p_report);
}

static void
test_monitor_stop(void)
{
    obstacle_monitor_t mon;
    obstacle_status_t  rep;
    uint32_t           idx;

    /* Approach: two readings under 250 mm are needed. */
    obstacle_monitor_reset(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, 600U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 400U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 300U, &rep);
    check_u32("approach 300 not ahead", mon.b_ahead, false);
    feed(&mon, OBSTACLE_RANGE_OK, 240U, &rep);
    check_u32("approach one close", mon.b_ahead, false);
    check_u32("report distance", rep.distance_mm, 240U);
    check_u32("report state", rep.state, OBSTACLE_STATE_MONITOR);
    feed(&mon, OBSTACLE_RANGE_OK, 230U, &rep);
    check_u32("approach two close", mon.b_ahead, true);

    /* Neither far readings nor TIMEOUT clear it; only the ack does. */
    feed(&mon, OBSTACLE_RANGE_OK, 2000U, &rep);
    feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    check_u32("held until ack", mon.b_ahead, true);
    obstacle_monitor_ack(&mon);
    check_u32("ack clears", mon.b_ahead, false);
    feed(&mon, OBSTACLE_RANGE_OK, 2000U, &rep);
    check_u32("clear after ack", mon.b_ahead, false);

    /* Still there after the ack: raised again after two readings. */
    obstacle_monitor_ack(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, 230U, &rep);
    check_u32("re-raise one", mon.b_ahead, false);
    feed(&mon, OBSTACLE_RANGE_OK, 230U, &rep);
    check_u32("re-raise two", mon.b_ahead, true);

    /* One stray far echo between close readings neither stops nor
     * counts as lost. */
    obstacle_monitor_reset(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, 300U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 240U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 761U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 235U, &rep);
    check_u32("stray not ahead", mon.b_ahead, false);
    feed(&mon, OBSTACLE_RANGE_OK, 230U, &rep);
    check_u32("stray then two close", mon.b_ahead, true);

    /* Open space far away is not an obstacle. */
    obstacle_monitor_reset(&mon);
    for (idx = 0U; idx < 10U; idx++)
    {
        feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    }
    check_u32("open space", mon.b_ahead, false);
    check_u32("open space no fault", mon.b_fault, false);
}

static void
test_monitor_lost_and_blind(void)
{
    obstacle_monitor_t mon;
    obstacle_status_t  rep;

    /* D4: off-centre obstacle seen at 500 mm, then the wall at 2.16 m. */
    obstacle_monitor_reset(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, 528U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 500U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 2162U, &rep);
    check_u32("lost one far", mon.b_ahead, false);
    feed(&mon, OBSTACLE_RANGE_OK, 2161U, &rep);
    check_u32("lost two far", mon.b_ahead, true);

    /* Lost into TIMEOUT. */
    obstacle_monitor_reset(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, 500U, &rep);
    feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    check_u32("lost one timeout", mon.b_ahead, false);
    feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    check_u32("lost two timeouts", mon.b_ahead, true);

    /* Something far (> 600 mm) that disappears is not remembered. */
    obstacle_monitor_reset(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, 900U, &rep);
    feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    check_u32("far lost ignored", mon.b_ahead, false);

    /* Moving away gradually is not "lost". */
    obstacle_monitor_reset(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, 400U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 550U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 700U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 1500U, &rep);
    check_u32("moving away", mon.b_ahead, false);

    /* Blind zone: 47 mm then TIMEOUT = still blocked, reported as
     * TOO_CLOSE with no distance, even straight after an ack. */
    obstacle_monitor_reset(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, 48U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 47U, &rep);
    obstacle_monitor_ack(&mon);
    feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    check_u32("blind ahead", mon.b_ahead, true);
    check_u32("blind status", rep.range_status, OBSTACLE_RANGE_TOO_CLOSE);
    check_u32("blind distance", rep.distance_mm, 0U);

    /* The latch clears with a valid reading over 100 mm. */
    obstacle_monitor_ack(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, 150U, &rep);
    obstacle_monitor_ack(&mon);
    feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    check_u32("latch cleared", rep.range_status, OBSTACLE_RANGE_TIMEOUT);

    /* A TOO_CLOSE reading stops at once. */
    obstacle_monitor_reset(&mon);
    feed(&mon, OBSTACLE_RANGE_TOO_CLOSE, 0U, &rep);
    check_u32("too close ahead", mon.b_ahead, true);
}

static void
test_monitor_fault(void)
{
    obstacle_monitor_t mon;
    obstacle_status_t  rep;
    uint32_t           idx;

    obstacle_monitor_reset(&mon);

    for (idx = 0U; idx < 4U; idx++)
    {
        feed(&mon, OBSTACLE_RANGE_NO_ECHO, 0U, &rep);
    }
    check_u32("4 no echo", mon.b_fault, false);

    /* One answer resets the count. */
    feed(&mon, OBSTACLE_RANGE_OK, 800U, &rep);
    for (idx = 0U; idx < 4U; idx++)
    {
        feed(&mon, OBSTACLE_RANGE_NO_ECHO, 0U, &rep);
    }
    check_u32("count reset", mon.b_fault, false);

    feed(&mon, OBSTACLE_RANGE_NO_ECHO, 0U, &rep);
    check_u32("5 no echo", mon.b_fault, true);
    check_u32("fault state", rep.state, OBSTACLE_STATE_SENSOR_FAULT);

    /* TIMEOUT proves the sensor answers; 3 are needed to recover. */
    feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 800U, &rep);
    check_u32("2 answers", mon.b_fault, true);
    feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    check_u32("3 answers", mon.b_fault, false);
    check_u32("monitor state", rep.state, OBSTACLE_STATE_MONITOR);

    /* NOT_READY counts as no answer. */
    obstacle_monitor_reset(&mon);
    for (idx = 0U; idx < 5U; idx++)
    {
        feed(&mon, OBSTACLE_RANGE_NOT_READY, 0U, &rep);
    }
    check_u32("not ready fault", mon.b_fault, true);
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
    test_pack_status();
    test_pack_profile();
    test_monitor_stop();
    test_monitor_lost_and_blind();
    test_monitor_fault();

    (void)printf("%u checks, %u failures\n",
                 (unsigned int)checks, (unsigned int)failures);

    return ((0U == failures) ? 0 : 1);
}

/*** end of file ***/
