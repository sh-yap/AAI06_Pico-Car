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
#include <math.h>

#include "obstacle_config.h"
#include "obstacle_types.h"
#include "obstacle_calc.h"
#include "obstacle_monitor.h"
#include "obstacle_scan.h"
#include "obstacle_plan.h"

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

    /* Approach: two readings under the stop distance are needed. */
    obstacle_monitor_reset(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_STOP_DISTANCE_MM + 200U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_STOP_DISTANCE_MM + 100U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_STOP_DISTANCE_MM + 50U, &rep);
    check_u32("approach not ahead", mon.b_ahead, false);
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_STOP_DISTANCE_MM - 10U, &rep);
    check_u32("approach one close", mon.b_ahead, false);
    check_u32("report distance", rep.distance_mm,
              OBSTACLE_STOP_DISTANCE_MM - 10U);
    check_u32("report state", rep.state, OBSTACLE_STATE_MONITOR);
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_STOP_DISTANCE_MM - 20U, &rep);
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
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_STOP_DISTANCE_MM - 20U, &rep);
    check_u32("re-raise one", mon.b_ahead, false);
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_STOP_DISTANCE_MM - 20U, &rep);
    check_u32("re-raise two", mon.b_ahead, true);

    /* One stray far echo between close readings neither stops nor
     * counts as lost. */
    obstacle_monitor_reset(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_STOP_DISTANCE_MM + 50U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_STOP_DISTANCE_MM - 10U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 761U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_STOP_DISTANCE_MM - 15U, &rep);
    check_u32("stray not ahead", mon.b_ahead, false);
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_STOP_DISTANCE_MM - 20U, &rep);
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

    /* D4: off-centre obstacle seen nearer than LOST_NEAR_MM, then the
     * wall at 2.16 m. */
    obstacle_monitor_reset(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_LOST_NEAR_MM - 10U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_LOST_NEAR_MM - 20U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 2162U, &rep);
    check_u32("lost one far", mon.b_ahead, false);
    feed(&mon, OBSTACLE_RANGE_OK, 2161U, &rep);
    check_u32("lost two far", mon.b_ahead, true);

    /* Lost into TIMEOUT. */
    obstacle_monitor_reset(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, OBSTACLE_LOST_NEAR_MM - 20U, &rep);
    feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    check_u32("lost one timeout", mon.b_ahead, false);
    feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    check_u32("lost two timeouts", mon.b_ahead, true);

    /* Track test 2026-11-03: in a turn the course wall passes 300 mm away
     * and then leaves the beam.  Neither rule may stop the car. */
    obstacle_monitor_reset(&mon);
    feed(&mon, OBSTACLE_RANGE_OK, 340U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 310U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 300U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 305U, &rep);
    feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    feed(&mon, OBSTACLE_RANGE_TIMEOUT, 0U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 1800U, &rep);
    feed(&mon, OBSTACLE_RANGE_OK, 1790U, &rep);
    check_u32("wall in a turn", mon.b_ahead, false);

    /* Something far (> LOST_NEAR_MM) that disappears is not remembered. */
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

/*!
 * @brief Build a coarse scan (60, 75, 90, 105, 120 deg) from distances.
 *
 * @param[out] p_res  Scan to fill.
 * @param[in]  p_mm   Five distances, right to left; 0 = unknown.
 * @param[in]  silent Silent pings at every unknown angle.
 */
static void
make_scan(obstacle_scan_result_t * p_res, uint32_t const * p_mm,
          uint32_t silent)
{
    uint32_t idx;

    p_res->count = 5U;

    for (idx = 0U; idx < 5U; idx++)
    {
        p_res->points[idx].angle_deg    = 60U + (idx * 15U);
        p_res->points[idx].distance_mm  = p_mm[idx];
        p_res->points[idx].valid_count  = (0U == p_mm[idx]) ? 0U : 3U;
        p_res->points[idx].silent_count = (0U == p_mm[idx]) ? silent : 0U;
        p_res->points[idx].point_class  = OBSTACLE_POINT_UNKNOWN;
    }

    obstacle_scan_analyse(p_res);
}

static void
test_scan_reduce(void)
{
    obstacle_range_t      pings[3] =
    {
        { 0U, 300U, 0U, OBSTACLE_RANGE_OK },
        { 0U, 310U, 0U, OBSTACLE_RANGE_OK },
        { 0U, 0U, 0U, OBSTACLE_RANGE_TIMEOUT }
    };
    obstacle_scan_point_t pt;

    obstacle_scan_reduce(90U, pings, 3U, &pt);
    check_u32("reduce 2 of 3 valid", pt.valid_count, 2U);
    check_u32("reduce median", pt.distance_mm, 305U);
    check_u32("reduce angle", pt.angle_deg, 90U);

    pings[1].status = OBSTACLE_RANGE_NO_ECHO;
    pings[1].distance_mm = 0U;
    obstacle_scan_reduce(75U, pings, 3U, &pt);
    check_u32("reduce 1 valid = unknown", pt.distance_mm, 0U);
    check_u32("reduce silent", pt.silent_count, 1U);
}

static void
test_scan_decisions(void)
{
    static obstacle_scan_result_t res;
    uint32_t const centred[5]  = { 900U, 470U, 450U, 470U, 900U };
    uint32_t const right[5]    = { 310U, 300U, 900U, 900U, 900U };
    uint32_t const clear[5]    = { 900U, 1500U, 2000U, 1500U, 900U };
    uint32_t const blind[5]    = { 900U, 0U, 0U, 0U, 900U };
    uint32_t const angled[5]   = { 0U, 470U, 450U, 470U, 900U };
    uint32_t const none[5]     = { 0U, 0U, 0U, 0U, 0U };
    uint32_t const wall[5]     = { 460U, 455U, 450U, 455U, 460U };
    uint32_t const hyst[5]     = { 900U, 640U, 450U, 470U, 900U };
    uint32_t const leftish[5]  = { 900U, 900U, 460U, 450U, 900U };

    /* These scenes test the side chosen from the scan itself. */
    obstacle_scan_set_prefer(OBSTACLE_PREFER_SCAN);

    /* 20 cm box straight ahead at 45 cm, background at 90 cm. */
    make_scan(&res, centred, 0U);
    check_u32("centred action", res.profile.action,
              OBSTACLE_ACTION_TURN_LEFT);
    check_u32("centred closest", res.profile.distance_mm, 450U);
    check_u32("centred at", res.profile.angle_deg, 90U);
    check_u32("centred right edge", res.right_edge_deg, 75U);
    check_u32("centred left edge", res.left_edge_deg, 105U);
    check_u32("centred width bound", res.profile.width_mm, 450U);
    check_u32("centred clear left", res.profile.clear_left_mm, 900U);
    check_u32("centred clear right", res.profile.clear_right_mm, 900U);
    check_u32("centred 60 background", res.points[0].point_class,
              OBSTACLE_POINT_BACKGROUND);
    check_u32("centred 90 obstacle", res.points[2].point_class,
              OBSTACLE_POINT_OBSTACLE);

    /* Obstacle to the right: go left; width unknown (span at the edge). */
    make_scan(&res, right, 0U);
    check_u32("right action", res.profile.action, OBSTACLE_ACTION_TURN_LEFT);
    check_u32("right width open", res.profile.width_mm, 1023U);
    check_u32("right no right clear", res.profile.clear_right_mm, 0U);

    /* Obstacle a little left of centre, both sides clear: go right. */
    make_scan(&res, leftish, 0U);
    check_u32("leftish action", res.profile.action,
              OBSTACLE_ACTION_TURN_RIGHT);

    /* The obstacle has gone. */
    make_scan(&res, clear, 0U);
    check_u32("clear action", res.profile.action, OBSTACLE_ACTION_CONTINUE);

    /* No echo straight ahead: never CONTINUE.  Nothing proves either
     * side, so right first, for the path check to verify. */
    make_scan(&res, blind, 0U);
    check_u32("blind action", res.profile.action,
              OBSTACLE_ACTION_TURN_RIGHT);

    /* TIMEOUT on the right (angled face): only the left is proven. */
    make_scan(&res, angled, 0U);
    check_u32("angled action", res.profile.action,
              OBSTACLE_ACTION_TURN_LEFT);
    check_u32("angled right edge", res.right_edge_deg, 60U);

    /* Nothing measured at all (all TIMEOUT). */
    make_scan(&res, none, 0U);
    check_u32("none action", res.profile.action,
              OBSTACLE_ACTION_REVERSE_RETRY);

    /* Heard across the whole view: unproven both sides, right first. */
    make_scan(&res, wall, 0U);
    check_u32("wall action", res.profile.action,
              OBSTACLE_ACTION_TURN_RIGHT);
    check_u32("wall width open", res.profile.width_mm, 1023U);

    /* Hysteresis: 640 mm is background by the 150 mm rule but still
     * extends the span (within 250 mm). */
    make_scan(&res, hyst, 0U);
    check_u32("hysteresis right edge", res.right_edge_deg, 75U);
    check_u32("hysteresis action", res.profile.action,
              OBSTACLE_ACTION_TURN_LEFT);

    /* Sensor unplugged: no echo at 3 or more angles. */
    make_scan(&res, none, 3U);
    check_u32("fault action", res.profile.action, OBSTACLE_ACTION_STOP);
}

/*!
 * @brief Build a 5-degree scan (60..120, 13 angles) from distances.
 *
 * @param[out] p_res Scan to fill and analyse.
 * @param[in]  p_mm  Thirteen distances, right to left; 0 = unknown.
 */
static void
make_fine_scan(obstacle_scan_result_t * p_res, uint32_t const * p_mm)
{
    uint32_t idx;

    p_res->count = 13U;

    for (idx = 0U; idx < 13U; idx++)
    {
        p_res->points[idx].angle_deg    = 60U + (idx * 5U);
        p_res->points[idx].distance_mm  = p_mm[idx];
        p_res->points[idx].valid_count  = (0U == p_mm[idx]) ? 0U : 3U;
        p_res->points[idx].silent_count = 0U;
        p_res->points[idx].point_class  = OBSTACLE_POINT_UNKNOWN;
    }

    obstacle_scan_analyse(p_res);
}

/* Real 5-degree scans, 9.5 cm box at about 441 mm (2026-10-31). */
static void
test_scan_real(void)
{
    static obstacle_scan_result_t res;
    uint32_t const centred[13] =
        { 3007U, 924U, 450U, 446U, 445U, 441U, 441U, 441U, 440U, 441U,
          441U, 441U, 446U };
    uint32_t const box_left[13] =
        { 3003U, 2990U, 2990U, 467U, 454U, 458U, 458U, 445U, 450U, 454U,
          454U, 467U, 459U };
    uint32_t const box_right[13] =
        { 481U, 456U, 446U, 442U, 442U, 433U, 433U, 442U, 442U, 442U,
          443U, 443U, 451U };
    uint32_t const nothing[13] =
        { 0U, 0U, 1887U, 1880U, 1857U, 1869U, 1861U, 1861U, 1857U, 1861U,
          1865U, 1879U, 1888U };

    obstacle_scan_set_prefer(OBSTACLE_PREFER_SCAN);

    /* Right side proven (background at 60-65), left unproven. */
    make_fine_scan(&res, centred);
    check_u32("real centred", res.profile.action,
              OBSTACLE_ACTION_TURN_RIGHT);
    check_u32("real centred right edge", res.right_edge_deg, 70U);

    make_fine_scan(&res, box_left);
    check_u32("real box left", res.profile.action,
              OBSTACLE_ACTION_TURN_RIGHT);
    check_u32("real box left clear right", res.profile.clear_right_mm,
              3003U);

    /* Heard across the whole view: neither side is proven, so right
     * first, to be verified by the path check. */
    make_fine_scan(&res, box_right);
    check_u32("real box right", res.profile.action,
              OBSTACLE_ACTION_TURN_RIGHT);
    check_u32("real box right no proof", res.profile.clear_left_mm, 0U);

    make_fine_scan(&res, nothing);
    check_u32("real nothing", res.profile.action,
              OBSTACLE_ACTION_CONTINUE);
    check_u32("real nothing closest", res.profile.distance_mm, 1857U);

    /* Side fixed for the course (track 2026-11-02): left whatever the
     * scan's proven side, but the other decisions are unchanged. */
    obstacle_scan_set_prefer(OBSTACLE_PREFER_LEFT);
    make_fine_scan(&res, centred);
    check_u32("prefer left centred", res.profile.action,
              OBSTACLE_ACTION_TURN_LEFT);
    make_fine_scan(&res, box_left);
    check_u32("prefer left box left", res.profile.action,
              OBSTACLE_ACTION_TURN_LEFT);
    make_fine_scan(&res, box_right);
    check_u32("prefer left box right", res.profile.action,
              OBSTACLE_ACTION_TURN_LEFT);
    make_fine_scan(&res, nothing);
    check_u32("prefer left nothing", res.profile.action,
              OBSTACLE_ACTION_CONTINUE);

    obstacle_scan_set_prefer(OBSTACLE_PREFER_RIGHT);
    make_fine_scan(&res, box_right);
    check_u32("prefer right box right", res.profile.action,
              OBSTACLE_ACTION_TURN_RIGHT);

    /* An unknown value is ignored. */
    obstacle_scan_set_prefer(7U);
    make_fine_scan(&res, centred);
    check_u32("prefer ignored", res.profile.action,
              OBSTACLE_ACTION_TURN_RIGHT);

    obstacle_scan_set_prefer(OBSTACLE_BYPASS_PREFER);
}

/*!
 * @brief Fill n pings with one status and distance.
 */
static void
fill_pings(obstacle_range_t * p_pings, uint32_t n,
           obstacle_range_status_t status, uint32_t distance_mm)
{
    uint32_t idx;

    for (idx = 0U; idx < n; idx++)
    {
        p_pings[idx].echo_us      = 0U;
        p_pings[idx].distance_mm  = distance_mm;
        p_pings[idx].timestamp_us = 0U;
        p_pings[idx].status       = status;
    }
}

/*!
 * @brief Start a path check.
 */
static void
path_start(obstacle_path_t * p_path, uint32_t need_mm, uint32_t known_mm)
{
    p_path->need_mm  = need_mm;
    p_path->known_mm = known_mm;
    p_path->count    = 0U;
    p_path->verdict  = OBSTACLE_PATH_FAULT;
}

/*!
 * @brief Add one path-check angle with three identical pings.
 */
static void
path_add(obstacle_path_t * p_path, uint32_t angle_deg,
         obstacle_range_status_t status, uint32_t distance_mm)
{
    obstacle_range_t pings[3];

    fill_pings(pings, 3U, status, distance_mm);
    obstacle_scan_path_add(p_path, angle_deg, pings, 3U);
}

/*!
 * @brief Verdict of a right-side check with OK readings (0 = TIMEOUT).
 */
static obstacle_path_verdict_t
path_run(uint32_t need_mm, uint32_t known_mm,
         uint32_t outer_mm, uint32_t middle_mm, uint32_t ahead_mm)
{
    obstacle_path_t path;
    uint32_t const  mm[3] = { outer_mm, middle_mm, ahead_mm };
    uint32_t        idx;

    path_start(&path, need_mm, known_mm);

    for (idx = 0U; idx < 3U; idx++)
    {
        path_add(&path, obstacle_scan_path_angle(OBSTACLE_ACTION_TURN_RIGHT,
                                                 idx),
                 (0U == mm[idx]) ? OBSTACLE_RANGE_TIMEOUT : OBSTACLE_RANGE_OK,
                 mm[idx]);
    }

    obstacle_scan_path_verdict(&path);

    return (path.verdict);
}

static void
test_scan_path(void)
{
    obstacle_range_t pings[3];
    obstacle_path_t  path;

    /* Angles: from the bypass side back to straight ahead. */
    check_u32("path angle right 0",
              obstacle_scan_path_angle(OBSTACLE_ACTION_TURN_RIGHT, 0U), 70U);
    check_u32("path angle right 1",
              obstacle_scan_path_angle(OBSTACLE_ACTION_TURN_RIGHT, 1U), 80U);
    check_u32("path angle right 2",
              obstacle_scan_path_angle(OBSTACLE_ACTION_TURN_RIGHT, 2U), 90U);
    check_u32("path angle left 0",
              obstacle_scan_path_angle(OBSTACLE_ACTION_TURN_LEFT, 0U), 115U);
    check_u32("path angle left 1",
              obstacle_scan_path_angle(OBSTACLE_ACTION_TURN_LEFT, 1U), 103U);
    check_u32("path angle left 2",
              obstacle_scan_path_angle(OBSTACLE_ACTION_TURN_LEFT, 2U), 90U);
    check_u32("path angle clamp",
              obstacle_scan_path_angle(OBSTACLE_ACTION_TURN_LEFT, 7U), 90U);
    check_u32("path angle no turn",
              obstacle_scan_path_angle(OBSTACLE_ACTION_CONTINUE, 0U), 90U);

    /* Bench 2026-11-01, 21 cm card at 455 mm, car turned 21 deg right:
     * 70 sees past it, 80 and 90 still hear it. */
    path_start(&path, 1047U, 455U);
    path_add(&path, 70U, OBSTACLE_RANGE_OK, 2270U);
    path_add(&path, 80U, OBSTACLE_RANGE_OK, 465U);
    path_add(&path, 90U, OBSTACLE_RANGE_OK, 456U);
    obstacle_scan_path_verdict(&path);
    check_u32("path bench right", path.verdict, OBSTACLE_PATH_CLEAR);
    check_u32("path bench right count", path.count, 3U);
    check_u32("path bench right 70", path.points[0].verdict,
              OBSTACLE_PATH_CLEAR);
    check_u32("path bench right 80", path.points[1].verdict,
              OBSTACLE_PATH_KNOWN);
    check_u32("path bench right 80 mm", path.points[1].distance_mm, 465U);
    check_u32("path bench right 90 angle", path.points[2].angle_deg, 90U);

    /* Same, but turned too little: the outer angle still hears it. */
    check_u32("path outer known",
              path_run(1047U, 455U, 483U, 473U, 460U), OBSTACLE_PATH_BLOCKED);

    /* Bench left turn: a real object at 989 mm, short of the 1047 needed. */
    check_u32("path room object",
              path_run(1047U, 467U, 989U, 502U, 471U), OBSTACLE_PATH_BLOCKED);

    /* Something new, nearer than the box or between it and need_mm. */
    check_u32("path new near",
              path_run(1047U, 455U, 2270U, 465U, 300U), OBSTACLE_PATH_BLOCKED);
    check_u32("path new between",
              path_run(1047U, 455U, 2270U, 700U, 456U), OBSTACLE_PATH_BLOCKED);

    /* No known obstacle: every near reading blocks. */
    check_u32("path no known",
              path_run(1047U, 0U, 2270U, 465U, 456U), OBSTACLE_PATH_BLOCKED);

    /* Known window: 455 - 40 .. 455 + 80. */
    check_u32("path window low",
              path_run(1047U, 455U, 2270U, 415U, 2270U), OBSTACLE_PATH_CLEAR);
    check_u32("path window below",
              path_run(1047U, 455U, 2270U, 414U, 2270U),
              OBSTACLE_PATH_BLOCKED);
    check_u32("path window high",
              path_run(1047U, 455U, 2270U, 535U, 2270U), OBSTACLE_PATH_CLEAR);
    check_u32("path window above",
              path_run(1047U, 455U, 2270U, 536U, 2270U),
              OBSTACLE_PATH_BLOCKED);

    /* No echo is not proof (angled faces are silent). */
    check_u32("path open",
              path_run(1047U, 455U, 0U, 465U, 456U), OBSTACLE_PATH_OPEN);

    /* Sensor silent at one angle. */
    path_start(&path, 1047U, 455U);
    path_add(&path, 70U, OBSTACLE_RANGE_OK, 2270U);
    path_add(&path, 80U, OBSTACLE_RANGE_NO_ECHO, 0U);
    path_add(&path, 90U, OBSTACLE_RANGE_OK, 456U);
    obstacle_scan_path_verdict(&path);
    check_u32("path fault", path.verdict, OBSTACLE_PATH_FAULT);

    /* One reading among timeouts: near blocks, known is known, far is
     * not enough to call clear. */
    path_start(&path, 1047U, 455U);
    fill_pings(pings, 3U, OBSTACLE_RANGE_TIMEOUT, 0U);
    pings[1].status = OBSTACLE_RANGE_OK;
    pings[1].distance_mm = 300U;
    obstacle_scan_path_add(&path, 70U, pings, 3U);
    pings[1].distance_mm = 470U;
    obstacle_scan_path_add(&path, 80U, pings, 3U);
    pings[1].distance_mm = 2000U;
    obstacle_scan_path_add(&path, 90U, pings, 3U);
    check_u32("path one near", path.points[0].verdict,
              OBSTACLE_PATH_BLOCKED);
    check_u32("path one known", path.points[1].verdict, OBSTACLE_PATH_KNOWN);
    check_u32("path one far", path.points[2].verdict, OBSTACLE_PATH_OPEN);

    /* Too close is always blocked; a fourth angle is ignored. */
    fill_pings(pings, 3U, OBSTACLE_RANGE_OK, 2270U);
    pings[2].status = OBSTACLE_RANGE_TOO_CLOSE;
    path_start(&path, 1047U, 455U);
    path_add(&path, 70U, OBSTACLE_RANGE_OK, 2270U);
    path_add(&path, 80U, OBSTACLE_RANGE_OK, 2270U);
    obstacle_scan_path_add(&path, 90U, pings, 3U);
    path_add(&path, 95U, OBSTACLE_RANGE_OK, 2270U);
    obstacle_scan_path_verdict(&path);
    check_u32("path too close", path.verdict, OBSTACLE_PATH_BLOCKED);
    check_u32("path max points", path.count, 3U);

    /* Nothing measured at all. */
    path_start(&path, 1047U, 455U);
    obstacle_scan_path_verdict(&path);
    check_u32("path empty", path.verdict, OBSTACLE_PATH_FAULT);
}

/*!
 * @brief Smallest gap between the car's outline and a box, at one pose.
 *
 * @param[in] ax,ay   Rear axle position (mm).
 * @param[in] heading Car heading (radians, 0 = along the line).
 * @param[in] x0,x1   Box extent along the line.
 * @param[in] y0,y1   Box extent sideways.
 *
 * @return Gap in mm (0 if they touch or overlap).
 */
static double
car_box_gap(double ax, double ay, double heading,
            double x0, double x1, double y0, double y1)
{
    double const back  = -(double)OBSTACLE_REAR_OVERHANG_MM;
    double const front = (double)OBSTACLE_AXLE_TO_FRONT_MM;
    double const half  = (double)OBSTACLE_CAR_HALF_WIDTH_MM;
    double       best = 1.0e9;
    double       cx;
    double       cy;
    double       px;
    double       py;
    double       dx;
    double       dy;
    double       gap;
    double       t;
    int          edge;

    /* Walk the outline in 2 mm steps. */
    for (edge = 0; edge < 4; edge++)
    {
        double const len = ((0 == edge) || (2 == edge)) ?
                           (front - back) : (2.0 * half);

        for (t = 0.0; t <= len; t += 2.0)
        {
            switch (edge)
            {
                case 0:  cx = back + t;  cy = -half;      break;
                case 1:  cx = front;     cy = -half + t;  break;
                case 2:  cx = front - t; cy = half;       break;
                default: cx = back;      cy = half - t;   break;
            }

            px = ax + (cx * cos(heading)) - (cy * sin(heading));
            py = ay + (cx * sin(heading)) + (cy * cos(heading));
            dx = (px < x0) ? (x0 - px) : ((px > x1) ? (px - x1) : 0.0);
            dy = (py < y0) ? (y0 - py) : ((py > y1) ? (py - y1) : 0.0);
            gap = sqrt((dx * dx) + (dy * dy));

            if (gap < best)
            {
                best = gap;
            }
        }
    }

    return (best);
}

/*!
 * @brief Drive the whole out-along-back path and return the smallest gap
 *        to a box.
 *
 * Left is +y.  A right bypass is the mirror image, so the box offset is
 * given towards the bypass side.  After the parallel leg the car turns
 * return_deg towards the line (vc_task.c) and drives until it is 50 mm
 * past the line.
 *
 * @param[in] p_plan      Plan to follow.
 * @param[in] near_mm     Box near face, from the rear axle at the stop.
 * @param[in] offset_mm   Box centre shifted towards the bypass side.
 * @param[in] return_deg  Turn towards the line after the parallel leg.
 *
 * @return Smallest gap (mm) seen anywhere along the path.
 */
static double
simulate_bypass(obstacle_bypass_plan_t const * p_plan,
                double near_mm, double offset_mm, double return_deg)
{
    double const deg = 3.14159265358979 / 180.0;
    double const w = (double)OBSTACLE_MAX_WIDTH_MM;
    double const th = (double)p_plan->turn_deg * deg;
    double const back = return_deg * deg;
    double const out = (double)p_plan->out_mm;
    double const along = (double)p_plan->along_mm;
    double const x0 = near_mm;
    double const x1 = near_mm + w;
    double const y0 = offset_mm - (w / 2.0);
    double const y1 = offset_mm + (w / 2.0);
    double       best = 1.0e9;
    double       gap;
    double       a;
    double       s;
    double       ax;
    double       ay;

    for (a = 0.0; a <= th; a += 0.005)                  /* Turn out.    */
    {
        gap = car_box_gap(0.0, 0.0, a, x0, x1, y0, y1);
        best = (gap < best) ? gap : best;
    }

    for (s = 0.0; s <= out; s += 2.0)                   /* Out.         */
    {
        gap = car_box_gap(s * cos(th), s * sin(th), th, x0, x1, y0, y1);
        best = (gap < best) ? gap : best;
    }

    ax = out * cos(th);
    ay = out * sin(th);

    for (a = th; a >= 0.0; a -= 0.005)                  /* To parallel. */
    {
        gap = car_box_gap(ax, ay, a, x0, x1, y0, y1);
        best = (gap < best) ? gap : best;
    }

    for (s = 0.0; s <= along; s += 2.0)                 /* Along.       */
    {
        gap = car_box_gap(ax + s, ay, 0.0, x0, x1, y0, y1);
        best = (gap < best) ? gap : best;
    }

    ax += along;

    for (a = 0.0; a >= -back; a -= 0.005)               /* To the line. */
    {
        gap = car_box_gap(ax, ay, a, x0, x1, y0, y1);
        best = (gap < best) ? gap : best;
    }

    for (s = 0.0; (ay - (s * sin(back))) >= -50.0; s += 2.0)  /* Back. */
    {
        gap = car_box_gap(ax + (s * cos(back)), ay - (s * sin(back)), -back,
                          x0, x1, y0, y1);
        best = (gap < best) ? gap : best;
    }

    return (best);
}

static void
test_plan_bypass(void)
{
    static uint32_t const distances[] = { 250U, 350U, 441U, 600U };
    /* Return angles; 0 = the plan's back_deg (the path vc_task.c
     * drives).  The others cover a slipped turn. */
    static double const   returns[] = { 0.0, 30.0, 45.0, 60.0 };
    obstacle_bypass_plan_t plan;
    uint32_t               idx;
    uint32_t               ret;
    double                 near_mm;
    double                 gap;
    double                 back_deg;

    /* Worked example: box at 441 mm.  Out 512 mm to 200 mm aside (the
     * diagonal ends at x = 471), along to x = 935 (506 + 200 + 30 + 99
     * + 100 extra), back at 2 x 23 = 46 deg: 200 / sin 46 = 279 mm. */
    check_u32("plan ok", obstacle_plan_bypass(441U,
              OBSTACLE_ACTION_TURN_LEFT, &plan), true);
    check_u32("plan side", plan.side, OBSTACLE_ACTION_TURN_LEFT);
    check_u32("plan turn", plan.turn_deg, 23U);
    check_u32("plan out", plan.out_mm, 512U);
    check_u32("plan aside", plan.side_mm, 200U);
    check_u32("plan along", plan.along_mm,
              364U + OBSTACLE_PLAN_ALONG_EXTRA_MM);
    check_u32("plan back deg", plan.back_deg, 46U);
    check_u32("plan back", plan.back_mm, 279U);
    check_u32("plan check", plan.check_mm, 612U);

    check_u32("plan not a turn", obstacle_plan_bypass(441U,
              OBSTACLE_ACTION_REVERSE_RETRY, &plan), false);
    check_u32("plan no distance", obstacle_plan_bypass(0U,
              OBSTACLE_ACTION_TURN_LEFT, &plan), false);
    check_u32("plan too close", obstacle_plan_bypass(60U,
              OBSTACLE_ACTION_TURN_LEFT, &plan), false);

    /* Simulate the car's outline along the whole path. */
    for (idx = 0U; idx < 4U; idx++)
    {
        check_u32("plan sim ok", obstacle_plan_bypass(distances[idx],
                  OBSTACLE_ACTION_TURN_RIGHT, &plan), true);
        near_mm = (double)(distances[idx] + OBSTACLE_AXLE_TO_SENSOR_MM);

        for (ret = 0U; ret < 4U; ret++)
        {
            back_deg = (0.0 == returns[ret]) ?
                       (double)plan.back_deg : returns[ret];

            /* As planned: the margin is kept everywhere. */
            gap = simulate_bypass(&plan, near_mm, 0.0, back_deg);
            check_u32("plan sim margin",
                      (gap >= ((double)OBSTACLE_PLAN_MARGIN_MM - 1.0)) ?
                      1U : 0U, 1U);

            /* Sensor 15 mm long and the box 20 mm towards the bypass
             * side: still no contact. */
            gap = simulate_bypass(&plan, near_mm - 15.0, 20.0, back_deg);
            check_u32("plan sim worst case", (gap > 0.0) ? 1U : 0U, 1U);
        }
    }
}

static void
test_plan_line_watch(void)
{
    obstacle_line_watch_t watch;
    int32_t const         held = OBSTACLE_PLAN_LINE_HELD_MM;

    obstacle_plan_line_reset(&watch);
    check_u32("line white", obstacle_plan_line_watch(&watch, false, 200),
              OBSTACLE_LINE_NONE);

    /* Track test 2026-11-02: black for about 25 mm, crossed almost
     * square.  Never handed over; reported as a crossing. */
    check_u32("line cross start", obstacle_plan_line_watch(&watch, true, 290),
              OBSTACLE_LINE_NONE);
    check_u32("line cross mid", obstacle_plan_line_watch(&watch, true, 315),
              OBSTACLE_LINE_NONE);
    check_u32("line crossed", obstacle_plan_line_watch(&watch, false, 320),
              OBSTACLE_LINE_CROSSED);
    check_u32("line after cross", obstacle_plan_line_watch(&watch, false, 330),
              OBSTACLE_LINE_NONE);

    /* Running along a line: held once black lasts HELD_MM from where it
     * began (not from the previous crossing). */
    check_u32("line along start", obstacle_plan_line_watch(&watch, true, 340),
              OBSTACLE_LINE_NONE);
    check_u32("line along short", obstacle_plan_line_watch(&watch, true,
              340 + held - 1), OBSTACLE_LINE_NONE);
    check_u32("line held", obstacle_plan_line_watch(&watch, true, 340 + held),
              OBSTACLE_LINE_HELD);

    /* A reset forgets black from an earlier leg. */
    obstacle_plan_line_reset(&watch);
    check_u32("line reset white", obstacle_plan_line_watch(&watch, false, 0),
              OBSTACLE_LINE_NONE);
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
    test_scan_reduce();
    test_scan_decisions();
    test_scan_real();
    test_scan_path();
    test_plan_bypass();
    test_plan_line_watch();

    (void)printf("%u checks, %u failures\n",
                 (unsigned int)checks, (unsigned int)failures);

    return ((0U == failures) ? 0 : 1);
}

/*** end of file ***/
