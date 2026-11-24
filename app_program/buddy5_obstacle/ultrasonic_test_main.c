/** @file ultrasonic_test_main.c
 *
 * @brief Buddy 5 bench test for the HC-SR04 (Stage 3 / Stage 7A).
 *
 * Build:  make APP=ultrasonic_test BOARD=robo_pico_robot SMP=1 \
 *              CONSOLE=usb_cdc PICO_SDK_PATH=...
 *
 * Streams one CSV row per measurement.  Keys on the USB console:
 *   b  batch of 10 readings with min / median / max / mean / spread
 *   d  diagnose wiring: watch the Echo pin directly (3 pings)
 *   p  pause or resume the stream
 *   h  help
 *
 * Calibration: place a flat board at a measured distance, press b, and
 * copy the rows into evidence/ultrasonic/.
 * Procedure: docs/project/BUDDY5_OBSTACLE.md.
 */

#include <stdint.h>
#include <stdbool.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "obstacle_config.h"
#include "obstacle_types.h"
#include "obstacle_calc.h"
#include "obstacle_ultrasonic.h"
#include "obstacle_console.h"

#define TEST_BATCH_SIZE      (10U)
#define TEST_PROBE_COUNT     (3U)
#define TEST_TIMER_PCT_DIV   (1000U)  /* 100000 us per 100 ms = 100 %.  */
#define TEST_TIMER_PCT_MIN   (90U)
#define TEST_TIMER_PCT_MAX   (125U)   /* A 10 ms tick may add a tick.   */
#define TEST_STREAM_GAP_MS   (140)    /* TMO: ~5 rows per second.     */
#define TEST_IDLE_POLL_MS    (50)     /* TMO: key polling when paused. */
#define TEST_US_PER_MS       (1000U)

/* Console messages.  Declared as UB arrays because tm_printf() takes
 * const UB *; this avoids a cast at every call. */
static UB const msg_banner[] =
    "\n=== Buddy 5: ultrasonic_test (HC-SR04) ===\n"
    "Trig=GP%u Echo=GP%u  range %u..%u us  sound %u m/s\n";
static UB const msg_init_failed[] =
    "[ultra] init FAILED: PWM held in reset. "
    "Rebuild with BOARD=robo_pico_robot.\n";
static UB const msg_help[] =
    "[ultra] keys: b=batch of 10  d=diagnose wiring  p=pause/resume"
    "  h=help\n";
static UB const msg_header[] =
    "seq,time_ms,echo_us,distance_mm,status\n";
static UB const msg_row[] = "%u,%u,%u,%u,%s\n";
static UB const msg_paused[] = "[ultra] stream paused (p to resume)\n";
static UB const msg_resumed[] = "[ultra] stream resumed\n";
static UB const msg_batch_start[] = "[ultra] batch of %u readings\n";
static UB const msg_batch_result[] =
    "batch,valid=%u/%u,min=%u,median=%u,max=%u,mean=%u,spread=%u\n";
static UB const msg_batch_none[] = "batch,valid=0/%u (no usable readings)\n";
static UB const msg_probe_start[] =
    "[ultra] diagnose: Trig=GP%u Echo=GP%u, watching Echo for 50 ms\n"
    "probe,high_before,rose,rise_us,fell,fall_us,pin_width_us,counter_us\n";
static UB const msg_probe_row[] = "probe,%u,%u,%u,%u,%u,%u,%u\n";
static UB const msg_verdict[] = "[ultra] verdict: %s\n";
static UB const msg_pin_start[] =
    "[ultra] pin test: Trig held high ~10 ms, both pins read back\n"
    "pin,trig_high,trig_low,echo_high,echo_low,counter_us,timer_pct\n";
static UB const msg_pin_row[] = "pin,%u,%u,%u,%u,%u,%u\n";
static UB const msg_pin_check[] = "[ultra] %s\n";

static char const pin_trig_ok[] = "Trig pin: OK, it follows the software";
static char const pin_trig_bad[] =
    "Trig pin: FAULT, the pin does not follow the software";
static char const pin_echo_ok[] =
    "Echo pin: sees Trig (correct ONLY with the loopback jumper)";
static char const pin_echo_none[] =
    "Echo pin: saw nothing (normal with the sensor; in loopback the "
    "jumper or the servo-port input is the problem)";
static char const pin_echo_stuck[] = "Echo pin: stuck high";
static char const pin_counter_ok[] = "Echo counter: OK, counted the pulse";
static char const pin_counter_bad[] =
    "Echo counter: 0 although Echo was high, software fault, report this";
static char const pin_counter_na[] = "Echo counter: not tested (no pulse)";
static char const pin_timer_ok[] = "Microsecond timer: OK";
static char const pin_timer_bad[] =
    "Microsecond timer: WRONG RATE (timer_pct should be ~100), kernel "
    "clock setup issue, report this";

static char const verdict_stuck[] =
    "ECHO STUCK HIGH before triggering - Echo wire on a V+ pin?";
static char const verdict_silent[] =
    "ECHO NEVER WENT HIGH - sensor not firing: check VCC/GND, try "
    "swapping Trig/Echo, or the sensor needs 5 V";
static char const verdict_no_end[] =
    "ECHO WENT HIGH BUT NEVER ENDED - nothing reflected, or wiring";
static char const verdict_counter[] =
    "PIN OK BUT COUNTER 0 - software echo-timing fault, report this";
static char const verdict_ok[] =
    "ECHO OK - wiring and echo timing both work";

static uint32_t row_seq = 0U;


/*!
 * @brief Print one measurement as a CSV row.
 *
 * @param[in] p_range Measurement to print.
 */
static void
test_print_row(obstacle_range_t const * p_range)
{
    row_seq++;

    (void)tm_printf(msg_row,
                    row_seq,
                    p_range->timestamp_us / TEST_US_PER_MS,
                    p_range->echo_us,
                    p_range->distance_mm,
                    obstacle_console_status_name(p_range->status));
}

/*!
 * @brief Take TEST_BATCH_SIZE readings and print their statistics.
 *
 * Only OK readings count towards the statistics; the rest are still
 * printed so failures are visible in the log.
 */
static void
test_run_batch(void)
{
    obstacle_range_t range;
    uint32_t         distances[TEST_BATCH_SIZE];
    uint32_t         valid = 0U;
    uint32_t         sum = 0U;
    uint32_t         idx;
    uint32_t         median;

    (void)tm_printf(msg_batch_start, TEST_BATCH_SIZE);
    (void)tm_printf(msg_header);

    for (idx = 0U; idx < TEST_BATCH_SIZE; idx++)
    {
        obstacle_ultrasonic_measure(&range);
        test_print_row(&range);

        if (OBSTACLE_RANGE_OK == range.status)
        {
            distances[valid] = range.distance_mm;
            sum += range.distance_mm;
            valid++;
        }
    }

    if (0U == valid)
    {
        (void)tm_printf(msg_batch_none, TEST_BATCH_SIZE);
    }
    else
    {
        /* obstacle_calc_median() sorts, so [0] and [valid-1] are the
         * minimum and maximum afterwards. */
        median = obstacle_calc_median(distances, valid);

        (void)tm_printf(msg_batch_result,
                        valid,
                        TEST_BATCH_SIZE,
                        distances[0],
                        median,
                        distances[valid - 1U],
                        sum / valid,
                        distances[valid - 1U] - distances[0]);
    }
}

/*!
 * @brief Explain one probe result in plain words.
 *
 * @param[in] p_probe Result from obstacle_ultrasonic_probe().
 *
 * @return Constant verdict string, never NULL.
 */
static char const *
test_probe_verdict(obstacle_probe_t const * p_probe)
{
    char const * p_verdict = verdict_ok;

    if (p_probe->b_high_before)
    {
        p_verdict = verdict_stuck;
    }
    else if (!p_probe->b_rose)
    {
        p_verdict = verdict_silent;
    }
    else if (!p_probe->b_fell)
    {
        p_verdict = verdict_no_end;
    }
    else if (0U == p_probe->counter_us)
    {
        p_verdict = verdict_counter;
    }
    else
    {
        p_verdict = verdict_ok;
    }

    return (p_verdict);
}

/*!
 * @brief Diagnose the wiring: probe TEST_PROBE_COUNT times and explain.
 *
 * pin_width_us (sampled) and counter_us (hardware) should agree to within
 * a few microseconds when everything works.
 */
static void
test_run_probe(void)
{
    obstacle_probe_t    probe;
    obstacle_pin_test_t pins;
    uint32_t            idx;
    uint32_t            width_us;
    uint32_t            timer_pct;
    char const *        p_check;

    obstacle_ultrasonic_pin_test(&pins);
    timer_pct = pins.timer_us_per_100ms / TEST_TIMER_PCT_DIV;

    /* Casts to uint32_t: bool is 0 or 1, printed as a number. */
    (void)tm_printf(msg_pin_start);
    (void)tm_printf(msg_pin_row,
                    (uint32_t)pins.b_trig_high_ok,
                    (uint32_t)pins.b_trig_low_ok,
                    (uint32_t)pins.b_echo_high,
                    (uint32_t)pins.b_echo_low,
                    pins.counter_us,
                    timer_pct);

    if (pins.b_trig_high_ok && pins.b_trig_low_ok)
    {
        p_check = pin_trig_ok;
    }
    else
    {
        p_check = pin_trig_bad;
    }
    (void)tm_printf(msg_pin_check, p_check);

    if (pins.b_echo_high && pins.b_echo_low)
    {
        p_check = pin_echo_ok;
    }
    else if (pins.b_echo_high)
    {
        p_check = pin_echo_stuck;
    }
    else
    {
        p_check = pin_echo_none;
    }
    (void)tm_printf(msg_pin_check, p_check);

    if (!pins.b_echo_high)
    {
        p_check = pin_counter_na;
    }
    else if (0U == pins.counter_us)
    {
        p_check = pin_counter_bad;
    }
    else
    {
        p_check = pin_counter_ok;
    }
    (void)tm_printf(msg_pin_check, p_check);

    if ((timer_pct >= TEST_TIMER_PCT_MIN) && (timer_pct <= TEST_TIMER_PCT_MAX))
    {
        p_check = pin_timer_ok;
    }
    else
    {
        p_check = pin_timer_bad;
    }
    (void)tm_printf(msg_pin_check, p_check);

    (void)tm_printf(msg_probe_start, OBSTACLE_TRIG_PIN, OBSTACLE_ECHO_PIN);

    for (idx = 0U; idx < TEST_PROBE_COUNT; idx++)
    {
        obstacle_ultrasonic_probe(&probe);

        width_us = 0U;

        if (probe.b_fell)
        {
            width_us = probe.fall_us - probe.rise_us;
        }

        /* Casts to uint32_t: bool is 0 or 1, printed as a number. */
        (void)tm_printf(msg_probe_row,
                        (uint32_t)probe.b_high_before,
                        (uint32_t)probe.b_rose,
                        probe.rise_us,
                        (uint32_t)probe.b_fell,
                        probe.fall_us,
                        width_us,
                        probe.counter_us);
        (void)tm_printf(msg_verdict, test_probe_verdict(&probe));
    }
}

/*!
 * @brief Program entry, called by the kernel's initial task.
 *
 * Deviation from BARR-C 5.2: the INT return type is fixed by micro
 * T-Kernel.  This function must never return.
 */
EXPORT INT
usermain(void)
{
    obstacle_range_t range;
    int32_t          key;
    bool             b_streaming = true;

    obstacle_console_wait_host();

    (void)tm_printf(msg_banner,
                    OBSTACLE_TRIG_PIN,
                    OBSTACLE_ECHO_PIN,
                    OBSTACLE_ECHO_MIN_US,
                    OBSTACLE_ECHO_MAX_US,
                    OBSTACLE_SOUND_SPEED_M_S);

    if (!obstacle_ultrasonic_init())
    {
        (void)tm_printf(msg_init_failed);
    }

    /* Diagnose the wiring once at start-up, so the verdict is visible
     * even if key presses are not reaching the board. */
    test_run_probe();

    (void)tm_printf(msg_help);
    (void)tm_printf(msg_header);

    for (;;)
    {
        key = obstacle_console_read_key();

        switch (key)
        {
            case 'b':
            case 'B':
                test_run_batch();
                break;

            case 'd':
            case 'D':
                test_run_probe();
                break;

            case 'p':
            case 'P':
                b_streaming = !b_streaming;

                if (b_streaming)
                {
                    (void)tm_printf(msg_resumed);
                    (void)tm_printf(msg_header);
                }
                else
                {
                    (void)tm_printf(msg_paused);
                }
                break;

            case 'h':
            case 'H':
                (void)tm_printf(msg_help);
                break;

            default:
                /* No key, or a key this test does not use. */
                break;
        }

        if (b_streaming)
        {
            obstacle_ultrasonic_measure(&range);
            test_print_row(&range);
            (void)tk_dly_tsk(TEST_STREAM_GAP_MS);
        }
        else
        {
            (void)tk_dly_tsk(TEST_IDLE_POLL_MS);
        }
    }

    return (0);
}

/*** end of file ***/
