/** @file ultrasonic_test_main.c
 *
 * @brief Buddy 5 bench test for the HC-SR04 (Stage 3 / Stage 7A).
 *
 * Build:  make APP=ultrasonic_test BOARD=robo_pico_robot SMP=1 \
 *              CONSOLE=usb_cdc PICO_SDK_PATH=...
 *
 * Streams one CSV row per measurement.  Keys on the USB console:
 *   b  batch of 10 readings with min / median / max / mean / spread
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
    "[ultra] keys: b=batch of 10  p=pause/resume  h=help\n";
static UB const msg_header[] =
    "seq,time_ms,echo_us,distance_mm,status\n";
static UB const msg_row[] = "%u,%u,%u,%u,%s\n";
static UB const msg_paused[] = "[ultra] stream paused (p to resume)\n";
static UB const msg_resumed[] = "[ultra] stream resumed\n";
static UB const msg_batch_start[] = "[ultra] batch of %u readings\n";
static UB const msg_batch_result[] =
    "batch,valid=%u/%u,min=%u,median=%u,max=%u,mean=%u,spread=%u\n";
static UB const msg_batch_none[] = "batch,valid=0/%u (no usable readings)\n";

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
