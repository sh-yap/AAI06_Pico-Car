/** @file servo_test_main.c
 *
 * @brief Buddy 5 bench test for the SG90 scan servo (Stage 3 / Stage 7B).
 *
 * Build:  make APP=servo_test BOARD=robo_pico_robot SMP=1 \
 *              CONSOLE=usb_cdc PICO_SDK_PATH=...
 *
 * Keys on the USB console:
 *   a / d    angle -5 / +5 deg        A / D    angle -1 / +1 deg
 *   c        centre (90 deg)          1..5     30 / 60 / 90 / 120 / 150 deg
 *   [ / ]    raw pulse -10 / +10 us   { / }    raw pulse -50 / +50 us
 *   s        coarse scan 30..150 deg with the ultrasonic sensor
 *   t        settle test: swing 30 -> 150 deg and range every ping
 *   h        help
 *
 * Angle keys respect the safe range in obstacle_config.h.  Raw pulse keys
 * ignore it (absolute limits still apply) and are for calibration only:
 * step slowly and stop as soon as the servo buzzes at an end stop.
 */

#include <stdint.h>
#include <stdbool.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <sys/sysdef.h>

#include "obstacle_config.h"
#include "obstacle_types.h"
#include "obstacle_calc.h"
#include "obstacle_servo.h"
#include "obstacle_ultrasonic.h"
#include "obstacle_console.h"

#define TEST_STEP_SMALL_DEG     (1U)
#define TEST_STEP_LARGE_DEG     (5U)
#define TEST_STEP_SMALL_US      (10U)
#define TEST_STEP_LARGE_US      (50U)
#define TEST_CENTRE_DEG         (90U)
#define TEST_SETTLE_FROM_DEG    (30U)
#define TEST_SETTLE_TO_DEG      (150U)
#define TEST_SETTLE_PARK_MS     (1000)  /* TMO: rest before the swing.   */
#define TEST_SETTLE_WINDOW_MS   (1000U) /* How long to range after swing. */
#define TEST_IDLE_POLL_MS       (20)    /* TMO: key polling.             */
#define TEST_US_PER_MS          (1000U)

/* Coarse scan angles from the project brief, also reached by keys 1..5. */
static uint32_t const scan_angles[] = { 30U, 60U, 90U, 120U, 150U };

#define TEST_SCAN_COUNT (sizeof(scan_angles) / sizeof(scan_angles[0]))

/* Console messages.  Declared as UB arrays because tm_printf() takes
 * const UB *; this avoids a cast at every call. */
static UB const msg_banner[] =
    "\n=== Buddy 5: servo_test (SG90) ===\n"
    "Servo=GP%u  safe %u..%u deg  pulse 0/90/180 = %u/%u/%u us\n";
static UB const msg_servo_failed[] =
    "[servo] init FAILED: PWM held in reset. "
    "Rebuild with BOARD=robo_pico_robot.\n";
static UB const msg_ultra_failed[] =
    "[servo] ultrasonic init FAILED; s and t will report NOT_READY.\n";
static UB const msg_help[] =
    "[servo] keys: a/d -+5deg  A/D -+1deg  c centre  1..5 30..150deg\n"
    "[servo]       [/] -+10us  {/} -+50us (raw, calibration only)\n"
    "[servo]       s coarse scan  t settle test  h help\n";
static UB const msg_angle[] = "[servo] angle=%u deg pulse=%u us settle~%u ms\n";
static UB const msg_raw[] = "[servo] RAW pulse=%u us (angle not tracked)\n";
static UB const msg_scan_header[] = "scan,angle_deg,distance_mm,status\n";
static UB const msg_scan_row[] = "scan,%u,%u,%s\n";
static UB const msg_settle_header[] =
    "[servo] settle test: park %u deg, swing to %u deg (predicted %u ms)\n"
    "settle,t_ms,distance_mm,status\n";
static UB const msg_settle_row[] = "settle,%u,%u,%s\n";


/*!
 * @brief Move to an angle and report where the servo was sent.
 *
 * @param[in] angle_deg Requested angle (the driver clamps it).
 */
static void
test_move(uint32_t angle_deg)
{
    uint32_t settle_ms = 0U;

    (void)obstacle_servo_set_angle(angle_deg, &settle_ms);
    (void)tm_printf(msg_angle,
                    obstacle_servo_get_angle(),
                    obstacle_servo_get_pulse_us(),
                    settle_ms);
}

/*!
 * @brief Step the raw pulse width, for finding end stops and centre.
 *
 * @param[in] step_us    Size of the step.
 * @param[in] b_increase true to lengthen the pulse, false to shorten it.
 */
static void
test_step_pulse(uint32_t step_us, bool b_increase)
{
    uint32_t pulse_us = obstacle_servo_get_pulse_us();

    if (b_increase)
    {
        pulse_us += step_us;
    }
    else if (pulse_us > step_us)
    {
        pulse_us -= step_us;
    }
    else
    {
        pulse_us = 0U;
    }

    (void)obstacle_servo_set_pulse_us(pulse_us);
    (void)tm_printf(msg_raw, obstacle_servo_get_pulse_us());
}

/*!
 * @brief Step the angle down, stopping at zero (the driver then clamps).
 *
 * @param[in] step_deg Size of the step.
 */
static void
test_step_down(uint32_t step_deg)
{
    uint32_t angle_deg = obstacle_servo_get_angle();

    if (angle_deg > step_deg)
    {
        angle_deg -= step_deg;
    }
    else
    {
        angle_deg = 0U;
    }

    test_move(angle_deg);
}

/*!
 * @brief Coarse scan: range once at each brief angle, then re-centre.
 *
 * A preview of Stage 7C with the car stationary.
 */
static void
test_coarse_scan(void)
{
    obstacle_range_t range;
    uint32_t         settle_ms = 0U;
    uint32_t         idx;

    (void)tm_printf(msg_scan_header);

    for (idx = 0U; idx < TEST_SCAN_COUNT; idx++)
    {
        (void)obstacle_servo_set_angle(scan_angles[idx], &settle_ms);

        /* Cast to TMO: settle_ms is at most a few hundred ms. */
        (void)tk_dly_tsk((TMO)settle_ms);

        obstacle_ultrasonic_measure(&range);
        (void)tm_printf(msg_scan_row,
                        obstacle_servo_get_angle(),
                        range.distance_mm,
                        obstacle_console_status_name(range.status));
    }

    test_move(TEST_CENTRE_DEG);
}

/*!
 * @brief Settle test: swing across the scan range while ranging.
 *
 * Point the sensor so the far angle faces a wall.  The distance keeps
 * changing while the horn moves and wobbles; the time at which it stops
 * changing is the real settling time, to compare with the prediction.
 */
static void
test_settle(void)
{
    obstacle_range_t range;
    uint32_t         predicted_ms = 0U;
    uint32_t         start_us;
    uint32_t         elapsed_ms = 0U;

    (void)obstacle_servo_set_angle(TEST_SETTLE_FROM_DEG, NULL);
    (void)tk_dly_tsk(TEST_SETTLE_PARK_MS);

    (void)obstacle_servo_set_angle(TEST_SETTLE_TO_DEG, &predicted_ms);
    start_us = in_w(TIMER_TIMERAWL);

    (void)tm_printf(msg_settle_header,
                    TEST_SETTLE_FROM_DEG,
                    TEST_SETTLE_TO_DEG,
                    predicted_ms);

    while (elapsed_ms < TEST_SETTLE_WINDOW_MS)
    {
        obstacle_ultrasonic_measure(&range);
        elapsed_ms = (range.timestamp_us - start_us) / TEST_US_PER_MS;
        (void)tm_printf(msg_settle_row,
                        elapsed_ms,
                        range.distance_mm,
                        obstacle_console_status_name(range.status));
    }

    test_move(TEST_CENTRE_DEG);
}

/*!
 * @brief Act on one console key.
 *
 * @param[in] key Key code from obstacle_console_read_key().
 */
static void
test_handle_key(int32_t key)
{
    switch (key)
    {
        case 'a':
            test_step_down(TEST_STEP_LARGE_DEG);
            break;

        case 'd':
            test_move(obstacle_servo_get_angle() + TEST_STEP_LARGE_DEG);
            break;

        case 'A':
            test_step_down(TEST_STEP_SMALL_DEG);
            break;

        case 'D':
            test_move(obstacle_servo_get_angle() + TEST_STEP_SMALL_DEG);
            break;

        case 'c':
        case 'C':
            test_move(TEST_CENTRE_DEG);
            break;

        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
            /* Cast to uint32_t: key is '1'..'5' here, so the index is
             * 0..4, inside scan_angles[]. */
            test_move(scan_angles[(uint32_t)(key - '1')]);
            break;

        case '[':
            test_step_pulse(TEST_STEP_SMALL_US, false);
            break;

        case ']':
            test_step_pulse(TEST_STEP_SMALL_US, true);
            break;

        case '{':
            test_step_pulse(TEST_STEP_LARGE_US, false);
            break;

        case '}':
            test_step_pulse(TEST_STEP_LARGE_US, true);
            break;

        case 's':
        case 'S':
            test_coarse_scan();
            break;

        case 't':
        case 'T':
            test_settle();
            break;

        case 'h':
        case 'H':
            (void)tm_printf(msg_help);
            break;

        default:
            /* No key, or a key this test does not use. */
            break;
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
    obstacle_console_wait_host();

    (void)tm_printf(msg_banner,
                    OBSTACLE_SERVO_PIN,
                    OBSTACLE_SERVO_ANGLE_MIN_DEG,
                    OBSTACLE_SERVO_ANGLE_MAX_DEG,
                    OBSTACLE_SERVO_PULSE_0_US,
                    OBSTACLE_SERVO_PULSE_90_US,
                    OBSTACLE_SERVO_PULSE_180_US);

    if (!obstacle_servo_init())
    {
        (void)tm_printf(msg_servo_failed);
    }

    if (!obstacle_ultrasonic_init())
    {
        (void)tm_printf(msg_ultra_failed);
    }

    (void)tm_printf(msg_help);
    test_move(TEST_CENTRE_DEG);

    for (;;)
    {
        test_handle_key(obstacle_console_read_key());
        (void)tk_dly_tsk(TEST_IDLE_POLL_MS);
    }

    return (0);
}

/*** end of file ***/
