/** @file obstacle_task_test_main.c
 *
 * @brief Buddy 5 test for the obstacle task, with a stand-in controller.
 *
 * Build:  make APP=obstacle_task_test BOARD=robo_pico_robot SMP=1 \
 *              CONSOLE=usb_cdc PICO_SDK_PATH=...
 *
 * Runs the real obstacle task and plays the Vehicle Controller: it waits
 * on the task's event flag, prints every change, and acknowledges
 * OBSTACLE_AHEAD when you press 'a' (a real controller would stop the car
 * and ask for a scan instead).
 *
 * Keys on the USB console:
 *   a   acknowledge OBSTACLE_AHEAD
 *   p   pause or resume the status rows (events are always printed)
 *   h   help
 *
 * Output:
 *   mon,t_ms,seq,distance_mm,status,state,ahead,fault   about 5 per second
 *   [ctrl] ...                                           on every event
 */

#include <stdint.h>
#include <stdbool.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "obstacle_config.h"
#include "obstacle_types.h"
#include "obstacle_task.h"
#include "obstacle_console.h"

#define TEST_POLL_MS        (200)   /* TMO: one status row per poll.     */
#define TEST_KEY_ACK        ('a')
#define TEST_KEY_PAUSE      ('p')
#define TEST_KEY_HELP       ('h')

/* Console messages.  Declared as UB arrays because tm_printf() takes
 * const UB *; this avoids a cast at every call. */
static UB const msg_banner[] =
    "\n=== Buddy 5: obstacle_task_test (MONITOR) ===\n"
    "stop < %u mm x%u  blind latch < %u mm  lost: near < %u mm, "
    "jump > %u mm x%u\n";
static UB const msg_init_failed[] =
    "[ctrl] obstacle_task_init FAILED: PWM held in reset? "
    "Rebuild with BOARD=robo_pico_robot.\n";
static UB const msg_start_failed[] = "[ctrl] obstacle_task_start FAILED\n";
static UB const msg_help[] =
    "[ctrl] keys: a=acknowledge OBSTACLE_AHEAD  p=pause/resume rows"
    "  h=help\n";
static UB const msg_header[] =
    "mon,t_ms,seq,distance_mm,status,state,ahead,fault\n";
static UB const msg_row[] = "mon,%u,%u,%u,%s,%s,%u,%u\n";
static UB const msg_stale[] = "mon,%u,no fresh status (older than %u ms)\n";
static UB const msg_ahead_on[] =
    "[ctrl] OBSTACLE_AHEAD set: a controller would STOP and scan now "
    "(press a to acknowledge)\n";
static UB const msg_ahead_off[] = "[ctrl] OBSTACLE_AHEAD cleared\n";
static UB const msg_fault_on[] = "[ctrl] SENSOR_FAULT set\n";
static UB const msg_fault_off[] = "[ctrl] SENSOR_FAULT cleared\n";
static UB const msg_acked[] = "[ctrl] acknowledged OBSTACLE_AHEAD\n";
static UB const msg_paused[] = "[ctrl] rows paused (p to resume)\n";
static UB const msg_resumed[] = "[ctrl] rows resumed\n";

static bool b_rows_on = true;


/*!
 * @brief Short text name for an obstacle task state.
 *
 * @param[in] state State to name.
 *
 * @return Constant string.
 */
static char const *
test_state_name(obstacle_state_t state)
{
    char const * p_name = "?";

    switch (state)
    {
        case OBSTACLE_STATE_INIT:
            p_name = "INIT";
            break;

        case OBSTACLE_STATE_MONITOR:
            p_name = "MONITOR";
            break;

        case OBSTACLE_STATE_SCANNING:
            p_name = "SCANNING";
            break;

        case OBSTACLE_STATE_SIDE_WATCH:
            p_name = "SIDE_WATCH";
            break;

        case OBSTACLE_STATE_LINE_RECOVERY:
            p_name = "LINE_RECOVERY";
            break;

        case OBSTACLE_STATE_SENSOR_FAULT:
            p_name = "SENSOR_FAULT";
            break;

        default:
            /* Unknown state: keep "?". */
            break;
    }

    return (p_name);
}

/*!
 * @brief Kernel time in milliseconds.
 *
 * @return Low 32 bits of the kernel's operating time.
 */
static uint32_t
test_now_ms(void)
{
    SYSTIM now;

    now.hi = 0;
    now.lo = 0U;
    (void)tk_get_otm(&now);

    /* Cast to uint32_t: only the low word is printed. */
    return ((uint32_t)now.lo);
}

/*!
 * @brief Print one status row, or say that there is no fresh status.
 *
 * @param[in] flags Current event-flag pattern.
 */
static void
test_print_row(UINT flags)
{
    obstacle_status_t status;

    if (obstacle_task_get_status(&status))
    {
        (void)tm_printf(msg_row,
                        test_now_ms(),
                        status.seq,
                        status.distance_mm,
                        obstacle_console_status_name(status.range_status),
                        test_state_name(status.state),
                        (0U != (flags & OBSTACLE_EVT_AHEAD)) ? 1U : 0U,
                        (0U != (flags & OBSTACLE_EVT_FAULT)) ? 1U : 0U);
    }
    else
    {
        (void)tm_printf(msg_stale, test_now_ms(), OBSTACLE_STALE_MS);
    }
}

/*!
 * @brief Print the events whose bits changed since the last check.
 *
 * @param[in] previous Pattern seen last time.
 * @param[in] current  Pattern now.
 */
static void
test_print_events(UINT previous, UINT current)
{
    UINT changed = previous ^ current;

    if (0U != (changed & OBSTACLE_EVT_AHEAD))
    {
        (void)tm_printf((0U != (current & OBSTACLE_EVT_AHEAD)) ?
                        msg_ahead_on : msg_ahead_off);
    }

    if (0U != (changed & OBSTACLE_EVT_FAULT))
    {
        (void)tm_printf((0U != (current & OBSTACLE_EVT_FAULT)) ?
                        msg_fault_on : msg_fault_off);
    }
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
        case TEST_KEY_ACK:
            obstacle_task_ack_ahead();
            (void)tm_printf(msg_acked);
            break;

        case TEST_KEY_PAUSE:
            b_rows_on = !b_rows_on;
            (void)tm_printf(b_rows_on ? msg_resumed : msg_paused);
            break;

        case TEST_KEY_HELP:
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
    T_RFLG flag_info;
    UINT   previous = 0U;
    UINT   current = 0U;
    ER     result;

    obstacle_console_wait_host();

    (void)tm_printf(msg_banner,
                    OBSTACLE_STOP_DISTANCE_MM,
                    OBSTACLE_STOP_READINGS,
                    OBSTACLE_BLIND_LATCH_MM,
                    OBSTACLE_LOST_NEAR_MM,
                    OBSTACLE_LOST_JUMP_MM,
                    OBSTACLE_LOST_READINGS);

    if (!obstacle_task_init())
    {
        (void)tm_printf(msg_init_failed);
    }
    else if (!obstacle_task_start())
    {
        (void)tm_printf(msg_start_failed);
    }
    else
    {
        (void)tm_printf(msg_help);
        (void)tm_printf(msg_header);
    }

    for (;;)
    {
        /* Poll: the bits are levels, so waiting on them would return at
         * once for as long as OBSTACLE_AHEAD stays set. */
        (void)tk_dly_tsk(TEST_POLL_MS);

        result = tk_ref_flg(obstacle_task_get_flag(), &flag_info);
        current = (E_OK == result) ?
                  (flag_info.flgptn & (OBSTACLE_EVT_AHEAD
                                       | OBSTACLE_EVT_FAULT)) : 0U;

        test_print_events(previous, current);
        previous = current;

        if (b_rows_on)
        {
            test_print_row(current);
        }

        test_handle_key(obstacle_console_read_key());
    }

    return (0);
}

/*** end of file ***/
