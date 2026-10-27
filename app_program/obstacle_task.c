/** @file obstacle_task.c
 *
 * @brief Buddy 5 obstacle task, MONITOR state.
 *
 * One loop per reading:  measure -> obstacle_monitor_step() -> pack and
 * publish the status word -> update the event-flag bits.
 *
 * Sharing with other tasks and the other core:
 *   - The status word is one aligned 32-bit store, which the Cortex-M0+
 *     never splits, so readers need no lock.  Its timestamp is written
 *     after it, so a reader racing an update can only see a timestamp that
 *     is older than the word: the age check errs towards "stale".
 *   - Only this task writes the event-flag bits.  The controller's
 *     acknowledgement arrives as a request bit that this task consumes.
 *   - Timing uses the kernel clock, not the RP2040 TIMER, which runs at
 *     the wrong rate (see evidence/ultrasonic/README.md).
 */

#include <stdint.h>
#include <stdbool.h>

#include <tk/tkernel.h>

#include "obstacle_config.h"
#include "obstacle_types.h"
#include "obstacle_calc.h"
#include "obstacle_monitor.h"
#include "obstacle_servo.h"
#include "obstacle_ultrasonic.h"
#include "obstacle_task.h"

/* Request from obstacle_task_ack_ahead(), consumed by the task. */
#define TASK_REQ_ACK        (0x00000100U)

/* A reader retries this often if the word changes while it reads. */
#define TASK_READ_TRIES     (3U)

static UW task_stack[OBSTACLE_TASK_STACK_BYTES / sizeof(UW)];

static ID   flag_id = 0;
static bool b_initialised = false;
static bool b_started = false;

/* Owned by the task. */
static obstacle_monitor_t monitor;

/* Published by the task, read by anyone (see the file comment). */
static uint32_t volatile status_word = 0U;
static uint32_t volatile status_time_ms = 0U;
static bool volatile     b_published = false;


/*!
 * @brief Kernel time in milliseconds (wraps after about 49 days).
 *
 * @return Low 32 bits of the kernel's operating time.
 */
static uint32_t
task_now_ms(void)
{
    SYSTIM now;

    now.hi = 0;
    now.lo = 0U;
    (void)tk_get_otm(&now);

    /* Cast to uint32_t: only differences of the low word are used. */
    return ((uint32_t)now.lo);
}

/*!
 * @brief Pack and publish one status, then its timestamp.
 *
 * @param[in] p_report Status to publish.
 */
static void
task_publish(obstacle_status_t const * p_report)
{
    status_word = obstacle_calc_pack_status(p_report);
    status_time_ms = task_now_ms();
    b_published = true;
}

/*!
 * @brief Make the event-flag bits match the monitor's decisions.
 */
static void
task_update_flags(void)
{
    if (monitor.b_ahead)
    {
        (void)tk_set_flg(flag_id, OBSTACLE_EVT_AHEAD);
    }
    else
    {
        /* tk_clr_flg() keeps the bits that are set in its pattern. */
        (void)tk_clr_flg(flag_id, ~OBSTACLE_EVT_AHEAD);
    }

    if (monitor.b_fault)
    {
        (void)tk_set_flg(flag_id, OBSTACLE_EVT_FAULT);
    }
    else
    {
        (void)tk_clr_flg(flag_id, ~OBSTACLE_EVT_FAULT);
    }
}

/*!
 * @brief Task body: range forward forever.
 *
 * @param[in] stacd   Start code (unused).
 * @param[in] p_exinf Extended information (unused).
 */
static void
task_entry(INT stacd, void * p_exinf)
{
    obstacle_range_t  range;
    obstacle_status_t report;
    uint32_t          seq = 0U;
    UINT              pattern = 0U;

    (void)stacd;
    (void)p_exinf;

    obstacle_monitor_reset(&monitor);
    (void)obstacle_servo_set_angle(OBSTACLE_FORWARD_DEG, NULL);

    for (;;)
    {
        /* Sleeps about 60 ms per reading, so lower-priority tasks run. */
        obstacle_ultrasonic_measure(&range);
        obstacle_monitor_step(&monitor, &range, &report);

        /* Consume an acknowledgement (polling, clears only that bit). */
        if (E_OK == tk_wai_flg(flag_id, TASK_REQ_ACK,
                               TWF_ORW | TWF_BITCLR, &pattern, TMO_POL))
        {
            obstacle_monitor_ack(&monitor);
        }

        report.seq = seq;
        seq++;

        task_publish(&report);
        task_update_flags();
    }
}

/*!
 * @brief Start the drivers and create the event flag.
 *
 * @return true if the obstacle task can be started.
 */
bool
obstacle_task_init(void)
{
    T_CFLG cflg = { 0 };

    if (!b_initialised)
    {
        cflg.flgatr  = TA_TFIFO | TA_WMUL;
        cflg.iflgptn = 0U;

        if (obstacle_ultrasonic_init() && obstacle_servo_init())
        {
            flag_id = tk_cre_flg(&cflg);
            b_initialised = (flag_id > 0);
        }
    }

    return (b_initialised);
}

/*!
 * @brief Create and start the obstacle task.
 *
 * @return true if the task is running.
 */
bool
obstacle_task_start(void)
{
    T_CTSK ctsk = { 0 };
    ID     task_id;

    if (b_initialised && (!b_started))
    {
        ctsk.tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF;
        ctsk.task    = task_entry;
        ctsk.itskpri = OBSTACLE_TASK_PRIORITY;
        ctsk.stksz   = (SZ)sizeof(task_stack);  /* Cast: small constant. */
        ctsk.bufptr  = task_stack;
        ctsk.assprc  = 0U;
#if defined(CNF_SMP) && (0 != CNF_SMP)
        ctsk.tskatr |= TA_ASSPRC;
        ctsk.assprc  = TP_PRC2;
#endif

        task_id = tk_cre_tsk(&ctsk);

        if (task_id > 0)
        {
            b_started = (E_OK == tk_sta_tsk(task_id, 0));
        }
    }

    return (b_started);
}

/*!
 * @brief Event flag carrying the OBSTACLE_EVT_* bits.
 *
 * @return Flag ID, or 0 before obstacle_task_init().
 */
ID
obstacle_task_get_flag(void)
{
    return (flag_id);
}

/*!
 * @brief Latest status, decoded, if it is fresh.
 *
 * @param[out] p_status Decoded status; written only when true is returned.
 *
 * @return false if there is no status yet, it is older than
 *         OBSTACLE_STALE_MS, or it could not be read consistently.
 */
bool
obstacle_task_get_status(obstacle_status_t * p_status)
{
    bool     b_fresh = false;
    bool     b_stable = false;
    uint32_t first = 0U;
    uint32_t second;
    uint32_t stamp_ms = 0U;
    uint32_t tries;

    if ((NULL != p_status) && b_published)
    {
        /* The word changes (seq) on every update, so two equal reads
         * around the timestamp mean the timestamp is not newer than it. */
        for (tries = 0U; (tries < TASK_READ_TRIES) && (!b_stable); tries++)
        {
            first = status_word;
            stamp_ms = status_time_ms;
            second = status_word;
            b_stable = (first == second);
        }

        if (b_stable && ((task_now_ms() - stamp_ms) <= OBSTACLE_STALE_MS))
        {
            b_fresh = obstacle_calc_unpack_status(first, p_status);
        }
    }

    return (b_fresh);
}

/*!
 * @brief Latest packed status word, for telemetry.
 *
 * @return The word; before the first reading it packs as NOT_READY.
 */
uint32_t
obstacle_task_get_word(void)
{
    uint32_t word = obstacle_calc_pack_status(NULL);

    if (b_published)
    {
        word = status_word;
    }

    return (word);
}

/*!
 * @brief Acknowledge OBSTACLE_EVT_AHEAD.
 */
void
obstacle_task_ack_ahead(void)
{
    if (b_initialised)
    {
        (void)tk_set_flg(flag_id, TASK_REQ_ACK);
    }
}

/*** end of file ***/
