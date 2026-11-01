/** @file obstacle_task.c
 *
 * @brief Buddy 5 obstacle task: MONITOR and SCANNING states.
 *
 * MONITOR, one loop per reading:  measure -> obstacle_monitor_step() ->
 * pack and publish the status word -> update the event-flag bits.
 *
 * SCANNING, on obstacle_task_request_scan(): sweep the safe servo range,
 * take OBSTACLE_SCAN_PINGS readings at each angle after the servo settles,
 * analyse them (obstacle_scan.c), publish the result, set
 * OBSTACLE_EVT_SCAN_DONE, face forward again and go back to MONITOR.
 *
 * PATH CHECK, on obstacle_task_request_path_check() (the controller has
 * pivoted the car towards the bypass side): OBSTACLE_SCAN_PINGS readings
 * at each of OBSTACLE_PATH_MAX_POINTS angles, from the bypass side back to
 * straight ahead, judged in obstacle_scan.c, then OBSTACLE_EVT_PATH_DONE.
 *
 * Sharing with other tasks and the other core:
 *   - The status word is one aligned 32-bit store, which the Cortex-M0+
 *     never splits, so readers need no lock.  Its timestamp is written
 *     after it, so a reader racing an update can only see a timestamp that
 *     is older than the word: the age check errs towards "stale".
 *   - The scan and path results are each guarded by a counter that is
 *     odd while they are being written, so a reader can tell whether its
 *     copy is whole.
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
#include "obstacle_scan.h"
#include "obstacle_servo.h"
#include "obstacle_ultrasonic.h"
#include "obstacle_task.h"

/* Request from obstacle_task_ack_ahead(), consumed by the task. */
#define TASK_REQ_ACK        (0x00000100U)

/* Request from obstacle_task_request_scan(), consumed by the task. */
#define TASK_REQ_SCAN       (0x00000200U)

/* Request from obstacle_task_request_path_check(), consumed by the task. */
#define TASK_REQ_PATH       (0x00000400U)

/* During a scan the status is republished at least this often, well
 * inside OBSTACLE_STALE_MS. */
#define TASK_SCAN_PUBLISH_MS (100U)

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

/* Latest scan (see the file comment): written only by the task. */
static obstacle_scan_result_t scan_result;
static uint32_t volatile      scan_counter = 0U;   /* Odd while writing. */
static obstacle_status_t      last_report;

/* Latest path check, guarded the same way. */
static obstacle_path_t   path_result;
static uint32_t volatile path_counter = 0U;      /* Odd while writing. */
static uint32_t volatile path_need_mm = 0U;      /* Set by the request. */
static uint32_t volatile path_known_mm = 0U;     /* Set by the request. */
static obstacle_action_t volatile path_side = OBSTACLE_ACTION_TURN_RIGHT;


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
    last_report = *p_report;
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
 * @brief Publish a SCANNING status with the next sequence number.
 *
 * @param[in,out] p_report Status to publish (seq is updated).
 * @param[in,out] p_seq    Status-word sequence number.
 */
static void
task_publish_scanning(obstacle_status_t * p_report, uint32_t * p_seq)
{
    p_report->state = OBSTACLE_STATE_SCANNING;
    p_report->seq = *p_seq;
    (*p_seq)++;
    task_publish(p_report);
}

/*!
 * @brief Sleep while the servo settles, keeping the status fresh.
 *
 * @param[in]     wait_ms  Total time to sleep.
 * @param[in,out] p_report Status to republish.
 * @param[in,out] p_seq    Status-word sequence number.
 */
static void
task_settle(uint32_t            wait_ms,
            obstacle_status_t * p_report,
            uint32_t *          p_seq)
{
    uint32_t left_ms = wait_ms;
    uint32_t step_ms;

    while (left_ms > 0U)
    {
        step_ms = (left_ms > TASK_SCAN_PUBLISH_MS) ?
                  TASK_SCAN_PUBLISH_MS : left_ms;

        /* Cast to TMO: at most TASK_SCAN_PUBLISH_MS. */
        (void)tk_dly_tsk((TMO)step_ms);
        left_ms -= step_ms;
        task_publish_scanning(p_report, p_seq);
    }
}

/*!
 * @brief SCANNING: sweep, analyse, publish, face forward again.
 *
 * @param[in,out] p_seq Status-word sequence number.
 */
static void
task_scan(uint32_t * p_seq)
{
    obstacle_range_t  pings[OBSTACLE_SCAN_PINGS];
    obstacle_status_t report = last_report;
    uint32_t          start_ms = task_now_ms();
    uint32_t          settle_ms = 0U;
    uint32_t          angle_deg;
    uint32_t          idx;
    uint32_t          ping;

    (void)tk_clr_flg(flag_id, ~OBSTACLE_EVT_SCAN_DONE);
    scan_counter++;                               /* Odd: being written. */

    task_publish_scanning(&report, p_seq);

    for (idx = 0U; idx < OBSTACLE_SCAN_COUNT; idx++)
    {
        angle_deg = OBSTACLE_SERVO_ANGLE_MIN_DEG
                    + (idx * OBSTACLE_SCAN_STEP_DEG);
        (void)obstacle_servo_set_angle(angle_deg, &settle_ms);
        task_settle(settle_ms, &report, p_seq);

        for (ping = 0U; ping < OBSTACLE_SCAN_PINGS; ping++)
        {
            obstacle_ultrasonic_measure(&pings[ping]);
            task_publish_scanning(&report, p_seq);
        }

        obstacle_scan_reduce(angle_deg, pings, OBSTACLE_SCAN_PINGS,
                             &scan_result.points[idx]);
    }

    scan_result.count = OBSTACLE_SCAN_COUNT;
    obstacle_scan_analyse(&scan_result);
    scan_result.duration_ms = task_now_ms() - start_ms;

    scan_counter++;                               /* Even: complete.     */
    (void)tk_set_flg(flag_id, OBSTACLE_EVT_SCAN_DONE);

    /* Face forward again and start MONITOR afresh: what it remembered
     * from before the scan is out of date. */
    (void)obstacle_servo_set_angle(OBSTACLE_FORWARD_DEG, &settle_ms);
    task_settle(settle_ms, &report, p_seq);
    obstacle_monitor_reset(&monitor);
}

/*!
 * @brief PATH CHECK: range from the bypass side back to straight ahead.
 *
 * @param[in,out] p_seq Status-word sequence number.
 */
static void
task_path_check(uint32_t * p_seq)
{
    obstacle_range_t  pings[OBSTACLE_SCAN_PINGS];
    obstacle_status_t report = last_report;
    uint32_t          settle_ms = 0U;
    uint32_t          angle_deg;
    uint32_t          idx;
    uint32_t          ping;

    (void)tk_clr_flg(flag_id, ~OBSTACLE_EVT_PATH_DONE);
    path_counter++;                               /* Odd: being written. */

    path_result.need_mm  = path_need_mm;
    path_result.known_mm = path_known_mm;
    path_result.count    = 0U;
    task_publish_scanning(&report, p_seq);

    /* Ends facing straight ahead, ready for MONITOR. */
    for (idx = 0U; idx < OBSTACLE_PATH_MAX_POINTS; idx++)
    {
        angle_deg = obstacle_scan_path_angle(path_side, idx);
        (void)obstacle_servo_set_angle(angle_deg, &settle_ms);
        task_settle(settle_ms, &report, p_seq);

        for (ping = 0U; ping < OBSTACLE_SCAN_PINGS; ping++)
        {
            obstacle_ultrasonic_measure(&pings[ping]);
            task_publish_scanning(&report, p_seq);
        }

        obstacle_scan_path_add(&path_result, angle_deg, pings,
                               OBSTACLE_SCAN_PINGS);
    }

    obstacle_scan_path_verdict(&path_result);

    path_counter++;                               /* Even: complete.     */
    (void)tk_set_flg(flag_id, OBSTACLE_EVT_PATH_DONE);

    /* The car has turned since MONITOR last looked. */
    obstacle_monitor_reset(&monitor);
}

/*!
 * @brief Task body: range forward, scan on request.
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
        if (E_OK == tk_wai_flg(flag_id, TASK_REQ_SCAN,
                               TWF_ORW | TWF_BITCLR, &pattern, TMO_POL))
        {
            task_scan(&seq);
        }

        if (E_OK == tk_wai_flg(flag_id, TASK_REQ_PATH,
                               TWF_ORW | TWF_BITCLR, &pattern, TMO_POL))
        {
            task_path_check(&seq);
        }

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
 * @brief Ask for a scan.  OBSTACLE_EVT_SCAN_DONE is cleared when it starts
 *        and set when the result is ready (about 2 s).
 */
void
obstacle_task_request_scan(void)
{
    if (b_initialised)
    {
        (void)tk_set_flg(flag_id, TASK_REQ_SCAN);
    }
}

/*!
 * @brief Copy of the latest complete scan.
 *
 * @param[out] p_scan Copy; may be partly written when false is returned.
 *
 * @return false if no scan has completed or one is being written.
 */
bool
obstacle_task_get_scan(obstacle_scan_result_t * p_scan)
{
    bool     b_whole = false;
    uint32_t before = scan_counter;

    if ((NULL != p_scan) && (0U != before) && (0U == (before % 2U)))
    {
        *p_scan = scan_result;
        b_whole = (before == scan_counter);
    }

    return (b_whole);
}

/*!
 * @brief The recommendation and profile from the latest complete scan.
 *
 * @param[out] p_profile Profile; written only when true is returned.
 *
 * @return false if no complete scan is available.
 */
bool
obstacle_task_get_profile(obstacle_profile_t * p_profile)
{
    obstacle_scan_result_t copy;
    bool                   b_ok = false;

    if ((NULL != p_profile) && obstacle_task_get_scan(&copy))
    {
        *p_profile = copy.profile;
        b_ok = true;
    }

    return (b_ok);
}

/*!
 * @brief Ask for a path check before a bypass leg.
 *
 * @param[in] side     The plan's side (TURN_LEFT or TURN_RIGHT).
 * @param[in] need_mm  Free distance needed (the plan's check_mm).
 * @param[in] known_mm The scan's closest distance; 0 if none.
 */
void
obstacle_task_request_path_check(obstacle_action_t side,
                                 uint32_t          need_mm,
                                 uint32_t          known_mm)
{
    if (b_initialised)
    {
        path_side = side;
        path_need_mm = need_mm;
        path_known_mm = known_mm;
        (void)tk_set_flg(flag_id, TASK_REQ_PATH);
    }
}

/*!
 * @brief Result of the latest complete path check.
 *
 * @param[out] p_path Copy; written only when true is returned.
 *
 * @return false if no path check has completed or one is running.
 */
bool
obstacle_task_get_path(obstacle_path_t * p_path)
{
    obstacle_path_t copy;
    bool            b_whole = false;
    uint32_t        before = path_counter;

    if ((NULL != p_path) && (0U != before) && (0U == (before % 2U)))
    {
        copy = path_result;
        b_whole = (before == path_counter);

        if (b_whole)
        {
            *p_path = copy;
        }
    }

    return (b_whole);
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
