/** @file vc_task.c
 *
 * @brief Vehicle Controller RTOS task (see vc_task.h).
 *
 * Every 20 ms: gather inputs -> vc_step() -> act on the outputs.
 *
 * Subsystems not integrated yet are placeholders:
 *   - Buddy 3 (line/barcode): until VC_HAVE_BUDDY3 = 1, the line is always
 *     "centred", and a run ends itself after VC_TEST_RUN_MS (as if the
 *     laptop sent STOP) so a bench test can never run away;
 *   - Buddy 4 (hump): never on a hump;
 *   - Buddy 5 (obstacle): never an obstacle; a bypass request just stops.
 *
 * Two Motion (Buddy 2) behaviours shape this file:
 *   - every SET_VELOCITY command resets Motion's speed controller, so a
 *     drive command is only re-sent when speed or steering really changes;
 *   - motion_stop() is processed on Motion's next loop and discards any
 *     command queued before then, so after a stop nothing is sent for
 *     VC_STOP_HOLD_STEPS steps.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <stdint.h>
#include <stdbool.h>

#include "vc.h"
#include "vc_task.h"
#include "motion_types.h"
#include "motion_task.h"
#include "comms.h"

#ifndef VC_HAVE_BUDDY3
#define VC_HAVE_BUDDY3           (0)    /* 1 once Buddy 3 is in main.     */
#endif

#if VC_HAVE_BUDDY3
#include "line_follow.h"
#include "navigation.h"
#include "buddy3_task.h"
#endif

/* ------------------------------------------------------------------------ *
 * Tunables.  All TBC: tune on the track.
 * ------------------------------------------------------------------------ */

#define VC_TASK_PRIORITY         (5)       /* Just below Motion (4).       */
#define VC_TASK_STACK_BYTES      (4096U)   /* SMP kernel path is deep.     */

#define VC_STEER_GAIN_MDEG_S     (400)     /* Turn rate per steer unit;
                                              127 -> about 50 deg/s.       */
#define VC_STEER_RESEND          (8)       /* Re-send drive if steer moves
                                              this much (of 127).          */
#define VC_TURN_SPEED_MM_S       (100)     /* Wheel speed for encoder turns.*/
#define VC_REVERSE_MM            (100)     /* Back off before a rescan.    */
#define VC_REVERSE_SPEED_MM_S    (80)
#define VC_STATIONARY_MM_S       (10)      /* Both wheels below = still.   */
#define VC_STOP_HOLD_STEPS       (2U)      /* 40 ms: Motion sees the stop. */
#define VC_SUBMIT_RETRIES        (5U)      /* BUSY / QUEUE_FULL retries.   */
#define VC_TEST_RUN_MS           (5000U)   /* Placeholder-line run length. */
#define VC_LINE_STALE_MS         (100U)    /* No Buddy 3 data = line lost. */
#define VC_LINE_DRAIN_MAX        (16U)     /* Messages read per step, max. */

/* ------------------------------------------------------------------------ *
 * Private data (used only by the VC task)
 * ------------------------------------------------------------------------ */

static vc_t     g_vc;
static bool     g_b_init_ok;
static UW       g_stack[VC_TASK_STACK_BYTES / sizeof(UW)];

static uint32_t g_seq;                 /* Last Motion sequence id used.   */
static uint32_t g_manoeuvre_seq;       /* Turn/reverse in progress; 0 none.*/
static bool     g_b_manoeuvre_failed;  /* Motion refused the manoeuvre.   */
static bool     g_b_rejected;          /* Motion refused a drive command. */

static bool              g_b_pending;  /* Manoeuvre waiting to be sent.   */
static motion_command_t  g_pending;
static uint32_t          g_pending_tries;
static uint32_t          g_hold_steps; /* Steps left after a stop.        */

static bool     g_b_driving;           /* A velocity command is active.   */
static int16_t  g_drive_speed;
static int8_t   g_drive_steer;

static bool     g_b_run_active;
static uint32_t g_run_start_ms;
static uint32_t g_overruns;            /* Steps that started late.        */

/* VC state -> the state codes comms publishes today (tracker item C5). */
static comms_state_t const g_comms_state[VC_STATE_COUNT] =
{
    COMMS_STATE_INIT,           /* INITIALISE      */
    COMMS_STATE_READY,          /* READY           */
    COMMS_STATE_LINE_FOLLOW,    /* FOLLOW_LINE     */
    COMMS_STATE_EXECUTE_NAV,    /* HANDLE_JUNCTION */
    COMMS_STATE_EXECUTE_NAV,    /* TURNING         */
    COMMS_STATE_OBSTACLE_SCAN,  /* OBSTACLE_SCAN   */
    COMMS_STATE_BYPASS,         /* BYPASS          */
    COMMS_STATE_LINE_RECOVERY,  /* REACQUIRE_LINE  */
    COMMS_STATE_SAFE_STOP,      /* EMERGENCY_STOP  */
    COMMS_STATE_FINISHED        /* FINISHED        */
};

/* ------------------------------------------------------------------------ *
 * Helpers
 * ------------------------------------------------------------------------ */

static uint32_t
now_ms (void)
{
    SYSTIM tim;

    (void) tk_get_otm(&tim);
    return (uint32_t) tim.lo;       /* Wraps after 49 days; VC is wrap-safe. */
}


static int32_t
abs_i32 (int32_t value)
{
    return (0 > value) ? -value : value;
}


/* Next Motion sequence id; 0 is reserved by Motion as invalid. */
static uint32_t
next_seq (void)
{
    g_seq++;

    if (0U == g_seq)
    {
        g_seq = 1U;
    }

    return g_seq;
}

/* ------------------------------------------------------------------------ *
 * Inputs
 * ------------------------------------------------------------------------ */

/* Motion status -> done / failed / stationary, faults, and telemetry. */
static void
read_motion (vc_inputs_t * p_in)
{
    motion_status_t status;
    comms_motion_t  report;

    if (MOTION_RESULT_OK == motion_get_status(&status))
    {
        p_in->b_stationary =
            (VC_STATIONARY_MM_S > abs_i32(status.left_speed_mm_s))
            && (VC_STATIONARY_MM_S > abs_i32(status.right_speed_mm_s));

        if (MOTION_STATE_FAULT == status.state)
        {
            p_in->serious_faults |= (uint16_t) VC_FAULT_MOTION;
            p_in->b_motion_failed = (0U != g_manoeuvre_seq);
        }
        else if ((0U != g_manoeuvre_seq)
                 && (g_manoeuvre_seq == status.active_sequence_id)
                 && (MOTION_STATE_COMPLETE == status.state))
        {
            p_in->b_motion_done = true;
            g_manoeuvre_seq     = 0U;
        }
        else
        {
            /* Nothing new. */
        }

        report.speed_left_mm_s  = status.left_speed_mm_s;
        report.speed_right_mm_s = status.right_speed_mm_s;
        report.encoder_left     = status.left_encoder_counts;
        report.encoder_right    = status.right_encoder_counts;
        report.distance_mm      = status.travelled_mm;
        comms_report_motion(&report);
    }
    else
    {
        /* Motion cannot be read: treat as faulty, never as still. */
        p_in->serious_faults |= (uint16_t) VC_FAULT_MOTION;
        p_in->b_stationary    = false;
    }

    if (g_b_manoeuvre_failed)
    {
        p_in->b_motion_failed = true;
        g_b_manoeuvre_failed  = false;
    }

    if (g_b_rejected)
    {
        p_in->serious_faults |= (uint16_t) VC_FAULT_MOTION_REJECTED;
    }
}


/* One laptop command per step; the rest wait in comms' queue. */
static void
read_command (vc_inputs_t * p_in)
{
    comms_cmd_t cmd;

    if (comms_get_command(&cmd))
    {
        if (COMMS_CMD_START == cmd)
        {
            p_in->cmd = VC_CMD_START;
        }
        else if (COMMS_CMD_STOP == cmd)
        {
            p_in->cmd = VC_CMD_STOP;
        }
        else
        {
            /* Unknown: ignore. */
        }
    }
}


#if VC_HAVE_BUDDY3

/* Buddy 3 line error (-1.0 .. +1.0) -> VC steer (-127 .. +127). */
static int8_t
error_to_steer (float error)
{
    float e = error;

    if (e > 1.0f)
    {
        e = 1.0f;
    }
    else if (e < -1.0f)
    {
        e = -1.0f;
    }
    else if (!(e == e))             /* NaN: treat as centred. */
    {
        e = 0.0f;
    }
    else
    {
        /* In range. */
    }

    return (int8_t) (e * 127.0f);   /* Range checked above. */
}


static vc_line_t
line_from_buddy3 (line_state_t state)
{
    vc_line_t line = VC_LINE_LOST;

    switch (state)
    {
        case LINE_LEFT:     line = VC_LINE_LEFT;     break;
        case LINE_CENTRE:   line = VC_LINE_CENTRE;   break;
        case LINE_RIGHT:    line = VC_LINE_RIGHT;    break;
        case LINE_JUNCTION: line = VC_LINE_JUNCTION; break;
        case LINE_LOST:
        default:            line = VC_LINE_LOST;     break;
    }

    return line;
}


static vc_nav_t
nav_from_buddy3 (nav_command_t nav)
{
    vc_nav_t out = VC_NAV_NONE;

    switch (nav)
    {
        case NAV_LEFT:     out = VC_NAV_LEFT;     break;
        case NAV_RIGHT:    out = VC_NAV_RIGHT;    break;
        case NAV_STRAIGHT: out = VC_NAV_STRAIGHT; break;
        case NAV_UTURN:    out = VC_NAV_UTURN;    break;
        case NAV_NONE:
        default:           out = VC_NAV_NONE;     break;
    }

    return out;
}


/* Drain Buddy 3's queue: latest line wins, first barcode is kept. */
static void
read_line (vc_inputs_t * p_in, uint32_t now)
{
    static vc_line_t s_line    = VC_LINE_LOST;
    static int8_t    s_error   = 0;
    static uint32_t  s_seen_ms = 0U;
    static bool      s_b_seen  = false;
    buddy3_output_msg_t msg;
    vc_nav_t            nav;
    uint32_t            i;

    for (i = 0U; i < VC_LINE_DRAIN_MAX; i++)
    {
        if ((INT) sizeof(msg)
            != tk_rcv_mbf(buddy3_output_mbfid, &msg, TMO_POL))
        {
            break;
        }

        s_line    = line_from_buddy3(msg.line_state);
        s_error   = error_to_steer(msg.line_error);
        s_seen_ms = now;
        s_b_seen  = true;
        nav       = nav_from_buddy3(msg.nav_command);

        if ((VC_NAV_NONE == p_in->barcode) && (VC_NAV_NONE != nav))
        {
            p_in->barcode = nav;
        }
    }

    if (s_b_seen && ((now - s_seen_ms) < VC_LINE_STALE_MS))
    {
        p_in->line       = s_line;
        p_in->line_error = s_error;
    }
    else
    {
        /* Buddy 3 silent: never steer on old data. */
        p_in->line       = VC_LINE_LOST;
        p_in->line_error = 0;
    }
}

#else  /* Placeholder until Buddy 3 is merged. */

static void
read_line (vc_inputs_t * p_in, uint32_t now)
{
    p_in->line       = VC_LINE_CENTRE;
    p_in->line_error = 0;

    /* Bench safety: end the run by itself, exactly like a laptop STOP. */
    if (g_b_run_active && ((now - g_run_start_ms) >= VC_TEST_RUN_MS)
        && (VC_CMD_NONE == p_in->cmd))
    {
        p_in->cmd = VC_CMD_STOP;
        tm_printf((UB *) "[vc] test run time up: auto STOP\n");
    }
}

#endif /* VC_HAVE_BUDDY3 */

/* ------------------------------------------------------------------------ *
 * Outputs
 * ------------------------------------------------------------------------ */

static void
stop_motion (void)
{
    (void) motion_stop();
    g_b_driving     = false;
    g_b_pending     = false;
    g_manoeuvre_seq = 0U;
    g_b_rejected    = false;
    g_hold_steps    = VC_STOP_HOLD_STEPS;
}


/* Stop now; send the manoeuvre once Motion has seen the stop. */
static void
queue_manoeuvre (motion_command_t const * p_cmd)
{
    stop_motion();
    g_pending       = *p_cmd;
    g_pending_tries = 0U;
    g_b_pending     = true;
}


static void
queue_turn (vc_nav_t turn)
{
    motion_command_t cmd = {0};

    cmd.type                       = MOTION_COMMAND_TURN_ANGLE;
    cmd.timeout_ms                 = VC_TURN_TIMEOUT_MS;
    cmd.value.turn.wheel_speed_mm_s = VC_TURN_SPEED_MM_S;

    /* Motion: positive angle = right turn. */
    if (VC_NAV_LEFT == turn)
    {
        cmd.value.turn.angle_mdeg = -90000;
    }
    else if (VC_NAV_RIGHT == turn)
    {
        cmd.value.turn.angle_mdeg = 90000;
    }
    else
    {
        cmd.value.turn.angle_mdeg = 180000;    /* U-turn. */
    }

    queue_manoeuvre(&cmd);
}


static void
queue_reverse (void)
{
    motion_command_t cmd = {0};

    cmd.type                      = MOTION_COMMAND_MOVE_DISTANCE;
    cmd.timeout_ms                = VC_SCAN_TIMEOUT_MS;
    cmd.value.distance.distance_mm = -VC_REVERSE_MM;
    cmd.value.distance.speed_mm_s  = -VC_REVERSE_SPEED_MM_S;
    queue_manoeuvre(&cmd);
}


/* Send a velocity command only when it really changed (see file note). */
static void
drive (int16_t speed_mm_s, int8_t steer)
{
    motion_command_t cmd = {0};
    motion_result_t  result;
    bool             b_changed;

    g_b_pending     = false;            /* Driving replaces any manoeuvre. */
    g_manoeuvre_seq = 0U;

    b_changed = (!g_b_driving) || (speed_mm_s != g_drive_speed)
                || (VC_STEER_RESEND
                    <= abs_i32((int32_t) steer - (int32_t) g_drive_steer));

    if (b_changed && (0U == g_hold_steps))
    {
        cmd.type                          = MOTION_COMMAND_SET_VELOCITY;
        cmd.sequence_id                   = next_seq();
        cmd.timeout_ms                    = 0U;
        cmd.value.velocity.linear_mm_s    = speed_mm_s;
        /* Line to the right (+) needs a right turn: negative rate. */
        cmd.value.velocity.angular_mdeg_s = -(int32_t) steer
                                            * VC_STEER_GAIN_MDEG_S;
        result = motion_submit(&cmd);

        if (MOTION_RESULT_OK == result)
        {
            g_b_driving   = true;
            g_drive_speed = speed_mm_s;
            g_drive_steer = steer;
        }
        else if ((MOTION_RESULT_BUSY == result)
                 || (MOTION_RESULT_QUEUE_FULL == result))
        {
            /* Try again next step. */
        }
        else
        {
            g_b_rejected = true;
            tm_printf((UB *) "[vc] drive rejected by Motion: %d\n",
                      (INT) result);
        }
    }
}


/* Once per step: count down the stop hold, then send a queued manoeuvre. */
static void
service_pending (void)
{
    motion_result_t result;

    if (0U < g_hold_steps)
    {
        g_hold_steps--;
    }
    else if (g_b_pending)
    {
        g_pending.sequence_id = next_seq();
        result = motion_submit(&g_pending);

        if (MOTION_RESULT_OK == result)
        {
            g_manoeuvre_seq = g_pending.sequence_id;
            g_b_pending     = false;
        }
        else if (((MOTION_RESULT_BUSY == result)
                  || (MOTION_RESULT_QUEUE_FULL == result))
                 && (VC_SUBMIT_RETRIES > g_pending_tries))
        {
            g_pending_tries++;
        }
        else
        {
            g_b_pending          = false;
            g_b_manoeuvre_failed = true;     /* VC -> EMERGENCY_STOP. */
            tm_printf((UB *) "[vc] manoeuvre rejected by Motion: %d\n",
                      (INT) result);
        }
    }
    else
    {
        /* Nothing waiting. */
    }
}


static void
act_on_motion (vc_outputs_t const * p_out)
{
    switch (p_out->motion)
    {
        case VC_MOTION_DRIVE:
            drive(p_out->speed_mm_s, p_out->steer);
            break;

        case VC_MOTION_TURN:
            queue_turn(p_out->turn);
            break;

        case VC_MOTION_REVERSE:
            queue_reverse();
            break;

        case VC_MOTION_BYPASS:
            /* Buddy 5 bypass not integrated: stop; the VC's bypass
             * timeout then ends in EMERGENCY_STOP (fail safe). */
            stop_motion();
            tm_printf((UB *) "[vc] bypass not available yet: stopped\n");
            break;

        case VC_MOTION_STOP:
            stop_motion();
            break;

        case VC_MOTION_NONE:
        default:
            break;
    }

    service_pending();
}


/* State changes -> comms state, run begin/end, console log. */
static void
report_state (vc_state_t from, vc_outputs_t const * p_out, uint32_t now)
{
    vc_state_t const to = p_out->state;

    if (p_out->b_state_changed)
    {
        tm_printf((UB *) "[vc] %s -> %s\n", (UB *) vc_state_name(from),
                  (UB *) vc_state_name(to));

        if (VC_STATE_COUNT > to)
        {
            comms_set_state(g_comms_state[to]);
        }

        if ((VC_STATE_FOLLOW_LINE == to) && (!g_b_run_active))
        {
            g_b_run_active = true;
            g_run_start_ms = now;
            (void) comms_begin_run();
        }
        else if (g_b_run_active
                 && ((VC_STATE_READY == to)
                     || (VC_STATE_EMERGENCY_STOP == to)
                     || (VC_STATE_FINISHED == to)))
        {
            g_b_run_active = false;
            (void) comms_end_run();
        }
        else
        {
            /* Still mid-run, or not running. */
        }
    }
}

/* ------------------------------------------------------------------------ *
 * The task
 * ------------------------------------------------------------------------ */

static void
vc_task_entry (INT stacd, void * p_exinf)
{
    vc_inputs_t  in;
    vc_outputs_t out;
    vc_state_t   before;
    uint32_t     now;
    uint32_t     next;
    int32_t      wait;

    (void) stacd;
    (void) p_exinf;

    next = now_ms();
    vc_init(&g_vc, next);
    comms_set_state(COMMS_STATE_INIT);

    for (;;)
    {
        now = now_ms();

        /* Inputs: start from "nothing happened", then fill in. */
        in = (vc_inputs_t) {0};
        in.now_ms        = now;
        in.b_init_ok     = g_b_init_ok;
        in.b_init_failed = !g_b_init_ok;
        in.recommendation = VC_AVOID_STOP;
        read_command(&in);
        read_motion(&in);
        read_line(&in, now);
        /* Buddy 4 / Buddy 5 placeholders: b_hump, b_obstacle_* = false. */

        before = g_vc.state;
        vc_step(&g_vc, &in, &out);

        act_on_motion(&out);
        report_state(before, &out, now);
        comms_set_fault((uint32_t) in.serious_faults);

        /* Fixed 20 ms rate from absolute time, so work time doesn't drift
         * the period; a late step is counted and the schedule resyncs. */
        next += VC_PERIOD_MS;
        wait  = (int32_t) (next - now_ms());

        if (0 < wait)
        {
            (void) tk_dly_tsk((RELTIM) wait);
        }
        else
        {
            g_overruns++;
            next = now_ms();
        }
    }
}


bool
vc_task_start (bool b_init_ok)
{
    T_CTSK ctsk = {0};
    ID     tid;

    g_b_init_ok = b_init_ok;

    ctsk.task    = vc_task_entry;
    ctsk.itskpri = VC_TASK_PRIORITY;
    ctsk.stksz   = (SZ) sizeof(g_stack);
    ctsk.bufptr  = g_stack;
#if TK_SUPPORT_SMP
    /* Core 1 (processor 2): Motion and the radio run on processor 1. */
    ctsk.tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF | TA_ASSPRC;
    ctsk.assprc  = TP_PRC2;
#else
    ctsk.tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF;
#endif

    tid = tk_cre_tsk(&ctsk);

    return (E_OK < tid) && (E_OK == tk_sta_tsk(tid, 0));
}

/*** end of file ***/
