/** @file vc.c
 *
 * @brief Vehicle Controller state machine (pure logic, host-testable).
 *
 * One call to vc_step() handles one 20 ms cycle, in this order:
 *   1. safety   - a serious fault in any state -> EMERGENCY_STOP;
 *   2. command  - laptop STOP in any running state -> READY;
 *   3. the current state's own transitions (in FOLLOW_LINE: finish, then
 *      obstacle, then junction, then line loss);
 *   4. the state's timeout, if no transition happened in step 3, so an
 *      expected event arriving at the deadline still wins.
 */

#include <stddef.h>     /* NULL.  No kernel headers here, so no clash. */
#include <stdint.h>
#include <stdbool.h>
#include "vc.h"

/* TEMP-STRAIGHT-TEST: remove before push. */
/* Test-only: 1 = drive straight at normal speed and ignore the line
 * sensors.  Default 0, so a normal build is unchanged. */
#ifndef VC_STRAIGHT_TEST
#define VC_STRAIGHT_TEST    (0)
#endif
/* TEMP-STRAIGHT-TEST end */

/* ------------------------------------------------------------------------ *
 * Private data
 * ------------------------------------------------------------------------ */

static char const * const g_state_names[VC_STATE_COUNT] =
{
    "INITIALISE", "READY", "FOLLOW_LINE", "HANDLE_JUNCTION", "TURNING",
    "OBSTACLE_SCAN", "BYPASS", "REACQUIRE_LINE", "EMERGENCY_STOP",
    "FINISHED"
};

/* Time allowed in each state before EMERGENCY_STOP; 0 = no limit.
 * One table, so every timeout from the rules table is in one place. */
static uint32_t const g_state_timeout_ms[VC_STATE_COUNT] =
{
    VC_INIT_TIMEOUT_MS,         /* INITIALISE                              */
    0U,                         /* READY: waits for START indefinitely     */
    0U,                         /* FOLLOW_LINE                             */
    0U,                         /* HANDLE_JUNCTION: decided immediately    */
    VC_TURN_TIMEOUT_MS,         /* TURNING                                 */
    VC_SCAN_TIMEOUT_MS,         /* OBSTACLE_SCAN (incl. reverse + rescan)  */
    VC_BYPASS_TIMEOUT_MS,       /* BYPASS                                  */
    VC_REACQUIRE_TIMEOUT_MS,    /* REACQUIRE_LINE                          */
    0U,                         /* EMERGENCY_STOP: latched until ack       */
    0U                          /* FINISHED                                */
};

static void step_checked(vc_t * p_vc, vc_inputs_t const * p_in,
                         vc_outputs_t * p_out);

/* ------------------------------------------------------------------------ *
 * Private helpers
 * ------------------------------------------------------------------------ */

/* True in the states where the car is (or may be) moving on a mission. */
static bool
is_running (vc_state_t state)
{
    return ((VC_STATE_FOLLOW_LINE <= state)
            && (VC_STATE_REACQUIRE_LINE >= state));
}


/* True in the states that belong to the obstacle sequence. */
static bool
is_obstacle_state (vc_state_t state)
{
    return ((VC_STATE_OBSTACLE_SCAN == state)
            || (VC_STATE_BYPASS == state)
            || (VC_STATE_REACQUIRE_LINE == state));
}


/* Line position with out-of-range codes treated as LOST (fail safe). */
static vc_line_t
line_of (vc_inputs_t const * p_in)
{
    vc_line_t line = VC_LINE_LOST;

    if (VC_LINE_JUNCTION >= p_in->line)
    {
        line = p_in->line;
    }

    return line;
}


/* Milliseconds spent in the current state (wrap-safe). */
static uint32_t
time_in_state (vc_t const * p_vc, uint32_t now_ms)
{
    return (now_ms - p_vc->entry_ms);
}


/* Change state and apply the new state's entry actions. */
static void
enter_state (vc_t * p_vc, vc_state_t next, uint32_t now_ms,
             vc_outputs_t * p_out)
{
    /* Leaving the obstacle sequence: cancel Buddy 5's request, but only if
     * one was actually made (REACQUIRE_LINE is also used after a plain
     * line loss, when Buddy 5 was never asked for anything). */
    if (p_vc->b_obstacle_active && !is_obstacle_state(next))
    {
        p_out->obstacle         = VC_OBST_ABORT;
        p_vc->b_obstacle_active = false;
    }

    p_vc->state            = next;
    p_vc->entry_ms         = now_ms;
    p_vc->b_reversing      = false;
    p_out->b_state_changed = true;

    switch (next)
    {
        case VC_STATE_READY:
        case VC_STATE_EMERGENCY_STOP:
        case VC_STATE_FINISHED:
            p_out->motion    = VC_MOTION_STOP;  /* Brake first, always. */
            p_vc->stored_nav = VC_NAV_NONE;
            break;

        case VC_STATE_OBSTACLE_SCAN:
            p_out->motion   = VC_MOTION_STOP;   /* Stopped while scanning. */
            p_out->obstacle = VC_OBST_SCAN_REQUEST;
            p_vc->b_obstacle_active = true;
            p_vc->rescans   = 0U;
            p_vc->last_side = 0;    /* After a bypass, search straight. */
            break;

        default:
            /* Other states act in their own step handling. */
            break;
    }
}


/* Remember which side the line was last seen on (from the sign of a
 * non-zero line error while the line is visible). */
static void
remember_side (vc_t * p_vc, vc_inputs_t const * p_in)
{
    vc_line_t const line = line_of(p_in);

    if ((VC_STATE_FOLLOW_LINE == p_vc->state)
        && (VC_LINE_LOST != line) && (VC_LINE_JUNCTION != line))
    {
        if (0 > p_in->line_error)
        {
            p_vc->last_side = -1;
        }
        else if (0 < p_in->line_error)
        {
            p_vc->last_side = 1;
        }
        else
        {
            /* Centred: keep the last known side. */
        }
    }
}


/* Line-following speed: capped while on a hump. */
static int16_t
follow_speed (vc_inputs_t const * p_in)
{
    return p_in->b_hump ? VC_SPEED_HUMP_MM_S : VC_SPEED_NORMAL_MM_S;
}


/* Slow down in proportion to the steer: full speed when centred, down to
 * VC_SPEED_CURVE_MM_S at full steer (never above the given speed). */
static int16_t
curve_speed (int16_t speed_mm_s, int8_t steer)
{
    int32_t const magnitude = (0 > steer) ? -(int32_t) steer : (int32_t) steer;
    int32_t       slowest   = VC_SPEED_CURVE_MM_S;
    int32_t       speed;

    if (slowest > speed_mm_s)
    {
        slowest = speed_mm_s;
    }

    speed = speed_mm_s - (((speed_mm_s - slowest) * magnitude) / 127);
    return (int16_t) speed;
}


/* Remember a decoded barcode until the next junction. */
static void
store_barcode (vc_t * p_vc, vc_inputs_t const * p_in)
{
    if ((VC_NAV_NONE != p_in->barcode) && (VC_NAV_UTURN >= p_in->barcode))
    {
        p_vc->stored_nav = p_in->barcode;
    }
}


/* HANDLE_JUNCTION: use the stored barcode command, then clear it.
 * Runs in the same step the junction is reached, so no cycle is lost. */
static void
handle_junction (vc_t * p_vc, vc_inputs_t const * p_in, vc_outputs_t * p_out)
{
    vc_nav_t const nav = p_vc->stored_nav;

    enter_state(p_vc, VC_STATE_HANDLE_JUNCTION, p_in->now_ms, p_out);
    p_vc->stored_nav = VC_NAV_NONE;

    if ((VC_NAV_LEFT == nav) || (VC_NAV_RIGHT == nav)
        || (VC_NAV_UTURN == nav))
    {
        enter_state(p_vc, VC_STATE_TURNING, p_in->now_ms, p_out);
        p_out->motion = VC_MOTION_TURN;
        p_out->turn   = nav;
    }
    else
    {
        /* STRAIGHT, no stored command or invalid barcode: carry on. */
        enter_state(p_vc, VC_STATE_FOLLOW_LINE, p_in->now_ms, p_out);
        p_out->motion     = VC_MOTION_DRIVE;
        p_out->speed_mm_s = follow_speed(p_in);
        p_out->steer      = 0;                  /* Straight across. */
    }
}


/* FOLLOW_LINE: steer along the line; watch for obstacle, junction, loss. */
static void
step_follow_line (vc_t * p_vc, vc_inputs_t const * p_in, vc_outputs_t * p_out)
{
    vc_line_t const line = line_of(p_in);

    /* A barcode is only remembered here; it is acted on at the junction. */
    store_barcode(p_vc, p_in);

#if VC_STRAIGHT_TEST
    /* TEMP-STRAIGHT-TEST: remove before push. */
    /* Line sensors ignored: no junction, no line loss, steer 0.  Finish
     * or an obstacle stops the car cleanly. */
    if ((p_in->b_finish) || (p_in->b_obstacle_ahead))
    {
        enter_state(p_vc, VC_STATE_FINISHED, p_in->now_ms, p_out);
    }
    else
    {
        p_out->motion     = VC_MOTION_DRIVE;
        p_out->steer      = 0;
        p_out->speed_mm_s = curve_speed(follow_speed(p_in), 0);
    }
    return;
    /* TEMP-STRAIGHT-TEST end */
#endif

    if (p_in->b_finish)
    {
        enter_state(p_vc, VC_STATE_FINISHED, p_in->now_ms, p_out);
    }
    else if (p_in->b_obstacle_ahead)
    {
        enter_state(p_vc, VC_STATE_OBSTACLE_SCAN, p_in->now_ms, p_out);
    }
    else if ((VC_LINE_JUNCTION == line)
             && (VC_LINE_JUNCTION != p_vc->prev_line))
    {
        /* Edge-triggered: one decision per junction, however long the
         * sensors stay over it. */
        handle_junction(p_vc, p_in, p_out);
    }
    else if (VC_LINE_LOST == line)
    {
        enter_state(p_vc, VC_STATE_REACQUIRE_LINE, p_in->now_ms, p_out);
    }
    else
    {
        p_out->motion     = VC_MOTION_DRIVE;
        /* Over a junction already handled: drive straight across. */
        p_out->steer      = (VC_LINE_JUNCTION == line) ? 0
                                                       : p_in->line_error;
        p_out->speed_mm_s = curve_speed(follow_speed(p_in), p_out->steer);
    }
}


/* TURNING: wait for Buddy 2 to finish the encoder turn. */
static void
step_turning (vc_t * p_vc, vc_inputs_t const * p_in, vc_outputs_t * p_out)
{
    if (p_in->b_motion_failed)
    {
        enter_state(p_vc, VC_STATE_EMERGENCY_STOP, p_in->now_ms, p_out);
    }
    else if (p_in->b_motion_done)
    {
        enter_state(p_vc, VC_STATE_FOLLOW_LINE, p_in->now_ms, p_out);
    }
    else
    {
        /* Keep turning. */
    }
}


/* OBSTACLE_SCAN, while reversing before a rescan: wait for the reverse
 * to finish, and only then ask Buddy 5 for a new scan. */
static void
step_rescan_reverse (vc_t * p_vc, vc_inputs_t const * p_in,
                     vc_outputs_t * p_out)
{
    if (p_in->b_motion_failed)
    {
        enter_state(p_vc, VC_STATE_EMERGENCY_STOP, p_in->now_ms, p_out);
    }
    else if (p_in->b_motion_done)
    {
        p_vc->b_reversing = false;
        p_vc->entry_ms    = p_in->now_ms;      /* Fresh scan time budget. */
        p_out->obstacle   = VC_OBST_SCAN_REQUEST;
    }
    else
    {
        /* Still reversing; any scan result now would be stale. */
    }
}


/* OBSTACLE_SCAN: act on Buddy 5's recommendation. */
static void
step_obstacle_scan (vc_t * p_vc, vc_inputs_t const * p_in,
                    vc_outputs_t * p_out)
{
    if (p_vc->b_reversing)
    {
        step_rescan_reverse(p_vc, p_in, p_out);
    }
    else if (p_in->b_scan_done)
    {
        switch (p_in->recommendation)
        {
            case VC_AVOID_CONTINUE:
                enter_state(p_vc, VC_STATE_FOLLOW_LINE, p_in->now_ms, p_out);
                break;

            case VC_AVOID_TURN_LEFT:
            case VC_AVOID_TURN_RIGHT:
                enter_state(p_vc, VC_STATE_BYPASS, p_in->now_ms, p_out);
                p_out->motion      = VC_MOTION_BYPASS;
                p_out->bypass_side = p_in->recommendation;
                p_out->obstacle    = VC_OBST_BYPASS_START;
                break;

            case VC_AVOID_REVERSE_RETRY:
                if (VC_MAX_RESCANS > p_vc->rescans)
                {
                    p_vc->rescans++;
                    p_vc->b_reversing = true;
                    p_vc->entry_ms    = p_in->now_ms;  /* Time the reverse. */
                    p_out->motion     = VC_MOTION_REVERSE;
                }
                else
                {
                    enter_state(p_vc, VC_STATE_EMERGENCY_STOP,
                                p_in->now_ms, p_out);
                }
                break;

            case VC_AVOID_STOP:
            default:
                /* STOP, or an unknown recommendation: fail safe. */
                enter_state(p_vc, VC_STATE_EMERGENCY_STOP,
                            p_in->now_ms, p_out);
                break;
        }
    }
    else
    {
        /* Still scanning. */
    }
}


/* BYPASS: wait for Buddy 5 to report the obstacle is passed. */
static void
step_bypass (vc_t * p_vc, vc_inputs_t const * p_in, vc_outputs_t * p_out)
{
    if (p_in->b_motion_failed)
    {
        enter_state(p_vc, VC_STATE_EMERGENCY_STOP, p_in->now_ms, p_out);
    }
    else if (p_in->b_obstacle_passed)
    {
        enter_state(p_vc, VC_STATE_REACQUIRE_LINE, p_in->now_ms, p_out);
    }
    else
    {
        /* Keep going around. */
    }
}


/* REACQUIRE_LINE: creep forward until Buddy 3 sees the line again. */
static void
step_reacquire_line (vc_t * p_vc, vc_inputs_t const * p_in,
                     vc_outputs_t * p_out)
{
    vc_line_t const line = line_of(p_in);

    store_barcode(p_vc, p_in);

    if (p_in->b_obstacle_ahead)
    {
        /* Something in the way while creeping: stop and scan again. */
        enter_state(p_vc, VC_STATE_OBSTACLE_SCAN, p_in->now_ms, p_out);
    }
    else if (VC_LINE_JUNCTION == line)
    {
        /* Found the line on a junction: act on it now, or it would be
         * skipped and its barcode used at the next junction instead. */
        handle_junction(p_vc, p_in, p_out);
    }
    else if (VC_LINE_LOST != line)
    {
        enter_state(p_vc, VC_STATE_FOLLOW_LINE, p_in->now_ms, p_out);
    }
    else
    {
        p_out->motion     = VC_MOTION_DRIVE;
        /* NOTE: The terrain slowdown covers line following only. Search
         * keeps its own speed even when Buddy 4 asks to slow, because
         * Buddy 2 found 60 and 80 mm/s stalled the pivot. */
        p_out->speed_mm_s = VC_SPEED_SEARCH_MM_S;
        /* Curve towards where the line was last seen (straight if
         * unknown): the line sensors report a side only briefly before
         * "lost", so driving straight would miss it on a bend. */
        p_out->steer      = (int8_t) (p_vc->last_side * VC_SEARCH_STEER);
    }
}


/* Rule 3: the current state's own transitions. */
static void
step_state (vc_t * p_vc, vc_inputs_t const * p_in, vc_outputs_t * p_out)
{
    switch (p_vc->state)
    {
        case VC_STATE_INITIALISE:
            if (p_in->b_init_failed)
            {
                enter_state(p_vc, VC_STATE_EMERGENCY_STOP,
                            p_in->now_ms, p_out);
            }
            else if (p_in->b_init_ok)
            {
                enter_state(p_vc, VC_STATE_READY, p_in->now_ms, p_out);
            }
            else
            {
                /* Still initialising. */
            }
            break;

        case VC_STATE_READY:
            /* A run needs Buddy 4 ready, or a ramp could not be measured
             * or slowed for.  A refused START is simply dropped. */
            if ((VC_CMD_START == p_in->cmd) && p_in->b_terrain_ready)
            {
                enter_state(p_vc, VC_STATE_FOLLOW_LINE, p_in->now_ms, p_out);
            }
            break;

        case VC_STATE_FOLLOW_LINE:
            step_follow_line(p_vc, p_in, p_out);
            break;

        case VC_STATE_TURNING:
            step_turning(p_vc, p_in, p_out);
            break;

        case VC_STATE_OBSTACLE_SCAN:
            step_obstacle_scan(p_vc, p_in, p_out);
            break;

        case VC_STATE_BYPASS:
            step_bypass(p_vc, p_in, p_out);
            break;

        case VC_STATE_REACQUIRE_LINE:
            step_reacquire_line(p_vc, p_in, p_out);
            break;

        case VC_STATE_EMERGENCY_STOP:
            /* Latched: exit only when the fault is cleared, the car is
             * still, AND the operator acknowledges with STOP ("restart
             * permitted").  Timeouts raise no fault bit, so without the
             * acknowledgement the emergency would end after one step. */
            if ((0U == p_in->serious_faults) && p_in->b_stationary
                && (VC_CMD_STOP == p_in->cmd))
            {
                enter_state(p_vc, VC_STATE_READY, p_in->now_ms, p_out);
            }
            break;

        case VC_STATE_HANDLE_JUNCTION:  /* Transient: never seen here.    */
        case VC_STATE_FINISHED:
            /* Remain where we are. */
            break;

        default:
            /* Corrupted state value: fail safe. */
            enter_state(p_vc, VC_STATE_EMERGENCY_STOP, p_in->now_ms, p_out);
            break;
    }
}


/* ------------------------------------------------------------------------ *
 * Public API
 * ------------------------------------------------------------------------ */

void
vc_init (vc_t * p_vc, uint32_t now_ms)
{
    if (NULL != p_vc)
    {
        p_vc->state             = VC_STATE_INITIALISE;
        p_vc->entry_ms          = now_ms;
        p_vc->stored_nav        = VC_NAV_NONE;
        p_vc->rescans           = 0U;
        p_vc->b_obstacle_active = false;
        p_vc->b_reversing       = false;
        p_vc->prev_line         = VC_LINE_LOST;
        p_vc->last_side         = 0;
    }
}


void
vc_step (vc_t * p_vc, vc_inputs_t const * p_in, vc_outputs_t * p_out)
{
    if ((NULL != p_vc) && (NULL != p_in) && (NULL != p_out))
    {
        step_checked(p_vc, p_in, p_out);
    }
}


/* Body of vc_step(), with all pointers already checked. */
static void
step_checked (vc_t * p_vc, vc_inputs_t const * p_in, vc_outputs_t * p_out)
{
    uint32_t timeout_ms;

    /* Default: no change, no new actions. */
    p_out->b_state_changed = false;
    p_out->motion          = VC_MOTION_NONE;
    p_out->speed_mm_s      = 0;
    p_out->steer           = 0;
    p_out->turn            = VC_NAV_NONE;
    p_out->bypass_side     = VC_AVOID_STOP;
    p_out->obstacle        = VC_OBST_NONE;

    /* Rule 1: safety first.  Any serious fault stops the car. */
    if ((0U != p_in->serious_faults)
        && (VC_STATE_EMERGENCY_STOP != p_vc->state)
        && (VC_STATE_FINISHED != p_vc->state))
    {
        enter_state(p_vc, VC_STATE_EMERGENCY_STOP, p_in->now_ms, p_out);
    }
    /* Rule 2: laptop STOP returns a running car to READY. */
    else if ((VC_CMD_STOP == p_in->cmd) && is_running(p_vc->state))
    {
        enter_state(p_vc, VC_STATE_READY, p_in->now_ms, p_out);
    }
    else
    {
        /* Rule 3: the state's own transitions. */
        step_state(p_vc, p_in, p_out);

        /* Rule 4: the state's timeout, only if nothing else happened.
         * step_state() has already replaced any out-of-range state. */
        timeout_ms = g_state_timeout_ms[p_vc->state];

        if ((!p_out->b_state_changed) && (0U != timeout_ms)
            && (time_in_state(p_vc, p_in->now_ms) >= timeout_ms))
        {
            enter_state(p_vc, VC_STATE_EMERGENCY_STOP, p_in->now_ms, p_out);
        }
    }

    remember_side(p_vc, p_in);
    p_vc->prev_line = line_of(p_in);
    p_out->state    = p_vc->state;
}


char const *
vc_state_name (vc_state_t state)
{
    char const * p_name = "UNKNOWN";

    if (VC_STATE_COUNT > state)
    {
        p_name = g_state_names[state];
    }

    return p_name;
}

/*** end of file ***/
