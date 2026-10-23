/** @file vc.c
 *
 * @brief Vehicle Controller state machine (pure logic, host-testable).
 *
 * One call to vc_step() handles one 20 ms cycle.  Rules are checked in
 * the Week 6 arbitration order:
 *   1. safety     - a serious fault in any state -> EMERGENCY_STOP;
 *   2. command    - laptop STOP in any running state -> READY;
 *   3. the current state's own transitions (obstacle before junction
 *      before line loss in FOLLOW_LINE).
 */

#include <stddef.h>     /* NULL.  No kernel headers here, so no clash. */
#include <stdint.h>
#include <stdbool.h>
#include "vc.h"

/* ------------------------------------------------------------------------ *
 * Private helpers
 * ------------------------------------------------------------------------ */

static char const * const g_state_names[VC_STATE_COUNT] =
{
    "INITIALISE", "READY", "FOLLOW_LINE", "HANDLE_JUNCTION", "TURNING",
    "OBSTACLE_SCAN", "BYPASS", "REACQUIRE_LINE", "EMERGENCY_STOP",
    "FINISHED"
};

static void step_checked(vc_t * p_vc, vc_inputs_t const * p_in,
                         vc_outputs_t * p_out);


/* True in the states where the car is (or may be) moving on a mission. */
static bool
is_running (vc_state_t state)
{
    return ((VC_STATE_FOLLOW_LINE <= state)
            && (VC_STATE_REACQUIRE_LINE >= state));
}


/* True in the states that use Buddy 5 (an obstacle request may be active). */
static bool
is_obstacle_state (vc_state_t state)
{
    return ((VC_STATE_OBSTACLE_SCAN == state)
            || (VC_STATE_BYPASS == state)
            || (VC_STATE_REACQUIRE_LINE == state));
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

    p_vc->state      = next;
    p_vc->entry_ms   = now_ms;
    p_out->b_state_changed = true;

    switch (next)
    {
        case VC_STATE_READY:
        case VC_STATE_EMERGENCY_STOP:
        case VC_STATE_FINISHED:
            p_out->motion   = VC_MOTION_STOP;   /* Brake first, always. */
            p_vc->stored_nav = VC_NAV_NONE;
            break;

        case VC_STATE_OBSTACLE_SCAN:
            p_out->motion   = VC_MOTION_STOP;   /* Stopped while scanning. */
            p_out->obstacle = VC_OBST_SCAN_REQUEST;
            p_vc->b_obstacle_active = true;
            break;

        default:
            /* Other states act in their own step handling. */
            break;
    }
}


/* FOLLOW_LINE: steer along the line; watch for obstacle, junction, loss. */
static void
step_follow_line (vc_t * p_vc, vc_inputs_t const * p_in, vc_outputs_t * p_out)
{
    /* A barcode is only remembered here; it is acted on at the junction. */
    if (VC_NAV_NONE != p_in->barcode)
    {
        p_vc->stored_nav = p_in->barcode;
    }

    if (p_in->b_finish)
    {
        enter_state(p_vc, VC_STATE_FINISHED, p_in->now_ms, p_out);
    }
    else if (p_in->b_obstacle_ahead)
    {
        p_vc->rescans = 0U;
        enter_state(p_vc, VC_STATE_OBSTACLE_SCAN, p_in->now_ms, p_out);
    }
    else if (VC_LINE_JUNCTION == p_in->line)
    {
        enter_state(p_vc, VC_STATE_HANDLE_JUNCTION, p_in->now_ms, p_out);
    }
    else if (VC_LINE_LOST == p_in->line)
    {
        enter_state(p_vc, VC_STATE_REACQUIRE_LINE, p_in->now_ms, p_out);
    }
    else
    {
        p_out->motion     = VC_MOTION_DRIVE;
        p_out->speed_mm_s = p_in->b_hump ? VC_SPEED_HUMP_MM_S
                                         : VC_SPEED_NORMAL_MM_S;
        p_out->steer      = p_in->line_error;
    }
}


/* HANDLE_JUNCTION: use the stored barcode command, then clear it. */
static void
step_handle_junction (vc_t * p_vc, vc_inputs_t const * p_in,
                      vc_outputs_t * p_out)
{
    vc_nav_t const nav = p_vc->stored_nav;

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
    }
}


/* TURNING: wait for Buddy 2 to finish the encoder turn. */
static void
step_turning (vc_t * p_vc, vc_inputs_t const * p_in, vc_outputs_t * p_out)
{
    if (p_in->b_motion_failed
        || (time_in_state(p_vc, p_in->now_ms) >= VC_TURN_TIMEOUT_MS))
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


/* OBSTACLE_SCAN: act on Buddy 5's recommendation. */
static void
step_obstacle_scan (vc_t * p_vc, vc_inputs_t const * p_in,
                    vc_outputs_t * p_out)
{
    if (time_in_state(p_vc, p_in->now_ms) >= VC_SCAN_TIMEOUT_MS)
    {
        enter_state(p_vc, VC_STATE_EMERGENCY_STOP, p_in->now_ms, p_out);
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
                    p_vc->entry_ms  = p_in->now_ms;  /* Restart scan timer. */
                    p_out->motion   = VC_MOTION_REVERSE;
                    p_out->obstacle = VC_OBST_SCAN_REQUEST;
                }
                else
                {
                    enter_state(p_vc, VC_STATE_EMERGENCY_STOP,
                                p_in->now_ms, p_out);
                }
                break;

            case VC_AVOID_STOP:
            default:
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
    if (p_in->b_motion_failed
        || (time_in_state(p_vc, p_in->now_ms) >= VC_BYPASS_TIMEOUT_MS))
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
    if (VC_LINE_LOST != p_in->line)
    {
        enter_state(p_vc, VC_STATE_FOLLOW_LINE, p_in->now_ms, p_out);
    }
    else if (time_in_state(p_vc, p_in->now_ms) >= VC_REACQUIRE_TIMEOUT_MS)
    {
        enter_state(p_vc, VC_STATE_EMERGENCY_STOP, p_in->now_ms, p_out);
    }
    else
    {
        p_out->motion     = VC_MOTION_DRIVE;
        p_out->speed_mm_s = VC_SPEED_SEARCH_MM_S;
        p_out->steer      = 0;
    }
}


/* ------------------------------------------------------------------------ *
 * Public API
 * ------------------------------------------------------------------------ */

void
vc_init (vc_t * p_vc)
{
    if (NULL != p_vc)
    {
        p_vc->state      = VC_STATE_INITIALISE;
        p_vc->entry_ms   = 0U;
        p_vc->stored_nav = VC_NAV_NONE;
        p_vc->rescans    = 0U;
        p_vc->b_obstacle_active = false;
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
    /* Rule 3: the current state's own transitions. */
    else
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
                if (VC_CMD_START == p_in->cmd)
                {
                    enter_state(p_vc, VC_STATE_FOLLOW_LINE,
                                p_in->now_ms, p_out);
                }
                break;

            case VC_STATE_FOLLOW_LINE:
                step_follow_line(p_vc, p_in, p_out);
                break;

            case VC_STATE_HANDLE_JUNCTION:
                step_handle_junction(p_vc, p_in, p_out);
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

            case VC_STATE_FINISHED:
            default:
                /* Remain stopped. */
                break;
        }
    }

    p_out->state = p_vc->state;
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
