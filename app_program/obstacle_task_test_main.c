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
 *   s   request a scan (prints every angle, the profile, the
 *       recommendation and the bypass plan when it finishes)
 *   c   path check (after pivoting the car by hand as the plan says):
 *       is the leg free for the plan's check_mm?
 *   r/l path check as for a right / left bypass, whatever the plan says
 *   t   the car turns itself on the spot by the plan's turn (Buddy 2's
 *       motion task, encoder-counted), then runs the path check
 *   u   turn back by the same angle (to repeat a test)
 *   b   the bypass by the plan, as vc_task.c runs it: turn, path check,
 *       leg, turn back 30 deg past straight, drive to where the line
 *       should be crossed (vc_task.c stops on the line instead)
 *   x   stop the motors at once (also ends a bypass)
 *
 * The motors move only on t, u or b.  First runs: wheels off the floor.
 *   p   pause or resume the status rows (events are always printed)
 *   h   help
 *
 * Output:
 *   mon,t_ms,seq,distance_mm,status,state,ahead,fault   about 5 per second
 *   [ctrl] ...                                           on every event
 *   scan,angle_deg,median_mm,valid,silent,class          after a scan
 *   profile,... / plan,...                               after a scan
 *   path,angle_deg,median_mm,result / path,verdict,...   after a check
 *   move,done,... / move,failed,...                      after a turn/leg
 */

#include <stdint.h>
#include <stdbool.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "obstacle_config.h"
#include "obstacle_types.h"
#include "obstacle_plan.h"
#include "obstacle_task.h"
#include "obstacle_console.h"
#include "motion_types.h"
#include "motion_task.h"

#define TEST_POLL_MS        (200)   /* TMO: one status row per poll.     */
#define TEST_KEY_ACK        ('a')
#define TEST_KEY_SCAN       ('s')
#define TEST_KEY_PATH       ('c')
#define TEST_KEY_PATH_RIGHT ('r')
#define TEST_KEY_PATH_LEFT  ('l')
#define TEST_DEFAULT_NEED_MM (1000U)  /* Path check without a plan.    */
#define TEST_KEY_PAUSE      ('p')
#define TEST_KEY_HELP       ('h')
#define TEST_KEY_TURN       ('t')
#define TEST_KEY_UNDO       ('u')
#define TEST_KEY_STOP       ('x')
#define TEST_KEY_BYPASS     ('b')

/* Turn on the spot, as vc_task.c does (VC_TURN_SPEED_MM_S, VC_TURN_
 * TIMEOUT_MS).  Motion: positive angle = right turn. */
#define TEST_TURN_SPEED_MM_S (80)
#define TEST_TURN_TIMEOUT_MS (3000U)
#define TEST_MDEG_PER_DEG    (1000)

/* Bypass legs and the return, as vc_task.c does (VC_BYPASS_SPEED_MM_S,
 * VC_BYPASS_GUARD_MM, VC_BYPASS_RETURN_DEG).  With no line sensing here,
 * the way back stops where it should cross the line at RETURN_DEG:
 * apex_mm / sin(30 deg) = 2 x apex_mm. */
#define TEST_LEG_SPEED_MM_S  (150)
#define TEST_LEG_GUARD_MM    (150U)
#define TEST_RETURN_DEG      (30U)
#define TEST_RETURN_LEG_X    (2U)

/* Where the b sequence is. */
typedef enum
{
    TEST_BYPASS_IDLE = 0,
    TEST_BYPASS_TURN_OUT,        /* Turn towards the side, then check.     */
    TEST_BYPASS_CHECK,           /* Waiting for the path check.            */
    TEST_BYPASS_LEG_OUT,
    TEST_BYPASS_TURN_APEX,       /* Turn back turn_deg + TEST_RETURN_DEG.  */
    TEST_BYPASS_LEG_BACK         /* Towards the line; stops on it.         */
} test_bypass_t;

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
    "[ctrl] keys: a=acknowledge OBSTACLE_AHEAD  s=scan  c=path check"
    "  r/l=path check right/left  t=turn by the plan, then check"
    "  u=turn back  b=whole bypass  x=stop motors  p=pause/resume rows"
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
static UB const msg_scan_req[] = "[ctrl] scan requested\n";
static UB const msg_scan_fail[] = "[ctrl] SCAN_DONE but no result\n";
static UB const msg_scan_header[] =
    "scan,angle_deg,median_mm,valid,silent,class\n";
static UB const msg_scan_row[] = "scan,%u,%u,%u,%u,%s\n";
static UB const msg_profile[] =
    "profile,closest_mm=%u,at_deg=%u,span_deg=%u..%u,width_mm=%u,"
    "clear_right_mm=%u,clear_left_mm=%u,time_ms=%u\n";
static UB const msg_action[] = "[ctrl] SCAN_DONE: recommendation %s\n";
static UB const msg_plan[] =
    "plan,side=%s,turn_deg=%u,leg_mm=%u,apex_mm=%u,check_mm=%u\n";
static UB const msg_plan_hint[] =
    "[ctrl] now pivot the car %u deg to the %s by hand, then press c\n";
static UB const msg_no_plan[] = "[ctrl] no bypass plan for this action\n";
static UB const msg_path_req[] =
    "[ctrl] path check %s, need %u mm, known obstacle %u mm\n";
static UB const msg_path_header[] = "path,angle_deg,median_mm,result\n";
static UB const msg_path_row[] = "path,%u,%u,%s\n";
static UB const msg_path[] = "path,verdict=%s,need_mm=%u,known_mm=%u\n";
static UB const msg_path_fail[] = "[ctrl] PATH_DONE but no result\n";
static UB const msg_motion_failed[] =
    "[ctrl] motion task FAILED (result %d): t and u will not work\n";
static UB const msg_turn_req[] =
    "[ctrl] turning %s %u deg (motion seq %u)\n";
static UB const msg_turn_refused[] = "[ctrl] turn refused (result %d)\n";
static UB const msg_turn_no_plan[] = "[ctrl] no plan yet: press s first\n";
static UB const msg_turn_done[] =
    "move,done,target_mdeg=%d,wheel_mm=%d,left_counts=%d,right_counts=%d\n";
static UB const msg_turn_failed[] = "move,failed,state=%d,fault=%d\n";
static UB const msg_leg_req[] = "[ctrl] driving %u mm (motion seq %u)\n";
static UB const msg_bypass_start[] =
    "[ctrl] bypass %s: turn %u deg, legs %u mm, check %u mm\n";
static UB const msg_bypass_done[] =
    "[ctrl] bypass done: the car should be on the line now\n";
static UB const msg_bypass_stop[] = "[ctrl] bypass stopped: %s\n";
static UB const msg_stopped[] = "[ctrl] motors stopped\n";
static UB const msg_paused[] = "[ctrl] rows paused (p to resume)\n";
static UB const msg_resumed[] = "[ctrl] rows resumed\n";

static bool              b_rows_on = true;
static uint32_t          check_need_mm = TEST_DEFAULT_NEED_MM;
static uint32_t          check_known_mm = 0U;    /* From the last scan. */
static obstacle_action_t check_side = OBSTACLE_ACTION_TURN_RIGHT;
static uint32_t          plan_turn_deg = 0U;     /* 0 = no plan yet.    */
static obstacle_bypass_plan_t last_plan;         /* Valid if turn != 0. */
static test_bypass_t     bypass_step = TEST_BYPASS_IDLE;

/* The turn in progress, if any. */
static bool              b_turning = false;
static bool              b_check_after_turn = false;
static uint32_t          turn_seq = 0U;
static obstacle_action_t turn_side = OBSTACLE_ACTION_TURN_RIGHT;
static int32_t           turn_start_left = 0;    /* Encoder counts.     */
static int32_t           turn_start_right = 0;


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
 * @brief Short text name for a scan point class.
 *
 * @param[in] point_class Class to name.
 *
 * @return Constant string.
 */
static char const *
test_class_name(obstacle_point_class_t point_class)
{
    char const * p_name = "?";

    switch (point_class)
    {
        case OBSTACLE_POINT_UNKNOWN:
            p_name = "UNKNOWN";
            break;

        case OBSTACLE_POINT_OBSTACLE:
            p_name = "OBSTACLE";
            break;

        case OBSTACLE_POINT_BACKGROUND:
            p_name = "BACKGROUND";
            break;

        default:
            /* Unknown class: keep "?". */
            break;
    }

    return (p_name);
}

/*!
 * @brief Short text name for a recommended action.
 *
 * @param[in] action Action to name.
 *
 * @return Constant string.
 */
static char const *
test_action_name(obstacle_action_t action)
{
    char const * p_name = "?";

    switch (action)
    {
        case OBSTACLE_ACTION_STOP:
            p_name = "STOP";
            break;

        case OBSTACLE_ACTION_CONTINUE:
            p_name = "CONTINUE";
            break;

        case OBSTACLE_ACTION_TURN_LEFT:
            p_name = "TURN_LEFT";
            break;

        case OBSTACLE_ACTION_TURN_RIGHT:
            p_name = "TURN_RIGHT";
            break;

        case OBSTACLE_ACTION_REVERSE_RETRY:
            p_name = "REVERSE_RETRY";
            break;

        default:
            /* Unknown action: keep "?". */
            break;
    }

    return (p_name);
}

/*!
 * @brief Short text name for a path-check verdict.
 *
 * @param[in] verdict Verdict to name.
 *
 * @return Constant string.
 */
static char const *
test_verdict_name(obstacle_path_verdict_t verdict)
{
    char const * p_name = "?";

    switch (verdict)
    {
        case OBSTACLE_PATH_CLEAR:
            p_name = "CLEAR";
            break;

        case OBSTACLE_PATH_OPEN:
            p_name = "OPEN";
            break;

        case OBSTACLE_PATH_BLOCKED:
            p_name = "BLOCKED";
            break;

        case OBSTACLE_PATH_FAULT:
            p_name = "FAULT";
            break;

        case OBSTACLE_PATH_KNOWN:
            p_name = "KNOWN";
            break;

        default:
            /* Unknown verdict: keep "?". */
            break;
    }

    return (p_name);
}

/*!
 * @brief Print the bypass plan for a scan's recommendation, if it turns.
 *
 * @param[in] p_profile Profile from the scan.
 */
static void
test_print_plan(obstacle_profile_t const * p_profile)
{
    obstacle_bypass_plan_t plan;
    char const *           p_side;

    check_known_mm = p_profile->distance_mm;

    if (obstacle_plan_bypass(p_profile->distance_mm, p_profile->action,
                             &plan))
    {
        p_side = (OBSTACLE_ACTION_TURN_LEFT == plan.side) ? "LEFT" : "RIGHT";
        (void)tm_printf(msg_plan, p_side, plan.turn_deg, plan.leg_mm,
                        plan.apex_mm, plan.check_mm);
        (void)tm_printf(msg_plan_hint, plan.turn_deg, p_side);
        check_need_mm = plan.check_mm;
        check_side = plan.side;
        plan_turn_deg = plan.turn_deg;
        last_plan = plan;
    }
    else
    {
        /* Forget the old plan, so t cannot turn by a stale one. */
        plan_turn_deg = 0U;
        check_need_mm = TEST_DEFAULT_NEED_MM;
        (void)tm_printf(msg_no_plan);
    }
}

/*!
 * @brief Print the latest path check.
 */
static void
test_print_path(void)
{
    obstacle_path_t path;
    uint32_t        idx;

    if (obstacle_task_get_path(&path))
    {
        (void)tm_printf(msg_path_header);

        for (idx = 0U; (idx < path.count) && (idx < OBSTACLE_PATH_MAX_POINTS);
             idx++)
        {
            (void)tm_printf(msg_path_row,
                            path.points[idx].angle_deg,
                            path.points[idx].distance_mm,
                            test_verdict_name(path.points[idx].verdict));
        }

        (void)tm_printf(msg_path, test_verdict_name(path.verdict),
                        path.need_mm, path.known_mm);
    }
    else
    {
        (void)tm_printf(msg_path_fail);
    }
}

/*!
 * @brief Print the latest scan: every angle, the profile, the action.
 */
static void
test_print_scan(void)
{
    static obstacle_scan_result_t scan;   /* Static: keeps it off the stack. */
    uint32_t                      idx;

    if (obstacle_task_get_scan(&scan))
    {
        (void)tm_printf(msg_scan_header);

        for (idx = 0U; idx < scan.count; idx++)
        {
            (void)tm_printf(msg_scan_row,
                            scan.points[idx].angle_deg,
                            scan.points[idx].distance_mm,
                            scan.points[idx].valid_count,
                            scan.points[idx].silent_count,
                            test_class_name(scan.points[idx].point_class));
        }

        (void)tm_printf(msg_profile,
                        scan.profile.distance_mm,
                        scan.profile.angle_deg,
                        scan.right_edge_deg,
                        scan.left_edge_deg,
                        scan.profile.width_mm,
                        scan.profile.clear_right_mm,
                        scan.profile.clear_left_mm,
                        scan.duration_ms);
        (void)tm_printf(msg_action, test_action_name(scan.profile.action));
        test_print_plan(&scan.profile);
    }
    else
    {
        (void)tm_printf(msg_scan_fail);
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

    if ((0U != (changed & OBSTACLE_EVT_SCAN_DONE)) &&
        (0U != (current & OBSTACLE_EVT_SCAN_DONE)))
    {
        test_print_scan();
    }

    if ((0U != (changed & OBSTACLE_EVT_PATH_DONE)) &&
        (0U != (current & OBSTACLE_EVT_PATH_DONE)))
    {
        test_print_path();
    }
}

/*!
 * @brief Ask for a path check with the last plan's need and obstacle.
 *
 * @param[in] side TURN_RIGHT or TURN_LEFT.
 */
static void
test_request_path(obstacle_action_t side)
{
    obstacle_task_request_path_check(side, check_need_mm, check_known_mm);
    (void)tm_printf(msg_path_req,
                    (OBSTACLE_ACTION_TURN_LEFT == side) ? "LEFT" : "RIGHT",
                    check_need_mm, check_known_mm);
}

/*!
 * @brief Send one turn or leg to Buddy 2's motion task.
 *
 * @param[in,out] p_cmd Command; its sequence id is filled in here.
 *
 * @return true if Motion accepted it.
 */
static bool
test_submit(motion_command_t * p_cmd)
{
    motion_status_t status;
    motion_result_t result;

    if (MOTION_RESULT_OK == motion_get_status(&status))
    {
        turn_start_left = status.left_encoder_counts;
        turn_start_right = status.right_encoder_counts;
    }

    turn_seq++;

    if (0U == turn_seq)
    {
        turn_seq = 1U;                      /* 0 is not a valid sequence. */
    }

    p_cmd->sequence_id = turn_seq;
    result = motion_submit(p_cmd);
    b_turning = (MOTION_RESULT_OK == result);

    if (!b_turning)
    {
        /* Cast to int: the enum is printed as its number. */
        (void)tm_printf(msg_turn_refused, (int)result);
    }

    return (b_turning);
}

/*!
 * @brief Start a turn on the spot with Buddy 2's motion task.
 *
 * @param[in] side    TURN_RIGHT or TURN_LEFT.
 * @param[in] deg     Angle to turn.
 * @param[in] b_check true to run the path check when the turn completes.
 *
 * @return true if Motion accepted it.
 */
static bool
test_start_turn(obstacle_action_t side, uint32_t deg, bool b_check)
{
    motion_command_t cmd = { 0 };
    int32_t          angle_mdeg;
    bool             b_ok;

    /* Cast to int32_t: deg is at most 2 x OBSTACLE_PLAN_MAX_DEG. */
    angle_mdeg = (int32_t)deg * TEST_MDEG_PER_DEG;

    if (OBSTACLE_ACTION_TURN_LEFT == side)
    {
        angle_mdeg = -angle_mdeg;
    }

    cmd.type                        = MOTION_COMMAND_TURN_ANGLE;
    cmd.timeout_ms                  = TEST_TURN_TIMEOUT_MS;
    cmd.value.turn.angle_mdeg       = angle_mdeg;
    cmd.value.turn.wheel_speed_mm_s = TEST_TURN_SPEED_MM_S;

    b_ok = test_submit(&cmd);

    if (b_ok)
    {
        b_check_after_turn = b_check;
        turn_side = side;
        (void)tm_printf(msg_turn_req,
                        (OBSTACLE_ACTION_TURN_LEFT == side) ? "LEFT" : "RIGHT",
                        deg, turn_seq);
    }

    return (b_ok);
}

/*!
 * @brief Drive one straight leg.
 *
 * @param[in] length_mm Distance to drive.
 *
 * @return true if Motion accepted it.
 */
static bool
test_start_leg(uint32_t length_mm)
{
    motion_command_t cmd = { 0 };
    bool             b_ok;

    cmd.type                       = MOTION_COMMAND_MOVE_DISTANCE;
    /* Twice the driving time, plus 1 s for starting and stopping. */
    cmd.timeout_ms                 = ((length_mm * 2000U)
                                      / (uint32_t)TEST_LEG_SPEED_MM_S) + 1000U;
    /* Cast to int32_t: a leg is at most a few metres. */
    cmd.value.distance.distance_mm = (int32_t)length_mm;
    cmd.value.distance.speed_mm_s  = TEST_LEG_SPEED_MM_S;

    b_ok = test_submit(&cmd);

    if (b_ok)
    {
        b_check_after_turn = false;
        (void)tm_printf(msg_leg_req, length_mm, turn_seq);
    }

    return (b_ok);
}

/*!
 * @brief End the bypass early: stop the motors and say why.
 *
 * @param[in] p_why Reason, printed.
 */
static void
test_bypass_stop(char const * p_why)
{
    (void)motion_stop();
    b_turning = false;
    bypass_step = TEST_BYPASS_IDLE;
    (void)tm_printf(msg_bypass_stop, p_why);
}

/*!
 * @brief b: start the whole bypass with the first turn.
 */
static void
test_bypass_begin(void)
{
    (void)tm_printf(msg_bypass_start,
                    (OBSTACLE_ACTION_TURN_LEFT == last_plan.side) ?
                    "LEFT" : "RIGHT",
                    last_plan.turn_deg, last_plan.leg_mm, last_plan.check_mm);
    bypass_step = TEST_BYPASS_TURN_OUT;

    if (!test_start_turn(last_plan.side, last_plan.turn_deg, true))
    {
        test_bypass_stop("first turn refused");
    }
}

/*!
 * @brief A bypass turn or leg has finished: start the next one.
 */
static void
test_bypass_next(void)
{
    bool b_ok = true;

    switch (bypass_step)
    {
        case TEST_BYPASS_TURN_OUT:
            /* test_poll_turn() has asked for the path check. */
            bypass_step = TEST_BYPASS_CHECK;
            break;

        case TEST_BYPASS_LEG_OUT:
            bypass_step = TEST_BYPASS_TURN_APEX;
            b_ok = test_start_turn((OBSTACLE_ACTION_TURN_LEFT ==
                                    last_plan.side) ?
                                   OBSTACLE_ACTION_TURN_RIGHT :
                                   OBSTACLE_ACTION_TURN_LEFT,
                                   last_plan.turn_deg + TEST_RETURN_DEG,
                                   false);
            break;

        case TEST_BYPASS_TURN_APEX:
            bypass_step = TEST_BYPASS_LEG_BACK;
            b_ok = test_start_leg(TEST_RETURN_LEG_X * last_plan.apex_mm);
            break;

        case TEST_BYPASS_LEG_BACK:
            /* Where the car should be crossing the line. */
            bypass_step = TEST_BYPASS_IDLE;
            (void)tm_printf(msg_bypass_done);
            break;

        default:
            /* Not in a bypass (t or u). */
            break;
    }

    if (!b_ok)
    {
        test_bypass_stop("Motion refused the next step");
    }
}

/*!
 * @brief The path check after the first turn is ready: go, or stop.
 */
static void
test_bypass_checked(void)
{
    obstacle_path_t path;

    if (TEST_BYPASS_CHECK == bypass_step)
    {
        /* OPEN (no echo) is accepted, as in vc_task.c: the guard watches
         * the leg. */
        if (obstacle_task_get_path(&path) &&
            ((OBSTACLE_PATH_CLEAR == path.verdict) ||
             (OBSTACLE_PATH_OPEN == path.verdict)))
        {
            bypass_step = TEST_BYPASS_LEG_OUT;

            if (!test_start_leg(last_plan.leg_mm))
            {
                test_bypass_stop("leg refused");
            }
        }
        else
        {
            test_bypass_stop("path check did not clear the leg");
        }
    }
}

/*!
 * @brief On a leg, stop if the sensor sees something within the guard.
 */
static void
test_bypass_guard(void)
{
    obstacle_status_t status;

    if (((TEST_BYPASS_LEG_OUT == bypass_step) ||
         (TEST_BYPASS_LEG_BACK == bypass_step)) &&
        obstacle_task_get_status(&status) &&
        (OBSTACLE_RANGE_OK == status.range_status) &&
        (status.distance_mm < TEST_LEG_GUARD_MM))
    {
        test_bypass_stop("something within the guard distance");
    }
}

/*!
 * @brief Report the turn when it ends; start the path check if asked.
 */
static void
test_poll_turn(void)
{
    motion_status_t status;

    if (b_turning && (MOTION_RESULT_OK == motion_get_status(&status)) &&
        (turn_seq == status.active_sequence_id))
    {
        if (MOTION_STATE_COMPLETE == status.state)
        {
            b_turning = false;
            (void)tm_printf(msg_turn_done, status.target_angle_mdeg,
                            status.travelled_mm,
                            status.left_encoder_counts - turn_start_left,
                            status.right_encoder_counts - turn_start_right);

            if (b_check_after_turn)
            {
                test_request_path(turn_side);
            }

            test_bypass_next();
        }
        else if ((MOTION_STATE_STOPPED == status.state) ||
                 (MOTION_STATE_FAULT == status.state))
        {
            b_turning = false;

            /* Casts to int: the enums are printed as numbers. */
            (void)tm_printf(msg_turn_failed, (int)status.state,
                            (int)status.fault);

            if (TEST_BYPASS_IDLE != bypass_step)
            {
                test_bypass_stop("Motion stopped or faulted");
            }
        }
        else
        {
            /* Still turning. */
        }
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

        case TEST_KEY_SCAN:
            obstacle_task_request_scan();
            (void)tm_printf(msg_scan_req);
            break;

        case TEST_KEY_PATH:
            test_request_path(check_side);
            break;

        case TEST_KEY_PATH_RIGHT:
            test_request_path(OBSTACLE_ACTION_TURN_RIGHT);
            break;

        case TEST_KEY_PATH_LEFT:
            test_request_path(OBSTACLE_ACTION_TURN_LEFT);
            break;

        case TEST_KEY_TURN:
            if (0U == plan_turn_deg)
            {
                (void)tm_printf(msg_turn_no_plan);
            }
            else
            {
                (void)test_start_turn(check_side, plan_turn_deg, true);
            }
            break;

        case TEST_KEY_UNDO:
            if (0U == plan_turn_deg)
            {
                (void)tm_printf(msg_turn_no_plan);
            }
            else
            {
                (void)test_start_turn((OBSTACLE_ACTION_TURN_LEFT == check_side) ?
                                OBSTACLE_ACTION_TURN_RIGHT :
                                OBSTACLE_ACTION_TURN_LEFT,
                                plan_turn_deg, false);
            }
            break;

        case TEST_KEY_BYPASS:
            if (0U == plan_turn_deg)
            {
                (void)tm_printf(msg_turn_no_plan);
            }
            else
            {
                test_bypass_begin();
            }
            break;

        case TEST_KEY_STOP:
            (void)motion_stop();
            bypass_step = TEST_BYPASS_IDLE;
            (void)tm_printf(msg_stopped);
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
    T_RFLG          flag_info;
    UINT            previous = 0U;
    UINT            current = 0U;
    ER              result;
    motion_result_t motion_result;

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

    /* Motors stay still until t or u. */
    motion_result = motion_init();

    if (MOTION_RESULT_OK == motion_result)
    {
        motion_result = motion_start_task();
    }

    if (MOTION_RESULT_OK != motion_result)
    {
        /* Cast to int: the enum is printed as its number. */
        (void)tm_printf(msg_motion_failed, (int)motion_result);
    }

    for (;;)
    {
        /* Poll: the bits are levels, so waiting on them would return at
         * once for as long as OBSTACLE_AHEAD stays set. */
        (void)tk_dly_tsk(TEST_POLL_MS);

        result = tk_ref_flg(obstacle_task_get_flag(), &flag_info);
        current = (E_OK == result) ?
                  (flag_info.flgptn & (OBSTACLE_EVT_AHEAD
                                       | OBSTACLE_EVT_FAULT
                                       | OBSTACLE_EVT_SCAN_DONE
                                       | OBSTACLE_EVT_PATH_DONE)) : 0U;

        test_print_events(previous, current);

        if ((0U != ((previous ^ current) & OBSTACLE_EVT_PATH_DONE)) &&
            (0U != (current & OBSTACLE_EVT_PATH_DONE)))
        {
            test_bypass_checked();
        }

        previous = current;
        test_poll_turn();
        test_bypass_guard();

        if (b_rows_on)
        {
            test_print_row(current);
        }

        test_handle_key(obstacle_console_read_key());
    }

    return (0);
}

/*** end of file ***/
