/** @file test_vc.c
 *
 * @brief Host unit tests for the Vehicle Controller state machine (vc.c).
 *
 * Each test drives vc_step() with simulated subsystem inputs, one 20 ms
 * step at a time, and checks the resulting state and outputs against the
 * system-level statechart.  No hardware or RTOS is needed.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "vc.h"

/* ------------------------------------------------------------------------ *
 * Minimal test harness
 * ------------------------------------------------------------------------ */

static uint32_t g_checks;
static uint32_t g_failures;

#define CHECK(cond)                                                         \
    do                                                                      \
    {                                                                       \
        g_checks++;                                                         \
        if (!(cond))                                                        \
        {                                                                   \
            g_failures++;                                                   \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                   \
    } while (0)

#define CHECK_STATE(expected)                                               \
    do                                                                      \
    {                                                                       \
        g_checks++;                                                         \
        if ((expected) != g_out.state)                                      \
        {                                                                   \
            g_failures++;                                                   \
            printf("  FAIL %s:%d: state %s, expected %s\n", __FILE__,       \
                   __LINE__, vc_state_name(g_out.state),                    \
                   vc_state_name(expected));                                \
        }                                                                   \
    } while (0)

/* ------------------------------------------------------------------------ *
 * Simulation helpers
 * ------------------------------------------------------------------------ */

static vc_t         g_vc;
static vc_inputs_t  g_in;
static vc_outputs_t g_out;

/* Fresh controller and calm inputs: on the line, centred, car stopped. */
static void
reset (uint32_t start_ms)
{
    vc_init(&g_vc);
    memset(&g_in, 0, sizeof(g_in));
    memset(&g_out, 0, sizeof(g_out));
    g_in.now_ms       = start_ms;
    g_in.line         = VC_LINE_CENTRE;
    g_in.b_stationary = true;
}

/* Run one 20 ms step, then clear the one-shot event inputs. */
static void
step (void)
{
    vc_step(&g_vc, &g_in, &g_out);

    g_in.now_ms           += VC_PERIOD_MS;
    g_in.cmd               = VC_CMD_NONE;
    g_in.barcode           = VC_NAV_NONE;
    g_in.b_motion_done     = false;
    g_in.b_motion_failed   = false;
    g_in.b_obstacle_ahead  = false;
    g_in.b_scan_done       = false;
    g_in.b_obstacle_passed = false;
}

/* Run steps until roughly ms milliseconds of quiet time have passed. */
static void
wait_ms (uint32_t ms)
{
    uint32_t elapsed;

    for (elapsed = 0U; elapsed < ms; elapsed += VC_PERIOD_MS)
    {
        step();
    }
}

/* INITIALISE -> READY -> FOLLOW_LINE. */
static void
go_to_follow_line (uint32_t start_ms)
{
    reset(start_ms);
    g_in.b_init_ok = true;
    step();
    g_in.cmd = VC_CMD_START;
    step();
}

/* FOLLOW_LINE -> OBSTACLE_SCAN. */
static void
go_to_obstacle_scan (void)
{
    go_to_follow_line(1000U);
    g_in.b_obstacle_ahead = true;
    step();
}

/* ------------------------------------------------------------------------ *
 * Tests
 * ------------------------------------------------------------------------ */

static void
test_start_up (void)
{
    printf("start-up\n");

    reset(0U);
    step();
    CHECK_STATE(VC_STATE_INITIALISE);          /* Waits for init result. */

    g_in.b_init_ok = true;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_MOTION_STOP == g_out.motion);    /* READY keeps the car still. */

    step();
    CHECK_STATE(VC_STATE_READY);              /* No START yet. */

    g_in.cmd = VC_CMD_START;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(g_out.b_state_changed);

    reset(0U);
    g_in.b_init_failed = true;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
}

static void
test_follow_line_and_hump (void)
{
    printf("follow line + hump speed cap\n");

    go_to_follow_line(0U);
    g_in.line_error = -40;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_MOTION_DRIVE == g_out.motion);
    CHECK(VC_SPEED_NORMAL_MM_S == g_out.speed_mm_s);
    CHECK(-40 == g_out.steer);
    CHECK(!g_out.b_state_changed);

    g_in.b_hump = true;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);        /* Hump is not a state. */
    CHECK(VC_SPEED_HUMP_MM_S == g_out.speed_mm_s);

    g_in.b_hump = false;
    step();
    CHECK(VC_SPEED_NORMAL_MM_S == g_out.speed_mm_s);
}

static void
test_barcode_turn (void)
{
    printf("barcode stored, turn at junction\n");

    go_to_follow_line(0U);
    g_in.barcode = VC_NAV_LEFT;               /* Read before the junction. */
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);        /* Reading alone: no turn. */

    step();                                   /* Keep driving. */
    CHECK_STATE(VC_STATE_FOLLOW_LINE);

    g_in.line = VC_LINE_JUNCTION;
    step();
    CHECK_STATE(VC_STATE_HANDLE_JUNCTION);

    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK_STATE(VC_STATE_TURNING);
    CHECK(VC_MOTION_TURN == g_out.motion);
    CHECK(VC_NAV_LEFT == g_out.turn);

    step();
    CHECK_STATE(VC_STATE_TURNING);            /* Turn still running. */

    g_in.b_motion_done = true;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);

    /* The stored command was used up: the next junction goes straight. */
    g_in.line = VC_LINE_JUNCTION;
    step();
    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
}

static void
test_junction_straight_and_uturn (void)
{
    printf("junction: STRAIGHT and U-TURN\n");

    go_to_follow_line(0U);
    g_in.barcode = VC_NAV_STRAIGHT;
    step();
    g_in.line = VC_LINE_JUNCTION;
    step();
    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);        /* Straight is not a turn. */
    CHECK(VC_MOTION_TURN != g_out.motion);

    g_in.barcode = VC_NAV_UTURN;
    step();
    g_in.line = VC_LINE_JUNCTION;
    step();
    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK_STATE(VC_STATE_TURNING);
    CHECK(VC_NAV_UTURN == g_out.turn);
}

static void
test_turn_failures (void)
{
    printf("turn timeout and motion failure\n");

    go_to_follow_line(0U);
    g_in.barcode = VC_NAV_RIGHT;
    step();
    g_in.line = VC_LINE_JUNCTION;
    step();
    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK_STATE(VC_STATE_TURNING);

    wait_ms(VC_TURN_TIMEOUT_MS - 100U);
    CHECK_STATE(VC_STATE_TURNING);            /* Not yet timed out. */

    wait_ms(200U);
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
    step();                                   /* While stopped: no driving. */
    CHECK(VC_MOTION_DRIVE != g_out.motion);
    CHECK(VC_MOTION_TURN != g_out.motion);

    go_to_follow_line(0U);
    g_in.barcode = VC_NAV_LEFT;
    step();
    g_in.line = VC_LINE_JUNCTION;
    step();
    g_in.line = VC_LINE_CENTRE;
    step();
    g_in.b_motion_failed = true;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
}

static void
test_line_lost_and_found (void)
{
    printf("line lost, reacquire, recovery timeout\n");

    go_to_follow_line(0U);
    g_in.line = VC_LINE_LOST;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    CHECK(VC_OBST_ABORT != g_out.obstacle);   /* Buddy 5 not involved. */

    step();
    CHECK(VC_MOTION_DRIVE == g_out.motion);   /* Creep forward. */
    CHECK(VC_SPEED_SEARCH_MM_S == g_out.speed_mm_s);
    CHECK(0 == g_out.steer);

    g_in.line = VC_LINE_RIGHT;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_OBST_ABORT != g_out.obstacle);   /* The bug fixed earlier. */

    g_in.line = VC_LINE_LOST;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    wait_ms(VC_REACQUIRE_TIMEOUT_MS + 40U);
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
}

static void
test_obstacle_continue (void)
{
    printf("obstacle: scan, CONTINUE\n");

    go_to_obstacle_scan();
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);
    CHECK(VC_MOTION_STOP == g_out.motion);    /* Stop before scanning. */
    CHECK(VC_OBST_SCAN_REQUEST == g_out.obstacle);

    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_CONTINUE;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
}

static void
test_obstacle_bypass (void)
{
    printf("obstacle: bypass, reacquire\n");

    go_to_obstacle_scan();
    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_TURN_LEFT;
    step();
    CHECK_STATE(VC_STATE_BYPASS);
    CHECK(VC_MOTION_BYPASS == g_out.motion);
    CHECK(VC_AVOID_TURN_LEFT == g_out.bypass_side);
    CHECK(VC_OBST_BYPASS_START == g_out.obstacle);

    g_in.line = VC_LINE_LOST;                 /* Off the line while bypassing. */
    g_in.b_obstacle_passed = true;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);

    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_OBST_ABORT == g_out.obstacle);   /* Release Buddy 5. */

    go_to_obstacle_scan();
    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_TURN_RIGHT;
    step();
    g_in.b_motion_failed = true;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);

    go_to_obstacle_scan();
    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_TURN_RIGHT;
    step();
    wait_ms(VC_BYPASS_TIMEOUT_MS + 40U);
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
}

static void
test_obstacle_retry_and_stop (void)
{
    printf("obstacle: reverse + rescan limit, STOP, scan timeout\n");

    go_to_obstacle_scan();
    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_REVERSE_RETRY;
    step();
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);      /* Retry 1. */
    CHECK(VC_MOTION_REVERSE == g_out.motion);
    CHECK(VC_OBST_SCAN_REQUEST == g_out.obstacle);

    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_REVERSE_RETRY;
    step();
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);      /* Retry 2. */

    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_REVERSE_RETRY;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);     /* Retries used up. */
    CHECK(VC_OBST_ABORT == g_out.obstacle);

    go_to_obstacle_scan();
    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_STOP;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);

    go_to_obstacle_scan();
    wait_ms(VC_SCAN_TIMEOUT_MS - 100U);
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);
    wait_ms(200U);
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);

    /* A retry restarts the scan timer. */
    go_to_obstacle_scan();
    wait_ms(VC_SCAN_TIMEOUT_MS - 200U);
    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_REVERSE_RETRY;
    step();
    wait_ms(500U);
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);
}

static void
test_arbitration (void)
{
    printf("arbitration order\n");

    /* Obstacle beats junction in the same step. */
    go_to_follow_line(0U);
    g_in.line             = VC_LINE_JUNCTION;
    g_in.b_obstacle_ahead = true;
    step();
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);

    /* A serious fault beats a laptop STOP. */
    go_to_follow_line(0U);
    g_in.cmd            = VC_CMD_STOP;
    g_in.serious_faults = 1U;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
}

static void
test_stop_command (void)
{
    printf("laptop STOP\n");

    go_to_follow_line(0U);
    g_in.cmd = VC_CMD_STOP;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_MOTION_STOP == g_out.motion);    /* Brake first. */

    g_in.cmd = VC_CMD_STOP;                   /* STOP when already READY. */
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(!g_out.b_state_changed);

    /* STOP during a bypass also releases Buddy 5. */
    go_to_obstacle_scan();
    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_TURN_LEFT;
    step();
    g_in.cmd = VC_CMD_STOP;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_OBST_ABORT == g_out.obstacle);

    /* A stored barcode does not survive a stop. */
    go_to_follow_line(0U);
    g_in.barcode = VC_NAV_LEFT;
    step();
    g_in.cmd = VC_CMD_STOP;
    step();
    g_in.cmd = VC_CMD_START;
    step();
    g_in.line = VC_LINE_JUNCTION;
    step();
    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);        /* No old turn executed. */
}

static void
test_emergency_recovery (void)
{
    printf("emergency stop and recovery\n");

    go_to_follow_line(0U);
    g_in.b_stationary   = false;              /* Car moving. */
    g_in.serious_faults = 0x0004U;            /* e.g. motion timeout. */
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
    CHECK(VC_MOTION_STOP == g_out.motion);

    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);     /* Fault still set. */

    g_in.serious_faults = 0U;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);     /* Still moving. */

    g_in.b_stationary = true;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);     /* Latched: not acknowledged. */

    g_in.cmd = VC_CMD_START;                  /* START is not an ack.      */
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);

    g_in.cmd = VC_CMD_STOP;                   /* Operator acknowledges.    */
    step();
    CHECK_STATE(VC_STATE_READY);

    step();
    CHECK_STATE(VC_STATE_READY);              /* Needs a new START.        */

    /* STOP is ignored while the fault is still present. */
    go_to_follow_line(0U);
    g_in.serious_faults = 1U;
    step();
    g_in.cmd = VC_CMD_STOP;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);

    /* A timeout-caused emergency also stays latched. */
    go_to_follow_line(0U);
    g_in.line = VC_LINE_LOST;
    step();
    wait_ms(VC_REACQUIRE_TIMEOUT_MS + 40U);
    wait_ms(500U);
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
    g_in.cmd = VC_CMD_STOP;
    step();
    CHECK_STATE(VC_STATE_READY);
}

static void
test_finished (void)
{
    printf("finish\n");

    go_to_follow_line(0U);
    g_in.b_finish = true;
    step();
    CHECK_STATE(VC_STATE_FINISHED);
    CHECK(VC_MOTION_STOP == g_out.motion);

    g_in.cmd            = VC_CMD_START;
    g_in.serious_faults = 1U;
    step();
    CHECK_STATE(VC_STATE_FINISHED);           /* Stays finished. */
}

static void
test_timer_wraparound (void)
{
    printf("kernel time wrap-around\n");

    /* Start just before the 32-bit millisecond counter wraps. */
    go_to_follow_line(0xFFFFFF00U);
    g_in.line = VC_LINE_LOST;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    wait_ms(1000U);                           /* Crosses the wrap. */
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);     /* No false timeout. */
    wait_ms(VC_REACQUIRE_TIMEOUT_MS);
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
}

static void
test_names_and_null (void)
{
    printf("state names and NULL safety\n");

    CHECK(0 == strcmp("FOLLOW_LINE", vc_state_name(VC_STATE_FOLLOW_LINE)));
    CHECK(0 == strcmp("UNKNOWN", vc_state_name(VC_STATE_COUNT)));

    vc_init(NULL);                            /* Must not crash. */
    vc_step(NULL, &g_in, &g_out);
    vc_step(&g_vc, NULL, &g_out);
    vc_step(&g_vc, &g_in, NULL);
    CHECK(true);
}

/* ------------------------------------------------------------------------ */

int
main (void)
{
    test_start_up();
    test_follow_line_and_hump();
    test_barcode_turn();
    test_junction_straight_and_uturn();
    test_turn_failures();
    test_line_lost_and_found();
    test_obstacle_continue();
    test_obstacle_bypass();
    test_obstacle_retry_and_stop();
    test_arbitration();
    test_stop_command();
    test_emergency_recovery();
    test_finished();
    test_timer_wraparound();
    test_names_and_null();

    printf("\n%u checks, %u failed\n", (unsigned) g_checks,
           (unsigned) g_failures);

    return (0U == g_failures) ? 0 : 1;
}

/*** end of file ***/
