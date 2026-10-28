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
    vc_init(&g_vc, start_ms);
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
    g_in.line_error = 0;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_MOTION_DRIVE == g_out.motion);
    CHECK(VC_SPEED_NORMAL_MM_S == g_out.speed_mm_s);
    CHECK(!g_out.b_state_changed);

    /* Off centre: steer by the error, and slow down for the bend. */
    g_in.line_error = -40;
    step();
    CHECK(-40 == g_out.steer);
    CHECK(VC_SPEED_NORMAL_MM_S > g_out.speed_mm_s);
    CHECK(VC_SPEED_CURVE_MM_S < g_out.speed_mm_s);

    g_in.line_error = 127;
    step();
    CHECK(VC_SPEED_CURVE_MM_S == g_out.speed_mm_s);

    g_in.line_error = 0;
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
    CHECK_STATE(VC_STATE_TURNING);            /* Decided in the same step. */
    CHECK(g_out.b_state_changed);
    CHECK(VC_MOTION_TURN == g_out.motion);
    CHECK(VC_NAV_LEFT == g_out.turn);

    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK_STATE(VC_STATE_TURNING);            /* Turn still running. */
    CHECK(VC_MOTION_NONE == g_out.motion);    /* Command sent only once. */

    g_in.b_motion_done = true;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);

    /* The stored command was used up: the next junction goes straight. */
    g_in.line = VC_LINE_JUNCTION;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(g_out.b_state_changed);             /* Via HANDLE_JUNCTION. */
    CHECK(VC_MOTION_DRIVE == g_out.motion);
    CHECK(0 == g_out.steer);
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
    CHECK_STATE(VC_STATE_FOLLOW_LINE);        /* Straight is not a turn. */
    CHECK(VC_MOTION_DRIVE == g_out.motion);
    g_in.line = VC_LINE_CENTRE;
    step();

    g_in.barcode = VC_NAV_UTURN;
    step();
    g_in.line = VC_LINE_JUNCTION;
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
    CHECK_STATE(VC_STATE_TURNING);
    g_in.line = VC_LINE_CENTRE;

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
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);      /* Retry 1: reverse first. */
    CHECK(VC_MOTION_REVERSE == g_out.motion);
    CHECK(VC_OBST_SCAN_REQUEST != g_out.obstacle);  /* Not while reversing. */

    step();
    CHECK(VC_OBST_SCAN_REQUEST != g_out.obstacle);  /* Still reversing. */

    g_in.b_scan_done    = true;               /* Stale result: ignored. */
    g_in.recommendation = VC_AVOID_CONTINUE;
    step();
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);

    g_in.b_motion_done = true;                /* Reverse finished. */
    step();
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);
    CHECK(VC_OBST_SCAN_REQUEST == g_out.obstacle);  /* Now rescan. */

    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_REVERSE_RETRY;
    step();
    CHECK(VC_MOTION_REVERSE == g_out.motion); /* Retry 2. */
    g_in.b_motion_done = true;
    step();
    CHECK(VC_OBST_SCAN_REQUEST == g_out.obstacle);

    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_REVERSE_RETRY;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);     /* Retries used up. */
    CHECK(VC_OBST_ABORT == g_out.obstacle);

    /* Reverse fails -> emergency. */
    go_to_obstacle_scan();
    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_REVERSE_RETRY;
    step();
    g_in.b_motion_failed = true;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
    CHECK(VC_OBST_ABORT == g_out.obstacle);

    /* Reverse never finishes -> emergency. */
    go_to_obstacle_scan();
    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_REVERSE_RETRY;
    step();
    wait_ms(VC_SCAN_TIMEOUT_MS + 40U);
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);

    /* Unknown recommendation -> emergency (fail safe). */
    go_to_obstacle_scan();
    g_in.b_scan_done    = true;
    g_in.recommendation = (vc_avoid_t) 99;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);

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
    g_in.b_motion_done = true;
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
    CHECK_STATE(VC_STATE_FOLLOW_LINE);        /* No old turn executed. */
    CHECK(VC_MOTION_TURN != g_out.motion);
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

    vc_init(NULL, 0U);                        /* Must not crash. */
    vc_step(NULL, &g_in, &g_out);
    vc_step(&g_vc, NULL, &g_out);
    vc_step(&g_vc, &g_in, NULL);
    CHECK(true);
}

static void
test_junction_edge (void)
{
    printf("junction handled once, however long it is seen\n");

    go_to_follow_line(0U);
    g_in.line = VC_LINE_JUNCTION;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);        /* No barcode: straight on. */

    g_in.barcode = VC_NAV_LEFT;               /* Next junction's barcode. */
    step();                                   /* Still over junction 1.   */
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(!g_out.b_state_changed);            /* Not re-triggered.        */
    CHECK(0 == g_out.steer);
    wait_ms(200U);
    CHECK_STATE(VC_STATE_FOLLOW_LINE);

    g_in.line = VC_LINE_CENTRE;               /* Leave junction 1. */
    step();
    g_in.line = VC_LINE_JUNCTION;             /* Junction 2. */
    step();
    CHECK_STATE(VC_STATE_TURNING);
    CHECK(VC_NAV_LEFT == g_out.turn);         /* Barcode kept for it. */
}

static void
test_timeout_vs_success (void)
{
    printf("success at the deadline beats the timeout\n");

    go_to_follow_line(0U);
    g_in.barcode = VC_NAV_LEFT;
    step();
    g_in.line = VC_LINE_JUNCTION;
    step();                                   /* Turn starts here. */
    g_in.line = VC_LINE_CENTRE;
    wait_ms(VC_TURN_TIMEOUT_MS - VC_PERIOD_MS);
    CHECK_STATE(VC_STATE_TURNING);
    g_in.b_motion_done = true;                /* Exactly at the deadline. */
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
}

static void
test_init_timeout (void)
{
    printf("initialise timeout\n");

    reset(0U);
    wait_ms(VC_INIT_TIMEOUT_MS - 100U);
    CHECK_STATE(VC_STATE_INITIALISE);
    wait_ms(200U);
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
}

static void
test_obstacle_while_reacquiring (void)
{
    printf("obstacle while searching for the line\n");

    go_to_follow_line(0U);
    g_in.line = VC_LINE_LOST;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    g_in.b_obstacle_ahead = true;
    step();
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);
    CHECK(VC_MOTION_STOP == g_out.motion);
    CHECK(VC_OBST_SCAN_REQUEST == g_out.obstacle);

    /* During a bypass: a barcode read while searching is kept, and
     * Buddy 5 is released only once the line is found. */
    go_to_obstacle_scan();
    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_TURN_LEFT;
    step();
    g_in.line              = VC_LINE_LOST;
    g_in.b_obstacle_passed = true;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    g_in.barcode          = VC_NAV_RIGHT;
    g_in.b_obstacle_ahead = true;
    step();
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);
    CHECK(VC_OBST_ABORT != g_out.obstacle);   /* Still Buddy 5's job. */
    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_CONTINUE;
    g_in.line           = VC_LINE_CENTRE;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_OBST_ABORT == g_out.obstacle);   /* Released once. */
    g_in.line = VC_LINE_JUNCTION;
    step();
    CHECK_STATE(VC_STATE_TURNING);            /* Barcode was kept. */
    CHECK(VC_NAV_RIGHT == g_out.turn);
}

static void
test_search_towards_last_side (void)
{
    printf("line search curves towards where the line was last seen\n");

    /* Line drifts off to the left, then lost: search steering left. */
    go_to_follow_line(0U);
    g_in.line       = VC_LINE_LEFT;
    g_in.line_error = -127;
    step();
    g_in.line       = VC_LINE_LOST;
    g_in.line_error = 0;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    step();
    CHECK(VC_MOTION_DRIVE == g_out.motion);
    CHECK(VC_SPEED_SEARCH_MM_S == g_out.speed_mm_s);
    CHECK(-VC_SEARCH_STEER == g_out.steer);

    /* Off to the right: search steering right.  A centred reading in
     * between keeps the last side. */
    go_to_follow_line(0U);
    g_in.line       = VC_LINE_RIGHT;
    g_in.line_error = 63;
    step();
    g_in.line       = VC_LINE_CENTRE;
    g_in.line_error = 0;
    step();
    g_in.line = VC_LINE_LOST;
    step();
    step();
    CHECK(VC_SEARCH_STEER == g_out.steer);

    /* Never seen a side: search straight. */
    go_to_follow_line(0U);
    g_in.line = VC_LINE_LOST;
    step();
    step();
    CHECK(0 == g_out.steer);
}

static void
test_reacquire_on_junction (void)
{
    printf("line found again on a junction\n");

    /* After a plain line loss: the stored turn happens at THIS junction. */
    go_to_follow_line(0U);
    g_in.barcode = VC_NAV_LEFT;
    step();
    g_in.line = VC_LINE_LOST;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    g_in.line = VC_LINE_JUNCTION;
    step();
    CHECK_STATE(VC_STATE_TURNING);
    CHECK(VC_MOTION_TURN == g_out.motion);
    CHECK(VC_NAV_LEFT == g_out.turn);
    CHECK(VC_OBST_ABORT != g_out.obstacle);   /* Buddy 5 not involved. */

    /* No stored barcode: straight across, and only once. */
    go_to_follow_line(0U);
    g_in.line = VC_LINE_LOST;
    step();
    g_in.line = VC_LINE_JUNCTION;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_MOTION_DRIVE == g_out.motion);
    CHECK(0 == g_out.steer);
    step();                                   /* Still over it. */
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(!g_out.b_state_changed);

    /* After a bypass: turn, and Buddy 5 is released. */
    go_to_obstacle_scan();
    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_TURN_RIGHT;
    step();
    g_in.line              = VC_LINE_LOST;
    g_in.b_obstacle_passed = true;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    g_in.barcode = VC_NAV_RIGHT;
    step();
    g_in.line = VC_LINE_JUNCTION;
    step();
    CHECK_STATE(VC_STATE_TURNING);
    CHECK(VC_NAV_RIGHT == g_out.turn);
    CHECK(VC_OBST_ABORT == g_out.obstacle);
}

static void
test_invalid_inputs (void)
{
    printf("invalid input values fail safe\n");

    /* Unknown line code counts as LOST. */
    go_to_follow_line(0U);
    g_in.line = (vc_line_t) 77;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);

    /* Unknown command and barcode codes are ignored. */
    go_to_follow_line(0U);
    g_in.cmd     = (vc_cmd_t) 9;
    g_in.barcode = (vc_nav_t) 9;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    g_in.line = VC_LINE_JUNCTION;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);        /* No turn from bad barcode. */

    /* Corrupted controller state goes to emergency. */
    go_to_follow_line(0U);
    g_vc.state = (vc_state_t) 200;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
    CHECK(VC_MOTION_STOP == g_out.motion);
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
    test_junction_edge();
    test_timeout_vs_success();
    test_init_timeout();
    test_obstacle_while_reacquiring();
    test_reacquire_on_junction();
    test_search_towards_last_side();
    test_invalid_inputs();

    printf("\n%u checks, %u failed\n", (unsigned) g_checks,
           (unsigned) g_failures);

    return (0U == g_failures) ? 0 : 1;
}

/*** end of file ***/
