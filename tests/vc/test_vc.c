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
    g_in.line_sensors = VC_LINE_SENSOR_CENTRE;
    g_in.b_terrain_ready = true;
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

/* CALIBRATE -> FOLLOW_LINE: still, sweep left until Buddy 3 and Buddy 4
 * are satisfied, pivot back onto the line, Buddy 4 sets its floor. */
static void
run_calibration (void)
{
    wait_ms(VC_CAL_STILL_MS + VC_PERIOD_MS);  /* Still, then sweep.       */
    g_in.line           = VC_LINE_LOST;
    g_in.cal_b3         = VC_CAL_B3_OK;
    g_in.cal_b4         = VC_CAL_B4_ENOUGH;
    g_in.cal_angle_mdeg = -40000;
    step();                                   /* Turn back.               */
    g_in.line           = VC_LINE_CENTRE;
    step();                                   /* On the line: stop.       */
    g_in.cal_b4         = VC_CAL_B4_DONE;
    step();                                   /* Floor set: run.          */
}

/* While following the line: onto a junction, and along it far enough for
 * the VC to accept it (it is handled in the second step). */
static void
cross_onto_junction (void)
{
    g_in.line = VC_LINE_JUNCTION;
    step();
    g_in.travel_mm += VC_JUNCTION_CONFIRM_MM;
    step();
}

/* LEFT or RIGHT stored and the junction just handled: drive on until the
 * axle is on the crossing line, where the turn starts. */
static void
advance_to_turn (void)
{
    g_in.travel_mm += VC_JUNCTION_ADVANCE_MM;
    step();
}

/* INITIALISE -> READY -> CALIBRATE -> FOLLOW_LINE. */
static void
go_to_follow_line (uint32_t start_ms)
{
    reset(start_ms);
    g_in.b_init_ok = true;
    step();
    g_in.cmd = VC_CMD_START;
    step();
    run_calibration();
}

/* INITIALISE -> READY -> CALIBRATE, pivoting left (the sweep). */
static void
go_to_sweep (void)
{
    reset(0U);
    g_in.b_init_ok = true;
    step();
    g_in.cmd = VC_CMD_START;
    step();
    wait_ms(VC_CAL_STILL_MS + VC_PERIOD_MS);
}

/* READY -> CALIBRATE with the line reading as given when still ends, then
 * the first pivot: the way it starts depends on that reading. */
static void
go_to_sweep_with_line (vc_line_t line)
{
    reset(0U);
    g_in.b_init_ok = true;
    step();
    g_in.cmd = VC_CMD_START;
    step();
    g_in.line = line;

    while ((VC_STATE_CALIBRATE == g_out.state)
           && (VC_MOTION_PIVOT != g_out.motion))
    {
        step();                               /* Until the pivot starts.  */
    }
}

/* Both sides reached their bound with nothing learned: waiting for the
 * verdict. */
static void
go_to_judge (void)
{
    go_to_sweep();
    g_in.b_motion_done = true;                /* First side: bound.       */
    step();
    g_in.b_motion_done = true;                /* Other side: bound.       */
    step();
}

/* Sweep done on both buddies: the car is pivoting back right. */
static void
go_to_way_back (void)
{
    go_to_sweep();
    g_in.cal_b3         = VC_CAL_B3_OK;
    g_in.cal_b4         = VC_CAL_B4_ENOUGH;
    g_in.cal_angle_mdeg = -35000;
    g_in.line           = VC_LINE_LOST;
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

    g_in.b_terrain_ready = false;             /* Buddy 4 not ready. */
    g_in.cmd = VC_CMD_START;
    step();
    CHECK_STATE(VC_STATE_READY);              /* START refused. */
    CHECK(!g_out.b_state_changed);
    CHECK(VC_MOTION_DRIVE != g_out.motion);

    g_in.b_terrain_ready = true;
    g_in.cmd = VC_CMD_START;
    step();
    CHECK_STATE(VC_STATE_CALIBRATE);          /* Every START calibrates. */
    CHECK(g_out.b_state_changed);
    run_calibration();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);

    /* Once running, Buddy 4 losing readiness only slows the car. */
    g_in.b_terrain_ready = false;
    g_in.b_hump = true;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_SPEED_HUMP_MM_S == g_out.speed_mm_s);

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

    /* Centred, line not yet seen off centre: a fading centre could be
     * the line going either way, so no steer. */
    g_in.line_error = -64;
    step();
    CHECK(0 == g_out.steer);
    CHECK(VC_SPEED_NORMAL_MM_S == g_out.speed_mm_s);

    /* Line last seen on the left: steer in proportion to the error,
     * START at the edge. */
    g_in.line = VC_LINE_LEFT;
    step();
    g_in.line       = VC_LINE_CENTRE;
    g_in.line_error = -127;
    step();
    CHECK(-VC_STEER_START == g_out.steer);
    g_in.line_error = -64;
    step();
    CHECK(-32 == g_out.steer);
    CHECK(VC_SPEED_NORMAL_MM_S > g_out.speed_mm_s);
    g_in.line_error = 0;
    step();
    CHECK(0 == g_out.steer);
    CHECK(VC_SPEED_NORMAL_MM_S == g_out.speed_mm_s);

    /* Line last seen on the right: the fade may be the line drifting
     * right through the gap, so no left steer. */
    g_in.line = VC_LINE_RIGHT;
    step();
    g_in.line       = VC_LINE_CENTRE;
    g_in.line_error = -100;
    step();
    CHECK(0 == g_out.steer);
    g_in.line_error = 0;
    {
        int32_t n;

        for (n = 0; 10 > n; n++)            /* Let the ramp fade out. */
        {
            step();
        }
    }

    /* Off centre: steer starts at half (64 at 150 -> 100 mm/s). */
    g_in.line       = VC_LINE_LEFT;
    g_in.line_error = 0;
    step();
    CHECK(-64 == g_out.steer);
    CHECK(VC_SPEED_NORMAL_MM_S > g_out.speed_mm_s);
    CHECK(VC_SPEED_CURVE_MM_S < g_out.speed_mm_s);

    /* Grows by 4 per step, capped at full. */
    step();
    CHECK(-68 == g_out.steer);
    {
        int32_t n;

        for (n = 0; 20 > n; n++)
        {
            step();
        }
    }
    CHECK(-127 == g_out.steer);
    CHECK(VC_SPEED_CURVE_MM_S == g_out.speed_mm_s);

    /* Side change restarts at half of the new sign. */
    g_in.line = VC_LINE_RIGHT;
    step();
    CHECK(64 == g_out.steer);
    step();
    CHECK(68 == g_out.steer);

    /* Centre on the line gives 0; the ramp has faded below START, so the
     * next off-centre restarts. */
    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK(0 == g_out.steer);
    g_in.line = VC_LINE_RIGHT;
    step();
    CHECK(64 == g_out.steer);

    /* A short centre spell on a bend: the ramp carries on.  At full,
     * one centre step fades 127 to 111, so it resumes at 115. */
    {
        int32_t n;

        for (n = 0; 20 > n; n++)
        {
            step();
        }
    }
    CHECK(127 == g_out.steer);
    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK(0 == g_out.steer);
    g_in.line = VC_LINE_RIGHT;
    step();
    CHECK(115 == g_out.steer);

    /* A long centre spell: faded below START, it restarts. */
    g_in.line = VC_LINE_CENTRE;
    {
        int32_t n;

        for (n = 0; 6 > n; n++)
        {
            step();
        }
    }
    g_in.line = VC_LINE_RIGHT;
    step();
    CHECK(64 == g_out.steer);

    /* Lost or junction clears the ramp at once. */
    step();
    g_in.line = VC_LINE_JUNCTION;
    step();
    CHECK(0 == g_out.steer);
    g_in.line = VC_LINE_RIGHT;
    step();
    CHECK(64 == g_out.steer);

    g_in.line       = VC_LINE_CENTRE;
    g_in.line_error = 0;
    step();

    g_in.b_hump = true;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);        /* Hump is not a state. */
    CHECK(VC_SPEED_HUMP_MM_S == g_out.speed_mm_s);

    g_in.b_hump = false;
    step();
    CHECK(VC_SPEED_NORMAL_MM_S == g_out.speed_mm_s);
}

/* A barcode capture holds steering only while the line is CENTRE.  Once
 * the line is off centre, normal steering resumes and the hold has reset
 * the ramp, so the normal ramp starts fresh. */
static void
test_barcode_capture_hold (void)
{
    printf("barcode capture holds only while centred\n");

    go_to_follow_line(0U);

    /* On the line: capture holds straight at normal forward speed. */
    g_in.line              = VC_LINE_CENTRE;
    g_in.line_sensors      = VC_LINE_SENSOR_CENTRE;
    g_in.b_barcode_capture = true;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_MOTION_DRIVE == g_out.motion);
    CHECK(0 == g_out.steer);
    CHECK(VC_SPEED_NORMAL_MM_S == g_out.speed_mm_s);
    CHECK(!g_out.b_state_changed);

    /* No steering ramp builds behind the centred hold. */
    wait_ms(200U);
    CHECK(0 == g_out.steer);

    /* RIGHT during the capture: normal ramp starts fresh at START. */
    g_in.line         = VC_LINE_RIGHT;
    g_in.line_sensors = VC_LINE_SENSOR_RIGHT;
    step();
    CHECK(VC_STEER_START == g_out.steer);
    step();
    CHECK((VC_STEER_START + VC_STEER_STEP) == g_out.steer);

    /* Back on CENTRE during the capture: hold and reset again. */
    g_in.line         = VC_LINE_CENTRE;
    g_in.line_sensors = VC_LINE_SENSOR_CENTRE;
    step();
    CHECK(0 == g_out.steer);
    g_in.line         = VC_LINE_RIGHT;
    g_in.line_sensors = VC_LINE_SENSOR_RIGHT;
    step();
    CHECK(VC_STEER_START == g_out.steer);

    /* Capture ends on RIGHT: the normal ramp carries on growing. */
    g_in.b_barcode_capture = false;
    step();
    CHECK((VC_STEER_START + VC_STEER_STEP) == g_out.steer);

    /* A fresh active ramp still keeps its forward speed while centred. */
    g_in.line              = VC_LINE_CENTRE;
    g_in.line_sensors      = VC_LINE_SENSOR_CENTRE;
    g_in.b_barcode_capture = true;
    g_in.b_hump            = true;
    g_in.b_ramp_active     = true;
    step();
    CHECK(0 == g_out.steer);
    CHECK(VC_SPEED_HUMP_MM_S == g_out.speed_mm_s);
}

/* The capture hold clears the right ramp explicitly, so the next RIGHT
 * restarts at START. */
static void
test_barcode_capture_resets_ramp (void)
{
    printf("barcode capture clears the right ramp\n");

    go_to_follow_line(0U);
    g_in.line         = VC_LINE_RIGHT;
    g_in.line_sensors = VC_LINE_SENSOR_RIGHT;
    step();
    step();
    CHECK(68 == g_out.steer);

    /* The capture starts on CENTRE: the hold resets the right ramp. */
    g_in.line              = VC_LINE_CENTRE;
    g_in.line_sensors      = VC_LINE_SENSOR_CENTRE;
    g_in.b_barcode_capture = true;
    step();
    CHECK(0 == g_out.steer);

    g_in.b_barcode_capture = false;
    step();
    CHECK(0 == g_out.steer);
    g_in.line         = VC_LINE_RIGHT;
    g_in.line_sensors = VC_LINE_SENSOR_RIGHT;
    step();
    CHECK(VC_STEER_START == g_out.steer);
}

/* A real line loss during a barcode capture still starts recovery. */
static void
test_barcode_capture_lost (void)
{
    printf("barcode capture does not hide a lost line\n");

    go_to_follow_line(0U);
    g_in.b_barcode_capture = true;
    g_in.line = VC_LINE_LOST;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    CHECK(VC_MOTION_PIVOT == g_out.motion);
    CHECK(0 != g_out.pivot_mdeg);
}

/* Capture inactive: the ordinary follow-line steering is unchanged. */
static void
test_barcode_capture_inactive (void)
{
    printf("barcode capture inactive: steering unchanged\n");

    go_to_follow_line(0U);
    g_in.b_barcode_capture = false;
    g_in.line = VC_LINE_LEFT;
    g_in.line_error = 0;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(-VC_STEER_START == g_out.steer);
    CHECK(VC_SPEED_NORMAL_MM_S > g_out.speed_mm_s);
}

/* The ramp exception changes speed only, never line or safety decisions. */
static void
test_ramp_follow_speed (void)
{
    int16_t magnitude;
    int8_t side;

    printf("fresh active ramp: retain forward speed and steering\n");

    for (side = -1; side <= 1; side += 2)
    {
        go_to_follow_line(0U);
        g_in.b_hump = true;
        g_in.b_ramp_active = true;
        g_in.b_terrain_ready = false;   /* Active ramp is not level. */
        step();
        CHECK(VC_SPEED_HUMP_MM_S == g_out.speed_mm_s);
        CHECK(0 == g_out.steer);

        g_in.line = (0 > side) ? VC_LINE_LEFT : VC_LINE_RIGHT;
        for (magnitude = VC_STEER_START; magnitude < 127;
             magnitude += VC_STEER_STEP)
        {
            step();
            CHECK_STATE(VC_STATE_FOLLOW_LINE);
            CHECK(VC_MOTION_DRIVE == g_out.motion);
            CHECK((side * magnitude) == g_out.steer);
            CHECK(VC_SPEED_HUMP_MM_S == g_out.speed_mm_s);
        }
        step();
        CHECK((side * 127) == g_out.steer);
        CHECK(VC_SPEED_HUMP_MM_S == g_out.speed_mm_s);

        /* A stale/not-ready report keeps b_hump, but loses the exception. */
        g_in.b_ramp_active = false;
        step();
        CHECK(VC_SPEED_CURVE_MM_S == g_out.speed_mm_s);
        CHECK((side * 127) == g_out.steer);

        g_in.b_hump = false;
        g_in.b_terrain_ready = true;
        g_in.line = VC_LINE_CENTRE;
        step();
        CHECK(VC_SPEED_NORMAL_MM_S == g_out.speed_mm_s);

        /* A ramp does not mask LOST, even if it remains active. */
        g_in.b_hump = true;
        g_in.b_ramp_active = true;
        g_in.line = VC_LINE_LOST;
        step();
        CHECK_STATE(VC_STATE_REACQUIRE_LINE);
        CHECK(VC_MOTION_PIVOT == g_out.motion);
        g_in.line = VC_LINE_CENTRE;
        step();
        CHECK_STATE(VC_STATE_FOLLOW_LINE);
    }
}

static void
test_ramp_preemption (void)
{
    printf("active ramp: STOP, faults, finish, obstacle and junction win\n");

    go_to_follow_line(0U);
    g_in.b_hump = true;
    g_in.b_ramp_active = true;
    g_in.cmd = VC_CMD_STOP;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_MOTION_STOP == g_out.motion);

    go_to_follow_line(0U);
    g_in.b_hump = true;
    g_in.b_ramp_active = true;
    g_in.serious_faults = 1U;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
    CHECK(VC_MOTION_STOP == g_out.motion);

    go_to_follow_line(0U);
    g_in.b_hump = true;
    g_in.b_ramp_active = true;
    g_in.b_finish = true;
    step();
    CHECK_STATE(VC_STATE_FINISHED);
    CHECK(VC_MOTION_STOP == g_out.motion);

    go_to_follow_line(0U);
    g_in.b_hump = true;
    g_in.b_ramp_active = true;
    g_in.b_obstacle_ahead = true;
    step();
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);
    CHECK(VC_MOTION_STOP == g_out.motion);
    CHECK(VC_OBST_SCAN_REQUEST == g_out.obstacle);

    go_to_follow_line(0U);
    g_in.b_hump = true;
    g_in.b_ramp_active = true;
    g_in.barcode = VC_NAV_RIGHT;
    step();
    cross_onto_junction();
    advance_to_turn();
    CHECK_STATE(VC_STATE_TURNING);
    CHECK(VC_MOTION_TURN == g_out.motion);
    CHECK(VC_NAV_RIGHT == g_out.turn);
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

    cross_onto_junction();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);        /* Drive on to the axle first. */
    CHECK(VC_MOTION_DRIVE == g_out.motion);
    CHECK(0 == g_out.steer);

    g_in.line = VC_LINE_CENTRE;
    advance_to_turn();
    CHECK_STATE(VC_STATE_TURNING);
    CHECK(g_out.b_state_changed);
    CHECK(VC_MOTION_TURN == g_out.motion);
    CHECK(VC_NAV_LEFT == g_out.turn);

    step();
    CHECK_STATE(VC_STATE_TURNING);            /* Turn still running. */
    CHECK(VC_MOTION_NONE == g_out.motion);    /* Command sent only once. */

    g_in.b_motion_done = true;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);

    /* The stored command was used up: the next junction goes straight. */
    cross_onto_junction();
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
    cross_onto_junction();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);        /* Straight is not a turn. */
    CHECK(VC_MOTION_DRIVE == g_out.motion);
    g_in.line = VC_LINE_CENTRE;
    g_in.travel_mm += VC_JUNCTION_HOLD_MM;    /* Clear of the crossing. */
    step();

    g_in.barcode = VC_NAV_UTURN;
    step();
    cross_onto_junction();
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
    cross_onto_junction();
    g_in.line = VC_LINE_CENTRE;
    advance_to_turn();
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
    cross_onto_junction();
    g_in.line = VC_LINE_CENTRE;
    advance_to_turn();
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

    CHECK(VC_MOTION_PIVOT == g_out.motion);   /* Pivot search, no creep. */
    CHECK(-VC_SEARCH_LIMIT_MDEG == g_out.pivot_mdeg);   /* Left if unknown. */

    g_in.line = VC_LINE_RIGHT;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_OBST_ABORT != g_out.obstacle);   /* The bug fixed earlier. */

    g_in.line = VC_LINE_LOST;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    wait_ms(VC_SEARCH_TIMEOUT_MS + 40U);      /* Backstop only. */
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
    run_calibration();
    cross_onto_junction();
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
    wait_ms(VC_SEARCH_TIMEOUT_MS + 40U);
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

    /* A fault cleared, STOP then START is the other way back. */
    g_in.serious_faults = 0U;
    g_in.cmd            = VC_CMD_STOP;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_MOTION_STOP == g_out.motion);

    g_in.cmd = VC_CMD_START;
    step();
    CHECK_STATE(VC_STATE_CALIBRATE);
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
    wait_ms(VC_SEARCH_TIMEOUT_MS);
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
    cross_onto_junction();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);        /* No barcode: straight on. */

    g_in.barcode = VC_NAV_LEFT;               /* Next junction's barcode. */
    step();                                   /* Still over junction 1.   */
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(!g_out.b_state_changed);            /* Not re-triggered.        */
    CHECK(0 == g_out.steer);
    wait_ms(200U);
    CHECK_STATE(VC_STATE_FOLLOW_LINE);

    g_in.line = VC_LINE_CENTRE;               /* Leave junction 1. */
    g_in.travel_mm += VC_JUNCTION_HOLD_MM;
    step();
    cross_onto_junction();                    /* Junction 2. */
    advance_to_turn();
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
    cross_onto_junction();
    g_in.line = VC_LINE_CENTRE;
    advance_to_turn();                        /* Turn starts here. */
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
    cross_onto_junction();
    advance_to_turn();
    CHECK_STATE(VC_STATE_TURNING);            /* Barcode was kept. */
    CHECK(VC_NAV_RIGHT == g_out.turn);
}

/* Put the car in REACQUIRE_LINE after an obstacle bypass (search NONE). */
static void
go_to_bypass_reacquire (void)
{
    go_to_obstacle_scan();
    g_in.b_scan_done    = true;
    g_in.recommendation = VC_AVOID_TURN_LEFT;
    step();
    g_in.line              = VC_LINE_LOST;
    g_in.b_obstacle_passed = true;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
}

/* Lose the line after last seeing it on the given side (-1, 0, +1). */
static void
lose_line_after (int8_t side)
{
    go_to_follow_line(0U);
    if (0 != side)
    {
        g_in.line       = (0 > side) ? VC_LINE_LEFT : VC_LINE_RIGHT;
        g_in.line_error = 0;              /* The error is ignored. */
        step();
    }
    g_in.line = VC_LINE_LOST;
    step();
}

static void
test_search_after_bypass (void)
{
    printf("just after a bypass the line search pivots at most 45 degrees\n");

    go_to_bypass_reacquire();
    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    g_in.line       = VC_LINE_RIGHT;
    g_in.line_error = 0;
    step();
    g_in.line = VC_LINE_LOST;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    CHECK(VC_MOTION_PIVOT == g_out.motion);
    CHECK(VC_SEARCH_LIMIT_POST_BYPASS_MDEG == g_out.pivot_mdeg);

    /* The other side, to the same bound from where the search began. */
    g_in.cal_angle_mdeg += VC_SEARCH_LIMIT_POST_BYPASS_MDEG;
    g_in.b_motion_done   = true;
    step();
    CHECK(-(2 * VC_SEARCH_LIMIT_POST_BYPASS_MDEG) == g_out.pivot_mdeg);

    /* Once VC_POST_BYPASS_MS has passed, the full bound again. */
    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    wait_ms(VC_POST_BYPASS_MS);
    g_in.line = VC_LINE_RIGHT;
    step();
    g_in.line = VC_LINE_LOST;
    step();
    CHECK(VC_SEARCH_LIMIT_MDEG == g_out.pivot_mdeg);
}

static void
test_search_towards_last_side (void)
{
    printf("pivot search starts towards where the line was last seen\n");

    lose_line_after(-1);
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    CHECK(VC_MOTION_PIVOT == g_out.motion);
    CHECK(-VC_SEARCH_LIMIT_MDEG == g_out.pivot_mdeg);

    lose_line_after(1);
    CHECK(VC_MOTION_PIVOT == g_out.motion);
    CHECK(VC_SEARCH_LIMIT_MDEG == g_out.pivot_mdeg);

    /* A centred reading in between keeps the last side. */
    go_to_follow_line(0U);
    g_in.line       = VC_LINE_RIGHT;
    g_in.line_error = 0;
    step();
    g_in.line       = VC_LINE_CENTRE;
    g_in.line_error = 0;
    step();
    g_in.line = VC_LINE_LOST;
    step();
    CHECK(VC_SEARCH_LIMIT_MDEG == g_out.pivot_mdeg);

    /* Never seen a side: left first. */
    lose_line_after(0);
    CHECK(-VC_SEARCH_LIMIT_MDEG == g_out.pivot_mdeg);

    /* The pivot is held, not re-issued, while nothing changes. */
    step();
    CHECK(!g_out.b_state_changed);
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
}

static void
test_pivot_search_sequence (void)
{
    int32_t origin;

    printf("pivot search: first side, second side, fail\n");

    /* Found on the first side: stop and follow. */
    lose_line_after(1);
    g_in.cal_angle_mdeg += 30000;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_MOTION_STOP == g_out.motion);

    /* A right sensor reading ends the search too. */
    lose_line_after(-1);
    g_in.line = VC_LINE_RIGHT;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_MOTION_STOP == g_out.motion);

    /* First side finishes at the bound: the other side, from the start
     * heading, so the pivot is the turned angle plus the bound. */
    lose_line_after(1);
    origin = g_in.cal_angle_mdeg;             /* Heading at the loss. */
    g_in.cal_angle_mdeg = origin + VC_SEARCH_LIMIT_MDEG;
    g_in.b_motion_done  = true;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    CHECK(VC_MOTION_PIVOT == g_out.motion);
    CHECK(-(VC_SEARCH_LIMIT_MDEG + VC_SEARCH_LIMIT_MDEG) == g_out.pivot_mdeg);

    /* Found on the second side. */
    g_in.cal_angle_mdeg = origin - 60000;
    g_in.line = VC_LINE_LEFT;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_MOTION_STOP == g_out.motion);

    /* Both sides fail: emergency stop. */
    lose_line_after(-1);
    origin = g_in.cal_angle_mdeg;
    g_in.cal_angle_mdeg = origin - VC_SEARCH_LIMIT_MDEG;
    g_in.b_motion_done  = true;
    step();
    CHECK(VC_MOTION_PIVOT == g_out.motion);
    CHECK(2 * VC_SEARCH_LIMIT_MDEG == g_out.pivot_mdeg);
    g_in.cal_angle_mdeg = origin + VC_SEARCH_LIMIT_MDEG;
    g_in.b_motion_done  = true;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);

    /* A motion fault during the search. */
    lose_line_after(1);
    g_in.b_motion_failed = true;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);

    /* STOP during the search: ready, motor stopped. */
    lose_line_after(1);
    g_in.cmd = VC_CMD_STOP;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_MOTION_STOP == g_out.motion);

    /* A junction found during the search stops it and is handled. */
    go_to_follow_line(0U);
    g_in.barcode = VC_NAV_LEFT;
    step();
    g_in.line = VC_LINE_LOST;
    step();
    g_in.line = VC_LINE_JUNCTION;
    step();
    advance_to_turn();
    CHECK_STATE(VC_STATE_TURNING);
}

static void
test_bypass_reacquire_creeps (void)
{
    printf("after a bypass the line search still creeps\n");

    go_to_bypass_reacquire();
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);
    CHECK(VC_MOTION_DRIVE == g_out.motion);
    CHECK(VC_SPEED_SEARCH_MM_S == g_out.speed_mm_s);
    CHECK(0 == g_out.steer);
    wait_ms(VC_REACQUIRE_TIMEOUT_MS + 40U);
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);     /* Own, shorter timeout. */
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
    CHECK(VC_OBST_ABORT != g_out.obstacle);   /* Buddy 5 not involved. */
    advance_to_turn();
    CHECK_STATE(VC_STATE_TURNING);
    CHECK(VC_MOTION_TURN == g_out.motion);
    CHECK(VC_NAV_LEFT == g_out.turn);

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
    CHECK(VC_OBST_ABORT == g_out.obstacle);
    advance_to_turn();
    CHECK_STATE(VC_STATE_TURNING);
    CHECK(VC_NAV_RIGHT == g_out.turn);
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

static void
test_calibration (void)
{
    uint32_t elapsed;

    printf("START calibration\n");

    /* START: still first, no pivot yet, Buddy 3 and 4 told to begin. */
    reset(0U);
    g_in.b_init_ok = true;
    step();
    g_in.cmd = VC_CMD_START;
    step();
    CHECK_STATE(VC_STATE_CALIBRATE);
    CHECK(VC_CAL_REQ_BEGIN == g_out.cal_req);
    CHECK((VC_MOTION_PIVOT != g_out.motion)
          && (VC_MOTION_DRIVE != g_out.motion));    /* Standing still. */
    wait_ms(VC_CAL_STILL_MS - (3U * VC_PERIOD_MS));
    CHECK(VC_MOTION_PIVOT != g_out.motion);   /* Not still long enough.  */
    wait_ms(3U * VC_PERIOD_MS);
    CHECK(VC_MOTION_PIVOT == g_out.motion);
    CHECK(-VC_CAL_LIMIT_MDEG == g_out.pivot_mdeg);      /* Left, limit.  */
    CHECK(VC_CAL_REQ_SWEEP == g_out.cal_req);

    /* The sweep stops on what the sensors saw, not on an angle. */
    g_in.cal_b3         = VC_CAL_B3_OK;
    g_in.cal_b4         = VC_CAL_B4_ENOUGH;
    g_in.cal_angle_mdeg = -35000;
    g_in.line           = VC_LINE_LOST;
    step();
    CHECK_STATE(VC_STATE_CALIBRATE);
    CHECK(VC_MOTION_PIVOT == g_out.motion);
    CHECK((35000 + VC_CAL_BACK_MARGIN_MDEG) == g_out.pivot_mdeg);

    /* BACK stops on CENTRE even with a line error. */
    step();
    CHECK(VC_MOTION_STOP != g_out.motion);    /* Still searching.        */
    g_in.line       = VC_LINE_CENTRE;
    g_in.line_error = 40;
    step();
    CHECK(VC_MOTION_STOP == g_out.motion);
    CHECK(VC_CAL_REQ_FINISH == g_out.cal_req);
    g_in.cal_b4 = VC_CAL_B4_DONE;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);

    /* BACK also stops on JUNCTION (right sensor crossing first). */
    go_to_way_back();
    g_in.line = VC_LINE_JUNCTION;
    step();
    CHECK(VC_MOTION_STOP == g_out.motion);
    CHECK(VC_CAL_REQ_FINISH == g_out.cal_req);

    /* The turn limit on the way back without the line fails. */
    go_to_way_back();
    g_in.b_motion_done = true;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_CAL_FAIL_LINE == g_out.cal_fail);

    /* STOP during the sweep, and during the way back. */
    go_to_sweep();
    g_in.cmd = VC_CMD_STOP;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_MOTION_STOP == g_out.motion);
    CHECK(VC_CAL_REQ_ABORT == g_out.cal_req);
    CHECK(VC_CAL_FAIL_NONE == g_out.cal_fail);

    go_to_way_back();
    g_in.cmd = VC_CMD_STOP;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_MOTION_STOP == g_out.motion);
    CHECK(VC_CAL_REQ_ABORT == g_out.cal_req);

    /* Line sensor contrast too low: back in READY, stopped, with reason. */
    go_to_judge();
    CHECK_STATE(VC_STATE_CALIBRATE);
    CHECK(VC_MOTION_STOP == g_out.motion);
    CHECK(VC_CAL_REQ_JUDGE == g_out.cal_req);
    g_in.cal_b3 = VC_CAL_B3_FAIL_CONTRAST;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_MOTION_STOP == g_out.motion);
    CHECK(VC_CAL_FAIL_CONTRAST == g_out.cal_fail);
    CHECK(VC_CAL_REQ_ABORT == g_out.cal_req);

    /* Line never found: Buddy 3 says so, or says nothing. */
    go_to_judge();
    g_in.cal_b3 = VC_CAL_B3_FAIL_LINE;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_CAL_FAIL_LINE == g_out.cal_fail);

    go_to_judge();
    for (elapsed = 0U; (elapsed <= VC_CAL_VERDICT_MS)
                       && (VC_STATE_CALIBRATE == g_out.state);
         elapsed += VC_PERIOD_MS)
    {
        step();
    }
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_CAL_FAIL_LINE == g_out.cal_fail);

    /* Buddy 4 never ready, never enough samples, or no valid floor. */
    reset(0U);
    g_in.b_init_ok = true;
    step();
    g_in.cmd = VC_CMD_START;
    step();
    CHECK_STATE(VC_STATE_CALIBRATE);
    g_in.b_terrain_ready = false;
    for (elapsed = 0U; (elapsed < VC_CAL_STILL_TIMEOUT_MS)
                       && (VC_STATE_CALIBRATE == g_out.state);
         elapsed += VC_PERIOD_MS)
    {
        step();
    }
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_CAL_FAIL_BUDDY4 == g_out.cal_fail);

    go_to_judge();
    g_in.cal_b3 = VC_CAL_B3_OK;                         /* B4 not enough. */
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_CAL_FAIL_BUDDY4 == g_out.cal_fail);

    go_to_way_back();
    g_in.line = VC_LINE_CENTRE;
    step();
    g_in.cal_b4 = VC_CAL_B4_FAILED;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_CAL_FAIL_BUDDY4 == g_out.cal_fail);

    /* A Buddy 2 failure during the pivot is an emergency stop. */
    go_to_sweep();
    g_in.b_motion_failed = true;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);
}

/* A white gap between the two line sensors fails the calibration on
 * whichever side and phase Buddy 3 reports it. */
static void
test_calibration_gap (void)
{
    /* The angle before the sweep starts is stale (origin not set yet): it
     * must not show up as the furthest turned. */
    reset(0U);
    g_in.b_init_ok = true;
    step();
    g_in.cmd = VC_CMD_START;
    step();
    g_in.cal_angle_mdeg = 300000;
    step();
    CHECK_STATE(VC_STATE_CALIBRATE);
    CHECK(0 == g_vc.cal_far_mdeg);

    /* First side. */
    go_to_sweep_with_line(VC_LINE_RIGHT);
    g_in.cal_angle_mdeg = 20000;
    g_in.cal_b3         = VC_CAL_B3_FAIL_GAP;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_MOTION_STOP == g_out.motion);
    CHECK(VC_CAL_FAIL_GAP == g_out.cal_fail);
    CHECK(VC_CAL_REQ_ABORT == g_out.cal_req);

    /* Other side, after the first reached its bound. */
    go_to_sweep();
    g_in.cal_angle_mdeg = -VC_CAL_LIMIT_MDEG;
    step();
    CHECK_STATE(VC_STATE_CALIBRATE);
    g_in.cal_angle_mdeg = -VC_CAL_LIMIT_MDEG - 20000;
    g_in.cal_b3         = VC_CAL_B3_FAIL_GAP;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_CAL_FAIL_GAP == g_out.cal_fail);

    /* While waiting for the verdict. */
    go_to_judge();
    g_in.cal_b3 = VC_CAL_B3_FAIL_GAP;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_CAL_FAIL_GAP == g_out.cal_fail);

    /* A gap is not reported when Buddy 3 says OK. */
    go_to_way_back();
    CHECK_STATE(VC_STATE_CALIBRATE);
    CHECK(VC_CAL_FAIL_NONE == g_out.cal_fail);
}


static void
test_calibration_sides (void)
{
    /* First way follows the right sensor: black there means right first. */
    go_to_sweep_with_line(VC_LINE_RIGHT);
    CHECK(VC_MOTION_PIVOT == g_out.motion);
    CHECK(VC_CAL_LIMIT_MDEG == g_out.pivot_mdeg);
    CHECK(1 == g_vc.cal_first_dir);

    go_to_sweep_with_line(VC_LINE_JUNCTION);
    CHECK(VC_CAL_LIMIT_MDEG == g_out.pivot_mdeg);

    go_to_sweep_with_line(VC_LINE_CENTRE);
    CHECK(-VC_CAL_LIMIT_MDEG == g_out.pivot_mdeg);
    CHECK(-1 == g_vc.cal_first_dir);

    go_to_sweep_with_line(VC_LINE_LEFT);
    CHECK(-VC_CAL_LIMIT_MDEG == g_out.pivot_mdeg);

    go_to_sweep_with_line(VC_LINE_LOST);
    CHECK(-VC_CAL_LIMIT_MDEG == g_out.pivot_mdeg);

    /* Right first, done on that side: back left, turned angle + margin. */
    go_to_sweep_with_line(VC_LINE_RIGHT);
    g_in.cal_b3         = VC_CAL_B3_OK;
    g_in.cal_b4         = VC_CAL_B4_ENOUGH;
    g_in.cal_angle_mdeg = 12000;
    g_in.line           = VC_LINE_LOST;
    step();
    CHECK(-(12000 + VC_CAL_BACK_MARGIN_MDEG) == g_out.pivot_mdeg);
    g_in.cal_angle_mdeg = -5000;              /* Back past the start by   */
    step();                                   /* less than the margin.    */
    CHECK_STATE(VC_STATE_CALIBRATE);
    g_in.cal_angle_mdeg = -VC_CAL_BACK_MARGIN_MDEG;     /* By the margin. */
    step();
    CHECK_STATE(VC_STATE_READY);              /* Line not found.          */
    CHECK(VC_CAL_FAIL_LINE == g_out.cal_fail);

    /* Left first, nothing learned at the bound: the other side, not a
     * verdict.  Buddy 3 and 4 are not restarted. */
    go_to_sweep();
    g_in.cal_angle_mdeg = -VC_CAL_LIMIT_MDEG;
    step();
    CHECK_STATE(VC_STATE_CALIBRATE);
    CHECK(VC_MOTION_PIVOT == g_out.motion);
    CHECK((VC_CAL_LIMIT_MDEG + VC_CAL_LIMIT_MDEG) == g_out.pivot_mdeg);
    CHECK(VC_CAL_REQ_NONE == g_out.cal_req);
    CHECK(VC_CAL_FAIL_NONE == g_out.cal_fail);

    /* Success on the second side: back the way it came (left). */
    g_in.cal_angle_mdeg = 15000;
    g_in.cal_b3         = VC_CAL_B3_OK;
    g_in.cal_b4         = VC_CAL_B4_ENOUGH;
    g_in.line           = VC_LINE_LOST;
    step();
    CHECK(-(15000 + VC_CAL_BACK_MARGIN_MDEG) == g_out.pivot_mdeg);
    g_in.line = VC_LINE_CENTRE;
    step();
    CHECK(VC_CAL_REQ_FINISH == g_out.cal_req);
    CHECK(15000 == g_vc.cal_out_mdeg);
    g_in.cal_b4 = VC_CAL_B4_DONE;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);

    /* The mirror image: right first, then success on the left. */
    go_to_sweep_with_line(VC_LINE_RIGHT);
    g_in.cal_angle_mdeg = VC_CAL_LIMIT_MDEG;
    step();
    CHECK(-(VC_CAL_LIMIT_MDEG + VC_CAL_LIMIT_MDEG) == g_out.pivot_mdeg);
    CHECK(VC_CAL_REQ_NONE == g_out.cal_req);
    g_in.cal_angle_mdeg = -15000;
    g_in.cal_b3         = VC_CAL_B3_OK;
    g_in.cal_b4         = VC_CAL_B4_ENOUGH;
    g_in.line           = VC_LINE_LOST;
    step();
    CHECK((15000 + VC_CAL_BACK_MARGIN_MDEG) == g_out.pivot_mdeg);
    g_in.line = VC_LINE_JUNCTION;
    step();
    CHECK(VC_CAL_REQ_FINISH == g_out.cal_req);

    /* Failure only after both sides, with the furthest angle kept. */
    go_to_sweep();
    g_in.cal_angle_mdeg = -VC_CAL_LIMIT_MDEG;
    step();                                   /* First side: bound.       */
    g_in.cal_angle_mdeg = VC_CAL_LIMIT_MDEG - 1000;
    step();
    CHECK_STATE(VC_STATE_CALIBRATE);          /* Not at the bound yet.    */
    CHECK(VC_CAL_REQ_NONE == g_out.cal_req);
    g_in.cal_angle_mdeg = VC_CAL_LIMIT_MDEG;
    step();                                   /* Other side: bound.       */
    CHECK_STATE(VC_STATE_CALIBRATE);
    CHECK(VC_MOTION_STOP == g_out.motion);
    CHECK(VC_CAL_REQ_JUDGE == g_out.cal_req);
    g_in.cal_b3 = VC_CAL_B3_FAIL_CONTRAST;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_CAL_FAIL_CONTRAST == g_out.cal_fail);
    CHECK(VC_CAL_LIMIT_MDEG == g_vc.cal_far_mdeg);
    CHECK(-1 == g_vc.cal_first_dir);
}

/* ------------------------------------------------------------------------ */

/* JUNCTION with nothing stored, then the car at travel_mm = 0. */
static void
cross_junction (void)
{
    go_to_follow_line(0U);
    g_in.travel_mm = 1000;                    /* Running total: any origin. */
    step();
    cross_onto_junction();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
}

static void
test_junction_hold_straight (void)
{
    printf("junction straight: release neutral on centre loss\n");

    cross_junction();
    g_in.line = VC_LINE_RIGHT;                /* Crossing still covers centre. */
    g_in.travel_mm = 1000 + VC_JUNCTION_CONFIRM_MM;
    step();
    CHECK(0 == g_out.steer);
    CHECK(g_vc.b_junction_hold);
    CHECK(!g_vc.b_junction_track);

    g_in.line_sensors = 2U;                   /* Right-only: track immediately. */
    step();
    CHECK(64 == g_out.steer);
    CHECK(g_vc.b_junction_hold);              /* Still suppress duplicate events. */
    CHECK(g_vc.b_junction_track);

    g_in.line_sensors = 3U;                   /* Centre return cannot re-arm. */
    step();
    CHECK(68 == g_out.steer);
    g_in.line_sensors = 0U;                   /* Both white before LOST. */
    step();
    CHECK(59 == g_out.steer);                 /* Right fades: 68 * 7 / 8. */
    g_in.line = VC_LINE_LOST;                 /* LOST is no longer suppressed. */
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);

    cross_junction();
    g_in.line = VC_LINE_LOST;                 /* Stale status clears the bits. */
    g_in.line_sensors = 0U;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);

    cross_junction();
    g_in.line = VC_LINE_RIGHT;
    g_in.cal_b3 = VC_CAL_B3_IDLE;             /* No calibrated centre evidence. */
    step();
    CHECK(g_vc.b_junction_track);
    CHECK(64 == g_out.steer);

    cross_junction();
    g_in.line = VC_LINE_RIGHT;
    g_in.line_sensors = 3U;
    g_in.travel_mm = 1000 + VC_JUNCTION_HOLD_MM - 1;
    step();
    CHECK(0 == g_out.steer);                  /* Genuine crossing remains neutral. */
    g_in.travel_mm++;
    step();
    CHECK(64 == g_out.steer);                 /* Distance window still expires. */

}

static void
test_right_white_fades (void)
{
    int32_t n;
    int32_t previous;

    printf("right last seen, sensors white: fade, never grow\n");

    go_to_follow_line(0U);
    g_in.line         = VC_LINE_RIGHT;
    g_in.line_sensors = VC_LINE_SENSOR_RIGHT;
    step();
    CHECK(VC_STEER_START == g_out.steer);
    step();
    CHECK((VC_STEER_START + VC_STEER_STEP) == g_out.steer);

    g_in.line_sensors = 0U;
    previous = g_out.steer;
    for (n = 0; 10 > n; n++)
    {
        step();
        CHECK(g_out.steer <= previous);
        CHECK(0 <= g_out.steer);
        previous = g_out.steer;
    }
    CHECK(VC_STEER_START > g_out.steer);

    /* Right black again with the ramp below START: restart there. */
    g_in.line_sensors = VC_LINE_SENSOR_RIGHT;
    step();
    CHECK(VC_STEER_START == g_out.steer);

    /* Right black again before it falls below START: carry on growing.
     * 76 fades to 66, which is still above START, so it resumes at 70. */
    step();
    step();
    step();
    CHECK(76 == g_out.steer);
    g_in.line_sensors = 0U;
    step();
    CHECK(66 == g_out.steer);
    g_in.line_sensors = VC_LINE_SENSOR_RIGHT;
    step();
    CHECK(70 == g_out.steer);

    /* LEFT has no sensor, so bits clear is the normal correction: grows. */
    go_to_follow_line(0U);
    g_in.line         = VC_LINE_LEFT;
    g_in.line_sensors = 0U;
    step();
    CHECK(-VC_STEER_START == g_out.steer);
    step();
    CHECK(-(VC_STEER_START + VC_STEER_STEP) == g_out.steer);

    /* RIGHT with bits clear entered from CENTRE: the line is approaching
     * the right sensor, so the normal ramp. */
    go_to_follow_line(0U);
    g_in.line         = VC_LINE_CENTRE;
    g_in.line_sensors = VC_LINE_SENSOR_CENTRE;
    step();
    g_in.line         = VC_LINE_RIGHT;
    g_in.line_sensors = 0U;
    step();
    CHECK(VC_STEER_START == g_out.steer);
    step();
    CHECK((VC_STEER_START + VC_STEER_STEP) == g_out.steer);

    /* Entered from a LEFT ramp: restart at +START, no carried magnitude. */
    go_to_follow_line(0U);
    g_in.line         = VC_LINE_LEFT;
    g_in.line_sensors = 0U;
    for (n = 0; 10 > n; n++)
    {
        step();
    }
    CHECK(-(VC_STEER_START + (9 * VC_STEER_STEP)) == g_out.steer);
    g_in.line = VC_LINE_RIGHT;
    step();
    CHECK(VC_STEER_START == g_out.steer);

    /* A CENTRE step clears the flag: bits clear after it use the ramp. */
    go_to_follow_line(0U);
    g_in.line         = VC_LINE_RIGHT;
    g_in.line_sensors = VC_LINE_SENSOR_RIGHT;
    step();
    step();
    g_in.line_sensors = 0U;
    step();
    CHECK(59 == g_out.steer);                 /* Faded: 68 * 7 / 8. */
    g_in.line         = VC_LINE_CENTRE;
    step();
    g_in.line         = VC_LINE_RIGHT;
    step();
    CHECK(VC_STEER_START == g_out.steer);     /* Ramp path, not a fade. */
    step();
    CHECK((VC_STEER_START + VC_STEER_STEP) == g_out.steer);
}

static void
test_junction_hold_barcode_turn (void)
{
    printf("junction hold: barcode turn unchanged\n");

    go_to_follow_line(0U);
    g_in.barcode = VC_NAV_RIGHT;
    step();
    cross_onto_junction();
    advance_to_turn();
    CHECK_STATE(VC_STATE_TURNING);
    CHECK(VC_NAV_RIGHT == g_out.turn);

    g_in.b_motion_done = true;
    g_in.line          = VC_LINE_RIGHT;
    g_in.line_error    = 90;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    step();
    CHECK(64 == g_out.steer);                 /* No hold after a turn. */
}

static void
test_junction_advance (void)
{
    printf("junction turn: drive %d mm to the axle first\n",
           VC_JUNCTION_ADVANCE_MM);

    go_to_follow_line(0U);
    g_in.travel_mm = 1000;
    g_in.barcode   = VC_NAV_LEFT;
    step();
    cross_onto_junction();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);

    /* Past the straight-across hold: still straight, LOST ignored. */
    g_in.line      = VC_LINE_LOST;
    g_in.travel_mm = 1000 + VC_JUNCTION_HOLD_MM;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_MOTION_DRIVE == g_out.motion);
    CHECK(0 == g_out.steer);

    g_in.line      = VC_LINE_RIGHT;
    g_in.travel_mm = 1000 + VC_JUNCTION_ADVANCE_MM - 1;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);        /* Not there yet. */
    CHECK(0 == g_out.steer);

    g_in.travel_mm = 1000 + VC_JUNCTION_ADVANCE_MM;
    step();
    CHECK_STATE(VC_STATE_TURNING);
    CHECK(VC_MOTION_TURN == g_out.motion);
    CHECK(VC_NAV_LEFT == g_out.turn);

    /* An obstacle while advancing wins, and the turn is dropped. */
    go_to_follow_line(0U);
    g_in.barcode = VC_NAV_RIGHT;
    step();
    cross_onto_junction();
    g_in.line             = VC_LINE_CENTRE;
    g_in.b_obstacle_ahead = true;
    step();
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);
    CHECK(VC_NAV_NONE == g_vc.pending_turn);
}

static void
test_junction_confirm (void)
{
    printf("junction: only after %d mm of both-black\n",
           VC_JUNCTION_CONFIRM_MM);

    /* A moment of both-black on a curve: no turn, barcode kept. */
    go_to_follow_line(0U);
    g_in.travel_mm = 1000;
    g_in.barcode   = VC_NAV_RIGHT;
    step();
    g_in.line = VC_LINE_JUNCTION;
    step();
    g_in.travel_mm = 1000 + VC_JUNCTION_CONFIRM_MM - 1;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(!g_out.b_state_changed);            /* Not accepted yet. */
    CHECK(0 == g_out.steer);                  /* Straight while over it. */
    g_in.line = VC_LINE_LEFT;                 /* Curve goes on to the left. */
    g_in.travel_mm = 1000 + VC_JUNCTION_CONFIRM_MM + 50;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_NAV_NONE == g_vc.pending_turn);
    CHECK(VC_NAV_RIGHT == g_vc.stored_nav);   /* Kept for the real one. */
    CHECK(0 > g_out.steer);                   /* Steering along the curve. */

    /* The real junction: long enough, turn measured from where it began. */
    g_in.line = VC_LINE_JUNCTION;
    g_in.travel_mm = 2000;
    step();
    g_in.travel_mm = 2000 + VC_JUNCTION_CONFIRM_MM;
    step();
    CHECK(VC_NAV_RIGHT == g_vc.pending_turn);
    CHECK(2000 == g_vc.junction_origin_mm);
    g_in.line = VC_LINE_CENTRE;
    g_in.travel_mm = 2000 + VC_JUNCTION_ADVANCE_MM;
    step();
    CHECK_STATE(VC_STATE_TURNING);
    CHECK(VC_NAV_RIGHT == g_out.turn);
}

static void
test_junction_hold_second_junction (void)
{
    printf("junction hold: second JUNCTION edge ignored\n");

    cross_junction();
    g_in.line      = VC_LINE_CENTRE;
    step();
    g_in.barcode   = VC_NAV_LEFT;             /* Kept for a later junction. */
    g_in.line      = VC_LINE_JUNCTION;
    g_in.travel_mm = 1010;
    step();
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(!g_out.b_state_changed);
    CHECK(0 == g_out.steer);

    g_in.line       = VC_LINE_RIGHT;          /* Hold still from the first. */
    g_in.line_error = 50;
    g_in.travel_mm  = 1000 + VC_JUNCTION_HOLD_MM - 1;
    step();
    CHECK(0 == g_out.steer);
    g_in.travel_mm  = 1000 + VC_JUNCTION_HOLD_MM;
    step();
    CHECK(64 == g_out.steer);

    cross_onto_junction();                    /* A new junction: acted on. */
    advance_to_turn();
    CHECK_STATE(VC_STATE_TURNING);
    CHECK(VC_NAV_LEFT == g_out.turn);
}

static void
test_junction_hold_preempted (void)
{
    printf("junction hold: STOP, emergency, obstacle still pre-empt\n");

    cross_junction();
    g_in.cmd = VC_CMD_STOP;
    step();
    CHECK_STATE(VC_STATE_READY);
    CHECK(VC_MOTION_STOP == g_out.motion);

    cross_junction();
    g_in.serious_faults = 1U;
    step();
    CHECK_STATE(VC_STATE_EMERGENCY_STOP);

    cross_junction();
    g_in.b_obstacle_ahead = true;
    step();
    CHECK_STATE(VC_STATE_OBSTACLE_SCAN);
}

/* Both line sensors read black: the lift rule's input. */
static void
set_both_black (void)
{
    g_in.line         = VC_LINE_JUNCTION;
    g_in.line_sensors = (uint8_t) (VC_LINE_SENSOR_CENTRE
                                   | VC_LINE_SENSOR_RIGHT);
}

/* Hold both sensors on black for a number of VC cycles. */
static void
hold_both_black (uint32_t cycles)
{
    uint32_t i;

    for (i = 0U; i < cycles; i++)
    {
        step();
    }
}

/* One normal crossing of a given length, then back on the centre sensor,
 * so the lift rule learns the length. */
static void
learn_normal_spell (uint32_t cycles)
{
    set_both_black();
    hold_both_black(cycles);
    g_in.line         = VC_LINE_CENTRE;
    g_in.line_sensors = VC_LINE_SENSOR_CENTRE;
    step();
}

/* A lift on a straight run: the long both-black spell with the IMU cue
 * finishes the run, really stops Motion, and a new START works. */
static void
test_lift_confirmed (void)
{
    printf("lift: long both-black spell with IMU cue -> FINISHED\n");

    go_to_follow_line(0U);
    learn_normal_spell(5U);                   /* Learned: 5 cycles.       */
    CHECK(5U == g_vc.lift_spell_max_cycles);

    set_both_black();
    g_in.b_imu_cue = true;
    hold_both_black(5U);                      /* Equal: not longer.       */
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(!g_out.b_lift_confirmed);

    step();                                   /* Spell 6 > 5: confirm.    */
    CHECK_STATE(VC_STATE_FINISHED);
    CHECK(VC_MOTION_STOP == g_out.motion);
    CHECK(g_out.b_lift_confirmed);
    CHECK(6U == g_vc.lift_spell_cycles);       /* For the event log.       */

    /* Stays stopped, whatever the line and cue say now. */
    hold_both_black(20U);
    CHECK_STATE(VC_STATE_FINISHED);
    CHECK(VC_MOTION_NONE == g_out.motion);     /* No new Motion request.   */

    /* A new START from FINISHED starts another run. */
    g_in.b_imu_cue   = false;
    g_in.line        = VC_LINE_CENTRE;
    g_in.line_sensors = VC_LINE_SENSOR_CENTRE;
    g_in.cmd         = VC_CMD_START;
    step();
    CHECK_STATE(VC_STATE_CALIBRATE);
    CHECK(VC_CAL_REQ_BEGIN == g_out.cal_req);
}

/* A long both-black spell with no IMU cue is a junction, not a lift, and it
 * teaches only up to the safety cap. */
static void
test_lift_without_cue (void)
{
    printf("lift: long both-black spell without cue -> no lift\n");

    go_to_follow_line(0U);
    set_both_black();
    g_in.b_imu_cue = false;
    hold_both_black(60U);
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(!g_out.b_lift_confirmed);

    g_in.line         = VC_LINE_CENTRE;
    g_in.line_sensors = VC_LINE_SENSOR_CENTRE;
    step();
    CHECK(VC_LIFT_SPELL_CAP_CYCLES == g_vc.lift_spell_max_cycles);
}

/* A short both-black spell with the cue is not a lift, even after a longer
 * normal spell has been learned. */
static void
test_lift_short_with_cue (void)
{
    printf("lift: short both-black spell with cue -> no lift\n");

    go_to_follow_line(0U);
    learn_normal_spell(8U);

    set_both_black();
    g_in.b_imu_cue = true;
    hold_both_black(8U);                      /* Equal to the learned max. */
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(!g_out.b_lift_confirmed);

    g_in.b_imu_cue    = false;
    g_in.line         = VC_LINE_CENTRE;
    g_in.line_sensors = VC_LINE_SENSOR_CENTRE;
    step();
    CHECK(8U == g_vc.lift_spell_max_cycles);
}

/* Before any normal spell has ended, the conservative fallback bound
 * applies: exactly the bound is not enough, one more cycle is. */
static void
test_lift_fallback (void)
{
    printf("lift: fallback bound before learning\n");

    go_to_follow_line(0U);
    set_both_black();
    g_in.b_imu_cue = true;
    hold_both_black(VC_LIFT_SPELL_FALLBACK_CYCLES);
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(!g_out.b_lift_confirmed);

    step();
    CHECK_STATE(VC_STATE_FINISHED);
    CHECK(g_out.b_lift_confirmed);
}

/* The calibration sweep crosses the line with both sensors black on
 * purpose, so the lift rule must not count it. */
static void
test_lift_not_in_calibration (void)
{
    printf("lift: calibration sweep is not a lift\n");

    go_to_sweep();
    set_both_black();
    g_in.b_imu_cue = true;
    hold_both_black(2U * VC_LIFT_SPELL_CAP_CYCLES);
    CHECK_STATE(VC_STATE_CALIBRATE);
    CHECK(!g_out.b_lift_confirmed);
}

/* A lift outranks the junction and its stored barcode turn, and the ramp
 * rule, even on the step where the turn would start. */
static void
test_lift_priority (void)
{
    uint32_t i;

    printf("lift: beats junction, barcode turn and ramp\n");

    go_to_follow_line(0U);
    learn_normal_spell(5U);
    g_in.barcode = VC_NAV_LEFT;
    step();

    g_in.travel_mm  = 1000;
    g_in.b_ramp_active = true;
    g_in.b_hump     = true;
    set_both_black();
    g_in.b_imu_cue  = true;

    for (i = 0U; i < 5U; i++)
    {
        g_in.travel_mm += 1;
        step();
    }

    /* The junction was accepted, so the stored turn is pending. */
    CHECK_STATE(VC_STATE_FOLLOW_LINE);
    CHECK(VC_NAV_LEFT == g_vc.pending_turn);

    g_in.travel_mm = 1000 + VC_JUNCTION_ADVANCE_MM;
    step();                                   /* Spell 6: both would fire. */
    CHECK_STATE(VC_STATE_FINISHED);
    CHECK(VC_MOTION_STOP == g_out.motion);
    CHECK(g_out.b_lift_confirmed);
}

/* A long both-black spell that is found while recovering the line also
 * finishes the run, before the junction is handled. */
static void
test_lift_during_recovery (void)
{
    printf("lift: beats line recovery\n");

    go_to_follow_line(0U);
    g_in.line         = VC_LINE_LOST;
    g_in.line_sensors = 0U;
    step();
    CHECK_STATE(VC_STATE_REACQUIRE_LINE);

    set_both_black();
    g_in.b_imu_cue = true;
    hold_both_black(VC_LIFT_SPELL_FALLBACK_CYCLES + 1U);
    CHECK_STATE(VC_STATE_FINISHED);
    CHECK(g_out.b_lift_confirmed);
}

/* ------------------------------------------------------------------------ */

int
main (void)
{
    test_start_up();
    test_follow_line_and_hump();
    test_barcode_capture_hold();
    test_barcode_capture_resets_ramp();
    test_barcode_capture_lost();
    test_barcode_capture_inactive();
    test_ramp_follow_speed();
    test_ramp_preemption();
    test_barcode_turn();
    test_junction_straight_and_uturn();
    test_turn_failures();
    test_line_lost_and_found();
    test_obstacle_continue();
    test_obstacle_bypass();
    test_obstacle_retry_and_stop();
    test_arbitration();
    test_calibration();
    test_calibration_sides();
    test_calibration_gap();
    test_stop_command();
    test_emergency_recovery();
    test_finished();
    test_lift_confirmed();
    test_lift_without_cue();
    test_lift_short_with_cue();
    test_lift_fallback();
    test_lift_not_in_calibration();
    test_lift_priority();
    test_lift_during_recovery();
    test_timer_wraparound();
    test_names_and_null();
    test_junction_edge();
    test_timeout_vs_success();
    test_init_timeout();
    test_obstacle_while_reacquiring();
    test_reacquire_on_junction();
    test_search_towards_last_side();
    test_search_after_bypass();
    test_pivot_search_sequence();
    test_bypass_reacquire_creeps();
    test_invalid_inputs();
    test_junction_hold_straight();
    test_right_white_fades();
    test_junction_hold_barcode_turn();
    test_junction_advance();
    test_junction_confirm();
    test_junction_hold_second_junction();
    test_junction_hold_preempted();

    printf("\n%u checks, %u failed\n", (unsigned) g_checks,
           (unsigned) g_failures);

    return (0U == g_failures) ? 0 : 1;
}

/*** end of file ***/
