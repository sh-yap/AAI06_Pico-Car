/** @file test_vc_pack.c
 *
 * @brief Host unit tests for the packed VC state and bypass plan
 *        (vc_pack.c), CR1 phase 5c.
 *
 * The controller state record must be a lossless store for every value the
 * code can produce: the VC task unpacks it, runs vc_step(), and packs it
 * back every 20 ms, so a width that clamps a reachable value would change
 * the car's behaviour.  These tests pin the record size and layout, check
 * the field ends, and round-trip a real vc_t captured mid-run.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "vc.h"
#include "obstacle_types.h"
#include "obstacle_plan.h"
#include "vc_pack.h"

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

/* Compare two controller states field by field; one check per field. */
static void
check_vc_same (vc_t const * a, vc_t const * b, int line)
{
#define VC_FIELD(field)                                                     \
    do                                                                      \
    {                                                                       \
        g_checks++;                                                         \
        if (a->field != b->field)                                           \
        {                                                                   \
            g_failures++;                                                   \
            printf("  FAIL %s:%d: vc_t.%s differs\n", __FILE__, line,       \
                   #field);                                                 \
        }                                                                   \
    } while (0)

    VC_FIELD(state);
    VC_FIELD(entry_ms);
    VC_FIELD(stored_nav);
    VC_FIELD(rescans);
    VC_FIELD(b_obstacle_active);
    VC_FIELD(b_reversing);
    VC_FIELD(prev_line);
    VC_FIELD(last_side);
    VC_FIELD(search);
    VC_FIELD(search_dir);
    VC_FIELD(search_origin_mdeg);
    VC_FIELD(search_limit_mdeg);
    VC_FIELD(b_after_bypass);
    VC_FIELD(bypass_end_ms);
    VC_FIELD(cal_phase);
    VC_FIELD(cal_phase_ms);
    VC_FIELD(cal_out_mdeg);
    VC_FIELD(cal_far_mdeg);
    VC_FIELD(cal_first_dir);
    VC_FIELD(cal_dir);
    VC_FIELD(b_junction_hold);
    VC_FIELD(b_junction_track);
    VC_FIELD(junction_origin_mm);
    VC_FIELD(b_junction_seen);
    VC_FIELD(junction_seen_mm);
    VC_FIELD(pending_turn);
    VC_FIELD(steer_side);
    VC_FIELD(b_right_exit);
    VC_FIELD(steer_mag);
    VC_FIELD(lift_spell_cycles);
    VC_FIELD(lift_spell_max_cycles);
    VC_FIELD(b_lift_spell_cued);

#undef VC_FIELD
}

/* Pack, unpack and compare; true when both steps succeed and match. */
static bool
round_trip_ok (vc_t const * p_vc)
{
    vc_packed_t packed;
    vc_t        back;
    bool        b_ok;

    memset(&packed, 0, sizeof(packed));
    memset(&back, 0, sizeof(back));
    vc_pack_state(p_vc, &packed);
    b_ok = vc_unpack_state(&packed, &back);

    if (b_ok)
    {
        check_vc_same(p_vc, &back, __LINE__);
    }
    else
    {
        g_checks++;
        g_failures++;
        printf("  FAIL %s:%d: round trip rejected\n", __FILE__, __LINE__);
    }

    /* Packing the decoded copy again must reproduce the record. */
    {
        vc_packed_t again;

        memset(&again, 0, sizeof(again));
        vc_pack_state(&back, &again);
        g_checks++;
        if (0 != memcmp(&packed, &again, sizeof(packed)))
        {
            g_failures++;
            printf("  FAIL %s:%d: repacked record differs\n", __FILE__,
                   __LINE__);
        }
    }

    return (b_ok);
}

/* Write one field into a packed word. */
static void
set_field (uint32_t * p_word, uint32_t shift, uint32_t bits, uint32_t value)
{
    uint32_t mask = ((1U << bits) - 1U) << shift;

    *p_word = (*p_word & ~mask) | ((value << shift) & mask);
}

/* ------------------------------------------------------------------------ *
 * Tests
 * ------------------------------------------------------------------------ */

static void
test_sizes (void)
{
    printf("record sizes\n");

    CHECK(VC_STATE_PACKED_WORDS == 10U);
    CHECK(VC_PLAN_PACKED_WORDS == 3U);
    CHECK(sizeof(vc_packed_t) == 40U);
    CHECK(sizeof(vc_plan_packed_t) == 12U);
}

/* A zeroed state (every field at its minimum) round-trips to ten zero
 * words: no field shares a bit with the reserved masks. */
static void
test_zero (void)
{
    vc_t        zero;
    vc_packed_t packed;
    vc_t        back;
    uint32_t    word;
    bool        b_all_zero = true;

    printf("zero record\n");

    memset(&zero, 0, sizeof(zero));
    memset(&packed, 0xFF, sizeof(packed));
    memset(&back, 0xFF, sizeof(back));
    vc_pack_state(&zero, &packed);

    for (word = 0U; word < VC_STATE_PACKED_WORDS; word++)
    {
        if (0U != packed.word[word])
        {
            b_all_zero = false;
        }
    }
    CHECK(b_all_zero);

    CHECK(vc_unpack_state(&packed, &back));
    check_vc_same(&zero, &back, __LINE__);
    CHECK(!vc_unpack_state(NULL, &back));
    CHECK(!vc_unpack_state(&packed, NULL));
}

/* Every field at the largest value the code can produce must survive. */
static void
test_max (void)
{
    vc_t vc;

    printf("field ends\n");

    memset(&vc, 0, sizeof(vc));
    vc.state                    = (vc_state_t) (VC_STATE_COUNT - 1);
    vc.entry_ms                 = UINT32_MAX;
    vc.stored_nav               = VC_NAV_UTURN;
    vc.rescans                  = (uint8_t) VC_MAX_RESCANS;
    vc.b_obstacle_active        = true;
    vc.b_reversing              = true;
    vc.prev_line                = VC_LINE_JUNCTION;
    vc.last_side                = 1;
    vc.search                   = VC_SEARCH_SECOND;
    vc.search_dir               = 1;
    vc.search_origin_mdeg       = INT32_MAX;
    vc.search_limit_mdeg        = VC_SEARCH_LIMIT_MDEG;
    vc.b_after_bypass           = true;
    vc.bypass_end_ms            = UINT32_MAX;
    vc.cal_phase                = VC_CAL_FINISH;
    vc.cal_phase_ms             = UINT32_MAX;
    vc.cal_out_mdeg             = 262143;   /* 18 U field end.          */
    vc.cal_far_mdeg             = 262143;
    vc.cal_first_dir            = 1;
    vc.cal_dir                  = -1;
    vc.b_junction_hold          = true;
    vc.b_junction_track         = true;
    vc.junction_origin_mm       = 8388607;  /* 24 s field end.          */
    vc.b_junction_seen          = true;
    vc.junction_seen_mm         = 8388607;
    vc.pending_turn             = VC_NAV_STRAIGHT;
    vc.steer_side               = -1;
    vc.b_right_exit             = true;
    vc.steer_mag                = (int16_t) VC_SEARCH_STEER;
    vc.lift_spell_cycles        = (uint8_t) (VC_LIFT_UNCUED_CYCLES + 1U);
    vc.lift_spell_max_cycles    = (uint8_t) VC_LIFT_SPELL_CAP_CYCLES;
    vc.b_lift_spell_cued        = true;

    CHECK(round_trip_ok(&vc));

    /* The other ends of the signed fields, including the whole-word
     * encoder angle, which must keep every bit. */
    memset(&vc, 0, sizeof(vc));
    vc.state              = VC_STATE_INITIALISE;
    vc.last_side          = -1;
    vc.search_dir         = -1;
    vc.search_origin_mdeg = INT32_MIN;
    vc.cal_first_dir      = -1;
    vc.cal_dir            = -1;
    vc.junction_origin_mm = -8388608;
    vc.junction_seen_mm   = -8388608;
    vc.steer_side         = -1;
    vc.entry_ms           = 0x1234U;
    vc.bypass_end_ms      = 0xABCDU;
    vc.cal_phase_ms       = 0x5678U;

    CHECK(round_trip_ok(&vc));

    /* A negative absolute encoder angle, which the search subtracts. */
    memset(&vc, 0, sizeof(vc));
    vc.search_origin_mdeg = -123456789;
    vc.search_limit_mdeg  = VC_SEARCH_LIMIT_POST_BYPASS_MDEG;
    CHECK(round_trip_ok(&vc));
}

/* Values outside a field saturate at the field end (rule: measured values
 * clamp).  These are not values the code produces; the point is that the
 * store is defined for them and the record stays decodable. */
static void
test_saturation (void)
{
    vc_t        vc;
    vc_packed_t packed;
    vc_t        back;

    printf("out-of-range fields\n");

    memset(&vc, 0, sizeof(vc));
    vc.lift_spell_cycles     = 200U;      /* 7 U field.                   */
    vc.lift_spell_max_cycles = 200U;      /* 5 U field.                   */
    vc.steer_mag             = 1000;      /* 7 U field.                   */
    vc.search_limit_mdeg     = 200000;    /* 17 U field.                  */
    vc.cal_out_mdeg          = 300000;    /* 18 U field.                  */
    vc.cal_far_mdeg          = 300000;
    vc.last_side             = -5;        /* 2 s field.                   */
    vc.junction_origin_mm    = 20000000;  /* 24 s field.                  */
    vc.rescans               = 9U;        /* 2 U field.                   */

    vc_pack_state(&vc, &packed);
    CHECK(vc_unpack_state(&packed, &back));
    CHECK(127U == back.lift_spell_cycles);
    CHECK(31U == back.lift_spell_max_cycles);
    CHECK(127 == back.steer_mag);
    CHECK(131071 == back.search_limit_mdeg);
    CHECK(262143 == back.cal_out_mdeg);
    CHECK(262143 == back.cal_far_mdeg);
    CHECK(-1 == back.last_side);
    CHECK(8388607 == back.junction_origin_mm);
    CHECK(3U == back.rescans);

    /* An enum that is not a code of its enum is stored as the no-data
     * code, so the record is rejected instead of presenting code 0. */
    memset(&vc, 0, sizeof(vc));
    vc.state = (vc_state_t) 200;
    vc_pack_state(&vc, &packed);
    CHECK(!vc_unpack_state(&packed, &back));
}

/* Every enum and every reserved bit: a record the packer cannot write is
 * rejected and the caller's state is left alone. */
static void
test_bad_records (void)
{
    vc_t        good;
    vc_packed_t packed;
    vc_packed_t bad;
    vc_t        sentinel;
    vc_t        back;
    uint32_t    word;

    printf("damaged records\n");

    memset(&good, 0, sizeof(good));
    good.state       = VC_STATE_FOLLOW_LINE;
    good.stored_nav  = VC_NAV_LEFT;
    good.prev_line   = VC_LINE_RIGHT;
    good.search      = VC_SEARCH_FIRST;
    good.cal_phase   = VC_CAL_OUT;
    good.pending_turn = VC_NAV_RIGHT;
    vc_pack_state(&good, &packed);

    /* state: 0..10 (4 U, codes 11..15 are not values). */
    for (word = 11U; word < 16U; word++)
    {
        bad = packed;
        set_field(&bad.word[0], 0U, 4U, word);
        CHECK(!vc_unpack_state(&bad, &back));
    }

    /* vc_nav_t: 0..4 in stored_nav and in pending_turn. */
    for (word = 5U; word < 8U; word++)
    {
        bad = packed;
        set_field(&bad.word[0], 4U, 3U, word);
        CHECK(!vc_unpack_state(&bad, &back));

        bad = packed;
        set_field(&bad.word[0], 27U, 3U, word);
        CHECK(!vc_unpack_state(&bad, &back));
    }

    /* vc_line_t: 0..4. */
    for (word = 5U; word < 8U; word++)
    {
        bad = packed;
        set_field(&bad.word[0], 11U, 3U, word);
        CHECK(!vc_unpack_state(&bad, &back));
    }

    /* vc_search_t: 0..2. */
    bad = packed;
    set_field(&bad.word[0], 16U, 2U, 3U);
    CHECK(!vc_unpack_state(&bad, &back));

    /* vc_cal_phase_t: 0..5. */
    for (word = 6U; word < 8U; word++)
    {
        bad = packed;
        set_field(&bad.word[0], 21U, 3U, word);
        CHECK(!vc_unpack_state(&bad, &back));
    }

    /* The reserved bits of every word with spare bits. */
    bad = packed;
    bad.word[0] |= 0x80000000U;
    CHECK(!vc_unpack_state(&bad, &back));

    bad = packed;
    bad.word[5] |= 0x7C000000U;
    CHECK(!vc_unpack_state(&bad, &back));

    bad = packed;
    bad.word[6] |= 0xFFC00000U;
    CHECK(!vc_unpack_state(&bad, &back));

    bad = packed;
    bad.word[7] |= 0xC0000000U;
    CHECK(!vc_unpack_state(&bad, &back));

    bad = packed;
    bad.word[8] |= 0xFF000000U;
    CHECK(!vc_unpack_state(&bad, &back));

    bad = packed;
    bad.word[9] |= 0xFF000000U;
    CHECK(!vc_unpack_state(&bad, &back));

    /* The record itself still decodes, and a rejected record leaves the
     * caller's state untouched. */
    CHECK(vc_unpack_state(&packed, &back));
    memset(&sentinel, 0xA5, sizeof(sentinel));
    {
        vc_t const before = sentinel;

        bad = packed;
        bad.word[0] |= 0x80000000U;
        CHECK(!vc_unpack_state(&bad, &sentinel));
        CHECK(0 == memcmp(&sentinel, &before, sizeof(sentinel)));
    }
}

/* ------------------------------------------------------------------------ *
 * A real mid-run state, from the same scenarios tests/vc uses
 * ------------------------------------------------------------------------ */

static vc_t        g_vc;
static vc_inputs_t g_in;
static vc_outputs_t g_out;

static void
reset (uint32_t start_ms)
{
    vc_init(&g_vc, start_ms);
    memset(&g_in, 0, sizeof(g_in));
    memset(&g_out, 0, sizeof(g_out));
    g_in.now_ms          = start_ms;
    g_in.line            = VC_LINE_CENTRE;
    g_in.line_sensors    = VC_LINE_SENSOR_CENTRE;
    g_in.b_terrain_ready = true;
    g_in.b_stationary    = true;
}

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

static void
wait_ms (uint32_t ms)
{
    uint32_t elapsed;

    for (elapsed = 0U; elapsed < ms; elapsed += VC_PERIOD_MS)
    {
        step();
    }
}

static void
run_calibration (void)
{
    wait_ms(VC_CAL_STILL_MS + VC_PERIOD_MS);
    g_in.line           = VC_LINE_LOST;
    g_in.cal_b3         = VC_CAL_B3_OK;
    g_in.cal_b4         = VC_CAL_B4_ENOUGH;
    g_in.cal_angle_mdeg = -40000;
    step();
    g_in.line           = VC_LINE_CENTRE;
    step();
    g_in.cal_b4         = VC_CAL_B4_DONE;
    step();
}

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

/* A state captured at three points of one run must survive the record. */
static void
test_mid_run (void)
{
    printf("mid-run round trips\n");

    /* 1. Mid-calibration: direction fields and the phase counter. */
    reset(0U);
    g_in.b_init_ok = true;
    step();
    g_in.cmd = VC_CMD_START;
    step();
    wait_ms(VC_CAL_STILL_MS + VC_PERIOD_MS);
    g_in.cal_angle_mdeg = 23456;
    step();
    CHECK(VC_STATE_CALIBRATE == g_vc.state);
    CHECK(0 != g_vc.cal_dir);
    CHECK(round_trip_ok(&g_vc));

    /* 2. Following the line: steer ramp, junction hold and the stored
     * barcode navigation command. */
    go_to_follow_line(1000U);
    g_in.line         = VC_LINE_RIGHT;
    g_in.line_sensors = VC_LINE_SENSOR_RIGHT;
    g_in.barcode      = VC_NAV_LEFT;
    wait_ms(200U);
    CHECK(VC_STATE_FOLLOW_LINE == g_vc.state);
    CHECK(0 != g_vc.steer_mag);
    CHECK(1 == g_vc.steer_side);
    CHECK(VC_NAV_LEFT == g_vc.stored_nav);
    CHECK(round_trip_ok(&g_vc));

    /* 3. A junction hold, then a turn, then a pivot search with an
     * absolute encoder angle far outside the 18-bit proposal and a
     * junction origin in travel_mm. */
    g_in.line          = VC_LINE_JUNCTION;
    g_in.travel_mm     = 345678;
    step();                                    /* Junction starts.    */
    g_in.travel_mm    += VC_JUNCTION_CONFIRM_MM;
    step();                                    /* Accepted: hold on.  */
    CHECK(VC_STATE_FOLLOW_LINE == g_vc.state);
    CHECK(VC_NAV_LEFT == g_vc.pending_turn);
    CHECK(g_vc.b_junction_hold);
    CHECK(345678 == g_vc.junction_origin_mm);
    CHECK(round_trip_ok(&g_vc));

    g_in.line          = VC_LINE_CENTRE;
    g_in.travel_mm    += VC_JUNCTION_ADVANCE_MM;
    step();
    CHECK(VC_STATE_TURNING == g_vc.state);
    CHECK(round_trip_ok(&g_vc));

    g_in.b_motion_done = true;
    step();
    CHECK(VC_STATE_FOLLOW_LINE == g_vc.state);
    CHECK(round_trip_ok(&g_vc));

    g_in.line           = VC_LINE_LOST;
    g_in.line_sensors   = 0U;
    g_in.cal_angle_mdeg = 1234567;        /* Far beyond +-131 071.      */
    step();
    CHECK(VC_STATE_REACQUIRE_LINE == g_vc.state);
    CHECK(g_vc.search_origin_mdeg == 1234567);
    CHECK(VC_SEARCH_LIMIT_MDEG == g_vc.search_limit_mdeg);
    CHECK(round_trip_ok(&g_vc));

    /* The search runs on the unpacked copy in the task, so the recorded
     * state has to keep the origin exactly to reach the same second
     * pivot: the decoded copy must produce the same outputs. */
    {
        vc_packed_t  packed;
        vc_t         copy;
        vc_outputs_t stored_out;
        int32_t      moved;

        memset(&packed, 0, sizeof(packed));
        vc_pack_state(&g_vc, &packed);
        CHECK(vc_unpack_state(&packed, &copy));
        g_in.cal_angle_mdeg += VC_SEARCH_LIMIT_MDEG;
        g_in.b_motion_done   = true;
        memset(&stored_out, 0, sizeof(stored_out));
        vc_step(&copy, &g_in, &stored_out);
        memset(&g_out, 0, sizeof(g_out));
        vc_step(&g_vc, &g_in, &g_out);
        CHECK(stored_out.pivot_mdeg == g_out.pivot_mdeg);
        CHECK(stored_out.state == g_out.state);
        moved = (g_out.pivot_mdeg < 0) ? -g_out.pivot_mdeg
                                       : g_out.pivot_mdeg;
        CHECK((2 * VC_SEARCH_LIMIT_MDEG) == moved);
    }
}

/* A bypass plan from the real planner survives its stored form. */
static void
test_plan (void)
{
    obstacle_bypass_plan_t plan;
    vc_plan_packed_t       packed;
    obstacle_bypass_plan_t back;
    uint32_t               distance;
    uint32_t               side;
    bool                   b_ok = true;

    printf("bypass plan record\n");

    /* Every plan the planner can produce for a distance in range. */
    for (distance = 100U; distance <= 4000U; distance += 100U)
    {
        for (side = OBSTACLE_ACTION_TURN_LEFT;
             side <= OBSTACLE_ACTION_TURN_RIGHT; side++)
        {
            if (obstacle_plan_bypass(distance, (obstacle_action_t) side,
                                     &plan))
            {
                vc_pack_plan(&plan, &packed);
                b_ok = vc_unpack_plan(&packed, &back) && b_ok;
                b_ok = (plan.side == back.side) && b_ok;
                b_ok = (plan.turn_deg == back.turn_deg) && b_ok;
                b_ok = (plan.out_mm == back.out_mm) && b_ok;
                b_ok = (plan.side_mm == back.side_mm) && b_ok;
                b_ok = (plan.along_mm == back.along_mm) && b_ok;
                b_ok = (plan.back_deg == back.back_deg) && b_ok;
                b_ok = (plan.back_mm == back.back_mm) && b_ok;
                b_ok = (plan.check_mm == back.check_mm) && b_ok;
            }
        }
    }
    CHECK(b_ok);

    /* The worst case the widths come from: 5 degree pivot at 4 m. */
    CHECK(obstacle_plan_bypass(4000U, OBSTACLE_ACTION_TURN_RIGHT, &plan));
    vc_pack_plan(&plan, &packed);
    CHECK(vc_unpack_plan(&packed, &back));
    CHECK(plan.turn_deg == back.turn_deg);
    CHECK(plan.out_mm == back.out_mm);
    CHECK(plan.along_mm == back.along_mm);
    CHECK(plan.back_mm == back.back_mm);
    CHECK(plan.check_mm == back.check_mm);
    CHECK(2294U == plan.out_mm);           /* 200 mm / sin(5 deg).       */
    CHECK(plan.along_mm <= 8191U);

    /* A side the planner never produces is rejected. */
    memset(&plan, 0, sizeof(plan));
    plan.side = OBSTACLE_ACTION_STOP;
    vc_pack_plan(&plan, &packed);
    CHECK(!vc_unpack_plan(&packed, &back));

    /* Reserved bits. */
    memset(&plan, 0, sizeof(plan));
    plan.side     = OBSTACLE_ACTION_TURN_LEFT;
    plan.turn_deg = 7U;
    vc_pack_plan(&plan, &packed);
    CHECK(vc_unpack_plan(&packed, &back));
    CHECK(OBSTACLE_ACTION_TURN_LEFT == back.side);
    CHECK(7U == back.turn_deg);

    packed.word[0] |= 0xC0000000U;
    CHECK(!vc_unpack_plan(&packed, &back));
    packed.word[0] = 0U;

    packed.word[1] |= 0xF0000000U;
    CHECK(!vc_unpack_plan(&packed, &back));
    packed.word[1] = 0U;

    packed.word[2] |= 0xFFFFC000U;
    CHECK(!vc_unpack_plan(&packed, &back));

    CHECK(!vc_unpack_plan(NULL, &back));
    CHECK(!vc_unpack_plan(&packed, NULL));
}

/* ------------------------------------------------------------------------ */

int
main (void)
{
    test_sizes();
    test_zero();
    test_max();
    test_saturation();
    test_bad_records();
    test_mid_run();
    test_plan();

    printf("\n%u checks, %u failed\n", (unsigned) g_checks,
           (unsigned) g_failures);

    return (0U == g_failures) ? 0 : 1;
}

/*** end of file ***/
