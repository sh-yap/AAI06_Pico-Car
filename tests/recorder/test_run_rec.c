/** @file test_run_rec.c
 *
 * @brief Host unit tests for the run recorder ring (run_rec.c).
 *
 * Run with:  make -C tests/recorder test
 *
 * Also writes the ring to the file named by argv[1], if given, so the
 * offline decoder (tools/run_rec_read.ps1 -File) can be checked against it.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>

#include "run_rec.h"

#ifndef B4_RECORDER
#define B4_RECORDER (0)
#endif

static uint32_t failures = 0U;
static uint32_t checks = 0U;


static void
check_u32(char const * p_name, uint32_t actual, uint32_t expected)
{
    checks++;

    if (actual != expected)
    {
        failures++;
        (void)printf("FAIL %s: got %u, expected %u\n",
                     p_name, (unsigned int)actual, (unsigned int)expected);
    }
}


/* A record whose ms identifies it. */
static void
add_ms(uint32_t ms)
{
    run_rec_record_t rec = {0};

    rec.ms = ms;
    rec.right_adc = (uint16_t)(ms & 0xFFFU);
    run_rec_add(&rec);
}


static void
test_layout(void)
{
    check_u32("record size", (uint32_t)sizeof(run_rec_record_t), 44U);
    check_u32("header size",
              (uint32_t)offsetof(run_rec_t, records), 20U);
    check_u32("pwm offset",
              (uint32_t)offsetof(run_rec_record_t, left_pwm_permille), 28U);
    check_u32("right_i offset",
              (uint32_t)offsetof(run_rec_record_t, right_i_permille), 34U);
    check_u32("learn_fwd_gain offset",
              (uint32_t)offsetof(run_rec_record_t, learn_fwd_gain), 36U);
    check_u32("learn_state offset",
              (uint32_t)offsetof(run_rec_record_t, learn_state), 42U);
    check_u32("capacity", RUN_REC_CAPACITY >= 1500U ? 1U : 0U, 1U);
}


static void
test_before_begin(void)
{
    add_ms(1U);
    check_u32("no begin: magic", g_run_rec.magic, 0U);
    check_u32("no begin: total", g_run_rec.total, 0U);
    ev_rec_hump(1U, 1U, 1U, true);
    ev_rec_max(1U, 5U);
    check_u32("ev no begin: magic", g_ev_rec.magic, 0U);
    check_u32("ev no begin: count", g_ev_rec.count, 0U);
#if B4_RECORDER
    {
        b4_rec_record_t rec = {0};

        b4_rec_add(&rec);
        check_u32("b4 no begin: magic", g_b4_rec.magic, 0U);
        check_u32("b4 no begin: total", g_b4_rec.total, 0U);
    }
#endif
}


static void
test_fill_without_wrap(void)
{
    uint32_t i;

    run_rec_begin();
    check_u32("begin: magic", g_run_rec.magic, RUN_REC_MAGIC);
    check_u32("begin: record_size", g_run_rec.record_size, 44U);
    check_u32("begin: capacity", g_run_rec.capacity, RUN_REC_CAPACITY);
    check_u32("begin: frozen", g_run_rec.frozen, 0U);

    for (i = 0U; i < 10U; i++)
    {
        add_ms(1000U + (i * 20U));
    }

    check_u32("10: total", g_run_rec.total, 10U);
    check_u32("10: write_index", g_run_rec.write_index, 10U);
    check_u32("10: first ms", g_run_rec.records[0].ms, 1000U);
    check_u32("10: last ms", g_run_rec.records[9].ms, 1180U);
    check_u32("10: cycle of 4th", g_run_rec.records[3].cycle, 3U);
}


static void
test_wrap_keeps_newest(void)
{
    uint32_t i;
    uint32_t const n = RUN_REC_CAPACITY + 7U;   /* Seven past full.          */

    run_rec_begin();

    for (i = 0U; i < n; i++)
    {
        add_ms(i);
    }

    check_u32("wrap: total", g_run_rec.total, n);
    check_u32("wrap: write_index", g_run_rec.write_index, 7U);
    /* Oldest kept is record 7, at write_index; newest is n - 1 just before. */
    check_u32("wrap: oldest ms", g_run_rec.records[7].ms, 7U);
    check_u32("wrap: newest ms", g_run_rec.records[6].ms, n - 1U);
    check_u32("wrap: newest cycle", g_run_rec.records[6].cycle,
              (n - 1U) & 0xFFFFU);
    check_u32("wrap: overwritten slot 0", g_run_rec.records[0].ms,
              RUN_REC_CAPACITY);
    check_u32("wrap: last slot untouched",
              g_run_rec.records[RUN_REC_CAPACITY - 1U].ms,
              RUN_REC_CAPACITY - 1U);
}


static void
test_freeze_and_reset(void)
{
    uint32_t total;
    uint32_t index;

    run_rec_freeze();
    total = g_run_rec.total;
    index = g_run_rec.write_index;
    check_u32("freeze: frozen", g_run_rec.frozen, 1U);

    add_ms(99999U);
    check_u32("frozen: total unchanged", g_run_rec.total, total);
    check_u32("frozen: index unchanged", g_run_rec.write_index, index);
    check_u32("frozen: newest unchanged", g_run_rec.records[6].ms,
              RUN_REC_CAPACITY + 6U);

    run_rec_begin();
    check_u32("reset: total", g_run_rec.total, 0U);
    check_u32("reset: write_index", g_run_rec.write_index, 0U);
    check_u32("reset: frozen", g_run_rec.frozen, 0U);

    add_ms(5U);
    check_u32("reset: records again", g_run_rec.total, 1U);
    check_u32("reset: first ms", g_run_rec.records[0].ms, 5U);
    check_u32("reset: cycle restarts", g_run_rec.records[0].cycle, 0U);
}


static void
add_bc(uint32_t ms)
{
    bc_rec_record_t rec = {0};

    rec.ms = ms;
    rec.barcode_adc = (uint16_t)(ms & 0xFFFU);
    bc_rec_add(&rec);
}


static void
test_bc_ring(void)
{
    uint32_t i;
    uint32_t const n = BC_REC_CAPACITY + 5U;    /* Five past full.           */
    uint32_t total;

    check_u32("bc layout: record", (uint32_t)sizeof(bc_rec_record_t), 36U);
    check_u32("bc layout: header",
              (uint32_t)offsetof(bc_rec_t, records), 20U);
    check_u32("bc layout: flags",
              (uint32_t)offsetof(bc_rec_record_t, flags), 16U);
#if B4_RECORDER
    check_u32("bc capacity", BC_REC_CAPACITY, 500U);
#else
    check_u32("bc capacity", BC_REC_CAPACITY, 2000U);
#endif

    /* A reset of the run ring resets this one too. */
    run_rec_begin();
    check_u32("bc begin: magic", g_bc_rec.magic, BC_REC_MAGIC);
    check_u32("bc begin: record_size", g_bc_rec.record_size, 36U);
    check_u32("bc begin: capacity", g_bc_rec.capacity, BC_REC_CAPACITY);
    check_u32("bc begin: total", g_bc_rec.total, 0U);
    check_u32("bc begin: frozen", g_bc_rec.frozen, 0U);

    add_bc(10U);
    add_bc(20U);
    check_u32("bc 2: total", g_bc_rec.total, 2U);
    check_u32("bc 2: write_index", g_bc_rec.write_index, 2U);
    check_u32("bc 2: first ms", g_bc_rec.records[0].ms, 10U);
    check_u32("bc 2: second adc", g_bc_rec.records[1].barcode_adc, 20U);

    run_rec_begin();

    for (i = 0U; i < n; i++)
    {
        add_bc(i);
    }

    check_u32("bc wrap: total", g_bc_rec.total, n);
    check_u32("bc wrap: write_index", g_bc_rec.write_index, 5U);
    check_u32("bc wrap: oldest ms", g_bc_rec.records[5].ms, 5U);
    check_u32("bc wrap: newest ms", g_bc_rec.records[4].ms, n - 1U);
    check_u32("bc wrap: last slot untouched",
              g_bc_rec.records[BC_REC_CAPACITY - 1U].ms,
              BC_REC_CAPACITY - 1U);

    run_rec_freeze();
    total = g_bc_rec.total;
    check_u32("bc freeze: frozen", g_bc_rec.frozen, 1U);
    add_bc(99999U);
    check_u32("bc frozen: total unchanged", g_bc_rec.total, total);
    check_u32("bc frozen: index unchanged", g_bc_rec.write_index, 5U);
    check_u32("bc frozen: newest unchanged", g_bc_rec.records[4].ms, n - 1U);

    run_rec_begin();
    check_u32("bc reset: total", g_bc_rec.total, 0U);
    check_u32("bc reset: frozen", g_bc_rec.frozen, 0U);
    add_bc(7U);
    check_u32("bc reset: first ms", g_bc_rec.records[0].ms, 7U);
}


#if B4_RECORDER
static void
add_b4(uint32_t ms)
{
    b4_rec_record_t rec = {0};

    rec.ms = ms;
    rec.noise_mg = (uint16_t)(ms & 0xFFFU);
    b4_rec_add(&rec);
}


static void
test_b4_ring(void)
{
    uint32_t i;
    uint32_t const n = B4_REC_CAPACITY + 5U;    /* Five past full.           */
    uint32_t total;

    check_u32("b4 layout: record", (uint32_t)sizeof(b4_rec_record_t), 40U);
    check_u32("b4 layout: header",
              (uint32_t)offsetof(b4_rec_t, records), 20U);
    check_u32("b4 layout: dt_us",
              (uint32_t)offsetof(b4_rec_record_t, dt_us), 34U);
    check_u32("b4 layout: hump_state",
              (uint32_t)offsetof(b4_rec_record_t, hump_state), 36U);
    check_u32("b4 layout: tilt",
              (uint32_t)offsetof(b4_rec_record_t, tilt), 39U);
    check_u32("b4 capacity", B4_REC_CAPACITY, 1500U);
    check_u32("b4 magic", B4_REC_MAGIC, 0x31523442UL);

    /* A reset of the run ring resets this one too. */
    run_rec_begin();
    check_u32("b4 begin: magic", g_b4_rec.magic, B4_REC_MAGIC);
    check_u32("b4 begin: record_size", g_b4_rec.record_size, 40U);
    check_u32("b4 begin: capacity", g_b4_rec.capacity, B4_REC_CAPACITY);
    check_u32("b4 begin: total", g_b4_rec.total, 0U);
    check_u32("b4 begin: frozen", g_b4_rec.frozen, 0U);

    add_b4(10U);
    add_b4(20U);
    check_u32("b4 2: total", g_b4_rec.total, 2U);
    check_u32("b4 2: write_index", g_b4_rec.write_index, 2U);
    check_u32("b4 2: first ms", g_b4_rec.records[0].ms, 10U);
    check_u32("b4 2: second noise", g_b4_rec.records[1].noise_mg, 20U);

    run_rec_begin();

    for (i = 0U; i < n; i++)
    {
        add_b4(i);
    }

    check_u32("b4 wrap: total", g_b4_rec.total, n);
    check_u32("b4 wrap: write_index", g_b4_rec.write_index, 5U);
    check_u32("b4 wrap: oldest ms", g_b4_rec.records[5].ms, 5U);
    check_u32("b4 wrap: newest ms", g_b4_rec.records[4].ms, n - 1U);
    check_u32("b4 wrap: last slot untouched",
              g_b4_rec.records[B4_REC_CAPACITY - 1U].ms,
              B4_REC_CAPACITY - 1U);

    run_rec_freeze();
    total = g_b4_rec.total;
    check_u32("b4 freeze: frozen", g_b4_rec.frozen, 1U);
    add_b4(99999U);
    check_u32("b4 frozen: total unchanged", g_b4_rec.total, total);
    check_u32("b4 frozen: index unchanged", g_b4_rec.write_index, 5U);
    check_u32("b4 frozen: newest unchanged", g_b4_rec.records[4].ms, n - 1U);

    run_rec_begin();
    check_u32("b4 reset: total", g_b4_rec.total, 0U);
    check_u32("b4 reset: write_index", g_b4_rec.write_index, 0U);
    check_u32("b4 reset: frozen", g_b4_rec.frozen, 0U);
    add_b4(7U);
    check_u32("b4 reset: first ms", g_b4_rec.records[0].ms, 7U);
}
#endif /* B4_RECORDER */


static void
write_sample(char const * p_path)
{
    FILE * p_file = fopen(p_path, "wb");
    FILE * p_bc_file;
    char bc_path[512];
    uint32_t i;

    if (NULL != p_file)
    {
        /* 1600 records with one 70 ms hole, so the ring has wrapped and the
         * decoder has a missed cycle to flag. */
        run_rec_begin();

        for (i = 0U; i < 1600U; i++)
        {
            run_rec_record_t rec = {0};

            rec.ms = 100000U + (i * 20U) + ((i >= 800U) ? 70U : 0U);
            rec.centre_adc = (uint16_t)(1000U + i);
            rec.right_adc = 3000U;
            rec.line_state = 1U;
            rec.line_error = (int8_t)((int32_t)(i % 21U) - 10);
            rec.sensors = 2U;
            rec.vc_state = 3U;
            rec.steer = (int8_t)(-5);
            rec.flags = (i >= 1500U) ? RUN_REC_FLAG_RAMP : 0U;
            rec.speed_mm_s = 150;
            rec.target_left_mm_s = 140;
            rec.target_right_mm_s = 160;
            rec.left_mm_s = 138;
            rec.right_mm_s = 158;
            rec.left_pwm_permille = (int16_t)(-300 + (int32_t)(i % 50U));
            rec.right_pwm_permille = 410;
            rec.left_i_permille = -120;
            rec.right_i_permille = (int16_t)(i % 90U);
            rec.learn_fwd_gain = (int16_t)(550 + (int32_t)(i % 7U));
            rec.learn_fwd_offset = (int16_t)(-320);
            rec.learn_turn_gain = (int16_t)(300 + (int32_t)(i % 5U));
            rec.learn_state = (i >= 800U) ? 6U : 1U;
            run_rec_add(&rec);
        }

        /* 2100 barcode records (wraps), a 30 ms hole at 1000, black over
         * 1200..1209, and a capture of 'A' (code 1) finishing at 1500. */
        for (i = 0U; i < 2100U; i++)
        {
            bc_rec_record_t bc = {0};

            bc.ms = 200000U + (i * 10U) + ((i >= 1000U) ? 30U : 0U);
            bc.enc = (int32_t)i * 3 - 100;
            bc.left_enc = bc.enc + 17;
            bc.right_enc = bc.enc - 17;
            bc.encoder_us = bc.ms * 1000U + 500U;
            bc.sample_end_ms = bc.ms + 1U;
            bc.threshold = 1600U;
            bc.centre_adc = 3000U;
            bc.right_adc = 150U;
            bc.barcode_adc = (uint16_t)(((i >= 1200U) && (i < 1210U)) ?
                                        3700U : 200U);
            bc.flags = (uint8_t)(((i >= 1200U) && (i < 1210U)) ?
                                 (BC_REC_FLAG_BLACK | BC_REC_FLAG_ACTIVE) : 0U);
            bc.edges = (uint8_t)(((i >= 1200U) && (i <= 1500U)) ? 9U : 0U);

            if (1500U == i)
            {
                bc.flags = BC_REC_FLAG_DONE;
                bc.code = 1U;
                bc.ch = (uint8_t)'A';
            }

            bc_rec_add(&bc);
        }

        run_rec_freeze();
        (void)fwrite(&g_run_rec, sizeof(g_run_rec), 1U, p_file);
        (void)fclose(p_file);

        (void)snprintf(bc_path, sizeof(bc_path), "%s.bc", p_path);
        p_bc_file = fopen(bc_path, "wb");

        if (NULL != p_bc_file)
        {
            (void)fwrite(&g_bc_rec, sizeof(g_bc_rec), 1U, p_bc_file);
            (void)fclose(p_bc_file);
        }
    }
}


/* One 20 ms step of the event detector. */
static void
ev_step(uint32_t ms, bool b_lost, bool b_follow, int8_t steer, bool b_ramp)
{
    ev_rec_step(ms, b_lost, b_follow, steer, b_ramp);
}


static void
test_ev_rec(void)
{
    uint32_t i;

    check_u32("ev layout: record size", (uint32_t)sizeof(ev_rec_record_t), 8U);
    check_u32("ev layout: total size", (uint32_t)sizeof(ev_rec_t), 512U);
    check_u32("ev layout: records offset",
              (uint32_t)offsetof(ev_rec_t, records), 16U);

    /* BEGIN clears it, and nothing is logged without a change. */
    run_rec_begin();
    check_u32("ev begin: magic", g_ev_rec.magic, EV_REC_MAGIC);
    check_u32("ev begin: record_size", g_ev_rec.record_size, 8U);
    check_u32("ev begin: capacity", g_ev_rec.capacity, EV_REC_CAPACITY);
    check_u32("ev begin: count", g_ev_rec.count, 0U);
    check_u32("ev begin: dropped", g_ev_rec.dropped, 0U);
    ev_step(1000U, false, true, 10, false);
    ev_step(1020U, false, true, 20, false);
    check_u32("ev steady: count", g_ev_rec.count, 0U);

    /* LOST in FOLLOW_LINE: value is the previous step's steer. */
    ev_step(1040U, true, false, 127, false);
    check_u32("ev lost: count", g_ev_rec.count, 1U);
    check_u32("ev lost: type", g_ev_rec.records[0].type, EV_REC_LINE_LOST);
    check_u32("ev lost: ms", g_ev_rec.records[0].ms, 1040U);
    check_u32("ev lost: value", (uint32_t)g_ev_rec.records[0].value, 20U);
    ev_step(1060U, true, false, 0, false);
    check_u32("ev still lost: count", g_ev_rec.count, 1U);

    /* LINE_BACK: ms since LOST. */
    ev_step(1340U, false, false, 0, false);
    check_u32("ev back: count", g_ev_rec.count, 2U);
    check_u32("ev back: type", g_ev_rec.records[1].type, EV_REC_LINE_BACK);
    check_u32("ev back: value", (uint32_t)g_ev_rec.records[1].value, 300U);

    /* LOST outside FOLLOW_LINE is not logged, nor its return. */
    ev_step(1360U, true, false, 0, false);
    ev_step(1380U, false, false, 0, false);
    check_u32("ev lost outside follow: count", g_ev_rec.count, 2U);

    /* Ramp edges: one event each. */
    ev_step(1400U, false, true, 0, true);
    ev_step(1420U, false, true, 0, true);
    check_u32("ev ramp start: count", g_ev_rec.count, 3U);
    check_u32("ev ramp start: type", g_ev_rec.records[2].type,
              EV_REC_RAMP_START);
    ev_step(1440U, false, true, 0, false);
    check_u32("ev ramp end: count", g_ev_rec.count, 4U);
    check_u32("ev ramp end: type", g_ev_rec.records[3].type, EV_REC_RAMP_END);

    /* Hump: newest height, count and the valid bit, both ways. */
    ev_rec_hump(1500U, 123U, 2U, true);
    check_u32("ev hump: count", g_ev_rec.count, 5U);
    check_u32("ev hump: type", g_ev_rec.records[4].type, EV_REC_HUMP_DONE);
    check_u32("ev hump: value", (uint32_t)g_ev_rec.records[4].value, 123U);
    check_u32("ev hump: valid stored", g_ev_rec.records[4].count,
              2U | EV_REC_VALID_BIT);
    ev_rec_hump(1505U, 40U, 3U, false);
    check_u32("ev hump invalid: count", g_ev_rec.count, 6U);
    check_u32("ev hump invalid: byte", g_ev_rec.records[5].count, 3U);

    /* RUN_MAX: nothing for 0 after BEGIN, once per change. */
    ev_rec_max(1510U, 0U);
    check_u32("ev max 0 after begin: count", g_ev_rec.count, 6U);
    ev_rec_max(1520U, 150U);
    check_u32("ev max: count", g_ev_rec.count, 7U);
    check_u32("ev max: type", g_ev_rec.records[6].type, EV_REC_RUN_MAX);
    check_u32("ev max: value", (uint32_t)g_ev_rec.records[6].value, 150U);
    ev_rec_max(1530U, 150U);
    check_u32("ev max unchanged: count", g_ev_rec.count, 7U);
    ev_rec_max(1540U, 180U);
    check_u32("ev max changed: count", g_ev_rec.count, 8U);

    /* Ordering: the times never go backwards. */
    for (i = 1U; i < g_ev_rec.count; i++)
    {
        check_u32("ev order", (g_ev_rec.records[i].ms
                               >= g_ev_rec.records[i - 1U].ms) ? 1U : 0U, 1U);
    }

    /* Full: the earliest events stay, the rest are counted, no wrap. */
    for (i = 0U; i < 100U; i++)
    {
        ev_rec_hump(2000U + i, 1U, 3U, true);
    }

    check_u32("ev full: count", g_ev_rec.count, EV_REC_CAPACITY);
    check_u32("ev full: dropped", g_ev_rec.dropped,
              8U + 100U - EV_REC_CAPACITY);
    check_u32("ev full: first kept", g_ev_rec.records[0].type,
              EV_REC_LINE_LOST);

    /* Frozen: nothing more is logged. */
    run_rec_freeze();
    ev_rec_hump(9000U, 1U, 4U, true);
    ev_rec_max(9010U, 999U);
    check_u32("ev frozen: dropped", g_ev_rec.dropped,
              8U + 100U - EV_REC_CAPACITY);

    /* BEGIN again clears it, including the edge memory. */
    run_rec_begin();
    check_u32("ev rebegin: count", g_ev_rec.count, 0U);
    check_u32("ev rebegin: dropped", g_ev_rec.dropped, 0U);
    ev_rec_max(3005U, 0U);
    check_u32("ev rebegin: max 0 logs nothing", g_ev_rec.count, 0U);
    ev_step(3000U, true, true, 5, false);
    check_u32("ev rebegin: no stale edge", g_ev_rec.count, 0U);
}


int
main(int argc, char * argv[])
{
    test_layout();
    test_before_begin();
    test_fill_without_wrap();
    test_wrap_keeps_newest();
    test_freeze_and_reset();
    test_bc_ring();
    test_ev_rec();
#if B4_RECORDER
    test_b4_ring();
#endif

    (void)printf("%u checks, %u failures\n",
                 (unsigned int)checks, (unsigned int)failures);

    if ((2 <= argc) && (0U == failures))
    {
        write_sample(argv[1]);
    }

    return ((0U == failures) ? 0 : 1);
}

/*** end of file ***/
