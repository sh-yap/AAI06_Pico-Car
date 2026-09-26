/*
 * Copyright (c) 2026 AAI06 Pico-Car team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * comms_demo.c
 * Fake-data driver for the comms API.  Built only with COMMS_DEMO=1.
 *
 * Stands in for the other four subsystems and the vehicle controller so the
 * telemetry link can be tested and demonstrated before they are ready.  It
 * uses exactly the calls the real modules will use (see comms.h).
 *
 *   mosquitto_pub -t robot/command -m start   -> fake run begins
 *   mosquitto_pub -t robot/command -m stop    -> run ends, summary sent
 */

#include <tk/tkernel.h>

#include "comms.h"
#include "comms_demo.h"

#if COMMS_DEMO

#define DEMO_TICK_MS          100U   /* 10 Hz: faster than the 5 Hz link  */
#define DEMO_SPEED_MM_S       200
#define DEMO_COUNTS_PER_MM    2      /* made-up encoder resolution        */
#define DEMO_BARCODE_TICKS    50U    /* every 5 s                         */
#define DEMO_HUMP_TICKS       70U    /* every 7 s                         */
#define DEMO_OBSTACLE_TICKS   110U   /* every 11 s                        */
#define DEMO_HUMP_BASE_MM     20
#define DEMO_HUMP_STEP_MM     7
#define DEMO_HUMP_STEPS       5U

static char const        g_codes[] = "ABCD";
static comms_nav_t const g_code_cmds[] =
{
    COMMS_NAV_LEFT, COMMS_NAV_RIGHT, COMMS_NAV_STRAIGHT, COMMS_NAV_UTURN
};


static void
demo_tick (uint32_t tick, int32_t * p_distance_mm)
{
    comms_motion_t   motion  = {0};
    comms_line_t     line    = {0};
    comms_terrain_t  terrain = {0};
    comms_obstacle_t obstacle;
    uint32_t         index;

    /* Buddy 2: steady speed with a small left/right wobble. */
    *p_distance_mm += (DEMO_SPEED_MM_S * (int32_t) DEMO_TICK_MS) / 1000;
    motion.speed_left_mm_s  = DEMO_SPEED_MM_S + (int32_t) (tick % 5U);
    motion.speed_right_mm_s = DEMO_SPEED_MM_S - (int32_t) (tick % 3U);
    motion.encoder_left     = *p_distance_mm * DEMO_COUNTS_PER_MM;
    motion.encoder_right    = *p_distance_mm * DEMO_COUNTS_PER_MM;
    motion.distance_mm      = *p_distance_mm;
    comms_report_motion(&motion);

    /* Buddy 3: line drifts left, centre, right. */
    line.position = (comms_line_pos_t) (COMMS_LINE_LEFT + (tick / 10U) % 3U);
    line.error    = ((int32_t) ((tick / 10U) % 3U) - 1) * 40;
    line.sensors  = (uint8_t) (1U << ((tick / 10U) % 3U));
    comms_report_line(&line);

    /* Buddy 4: flat ground. */
    terrain.motion = COMMS_MOTION_ACCELERATING;
    comms_report_terrain(&terrain);

    if (0U == (tick % DEMO_BARCODE_TICKS))
    {
        index = ((tick / DEMO_BARCODE_TICKS) - 1U) % 4U;   /* A first */
        comms_set_state(COMMS_STATE_EXECUTE_NAV);
        (void) comms_event_barcode(g_codes[index], g_code_cmds[index]);
    }
    else if (1U == (tick % DEMO_BARCODE_TICKS))
    {
        comms_set_state(COMMS_STATE_LINE_FOLLOW);
    }
    else
    {
        /* No state change on this tick. */
    }

    if (0U == (tick % DEMO_HUMP_TICKS))
    {
        (void) comms_event_hump(DEMO_HUMP_BASE_MM + DEMO_HUMP_STEP_MM
                                * (int32_t) ((tick / DEMO_HUMP_TICKS)
                                             % DEMO_HUMP_STEPS));
    }

    if (0U == (tick % DEMO_OBSTACLE_TICKS))
    {
        obstacle.distance_mm    = 180;
        obstacle.angle_deg      = 85;
        obstacle.width_mm       = 120;
        obstacle.clear_left_mm  = 250;
        obstacle.clear_right_mm = 90;
        obstacle.action         = COMMS_AVOID_TURN_LEFT;
        (void) comms_event_obstacle(&obstacle);
    }
}


static void
comms_demo_task (INT stacd, void * p_exinf)
{
    comms_cmd_t cmd;
    bool        b_running   = false;
    uint32_t    tick        = 0U;
    int32_t     distance_mm = 0;

    (void) stacd;
    (void) p_exinf;

    comms_set_state(COMMS_STATE_READY);

    for (;;)
    {
        /* This is how the real vehicle controller will read commands. */
        if (comms_get_command(&cmd))
        {
            if ((COMMS_CMD_START == cmd) && !b_running)
            {
                b_running = true;
                tick      = 1U;
                (void) comms_begin_run();
                comms_set_state(COMMS_STATE_LINE_FOLLOW);
            }
            else if ((COMMS_CMD_STOP == cmd) && b_running)
            {
                b_running = false;
                comms_set_state(COMMS_STATE_FINISHED);
                (void) comms_end_run();
            }
            else
            {
                /* Duplicate start/stop: nothing to do. */
            }
        }

        if (b_running)
        {
            demo_tick(tick, &distance_mm);
            tick++;
        }

        (void) tk_dly_tsk(DEMO_TICK_MS);
    }
}


bool
comms_demo_start (void)
{
    T_CTSK ctsk = {0};
    ID     task;

    ctsk.tskatr  = TA_HLNG | TA_RNG3;
    ctsk.task    = (FP) comms_demo_task;
    ctsk.itskpri = 9;
    ctsk.stksz   = 2048;

    task = tk_cre_tsk(&ctsk);

    return (task > 0) && (E_OK == tk_sta_tsk(task, 0));
}

#endif /* COMMS_DEMO */
