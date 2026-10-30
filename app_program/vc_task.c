/** @file vc_task.c
 *
 * @brief Vehicle Controller RTOS task (see vc_task.h).
 *
 * Every 20 ms: gather inputs -> vc_step() -> act on the outputs.
 *
 * Subsystems, each behind a switch so a missing one can be left out:
 *   - Buddy 3 (line/barcode), VC_HAVE_BUDDY3: packed status words from
 *     Buddy 3's task.  With 0 the line is always "centred" and a run ends
 *     itself after VC_TEST_RUN_MS (bench safety);
 *   - Buddy 5 (obstacle), VC_HAVE_BUDDY5: "obstacle ahead" and sensor
 *     fault from Buddy 5's task.  Scan and bypass are not written yet, so
 *     an obstacle stops the car and the scan timeout ends in
 *     EMERGENCY_STOP (fail safe);
 *   - Buddy 4 (terrain), VC_HAVE_BUDDY4: packed terrain words from Buddy
 *     4's task.  The car slows on a ramp and whenever Buddy 4 is not
 *     ready or silent, and START waits for it.  With 0 the car never
 *     slows for terrain and START is always allowed.
 *
 * Motion (Buddy 2) keeps velocity updates inside one controller run and
 * serialises stop handling. The VC still holds for VC_STOP_HOLD_STEPS steps
 * after a stop as a conservative transition delay.
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
#define VC_HAVE_BUDDY3           (1)    /* Line + barcode from Buddy 3.   */
#endif

#ifndef VC_HAVE_BUDDY4
#define VC_HAVE_BUDDY4           (1)    /* Ramp status from Buddy 4.      */
#endif

#ifndef VC_HAVE_BUDDY5
#define VC_HAVE_BUDDY5           (1)    /* Obstacle ahead from Buddy 5.   */
#endif

#if VC_HAVE_BUDDY3
#include "buddy3_task.h"
#include "line_follow.h"
#endif

#if VC_HAVE_BUDDY4
#include "b4_types.h"
#include "b4_task.h"
#endif

#if VC_HAVE_BUDDY5
#include "obstacle_types.h"
#include "obstacle_task.h"
#endif

/* ------------------------------------------------------------------------ *
 * Tunables.  All TBC: tune on the track.
 * ------------------------------------------------------------------------ */

#define VC_TASK_PRIORITY         (5)       /* Just below Motion (4).       */
#define VC_TASK_STACK_BYTES      (4096U)   /* SMP kernel path is deep.     */

#define VC_STEER_GAIN_MDEG_S     (350)     /* Turn rate per steer unit;
                                              127 -> about 44 deg/s.  700
                                              suited 160 mm/s; at 80 mm/s it
                                              turned twice as sharply per mm
                                              and zig-zagged (29 Oct: full
                                              left to full right in 0.2 s).
                                              350 at 80 = 700 at 160 per mm.
                                              400 at 160 ran off bends.    */
#define VC_TRACK_HALF_MM         (60)      /* Half the 119 mm track width. */
#define VC_TURN_INNER_FLOOR_MM_S (0)       /* Slowest the inner wheel may be
                                              told to go in a turn.  Was 10
                                              (29 Oct: both wheels stalled
                                              on hard pivots, with the old
                                              feedforward).  0 lets full
                                              steer at 50 mm/s turn about
                                              the inner wheel (0/100 mm/s,
                                              about 60 mm radius); Motion
                                              brakes a wheel told to go
                                              slower than it can drive.    */
/* mdeg/s of turn that moves the inner wheel by 1 mm/s = 180000/pi / half. */
#define VC_MDEGPS_PER_MM_S_DIFF  (57296 / VC_TRACK_HALF_MM)
#define VC_STEER_RESEND          (1)       /* Re-send drive on every steer
                                              change (of 127).  Motion and
                                              the controller both run every
                                              20 ms, so this is at most one
                                              command per Motion cycle.    */
#define VC_SPEED_RESEND          (10)      /* ...or speed moves this much
                                              (mm/s).  The bend slowdown
                                              changes speed with every
                                              small steer change.          */
#define VC_TURN_SPEED_MM_S       (80)      /* Wheel speed for encoder turns.
                                              Buddy 2's validated loaded
                                              wheel speed (was 100).        */
/* Turn per encoder count of left-minus-right difference, in a pivot:
 * (wheel circumference 181 mm / 2536 counts) / 119 mm track, in mdeg.
 * Motion's own configuration (motion_config.c), good to about 1 percent;
 * enough for a safety limit and a log. */
#define VC_CAL_MDEG_PER_COUNT    (34)
/* Distance per encoder count: 181 mm wheel circumference per 2536 counts. */
#define VC_WHEEL_MM              (181)
#define VC_WHEEL_COUNTS          (2536)
#define VC_REVERSE_MM            (100)     /* Back off before a rescan.    */
#define VC_REVERSE_SPEED_MM_S    (80)
#define VC_STATIONARY_MM_S       (10)      /* Both wheels below = still.   */
#define VC_STOP_HOLD_STEPS       (2U)      /* 40 ms: Motion sees the stop. */
#define VC_SUBMIT_RETRIES        (5U)      /* BUSY / QUEUE_FULL retries.   */
#define VC_SUBMIT_RETRIES_DRIVE  (10U)     /* Same for drive: 0.2 s.       */
#define VC_TEST_RUN_MS           (5000U)   /* Placeholder-line run length. */
#define VC_LINE_STALE_MS         (100U)    /* No Buddy 3 data = line lost. */
#define VC_LINE_DRAIN_MAX        (16U)     /* Messages read per step, max. */
#define VC_TERRAIN_STALE_MS      (100U)    /* No Buddy 4 word = not ready.
                                              Five VC steps. The queue
                                              holds two words, so a word
                                              is at most 40 ms old when
                                              received.                    */
#define VC_TERRAIN_DRAIN_MAX     (8U)      /* Buddy 4 words read per step. */
#define VC_TRACE_STEPS           (5U)      /* Run trace every 100 ms.      */

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
static uint32_t g_drive_busy_try;      /* Drive refused as busy in a row. */
static int16_t  g_drive_speed;
static int8_t   g_drive_steer;

static bool     g_b_run_active;
static uint32_t g_run_start_ms;
static uint32_t g_overruns;            /* Steps that started late.        */
static uint16_t g_faults_seen;         /* Faults since the run started.   */
static int32_t  g_left_mm_s;           /* Measured wheel speeds, for the  */
static int32_t  g_right_mm_s;          /* run trace.                      */
static int32_t  g_diff_counts;         /* Left minus right encoder counts. */
static int32_t  g_sweep_origin_counts; /* g_diff_counts when the sweep began.*/

/* VC state -> the state codes comms publishes today (tracker item C5). */
static comms_state_t const g_comms_state[VC_STATE_COUNT] =
{
    COMMS_STATE_INIT,           /* INITIALISE      */
    COMMS_STATE_READY,          /* READY           */
    COMMS_STATE_READY,          /* CALIBRATE: comms has no state for it
                                   yet; Buddy 1 to add one              */
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


/* Encoder counts to mm.  The whole revolutions and the remainder are scaled
 * separately, so the product stays below 2536 * 181 and cannot overflow for
 * any int32 count (a plain counts * 181 would, after about 840 m). */
static int32_t
counts_to_mm (int32_t counts)
{
    return ((counts / VC_WHEEL_COUNTS) * VC_WHEEL_MM)
           + (((counts % VC_WHEEL_COUNTS) * VC_WHEEL_MM) / VC_WHEEL_COUNTS);
}


/* Motion was only busy or full: worth trying again next step. */
static bool
is_retryable (motion_result_t result)
{
    return (MOTION_RESULT_BUSY == result)
           || (MOTION_RESULT_QUEUE_FULL == result);
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

/* Motion's own fault code -> the shared fault bit, so the laptop shows
 * which Motion fault stopped the car. */
static uint16_t
motion_fault_bit (motion_fault_t fault)
{
    uint16_t bit = (uint16_t) VC_FAULT_MOTION_CMD;

    if (MOTION_FAULT_STALL == fault)
    {
        bit = (uint16_t) VC_FAULT_STALL;
    }
    else if (MOTION_FAULT_ENCODER == fault)
    {
        bit = (uint16_t) VC_FAULT_ENCODER;
    }
    else
    {
        /* Timeout, invalid command, hardware: a command problem. */
    }

    return bit;
}


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
            p_in->serious_faults |= motion_fault_bit(status.fault);
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

        g_left_mm_s             = status.left_speed_mm_s;
        g_right_mm_s            = status.right_speed_mm_s;
        g_diff_counts           = status.left_encoder_counts
                                  - status.right_encoder_counts;
        p_in->cal_angle_mdeg    = (g_diff_counts - g_sweep_origin_counts)
                                  * VC_CAL_MDEG_PER_COUNT;
        p_in->travel_mm         = counts_to_mm(status.left_encoder_counts / 2)
                                  + counts_to_mm(status.right_encoder_counts
                                                 / 2);
        report.speed_left_mm_s  = status.left_speed_mm_s;
        report.speed_right_mm_s = status.right_speed_mm_s;
        report.encoder_left     = status.left_encoder_counts;
        report.encoder_right    = status.right_encoder_counts;
        report.distance_mm      = status.travelled_mm;
        report.target_left_mm_s  = status.left_target_mm_s;
        report.target_right_mm_s = status.right_target_mm_s;
        report.slewed_left_mm_s  = status.left_slewed_target_mm_s;
        report.slewed_right_mm_s = status.right_slewed_target_mm_s;
        report.drive_left_permille  = status.left_drive_permille;
        report.drive_right_permille = status.right_drive_permille;
        comms_report_motion(&report);
    }
    else
    {
        /* Motion cannot be read: treat as faulty, never as still. */
        p_in->serious_faults |= (uint16_t) VC_FAULT_MOTION_CMD;
        p_in->b_stationary    = false;
    }

    if (g_b_manoeuvre_failed)
    {
        p_in->b_motion_failed = true;
        g_b_manoeuvre_failed  = false;
    }

    if (g_b_rejected)
    {
        p_in->serious_faults |= (uint16_t) VC_FAULT_MOTION_CMD;
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

static vc_line_t
line_from_buddy3 (line_state_t state)
{
    vc_line_t line = VC_LINE_LOST;

    switch (state)
    {
        case LINE_LEFT:
            line = VC_LINE_LEFT;
            break;

        case LINE_CENTRE:
            line = VC_LINE_CENTRE;
            break;

        case LINE_RIGHT:
            line = VC_LINE_RIGHT;
            break;

        case LINE_JUNCTION:
            line = VC_LINE_JUNCTION;
            break;

        case LINE_LOST:
        default:
            line = VC_LINE_LOST;
            break;
    }

    return line;
}


static vc_nav_t
nav_from_buddy3 (nav_command_t nav)
{
    vc_nav_t out = VC_NAV_NONE;

    switch (nav)
    {
        case NAV_LEFT:
            out = VC_NAV_LEFT;
            break;

        case NAV_RIGHT:
            out = VC_NAV_RIGHT;
            break;

        case NAV_STRAIGHT:
            out = VC_NAV_STRAIGHT;
            break;

        case NAV_UTURN:
            out = VC_NAV_UTURN;
            break;

        case NAV_NONE:
        default:
            out = VC_NAV_NONE;
            break;
    }

    return out;
}


/* Buddy 3 barcode -> the character sent to the laptop. */
static char
barcode_char (barcode_t code)
{
    static char const g_chars[] = "-AFZ?";     /* NONE, A, F, Z, INVALID */
    char              c         = '?';

    if ((uint32_t) code < (sizeof(g_chars) - 1U))
    {
        c = g_chars[code];
    }

    return c;
}


/* Line telemetry and barcode events to the laptop (comms, Buddy 1). */
static void
report_line (buddy3_status_t const * p_status, vc_nav_t nav)
{
    comms_line_t report;

    report.position = (comms_line_pos_t) p_status->line_state; /* Same order. */
    report.error    = p_status->line_error;
    report.sensors  = p_status->line_sensors;
    buddy3_get_readings(&report.centre_adc, &report.right_adc);
    comms_report_line(&report);

    if (BARCODE_NONE != p_status->barcode)
    {
        (void) comms_event_barcode(barcode_char(p_status->barcode),
                                   (comms_nav_t) nav);         /* Same order. */
    }
}


/* Drain Buddy 3's queue of packed 32-bit status words: the latest line
 * wins, the first barcode command is kept.  A word that fails
 * buddy3_status_unpack() (reserved bits / codes) is ignored. */
static void
read_line (vc_inputs_t * p_in, uint32_t now)
{
    static vc_line_t s_line    = VC_LINE_LOST;
    static int8_t    s_error   = 0;
    static vc_cal_b3_t s_cal   = VC_CAL_B3_IDLE;
    static uint32_t  s_seen_ms = 0U;
    static bool      s_b_seen  = false;
    buddy3_packed_t  packed;
    buddy3_status_t  status;
    vc_nav_t         nav;
    uint32_t         i;
    bool             b_more = true;

    for (i = 0U; b_more && (i < VC_LINE_DRAIN_MAX); i++)
    {
        b_more = ((INT) sizeof(packed)
                  == tk_rcv_mbf(buddy3_output_mbfid, &packed, TMO_POL));

        if (b_more && buddy3_status_unpack(packed, &status))
        {
            s_line    = line_from_buddy3(status.line_state);
            s_error   = status.line_error;      /* Already -127..+127. */
            s_cal     = (vc_cal_b3_t) status.cal_state;     /* Same order. */
            s_seen_ms = now;
            s_b_seen  = true;
            nav       = nav_from_buddy3(status.nav_command);

            if ((VC_NAV_NONE == p_in->barcode) && (VC_NAV_NONE != nav))
            {
                p_in->barcode = nav;
            }

            report_line(&status, nav);
        }
    }

    p_in->cal_b3 = s_cal;

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
    p_in->cal_b3     = VC_CAL_B3_OK;        /* Nothing to calibrate.       */

    /* Bench safety: end the run by itself, exactly like a laptop STOP. */
    if (g_b_run_active && ((now - g_run_start_ms) >= VC_TEST_RUN_MS)
        && (VC_CMD_NONE == p_in->cmd))
    {
        p_in->cmd = VC_CMD_STOP;
        tm_printf((UB *) "[vc] test run time up: auto STOP\n");
    }
}

#endif /* VC_HAVE_BUDDY3 */


#if VC_HAVE_BUDDY4

/* A completed hump: report a valid height to the laptop, log either way. */
static void
report_hump (b4_task_terrain_t const * p_terrain)
{
    UINT const height = (UINT) p_terrain->current_height_dmm;
    UINT const highest = (UINT) p_terrain->highest_height_dmm;

    if (p_terrain->b_height_valid)
    {
        (void) comms_event_hump((int32_t) p_terrain->current_height_dmm);
        tm_printf((UB *) "[b4] hump %u.%u mm, run max %u.%u mm\n",
                  height / 10U, height % 10U, highest / 10U, highest % 10U);
    }
    else
    {
        tm_printf((UB *) "[b4] hump with no valid height, run max %u.%u mm\n",
                  highest / 10U, highest % 10U);
    }
}

/* Drain Buddy 4's terrain words (the latest wins).  The car slows unless
 * a fresh READY word shows no ramp, so a ramp, a Buddy 4 fault and silence
 * all mean "slow", never "stop".  Returns the IMU fault bit for the
 * published fault word only. */
static uint16_t
read_terrain (vc_inputs_t * p_in, uint32_t now)
{
    static b4_task_terrain_t s_terrain;
    static uint32_t          s_seen_ms = 0U;
    static bool              s_b_seen  = false;
    static uint8_t           s_humps   = 0U;
    uint32_t                 word;
    uint32_t                 i;
    bool                     b_more = true;
    bool                     b_ready;
    bool                     b_ramp;

    for (i = 0U; b_more && (i < VC_TERRAIN_DRAIN_MAX); i++)
    {
        b_more = b4_task_receive_word(&word);

        if (b_more)
        {
            b4_task_unpack(word, &s_terrain);

            /* The count, not a one-sample flag, marks a completed hump,
             * so a word lost in between cannot hide one. */
            if (s_b_seen && (s_terrain.hump_count != s_humps))
            {
                report_hump(&s_terrain);
            }

            s_humps   = s_terrain.hump_count;
            s_seen_ms = now;
            s_b_seen  = true;
        }
    }

    b_ready = s_b_seen && ((now - s_seen_ms) < VC_TERRAIN_STALE_MS)
              && (B4_HEALTH_READY == s_terrain.health);
    b_ramp  = b4_task_is_ramp_active(&s_terrain);

    p_in->b_hump          = (!b_ready) || b_ramp;
    p_in->b_terrain_ready = b_ready && (!b_ramp);
    p_in->cal_b4          = (vc_cal_b4_t) b4_task_get_calibration(NULL);
                                                        /* Same order.     */

    return b_ready ? 0U : (uint16_t) VC_FAULT_IMU;
}

#else  /* No Buddy 4: never slow for terrain, never hold START. */

static uint16_t
read_terrain (vc_inputs_t * p_in, uint32_t now)
{
    (void) now;
    p_in->b_hump          = false;
    p_in->b_terrain_ready = true;
    p_in->cal_b4          = VC_CAL_B4_DONE;     /* Nothing to calibrate.   */

    return 0U;
}

#endif /* VC_HAVE_BUDDY4 */


#if VC_HAVE_BUDDY5

/* True while the car is on a mission (may be moving). */
static bool
is_running (vc_state_t state)
{
    return (VC_STATE_FOLLOW_LINE <= state)
           && (VC_STATE_REACQUIRE_LINE >= state);
}


/* Buddy 5's event flag: OBSTACLE_EVT_AHEAD stays set until acknowledged;
 * OBSTACLE_EVT_FAULT is set while the sensor is faulty (a serious fault:
 * the car must not drive blind).  Read without clearing: only Buddy 5
 * writes these bits.
 *
 * While the car is NOT running, "ahead" is acknowledged every step and
 * ignored: otherwise a hand or wall seen while the car sat in READY stays
 * latched and stops the car the moment it is started.  A real obstacle is
 * raised again within one reading (~60 ms) after the start. */
static void
read_obstacle (vc_inputs_t * p_in, vc_state_t state)
{
    T_RFLG    rflg;
    ID const  flg = obstacle_task_get_flag();

    if ((0 < flg) && (E_OK == tk_ref_flg(flg, &rflg)))
    {
        if (is_running(state))
        {
            p_in->b_obstacle_ahead =
                (0U != (rflg.flgptn & OBSTACLE_EVT_AHEAD));
        }
        else if (0U != (rflg.flgptn & OBSTACLE_EVT_AHEAD))
        {
            obstacle_task_ack_ahead();          /* Stale: clear it. */
        }
        else
        {
            /* Nothing latched. */
        }

        if (0U != (rflg.flgptn & OBSTACLE_EVT_FAULT))
        {
            p_in->serious_faults |= (uint16_t) VC_FAULT_ULTRASONIC;
        }
    }
    else
    {
        /* No obstacle task: treat as a sensor fault, never as "clear". */
        p_in->serious_faults |= (uint16_t) VC_FAULT_ULTRASONIC;
    }
}


/* The VC has acted on "obstacle ahead" (stopped, scanning): tell Buddy 5,
 * which clears the bit and raises it again if the obstacle is still there.
 * Also report what was seen, so the laptop shows why the car stopped. */
static void
act_on_obstacle (vc_outputs_t const * p_out)
{
    obstacle_status_t status;
    comms_obstacle_t  report = {0};

    if (VC_OBST_SCAN_REQUEST == p_out->obstacle)
    {
        obstacle_task_ack_ahead();

        if (obstacle_task_get_status(&status))
        {
            report.distance_mm = (int32_t) status.distance_mm;
        }

        report.angle_deg = 90;                  /* Straight ahead.       */
        report.action    = COMMS_AVOID_STOP;    /* No scan written yet.  */
        (void) comms_event_obstacle(&report);
    }
}

#else  /* Placeholder: never an obstacle. */

static void
read_obstacle (vc_inputs_t * p_in, vc_state_t state)
{
    (void) p_in;
    (void) state;
}


static void
act_on_obstacle (vc_outputs_t const * p_out)
{
    (void) p_out;
}

#endif /* VC_HAVE_BUDDY5 */

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
    g_b_rejected     = false;
    g_drive_busy_try = 0U;
    g_hold_steps     = VC_STOP_HOLD_STEPS;
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


/* Calibration pivot: an encoder turn at Buddy 2's validated wheel speed,
 * so the inner wheel is not held forward as in a drive command.  The VC
 * stops it early when the sensors have seen enough; the angle is only the
 * safety limit.  Positive = right. */
static void
queue_pivot (int32_t angle_mdeg)
{
    motion_command_t cmd = {0};

    cmd.type                        = MOTION_COMMAND_TURN_ANGLE;
    cmd.timeout_ms                  = VC_TURN_TIMEOUT_MS;
    cmd.value.turn.wheel_speed_mm_s = VC_TURN_SPEED_MM_S;
    cmd.value.turn.angle_mdeg       = angle_mdeg;
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

    /* A change to or from 0 mm/s is always sent, however small. */
    b_changed = (!g_b_driving)
                || ((speed_mm_s != g_drive_speed)
                    && ((0 == speed_mm_s) || (0 == g_drive_speed)
                        || (VC_SPEED_RESEND
                            <= abs_i32((int32_t) speed_mm_s
                                       - (int32_t) g_drive_speed))))
                || (VC_STEER_RESEND
                    <= abs_i32((int32_t) steer - (int32_t) g_drive_steer));

    if (b_changed && (0U == g_hold_steps))
    {
        cmd.type                          = MOTION_COMMAND_SET_VELOCITY;
        cmd.sequence_id                   = next_seq();
        cmd.timeout_ms                    = 0U;
        cmd.value.velocity.linear_mm_s    = speed_mm_s;
        /* Line to the right (+) needs the agreed positive right-turn rate. */
        {
            int32_t angular  = (int32_t) steer * VC_STEER_GAIN_MDEG_S;
            int32_t lin      = (0 > speed_mm_s) ? -(int32_t) speed_mm_s
                                                : (int32_t) speed_mm_s;
            /* Cap the turn rate so the inner wheel keeps a small forward
             * speed instead of being driven to zero/reverse, where it
             * stalls at low PWM.  Only bites on hard steer; small
             * corrections pass through unchanged. */
            int32_t max_diff = lin - VC_TURN_INNER_FLOOR_MM_S;
            int32_t max_ang;
            if (0 > max_diff) { max_diff = 0; }
            max_ang = max_diff * VC_MDEGPS_PER_MM_S_DIFF;
            if (angular >  max_ang) { angular =  max_ang; }
            if (angular < -max_ang) { angular = -max_ang; }
            cmd.value.velocity.angular_mdeg_s = angular;
        }
        result = motion_submit(&cmd);

        if (MOTION_RESULT_OK == result)
        {
            g_b_driving      = true;
            g_drive_speed    = speed_mm_s;
            g_drive_steer    = steer;
            g_drive_busy_try = 0U;
        }
        else if (is_retryable(result)
                 && (VC_SUBMIT_RETRIES_DRIVE > g_drive_busy_try))
        {
            g_drive_busy_try++;                 /* Try again next step. */
        }
        else
        {
            /* Refused, or busy for too long: a fault, not a silent wait
             * (a busy Motion once left the car still with fault 0). */
            g_b_rejected     = true;
            g_drive_busy_try = 0U;
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
        else if (is_retryable(result)
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

        case VC_MOTION_PIVOT:
            queue_pivot(p_out->pivot_mdeg);
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


/* Calibration requests to Buddy 3 and Buddy 4, and the console log.  The
 * sensor tables are printed by Buddy 3 and Buddy 4 themselves. */
static void
act_on_calibration (vc_outputs_t const * p_out)
{
    static char const * const names[] = {"none", "line sensor contrast",
                                         "line not found", "Buddy 4",
                                         "sensor gap"};

    switch (p_out->cal_req)
    {
        case VC_CAL_REQ_BEGIN:
#if VC_HAVE_BUDDY3
            buddy3_calibration_request(BUDDY3_CAL_BEGIN);
#endif
#if VC_HAVE_BUDDY4
            b4_task_request_calibration(B4_TASK_CAL_LEVEL);
#endif
            break;

        case VC_CAL_REQ_SWEEP:
            g_sweep_origin_counts = g_diff_counts;
#if VC_HAVE_BUDDY3
            buddy3_calibration_request(BUDDY3_CAL_SWEEP);
#endif
#if VC_HAVE_BUDDY4
            b4_task_request_calibration(B4_TASK_CAL_MEASURE);
#endif
            break;

        case VC_CAL_REQ_JUDGE:
#if VC_HAVE_BUDDY3
            buddy3_calibration_request(BUDDY3_CAL_END);
#endif
            tm_printf((UB *) "[cal] vc both sides hit the %d mdeg bound\n",
                      (INT) VC_CAL_LIMIT_MDEG);
            break;

        case VC_CAL_REQ_FINISH:
#if VC_HAVE_BUDDY4
            b4_task_request_calibration(B4_TASK_CAL_END);
#endif
            tm_printf((UB *) "[cal] vc swept %d mdeg, centre back on"
                             " the line\n", (INT) g_vc.cal_out_mdeg);
            break;

        case VC_CAL_REQ_ABORT:
#if VC_HAVE_BUDDY3
            buddy3_calibration_request(BUDDY3_CAL_ABORT);
#endif
            break;

        case VC_CAL_REQ_NONE:
        default:
            break;
    }

    if (VC_CAL_FAIL_NONE != p_out->cal_fail)
    {
        tm_printf((UB *) "[cal] vc FAILED: %s\n",
                  (UB *) names[p_out->cal_fail]);
    }
}


/* Send the calibration result to the laptop: what Buddy 3 learned, what
 * Buddy 4 set, how far the car swept.  Queued, never blocks; a full queue
 * only loses this event (comms counts it). */
static void
publish_calibration (comms_cal_result_t result, int32_t angle_mdeg,
                     bool b_first_right)
{
    comms_calibration_t cal = {0};
    int32_t             sweep_mdeg = (0 > angle_mdeg) ? -angle_mdeg
                                                      : angle_mdeg;
#if VC_HAVE_BUDDY3
    line_cal_result_t   lines;

    line_cal_get_result(&lines);
    cal.centre_low        = lines.centre.low;
    cal.centre_high       = lines.centre.high;
    cal.centre_threshold  = lines.centre.threshold;
    cal.centre_full_black = lines.centre_full_black;
    cal.right_low         = lines.right.low;
    cal.right_high        = lines.right.high;
    cal.right_threshold   = lines.right.threshold;
    cal.barcode_low       = lines.barcode.low;
    cal.barcode_high      = lines.barcode.high;
    cal.overlap_samples   = lines.gap_measured ? lines.gap_overlap
                                               : COMMS_CAL_OVERLAP_NOT_MEASURED;
    cal.barcode_threshold = buddy3_barcode_threshold();
    cal.b_barcode_learned = (COMMS_CAL_OK == result)
                            && (0U != lines.barcode.threshold);
#else
    cal.overlap_samples   = COMMS_CAL_OVERLAP_NOT_MEASURED;
#endif
#if VC_HAVE_BUDDY4
    (void) b4_task_get_calibration(&cal.b4_floor_mg);
    cal.b4_samples = b4_task_get_calibration_samples();
#endif
    cal.result      = result;
    cal.b_first_right = b_first_right;
    cal.sweep_deg = sweep_mdeg / 1000;
    (void) comms_event_calibration(&cal);
}


static comms_cal_result_t
cal_result_of (vc_cal_fail_t why)
{
    comms_cal_result_t result = COMMS_CAL_OK;

    if (VC_CAL_FAIL_CONTRAST == why)
    {
        result = COMMS_CAL_CONTRAST;
    }
    else if (VC_CAL_FAIL_LINE == why)
    {
        result = COMMS_CAL_LINE;
    }
    else if (VC_CAL_FAIL_BUDDY4 == why)
    {
        result = COMMS_CAL_BUDDY4;
    }
    else if (VC_CAL_FAIL_GAP == why)
    {
        result = COMMS_CAL_SENSOR_GAP;
    }
    else
    {
        /* No failure. */
    }

    return result;
}


/* The fault bit that tells the laptop why a calibration failed. */
static uint16_t
cal_fault_bit (vc_cal_fail_t why)
{
    uint16_t bit = 0U;

    if (VC_CAL_FAIL_CONTRAST == why)
    {
        bit = (uint16_t) VC_FAULT_CAL_CONTRAST;
    }
    else if (VC_CAL_FAIL_LINE == why)
    {
        bit = (uint16_t) VC_FAULT_CAL_LINE;
    }
    else if (VC_CAL_FAIL_BUDDY4 == why)
    {
        bit = (uint16_t) VC_FAULT_CAL_BUDDY4;
    }
    else if (VC_CAL_FAIL_GAP == why)
    {
        bit = (uint16_t) VC_FAULT_CAL_GAP;
    }
    else
    {
        /* No failure. */
    }

    return bit;
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
#if VC_HAVE_BUDDY4
            b4_task_request_run_reset();    /* New run: clear max height. */
#endif
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


/* Every VC_TRACE_STEPS during a run: requested against measured speed, so
 * a slowdown on the ramp is shown as physical, not just commanded. */
static void
report_trace (vc_inputs_t const * p_in)
{
    static uint32_t s_steps = 0U;

    s_steps++;

    if (g_b_run_active && (VC_TRACE_STEPS <= s_steps))
    {
        s_steps = 0U;
        tm_printf((UB *) "[trace] t=%u st=%s req=%d L=%d R=%d steer=%d"
                         " slow=%d ready=%d overruns=%u\n",
                  (UINT) p_in->now_ms, (UB *) vc_state_name(g_vc.state),
                  g_b_driving ? (INT) g_drive_speed : 0,
                  (INT) g_left_mm_s, (INT) g_right_mm_s,
                  (INT) g_drive_steer, (INT) p_in->b_hump,
                  (INT) p_in->b_terrain_ready, (UINT) g_overruns);
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
    uint16_t     terrain_fault;
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
        read_command(&in);
        read_motion(&in);
        read_line(&in, now);
        read_obstacle(&in, g_vc.state);
        terrain_fault = read_terrain(&in, now);

        if ((VC_STATE_READY == g_vc.state) && (VC_CMD_START == in.cmd)
            && (!in.b_terrain_ready))
        {
            tm_printf((UB *) "[vc] START refused: Buddy 4 not ready\n");
        }

        before = g_vc.state;
        vc_step(&g_vc, &in, &out);

        act_on_motion(&out);
        act_on_obstacle(&out);
        act_on_calibration(&out);

        /* One calibration event per START, on success and on failure. */
        if (VC_CAL_FAIL_NONE != out.cal_fail)
        {
            publish_calibration(cal_result_of(out.cal_fail),
                                g_vc.cal_far_mdeg, 0 < g_vc.cal_first_dir);
        }
        else if ((VC_STATE_CALIBRATE == before)
                 && (VC_STATE_FOLLOW_LINE == out.state))
        {
            publish_calibration(COMMS_CAL_OK, g_vc.cal_out_mdeg,
                                0 < g_vc.cal_first_dir);
        }
        else
        {
            /* No calibration result this step. */
        }

        /* Faults seen during a run stay in the published word until the
         * next start, so a fault that lasted one step still shows why the
         * car stopped.  Set before the state, so the state message that
         * reports EMERGENCY_STOP already carries the reason.  While
         * idle, the IMU bit follows the live value instead. */
        if ((VC_STATE_READY == before) && (VC_STATE_READY != out.state))
        {
            g_faults_seen = 0U;                 /* New run: clean slate. */
        }

        if ((VC_STATE_INITIALISE == g_vc.state) ||
            (VC_STATE_READY == g_vc.state))
        {
            /* Idle: the IMU bit shows whether START would be accepted
             * now, so a not-ready spell at boot does not linger. Other
             * bits stay, so the reason for a stop remains visible. */
            g_faults_seen = (uint16_t) ((g_faults_seen
                                         & (uint16_t) ~VC_FAULT_IMU)
                                        | in.serious_faults
                                        | terrain_fault);
        }
        else
        {
            /* The IMU bit joins the published word only, after
             * vc_step(). */
            g_faults_seen = (uint16_t) (g_faults_seen | in.serious_faults
                                        | terrain_fault);
        }
        /* A failed calibration is back in READY: its reason stays in the
         * word until the next START clears it. */
        g_faults_seen = (uint16_t) (g_faults_seen
                                    | cal_fault_bit(out.cal_fail));
        comms_set_fault((uint32_t) g_faults_seen);

        if ((VC_STATE_EMERGENCY_STOP == out.state) && out.b_state_changed)
        {
            tm_printf((UB *) "[vc] EMERGENCY_STOP: faults=0x%03x\n",
                      (UINT) g_faults_seen);
        }

        report_state(before, &out, now);
        report_trace(&in);

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
