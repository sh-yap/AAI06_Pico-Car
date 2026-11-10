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
 *     fault from Buddy 5's task; the scan, its recommendation and the
 *     bypass plan (obstacle_plan.h).  The bypass itself is run here, step
 *     by step, while the VC is in BYPASS: turn, Buddy 5 path check, out,
 *     turn parallel, along past the obstacle, back at the same angle until
 *     Buddy 3 sees the line, onto it, turn straight; then "obstacle
 *     passed" and line following takes over;
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
#include "obstacle_config.h"
#include "obstacle_types.h"
#include "obstacle_task.h"
#include "obstacle_plan.h"
#endif

#ifndef VC_RECORDER
#define VC_RECORDER              (0)    /* Test-only run recorder, run_rec.h.*/
#endif

#if VC_RECORDER
#include "run_rec.h"
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

/* Bypass.  Turns are encoder turns at VC_TURN_SPEED_MM_S, calibrated by
 * OBSTACLE_TURN_LEFT_PCT / _RIGHT_PCT (obstacle_config.h). */
#define VC_BYPASS_SPEED_MM_S     (150)     /* Leg speed.                   */
#define VC_BYPASS_GUARD_MM       (150U)    /* Nearer on a leg = stop.      */
#define VC_BYPASS_TURN_MS        (3000U)   /* Motion timeout per turn.     */
#define VC_BYPASS_QUIET_MS       (800U)    /* Stale "ahead" ignored after
                                              the bypass (unless nearer
                                              than VC_BYPASS_GUARD_MM).    */
/* Path --/--\-- : out to side_mm, parallel past the obstacle, back at
 * back_deg (twice the way out, from the plan), then turn straight and hand
 * over to line following.  The parallel leg keeps the car as near the
 * line as the obstacle allows: the course has walls, and the earlier "^"
 * went about 36 cm out and left the course.  A 45 deg return that stopped
 * on the line left line following to swing 45 deg, which it rarely managed
 * (track tests 2026-11-02).  So as soon as Buddy 3 sees the line the car
 * drives on SNAP_MM (rear axle to the line sensors) to bring the axle over
 * the line, turns straight and hands over.  The way back is
 * RETURN_MAX_PCT % of back_mm, so with the +-10 deg turn slip
 * (2026-11-01) it still crosses the line. */
#define VC_BYPASS_RETURN_MAX_PCT (130U)
#define VC_BYPASS_SNAP_MM        (100U)
/* A line crossed on the parallel leg is the course bending towards the
 * bypass side, so its way on lies further that way: pivot that way until a
 * line sensor sees it, at most this far (track test 2026-11-02: on the
 * way out the right sensor ran along the line for 140 mm, then the
 * parallel leg crossed it; pivoting right finds the way back instead). */
#define VC_BYPASS_ATTACH_DEG     (90U)

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
static int32_t  g_manoeuvre_mm;        /* Motion's distance in its current
                                          command (bypass leg progress).  */
static bool     g_b_line_black;        /* A line sensor sees black now
                                          (not Buddy 3's last-seen side). */
static int32_t  g_target_left_mm_s;    /* Motion wheel targets, for the    */
static int32_t  g_target_right_mm_s;   /* drive word and the recorder.    */
#if VC_RECORDER
static bool     g_b_rec_active;        /* Recording: CALIBRATE to run end. */
static uint8_t  g_rec_sensors;        /* Line sensor bits, for the recorder.*/
static int32_t  g_rec_pwm_left;        /* Signed PWM applied, permille.    */
static int32_t  g_rec_pwm_right;
#endif

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
        g_target_left_mm_s      = status.left_target_mm_s;
        g_target_right_mm_s     = status.right_target_mm_s;
#if VC_RECORDER
        g_rec_pwm_left          = status.left_drive_permille;
        g_rec_pwm_right         = status.right_drive_permille;
#endif
        g_manoeuvre_mm          = status.travelled_mm;
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
    static char const g_chars[] = "-AFZYX?";   /* NONE, A, F, Z, Y, X, INVALID */
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
    static uint8_t   s_sensors = 0U;
    static bool      s_capture = false;
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
            s_sensors = status.line_sensors;
            s_capture = status.b_barcode_capture;
            s_error   = status.line_error;      /* Already -127..+127. */
            s_cal     = (vc_cal_b3_t) status.cal_state;     /* Same order. */
            s_seen_ms = now;
            s_b_seen  = true;
            g_b_line_black = (0U != status.line_sensors);
#if VC_RECORDER
            g_rec_sensors  = status.line_sensors;
#endif
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
        p_in->line_sensors = s_sensors;
        p_in->b_barcode_capture = s_capture;
    }
    else
    {
        /* Buddy 3 silent: never steer on old data. */
        p_in->line       = VC_LINE_LOST;
        p_in->line_error = 0;
        p_in->line_sensors = 0U;
        p_in->b_barcode_capture = false;
        g_b_line_black   = false;
#if VC_RECORDER
        g_rec_sensors    = 0U;
#endif
    }
}

#else  /* Placeholder until Buddy 3 is merged. */

static void
read_line (vc_inputs_t * p_in, uint32_t now)
{
    p_in->line       = VC_LINE_CENTRE;
    p_in->line_error = 0;
    p_in->line_sensors = VC_LINE_SENSOR_CENTRE;
    p_in->b_barcode_capture = false;
    p_in->cal_b3     = VC_CAL_B3_OK;        /* Nothing to calibrate.       */
    g_b_line_black   = true;

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

/* A run reset was requested and Buddy 4's hump count has not shown 0 yet,
 * so that drop is not a finished hump.  Set where the reset is requested,
 * cleared in read_terrain(); both run in the VC task. */
static bool s_b_hump_reset_pending = false;


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
            if (s_b_hump_reset_pending && (0U == s_terrain.hump_count))
            {
                s_b_hump_reset_pending = false;     /* Reset, not a hump. */
            }
            else if (s_b_seen && (s_terrain.hump_count != s_humps))
            {
                report_hump(&s_terrain);
#if VC_RECORDER
                ev_rec_hump(now, s_terrain.current_height_dmm,
                            s_terrain.hump_count, s_terrain.b_height_valid);
#endif
            }
#if VC_RECORDER
            ev_rec_max(now, s_terrain.highest_height_dmm);
#endif

            s_humps   = s_terrain.hump_count;
            s_seen_ms = now;
            s_b_seen  = true;
        }
    }

    b_ready = s_b_seen && ((now - s_seen_ms) < VC_TERRAIN_STALE_MS)
              && (B4_HEALTH_READY == s_terrain.health);
    b_ramp  = b4_task_is_ramp_active(&s_terrain);

    p_in->b_hump          = (!b_ready) || b_ramp;
    p_in->b_ramp_active   = b_ready && b_ramp;
    /* A stale or unhealthy terrain word is not a lift cue. */
    p_in->b_imu_cue       = b_ready && s_terrain.b_lift_cue;
    p_in->b_terrain_ready = b_ready && (!b_ramp);
    p_in->cal_b4          = (vc_cal_b4_t) b4_task_get_calibration(NULL);
                                                        /* Same order.     */

    return b_ready ? 0U : (uint16_t) VC_FAULT_IMU;
}

/* True while the car is on a mission that drives it: it follows the line,
 * handles a junction, turns, reacquires the line or bypasses an obstacle.
 * Calibration, scanning, stops and idle states are not driving. */
static bool
is_driving (vc_state_t state)
{
    return (VC_STATE_FOLLOW_LINE == state)
           || (VC_STATE_HANDLE_JUNCTION == state)
           || (VC_STATE_TURNING == state)
           || (VC_STATE_REACQUIRE_LINE == state)
           || (VC_STATE_BYPASS == state);
}


/* One wheel speed as an 11-bit two's complement field, saturated. */
static uint32_t
pack_speed (int32_t speed_mm_s, uint32_t shift)
{
    int32_t speed = speed_mm_s;

    if (B4_TASK_DRIVE_SPEED_LIMIT < speed)
    {
        speed = B4_TASK_DRIVE_SPEED_LIMIT;
    }
    else if (-B4_TASK_DRIVE_SPEED_LIMIT > speed)
    {
        speed = -B4_TASK_DRIVE_SPEED_LIMIT;
    }
    else
    {
        /* Already within the field. */
    }

    /* The speed is within +-1023; the conversion to unsigned wraps modulo
     * 2^32 by definition and the mask keeps the low 11 bits, which is the
     * two's complement field. */
    return (((uint32_t) speed) & B4_TASK_DRIVE_SPEED_MASK) << shift;
}


/* Hand Buddy 4 the drive word (layout in b4_task.h), once per VC cycle.
 * Buddy 4 learns its noise only while the word says the car is driving and
 * moving, and treats a word whose sequence stops changing as stale. The
 * step field is wire data only: it is the larger target change since the
 * previous publish, in 8 mm/s units, and Buddy 4 decides what is a jolt. */
static void
publish_drive (vc_state_t state, bool b_stationary)
{
    static uint32_t s_seq = 0U;
    static int32_t  s_target_left_mm_s = 0;
    static int32_t  s_target_right_mm_s = 0;
    static bool     s_have_target = false;
    uint32_t        word  = 0U;
    uint32_t        step  = 0U;
    int64_t         delta_left = 0;
    int64_t         delta_right = 0;

    if (is_driving(state))
    {
        word |= B4_TASK_DRIVE_DRIVING_BIT;
    }

    if (!b_stationary)
    {
        word |= B4_TASK_DRIVE_MOVING_BIT;
    }

    if (true == s_have_target)
    {
        /* The targets are bounded Motion speeds, so the 64-bit difference
         * is exact and its magnitude fits the step field after scaling. */
        delta_left = (int64_t) g_target_left_mm_s -
                     (int64_t) s_target_left_mm_s;
        delta_right = (int64_t) g_target_right_mm_s -
                      (int64_t) s_target_right_mm_s;
        if (delta_left < 0)
        {
            delta_left = -delta_left;
        }
        if (delta_right < 0)
        {
            delta_right = -delta_right;
        }
        if (delta_right > delta_left)
        {
            delta_left = delta_right;
        }
        step = (uint32_t) (delta_left / (int64_t) B4_TASK_DRIVE_STEP_MM_S);
        if (step > B4_TASK_DRIVE_STEP_LIMIT)
        {
            step = B4_TASK_DRIVE_STEP_LIMIT;
        }
    }
    s_target_left_mm_s  = g_target_left_mm_s;
    s_target_right_mm_s = g_target_right_mm_s;
    s_have_target = true;

    word |= pack_speed(g_left_mm_s, B4_TASK_DRIVE_LEFT_SHIFT);
    word |= pack_speed(g_right_mm_s, B4_TASK_DRIVE_RIGHT_SHIFT);
    word |= (s_seq & B4_TASK_DRIVE_SEQ_MASK) << B4_TASK_DRIVE_SEQ_SHIFT;
    word |= (step & B4_TASK_DRIVE_STEP_MASK) << B4_TASK_DRIVE_STEP_SHIFT;
    s_seq++;                                /* Wraps; only 4 bits are sent. */

    b4_task_set_drive(word);
}

#else  /* No Buddy 4: never slow for terrain, never hold START. */

static uint16_t
read_terrain (vc_inputs_t * p_in, uint32_t now)
{
    (void) now;
    p_in->b_hump          = false;
    p_in->b_ramp_active   = false;
    p_in->b_terrain_ready = true;
    p_in->cal_b4          = VC_CAL_B4_DONE;     /* Nothing to calibrate.   */

    return 0U;
}

static void
publish_drive (vc_state_t state, bool b_stationary)
{
    (void) state;
    (void) b_stationary;
}

#endif /* VC_HAVE_BUDDY4 */


#if VC_HAVE_BUDDY5

/* A Buddy 5 request (scan or path check) in progress.  Its DONE bit is
 * cleared when Buddy 5 starts the work and set when the result is ready,
 * so a DONE left over from the previous request is never taken as new. */
typedef enum
{
    OBST_REQ_IDLE = 0,
    OBST_REQ_SENT,                      /* Asked; old DONE may still show. */
    OBST_REQ_RUNNING                    /* DONE seen clear: work started.  */
} obst_req_t;

static obst_req_t             g_scan_req;
static obstacle_profile_t     g_profile;    /* Latest scan.              */
static obstacle_bypass_plan_t g_plan;       /* For TURN recommendations. */
static bool                   g_b_ahead_quiet;  /* See read_obstacle(). */
static uint32_t               g_ahead_quiet_until_ms;


/* True if Buddy 5's latest reading is within the bypass guard distance. */
static bool
is_ahead_close (void)
{
    obstacle_status_t status;

    return obstacle_task_get_status(&status)
           && (OBSTACLE_RANGE_OK == status.range_status)
           && (VC_BYPASS_GUARD_MM > status.distance_mm);
}


/* The bypass has ended: ignore a stale "ahead" for VC_BYPASS_QUIET_MS. */
static void
start_ahead_quiet (uint32_t now)
{
    obstacle_task_forget();
    obstacle_task_ack_ahead();
    g_b_ahead_quiet        = true;
    g_ahead_quiet_until_ms = now + VC_BYPASS_QUIET_MS;
}


/* Advance a request on its DONE bit; true once, when the result is ready. */
static bool
request_done (obst_req_t * p_req, UINT flags, UINT done_bit)
{
    bool b_done = false;

    if ((OBST_REQ_SENT == *p_req) && (0U == (flags & done_bit)))
    {
        *p_req = OBST_REQ_RUNNING;
    }
    else if ((OBST_REQ_RUNNING == *p_req) && (0U != (flags & done_bit)))
    {
        *p_req = OBST_REQ_IDLE;
        b_done = true;
    }
    else
    {
        /* Waiting. */
    }

    return b_done;
}


static vc_avoid_t
avoid_from_buddy5 (obstacle_action_t action)
{
    vc_avoid_t avoid = VC_AVOID_STOP;

    switch (action)
    {
        case OBSTACLE_ACTION_CONTINUE:
            avoid = VC_AVOID_CONTINUE;
            break;

        case OBSTACLE_ACTION_TURN_LEFT:
            avoid = VC_AVOID_TURN_LEFT;
            break;

        case OBSTACLE_ACTION_TURN_RIGHT:
            avoid = VC_AVOID_TURN_RIGHT;
            break;

        case OBSTACLE_ACTION_REVERSE_RETRY:
            avoid = VC_AVOID_REVERSE_RETRY;
            break;

        case OBSTACLE_ACTION_STOP:
        default:
            avoid = VC_AVOID_STOP;
            break;
    }

    return avoid;
}


/* Scan finished: read the profile, plan the bypass, give the VC its
 * recommendation, and tell the laptop what was seen. */
static void
scan_finished (vc_inputs_t * p_in)
{
    comms_obstacle_t report = {0};
    vc_avoid_t       avoid  = VC_AVOID_STOP;    /* No result: fail safe. */

    if (obstacle_task_get_profile(&g_profile))
    {
        avoid = avoid_from_buddy5(g_profile.action);

        if (((VC_AVOID_TURN_LEFT == avoid) || (VC_AVOID_TURN_RIGHT == avoid))
            && (!obstacle_plan_bypass(g_profile.distance_mm, g_profile.action,
                                      &g_plan)))
        {
            /* Too close for any allowed turn: back off and scan again. */
            avoid = VC_AVOID_REVERSE_RETRY;
        }

        report.distance_mm    = (int32_t) g_profile.distance_mm;
        report.angle_deg      = (int32_t) g_profile.angle_deg;
        report.width_mm       = (int32_t) g_profile.width_mm;
        report.clear_left_mm  = (int32_t) g_profile.clear_left_mm;
        report.clear_right_mm = (int32_t) g_profile.clear_right_mm;
        report.action         = (comms_avoid_t) avoid;      /* Same order. */
        (void) comms_event_obstacle(&report);

        tm_printf((UB *) "[vc] scan: closest %u mm, recommendation %d\n",
                  (UINT) g_profile.distance_mm, (INT) avoid);
    }

    p_in->b_scan_done    = true;
    p_in->recommendation = avoid;
}


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
        if (g_b_ahead_quiet
            && ((int32_t) (p_in->now_ms - g_ahead_quiet_until_ms) >= 0))
        {
            g_b_ahead_quiet = false;
        }

        if (is_running(state) && g_b_ahead_quiet && (!is_ahead_close()))
        {
            /* Just after a bypass: "ahead" was latched while passing the
             * obstacle (track test 2026-11-02 started a second bypass from
             * it).  Keep clearing it; a real obstacle is raised again. */
            if (0U != (rflg.flgptn & OBSTACLE_EVT_AHEAD))
            {
                obstacle_task_ack_ahead();
            }
        }
        else if (is_running(state))
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

        if (request_done(&g_scan_req, rflg.flgptn, OBSTACLE_EVT_SCAN_DONE))
        {
            scan_finished(p_in);
        }
    }
    else
    {
        /* No obstacle task: treat as a sensor fault, never as "clear". */
        p_in->serious_faults |= (uint16_t) VC_FAULT_ULTRASONIC;
    }
}


/* The VC has acted on "obstacle ahead" (stopped, will scan): tell Buddy 5,
 * which clears the bit and raises it again if the obstacle is still there,
 * and ask for the scan.  The result is picked up in read_obstacle(). */
static void
act_on_obstacle (vc_outputs_t const * p_out)
{
    if (VC_OBST_SCAN_REQUEST == p_out->obstacle)
    {
        obstacle_task_ack_ahead();
        obstacle_task_request_scan();
        g_scan_req = OBST_REQ_SENT;
    }
    else if (VC_OBST_ABORT == p_out->obstacle)
    {
        g_scan_req = OBST_REQ_IDLE;             /* Ignore a late result. */
    }
    else
    {
        /* BYPASS_START: the bypass is run by bypass_run(). */
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


#if VC_HAVE_BUDDY5

/* ------------------------------------------------------------------------ *
 * Bypass ("^"), run while the VC is in BYPASS
 * ------------------------------------------------------------------------ */

typedef enum
{
    BYPASS_IDLE = 0,
    BYPASS_TURN_OUT,        /* Turn turn_deg towards the bypass side.     */
    BYPASS_CHECK,           /* Buddy 5 path check along leg 1.            */
    BYPASS_LEG_OUT,         /* Diagonal out to side_mm.                   */
    BYPASS_TURN_ALONG,      /* Turn back turn_deg: parallel to the line.  */
    BYPASS_LEG_ALONG,       /* Parallel, past the obstacle.               */
    BYPASS_TURN_BACK,       /* Turn back_deg towards the line.            */
    BYPASS_LEG_BACK,        /* Back towards the line, watching for it.    */
    BYPASS_SNAP,            /* Line seen: on until the axle is over it.   */
    BYPASS_TURN_STRAIGHT,   /* Turn back_deg to face along the line.      */
    BYPASS_ATTACH           /* Line crossed on the parallel leg: pivot
                               towards the bypass side onto it.           */
} bypass_step_t;

static bypass_step_t         g_bypass_step;
static obst_req_t            g_path_req;
static obstacle_line_watch_t g_line_watch;   /* Parallel leg only. */


static obstacle_action_t
other_side (obstacle_action_t side)
{
    return (OBSTACLE_ACTION_TURN_LEFT == side) ? OBSTACLE_ACTION_TURN_RIGHT
                                               : OBSTACLE_ACTION_TURN_LEFT;
}


/* Encoder turn of deg towards side, calibrated per direction
 * (OBSTACLE_TURN_LEFT_PCT / _RIGHT_PCT). */
static void
queue_bypass_turn (obstacle_action_t side, uint32_t deg)
{
    motion_command_t cmd = {0};
    int32_t          mdeg;
    uint32_t const   pct = (OBSTACLE_ACTION_TURN_LEFT == side)
                           ? OBSTACLE_TURN_LEFT_PCT : OBSTACLE_TURN_RIGHT_PCT;

    /* deg is at most VC_BYPASS_ATTACH_DEG, so this fits easily. */
    mdeg = (int32_t) ((deg * 1000U * pct) / 100U);

    cmd.type                        = MOTION_COMMAND_TURN_ANGLE;
    cmd.timeout_ms                  = VC_BYPASS_TURN_MS;
    cmd.value.turn.wheel_speed_mm_s = VC_TURN_SPEED_MM_S;
    /* Motion: positive angle = right turn. */
    cmd.value.turn.angle_mdeg = (OBSTACLE_ACTION_TURN_LEFT == side) ? -mdeg
                                                                    : mdeg;
    queue_manoeuvre(&cmd);
}


/* One straight leg of length_mm. */
static void
queue_bypass_leg (uint32_t length_mm)
{
    motion_command_t cmd = {0};

    cmd.type                       = MOTION_COMMAND_MOVE_DISTANCE;
    /* Twice the driving time, plus 1 s for starting and stopping. */
    cmd.timeout_ms                 = ((length_mm * 2000U)
                                      / (uint32_t) VC_BYPASS_SPEED_MM_S)
                                     + 1000U;
    cmd.value.distance.distance_mm = (int32_t) length_mm;
    cmd.value.distance.speed_mm_s  = VC_BYPASS_SPEED_MM_S;
    queue_manoeuvre(&cmd);
}


/* VC_MOTION_BYPASS: start the sequence with the first turn. */
static void
bypass_start (void)
{
    g_bypass_step = BYPASS_TURN_OUT;
    g_path_req    = OBST_REQ_IDLE;
    queue_bypass_turn(g_plan.side, g_plan.turn_deg);
    tm_printf((UB *) "[vc] bypass %s: turn %u deg, out %u mm, along %u mm\n",
              (UB *) ((OBSTACLE_ACTION_TURN_LEFT == g_plan.side) ? "LEFT"
                                                                 : "RIGHT"),
              (UINT) g_plan.turn_deg, (UINT) g_plan.out_mm,
              (UINT) g_plan.along_mm);
}


/* True when Buddy 3 sees the line on the way back. */
static bool
is_line_back (vc_inputs_t const * p_in)
{
    return (0U != g_manoeuvre_seq) && (BYPASS_LEG_BACK == g_bypass_step)
           && (VC_LINE_LOST != p_in->line);
}


/* A line on the second half of the parallel leg (past most of the
 * obstacle; it is there when the car stopped off the line, the turns
 * slipped, or the course bends - track tests 2026-11-02).  HELD: the car
 * runs along it.  CROSSED: handing over there lost it (same day), so the
 * bypass attaches to it instead. */
static obstacle_line_seen_t
watch_line_along (void)
{
    obstacle_line_seen_t seen = OBSTACLE_LINE_NONE;

    if ((0U != g_manoeuvre_seq) && (BYPASS_LEG_ALONG == g_bypass_step)
        && (g_manoeuvre_mm >= (int32_t) (g_plan.along_mm / 2U)))
    {
        seen = obstacle_plan_line_watch(&g_line_watch, g_b_line_black,
                                        g_manoeuvre_mm);
    }

    return (seen);
}


/* True when the attach pivot has brought a line sensor onto the line. */
static bool
is_line_attached (void)
{
    return (0U != g_manoeuvre_seq) && (BYPASS_ATTACH == g_bypass_step)
           && g_b_line_black;
}


/* The car faces along the line again: hand over to line following. */
static void
bypass_finish (vc_inputs_t * p_in)
{
    start_ahead_quiet(p_in->now_ms);
    g_bypass_step           = BYPASS_IDLE;
    p_in->b_obstacle_passed = true;
}


/* Each step, before vc_step(): move the bypass on when Motion finishes a
 * step or Buddy 5's path check is ready.  Ends with "obstacle passed" -
 * after snapping onto the line if Buddy 3 sees it on the way back;
 * a refused path, or something close on a leg, is reported as a motion
 * failure, which the VC turns into EMERGENCY_STOP. */
static void
bypass_run (vc_inputs_t * p_in)
{
    T_RFLG               rflg;
    obstacle_path_t      path;
    obstacle_line_seen_t along;
    UINT                 flags = 0U;
    bool                 b_fail = false;

    if (VC_STATE_BYPASS != g_vc.state)
    {
        g_bypass_step = BYPASS_IDLE;            /* Left BYPASS: stand down. */
    }
    else if (BYPASS_IDLE != g_bypass_step)
    {
        if (E_OK == tk_ref_flg(obstacle_task_get_flag(), &rflg))
        {
            flags = rflg.flgptn;
        }

        along = watch_line_along();

        if (((BYPASS_LEG_OUT == g_bypass_step)
             || (BYPASS_LEG_ALONG == g_bypass_step)
             || (BYPASS_LEG_BACK == g_bypass_step)
             || (BYPASS_SNAP == g_bypass_step)) && is_ahead_close())
        {
            tm_printf((UB *) "[vc] bypass: something within %u mm\n",
                      (UINT) VC_BYPASS_GUARD_MM);
            b_fail = true;
        }
        else if (OBSTACLE_LINE_HELD == along)
        {
            /* Already facing along the line and on it: no more turns. */
            stop_motion();
            bypass_finish(p_in);
            tm_printf((UB *) "[vc] bypass: on the line early: following it\n");
        }
        else if (OBSTACLE_LINE_CROSSED == along)
        {
            queue_bypass_turn(g_plan.side, VC_BYPASS_ATTACH_DEG);
            g_bypass_step = BYPASS_ATTACH;
            tm_printf((UB *) "[vc] bypass: line crossed at %d mm: "
                      "pivoting onto it\n", (INT) g_manoeuvre_mm);
        }
        else if (is_line_attached())
        {
            stop_motion();
            bypass_finish(p_in);
            tm_printf((UB *) "[vc] bypass: attached to the line: "
                      "following it\n");
        }
        else if (is_line_back(p_in))
        {
            /* The line sensors are SNAP_MM ahead of the axle: drive that
             * far so the axle is over the line, then turn straight. */
            queue_bypass_leg(VC_BYPASS_SNAP_MM);
            g_bypass_step = BYPASS_SNAP;
            tm_printf((UB *) "[vc] bypass: line found\n");
        }
        else
        {
            switch (g_bypass_step)
            {
                case BYPASS_TURN_OUT:
                    if (p_in->b_motion_done)
                    {
                        obstacle_task_request_path_check(g_plan.side,
                                                         g_plan.check_mm,
                                                         g_profile.distance_mm);
                        g_path_req    = OBST_REQ_SENT;
                        g_bypass_step = BYPASS_CHECK;
                    }
                    break;

                case BYPASS_CHECK:
                    if (request_done(&g_path_req, flags,
                                     OBSTACLE_EVT_PATH_DONE))
                    {
                        /* OPEN (no echo) is accepted: MONITOR and the
                         * guard watch the leg. */
                        if (obstacle_task_get_path(&path)
                            && ((OBSTACLE_PATH_CLEAR == path.verdict)
                                || (OBSTACLE_PATH_OPEN == path.verdict)))
                        {
                            queue_bypass_leg(g_plan.out_mm);
                            g_bypass_step = BYPASS_LEG_OUT;
                        }
                        else
                        {
                            tm_printf((UB *) "[vc] bypass: path refused\n");
                            b_fail = true;
                        }
                    }
                    break;

                case BYPASS_LEG_OUT:
                    if (p_in->b_motion_done)
                    {
                        queue_bypass_turn(other_side(g_plan.side),
                                          g_plan.turn_deg);
                        g_bypass_step = BYPASS_TURN_ALONG;
                    }
                    break;

                case BYPASS_TURN_ALONG:
                    if (p_in->b_motion_done)
                    {
                        queue_bypass_leg(g_plan.along_mm);
                        obstacle_plan_line_reset(&g_line_watch);
                        g_bypass_step = BYPASS_LEG_ALONG;
                    }
                    break;

                case BYPASS_LEG_ALONG:
                    if (p_in->b_motion_done && g_b_line_black)
                    {
                        /* Stopped on a line at the end of the leg: follow
                         * it, rather than turn back off it (track test
                         * 2026-11-02: both sensors black at 397 of 397 mm,
                         * then the turn back swept them off it). */
                        bypass_finish(p_in);
                        tm_printf((UB *) "[vc] bypass: on a line at the "
                                  "end of the parallel leg: following it\n");
                    }
                    else if (p_in->b_motion_done)
                    {
                        queue_bypass_turn(other_side(g_plan.side),
                                          g_plan.back_deg);
                        g_bypass_step = BYPASS_TURN_BACK;
                    }
                    else
                    {
                        /* Still driving the leg. */
                    }
                    break;

                case BYPASS_TURN_BACK:
                    if (p_in->b_motion_done)
                    {
                        queue_bypass_leg((g_plan.back_mm
                                          * VC_BYPASS_RETURN_MAX_PCT) / 100U);
                        g_bypass_step = BYPASS_LEG_BACK;
                    }
                    break;

                case BYPASS_LEG_BACK:
                    if (p_in->b_motion_done)
                    {
                        /* No line seen: by the turns the car is near it;
                         * turn straight and let line following search. */
                        tm_printf((UB *) "[vc] bypass: no line seen\n");
                        queue_bypass_turn(g_plan.side, g_plan.back_deg);
                        g_bypass_step = BYPASS_TURN_STRAIGHT;
                    }
                    break;

                case BYPASS_SNAP:
                    if (p_in->b_motion_done)
                    {
                        queue_bypass_turn(g_plan.side, g_plan.back_deg);
                        g_bypass_step = BYPASS_TURN_STRAIGHT;
                    }
                    break;

                case BYPASS_TURN_STRAIGHT:
                    if (p_in->b_motion_done)
                    {
                        bypass_finish(p_in);
                        tm_printf((UB *) "[vc] bypass done: line following\n");
                    }
                    break;

                case BYPASS_ATTACH:
                    if (p_in->b_motion_done)
                    {
                        /* No line within the pivot: let line following
                         * search for it. */
                        bypass_finish(p_in);
                        tm_printf((UB *) "[vc] bypass: no line on the "
                                  "attach pivot\n");
                    }
                    break;

                case BYPASS_IDLE:
                default:
                    break;
            }
        }

        if (b_fail)
        {
            stop_motion();
            g_bypass_step         = BYPASS_IDLE;
            p_in->b_motion_failed = true;       /* VC -> EMERGENCY_STOP. */
        }
    }
    else
    {
        /* In BYPASS but not started (or already ended): nothing to do. */
    }
}

#else  /* Placeholder: no Buddy 5, no bypass. */

static void
bypass_start (void)
{
    /* Stop; the VC's bypass timeout then ends in EMERGENCY_STOP. */
    stop_motion();
    tm_printf((UB *) "[vc] bypass not available: stopped\n");
}


static void
bypass_run (vc_inputs_t * p_in)
{
    (void) p_in;
}

#endif /* VC_HAVE_BUDDY5 */


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
            bypass_start();
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

#if VC_RECORDER
        /* Recording starts with the calibration and ends with the run.  A
         * run entered without a calibration is still recorded. */
        if ((VC_STATE_CALIBRATE == to)
            || ((VC_STATE_FOLLOW_LINE == to) && (!g_b_rec_active)))
        {
            run_rec_begin();
            g_b_rec_active = true;
        }
        else if ((VC_STATE_READY == to)
                 || (VC_STATE_EMERGENCY_STOP == to)
                 || (VC_STATE_FINISHED == to))
        {
            g_b_rec_active = false;
        }
        else
        {
            /* Recording continues. */
        }
#endif

        if ((VC_STATE_FOLLOW_LINE == to) && (!g_b_run_active))
        {
            g_b_run_active = true;
            g_run_start_ms = now;
            (void) comms_begin_run();
#if VC_HAVE_BUDDY4
            b4_task_request_run_reset();    /* New run: clear max and count. */
            s_b_hump_reset_pending = true;
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

#if VC_RECORDER

/* Test-only: limit a signed value to the int16_t range. */
static int16_t
clamp_i16 (int32_t value)
{
    int16_t result = (int16_t) value;

    if (value > INT16_MAX)
    {
        result = INT16_MAX;
    }
    else if (value < INT16_MIN)
    {
        result = INT16_MIN;
    }
    else
    {
        /* In range. */
    }

    return result;
}


/* Test-only: copy this cycle's values into the run recorder.  Called after
 * report_state(), so the cycle that enters CALIBRATE is the first record and
 * the cycle that ends the run (or a failed calibration) is the last, then the
 * ring freezes.  Idle cycles are ignored by the frozen ring.  Reads only
 * values the step already has. */
static void
record_cycle (vc_inputs_t const * p_in, vc_outputs_t const * p_out)
{
    run_rec_record_t rec = {0};

    rec.ms                = p_in->now_ms;
#if VC_HAVE_BUDDY3
    buddy3_get_readings(&rec.centre_adc, &rec.right_adc);
#endif
    rec.line_state        = (uint8_t) p_in->line;
    rec.line_error        = p_in->line_error;
    rec.sensors           = g_rec_sensors;
    rec.vc_state          = (uint8_t) p_out->state;
    rec.steer             = p_out->steer;
    rec.flags             = p_in->b_ramp_active ? RUN_REC_FLAG_RAMP : 0U;
    rec.speed_mm_s        = p_out->speed_mm_s;
    rec.target_left_mm_s  = (int16_t) g_target_left_mm_s;
    rec.target_right_mm_s = (int16_t) g_target_right_mm_s;
    rec.left_mm_s         = (int16_t) g_left_mm_s;
    rec.right_mm_s        = (int16_t) g_right_mm_s;
    rec.overruns          = (uint16_t) g_overruns;
    rec.left_pwm_permille  = clamp_i16(g_rec_pwm_left);
    rec.right_pwm_permille = clamp_i16(g_rec_pwm_right);
    motion_get_pid_integral_permille(&rec.left_i_permille,
                                     &rec.right_i_permille);
    motion_get_learned(&rec.learn_fwd_gain, &rec.learn_fwd_offset,
                       &rec.learn_turn_gain, &rec.learn_state);
    run_rec_add(&rec);
    ev_rec_step(p_in->now_ms, (VC_LINE_LOST == p_in->line),
                (VC_STATE_FOLLOW_LINE == p_out->state), p_out->steer,
                p_in->b_ramp_active);

    if (!g_b_rec_active)
    {
        run_rec_freeze();
    }
}

#endif /* VC_RECORDER */

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
        bypass_run(&in);
        terrain_fault = read_terrain(&in, now);

        if (((VC_STATE_READY == g_vc.state)
             || (VC_STATE_FINISHED == g_vc.state))
            && (VC_CMD_START == in.cmd) && (!in.b_terrain_ready))
        {
            tm_printf((UB *) "[vc] START refused: Buddy 4 not ready\n");
        }

        before = g_vc.state;
        vc_step(&g_vc, &in, &out);
        publish_drive(out.state, in.b_stationary);

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
         * idle, the IMU bit follows the live value instead.  During
         * calibration it is not published. */
        if (((VC_STATE_READY == before) && (VC_STATE_READY != out.state))
            || ((VC_STATE_FINISHED == before)
                && (VC_STATE_CALIBRATE == out.state)))
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
        else if (VC_STATE_CALIBRATE == g_vc.state)
        {
            /* START makes Buddy 4 re-level, which briefly makes it not
             * ready on purpose, so the IMU bit is left out here.  A real
             * failure is reported by VC_FAULT_CAL_BUDDY4 (cal_fail).
             * Calibration ends only with Buddy 4 ready, so an IMU bit
             * seen later is a real fault. */
            g_faults_seen = (uint16_t) ((g_faults_seen
                                         & (uint16_t) ~VC_FAULT_IMU)
                                        | in.serious_faults);
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

        if (true == out.b_lift_confirmed)
        {
            tm_printf((UB *) "[vc] LIFTED: run finished\n");
#if VC_RECORDER
            ev_rec_lift(now, (uint16_t) g_vc.lift_spell_cycles);
#endif
            (void) comms_event_motion(COMMS_MOTION_LIFTED);
        }

        report_state(before, &out, now);
        report_trace(&in);
#if VC_RECORDER
        record_cycle(&in, &out);
#endif

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
