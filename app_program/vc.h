/** @file vc.h
 *
 * @brief Vehicle Controller (VC): mission state machine.
 *
 * The VC decides what the car does next.  This module is pure logic: no
 * RTOS calls, no hardware access.  Each 20 ms the VC task gathers the
 * latest subsystem reports into a vc_inputs_t, calls vc_step(), and turns
 * the returned vc_outputs_t into calls to Buddy 2 (motion), Buddy 5
 * (obstacle) and Buddy 1 (comms).  Because it is pure, it can be tested on
 * the host with simulated inputs (tests/vc).
 *
 * States and transitions follow the system-level statechart in the
 * Vehicle Controller design.  Enum values here are the VC's own; mapping
 * from each buddy's codes happens in the task wrapper, using the agreed
 * shared data definitions (docs/DATA_DEFINITIONS.md).
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h> and <stdbool.h>
 * before this header.
 */

#ifndef VC_H
#define VC_H

/* ------------------------------------------------------------------------ *
 * Tunable values.  All are PROPOSED; see the VC design rules table.
 * ------------------------------------------------------------------------ */

#define VC_PERIOD_MS            (20U)    /* Matches Motion's 20 ms loop.     */
#define VC_INIT_TIMEOUT_MS      (10000U) /* TBC: longest subsystem start-up. */
#define VC_SPEED_NORMAL_MM_S    (150)    /* On the flat line.  Set to 150 by
                                            the user on 30 Oct 2026 to cover
                                            the course faster.  Was 70 (team-
                                            mate, fewer lost left bends) before
                                            Motion's low-speed tracking and
                                            the pivot line search.  Full steer
                                            still slows to CURVE.            */
#define VC_SPEED_CURVE_MM_S     (50)     /* At full steer: slow on a bend so
                                            the car stays on it.  Matches
                                            Buddy 2's validated slow speed.  */
#define VC_SPEED_HUMP_MM_S      (100)    /* Ramp speed.  All ramp evidence is
                                            at 100 and 50 stalled on the
                                            slope, so the car slows from
                                            cruise on a ramp.                */
#define VC_SPEED_SEARCH_MM_S    (120)    /* While searching.  Tried 60 (30 Oct)
                                            for a tighter pivot, but at 60 the
                                            search was too slow to reach the
                                            line on a big loss and timed out
                                            -> SAFE_STOP (two 4 s timeouts).
                                            120 covers the ground to re-find
                                            it; the wide-loop case is the
                                            2-sensor limit, fixed properly by
                                            the straddle sensor, not here.   */
#define VC_SEARCH_STEER         (127)    /* Full steer towards the side the
                                            line was last seen on.           */
#define VC_TURN_TIMEOUT_MS      (8000U)  /* Buddy 2: safe until Stage 7
                                            timing; 3000 too short at 80.    */
#define VC_SCAN_TIMEOUT_MS      (3000U)  /* 1.5 s scan budget (A7) x 2.      */
#define VC_BYPASS_TIMEOUT_MS    (6000U)  /* 3 s bypass target x 2.           */
#define VC_REACQUIRE_TIMEOUT_MS (4000U)  /* TBC: from bypass geometry.       */
#define VC_MAX_RESCANS          (2U)     /* Reverse + rescan attempts.       */

/* After a junction taken straight across, hold steer 0 for this far.  It is
 * twice the 18 mm line width in the course Barcode and Line Specification,
 * so a crossing approached up to 60 degrees off square (which looks up to
 * twice as wide) is fully passed before normal steering resumes. */
#define VC_JUNCTION_HOLD_MM     (36)

/* Line lost while following: stop, then pivot in place to find it again,
 * towards the side it was last seen on first, then the other side.  The
 * pivot is the same TURN_ANGLE as the calibration sweep, at 80 mm/s. */
#define VC_SEARCH_LIMIT_MDEG    (90000)  /* Safety bound per side, measured
                                            from the heading where the line
                                            was lost: Buddy 2's validated
                                            90 degree turn.  The line was
                                            seen within the last 300 ms, so
                                            it lies within a quarter turn on
                                            the last-seen side, and the
                                            second side covers a wrong guess.
                                            It is a bound, not a stop point:
                                            the search stops on what the
                                            sensors see.  Both sides without
                                            the line is a failure.          */
#define VC_SEARCH_TIMEOUT_MS    (9000U)  /* Backstop only; the angle bound
                                            ends a search first.  The two
                                            pivots turn 90 + 180 = 270
                                            degrees at about 77 degrees/s
                                            (80 mm/s wheels, 119 mm track):
                                            3.5 s, plus about 0.5 s each for
                                            the stop and the ramp, so 4.5 s.
                                            Twice that.                     */

/* START calibration: still, pivot across the line (towards the side the
 * right sensor says the line is on, else left first, then the other side
 * if needed), pivot back. */
#define VC_CAL_STILL_MS         (600U)   /* Still before the sweep.  Buddy 4
                                            needs a 0.3 s settled window and
                                            five level samples to take its
                                            level reference again, plus a
                                            word or two of delay.            */
#define VC_CAL_STILL_TIMEOUT_MS (3000U)  /* Buddy 4 not ready by then: fail. */
#define VC_CAL_LIMIT_MDEG       (45000)  /* Safety bound per side, measured
                                            from the START heading: the most
                                            the car may turn either way. It
                                            is a bound, not a stop point:
                                            the sweep stops on what the
                                            sensors see, and a successful one
                                            took about 12 degrees.  Kept
                                            small so a failed search cannot
                                            swing the car onto another part
                                            of the course.  Reached without a
                                            finished sweep on both sides is a
                                            failure.                        */
#define VC_CAL_BACK_MARGIN_MDEG (20000)  /* Turning back may pass the start
                                            heading by this much (also covers
                                            the turn while braking) before
                                            the line is declared not found.  */
#define VC_CAL_VERDICT_MS       (500U)   /* Wait this long for Buddy 3's or
                                            Buddy 4's verdict, then fail.    */
#define VC_CAL_TIMEOUT_MS       (20000U) /* Whole calibration: backstop.     */

/* ------------------------------------------------------------------------ *
 * States (system-level statechart)
 * ------------------------------------------------------------------------ */

typedef enum
{
    VC_STATE_INITIALISE = 0,
    VC_STATE_READY,
    VC_STATE_CALIBRATE,
    VC_STATE_FOLLOW_LINE,
    VC_STATE_HANDLE_JUNCTION,
    VC_STATE_TURNING,
    VC_STATE_OBSTACLE_SCAN,
    VC_STATE_BYPASS,
    VC_STATE_REACQUIRE_LINE,
    VC_STATE_EMERGENCY_STOP,
    VC_STATE_FINISHED,
    VC_STATE_COUNT
} vc_state_t;

/* ------------------------------------------------------------------------ *
 * Input codes
 * ------------------------------------------------------------------------ */

typedef enum                   /* Laptop command, from Buddy 1.           */
{
    VC_CMD_NONE = 0,
    VC_CMD_START,
    VC_CMD_STOP
} vc_cmd_t;

typedef enum                   /* Line position, from Buddy 3.            */
{
    VC_LINE_LEFT = 0,
    VC_LINE_CENTRE,
    VC_LINE_RIGHT,
    VC_LINE_LOST,              /* Buddy 3 reports this after 300 ms white. */
    VC_LINE_JUNCTION
} vc_line_t;

typedef enum                   /* Barcode navigation command, Buddy 3.    */
{
    VC_NAV_NONE = 0,
    VC_NAV_LEFT,
    VC_NAV_RIGHT,
    VC_NAV_STRAIGHT,
    VC_NAV_UTURN
} vc_nav_t;

typedef enum                   /* START calibration, from Buddy 3.        */
{
    VC_CAL_B3_IDLE = 0,
    VC_CAL_B3_RUNNING,
    VC_CAL_B3_OK,
    VC_CAL_B3_FAIL_CONTRAST,   /* A line sensor saw too little contrast.   */
    VC_CAL_B3_FAIL_LINE,       /* Contrast seen, sweep never finished.     */
    VC_CAL_B3_FAIL_GAP         /* White gap between the two line sensors.  */
} vc_cal_b3_t;

typedef enum                   /* START calibration, from Buddy 4.        */
{
    VC_CAL_B4_IDLE = 0,
    VC_CAL_B4_MEASURING,       /* Not enough noise samples yet.            */
    VC_CAL_B4_ENOUGH,          /* Enough samples: the sweep may end.       */
    VC_CAL_B4_DONE,            /* Noise floor set from the sweep.          */
    VC_CAL_B4_FAILED
} vc_cal_b4_t;

typedef enum                   /* Avoidance recommendation, Buddy 5.      */
{
    VC_AVOID_STOP = 0,
    VC_AVOID_CONTINUE,
    VC_AVOID_TURN_LEFT,
    VC_AVOID_TURN_RIGHT,
    VC_AVOID_REVERSE_RETRY
} vc_avoid_t;

/* Everything the VC sees in one 20 ms step.  Event fields (true or not
 * NONE) describe something that happened since the previous step. */
typedef struct
{
    uint32_t   now_ms;             /* Kernel time.                        */
    bool       b_init_ok;          /* All subsystems initialised.         */
    bool       b_init_failed;
    vc_cmd_t   cmd;                /* Event: laptop command.              */
    vc_line_t  line;               /* Latest line position.               */
    int8_t     line_error;         /* -127 (left) .. +127 (right).        */
    vc_nav_t   barcode;            /* Event: barcode decoded.             */
    bool       b_motion_done;      /* Event: turn / distance complete.    */
    bool       b_motion_failed;    /* Motion fault or command rejected.   */
    bool       b_stationary;       /* Both wheels stopped.                */
    bool       b_hump;             /* Buddy 4: slow down (on a ramp, or
                                      Buddy 4 not ready or silent).      */
    bool       b_terrain_ready;    /* Buddy 4: fresh, ready, level.  START
                                      is refused without it.             */
    bool       b_obstacle_ahead;   /* Event: Buddy 5 OBSTACLE_AHEAD.      */
    bool       b_scan_done;        /* Event: Buddy 5 SCAN_DONE.           */
    vc_avoid_t recommendation;     /* Valid when b_scan_done.             */
    bool       b_obstacle_passed;  /* Event: Buddy 5 OBSTACLE_PASSED.     */
    uint16_t   serious_faults;     /* Fault bits; 0 = none.               */
    bool       b_finish;           /* Finish condition met (TBC).         */
    vc_cal_b3_t cal_b3;            /* Buddy 3 calibration state.          */
    vc_cal_b4_t cal_b4;            /* Buddy 4 calibration state.          */
    int32_t    cal_angle_mdeg;     /* Turned since the sweep began, from
                                      the encoders; sign not relied on.   */
    int32_t    travel_mm;          /* Mean of both wheels, from the
                                      encoders: a running total.  Only
                                      differences are used.               */
} vc_inputs_t;

/* ------------------------------------------------------------------------ *
 * Output codes
 * ------------------------------------------------------------------------ */

typedef enum                   /* What to ask Buddy 2 for this step.      */
{
    VC_MOTION_NONE = 0,        /* Keep the current command.               */
    VC_MOTION_DRIVE,           /* Velocity: speed + steering.             */
    VC_MOTION_TURN,            /* Encoder turn: see turn.                 */
    VC_MOTION_BYPASS,          /* Bypass manoeuvre: see bypass_side.      */
    VC_MOTION_REVERSE,         /* Short reverse before a rescan.          */
    VC_MOTION_STOP,            /* Urgent stop: motion_stop().             */
    VC_MOTION_PIVOT            /* Calibration pivot: see pivot_mdeg.      */
} vc_motion_act_t;

typedef enum                   /* What to ask Buddy 3 and 4 for.          */
{
    VC_CAL_REQ_NONE = 0,
    VC_CAL_REQ_BEGIN,          /* Buddy 3 records; Buddy 4 retakes level. */
    VC_CAL_REQ_SWEEP,          /* Pivot started: Buddy 4 measures noise;
                                  the turned angle counts from now.       */
    VC_CAL_REQ_JUDGE,          /* Pivot hit its limit: Buddy 3 judges.    */
    VC_CAL_REQ_FINISH,         /* Back on the line: Buddy 4 sets floor.   */
    VC_CAL_REQ_ABORT           /* Calibration over without a run.         */
} vc_cal_req_t;

typedef enum                   /* Why a calibration failed.               */
{
    VC_CAL_FAIL_NONE = 0,
    VC_CAL_FAIL_CONTRAST,      /* A line sensor saw too little contrast.  */
    VC_CAL_FAIL_LINE,          /* Line not found within the limit.        */
    VC_CAL_FAIL_BUDDY4,        /* Buddy 4 not ready, or no valid floor.   */
    VC_CAL_FAIL_GAP            /* White gap between the two line sensors. */
} vc_cal_fail_t;

typedef enum                   /* What to ask Buddy 5 for this step.      */
{
    VC_OBST_NONE = 0,
    VC_OBST_SCAN_REQUEST,
    VC_OBST_BYPASS_START,
    VC_OBST_ABORT
} vc_obst_act_t;

typedef struct
{
    vc_state_t      state;          /* State after this step.             */
    bool            b_state_changed;
    vc_motion_act_t motion;
    int16_t         speed_mm_s;     /* For VC_MOTION_DRIVE.               */
    int8_t          steer;          /* For VC_MOTION_DRIVE (line error).  */
    vc_nav_t        turn;           /* For VC_MOTION_TURN.                */
    vc_avoid_t      bypass_side;    /* For VC_MOTION_BYPASS.              */
    vc_obst_act_t   obstacle;
    int32_t         pivot_mdeg;     /* For VC_MOTION_PIVOT: + is right.   */
    vc_cal_req_t    cal_req;
    vc_cal_fail_t   cal_fail;       /* Set on the step that returns to
                                       READY after a failed calibration.  */
} vc_outputs_t;

typedef enum                   /* Pivot search after losing the line.     */
{
    VC_SEARCH_NONE = 0,        /* No pivot search (after a bypass: creep). */
    VC_SEARCH_FIRST,           /* Pivoting towards where the line was.    */
    VC_SEARCH_SECOND           /* Pivoting the other way.                 */
} vc_search_t;

typedef enum                   /* Where the calibration is.               */
{
    VC_CAL_STILL = 0,          /* Standing still; Buddy 4 levels.         */
    VC_CAL_OUT,                /* Pivoting the first way, B3 and B4 measure. */
    VC_CAL_OUT_OTHER,          /* First side hit its limit: other way.    */
    VC_CAL_JUDGE,              /* Hit the limit; waiting for Buddy 3.     */
    VC_CAL_BACK,               /* Pivoting back to the line.              */
    VC_CAL_FINISH              /* Stopped on the line; waiting Buddy 4.   */
} vc_cal_phase_t;

/* Controller context.  Owned by the VC task; never shared. */
typedef struct
{
    vc_state_t state;
    uint32_t   entry_ms;           /* When the current state was entered. */
    vc_nav_t   stored_nav;         /* Barcode command waiting for junction. */
    uint8_t    rescans;            /* Reverse + rescan attempts so far.   */
    bool       b_obstacle_active;  /* A Buddy 5 request is in progress.   */
    bool       b_reversing;        /* Reversing before a rescan.          */
    vc_line_t  prev_line;          /* Line position at the previous step. */
    int8_t     last_side;          /* -1 line last seen left, +1 right,
                                      0 unknown: where to search.          */
    vc_search_t search;            /* Pivot search in REACQUIRE_LINE.     */
    int32_t    search_dir;         /* Way it pivots now (+1 right).       */
    int32_t    search_origin_mdeg; /* Turned-angle input when it began.   */
    vc_cal_phase_t cal_phase;      /* Valid in CALIBRATE.                 */
    uint32_t   cal_phase_ms;       /* When the phase began.               */
    int32_t    cal_out_mdeg;       /* Turned by the end of a successful
                                      sweep (magnitude), for the log.     */
    int32_t    cal_far_mdeg;       /* Furthest turned from the start
                                      heading (magnitude), for the log.   */
    int32_t    cal_first_dir;      /* First way: +1 right, -1 left, 0 not
                                      chosen yet.                         */
    int32_t    cal_dir;            /* Way the car pivots now (+1 right,
                                      -1 left); the way back in BACK.     */
    bool       b_junction_hold;    /* FOLLOW_LINE: driving straight across
                                      a junction (steer 0, LOST ignored). */
    int32_t    junction_origin_mm; /* travel_mm when the hold began.      */
} vc_t;

/* ------------------------------------------------------------------------ *
 * API
 * ------------------------------------------------------------------------ */

/* Reset the controller to INITIALISE; now_ms starts the init timeout. */
void vc_init(vc_t * p_vc, uint32_t now_ms);

/* Run one 20 ms step: read p_in, update p_vc, fill p_out. */
void vc_step(vc_t * p_vc, vc_inputs_t const * p_in, vc_outputs_t * p_out);

/* Printable state name, for logs and telemetry. */
char const * vc_state_name(vc_state_t state);

#endif /* VC_H */

/*** end of file ***/
