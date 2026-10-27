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
#define VC_SPEED_NORMAL_MM_S    (200)    /* TBC with Buddy 2.                */
#define VC_SPEED_HUMP_MM_S      (100)    /* 50 % cap while on a hump.        */
#define VC_SPEED_SEARCH_MM_S    (120)    /* Slow forward while searching;
                                            80 stalled on the floor.         */
#define VC_SEARCH_STEER         (60)     /* Steer towards the side the line
                                            was last seen on (of 127).       */
#define VC_TURN_TIMEOUT_MS      (3000U)  /* TBC: 2 x expected turn time.     */
#define VC_SCAN_TIMEOUT_MS      (3000U)  /* 1.5 s scan budget (A7) x 2.      */
#define VC_BYPASS_TIMEOUT_MS    (6000U)  /* 3 s bypass target x 2.           */
#define VC_REACQUIRE_TIMEOUT_MS (4000U)  /* TBC: from bypass geometry.       */
#define VC_MAX_RESCANS          (2U)     /* Reverse + rescan attempts.       */

/* ------------------------------------------------------------------------ *
 * States (system-level statechart)
 * ------------------------------------------------------------------------ */

typedef enum
{
    VC_STATE_INITIALISE = 0,
    VC_STATE_READY,
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
    bool       b_hump;             /* Buddy 4: currently on a hump.       */
    bool       b_obstacle_ahead;   /* Event: Buddy 5 OBSTACLE_AHEAD.      */
    bool       b_scan_done;        /* Event: Buddy 5 SCAN_DONE.           */
    vc_avoid_t recommendation;     /* Valid when b_scan_done.             */
    bool       b_obstacle_passed;  /* Event: Buddy 5 OBSTACLE_PASSED.     */
    uint16_t   serious_faults;     /* Fault bits; 0 = none.               */
    bool       b_finish;           /* Finish condition met (TBC).         */
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
    VC_MOTION_STOP             /* Urgent stop: motion_stop().             */
} vc_motion_act_t;

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
} vc_outputs_t;

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
