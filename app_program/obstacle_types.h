/** @file obstacle_types.h
 *
 * @brief Data types shared by the Buddy 5 obstacle modules.
 *
 * Units: distances in millimetres, times in microseconds or milliseconds
 * as named, servo angles in whole degrees (90 = straight ahead).
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h> and <stdbool.h>
 * before this header.
 */

#ifndef OBSTACLE_TYPES_H
#define OBSTACLE_TYPES_H

/* Outcome of one ultrasonic measurement. */
typedef enum
{
    OBSTACLE_RANGE_OK = 0,       /* Valid distance.                        */
    OBSTACLE_RANGE_NO_ECHO,      /* Echo never went high: check wiring.    */
    OBSTACLE_RANGE_TOO_CLOSE,    /* Closer than the 20 mm minimum.         */
    OBSTACLE_RANGE_TOO_FAR,      /* Beyond the 4 m maximum.                */
    OBSTACLE_RANGE_TIMEOUT,      /* Echo still high: nothing reflected.    */
    OBSTACLE_RANGE_NOT_READY     /* Driver not initialised.                */
} obstacle_range_status_t;

/* Obstacle task states (Week 6 statechart).  The numbers are part of the
 * packed status word, so add new states at the end; never renumber. */
typedef enum
{
    OBSTACLE_STATE_INIT = 0,
    OBSTACLE_STATE_MONITOR,
    OBSTACLE_STATE_SCANNING,
    OBSTACLE_STATE_SIDE_WATCH,
    OBSTACLE_STATE_LINE_RECOVERY,
    OBSTACLE_STATE_SENSOR_FAULT,
    OBSTACLE_STATE_COUNT
} obstacle_state_t;

/* Recommended action for the Vehicle Controller.  Same numbers as
 * comms_avoid_t in comms.h; keep the two lists in step. */
typedef enum
{
    OBSTACLE_ACTION_STOP = 0,
    OBSTACLE_ACTION_CONTINUE,
    OBSTACLE_ACTION_TURN_LEFT,
    OBSTACLE_ACTION_TURN_RIGHT,
    OBSTACLE_ACTION_REVERSE_RETRY,
    OBSTACLE_ACTION_COUNT
} obstacle_action_t;

/* Latest forward reading and task state, before packing into one 32-bit
 * word with obstacle_calc_pack_status(). */
typedef struct
{
    uint32_t                distance_mm;   /* 0 unless range_status is OK. */
    obstacle_range_status_t range_status;
    obstacle_state_t        state;
    uint32_t                seq;           /* Packed modulo 256.           */
} obstacle_status_t;

/* Result of a scan, before packing into one 64-bit word with
 * obstacle_calc_pack_profile().  Distances of 0 mean "no valid reading". */
typedef struct
{
    uint32_t          distance_mm;     /* Closest point.                   */
    uint32_t          angle_deg;       /* Bearing of the closest point,
                                        * 90 = straight ahead.             */
    uint32_t          width_mm;        /* Packed as at most 1023.          */
    uint32_t          clear_left_mm;   /* 0 = unknown, not "blocked".      */
    uint32_t          clear_right_mm;  /* 0 = unknown, not "blocked".      */
    obstacle_action_t action;
} obstacle_profile_t;

/* Largest number of scan angles (60..120 deg in 5 deg steps). */
#define OBSTACLE_SCAN_MAX_POINTS    (13U)

/* What one scan angle shows, after background subtraction. */
typedef enum
{
    OBSTACLE_POINT_UNKNOWN = 0,    /* Too few OK pings: never "clear".    */
    OBSTACLE_POINT_OBSTACLE,       /* Part of the obstacle span.          */
    OBSTACLE_POINT_BACKGROUND      /* Measured, well beyond the obstacle. */
} obstacle_point_class_t;

/* One scan angle: the median of its OK pings. */
typedef struct
{
    uint32_t               angle_deg;
    uint32_t               distance_mm;   /* Median; 0 if not measured.   */
    uint32_t               valid_count;   /* OK pings.                    */
    uint32_t               silent_count;  /* NO_ECHO / NOT_READY pings.   */
    obstacle_point_class_t point_class;
} obstacle_scan_point_t;

/* A whole scan and what was concluded from it. */
typedef struct
{
    obstacle_scan_point_t points[OBSTACLE_SCAN_MAX_POINTS];
    uint32_t              count;
    obstacle_profile_t    profile;        /* Packed for the controller.   */
    uint32_t              right_edge_deg; /* Obstacle span, 0 if none.    */
    uint32_t              left_edge_deg;
    uint32_t              duration_ms;    /* Request to result.           */
} obstacle_scan_result_t;

/* Path check: what lies along a planned bypass leg after the first pivot.
 * KNOWN is only given to single angles, never as the overall verdict. */
typedef enum
{
    OBSTACLE_PATH_CLEAR = 0,     /* Measured free for at least need_mm.  */
    OBSTACLE_PATH_OPEN,          /* No echo at all: nothing seen in range
                                  * (but an angled face is also silent). */
    OBSTACLE_PATH_BLOCKED,       /* Something nearer than need_mm that is
                                  * not the known obstacle.              */
    OBSTACLE_PATH_FAULT,         /* Sensor did not answer.               */
    OBSTACLE_PATH_KNOWN          /* The obstacle that was scanned.       */
} obstacle_path_verdict_t;

/* Angles in one path check. */
#define OBSTACLE_PATH_MAX_POINTS    (3U)

/* One path-check angle. */
typedef struct
{
    uint32_t                angle_deg;
    uint32_t                distance_mm;  /* Median; 0 if not measured.  */
    obstacle_path_verdict_t verdict;
} obstacle_path_point_t;

typedef struct
{
    obstacle_path_verdict_t verdict;
    uint32_t                need_mm;      /* Free distance asked for.    */
    uint32_t                known_mm;     /* Scanned obstacle's distance;
                                           * 0 = none known.             */
    obstacle_path_point_t   points[OBSTACLE_PATH_MAX_POINTS];
    uint32_t                count;        /* Outermost angle first.      */
} obstacle_path_t;

/* The out-along-back bypass: pivot turn_deg towards side, drive out_mm
 * (the rear axle is then side_mm from the line), pivot back turn_deg to
 * run parallel to the line, drive along_mm past the obstacle, then turn
 * back_deg towards the line, drive back (back_mm, or until the line is
 * seen) and turn straight.  Distances are rear-axle travel. */
typedef struct
{
    obstacle_action_t side;       /* TURN_LEFT or TURN_RIGHT.           */
    uint32_t          turn_deg;
    uint32_t          out_mm;     /* Diagonal leg out.                  */
    uint32_t          side_mm;    /* Sideways offset while passing.     */
    uint32_t          along_mm;   /* Parallel leg past the obstacle.    */
    uint32_t          back_deg;   /* Turn towards the line.             */
    uint32_t          back_mm;    /* Diagonal leg back to the line.     */
    uint32_t          check_mm;   /* Path check need after the turn.    */
} obstacle_bypass_plan_t;

/* What the line sensors showed on the parallel leg (obstacle_plan.h). */
typedef enum
{
    OBSTACLE_LINE_NONE = 0,    /* White, or black for less than HELD_MM.  */
    OBSTACLE_LINE_CROSSED,     /* Black, then white within HELD_MM.       */
    OBSTACLE_LINE_HELD         /* Black for HELD_MM: running along it.    */
} obstacle_line_seen_t;

typedef struct
{
    bool    b_black;           /* Black at the last call.                 */
    int32_t start_mm;          /* Leg distance where black began.         */
} obstacle_line_watch_t;

/* One ultrasonic measurement. */
typedef struct
{
    uint32_t                echo_us;       /* Raw echo pulse width.        */
    uint32_t                distance_mm;   /* 0 unless status is OK.       */
    uint32_t                timestamp_us;  /* Trigger time, 32-bit timer.  */
    obstacle_range_status_t status;
} obstacle_range_t;

/* Wiring diagnosis from obstacle_ultrasonic_probe(): what the Echo pin did
 * after one trigger, sampled directly, next to what the counter measured.
 * Times are microseconds after the start of the trigger pulse. */
typedef struct
{
    bool     b_high_before;  /* Echo already high before triggering.  */
    bool     b_rose;         /* Echo went high after the trigger.      */
    bool     b_fell;         /* ...and then low again.                 */
    uint32_t rise_us;
    uint32_t fall_us;
    uint32_t counter_us;     /* Hardware echo counter for the same ping. */
} obstacle_probe_t;

/* Static pin test from obstacle_ultrasonic_pin_test(): Trig is held high
 * for about 10 ms, then low.  With a jumper from Trig to Echo (loopback)
 * every field should read true and the counter should be about 10000. */
typedef struct
{
    bool     b_trig_high_ok;   /* Trig pin really went high.            */
    bool     b_trig_low_ok;    /* Trig pin really went low again.       */
    bool     b_echo_high;      /* Echo pin read high while Trig high.   */
    bool     b_echo_low;       /* Echo pin read low after Trig low.     */
    uint32_t counter_us;       /* Echo counter over the high phase.     */
    uint32_t timer_us_per_100ms; /* Timer ticks in a 100 ms kernel sleep. */
} obstacle_pin_test_t;

/* Servo calibration: pulse widths that put the horn at 0, 90 and 180
 * degrees.  The mapping between points is linear. */
typedef struct
{
    uint32_t pulse_0_us;
    uint32_t pulse_90_us;
    uint32_t pulse_180_us;
} obstacle_servo_cal_t;

#endif /* OBSTACLE_TYPES_H */

/*** end of file ***/
