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
