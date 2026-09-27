/** @file obstacle_types.h
 *
 * @brief Data types shared by the Buddy 5 obstacle modules.
 *
 * Units: distances in millimetres, times in microseconds or milliseconds
 * as named, servo angles in whole degrees (90 = straight ahead).
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h> before this header.
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
