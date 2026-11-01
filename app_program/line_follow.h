#ifndef LINE_FOLLOW_H_
#define LINE_FOLLOW_H_

#include <stdbool.h>
#include <stdint.h>

#include "ir_sensor.h"


/* Line error range: -LINE_ERROR_MAX = left, +LINE_ERROR_MAX = right. */
#define LINE_ERROR_MAX         127

/*
 * Line sensor bits, 1 = black. Only two line sensors exist
 * (centre GP27, right GP26), so two bits are enough.
 */
#define LINE_SENSOR_CENTRE     (1U << 0)
#define LINE_SENSOR_RIGHT      (1U << 1)


/* Detected line position relative to the three IR sensors. */
typedef enum
{
    LINE_LEFT = 0,
    LINE_CENTRE,
    LINE_RIGHT,
    LINE_LOST,
    LINE_JUNCTION
} line_state_t;


/* State of the on-track calibration; carried in the Buddy 3 status word. */
typedef enum
{
    LINE_CAL_IDLE = 0,
    LINE_CAL_RUNNING,
    LINE_CAL_OK,
    LINE_CAL_FAIL_CONTRAST,   /* A line sensor saw too little black/white. */
    LINE_CAL_FAIL_LINE,       /* Contrast seen, but the sweep never ended. */
    LINE_CAL_FAIL_GAP         /* A white gap between the two line sensors. */
} line_cal_state_t;

/* What one sensor saw during the sweep (filtered ADC counts). */
typedef struct
{
    uint16_t low;
    uint16_t high;
    uint16_t threshold;       /* 0 = not learned. */
} line_cal_sensor_t;

typedef struct
{
    line_cal_sensor_t centre;
    line_cal_sensor_t right;
    line_cal_sensor_t barcode;
    uint16_t          centre_full_black;
    bool              gap_measured;   /* The gap check had enough data.     */
    int16_t           gap_overlap;    /* Samples both sensors were black at
                                         once; negative = samples both were
                                         white with the line between them.
                                         Only valid if gap_measured.        */
} line_cal_result_t;


/* Initialise the line-following module. */
void line_follow_init(void);

/* Start a calibration: forget the last one. Nothing is recorded until
 * line_cal_sweep(), so the still phase does not set any level. */
void line_cal_begin(void);

/* The pivot has started: record the sensors from now on. */
void line_cal_sweep(void);

/*
 * Feed one reading while the car pivots across the line. Applies the new
 * thresholds and sets LINE_CAL_OK as soon as both line sensors have seen
 * black and white and come to rest on white. No effect unless RUNNING and swept.
 */
void line_cal_update(ir_readings_t readings);

/*
 * The sweep is over (the pivot hit its limit). A sweep still RUNNING
 * fails: LINE_CAL_FAIL_CONTRAST if a line sensor saw too little contrast,
 * else LINE_CAL_FAIL_LINE. Thresholds stay as they were on failure.
 */
line_cal_state_t line_cal_end(void);

/* Drop a sweep in progress or a result (stop, failure elsewhere). */
void line_cal_abort(void);

line_cal_state_t line_cal_state(void);

/* Ranges seen in the latest sweep, and the thresholds if it succeeded. */
void line_cal_get_result(line_cal_result_t * p_result);

/* Learned barcode threshold, or fallback if the sweep saw no black. */
uint16_t line_cal_barcode_threshold(uint16_t fallback);

/* Determine line position from the three IR sensor readings. */
line_state_t line_get_position(ir_readings_t readings);

/*
 * Continuous line error, -LINE_ERROR_MAX..+LINE_ERROR_MAX:
 * negative = left, 0 = centred, positive = right.
 */
int8_t line_get_error(ir_readings_t readings);

/* Black/white state of each line sensor as LINE_SENSOR_* bits. */
uint8_t line_get_sensor_bits(ir_readings_t readings);


#endif /* LINE_FOLLOW_H_ */