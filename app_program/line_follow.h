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


/* Initialise the line-following module. */
void line_follow_init(void);

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