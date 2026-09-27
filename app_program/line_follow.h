#ifndef LINE_FOLLOW_H_
#define LINE_FOLLOW_H_

#include <stdbool.h>

#include "ir_sensor.h"


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

/* Continuous line error: negative = left, 0 = centred, positive = right. */
float line_get_error(ir_readings_t readings);


#endif /* LINE_FOLLOW_H_ */