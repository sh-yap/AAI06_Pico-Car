#ifndef IR_SENSOR_H_
#define IR_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>

/* Raw readings from the three IR sensors (from AO pins). */
typedef struct
{
    uint16_t left;
    uint16_t centre;
    uint16_t right;
} ir_readings_t;


/* Initialise the three IR sensors. */
void ir_sensor_init(void);


/* Read all three IR sensors. */
ir_readings_t ir_sensor_read(void);


/* Check if a reading represents the black line. Threshold set via calibration. */
bool ir_sensor_is_black(uint16_t reading, uint16_t threshold);

#endif /* IR_SENSOR_H_ */