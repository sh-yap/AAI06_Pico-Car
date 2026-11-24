#ifndef BUDDY2_MOTION_MOTOR_H
#define BUDDY2_MOTION_MOTOR_H

#include "motion_config.h"
#include "motion_types.h"

typedef enum {
    MOTION_MOTOR_BRAKE = 0,
    MOTION_MOTOR_FORWARD,
    MOTION_MOTOR_REVERSE,
    MOTION_MOTOR_COAST
} motion_motor_mode_t;

motion_result_t motion_motor_init(const motion_config_t *config);
motion_result_t motion_motor_apply(uint8_t motor_index, motion_motor_mode_t mode,
                                   uint16_t duty_permille);
motion_result_t motion_motor_brake_all(void);
uint16_t motion_motor_last_duty(uint8_t motor_index);

#endif
