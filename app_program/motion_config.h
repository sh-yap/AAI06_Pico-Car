#ifndef BUDDY2_MOTION_CONFIG_H
#define BUDDY2_MOTION_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t left_motor_a;
    uint8_t left_motor_b;
    uint8_t right_motor_a;
    uint8_t right_motor_b;
    uint8_t left_encoder_a;
    uint8_t left_encoder_b;
    uint8_t right_encoder_a;
    uint8_t right_encoder_b;
    int32_t left_counts_per_wheel_rev;
    int32_t right_counts_per_wheel_rev;
    int32_t left_wheel_diameter_um;
    int32_t right_wheel_diameter_um;
    /* Q10 multiplier for loaded distance calibration.  1024 means 1.000. */
    uint16_t distance_scale_q10;
    int32_t track_width_um;
    int32_t max_speed_mm_s;
    int32_t encoder_physical_max_speed_mm_s;
    uint16_t max_pwm_permille;
    uint16_t deadband_permille;
    uint16_t startup_pwm_permille;
    uint16_t left_feedforward_offset_permille;
    uint16_t right_feedforward_offset_permille;
    uint16_t left_feedforward_slope_q10;
    uint16_t right_feedforward_slope_q10;
    uint16_t left_feedforward_high_slope_q10;
    uint16_t right_feedforward_high_slope_q10;
    uint16_t feedforward_knee_mm_s;
    uint16_t left_reverse_boost_permille;
    uint16_t right_reverse_boost_permille;
} motion_config_t;

void motion_config_default(motion_config_t *config);
bool motion_config_validate(const motion_config_t *config);

#endif
