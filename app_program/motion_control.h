#ifndef BUDDY2_MOTION_CONTROL_H
#define BUDDY2_MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_types.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float integral_limit;
    float output_min;
    float output_max;
} motion_pid_t;

typedef struct {
    float output;
    float error;
} motion_pid_result_t;

int8_t motion_quadrature_delta(uint8_t previous, uint8_t current);
int32_t motion_counts_delta(uint32_t previous, uint32_t current);
bool motion_speed_from_counts(int32_t count_delta, uint32_t elapsed_ms,
                              int32_t mm_per_count_q16, int32_t *speed_mm_s);
int32_t motion_counts_to_mm(int32_t counts, int32_t mm_per_count_q16);
int32_t motion_relative_mm(int32_t counts, int32_t origin,
                           int32_t mm_per_count_q16);
int32_t motion_average(int32_t left, int32_t right);
int32_t motion_turn_wheel_travel_mm(int32_t track_width_mm, int32_t angle_mdeg);
bool motion_target_reached(int32_t target_mm, int32_t travelled_mm,
                           bool b_turn);
bool motion_velocity_targets(int32_t linear_mm_s, int32_t angular_mdeg_s,
                             int32_t track_width_mm, int32_t maximum_mm_s,
                             int32_t *left_mm_s, int32_t *right_mm_s);
bool motion_turn_targets(int32_t angle_mdeg, int32_t wheel_speed_mm_s,
                         int32_t *left_mm_s, int32_t *right_mm_s);
bool motion_velocity_requires_reset(bool b_active,
                                    motion_command_type_t active_type,
                                    int32_t previous_linear_mm_s,
                                    int32_t next_linear_mm_s);
bool motion_command_is_valid(const motion_command_t *command,
                             int32_t maximum_mm_s,
                             int32_t track_width_mm,
                             int32_t maximum_turn_angle_mdeg);
bool motion_encoder_delta_plausible(int32_t count_delta,
                                    uint32_t elapsed_ms,
                                    int32_t mm_per_count_q16,
                                    int32_t maximum_mm_s);
int32_t motion_feedforward_permille(int32_t target_mm_s,
                                   uint16_t offset_permille,
                                   uint16_t slope_q10,
                                   uint16_t high_speed_slope_q10,
                                   uint16_t knee_mm_s,
                                   uint16_t reverse_boost_permille,
                                   uint16_t deadband_permille,
                                   uint16_t maximum_permille);
motion_pid_result_t motion_pid_step(motion_pid_t *pid, float target,
                                    float measured, float dt_s);
float motion_slew_target(float current, float target, float max_delta);

#endif
