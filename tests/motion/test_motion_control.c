#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../app_program/motion_config.h"
#include "../../app_program/motion_control.h"
#include "../../app_program/motion_stage4_profile.h"

static void expect(int condition, const char *name)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", name);
        exit(EXIT_FAILURE);
    }
    printf("PASS: %s\n", name);
}

int main(void)
{
    motion_command_t command = {0};
    motion_config_t config;
    int32_t left_target;
    int32_t right_target;
    int32_t speed;
    motion_pid_t pid = { 1.0f, 0.5f, 0.0f, 0.0f, 0.0f, 20.0f, -100.0f, 100.0f };
    motion_pid_result_t pid_result;
    motion_stage4_case_t stage4_case;
    uint32_t stage4_flags;
    expect(motion_quadrature_delta(0U, 1U) == -1, "forward quadrature");
    expect(motion_quadrature_delta(1U, 0U) == 1, "reverse quadrature");
    expect(motion_quadrature_delta(0U, 3U) == 0, "invalid quadrature");
    expect(motion_counts_delta(UINT32_MAX - 2U, 1U) == 4, "counter wrap");
    expect(motion_speed_from_counts(100, 100, 65536, &speed) && speed == 1000,
           "count-to-speed");
    expect(!motion_speed_from_counts(1, 0, 65536, &speed), "reject zero elapsed time");
    expect(motion_counts_to_mm(100, 65536) == 100, "count-to-mm");
    expect(motion_relative_mm(1120, 1000, 65536) == 120,
           "first command relative origin");
    expect(motion_relative_mm(1250, 1120, 65536) == 130,
           "second command relative origin");
    expect(motion_relative_mm(1, -3, 65536) == 4,
           "relative origin across encoder wrap");
    expect(motion_average(100, 200) == 150, "wheel average");
    expect(motion_turn_wheel_travel_mm(145, 90000) == 113, "turn geometry");
    expect(motion_turn_wheel_travel_mm(145, -90000) == -113,
           "negative turn geometry");
    expect(motion_turn_wheel_travel_mm(145, 180000) == 227,
           "U-turn geometry");
    expect(motion_target_reached(113, 113, true), "right turn completion");
    expect(motion_target_reached(-113, 113, true), "left turn completion magnitude");
    expect(!motion_target_reached(-100, 99, false), "distance not yet complete");
    expect(motion_target_reached(-100, -100, false), "reverse distance completion");
    expect(motion_velocity_targets(0, 90000, 145, 300,
                                   &left_target, &right_target)
           && left_target == 113 && right_target == -113,
           "positive angular rate turns right");
    expect(motion_velocity_targets(0, -90000, 145, 300,
                                   &left_target, &right_target)
           && left_target == -113 && right_target == 113,
           "negative angular rate turns left");
    expect(motion_turn_targets(90000, 100, &left_target, &right_target)
           && left_target == 100 && right_target == -100,
           "positive discrete turn turns right");
    expect(motion_turn_targets(-90000, 100, &left_target, &right_target)
           && left_target == -100 && right_target == 100,
           "negative discrete turn turns left");
    expect(!motion_velocity_requires_reset(true, MOTION_COMMAND_SET_VELOCITY,
                                           200, 200),
           "same-mode velocity update preserves PID");
    expect(!motion_velocity_requires_reset(true, MOTION_COMMAND_SET_VELOCITY,
                                           200, 100),
           "same-direction speed update preserves PID");
    expect(motion_velocity_requires_reset(true, MOTION_COMMAND_SET_VELOCITY,
                                          200, -100),
           "velocity reversal resets PID");
    expect(motion_velocity_requires_reset(false, MOTION_COMMAND_SET_VELOCITY,
                                          0, 100),
           "velocity start resets PID");

    command.sequence_id = 1U;
    command.type = MOTION_COMMAND_SET_VELOCITY;
    command.value.velocity.linear_mm_s = 200;
    command.value.velocity.angular_mdeg_s = 0;
    expect(motion_command_is_valid(&command, 300, 145, 360000),
           "valid velocity command");
    command.value.velocity.angular_mdeg_s = 360000;
    expect(!motion_command_is_valid(&command, 300, 145, 360000),
           "reject excessive derived wheel speed");
    command.type = MOTION_COMMAND_MOVE_DISTANCE;
    command.value.distance.distance_mm = -100;
    command.value.distance.speed_mm_s = 80;
    expect(!motion_command_is_valid(&command, 300, 145, 360000),
           "reject mismatched distance and speed signs");
    command.value.distance.speed_mm_s = -80;
    expect(motion_command_is_valid(&command, 300, 145, 360000),
           "accept reverse distance command");
    command.type = MOTION_COMMAND_TURN_ANGLE;
    command.value.turn.angle_mdeg = -90000;
    command.value.turn.wheel_speed_mm_s = -100;
    expect(!motion_command_is_valid(&command, 300, 145, 360000),
           "reject negative turn-speed magnitude");
    command.value.turn.wheel_speed_mm_s = 100;
    expect(motion_command_is_valid(&command, 300, 145, 360000),
           "accept negative-angle left turn");

    expect(motion_encoder_delta_plausible(100, 20U, 5275, 300),
           "accept plausible encoder delta");
    expect(!motion_encoder_delta_plausible(1000, 20U, 5275, 300),
           "reject impossible encoder delta");
    expect(motion_encoder_delta_plausible(250, 20U, 4684, 1000),
           "accept measured loaded launch encoder delta");
    expect(!motion_encoder_delta_plausible(250, 20U, 4684, 300),
           "old command-speed bound rejects loaded launch delta");
    expect(motion_speed_from_counts(100, 40U, 5275, &speed)
           && speed == 201,
           "delayed control period uses measured time");

    motion_config_default(&config);
    expect(config.left_counts_per_wheel_rev == 2536,
           "measured left encoder calibration");
    expect(config.right_counts_per_wheel_rev == 2529,
           "measured right encoder calibration");
    expect(config.left_wheel_diameter_um == 57700,
           "loaded left wheel diameter calibration");
    expect(config.right_wheel_diameter_um == 57600,
           "loaded right wheel diameter calibration");
    expect(config.startup_pwm_permille == 200U,
           "loaded startup PWM calibration");
    expect(config.encoder_physical_max_speed_mm_s == 1000,
           "loaded encoder physical-speed bound");
    expect(motion_config_validate(&config), "default motion configuration");
    expect(motion_feedforward_permille(100, 57U, 318U, 225U, 100U, 6U, 80U, 900U) == 88,
           "left feedforward");
    expect(motion_feedforward_permille(-100, 55U, 301U, 246U, 100U, 6U, 80U, 900U) == -90,
           "reverse feedforward");
    expect(motion_feedforward_permille(200, 59U, 318U, 400U, 100U, 6U, 80U, 900U) == 160,
           "high-speed feedforward");
    expect(motion_feedforward_permille(1, 57U, 318U, 225U, 100U, 6U, 80U, 900U) == 80,
           "feedforward deadband");
    expect(motion_feedforward_permille(10000, 57U, 318U, 225U, 100U, 6U, 80U, 900U) == 900,
           "feedforward saturation");
    expect(motion_feedforward_permille(-10000, 57U, 318U, 225U, 100U, 50U, 80U, 900U) == -900,
           "reverse feedforward saturation");
    expect(motion_slew_target(0.0f, 100.0f, 30.0f) == 30.0f, "slew limit");
    pid_result = motion_pid_step(&pid, 100.0f, 0.0f, 0.02f);
    expect(pid_result.output <= 100.0f && pid_result.output > 0.0f, "PID saturation");
    pid_result = motion_pid_step(&pid, 0.0f, 100.0f, 0.02f);
    expect(pid_result.output < 0.0f, "PID reverse output");
    expect(motion_stage4_case_count() == 18U, "Stage 4 sweep case count");
    expect(motion_stage4_case_get(0U, &stage4_case) &&
           stage4_case.direction == MOTION_STAGE4_FORWARD &&
           stage4_case.duty_permille == 150U && stage4_case.duration_ms == 1000U,
           "Stage 4 first forward case");
    expect(motion_stage4_case_get(8U, &stage4_case) &&
           stage4_case.direction == MOTION_STAGE4_FORWARD &&
           stage4_case.duty_permille == 350U,
           "Stage 4 last forward case");
    expect(motion_stage4_case_get(9U, &stage4_case) &&
           stage4_case.direction == MOTION_STAGE4_REVERSE &&
           stage4_case.duty_permille == 150U,
           "Stage 4 first reverse case");
    expect(motion_stage4_case_get(17U, &stage4_case) &&
           stage4_case.direction == MOTION_STAGE4_REVERSE &&
           stage4_case.duty_permille == 350U,
           "Stage 4 last reverse case");
    expect(!motion_stage4_case_get(18U, &stage4_case),
           "reject Stage 4 case outside sweep");
    stage4_flags = motion_stage4_assess(&stage4_case, -1000, -990, 0U, false);
    expect(stage4_flags == MOTION_STAGE4_RESULT_NONE &&
           motion_stage4_result_code(stage4_flags) == 1U,
           "Stage 4 reverse pass assessment");
    stage4_flags = motion_stage4_assess(&stage4_case, 0, -990, 0U, false);
    expect((stage4_flags & MOTION_STAGE4_RESULT_LEFT_LOW_ACTIVITY) != 0U &&
           motion_stage4_result_code(stage4_flags) == 3U,
           "Stage 4 left low-activity assessment");
    stage4_flags = motion_stage4_assess(&stage4_case, 0, 0, 0U, false);
    expect(motion_stage4_result_code(stage4_flags) == 5U,
           "Stage 4 both low-activity assessment");
    stage4_flags = motion_stage4_assess(&stage4_case, 1000, -990, 0U, false);
    expect((stage4_flags & MOTION_STAGE4_RESULT_WRONG_DIRECTION) != 0U &&
           motion_stage4_result_code(stage4_flags) == 6U,
           "Stage 4 wrong-direction priority");
    stage4_flags = motion_stage4_assess(&stage4_case, -1000, -990, 1U, false);
    expect((stage4_flags & MOTION_STAGE4_RESULT_INVALID_ENCODER) != 0U &&
           motion_stage4_result_code(stage4_flags) == 7U,
           "Stage 4 invalid-encoder assessment");
    stage4_flags = motion_stage4_assess(&stage4_case, 0, 0, 0U, true);
    expect(motion_stage4_result_code(stage4_flags) == 2U,
           "Stage 4 abort result priority");
    puts("All Buddy 2 host tests passed.");
    return EXIT_SUCCESS;
}
