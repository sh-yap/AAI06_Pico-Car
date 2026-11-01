#ifndef BUDDY2_MOTION_STAGE4_PROFILE_H
#define BUDDY2_MOTION_STAGE4_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#define MOTION_STAGE4_FIRST_PWM_PERMILLE 150U
#define MOTION_STAGE4_LAST_PWM_PERMILLE 350U
#define MOTION_STAGE4_PWM_STEP_PERMILLE 25U
#define MOTION_STAGE4_RUN_MS 1000U
#define MOTION_STAGE4_MIN_ACTIVITY_COUNTS 100U

typedef enum {
    MOTION_STAGE4_FORWARD = 0,
    MOTION_STAGE4_REVERSE
} motion_stage4_direction_t;

typedef struct {
    uint16_t duty_permille;
    motion_stage4_direction_t direction;
    uint32_t duration_ms;
} motion_stage4_case_t;

enum {
    MOTION_STAGE4_RESULT_NONE = 0U,
    MOTION_STAGE4_RESULT_ABORTED = 1U << 0,
    MOTION_STAGE4_RESULT_LEFT_LOW_ACTIVITY = 1U << 1,
    MOTION_STAGE4_RESULT_RIGHT_LOW_ACTIVITY = 1U << 2,
    MOTION_STAGE4_RESULT_WRONG_DIRECTION = 1U << 3,
    MOTION_STAGE4_RESULT_INVALID_ENCODER = 1U << 4
};

uint8_t motion_stage4_case_count(void);
bool motion_stage4_case_get(uint8_t index, motion_stage4_case_t *test_case);
uint32_t motion_stage4_assess(const motion_stage4_case_t *test_case,
                              int32_t left_counts, int32_t right_counts,
                              uint32_t invalid_transitions, bool aborted);
uint8_t motion_stage4_result_code(uint32_t result_flags);

#endif
