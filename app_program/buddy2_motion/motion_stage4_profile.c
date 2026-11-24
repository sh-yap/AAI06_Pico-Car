#include "motion_stage4_profile.h"

#include <stddef.h>

#define MOTION_STAGE4_CASES_PER_DIRECTION \
    (((MOTION_STAGE4_LAST_PWM_PERMILLE - MOTION_STAGE4_FIRST_PWM_PERMILLE) / \
      MOTION_STAGE4_PWM_STEP_PERMILLE) + 1U)
#define MOTION_STAGE4_TOTAL_CASES (MOTION_STAGE4_CASES_PER_DIRECTION * 2U)

static uint32_t count_magnitude(int32_t count)
{
    return count < 0 ? (uint32_t)(-(int64_t)count) : (uint32_t)count;
}

uint8_t motion_stage4_case_count(void)
{
    return (uint8_t)MOTION_STAGE4_TOTAL_CASES;
}

bool motion_stage4_case_get(uint8_t index, motion_stage4_case_t *test_case)
{
    uint8_t step;

    if (test_case == NULL || index >= MOTION_STAGE4_TOTAL_CASES) {
        return false;
    }
    step = (uint8_t)(index % MOTION_STAGE4_CASES_PER_DIRECTION);
    test_case->duty_permille = (uint16_t)(MOTION_STAGE4_FIRST_PWM_PERMILLE +
        ((uint16_t)step * MOTION_STAGE4_PWM_STEP_PERMILLE));
    test_case->direction = index < MOTION_STAGE4_CASES_PER_DIRECTION ?
        MOTION_STAGE4_FORWARD : MOTION_STAGE4_REVERSE;
    test_case->duration_ms = MOTION_STAGE4_RUN_MS;
    return true;
}

uint32_t motion_stage4_assess(const motion_stage4_case_t *test_case,
                              int32_t left_counts, int32_t right_counts,
                              uint32_t invalid_transitions, bool aborted)
{
    uint32_t result = MOTION_STAGE4_RESULT_NONE;

    if (test_case == NULL) {
        return MOTION_STAGE4_RESULT_WRONG_DIRECTION;
    }
    if (aborted) {
        result |= MOTION_STAGE4_RESULT_ABORTED;
    }
    if (count_magnitude(left_counts) < MOTION_STAGE4_MIN_ACTIVITY_COUNTS) {
        result |= MOTION_STAGE4_RESULT_LEFT_LOW_ACTIVITY;
    }
    if (count_magnitude(right_counts) < MOTION_STAGE4_MIN_ACTIVITY_COUNTS) {
        result |= MOTION_STAGE4_RESULT_RIGHT_LOW_ACTIVITY;
    }
    if ((test_case->direction == MOTION_STAGE4_FORWARD &&
         (left_counts < 0 || right_counts < 0)) ||
        (test_case->direction == MOTION_STAGE4_REVERSE &&
         (left_counts > 0 || right_counts > 0))) {
        result |= MOTION_STAGE4_RESULT_WRONG_DIRECTION;
    }
    if (invalid_transitions != 0U) {
        result |= MOTION_STAGE4_RESULT_INVALID_ENCODER;
    }
    return result;
}

uint8_t motion_stage4_result_code(uint32_t result_flags)
{
    uint32_t both_low = MOTION_STAGE4_RESULT_LEFT_LOW_ACTIVITY |
                        MOTION_STAGE4_RESULT_RIGHT_LOW_ACTIVITY;

    if ((result_flags & MOTION_STAGE4_RESULT_ABORTED) != 0U) {
        return 2U;
    }
    if ((result_flags & MOTION_STAGE4_RESULT_INVALID_ENCODER) != 0U) {
        return 7U;
    }
    if ((result_flags & MOTION_STAGE4_RESULT_WRONG_DIRECTION) != 0U) {
        return 6U;
    }
    if ((result_flags & both_low) == both_low) {
        return 5U;
    }
    if ((result_flags & MOTION_STAGE4_RESULT_RIGHT_LOW_ACTIVITY) != 0U) {
        return 4U;
    }
    if ((result_flags & MOTION_STAGE4_RESULT_LEFT_LOW_ACTIVITY) != 0U) {
        return 3U;
    }
    return 1U;
}
