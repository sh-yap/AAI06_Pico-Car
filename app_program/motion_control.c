#include "motion_control.h"

#include <limits.h>
#include <stddef.h>

int8_t motion_quadrature_delta(uint8_t previous, uint8_t current)
{
    static const int8_t table[16] = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0
    };
    uint8_t index = (uint8_t)(((previous & 3U) << 2) | (current & 3U));
    return table[index];
}

int32_t motion_counts_delta(uint32_t previous, uint32_t current)
{
    return (int32_t)(current - previous);
}

bool motion_speed_from_counts(int32_t count_delta, uint32_t elapsed_ms,
                              int32_t mm_per_count_q16, int32_t *speed_mm_s)
{
    int64_t numerator;
    if (speed_mm_s == NULL || elapsed_ms == 0U || mm_per_count_q16 <= 0) {
        return false;
    }
    numerator = (int64_t)count_delta * (int64_t)mm_per_count_q16 * 1000LL;
    numerator /= ((int64_t)elapsed_ms * 65536LL);
    if (numerator > INT32_MAX || numerator < INT32_MIN) {
        return false;
    }
    *speed_mm_s = (int32_t)numerator;
    return true;
}

int32_t motion_counts_to_mm(int32_t counts, int32_t mm_per_count_q16)
{
    int64_t value = ((int64_t)counts * mm_per_count_q16) / 65536LL;
    if (value > INT32_MAX) return INT32_MAX;
    if (value < INT32_MIN) return INT32_MIN;
    return (int32_t)value;
}

int32_t motion_average(int32_t left, int32_t right)
{
    return (int32_t)(((int64_t)left + (int64_t)right) / 2LL);
}

int32_t motion_turn_wheel_travel_mm(int32_t track_width_mm, int32_t angle_mdeg)
{
    /* pi is retained at 1e6 scale to keep the public API integer-only. */
    int64_t value = (int64_t)track_width_mm * 3141593LL * angle_mdeg;
    value /= 360000000000LL;
    if (value > INT32_MAX) return INT32_MAX;
    if (value < INT32_MIN) return INT32_MIN;
    return (int32_t)value;
}

int32_t motion_feedforward_permille(int32_t target_mm_s,
                                   uint16_t offset_permille,
                                   uint16_t slope_q10,
                                   uint16_t high_speed_slope_q10,
                                   uint16_t knee_mm_s,
                                   uint16_t reverse_boost_permille,
                                   uint16_t deadband_permille,
                                   uint16_t maximum_permille)
{
    int64_t magnitude;
    int64_t output;
    if (target_mm_s == 0 || maximum_permille == 0U) {
        return 0;
    }
    magnitude = target_mm_s < 0 ? -(int64_t)target_mm_s : (int64_t)target_mm_s;
    output = (int64_t)offset_permille +
             ((magnitude * (int64_t)slope_q10 + 512LL) / 1024LL);
    if (magnitude > knee_mm_s) {
        output += (((magnitude - knee_mm_s) *
                    (int64_t)high_speed_slope_q10) + 512LL) / 1024LL;
    }
    if (target_mm_s < 0) {
        output += reverse_boost_permille;
    }
    if (output < deadband_permille) output = deadband_permille;
    if (output > maximum_permille) output = maximum_permille;
    return target_mm_s < 0 ? -(int32_t)output : (int32_t)output;
}

motion_pid_result_t motion_pid_step(motion_pid_t *pid, float target,
                                    float measured, float dt_s)
{
    motion_pid_result_t result = {0.0f, target - measured};
    float derivative;
    float unclamped;
    if (pid == NULL || dt_s <= 0.0f) {
        return result;
    }
    derivative = (result.error - pid->previous_error) / dt_s;
    unclamped = pid->kp * result.error + pid->ki * pid->integral + pid->kd * derivative;
    if ((unclamped < pid->output_max && unclamped > pid->output_min) ||
        (unclamped >= pid->output_max && result.error < 0.0f) ||
        (unclamped <= pid->output_min && result.error > 0.0f)) {
        pid->integral += result.error * dt_s;
        if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    }
    unclamped = pid->kp * result.error + pid->ki * pid->integral + pid->kd * derivative;
    result.output = unclamped;
    if (result.output > pid->output_max) result.output = pid->output_max;
    if (result.output < pid->output_min) result.output = pid->output_min;
    pid->previous_error = result.error;
    return result;
}

float motion_slew_target(float current, float target, float max_delta)
{
    if (max_delta < 0.0f) max_delta = -max_delta;
    if (target > current + max_delta) return current + max_delta;
    if (target < current - max_delta) return current - max_delta;
    return target;
}
