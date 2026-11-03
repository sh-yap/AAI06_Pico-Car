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

int32_t motion_relative_mm(int32_t counts, int32_t origin,
                           int32_t mm_per_count_q16)
{
    int32_t relative_counts = motion_counts_delta((uint32_t)origin,
                                                   (uint32_t)counts);

    return motion_counts_to_mm(relative_counts, mm_per_count_q16);
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

bool motion_target_reached(int32_t target_mm, int32_t travelled_mm,
                           bool b_turn)
{
    if (target_mm == 0) {
        return false;
    }
    if (b_turn) {
        if (target_mm < 0) {
            target_mm = target_mm == INT32_MIN ? INT32_MAX : -target_mm;
        }
        return travelled_mm >= target_mm;
    }
    if (target_mm > 0) {
        return travelled_mm >= target_mm;
    }
    return travelled_mm <= target_mm;
}

bool motion_velocity_targets(int32_t linear_mm_s, int32_t angular_mdeg_s,
                             int32_t track_width_mm, int32_t maximum_mm_s,
                             int32_t *left_mm_s, int32_t *right_mm_s)
{
    int32_t angular_wheel_speed;
    int64_t left;
    int64_t right;

    if (left_mm_s == NULL || right_mm_s == NULL || track_width_mm <= 0
        || maximum_mm_s <= 0) {
        return false;
    }

    angular_wheel_speed = motion_turn_wheel_travel_mm(track_width_mm,
                                                       angular_mdeg_s);
    /* Positive angular rate is a vehicle-right turn: the left wheel travels
       faster than the right wheel. */
    left = (int64_t)linear_mm_s + (int64_t)angular_wheel_speed;
    right = (int64_t)linear_mm_s - (int64_t)angular_wheel_speed;
    if (left > maximum_mm_s || left < -maximum_mm_s
        || right > maximum_mm_s || right < -maximum_mm_s) {
        return false;
    }

    *left_mm_s = (int32_t)left;
    *right_mm_s = (int32_t)right;
    return true;
}

bool motion_turn_targets(int32_t angle_mdeg, int32_t wheel_speed_mm_s,
                         int32_t *left_mm_s, int32_t *right_mm_s)
{
    if (left_mm_s == NULL || right_mm_s == NULL || angle_mdeg == 0
        || wheel_speed_mm_s <= 0) {
        return false;
    }

    if (angle_mdeg > 0) {
        *left_mm_s = wheel_speed_mm_s;
        *right_mm_s = -wheel_speed_mm_s;
    } else {
        *left_mm_s = -wheel_speed_mm_s;
        *right_mm_s = wheel_speed_mm_s;
    }
    return true;
}

bool motion_velocity_requires_reset(bool b_active,
                                    motion_command_type_t active_type,
                                    int32_t previous_linear_mm_s,
                                    int32_t next_linear_mm_s)
{
    if (!b_active || active_type != MOTION_COMMAND_SET_VELOCITY) {
        return true;
    }
    if ((previous_linear_mm_s == 0) != (next_linear_mm_s == 0)) {
        return true;
    }
    return previous_linear_mm_s != 0 && next_linear_mm_s != 0
           && ((previous_linear_mm_s < 0) != (next_linear_mm_s < 0));
}

bool motion_command_is_valid(const motion_command_t *command,
                             int32_t maximum_mm_s,
                             int32_t track_width_mm,
                             int32_t maximum_turn_angle_mdeg)
{
    int32_t left_target;
    int32_t right_target;
    int64_t angle_magnitude;

    if (command == NULL || command->sequence_id == 0U || maximum_mm_s <= 0
        || track_width_mm <= 0 || maximum_turn_angle_mdeg <= 0
        || (uint32_t)command->type > (uint32_t)MOTION_COMMAND_TURN_ANGLE) {
        return false;
    }
    if (command->type == MOTION_COMMAND_SET_VELOCITY) {
        return motion_velocity_targets(command->value.velocity.linear_mm_s,
                                       command->value.velocity.angular_mdeg_s,
                                       track_width_mm, maximum_mm_s,
                                       &left_target, &right_target);
    }
    if (command->type == MOTION_COMMAND_MOVE_DISTANCE) {
        if (command->value.distance.distance_mm == 0
            || command->value.distance.speed_mm_s == 0
            || ((command->value.distance.distance_mm < 0)
                != (command->value.distance.speed_mm_s < 0))) {
            return false;
        }
        return command->value.distance.speed_mm_s <= maximum_mm_s
               && command->value.distance.speed_mm_s >= -maximum_mm_s;
    }

    angle_magnitude = command->value.turn.angle_mdeg;
    if (angle_magnitude < 0) {
        angle_magnitude = -angle_magnitude;
    }
    return angle_magnitude > 0
           && angle_magnitude <= maximum_turn_angle_mdeg
           && command->value.turn.wheel_speed_mm_s > 0
           && command->value.turn.wheel_speed_mm_s <= maximum_mm_s;
}

bool motion_encoder_delta_plausible(int32_t count_delta,
                                    uint32_t elapsed_ms,
                                    int32_t mm_per_count_q16,
                                    int32_t maximum_mm_s)
{
    uint64_t maximum_counts;
    uint64_t magnitude;

    if (elapsed_ms == 0U || mm_per_count_q16 <= 0 || maximum_mm_s <= 0) {
        return false;
    }
    magnitude = count_delta < 0 ? (uint64_t)(-(int64_t)count_delta)
                                : (uint64_t)count_delta;
    maximum_counts = ((uint64_t)maximum_mm_s * (uint64_t)elapsed_ms
                      * 65536ULL)
                     / ((uint64_t)mm_per_count_q16 * 1000ULL);
    /* Permit a two-times transient margin and a small quantisation margin. */
    maximum_counts = (maximum_counts * 2ULL) + 16ULL;
    return magnitude <= maximum_counts;
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

/* Limit the forward part (L + R) / 2 of a wheel pair's output change and let
   the steering part (L - R) / 2 follow the wanted value at once.  A wheel
   that the recombined output would reverse is limited on its own. */
void motion_slew_split(float applied_left, float applied_right,
                       float want_left, float want_right,
                       float max_delta, float max_output,
                       float *out_left, float *out_right)
{
    float forward = motion_slew_target((applied_left + applied_right) * 0.5f,
                                       (want_left + want_right) * 0.5f,
                                       max_delta);
    float steer = (want_left - want_right) * 0.5f;
    float left = forward + steer;
    float right = forward - steer;

    if (left > max_output) left = max_output;
    if (left < -max_output) left = -max_output;
    if (right > max_output) right = max_output;
    if (right < -max_output) right = -max_output;
    /* A lagging forward part must not drive a wheel against its wanted
       direction (for example a braked wheel beside a rising one). */
    if ((want_left >= 0.0f && left < 0.0f) || (want_left <= 0.0f && left > 0.0f)) {
        left = 0.0f;
    }
    if ((want_right >= 0.0f && right < 0.0f) || (want_right <= 0.0f && right > 0.0f)) {
        right = 0.0f;
    }
    if ((applied_left * left) < 0.0f) {
        left = motion_slew_target(applied_left, want_left, max_delta);
    }
    if ((applied_right * right) < 0.0f) {
        right = motion_slew_target(applied_right, want_right, max_delta);
    }
    *out_left = left;
    *out_right = right;
}

/* ---- Response learner (see motion_control.h) ------------------------- */

static void motion_fit_clear(motion_fit_t *fit)
{
    fit->n = 0.0f;
    fit->sx = 0.0f;
    fit->sy = 0.0f;
    fit->sxx = 0.0f;
    fit->sxy = 0.0f;
    fit->syy = 0.0f;
}

static void motion_fit_add(motion_fit_t *fit, float x, float y)
{
    fit->n = (fit->n * MOTION_LEARN_FORGET) + 1.0f;
    fit->sx = (fit->sx * MOTION_LEARN_FORGET) + x;
    fit->sy = (fit->sy * MOTION_LEARN_FORGET) + y;
    fit->sxx = (fit->sxx * MOTION_LEARN_FORGET) + (x * x);
    fit->sxy = (fit->sxy * MOTION_LEARN_FORGET) + (x * y);
    fit->syy = (fit->syy * MOTION_LEARN_FORGET) + (y * y);
}

/* Slope and intercept of the weighted least-squares line.  False when the
   data cannot yet fix the slope to a quarter of its value: too little
   data, too little spread in x, or a negative slope. */
static bool motion_fit_estimate(const motion_fit_t *fit, float *slope,
                                float *intercept)
{
    float mean_x;
    float mean_y;
    float cxx;
    float cxy;
    float cyy;
    float residual;
    float gain;
    float variance;

    if (fit->n < (0.5f / (1.0f - MOTION_LEARN_FORGET))) {
        return false;
    }
    mean_x = fit->sx / fit->n;
    mean_y = fit->sy / fit->n;
    cxx = fit->sxx - (fit->sx * mean_x);
    cxy = fit->sxy - (fit->sx * mean_y);
    cyy = fit->syy - (fit->sy * mean_y);
    if (cxx <= 0.0f) {
        return false;
    }
    gain = cxy / cxx;
    residual = cyy - (gain * cxy);
    if (residual < 0.0f) {
        residual = 0.0f;
    }
    /* Variance of the slope: residual variance over the spread of x. */
    variance = residual / ((fit->n - 2.0f) * cxx);
    if (gain <= 0.0f ||
        (variance * MOTION_LEARN_SE_FRACTION_INV * MOTION_LEARN_SE_FRACTION_INV)
            > (gain * gain)) {
        return false;
    }
    *slope = gain;
    *intercept = mean_y - (gain * mean_x);
    return true;
}

void motion_learn_flush(motion_learn_t *learn)
{
    learn->run = 0U;
    learn->mode = MOTION_LEARN_NONE;
}

void motion_learn_init(motion_learn_t *learn, uint8_t delay_cycles,
                       float fwd_seed_gain, float fwd_seed_offset,
                       float turn_default_gain)
{
    uint8_t i;

    motion_fit_clear(&learn->turn_fit);
    motion_fit_clear(&learn->fwd_fit);
    learn->pivot_xx = 0.0f;
    learn->pivot_xy = 0.0f;
    learn->pivot_samples = 0U;
    learn->turn_gain = turn_default_gain;
    learn->turn_seed = turn_default_gain;
    learn->turn_source = MOTION_LEARN_SRC_DEFAULT;
    learn->turn_seed_source = MOTION_LEARN_SRC_DEFAULT;
    learn->fwd_gain = fwd_seed_gain;
    learn->fwd_offset = fwd_seed_offset;
    learn->fwd_seed_gain = fwd_seed_gain;
    learn->fwd_seed_offset = fwd_seed_offset;
    learn->b_fwd_from_driving = false;
    learn->delay = delay_cycles > MOTION_LEARN_DELAY_MAX
                       ? (uint8_t)MOTION_LEARN_DELAY_MAX : delay_cycles;
    if (learn->delay == 0U) {
        learn->delay = 1U;      /* Power can act no sooner than next cycle. */
    }
    learn->head = 0U;
    for (i = 0U; i < MOTION_LEARN_HISTORY; i++) {
        learn->pwm_left[i] = 0;
        learn->pwm_right[i] = 0;
    }
    motion_learn_flush(learn);
}

static void motion_learn_driving(motion_learn_t *learn, float pwm_left,
                                 float pwm_right, float speed_left,
                                 float speed_right)
{
    float slope;
    float intercept;
    float zero_pwm;

    motion_fit_add(&learn->turn_fit, pwm_left - pwm_right,
                   speed_left - speed_right);
    if (motion_fit_estimate(&learn->turn_fit, &slope, &intercept)) {
        if (slope >= MOTION_LEARN_TURN_GAIN_MIN
            && slope <= MOTION_LEARN_TURN_GAIN_MAX) {
            learn->turn_gain = slope;
            learn->turn_source = MOTION_LEARN_SRC_DRIVING;
        } else {
            learn->turn_gain = learn->turn_seed;
            learn->turn_source = learn->turn_seed_source;
        }
    }

    motion_fit_add(&learn->fwd_fit, (pwm_left + pwm_right) * 0.5f,
                   (speed_left + speed_right) * 0.5f);
    if (motion_fit_estimate(&learn->fwd_fit, &slope, &intercept)) {
        zero_pwm = -intercept / slope;
        if (slope >= MOTION_LEARN_FWD_GAIN_MIN
            && slope <= MOTION_LEARN_FWD_GAIN_MAX
            && zero_pwm >= 0.0f && zero_pwm <= MOTION_LEARN_FWD_ZERO_PWM_MAX) {
            learn->fwd_gain = slope;
            learn->fwd_offset = intercept;
            learn->b_fwd_from_driving = true;
        } else {
            learn->fwd_gain = learn->fwd_seed_gain;
            learn->fwd_offset = learn->fwd_seed_offset;
            learn->b_fwd_from_driving = false;
        }
    }
}

static void motion_learn_pivot(motion_learn_t *learn, float pwm_left,
                               float pwm_right, float speed_left,
                               float speed_right)
{
    float x = pwm_left - pwm_right;
    float y = speed_left - speed_right;
    float slope;

    /* The two directions of a pivot cancel any left-right offset, so the
       line goes through the origin. */
    learn->pivot_xx += x * x;
    learn->pivot_xy += x * y;
    if (learn->pivot_samples < UINT8_MAX) {
        learn->pivot_samples++;
    }
    if (learn->pivot_samples >= MOTION_LEARN_PIVOT_MIN_SAMPLES
        && learn->pivot_xx > 0.0f) {
        slope = learn->pivot_xy / learn->pivot_xx;
        if (slope >= MOTION_LEARN_TURN_GAIN_MIN
            && slope <= MOTION_LEARN_TURN_GAIN_MAX) {
            learn->turn_seed = slope;
            learn->turn_seed_source = MOTION_LEARN_SRC_PIVOT;
            if (learn->turn_source != MOTION_LEARN_SRC_DRIVING) {
                learn->turn_gain = slope;
                learn->turn_source = MOTION_LEARN_SRC_PIVOT;
            }
        }
    }
}

void motion_learn_observe(motion_learn_t *learn, int32_t left_speed,
                          int32_t right_speed)
{
    uint8_t slot;
    float pwm_left;
    float pwm_right;

    /* Every cycle from the power to the speed must have been informative. */
    if (learn->mode == MOTION_LEARN_NONE
        || learn->run < (uint8_t)(learn->delay + 1U)) {
        return;
    }
    /* head is the newest entry, one cycle old: the entry delay cycles old
       is delay - 1 behind it. */
    slot = (uint8_t)((learn->head + MOTION_LEARN_HISTORY
                      - (uint8_t)(learn->delay - 1U))
                     & (MOTION_LEARN_HISTORY - 1U));
    pwm_left = (float)learn->pwm_left[slot];
    pwm_right = (float)learn->pwm_right[slot];
    if (learn->mode == MOTION_LEARN_DRIVE) {
        motion_learn_driving(learn, pwm_left, pwm_right, (float)left_speed,
                             (float)right_speed);
    } else {
        motion_learn_pivot(learn, pwm_left, pwm_right, (float)left_speed,
                           (float)right_speed);
    }
}

void motion_learn_record(motion_learn_t *learn, motion_learn_mode_t mode,
                         int32_t left_pwm, int32_t right_pwm)
{
    learn->head = (uint8_t)((learn->head + 1U) & (MOTION_LEARN_HISTORY - 1U));
    learn->pwm_left[learn->head] = (int16_t)left_pwm;
    learn->pwm_right[learn->head] = (int16_t)right_pwm;
    if (mode == MOTION_LEARN_NONE) {
        learn->run = 0U;
        learn->mode = MOTION_LEARN_NONE;
        return;
    }
    if (mode != learn->mode) {
        learn->run = 0U;            /* A different kind of data starts over. */
        learn->mode = mode;
    }
    if (learn->run < UINT8_MAX) {
        learn->run++;
    }
}

static float motion_learn_clamp_forward(float value, float deadband,
                                        float maximum)
{
    if (value < deadband) {
        return deadband;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

void motion_learn_feedforward(const motion_learn_t *learn,
                              float left_target, float right_target,
                              float deadband_permille, float maximum_permille,
                              float *left_ff, float *right_ff)
{
    float mean_pwm;
    float half_difference_pwm;

    if (learn->fwd_gain <= 0.0f || learn->turn_gain <= 0.0f
        || (left_target + right_target) <= 0.0f) {
        return;
    }
    mean_pwm = (((left_target + right_target) * 0.5f) - learn->fwd_offset)
               / learn->fwd_gain;
    half_difference_pwm = ((left_target - right_target) * 0.5f)
                          / learn->turn_gain;
    /* A wheel that is not going forward keeps the old feedforward. */
    if (left_target > 0.0f) {
        *left_ff = motion_learn_clamp_forward(mean_pwm + half_difference_pwm,
                                              deadband_permille,
                                              maximum_permille);
    }
    if (right_target > 0.0f) {
        *right_ff = motion_learn_clamp_forward(mean_pwm - half_difference_pwm,
                                               deadband_permille,
                                               maximum_permille);
    }
}
