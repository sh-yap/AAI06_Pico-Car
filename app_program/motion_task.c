#include "motion_task.h"

#include <sys/machine.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>
#include <limits.h>

#include "motion_config.h"
#include "motion_control.h"
#include "motion_encoder.h"
#include "motion_clock.h"
#include "motion_motor.h"

#define MOTION_PERIOD_MS 20U
#define MOTION_QUEUE_DEPTH 4U
#define MOTION_STOP_EVENT 1U
#define MOTION_STACK_SIZE 4096U
#define MOTION_PI_Q16 205887
#define MOTION_DISTANCE_SCALE_Q10_ONE 1024LL
/* The loaded raw-PWM sweep showed that 225 permille starts the car reliably.
   A shorter boost limits the launch distance before encoder/PID control takes
   over at low mission speeds. */
#define MOTION_STARTUP_BOOST_MS 250U
#define MOTION_STALL_MS 1000U
#define MOTION_MIN_ACTIVE_SPEED_MM_S 50
#define MOTION_ZERO_SPEED_MM_S 20
#define MOTION_MAX_TURN_ANGLE_MDEG 360000
#define MOTION_INVALID_TRANSITIONS_PER_SAMPLE 8U
#define MOTION_TARGET_SLEW_MM_S_PER_PERIOD 6.0f
#define MOTION_PWM_SLEW_PER_PERIOD 25.0f
#define MOTION_SUBMIT_LOCK_MS 30 /* motion_submit(): max wait for the lock. */
#define MOTION_STRAIGHT_CORRECTION_GAIN 0.10f
#define MOTION_STRAIGHT_CORRECTION_MAX_MM_S 30.0f
#define MOTION_MIN_DT_MS 1U
#define MOTION_MAX_DT_MS 1000U

static motion_config_t motion_config;
static motion_status_t motion_status;
/* kp/ki were 0.15/0.30 (unloaded): on the floor a slowed wheel gained only
   ~60 permille per second and stalled (27 Oct closed-loop test).  Faster
   test_integrate values - Buddy 2 to retune loaded. */
static motion_pid_t left_pid = { 0.40f, 1.20f, 0.0f, 0.0f, 0.0f, 400.0f, -200.0f, 200.0f };
static motion_pid_t right_pid = { 0.40f, 1.20f, 0.0f, 0.0f, 0.0f, 400.0f, -200.0f, 200.0f };
static motion_command_t command_queue[MOTION_QUEUE_DEPTH];
static uint8_t command_buffer[sizeof(command_queue) + MOTION_QUEUE_DEPTH * 8U];
static UW motion_stack[MOTION_STACK_SIZE / sizeof(UW)];
static ID command_mbf = 0;
static ID command_mtx = 0;
static ID stop_flg = 0;
static ID status_mtx = 0;
static ID motion_tid = 0;
static bool motion_initialised;
static bool motion_task_started;
static bool active_command;
static bool manoeuvre_pending;
static bool fault_latched;
static bool reinitialisation_required;
static motion_command_type_t active_type;
static uint32_t active_timeout;
static uint32_t active_elapsed;
static int32_t previous_left;
static int32_t previous_right;
static uint32_t left_stall_ms;
static uint32_t right_stall_ms;
static float slewed_left_target;
static float slewed_right_target;
static uint32_t left_startup_ms;
static uint32_t right_startup_ms;
static int32_t command_origin_left;
static int32_t command_origin_right;
static uint64_t previous_timestamp_us;
static uint32_t previous_invalid_transitions;
static bool encoder_sample_valid;
static uint32_t previous_irq_count;
static float left_applied_output;
static float right_applied_output;

static int32_t motion_abs_i32(int32_t value)
{
    return value == INT32_MIN ? INT32_MAX : (value < 0 ? -value : value);
}

static void reset_pid(motion_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

static void reset_velocity_control(void)
{
    reset_pid(&left_pid);
    reset_pid(&right_pid);
    slewed_left_target = 0.0f;
    slewed_right_target = 0.0f;
    left_startup_ms = 0U;
    right_startup_ms = 0U;
    left_stall_ms = 0U;
    right_stall_ms = 0U;
    left_applied_output = 0.0f;
    right_applied_output = 0.0f;
}

static uint64_t motion_timestamp_us(void)
{
    return motion_clock_us();
}

static void motion_set_state(motion_state_t state)
{
    if (motion_status.state != state) {
        motion_status.state = state;
        motion_status.state_timestamp_us = motion_timestamp_us();
    }
}

static void clear_encoder_fault_detail(void)
{
    motion_status.encoder_fault = MOTION_ENCODER_FAULT_NONE;
    motion_status.encoder_fault_wheel_mask = 0U;
    motion_status.encoder_fault_elapsed_ms = 0U;
    motion_status.encoder_fault_left_delta = 0;
    motion_status.encoder_fault_right_delta = 0;
    motion_status.encoder_fault_previous_left = 0;
    motion_status.encoder_fault_previous_right = 0;
    motion_status.encoder_fault_previous_irq = 0U;
    motion_status.encoder_fault_previous_time_us = 0U;
    motion_status.encoder_fault_previous_left_pwm = 0U;
    motion_status.encoder_fault_previous_right_pwm = 0U;
}

static void publish_status(void)
{
    /* The Motion task owns the status mutex while publishing. */
    motion_status.timestamp_us = motion_timestamp_us();
    motion_status.valid = true;
}

static void motion_fault(motion_fault_t fault)
{
    if (motion_motor_brake_all() != MOTION_RESULT_OK) {
        fault = MOTION_FAULT_HARDWARE_CONFIG;
    }
    motion_status.left_pwm_permille = 0U;
    motion_status.right_pwm_permille = 0U;
    motion_set_state(MOTION_STATE_FAULT);
    motion_status.fault = fault;
    if (fault != MOTION_FAULT_ENCODER) {
        clear_encoder_fault_detail();
    }
    active_command = false;
    manoeuvre_pending = false;
    fault_latched = true;
    if (fault == MOTION_FAULT_HARDWARE_CONFIG) {
        reinitialisation_required = true;
    }
}

static void motion_encoder_fault(motion_encoder_fault_t detail,
                                 uint8_t wheel_mask,
                                 uint32_t elapsed_ms,
                                 int32_t left_delta,
                                 int32_t right_delta)
{
    motion_status.encoder_fault = detail;
    motion_status.encoder_fault_wheel_mask = wheel_mask;
    motion_status.encoder_fault_elapsed_ms = elapsed_ms;
    motion_status.encoder_fault_left_delta = left_delta;
    motion_status.encoder_fault_right_delta = right_delta;
    motion_status.encoder_fault_previous_left = previous_left;
    motion_status.encoder_fault_previous_right = previous_right;
    motion_status.encoder_fault_previous_irq = previous_irq_count;
    motion_status.encoder_fault_previous_time_us = previous_timestamp_us;
    motion_status.encoder_fault_previous_left_pwm = motion_motor_last_duty(0U);
    motion_status.encoder_fault_previous_right_pwm = motion_motor_last_duty(1U);
    motion_fault(MOTION_FAULT_ENCODER);
}

static void start_command(const motion_command_t *command,
                           const motion_encoder_snapshot_t *encoder)
{
    bool reset_control;

    if (command->type == MOTION_COMMAND_SET_VELOCITY) {
        /* A steering update is a setpoint update, not a new plant start. */
        reset_control = motion_velocity_requires_reset(
            active_command, active_type, motion_status.target_speed_mm_s,
            command->value.velocity.linear_mm_s);

        if (reset_control) {
            reset_velocity_control();
            active_elapsed = 0U;
            command_origin_left = encoder->left_counts;
            command_origin_right = encoder->right_counts;
        }

        motion_status.active_sequence_id = command->sequence_id;
        motion_set_state(MOTION_STATE_RUNNING);
        motion_status.fault = MOTION_FAULT_NONE;
        clear_encoder_fault_detail();
        motion_status.target_mm = 0;
        motion_status.target_speed_mm_s = command->value.velocity.linear_mm_s;
        motion_status.target_angle_mdeg = command->value.velocity.angular_mdeg_s;
        active_type = MOTION_COMMAND_SET_VELOCITY;
        active_timeout = command->timeout_ms;
        active_command = true;
    } else if (!active_command) {
        active_command = true;
        manoeuvre_pending = false;
        active_type = command->type;
        active_timeout = command->timeout_ms;
        active_elapsed = 0U;
        command_origin_left = encoder->left_counts;
        command_origin_right = encoder->right_counts;
        motion_status.target_mm = command->type == MOTION_COMMAND_MOVE_DISTANCE ?
            command->value.distance.distance_mm :
            motion_abs_i32(motion_turn_wheel_travel_mm(
                (int32_t)(motion_config.track_width_um / 1000),
                command->value.turn.angle_mdeg));
        motion_status.target_speed_mm_s = command->type == MOTION_COMMAND_MOVE_DISTANCE ?
            command->value.distance.speed_mm_s : command->value.turn.wheel_speed_mm_s;
        motion_status.target_angle_mdeg = command->type == MOTION_COMMAND_TURN_ANGLE ?
            command->value.turn.angle_mdeg : 0;
        motion_status.active_sequence_id = command->sequence_id;
        motion_set_state(MOTION_STATE_RUNNING);
        motion_status.fault = MOTION_FAULT_NONE;
        clear_encoder_fault_detail();
        reset_velocity_control();
    }
}

static uint32_t motion_elapsed_ms(uint64_t now_us, uint64_t previous_us)
{
    uint64_t elapsed_us = now_us - previous_us;
    uint32_t elapsed_ms = (uint32_t)(elapsed_us / 1000ULL);

    if (elapsed_ms < MOTION_MIN_DT_MS) {
        elapsed_ms = MOTION_MIN_DT_MS;
    }
    if (elapsed_ms > MOTION_MAX_DT_MS) {
        elapsed_ms = MOTION_MAX_DT_MS;
    }
    return elapsed_ms;
}

static int32_t motion_clamp_target(int32_t target)
{
    if (target > motion_config.max_speed_mm_s) {
        return motion_config.max_speed_mm_s;
    }
    if (target < -motion_config.max_speed_mm_s) {
        return -motion_config.max_speed_mm_s;
    }
    return target;
}

static float motion_clamp_output(float output)
{
    float maximum = (float)motion_config.max_pwm_permille;

    if (output > maximum) {
        return maximum;
    }
    if (output < -maximum) {
        return -maximum;
    }
    return output;
}

static void motion_update_travelled(const motion_encoder_snapshot_t *encoder,
                                    int32_t left_mm_per_count_q16,
                                    int32_t right_mm_per_count_q16)
{
    int32_t left_mm;
    int32_t right_mm;

    /* Preserve a completed manoeuvre's final relative travel until the next
       command starts. Otherwise the following idle cycle replaces it with
       lifetime encoder travel before a diagnostic or the VC can read it. */
    if (!active_command) {
        return;
    }
    if (active_type != MOTION_COMMAND_SET_VELOCITY) {
        left_mm = motion_relative_mm(encoder->left_counts, command_origin_left,
                                     left_mm_per_count_q16);
        right_mm = motion_relative_mm(encoder->right_counts, command_origin_right,
                                      right_mm_per_count_q16);
        if (active_type == MOTION_COMMAND_TURN_ANGLE) {
            left_mm = motion_abs_i32(left_mm);
            right_mm = motion_abs_i32(right_mm);
        }
    } else {
        left_mm = motion_counts_to_mm(encoder->left_counts,
                                      left_mm_per_count_q16);
        right_mm = motion_counts_to_mm(encoder->right_counts,
                                       right_mm_per_count_q16);
    }
    motion_status.travelled_mm = motion_average(left_mm, right_mm);
}

static motion_result_t motion_apply_output(uint8_t motor,
                                            float output)
{
    float bounded = motion_clamp_output(output);
    uint16_t duty = (uint16_t)(bounded < 0.0f ? -bounded : bounded);

    if (bounded >= 0.0f) {
        return motion_motor_apply(motor, MOTION_MOTOR_FORWARD, duty);
    }
    return motion_motor_apply(motor, MOTION_MOTOR_REVERSE, duty);
}

static void motion_loop(void)
{
    UINT stop_pattern;
    motion_encoder_snapshot_t enc;
    int32_t left_delta;
    int32_t right_delta;
    int32_t left_speed = 0;
    int32_t right_speed = 0;
    int32_t left_mm_per_count_q16;
    int32_t right_mm_per_count_q16;
    int32_t left_target_speed;
    int32_t right_target_speed;
    int32_t left_progress_mm;
    int32_t right_progress_mm;
    motion_pid_result_t left_output;
    motion_pid_result_t right_output;
    motion_command_t command;
    uint64_t now_us;
    uint64_t snapshot_start_us;
    uint32_t elapsed_ms;
    uint32_t invalid_delta;
    float correction;
    float slew_delta;
    float pwm_slew_delta;
    bool status_locked = false;
    bool command_locked = false;
    bool straight_command;
    bool left_delta_plausible;
    bool right_delta_plausible;

    if (status_mtx <= E_OK
        || tk_loc_mtx(status_mtx, TMO_FEVR) != E_OK) {
        return;
    }
    status_locked = true;
    if (command_mtx <= E_OK
        || tk_loc_mtx(command_mtx, TMO_FEVR) != E_OK) {
        (void)tk_unl_mtx(status_mtx);
        return;
    }
    command_locked = true;

    left_mm_per_count_q16 = (int32_t)(((int64_t)MOTION_PI_Q16 *
                                       motion_config.left_wheel_diameter_um *
                                       motion_config.distance_scale_q10) /
                                      ((int64_t)motion_config.left_counts_per_wheel_rev *
                                       1000LL * MOTION_DISTANCE_SCALE_Q10_ONE));
    right_mm_per_count_q16 = (int32_t)(((int64_t)MOTION_PI_Q16 *
                                        motion_config.right_wheel_diameter_um *
                                        motion_config.distance_scale_q10) /
                                       ((int64_t)motion_config.right_counts_per_wheel_rev *
                                        1000LL * MOTION_DISTANCE_SCALE_Q10_ONE));

    snapshot_start_us = motion_timestamp_us();
    motion_encoder_get_snapshot(&enc);
    now_us = enc.sampled_at_us;

    /* Stop and command queue state share one mutex across processors.  A
       command accepted after motion_stop() releases this mutex cannot be
       removed by that older stop. */
    if (tk_wai_flg(stop_flg, MOTION_STOP_EVENT, TWF_ORW | TWF_CLR,
                   &stop_pattern, TMO_POL) == E_OK) {
        if (motion_motor_brake_all() != MOTION_RESULT_OK) {
            motion_fault(MOTION_FAULT_HARDWARE_CONFIG);
        } else {
            active_command = false;
            manoeuvre_pending = false;
            reset_velocity_control();
            encoder_sample_valid = false;
            previous_invalid_transitions = enc.invalid_transitions;
            if (!reinitialisation_required) {
                fault_latched = false;
                motion_set_state(MOTION_STATE_STOPPED);
                motion_status.fault = MOTION_FAULT_NONE;
                clear_encoder_fault_detail();
            }
        }
    }
    /* Freeze the first fault sample until explicit stop acknowledgement.
       In particular, later idle reads must not replace its counts or cause. */
    if (fault_latched) {
        goto motion_loop_finish;
    }
    motion_status.encoder_sample_time_us = now_us;
    motion_status.encoder_snapshot_duration_us =
        (uint32_t)(now_us - snapshot_start_us);
    motion_status.left_encoder_counts = enc.left_counts;
    motion_status.right_encoder_counts = enc.right_counts;
    motion_status.left_invalid_encoder_transitions = enc.left_invalid_transitions;
    motion_status.right_invalid_encoder_transitions = enc.right_invalid_transitions;
    motion_status.invalid_encoder_transitions = enc.invalid_transitions;
    motion_status.encoder_irq_count = enc.irq_count;
    if (tk_rcv_mbf(command_mbf, &command, TMO_POL)
        >= (INT)sizeof(command)) {
        if (!motion_command_is_valid(
                &command, motion_config.max_speed_mm_s,
                (int32_t)(motion_config.track_width_um / 1000),
                MOTION_MAX_TURN_ANGLE_MDEG)) {
            motion_fault(MOTION_FAULT_INVALID_COMMAND);
        } else if (fault_latched || reinitialisation_required) {
            /* A faulted subsystem requires an explicit stop acknowledgement;
               hardware-configuration faults require reinitialisation. */
        } else if (command.type != MOTION_COMMAND_SET_VELOCITY
                   && active_command) {
            /* A second manoeuvre cannot displace the one already running. */
        } else {
            start_command(&command, &enc);
        }
    }

    if (!encoder_sample_valid) {
        elapsed_ms = MOTION_PERIOD_MS;
        left_delta = 0;
        right_delta = 0;
        left_speed = 0;
        right_speed = 0;
        encoder_sample_valid = true;
    } else {
        elapsed_ms = motion_elapsed_ms(now_us, previous_timestamp_us);
        left_delta = motion_counts_delta((uint32_t)previous_left,
                                          (uint32_t)enc.left_counts);
        right_delta = motion_counts_delta((uint32_t)previous_right,
                                           (uint32_t)enc.right_counts);
        left_delta_plausible = motion_encoder_delta_plausible(
            left_delta, elapsed_ms, left_mm_per_count_q16,
            motion_config.encoder_physical_max_speed_mm_s);
        right_delta_plausible = motion_encoder_delta_plausible(
            right_delta, elapsed_ms, right_mm_per_count_q16,
            motion_config.encoder_physical_max_speed_mm_s);
        if (!left_delta_plausible || !right_delta_plausible) {
            uint8_t wheel_mask = (uint8_t)(
                (!left_delta_plausible ? 1U : 0U) |
                (!right_delta_plausible ? 2U : 0U));
            motion_encoder_fault(MOTION_ENCODER_FAULT_COUNT_DELTA,
                                 wheel_mask, elapsed_ms,
                                 left_delta, right_delta);
            goto motion_loop_finish;
        }
        /* Only validated samples can reach speed, distance or steering. */
        if (left_delta_plausible) {
            if (!motion_speed_from_counts(left_delta, elapsed_ms,
                                          left_mm_per_count_q16,
                                          &left_speed)) {
                motion_encoder_fault(
                    MOTION_ENCODER_FAULT_SPEED_CALCULATION,
                    1U, elapsed_ms, left_delta, right_delta);
                goto motion_loop_finish;
            }
        } else {
            left_speed = motion_status.left_speed_mm_s;
        }
        if (right_delta_plausible) {
            if (!motion_speed_from_counts(right_delta, elapsed_ms,
                                          right_mm_per_count_q16,
                                          &right_speed)) {
                motion_encoder_fault(
                    MOTION_ENCODER_FAULT_SPEED_CALCULATION,
                    2U, elapsed_ms, left_delta, right_delta);
                goto motion_loop_finish;
            }
        } else {
            right_speed = motion_status.right_speed_mm_s;
        }
    }
    invalid_delta = enc.invalid_transitions - previous_invalid_transitions;
    if (invalid_delta > MOTION_INVALID_TRANSITIONS_PER_SAMPLE) {
        motion_encoder_fault(MOTION_ENCODER_FAULT_INVALID_TRANSITIONS,
                             0U, elapsed_ms, left_delta, right_delta);
        goto motion_loop_finish;
    }
    previous_left = enc.left_counts;
    previous_right = enc.right_counts;
    previous_timestamp_us = now_us;
    previous_irq_count = enc.irq_count;
    previous_invalid_transitions = enc.invalid_transitions;

    motion_status.left_encoder_counts = enc.left_counts;
    motion_status.right_encoder_counts = enc.right_counts;
    motion_status.left_invalid_encoder_transitions = enc.left_invalid_transitions;
    motion_status.right_invalid_encoder_transitions = enc.right_invalid_transitions;
    motion_status.invalid_encoder_transitions = enc.invalid_transitions;
    motion_status.encoder_irq_count = enc.irq_count;
    motion_status.left_speed_mm_s = left_speed;
    motion_status.right_speed_mm_s = right_speed;

    motion_update_travelled(&enc, left_mm_per_count_q16,
                            right_mm_per_count_q16);
    if (active_command && motion_status.state == MOTION_STATE_RUNNING) {
        left_target_speed = motion_status.target_speed_mm_s;
        right_target_speed = motion_status.target_speed_mm_s;
        straight_command = false;
        if (active_type == MOTION_COMMAND_SET_VELOCITY) {
            if (!motion_velocity_targets(
                    motion_status.target_speed_mm_s,
                    motion_status.target_angle_mdeg,
                    (int32_t)(motion_config.track_width_um / 1000),
                    motion_config.max_speed_mm_s,
                    &left_target_speed, &right_target_speed)) {
                motion_fault(MOTION_FAULT_INVALID_COMMAND);
                goto motion_loop_finish;
            }
            straight_command = motion_status.target_angle_mdeg == 0
                               && motion_status.target_speed_mm_s != 0;
        } else if (active_type == MOTION_COMMAND_TURN_ANGLE && motion_status.target_angle_mdeg != 0) {
            if (!motion_turn_targets(motion_status.target_angle_mdeg,
                                     motion_status.target_speed_mm_s,
                                     &left_target_speed,
                                     &right_target_speed)) {
                motion_fault(MOTION_FAULT_INVALID_COMMAND);
                goto motion_loop_finish;
            }
        } else {
            straight_command = true;
        }
        if (straight_command) {
            /* Compare normalised wheel distance, not raw encoder counts. */
            left_progress_mm = motion_relative_mm(enc.left_counts,
                                                  command_origin_left,
                                                  left_mm_per_count_q16);
            right_progress_mm = motion_relative_mm(enc.right_counts,
                                                   command_origin_right,
                                                   right_mm_per_count_q16);
            correction = (float)(left_progress_mm - right_progress_mm)
                         * MOTION_STRAIGHT_CORRECTION_GAIN;
            if (correction > MOTION_STRAIGHT_CORRECTION_MAX_MM_S) {
                correction = MOTION_STRAIGHT_CORRECTION_MAX_MM_S;
            } else if (correction < -MOTION_STRAIGHT_CORRECTION_MAX_MM_S) {
                correction = -MOTION_STRAIGHT_CORRECTION_MAX_MM_S;
            }
            left_target_speed -= (int32_t)correction;
            right_target_speed += (int32_t)correction;
        }
        left_target_speed = motion_clamp_target(left_target_speed);
        right_target_speed = motion_clamp_target(right_target_speed);
        slew_delta = MOTION_TARGET_SLEW_MM_S_PER_PERIOD
                     * ((float)elapsed_ms / (float)MOTION_PERIOD_MS);
        slewed_left_target = motion_slew_target(slewed_left_target,
            (float)left_target_speed, slew_delta);
        slewed_right_target = motion_slew_target(slewed_right_target,
            (float)right_target_speed, slew_delta);
        left_output = motion_pid_step(&left_pid, slewed_left_target,
                                      (float)left_speed,
                                      (float)elapsed_ms / 1000.0f);
        right_output = motion_pid_step(&right_pid, slewed_right_target,
                                       (float)right_speed,
                                       (float)elapsed_ms / 1000.0f);
        left_output.output += (float)motion_feedforward_permille(
            (int32_t)slewed_left_target,
            motion_config.left_feedforward_offset_permille,
            motion_config.left_feedforward_slope_q10,
            motion_config.left_feedforward_high_slope_q10,
            motion_config.feedforward_knee_mm_s,
            motion_config.left_reverse_boost_permille,
            motion_config.deadband_permille,
            motion_config.max_pwm_permille);
        right_output.output += (float)motion_feedforward_permille(
            (int32_t)slewed_right_target,
            motion_config.right_feedforward_offset_permille,
            motion_config.right_feedforward_slope_q10,
            motion_config.right_feedforward_high_slope_q10,
            motion_config.feedforward_knee_mm_s,
            motion_config.right_reverse_boost_permille,
            motion_config.deadband_permille,
            motion_config.max_pwm_permille);
        if (slewed_left_target > 0.0f && left_output.output < 0.0f) left_output.output = 0.0f;
        if (slewed_left_target < 0.0f && left_output.output > 0.0f) left_output.output = 0.0f;
        if (slewed_right_target > 0.0f && right_output.output < 0.0f) right_output.output = 0.0f;
        if (slewed_right_target < 0.0f && right_output.output > 0.0f) right_output.output = 0.0f;
        /* Apply a short, bounded boost only while a wheel is stationary.
           This overcomes static friction without becoming a permanent PWM
           offset once encoder feedback is present. */
        if (motion_abs_i32(left_target_speed) >= MOTION_MIN_ACTIVE_SPEED_MM_S &&
            motion_abs_i32(left_speed) < MOTION_ZERO_SPEED_MM_S &&
            left_startup_ms < MOTION_STARTUP_BOOST_MS) {
            if (motion_abs_i32((int32_t)left_output.output) < motion_config.startup_pwm_permille) {
                left_output.output = left_target_speed < 0 ?
                    -(float)motion_config.startup_pwm_permille :
                    (float)motion_config.startup_pwm_permille;
            }
            left_startup_ms += elapsed_ms;
        } else if (motion_abs_i32(left_speed) >= MOTION_ZERO_SPEED_MM_S) {
            /* Re-arm the bounded boost after real movement so this wheel can
               overcome static friction again if it later stops. */
            left_startup_ms = 0U;
        }
        if (motion_abs_i32(right_target_speed) >= MOTION_MIN_ACTIVE_SPEED_MM_S &&
            motion_abs_i32(right_speed) < MOTION_ZERO_SPEED_MM_S &&
            right_startup_ms < MOTION_STARTUP_BOOST_MS) {
            if (motion_abs_i32((int32_t)right_output.output) < motion_config.startup_pwm_permille) {
                right_output.output = right_target_speed < 0 ?
                    -(float)motion_config.startup_pwm_permille :
                    (float)motion_config.startup_pwm_permille;
            }
            right_startup_ms += elapsed_ms;
        } else if (motion_abs_i32(right_speed) >= MOTION_ZERO_SPEED_MM_S) {
            right_startup_ms = 0U;
        }
        /* Limit only commanded drive changes. Emergency stop, completion and
           faults still call the motor brake immediately. This limits drive
           changes; a power-related cause of the encoder fault is unproven. */
        pwm_slew_delta = MOTION_PWM_SLEW_PER_PERIOD
                         * ((float)elapsed_ms / (float)MOTION_PERIOD_MS);
        left_output.output = motion_slew_target(left_applied_output,
                                                left_output.output,
                                                pwm_slew_delta);
        right_output.output = motion_slew_target(right_applied_output,
                                                 right_output.output,
                                                 pwm_slew_delta);
        if (motion_apply_output(0U, left_output.output) != MOTION_RESULT_OK) {
            motion_fault(MOTION_FAULT_HARDWARE_CONFIG);
        } else {
            left_applied_output = left_output.output;
        }
        if (motion_status.state == MOTION_STATE_RUNNING
            && motion_apply_output(1U, right_output.output) != MOTION_RESULT_OK) {
            motion_fault(MOTION_FAULT_HARDWARE_CONFIG);
        } else if (motion_status.state == MOTION_STATE_RUNNING) {
            right_applied_output = right_output.output;
        }
        active_elapsed += elapsed_ms;
        if (motion_target_reached(
                motion_status.target_mm,
                motion_status.travelled_mm,
                active_type == MOTION_COMMAND_TURN_ANGLE)) {
            if (motion_motor_brake_all() != MOTION_RESULT_OK) {
                motion_fault(MOTION_FAULT_HARDWARE_CONFIG);
            } else {
                motion_status.left_pwm_permille = 0U;
                motion_status.right_pwm_permille = 0U;
                active_command = false;
                manoeuvre_pending = false;
                motion_set_state(MOTION_STATE_COMPLETE);
            }
        } else if (active_timeout != 0U && active_elapsed >= active_timeout) {
            motion_fault(MOTION_FAULT_TIMEOUT);
        } else {
            /* Each driven wheel is monitored independently. Requiring both
               wheels to be stationary would miss a disconnected encoder or
               a single jammed wheel while its partner continued moving. */
            if (motion_abs_i32(left_target_speed) >= MOTION_MIN_ACTIVE_SPEED_MM_S &&
                motion_abs_i32(left_speed) < MOTION_ZERO_SPEED_MM_S) {
                left_stall_ms += elapsed_ms;
            } else {
                left_stall_ms = 0U;
            }
            if (motion_abs_i32(right_target_speed) >= MOTION_MIN_ACTIVE_SPEED_MM_S &&
                motion_abs_i32(right_speed) < MOTION_ZERO_SPEED_MM_S) {
                right_stall_ms += elapsed_ms;
            } else {
                right_stall_ms = 0U;
            }
            if (left_stall_ms >= MOTION_STALL_MS || right_stall_ms >= MOTION_STALL_MS) {
                motion_fault(MOTION_FAULT_STALL);
            }
        }
    } else {
        if (motion_motor_brake_all() != MOTION_RESULT_OK) {
            motion_fault(MOTION_FAULT_HARDWARE_CONFIG);
        }
    }

motion_loop_finish:
    motion_status.left_pwm_permille = motion_motor_last_duty(0U);
    motion_status.right_pwm_permille = motion_motor_last_duty(1U);
    publish_status();
    if (command_locked) {
        (void)tk_unl_mtx(command_mtx);
    }
    if (status_locked) {
        (void)tk_unl_mtx(status_mtx);
    }
}

static void motion_task_entry(INT stacd, void *exinf)
{
    uint64_t next_us = motion_timestamp_us();
    uint64_t now_us;
    uint32_t delay_ms;

    (void)stacd;
    (void)exinf;
    for (;;) {
        motion_loop();
        next_us += (uint64_t)MOTION_PERIOD_MS * 1000ULL;
        now_us = motion_timestamp_us();
        if (next_us > now_us) {
            delay_ms = (uint32_t)((next_us - now_us + 999ULL) / 1000ULL);
            (void)tk_dly_tsk((RELTIM)delay_ms);
        } else {
            /* A late iteration must not make every later iteration late. */
            next_us = now_us;
        }
    }
}

motion_result_t motion_init(void)
{
    T_CMBF mbf = {0};
    T_CMTX mtx = {0};
    T_CFLG flg = {0};
    if (motion_initialised) return MOTION_RESULT_OK;
    motion_config_default(&motion_config);
    if (!motion_config_validate(&motion_config)) return MOTION_RESULT_HARDWARE;
    motion_status = (motion_status_t){0};
    motion_status.state = MOTION_STATE_UNINITIALISED;
    motion_status.state_timestamp_us = motion_timestamp_us();
    if (motion_motor_init(&motion_config) != MOTION_RESULT_OK ||
        motion_encoder_init(&motion_config) != MOTION_RESULT_OK) {
        motion_set_state(MOTION_STATE_FAULT);
        motion_status.fault = MOTION_FAULT_HARDWARE_CONFIG;
        return MOTION_RESULT_HARDWARE;
    }
    mbf.mbfatr = TA_TFIFO | TA_USERBUF;
    mbf.bufsz = sizeof(command_buffer);
    mbf.maxmsz = sizeof(motion_command_t);
    mbf.bufptr = command_buffer;
    command_mbf = tk_cre_mbf(&mbf);
    mtx.mtxatr = TA_TFIFO | TA_INHERIT;
    command_mtx = tk_cre_mtx(&mtx);
    status_mtx = tk_cre_mtx(&mtx);
    flg.flgatr = TA_TFIFO | TA_WMUL;
    stop_flg = tk_cre_flg(&flg);
    if (command_mbf <= E_OK || command_mtx <= E_OK || status_mtx <= E_OK
        || stop_flg <= E_OK) return MOTION_RESULT_HARDWARE;
    active_command = false;
    manoeuvre_pending = false;
    fault_latched = false;
    reinitialisation_required = false;
    encoder_sample_valid = false;
    previous_invalid_transitions = 0U;
    motion_set_state(MOTION_STATE_IDLE);
    motion_initialised = true;
    return MOTION_RESULT_OK;
}

motion_result_t motion_start_task(void)
{
    T_CTSK task = {0};
    if (!motion_initialised) return MOTION_RESULT_UNINITIALISED;
    if (motion_task_started) return MOTION_RESULT_OK;
    task.tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC | TA_USERBUF;
    task.task = motion_task_entry;
    task.itskpri = 4;
    task.stksz = MOTION_STACK_SIZE;
    task.bufptr = motion_stack;
    task.assprc = TP_PRC1;
    motion_tid = tk_cre_tsk(&task);
    if (motion_tid <= E_OK || tk_sta_tsk(motion_tid, 0) < E_OK) return MOTION_RESULT_HARDWARE;
    motion_task_started = true;
    return MOTION_RESULT_OK;
}

motion_result_t motion_submit(const motion_command_t *command)
{
    if (!motion_initialised) return MOTION_RESULT_UNINITIALISED;
    if (!motion_command_is_valid(
            command, motion_config.max_speed_mm_s,
            (int32_t)(motion_config.track_width_um / 1000),
            MOTION_MAX_TURN_ANGLE_MDEG)) return MOTION_RESULT_INVALID;
    /* Wait briefly: the Motion loop holds this lock for its whole body, and
       both tasks run every 20 ms from the same tick, so a single TMO_POL
       try could hit the held lock on every step and the drive command never
       got in (car still, no fault; test_integrate 28 Oct, Buddy 2 to
       review).  Bounded so a stuck Motion task cannot hang the caller. */
    if (command_mtx <= E_OK
        || tk_loc_mtx(command_mtx, MOTION_SUBMIT_LOCK_MS) != E_OK) {
        return MOTION_RESULT_BUSY;
    }
    if (reinitialisation_required) {
        (void)tk_unl_mtx(command_mtx);
        return MOTION_RESULT_HARDWARE;
    }
    if (fault_latched || ((active_command || manoeuvre_pending)
        && command->type != MOTION_COMMAND_SET_VELOCITY)) {
        (void)tk_unl_mtx(command_mtx);
        return MOTION_RESULT_BUSY;
    }
    if (tk_snd_mbf(command_mbf, command, sizeof(*command), TMO_POL) < E_OK) {
        (void)tk_unl_mtx(command_mtx);
        return MOTION_RESULT_QUEUE_FULL;
    }
    if (command->type != MOTION_COMMAND_SET_VELOCITY) {
        manoeuvre_pending = true;
    }
    (void)tk_unl_mtx(command_mtx);
    return MOTION_RESULT_OK;
}

motion_result_t motion_stop(void)
{
    motion_command_t discarded;

    if (!motion_initialised) return MOTION_RESULT_UNINITIALISED;
    if (command_mtx <= E_OK
        || tk_loc_mtx(command_mtx, TMO_FEVR) != E_OK) {
        return MOTION_RESULT_HARDWARE;
    }
    if (tk_set_flg(stop_flg, MOTION_STOP_EVENT) != E_OK) {
        (void)tk_unl_mtx(command_mtx);
        return MOTION_RESULT_HARDWARE;
    }
    /* Remove only commands that were already queued before this stop. Any
       command submitted after the mutex is released remains available to the
       Motion task and cannot be lost by the old stop-side flush. */
    while (tk_rcv_mbf(command_mbf, &discarded, TMO_POL)
           >= (INT)sizeof(discarded)) {
    }
    (void)tk_unl_mtx(command_mtx);
    return MOTION_RESULT_OK;
}

motion_result_t motion_get_status(motion_status_t *status)
{
    if (!motion_initialised) return MOTION_RESULT_UNINITIALISED;
    if (status == NULL) return MOTION_RESULT_INVALID;
    if (status_mtx > 0 && tk_loc_mtx(status_mtx, TMO_FEVR) == E_OK) {
        *status = motion_status;
        (void)tk_unl_mtx(status_mtx);
    } else {
        return MOTION_RESULT_HARDWARE;
    }
    return MOTION_RESULT_OK;
}
