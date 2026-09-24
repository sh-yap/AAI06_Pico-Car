#include "motion_task.h"

#include <sys/machine.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>

#include "motion_config.h"
#include "motion_control.h"
#include "motion_encoder.h"
#include "motion_motor.h"

#define MOTION_PERIOD_MS 20U
#define MOTION_QUEUE_DEPTH 4U
#define MOTION_STOP_EVENT 1U
#define MOTION_STACK_SIZE 4096U
#define MOTION_PI_Q16 205887
#define MOTION_STARTUP_BOOST_MS 200U
#define MOTION_ZERO_SPEED_MM_S 20

static motion_config_t motion_config;
static motion_status_t motion_status;
static motion_pid_t left_pid = { 0.15f, 0.30f, 0.0f, 0.0f, 0.0f, 400.0f, -200.0f, 200.0f };
static motion_pid_t right_pid = { 0.15f, 0.30f, 0.0f, 0.0f, 0.0f, 400.0f, -200.0f, 200.0f };
static motion_command_t command_queue[MOTION_QUEUE_DEPTH];
static uint8_t command_buffer[sizeof(command_queue) + MOTION_QUEUE_DEPTH * 8U];
static UW motion_stack[MOTION_STACK_SIZE / sizeof(UW)];
static ID command_mbf = 0;
static ID stop_flg = 0;
static ID status_mtx = 0;
static ID motion_tid = 0;
static bool motion_initialised;
static bool motion_task_started;
static volatile bool active_command;
static volatile bool manoeuvre_pending;
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

static int32_t motion_abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
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
}

static uint64_t motion_timestamp_us(void)
{
    uint32_t high_a;
    uint32_t low;
    uint32_t high_b;
    do {
        high_a = in_w(TIMER_TIMEHR);
        low = in_w(TIMER_TIMELR);
        high_b = in_w(TIMER_TIMEHR);
    } while (high_a != high_b);
    return ((uint64_t)high_a << 32) | low;
}

static void publish_status(void)
{
    if (status_mtx > 0 && tk_loc_mtx(status_mtx, TMO_POL) == E_OK) {
        motion_status.timestamp_us = motion_timestamp_us();
        motion_status.valid = true;
        (void)tk_unl_mtx(status_mtx);
    }
}

static void motion_fault(motion_fault_t fault)
{
    (void)motion_motor_brake_all();
    motion_status.left_pwm_permille = 0U;
    motion_status.right_pwm_permille = 0U;
    motion_status.state = MOTION_STATE_FAULT;
    motion_status.fault = fault;
    active_command = false;
    manoeuvre_pending = false;
    publish_status();
}

static void start_command(const motion_command_t *command)
{
    if (command->type == MOTION_COMMAND_SET_VELOCITY) {
        /* Do not carry stale integral/slew state through a reversal. */
        reset_velocity_control();
        motion_status.active_sequence_id = command->sequence_id;
        motion_status.state = MOTION_STATE_RUNNING;
        motion_status.fault = MOTION_FAULT_NONE;
        motion_status.target_mm = 0;
        motion_status.target_speed_mm_s = command->value.velocity.linear_mm_s;
        motion_status.target_angle_mdeg = command->value.velocity.angular_mdeg_s;
        active_type = MOTION_COMMAND_SET_VELOCITY;
        active_timeout = command->timeout_ms;
        active_elapsed = 0U;
        active_command = true;
        active_type = command->type;
    } else if (!active_command) {
        active_command = true;
        manoeuvre_pending = false;
        active_timeout = command->timeout_ms;
        active_elapsed = 0U;
        motion_status.target_mm = command->type == MOTION_COMMAND_MOVE_DISTANCE ?
            command->value.distance.distance_mm :
            motion_turn_wheel_travel_mm((int32_t)(motion_config.track_width_um / 1000),
                command->value.turn.angle_mdeg);
        motion_status.target_speed_mm_s = command->type == MOTION_COMMAND_MOVE_DISTANCE ?
            command->value.distance.speed_mm_s : command->value.turn.wheel_speed_mm_s;
        motion_status.target_angle_mdeg = command->type == MOTION_COMMAND_TURN_ANGLE ?
            command->value.turn.angle_mdeg : 0;
        motion_status.active_sequence_id = command->sequence_id;
        motion_status.state = MOTION_STATE_RUNNING;
        motion_status.fault = MOTION_FAULT_NONE;
        previous_left = 0;
        previous_right = 0;
        reset_velocity_control();
    }
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
    motion_pid_result_t left_output;
    motion_pid_result_t right_output;
    motion_command_t command;
    left_mm_per_count_q16 = (int32_t)(((int64_t)MOTION_PI_Q16 * motion_config.left_wheel_diameter_um) /
                                      (motion_config.left_counts_per_wheel_rev * 1000LL));
    right_mm_per_count_q16 = (int32_t)(((int64_t)MOTION_PI_Q16 * motion_config.right_wheel_diameter_um) /
                                       (motion_config.right_counts_per_wheel_rev * 1000LL));
    if (tk_wai_flg(stop_flg, MOTION_STOP_EVENT, TWF_ORW | TWF_CLR, &stop_pattern, TMO_POL) == E_OK) {
        (void)motion_motor_brake_all();
        active_command = false;
        manoeuvre_pending = false;
        reset_velocity_control();
        motion_status.state = MOTION_STATE_STOPPED;
        /* A stop supersedes ordinary queued manoeuvres. */
        while (tk_rcv_mbf(command_mbf, &command, TMO_POL) >= (INT)sizeof(command)) {
        }
    }
    if (tk_rcv_mbf(command_mbf, &command, TMO_POL) >= (INT)sizeof(command)) {
        if (command.type > MOTION_COMMAND_TURN_ANGLE || command.sequence_id == 0U) {
            motion_fault(MOTION_FAULT_INVALID_COMMAND);
        } else if (command.type != MOTION_COMMAND_SET_VELOCITY && active_command) {
            /* A second manoeuvre cannot displace the one already running. */
        } else {
            start_command(&command);
        }
    }
    motion_encoder_get_snapshot(&enc);
    left_delta = enc.left_counts - previous_left;
    right_delta = enc.right_counts - previous_right;
    previous_left = enc.left_counts;
    previous_right = enc.right_counts;
    if (!motion_speed_from_counts(left_delta, MOTION_PERIOD_MS, left_mm_per_count_q16, &left_speed) ||
        !motion_speed_from_counts(right_delta, MOTION_PERIOD_MS, right_mm_per_count_q16, &right_speed)) {
        motion_fault(MOTION_FAULT_ENCODER);
        return;
    }
    motion_status.left_encoder_counts = enc.left_counts;
    motion_status.right_encoder_counts = enc.right_counts;
    motion_status.left_invalid_encoder_transitions = enc.left_invalid_transitions;
    motion_status.right_invalid_encoder_transitions = enc.right_invalid_transitions;
    motion_status.invalid_encoder_transitions = enc.invalid_transitions;
    motion_status.encoder_irq_count = enc.irq_count;
    motion_status.left_speed_mm_s = left_speed;
    motion_status.right_speed_mm_s = right_speed;
    motion_status.travelled_mm = motion_average(
        motion_counts_to_mm(enc.left_counts, left_mm_per_count_q16),
        motion_counts_to_mm(enc.right_counts, right_mm_per_count_q16));
    if (active_command && motion_status.state == MOTION_STATE_RUNNING) {
        left_target_speed = motion_status.target_speed_mm_s;
        right_target_speed = motion_status.target_speed_mm_s;
        if (active_type == MOTION_COMMAND_SET_VELOCITY && motion_status.target_angle_mdeg != 0) {
            int32_t angular_wheel_speed = motion_turn_wheel_travel_mm(
                (int32_t)(motion_config.track_width_um / 1000),
                motion_status.target_angle_mdeg);
            left_target_speed -= angular_wheel_speed;
            right_target_speed += angular_wheel_speed;
        } else if (active_type == MOTION_COMMAND_TURN_ANGLE && motion_status.target_angle_mdeg != 0) {
            /* Positive angle is a right turn: left wheel forward, right wheel reverse. */
            right_target_speed = motion_status.target_angle_mdeg > 0 ?
                -right_target_speed : right_target_speed;
            left_target_speed = motion_status.target_angle_mdeg > 0 ?
                left_target_speed : -left_target_speed;
            motion_status.travelled_mm = motion_average(
                motion_counts_to_mm(motion_abs_i32(enc.left_counts), left_mm_per_count_q16),
                motion_counts_to_mm(motion_abs_i32(enc.right_counts), right_mm_per_count_q16));
        } else {
            /* Small differential correction keeps a straight command centred. */
            float correction = (float)(enc.left_counts - enc.right_counts) * 0.01f;
            left_target_speed -= (int32_t)correction;
            right_target_speed += (int32_t)correction;
        }
        slewed_left_target = motion_slew_target(slewed_left_target,
            (float)left_target_speed, 6.0f);
        slewed_right_target = motion_slew_target(slewed_right_target,
            (float)right_target_speed, 6.0f);
        left_output = motion_pid_step(&left_pid, slewed_left_target,
                                      (float)left_speed, 0.020f);
        right_output = motion_pid_step(&right_pid, slewed_right_target,
                                       (float)right_speed, 0.020f);
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
        if (motion_abs_i32(left_target_speed) >= 80 &&
            motion_abs_i32(left_speed) < MOTION_ZERO_SPEED_MM_S &&
            left_startup_ms < MOTION_STARTUP_BOOST_MS) {
            if (motion_abs_i32((int32_t)left_output.output) < motion_config.startup_pwm_permille) {
                left_output.output = left_target_speed < 0 ?
                    -(float)motion_config.startup_pwm_permille :
                    (float)motion_config.startup_pwm_permille;
            }
            left_startup_ms += MOTION_PERIOD_MS;
        } else if (motion_abs_i32(left_speed) >= MOTION_ZERO_SPEED_MM_S) {
            left_startup_ms = MOTION_STARTUP_BOOST_MS;
        }
        if (motion_abs_i32(right_target_speed) >= 80 &&
            motion_abs_i32(right_speed) < MOTION_ZERO_SPEED_MM_S &&
            right_startup_ms < MOTION_STARTUP_BOOST_MS) {
            if (motion_abs_i32((int32_t)right_output.output) < motion_config.startup_pwm_permille) {
                right_output.output = right_target_speed < 0 ?
                    -(float)motion_config.startup_pwm_permille :
                    (float)motion_config.startup_pwm_permille;
            }
            right_startup_ms += MOTION_PERIOD_MS;
        } else if (motion_abs_i32(right_speed) >= MOTION_ZERO_SPEED_MM_S) {
            right_startup_ms = MOTION_STARTUP_BOOST_MS;
        }
        if (left_output.output >= 0.0f) {
            (void)motion_motor_apply(0U, MOTION_MOTOR_FORWARD, (uint16_t)left_output.output);
        } else {
            (void)motion_motor_apply(0U, MOTION_MOTOR_REVERSE, (uint16_t)-left_output.output);
        }
        if (right_output.output >= 0.0f) {
            (void)motion_motor_apply(1U, MOTION_MOTOR_FORWARD, (uint16_t)right_output.output);
        } else {
            (void)motion_motor_apply(1U, MOTION_MOTOR_REVERSE, (uint16_t)-right_output.output);
        }
        active_elapsed += MOTION_PERIOD_MS;
        if (motion_status.target_mm != 0 &&
            ((motion_status.target_mm > 0 && motion_status.travelled_mm >= motion_status.target_mm) ||
             (motion_status.target_mm < 0 && motion_status.travelled_mm <= motion_status.target_mm))) {
            (void)motion_motor_brake_all();
            motion_status.left_pwm_permille = 0U;
            motion_status.right_pwm_permille = 0U;
            active_command = false;
            manoeuvre_pending = false;
            motion_status.state = MOTION_STATE_COMPLETE;
        } else if (active_timeout != 0U && active_elapsed >= active_timeout) {
            motion_fault(MOTION_FAULT_TIMEOUT);
        } else {
            /* Each driven wheel is monitored independently. Requiring both
               wheels to be stationary would miss a disconnected encoder or
               a single jammed wheel while its partner continued moving. */
            if (motion_abs_i32(left_target_speed) >= 80 &&
                motion_abs_i32(left_speed) < MOTION_ZERO_SPEED_MM_S) {
                left_stall_ms += MOTION_PERIOD_MS;
            } else {
                left_stall_ms = 0U;
            }
            if (motion_abs_i32(right_target_speed) >= 80 &&
                motion_abs_i32(right_speed) < MOTION_ZERO_SPEED_MM_S) {
                right_stall_ms += MOTION_PERIOD_MS;
            } else {
                right_stall_ms = 0U;
            }
            if (left_stall_ms >= 500U || right_stall_ms >= 500U) {
                motion_fault(MOTION_FAULT_STALL);
            }
        }
    } else {
        (void)motion_motor_brake_all();
    }
    motion_status.left_pwm_permille = motion_motor_last_duty(0U);
    motion_status.right_pwm_permille = motion_motor_last_duty(1U);
    publish_status();
}

static void motion_task_entry(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    for (;;) {
        motion_loop();
        tk_dly_tsk(MOTION_PERIOD_MS);
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
    if (motion_motor_init(&motion_config) != MOTION_RESULT_OK ||
        motion_encoder_init(&motion_config) != MOTION_RESULT_OK) {
        motion_status.state = MOTION_STATE_FAULT;
        motion_status.fault = MOTION_FAULT_HARDWARE_CONFIG;
        return MOTION_RESULT_HARDWARE;
    }
    mbf.mbfatr = TA_TFIFO | TA_USERBUF;
    mbf.bufsz = sizeof(command_buffer);
    mbf.maxmsz = sizeof(motion_command_t);
    mbf.bufptr = command_buffer;
    command_mbf = tk_cre_mbf(&mbf);
    mtx.mtxatr = TA_TFIFO | TA_INHERIT;
    status_mtx = tk_cre_mtx(&mtx);
    flg.flgatr = TA_TFIFO | TA_WMUL;
    stop_flg = tk_cre_flg(&flg);
    if (command_mbf <= E_OK || status_mtx <= E_OK || stop_flg <= E_OK) return MOTION_RESULT_HARDWARE;
    motion_status.state = MOTION_STATE_IDLE;
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
    if (command == NULL || command->sequence_id == 0U || command->type > MOTION_COMMAND_TURN_ANGLE) {
        return MOTION_RESULT_INVALID;
    }
    if (command->type == MOTION_COMMAND_SET_VELOCITY &&
        (command->value.velocity.linear_mm_s > motion_config.max_speed_mm_s ||
         command->value.velocity.linear_mm_s < -motion_config.max_speed_mm_s)) return MOTION_RESULT_INVALID;
    if (command->type == MOTION_COMMAND_MOVE_DISTANCE && command->value.distance.speed_mm_s == 0) return MOTION_RESULT_INVALID;
    if (command->type == MOTION_COMMAND_TURN_ANGLE && command->value.turn.wheel_speed_mm_s == 0) return MOTION_RESULT_INVALID;
    if (command->type == MOTION_COMMAND_MOVE_DISTANCE &&
        (command->value.distance.speed_mm_s > motion_config.max_speed_mm_s ||
         command->value.distance.speed_mm_s < -motion_config.max_speed_mm_s)) return MOTION_RESULT_INVALID;
    if (command->type == MOTION_COMMAND_TURN_ANGLE &&
        (command->value.turn.wheel_speed_mm_s > motion_config.max_speed_mm_s ||
         command->value.turn.wheel_speed_mm_s < -motion_config.max_speed_mm_s)) return MOTION_RESULT_INVALID;
    if ((active_command || manoeuvre_pending) && command->type != MOTION_COMMAND_SET_VELOCITY) return MOTION_RESULT_BUSY;
    if (tk_snd_mbf(command_mbf, command, sizeof(*command), TMO_POL) < E_OK) return MOTION_RESULT_QUEUE_FULL;
    if (command->type != MOTION_COMMAND_SET_VELOCITY) manoeuvre_pending = true;
    return MOTION_RESULT_OK;
}

motion_result_t motion_stop(void)
{
    if (!motion_initialised) return MOTION_RESULT_UNINITIALISED;
    (void)tk_set_flg(stop_flg, MOTION_STOP_EVENT);
    (void)motion_motor_brake_all();
    motion_status.left_pwm_permille = 0U;
    motion_status.right_pwm_permille = 0U;
    motion_status.state = MOTION_STATE_STOPPED;
    active_command = false;
    manoeuvre_pending = false;
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
