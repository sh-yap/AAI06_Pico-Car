/* Execute the actual motion_loop with deterministic counts, clock and RTOS
 * calls. This covers fault-to-actuator behaviour absent from helper tests. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../app_program/motion_task.c"

static uint64_t test_clock;
static motion_encoder_snapshot_t test_encoder;
static motion_command_t test_command;
static bool queued;
static bool stop_pending;
static unsigned drive_calls;
static uint16_t test_duty[2];
static uint32_t snapshot_delay_us;
static motion_result_t brake_result = MOTION_RESULT_OK;

uint32_t in_w(uint32_t address)
{ return address == TIMER_TIMERAWL ? (uint32_t)test_clock : (uint32_t)(test_clock >> 32); }
ER tk_loc_mtx(ID id, INT timeout) { (void)id; (void)timeout; return E_OK; }
ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT *pattern, INT timeout)
{
    (void)id; (void)mode; (void)timeout;
    if (!stop_pending) return -1;
    stop_pending = false; *pattern = bits; return E_OK;
}
ER tk_set_flg(ID id, UINT bits) { (void)id; (void)bits; stop_pending = true; return E_OK; }
INT tk_rcv_mbf(ID id, void *data, INT timeout)
{
    (void)id; (void)timeout;
    if (!queued) return -1;
    memcpy(data, &test_command, sizeof(test_command)); queued = false;
    return (INT)sizeof(test_command);
}
ER tk_snd_mbf(ID id, const void *data, INT size, INT timeout)
{
    (void)id; (void)size; (void)timeout;
    memcpy(&test_command, data, sizeof(test_command)); queued = true; return E_OK;
}
ID tk_cre_mbf(const T_CMBF *p) { (void)p; return 1; }
ID tk_cre_mtx(const T_CMTX *p) { (void)p; return 2; }
ID tk_cre_flg(const T_CFLG *p) { (void)p; return 3; }
ID tk_cre_tsk(const T_CTSK *p) { (void)p; return 4; }
ER tk_sta_tsk(ID id, INT code) { (void)id; (void)code; return E_OK; }
ER tk_dly_tsk(RELTIM ms) { test_clock += (uint64_t)ms * 1000U; return E_OK; }
motion_result_t motion_motor_init(const motion_config_t *p) { (void)p; return MOTION_RESULT_OK; }
motion_result_t motion_encoder_init(const motion_config_t *p) { (void)p; return MOTION_RESULT_OK; }
void motion_encoder_get_snapshot(motion_encoder_snapshot_t *p)
{
    /* An IRQ or higher-priority task can run after Motion reads the clock
       but before its encoder snapshot is acquired. */
    test_clock += snapshot_delay_us;
    snapshot_delay_us = 0U;
    *p = test_encoder;
    p->sampled_at_us = test_clock;
}
motion_result_t motion_motor_apply(uint8_t motor, motion_motor_mode_t mode, uint16_t duty)
{ (void)mode; ++drive_calls; test_duty[motor] = duty; return MOTION_RESULT_OK; }
motion_result_t motion_motor_brake_all(void)
{
    test_duty[0] = 0U;
    test_duty[1] = 0U;
    return brake_result;
}
uint16_t motion_motor_last_duty(uint8_t motor) { return test_duty[motor]; }

static void check(bool ok, const char *name)
{
    if (!ok) { fprintf(stderr, "FAIL: %s\n", name); exit(1); }
    printf("PASS: %s\n", name);
}

/* Run n Motion periods with the given wheel counts added each period. */
static void run_periods(unsigned n, int32_t left_inc, int32_t right_inc)
{
    unsigned i;
    for (i = 0U; i < n; i++) {
        test_clock += 20000U;
        test_encoder.left_counts += left_inc;
        test_encoder.right_counts += right_inc;
        test_encoder.irq_count += (uint32_t)(left_inc + right_inc);
        motion_loop();
    }
}

/* Steered velocity commands: a wheel told to go slower than the deadband
 * power can drive is braked (duty 0), and the driven wheel is not stalled. */
static void steered_low_target_checks(void)
{
    motion_command_t v = {0};
    int32_t l, r;
    int32_t ang;

    v.type = MOTION_COMMAND_SET_VELOCITY;
    v.timeout_ms = 10000U;
    v.value.velocity.linear_mm_s = 50;

    /* Full steer at 50 mm/s: 0 / 100 mm/s. The slower (right) wheel is the inner. */
    ang = 48152;
    (void)motion_velocity_targets(50, ang, 119, motion_config.max_speed_mm_s, &l, &r);
    check(r <= 1 && l >= 99, "full steer targets are about 0 and 100");
    v.sequence_id = 10U;
    v.value.velocity.angular_mdeg_s = ang;
    check(motion_submit(&v) == MOTION_RESULT_OK, "full steer command accepted");
    /* The outer (left) wheel turns at about 100 mm/s: 28 counts per 20 ms. */
    run_periods(80U, 28, 0);
    check(motion_status.state == MOTION_STATE_RUNNING &&
          motion_status.fault == MOTION_FAULT_NONE,
          "inner wheel at 0 mm/s: no stall or fault in 1.6 s");
    check(test_duty[1] == 0U, "inner wheel target 0 gives duty 0, not the deadband floor");
    check(test_duty[0] > 80U, "outer wheel is still driven");

    /* Small non-zero inner targets (15 and 25 mm/s) are also braked. */
    ang = 48152 * 7 / 10;     /* 70 mm/s difference: 15 / 85 */
    v.sequence_id = 11U;
    v.value.velocity.angular_mdeg_s = ang;
    (void)motion_submit(&v);
    run_periods(40U, 24, 0);
    check(test_duty[1] == 0U && test_duty[0] > 80U, "inner target 15 mm/s is braked");
    ang = 48152 / 2;          /* 50 mm/s difference: 25 / 75 */
    v.sequence_id = 12U;
    v.value.velocity.angular_mdeg_s = ang;
    (void)motion_submit(&v);
    run_periods(40U, 20, 0);
    check(test_duty[1] == 0U && test_duty[0] > 80U, "inner target 25 mm/s is braked");

    /* 40 mm/s difference gives an inner target of 30: still braked.
       30 mm/s difference gives 35: driven (at least the deadband floor). */
    ang = 48152 * 3 / 10;
    v.sequence_id = 13U;
    v.value.velocity.angular_mdeg_s = ang;
    (void)motion_submit(&v);
    run_periods(40U, 18, 10);
    check(test_duty[1] >= 80U && test_duty[0] > 80U, "inner target 35 mm/s is driven");

    /* A straight command at low speed is not affected. */
    v.sequence_id = 14U;
    v.value.velocity.linear_mm_s = 30;
    v.value.velocity.angular_mdeg_s = 0;
    (void)motion_submit(&v);
    run_periods(40U, 8, 8);
    check(test_duty[1] >= 80U && test_duty[0] >= 80U,
          "straight command at 30 mm/s keeps both wheels driven");

    (void)motion_stop();
    motion_loop();
    reset_velocity_control();
    encoder_sample_valid = false;
    test_clock = 0U;
    memset(&test_encoder, 0, sizeof(test_encoder));
    test_duty[1] = 0U;
    test_duty[0] = 0U;
}

/* Braking the forward part is immediate, speeding up stays slewed. */
static void forward_brake_checks(void)
{
    motion_command_t v = {0};
    int32_t i;
    int32_t before;

    v.type = MOTION_COMMAND_SET_VELOCITY;
    v.timeout_ms = 10000U;
    v.sequence_id = 20U;
    v.value.velocity.linear_mm_s = 150;
    check(motion_submit(&v) == MOTION_RESULT_OK, "cruise command accepted");
    run_periods(30U, 42, 42);
    check(motion_status.left_slewed_target_mm_s >= 148 &&
          motion_status.right_slewed_target_mm_s >= 148,
          "forward part has slewed up to 150 mm/s");

    /* Full steer at 50 mm/s: the inner wheel must drop at once. */
    v.sequence_id = 21U;
    v.value.velocity.linear_mm_s = 50;
    v.value.velocity.angular_mdeg_s = 48152;
    (void)motion_submit(&v);
    run_periods(1U, 42, 42);
    check(motion_status.right_slewed_target_mm_s <= 35,
          "150 to 50 with full steer: inner target is at the command on the first period");

    /* Straight at 50, then speed up to 150: rise stays within 6 mm/s per period. */
    v.sequence_id = 22U;
    v.value.velocity.angular_mdeg_s = 0;
    (void)motion_submit(&v);
    run_periods(5U, 14, 14);
    v.sequence_id = 23U;
    v.value.velocity.linear_mm_s = 150;
    (void)motion_submit(&v);
    for (i = 0; i < 5; i++) {
        before = motion_status.left_slewed_target_mm_s;
        run_periods(1U, 14, 14);
        check(motion_status.left_slewed_target_mm_s > before &&
              motion_status.left_slewed_target_mm_s - before <= 6,
              "speeding up 50 to 150 rises by at most 6 mm/s per period");
    }

    (void)motion_stop();
    motion_loop();
    reset_velocity_control();
    encoder_sample_valid = false;
    test_clock = 0U;
    memset(&test_encoder, 0, sizeof(test_encoder));
    test_duty[1] = 0U;
    test_duty[0] = 0U;
}

int main(void)
{
    motion_status_t first_fault;
    unsigned drives_before;
    float integral_before;
    int32_t travel_before;
    motion_command_t cmd = {0};
    cmd.type = MOTION_COMMAND_MOVE_DISTANCE;
    cmd.sequence_id = 1U;
    cmd.timeout_ms = 6000U;
    cmd.value.distance.distance_mm = 300;
    cmd.value.distance.speed_mm_s = 50;
    check(motion_init() == MOTION_RESULT_OK, "task initialises");
    steered_low_target_checks();
    forward_brake_checks();
    check(motion_submit(&cmd) == MOTION_RESULT_OK, "command accepted");
    motion_loop();
    check(test_duty[0] == 25U && test_duty[1] == 25U,
          "50 mm/s command receives bounded startup drive");
    test_clock = 20000U;
    test_encoder.left_counts = 25;
    test_encoder.right_counts = 25;
    test_encoder.irq_count = 50U;
    motion_loop();
    check(motion_status.state == MOTION_STATE_RUNNING, "valid feedback drives normally");
    drives_before = drive_calls;
    integral_before = right_pid.integral;
    travel_before = motion_status.travelled_mm;
    test_clock = 40000U;
    test_encoder.left_counts = 50;
    test_encoder.right_counts = 1525;
    test_encoder.irq_count = 1575U;
    motion_loop();
    first_fault = motion_status;
    check(first_fault.fault == MOTION_FAULT_ENCODER &&
          first_fault.encoder_fault_wheel_mask == 2U, "first right count jump faults immediately");
    check(test_duty[0] == 0U && test_duty[1] == 0U && drive_calls == drives_before,
          "rejected sample brakes both motors without a drive update");
    check(right_pid.integral == integral_before && motion_status.travelled_mm == travel_before,
          "rejected sample cannot update PID or distance");
    check(first_fault.encoder_fault_right_delta == 1500 &&
          first_fault.encoder_fault_previous_right == 25 && first_fault.right_encoder_counts == 1525 &&
          first_fault.encoder_fault_elapsed_ms == 20U && first_fault.encoder_irq_count == 1575U,
          "fault captures matching previous/current counts, interval and IRQ");
    test_clock = 60000U;
    test_encoder.right_counts = 5000;
    test_encoder.invalid_transitions = 100U;
    motion_loop();
    check(motion_status.encoder_fault == first_fault.encoder_fault &&
          motion_status.right_encoder_counts == first_fault.right_encoder_counts &&
          motion_status.encoder_sample_time_us == first_fault.encoder_sample_time_us,
          "subsequent idle samples preserve original fault evidence");
    check(motion_submit(&cmd) == MOTION_RESULT_BUSY, "fault requires acknowledgement");
    check(motion_stop() == MOTION_RESULT_OK, "stop acknowledgement accepted");
    motion_loop();
    check(motion_status.state == MOTION_STATE_STOPPED && !fault_latched,
          "stop clears fault and rebaselines encoder sample");
    cmd.sequence_id = 2U;
    check(motion_submit(&cmd) == MOTION_RESULT_OK, "retry accepted after acknowledgement");
    test_clock += 20000U;
    motion_loop();
    check(motion_status.state == MOTION_STATE_RUNNING, "retry starts with a fresh baseline");
    test_clock += 20000U;
    test_encoder.invalid_transitions += 9U;
    motion_loop();
    check(motion_status.encoder_fault == MOTION_ENCODER_FAULT_INVALID_TRANSITIONS &&
          test_duty[0] == 0U && test_duty[1] == 0U,
          "invalid transitions still brake immediately");

    (void)motion_stop();
    motion_loop();
    cmd.sequence_id = 3U;
    check(motion_submit(&cmd) == MOTION_RESULT_OK, "timing regression command accepted");
    motion_loop();
    /* 800 counts over 120 ms is about 544 mm/s with the loaded distance
       scale, below the physical bound. Pairing the counts with a
       pre-snapshot 20 ms timestamp falsely faults. */
    test_clock += 20000U;
    snapshot_delay_us = 100000U;
    test_encoder.left_counts += 800;
    test_encoder.right_counts += 800;
    motion_loop();
    check(motion_status.state == MOTION_STATE_RUNNING &&
          motion_status.fault == MOTION_FAULT_NONE,
          "preemption before snapshot does not create a false count fault");
    check(motion_status.encoder_sample_time_us == test_clock &&
          motion_status.encoder_snapshot_duration_us == 100000U &&
          motion_status.left_speed_mm_s >= 540 && motion_status.left_speed_mm_s <= 550,
          "speed uses the count acquisition time, with acquisition delay retained");
    test_clock += 20000U;
    test_encoder.left_counts += 28;
    test_encoder.right_counts += 28;
    motion_loop();
    check(motion_status.left_speed_mm_s >= 108 && motion_status.left_speed_mm_s <= 120,
          "sample after delayed acquisition uses its own 20 ms interval");
    drives_before = drive_calls;
    test_clock += 20000U;
    test_encoder.left_counts += 1500;
    motion_loop();
    check(motion_status.encoder_fault == MOTION_ENCODER_FAULT_COUNT_DELTA &&
          motion_status.encoder_fault_wheel_mask == 1U &&
          drive_calls == drives_before && test_duty[0] == 0U && test_duty[1] == 0U,
          "real count spike after timing correction still brakes immediately");
    /* Replay the user's 28 Oct diag-03 failure, retaining exact timestamps. */
    test_clock = 422617U;
    test_encoder.left_counts = 175;
    test_encoder.right_counts = 2;
    test_encoder.irq_count = 177U;
    (void)motion_stop();
    motion_loop();
    cmd.sequence_id = 4U;
    (void)motion_submit(&cmd);
    motion_loop();
    test_clock = 436680U;
    test_encoder.left_counts = 774;
    test_encoder.irq_count = 776U;
    motion_loop();
    check(motion_status.encoder_fault == MOTION_ENCODER_FAULT_COUNT_DELTA &&
          motion_status.encoder_fault_wheel_mask == 1U &&
          motion_status.encoder_fault_elapsed_ms == 14U &&
          motion_status.encoder_fault_left_delta == 599 &&
          motion_status.encoder_fault_right_delta == 0 &&
          test_duty[0] == 0U && test_duty[1] == 0U,
          "reported 599/0 counts over 14063 us correctly latch a left fault and brake");
    brake_result = MOTION_RESULT_HARDWARE;
    (void)motion_stop();
    motion_loop();
    check(motion_status.state == MOTION_STATE_FAULT &&
          motion_status.fault == MOTION_FAULT_HARDWARE_CONFIG &&
          reinitialisation_required,
          "brake write failure becomes a latched hardware fault");
    return 0;
}
