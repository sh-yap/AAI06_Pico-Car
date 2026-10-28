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
{ test_duty[0] = 0U; test_duty[1] = 0U; return MOTION_RESULT_OK; }
uint16_t motion_motor_last_duty(uint8_t motor) { return test_duty[motor]; }

static void check(bool ok, const char *name)
{
    if (!ok) { fprintf(stderr, "FAIL: %s\n", name); exit(1); }
    printf("PASS: %s\n", name);
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
    cmd.value.distance.speed_mm_s = 100;
    check(motion_init() == MOTION_RESULT_OK, "task initialises");
    check(motion_submit(&cmd) == MOTION_RESULT_OK, "command accepted");
    motion_loop();
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
    /* 800 counts over 120 ms is about 477 mm/s, below the physical bound.
       Pairing the counts with a pre-snapshot 20 ms timestamp falsely faults. */
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
          motion_status.left_speed_mm_s >= 470 && motion_status.left_speed_mm_s <= 480,
          "speed uses the count acquisition time, with acquisition delay retained");
    test_clock += 20000U;
    test_encoder.left_counts += 28;
    test_encoder.right_counts += 28;
    motion_loop();
    check(motion_status.left_speed_mm_s >= 95 && motion_status.left_speed_mm_s <= 105,
          "sample after delayed acquisition uses its own 20 ms interval");
    drives_before = drive_calls;
    test_clock += 20000U;
    test_encoder.left_counts += 1500;
    motion_loop();
    check(motion_status.encoder_fault == MOTION_ENCODER_FAULT_COUNT_DELTA &&
          motion_status.encoder_fault_wheel_mask == 1U &&
          drive_calls == drives_before && test_duty[0] == 0U && test_duty[1] == 0U,
          "real count spike after timing correction still brakes immediately");
    return 0;
}
