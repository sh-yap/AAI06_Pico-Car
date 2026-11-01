/* Exercise the actual button diagnostic orchestration, including failure
 * exits. Electrical detach and real USB enumeration still need a board. */
#include <stdio.h>
#include <stdlib.h>
#define TM_CONSOLE_USB_CDC 1
#define TM_USB_HOST_DIAGNOSTIC 1
#include "../../app_program/motion_closed_loop_button_test_main.c"

static uint64_t test_clock;
static BOOL attached;
static bool host_present;
static bool fail_detach;
static bool stop_during_run;
static bool submitted;
static bool submitted_without_detach;
static unsigned restores;
static unsigned stops;
static unsigned submits;
static motion_result_t submit_result;
static motion_state_t final_state;
static motion_status_t test_status;
volatile UW tm_usb_irq_count;

static void check(bool ok, const char *name)
{
    if (!ok) { fprintf(stderr, "FAIL: %s\n", name); exit(1); }
    printf("PASS: %s\n", name);
}

uint32_t in_w(uint32_t address)
{ return address == TIMER_TIMERAWL ? (uint32_t)test_clock : (uint32_t)(test_clock >> 32); }
void out_w(uint32_t address, uint32_t value) { (void)address; (void)value; }
ER tk_dly_tsk(RELTIM ms) { test_clock += (uint64_t)ms * 1000U; return E_OK; }
INT tk_get_prc(void) { return 1; }
INT tm_printf(const UB *format, ...) { (void)format; return 0; }
INT tm_usb_console_getchar(void) { return -1; }
UW tm_usb_state(void) { return attached && host_present ? 3U : 1U; }
ER tm_usb_diag_set_attached(BOOL value)
{
    if (value) ++restores;
    if (value || !fail_detach) attached = value;
    return E_OK;
}
BOOL tm_usb_diag_is_detached(void) { return !attached; }
ER gpio_set_pin(UINT pin, UINT mode) { (void)pin; (void)mode; return E_OK; }
ER gpio_set_val(UINT pin, UINT value) { (void)pin; (void)value; return E_OK; }
UINT gpio_get_val(UINT pin)
{
    if (pin == CLOSED_LOOP_STOP_BUTTON_PIN && submitted && stop_during_run) {
        stop_during_run = false;
        return 0U;
    }
    return 1U;
}
ER pwm_set_pin(UINT pin) { (void)pin; return E_OK; }
ER pwm_set_wrap(UINT pin, UW value) { (void)pin; (void)value; return E_OK; }
ER pwm_set_cc(UINT pin, UW value) { (void)pin; (void)value; return E_OK; }
ER pwm_set_enabled(UINT pin, BOOL enabled) { (void)pin; (void)enabled; return E_OK; }
motion_result_t motion_init(void) { return MOTION_RESULT_OK; }
motion_result_t motion_start_task(void) { return MOTION_RESULT_OK; }
motion_result_t motion_submit(const motion_command_t *command)
{
    ++submits;
    submitted_without_detach = attached != FALSE;
    if (submit_result != MOTION_RESULT_OK) return submit_result;
    submitted = true;
    test_status.active_sequence_id = command->sequence_id;
    return MOTION_RESULT_OK;
}
motion_result_t motion_stop(void) { ++stops; return MOTION_RESULT_OK; }
motion_result_t motion_get_status(motion_status_t *status)
{
    if (submitted) {
        test_status.state = final_state;
        if (final_state == MOTION_STATE_FAULT) {
            test_status.fault = MOTION_FAULT_ENCODER;
            test_status.encoder_fault = MOTION_ENCODER_FAULT_COUNT_DELTA;
            test_status.encoder_fault_wheel_mask = 1U;
            test_status.encoder_fault_left_delta = 400;
        }
    }
    *status = test_status;
    return MOTION_RESULT_OK;
}

static void reset_case(void)
{
    test_clock = 0U;
    attached = TRUE;
    host_present = true;
    fail_detach = false;
    stop_during_run = false;
    submitted = false;
    submitted_without_detach = false;
    restores = stops = submits = 0U;
    submit_result = MOTION_RESULT_OK;
    final_state = MOTION_STATE_COMPLETE;
    test_status = (motion_status_t){0};
    test_status.state = MOTION_STATE_IDLE;
    last_result_valid = false;
    defer_result_print = false;
}

int main(void)
{
    reset_case();
    host_present = false;
    run_usb_isolation_test();
    check(submits == 0U, "USB isolation refuses movement without an attached host");
    reset_case();
    fail_detach = true;
    run_usb_isolation_test();
    check(submits == 0U && restores == 1U && attached,
          "detach timeout submits no motion and requests restoration");
    reset_case();
    test_status.state = MOTION_STATE_FAULT;
    run_usb_isolation_test();
    check(submits == 0U, "USB isolation does not clear a latched fault implicitly");
    reset_case();
    run_usb_isolation_test();
    check(submits == 1U && !submitted_without_detach && attached && restores == 1U &&
          last_result_valid && last_result.state == MOTION_STATE_COMPLETE,
          "completion restores USB and retains the detached-run report");
    reset_case();
    final_state = MOTION_STATE_FAULT;
    run_usb_isolation_test();
    check(attached && restores == 1U && last_result_valid &&
          last_result.encoder_fault_left_delta == 400 && !defer_result_print,
          "encoder fault restores USB and preserves exact fault evidence");
    reset_case();
    submit_result = MOTION_RESULT_BUSY;
    run_usb_isolation_test();
    check(!submitted && attached && restores == 1U,
          "command rejection also restores USB");
    reset_case();
    stop_during_run = true;
    run_usb_isolation_test();
    check(stops == 1U && attached && restores == 1U,
          "GP21 operator stop remains effective and restores USB");
    return 0;
}
