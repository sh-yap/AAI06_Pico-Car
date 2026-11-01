#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>

#include "motion_task.h"
#include "motion_clock.h"

#define CLOSED_LOOP_RUN_BUTTON_PIN 20U
#define CLOSED_LOOP_STOP_BUTTON_PIN 21U
#define CLOSED_LOOP_STATUS_LED_PIN BOARD_LED_PIN
#define CLOSED_LOOP_BUZZER_PIN 22U
#define CLOSED_LOOP_BUZZER_TOP 62499U
#define CLOSED_LOOP_BUZZER_DUTY 31250U
#define CLOSED_LOOP_POLL_MS 10U
#define CLOSED_LOOP_DEBOUNCE_MS 30U
#define CLOSED_LOOP_DISTANCE_MM 300
#define CLOSED_LOOP_SPEED_MM_S 100
#define CLOSED_LOOP_TIMEOUT_MS 6000U

static uint32_t next_sequence = 1U;
static bool buzzer_ready;
static INT encoder_init_processor;
static motion_status_t last_result;
static bool last_result_valid;
static bool defer_result_print;
static bool last_run_detached;
static uint32_t run_usb_irq_start;
static uint32_t last_usb_irq_end;

#if TM_CONSOLE_USB_CDC
extern INT tm_usb_console_getchar(void);
extern UW tm_usb_state(void);
extern volatile UW tm_usb_irq_count;
#if TM_USB_HOST_DIAGNOSTIC
extern ER tm_usb_diag_set_attached(BOOL attached);
extern BOOL tm_usb_diag_is_detached(void);
#endif
#endif

/* This same image runs on USB and powerbank. Print only after Motion has
 * braked so console output cannot disturb the measured encoder stream. */
static void print_result(void)
{
    const motion_status_t *s = &last_result;
    if (!last_result_valid) {
        tm_printf((UB *)"[button-diag-03] GP20=300mm GP21=stop; n=USB data-off trial; p=replay\n");
        return;
    }
    tm_printf((UB *)"[button-diag-03] init_core=%d seq=%u state=%d fault=%d detail=%d wheel=%u\n",
              encoder_init_processor, s->active_sequence_id, s->state,
              s->fault, s->encoder_fault, s->encoder_fault_wheel_mask);
#if TM_CONSOLE_USB_CDC
    tm_printf((UB *)"[button-diag] usb_state_at_print=%u (1=no host, 2=mounted, 3=serial open)\n",
              tm_usb_state());
    tm_printf((UB *)"[button-diag] data_off_trial=%u usb_irq_start=%u usb_irq_end=%u\n",
              last_run_detached ? 1U : 0U, run_usb_irq_start, last_usb_irq_end);
#endif
    (void)tk_dly_tsk(20U);
    tm_printf((UB *)"[button-diag] dt_ms=%u prev_us=%u sample_us=%u read_us=%u\n",
              s->encoder_fault_elapsed_ms,
              (uint32_t)s->encoder_fault_previous_time_us,
              (uint32_t)s->encoder_sample_time_us,
              s->encoder_snapshot_duration_us);
    (void)tk_dly_tsk(20U);
    tm_printf((UB *)"[button-diag] prev=%d/%d count=%d/%d delta=%d/%d\n",
              s->encoder_fault_previous_left, s->encoder_fault_previous_right,
              s->left_encoder_counts, s->right_encoder_counts,
              s->encoder_fault_left_delta, s->encoder_fault_right_delta);
    (void)tk_dly_tsk(20U);
    tm_printf((UB *)"[button-diag] irq_prev=%u irq=%u invalid=%u/%u\n",
              s->encoder_fault_previous_irq, s->encoder_irq_count,
              s->left_invalid_encoder_transitions,
              s->right_invalid_encoder_transitions);
    (void)tk_dly_tsk(20U);
    tm_printf((UB *)"[button-diag] last_speed=%d/%d pre_fault_pwm=%u/%u pwm=%u/%u travel=%d target=%d\n",
              s->left_speed_mm_s, s->right_speed_mm_s,
              s->encoder_fault_previous_left_pwm,
              s->encoder_fault_previous_right_pwm,
              s->left_pwm_permille, s->right_pwm_permille,
              s->travelled_mm, s->target_mm);
}

static void save_result(const motion_status_t *status)
{
    last_result = *status;
    last_result_valid = true;
#if TM_CONSOLE_USB_CDC
    last_usb_irq_end = tm_usb_irq_count;
#endif
    if (!defer_result_print) {
        print_result();
    }
}

static bool button_pressed(uint8_t pin)
{
    return gpio_get_val(pin) == 0U;
}

static bool button_init(uint8_t pin)
{
    if (gpio_set_pin(pin, GPIO_MODE_IN) < E_OK) {
        return false;
    }
    out_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_SHEMITT);
    return true;
}

static void led_set(bool enabled)
{
    (void)gpio_set_val(CLOSED_LOOP_STATUS_LED_PIN, enabled ? 1U : 0U);
}

static void buzzer_set(bool enabled)
{
    if (buzzer_ready) {
        (void)pwm_set_enabled(CLOSED_LOOP_BUZZER_PIN,
                              enabled ? TRUE : FALSE);
    }
}

static void status_set(bool enabled)
{
    led_set(enabled);
    buzzer_set(enabled);
}

static void beep(uint32_t on_ms)
{
    status_set(true);
    (void)tk_dly_tsk(on_ms);
    status_set(false);
}

static void beep_code(uint8_t count)
{
    uint8_t index;

    (void)tk_dly_tsk(250U);
    for (index = 0U; index < count; ++index) {
        beep(150U);
        (void)tk_dly_tsk(150U);
    }
}

static void wait_released(uint8_t pin)
{
    while (button_pressed(pin)) {
        (void)tk_dly_tsk(CLOSED_LOOP_POLL_MS);
    }
    (void)tk_dly_tsk(CLOSED_LOOP_DEBOUNCE_MS);
}

static bool accept_press(uint8_t pin)
{
    if (!button_pressed(pin)) {
        return false;
    }
    (void)tk_dly_tsk(CLOSED_LOOP_DEBOUNCE_MS);
    if (!button_pressed(pin)) {
        return false;
    }
    beep(80U);
    return true;
}

static motion_result_t submit_straight_move(uint32_t *sequence)
{
    motion_command_t command = {0};

    command.type = MOTION_COMMAND_MOVE_DISTANCE;
    command.sequence_id = next_sequence++;
    command.timeout_ms = CLOSED_LOOP_TIMEOUT_MS;
    command.value.distance.distance_mm = CLOSED_LOOP_DISTANCE_MM;
    command.value.distance.speed_mm_s = CLOSED_LOOP_SPEED_MM_S;
    *sequence = command.sequence_id;
    return motion_submit(&command);
}

static uint8_t fault_beep_code(motion_fault_t fault)
{
    switch (fault) {
    case MOTION_FAULT_TIMEOUT:
        return 3U;
    case MOTION_FAULT_STALL:
        return 4U;
    case MOTION_FAULT_ENCODER:
        return 5U;
    case MOTION_FAULT_INVALID_COMMAND:
        return 6U;
    case MOTION_FAULT_HARDWARE_CONFIG:
        return 7U;
    case MOTION_FAULT_NONE:
    default:
        return 7U;
    }
}

static uint8_t encoder_detail_beep_code(motion_encoder_fault_t detail)
{
    switch (detail) {
    case MOTION_ENCODER_FAULT_COUNT_DELTA:
        return 1U;
    case MOTION_ENCODER_FAULT_SPEED_CALCULATION:
        return 2U;
    case MOTION_ENCODER_FAULT_INVALID_TRANSITIONS:
        return 3U;
    case MOTION_ENCODER_FAULT_NONE:
    default:
        return 4U;
    }
}

static uint8_t encoder_elapsed_beep_code(uint32_t elapsed_ms)
{
    if (elapsed_ms <= 5U) {
        return 1U;
    }
    if (elapsed_ms <= 15U) {
        return 2U;
    }
    if (elapsed_ms <= 30U) {
        return 3U;
    }
    return 4U;
}

static uint8_t encoder_delta_beep_code(int32_t left_delta,
                                       int32_t right_delta)
{
    int64_t left_magnitude = left_delta < 0 ? -(int64_t)left_delta : left_delta;
    int64_t right_magnitude = right_delta < 0 ? -(int64_t)right_delta : right_delta;
    int64_t maximum = left_magnitude > right_magnitude ?
                      left_magnitude : right_magnitude;

    if (maximum <= 100) {
        return 1U;
    }
    if (maximum <= 250) {
        return 2U;
    }
    if (maximum <= 500) {
        return 3U;
    }
    if (maximum <= 1000) {
        return 4U;
    }
    return 5U;
}

static uint8_t run_straight_test(void)
{
    motion_status_t status;
    motion_result_t result;
    uint32_t sequence = 0U;

    last_result_valid = false;
#if TM_CONSOLE_USB_CDC
    run_usb_irq_start = tm_usb_irq_count;
#endif
    result = submit_straight_move(&sequence);
    if (result != MOTION_RESULT_OK) {
        beep_code(8U);
        return 8U;
    }
    for (;;) {
        if (button_pressed(CLOSED_LOOP_STOP_BUTTON_PIN)) {
            (void)motion_stop();
            wait_released(CLOSED_LOOP_STOP_BUTTON_PIN);
            beep_code(2U);
            return 2U;
        }
        if (motion_get_status(&status) == MOTION_RESULT_OK &&
            status.active_sequence_id == sequence) {
            if (status.state == MOTION_STATE_COMPLETE) {
                save_result(&status);
                beep_code(1U);
                return 1U;
            } else if (status.state == MOTION_STATE_FAULT) {
                uint8_t code = fault_beep_code(status.fault);
                save_result(&status);
                beep_code(code);
                if (status.fault == MOTION_FAULT_ENCODER) {
                    /* Preserve the public five-beep encoder code, then emit a
                     * second group identifying which encoder check failed. */
                    (void)tk_dly_tsk(700U);
                    beep_code(encoder_detail_beep_code(status.encoder_fault));
                    if (status.encoder_fault ==
                        MOTION_ENCODER_FAULT_COUNT_DELTA) {
                        /* For a count-delta fault, add wheel, elapsed-time and
                         * delta-magnitude groups, separated by long pauses. */
                        (void)tk_dly_tsk(700U);
                        beep_code(status.encoder_fault_wheel_mask >= 1U &&
                                  status.encoder_fault_wheel_mask <= 3U ?
                                  status.encoder_fault_wheel_mask : 4U);
                        (void)tk_dly_tsk(700U);
                        beep_code(encoder_elapsed_beep_code(
                            status.encoder_fault_elapsed_ms));
                        (void)tk_dly_tsk(700U);
                        beep_code(encoder_delta_beep_code(
                            status.encoder_fault_left_delta,
                            status.encoder_fault_right_delta));
                    }
                }
                return code;
            }
        }
        (void)tk_dly_tsk(CLOSED_LOOP_POLL_MS);
    }
}

#if TM_CONSOLE_USB_CDC && TM_USB_HOST_DIAGNOSTIC
static bool wait_usb_detached(bool expected)
{
    uint64_t deadline_us = motion_clock_us() + 1000000ULL;
    do {
        if ((tm_usb_diag_is_detached() != FALSE) == expected) {
            return true;
        }
        (void)tk_dly_tsk(CLOSED_LOOP_POLL_MS);
    } while (motion_clock_us() < deadline_us);
    return false;
}

/* Keep computer power, cable, encoder/PWM settings and Motion unchanged.
 * Only the USB service disconnects/reconnects its data pull-up. Preserve the
 * final snapshot in RAM so it can be replayed when COM reappears. */
static void run_usb_isolation_test(void)
{
    motion_status_t status;

    if (tm_usb_state() < 2U) {
        tm_printf((UB *)"[usb-isolation] requires computer USB; no move submitted\n");
        return;
    }
    if (motion_get_status(&status) != MOTION_RESULT_OK ||
        status.state == MOTION_STATE_RUNNING || status.state == MOTION_STATE_FAULT) {
        tm_printf((UB *)"[usb-isolation] not ready; GP21 acknowledges a fault before retry\n");
        return;
    }
    tm_printf((UB *)"[usb-isolation] Keep wheels raised. USB data will disconnect, power stays on. GP21 stops.\n");
    last_result_valid = false;
    (void)tk_dly_tsk(500U);
    if (tm_usb_diag_set_attached(FALSE) != E_OK || !wait_usb_detached(true)) {
        (void)tm_usb_diag_set_attached(TRUE);
        tm_printf((UB *)"[usb-isolation] detach not acknowledged; no move submitted\n");
        return;
    }
    (void)tk_dly_tsk(1000U);
    if (!button_pressed(CLOSED_LOOP_STOP_BUTTON_PIN)) {
        last_run_detached = true;
        defer_result_print = true;
        (void)run_straight_test();
        defer_result_print = false;
    }
    /* All run exits, including rejection and operator stop, restore USB. */
    (void)tm_usb_diag_set_attached(TRUE);
    if (wait_usb_detached(false)) {
        (void)tk_dly_tsk(1000U);
        tm_printf((UB *)"[usb-isolation] USB restored; reopen Serial Monitor and send p. Do not reset.\n");
        print_result();
    }
}
#endif

EXPORT INT usermain(void)
{
    motion_result_t init_result;

    if (gpio_set_pin(CLOSED_LOOP_STATUS_LED_PIN, GPIO_MODE_OUT) < E_OK ||
        !button_init(CLOSED_LOOP_RUN_BUTTON_PIN) ||
        !button_init(CLOSED_LOOP_STOP_BUTTON_PIN)) {
        for (;;) {
            beep_code(4U);
        }
    }
    buzzer_ready = pwm_set_pin(CLOSED_LOOP_BUZZER_PIN) >= E_OK &&
                   pwm_set_wrap(CLOSED_LOOP_BUZZER_PIN,
                                CLOSED_LOOP_BUZZER_TOP) >= E_OK &&
                   pwm_set_cc(CLOSED_LOOP_BUZZER_PIN,
                              CLOSED_LOOP_BUZZER_DUTY) >= E_OK;
    if (buzzer_ready) {
        buzzer_set(false);
    }
    led_set(false);
    wait_released(CLOSED_LOOP_RUN_BUTTON_PIN);
    wait_released(CLOSED_LOOP_STOP_BUTTON_PIN);

    encoder_init_processor = tk_get_prc();
    init_result = motion_init();
    if (init_result != MOTION_RESULT_OK ||
        motion_start_task() != MOTION_RESULT_OK) {
        for (;;) {
            beep_code(4U);
        }
    }
    beep_code(1U);

    for (;;) {
#if TM_CONSOLE_USB_CDC
        INT key = tm_usb_console_getchar();
        if (key == 'p' || key == 'h') {
            print_result();
        }
#if TM_USB_HOST_DIAGNOSTIC
        else if (key == 'n') {
            run_usb_isolation_test();
        }
#endif
#endif
        if (accept_press(CLOSED_LOOP_RUN_BUTTON_PIN)) {
            wait_released(CLOSED_LOOP_RUN_BUTTON_PIN);
            /* Give the operator one second to remove their hand before the
             * closed-loop move starts. */
            (void)tk_dly_tsk(1000U);
            last_run_detached = false;
            (void)run_straight_test();
        } else if (accept_press(CLOSED_LOOP_STOP_BUTTON_PIN)) {
            wait_released(CLOSED_LOOP_STOP_BUTTON_PIN);
            (void)motion_stop();
            beep_code(2U);
        }
        (void)tk_dly_tsk(CLOSED_LOOP_POLL_MS);
    }
    return 0;
}
