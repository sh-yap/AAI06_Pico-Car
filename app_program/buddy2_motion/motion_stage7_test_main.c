#include <stdbool.h>
#include <stdint.h>

#include <sys/machine.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>

#include "motion_task.h"

#define STAGE7_RUN_BUTTON_PIN       20U
#define STAGE7_NEXT_STOP_BUTTON_PIN 21U
#define STAGE7_STATUS_LED_PIN       BOARD_LED_PIN
#define STAGE7_BUZZER_PIN           22U
#define STAGE7_BUZZER_TOP           62499U
#define STAGE7_BUZZER_DUTY          31250U
#define STAGE7_POLL_MS              20U
#define STAGE7_DEBOUNCE_MS          30U
#define STAGE7_PRE_RUN_DELAY_MS     1000U
#define STAGE7_WHEEL_SPEED_MM_S     80
#define STAGE7_TIMEOUT_MS           8000U

typedef struct {
    int32_t angle_mdeg;
    const char *name;
} stage7_case_t;

static const stage7_case_t stage7_cases[] = {
    { -90000, "left-90" },
    {  90000, "right-90" },
    { 180000, "u-turn-180" }
};

static bool buzzer_ready;
static uint32_t next_sequence = 1U;

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
    (void)gpio_set_val(STAGE7_STATUS_LED_PIN, enabled ? 1U : 0U);
}

static void buzzer_set(bool enabled)
{
    if (buzzer_ready) {
        (void)pwm_set_enabled(STAGE7_BUZZER_PIN, enabled ? TRUE : FALSE);
    }
}

static void indication_set(bool enabled)
{
    led_set(enabled);
    buzzer_set(enabled);
}

static void beep(uint32_t on_ms)
{
    indication_set(true);
    (void)tk_dly_tsk(on_ms);
    indication_set(false);
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
        (void)tk_dly_tsk(STAGE7_POLL_MS);
    }
    (void)tk_dly_tsk(STAGE7_DEBOUNCE_MS);
}

static bool accept_press(uint8_t pin)
{
    if (!button_pressed(pin)) {
        return false;
    }
    (void)tk_dly_tsk(STAGE7_DEBOUNCE_MS);
    if (!button_pressed(pin)) {
        return false;
    }
    beep(80U);
    return true;
}

static uint8_t fault_code(motion_fault_t fault)
{
    switch (fault) {
    case MOTION_FAULT_TIMEOUT: return 3U;
    case MOTION_FAULT_STALL: return 4U;
    case MOTION_FAULT_ENCODER: return 5U;
    case MOTION_FAULT_INVALID_COMMAND: return 6U;
    case MOTION_FAULT_HARDWARE_CONFIG: return 7U;
    case MOTION_FAULT_NONE:
    default: return 7U;
    }
}

static void fatal_beep(void)
{
    (void)motion_stop();
    for (;;) {
        beep_code(9U);
        (void)tk_dly_tsk(1000U);
    }
}

static bool wait_before_run(void)
{
    uint32_t elapsed_ms = 0U;

    while (elapsed_ms < STAGE7_PRE_RUN_DELAY_MS) {
        if (button_pressed(STAGE7_NEXT_STOP_BUTTON_PIN)) {
            (void)motion_stop();
            wait_released(STAGE7_NEXT_STOP_BUTTON_PIN);
            return false;
        }
        led_set((elapsed_ms % 200U) < 100U);
        (void)tk_dly_tsk(STAGE7_POLL_MS);
        elapsed_ms += STAGE7_POLL_MS;
    }
    led_set(false);
    return true;
}

static uint8_t run_case(const stage7_case_t *test_case)
{
    motion_command_t command = {0};
    motion_status_t status = {0};
    motion_result_t result;
    uint32_t sequence;

    if (!wait_before_run()) {
        beep_code(2U);
        return 2U;
    }
    command.type = MOTION_COMMAND_TURN_ANGLE;
    command.sequence_id = next_sequence++;
    command.timeout_ms = STAGE7_TIMEOUT_MS;
    command.value.turn.angle_mdeg = test_case->angle_mdeg;
    command.value.turn.wheel_speed_mm_s = STAGE7_WHEEL_SPEED_MM_S;
    sequence = command.sequence_id;
    result = motion_submit(&command);
    if (result != MOTION_RESULT_OK) {
        beep_code(8U);
        return 8U;
    }

    for (;;) {
        if (button_pressed(STAGE7_NEXT_STOP_BUTTON_PIN)) {
            (void)motion_stop();
            wait_released(STAGE7_NEXT_STOP_BUTTON_PIN);
            beep_code(2U);
            return 2U;
        }
        if (motion_get_status(&status) == MOTION_RESULT_OK &&
            status.active_sequence_id == sequence) {
            if (status.state == MOTION_STATE_COMPLETE) {
                tm_printf((UB *)"[motion-stage7] case=%s angle_mdeg=%d "
                          "counts=%d/%d invalid=%u/%u seq=%u complete\n",
                          (UB *)test_case->name, test_case->angle_mdeg,
                          status.left_encoder_counts,
                          status.right_encoder_counts,
                          status.left_invalid_encoder_transitions,
                          status.right_invalid_encoder_transitions,
                          status.active_sequence_id);
                beep_code(1U);
                return 1U;
            }
            if (status.state == MOTION_STATE_FAULT) {
                uint8_t code = fault_code(status.fault);
                beep_code(code);
                return code;
            }
        }
        (void)tk_dly_tsk(STAGE7_POLL_MS);
    }
}

EXPORT INT usermain(void)
{
    motion_status_t status;
    uint8_t case_index = 0U;
    uint8_t case_count = (uint8_t)(sizeof(stage7_cases) /
                                   sizeof(stage7_cases[0]));

    if (gpio_set_pin(STAGE7_STATUS_LED_PIN, GPIO_MODE_OUT) < E_OK ||
        !button_init(STAGE7_RUN_BUTTON_PIN) ||
        !button_init(STAGE7_NEXT_STOP_BUTTON_PIN)) {
        fatal_beep();
    }
    buzzer_ready = pwm_set_pin(STAGE7_BUZZER_PIN) >= E_OK &&
                   pwm_set_wrap(STAGE7_BUZZER_PIN,
                                STAGE7_BUZZER_TOP) >= E_OK &&
                   pwm_set_cc(STAGE7_BUZZER_PIN,
                              STAGE7_BUZZER_DUTY) >= E_OK;
    buzzer_set(false);
    led_set(false);
    wait_released(STAGE7_RUN_BUTTON_PIN);
    wait_released(STAGE7_NEXT_STOP_BUTTON_PIN);
    if (motion_init() != MOTION_RESULT_OK ||
        motion_start_task() != MOTION_RESULT_OK) {
        fatal_beep();
    }
    beep_code(1U);

    for (;;) {
        if (motion_get_status(&status) == MOTION_RESULT_OK &&
            status.state == MOTION_STATE_FAULT) {
            if (accept_press(STAGE7_NEXT_STOP_BUTTON_PIN)) {
                wait_released(STAGE7_NEXT_STOP_BUTTON_PIN);
                (void)motion_stop();
                beep_code(2U);
            }
        } else if (accept_press(STAGE7_NEXT_STOP_BUTTON_PIN)) {
            wait_released(STAGE7_NEXT_STOP_BUTTON_PIN);
            case_index = (uint8_t)((case_index + 1U) % case_count);
            beep_code((uint8_t)(case_index + 1U));
        } else if (accept_press(STAGE7_RUN_BUTTON_PIN)) {
            wait_released(STAGE7_RUN_BUTTON_PIN);
            (void)run_case(&stage7_cases[case_index]);
        }
        (void)tk_dly_tsk(STAGE7_POLL_MS);
    }
    return 0;
}
