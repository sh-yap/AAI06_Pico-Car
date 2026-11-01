#include <stdbool.h>
#include <stdint.h>

#include <sys/machine.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>

#include "motion_task.h"

#define ROUTE_RUN_BUTTON_PIN       20U
#define ROUTE_STOP_BUTTON_PIN      21U
#define ROUTE_STATUS_LED_PIN       BOARD_LED_PIN
#define ROUTE_BUZZER_PIN           22U
#define ROUTE_BUZZER_TOP           62499U
#define ROUTE_BUZZER_DUTY          31250U
#define ROUTE_POLL_MS              20U
#define ROUTE_DEBOUNCE_MS          30U
#define ROUTE_PRE_RUN_DELAY_MS     1000U
#define ROUTE_BETWEEN_STEPS_MS     300U
#define ROUTE_STRAIGHT_SPEED_MM_S  50
#define ROUTE_TURN_SPEED_MM_S      80
#define ROUTE_TURN_TIMEOUT_MS      8000U

typedef struct {
    motion_command_type_t type;
    int32_t value;
    const char *name;
} route_step_t;

typedef struct {
    uint32_t sequence_id;
    int32_t travelled_mm;
    int32_t left_counts;
    int32_t right_counts;
    uint32_t left_invalid;
    uint32_t right_invalid;
} route_step_result_t;

static const route_step_t route_steps[] = {
    { MOTION_COMMAND_MOVE_DISTANCE,  500, "forward-500-1" },
    { MOTION_COMMAND_TURN_ANGLE,   -90000, "left-90" },
    { MOTION_COMMAND_MOVE_DISTANCE,  500, "forward-500-2" },
    { MOTION_COMMAND_TURN_ANGLE,    90000, "right-90" },
    { MOTION_COMMAND_MOVE_DISTANCE, 1000, "forward-1000" },
    { MOTION_COMMAND_TURN_ANGLE,   180000, "right-u-turn-180" },
    { MOTION_COMMAND_MOVE_DISTANCE, 1500, "forward-1500" }
};

static bool buzzer_ready;
static uint32_t next_sequence = 1U;

static uint8_t route_step_count(void)
{
    return (uint8_t)(sizeof(route_steps) / sizeof(route_steps[0]));
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
    (void)gpio_set_val(ROUTE_STATUS_LED_PIN, enabled ? 1U : 0U);
}

static void buzzer_set(bool enabled)
{
    if (buzzer_ready) {
        (void)pwm_set_enabled(ROUTE_BUZZER_PIN, enabled ? TRUE : FALSE);
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
        (void)tk_dly_tsk(ROUTE_POLL_MS);
    }
    (void)tk_dly_tsk(ROUTE_DEBOUNCE_MS);
}

static bool accept_press(uint8_t pin)
{
    if (!button_pressed(pin)) {
        return false;
    }
    (void)tk_dly_tsk(ROUTE_DEBOUNCE_MS);
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

static bool wait_with_stop(uint32_t delay_ms)
{
    uint32_t elapsed_ms = 0U;

    while (elapsed_ms < delay_ms) {
        if (button_pressed(ROUTE_STOP_BUTTON_PIN)) {
            (void)motion_stop();
            wait_released(ROUTE_STOP_BUTTON_PIN);
            return false;
        }
        led_set((elapsed_ms % 200U) < 100U);
        (void)tk_dly_tsk(ROUTE_POLL_MS);
        elapsed_ms += ROUTE_POLL_MS;
    }
    led_set(false);
    return true;
}

static uint32_t distance_timeout_ms(int32_t distance_mm)
{
    uint32_t nominal_ms = ((uint32_t)distance_mm * 1000U) /
                          (uint32_t)ROUTE_STRAIGHT_SPEED_MM_S;

    return (nominal_ms * 2U) + 3000U;
}

static void prepare_command(const route_step_t *step,
                            motion_command_t *command)
{
    *command = (motion_command_t){0};
    command->type = step->type;
    command->sequence_id = next_sequence++;
    if (step->type == MOTION_COMMAND_MOVE_DISTANCE) {
        command->timeout_ms = distance_timeout_ms(step->value);
        command->value.distance.distance_mm = step->value;
        command->value.distance.speed_mm_s = ROUTE_STRAIGHT_SPEED_MM_S;
    } else {
        command->timeout_ms = ROUTE_TURN_TIMEOUT_MS;
        command->value.turn.angle_mdeg = step->value;
        command->value.turn.wheel_speed_mm_s = ROUTE_TURN_SPEED_MM_S;
    }
}

static void store_result(route_step_result_t *result,
                         const motion_status_t *status)
{
    result->sequence_id = status->active_sequence_id;
    result->travelled_mm = status->travelled_mm;
    result->left_counts = status->left_encoder_counts;
    result->right_counts = status->right_encoder_counts;
    result->left_invalid = status->left_invalid_encoder_transitions;
    result->right_invalid = status->right_invalid_encoder_transitions;
}

static void print_results(const route_step_result_t *results,
                          uint8_t completed_steps,
                          uint8_t result_code)
{
    uint8_t index;

    tm_printf((UB *)"[motion-route] completed=%u/%u result=%u\n",
              completed_steps, route_step_count(), result_code);
    for (index = 0U; index < completed_steps; ++index) {
        tm_printf((UB *)"[motion-route] step=%u name=%s seq=%u travel=%d "
                  "counts=%d/%d invalid=%u/%u\n",
                  (uint32_t)(index + 1U), (UB *)route_steps[index].name,
                  results[index].sequence_id, results[index].travelled_mm,
                  results[index].left_counts, results[index].right_counts,
                  results[index].left_invalid, results[index].right_invalid);
    }
}

static uint8_t wait_for_step(uint32_t sequence,
                             route_step_result_t *step_result)
{
    motion_status_t status = {0};

    for (;;) {
        if (button_pressed(ROUTE_STOP_BUTTON_PIN)) {
            (void)motion_stop();
            wait_released(ROUTE_STOP_BUTTON_PIN);
            return 2U;
        }
        if (motion_get_status(&status) == MOTION_RESULT_OK &&
            status.active_sequence_id == sequence) {
            if (status.state == MOTION_STATE_COMPLETE) {
                store_result(step_result, &status);
                return 1U;
            }
            if (status.state == MOTION_STATE_FAULT) {
                return fault_code(status.fault);
            }
        }
        (void)tk_dly_tsk(ROUTE_POLL_MS);
    }
}

static uint8_t run_route(void)
{
    motion_command_t command;
    route_step_result_t results[sizeof(route_steps) / sizeof(route_steps[0])] = {0};
    motion_result_t submit_result;
    uint8_t index;
    uint8_t result_code = 1U;
    uint8_t completed_steps = 0U;

    if (!wait_with_stop(ROUTE_PRE_RUN_DELAY_MS)) {
        beep_code(2U);
        return 2U;
    }

    for (index = 0U; index < route_step_count(); ++index) {
        if (index != 0U && !wait_with_stop(ROUTE_BETWEEN_STEPS_MS)) {
            result_code = 2U;
            break;
        }
        prepare_command(&route_steps[index], &command);
        submit_result = motion_submit(&command);
        if (submit_result != MOTION_RESULT_OK) {
            (void)motion_stop();
            result_code = 8U;
            break;
        }
        result_code = wait_for_step(command.sequence_id, &results[index]);
        if (result_code != 1U) {
            break;
        }
        ++completed_steps;
    }

    beep_code(result_code);
    print_results(results, completed_steps, result_code);
    return result_code;
}

EXPORT INT usermain(void)
{
    motion_status_t status;

    if (gpio_set_pin(ROUTE_STATUS_LED_PIN, GPIO_MODE_OUT) < E_OK ||
        !button_init(ROUTE_RUN_BUTTON_PIN) ||
        !button_init(ROUTE_STOP_BUTTON_PIN)) {
        fatal_beep();
    }
    buzzer_ready = pwm_set_pin(ROUTE_BUZZER_PIN) >= E_OK &&
                   pwm_set_wrap(ROUTE_BUZZER_PIN, ROUTE_BUZZER_TOP) >= E_OK &&
                   pwm_set_cc(ROUTE_BUZZER_PIN, ROUTE_BUZZER_DUTY) >= E_OK;
    buzzer_set(false);
    led_set(false);
    wait_released(ROUTE_RUN_BUTTON_PIN);
    wait_released(ROUTE_STOP_BUTTON_PIN);
    if (motion_init() != MOTION_RESULT_OK ||
        motion_start_task() != MOTION_RESULT_OK) {
        fatal_beep();
    }
    beep_code(1U);

    for (;;) {
        if (motion_get_status(&status) == MOTION_RESULT_OK &&
            status.state == MOTION_STATE_FAULT) {
            if (accept_press(ROUTE_STOP_BUTTON_PIN)) {
                wait_released(ROUTE_STOP_BUTTON_PIN);
                (void)motion_stop();
                beep_code(2U);
            }
        } else if (accept_press(ROUTE_RUN_BUTTON_PIN)) {
            wait_released(ROUTE_RUN_BUTTON_PIN);
            (void)run_route();
        }
        (void)tk_dly_tsk(ROUTE_POLL_MS);
    }
    return 0;
}
