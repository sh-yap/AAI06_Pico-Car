#include <stdbool.h>
#include <stdint.h>

#include <sys/machine.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>

#include "motion_task.h"

#define STAGE5_RUN_BUTTON_PIN 20U
#define STAGE5_NEXT_STOP_BUTTON_PIN 21U
#define STAGE5_STATUS_LED_PIN BOARD_LED_PIN
#define STAGE5_BUZZER_PIN 22U
#define STAGE5_BUZZER_TOP 62499U
#define STAGE5_BUZZER_DUTY 31250U
#define STAGE5_POLL_MS 20U
#define STAGE5_DEBOUNCE_MS 30U
#define STAGE5_LONG_PRESS_MS 1000U
#define STAGE5_PRE_RUN_DELAY_MS 1000U
#define STAGE5_DISTANCE_MM 500
#define STAGE5_STEADY_START_MS 1000U

typedef struct {
    int32_t speed_mm_s;
    bool reverse;
} stage5_case_t;

typedef struct {
    uint64_t started_us;
    uint32_t rise_time_ms;
    int32_t maximum_left_mm_s;
    int32_t maximum_right_mm_s;
    int64_t steady_left_sum;
    int64_t steady_right_sum;
    uint64_t steady_left_pwm_sum;
    uint64_t steady_right_pwm_sum;
    uint32_t steady_samples;
} stage5_stats_t;

/* Test one required Vehicle Controller operating point at a time. The
   50 mm/s point is complete; 80 mm/s is the current reverse/rescan setpoint. */
static const int32_t stage5_speeds_mm_s[] = { 80 };
static bool buzzer_ready;
static uint32_t next_sequence = 1U;

static int32_t magnitude_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static uint8_t case_count(void)
{
    return (uint8_t)((sizeof(stage5_speeds_mm_s) /
                      sizeof(stage5_speeds_mm_s[0])) * 2U);
}

static bool case_get(uint8_t index, stage5_case_t *test_case)
{
    uint8_t speed_count = (uint8_t)(sizeof(stage5_speeds_mm_s) /
                                    sizeof(stage5_speeds_mm_s[0]));

    if (test_case == 0 || index >= case_count()) {
        return false;
    }
    test_case->reverse = index >= speed_count;
    test_case->speed_mm_s = stage5_speeds_mm_s[index % speed_count];
    return true;
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
    (void)gpio_set_val(STAGE5_STATUS_LED_PIN, enabled ? 1U : 0U);
}

static void buzzer_set(bool enabled)
{
    if (buzzer_ready) {
        (void)pwm_set_enabled(STAGE5_BUZZER_PIN,
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

static void fatal_beep(void)
{
    (void)motion_stop();
    for (;;) {
        beep_code(9U);
        (void)tk_dly_tsk(1000U);
    }
}

static void wait_released(uint8_t pin)
{
    while (button_pressed(pin)) {
        (void)tk_dly_tsk(STAGE5_POLL_MS);
    }
    (void)tk_dly_tsk(STAGE5_DEBOUNCE_MS);
}

static bool accept_press(uint8_t pin)
{
    if (!button_pressed(pin)) {
        return false;
    }
    (void)tk_dly_tsk(STAGE5_DEBOUNCE_MS);
    if (!button_pressed(pin)) {
        return false;
    }
    beep(80U);
    return true;
}

static void indicate_case(const stage5_case_t *test_case)
{
    uint8_t index;
    uint8_t speed_count = (uint8_t)(sizeof(stage5_speeds_mm_s) /
                                    sizeof(stage5_speeds_mm_s[0]));

    if (test_case == 0) {
        return;
    }
    for (index = 0U; index < speed_count; ++index) {
        if (stage5_speeds_mm_s[index] == test_case->speed_mm_s) {
            break;
        }
    }
    (void)tk_dly_tsk(300U);
    if (test_case->reverse) {
        status_set(true);
        (void)tk_dly_tsk(600U);
        status_set(false);
        (void)tk_dly_tsk(300U);
    }
    beep_code((uint8_t)(index + 1U));
}

static uint8_t fault_code(motion_fault_t fault)
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

static void stats_update(stage5_stats_t *stats,
                         const motion_status_t *status,
                         int32_t target_mm_s)
{
    uint32_t elapsed_ms;
    int32_t left_speed;
    int32_t right_speed;

    if (status->state != MOTION_STATE_RUNNING) {
        return;
    }
    if (stats->started_us == 0U) {
        stats->started_us = status->state_timestamp_us;
    }
    elapsed_ms = (uint32_t)((status->timestamp_us - stats->started_us) /
                            1000ULL);
    left_speed = magnitude_i32(status->left_speed_mm_s);
    right_speed = magnitude_i32(status->right_speed_mm_s);
    if (left_speed > stats->maximum_left_mm_s) {
        stats->maximum_left_mm_s = left_speed;
    }
    if (right_speed > stats->maximum_right_mm_s) {
        stats->maximum_right_mm_s = right_speed;
    }
    if (stats->rise_time_ms == 0U &&
        left_speed >= ((target_mm_s * 9) / 10) &&
        right_speed >= ((target_mm_s * 9) / 10)) {
        stats->rise_time_ms = elapsed_ms;
    }
    if (elapsed_ms >= STAGE5_STEADY_START_MS) {
        stats->steady_left_sum += left_speed;
        stats->steady_right_sum += right_speed;
        stats->steady_left_pwm_sum += status->left_pwm_permille;
        stats->steady_right_pwm_sum += status->right_pwm_permille;
        ++stats->steady_samples;
    }
}

static void print_result(const stage5_case_t *test_case,
                         const motion_status_t *status,
                         const stage5_stats_t *stats)
{
    int32_t mean_left = 0;
    int32_t mean_right = 0;
    uint32_t mean_left_pwm = 0U;
    uint32_t mean_right_pwm = 0U;

    if (stats->steady_samples != 0U) {
        mean_left = (int32_t)(stats->steady_left_sum / stats->steady_samples);
        mean_right = (int32_t)(stats->steady_right_sum / stats->steady_samples);
        mean_left_pwm = (uint32_t)(stats->steady_left_pwm_sum /
                                   stats->steady_samples);
        mean_right_pwm = (uint32_t)(stats->steady_right_pwm_sum /
                                    stats->steady_samples);
    }
    tm_printf((UB *)"[motion-stage5] dir=%s target=%d distance=%d seq=%u state=%d fault=%d travel=%d\n",
              (UB *)(test_case->reverse ? "reverse" : "forward"),
              test_case->speed_mm_s, STAGE5_DISTANCE_MM,
              status->active_sequence_id, status->state, status->fault,
              status->travelled_mm);
    tm_printf((UB *)"[motion-stage5] rise_ms=%u max=%d/%d steady_mean=%d/%d steady_pwm=%u/%u samples=%u counts=%d/%d invalid=%u/%u\n",
              stats->rise_time_ms, stats->maximum_left_mm_s,
              stats->maximum_right_mm_s, mean_left, mean_right,
              mean_left_pwm, mean_right_pwm, stats->steady_samples,
              status->left_encoder_counts, status->right_encoder_counts,
              status->left_invalid_encoder_transitions,
              status->right_invalid_encoder_transitions);
}

static bool wait_before_run(void)
{
    uint32_t elapsed_ms = 0U;

    while (elapsed_ms < STAGE5_PRE_RUN_DELAY_MS) {
        if (button_pressed(STAGE5_NEXT_STOP_BUTTON_PIN)) {
            (void)motion_stop();
            wait_released(STAGE5_NEXT_STOP_BUTTON_PIN);
            return false;
        }
        led_set((elapsed_ms % 200U) < 100U);
        (void)tk_dly_tsk(STAGE5_POLL_MS);
        elapsed_ms += STAGE5_POLL_MS;
    }
    led_set(false);
    return true;
}

static uint8_t run_case(const stage5_case_t *test_case)
{
    motion_command_t command = {0};
    motion_status_t status = {0};
    stage5_stats_t stats = {0};
    motion_result_t result;
    uint32_t nominal_ms;
    uint32_t sequence;

    if (!wait_before_run()) {
        beep_code(2U);
        return 2U;
    }
    nominal_ms = ((uint32_t)STAGE5_DISTANCE_MM * 1000U) /
                 (uint32_t)test_case->speed_mm_s;
    command.type = MOTION_COMMAND_MOVE_DISTANCE;
    command.sequence_id = next_sequence++;
    command.timeout_ms = (nominal_ms * 2U) + 2000U;
    command.value.distance.distance_mm = test_case->reverse ?
                                         -STAGE5_DISTANCE_MM :
                                         STAGE5_DISTANCE_MM;
    command.value.distance.speed_mm_s = test_case->reverse ?
                                        -test_case->speed_mm_s :
                                        test_case->speed_mm_s;
    sequence = command.sequence_id;
    result = motion_submit(&command);
    if (result != MOTION_RESULT_OK) {
        beep_code(8U);
        return 8U;
    }

    for (;;) {
        if (button_pressed(STAGE5_NEXT_STOP_BUTTON_PIN)) {
            (void)motion_stop();
            wait_released(STAGE5_NEXT_STOP_BUTTON_PIN);
            beep_code(2U);
            return 2U;
        }
        if (motion_get_status(&status) == MOTION_RESULT_OK &&
            status.active_sequence_id == sequence) {
            stats_update(&stats, &status, test_case->speed_mm_s);
            if (status.state == MOTION_STATE_COMPLETE) {
                print_result(test_case, &status, &stats);
                beep_code(1U);
                return 1U;
            }
            if (status.state == MOTION_STATE_FAULT) {
                uint8_t code = fault_code(status.fault);
                print_result(test_case, &status, &stats);
                beep_code(code);
                return code;
            }
        }
        (void)tk_dly_tsk(STAGE5_POLL_MS);
    }
}

static bool idle_next_or_reset(uint8_t *case_index)
{
    uint32_t held_ms = 0U;

    if (!accept_press(STAGE5_NEXT_STOP_BUTTON_PIN)) {
        return false;
    }
    while (button_pressed(STAGE5_NEXT_STOP_BUTTON_PIN)) {
        if (held_ms < STAGE5_LONG_PRESS_MS) {
            held_ms += STAGE5_POLL_MS;
        }
        (void)tk_dly_tsk(STAGE5_POLL_MS);
    }
    (void)tk_dly_tsk(STAGE5_DEBOUNCE_MS);
    if (held_ms >= STAGE5_LONG_PRESS_MS) {
        *case_index = 0U;
    } else {
        *case_index = (uint8_t)((*case_index + 1U) % case_count());
    }
    return true;
}

EXPORT INT usermain(void)
{
    stage5_case_t test_case;
    motion_status_t status;
    uint8_t case_index = 0U;

    if (gpio_set_pin(STAGE5_STATUS_LED_PIN, GPIO_MODE_OUT) < E_OK ||
        !button_init(STAGE5_RUN_BUTTON_PIN) ||
        !button_init(STAGE5_NEXT_STOP_BUTTON_PIN)) {
        fatal_beep();
    }
    buzzer_ready = pwm_set_pin(STAGE5_BUZZER_PIN) >= E_OK &&
                   pwm_set_wrap(STAGE5_BUZZER_PIN,
                                STAGE5_BUZZER_TOP) >= E_OK &&
                   pwm_set_cc(STAGE5_BUZZER_PIN,
                              STAGE5_BUZZER_DUTY) >= E_OK;
    if (buzzer_ready) {
        buzzer_set(false);
    }
    led_set(false);
    wait_released(STAGE5_RUN_BUTTON_PIN);
    wait_released(STAGE5_NEXT_STOP_BUTTON_PIN);
    if (motion_init() != MOTION_RESULT_OK ||
        motion_start_task() != MOTION_RESULT_OK ||
        !case_get(case_index, &test_case)) {
        fatal_beep();
    }
    indicate_case(&test_case);

    for (;;) {
        if (motion_get_status(&status) == MOTION_RESULT_OK &&
            status.state == MOTION_STATE_FAULT) {
            if (accept_press(STAGE5_NEXT_STOP_BUTTON_PIN)) {
                wait_released(STAGE5_NEXT_STOP_BUTTON_PIN);
                (void)motion_stop();
                beep_code(2U);
            }
        } else if (idle_next_or_reset(&case_index)) {
            if (!case_get(case_index, &test_case)) {
                fatal_beep();
            }
            indicate_case(&test_case);
        } else if (accept_press(STAGE5_RUN_BUTTON_PIN)) {
            wait_released(STAGE5_RUN_BUTTON_PIN);
            (void)run_case(&test_case);
        }
        (void)tk_dly_tsk(STAGE5_POLL_MS);
    }
    return 0;
}
