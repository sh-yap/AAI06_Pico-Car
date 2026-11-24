#include <stdbool.h>
#include <stdint.h>

#include <sys/machine.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>

#include "motion_task.h"

#define STAGE6_RUN_BUTTON_PIN 20U
#define STAGE6_NEXT_STOP_BUTTON_PIN 21U
#define STAGE6_STATUS_LED_PIN BOARD_LED_PIN
#define STAGE6_BUZZER_PIN 22U
#define STAGE6_BUZZER_TOP 62499U
#define STAGE6_BUZZER_DUTY 31250U
#define STAGE6_POLL_MS 20U
#define STAGE6_DEBOUNCE_MS 30U
#define STAGE6_LONG_PRESS_MS 1000U
#define STAGE6_PRE_RUN_DELAY_MS 1000U
#define STAGE6_SPEED_MM_S 50

typedef struct {
    int32_t distance_mm;
    bool reverse;
} stage6_case_t;

typedef struct {
    uint64_t started_us;
    int64_t left_speed_sum;
    int64_t right_speed_sum;
    uint32_t samples;
    int32_t peak_left_mm_s;
    int32_t peak_right_mm_s;
} stage6_stats_t;

/* Minimal required Stage 6 scope: normal straight travel plus the Vehicle
   Controller's reverse-rescan distance. */
/* Keep the completed 500 mm reference and add one longer 1.5 m straight run
   to expose cumulative drift without introducing a repeat matrix. */
static const int32_t stage6_distances_mm[] = { 500, 1500 };
static bool buzzer_ready;
static uint32_t next_sequence = 1U;

static uint8_t case_count(void)
{
    return (uint8_t)(sizeof(stage6_distances_mm) /
                     sizeof(stage6_distances_mm[0]) + 1U);
}

static bool case_get(uint8_t index, stage6_case_t *test_case)
{
    uint8_t forward_count = (uint8_t)(sizeof(stage6_distances_mm) /
                                      sizeof(stage6_distances_mm[0]));

    if (test_case == NULL || index >= case_count()) {
        return false;
    }
    test_case->reverse = index >= forward_count;
    test_case->distance_mm = test_case->reverse ? 100 :
                             stage6_distances_mm[index];
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
    (void)gpio_set_val(STAGE6_STATUS_LED_PIN, enabled ? 1U : 0U);
}

static void buzzer_set(bool enabled)
{
    if (buzzer_ready) {
        (void)pwm_set_enabled(STAGE6_BUZZER_PIN, enabled ? TRUE : FALSE);
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

static int32_t magnitude_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static uint8_t speed_code(const stage6_case_t *test_case,
                          const motion_status_t *status,
                          const stage6_stats_t *stats)
{
    int32_t left;
    int32_t right;
    int32_t mean_speed;
    uint64_t elapsed_us = 0U;

    if (test_case != NULL && status != NULL && stats != NULL &&
        stats->started_us != 0U && status->timestamp_us >= stats->started_us) {
        elapsed_us = status->timestamp_us - stats->started_us;
    }
    if (elapsed_us != 0U) {
        mean_speed = (int32_t)(((int64_t)test_case->distance_mm * 1000000LL) /
                               (int64_t)elapsed_us);
    } else if (stats != NULL && stats->samples != 0U) {
        left = magnitude_i32((int32_t)(stats->left_speed_sum /
                                       (int64_t)stats->samples));
        right = magnitude_i32((int32_t)(stats->right_speed_sum /
                                        (int64_t)stats->samples));
        mean_speed = (left + right) / 2;
    } else {
        mean_speed = 0;
    }
    mean_speed = (mean_speed + 5) / 10;
    if (mean_speed < 1) {
        mean_speed = 1;
    }
    if (mean_speed > 9) {
        mean_speed = 9;
    }
    return (uint8_t)mean_speed;
}

static void indicate_speed(const stage6_case_t *test_case,
                           const motion_status_t *status,
                           const stage6_stats_t *stats)
{
    uint8_t count = speed_code(test_case, status, stats);
    uint8_t index;

    /* Keep the completion beep separate.  Speed is shown only on the LED so
       it cannot be confused with the one-beep completion result. */
    (void)tk_dly_tsk(300U);
    for (index = 0U; index < count; ++index) {
        led_set(true);
        (void)tk_dly_tsk(400U);
        led_set(false);
        (void)tk_dly_tsk(400U);
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
        (void)tk_dly_tsk(STAGE6_POLL_MS);
    }
    (void)tk_dly_tsk(STAGE6_DEBOUNCE_MS);
}

static bool accept_press(uint8_t pin)
{
    if (!button_pressed(pin)) {
        return false;
    }
    (void)tk_dly_tsk(STAGE6_DEBOUNCE_MS);
    if (!button_pressed(pin)) {
        return false;
    }
    beep(80U);
    return true;
}

static void indicate_case(const stage6_case_t *test_case)
{
    uint8_t index;

    if (test_case == NULL) {
        return;
    }
    if (test_case->reverse) {
        status_set(true);
        (void)tk_dly_tsk(600U);
        status_set(false);
        (void)tk_dly_tsk(300U);
        beep_code(1U);
        return;
    }
    for (index = 0U; index < (uint8_t)(sizeof(stage6_distances_mm) /
                                       sizeof(stage6_distances_mm[0]));
         ++index) {
        if (stage6_distances_mm[index] == test_case->distance_mm) {
            beep_code((uint8_t)(index + 1U));
            return;
        }
    }
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

static void stats_update(stage6_stats_t *stats,
                         const motion_status_t *status)
{
    int32_t left_magnitude;
    int32_t right_magnitude;

    if (stats == NULL || status == NULL ||
        status->state != MOTION_STATE_RUNNING) {
        return;
    }
    if (stats->started_us == 0U) {
        stats->started_us = status->state_timestamp_us;
    }
    stats->left_speed_sum += status->left_speed_mm_s;
    stats->right_speed_sum += status->right_speed_mm_s;
    ++stats->samples;
    left_magnitude = magnitude_i32(status->left_speed_mm_s);
    right_magnitude = magnitude_i32(status->right_speed_mm_s);
    if (left_magnitude > stats->peak_left_mm_s) {
        stats->peak_left_mm_s = left_magnitude;
    }
    if (right_magnitude > stats->peak_right_mm_s) {
        stats->peak_right_mm_s = right_magnitude;
    }
}

static void print_result(const stage6_case_t *test_case,
                         const motion_status_t *status,
                         const stage6_stats_t *stats)
{
    uint64_t elapsed_us = 0U;
    int32_t mean_left = 0;
    int32_t mean_right = 0;

    if (stats != NULL && stats->started_us != 0U &&
        status->timestamp_us >= stats->started_us) {
        elapsed_us = status->timestamp_us - stats->started_us;
    }
    if (stats != NULL && stats->samples != 0U) {
        mean_left = (int32_t)(stats->left_speed_sum /
                              (int64_t)stats->samples);
        mean_right = (int32_t)(stats->right_speed_sum /
                               (int64_t)stats->samples);
    }
    tm_printf((UB *)"[motion-stage6] dir=%s target_mm=%d speed_mm_s=%d "
              "travelled_mm=%d counts=%d/%d invalid=%u/%u seq=%u "
              "state=%d fault=%d elapsed_ms=%u mean_speed=%d/%d "
              "peak_speed=%d/%d samples=%u\n",
              (UB *)(test_case->reverse ? "reverse" : "forward"),
              test_case->distance_mm, STAGE6_SPEED_MM_S,
              status->travelled_mm, status->left_encoder_counts,
              status->right_encoder_counts,
              status->left_invalid_encoder_transitions,
              status->right_invalid_encoder_transitions,
              status->active_sequence_id, status->state, status->fault,
              (uint32_t)(elapsed_us / 1000ULL), mean_left, mean_right,
              stats == NULL ? 0 : stats->peak_left_mm_s,
              stats == NULL ? 0 : stats->peak_right_mm_s,
              stats == NULL ? 0U : stats->samples);
}

static bool wait_before_run(void)
{
    uint32_t elapsed_ms = 0U;

    while (elapsed_ms < STAGE6_PRE_RUN_DELAY_MS) {
        if (button_pressed(STAGE6_NEXT_STOP_BUTTON_PIN)) {
            (void)motion_stop();
            wait_released(STAGE6_NEXT_STOP_BUTTON_PIN);
            return false;
        }
        led_set((elapsed_ms % 200U) < 100U);
        (void)tk_dly_tsk(STAGE6_POLL_MS);
        elapsed_ms += STAGE6_POLL_MS;
    }
    led_set(false);
    return true;
}

static uint8_t run_case(const stage6_case_t *test_case)
{
    motion_command_t command = {0};
    motion_status_t status = {0};
    stage6_stats_t stats = {0};
    motion_result_t result;
    uint32_t nominal_ms;
    uint32_t sequence;

    if (!wait_before_run()) {
        beep_code(2U);
        return 2U;
    }
    nominal_ms = ((uint32_t)test_case->distance_mm * 1000U) /
                 STAGE6_SPEED_MM_S;
    command.type = MOTION_COMMAND_MOVE_DISTANCE;
    command.sequence_id = next_sequence++;
    command.timeout_ms = (nominal_ms * 2U) + 3000U;
    command.value.distance.distance_mm = test_case->reverse ?
                                         -test_case->distance_mm :
                                         test_case->distance_mm;
    command.value.distance.speed_mm_s = test_case->reverse ?
                                        -STAGE6_SPEED_MM_S :
                                        STAGE6_SPEED_MM_S;
    sequence = command.sequence_id;
    result = motion_submit(&command);
    if (result != MOTION_RESULT_OK) {
        beep_code(8U);
        return 8U;
    }

    for (;;) {
        if (button_pressed(STAGE6_NEXT_STOP_BUTTON_PIN)) {
            (void)motion_stop();
            wait_released(STAGE6_NEXT_STOP_BUTTON_PIN);
            beep_code(2U);
            return 2U;
        }
        if (motion_get_status(&status) == MOTION_RESULT_OK &&
            status.active_sequence_id == sequence) {
            stats_update(&stats, &status);
            if (status.state == MOTION_STATE_COMPLETE) {
                print_result(test_case, &status, &stats);
                beep_code(1U);
                indicate_speed(test_case, &status, &stats);
                return 1U;
            }
            if (status.state == MOTION_STATE_FAULT) {
                print_result(test_case, &status, &stats);
                beep_code(fault_code(status.fault));
                return fault_code(status.fault);
            }
        }
        (void)tk_dly_tsk(STAGE6_POLL_MS);
    }
}

static bool idle_next_or_reset(uint8_t *case_index)
{
    uint32_t held_ms = 0U;

    if (!accept_press(STAGE6_NEXT_STOP_BUTTON_PIN)) {
        return false;
    }
    while (button_pressed(STAGE6_NEXT_STOP_BUTTON_PIN)) {
        if (held_ms < STAGE6_LONG_PRESS_MS) {
            held_ms += STAGE6_POLL_MS;
        }
        (void)tk_dly_tsk(STAGE6_POLL_MS);
    }
    (void)tk_dly_tsk(STAGE6_DEBOUNCE_MS);
    if (held_ms >= STAGE6_LONG_PRESS_MS) {
        *case_index = 0U;
    } else {
        *case_index = (uint8_t)((*case_index + 1U) % case_count());
    }
    return true;
}

EXPORT INT usermain(void)
{
    stage6_case_t test_case;
    motion_status_t status;
    uint8_t case_index = 0U;

    if (gpio_set_pin(STAGE6_STATUS_LED_PIN, GPIO_MODE_OUT) < E_OK ||
        !button_init(STAGE6_RUN_BUTTON_PIN) ||
        !button_init(STAGE6_NEXT_STOP_BUTTON_PIN)) {
        fatal_beep();
    }
    buzzer_ready = pwm_set_pin(STAGE6_BUZZER_PIN) >= E_OK &&
                   pwm_set_wrap(STAGE6_BUZZER_PIN, STAGE6_BUZZER_TOP) >= E_OK &&
                   pwm_set_cc(STAGE6_BUZZER_PIN, STAGE6_BUZZER_DUTY) >= E_OK;
    buzzer_set(false);
    led_set(false);
    wait_released(STAGE6_RUN_BUTTON_PIN);
    wait_released(STAGE6_NEXT_STOP_BUTTON_PIN);
    if (motion_init() != MOTION_RESULT_OK ||
        motion_start_task() != MOTION_RESULT_OK ||
        !case_get(case_index, &test_case)) {
        fatal_beep();
    }
    indicate_case(&test_case);

    for (;;) {
        if (motion_get_status(&status) == MOTION_RESULT_OK &&
            status.state == MOTION_STATE_FAULT) {
            if (accept_press(STAGE6_NEXT_STOP_BUTTON_PIN)) {
                wait_released(STAGE6_NEXT_STOP_BUTTON_PIN);
                (void)motion_stop();
                beep_code(2U);
            }
        } else if (idle_next_or_reset(&case_index)) {
            if (!case_get(case_index, &test_case)) {
                fatal_beep();
            }
            indicate_case(&test_case);
        } else if (accept_press(STAGE6_RUN_BUTTON_PIN)) {
            wait_released(STAGE6_RUN_BUTTON_PIN);
            (void)run_case(&test_case);
        }
        (void)tk_dly_tsk(STAGE6_POLL_MS);
    }
    return 0;
}
