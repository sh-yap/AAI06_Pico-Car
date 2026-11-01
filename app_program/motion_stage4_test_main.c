#include <sys/machine.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>
#include <tm/tmonitor.h>

#include "motion_clock.h"
#include "motion_config.h"
#include "motion_encoder.h"
#include "motion_motor.h"
#include "motion_stage4_profile.h"

#define STAGE4_RUN_BUTTON_PIN 20U
#define STAGE4_NEXT_STOP_BUTTON_PIN 21U
#define STAGE4_STATUS_LED_PIN BOARD_LED_PIN
#define STAGE4_STATUS_BUZZER_PIN 22U
#define STAGE4_BUZZER_TOP 62499U
#define STAGE4_BUZZER_DUTY 31250U
#define STAGE4_POLL_MS 5U
#define STAGE4_DEBOUNCE_MS 30U
#define STAGE4_LONG_PRESS_MS 1000U
#define STAGE4_PRE_RUN_DELAY_MS 1000U

static bool button_pressed(uint8_t pin)
{
    /* Robo-Pico user buttons connect the GPIO to ground when pressed. */
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
    (void)gpio_set_val(STAGE4_STATUS_LED_PIN, enabled ? 1U : 0U);
}

static bool buzzer_ready;

static void buzzer_set(bool enabled)
{
    if (buzzer_ready) {
        (void)pwm_set_enabled(STAGE4_STATUS_BUZZER_PIN,
                              enabled ? TRUE : FALSE);
    }
}

static void status_set(bool enabled)
{
    led_set(enabled);
    buzzer_set(enabled);
}

static void button_acknowledge(void)
{
    status_set(true);
    (void)tk_dly_tsk(80U);
    status_set(false);
}

static void blink(uint8_t count, uint32_t on_ms, uint32_t off_ms)
{
    uint8_t index;

    for (index = 0U; index < count; ++index) {
        status_set(true);
        (void)tk_dly_tsk(on_ms);
        status_set(false);
        (void)tk_dly_tsk(off_ms);
    }
}

static void fatal_blink(void)
{
    (void)motion_motor_brake_all();
    for (;;) {
        blink(9U, 100U, 100U);
        (void)tk_dly_tsk(1000U);
    }
}

static void wait_until_released(uint8_t pin)
{
    while (button_pressed(pin)) {
        (void)tk_dly_tsk(STAGE4_POLL_MS);
    }
    (void)tk_dly_tsk(STAGE4_DEBOUNCE_MS);
}

static bool accept_press(uint8_t pin)
{
    if (!button_pressed(pin)) {
        return false;
    }
    (void)tk_dly_tsk(STAGE4_DEBOUNCE_MS);
    if (!button_pressed(pin)) {
        return false;
    }
    button_acknowledge();
    return true;
}

static void indicate_case(const motion_stage4_case_t *test_case)
{
    uint8_t step;

    if (test_case == 0) {
        return;
    }
    step = (uint8_t)(((test_case->duty_permille -
                       MOTION_STAGE4_FIRST_PWM_PERMILLE) /
                      MOTION_STAGE4_PWM_STEP_PERMILLE) + 1U);
    (void)tk_dly_tsk(300U);
    if (test_case->direction == MOTION_STAGE4_REVERSE) {
        status_set(true);
        (void)tk_dly_tsk(600U);
        status_set(false);
        (void)tk_dly_tsk(300U);
    }
    blink(step, 100U, 150U);
}

static void indicate_result(uint32_t flags)
{
    (void)tk_dly_tsk(300U);
    blink(motion_stage4_result_code(flags), 150U, 150U);
}

static bool wait_before_run(void)
{
    uint32_t elapsed_ms = 0U;

    while (elapsed_ms < STAGE4_PRE_RUN_DELAY_MS) {
        if (button_pressed(STAGE4_NEXT_STOP_BUTTON_PIN)) {
            if (motion_motor_brake_all() != MOTION_RESULT_OK) {
                fatal_blink();
            }
            wait_until_released(STAGE4_NEXT_STOP_BUTTON_PIN);
            return false;
        }
        led_set((elapsed_ms % 200U) < 100U);
        (void)tk_dly_tsk(STAGE4_POLL_MS);
        elapsed_ms += STAGE4_POLL_MS;
    }
    led_set(false);
    return true;
}

static uint32_t run_case(const motion_stage4_case_t *test_case)
{
    motion_encoder_snapshot_t snapshot;
    motion_motor_mode_t mode;
    motion_result_t left_result;
    motion_result_t right_result;
    uint64_t start_us;
    uint64_t stop_us;
    uint64_t deadline_us;
    uint32_t elapsed_us;
    uint32_t result_flags;
    bool aborted = false;

    if (motion_motor_brake_all() != MOTION_RESULT_OK) {
        fatal_blink();
    }
    if (!wait_before_run()) {
        return MOTION_STAGE4_RESULT_ABORTED;
    }
    if (motion_encoder_reset() != MOTION_RESULT_OK) {
        fatal_blink();
    }
    mode = test_case->direction == MOTION_STAGE4_FORWARD ?
        MOTION_MOTOR_FORWARD : MOTION_MOTOR_REVERSE;
    left_result = motion_motor_apply(0U, mode, test_case->duty_permille);
    right_result = motion_motor_apply(1U, mode, test_case->duty_permille);
    if (left_result != MOTION_RESULT_OK || right_result != MOTION_RESULT_OK) {
        fatal_blink();
    }
    start_us = motion_clock_us();
    deadline_us = start_us + ((uint64_t)test_case->duration_ms * 1000ULL);
    led_set(true);
    while (motion_clock_us() < deadline_us) {
        if (button_pressed(STAGE4_NEXT_STOP_BUTTON_PIN)) {
            aborted = true;
            break;
        }
        (void)tk_dly_tsk(STAGE4_POLL_MS);
    }
    stop_us = motion_clock_us();
    if (motion_motor_brake_all() != MOTION_RESULT_OK) {
        fatal_blink();
    }
    led_set(false);
    motion_encoder_get_snapshot(&snapshot);
    if (aborted) {
        wait_until_released(STAGE4_NEXT_STOP_BUTTON_PIN);
    }
    result_flags = motion_stage4_assess(test_case, snapshot.left_counts,
                                        snapshot.right_counts,
                                        snapshot.invalid_transitions, aborted);
    elapsed_us = (uint32_t)(stop_us - start_us);
    /* Printing happens only after both motors are braked. GP20 remains the
     * motion trigger; this line is optional evidence when a console is
     * attached and is not required for powerbank-only operation. */
    tm_printf((UB *)"[motion-stage4] dir=%s pwm=%u elapsed_us=%u counts=%d/%d invalid=%u/%u flags=%u code=%u\n",
              (UB *)(test_case->direction == MOTION_STAGE4_FORWARD ?
                     "forward" : "reverse"),
              test_case->duty_permille, elapsed_us,
              snapshot.left_counts, snapshot.right_counts,
              snapshot.left_invalid_transitions,
              snapshot.right_invalid_transitions,
              result_flags, motion_stage4_result_code(result_flags));
    return result_flags;
}

static bool idle_next_or_reset(uint8_t *case_index)
{
    uint32_t held_ms = 0U;

    if (!accept_press(STAGE4_NEXT_STOP_BUTTON_PIN)) {
        return false;
    }
    while (button_pressed(STAGE4_NEXT_STOP_BUTTON_PIN)) {
        if (held_ms < STAGE4_LONG_PRESS_MS) {
            held_ms += STAGE4_POLL_MS;
        }
        (void)tk_dly_tsk(STAGE4_POLL_MS);
    }
    (void)tk_dly_tsk(STAGE4_DEBOUNCE_MS);
    if (held_ms >= STAGE4_LONG_PRESS_MS) {
        *case_index = 0U;
    } else {
        *case_index = (uint8_t)((*case_index + 1U) %
                               motion_stage4_case_count());
    }
    return true;
}

EXPORT INT usermain(void)
{
    motion_config_t config;
    motion_stage4_case_t test_case;
    uint8_t case_index = 0U;
    uint32_t result_flags;

    motion_config_default(&config);
    if (gpio_set_pin(STAGE4_STATUS_LED_PIN, GPIO_MODE_OUT) < E_OK) {
        return 0;
    }
    buzzer_ready = pwm_set_pin(STAGE4_STATUS_BUZZER_PIN) >= E_OK &&
                   pwm_set_wrap(STAGE4_STATUS_BUZZER_PIN,
                                STAGE4_BUZZER_TOP) >= E_OK &&
                   pwm_set_cc(STAGE4_STATUS_BUZZER_PIN,
                              STAGE4_BUZZER_DUTY) >= E_OK;
    if (buzzer_ready) {
        buzzer_set(false);
    }
    led_set(false);
    if (!button_init(STAGE4_RUN_BUTTON_PIN) ||
        !button_init(STAGE4_NEXT_STOP_BUTTON_PIN) ||
        motion_motor_init(&config) != MOTION_RESULT_OK ||
        motion_encoder_init(&config) != MOTION_RESULT_OK) {
        fatal_blink();
    }
    if (motion_motor_brake_all() != MOTION_RESULT_OK) {
        fatal_blink();
    }
    wait_until_released(STAGE4_RUN_BUTTON_PIN);
    wait_until_released(STAGE4_NEXT_STOP_BUTTON_PIN);
    if (!motion_stage4_case_get(case_index, &test_case)) {
        fatal_blink();
    }
    indicate_case(&test_case);

    for (;;) {
        if (idle_next_or_reset(&case_index)) {
            if (!motion_stage4_case_get(case_index, &test_case)) {
                fatal_blink();
            }
            indicate_case(&test_case);
        } else if (accept_press(STAGE4_RUN_BUTTON_PIN)) {
            wait_until_released(STAGE4_RUN_BUTTON_PIN);
            result_flags = run_case(&test_case);
            indicate_result(result_flags);
        }
        (void)tk_dly_tsk(STAGE4_POLL_MS);
    }
    return 0;
}
