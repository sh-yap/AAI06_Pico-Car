#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define BOARD_ROBO_PICO_ROBOT 1
#define PWM_BASE 0x40050000U
#define PWM_CHx_DIV 0x04U

#include "../../app_program/motion_motor.c"

static bool fail_pin;
static bool fail_cc;
static bool fail_enable;
static uint16_t compare_value[30];

void out_w(uint32_t address, uint32_t value)
{
    (void)address;
    (void)value;
}

ER pwm_set_pin(UINT pin)
{
    (void)pin;
    return fail_pin ? -1 : E_OK;
}

ER pwm_set_wrap(UINT pin, UW value)
{
    (void)pin;
    (void)value;
    return E_OK;
}

ER pwm_set_cc(UINT pin, UW value)
{
    if (pin < 30U) {
        compare_value[pin] = (uint16_t)value;
    }
    return fail_cc ? -1 : E_OK;
}

ER pwm_set_enabled(UINT pin, BOOL enabled)
{
    (void)pin;
    (void)enabled;
    return fail_enable ? -1 : E_OK;
}

static void check(bool ok, const char *name)
{
    if (!ok) {
        fprintf(stderr, "FAIL: %s\n", name);
        exit(1);
    }
    printf("PASS: %s\n", name);
}

int main(void)
{
    motion_config_t config;

    motion_config_default(&config);
    check(motion_motor_init(&config) == MOTION_RESULT_OK,
          "motor driver initialises all four PWM phases");
    check(motion_motor_apply(0U, MOTION_MOTOR_FORWARD, 200U) ==
              MOTION_RESULT_OK &&
          motion_motor_last_duty(0U) == 200U &&
          compare_value[config.left_motor_a] == 2500U &&
          compare_value[config.left_motor_b] == 0U,
          "forward command applies the requested duty");

    fail_cc = true;
    check(motion_motor_apply(0U, MOTION_MOTOR_REVERSE, 200U) ==
              MOTION_RESULT_HARDWARE &&
          motion_motor_last_duty(0U) == 0U,
          "compare-register failure is reported with safe driver state");
    fail_cc = false;

    fail_enable = true;
    check(motion_motor_init(&config) == MOTION_RESULT_HARDWARE &&
          motion_motor_apply(0U, MOTION_MOTOR_FORWARD, 200U) ==
              MOTION_RESULT_UNINITIALISED,
          "initialisation failure cannot leave motor output armed");
    return 0;
}
