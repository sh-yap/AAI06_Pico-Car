#include "motion_motor.h"

#include <sys/machine.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>

#define MOTION_PWM_TOP 12499U
#define MOTION_PWM_COUNTS 12500U
#define MOTION_PWM_DIVIDER_REGISTER (1U << 4)

static motion_config_t motor_config;
static motion_motor_mode_t motor_mode[2] = { MOTION_MOTOR_BRAKE, MOTION_MOTOR_BRAKE };
static uint16_t motor_duty[2] = { 0U, 0U };
static bool motor_ready;

static uint8_t motor_pin(uint8_t motor, uint8_t phase)
{
    if (motor == 0U) return phase == 0U ? motor_config.left_motor_a : motor_config.left_motor_b;
    return phase == 0U ? motor_config.right_motor_a : motor_config.right_motor_b;
}

#ifdef BOARD_ROBO_PICO_ROBOT
static ER set_phase(uint8_t pin, uint16_t duty)
{
    uint32_t channel_base;
    ER er = pwm_set_pin(pin);
    if (er < E_OK) return er;
    channel_base = PWM_BASE + ((pin >> 1U) & 7U) * 0x14U;
    out_w(channel_base + PWM_CHx_DIV, MOTION_PWM_DIVIDER_REGISTER);
    er = pwm_set_wrap(pin, MOTION_PWM_TOP);
    if (er < E_OK) return er;
    return pwm_set_cc(pin, duty > MOTION_PWM_TOP ? MOTION_PWM_TOP : duty);
}
#endif

static motion_result_t apply_one(uint8_t motor, motion_motor_mode_t mode, uint16_t duty)
{
    uint16_t requested_duty = duty;
    uint16_t counts = (uint16_t)(((uint32_t)duty * MOTION_PWM_COUNTS) / 1000U);
    uint8_t a;
    uint8_t b;
    ER result_a;
    ER result_b;
    if (motor > 1U || !motor_ready) return MOTION_RESULT_UNINITIALISED;
    if (mode != MOTION_MOTOR_BRAKE && mode != MOTION_MOTOR_COAST &&
        mode != MOTION_MOTOR_FORWARD && mode != MOTION_MOTOR_REVERSE) {
        return MOTION_RESULT_INVALID;
    }
    /* Direction changes are intentionally two control periods: brake first. */
    if (motor_mode[motor] != MOTION_MOTOR_BRAKE &&
        mode != MOTION_MOTOR_BRAKE && mode != MOTION_MOTOR_COAST &&
        motor_mode[motor] != mode) {
        mode = MOTION_MOTOR_BRAKE;
        counts = 0U;
    }
    a = motor_pin(motor, 0U);
    b = motor_pin(motor, 1U);
    switch (mode) {
    case MOTION_MOTOR_FORWARD:
        result_b = pwm_set_cc(b, 0U);
        result_a = pwm_set_cc(a, counts);
        break;
    case MOTION_MOTOR_REVERSE:
        result_a = pwm_set_cc(a, 0U);
        result_b = pwm_set_cc(b, counts);
        break;
    case MOTION_MOTOR_COAST:
        result_a = pwm_set_cc(a, MOTION_PWM_TOP);
        result_b = pwm_set_cc(b, MOTION_PWM_TOP);
        counts = 1000U;
        break;
    case MOTION_MOTOR_BRAKE:
    default:
        result_a = pwm_set_cc(a, 0U);
        result_b = pwm_set_cc(b, 0U);
        counts = 0U;
        break;
    }
    if (result_a < E_OK || result_b < E_OK) {
        /* Make a best-effort safe write to both phases before reporting the
         * hardware failure. Do not publish the requested mode as applied. */
        (void)pwm_set_cc(a, 0U);
        (void)pwm_set_cc(b, 0U);
        motor_mode[motor] = MOTION_MOTOR_BRAKE;
        motor_duty[motor] = 0U;
        return MOTION_RESULT_HARDWARE;
    }
    motor_mode[motor] = mode;
    motor_duty[motor] = mode == MOTION_MOTOR_COAST ? 1000U :
        (mode == MOTION_MOTOR_BRAKE ? 0U : requested_duty);
    return MOTION_RESULT_OK;
}

motion_result_t motion_motor_init(const motion_config_t *config)
{
    if (config == 0 || !motion_config_validate(config)) return MOTION_RESULT_HARDWARE;
#ifndef BOARD_ROBO_PICO_ROBOT
    (void)config;
    motor_ready = false;
    return MOTION_RESULT_HARDWARE;
#else
    uint8_t motor;
    motor_config = *config;
    motor_ready = false;
    motor_mode[0] = MOTION_MOTOR_BRAKE;
    motor_mode[1] = MOTION_MOTOR_BRAKE;
    motor_duty[0] = 0U;
    motor_duty[1] = 0U;
    for (motor = 0U; motor < 2U; ++motor) {
        if (set_phase(motor_pin(motor, 0U), 0U) < E_OK ||
            set_phase(motor_pin(motor, 1U), 0U) < E_OK ||
            pwm_set_enabled(motor_pin(motor, 0U), TRUE) < E_OK ||
            pwm_set_enabled(motor_pin(motor, 1U), TRUE) < E_OK) {
            return MOTION_RESULT_HARDWARE;
        }
    }
    motor_ready = true;
    return motion_motor_brake_all();
#endif
}

motion_result_t motion_motor_apply(uint8_t motor_index, motion_motor_mode_t mode,
                                   uint16_t duty_permille)
{
    if (motor_ready && duty_permille > motor_config.max_pwm_permille) {
        duty_permille = motor_config.max_pwm_permille;
    }
    if (motor_ready && mode != MOTION_MOTOR_BRAKE && mode != MOTION_MOTOR_COAST &&
        duty_permille != 0U && duty_permille < motor_config.deadband_permille) {
        duty_permille = motor_config.deadband_permille;
    }
    if (duty_permille > 1000U) duty_permille = 1000U;
    return apply_one(motor_index, mode, duty_permille);
}

motion_result_t motion_motor_brake_all(void)
{
    motion_result_t left = apply_one(0U, MOTION_MOTOR_BRAKE, 0U);
    motion_result_t right = apply_one(1U, MOTION_MOTOR_BRAKE, 0U);
    return left != MOTION_RESULT_OK ? left : right;
}

uint16_t motion_motor_last_duty(uint8_t motor_index)
{
    return motor_index < 2U ? motor_duty[motor_index] : 0U;
}
