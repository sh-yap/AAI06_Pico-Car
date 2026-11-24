/** @file obstacle_servo.c
 *
 * @brief SG90 scan-servo driver (Buddy 5).
 *
 * The servo's PWM slice runs at 1 us per count with a 20 ms wrap, so the
 * compare value is simply the pulse width in microseconds.  The other
 * channel of the same slice (the neighbouring servo port) keeps the same
 * 50 Hz frame, which is what a second servo would need anyway.
 */

/* No <stddef.h>: micro T-Kernel's syslib.h defines its own size_t, which
 * conflicts with it.  NULL comes from the kernel headers. */
#include <stdint.h>
#include <stdbool.h>

#include <tk/tkernel.h>
#include <sys/sysdef.h>
#include <bsp/libbsp.h>

#include "obstacle_config.h"
#include "obstacle_types.h"
#include "obstacle_calc.h"
#include "obstacle_servo.h"

/* RP2040 PWM slice registers (RP2040 datasheet section 4.5.3). */
#define SERVO_PWM_SLICE_STRIDE    (0x14U)
#define SERVO_PWM_CSR_EN          (1U << 0)
#define SERVO_PWM_DIV_INT_SHIFT   (4U)

#if (SYSCLK > 255)
#error "SYSCLK too high for an integer 1 us PWM divider"
#endif
#define SERVO_PWM_DIV_1US         ((uint32_t)SYSCLK << SERVO_PWM_DIV_INT_SHIFT)
#define SERVO_PWM_TOP             (OBSTACLE_SERVO_PERIOD_US - 1U)

#define SERVO_CENTRE_DEG          (90U)

static obstacle_servo_cal_t const servo_cal =
{
    OBSTACLE_SERVO_PULSE_0_US,
    OBSTACLE_SERVO_PULSE_90_US,
    OBSTACLE_SERVO_PULSE_180_US
};

static bool     b_ready = false;
static uint32_t current_angle_deg = SERVO_CENTRE_DEG;
static uint32_t current_pulse_us = OBSTACLE_SERVO_PULSE_90_US;


/*!
 * @brief Write a pulse width (already limited) to the servo channel.
 *
 * @param[in] pulse_us Pulse width in microseconds.
 */
static void
servo_write_pulse(uint32_t pulse_us)
{
    (void)pwm_set_cc(OBSTACLE_SERVO_PIN, pulse_us);
    current_pulse_us = pulse_us;
}

/*!
 * @brief Start 50 Hz PWM on the servo pin and centre the servo.
 *
 * @return true on success; false if the PWM block is held in reset, which
 *         means the image was not built with BOARD=robo_pico_robot.
 */
bool
obstacle_servo_init(void)
{
    uint32_t slice_base;

    b_ready = false;

    if (0U != (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        slice_base = PWM_BASE + (obstacle_calc_pwm_slice(OBSTACLE_SERVO_PIN)
                                 * SERVO_PWM_SLICE_STRIDE);

        out_w(slice_base + PWM_CHx_CSR, 0U);
        out_w(slice_base + PWM_CHx_DIV, SERVO_PWM_DIV_1US);
        (void)pwm_set_wrap(OBSTACLE_SERVO_PIN, SERVO_PWM_TOP);

        current_angle_deg = SERVO_CENTRE_DEG;
        servo_write_pulse(obstacle_calc_angle_to_pulse(SERVO_CENTRE_DEG,
                                                       &servo_cal));

        /* Route the pin to PWM only once a valid pulse is loaded. */
        (void)pwm_set_pin(OBSTACLE_SERVO_PIN);
        out_w(slice_base + PWM_CHx_CSR, SERVO_PWM_CSR_EN);

        b_ready = true;
    }

    return (b_ready);
}

/*!
 * @brief Move the servo to an angle within the configured safe range.
 *
 * @param[in]  angle_deg   Target angle; clamped to the safe range.
 * @param[out] p_settle_ms Estimated settling time, or NULL if not needed.
 *
 * @return false if the driver is not initialised.
 */
bool
obstacle_servo_set_angle(uint32_t angle_deg, uint32_t * p_settle_ms)
{
    uint32_t target_deg = angle_deg;

    if (b_ready)
    {
        if (target_deg < OBSTACLE_SERVO_ANGLE_MIN_DEG)
        {
            target_deg = OBSTACLE_SERVO_ANGLE_MIN_DEG;
        }
        else if (target_deg > OBSTACLE_SERVO_ANGLE_MAX_DEG)
        {
            target_deg = OBSTACLE_SERVO_ANGLE_MAX_DEG;
        }
        else
        {
            /* Already within the safe range. */
        }

        if (NULL != p_settle_ms)
        {
            *p_settle_ms = obstacle_calc_settle_ms(current_angle_deg,
                                                   target_deg);
        }

        servo_write_pulse(obstacle_calc_angle_to_pulse(target_deg,
                                                       &servo_cal));
        current_angle_deg = target_deg;
    }

    return (b_ready);
}

/*!
 * @brief Output a raw pulse width, for calibration with servo_test.
 *
 * Bypasses the safe angle range but never the absolute pulse limits.
 *
 * @param[in] pulse_us Requested pulse width in microseconds.
 *
 * @return false if the driver is not initialised.
 */
bool
obstacle_servo_set_pulse_us(uint32_t pulse_us)
{
    if (b_ready)
    {
        servo_write_pulse(obstacle_calc_clamp_pulse(pulse_us));
    }

    return (b_ready);
}

/*!
 * @brief Last angle commanded through obstacle_servo_set_angle().
 */
uint32_t
obstacle_servo_get_angle(void)
{
    return (current_angle_deg);
}

/*!
 * @brief Pulse width currently being output, in microseconds.
 */
uint32_t
obstacle_servo_get_pulse_us(void)
{
    return (current_pulse_us);
}

/*** end of file ***/
