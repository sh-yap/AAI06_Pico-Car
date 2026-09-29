/** @file obstacle_ultrasonic.c
 *
 * @brief HC-SR04 ultrasonic ranging driver (Buddy 5).
 *
 * How the echo is timed: the Echo pin is a PWM channel-B pin, and its PWM
 * slice runs in "level" divider mode, where the counter only advances while
 * channel B is high.  With the divider set to the system clock in MHz the
 * counter ticks once per microsecond, so after the echo ends the counter
 * holds the echo width in microseconds.
 *
 * This needs no GPIO interrupt (Buddy 2's encoders own IO_IRQ_BANK0), no
 * CPU time while waiting, and is not disturbed by task pre-emption or by
 * the USB console masking interrupts.
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
#include "obstacle_ultrasonic.h"

/* RP2040 PWM slice registers (RP2040 datasheet section 4.5.3). */
#define ULTRA_PWM_SLICE_STRIDE    (0x14U)
#define ULTRA_PWM_CSR_EN          (1U << 0)
#define ULTRA_PWM_CSR_DIV_LEVEL   (1U << 4)   /* Count while B is high.   */
#define ULTRA_PWM_DIV_INT_SHIFT   (4U)        /* DIV is 8.4 fixed point.  */
#define ULTRA_PWM_TOP_MAX         (0xFFFFU)
#define ULTRA_PWM_CTR_MASK        (0xFFFFU)

/* SYSCLK is the system clock in MHz; as a divider it gives 1 us ticks. */
#if (SYSCLK > 255)
#error "SYSCLK too high for an integer 1 us PWM divider"
#endif
#define ULTRA_PWM_DIV_1US         ((uint32_t)SYSCLK << ULTRA_PWM_DIV_INT_SHIFT)

#define ULTRA_US_PER_MS           (1000U)
#define ULTRA_PING_PERIOD_US      (OBSTACLE_PING_PERIOD_MS * ULTRA_US_PER_MS)
#define ULTRA_PROBE_WINDOW_US     (50000U)  /* Longer than any echo.    */
#define ULTRA_PIN_TEST_HOLD_MS    (10)      /* TMO: Trig high phase.    */
#define ULTRA_PIN_TEST_TIMER_MS   (100)     /* TMO: timer rate check.   */

static bool     b_ready = false;
static bool     b_pinged_before = false;
static uint32_t last_ping_us = 0U;
static uint32_t slice_base = 0U;


/*!
 * @brief Read the free-running 1 MHz system timer (low 32 bits).
 *
 * TIMERAWL has no read side effects, unlike TIMELR, so it is safe to read
 * from any task on either core.  It wraps every 71 minutes; callers only
 * use unsigned differences, which stay correct across the wrap.
 */
static uint32_t
ultra_time_us(void)
{
    return (in_w(TIMER_TIMERAWL));
}

/*!
 * @brief Sleep until OBSTACLE_PING_PERIOD_MS has passed since the last ping.
 */
static void
ultra_wait_ping_period(void)
{
    uint32_t elapsed_us;
    uint32_t delay_ms;

    if (b_pinged_before)
    {
        elapsed_us = ultra_time_us() - last_ping_us;

        if (elapsed_us < ULTRA_PING_PERIOD_US)
        {
            delay_ms = ((ULTRA_PING_PERIOD_US - elapsed_us)
                        + (ULTRA_US_PER_MS - 1U)) / ULTRA_US_PER_MS;

            /* Cast to TMO: delay_ms is at most OBSTACLE_PING_PERIOD_MS. */
            (void)tk_dly_tsk((TMO)delay_ms);
        }
    }
}

/*!
 * @brief Arm the echo counter and send one trigger pulse.
 *
 * @return Timer value at the start of the trigger pulse.
 */
static uint32_t
ultra_send_ping(void)
{
    uint32_t start_us;

    /* Stop, zero, then re-arm the counter while echo is still low. */
    out_w(slice_base + PWM_CHx_CSR, ULTRA_PWM_CSR_DIV_LEVEL);
    out_w(slice_base + PWM_CHx_CTR, 0U);
    out_w(slice_base + PWM_CHx_CSR,
          ULTRA_PWM_CSR_DIV_LEVEL | ULTRA_PWM_CSR_EN);

    /* Trigger pulse.  The busy-wait is bounded by the hardware timer; if
     * the task is pre-empted the pulse only gets longer, which is fine. */
    start_us = ultra_time_us();
    (void)gpio_set_val(OBSTACLE_TRIG_PIN, 1U);

    while ((ultra_time_us() - start_us) < OBSTACLE_TRIG_PULSE_US)
    {
        /* Wait for the minimum trigger width. */
    }

    (void)gpio_set_val(OBSTACLE_TRIG_PIN, 0U);

    return (start_us);
}

/*!
 * @brief Configure the Trig/Echo pins and the echo-timing PWM slice.
 *
 * @return true on success; false if the PWM block is held in reset, which
 *         means the image was not built with BOARD=robo_pico_robot.
 */
bool
obstacle_ultrasonic_init(void)
{
    b_ready = false;

    if (0U != (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        slice_base = PWM_BASE + (obstacle_calc_pwm_slice(OBSTACLE_ECHO_PIN)
                                 * ULTRA_PWM_SLICE_STRIDE);

        /* Trig: drive low before enabling the output, so no stray pulse. */
        (void)gpio_set_val(OBSTACLE_TRIG_PIN, 0U);
        (void)gpio_set_pin(OBSTACLE_TRIG_PIN, GPIO_MODE_OUT);

        /* Echo: input only, Schmitt trigger, pull-down so an unplugged
         * sensor reads as "no echo" instead of floating. */
        out_w(GPIO(OBSTACLE_ECHO_PIN),
              GPIO_IE | GPIO_OD | GPIO_PDE | GPIO_SHEMITT);
        out_w(GPIO_CTRL(OBSTACLE_ECHO_PIN), GPIO_CTRL_FUNCSEL_PWM);

        /* Slice: disabled, 1 us ticks, full 16-bit range, gated by B. */
        out_w(slice_base + PWM_CHx_CSR, 0U);
        out_w(slice_base + PWM_CHx_DIV, ULTRA_PWM_DIV_1US);
        out_w(slice_base + PWM_CHx_TOP, ULTRA_PWM_TOP_MAX);
        out_w(slice_base + PWM_CHx_CC, 0U);
        out_w(slice_base + PWM_CHx_CTR, 0U);
        out_w(slice_base + PWM_CHx_CSR, ULTRA_PWM_CSR_DIV_LEVEL);

        b_pinged_before = false;
        b_ready = true;
    }

    return (b_ready);
}

/*!
 * @brief Take one ultrasonic measurement.
 *
 * Waits out the minimum ping period, sends a trigger, sleeps while the
 * hardware times the echo, then classifies the result.  Blocks the calling
 * task for OBSTACLE_ECHO_WAIT_MS plus up to OBSTACLE_PING_PERIOD_MS.
 *
 * @param[out] p_range Filled with the echo width, distance and status.
 */
void
obstacle_ultrasonic_measure(obstacle_range_t * p_range)
{
    uint32_t echo_us = 0U;
    uint32_t ping_us = 0U;
    bool     b_echo_high = false;

    if (NULL != p_range)
    {
        if (!b_ready)
        {
            p_range->status = OBSTACLE_RANGE_NOT_READY;
        }
        else
        {
            ultra_wait_ping_period();

            if (0U != gpio_get_val(OBSTACLE_ECHO_PIN))
            {
                /* The previous echo has not ended (nothing reflected yet).
                 * Do not trigger into it; report and try next time. */
                ping_us = ultra_time_us();
                b_echo_high = true;
            }
            else
            {
                ping_us = ultra_send_ping();

                /* Cast to TMO: a small compile-time constant. */
                (void)tk_dly_tsk((TMO)OBSTACLE_ECHO_WAIT_MS);

                b_echo_high = (0U != gpio_get_val(OBSTACLE_ECHO_PIN));
                echo_us = in_w(slice_base + PWM_CHx_CTR) & ULTRA_PWM_CTR_MASK;
                out_w(slice_base + PWM_CHx_CSR, ULTRA_PWM_CSR_DIV_LEVEL);
            }

            last_ping_us = ping_us;
            b_pinged_before = true;

            p_range->status = obstacle_calc_classify_echo(echo_us,
                                                          b_echo_high);
        }

        p_range->echo_us = echo_us;
        p_range->timestamp_us = ping_us;

        if (OBSTACLE_RANGE_OK == p_range->status)
        {
            p_range->distance_mm = obstacle_calc_echo_to_mm(echo_us);
        }
        else
        {
            p_range->distance_mm = 0U;
        }
    }
}

/*!
 * @brief Bench diagnosis: trigger once and watch the Echo pin directly.
 *
 * Arms the hardware counter exactly as a normal measurement does, then
 * busy-polls the Echo pin level for ULTRA_PROBE_WINDOW_US.  Comparing the
 * two separates wiring/power faults (the pin never rises) from echo-timing
 * faults (the pin rises but the counter stays at zero).
 *
 * Blocks the CPU for 50 ms: test programs only.
 *
 * @param[out] p_probe What the pin did, and what the counter measured.
 */
void
obstacle_ultrasonic_probe(obstacle_probe_t * p_probe)
{
    uint32_t start_us;
    uint32_t elapsed_us = 0U;
    bool     b_level;

    if (NULL != p_probe)
    {
        p_probe->b_high_before = false;
        p_probe->b_rose = false;
        p_probe->b_fell = false;
        p_probe->rise_us = 0U;
        p_probe->fall_us = 0U;
        p_probe->counter_us = 0U;

        if (b_ready)
        {
            ultra_wait_ping_period();
            p_probe->b_high_before = (0U != gpio_get_val(OBSTACLE_ECHO_PIN));

            start_us = ultra_send_ping();

            while (elapsed_us < ULTRA_PROBE_WINDOW_US)
            {
                elapsed_us = ultra_time_us() - start_us;
                b_level = (0U != gpio_get_val(OBSTACLE_ECHO_PIN));

                if (b_level && (!p_probe->b_rose))
                {
                    p_probe->b_rose = true;
                    p_probe->rise_us = elapsed_us;
                }
                else if ((!b_level) && p_probe->b_rose && (!p_probe->b_fell))
                {
                    p_probe->b_fell = true;
                    p_probe->fall_us = elapsed_us;
                }
                else
                {
                    /* No new edge. */
                }
            }

            p_probe->counter_us = in_w(slice_base + PWM_CHx_CTR)
                                  & ULTRA_PWM_CTR_MASK;
            out_w(slice_base + PWM_CHx_CSR, ULTRA_PWM_CSR_DIV_LEVEL);

            last_ping_us = start_us;
            b_pinged_before = true;
        }
    }
}

/*!
 * @brief Bench diagnosis: hold Trig high and read both pins back.
 *
 * Temporarily enables the Trig pad's input buffer so the real pin level
 * can be read, holds Trig high for about ULTRA_PIN_TEST_HOLD_MS with the
 * echo counter armed, then drives it low.  Also measures how many timer
 * microseconds pass during a 100 ms kernel sleep (should be ~100000).
 *
 * With a jumper from Trig to Echo, every flag should be true and the
 * counter about 10000.  With the sensor attached it merely causes one
 * ping.  Test programs only.
 *
 * @param[out] p_test Pin levels, counter value and timer rate.
 */
void
obstacle_ultrasonic_pin_test(obstacle_pin_test_t * p_test)
{
    uint32_t start_us;

    if (NULL != p_test)
    {
        p_test->b_trig_high_ok = false;
        p_test->b_trig_low_ok = false;
        p_test->b_echo_high = false;
        p_test->b_echo_low = false;
        p_test->counter_us = 0U;
        p_test->timer_us_per_100ms = 0U;

        start_us = ultra_time_us();
        (void)tk_dly_tsk(ULTRA_PIN_TEST_TIMER_MS);
        p_test->timer_us_per_100ms = ultra_time_us() - start_us;

        if (b_ready)
        {
            /* Let the Trig pad's input buffer see the driven level. */
            set_w(GPIO(OBSTACLE_TRIG_PIN), GPIO_IE);

            out_w(slice_base + PWM_CHx_CSR, ULTRA_PWM_CSR_DIV_LEVEL);
            out_w(slice_base + PWM_CHx_CTR, 0U);
            out_w(slice_base + PWM_CHx_CSR,
                  ULTRA_PWM_CSR_DIV_LEVEL | ULTRA_PWM_CSR_EN);

            (void)gpio_set_val(OBSTACLE_TRIG_PIN, 1U);
            (void)tk_dly_tsk(ULTRA_PIN_TEST_HOLD_MS);
            p_test->b_trig_high_ok = (0U != gpio_get_val(OBSTACLE_TRIG_PIN));
            p_test->b_echo_high = (0U != gpio_get_val(OBSTACLE_ECHO_PIN));

            (void)gpio_set_val(OBSTACLE_TRIG_PIN, 0U);
            (void)tk_dly_tsk(ULTRA_PIN_TEST_HOLD_MS);
            p_test->b_trig_low_ok = (0U == gpio_get_val(OBSTACLE_TRIG_PIN));
            p_test->b_echo_low = (0U == gpio_get_val(OBSTACLE_ECHO_PIN));

            p_test->counter_us = in_w(slice_base + PWM_CHx_CTR)
                                 & ULTRA_PWM_CTR_MASK;
            out_w(slice_base + PWM_CHx_CSR, ULTRA_PWM_CSR_DIV_LEVEL);

            clr_w(GPIO(OBSTACLE_TRIG_PIN), GPIO_IE);

            last_ping_us = ultra_time_us();
            b_pinged_before = true;
        }
    }
}

/*** end of file ***/
