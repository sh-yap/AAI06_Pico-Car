#include <sys/machine.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>

#include "motion_clock.h"

/* RP2040 WATCHDOG_TICK: base 0x40058000 + offset 0x2c. This is read-only
 * diagnostic code; it does not alter the platform clock or start motors. */
#define MOTION_CLOCK_TEST_WATCHDOG_TICK 0x4005802CU
#define MOTION_CLOCK_TEST_SAMPLE_MS 1000U

EXPORT INT usermain(void)
{
    SYSTIM os_before;
    SYSTIM os_after;
    uint64_t raw_before;
    uint64_t raw_after;
    uint32_t tick;
    uint32_t sequence = 0U;

    (void)tk_dly_tsk(500U);
    tm_printf((UB *)"[clock-test] motor-free: compare raw timer with one RTOS second\n");
    for (;;) {
        raw_before = motion_clock_us();
        (void)tk_get_otm(&os_before);
        tick = in_w(MOTION_CLOCK_TEST_WATCHDOG_TICK);
        (void)tk_dly_tsk(MOTION_CLOCK_TEST_SAMPLE_MS);
        raw_after = motion_clock_us();
        (void)tk_get_otm(&os_after);
        ++sequence;
        tm_printf((UB *)"[clock-test] seq=%u raw_us=%u os_ms=%u raw_now_us=%u os_now_ms=%u tick_cycles=%u running=%u enabled=%u pause=%u ref_div=%u\n",
                  sequence, (uint32_t)(raw_after - raw_before),
                  (uint32_t)(os_after.lo - os_before.lo),
                  (uint32_t)raw_after, (uint32_t)os_after.lo,
                  tick & 0x1FFU, (tick >> 10U) & 1U,
                  (tick >> 9U) & 1U, in_w(TIMER_PAUSE) & 1U,
                  in_w(CLK_REF + CLK_x_DIV));
    }
    return 0;
}
