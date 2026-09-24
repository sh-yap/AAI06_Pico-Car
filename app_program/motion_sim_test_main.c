#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "motion_control.h"

static int check(bool condition, const char *name)
{
    tm_printf((UB *)"[motion-sim] %s: %s\n", (UB *)name, (UB *)(condition ? "PASS" : "FAIL"));
    return condition ? 0 : 1;
}

EXPORT INT usermain(void)
{
    motion_pid_t pid = { 2.0f, 0.5f, 0.0f, 0.0f, 0.0f, 100.0f, -100.0f, 100.0f };
    int failures = 0;
    failures += check(motion_quadrature_delta(0U, 1U) == -1, "forward quadrature transition");
    failures += check(motion_quadrature_delta(0U, 2U) == 1, "reverse quadrature transition");
    failures += check(motion_quadrature_delta(0U, 3U) == 0, "invalid transition");
    failures += check(motion_turn_wheel_travel_mm(145, 90000) == 113, "90 degree turn geometry");
    failures += check(motion_slew_target(0.0f, 100.0f, 30.0f) == 30.0f, "target slew limit");
    failures += check(motion_pid_step(&pid, 100.0f, 0.0f, 0.02f).output <= 100.0f, "PID saturation");
    tm_printf((UB *)"[motion-sim] result=%s\n", (UB *)(failures == 0 ? "PASS" : "FAIL"));
    for (;;) tk_dly_tsk(1000);
    return failures;
}
