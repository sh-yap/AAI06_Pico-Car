#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../app_program/motion_control.h"

static void expect(int condition, const char *name)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", name);
        exit(EXIT_FAILURE);
    }
    printf("PASS: %s\n", name);
}

int main(void)
{
    int32_t speed;
    motion_pid_t pid = { 1.0f, 0.5f, 0.0f, 0.0f, 0.0f, 20.0f, -100.0f, 100.0f };
    motion_pid_result_t pid_result;
    expect(motion_quadrature_delta(0U, 1U) == -1, "forward quadrature");
    expect(motion_quadrature_delta(1U, 0U) == 1, "reverse quadrature");
    expect(motion_quadrature_delta(0U, 3U) == 0, "invalid quadrature");
    expect(motion_counts_delta(UINT32_MAX - 2U, 1U) == 4, "counter wrap");
    expect(motion_speed_from_counts(100, 100, 65536, &speed) && speed == 1000,
           "count-to-speed");
    expect(!motion_speed_from_counts(1, 0, 65536, &speed), "reject zero elapsed time");
    expect(motion_counts_to_mm(100, 65536) == 100, "count-to-mm");
    expect(motion_average(100, 200) == 150, "wheel average");
    expect(motion_turn_wheel_travel_mm(145, 90000) == 113, "turn geometry");
    expect(motion_feedforward_permille(100, 57U, 318U, 225U, 100U, 6U, 80U, 900U) == 88,
           "left feedforward");
    expect(motion_feedforward_permille(-100, 55U, 301U, 246U, 100U, 6U, 80U, 900U) == -90,
           "reverse feedforward");
    expect(motion_feedforward_permille(200, 59U, 318U, 400U, 100U, 6U, 80U, 900U) == 160,
           "high-speed feedforward");
    expect(motion_feedforward_permille(1, 57U, 318U, 225U, 100U, 6U, 80U, 900U) == 80,
           "feedforward deadband");
    expect(motion_feedforward_permille(10000, 57U, 318U, 225U, 100U, 6U, 80U, 900U) == 900,
           "feedforward saturation");
    expect(motion_feedforward_permille(-10000, 57U, 318U, 225U, 100U, 50U, 80U, 900U) == -900,
           "reverse feedforward saturation");
    expect(motion_slew_target(0.0f, 100.0f, 30.0f) == 30.0f, "slew limit");
    pid_result = motion_pid_step(&pid, 100.0f, 0.0f, 0.02f);
    expect(pid_result.output <= 100.0f && pid_result.output > 0.0f, "PID saturation");
    pid_result = motion_pid_step(&pid, 0.0f, 100.0f, 0.02f);
    expect(pid_result.output < 0.0f, "PID reverse output");
    puts("All Buddy 2 host tests passed.");
    return EXIT_SUCCESS;
}
