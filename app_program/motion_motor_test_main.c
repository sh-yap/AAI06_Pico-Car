#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "motion_config.h"
#include "motion_encoder.h"
#include "motion_motor.h"

#if TM_CONSOLE_USB_CDC
extern INT tm_usb_console_getchar(void);
#endif

static INT console_getchar(void)
{
#if TM_CONSOLE_USB_CDC
    return tm_usb_console_getchar();
#else
    return -1;
#endif
}

static void apply_drive(motion_motor_mode_t left_mode,
                        motion_motor_mode_t right_mode, uint16_t duty)
{
    (void)motion_motor_brake_all();
    tk_dly_tsk(50);
    motion_encoder_reset();
    (void)motion_motor_apply(0U, left_mode, duty);
    (void)motion_motor_apply(1U, right_mode, duty);
}

static void apply_single(uint8_t motor, motion_motor_mode_t mode, uint16_t duty)
{
    (void)motion_motor_brake_all();
    tk_dly_tsk(50);
    motion_encoder_reset();
    (void)motion_motor_apply(motor, mode, duty);
}

EXPORT INT usermain(void)
{
    motion_config_t config;
    motion_encoder_snapshot_t encoder;
    motion_result_t encoder_result;
    motion_result_t result;
    uint32_t heartbeat = 0U;
    uint16_t duty = 100U;
    uint32_t idle_ms = 0U;
    bool drive_active = false;
    /* Give USB CDC time to enumerate before the first diagnostic line. */
    tk_dly_tsk(500);
    motion_config_default(&config);
    result = motion_motor_init(&config);
    encoder_result = motion_encoder_init(&config);
    tm_printf((UB *)"[motion-motor] motor_init=%d encoder_init=%d; default state is BRAKE\n",
              result, encoder_result);
#ifdef BOARD_ROBO_PICO_ROBOT
    tm_printf((UB *)"[motion-motor] hardware test is armed only after wheels are raised; use low-duty commands from the bench procedure.\n");
#else
    tm_printf((UB *)"[motion-motor] BOARD=pico: physical motor outputs intentionally disabled.\n");
#endif
    tm_printf((UB *)"[motion-motor] keys: 1/2=left fwd/rev 3/4=right fwd/rev 5/6=left/right coast W/S=both A/D=pivot X=brake C=coast +/-=duty\n");
    for (;;) {
        INT key = console_getchar();
        if (key >= 0) {
            idle_ms = 0U;
            switch (key) {
            case '1':
                tm_printf((UB *)"[motion-motor] starting LEFT forward duty=%u; logging paused for 3 s\n", duty);
                apply_single(0U, MOTION_MOTOR_FORWARD, duty);
                drive_active = true;
                break;
            case '2':
                tm_printf((UB *)"[motion-motor] starting LEFT reverse duty=%u; logging paused for 3 s\n", duty);
                apply_single(0U, MOTION_MOTOR_REVERSE, duty);
                drive_active = true;
                break;
            case '3':
                tm_printf((UB *)"[motion-motor] starting RIGHT forward duty=%u; logging paused for 3 s\n", duty);
                apply_single(1U, MOTION_MOTOR_FORWARD, duty);
                drive_active = true;
                break;
            case '4':
                tm_printf((UB *)"[motion-motor] starting RIGHT reverse duty=%u; logging paused for 3 s\n", duty);
                apply_single(1U, MOTION_MOTOR_REVERSE, duty);
                drive_active = true;
                break;
            case '5':
                /* Coast is an explicit non-braking test mode. */
                (void)motion_motor_apply(0U, MOTION_MOTOR_COAST, 1000U);
                tm_printf((UB *)"[motion-motor] LEFT coast\n");
                break;
            case '6':
                /* Coast is an explicit non-braking test mode. */
                (void)motion_motor_apply(1U, MOTION_MOTOR_COAST, 1000U);
                tm_printf((UB *)"[motion-motor] RIGHT coast\n");
                break;
            case 'w': case 'W':
                tm_printf((UB *)"[motion-motor] starting both forward duty=%u; logging paused for 3 s\n", duty);
                apply_drive(MOTION_MOTOR_FORWARD, MOTION_MOTOR_FORWARD, duty);
                drive_active = true;
                break;
            case 's': case 'S':
                tm_printf((UB *)"[motion-motor] starting both reverse duty=%u; logging paused for 3 s\n", duty);
                apply_drive(MOTION_MOTOR_REVERSE, MOTION_MOTOR_REVERSE, duty);
                drive_active = true;
                break;
            case 'a': case 'A':
                tm_printf((UB *)"[motion-motor] starting pivot-left duty=%u; logging paused for 3 s\n", duty);
                apply_drive(MOTION_MOTOR_REVERSE, MOTION_MOTOR_FORWARD, duty);
                drive_active = true;
                break;
            case 'd': case 'D':
                tm_printf((UB *)"[motion-motor] starting pivot-right duty=%u; logging paused for 3 s\n", duty);
                apply_drive(MOTION_MOTOR_FORWARD, MOTION_MOTOR_REVERSE, duty);
                drive_active = true;
                break;
            case 'x': case 'X':
            case ' ':
                (void)motion_motor_brake_all();
                drive_active = false;
                tm_printf((UB *)"[motion-motor] BRAKE\n");
                break;
            case 'c': case 'C':
                (void)motion_motor_apply(0U, MOTION_MOTOR_COAST, 1000U);
                (void)motion_motor_apply(1U, MOTION_MOTOR_COAST, 1000U);
                tm_printf((UB *)"[motion-motor] COAST\n");
                break;
            case '+':
                if (duty < 900U) duty = (uint16_t)(duty + 25U);
                tm_printf((UB *)"[motion-motor] duty=%u\n", duty);
                break;
            case '-':
                if (duty > 25U) duty = (uint16_t)(duty - 25U);
                tm_printf((UB *)"[motion-motor] duty=%u\n", duty);
                break;
            default:
                break;
            }
        }
        tk_dly_tsk(10);
        idle_ms += 10U;
        if (drive_active && idle_ms >= 3000U) {
            (void)motion_motor_brake_all();
            idle_ms = 0U;
            drive_active = false;
            motion_encoder_get_snapshot(&encoder);
            tm_printf((UB *)"[motion-motor] final enc=%d/%d invalid=%u/%u total=%u irq=%u; safety BRAKE heartbeat=%u\n",
                      encoder.left_counts, encoder.right_counts,
                      encoder.left_invalid_transitions,
                      encoder.right_invalid_transitions,
                      encoder.invalid_transitions, encoder.irq_count,
                      heartbeat++);
        }
    }
    return 0;
}
