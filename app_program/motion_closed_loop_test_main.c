#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "motion_task.h"

#if TM_CONSOLE_USB_CDC
extern INT tm_usb_console_getchar(void);
#endif

#define MOTION_CAPTURE_SAMPLES 40U
#define MOTION_CAPTURE_PERIOD_MS 100U

static uint32_t test_sequence = 1U;
static uint32_t velocity_watchdog_ms;
static motion_status_t capture_samples[MOTION_CAPTURE_SAMPLES];
static uint32_t capture_count;
static bool capture_active;
static bool capture_seen_running;
static bool capture_force_finish;
static const char *capture_name;
static const char *capture_finish_reason;

static void print_help(void)
{
    tm_printf((UB *)"[motion-closed-loop] keys: 1=100 2=200 3=300 mm/s forward, r=100 reverse, x=stop\n");
    tm_printf((UB *)"[motion-closed-loop] m=100 mm move, t=90 deg turn, h=help; samples print only after brake\n");
    tm_printf((UB *)"[motion-closed-loop] safety: o=500 ms timeout, i=invalid command, q=zero-speed queue-full test\n");
}

static void capture_begin(const char *name)
{
    capture_count = 0U;
    capture_active = true;
    capture_seen_running = false;
    capture_force_finish = false;
    capture_name = name;
    capture_finish_reason = "complete";
}

static void capture_add(const motion_status_t *status)
{
    if (capture_count < MOTION_CAPTURE_SAMPLES) {
        capture_samples[capture_count++] = *status;
    }
}

static void print_status(uint32_t sample, const motion_status_t *status)
{
    tm_printf((UB *)"[motion-sample] n=%u seq=%u state=%d fault=%d count=%d/%d speed=%d/%d pwm=%u/%u invalid=%u/%u total=%u irq=%u\n",
              sample, status->active_sequence_id, status->state, status->fault,
              status->left_encoder_counts, status->right_encoder_counts,
              status->left_speed_mm_s, status->right_speed_mm_s,
              status->left_pwm_permille, status->right_pwm_permille,
              status->left_invalid_encoder_transitions,
              status->right_invalid_encoder_transitions,
              status->invalid_encoder_transitions,
              status->encoder_irq_count);
}

static void capture_dump(void)
{
    uint32_t i;
    tm_printf((UB *)"[motion-closed-loop] %s finished: %s; %u samples captured with motors braked\n",
              (UB *)capture_name, (UB *)capture_finish_reason, capture_count);
    for (i = 0U; i < capture_count; ++i) {
        print_status(i, &capture_samples[i]);
    }
}

static motion_result_t submit_velocity(int32_t linear_mm_s)
{
    motion_command_t command = {0};
    motion_result_t result;
    command.type = MOTION_COMMAND_SET_VELOCITY;
    command.sequence_id = test_sequence++;
    command.timeout_ms = 0U;
    command.value.velocity.linear_mm_s = linear_mm_s;
    command.value.velocity.angular_mdeg_s = 0;
    result = motion_submit(&command);
    if (result == MOTION_RESULT_OK) velocity_watchdog_ms = 3000U;
    return result;
}

static motion_result_t submit_timeout_test(void)
{
    motion_command_t command = {0};
    command.type = MOTION_COMMAND_SET_VELOCITY;
    command.sequence_id = test_sequence++;
    command.timeout_ms = 500U;
    command.value.velocity.linear_mm_s = 100;
    command.value.velocity.angular_mdeg_s = 0;
    velocity_watchdog_ms = 0U;
    return motion_submit(&command);
}

static void run_invalid_test(void)
{
    motion_command_t command = {0};
    motion_result_t result;
    command.type = MOTION_COMMAND_SET_VELOCITY;
    command.sequence_id = test_sequence++;
    command.value.velocity.linear_mm_s = 301;
    result = motion_submit(&command);
    tm_printf((UB *)"[motion-closed-loop] invalid-command result=%d expected=%d; motors remain braked\n",
              result, MOTION_RESULT_INVALID);
}

static void run_queue_full_test(void)
{
    motion_command_t command = {0};
    uint32_t accepted = 0U;
    uint32_t full = 0U;
    uint32_t other = 0U;
    uint32_t i;
    (void)motion_stop();
    command.type = MOTION_COMMAND_SET_VELOCITY;
    command.value.velocity.linear_mm_s = 0;
    for (i = 0U; i < 32U; ++i) {
        motion_result_t result;
        command.sequence_id = test_sequence++;
        result = motion_submit(&command);
        if (result == MOTION_RESULT_OK) {
            accepted++;
        } else if (result == MOTION_RESULT_QUEUE_FULL) {
            full++;
        } else {
            other++;
        }
    }
    (void)motion_stop();
    tm_printf((UB *)"[motion-closed-loop] queue-full zero-speed accepted=%u full=%u other=%u; expected full>0, motors braked\n",
              accepted, full, other);
}

static motion_result_t submit_move(void)
{
    motion_command_t command = {0};
    motion_result_t result;
    command.type = MOTION_COMMAND_MOVE_DISTANCE;
    command.sequence_id = test_sequence++;
    command.timeout_ms = 5000U;
    command.value.distance.distance_mm = 100;
    command.value.distance.speed_mm_s = 80;
    result = motion_submit(&command);
    if (result == MOTION_RESULT_OK) velocity_watchdog_ms = 0U;
    return result;
}

static motion_result_t submit_turn(void)
{
    motion_command_t command = {0};
    motion_result_t result;
    command.type = MOTION_COMMAND_TURN_ANGLE;
    command.sequence_id = test_sequence++;
    command.timeout_ms = 5000U;
    command.value.turn.angle_mdeg = 90000;
    command.value.turn.wheel_speed_mm_s = 60;
    result = motion_submit(&command);
    if (result == MOTION_RESULT_OK) velocity_watchdog_ms = 0U;
    return result;
}

static void start_test(const char *name, motion_result_t result)
{
    if (result == MOTION_RESULT_OK) {
        capture_begin(name);
    } else {
        tm_printf((UB *)"[motion-closed-loop] %s rejected result=%d\n",
                  (UB *)name, result);
    }
}

static void request_test(const char *name, int32_t velocity_mm_s)
{
    motion_result_t result;
    tm_printf((UB *)"[motion-closed-loop] starting %s; logging paused until brake\n",
              (UB *)name);
    result = submit_velocity(velocity_mm_s);
    start_test(name, result);
}

EXPORT INT usermain(void)
{
    motion_status_t status;
    motion_result_t result = motion_init();
    tm_printf((UB *)"[motion-closed-loop] init=%d\n", result);
    if (result == MOTION_RESULT_OK) {
        tm_printf((UB *)"[motion-closed-loop] task=%d\n", motion_start_task());
        print_help();
        for (;;) {
            INT key = -1;
#if TM_CONSOLE_USB_CDC
            key = tm_usb_console_getchar();
#endif
            if (key >= 0) {
                if (capture_active && key != 'x' && key != 'X' && key != ' ') {
                    /* Printing or replacing a command during capture would
                       disturb the encoder timing being measured. */
                } else {
                    switch (key) {
                    case '1':
                        request_test("100 mm/s", 100);
                        break;
                    case '2':
                        request_test("200 mm/s", 200);
                        break;
                    case '3':
                        request_test("300 mm/s", 300);
                        break;
                    case 'r': case 'R':
                        request_test("-100 mm/s", -100);
                        break;
                    case 'o': case 'O':
                        tm_printf((UB *)"[motion-closed-loop] starting 500 ms timeout; logging paused until brake\n");
                        start_test("500 ms timeout", submit_timeout_test());
                        break;
                    case 'i': case 'I':
                        run_invalid_test();
                        break;
                    case 'q': case 'Q':
                        run_queue_full_test();
                        break;
                    case 'm': case 'M':
                        tm_printf((UB *)"[motion-closed-loop] starting move 100 mm; logging paused until brake\n");
                        start_test("move 100 mm", submit_move());
                        break;
                    case 't': case 'T':
                        tm_printf((UB *)"[motion-closed-loop] starting turn +90 deg; logging paused until brake\n");
                        start_test("turn +90 deg", submit_turn());
                        break;
                    case 'x': case 'X': case ' ':
                        velocity_watchdog_ms = 0U;
                        result = motion_stop();
                        if (capture_active) {
                            capture_finish_reason = "operator stop";
                            capture_force_finish = true;
                        } else {
                            tm_printf((UB *)"[motion-closed-loop] stop result=%d\n", result);
                        }
                        break;
                    case 'h': case 'H':
                        print_help();
                        break;
                    default:
                        break;
                    }
                }
            }
            if (velocity_watchdog_ms > 0U) {
                if (velocity_watchdog_ms <= MOTION_CAPTURE_PERIOD_MS) {
                    velocity_watchdog_ms = 0U;
                    (void)motion_stop();
                    capture_finish_reason = "three-second safety stop";
                    capture_force_finish = true;
                } else {
                    velocity_watchdog_ms -= MOTION_CAPTURE_PERIOD_MS;
                }
            }
            if (motion_get_status(&status) == MOTION_RESULT_OK && capture_active) {
                if (status.state == MOTION_STATE_RUNNING) {
                    capture_seen_running = true;
                }
                if (capture_seen_running || capture_force_finish ||
                    status.state == MOTION_STATE_FAULT) {
                    capture_add(&status);
                }
                if (status.state == MOTION_STATE_FAULT) {
                    capture_finish_reason = "motion fault";
                }
                if (capture_force_finish || status.state == MOTION_STATE_COMPLETE ||
                    status.state == MOTION_STATE_FAULT ||
                    (capture_seen_running && status.state == MOTION_STATE_STOPPED)) {
                    capture_active = false;
                    capture_dump();
                }
            }
            tk_dly_tsk(MOTION_CAPTURE_PERIOD_MS);
        }
    }
    for (;;) tk_dly_tsk(1000);
    return 0;
}
