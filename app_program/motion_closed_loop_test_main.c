#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>

#include "motion_task.h"

#if TM_CONSOLE_USB_CDC
extern INT tm_usb_console_getchar(void);
#endif

#define MOTION_CAPTURE_SAMPLES 40U
#define MOTION_CAPTURE_PERIOD_MS 100U
#define MOTION_REVERSAL_DELAY_MS 500U

static uint32_t test_sequence = 1U;
static uint32_t submitted_sequence;
static uint32_t velocity_watchdog_ms;
static motion_status_t capture_samples[MOTION_CAPTURE_SAMPLES];
static uint32_t capture_count;
static bool capture_active;
static bool capture_seen_running;
static bool capture_force_finish;
static uint32_t capture_sequence;
static uint32_t capture_sequence_secondary;
static bool reversal_test_active;
static uint32_t reversal_delay_ms;
static bool stop_priority_test_active;
static uint32_t stop_priority_delay_ms;
static bool emergency_stop_test_active;
static uint32_t emergency_stop_delay_ms;
static bool emergency_stop_requested;
static uint64_t emergency_stop_request_timestamp_us;
static const char *capture_name;
static const char *capture_finish_reason;

static uint64_t test_timestamp_us(void)
{
    uint32_t low;
    uint32_t high;

    low = in_w(TIMER_TIMELR);
    high = in_w(TIMER_TIMEHR);
    return ((uint64_t)high << 32) | low;
}

static void print_help(void)
{
    tm_printf((UB *)"[motion-closed-loop] keys: 1=100 2=200 3=300 mm/s forward, r=100 reverse, x=stop\n");
    tm_printf((UB *)"[motion-closed-loop] a=left curve d=right curve; l=-90 t=+90 u=180 deg; m=100 mm g=300 mm diagnostic\n");
    tm_printf((UB *)"[motion-closed-loop] v=automatic reversal, p=stop-priority, e=timestamped emergency stop; h=help; samples print only after brake\n");
    tm_printf((UB *)"[motion-closed-loop] safety: o=500 ms timeout, i=invalid command, q=zero-speed queue-full test\n");
}

static void capture_begin(const char *name)
{
    capture_count = 0U;
    capture_active = true;
    capture_seen_running = false;
    capture_force_finish = false;
    capture_sequence = submitted_sequence;
    capture_sequence_secondary = 0U;
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
    tm_printf((UB *)"[motion-sample] n=%u t_us=%u state_t_us=%u seq=%u state=%d fault=%d enc_detail=%d enc_wheel=%u enc_dt_ms=%u enc_delta=%d/%d count=%d/%d speed=%d/%d travel=%d target=%d pwm=%u/%u invalid=%u/%u total=%u irq=%u\n",
              sample, (uint32_t)status->timestamp_us,
              (uint32_t)status->state_timestamp_us,
              status->active_sequence_id, status->state, status->fault,
              status->encoder_fault, status->encoder_fault_wheel_mask,
              status->encoder_fault_elapsed_ms,
              status->encoder_fault_left_delta,
              status->encoder_fault_right_delta,
              status->left_encoder_counts, status->right_encoder_counts,
              status->left_speed_mm_s, status->right_speed_mm_s,
              status->travelled_mm, status->target_mm,
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
        /* Avoid overrunning the USB CDC transmit path during a long dump. */
        (void)tk_dly_tsk(10U);
    }
}

static motion_result_t submit_velocity(int32_t linear_mm_s,
                                       int32_t angular_mdeg_s)
{
    motion_command_t command = {0};
    motion_result_t result;
    command.type = MOTION_COMMAND_SET_VELOCITY;
    command.sequence_id = test_sequence++;
    submitted_sequence = command.sequence_id;
    command.timeout_ms = 0U;
    command.value.velocity.linear_mm_s = linear_mm_s;
    command.value.velocity.angular_mdeg_s = angular_mdeg_s;
    result = motion_submit(&command);
    if (result == MOTION_RESULT_OK) velocity_watchdog_ms = 3000U;
    return result;
}

static motion_result_t submit_timeout_test(void)
{
    motion_command_t command = {0};
    command.type = MOTION_COMMAND_SET_VELOCITY;
    command.sequence_id = test_sequence++;
    submitted_sequence = command.sequence_id;
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

static motion_result_t submit_move(int32_t distance_mm,
                                   int32_t speed_mm_s,
                                   uint32_t timeout_ms)
{
    motion_command_t command = {0};
    motion_result_t result;
    command.type = MOTION_COMMAND_MOVE_DISTANCE;
    command.sequence_id = test_sequence++;
    submitted_sequence = command.sequence_id;
    command.timeout_ms = timeout_ms;
    command.value.distance.distance_mm = distance_mm;
    command.value.distance.speed_mm_s = speed_mm_s;
    result = motion_submit(&command);
    if (result == MOTION_RESULT_OK) velocity_watchdog_ms = 0U;
    return result;
}

static motion_result_t submit_turn(int32_t angle_mdeg)
{
    motion_command_t command = {0};
    motion_result_t result;
    command.type = MOTION_COMMAND_TURN_ANGLE;
    command.sequence_id = test_sequence++;
    submitted_sequence = command.sequence_id;
    command.timeout_ms = 5000U;
    command.value.turn.angle_mdeg = angle_mdeg;
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

static void request_test(const char *name, int32_t velocity_mm_s,
                         int32_t angular_mdeg_s)
{
    motion_result_t result;
    tm_printf((UB *)"[motion-closed-loop] starting %s; logging paused until brake\n",
              (UB *)name);
    result = submit_velocity(velocity_mm_s, angular_mdeg_s);
    start_test(name, result);
}

static void start_reversal_test(void)
{
    motion_result_t result;

    tm_printf((UB *)"[motion-closed-loop] starting automatic forward-to-reverse test; reversal in 500 ms\n");
    result = submit_velocity(100, 0);
    if (result == MOTION_RESULT_OK) {
        capture_begin("forward-to-reverse");
        reversal_test_active = true;
        reversal_delay_ms = MOTION_REVERSAL_DELAY_MS;
    } else {
        tm_printf((UB *)"[motion-closed-loop] forward-to-reverse rejected result=%d\n",
                  result);
    }
}

static void start_stop_priority_test(void)
{
    motion_result_t result;

    tm_printf((UB *)"[motion-closed-loop] starting automatic stop-priority test; stop and reverse in 500 ms\n");
    result = submit_velocity(100, 0);
    if (result == MOTION_RESULT_OK) {
        capture_begin("stop-priority");
        stop_priority_test_active = true;
        stop_priority_delay_ms = MOTION_REVERSAL_DELAY_MS;
    } else {
        tm_printf((UB *)"[motion-closed-loop] stop-priority forward command rejected result=%d\n",
                  result);
    }
}

static void start_emergency_stop_test(void)
{
    motion_result_t result;

    tm_printf((UB *)"[motion-closed-loop] starting timestamped emergency-stop test; stop in 500 ms\n");
    result = submit_velocity(100, 0);
    if (result == MOTION_RESULT_OK) {
        capture_begin("emergency-stop");
        emergency_stop_test_active = true;
        emergency_stop_delay_ms = MOTION_REVERSAL_DELAY_MS;
        emergency_stop_requested = false;
    } else {
        tm_printf((UB *)"[motion-closed-loop] emergency-stop forward command rejected result=%d\n",
                  result);
    }
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
                        request_test("100 mm/s", 100, 0);
                        break;
                    case '2':
                        request_test("200 mm/s", 200, 0);
                        break;
                    case '3':
                        request_test("300 mm/s", 300, 0);
                        break;
                    case 'r': case 'R':
                        request_test("-100 mm/s", -100, 0);
                        break;
                    case 'a': case 'A':
                        request_test("left curve", 100, -30000);
                        break;
                    case 'd': case 'D':
                        request_test("right curve", 100, 30000);
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
                    case 'v': case 'V':
                        start_reversal_test();
                        break;
                    case 'p': case 'P':
                        start_stop_priority_test();
                        break;
                    case 'e': case 'E':
                        start_emergency_stop_test();
                        break;
                    case 'm': case 'M':
                        tm_printf((UB *)"[motion-closed-loop] starting move 100 mm; logging paused until brake\n");
                        start_test("move 100 mm", submit_move(100, 80, 5000U));
                        break;
                    case 'g': case 'G':
                        tm_printf((UB *)"[motion-closed-loop] starting diagnostic move 300 mm at 100 mm/s; logging paused until brake\n");
                        start_test("diagnostic move 300 mm",
                                   submit_move(300, 100, 6000U));
                        break;
                    case 'l': case 'L':
                        tm_printf((UB *)"[motion-closed-loop] starting turn -90 deg; logging paused until brake\n");
                        start_test("turn -90 deg", submit_turn(-90000));
                        break;
                    case 't': case 'T':
                        tm_printf((UB *)"[motion-closed-loop] starting turn +90 deg; logging paused until brake\n");
                        start_test("turn +90 deg", submit_turn(90000));
                        break;
                    case 'u': case 'U':
                        tm_printf((UB *)"[motion-closed-loop] starting turn 180 deg; logging paused until brake\n");
                        start_test("turn 180 deg", submit_turn(180000));
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
            if (reversal_test_active && reversal_delay_ms > 0U) {
                if (reversal_delay_ms <= MOTION_CAPTURE_PERIOD_MS) {
                    result = submit_velocity(-100, 0);
                    if (result == MOTION_RESULT_OK) {
                        capture_sequence_secondary = submitted_sequence;
                        reversal_test_active = false;
                        reversal_delay_ms = 0U;
                        tm_printf((UB *)"[motion-closed-loop] reversal submitted seq=%u\n",
                                  capture_sequence_secondary);
                    } else {
                        reversal_test_active = false;
                        reversal_delay_ms = 0U;
                        capture_finish_reason = "reversal submission failed";
                        capture_force_finish = true;
                        (void)motion_stop();
                    }
                } else {
                    reversal_delay_ms -= MOTION_CAPTURE_PERIOD_MS;
                }
            }
            if (stop_priority_test_active && stop_priority_delay_ms > 0U) {
                if (stop_priority_delay_ms <= MOTION_CAPTURE_PERIOD_MS) {
                    motion_result_t stop_result = motion_stop();
                    if (stop_result == MOTION_RESULT_OK) {
                        result = submit_velocity(-100, 0);
                    } else {
                        result = stop_result;
                    }
                    if (result == MOTION_RESULT_OK) {
                        capture_sequence_secondary = submitted_sequence;
                        stop_priority_test_active = false;
                        stop_priority_delay_ms = 0U;
                        tm_printf((UB *)"[motion-closed-loop] stop requested result=%d; replacement submitted seq=%u\n",
                                  stop_result, capture_sequence_secondary);
                    } else {
                        stop_priority_test_active = false;
                        stop_priority_delay_ms = 0U;
                        capture_finish_reason = "stop-priority replacement failed";
                        capture_force_finish = true;
                        (void)motion_stop();
                    }
                } else {
                    stop_priority_delay_ms -= MOTION_CAPTURE_PERIOD_MS;
                }
            }
            if (emergency_stop_test_active && emergency_stop_delay_ms > 0U) {
                if (emergency_stop_delay_ms <= MOTION_CAPTURE_PERIOD_MS) {
                    uint64_t request_timestamp_us = test_timestamp_us();
                    motion_result_t stop_result = motion_stop();
                    velocity_watchdog_ms = 0U;
                    emergency_stop_test_active = false;
                    emergency_stop_delay_ms = 0U;
                    if (stop_result == MOTION_RESULT_OK) {
                        emergency_stop_request_timestamp_us = request_timestamp_us;
                        emergency_stop_requested = true;
                        capture_finish_reason = "emergency stop";
                        tm_printf((UB *)"[motion-closed-loop] emergency stop requested result=%d request_t_us=%u; waiting for STOPPED\n",
                                  stop_result, (uint32_t)request_timestamp_us);
                    } else {
                        emergency_stop_requested = false;
                        capture_finish_reason = "emergency stop request failed";
                        capture_force_finish = true;
                        tm_printf((UB *)"[motion-closed-loop] emergency stop request failed result=%d\n",
                                  stop_result);
                    }
                } else {
                    emergency_stop_delay_ms -= MOTION_CAPTURE_PERIOD_MS;
                }
            }
            if (motion_get_status(&status) == MOTION_RESULT_OK && capture_active) {
                bool sequence_matches =
                    status.active_sequence_id == capture_sequence ||
                    (capture_sequence_secondary != 0U &&
                     status.active_sequence_id == capture_sequence_secondary);
                bool forced_stop_complete = capture_force_finish &&
                    status.state == MOTION_STATE_STOPPED;

                if (sequence_matches && status.state == MOTION_STATE_RUNNING) {
                    capture_seen_running = true;
                }
                if ((sequence_matches &&
                     (capture_seen_running ||
                      status.state == MOTION_STATE_COMPLETE ||
                      status.state == MOTION_STATE_FAULT)) ||
                    forced_stop_complete) {
                    capture_add(&status);
                }
                if (sequence_matches && status.state == MOTION_STATE_FAULT) {
                    capture_finish_reason = "motion fault";
                }
                if (emergency_stop_requested && status.state == MOTION_STATE_STOPPED) {
                    uint64_t latency_us = status.state_timestamp_us -
                                          emergency_stop_request_timestamp_us;
                    tm_printf((UB *)"[motion-closed-loop] emergency stop observed request_t_us=%u brake_t_us=%u latency_us=%u latency_ms=%u\n",
                              (uint32_t)emergency_stop_request_timestamp_us,
                              (uint32_t)status.state_timestamp_us,
                              (uint32_t)latency_us,
                              (uint32_t)((latency_us + 999ULL) / 1000ULL));
                    emergency_stop_requested = false;
                }
                /* Ignore a stale terminal state belonging to the preceding
                 * command. motion_stop() is asynchronous, so an explicit stop
                 * also waits for STOPPED before the capture is printed. */
                if ((sequence_matches &&
                     (status.state == MOTION_STATE_COMPLETE ||
                      status.state == MOTION_STATE_FAULT ||
                      (capture_seen_running &&
                       status.state == MOTION_STATE_STOPPED))) ||
                    forced_stop_complete) {
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
