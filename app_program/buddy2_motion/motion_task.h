#ifndef BUDDY2_MOTION_TASK_H
#define BUDDY2_MOTION_TASK_H

#include "motion_types.h"

motion_result_t motion_init(void);
motion_result_t motion_start_task(void);
motion_result_t motion_submit(const motion_command_t *command);
motion_result_t motion_stop(void);
motion_result_t motion_get_status(motion_status_t *status);

#ifndef VC_RECORDER
#define VC_RECORDER 0
#endif

#if VC_RECORDER
/* Test-only (run recorder): each wheel's PID integral term, ki * integral,
   in permille.  Lock-free; one float read per value. */
void motion_get_pid_integral_permille(int16_t *left, int16_t *right);

/* Test-only: the learned response, lock-free.  Forward gain and turning
   gain in thousandths of mm/s per permille, forward offset in tenths of
   mm/s.  state bits 0-1: turning gain source (0 default line, 1 pivot seed,
   2 driving data); bit 2: forward values come from driving data. */
void motion_get_learned(int16_t *fwd_gain_milli, int16_t *fwd_offset_dmm_s,
                        int16_t *turn_gain_milli, uint16_t *state);
#endif

#endif
