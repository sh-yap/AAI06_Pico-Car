#ifndef BUDDY2_MOTION_TASK_H
#define BUDDY2_MOTION_TASK_H

#include "motion_types.h"

motion_result_t motion_init(void);
motion_result_t motion_start_task(void);
motion_result_t motion_submit(const motion_command_t *command);
motion_result_t motion_stop(void);
motion_result_t motion_get_status(motion_status_t *status);

#endif
