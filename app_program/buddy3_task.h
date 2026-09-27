#ifndef BUDDY3_TASK_H_
#define BUDDY3_TASK_H_

#include <tk/tkernel.h>

#include "line_follow.h"
#include "navigation.h"


/*
 * Buddy 3 RTOS task.
 *
 * Handles:
 * - IR sensor readings
 * - line position estimation
 * - continuous line error
 * - junction detection
 * - barcode processing
 */
void buddy3_task(INT stacd, void *exinf);


/* Message sent to consumers */
typedef struct
{
    line_state_t line_state;
    float line_error;
    nav_command_t nav_command;
} buddy3_output_msg_t;


extern ID buddy3_output_mbfid;


ID buddy3_output_mbf_create(void);


#endif /* BUDDY3_TASK_H_ */