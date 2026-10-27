#ifndef BUDDY3_TASK_H_
#define BUDDY3_TASK_H_

#include <tk/tkernel.h>

#include "buddy3_status.h"


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


/*
 * Output message buffer: each message is one buddy3_packed_t
 * (4 bytes). Consumers decode it with buddy3_status_unpack().
 */
extern ID buddy3_output_mbfid;


ID buddy3_output_mbf_create(void);


#endif /* BUDDY3_TASK_H_ */