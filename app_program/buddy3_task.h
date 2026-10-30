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

/* Barcode black threshold in use: learned by the START sweep, else the
 * fallback constant. */
uint16_t buddy3_barcode_threshold(void);

/* Latest raw 12-bit centre and right line sensor readings (telemetry). */
void buddy3_get_readings(uint16_t *p_centre, uint16_t *p_right);


/*
 * START calibration requests from the Vehicle Controller, which is the
 * only caller. Buddy 3 learns the IR thresholds from the readings it
 * sees while the car pivots across the line, and reports progress in its
 * status word (line_cal_state_t).
 *   BEGIN - start recording the sensors;
 *   END   - the pivot hit its limit: judge the sweep (OK or a failure);
 *   ABORT - forget the sweep (stop or failure elsewhere).
 */
typedef enum
{
    BUDDY3_CAL_BEGIN = 1,
    BUDDY3_CAL_END,
    BUDDY3_CAL_ABORT,
    BUDDY3_CAL_SWEEP            /* The pivot has started: record now.  */
} buddy3_cal_request_t;

void buddy3_calibration_request(buddy3_cal_request_t request);


#endif /* BUDDY3_TASK_H_ */