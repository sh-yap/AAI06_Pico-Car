/** @file vc_task.h
 *
 * @brief Vehicle Controller RTOS task: runs vc_step() every 20 ms with real
 *        subsystem inputs and turns its outputs into subsystem calls.
 *
 * vc.c is the pure decision logic (host-tested).  This task is the only
 * code that connects it to the car:
 *
 *   inputs  <- Motion (Buddy 2), line/barcode (Buddy 3), comms (Buddy 1),
 *              placeholders for subsystems not integrated yet
 *   outputs -> motion_submit()/motion_stop(), comms state/fault/run events
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h> and <stdbool.h>
 * before this header.
 */

#ifndef VC_TASK_H
#define VC_TASK_H

/* Serious fault bits passed to the VC (and published as robot/state fault).
 * PROPOSED until the team agrees the fault mask (tracker item C3). */
#define VC_FAULT_MOTION          (1U << 0)  /* Motion reports FAULT.        */
#define VC_FAULT_MOTION_REJECTED (1U << 1)  /* Motion refused a command.    */
#define VC_FAULT_OBSTACLE        (1U << 2)  /* Ultrasonic sensor fault.     */

/* Create and start the VC task.  b_init_ok: every subsystem started
 * (otherwise the VC goes straight to EMERGENCY_STOP).  Call once from
 * usermain() after comms_init() and motion_start_task().
 * Returns false if the task could not be created. */
bool vc_task_start(bool b_init_ok);

#endif /* VC_TASK_H */

/*** end of file ***/
