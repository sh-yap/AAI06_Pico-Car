/** @file vc_car_main.c
 *
 * @brief APP=vc_car: the Vehicle Controller driving the real car.
 *
 * Starts comms (Buddy 1), Motion (Buddy 2) and the VC task.  The WiFi
 * radio task is started by the kernel when built with WIFI=cyw43.
 *
 * First car test (before Buddy 3 is merged): laptop "start" -> the car
 * drives straight at normal speed (line is a placeholder "centred") and
 * stops by itself after 5 s; laptop "stop" brakes at once.
 *
 * Deviations (BARR-C): usermain() returns INT, signature fixed by
 * micro T-Kernel; this file has no header as it exports only usermain().
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <stdint.h>
#include <stdbool.h>

#include "usb_console_compat.h"
#include "build_info.h"
#include "comms.h"
#include "motion_types.h"
#include "motion_task.h"
#include "vc.h"
#include "vc_task.h"

#define USB_WAIT_POLLS     (50)      /* x 100 ms: wait up to 5 s for USB. */
#define LINK_POLL_MS       (500U)    /* How often to check the MQTT link. */

EXPORT INT
usermain (void)
{
    motion_result_t motion_result;
    bool            b_comms_ok;
    bool            b_ok;
    bool            b_was_connected = false;
    bool            b_connected;
    INT             i;

    /* Give the USB console a moment so the first lines are not lost; a
     * car without a laptop attached still starts after 5 s. */
    for (i = 0; (i < USB_WAIT_POLLS) && (2 > tm_usb_state()); i++)
    {
        (void) tk_dly_tsk(100U);
    }

    tm_printf((UB *) "\n=== Vehicle Controller (APP=vc_car) ===\n");
    tm_printf((UB *) "build %s\n", g_build_id);

    b_comms_ok    = comms_init();
    motion_result = motion_init();

    if (MOTION_RESULT_OK == motion_result)
    {
        motion_result = motion_start_task();
    }

    b_ok = b_comms_ok && (MOTION_RESULT_OK == motion_result);
    tm_printf((UB *) "[init] comms=%d motion=%d -> %s\n", (INT) b_comms_ok,
              (INT) motion_result, b_ok ? "OK" : "FAILED");

    if (!vc_task_start(b_ok))
    {
        tm_printf((UB *) "[init] VC task could not start\n");
    }

    tm_printf((UB *) "Send \"start\" / \"stop\" to robot/command.\n");

    /* Nothing else to start.  Report MQTT link changes; never return
     * (returning shuts the kernel down). */
    for (;;)
    {
        b_connected = comms_is_connected();

        if (b_connected != b_was_connected)
        {
            b_was_connected = b_connected;
            tm_printf((UB *) "[mqtt] %s\n",
                      b_connected ? "CONNECTED" : "not connected");
        }

        (void) tk_dly_tsk(LINK_POLL_MS);
    }

    return 0;
}

/*** end of file ***/
