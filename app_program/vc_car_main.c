/** @file vc_car_main.c
 *
 * @brief APP=vc_car: the Vehicle Controller driving the real car.
 *
 * Starts comms (Buddy 1), Motion (Buddy 2) and the VC task.  The WiFi
 * radio task is started by the kernel when built with WIFI=cyw43.
 *
 * Also starts Buddy 3 (line + barcode) and Buddy 5 (obstacle ahead); the
 * VC reads them as switched on in vc_task.c.  Laptop "start" -> the car
 * follows the line; "stop" brakes at once.
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
#include "buddy3_task.h"
#include "obstacle_types.h"
#include "obstacle_task.h"

#if TM_WIFI_CYW43
/* Plain C types on purpose - see the note in cyw43_utk.h. */
#include "cyw43_utk.h"
#endif

#define USB_WAIT_POLLS     (50)      /* x 100 ms: wait up to 5 s for USB. */
#define LINK_POLL_MS       (500U)    /* How often to check the MQTT link. */
#define BUDDY3_PRIORITY    (5)       /* As in Buddy 3's own app_main.c.   */
#define BUDDY3_STACK_BYTES (4096U)

static UW g_buddy3_stack[BUDDY3_STACK_BYTES / sizeof(UW)];

#if TM_WIFI_CYW43
/* USB interrupts taken since boot (usb_sdk_compat.c). */
IMPORT volatile UW tm_usb_irq_count;
#endif

#define DIAG_EVERY         (10U)     /* x LINK_POLL_MS = every 5 s.        */


/* Every 5 s: WiFi task polls in that time (about 2500 expected at one
 * poll per 2 ms; far fewer = the task is being held up), link, signal,
 * MQTT state and rejoin counters.  Diagnostics for test_integrate. */
static void
report_radio (void)
{
#if TM_WIFI_CYW43
    static uint32_t   s_tick       = 0U;
    static uint32_t   s_last_polls = 0U;
    static uint32_t   s_last_usb   = 0U;
    T_CYW43_UTK_STATUS st;
    uint32_t          usb_irqs;

    s_tick++;

    if (DIAG_EVERY <= s_tick)
    {
        s_tick = 0U;
        cyw43_utk_get_status(&st);
        usb_irqs = tm_usb_irq_count;
        tm_printf((UB *) "[diag] polls/5s=%u usbirq/5s=%u link=%u join=%d"
                         " rssi=%d mqtt=%u err=%d losses=%u rejoins=%u\n",
                  (UINT) (st.poll_count - s_last_polls),
                  (UINT) (usb_irqs - s_last_usb),
                  (UINT) st.netif_link_up, (INT) st.link_status,
                  (INT) st.link_rssi, (UINT) st.mqtt_state,
                  (INT) st.mqtt_last_error, (UINT) st.link_losses,
                  (UINT) st.rejoin_attempts);
        s_last_polls = st.poll_count;
        s_last_usb   = usb_irqs;
    }
#endif
}


/* Buddy 3: output message buffer, then the line/barcode task. */
static bool
start_buddy3 (void)
{
    T_CTSK ctsk = {0};
    ID     tid;
    bool   b_ok = false;

    if (0 < buddy3_output_mbf_create())
    {
        ctsk.tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF;
        ctsk.task    = buddy3_task;
        ctsk.itskpri = BUDDY3_PRIORITY;
        ctsk.stksz   = (SZ) sizeof(g_buddy3_stack);
        ctsk.bufptr  = g_buddy3_stack;
        tid          = tk_cre_tsk(&ctsk);
        b_ok         = (0 < tid) && (E_OK == tk_sta_tsk(tid, 0));
    }

    return b_ok;
}

EXPORT INT
usermain (void)
{
    motion_result_t motion_result;
    bool            b_comms_ok;
    bool            b_line_ok;
    bool            b_obstacle_ok;
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

    b_line_ok     = start_buddy3();
    b_obstacle_ok = obstacle_task_init() && obstacle_task_start();

    b_ok = b_comms_ok && (MOTION_RESULT_OK == motion_result) && b_line_ok
           && b_obstacle_ok;
    tm_printf((UB *) "[init] comms=%d motion=%d line=%d obstacle=%d -> %s\n",
              (INT) b_comms_ok, (INT) motion_result, (INT) b_line_ok,
              (INT) b_obstacle_ok, b_ok ? "OK" : "FAILED");

    if (!vc_task_start(b_ok))
    {
        tm_printf((UB *) "[init] VC task could not start\n");
    }

    tm_printf((UB *) "Send \"start\" / \"stop\" to robot/command.\n");

    /* Nothing else to start.  Report MQTT link changes, and every 5 s how
     * often the WiFi task ran (test_integrate diagnostics: the connection
     * drops for ~59 s at a time in the combined firmware).  Never return
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

        report_radio();
        (void) tk_dly_tsk(LINK_POLL_MS);
    }

    return 0;
}

/*** end of file ***/
