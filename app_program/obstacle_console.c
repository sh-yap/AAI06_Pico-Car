/** @file obstacle_console.c
 *
 * @brief Console helpers shared by the Buddy 5 bench test programs.
 */

#include <stdint.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "usb_console_compat.h"
#include "obstacle_types.h"
#include "obstacle_console.h"

#define CONSOLE_POLL_MS        (100)   /* TMO values for tk_dly_tsk().     */
#define CONSOLE_POLL_LIMIT     (200U)  /* 200 * 100 ms = 20 s.            */
#define CONSOLE_SETTLE_MS      (500)
#define CONSOLE_USB_ENUMERATED (2U)    /* tm_usb_state() once enumerated. */
#define CONSOLE_NO_KEY         (-1)

#if TM_CONSOLE_USB_CDC
/* Deviation from BARR-C 4.3: the USB console module is supplied by the
 * lecturer and exports this function without a header, so it is declared
 * here, in the one file that uses it. */
IMPORT INT tm_usb_console_getchar(void);
#endif


/*!
 * @brief Wait for a USB host to enumerate the serial port.
 *
 * Output printed before the host opens the port is lost, so the test
 * programs wait here first.  Bounded, so a board with no host still runs.
 * On a UART build tm_usb_state() is constant and this returns quickly.
 */
void
obstacle_console_wait_host(void)
{
    uint32_t polls = 0U;

    while ((polls < CONSOLE_POLL_LIMIT)
           && (tm_usb_state() < CONSOLE_USB_ENUMERATED))
    {
        (void)tk_dly_tsk(CONSOLE_POLL_MS);
        polls++;
    }

    (void)tk_dly_tsk(CONSOLE_SETTLE_MS);
}

/*!
 * @brief Read one key from the console without blocking.
 *
 * @return The key code, or -1 if no key is waiting (always -1 on a UART
 *         console build, which has no receive path here).
 */
int32_t
obstacle_console_read_key(void)
{
    int32_t key = CONSOLE_NO_KEY;

#if TM_CONSOLE_USB_CDC
    key = tm_usb_console_getchar();
#endif

    return (key);
}

/*!
 * @brief Short text name for a range status.
 *
 * @param[in] status Status from obstacle_ultrasonic_measure().
 *
 * @return Constant string, never NULL.
 */
char const *
obstacle_console_status_name(obstacle_range_status_t status)
{
    char const * p_name = "UNKNOWN";

    switch (status)
    {
        case OBSTACLE_RANGE_OK:
            p_name = "OK";
            break;

        case OBSTACLE_RANGE_NO_ECHO:
            p_name = "NO_ECHO";
            break;

        case OBSTACLE_RANGE_TOO_CLOSE:
            p_name = "TOO_CLOSE";
            break;

        case OBSTACLE_RANGE_TOO_FAR:
            p_name = "TOO_FAR";
            break;

        case OBSTACLE_RANGE_TIMEOUT:
            p_name = "TIMEOUT";
            break;

        case OBSTACLE_RANGE_NOT_READY:
            p_name = "NOT_READY";
            break;

        default:
            p_name = "UNKNOWN";
            break;
    }

    return (p_name);
}

/*** end of file ***/
