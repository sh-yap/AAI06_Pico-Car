/** @file obstacle_console.h
 *
 * @brief Console helpers shared by the Buddy 5 bench test programs.
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h> and
 * "obstacle_types.h" before this header.
 */

#ifndef OBSTACLE_CONSOLE_H
#define OBSTACLE_CONSOLE_H

/* Wait (at most ~20 s) for a USB host to open the serial port. */
void obstacle_console_wait_host(void);

/* Next key typed on the USB console, or -1 if none is waiting. */
int32_t obstacle_console_read_key(void);

/* Short text name for a range status, for logs. */
char const * obstacle_console_status_name(obstacle_range_status_t status);

#endif /* OBSTACLE_CONSOLE_H */

/*** end of file ***/
