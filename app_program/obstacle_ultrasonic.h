/** @file obstacle_ultrasonic.h
 *
 * @brief HC-SR04 ultrasonic ranging driver (Buddy 5).
 *
 * Single owner: call only from one task.  Not callable from an ISR.
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h>, <stdbool.h> and
 * "obstacle_types.h" before this header.
 */

#ifndef OBSTACLE_ULTRASONIC_H
#define OBSTACLE_ULTRASONIC_H

/* Configure the Trig/Echo pins and the echo-timing PWM slice.
 * Returns false if the PWM block is held in reset (wrong BOARD). */
bool obstacle_ultrasonic_init(void);

/* Take one measurement.  Blocks the calling task for 40..110 ms. */
void obstacle_ultrasonic_measure(obstacle_range_t * p_range);

#endif /* OBSTACLE_ULTRASONIC_H */

/*** end of file ***/
