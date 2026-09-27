/** @file obstacle_servo.h
 *
 * @brief SG90 scan-servo driver (Buddy 5).
 *
 * Angles are whole degrees; 90 points straight ahead.  Single owner: call
 * only from one task.
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h> and <stdbool.h>
 * before this header.
 */

#ifndef OBSTACLE_SERVO_H
#define OBSTACLE_SERVO_H

/* Start 50 Hz PWM and centre the servo.  Returns false if the PWM block
 * is held in reset (wrong BOARD). */
bool obstacle_servo_init(void);

/* Move to angle_deg, limited to the configured safe range.  If
 * p_settle_ms is not NULL it receives the estimated settling time. */
bool obstacle_servo_set_angle(uint32_t angle_deg, uint32_t * p_settle_ms);

/* Calibration only: output a raw pulse width (absolute limits applied). */
bool obstacle_servo_set_pulse_us(uint32_t pulse_us);

/* Last commanded angle (not updated by raw pulse commands). */
uint32_t obstacle_servo_get_angle(void);

/* Pulse width currently being output. */
uint32_t obstacle_servo_get_pulse_us(void);

#endif /* OBSTACLE_SERVO_H */

/*** end of file ***/
