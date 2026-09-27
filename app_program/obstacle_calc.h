/** @file obstacle_calc.h
 *
 * @brief Hardware-independent Buddy 5 calculations.
 *
 * Pure functions only: no registers, no RTOS.  They are shared by the
 * drivers and compiled on the host by tests/obstacle for unit testing.
 *
 * BARR-C 4.2: no #include here.  Include <stdint.h>, <stdbool.h> and
 * "obstacle_types.h" before this header.
 */

#ifndef OBSTACLE_CALC_H
#define OBSTACLE_CALC_H

/* Classify a raw echo width; b_echo_high = echo pin still high at read. */
obstacle_range_status_t obstacle_calc_classify_echo(uint32_t echo_us,
                                                    bool     b_echo_high);

/* Convert an echo width to a one-way distance in millimetres. */
uint32_t obstacle_calc_echo_to_mm(uint32_t echo_us);

/* Map a servo angle (0..180 deg) to a pulse width using p_cal. */
uint32_t obstacle_calc_angle_to_pulse(uint32_t                     angle_deg,
                                      obstacle_servo_cal_t const * p_cal);

/* Limit a pulse width to the absolute servo limits. */
uint32_t obstacle_calc_clamp_pulse(uint32_t pulse_us);

/* Estimated time for the servo to move and settle between two angles. */
uint32_t obstacle_calc_settle_ms(uint32_t from_deg, uint32_t to_deg);

/* Median of p_values[0..count-1].  Sorts the array in place. */
uint32_t obstacle_calc_median(uint32_t * p_values, uint32_t count);

/* RP2040 PWM slice (0..7) that drives a GPIO pin. */
uint32_t obstacle_calc_pwm_slice(uint32_t gpio_pin);

#endif /* OBSTACLE_CALC_H */

/*** end of file ***/
