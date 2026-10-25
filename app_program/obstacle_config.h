/** @file obstacle_config.h
 *
 * @brief Buddy 5 pin assignments and calibration constants.
 *
 * Every Buddy 5 pin and tunable lives here so that moving a connector or
 * applying a new calibration is a one-line change.  Values marked
 * "calibrate" are starting guesses; replace them with the measurements
 * recorded by ultrasonic_test and servo_test.
 *
 * BARR-C 4.2: this header contains no #include; it needs nothing.
 */

#ifndef OBSTACLE_CONFIG_H
#define OBSTACLE_CONFIG_H

/* ------------------------------------------------------------------------ *
 * Pins (Robo-Pico)
 * ------------------------------------------------------------------------ */

/* Servo signal.  S1 = GP12, S2 = GP13, S3 = GP14, S4 = GP15. */
#define OBSTACLE_SERVO_PIN              (12U)

/* HC-SR04 on Grove 1 (GP0/GP1), the only Grove port free on the car.
 * These are the UART0 console pins at boot; obstacle_ultrasonic_init()
 * takes them over, so only the UART copy of the console is lost (the USB
 * console is unaffected).  Servo ports S3/S4 were tried first but did not
 * pass a Trig-to-Echo loopback test, so they may be output-only. */

/* HC-SR04 Trigger: any free GPIO. */
#define OBSTACLE_TRIG_PIN               (0U)

/* HC-SR04 Echo: must be an ODD pin (PWM channel B), because its PWM slice
 * times the echo pulse in hardware.  Feed it through a divider to 3.3 V. */
#define OBSTACLE_ECHO_PIN               (1U)

/* ------------------------------------------------------------------------ *
 * Ultrasonic (HC-SR04)
 * ------------------------------------------------------------------------ */

/* Speed of sound in m/s: 343 at 20 C, about 349 at 30 C.  Calibrate. */
#define OBSTACLE_SOUND_SPEED_M_S        (343U)

/* Datasheet range 20 mm .. 4000 mm, as echo time: distance * 2000 / 343. */
#define OBSTACLE_ECHO_MIN_US            (117U)
#define OBSTACLE_ECHO_MAX_US            (23324U)

/* Datasheet asks for more than 60 ms between measurements. */
#define OBSTACLE_PING_PERIOD_MS         (60U)

/* Time allowed for the echo to finish (4 m needs about 23.3 ms). */
#define OBSTACLE_ECHO_WAIT_MS           (40U)

/* Trigger pulse; the datasheet minimum is 10 us. */
#define OBSTACLE_TRIG_PULSE_US          (12U)

/* ------------------------------------------------------------------------ *
 * Servo (TZT SG90)
 * ------------------------------------------------------------------------ */

/* 50 Hz frame. */
#define OBSTACLE_SERVO_PERIOD_US        (20000U)

/* Absolute pulse limits, never exceeded even in raw calibration mode.
 * Bench test 2026-10-21 on the car: the sensor mount tilts from 1050 us
 * and the servo stalls (buzzes) at 950 us and 1950 us, so the limits sit
 * where it moved freely.  Widen them only after re-testing the mount. */
#define OBSTACLE_SERVO_PULSE_MIN_US     (1100U)
#define OBSTACLE_SERVO_PULSE_MAX_US     (1900U)

/* Pulse widths that put the horn at 0, 90 and 180 degrees.  Measured
 * 2026-10-25 with a protractor (about +-5 deg): 1540 us = straight ahead,
 * 1100 us = 35 deg right, 1900 us = 35 deg left.  0 and 180 deg are those
 * two slopes (12.6 and 10.3 us/deg) extended; the servo never goes there.
 * Low pulses point right, high pulses point left. */
#define OBSTACLE_SERVO_PULSE_0_US       (409U)
#define OBSTACLE_SERVO_PULSE_90_US      (1540U)
#define OBSTACLE_SERVO_PULSE_180_US     (2466U)

/* Angles accepted by obstacle_servo_set_angle().  The sensor bracket hits
 * the Pico board beyond about +-35 deg from straight ahead, so the range
 * is 60..120 deg (1163..1848 us) instead of the brief's 30..150 deg.
 * Pending a mount change agreed with the professor. */
#define OBSTACLE_SERVO_ANGLE_MIN_DEG    (60U)
#define OBSTACLE_SERVO_ANGLE_MAX_DEG    (120U)

/* Settling estimate: travel * MS_PER_DEG + MARGIN.  Bench test 2026-10-21
 * ('t', 60 -> 120 deg): readings wandered (315/284/310 mm) until 290 ms
 * and were steady from 410 ms, so a 60 deg swing is allowed 400 ms.  The
 * SG90 datasheet figure (2 ms/deg + 60 ms) was too optimistic. */
#define OBSTACLE_SERVO_MS_PER_DEG       (4U)
#define OBSTACLE_SERVO_SETTLE_MARGIN_MS (160U)

/* ------------------------------------------------------------------------ *
 * Obstacle task, MONITOR state
 * ------------------------------------------------------------------------ */

/* FR-B5-03: OBSTACLE_AHEAD after two consecutive readings under 250 mm. */
#define OBSTACLE_STOP_DISTANCE_MM       (250U)
#define OBSTACLE_STOP_READINGS          (2U)

/* Blind-zone latch.  Bench test 2026-10-25: a flat board at 1-2 cm reads
 * TIMEOUT, the same as open space.  After a valid reading under this
 * distance, a TIMEOUT means "still blocked". */
#define OBSTACLE_BLIND_LATCH_MM         (100U)

/* Lost-obstacle rule.  Bench test 2026-10-25 (D4): an off-centre obstacle
 * seen at 500 mm was lost at 250 mm.  An obstacle seen nearer than
 * LOST_NEAR_MM that then reads TIMEOUT, or more than LOST_JUMP_MM further,
 * for LOST_READINGS readings in a row raises OBSTACLE_AHEAD. */
#define OBSTACLE_LOST_NEAR_MM           (600U)
#define OBSTACLE_LOST_JUMP_MM           (200U)
#define OBSTACLE_LOST_READINGS          (2U)

/* Statechart: 5 NO_ECHO in a row -> SENSOR_FAULT; 3 readings with an
 * echo (OK, TIMEOUT, TOO_FAR or TOO_CLOSE) -> back to MONITOR. */
#define OBSTACLE_FAULT_READINGS         (5U)
#define OBSTACLE_RECOVER_READINGS       (3U)

/* A status older than this (kernel time) counts as no data. */
#define OBSTACLE_STALE_MS               (200U)

/* Task: priority 3 (above Line/Barcode at 4) on core 1. */
#define OBSTACLE_TASK_PRIORITY          (3)     /* PRI */
#define OBSTACLE_TASK_STACK_BYTES       (1024U)
#define OBSTACLE_FORWARD_DEG            (90U)

/* ------------------------------------------------------------------------ *
 * Build-time checks
 * ------------------------------------------------------------------------ */

#if ((OBSTACLE_SERVO_PULSE_90_US <= OBSTACLE_SERVO_PULSE_MIN_US) || \
     (OBSTACLE_SERVO_PULSE_90_US >= OBSTACLE_SERVO_PULSE_MAX_US))
#error "Servo centre pulse must lie inside the pulse limits"
#endif

#if ((OBSTACLE_ECHO_PIN % 2U) == 0U)
#error "OBSTACLE_ECHO_PIN must be odd (PWM channel B input)"
#endif

#if (((OBSTACLE_ECHO_PIN / 2U) % 8U) == ((OBSTACLE_SERVO_PIN / 2U) % 8U))
#error "Echo and servo pins must use different PWM slices"
#endif

/* PWM slices 4 and 5 (GP8..GP11) belong to Buddy 2's drive motors. */
#if ((((OBSTACLE_ECHO_PIN / 2U) % 8U) == 4U) || \
     (((OBSTACLE_ECHO_PIN / 2U) % 8U) == 5U))
#error "Echo pin would share a PWM slice with the drive motors"
#endif

#if ((((OBSTACLE_SERVO_PIN / 2U) % 8U) == 4U) || \
     (((OBSTACLE_SERVO_PIN / 2U) % 8U) == 5U))
#error "Servo pin would share a PWM slice with the drive motors"
#endif

#endif /* OBSTACLE_CONFIG_H */

/*** end of file ***/
