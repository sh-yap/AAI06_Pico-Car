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

/* Pulse widths that put the horn at 0, 90 and 180 degrees.  90 deg
 * (straight ahead) was measured at 1540 us; 0 and 180 deg use the SG90's
 * nominal 10 us/deg about it until the angles are measured properly.
 * Low pulses point right, high pulses point left. */
#define OBSTACLE_SERVO_PULSE_0_US       (640U)
#define OBSTACLE_SERVO_PULSE_90_US      (1540U)
#define OBSTACLE_SERVO_PULSE_180_US     (2440U)

/* Angles accepted by obstacle_servo_set_angle().  The sensor bracket hits
 * the Pico board beyond about +-30 deg from straight ahead, so the range
 * is 60..120 deg (1240..1840 us) instead of the brief's 30..150 deg.
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
