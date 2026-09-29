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
 * times the echo pulse in hardware.  GP1 = PWM slice 0 channel B.
 * The sensor is powered from Grove 1's 3.3 V, so Echo is 3.3 V and connects
 * directly.  If the sensor is ever powered from 5 V, Echo MUST go through
 * a divider (1k series, 2k to GND): 5 V damages the Pico. */
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

/* Absolute pulse limits, never exceeded even in raw calibration mode. */
#define OBSTACLE_SERVO_PULSE_MIN_US     (500U)
#define OBSTACLE_SERVO_PULSE_MAX_US     (2500U)

/* Pulse widths that put the horn at 0, 90 and 180 degrees.  Calibrate. */
#define OBSTACLE_SERVO_PULSE_0_US       (600U)
#define OBSTACLE_SERVO_PULSE_90_US      (1500U)
#define OBSTACLE_SERVO_PULSE_180_US     (2400U)

/* Angles accepted by obstacle_servo_set_angle().  Kept away from the end
 * stops until servo_test has measured the real safe range. */
#define OBSTACLE_SERVO_ANGLE_MIN_DEG    (15U)
#define OBSTACLE_SERVO_ANGLE_MAX_DEG    (165U)

/* Settling estimate: SG90 is about 0.1 s per 60 deg at 4.8 V, plus margin
 * for the horn to stop wobbling.  Calibrate with servo_test 't'. */
#define OBSTACLE_SERVO_MS_PER_DEG       (2U)
#define OBSTACLE_SERVO_SETTLE_MARGIN_MS (60U)

/* ------------------------------------------------------------------------ *
 * Build-time checks
 * ------------------------------------------------------------------------ */

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
