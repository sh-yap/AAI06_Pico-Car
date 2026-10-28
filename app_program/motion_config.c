#include "motion_config.h"

#include <stddef.h>

void motion_config_default(motion_config_t *config)
{
    if (config == NULL) {
        return;
    }
    config->left_motor_a = 8U;
    config->left_motor_b = 9U;
    config->right_motor_a = 10U;
    config->right_motor_b = 11U;
    /* Physical six-pin motor/encoder harness:
       left pin 2 -> GP16, left pin 3 -> GP17;
       right pin 2 -> GP4, right pin 3 -> GP5. */
    /* Decode the left phases in reverse logical order so vehicle-forward
       motion is positive without changing the documented harness wiring. */
    config->left_encoder_a = 17U;
    config->left_encoder_b = 16U;
    config->right_encoder_a = 4U;
    config->right_encoder_b = 5U;
    /* Manual wheel calibration, 2026-10-27: two marked ten-revolution
       forward trials.  Means were 2535.7 left and 2528.6 right counts per
       wheel revolution; retain separate rounded values for each encoder. */
    config->left_counts_per_wheel_rev = 2536;
    config->right_counts_per_wheel_rev = 2529;
    /* Loaded rolling calibration, 2026-10-27, complete car approximately
       665 g. A controlled 300 mm forward-and-return test gave bidirectional
       effective diameters of 57.729 mm left and 57.597 mm right. Rounded
       values are retained here; Stage 6 powered distance tests will refine
       them if tyre slip produces a systematic error. */
    config->left_wheel_diameter_um = 57700;
    config->right_wheel_diameter_um = 57600;
    config->track_width_um = 145000;
    config->max_speed_mm_s = 300;
    /* This is a hardware plausibility bound, not a permitted command speed.
       Loaded raw-PWM testing measured about 910 mm/s at 350 permille, so a
       1000 mm/s physical bound covers real launch transients. */
    config->encoder_physical_max_speed_mm_s = 1000;
    config->max_pwm_permille = 900U;
    config->deadband_permille = 80U;
    /* Provisional loaded start value. Earlier tests changed both firmware and
       power source, so they do not establish powerbank noise as the cause of
       the encoder fault. Repeat with the same image on both supplies. */
    config->startup_pwm_permille = 200U;
    /* Initial unloaded affine fits from the measured 80/100-permille runs.
       Slope units are PWM permille per mm/s, represented in Q10.
       Offsets were 59/57 (unloaded).  On the floor 200 mm/s needed about
       220-260 permille vs 160 predicted (27 Oct closed-loop test), so the
       offsets are raised on test_integrate - Buddy 2 to retune loaded. */
    config->left_feedforward_offset_permille = 120U;
    config->right_feedforward_offset_permille = 120U;
    config->left_feedforward_slope_q10 = 318U;
    config->right_feedforward_slope_q10 = 301U;
    /* Extra slope above 100 mm/s, derived from the first 200 mm/s run. */
    config->left_feedforward_high_slope_q10 = 400U;
    config->right_feedforward_high_slope_q10 = 480U;
    config->feedforward_knee_mm_s = 100U;
    /* Two unloaded -100 mm/s runs showed the left reverse direction about
       5-6 percent slow.  Directional trim avoids disturbing forward tuning. */
    config->left_reverse_boost_permille = 6U;
    config->right_reverse_boost_permille = 0U;
}

bool motion_config_validate(const motion_config_t *config)
{
    uint8_t pins[8];
    size_t i;
    size_t j;

    if (config == NULL || config->max_speed_mm_s <= 0 ||
        config->encoder_physical_max_speed_mm_s < config->max_speed_mm_s ||
        config->max_pwm_permille > 1000U || config->deadband_permille > 1000U ||
        config->deadband_permille > config->max_pwm_permille ||
        config->startup_pwm_permille > config->max_pwm_permille ||
        config->left_feedforward_offset_permille > config->max_pwm_permille ||
        config->right_feedforward_offset_permille > config->max_pwm_permille ||
        config->left_feedforward_slope_q10 == 0U ||
        config->right_feedforward_slope_q10 == 0U ||
        config->left_feedforward_high_slope_q10 == 0U ||
        config->right_feedforward_high_slope_q10 == 0U ||
        config->feedforward_knee_mm_s == 0U ||
        config->feedforward_knee_mm_s > (uint16_t)config->max_speed_mm_s ||
        config->left_reverse_boost_permille > config->max_pwm_permille ||
        config->right_reverse_boost_permille > config->max_pwm_permille ||
        config->left_counts_per_wheel_rev <= 0 || config->right_counts_per_wheel_rev <= 0 ||
        config->left_wheel_diameter_um <= 0 || config->right_wheel_diameter_um <= 0 ||
        config->track_width_um <= 0) {
        return false;
    }
    pins[0] = config->left_motor_a;
    pins[1] = config->left_motor_b;
    pins[2] = config->right_motor_a;
    pins[3] = config->right_motor_b;
    pins[4] = config->left_encoder_a;
    pins[5] = config->left_encoder_b;
    pins[6] = config->right_encoder_a;
    pins[7] = config->right_encoder_b;
    for (i = 0U; i < 8U; ++i) {
        if (pins[i] >= 30U) {
            return false;
        }
        for (j = i + 1U; j < 8U; ++j) {
            if (pins[i] == pins[j]) {
                return false;
            }
        }
    }
    return true;
}
