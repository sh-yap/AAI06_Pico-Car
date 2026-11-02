#ifndef BUDDY2_MOTION_CONTROL_H
#define BUDDY2_MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "motion_types.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float integral_limit;
    float output_min;
    float output_max;
} motion_pid_t;

typedef struct {
    float output;
    float error;
} motion_pid_result_t;

int8_t motion_quadrature_delta(uint8_t previous, uint8_t current);
int32_t motion_counts_delta(uint32_t previous, uint32_t current);
bool motion_speed_from_counts(int32_t count_delta, uint32_t elapsed_ms,
                              int32_t mm_per_count_q16, int32_t *speed_mm_s);
int32_t motion_counts_to_mm(int32_t counts, int32_t mm_per_count_q16);
int32_t motion_relative_mm(int32_t counts, int32_t origin,
                           int32_t mm_per_count_q16);
int32_t motion_average(int32_t left, int32_t right);
int32_t motion_turn_wheel_travel_mm(int32_t track_width_mm, int32_t angle_mdeg);
bool motion_target_reached(int32_t target_mm, int32_t travelled_mm,
                           bool b_turn);
bool motion_velocity_targets(int32_t linear_mm_s, int32_t angular_mdeg_s,
                             int32_t track_width_mm, int32_t maximum_mm_s,
                             int32_t *left_mm_s, int32_t *right_mm_s);
bool motion_turn_targets(int32_t angle_mdeg, int32_t wheel_speed_mm_s,
                         int32_t *left_mm_s, int32_t *right_mm_s);
bool motion_velocity_requires_reset(bool b_active,
                                    motion_command_type_t active_type,
                                    int32_t previous_linear_mm_s,
                                    int32_t next_linear_mm_s);
bool motion_command_is_valid(const motion_command_t *command,
                             int32_t maximum_mm_s,
                             int32_t track_width_mm,
                             int32_t maximum_turn_angle_mdeg);
bool motion_encoder_delta_plausible(int32_t count_delta,
                                    uint32_t elapsed_ms,
                                    int32_t mm_per_count_q16,
                                    int32_t maximum_mm_s);
int32_t motion_feedforward_permille(int32_t target_mm_s,
                                   uint16_t offset_permille,
                                   uint16_t slope_q10,
                                   uint16_t high_speed_slope_q10,
                                   uint16_t knee_mm_s,
                                   uint16_t reverse_boost_permille,
                                   uint16_t deadband_permille,
                                   uint16_t maximum_permille);
motion_pid_result_t motion_pid_step(motion_pid_t *pid, float target,
                                    float measured, float dt_s);
float motion_slew_target(float current, float target, float max_delta);
void motion_slew_split(float applied_left, float applied_right,
                       float want_left, float want_right,
                       float max_delta, float max_output,
                       float *out_left, float *out_right);

/* ---- Response learner ------------------------------------------------
   Motion learns from its own encoders and applied power how the car answers
   to PWM, so the feedforward needs no per-course fit.  Two straight-line
   models, each fitted with exponentially forgotten least squares:
     turning  speed difference (L - R) = turn_gain * PWM difference + c
     forward  mean speed (L + R) / 2   = fwd_gain  * mean PWM + fwd_offset
   Gains are mm/s per permille, the offset is mm/s.  The newest speed is
   paired with the PWM applied MOTION_LEARN_DELAY_CYCLES earlier. */

/* Response delay in 20 ms Motion cycles.  Fitting the 2 Nov recorder runs
   (ramp2, pivot1) at each delay, the forward fit peaked at 4 to 5 cycles
   and the turning fit at 2 to 4 (ramp2) and 4 to 5 (pivot1).  3 cycles is
   within 0.05 of the best R-squared in every case.  The physical parts
   are the motor and wheel lag plus the 20 ms speed window, which looks
   back by half a period. */
#define MOTION_LEARN_DELAY_CYCLES 3U
#define MOTION_LEARN_DELAY_MAX 6U
#define MOTION_LEARN_HISTORY 8U              /* Power of two, > delay max. */

/* Forgetting factor per informative cycle: memory 1 / (1 - 0.97) = 33
   cycles = 0.67 s.  A surface, ramp or battery change then owns 78 percent
   of the weight after 50 cycles (1 s).  A longer memory follows too
   slowly, a shorter one lets one-count encoder steps (about 5 mm/s) move
   the estimate. */
#define MOTION_LEARN_FORGET 0.97f

/* Plausibility bounds.  An estimate outside them is noise or a model
   failure, and the seed is used instead. */
/* Turning gain.  Upper: today's feedforward line gives 2.0 mm/s per
   permille, and scrubbing a turn can only lower the response from that.
   Lower: below 0.1 a 100 mm/s wheel difference would need 1000 permille,
   more than the 900 permille limit. */
#define MOTION_LEARN_TURN_GAIN_MIN 0.1f
#define MOTION_LEARN_TURN_GAIN_MAX 2.0f
/* Forward gain.  Upper: the unloaded motors (slope 0.31 permille per mm/s
   in motion_config.c) give 3.2, and load only lowers it.  Lower: 0.2 would
   give only 180 mm/s at 900 permille, well under the 300 mm/s limit. */
#define MOTION_LEARN_FWD_GAIN_MIN 0.2f
#define MOTION_LEARN_FWD_GAIN_MAX 3.2f
/* Power at which the wheel starts to move (-offset / gain).  It cannot be
   negative, and it must be under the 200 permille startup boost that
   starts the car reliably. */
#define MOTION_LEARN_FWD_ZERO_PWM_MAX 200.0f

/* A fit is trusted when its standard error is under a quarter of the
   gain (one sigma), and it holds at least half its memory of data.  With
   too little spread in power (a long straight) the last value is held. */
#define MOTION_LEARN_SE_FRACTION_INV 4.0f

/* Pivot samples (opposite wheel directions, wheels rolling) needed before
   the pivot seeds the turning gain: 5 cycles = 100 ms, half of the
   steady speed the START sweep gives each way. */
#define MOTION_LEARN_PIVOT_MIN_SAMPLES 5U

typedef enum {
    MOTION_LEARN_NONE = 0,    /* Cycle carries no information.            */
    MOTION_LEARN_DRIVE,       /* Forward velocity command, unclamped.     */
    MOTION_LEARN_PIVOT        /* Turn command, wheels rolling opposite.   */
} motion_learn_mode_t;

typedef enum {
    MOTION_LEARN_SRC_DEFAULT = 0,   /* Today's feedforward line.          */
    MOTION_LEARN_SRC_PIVOT,         /* Seeded from TURN_ANGLE pivots.     */
    MOTION_LEARN_SRC_DRIVING        /* Estimated from driving data.       */
} motion_learn_source_t;

typedef struct {
    float n;                  /* Forgotten sums of 1, x, y, x*x, x*y, y*y. */
    float sx;
    float sy;
    float sxx;
    float sxy;
    float syy;
} motion_fit_t;

typedef struct {
    motion_fit_t turn_fit;
    motion_fit_t fwd_fit;
    float pivot_xx;           /* Pivot sums, line through the origin.     */
    float pivot_xy;
    uint8_t pivot_samples;
    float turn_gain;          /* In use.                                  */
    float turn_seed;          /* Fallback: pivot seed, else the default.  */
    float fwd_gain;           /* In use.                                  */
    float fwd_offset;
    float fwd_seed_gain;
    float fwd_seed_offset;
    motion_learn_source_t turn_source;
    motion_learn_source_t turn_seed_source;
    bool b_fwd_from_driving;
    motion_learn_mode_t mode;
    uint8_t run;              /* Consecutive informative cycles recorded. */
    uint8_t delay;
    uint8_t head;
    int16_t pwm_left[MOTION_LEARN_HISTORY];
    int16_t pwm_right[MOTION_LEARN_HISTORY];
} motion_learn_t;

/* Seeds: the forward line (gain mm/s per permille, offset mm/s) and the
   turning gain used until a pivot or driving data replaces it. */
void motion_learn_init(motion_learn_t *learn, uint8_t delay_cycles,
                       float fwd_seed_gain, float fwd_seed_offset,
                       float turn_default_gain);
/* Forget the recent power history (command change, stop). */
void motion_learn_flush(motion_learn_t *learn);
/* Call once per cycle with the newest measured speeds, before record. */
void motion_learn_observe(motion_learn_t *learn, int32_t left_speed,
                          int32_t right_speed);
/* Call once per cycle with the signed power actually applied and what the
   cycle can teach. */
void motion_learn_record(motion_learn_t *learn, motion_learn_mode_t mode,
                         int32_t left_pwm, int32_t right_pwm);
/* Replace the feedforward of each forward wheel by the learned one.
   Changes nothing unless the mean target is forward. */
void motion_learn_feedforward(const motion_learn_t *learn,
                              float left_target, float right_target,
                              float deadband_permille, float maximum_permille,
                              float *left_ff, float *right_ff);

#endif
