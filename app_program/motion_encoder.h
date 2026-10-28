#ifndef BUDDY2_MOTION_ENCODER_H
#define BUDDY2_MOTION_ENCODER_H

#include <stdint.h>

#include "motion_config.h"
#include "motion_types.h"

typedef struct {
    int32_t left_counts;
    int32_t right_counts;
    uint32_t left_invalid_transitions;
    uint32_t right_invalid_transitions;
    uint32_t invalid_transitions;
    uint32_t irq_count;
    /* Counts and timestamp belong to the same sequence-validated snapshot. */
    uint64_t sampled_at_us;
} motion_encoder_snapshot_t;

motion_result_t motion_encoder_init(const motion_config_t *config);
void motion_encoder_get_snapshot(motion_encoder_snapshot_t *snapshot);
void motion_encoder_get_levels(uint8_t *left_level, uint8_t *right_level);
motion_result_t motion_encoder_reset(void);

extern volatile uint32_t motion_encoder_irq_count;

#endif
