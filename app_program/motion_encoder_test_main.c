#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "motion_config.h"
#include "motion_encoder.h"

EXPORT INT usermain(void)
{
    motion_config_t config;
    motion_encoder_snapshot_t snapshot;
    uint8_t left_level;
    uint8_t right_level;
    motion_config_default(&config);
    tm_printf((UB *)"[motion-encoder] init=%d prc=%d (GP4/5 left, GP6/7 right; verify voltage first)\n",
              motion_encoder_init(&config), tk_get_prc());
    for (;;) {
        motion_encoder_get_snapshot(&snapshot);
        motion_encoder_get_levels(&left_level, &right_level);
        tm_printf((UB *)"[motion-encoder] L=%d R=%d invalid=%u/%u total=%u raw=%u/%u irq=%u\n",
                  snapshot.left_counts, snapshot.right_counts,
                  snapshot.left_invalid_transitions,
                  snapshot.right_invalid_transitions,
                  snapshot.invalid_transitions, left_level, right_level,
                  snapshot.irq_count);
        tk_dly_tsk(100);
    }
    return 0;
}
