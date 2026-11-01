#ifndef BUDDY2_MOTION_CLOCK_H
#define BUDDY2_MOTION_CLOCK_H

#include <stdint.h>
#include <sys/machine.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>

/* Use the same raw high/low/high algorithm as Pico SDK time_us_64().
 * Unlike TIMELR/TIMEHR, these reads do not share a latch with other cores
 * or interrupt handlers. Retry only if the low word rolled over. */
static inline uint64_t motion_clock_us(void)
{
    uint32_t high = in_w(TIMER_TIMERAWH);
    uint32_t low;
    uint32_t next_high;

    for (;;) {
        low = in_w(TIMER_TIMERAWL);
        next_high = in_w(TIMER_TIMERAWH);
        if (high == next_high) {
            return ((uint64_t)high << 32) | low;
        }
        high = next_high;
    }
}

#endif
