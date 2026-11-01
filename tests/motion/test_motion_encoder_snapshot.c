/* Exercise the production sequence-protected reader with an ISR update
 * injected between its count copy and timestamp read. No hardware needed. */
#include <stdio.h>
#include <stdlib.h>
#include "../../app_program/motion_encoder.c"

static uint64_t test_clock;
static bool inject_encoder_update;
static bool inject_rollover;

uint32_t in_w(uint32_t address)
{
    if (inject_encoder_update) {
        inject_encoder_update = false;
        ++encoder_snapshot_sequence;
        encoder_counts[0] = 50;
        encoder_counts[1] = 60;
        motion_encoder_irq_count = 110U;
        test_clock = 120000U;
        ++encoder_snapshot_sequence;
    }
    if (address == TIMER_TIMERAWL) {
        if (inject_rollover) {
            inject_rollover = false;
            test_clock = 0x200000005ULL;
        }
        return (uint32_t)test_clock;
    }
    return (uint32_t)(test_clock >> 32);
}

static void check(bool ok, const char *name)
{
    if (!ok) { fprintf(stderr, "FAIL: %s\n", name); exit(1); }
    printf("PASS: %s\n", name);
}

int main(void)
{
    motion_encoder_snapshot_t snapshot;
    encoder_counts[0] = 10;
    encoder_counts[1] = 20;
    motion_encoder_irq_count = 30U;
    test_clock = 20000U;
    motion_encoder_get_snapshot(&snapshot);
    check(snapshot.left_counts == 10 && snapshot.right_counts == 20 &&
          snapshot.sampled_at_us == 20000U && snapshot.irq_count == 30U,
          "encoder snapshot pairs stable counts, IRQ and time");
    inject_encoder_update = true;
    motion_encoder_get_snapshot(&snapshot);
    check(snapshot.left_counts == 50 && snapshot.right_counts == 60 &&
          snapshot.sampled_at_us == 120000U && snapshot.irq_count == 110U,
          "encoder change during timestamp read retries the entire snapshot");
    test_clock = 0x1fffffffeULL;
    inject_rollover = true;
    check(motion_clock_us() == 0x200000005ULL,
          "raw timer reader retries across low-word rollover");
    motion_encoder_get_snapshot(&snapshot);
    check(snapshot.sampled_at_us == 0x200000005ULL && snapshot.left_counts == 50,
          "encoder snapshot preserves full 64-bit acquisition time");
    return 0;
}
