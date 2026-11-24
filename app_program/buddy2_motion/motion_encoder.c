#include "motion_encoder.h"

#include <sys/machine.h>
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <bsp/libbsp.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>

#include "motion_control.h"
#include "motion_clock.h"

#define MOTION_GPIO_IRQ 13U /* RP2040 IO_IRQ_BANK0, owned by Buddy 2 on core 0. */
#define GPIO_INTR_BASE (IO_BANK0_BASE + 0xF0U)
#define GPIO_PROC0_INTE (IO_BANK0_BASE + 0x100U)
#define GPIO_PROC1_INTE (IO_BANK0_BASE + 0x130U)
#define GPIO_PROC0_INTF (IO_BANK0_BASE + 0x110U)
#define GPIO_PROC0_INTS (IO_BANK0_BASE + 0x120U)
#define GPIO_EDGE_MASK 0xCU
#define GPIO_IRQ_BANK_COUNT 4U

static volatile int32_t encoder_counts[2];
static volatile uint32_t encoder_invalid[2];
static volatile uint32_t encoder_snapshot_sequence;
EXPORT volatile uint32_t motion_encoder_irq_count;

static void encoder_memory_barrier(void)
{
    /* GCC emits DMB on RP2040; host regressions use the host's fence. */
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}
#ifdef BOARD_ROBO_PICO_ROBOT
static motion_config_t encoder_config;
static volatile uint8_t encoder_previous[2];
static bool encoder_ready;
static INT encoder_owner_processor;
static uint32_t encoder_proc_inte;
static uint32_t encoder_irq_masks[GPIO_IRQ_BANK_COUNT];

static uint8_t read_pair(uint8_t pin_a, uint8_t pin_b)
{
    uint32_t input = in_w(GPIO_IN);
    return (uint8_t)(((input >> pin_a) & 1U) | (((input >> pin_b) & 1U) << 1U));
}

/* test_integrate (Buddy 2 to review): on powerbank/VIN the encoders saw
 * >1000-count bursts in ~10 ms (motor noise; fine on computer USB).  At the
 * 300 mm/s limit a real edge comes every ~270 us and then holds, so a level
 * is only trusted if it reads the same twice this far apart. */
#define ENCODER_STABLE_US 3U

/* Both encoder pairs, read twice ENCODER_STABLE_US apart.  Returns false
 * if either pair changed between the reads (a noise spike in progress). */
static bool read_stable(uint8_t *p_left, uint8_t *p_right)
{
    uint64_t const start = motion_clock_us();
    uint8_t left = read_pair(encoder_config.left_encoder_a,
                             encoder_config.left_encoder_b);
    uint8_t right = read_pair(encoder_config.right_encoder_a,
                              encoder_config.right_encoder_b);

    while ((motion_clock_us() - start) < ENCODER_STABLE_US) {
        /* Short busy wait inside the ISR: a few microseconds per edge. */
    }
    *p_left = read_pair(encoder_config.left_encoder_a,
                        encoder_config.left_encoder_b);
    *p_right = read_pair(encoder_config.right_encoder_a,
                         encoder_config.right_encoder_b);
    return (left == *p_left) && (right == *p_right);
}

static void add_irq_pin(uint8_t pin)
{
    uint32_t bank = pin / 8U;
    encoder_irq_masks[bank] |= GPIO_EDGE_MASK << ((pin & 7U) * 4U);
}
#endif

#ifdef BOARD_ROBO_PICO_ROBOT
static void motion_encoder_isr(UINT intno)
{
    uint8_t current;
    uint8_t left_now;
    uint8_t right_now;
    int8_t delta;
    uint32_t bank;
    uint32_t pending;
    bool encoder_pending = false;
    if (!encoder_ready) return;
    for (bank = 0U; bank < GPIO_IRQ_BANK_COUNT; ++bank) {
        uint32_t intr_addr = GPIO_INTR_BASE + (bank * 4U);
        pending = in_w(intr_addr) & encoder_irq_masks[bank];
        /* Acknowledge only Buddy 2's GPIO edge bits.  The encoders may be
         * split across different groups of eight GPIOs. */
        if (pending != 0U) {
            out_w(intr_addr, pending);
            encoder_pending = true;
        }
    }
    if (!encoder_pending || !read_stable(&left_now, &right_now)) {
        /* Nothing for us, or a noise spike: count nothing.  A real edge
         * holds, and the next interrupt reads it. */
        ClearInt(intno);
        return;
    }
    ++encoder_snapshot_sequence;
    encoder_memory_barrier();
    ++motion_encoder_irq_count;
    current = left_now;
    delta = motion_quadrature_delta(encoder_previous[0], current);
    if (delta == 0 && current != encoder_previous[0]) ++encoder_invalid[0];
    else encoder_counts[0] += delta;
    encoder_previous[0] = current;
    current = right_now;
    delta = motion_quadrature_delta(encoder_previous[1], current);
    if (delta == 0 && current != encoder_previous[1]) ++encoder_invalid[1];
    else encoder_counts[1] += delta;
    encoder_previous[1] = current;
    encoder_memory_barrier();
    ++encoder_snapshot_sequence;
    ClearInt(intno);
}
#endif

motion_result_t motion_encoder_init(const motion_config_t *config)
{
    if (config == 0 || !motion_config_validate(config)) return MOTION_RESULT_HARDWARE;
#ifndef BOARD_ROBO_PICO_ROBOT
    (void)config;
    return MOTION_RESULT_HARDWARE;
#else
    T_DINT dint;
    uint32_t bank;
    encoder_ready = false;
    encoder_config = *config;
    encoder_owner_processor = tk_get_prc();
    encoder_counts[0] = 0;
    encoder_counts[1] = 0;
    encoder_invalid[0] = 0U;
    encoder_invalid[1] = 0U;
    motion_encoder_irq_count = 0U;
    encoder_snapshot_sequence = 0U;
    for (bank = 0U; bank < GPIO_IRQ_BANK_COUNT; ++bank) {
        encoder_irq_masks[bank] = 0U;
    }
    if (gpio_set_pin(encoder_config.left_encoder_a, GPIO_MODE_IN) < E_OK ||
        gpio_set_pin(encoder_config.left_encoder_b, GPIO_MODE_IN) < E_OK ||
        gpio_set_pin(encoder_config.right_encoder_a, GPIO_MODE_IN) < E_OK ||
        gpio_set_pin(encoder_config.right_encoder_b, GPIO_MODE_IN) < E_OK) {
        return MOTION_RESULT_HARDWARE;
    }
    /* Bias disconnected inputs so the encoder test cannot create an IRQ
     * storm while the motor harness is still unplugged.  The pull-up is a
     * provisional electrical default and must be checked against the encoder
     * datasheet before powered vehicle testing. */
    out_w(GPIO(encoder_config.left_encoder_a), GPIO_IE | GPIO_PUE | GPIO_SHEMITT);
    out_w(GPIO(encoder_config.left_encoder_b), GPIO_IE | GPIO_PUE | GPIO_SHEMITT);
    out_w(GPIO(encoder_config.right_encoder_a), GPIO_IE | GPIO_PUE | GPIO_SHEMITT);
    out_w(GPIO(encoder_config.right_encoder_b), GPIO_IE | GPIO_PUE | GPIO_SHEMITT);
    encoder_previous[0] = read_pair(encoder_config.left_encoder_a, encoder_config.left_encoder_b);
    encoder_previous[1] = read_pair(encoder_config.right_encoder_a, encoder_config.right_encoder_b);
    add_irq_pin(encoder_config.left_encoder_a);
    add_irq_pin(encoder_config.left_encoder_b);
    add_irq_pin(encoder_config.right_encoder_a);
    add_irq_pin(encoder_config.right_encoder_b);
    encoder_proc_inte = GPIO_PROC0_INTE;
#if TK_SUPPORT_SMP
    if (tk_get_prc() == 2) {
        encoder_proc_inte = GPIO_PROC1_INTE;
    }
#endif
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)motion_encoder_isr;
    if (tk_def_int(MOTION_GPIO_IRQ, &dint) < E_OK) return MOTION_RESULT_HARDWARE;
    ClearInt(MOTION_GPIO_IRQ);
    /* Install the handler and make its state valid before an encoder GPIO can
     * reach it. Enabling GPIO edges before encoder_ready left a small window
     * where the old handler returned without acknowledging the edge. */
    encoder_ready = true;
    for (bank = 0U; bank < GPIO_IRQ_BANK_COUNT; ++bank) {
        if (encoder_irq_masks[bank] != 0U) {
            /* Clear stale edge flags, then enable without disturbing GPIO
             * interrupt users in another pin group. */
            out_w(GPIO_INTR_BASE + (bank * 4U), encoder_irq_masks[bank]);
            set_w(encoder_proc_inte + (bank * 4U), encoder_irq_masks[bank]);
        }
    }
    EnableInt(MOTION_GPIO_IRQ, 1);
    return MOTION_RESULT_OK;
#endif
}

void motion_encoder_get_snapshot(motion_encoder_snapshot_t *snapshot)
{
    uint32_t sequence_start;
    uint32_t sequence_end;

    if (snapshot == 0) return;
    do {
        sequence_start = encoder_snapshot_sequence;
        encoder_memory_barrier();
        if ((sequence_start & 1U) == 0U) {
            snapshot->left_counts = encoder_counts[0];
            snapshot->right_counts = encoder_counts[1];
            snapshot->left_invalid_transitions = encoder_invalid[0];
            snapshot->right_invalid_transitions = encoder_invalid[1];
            snapshot->invalid_transitions = encoder_invalid[0] + encoder_invalid[1];
            snapshot->irq_count = motion_encoder_irq_count;
            /* An ISR anywhere between the initial sequence read and this
             * timestamp forces a retry of both counts AND time. */
            snapshot->sampled_at_us = motion_clock_us();
            encoder_memory_barrier();
            sequence_end = encoder_snapshot_sequence;
        } else {
            sequence_end = sequence_start;
        }
    } while (sequence_start != sequence_end
             || (sequence_end & 1U) != 0U);
}

void motion_encoder_get_levels(uint8_t *left_level, uint8_t *right_level)
{
    if (left_level == 0 || right_level == 0) return;
#ifdef BOARD_ROBO_PICO_ROBOT
    UINT intsts;

    DI(intsts);
    *left_level = read_pair(encoder_config.left_encoder_a,
                            encoder_config.left_encoder_b);
    *right_level = read_pair(encoder_config.right_encoder_a,
                             encoder_config.right_encoder_b);
    EI(intsts);
#else
    *left_level = 0U;
    *right_level = 0U;
#endif
}

motion_result_t motion_encoder_reset(void)
{
#ifdef BOARD_ROBO_PICO_ROBOT
    UINT intsts;

    /* DI() masks interrupts only on the calling processor. Refuse a reset
       from another processor so it cannot race the encoder ISR. */
    if (!encoder_ready || tk_get_prc() != encoder_owner_processor) {
        return MOTION_RESULT_INVALID;
    }
    DI(intsts);
    ++encoder_snapshot_sequence;
    encoder_memory_barrier();
    encoder_counts[0] = 0;
    encoder_counts[1] = 0;
    encoder_invalid[0] = 0U;
    encoder_invalid[1] = 0U;
    motion_encoder_irq_count = 0U;
    encoder_previous[0] = read_pair(encoder_config.left_encoder_a,
                                    encoder_config.left_encoder_b);
    encoder_previous[1] = read_pair(encoder_config.right_encoder_a,
                                    encoder_config.right_encoder_b);
    encoder_memory_barrier();
    ++encoder_snapshot_sequence;
    EI(intsts);
    return MOTION_RESULT_OK;
#else
    return MOTION_RESULT_HARDWARE;
#endif
}
