#include "motion_encoder.h"

#include <sys/machine.h>
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <bsp/libbsp.h>
#include <sys/sysdepend/cpu/rp2040/sysdef.h>

#include "motion_control.h"

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
EXPORT volatile uint32_t motion_encoder_irq_count;
#ifdef BOARD_ROBO_PICO_ROBOT
static motion_config_t encoder_config;
static volatile uint8_t encoder_previous[2];
static bool encoder_ready;
static uint32_t encoder_proc_inte;
static uint32_t encoder_irq_masks[GPIO_IRQ_BANK_COUNT];

static uint8_t read_pair(uint8_t pin_a, uint8_t pin_b)
{
    uint32_t input = in_w(GPIO_IN);
    return (uint8_t)(((input >> pin_a) & 1U) | (((input >> pin_b) & 1U) << 1U));
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
    if (!encoder_pending) {
        ClearInt(intno);
        return;
    }
    ++motion_encoder_irq_count;
    current = read_pair(encoder_config.left_encoder_a, encoder_config.left_encoder_b);
    delta = motion_quadrature_delta(encoder_previous[0], current);
    if (delta == 0 && current != encoder_previous[0]) ++encoder_invalid[0];
    else encoder_counts[0] += delta;
    encoder_previous[0] = current;
    current = read_pair(encoder_config.right_encoder_a, encoder_config.right_encoder_b);
    delta = motion_quadrature_delta(encoder_previous[1], current);
    if (delta == 0 && current != encoder_previous[1]) ++encoder_invalid[1];
    else encoder_counts[1] += delta;
    encoder_previous[1] = current;
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
    encoder_config = *config;
    encoder_counts[0] = 0;
    encoder_counts[1] = 0;
    encoder_invalid[0] = 0U;
    encoder_invalid[1] = 0U;
    motion_encoder_irq_count = 0U;
    for (bank = 0U; bank < GPIO_IRQ_BANK_COUNT; ++bank) {
        encoder_irq_masks[bank] = 0U;
    }
    gpio_set_pin(encoder_config.left_encoder_a, GPIO_MODE_IN);
    gpio_set_pin(encoder_config.left_encoder_b, GPIO_MODE_IN);
    gpio_set_pin(encoder_config.right_encoder_a, GPIO_MODE_IN);
    gpio_set_pin(encoder_config.right_encoder_b, GPIO_MODE_IN);
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
    for (bank = 0U; bank < GPIO_IRQ_BANK_COUNT; ++bank) {
        if (encoder_irq_masks[bank] != 0U) {
            /* Clear stale edge flags, then enable without disturbing GPIO
             * interrupt users in another pin group. */
            out_w(GPIO_INTR_BASE + (bank * 4U), encoder_irq_masks[bank]);
            set_w(encoder_proc_inte + (bank * 4U), encoder_irq_masks[bank]);
        }
    }
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)motion_encoder_isr;
    if (tk_def_int(MOTION_GPIO_IRQ, &dint) < E_OK) return MOTION_RESULT_HARDWARE;
    ClearInt(MOTION_GPIO_IRQ);
    EnableInt(MOTION_GPIO_IRQ, 1);
    encoder_ready = true;
    return MOTION_RESULT_OK;
#endif
}

void motion_encoder_get_snapshot(motion_encoder_snapshot_t *snapshot)
{
    UINT intsts;
    if (snapshot == 0) return;
    DI(intsts);
    snapshot->left_counts = encoder_counts[0];
    snapshot->right_counts = encoder_counts[1];
    snapshot->left_invalid_transitions = encoder_invalid[0];
    snapshot->right_invalid_transitions = encoder_invalid[1];
    snapshot->invalid_transitions = encoder_invalid[0] + encoder_invalid[1];
    snapshot->irq_count = motion_encoder_irq_count;
    EI(intsts);
}

void motion_encoder_get_levels(uint8_t *left_level, uint8_t *right_level)
{
    UINT intsts;
    if (left_level == 0 || right_level == 0) return;
#ifdef BOARD_ROBO_PICO_ROBOT
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

void motion_encoder_reset(void)
{
    UINT intsts;
    DI(intsts);
    encoder_counts[0] = 0;
    encoder_counts[1] = 0;
    encoder_invalid[0] = 0U;
    encoder_invalid[1] = 0U;
    motion_encoder_irq_count = 0U;
#ifdef BOARD_ROBO_PICO_ROBOT
    encoder_previous[0] = read_pair(encoder_config.left_encoder_a,
                                    encoder_config.left_encoder_b);
    encoder_previous[1] = read_pair(encoder_config.right_encoder_a,
                                    encoder_config.right_encoder_b);
#endif
    EI(intsts);
}
