/* Exercise the actual board ISR with simulated GPIO registers. These checks
 * distinguish duplicate IRQ delivery from genuine sampled phase changes;
 * they cannot establish whether physical transitions are noise or rotation. */
#include <stdio.h>
#include <stdlib.h>
#define BOARD_ROBO_PICO_ROBOT 1
#define TK_SUPPORT_SMP 1
#include "../../app_program/motion_encoder.c"

static uint32_t input_levels;
static uint32_t pending_edges[4];
static uint32_t enabled_edges[4];
static bool fail_gpio_init;
static bool handler_defined;
static bool edges_enabled_before_handler;
static uint32_t fake_time_us;

uint32_t in_w(uint32_t address)
{
    /* The ISR's read_stable() waits a few us: let the timer advance. */
    if (address == TIMER_TIMERAWL) return fake_time_us++;
    if (address == GPIO_IN) return input_levels;
    if (address >= GPIO_INTR_BASE && address < GPIO_INTR_BASE + 16U) {
        return pending_edges[(address - GPIO_INTR_BASE) / 4U];
    }
    return 0U;
}

void out_w(uint32_t address, uint32_t value)
{
    if (address >= GPIO_INTR_BASE && address < GPIO_INTR_BASE + 16U) {
        pending_edges[(address - GPIO_INTR_BASE) / 4U] &= ~value;
    }
}

void set_w(uint32_t address, uint32_t value)
{
    if (!handler_defined) {
        edges_enabled_before_handler = true;
    }
    if (address >= GPIO_PROC0_INTE && address < GPIO_PROC0_INTE + 16U) {
        enabled_edges[(address - GPIO_PROC0_INTE) / 4U] |= value;
    }
}

INT tk_get_prc(void) { return 1; }
ER tk_def_int(UINT intno, const T_DINT *definition)
{
    (void)intno;
    (void)definition;
    handler_defined = true;
    return E_OK;
}
ER gpio_set_pin(UINT pin, UINT mode)
{
    (void)pin;
    (void)mode;
    return fail_gpio_init ? -1 : E_OK;
}
void ClearInt(UINT intno) { (void)intno; }
void EnableInt(UINT intno, INT level) { (void)intno; (void)level; }

static void check(bool ok, const char *name)
{
    if (!ok) { fprintf(stderr, "FAIL: %s\n", name); exit(1); }
    printf("PASS: %s\n", name);
}

static void set_left(uint8_t state)
{
    uint32_t next = ((uint32_t)(state & 1U) << 17U)
                    | ((uint32_t)((state >> 1U) & 1U) << 16U);
    uint32_t changed = input_levels ^ next;
    uint8_t pin;
    for (pin = 16U; pin <= 17U; ++pin) {
        if ((changed & (1UL << pin)) != 0U) {
            uint32_t edge = (next & (1UL << pin)) != 0U ? 8U : 4U;
            pending_edges[2] |= edge << ((pin & 7U) * 4U);
        }
    }
    input_levels = next;
    motion_encoder_isr(MOTION_GPIO_IRQ);
}

int main(void)
{
    motion_config_t config;
    unsigned index;
    static const uint8_t forward[] = {2U, 3U, 1U, 0U};
    static const uint8_t reverse[] = {1U, 3U, 2U, 0U};
    motion_config_default(&config);
    fail_gpio_init = true;
    check(motion_encoder_init(&config) == MOTION_RESULT_HARDWARE &&
          enabled_edges[0] == 0U && enabled_edges[2] == 0U,
          "encoder GPIO failure is reported before IRQs are armed");
    fail_gpio_init = false;
    check(motion_encoder_init(&config) == MOTION_RESULT_OK &&
          enabled_edges[0] == 0x00CC0000U && enabled_edges[2] == 0xCCU &&
          enabled_edges[1] == 0U && enabled_edges[3] == 0U &&
          !edges_enabled_before_handler,
          "encoder IRQ masks cover only GP4/5 and GP16/17");
    for (index = 0U; index < 1000U; ++index) {
        pending_edges[2] = 0xCCU;
        motion_encoder_isr(MOTION_GPIO_IRQ);
    }
    check(encoder_counts[0] == 0 && encoder_counts[1] == 0 &&
          encoder_invalid[0] == 0U,
          "1000 pending-edge IRQs with unchanged levels cannot accumulate counts");
    (void)motion_encoder_reset();
    for (index = 0U; index < 600U; ++index) set_left((index & 1U) ? 0U : 1U);
    check(encoder_counts[0] == 0 && encoder_invalid[0] == 0U,
          "single-phase back-and-forth chatter cancels instead of adding 599 counts");
    (void)motion_encoder_reset();
    for (index = 0U; index < 599U; ++index) set_left(forward[index % 4U]);
    check(encoder_counts[0] == 599 && encoder_counts[1] == 0 &&
          motion_encoder_irq_count == 599U && encoder_invalid[0] == 0U,
          "599 forward phase changes reproduce the reported count and IRQ deltas");
    set_left(0U);
    (void)motion_encoder_reset();
    for (index = 0U; index < 100U; ++index) set_left(reverse[index % 4U]);
    check(encoder_counts[0] == -100 && encoder_counts[1] == 0,
          "reverse quadrature decrements only the left encoder");
    (void)motion_encoder_reset();
    set_left(3U);
    check(encoder_counts[0] == 0 && encoder_invalid[0] == 1U,
          "simultaneous two-bit transition is invalid rather than extra travel");
    pending_edges[2] = 0xC00U; /* GP18 is not an encoder pin. */
    motion_encoder_isr(MOTION_GPIO_IRQ);
    check(pending_edges[2] == 0xC00U && motion_encoder_irq_count == 1U,
          "unowned GPIO edge is neither acknowledged nor counted");
    return 0;
}
