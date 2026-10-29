#include <stdint.h>
#define TIMER_TIMELR 0U
#define TIMER_TIMEHR 1U
#define TIMER_TIMERAWL 2U
#define TIMER_TIMERAWH 3U
uint32_t in_w(uint32_t address);
#define BOARD_LED_PIN 19U
#define GPIO(pin) (100U + (pin))
#define GPIO_IE 1U
#define GPIO_PUE 2U
#define GPIO_SHEMITT 4U
void out_w(uint32_t address, uint32_t value);
#define IO_BANK0_BASE 0x40014000U
#define GPIO_IN 0xD0000004U
void set_w(uint32_t address, uint32_t value);
