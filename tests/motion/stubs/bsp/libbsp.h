/* Encoder snapshot tests do not initialise GPIO or PWM. */
#include <tk/tkernel.h>
#define GPIO_MODE_IN 0U
#define GPIO_MODE_OUT 1U
ER gpio_set_pin(UINT pin, UINT mode);
ER gpio_set_val(UINT pin, UINT value);
UINT gpio_get_val(UINT pin);
ER pwm_set_pin(UINT pin);
ER pwm_set_wrap(UINT pin, UW value);
ER pwm_set_cc(UINT pin, UW value);
ER pwm_set_enabled(UINT pin, BOOL enabled);
