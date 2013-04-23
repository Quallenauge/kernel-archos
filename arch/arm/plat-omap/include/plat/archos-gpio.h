#ifndef _ARCH_ARCHOS_GPIO_H_
#define _ARCH_ARCHOS_GPIO_H_

#include <mach/gpio.h>

extern int __init omap_mux_init_gpio(int gpio, int val);

#define GPIO_EXISTS(nb) ( nb >= 0 )
#define UNUSED_GPIO -1

#define PIN_INPUT			(1 << 8)
#define PIN_OUTPUT			0

static inline void archos_gpio_init_output(const int pin, const char* label)
{
	if (pin < 0)
		return;

	if (gpio_request(pin, label) < 0) {
		pr_debug("archos_gpio_init_output:"
				" cannot acquire GPIO%d for %s\n", pin, label);
		return;
	}
	omap_mux_init_gpio(pin, PIN_OUTPUT|PIN_INPUT);
	gpio_direction_output(pin, 0);
}
#define GPIO_INIT_OUTPUT(x) archos_gpio_init_output(&x, NULL)

static inline void archos_gpio_init_input(const int pin, const char* label)
{
	if (pin < 0)
		return;

	if (gpio_request(pin, label) < 0) {
		pr_debug("archos_gpio_init_input:"
				" cannot acquire GPIO%d for %s\n", pin, label);
		return;
	}
	omap_mux_init_gpio(pin, PIN_INPUT);
	gpio_direction_input(pin);
}
#define GPIO_INIT_INPUT(x) archos_gpio_init_input(&x, NULL)

#endif
