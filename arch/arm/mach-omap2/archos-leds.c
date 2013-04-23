/*
 * Led(s) Board configuration
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/stat.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/delay.h>

#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>
#include <plat/archos-gpio.h>
#include <asm/mach-types.h>
#include <plat/board.h>
#include <mach/board-archos.h>

#include "mux.h"

static struct gpio_led gpio_leds[3] = {
};

static struct gpio_led_platform_data board_led_data = {
	.leds		= gpio_leds,
	.num_leds	= 0,
};

static struct platform_device board_led_device = {
	.name		= "leds-gpio",
	.id		= 0,
	.dev            = {
		.platform_data = &board_led_data,
	},
};

#ifdef CONFIG_LEDS_OMAP_PWM
static struct omap_pwm_led_platform_data board_backlight_data = {
	.name			= "lcd-backlight",
	.default_trigger	= "backlight",
};

static int bkl_power_gpio;
struct archos_pwm_conf backlight_led_pwm;
static struct regulator *bkl_reg = NULL;
static struct omap_mux bkl_pad_mux;
static int bkl_pad_off_mode;
static int bkl_pad_on_mode;

static struct platform_device board_backlight_device = {
	.name		= "omap_pwm_led",
	.id		= 0,
	.dev.platform_data = &board_backlight_data,
};

static void bkl_set_pad(struct omap_pwm_led_platform_data *self, int on_off)
{
	if (on_off == 0) {
		if (self->invert)
			omap_mux_write(bkl_pad_mux.partition,
					bkl_pad_off_mode | OMAP_PIN_INPUT_PULLUP,
					bkl_pad_mux.reg_offset);
		else
			omap_mux_write(bkl_pad_mux.partition,
					bkl_pad_off_mode | OMAP_PIN_INPUT_PULLDOWN,
					bkl_pad_mux.reg_offset);

	} else
		omap_mux_write(bkl_pad_mux.partition,
				bkl_pad_on_mode | OMAP_PIN_OUTPUT,
				bkl_pad_mux.reg_offset);
}

static void bkl_set_power(struct omap_pwm_led_platform_data *self, int on_off)
{
	pr_debug("%s: on_off:%d\n", __func__, on_off);

	if (bkl_reg != NULL) {
		if (on_off)
			regulator_enable(bkl_reg);
		else
			regulator_disable(bkl_reg);
	}

	if (gpio_is_valid(bkl_power_gpio))
		gpio_set_value( bkl_power_gpio, on_off );

	// enable this fixed backlight startup for A100 on low level
	// but could generate a little white flash at start
	msleep(1);

}

static void bkl_set_power_initial(struct omap_pwm_led_platform_data *self)
{
	if (bkl_reg != NULL) {
		regulator_force_disable(bkl_reg);
	}

	if (gpio_is_valid(bkl_power_gpio))
		gpio_set_value( bkl_power_gpio, false );
}
#endif

static int __init archos_leds_init(void)
{
	const struct archos_leds_config *leds_cfg;
	const struct archos_leds_conf *cfg;
	int iled;
	int ret;

	pr_debug("archos_leds_init\n");

	leds_cfg = omap_get_config( ARCHOS_TAG_LEDS, struct archos_leds_config );
	cfg = hwrev_ptr(leds_cfg, system_rev);
	if (IS_ERR(cfg)) {
		pr_err("archos_leds_init: no configuration for hwrev %i\n",
				system_rev);
		return -ENODEV;
	}

	/* Power Led (GPIO) */
	if (gpio_is_valid(cfg->power_led)) {
		pr_debug("%s: power led on gpio %i\n", __func__, cfg->power_led);
		gpio_leds[board_led_data.num_leds].gpio = cfg->power_led;
		gpio_leds[board_led_data.num_leds].active_low = cfg->pwr_invert;
		gpio_leds[board_led_data.num_leds].name = "power";
		gpio_leds[board_led_data.num_leds].default_trigger = "default-on";
		omap_mux_init_gpio(cfg->power_led, PIN_OUTPUT);
		board_led_data.num_leds++;
	} else
		pr_debug("%s: no power led configured\n", __func__);

	/* Status Led (GPIO) */
	if (gpio_is_valid(cfg->status_led)) {
		pr_debug("%s: status led on gpio %i\n", __func__, cfg->status_led);
		gpio_leds[board_led_data.num_leds].gpio = cfg->status_led;
		gpio_leds[board_led_data.num_leds].name = "status";
		gpio_leds[board_led_data.num_leds].default_trigger = "default-on";
		omap_mux_init_gpio(cfg->status_led, PIN_OUTPUT);
		board_led_data.num_leds++;
	} else
		pr_debug("%s: no status led configured\n", __func__);

	if (board_led_data.num_leds) {
		ret = platform_device_register(&board_led_device);
		if (IS_ERR_VALUE(ret))
			pr_err("unable to register %d gpio LEDs\n",
					board_led_data.num_leds);
	}

	for (iled = 0; iled < board_led_data.num_leds; iled++) {
		/* FIXME: Use a generic way to setup off mode configuration */
		struct omap_mux *mux;
		u16 val;

		if (!gpio_leds[iled].active_low)
			continue;

		mux = omap_mux_get_gpio(gpio_leds[iled].gpio);
		if (mux->reg_offset != OMAP_MUX_TERMINATOR) {
			val = omap_mux_read(mux->partition, mux->reg_offset);
			val &= ~(OMAP_OFF_EN | OMAP_OFFOUT_EN \
					| OMAP_OFF_PULL_EN \
					| OMAP_OFF_PULL_UP);
			val |= OMAP_PIN_OFF_INPUT_PULLUP;
			omap_mux_set_gpio(val, gpio_leds[iled].gpio);
		}
	}

#ifdef CONFIG_LEDS_OMAP_PWM
	/* Backlight Led (PWM) */
	backlight_led_pwm = cfg->backlight_led;
	pr_debug("%s: backlight led on pwm type %d timer %i\n",
			__func__, backlight_led_pwm.src, backlight_led_pwm.timer);

	if (backlight_led_pwm.signal) {
		bkl_pad_on_mode = omap_mux_get_signal(
				backlight_led_pwm.signal, &bkl_pad_mux);
		if (!IS_ERR_VALUE(bkl_pad_on_mode)) {
			omap_mux_write(bkl_pad_mux.partition, 
					bkl_pad_on_mode | OMAP_PIN_OUTPUT,
					bkl_pad_mux.reg_offset);
		}
	}

	bkl_power_gpio = cfg->backlight_power;
	if (gpio_is_valid(bkl_power_gpio))
		archos_gpio_init_output(bkl_power_gpio, "bkl_power");

	switch (backlight_led_pwm.src) {
		case OMAP_DM_PWM:
			board_backlight_data.intensity_timer = backlight_led_pwm.timer;
			board_backlight_data.bkl_max = cfg->bkl_max;
			board_backlight_data.bkl_min = cfg->bkl_min;
			board_backlight_data.bkl_freq = cfg->bkl_freq;
			board_backlight_data.invert = cfg->bkl_invert;
			board_backlight_data.set_power = &bkl_set_power;
			if (backlight_led_pwm.signal_off != NULL) {
				bkl_pad_off_mode = omap_mux_get_signal(
						backlight_led_pwm.signal_off, &bkl_pad_mux);
				if (!IS_ERR_VALUE(bkl_pad_off_mode)) {
					board_backlight_data.set_pad = bkl_set_pad;
					bkl_set_pad(&board_backlight_data, 0);
				}
			}
			break;

		default:
			pr_err("%s: no support for backlight pwm source %d",
					__FUNCTION__, backlight_led_pwm.src);
			return -ENODEV;
	}

	ret = platform_device_register(&board_backlight_device);
	if (ret < 0) {
		pr_err("unable to register backlight pwm LED\n");
		return ret;
	}

	if (cfg->bkl_regulator_name) {
		bkl_reg = regulator_get( &board_backlight_device.dev,
				cfg->bkl_regulator_name);
		if (IS_ERR(bkl_reg)) {
			dev_err(&board_backlight_device.dev, "Unable to get LED regulator\n");
			bkl_reg = NULL;
		}
	}

	// set backlight power to 0
	bkl_set_power_initial(&board_backlight_data);
#endif

	return 0;
}

device_initcall(archos_leds_init);
