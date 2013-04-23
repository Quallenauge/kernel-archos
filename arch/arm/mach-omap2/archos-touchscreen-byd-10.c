/*
 *    archos-touchscreen-byd-10.c : 10/05/2012
 *    yvon robic robic@archos.com
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/stat.h>
#include <linux/types.h>

#include <mach/board-archos.h>
#include <mach/gpio.h>
#include <plat/archos-gpio.h>

#include <linux/input/byd_i2c_tsp.h>

#include "mux.h"

static int reset_gpio = UNUSED_GPIO;
static int irq_gpio = UNUSED_GPIO;

static struct regulator_consumer_supply tsp_vcc_consumer[] = {
	REGULATOR_SUPPLY("tsp_vcc", "4-005c"),
};
static struct regulator_init_data fixed_reg_tsp_vcc_initdata = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	
	.supply_regulator = "VCC",
	.consumer_supplies = tsp_vcc_consumer,
	.num_consumer_supplies = ARRAY_SIZE(tsp_vcc_consumer),
};
static struct fixed_voltage_config fixed_reg_tsp_vcc = {
	.supply_name	= "TSP_VCC",
	.microvolts	= 3300000,
	.gpio		= -EINVAL,
	.enable_high	= 1,
	.enabled_at_boot= 0,
	.init_data	= &fixed_reg_tsp_vcc_initdata,
};
static struct platform_device fixed_supply_tsp_vcc = {
	.name 	= "reg-fixed-voltage",
	.id = 99,
	.dev.platform_data = &fixed_reg_tsp_vcc,
};

static void byd_archos_reset(int on_off)
{
	pr_debug("%s:%d\n", __func__, on_off);

	if (gpio_is_valid(reset_gpio))
		gpio_set_value(reset_gpio, !on_off);
}

static int byd_archos_get_irq_level(void)
{
	int v = gpio_get_value(irq_gpio);

	pr_debug("%s:%d\n", __func__, v);

	return v;
}

int __init archos_touchscreen_byd_init(struct byd_platform_data *pdata)
{
	const struct archos_i2c_tsp_config *tsp_config;
	const struct archos_i2c_tsp_conf *conf;
	int ret;

	tsp_config = omap_get_config(ARCHOS_TAG_I2C_TSP,
			struct archos_i2c_tsp_config);

	conf = hwrev_ptr(tsp_config, system_rev);

	if (IS_ERR(conf)) {
		pr_err("%s: no device configuration for system revision %i\n",
				__FUNCTION__, system_rev);	
		return -ENODEV;
	}

	if (gpio_is_valid(conf->pwr_gpio)) {
		if (conf->pwr_signal) {
			omap_mux_init_signal(conf->pwr_signal, PIN_OUTPUT);
		} else {
			omap_mux_init_gpio(conf->pwr_gpio, PIN_OUTPUT);
		}

		fixed_reg_tsp_vcc.gpio = conf->pwr_gpio;
		platform_device_register(&fixed_supply_tsp_vcc);
	} else {
		pr_err("%s: ts pwron gpio is not valid.\n", __FUNCTION__);	
		return -ENODEV;
	}
	
	if (gpio_is_valid(conf->shtdwn_gpio)) {
		reset_gpio = conf->shtdwn_gpio;

		ret = gpio_request(reset_gpio, "byd_ts_rst");

		if (ret) {
			pr_err("%s : could not request shutdown gpio %d",
					__func__, reset_gpio);
			return ret;
		}

		gpio_direction_output(reset_gpio, 0);

		gpio_export(reset_gpio, false);
		gpio_set_value(reset_gpio, 0);
	} else {
		pr_err("%s: ts reset gpio is not valid.\n", __func__);
		return -ENODEV;
	}

	if (gpio_is_valid(conf->irq_gpio)) {
		irq_gpio = conf->irq_gpio;

		ret = gpio_request(conf->irq_gpio, "byd_ts_irq");

		if (!ret) {
			gpio_direction_input(conf->irq_gpio);

			if (conf->irq_signal)
				omap_mux_init_signal(conf->irq_signal,
						OMAP_PIN_INPUT_PULLUP
						| OMAP_PIN_OFF_WAKEUPENABLE);
			else
				omap_mux_init_gpio(conf->irq_gpio, 
						OMAP_PIN_INPUT_PULLUP
						| OMAP_PIN_OFF_WAKEUPENABLE);

			gpio_export(conf->irq_gpio, false);
		} else {
			pr_err("%s : could not request irq gpio %d\n",
					__FUNCTION__, conf->irq_gpio);
			return ret;
		}

		if (conf->irq_gpio!= -1) {
			pdata->irq = gpio_to_irq(conf->irq_gpio);
		} else {
			pdata->irq = -1;
		}
	} else {
		pr_err("%s: ts irq gpio is not valid.\n", __FUNCTION__);	
		return -ENODEV;
	}

	pdata->reset = &byd_archos_reset;
	pdata->get_irq_level = byd_archos_get_irq_level;
	pdata->regulator = tsp_vcc_consumer[0].supply;

	pr_debug("%s: irq_gpio %d - irq %d, pwr_gpio %d\n",
			__FUNCTION__, pdata->irq, conf->irq_gpio, conf->pwr_gpio);

	return 0;
}

