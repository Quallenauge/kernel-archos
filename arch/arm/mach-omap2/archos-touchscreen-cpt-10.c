/*
 *    archos-touchscreen-cpt-10.c : 09/12/2011
 *    g.revaillot, revaillot@archos.com
 */

//#define DEBUG

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

#include <linux/input/cpt_i2c_tsp.h>

#include "mux.h"

const int i2c4_scl_gpio = 132;
const int i2c4_sda_gpio = 133;

static int irq_gpio = UNUSED_GPIO;

static struct regulator_consumer_supply tsp_vcc_consumer[] = {
	REGULATOR_SUPPLY("tsp_vcc", "4-007f"),
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
	.id = 6,
	.dev.platform_data = &fixed_reg_tsp_vcc,
};

static void reset(int on_off)
{
	static int state = 0;

	if (on_off == state)
		return;

	if (on_off) {
		// disable irq pull up.
		omap_mux_init_gpio(irq_gpio, OMAP_PIN_INPUT);

		// switch back i2c lines to gpio mode
		gpio_request(i2c4_scl_gpio, "i2c4_scl_gpio");
		gpio_request(i2c4_sda_gpio, "i2c4_sda_gpio");

		omap_mux_init_gpio(i2c4_scl_gpio, OMAP_PIN_OUTPUT);
		omap_mux_init_gpio(i2c4_sda_gpio, OMAP_PIN_OUTPUT);

		gpio_direction_output(i2c4_scl_gpio, 0);
		gpio_direction_output(i2c4_sda_gpio, 0);

		// force low level.
		gpio_set_value(i2c4_scl_gpio, 0);
		gpio_set_value(i2c4_sda_gpio, 0);
	} else {
		// put back irq pull up.
		omap_mux_init_gpio(irq_gpio, OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE);

		gpio_free(i2c4_scl_gpio);
		gpio_free(i2c4_sda_gpio);

		// mux back to i2c lines.
		omap_mux_init_signal("i2c4_scl.i2c4_scl", OMAP_PIN_INPUT);
		omap_mux_init_signal("i2c4_sda.i2c4_sda", OMAP_PIN_INPUT);
	}

	state = on_off;

	return;
}

int __init archos_touchscreen_cpt_i2c_tsp_init(struct cpt_i2c_tsp_platform_data *pdata)
{
	const struct archos_i2c_tsp_config *tsp_config;
	const struct archos_i2c_tsp_conf *conf;
	int ret;

	tsp_config = omap_get_config(ARCHOS_TAG_I2C_TSP,
			struct archos_i2c_tsp_config);

	conf = hwrev_ptr(tsp_config, system_rev);

	if (IS_ERR(conf)) {
		pr_err("%s: no device configuration for system_rev %i\n",
				__FUNCTION__, system_rev);
		return -ENODEV;
	}

	if (gpio_is_valid(conf->pwr_gpio)) {
		if (conf->pwr_signal) {
			omap_mux_init_signal(conf->pwr_signal, OMAP_PIN_OUTPUT);
		} else {
			omap_mux_init_gpio(conf->pwr_gpio, OMAP_PIN_OUTPUT);
		}

		fixed_reg_tsp_vcc.gpio = conf->pwr_gpio;
		platform_device_register(&fixed_supply_tsp_vcc);
	} else {
		pr_err("%s: ts pwron gpio is not valid.\n", __FUNCTION__);
		return -ENODEV;
	}

	if (gpio_is_valid(conf->irq_gpio)) {
		if ((ret = gpio_request(conf->irq_gpio, "cpt_i2c_tsp_ts_irq"))) {
			pr_err("%s : could not request irq gpio %d\n",
					__func__, conf->irq_gpio);
			return ret;
		}

		gpio_direction_input(conf->irq_gpio);

		if (conf->irq_signal) {
			omap_mux_init_signal(conf->irq_signal,
					OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE);
		} else {
			omap_mux_init_gpio(conf->irq_gpio,
					OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE);
		}

		irq_gpio = conf->irq_gpio;
		pdata->irq = gpio_to_irq(conf->irq_gpio);
	} else {
		pr_err("%s: ts irq gpio is not valid.\n", __func__);
		return -ENODEV;
	}

	pr_debug("%s: irq_gpio %d - irq %d, pwr_gpio %d\n",
			__func__, irq_gpio, pdata->irq, conf->pwr_gpio);

	pdata->reset = &reset;

	return 0;
}



