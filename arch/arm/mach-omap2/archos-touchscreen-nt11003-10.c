/*
 *    archos-touchscreen-nt11003.c : 09/05/2012
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

#include <linux/input/nt11003.h>

#include "mux.h"

static int reset_gpio = UNUSED_GPIO;
static int irq_gpio = UNUSED_GPIO;
static const int i2c4_scl_gpio = 132;
static const int i2c4_sda_gpio = 133;

static struct regulator_consumer_supply tsp_vcc_consumer[] = {
	REGULATOR_SUPPLY("tsp_vcc", "4-0001"),
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

static void nt11003_archos_reset(int on_off)
{
	pr_debug("%s:%d\n", __func__, on_off);

	if (gpio_is_valid(reset_gpio))
		gpio_set_value(reset_gpio, !on_off);
}

static int nt11003_archos_get_irq_level(void)
{
	int v = gpio_get_value(irq_gpio);

	pr_debug("%s:%d\n", __func__, v);

	return v;
}

static int nt11003_archos_demux_i2c(int demux)
{

	pr_debug("%s:%d\n", __func__, demux);

	if (demux) {
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

	return 0;
}

int __init archos_touchscreen_nt11003_init(struct nt11003_platform_data *pdata)
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
			omap_mux_init_signal(conf->pwr_signal, PIN_OUTPUT);
		} else {
			omap_mux_init_gpio(conf->pwr_gpio, PIN_OUTPUT);
		}

		fixed_reg_tsp_vcc.gpio = conf->pwr_gpio;

		ret = platform_device_register(&fixed_supply_tsp_vcc);

		if (ret) {
			pr_err("%s: could not register tsp regulator.\n", __func__);
			return ret;
		}

	} else {
		pr_err("%s: ts pwron gpio is not valid.\n", __func__);
		return -ENODEV;
	}

	if (gpio_is_valid(conf->shtdwn_gpio)) {
		reset_gpio = conf->shtdwn_gpio;

		ret = gpio_request(reset_gpio, "nt11003_ts_rst");

		if (ret) {
			pr_err("%s : could not request shutdown gpio %d",
					__func__, reset_gpio);
			return ret;
		}

		gpio_direction_output(reset_gpio, 0);

		if (conf->shtdwn_signal) {
			omap_mux_init_signal(conf->shtdwn_signal, PIN_OUTPUT);
		} else {
			omap_mux_init_gpio(conf->shtdwn_gpio, PIN_OUTPUT);
		}

		gpio_export(reset_gpio, false);
		gpio_set_value(reset_gpio, 0);
	} else {
		pr_err("%s: ts reset gpio is not valid.\n", __func__);
		return -ENODEV;
	}

	if (gpio_is_valid(conf->irq_gpio)) {
		irq_gpio = conf->irq_gpio;

		ret = gpio_request(irq_gpio, "nt11003_ts_irq");

		if (ret) {
			pr_err("%s : could not request irq gpio %d\n",
					__func__, irq_gpio);
			return ret;
		}

		gpio_direction_input(irq_gpio);

		if (conf->irq_signal)
			omap_mux_init_signal(conf->irq_signal,
					OMAP_PIN_INPUT_PULLUP
					| OMAP_PIN_OFF_WAKEUPENABLE);
		else
			omap_mux_init_gpio(irq_gpio,
					OMAP_PIN_INPUT_PULLUP
					| OMAP_PIN_OFF_WAKEUPENABLE);

		gpio_export(irq_gpio, false);

		pdata->irq = gpio_to_irq(irq_gpio);
	} else {
		pr_err("%s: ts irq gpio is not valid.\n", __func__);
		return -ENODEV;
	}

	pdata->reset = nt11003_archos_reset;
	pdata->get_irq_level = nt11003_archos_get_irq_level;
	pdata->demux_i2c = nt11003_archos_demux_i2c;
	pdata->regulator = tsp_vcc_consumer[0].supply;

	pr_debug("%s: irq_gpio %d - irq %d, reset_gpio %d, pwr_gpio %d\n",
			__func__, pdata->irq, conf->irq_gpio, conf->shtdwn_gpio, conf->pwr_gpio);


	return 0;
}

