/*
 *    archos-touchscreen-pixcir-10.c : 15/04/2011
 *    g.revaillot, revaillot@archos.com
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

#include <linux/input/pixcir_i2c_tsp.h>

#include "mux.h"

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

int __init archos_touchscreen_pixcir_init(struct pixcir_platform_data *pdata)
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

	if (conf->pwr_gpio > 0) {
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

	if (conf->irq_gpio > 0) {
		ret = gpio_request(conf->irq_gpio, "pixcir_ts_irq");

		if (!ret) {
			gpio_direction_input(conf->irq_gpio);

			if (conf->irq_signal)
				omap_mux_init_signal(conf->irq_signal,
						OMAP_PIN_INPUT
						| OMAP_PIN_OFF_WAKEUPENABLE);
			else
				omap_mux_init_gpio(conf->irq_gpio, 
						OMAP_PIN_INPUT
						| OMAP_PIN_OFF_WAKEUPENABLE);

			gpio_export(conf->irq_gpio, false);
		} else {
			printk("%s : could not request irq gpio %d\n",
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

	pr_debug("%s: irq_gpio %d - irq %d, pwr_gpio %d\n",
			__FUNCTION__, pdata->irq, conf->irq_gpio, conf->pwr_gpio);

	// dvt/mtu units used to need one axis inverted
	// and some geometry tweaking..
	if (system_rev < 2) {
		pdata->flags |= PIXCIR_FLAGS_INV_X;
		pdata->x_scale = 60;	// +6.0%
		pdata->y_scale = 40;	// +4.0%
		pdata->x_offset = 25;	// +25 pix
		pdata->y_offset = 10;	// +10 pix 
	}

	return 0;
}


