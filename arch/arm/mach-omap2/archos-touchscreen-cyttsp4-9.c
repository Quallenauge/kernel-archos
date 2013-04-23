/*
 * archos-touchscreen-tma4xx.c
 * g.revaillot, revaillot@archos.com
 *
 * based on board-touch-cyttsp4_core.c
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 * Modified by: Cypress Semiconductor - 2011-2012
 *    - add auto load image include
 *    - add TMA400 support
 *    - add TMA884 support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define DEBUG

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <mach/board-archos.h>
#include <mach/gpio.h>
#include <plat/archos-gpio.h>

#include <linux/input/touch_platform.h>

#include "archos-touchscreen-cyttsp4-9.h"

#include "mux.h"

#include "cyttsp4_params.h"

static int reset_gpio = UNUSED_GPIO;
static int irq_gpio = UNUSED_GPIO;

#define TOUCH_GPIO_RST_CYTTSP reset_gpio
#define TOUCH_GPIO_IRQ_CYTTSP irq_gpio

#define CY_WAKE_DFLT                99	/* causes wake strobe on INT line
					 * in sample board configuration
					 * platform data->hw_recov() function
					 */

#define CY_USE_INCLUDE_FBL

static int cyttsp4_hw_reset(void)
{
	pr_info("%s: strobe RST(%d) pin\n", __func__,
			TOUCH_GPIO_RST_CYTTSP);

	gpio_set_value(TOUCH_GPIO_RST_CYTTSP, 1);
	msleep(20);
	gpio_set_value(TOUCH_GPIO_RST_CYTTSP, 0);
	msleep(40);
	gpio_set_value(TOUCH_GPIO_RST_CYTTSP, 1);
	msleep(20);

	return 0;
}

static int cyttsp4_hw_recov(int on)
{
	int retval = 0;

	switch (on) {
	case 0:
		cyttsp4_hw_reset();
		retval = 0;
		break;
	case CY_WAKE_DFLT:
		{
			retval = gpio_direction_output (TOUCH_GPIO_IRQ_CYTTSP, 0);
			if (retval < 0) {
				pr_err("%s: Fail switch IRQ pin to OUT r=%d\n",
					__func__, retval);
			} else {
				udelay(2000);
				retval = gpio_direction_input
					(TOUCH_GPIO_IRQ_CYTTSP);
				if (retval < 0) {
					pr_err("%s: Fail switch IRQ pin to IN"
						" r=%d\n", __func__, retval);
				}
			}
		}
		break;
	default:
		retval = -ENOSYS;
		break;
	}

	return retval;
}

static int cyttsp4_irq_stat(void)
{
	return gpio_get_value(TOUCH_GPIO_IRQ_CYTTSP);
}

static struct touch_settings cyttsp4_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp4_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};

/* Design Data Table */
static u8 cyttsp4_ddata[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23, 24 /* test padding
	, 25, 26, 27, 28, 29, 30, 31 */
};

static struct touch_settings cyttsp4_sett_ddata = {
	.data = (uint8_t *)&cyttsp4_ddata[0],
	.size = ARRAY_SIZE(cyttsp4_ddata),
	.tag = 0,
};

/* Manufacturing Data Table */
static u8 cyttsp4_mdata[] = {
	65, 64, /* test truncation */63, 62, 61, 60, 59, 58, 57, 56, 55,
	54, 53, 52, 51, 50, 49, 48,
	47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32,
	31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16,
	15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0
};

static struct touch_settings cyttsp4_sett_mdata = {
	.data = (uint8_t *)&cyttsp4_mdata[0],
	.size = ARRAY_SIZE(cyttsp4_mdata),
	.tag = 0,
};

/* Button to keycode conversion */
static u16 cyttsp4_btn_keys[] = {
	/* use this table to map buttons to keycodes (see input.h) */
	KEY_HOME,		/* 102 */
	KEY_MENU,		/* 139 */
	KEY_BACK,		/* 158 */
	KEY_SEARCH,		/* 217 */
	KEY_VOLUMEDOWN,		/* 114 */
	KEY_VOLUMEUP,		/* 115 */
	KEY_CAMERA,		/* 212 */
	KEY_POWER		/* 116 */
};

static struct touch_settings cyttsp4_sett_btn_keys = {
	.data = (uint8_t *)&cyttsp4_btn_keys[0],
	.size = ARRAY_SIZE(cyttsp4_btn_keys),
	.tag = 0,
};

/* use this define to include auto boot image
 */
#ifdef CY_USE_INCLUDE_FBL
#include "cyttsp4_img.h"
static struct touch_firmware cyttsp4_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
static struct touch_firmware cyttsp4_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

static const uint16_t cyttsp4_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	ABS_MT_TOUCH_MAJOR, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
};

struct touch_framework cyttsp4_framework = {
	.abs = (uint16_t *)&cyttsp4_abs[0],
	.size = ARRAY_SIZE(cyttsp4_abs),
	.enable_vkeys = 0,
};

struct touch_platform_data defaults = {
	.sett = {
		NULL,	/* Reserved */
		NULL,	/* Command Registers */
		NULL,	/* Touch Report */
		NULL,	/* Cypress Data Record */
		NULL,	/* Test Record */
		NULL,	/* Panel Configuration Record */
		&cyttsp4_sett_param_regs,
		&cyttsp4_sett_param_size,
		NULL,	/* Reserved */
		NULL,	/* Reserved */
		NULL,	/* Operational Configuration Record */
		NULL, /* &cyttsp4_sett_ddata, *//* Design Data Record */
		NULL, /* &cyttsp4_sett_mdata, *//* Manufacturing Data Record */
		NULL,	/* Config and Test Registers */
		NULL, /* &cyttsp4_sett_btn_keys, */	/* button-to-keycode table */
	},
	.fw = &cyttsp4_firmware,
	.frmwrk = &cyttsp4_framework,
	.addr = {CY_I2C_TCH_ADR, CY_I2C_LDR_ADR},
	.flags = 0x00,
	.hw_reset = cyttsp4_hw_reset,
	.hw_recov = cyttsp4_hw_recov,
	.irq_stat = cyttsp4_irq_stat,
};

static struct regulator_consumer_supply tsp_vcc_consumer[] = {
	REGULATOR_SUPPLY("tsp_vcc", "4-0067"),
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

void archos_touchscreen_cyttsp4_init(struct touch_platform_data *pdata)
{	
	const struct archos_i2c_tsp_config *tsp_config;
	const struct archos_i2c_tsp_conf *conf;

	tsp_config = omap_get_config(ARCHOS_TAG_I2C_TSP,
			struct archos_i2c_tsp_config);

	conf = hwrev_ptr(tsp_config, system_rev);

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
		return;
	}

	if (gpio_is_valid(conf->shtdwn_gpio)) {
		reset_gpio = conf->shtdwn_gpio;
		gpio_request(reset_gpio, "cyttsp4_rst");
		gpio_direction_output(reset_gpio, 0);
		omap_mux_init_gpio(conf->shtdwn_gpio, OMAP_PIN_OUTPUT);
		gpio_set_value(reset_gpio, 1);
	} else {
		pr_err("%s: ts reset gpio is not valid.\n", __func__);
		return;
	}

	if (gpio_is_valid(conf->irq_gpio)) {
		irq_gpio = conf->irq_gpio;
		gpio_request(irq_gpio, "cyttsp4_irq");
		gpio_direction_input(irq_gpio);

		if (conf->irq_signal)
			omap_mux_init_signal(conf->irq_signal,
					OMAP_PIN_INPUT_PULLUP
					| OMAP_PIN_OFF_WAKEUPENABLE);
		else
			omap_mux_init_gpio(conf->irq_gpio, 
					OMAP_PIN_INPUT_PULLUP
					| OMAP_PIN_OFF_WAKEUPENABLE);

	} else {
		pr_err("%s: ts irq gpio is not valid.\n", __FUNCTION__);	
		return;
	}

	memcpy(pdata, &defaults, sizeof(defaults));

	pdata->irq = gpio_to_irq(conf->irq_gpio);
	pdata->regulator = tsp_vcc_consumer[0].supply;

	pr_debug("%s: irq_gpio %d - irq %d, pwr_gpio %d\n",
			__FUNCTION__, pdata->irq, conf->irq_gpio, conf->pwr_gpio);

	return;
}


