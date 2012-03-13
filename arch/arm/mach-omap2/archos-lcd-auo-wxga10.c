/*
 * archos-lcd-auo-wxga.c
 *
 *  Created on: Feb 14, 2011
 *      Author: Yvon Robic <robic@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>

#include <mach/gpio.h>
#include <plat/archos-gpio.h>
#include <asm/mach-types.h>
#include <plat/board.h>
#include <mach/board-archos.h>
#include <linux/delay.h>

#include "clock.h"
#include "mux.h"
#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>

static struct platform_device lcd_device;
static struct archos_disp_conf display_gpio;
static int panel_state;

static struct regulator *lcd_1v8;
static struct regulator *lcd_vcc;
static bool have_panel;
static u32 gamma_table[256];

static int __init panel_init(void)
{
	if (!have_panel)
		return -ENODEV;
	
	pr_debug("panel_init\n");
	
	archos_gpio_init_output( display_gpio.lcd_pwon, "lcd_pwon" );
	archos_gpio_init_output( display_gpio.lvds_en, "lvds_en" );

	if (gpio_is_valid(display_gpio.lcd_stdby)){
		if (gpio_request(display_gpio.lcd_stdby, "dc_en") < 0) {
			pr_debug("gpio_init_output: cannot acquire GPIO%d \n", display_gpio.lcd_stdby);
			return -1;
		}
	}
	omap_mux_init_signal("gpmc_ncs4.gpio_101", OMAP_PIN_OUTPUT );
	gpio_direction_output(display_gpio.lcd_stdby, 0);

	archos_gpio_init_output( display_gpio.lcd_avdd_en, "bkl_en" );

	if (display_gpio.use_fixed_bkl) {
		if (gpio_is_valid(display_gpio.bkl_en)) {
			archos_gpio_init_output(display_gpio.bkl_en, "bkl_en");
			gpio_set_value( display_gpio.bkl_en, 1);
		}
		if (gpio_is_valid(display_gpio.bkl_pwr)) {
			archos_gpio_init_output(display_gpio.bkl_pwr, "bkl_power");
			gpio_set_value( display_gpio.bkl_pwr, 0);
		}
	}

	if (gpio_is_valid(display_gpio.lcd_pwon))
		gpio_set_value( display_gpio.lcd_pwon, 0);
	if (gpio_is_valid(display_gpio.lvds_en))
		gpio_set_value( display_gpio.lvds_en, 0);
	if (gpio_is_valid(display_gpio.lcd_stdby))
		gpio_set_value( display_gpio.lcd_stdby, 1);
	if (gpio_is_valid(display_gpio.lcd_avdd_en))
		gpio_set_value( display_gpio.lcd_avdd_en, 1);

	/* regulators */
	lcd_1v8 = regulator_get(&lcd_device.dev, "LCD_1V8");
	if (IS_ERR(lcd_1v8))
		dev_dbg(&lcd_device.dev, "no LCD_1V8 for this display\n");
	lcd_vcc = regulator_get(&lcd_device.dev, "LCD_VCC");
	if (IS_ERR(lcd_vcc))
		dev_dbg(&lcd_device.dev, "no LCD_VCC for this display\n");
	
	return 0;
}

static int panel_enable(struct omap_dss_device *disp)
{
	pr_debug("panel_enable [%s]\n", disp->name);

	if ( panel_state == 1)
		return -1;

	if (!IS_ERR_OR_NULL(lcd_1v8))
		regulator_enable(lcd_1v8);
	if (!IS_ERR_OR_NULL(lcd_vcc))
		regulator_enable(lcd_vcc);

	if (gpio_is_valid(display_gpio.lcd_pwon))
		gpio_set_value( display_gpio.lcd_pwon, 1 );
	
	msleep(10);

	if (gpio_is_valid(display_gpio.lcd_stdby))		//dc en/
		gpio_set_value( display_gpio.lcd_stdby, 0);

	msleep(10);

	if (gpio_is_valid(display_gpio.lvds_en))
		gpio_set_value( display_gpio.lvds_en, 1);

	msleep(200);

	if (gpio_is_valid(display_gpio.lcd_avdd_en))		// bkl_en/
		gpio_set_value( display_gpio.lcd_avdd_en, 0);

	if (display_gpio.use_fixed_bkl) {
		if (gpio_is_valid(display_gpio.bkl_pwr))
			gpio_set_value( display_gpio.bkl_pwr, 1);

		if (gpio_is_valid(display_gpio.bkl_en))
			gpio_set_value( display_gpio.bkl_en, 0);
	}

	panel_state = 1;
	return 0;
}

static void panel_disable(struct omap_dss_device *disp)
{
	pr_debug("panel_disable [%s]\n", disp->name);

	if (display_gpio.use_fixed_bkl) {
		if (gpio_is_valid(display_gpio.bkl_en))
			gpio_set_value( display_gpio.bkl_en, 1);

		if (gpio_is_valid(display_gpio.bkl_pwr))
			gpio_set_value( display_gpio.bkl_pwr, 0);
	}

	if (gpio_is_valid(display_gpio.lcd_avdd_en))		// bkl_en/
		gpio_set_value( display_gpio.lcd_avdd_en, 1);

	msleep(200);

	if (gpio_is_valid(display_gpio.lvds_en))
		gpio_set_value( display_gpio.lvds_en, 0);

	msleep(10);
	if (gpio_is_valid(display_gpio.lcd_stdby))		//dc en/
		gpio_set_value( display_gpio.lcd_stdby, 1);

	if (gpio_is_valid(display_gpio.lcd_pwon))
		gpio_set_value( display_gpio.lcd_pwon, 0 );

	if (!IS_ERR_OR_NULL(lcd_1v8))
		regulator_disable(lcd_1v8);
	if (!IS_ERR_OR_NULL(lcd_vcc))
		regulator_disable(lcd_vcc);
	
	panel_state = 0;
}

// init specific gamma table values for this panel, to mask artifacts in low rgb levels
static void gamma_tab_init(u32 *tab)
{
	int i;
	// crop value > 8 and < 16
	#define CROP_VAL 10

	for (i = 0; i< 256; i++) {
		if (i < CROP_VAL)
			tab[i] = (i << 24) | (CROP_VAL << 16) | (CROP_VAL << 8) | CROP_VAL;
		else
			tab[i] = (i << 24) | (i << 16) | (i << 8) | i;
	}
}

/* Using generic display panel */
static struct panel_generic_dpi_data auo_wxga_10_panel = {
	.name			= "auo_wxga10",
	.platform_enable	= panel_enable,
	.platform_disable	= panel_disable,
};

struct omap_dss_device auo_wxga_10_dss_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "wxga10",
	.driver_name		= "generic_dpi_panel",
	.data			= &auo_wxga_10_panel,
	.phy.dpi.data_lines	= 18,
	.phy.dpi.dither 	= OMAP_DSS_DITHER_SPATIAL,
	.phy.dpi.gamma_correction = gamma_table,
	.reset_gpio		= -1,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
	.clocks = {
		.dispc = {
			 .channel = {
				.lck_div        = 1,
				.pck_div        = 2,
				.lcd_clk_src    = OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC,
		},
	},
};

static struct platform_device lcd_device = {
	.name           = "lcd_panel",
	.id             = 0,
	.num_resources   = 0,
};

int __init panel_auo_wxga_10_init(struct omap_dss_device *disp_data)
{
	const struct archos_display_config *disp_cfg;
	const struct archos_disp_conf * conf;
	int ret = -ENODEV;

	pr_debug("panel_auo_wxga_10_init\n");
	
	disp_cfg = omap_get_config( ARCHOS_TAG_DISPLAY, struct archos_display_config );

	conf = hwrev_ptr(disp_cfg, system_rev);
	if (IS_ERR(conf)) {
		pr_info("panel_auo_wxga_10_init: no configuration for system_rev %i\n",
				system_rev);
		return ret;
	}

	display_gpio = *conf;

	if (IS_ERR_VALUE(platform_device_register(&lcd_device)))
		pr_info("%s: cannot register lcd_panel device\n", __func__);

	gamma_tab_init(gamma_table);
	*disp_data = auo_wxga_10_dss_device;
	if (conf->do_not_use_pll)
		disp_data->clocks.dispc.dispc_fclk_src = 
				disp_data->clocks.dispc.channel.lcd_clk_src = OMAP_DSS_CLK_SRC_FCK;
	have_panel = true;
	
	return 0;
}

device_initcall(panel_init);
