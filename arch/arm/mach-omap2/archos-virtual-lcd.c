/*
 * archos-virtual-lcd.c
 *
 *  Created on: jan 12, 2011
 *      Author: Caron Francois <caron@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>

#include <mach/gpio.h>
#include <plat/archos-gpio.h>
#include <asm/mach-types.h>
#include <plat/board.h>
#include <mach/board-archos.h>
#include <linux/delay.h>
#include <plat/dmtimer.h>
#include <video/omap-panel-generic-dpi.h>

#include <video/omapdss.h>


static int __init panel_init(void)
{
	return 0;
}

static int panel_enable(struct omap_dss_device *disp)
{
	return 0;
}

static void panel_disable(struct omap_dss_device *disp)
{
}


static struct platform_device lcd_device = {
	.name           = "lcd_panel",
	.id             = 0,
	.num_resources   = 0,
};

/* Using generic display panel */
static struct panel_generic_dpi_data virtual_1080p_panel = {
	.name			= "generic_32_16_9_1080p",
	.platform_enable	= panel_enable,
	.platform_disable	= panel_disable,
};

/* Using generic display panel */
static struct panel_generic_dpi_data virtual_720p_panel = {
	.name			= "generic_32_16_9_720p",
	.platform_enable	= panel_enable,
	.platform_disable	= panel_disable,
};

struct omap_dss_device virtual_dss_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "virtual",
	.driver_name		= "generic_dpi_panel",
	.data			= &virtual_720p_panel,
	.phy.dpi.data_lines	= 24,
	.reset_gpio		= -1,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
	.clocks = {
		.dispc = {
			 .channel = {
				.lck_div        = 1,
				.pck_div        = 2,
				.lcd_clk_src    = OMAP_DSS_CLK_SRC_FCK,
			},
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
		},
	},
};

int __init panel_virtual_init(struct omap_dss_device *disp_data)
{
	const struct archos_display_config *disp_cfg;
	const struct archos_disp_conf * conf;
	int ret = -ENODEV;

	pr_debug("panel_virtual_init\n");

	disp_cfg = omap_get_config( ARCHOS_TAG_DISPLAY, struct archos_display_config );
	conf = hwrev_ptr(disp_cfg, system_rev);
	if (IS_ERR(conf))
		return ret;

	if (IS_ERR_VALUE(platform_device_register(&lcd_device)))
		pr_info("%s: cannot register lcd_panel device\n", __func__);

	*disp_data = virtual_dss_device;
	
	return 0;
}

device_initcall(panel_init);
