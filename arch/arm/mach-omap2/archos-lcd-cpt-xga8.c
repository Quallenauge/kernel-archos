/*
 * archos-lcd-cpt-xga8.c
 *
 *  Created on: jan 12, 2011
 *      Author: Robic Yvon <robic@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define DEBUG

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


#include "clock.h"
#include "mux.h"
#include <video/omapdss.h>

static struct platform_device lcd_device;
static struct archos_disp_conf display_gpio;
static int panel_state;
static struct omap_dm_timer *vcom_timer;
static struct pwm_device *vcom_pwm;
static bool have_panel;

static struct regulator *lcd_1v8;
static struct regulator *lcd_vcc;

static int __init panel_init(void)
{
	if (!have_panel)
		return -ENODEV;
	
	archos_gpio_init_output( display_gpio.lcd_pwon, "lcd_pwon" );
	archos_gpio_init_output( display_gpio.lcd_rst, "lcd_rst" );
	archos_gpio_init_output( display_gpio.lvds_en, "lvds_en" );

	if (gpio_request(display_gpio.lcd_stdby, "lcd_stdby") < 0) {
		pr_debug("gpio_init_output: cannot acquire GPIO%d \n", display_gpio.lcd_stdby);
		return -1;
	}
	omap_mux_init_signal("gpmc_ncs4.gpio_101", OMAP_PIN_OUTPUT );
	gpio_direction_output(display_gpio.lcd_stdby, 0);
	archos_gpio_init_output( display_gpio.lcd_avdd_en, "lcd_avdd_en" );

	if (display_gpio.use_fixed_bkl) {
		if (gpio_is_valid(display_gpio.bkl_en)) {
			archos_gpio_init_output(display_gpio.bkl_en, "bkl_en");
			gpio_set_value( display_gpio.bkl_en, 1);
			gpio_export(display_gpio.bkl_en, false);
		}
		if (gpio_is_valid(display_gpio.bkl_pwr)) {
			archos_gpio_init_output(display_gpio.bkl_pwr, "bkl_power");
			gpio_set_value( display_gpio.bkl_pwr, 0);
		}
	}

	if (gpio_is_valid(display_gpio.lcd_rst))
		gpio_set_value( display_gpio.lcd_rst, 1);
	if (gpio_is_valid(display_gpio.lcd_pwon))
		gpio_set_value( display_gpio.lcd_pwon, 0);
	if (gpio_is_valid(display_gpio.lvds_en))
		gpio_set_value( display_gpio.lvds_en, 0);
	if (gpio_is_valid(display_gpio.lcd_stdby))
		gpio_set_value( display_gpio.lcd_stdby, 1);
	if (gpio_is_valid(display_gpio.lcd_avdd_en))
		gpio_set_value( display_gpio.lcd_avdd_en, 1);

	/* vcom */

	if (display_gpio.vcom_pwm.signal)
		omap_mux_init_signal(display_gpio.vcom_pwm.signal, OMAP_PIN_OUTPUT);

	switch (display_gpio.vcom_pwm.src) {
		case TWL6030_PWM:
			pr_debug("%s:%d\n", __FUNCTION__, __LINE__);

			vcom_pwm = pwm_request(display_gpio.vcom_pwm.timer, "vcom");
			if (vcom_pwm == NULL) {
				pr_err("failed to request vcom pwm %d \n", 
						display_gpio.vcom_pwm.timer);
				break;
			}

			break;

		case OMAP_DM_PWM:
			pr_debug("%s:%d\n", __FUNCTION__, __LINE__);

			vcom_timer = omap_dm_timer_request_specific(display_gpio.vcom_pwm.timer);
			if (vcom_timer == NULL) {
				pr_err("failed to request vcom pwm timer %d \n", 
						display_gpio.vcom_pwm.timer);
				break;
			}

			omap_dm_timer_set_source(vcom_timer, OMAP_TIMER_SRC_SYS_CLK);

			break;
	}

	/* regulators */
	lcd_1v8 = regulator_get(&lcd_device.dev, "LCD_1V8");
	if (IS_ERR(lcd_1v8))
		dev_dbg(&lcd_device.dev, "no LCD_1V8 for this display\n");
	lcd_vcc = regulator_get(&lcd_device.dev, "LCD_VCC");
	if (IS_ERR(lcd_vcc))
		dev_dbg(&lcd_device.dev, "no LCD_VCC for this display\n");

	return 0;
}

static void pwm_set_speed(struct omap_dm_timer *gpt,
		int frequency, int duty_cycle)
{
	u32 val;
	u32 period;
	struct clk *timer_fclk;

	/* and you will have an overflow in 1 sec         */
	/* so,                              */
	/* freq_timer     -> 1s             */
	/* carrier_period -> 1/carrier_freq */
	/* => carrier_period = freq_timer/carrier_freq */

	timer_fclk = omap_dm_timer_get_fclk(gpt);
	period = clk_get_rate(timer_fclk) / frequency;
	val = 0xFFFFFFFF+1-period;
	omap_dm_timer_set_load(gpt, 1, val);

	val = 0xFFFFFFFF+1-(period*duty_cycle/256);
	omap_dm_timer_set_match(gpt, 1, val);

	/* assume overflow first: no toogle if first trig is match */
	omap_dm_timer_write_counter(gpt, 0xFFFFFFFE);
}

static int panel_enable(struct omap_dss_device *disp)
{
	pr_debug("panel_enable [%s]\n", disp->name);

	if (panel_state == 1)
		return 0;
	
	if (!IS_ERR(lcd_1v8))
		regulator_enable(lcd_1v8);
	if (!IS_ERR(lcd_vcc))
		regulator_enable(lcd_vcc);
	
	if (gpio_is_valid(display_gpio.lcd_pwon))
		gpio_set_value( display_gpio.lcd_pwon, 1 );
	msleep(50);

	if (gpio_is_valid(display_gpio.lcd_rst))
		gpio_set_value( display_gpio.lcd_rst, 0);

	if (gpio_is_valid(display_gpio.lcd_avdd_en))
		gpio_set_value( display_gpio.lcd_avdd_en, 0);
	msleep(35);

	if (gpio_is_valid(display_gpio.lvds_en))
		gpio_set_value( display_gpio.lvds_en, 1);

	msleep(10);

	if (gpio_is_valid(display_gpio.lcd_stdby))
		gpio_set_value( display_gpio.lcd_stdby, 0);

	if (display_gpio.use_fixed_bkl) {
		if (gpio_is_valid(display_gpio.bkl_pwr))
			gpio_set_value( display_gpio.bkl_pwr, 1);

		if (gpio_is_valid(display_gpio.bkl_en))
			gpio_set_value( display_gpio.bkl_en, 0);
	}

	if (vcom_pwm != NULL) {
		pwm_enable(vcom_pwm);
		pwm_config(vcom_pwm, 128, 256);
	} else if (vcom_timer != NULL) {
		omap_dm_timer_set_pwm( vcom_timer, 1, 1, OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);

		pwm_set_speed(vcom_timer, 30000, 140);	//120< x <160
		omap_dm_timer_start(vcom_timer);
	}

	panel_state = 1;

	return 0;
}

static void panel_disable(struct omap_dss_device *disp)
{
	pr_debug("panel_disable [%s]\n", disp->name);

	if (panel_state == 0)
		return;
	
	if (display_gpio.use_fixed_bkl) {
		if (gpio_is_valid(display_gpio.bkl_en))
			gpio_set_value( display_gpio.bkl_en, 1);

		if (gpio_is_valid(display_gpio.bkl_pwr))
			gpio_set_value( display_gpio.bkl_pwr, 0);
	}

	if (gpio_is_valid(display_gpio.lcd_stdby))
		gpio_set_value( display_gpio.lcd_stdby, 1);
	msleep(1);

	if (gpio_is_valid(display_gpio.lvds_en))
		gpio_set_value( display_gpio.lvds_en, 0);

	if (gpio_is_valid(display_gpio.lcd_avdd_en))
		gpio_set_value( display_gpio.lcd_avdd_en, 1);
	msleep(10);

	if (gpio_is_valid(display_gpio.lcd_pwon))
		gpio_set_value( display_gpio.lcd_pwon, 0 );

	if (gpio_is_valid(display_gpio.lcd_rst))
		gpio_set_value( display_gpio.lcd_rst, 1);

	if (vcom_pwm != NULL) {
		pwm_disable(vcom_pwm);
	} else if (vcom_timer != NULL) {
		omap_dm_timer_stop(vcom_timer);
	}

	if (!IS_ERR(lcd_1v8))
		regulator_disable(lcd_1v8);
	if (!IS_ERR(lcd_vcc))
		regulator_disable(lcd_vcc);
	
	panel_state = 0;
}


static struct platform_device lcd_device = {
	.name           = "lcd_panel",
	.id             = 0,
	.num_resources   = 0,
};

/* Using generic display panel */
static struct panel_generic_dpi_data cpt_xga_8_panel = {
	.name			= "cpt_xga_8",
	.platform_enable	= panel_enable,
	.platform_disable	= panel_disable,
};

struct omap_dss_device cpt_xga_8_dss_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "xga8",
	.driver_name		= "generic_dpi_panel",
	.data			= &cpt_xga_8_panel,
	.phy.dpi.data_lines	= 24,
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

int __init panel_cpt_xga_8_init(struct omap_dss_device *disp_data)
{
	const struct archos_display_config *disp_cfg;
	const struct archos_disp_conf * conf;
	int ret = -ENODEV;

	pr_debug("panel_cpt_xga_8_init\n");

	disp_cfg = omap_get_config( ARCHOS_TAG_DISPLAY, struct archos_display_config );
	conf = hwrev_ptr(disp_cfg, system_rev);
	if (IS_ERR(conf))
		return ret;

	display_gpio = *conf;

	if (IS_ERR_VALUE(platform_device_register(&lcd_device)))
		pr_info("%s: cannot register lcd_panel device\n", __func__);
	
	*disp_data = cpt_xga_8_dss_device;
	if (conf->do_not_use_pll)
		disp_data->clocks.dispc.dispc_fclk_src = disp_data->clocks.dispc.channel.lcd_clk_src = OMAP_DSS_CLK_SRC_FCK;
	have_panel = true;
	
	return 0;
}

device_initcall(panel_init);
