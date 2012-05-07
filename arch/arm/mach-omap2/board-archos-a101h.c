/*
 * board-archos-a101h.c
 *
  *  Created on: Jan 09, 2012
 *      Author: Niklas Schroeter <schroeter@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/reboot.h>
#include <linux/usb/otg.h>
#include <linux/hwspinlock.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/gpio_keys.h>
#include <linux/hwspinlock.h>
#include <linux/wakelock.h>

#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/delay.h>

#include <mach/dmm.h>
#include <mach/hardware.h>
#include <mach/omap4-common.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/feature_list.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/omap-serial.h>
#include <plat/serial.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#include <plat/mmc.h>
#include <plat/omap-pm.h>
#include <video/omapdss.h>
#include <linux/wl12xx.h>

#include <linux/input/cypress-tma340.h>
#include <linux/input/tr16c0-i2c.h>
#include <linux/input/cpt_i2c_tsp.h>

#include <plat/archos-audio-twl6040.h>
#include <mach/board-archos.h>
#include <linux/usb/gpio_vbus.h>
#include <plat/temperature_sensor.h>

#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>

#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "control.h"
#include "common-board-devices.h"
#include "pm.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
#include "omap4_ion.h"

static struct mma8453q_pdata board_mma8453q_pdata;
static struct akm8975_platform_data board_akm8975_pdata;
static struct gpio_vbus_mach_info archos_vbus_info;

#define WILINK_UART_DEV_NAME "/dev/ttyO1"

#ifdef CONFIG_ARCHOS_UART3_ON_DPDM
#define uart3_on_usb true
#else
#define uart3_on_usb false
#endif

#define TPS62361_GPIO   7

#define GPIO_5V_PWRON            36	/* fixme: from config tags? */
#define GPIO_1V8_PWRON           34	/* fixme: from config tags? */
#define GPIO_VCC_PWRON           35	/* fixme: from config tags? */
#define GPIO_VBUS_MUSB_PWRON    111	/* fixme: from config tags? */
#define HDMI_GPIO_HPD		 63  	/* Hot plug pin for HDMI */

static void remux_regulator_gpio(int gpio)
{
	switch (gpio) {
	default:
		omap_mux_init_gpio(gpio, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
		break;

	case 104:
		omap_mux_init_signal("gpmc_ncs7.gpio_104",
				OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
		break;

	case GPIO_VCC_PWRON:
	case GPIO_1V8_PWRON:
		/* These signals control voltages that are needed after warm reset,
		   make sure, they are pulled high */
		omap_mux_init_gpio(gpio, OMAP_PIN_INPUT_PULLUP|OMAP_PIN_OUTPUT);
		break;
	}
}


static struct regulator_consumer_supply fixed_reg_5v_consumer[] = {
	REGULATOR_SUPPLY("hsusb_vbus0", "uhhtll-omap"),
	REGULATOR_SUPPLY("5V", "vbus_musb"),
	REGULATOR_SUPPLY("5V", "4-005c"),
};
static struct regulator_init_data fixed_reg_5v_initdata = {
	.constraints = {
		.min_uV = 5000000,
		.max_uV = 5000000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies      = fixed_reg_5v_consumer,
	.num_consumer_supplies  = ARRAY_SIZE(fixed_reg_5v_consumer),
};
static struct fixed_voltage_config fixed_reg_5v = {
	.supply_name	= "5V",
	.microvolts	= 5000000,
	.gpio		= GPIO_5V_PWRON,
	.enable_high	= 1,
	.enabled_at_boot= 0,
	.init_data	= &fixed_reg_5v_initdata,
	.remux		= remux_regulator_gpio,
};
static struct platform_device fixed_supply_5v = {
	.name 	= "reg-fixed-voltage",
	.id	= 0,
	.dev.platform_data = &fixed_reg_5v,
};

static struct regulator_consumer_supply fixed_reg_1v8_consumer[] = {
	REGULATOR_SUPPLY("GPS_1V8", "nl5550.0"),
	REGULATOR_SUPPLY("LCD_1V8", "lcd_panel.0"),
	REGULATOR_SUPPLY("ACCEL_1V8", "3-001c"),
	REGULATOR_SUPPLY("COMPASS_1V8", "3-000c"),
	REGULATOR_SUPPLY("hsusb0", "uhhtll-omap"),
	REGULATOR_SUPPLY("hdmi_vref", "omapdss_hdmi"),
};
static struct regulator_init_data fixed_reg_1v8_initdata = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = fixed_reg_1v8_consumer,
	.num_consumer_supplies = ARRAY_SIZE(fixed_reg_1v8_consumer),
};
static struct fixed_voltage_config fixed_reg_1v8 = {
	.supply_name	= "1V8",
	.microvolts	= 1800000,
	.gpio		= GPIO_1V8_PWRON,
	.enable_high	= 1,
	.enabled_at_boot= 1,
	.init_data	= &fixed_reg_1v8_initdata,
	.remux		= remux_regulator_gpio,
};
static struct platform_device fixed_supply_1v8 = {
	.name 	= "reg-fixed-voltage",
	.id	= 1,
	.dev.platform_data = &fixed_reg_1v8,
};

static struct regulator_consumer_supply fixed_reg_vcc_consumer[] = {
	REGULATOR_SUPPLY("LCD_VCC", "lcd_panel.0"),
	REGULATOR_SUPPLY("ACCEL_VCC", "3-001c"),
	REGULATOR_SUPPLY("COMPASS_VCC", "3-000c"),
	REGULATOR_SUPPLY("hsusb_opt0", "uhhtll-omap"),
};
static struct regulator_init_data fixed_reg_vcc_initdata = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,		/* FIXME: I2C4 leaks into VCC! */
	},
	.consumer_supplies = fixed_reg_vcc_consumer,
	.num_consumer_supplies = ARRAY_SIZE(fixed_reg_vcc_consumer),
};
static struct fixed_voltage_config fixed_reg_vcc = {
	.supply_name	= "VCC",
	.microvolts	= 3300000,
	.gpio		= GPIO_VCC_PWRON,
	.enable_high	= 1,
	.enabled_at_boot= 1,
	.init_data	= &fixed_reg_vcc_initdata,
	.remux		= remux_regulator_gpio,
};
static struct platform_device fixed_supply_vcc = {
	.name 	= "reg-fixed-voltage",
	.id	= 2,
	.dev.platform_data = &fixed_reg_vcc,
};

/*
 * supply vbus_musb, consumer of 5V
 */

static struct regulator_consumer_supply fixed_reg_vbus_musb_consumer[] = {
	REGULATOR_SUPPLY("vbus_musb", "archos_usb_xceiv"),
	REGULATOR_SUPPLY("vbus_musb", "archos_twl6030_usb_xceiv"),
};
static struct regulator_init_data fixed_reg_vbus_musb_initdata = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},

	.supply_regulator 	= "5V",

	.consumer_supplies      = fixed_reg_vbus_musb_consumer,
	.num_consumer_supplies  = ARRAY_SIZE(fixed_reg_vbus_musb_consumer),
};

static struct fixed_voltage_config fixed_reg_vbus_musb = {
	.supply_name		= "vbus_musb",
	.microvolts		= 5000000,
	.gpio			= GPIO_VBUS_MUSB_PWRON,
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &fixed_reg_vbus_musb_initdata,
	.remux			= remux_regulator_gpio,
};

static struct platform_device fixed_supply_vbus_musb = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &fixed_reg_vbus_musb,
	},
};

/* MMC2 "virtual" regulators */
static struct regulator_consumer_supply board_vmmc2_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.1"),
};
static struct regulator_init_data board_vmmc2_initdata = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
	},
	.supply_regulator = "1V8",
	.consumer_supplies = board_vmmc2_supply,
	.num_consumer_supplies = ARRAY_SIZE(board_vmmc2_supply),
};
static struct fixed_voltage_config board_vmmc2 = {
	.supply_name		= "vmmc2",
	.microvolts		= 1800000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &board_vmmc2_initdata,
};
static struct platform_device fixed_supply_vmmc2 = {
	.name 	= "reg-fixed-voltage",
	.id	= 4,
	.dev.platform_data = &board_vmmc2,
};

static struct regulator_consumer_supply board_vmmc2_aux_supply[] = {
	REGULATOR_SUPPLY("vmmc_aux", "omap_hsmmc.1"),
};
static struct regulator_init_data fixed_reg_vmmc_aux_initdata = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS
	},
	.supply_regulator = "VCC",
	.consumer_supplies = board_vmmc2_aux_supply,
	.num_consumer_supplies = ARRAY_SIZE(board_vmmc2_aux_supply),
};
static struct fixed_voltage_config board_vmmc2_aux = {
	.supply_name		= "vmmc2_aux",
	.microvolts		= 3300000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &fixed_reg_vmmc_aux_initdata,
};
static struct platform_device fixed_supply_vmmc2_aux = {
	.name 	= "reg-fixed-voltage",
	.id	= 5,
	.dev.platform_data = &board_vmmc2_aux,
};

static struct regulator_consumer_supply board_vmmc4_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.3"),
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.2"),
};

static struct regulator_init_data board_vmmc4_initdata = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(board_vmmc4_supply),
	.consumer_supplies = board_vmmc4_supply,
};

static struct fixed_voltage_config board_vwlan = {
	.supply_name = "vwl1271",
	.microvolts = 1800000, /* 1.8V */
	.gpio = UNUSED_GPIO, /* init later */
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &board_vmmc4_initdata,
};
static struct platform_device board_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 6,
	.dev = {
		.platform_data = &board_vwlan,
               }
};

static struct regulator_consumer_supply fixed_reg_hdmi_5v_consumer[] = {
	REGULATOR_SUPPLY("hdmi_5v", "omapdss_hdmi"),
};

static struct regulator_init_data fixed_reg_hdmi_5v_initdata = {
	.constraints = {
		.min_uV 		= 5000000,
		.max_uV 		= 5000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask 	= REGULATOR_CHANGE_STATUS,
		.always_on 		= true,
	},
	.consumer_supplies = fixed_reg_hdmi_5v_consumer,
	.num_consumer_supplies = ARRAY_SIZE(fixed_reg_hdmi_5v_consumer),
};
static struct fixed_voltage_config fixed_reg_hdmi_5v = {
	.supply_name	= "hdmi_5v",
	.microvolts	= 5000000,
	.gpio		= UNUSED_GPIO,
	.enable_high	= 1,
	.init_data	= &fixed_reg_hdmi_5v_initdata,
	.remux		= remux_regulator_gpio,
};

static struct platform_device fixed_supply_hdmi_5v = {
	.name 	= "reg-fixed-voltage",
	.id	= 7,
	.dev.platform_data = &fixed_reg_hdmi_5v,
};

static struct regulator_consumer_supply hsusb_ext_consumer[] = {
	REGULATOR_SUPPLY("3g_5v", "archos_usb_3g"),
};

static struct regulator_init_data hsusb_ext_supply_initdata = {
	.constraints = {
		.min_uV 		= 5000000,
		.max_uV 		= 5000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask 	= REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = hsusb_ext_consumer,
	.num_consumer_supplies = ARRAY_SIZE(hsusb_ext_consumer),
};
static struct fixed_voltage_config hsusb_ext_supply_config = {
	.supply_name	= "hsusb_ext",
	.microvolts	= 5000000,
	.gpio		= UNUSED_GPIO,
	.enable_high	= 1,
	.init_data	= &hsusb_ext_supply_initdata,
	.remux		= remux_regulator_gpio,
};

static struct platform_device hsusb_ext_supply = {
	.name 	= "reg-fixed-voltage",
	.id	= 8,
	.dev.platform_data = &hsusb_ext_supply_config,
};

/*
 * VCC_HUB regulator
 */
static struct regulator_consumer_supply fixed_reg_hub_vcc_consumer[] = {
	REGULATOR_SUPPLY("3g_hub", "archos_usb_3g"),
	REGULATOR_SUPPLY("jm20329_hub", "jm20329"),
};
static struct regulator_init_data fixed_reg_hub_vcc_initdata = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.supply_regulator = "VCC",
	.consumer_supplies = fixed_reg_hub_vcc_consumer,
	.num_consumer_supplies = ARRAY_SIZE(fixed_reg_hub_vcc_consumer),
};
static struct fixed_voltage_config fixed_reg_hub_vcc = {
	.supply_name	= "HUB_VCC",
	.microvolts	= 3300000,
	.gpio		= UNUSED_GPIO,
	.enable_high	= 1,
	.enabled_at_boot= 1,
	.init_data	= &fixed_reg_hub_vcc_initdata,
	.remux		= remux_regulator_gpio,
};
static struct platform_device fixed_supply_hub_vcc = {
	.name 	= "reg-fixed-voltage",
	.id	= 23,
	.dev.platform_data = &fixed_reg_hub_vcc,
};

static struct archos_usb_config usb_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.enable_usb_ehci = 42,
		.enable_usb_hub = 60,
		.usb_hub_rst = UNUSED_GPIO,
		.enable_5v = UNUSED_GPIO,
	},
};

static struct archos_3g_config usb_3g_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.enable = 40,
	},
};

static struct archos_sata_config sata_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.sata_power = 135,
		.sata_ready = 137,
		.hdd_power  = 100,
		.hdd_power_mux = "gpmc_wait2.gpio_100",
	},
};

static struct archos_i2c_tsp_config i2c_tsp_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.irq_gpio = 112,
		.irq_signal = "abe_mcbsp2_dx.gpio_112",
		.pwr_gpio = 110,
		.shtdwn_gpio = 0,
		.shtdwn_signal = "kpd_col1.gpio_0",
	},
};

static struct omap_die_governor_config omap_die_governor_config __initdata = {
		.panic_temp = 95000,
		.alert_temp = 85000,
		.monitor_temp = 75000,
};

static struct archos_gps_config gps_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.gps_enable = 41,
		.gps_int    = UNUSED_GPIO,
		.gps_reset  = UNUSED_GPIO,
	},
};

static struct archos_wifi_bt_config board_wifi_bt_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.wifi_power = 103,
		.wifi_power_signal = "gpmc_ncs6.gpio_103",
		.wifi_irq   = 102,
		.wifi_irq_signal = "gpmc_ncs5.gpio_102",
		.bt_power   = 104,
		.fm_power   = UNUSED_GPIO,
		.gps_power  = UNUSED_GPIO,
	},
};

/* wl127x BT, FM, GPS connectivity chip */
static int platform_kim_suspend(struct platform_device *pdev, pm_message_t state);
static int platform_kim_resume(struct platform_device *pdev);
static int plat_kim_chip_enable(void);
static int plat_kim_chip_disable(void);
static int plat_kim_chip_awake(void);
static int plat_kim_chip_asleep(void);

static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = -1,
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = platform_kim_suspend,
	.resume = platform_kim_resume,
	.chip_enable = plat_kim_chip_enable,
	.chip_disable = plat_kim_chip_disable,
	.chip_awake = plat_kim_chip_awake,
	.chip_asleep = plat_kim_chip_asleep,
};

static bool uart_req;
static struct wake_lock st_wk_lock;
static int platform_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}
static int platform_kim_resume(struct platform_device *pdev)
{
	return 0;
}

static int plat_kim_chip_awake(void)
{
	wake_lock(&st_wk_lock);
	return 0;
}

static int plat_kim_chip_asleep(void)
{
	wake_unlock(&st_wk_lock);
	return 0;
}

static int plat_kim_chip_disable(void)
{
	int port_id = 0;
	int err = 0;

	printk(KERN_INFO"%s\n", __func__);

	if (uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_disable(port_id);
		if (!err)
			uart_req = false;
	}
	wake_unlock(&st_wk_lock);
	return err;
}

static int plat_kim_chip_enable(void)
{
	int port_id = 0;
	int err = 0;

	printk(KERN_INFO"%s\n", __func__);
	if (!uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_enable(port_id);
		if (!err)
			uart_req = true;
	}
	wake_lock(&st_wk_lock);
	if (err)
		return err;

	return 0;
}

static void board_wifi_mux_init(void)
{
	omap_mux_init_signal("mcspi4_simo.sdmmc4_cmd",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("mcspi4_clk.sdmmc4_clk",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("mcspi4_somi.sdmmc4_dat0",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("uart4_tx.sdmmc4_dat1",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("uart4_rx.sdmmc4_dat2",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("mcspi4_cs0.sdmmc4_dat3",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
}

static struct wl12xx_platform_data board_wlan_data __initdata = {
	.irq = -1, /* init later */
	.board_ref_clock = WL12XX_REFCLOCK_38_XTAL,
};

static int __init board_wifi_init(void)
{
	const struct archos_wifi_bt_dev_conf *conf_ptr;

	conf_ptr = hwrev_ptr(&board_wifi_bt_config, system_rev);
	if (IS_ERR(conf_ptr))
		return -EINVAL;

	/* bt */
	wilink_pdata.nshutdown_gpio = conf_ptr->bt_power;
	remux_regulator_gpio(conf_ptr->bt_power);

	/* wifi */
	if (conf_ptr->wifi_irq_signal)
		omap_mux_init_signal(conf_ptr->wifi_irq_signal, OMAP_PIN_INPUT);
	else
		omap_mux_init_gpio(conf_ptr->wifi_irq, OMAP_PIN_INPUT);

	board_wlan_data.irq = OMAP_GPIO_IRQ(conf_ptr->wifi_irq);

	if (conf_ptr->wifi_power_signal)
		omap_mux_init_signal(conf_ptr->wifi_power_signal, OMAP_PIN_OUTPUT);
	else
		omap_mux_init_gpio(conf_ptr->wifi_power, OMAP_PIN_OUTPUT);

	board_vwlan.gpio = conf_ptr->wifi_power;

	board_wifi_mux_init();
	if (wl12xx_set_platform_data(&board_wlan_data))
		pr_err("Error setting wl12xx data\n");
	platform_device_register(&board_vwlan_device);

	return 0;
}

static struct platform_device wifi_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};
static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

// camera
static struct i2c_board_info __initdata board_i2c_2_boardinfo[] = {
// FIXME

};

// accel compas
static struct i2c_board_info __initdata board_i2c_3_boardinfo[] = {
	{
		I2C_BOARD_INFO("mma8453q", 0x1c),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_mma8453q_pdata,
	},
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_akm8975_pdata,
	},
// FIXME

};

static struct pixcir_platform_data board_pixcir_pdata= {
	.flags = PIXCIR_FLAGS_XY_SWAP,
	.x_scale = 20,	// +2.0%
	.y_scale = 20,	// +2.0%
};

// tsp
static struct i2c_board_info __initdata pixcir_tsp_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO(PIXCIR_NAME, PIXCIR_ADDR),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_pixcir_pdata,
	},
};

static struct tr16c0_platform_data board_tr16c0_pdata = {
	.x_max = 1280,
	.y_max = 800,
};

static struct i2c_board_info __initdata tr16c0_tsp_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO(TR16C0_NAME, TR16C0_ADDR),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_tr16c0_pdata,
	},
};

static struct cpt_i2c_tsp_platform_data board_cpt_i2c_tsp_pdata = {
	.x_max = 1280,
	.y_max = 800,
	.x_offset = 0,
	.x_scale = 0,
	.y_offset = 0,
	.y_scale = 0,
};

static struct i2c_board_info __initdata cpt_tsp_i2c_boardinfo [] = {
	{
		I2C_BOARD_INFO(CPT_I2C_TSP_NAME, CPT_I2C_TSP_ADDR),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_cpt_i2c_tsp_pdata,
	},
};

static struct archos_accel_config accel_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.accel_int1 = 45,
		.accel_int2 = UNUSED_GPIO,
	},
	.rev[1 ... 5] = {
		.accel_int1 = 45,
		.accel_int2 = 174,
	},
};

static struct archos_compass_config compass_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.data_ready = 51,
	},
};

static struct archos_audio_twl6040_config audio_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.power_on = 127,
	},
};

static struct archos_charge_config charge_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type	= CHARGER_TWL6030USB_DC,
	},
};

static struct archos_display_config display_config __initdata = {
	.nrev = 6,
	.rev[0 ... 3] = {
		.lcd_pwon = 	38 ,
		.lcd_rst = 	UNUSED_GPIO ,
		.lcd_pci = 	UNUSED_GPIO,
		.lvds_en = 	39 ,
		.lcd_stdby = 	101,
		.lcd_avdd_en = 	12 ,
		.hdmi_pwr = 	37 ,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.bkl_en = 	122,
		.bkl_pwr =	13,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
		.do_not_use_pll = 1,
	},
	.rev[4 ... 5] = {
		.lcd_pwon = 	38 ,
		.lcd_rst = 	UNUSED_GPIO ,
		.lcd_pci = 	UNUSED_GPIO,
		.lvds_en = 	39 ,
		.lcd_stdby = 	101,
		.lcd_avdd_en = 	12 ,
		.hdmi_pwr = 	37 ,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{ .timer = -1, .mux_cfg = -1 },
		.bkl_en = 	122,
		.bkl_pwr =	13,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
};

static struct archos_leds_config leds_config __initdata = {
	.nrev = 6,
	.rev[0 ... 1] = {
		.power_led = UNUSED_GPIO,
		.status_led = UNUSED_GPIO,
		.backlight_led = { .timer = -1 },
		.bkl_invert = 1,
		.backlight_power = 13,
		.bkl_freq = 200,
	},
	.rev[2 ... 5] = {
		.power_led = UNUSED_GPIO,
		.status_led = UNUSED_GPIO,
		.backlight_led = {
			.src = OMAP_DM_PWM,
			.timer = 8,
			.signal = "uart3_rx_irrx.dmtimer8_pwm_evt",
			.signal_off = "uart3_rx_irrx.safe_mode",
		},
		.bkl_invert = 1,
		.backlight_power = 13,
		.bkl_freq = 17000,
		.bkl_max = 189,
		.bkl_min = 20,
	},
};

static struct archos_musb_config musb_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.gpio_vbus_detect = 48,
		.gpio_vbus_flag = 113,
		.gpio_id = 49,
	},
};

static struct archos_camera_config camera_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.pwr_down = UNUSED_GPIO,
		.reset = 62,
	},
};

static struct gpio_keys_button gpio_volume_buttons[] = {
	{
		.code			= KEY_VOLUMEUP,
		.desc			= "volume up",
		.wakeup			= GPIO_KEY_AWAKE_ONLY,
		.active_low		= 1,
	},
	{
		.code			= KEY_VOLUMEDOWN,
		.desc			= "volume down",
		.wakeup			= GPIO_KEY_AWAKE_ONLY,
		.active_low		= 1,
	},
};

static struct gpio_keys_platform_data gpio_volume_keys_info = {
	.buttons	= gpio_volume_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_volume_buttons),
};

static struct platform_device volume_keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_volume_keys_info,
	},
};

static struct archos_usb_gadget_config gadget_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.product_name = "A101H",
		.product_id = 0x1520,
		.ums_luns = 2,
	},
};

static struct led_pwm pwm_leds[] = {
	{
		.name = "power",
		.default_trigger = "default-on",
		.pwm_id = 1,
		.max_brightness = 255,
		.pwm_period_ns = 7812500,
	},
};

static struct led_pwm_platform_data archos_pwm_led_data = {
	.num_leds = ARRAY_SIZE(pwm_leds),
	.leds = pwm_leds,
};

static struct platform_device archos_pwm_leds = {
	.name	= "leds_pwm",
	.id	= -1,
	.dev	= {
		.platform_data = &archos_pwm_led_data,
	},
};

static struct omap_board_config_kernel board_config[] __initdata = {
	{ ARCHOS_TAG_AUDIO_TWL6040,	&audio_config},
	{ ARCHOS_TAG_ACCEL,		&accel_config},
	{ ARCHOS_TAG_COMPASS,		&compass_config},
	{ ARCHOS_TAG_CHARGE,		&charge_config},
	{ ARCHOS_TAG_DISPLAY,		&display_config},
	{ ARCHOS_TAG_LEDS,		&leds_config},
	{ ARCHOS_TAG_WIFI_BT,		&board_wifi_bt_config},
	{ ARCHOS_TAG_MUSB,		&musb_config},
	{ ARCHOS_TAG_GPS,		&gps_config},
	{ ARCHOS_TAG_CAMERA,		&camera_config},
	{ ARCHOS_TAG_USB_GADGET,	&gadget_config},
	{ ARCHOS_TAG_USB,		&usb_config},
	{ ARCHOS_TAG_3G,		&usb_3g_config},
	{ ARCHOS_TAG_SATA,              &sata_config},
	{ ARCHOS_TAG_I2C_TSP,		&i2c_tsp_config},
	{ OMAP_TAG_DIE_GOVERNOR,	&omap_die_governor_config},
};

#ifdef CONFIG_OMAP4_DSS_HDMI

static struct omap_dss_device archos_4430_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.hpd_gpio = HDMI_GPIO_HPD,
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},
		.hdmi	= {
			.regn	= 7,
			.regm2	= 1,
		},
	},
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

#endif /* CONFIG_OMAP4_DSS_HDMI */

static __init int archos_hdmi_init(void)
{
	u32 r;
	const struct archos_disp_conf *conf;

	conf = hwrev_ptr(&display_config, system_rev);
	if (IS_ERR(conf))
		return -ENODEV;

	omap_mux_init_signal("hdmi_hpd", OMAP_PIN_INPUT);
	omap_mux_init_signal("hdmi_cec", OMAP_PIN_INPUT);
	omap_mux_init_signal("hdmi_ddc_scl", OMAP_PIN_INPUT);
	omap_mux_init_signal("hdmi_ddc_sda", OMAP_PIN_INPUT);

	/* strong pullup on DDC lines using unpublished register */
	r = ((1 << 24) | (1 << 28)) ;
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_1);

	gpio_request(HDMI_GPIO_HPD, "hdmi_hpd");
	omap_mux_init_gpio(HDMI_GPIO_HPD, OMAP_PIN_INPUT_PULLDOWN);
	gpio_direction_input(HDMI_GPIO_HPD);

	fixed_reg_hdmi_5v.gpio = conf->hdmi_pwr;

	if (platform_device_register(&fixed_supply_hdmi_5v) < 0) {
		pr_err("Error registering hdmi 5V regulator\n");
		return -1;
	}

	return 0;
}

static struct omap_dss_device board_lcd_device;

static struct omap_dss_device *board_dss_devices[] = {
	&board_lcd_device,
#ifdef CONFIG_OMAP4_DSS_HDMI
	&archos_4430_hdmi_device,
#endif /* CONFIG_OMAP4_DSS_HDMI */
};

static struct omap_dss_board_info board_dss_data = {
	.num_devices	=	ARRAY_SIZE(board_dss_devices),
	.devices	=	board_dss_devices,
#ifdef CONFIG_OMAP4_DSS_HDMI
	.default_device	=	&archos_4430_hdmi_device,
#else
	.default_device	=	NULL,
#endif /* CONFIG_OMAP4_DSS_HDMI */
};

static struct platform_device *a101h_devices[] __initdata = {
	&wifi_device,
	&btwilink_device,
	&fixed_supply_5v,
	&fixed_supply_1v8,
	&fixed_supply_vcc,
	&fixed_supply_vbus_musb,
	&fixed_supply_vmmc2,
	&fixed_supply_vmmc2_aux,
	&archos_pwm_leds,
};

static __init int archos_hsusb_ext_regulator_init(void)
{
	const struct archos_3g_conf *conf;
	conf = hwrev_ptr(&usb_3g_config, system_rev);
	if (IS_ERR(conf))
		return -EINVAL;

	hsusb_ext_supply_config.gpio = conf->enable;

	if (platform_device_register(&hsusb_ext_supply) < 0) {
		pr_err("Error registering 5V regulator of external hsusb\n");
		return -1;
	}

	return 0;
}

static __init int archos_hub_vcc_regulator_init(void)
{
	const struct archos_usb_conf *conf;
	conf = hwrev_ptr(&usb_config, system_rev);
	if (IS_ERR(conf))
		return -EINVAL;

	if (!gpio_is_valid(conf->enable_usb_hub))
		return -ENODEV;

	fixed_reg_hub_vcc.gpio = conf->enable_usb_hub;

	if (platform_device_register(&fixed_supply_hub_vcc) < 0) {
		pr_err("Error registering USB HUB regulator\n");
	}

	return 0;
}

static int __init omap4_leds_init(void)
{
	return 0;
}

static void __init board_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode			= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 500,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc            = 2,
		.name		= "internal",
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd        = -EINVAL,
		.gpio_wp        = -EINVAL,
		.ocr_mask       = MMC_VDD_165_195,
		.no_off_init	= true,
		.nonremovable	= true,
#ifdef CONFIG_PM_RUNTIME
		.power_saving   = true,
#endif
		//.vcc_aux_disable_is_sleep = true,
	},
	{
		.mmc		= 4,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},
	{}	/* Terminator */
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	return 0;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev)
		return;

	pdata = dev->platform_data;
	pdata->init = omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static int __init board_vaux3_init(void* driver_data)
{
	/*
	 * Clear group associations for VAUX3 in VAUX3_CFG_GRP
	 */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0, 0x8C);

	return 0;
}

static struct regulator_init_data board_vaux2 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct regulator_consumer_supply board_vaux3_supply[] = {
	REGULATOR_SUPPLY("vaux3", "vibrator"),
};

static struct regulator_init_data board_vaux3 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= false,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies	= ARRAY_SIZE(board_vaux3_supply),
	.consumer_supplies = board_vaux3_supply,
	.regulator_init = board_vaux3_init,
};

static struct regulator_consumer_supply board_vmmc1_supply[] = {
};

static struct regulator_init_data board_vmmc1 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
	.num_consumer_supplies  = ARRAY_SIZE(board_vmmc1_supply),
	.consumer_supplies      = board_vmmc1_supply,
};

static struct regulator_init_data board_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data board_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_consumer_supply board_vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi2"),
};

static struct regulator_init_data board_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies	= ARRAY_SIZE(board_vcxio_supply),
	.consumer_supplies	= board_vcxio_supply,
};

static struct regulator_init_data board_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 	= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data board_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= uart3_on_usb,
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
	},
};

static struct twl4030_usb_data board_twl6030_usb_data = {
	.platform 	= &archos_vbus_info,
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
	.phy_suspend	= omap4430_phy_suspend,
	.name = "archos_usb_xceiv",
};

static struct twl4030_codec_audio_data twl6040_audio = {
};

static struct twl4030_codec_data twl6040_codec = {
	.audio_mclk	= 19200000,
	.audio	= &twl6040_audio,
	.naudint_irq    = OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};

static struct twl4030_madc_platform_data board_gpadc_data = {
	.irq_line	= 1,
};

static int board_batt_table[] = {
	/* Sensicom CN0603R473B3750FB 47k B=3750  Rx=10k Ry=220k */
	/* adc code for temperature in degree C */
	925, 923,  /* -2 ,-1 */
	920, 917, 914, 911, 908, 904, 901, 898, 894, 890,  /* 00 - 09 */
	886, 882, 878, 874, 870, 865, 861, 856, 851, 846,  /* 10 - 19 */
	841, 836, 830, 825, 819, 813, 807, 801, 795, 789,  /* 20 - 29 */
	783, 776, 770, 763, 756, 749, 742, 735, 728, 721,  /* 30 - 39 */
	713, 706, 698, 691, 683, 675, 668, 660, 652, 644,  /* 40 - 49 */
	636, 628, 620, 612, 604, 596, 587, 579, 571, 563,  /* 50 - 59 */
	555, 547, 539,  /* 60 - 62 */
};

static struct twl4030_bci_platform_data board_bci_data = {
	.monitoring_interval		= 10,
	.max_charger_currentmA		= 1500,
	.max_charger_voltagemV		= 4560,
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= board_batt_table,
	.tblsize			= ARRAY_SIZE(board_batt_table),
};

static struct regulator_init_data board_clk32kg = {
       .constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
		.always_on		= true,
       },
};

static struct twl4030_platform_data board_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &board_vmmc1,
	.vpp		= &board_vpp,
	.vana		= &board_vana,
	.vcxio		= &board_vcxio,
	.vdac		= &board_vdac,
	.vusb		= &board_vusb,
	.vaux2		= &board_vaux2,
	.vaux3		= &board_vaux3,
	.madc           = &board_gpadc_data,
	.bci            = &board_bci_data,

	/* children */
	.codec          = &twl6040_codec,
	.usb            = &board_twl6030_usb_data,
	.clk32kg	= &board_clk32kg,
};

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
				struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request();
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
								bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata board_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata board_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata board_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata board_i2c_4_bus_pdata;

#define GPIO_MSECURE	30
static int __init omap4_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &board_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &board_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &board_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &board_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &board_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &board_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &board_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &board_i2c_4_bus_pdata);

	omap4_pmic_init("twl6030", &board_twldata);
	omap_register_i2c_bus(2, 100, board_i2c_2_boardinfo, 
			ARRAY_SIZE(board_i2c_2_boardinfo)); // FIXME speed
	omap_register_i2c_bus(3, 100, board_i2c_3_boardinfo, 
			ARRAY_SIZE(board_i2c_3_boardinfo)); // FIXME speed
	omap_register_i2c_bus(4, 400, NULL, 0);

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	/* To access twl registers we enable gpio_wk30
	 * we need this so the RTC driver can work.
	 */
	gpio_request(GPIO_MSECURE, "MSECURE");
	gpio_direction_output(GPIO_MSECURE, 1);

	omap_mux_init_gpio(GPIO_MSECURE, \
		OMAP_PIN_OUTPUT | OMAP_PIN_OFF_NONE);

	return 0;
}

static void enable_board_wakeup_source(void)
{
	/*
	 * Enable IO daisy for sys_nirq1/2, to be able to
	 * wakeup from interrupts from PMIC/Audio IC.
	 * Needed only in Device OFF mode.
	 */
	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
			OMAP_WAKEUP_EN);
	omap_mux_init_signal("sys_nirq2", OMAP_PIN_INPUT_PULLUP);
}


static bool enable_suspend_off = false;
module_param(enable_suspend_off, bool, S_IRUSR | S_IRGRP | S_IROTH);

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP4_MUX(UART3_RX_IRRX, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE7),
	OMAP4_MUX(UART3_TX_IRTX, OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct omap_device_pad tablet_uart1_pads[] __initdata = {
	{	/* UART1 TX on uart3_cts_rctx */
		.name	= "uart3_cts_rctx.uart3_cts_rctx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{ 	/* UART1 RX on mcspi1_cs1 */
		.name	= "mcspi1_cs1.mcspi1_cs1",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE1,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE1,
	},
};


static struct omap_device_pad tablet_uart2_pads[] __initdata = {
	{
		.name	= "uart2_cts.uart2_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.flags  = OMAP_DEVICE_PAD_REMUX,
		.idle   = OMAP_WAKEUP_EN | OMAP_PIN_OFF_INPUT_PULLUP |
			  OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rts.uart2_rts",
		.flags  = OMAP_DEVICE_PAD_REMUX,
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
		.idle   = OMAP_PIN_OFF_INPUT_PULLUP | OMAP_MUX_MODE7,
	},
	{
		.name	= "uart2_tx.uart2_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rx.uart2_rx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad tablet_uart3_pads[] __initdata = {
	{	/* UART3 TX on usba0_otg_dm */
		.name	= "usba0_otg_dm.usba0_otg_dm",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{	/* UART3 RX on usba0_otg_dp */
		.name	= "usba0_otg_dp.usba0_otg_dp",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE1,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE1,
	},
};

static struct omap_device_pad no_uart_pads[] __initdata = {
		/* empty structure */
};

static struct omap_uart_port_info tablet_uart_info_uncon __initdata = {
	.use_dma	= 0,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
        .wer = 0,
};

static struct omap_uart_port_info tablet_uart_info __initdata = {
	.use_dma	= 0,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
        .wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
};

static struct omap_uart_port_info tablet_uart_info_nocts __initdata = {
	.use_dma	= 0,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
        .wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX ),
};

static inline void __init board_serial_init(void)
{
	omap_serial_init_port_pads(0, tablet_uart1_pads,
		ARRAY_SIZE(tablet_uart1_pads), &tablet_uart_info_nocts);
	omap_serial_init_port_pads(1, tablet_uart2_pads,
		ARRAY_SIZE(tablet_uart2_pads), &tablet_uart_info);

	/* UART3 only for debug console on OTG DP/DM pins */
	if (uart3_on_usb) {
		omap_serial_init_port_pads(2, tablet_uart3_pads,
			ARRAY_SIZE(tablet_uart3_pads), &tablet_uart_info_nocts);
		/* set usb-otg phy to gpio mode */
		omap4_ctrl_pad_writel((1UL << 29),
				OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_USB2PHYCORE);
	} else
		omap_serial_init_port_pads(2, no_uart_pads,
				0, &tablet_uart_info_uncon);

}

static void __init board_touch_init(void)
{
	struct feature_tag_touchscreen * tsp;

	/* check which touch screen we have */
	tsp = get_feature_tag(FTAG_HAS_TOUCHSCREEN, 
			feature_tag_size(feature_tag_touchscreen));
	if (!tsp || (tsp->vendor == 0)) {
		i2c_register_board_info(4, pixcir_tsp_i2c_boardinfo, 1);
		archos_touchscreen_pixcir_init(&board_pixcir_pdata);
	} else if (tsp->vendor == 1) {
		i2c_register_board_info(4, tr16c0_tsp_i2c_boardinfo, 1);
		archos_touchscreen_tr16c0_init(&board_tr16c0_pdata);
	} else {
		i2c_register_board_info(4, cpt_tsp_i2c_boardinfo, 1);
		archos_touchscreen_cpt_i2c_tsp_init(&board_cpt_i2c_tsp_pdata);
	}
}

static void __init board_buttons_init(void)
{
	struct feature_tag_gpio_volume_keys * volume_keys;

	if ((volume_keys = get_feature_tag(FTAG_HAS_GPIO_VOLUME_KEYS,
					feature_tag_size(feature_tag_gpio_volume_keys)))) {

		gpio_volume_buttons[0].gpio = volume_keys->gpio_vol_up;
		gpio_volume_buttons[1].gpio = volume_keys->gpio_vol_down;

		omap_mux_init_gpio(gpio_volume_buttons[0].gpio, OMAP_PIN_INPUT);
		omap_mux_init_gpio(gpio_volume_buttons[1].gpio, OMAP_PIN_INPUT);
		
		platform_device_register(&volume_keys_gpio);
	}
}

static void omap_board_display_init(void)
{
	if (panel_auo_wxga_10_init(&board_lcd_device) == 0)
		board_dss_data.default_device = &board_lcd_device;

	archos_hdmi_init();

	omap_display_init(&board_dss_data);
}

static void board_set_osc_timings(void)
{
	/* Device Oscilator
	 * tstart = 2ms + 2ms = 4ms.
	 * tshut = Not defined in oscillator data sheet so setting to 1us
	 * FIXME: check our oscillator data sheet and match numbers
	 */
	omap_pm_set_osc_lp_time(4000, 1);
}


static void __init board_init(void)
{
	int package = OMAP_PACKAGE_CBS;
	int status;

	omap_board_config = board_config;
	omap_board_config_size = ARRAY_SIZE(board_config);

	board_set_osc_timings();

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, NULL, package);

	archos_memory_init();

	omap4_i2c_init();
	board_buttons_init();

	archos_usb_musb_init(&archos_vbus_info);
	archos_audio_twl6040_init(&twl6040_codec);
	archos_accel_mma8453q_init(&board_mma8453q_pdata);
	archos_compass_init(&board_akm8975_pdata);
	archos_battery_twl4030_bci_init(&board_bci_data);

	omap4_leds_init();

	platform_add_devices(a101h_devices, ARRAY_SIZE(a101h_devices));

	board_touch_init();
	wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");
	board_serial_init();
	board_wifi_init();

	usb_musb_init(&musb_board_data);

	omap4_twl6030_hsmmc_init(mmc);

	archos_omap4_ehci_init();
	archos_camera_mt9m114_init();

	omap_dmm_init();
	omap_board_display_init();

	archos_hsusb_ext_regulator_init();
	archos_hub_vcc_regulator_init();

	enable_board_wakeup_source();

	hardware_comp.tps62361 = 1;

	/* Vsel0 = gpio, vsel1 = gnd */
	status = omap_tps6236x_board_setup(true, TPS62361_GPIO, -1,
				OMAP_PIN_OFF_OUTPUT_HIGH, -1);
	if (status)
		pr_err("TPS62361 initialization failed: %d\n", status);

	omap_enable_smartreflex_on_init();

	if (omap_rev() >= OMAP4430_REV_ES2_3)
		if (enable_suspend_off)
			omap_pm_enable_off_mode();
}

static void __init board_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

MACHINE_START(ARCHOS_A101H, "ARCHOS A101H board")
	.boot_params	= 0x80000100,
	.reserve	= archos_reserve,
	.map_io		= board_map_io,
	.fixup		= fixup_archos,
	.init_early	= board_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= board_init,
	.timer		= &omap_timer,
MACHINE_END
