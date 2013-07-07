#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/ion.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/mfd/twl6040.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/platform_data/omap-abe-twl6040.h>
#include <linux/omap4_duty_cycle_governor.h>
#include <linux/omap_die_governor.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/wl12xx.h>

#include <plat/android-display.h>
#include <video/omapdss.h>

#include <mach/hardware.h>
#include <mach/omap-secure.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/feature_list.h>

#include <linux/gpio_keys.h>
#include <plat/gpio.h>

#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/delay.h>

#include <mach/board-archos.h>
#include <linux/usb/gpio_vbus.h>

// Fuel Gauge
#include <linux/power/ti-fg.h>


#include <plat/drm.h>
#include <plat/board.h>
#include "common.h"
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/remoteproc.h>
#include <plat/omap_apps_brd_id.h>

#include <plat/archos-audio-twl6040.h>

#include "hsmmc.h"
#include "control.h"
#include "mux.h"
#include "common-board-devices.h"
#include "pm.h"

#include "omap4_ion.h"
#include "omap_ram_console.h"

static struct gpio_vbus_mach_info archos_vbus_info;

static struct mma8453q_pdata board_mma8453q_pdata;
static struct akm8975_platform_data board_akm8975_pdata;

#define TPS62361_GPIO   7

#define GPIO_5V_PWRON            36	/* fixme: from config tags? */
#define GPIO_1V8_PWRON           34	/* fixme: from config tags? */
#define GPIO_VCC_PWRON           35	/* fixme: from config tags? */
#define GPIO_VBUS_MUSB_PWRON    111	/* fixme: from config tags? */

#define HDMI_GPIO_CT_CP_HPD 60 /* HPD mode enable/disable */
#define HDMI_GPIO_LS_OE 41 /* Level shifter for HDMI, default 41 - which is reserved by archos for gps */
#define HDMI_GPIO_HPD  63 /* Hotplug detect */

#define GPIO_KEY_AWAKE_ONLY	0x02

#define WILINK_UART_DEV_NAME "/dev/ttyO1"

static void remux_regulator_gpio(int gpio)
{
	printk(">>%s:%s: GPIO: %d\n",__FILE__,__FUNCTION__,gpio);
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
	printk("<<%s:%s\n",__FILE__,__FUNCTION__);
}


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
static int plat_kim_chip_enable(struct kim_data_s *);
static int plat_kim_chip_disable(struct kim_data_s *);
static int plat_kim_chip_awake(struct kim_data_s *);
static int plat_kim_chip_asleep(struct kim_data_s *);

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

static int plat_kim_chip_awake(struct kim_data_s *kim_data)
{
	wake_lock(&st_wk_lock);
	return 0;
}

static int plat_kim_chip_asleep(struct kim_data_s *kim_data)
{
	wake_unlock(&st_wk_lock);
	return 0;
}

extern int omap_serial_ext_uart_enable(u8 port_id);
extern int omap_serial_ext_uart_disable(u8 port_id);

static int plat_kim_chip_disable(struct kim_data_s *kim_data)
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

static int plat_kim_chip_enable(struct kim_data_s *kim_data)
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

static struct cypress_tma340_platform_data board_tma340_pdata = {
	.flags = CYPRESS_TMA340_FLAGS_INV_Y,
};

// tsp
static struct i2c_board_info __initdata cypress_tsp_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO(CYPRESS_TMA340_NAME, CYPRESS_TMA340_ADDR),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_tma340_pdata,
	},
};

static struct tr16c0_platform_data board_tr16c0_pdata = {
	.x_max = 800,
	.y_max = 600,
};

static struct archos_gps_config gps_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.gps_enable = 41,
		.gps_int    = UNUSED_GPIO,
		.gps_reset  = UNUSED_GPIO,
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

static struct archos_camera_config camera_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.pwr_down = UNUSED_GPIO,
		.reset = 62,
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


static struct i2c_board_info __initdata tr16c0_tsp_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO(TR16C0_NAME, TR16C0_ADDR),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_tr16c0_pdata,
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

static struct archos_charge_config charge_config __initdata = {
	.nrev = 6,
	.rev[0] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type	= CHARGER_TWL6030USB_DC,
	},
	.rev[1 ... 5] = {
		.charge_enable  = UNUSED_GPIO,
		.charge_high    = UNUSED_GPIO,
		.charge_low     = UNUSED_GPIO,
		.charger_type	= CHARGER_TWL6030USB,
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

static struct regulator_consumer_supply fixed_reg_5v_consumer[] = {
	REGULATOR_SUPPLY("hsusb_vbus0", "uhhtll-omap"),
	REGULATOR_SUPPLY("5V", "vbus_musb"),
	REGULATOR_SUPPLY("5V", "omap_pwm_led.0"),
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

static int __init board_wifi_init(void)
{
	const struct archos_wifi_bt_dev_conf *conf_ptr;
	printk(">>%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
	conf_ptr = hwrev_ptr(&board_wifi_bt_config, system_rev);
	if (IS_ERR(conf_ptr)){
		printk(KERN_ERR "No valid hw revision found for wifi init()\n");
		return -EINVAL;
	}

	/* bt */
	wilink_pdata.nshutdown_gpio = conf_ptr->bt_power;
	remux_regulator_gpio(conf_ptr->bt_power);

	/* wifi */
	if (conf_ptr->wifi_irq_signal)
		omap_mux_init_signal(conf_ptr->wifi_irq_signal, OMAP_PIN_INPUT);
	else
		omap_mux_init_gpio(conf_ptr->wifi_irq, OMAP_PIN_INPUT);

	board_wlan_data.irq = gpio_to_irq(conf_ptr->wifi_irq);

	if (conf_ptr->wifi_power_signal){
		printk("%s: Register wifi_power_signal\n",__FUNCTION__);
		omap_mux_init_signal(conf_ptr->wifi_power_signal, OMAP_PIN_OUTPUT);
	}
	else
		omap_mux_init_gpio(conf_ptr->wifi_power, OMAP_PIN_OUTPUT);

	board_vwlan.gpio = conf_ptr->wifi_power;

	board_wifi_mux_init();
	if (wl12xx_set_platform_data(&board_wlan_data))
		pr_err("Error setting wl12xx data\n");
	platform_device_register(&board_vwlan_device);

	printk("<<%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
	return 0;
}

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

static struct archos_usb_config usb_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.enable_usb_ehci = 42,
		.enable_usb_hub = UNUSED_GPIO,
		.usb_hub_rst = UNUSED_GPIO,
		.enable_5v = UNUSED_GPIO,
	},
};

static struct archos_3g_config usb_3g_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.enable = 40,
		.nreset = UNUSED_GPIO,
		.nenable = UNUSED_GPIO,
		.nsimdet = UNUSED_GPIO,
		.wake = UNUSED_GPIO,
		.wakehost = UNUSED_GPIO,
		.nwakedisable = UNUSED_GPIO,
	},
};

static struct archos_usb_gadget_config gadget_config __initdata = {
	.nrev = 6,
	.rev[0 ... 5] = {
		.product_name = "A80S",
		.product_id = 0x1500,
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
#ifdef ADD_VCOM_LED_CTRL_FOR_VCOM_TWEAKING
	{
		.name = "vcom",
		.pwm_id = 2,
		.max_brightness = 254,
		// meaningless : pmic's pwms speed not configurable
		.pwm_period_ns = 64,
	},
#endif
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

static struct omap_abe_twl6040_data archos_abe_audio_data = {
	/* Audio out */
	.has_hs		= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	/* HandsFree through expasion connector */
	.has_hf		= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	/* PandaBoard: FM TX, PandaBoardES: can be connected to audio out */
	.has_aux	= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	/* PandaBoard: FM RX, PandaBoardES: audio in */
	.has_afm	= ABE_TWL6040_LEFT | ABE_TWL6040_RIGHT,
	.has_abe	= 1,
	/* No jack detection. */
	.jack_detection	= 0,
	/* MCLK input is 19.2MHz */
	.mclk_freq	= 19200000,

	// Fix to a board
	.card_name = "ArchosBoard",

};

static struct platform_device archos_abe_audio = {
	.name		= "omap-abe-twl6040",
	.id		= -1,
	.dev = {
		.platform_data = &archos_abe_audio_data,
	},
};

static struct twl4030_usb_data board_twl6030_usb_data = {
	.platform = &archos_vbus_info,
//	.phy_init	= omap4430_phy_init,
//	.phy_exit	= omap4430_phy_exit,
//	.phy_power	= omap4430_phy_power,
//	.phy_set_clock	= omap4430_phy_set_clk,
//	.phy_suspend	= omap4430_phy_suspend,
	.name = "archos_twl6030_usb_xceiv",
};


static struct twl6040_codec_data twl6040_codec = {
	/* single-step ramp for headset and handsfree */
	.hs_left_step	= 0x0f,
	.hs_right_step	= 0x0f,
	.hf_left_step	= 0x1d,
	.hf_right_step	= 0x1d,
};

static struct twl6040_platform_data twl6040_data = {
	.codec		= &twl6040_codec,
	.audpwron_gpio	= 127,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};

#ifdef CONFIG_OMAP4_DSS_HDMI
static struct omap_dss_hdmi_data omap4_panda_hdmi_data = {
	.hpd_gpio = HDMI_GPIO_HPD,
	.ls_oe_gpio = HDMI_GPIO_LS_OE,
	.ct_cp_hpd_gpio = HDMI_GPIO_CT_CP_HPD,
};

static struct omap_dss_device archos_4430_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.data = &omap4_panda_hdmi_data,
//	.cec_auto_switch = 1,
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

static struct archos_display_config display_config __initdata = {
	.nrev = 6,
	.rev[0 ... 2] = {
		.lcd_pwon = 	38,
		.lcd_rst = 	53,
		.lcd_pci = 	UNUSED_GPIO,
		.bridge_en = 	39,
		.lcd_stdby = 	101,
		.lcd_avdd_en = 	12,
		.hdmi_pwr = 	37,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{
			.src = OMAP_DM_PWM,
			.timer = 11,
			.signal ="abe_dmic_din2.dmtimer11_pwm_evt",
		},
		.bkl_en = 	122,
		.bkl_pwr =	UNUSED_GPIO,
		.use_fixed_bkl = 1,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
		.do_not_use_pll = 1,
	},
	.rev[3] = {
		.lcd_pwon = 	38 ,
		.lcd_rst = 	53 ,
		.lcd_pci = 	UNUSED_GPIO,
		.bridge_en = 	39 ,
		.lcd_stdby = 	101,
		.lcd_avdd_en = 	12 ,
		.hdmi_pwr = 	37 ,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{
			.src = TWL6030_PWM,
			.timer = 2 // Pmic's PWM1 -> id = 2.
		},
		.bkl_en = 	122,
		.bkl_pwr =	UNUSED_GPIO,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
		.do_not_use_pll = 1,
	},
	.rev[4 ... 5] = {
		.lcd_pwon = 	38 ,
		.lcd_rst = 	53 ,
		.lcd_pci = 	UNUSED_GPIO,
		.bridge_en = 	39 ,
		.lcd_stdby = 	101,
		.lcd_avdd_en = 	12 ,
		.hdmi_pwr = 	37 ,
		.hdmi_int = 	UNUSED_GPIO,
		.vcom_pwm = 	{
			.src = TWL6030_PWM,
			.timer = 2 // Pmic's PWM1 -> id = 2.
		},
		.bkl_en = 	122,
		.bkl_pwr =	UNUSED_GPIO,
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
};

/* Allocate ( 18 + 9 ) MB for TILER1D slot size for WUXGA panel, total of
 * 54 MB of TILER1D
 */
static struct dsscomp_platform_data dsscomp_config_cpt_xga8_wuxga = {
	.tiler1d_slotsz = SZ_16M,
};

static struct dsscomp_platform_data dsscomp_config_hdmi_display = {
	.tiler1d_slotsz = (SZ_16M + SZ_2M + SZ_8M + SZ_1M),
};

/* HACK: use 2 devices, as expected by DDK */
static struct sgx_omaplfb_config omaplfb_plat_data_cpt_xga8_wuxga[] = {
//	{
//		.tiler2d_buffers = 2,
//		.swap_chain_length = 2,
//	},
//	{
//		.vram_buffers = 2,
//		.swap_chain_length = 2,
//	},
	{
		.vram_buffers = 2,
		.swap_chain_length = 2,
	},
	{
		.vram_buffers = 2,
		.swap_chain_length = 2,
	}
};

static struct sgx_omaplfb_config omaplfb_config_hdmi_default_display[] = {
	{
		.vram_buffers = 2,
		.swap_chain_length = 2,
	},
	{
		.vram_buffers = 2,
		.swap_chain_length = 2,
	}
};

static struct omapfb_platform_data tablet_fb_pdata = {
	.mem_desc = {
		.region_cnt = ARRAY_SIZE(omaplfb_plat_data_cpt_xga8_wuxga),
	},
};

static struct sgx_omaplfb_platform_data omaplfb_plat_data_tablet = {
	.num_configs = ARRAY_SIZE(omaplfb_plat_data_cpt_xga8_wuxga),
	.configs = omaplfb_plat_data_cpt_xga8_wuxga,
};

static struct sgx_omaplfb_platform_data omaplfb_plat_data_hdmi_default_display = {
	.num_configs = ARRAY_SIZE(omaplfb_config_hdmi_default_display),
	.configs = omaplfb_config_hdmi_default_display,
};


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

//	This is done when hdmi is initialized
//	gpio_request(HDMI_GPIO_HPD, "hdmi_hpd");
//	omap_mux_init_gpio(HDMI_GPIO_HPD, OMAP_PIN_INPUT_PULLDOWN);
//	gpio_direction_input(HDMI_GPIO_HPD);

	fixed_reg_hdmi_5v.gpio = conf->hdmi_pwr;

	if (platform_device_register(&fixed_supply_hdmi_5v) < 0) {
		pr_err("Error registering hdmi 5V regulator\n");
		return -1;
	}

	return 0;
}

extern struct omap_dss_device cpt_xga_8_dss_device;

static struct omap_dss_device *board_dss_devices[] = {
	&cpt_xga_8_dss_device,
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

static struct platform_device *a80s_devices[] __initdata = {
	&wifi_device,
	&btwilink_device,
	&fixed_supply_5v,
	&fixed_supply_1v8,
	&fixed_supply_vcc,
	&fixed_supply_vbus_musb,
	&fixed_supply_vmmc2,
//	&fixed_supply_vmmc2_aux,
	&archos_pwm_leds,
	&archos_abe_audio,
};

static struct archos_leds_config leds_config __initdata = {
	.nrev = 6,
	.rev[0 ... 2] = {
		.power_led = UNUSED_GPIO,
		.status_led = UNUSED_GPIO,
		.backlight_led = { .timer = -1 },
		.bkl_invert = 1,
		.backlight_power = UNUSED_GPIO,
		.bkl_max = 178,
		.bkl_regulator_name = "5V",
		.bkl_freq = 30000,
	},
	.rev[3 ... 5] = {
		.power_led = UNUSED_GPIO,
		.status_led = UNUSED_GPIO,
		.backlight_led = {
			.src = OMAP_DM_PWM,
			.timer = 8,
			.signal = "uart3_rx_irrx.dmtimer8_pwm_evt",
			.signal_off = "uart3_rx_irrx.safe_mode",
		},
		.bkl_invert = 1,
		.backlight_power = UNUSED_GPIO,
		.bkl_max = 254,	//200,
		.bkl_regulator_name = "5V",
		.bkl_freq = 30000,
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

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_UTMI,
	#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	.mode	= MUSB_OTG,
	#elif defined(CONFIG_USB_MUSB_HDRC)
	.mode	= MUSB_HOST,
	#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode	= MUSB_PERIPHERAL,
	#endif
	.power	= 500,
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
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR | MMC_CAP_NEEDS_POLL,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
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

#ifdef CONFIG_OMAP4_DUTY_CYCLE_GOVERNOR

static struct pcb_section omap4_duty_governor_pcb_sections[] = {
	{
		.pcb_temp_level			= DUTY_GOVERNOR_DEFAULT_TEMP,
		.max_opp			= 1200000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1200000,
			.cooling_rate		= 1008000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 24,
		},
	},
};

static void init_duty_governor(void)
{
	omap4_duty_pcb_section_reg(omap4_duty_governor_pcb_sections,
				   ARRAY_SIZE
				   (omap4_duty_governor_pcb_sections));
}
#else
static void init_duty_governor(void){}
#endif /*CONFIG_OMAP4_DUTY_CYCLE*/

/* Initial set of thresholds for different thermal zones */
static struct omap_thermal_zone thermal_zones[] = {
	OMAP_THERMAL_ZONE("safe", 0, 25000, 65000, 250, 1000, 400),
	OMAP_THERMAL_ZONE("monitor", 0, 60000, 80000, 250, 250,	250),
	OMAP_THERMAL_ZONE("alert", 0, 75000, 90000, 250, 250, 150),
	OMAP_THERMAL_ZONE("critical", 1, 85000,	115000,	250, 250, 50),
};

static struct omap_die_governor_pdata omap_gov_pdata = {
	.zones = thermal_zones,
	.zones_num = ARRAY_SIZE(thermal_zones),
};

static void omap_board_display_init(void)
{
	if (panel_cpt_xga_8_init(&cpt_xga_8_dss_device) == 0)
		board_dss_data.default_device = &cpt_xga_8_dss_device;

	omapfb_set_platform_data(&tablet_fb_pdata);

	omap_display_init(&board_dss_data);

	archos_hdmi_init();
}

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
				struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request(USE_MUTEX_LOCK);
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id,
							USE_MUTEX_LOCK);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
								bus_id);
	}
}

/*
 * Setup CFG_TRANS mode as follows:
 * 0x00 (OFF) when in OFF state(bit offset 4)
 * - these bits a read only, so don't touch them
 * 0x00 (OFF) when in sleep (bit offset 2)
 * 0x01 (PWM/PFM Auto) when in ACTive state (bit offset 0)
 */
#define TWL6030_REG_VCOREx_CFG_TRANS_MODE		(0x00 << 4 | \
		TWL6030_RES_OFF_CMD << TWL6030_REG_CFG_TRANS_SLEEP_CMD_OFFSET |\
		TWL6030_RES_AUTO_CMD << TWL6030_REG_CFG_TRANS_ACT_CMD_OFFSET)

#define TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC "OFF=OFF SLEEP=OFF ACT=AUTO"

/* OMAP4430 - All vcores: 1, 2 and 3 should go down with PREQ */
static struct twl_reg_setup_array omap4430_twl6030_setup[] = {
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE1_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE1 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE2_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE2 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE3_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE3 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{ .desc = NULL} /* TERMINATOR */
};

/* OMAP4460 - VCORE3 is unused, 1 and 2 should go down with PREQ */
static struct twl_reg_setup_array omap4460_twl6030_setup[] = {
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE1_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE1 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE2_CFG_TRANS,
		.val = TWL6030_REG_VCOREx_CFG_TRANS_MODE,
		.desc = "VCORE2 " TWL6030_REG_VCOREx_CFG_TRANS_MODE_DESC,
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_CFG_SMPS_PD,
		.val = 0x77,
		.desc = "VCORE1 disable PD on shutdown",
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VCORE3_CFG_GRP,
		.val = 0x00,
		.desc = "VCORE3 - remove binding to groups",
	},
	{
		.mod_no = TWL6030_MODULE_ID0,
		.addr = TWL6030_REG_VMEM_CFG_GRP,
		.val = 0x00,
		.desc = "VMEM - remove binding to groups",
	},
	{ .desc = NULL} /* TERMINATOR */
};

static struct regulator_consumer_supply board_vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi.0"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi.1"),

	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi2.0"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi2.1"),
};

static struct regulator_init_data board_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 	= REGULATOR_CHANGE_MODE
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

static int board_batt_table[] = {
	/* Sensicom CN0603R473B3750FB 47k B=3750  Rx=10k Ry=220k */
	/* adc code for temperature in degree C */
	1130, 1126,  /* -2 ,-1 */
	1123, 1119, 1116, 1112, 1108, 1104, 1100, 1096, 1091, 1087,  /* 00 - 09 */
	1082, 1077, 1072, 1067, 1062, 1056, 1051, 1045, 1039, 1033,  /* 10 - 19 */
	1027, 1020, 1014, 1007, 1000, 993, 986, 978, 971, 963,  /* 20 - 29 */
	956, 948, 940, 932, 923, 915, 906, 898, 889, 880,  /* 30 - 39 */
	871, 862, 853, 843, 834, 825, 815, 806, 796, 786,  /* 40 - 49 */
	776, 767, 757, 747, 737, 727, 717, 707, 697, 687,  /* 50 - 59 */
	677, 667, 658,  /* 60 - 62 */
};

/* Fuel Gauge EDV Configuration - taken from TWL_COMMON */
static struct edv_config edv_cfg = {
	.averaging = true,
	.seq_edv = 5,
	.filter_light = 155,
	.filter_heavy = 199,
	.overload_current = 1000,
	.edv = {
		{3150, 0},
		{3450, 4},
		{3600, 10},
	},
};

/* Fuel Gauge OCV Configuration  - taken from TWL_COMMON*/
static struct ocv_config ocv_cfg = {
	.voltage_diff = 75,
	.current_diff = 30,

	.sleep_enter_current = 60,
	.sleep_enter_samples = 3,

	.sleep_exit_current = 100,
	.sleep_exit_samples = 3,

	.long_sleep_current = 500,
	.ocv_period = 300,
	.relax_period = 600,

	.flat_zone_low = 35,
	.flat_zone_high = 65,

	.max_ocv_discharge = 1300,

	.table = {
		3300, 3603, 3650, 3662, 3700,
		3723, 3734, 3746, 3756, 3769,
		3786, 3807, 3850, 3884, 3916,
		3949, 3990, 4033, 4077, 4129,
		4193
	},
};

/* General OMAP4 Battery Cell Configuration  - taken from TWL_COMMON */
static struct cell_config cell_cfg =  {
	.cc_voltage = 4175,
	.cc_current = 250,
	.cc_capacity = 15,
	.seq_cc = 5,

	.cc_polarity = true,
	.cc_out = true,
	.ocv_below_edv1 = false,

	.design_capacity = 4000,
	.design_qmax = 4100,
	.r_sense = 10,

	.qmax_adjust = 1,
	.fcc_adjust = 2,

	.max_overcharge = 100,
	.electronics_load = 200, /* *10 uAh */

	.max_increment = 150,
	.max_decrement = 150,
	.low_temp = 119,
	.deep_dsg_voltage = 30,
	.max_dsg_estimate = 300,
	.light_load = 100,
	.near_full = 500,
	.cycle_threshold = 3500,
	.recharge = 300,

	.mode_switch_capacity = 5,

	.call_period = 2,

	.ocv = &ocv_cfg,
	.edv = &edv_cfg,
};

static struct twl4030_bci_platform_data board_bci_data = {
	.monitoring_interval		= 10,
	.max_charger_currentmA		= 1500,
	.max_charger_voltagemV		= 4560,
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= board_batt_table,
	.tblsize			= ARRAY_SIZE(board_batt_table),
	.cell_cfg			= &cell_cfg,
};

static struct regulator_init_data board_clk32kg = {
       .constraints = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
		.always_on		= true,
       },
};

static struct regulator_consumer_supply board_vusb_supply[] = {
		REGULATOR_SUPPLY("vusb", "archos_twl6030_usb_xceiv"),
};

static struct regulator_init_data board_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= false, // uart3_on_usb,
		.state_mem = {
			.disabled	= true,
		},
#ifdef CONFIG_ARCHOS_UART3_ON_DPDM
		.initial_state		= PM_SUSPEND_ON,
#else
		.initial_state		= PM_SUSPEND_MEM,
#endif

	},
	.num_consumer_supplies	= ARRAY_SIZE(board_vusb_supply),
	.consumer_supplies	= board_vusb_supply,
};

static struct regulator_consumer_supply board_vaux2_supply[] = {
	REGULATOR_SUPPLY("cam_vdd", "archos_camera_glue"),
};

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
		.state_mem = {
			.enabled	= false,
			.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
	.num_consumer_supplies	= ARRAY_SIZE(board_vaux2_supply),
	.consumer_supplies = board_vaux2_supply,
};

static struct twl4030_platform_data tablet_twldata;

static struct omap_i2c_bus_board_data __initdata omap4_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata omap4_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata omap4_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata omap4_i2c_4_bus_pdata;

static int __init omap4_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &omap4_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &omap4_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &omap4_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &omap4_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &omap4_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &omap4_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &omap4_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &omap4_i2c_4_bus_pdata);

	omap4_pmic_get_config(&tablet_twldata,
//			TWL_COMMON_PDATA_USB |
			TWL_COMMON_PDATA_MADC | \
//			TWL_COMMON_PDATA_BCI |
			TWL_COMMON_PDATA_THERMAL,
			TWL_COMMON_REGULATOR_VDAC |
//			TWL_COMMON_REGULATOR_VAUX1 |
//			TWL_COMMON_REGULATOR_VAUX2 | // Camera
			TWL_COMMON_REGULATOR_VAUX3 |
			TWL_COMMON_REGULATOR_VMMC |  // Memory Card
			TWL_COMMON_REGULATOR_VPP |
			TWL_COMMON_REGULATOR_VUSIM |
			TWL_COMMON_REGULATOR_VANA |
//			TWL_COMMON_REGULATOR_VCXIO | // Display Config
//			TWL_COMMON_REGULATOR_VUSB |
			TWL_COMMON_REGULATOR_CLK32KG |
			TWL_COMMON_REGULATOR_V1V8 |
			TWL_COMMON_REGULATOR_V2V1 |
			TWL_COMMON_REGULATOR_SYSEN |
			TWL_COMMON_REGULATOR_CLK32KAUDIO |
			TWL_COMMON_REGULATOR_REGEN1);

	// Override some details which are not standard
	tablet_twldata.usb		= &board_twl6030_usb_data;
	tablet_twldata.clk32kg	= &board_clk32kg;
	tablet_twldata.bci      = &board_bci_data;
	tablet_twldata.vcxio	= &board_vcxio;
	tablet_twldata.vusb		= &board_vusb;
	tablet_twldata.vaux2	= &board_vaux2;

	/* Add one-time registers configuration */
	if (cpu_is_omap443x())
		tablet_twldata.reg_setup_script = omap4430_twl6030_setup;
	else if (cpu_is_omap446x())
		tablet_twldata.reg_setup_script = omap4460_twl6030_setup;

	omap4_pmic_init("twl6030", &tablet_twldata, &twl6040_data, OMAP44XX_IRQ_SYS_2N);
	// i2c_register_board_info(1, tablet_i2c_boardinfo, ARRAY_SIZE(tablet_i2c_boardinfo));
	omap_register_i2c_bus(2, 100, board_i2c_2_boardinfo, ARRAY_SIZE(board_i2c_2_boardinfo)); // FIXME speed
	omap_register_i2c_bus(3, 100, board_i2c_3_boardinfo, ARRAY_SIZE(board_i2c_3_boardinfo)); // FIXME speed
	omap_register_i2c_bus(4, 400, NULL, 0);

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	//TODO: Archos: Check why GPIO_MSECURE is set to 30 on stock rom kernel
	/*
	 * Drive MSECURE high for TWL6030/6032 write access.
	 */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return 0;
}
// Taken from stock rom
#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP4_MUX(UART3_RX_IRRX, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE7),
	OMAP4_MUX(UART3_TX_IRTX, OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7),



		OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT
						| OMAP_PULL_ENA),
		OMAP4_MUX(SYS_NIRQ2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP
						| OMAP_PIN_OFF_WAKEUPENABLE),
		OMAP4_MUX(SYS_NIRQ1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP
						| OMAP_PIN_OFF_WAKEUPENABLE),

		/* IO optimization pdpu and offmode settings to reduce leakage */
		OMAP4_MUX(GPMC_A17, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
		OMAP4_MUX(GPMC_NBE1, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
		OMAP4_MUX(GPMC_NCS4, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
		OMAP4_MUX(GPMC_NCS5, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
						| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
		OMAP4_MUX(GPMC_NCS7, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
		OMAP4_MUX(GPMC_NBE1, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
						| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
		OMAP4_MUX(GPMC_WAIT0, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
		OMAP4_MUX(GPMC_NOE, OMAP_MUX_MODE1 | OMAP_INPUT_EN),
		OMAP4_MUX(MCSPI1_CS1, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
						| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
		OMAP4_MUX(MCSPI1_CS2, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
						| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
		OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_INPUT_EN | OMAP_OFF_EN
						| OMAP_OFF_PULL_EN),
		OMAP4_MUX(GPMC_NCS1, OMAP_MUX_MODE3 | OMAP_INPUT_EN | OMAP_WAKEUP_EN),
		OMAP4_MUX(GPMC_A24, OMAP_MUX_MODE3 | OMAP_INPUT_EN | OMAP_WAKEUP_EN),

		OMAP4_MUX(ABE_MCBSP1_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),


		OMAP4_MUX(UART3_RX_IRRX, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE7),
		OMAP4_MUX(UART3_TX_IRTX, OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

//#ifdef CONFIG_OMAP_MUX
//static struct omap_board_mux board_mux[] __initdata = {
//	OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT
//					| OMAP_PULL_ENA),
//	OMAP4_MUX(SYS_NIRQ2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP
//					| OMAP_PIN_OFF_WAKEUPENABLE),
//	OMAP4_MUX(SYS_NIRQ1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP
//					| OMAP_PIN_OFF_WAKEUPENABLE),
//
//	/* IO optimization pdpu and offmode settings to reduce leakage */
//	OMAP4_MUX(GPMC_A17, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
//	OMAP4_MUX(GPMC_NBE1, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
//	OMAP4_MUX(GPMC_NCS4, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
//	OMAP4_MUX(GPMC_NCS5, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
//					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
//	OMAP4_MUX(GPMC_NCS7, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
//	OMAP4_MUX(GPMC_NBE1, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
//					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
//	OMAP4_MUX(GPMC_WAIT0, OMAP_MUX_MODE3 | OMAP_INPUT_EN),
//	OMAP4_MUX(GPMC_NOE, OMAP_MUX_MODE1 | OMAP_INPUT_EN),
//	OMAP4_MUX(MCSPI1_CS1, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
//					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
//	OMAP4_MUX(MCSPI1_CS2, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP
//					| OMAP_OFF_EN | OMAP_OFF_PULL_EN),
//	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_INPUT_EN | OMAP_OFF_EN
//					| OMAP_OFF_PULL_EN),
//	OMAP4_MUX(GPMC_NCS1, OMAP_MUX_MODE3 | OMAP_INPUT_EN | OMAP_WAKEUP_EN),
//	OMAP4_MUX(GPMC_A24, OMAP_MUX_MODE3 | OMAP_INPUT_EN | OMAP_WAKEUP_EN),
//
//	OMAP4_MUX(ABE_MCBSP1_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
//
//
//	OMAP4_MUX(UART3_RX_IRRX, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE7),
//	OMAP4_MUX(UART3_TX_IRTX, OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7),
//	{ .reg_offset = OMAP_MUX_TERMINATOR },
//};
//
//#else
//#define board_mux	NULL
//#endif

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int irq = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		irq = twl6030_mmc_card_detect_config();
		if (irq < 0) {
			pr_err("Failed configuring MMC1 card detect\n");
			return irq;
		}
		pdata->slots[0].card_detect_irq = irq;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	return 0;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed %s\n", __func__);
		return;
	}
	pdata = dev->platform_data;
	pdata->init = omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(&c->pdev->dev);

	return 0;
}

static struct omap_board_config_kernel board_config[] __initdata = {
		{ ARCHOS_TAG_AUDIO_TWL6040,	&audio_config},
		{ ARCHOS_TAG_ACCEL,			&accel_config},
		{ ARCHOS_TAG_COMPASS,		&compass_config},
		{ ARCHOS_TAG_CHARGE,		&charge_config},
		{ ARCHOS_TAG_DISPLAY,		&display_config},

// DISABLED FOR DEVELOPMENT
		{ ARCHOS_TAG_LEDS,			&leds_config},

		{ ARCHOS_TAG_WIFI_BT,		&board_wifi_bt_config},
		{ ARCHOS_TAG_MUSB,			&musb_config},
		{ ARCHOS_TAG_GPS,			&gps_config},
		{ ARCHOS_TAG_CAMERA,		&camera_config},
		{ ARCHOS_TAG_USB_GADGET,	&gadget_config},
		{ ARCHOS_TAG_USB,			&usb_config},
//		{ ARCHOS_TAG_3G,			&usb_3g_config},
		{ ARCHOS_TAG_I2C_TSP,		&i2c_tsp_config},
//		{ OMAP_TAG_DIE_GOVERNOR,	&omap_die_governor_config},
};

static int __init omap4_leds_init(void)
{
	struct feature_tag_screen *screen;

	if (( screen = get_feature_tag(FTAG_SCREEN, feature_tag_size(feature_tag_screen)))) {
		if (!strcmp(screen->vendor, "CMI")) {
			if (screen->backlight != 0) {
				leds_config.rev[system_rev].bkl_max = screen->backlight;
				printk(KERN_INFO "Set Backlight value from tag\n");
			}
		}
	}
	return 0;
}

static void __init board_touch_init(void)
{
	struct feature_tag_touchscreen * tsp;
	int ret;

	/* check which touch screen we have */
	tsp = get_feature_tag(FTAG_HAS_TOUCHSCREEN,
			feature_tag_size(feature_tag_touchscreen));
	if (!tsp){
		printk("Feature tag resolution claims that FTAG_HAS_TOUCHSCREEN is not available!\n");
	}
//	My development device has a tr16c0 touchscreen
//	if (!tsp || (tsp->vendor == 0)) {
//		ret = i2c_register_board_info(4, cypress_tsp_i2c_boardinfo, 1);
//		printk("%s:%s: i2c_register_board_info returns %d\n",__FILE__,__FUNCTION__, ret);
//		printk("Using tm340_init method!\n");
//		archos_touchscreen_tm340_init(&board_tma340_pdata);
//	} else {
		i2c_register_board_info(4, tr16c0_tsp_i2c_boardinfo, 1);
		archos_touchscreen_tr16c0_init(&board_tr16c0_pdata);
		printk("Using tr16c0_init method!\n");
//	}
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
	}else{
		printk("Feature tag resolution claims that FTAG_HAS_GPIO_VOLUME_KEYS is not available!\n");
	}
}

static int twl6040_init(void)
{
	u8 rev = 0;
	int ret;

	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE,
				&rev, TWL6040_REG_ASICREV);
	if (ret)
		return ret;

	/*
	 * ERRATA: Reset value of PDM_UL buffer logic is 1 (VDDVIO)
	 * when AUDPWRON = 0, which causes current drain on this pin's
	 * pull-down on OMAP side. The workaround consists of disabling
	 * pull-down resistor of ABE_PDM_UL_DATA pin
	 * Impacted revisions: ES1.1 and ES1.2 (both share same ASICREV value)
	 */
	if (rev == TWL6040_REV_1_1)
		omap_mux_init_signal("abe_pdm_ul_data.abe_pdm_ul_data",
			OMAP_PIN_INPUT);

	/* ASoC audio configuration */
	archos_abe_audio_data.card_name = "tablet";

	return 0;
}

#if defined(CONFIG_TI_EMIF) || defined(CONFIG_TI_EMIF_MODULE)
static struct __devinitdata emif_custom_configs custom_configs = {
	.mask	= EMIF_CUSTOM_CONFIG_LPMODE,
	.lpmode	= EMIF_LP_MODE_SELF_REFRESH,
	.lpmode_timeout_performance = 512,
	.lpmode_timeout_power = 512,
	/* only at OPP100 should we use performance value */
	.lpmode_freq_threshold = 400000000,
};
#endif

static void set_osc_timings(void)
{
	/* Device Oscilator
	 * tstart = 2ms + 2ms = 4ms.
	 * tshut = Not defined in oscillator data sheet so setting to 1us
	 */
	omap_pm_setup_oscillator(4000, 1);
}


extern int add_preferred_console(char *name, int idx, char *options);
static void __init board_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;

	printk("%s: %u\n", __FILE__, __LINE__);

#if defined(CONFIG_TI_EMIF) || defined(CONFIG_TI_EMIF_MODULE)
	if (cpu_is_omap447x()) {
		omap_emif_set_device_details(1, &lpddr2_elpida_4G_S4_info,
				lpddr2_elpida_4G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_4G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);
		omap_emif_set_device_details(2, &lpddr2_elpida_4G_S4_info,
				lpddr2_elpida_4G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_4G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);
	} else {
		omap_emif_set_device_details(1, &lpddr2_elpida_2G_S4_x2_info,
				lpddr2_elpida_2G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);
		omap_emif_set_device_details(2, &lpddr2_elpida_2G_S4_x2_info,
				lpddr2_elpida_2G_S4_timings,
				ARRAY_SIZE(lpddr2_elpida_2G_S4_timings),
				&lpddr2_elpida_S4_min_tck, &custom_configs);
	}
#endif

	omap_board_config = board_config;
	omap_board_config_size = ARRAY_SIZE(board_config);

	set_osc_timings();

	omap4_mux_init(board_mux, NULL, package);
	omap_init_board_version(0);
	omap_create_board_props();

//	add_preferred_console("ttyO", 0, "115200");
	omap4_board_serial_init();
	wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");
	board_wifi_init();

//	Is this needed?
//	archos_create_board_props();

	omap4_i2c_init();

	archos_battery_twl4030_bci_init(&board_bci_data);

	omap4_leds_init();
	platform_add_devices(a80s_devices, ARRAY_SIZE(a80s_devices));

	archos_audio_twl6040_init(&twl6040_data);
	archos_accel_mma8453q_init(&board_mma8453q_pdata);
	archos_compass_init(&board_akm8975_pdata);

	omap_sdrc_init(NULL, NULL);
	omap4_twl6030_hsmmc_init(mmc);

	archos_usb_musb_init(&archos_vbus_info);
	usb_musb_init(&musb_board_data);
	archos_omap4_ehci_init();

	board_touch_init();
	board_buttons_init();

	twl6040_init();

	init_duty_governor();
	omap_die_governor_register_pdata(&omap_gov_pdata);

	omap_init_dmm_tiler();

	omap4_register_ion();

	omap_board_display_init();

	/* Vsel0 = gpio, vsel1 = gnd */
	status = omap_tps6236x_board_setup(true, TPS62361_GPIO, -1,
				OMAP_PIN_OFF_OUTPUT_HIGH, -1);
	if (status)
		pr_err("TPS62361 initialization failed: %d\n", status);

	omap_enable_smartreflex_on_init();

    printk("%s: %u\n", __FILE__, __LINE__);
}

static void tablet_android_display_setup(void)
{
//	panel_cpt_xga_8_preinit(&cpt_xga_8_dss_device);
	board_dss_data.default_device = &cpt_xga_8_dss_device;

	omap_android_display_setup(&board_dss_data,
				   &dsscomp_config_cpt_xga8_wuxga,
				   &omaplfb_plat_data_tablet,
				   &tablet_fb_pdata
				   );

//	omap_android_display_setup(&tablet_dss_data,
//		&dsscomp_config_tablet,
//		&omaplfb_plat_data_tablet,
//		&tablet_fb_pdata);

}

static void __init omap_tablet_reserve(void)
{
	board_memory_prepare();
#ifdef CONFIG_ION_OMAP
	tablet_android_display_setup();
	omap4_ion_init();
#else
	tablet_android_display_setup(NULL);
#endif

//	omap_rproc_reserve_cma(RPROC_CMA_OMAP4);

	printk(KERN_ERR "Archos Console values OMAP_RAM_CONSOLE_START_DEFAULT: 0x%x OMAP_RAM_CONSOLE_SIZE_DEFAULT: 0x%x!\n", OMAP_RAM_CONSOLE_START_DEFAULT, OMAP_RAM_CONSOLE_SIZE_DEFAULT);
	printk(KERN_ERR "Archos Console values ARCHOS_PHYS_ADDR_OMAP_RAM_CONSOLE: 0x%x ARCHOS_OMAP_RAM_CONSOLE_SIZE: 0x%x!\n", ARCHOS_PHYS_ADDR_OMAP_RAM_CONSOLE, ARCHOS_OMAP_RAM_CONSOLE_SIZE);

	/*
	 * If we are on a 512MByte device, we need to use the specific memory constants from archos.
	 * If start on a 1GByte device, we use the standard TI values. Note, that when using the archos values in a 1GByte device,
	 * only 512MByte are available to the kernel.
	 */
	if (is512MbyteG9Model()){
		omap_ram_console_init(ARCHOS_PHYS_ADDR_OMAP_RAM_CONSOLE,
				ARCHOS_OMAP_RAM_CONSOLE_SIZE);
	}else{
		omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
				OMAP_RAM_CONSOLE_SIZE_DEFAULT);
	}

//	omap4_secure_workspace_addr_default();
	//...  enable security workspace for 512MByte model
	omap_secure_set_secure_workspace_addr(PHYS_ADDR_SMC_MEM, SZ_1M);

	omap_reserve();
}


MACHINE_START(ARCHOS_A80S, "ARCHOS A80S board")
.atag_offset	= 0x100,
.reserve	= omap_tablet_reserve,
.map_io		= omap4_map_io,
.fixup		= fixup_archos,
.init_early	= omap4430_init_early,
.init_irq	= gic_init_irq,
.handle_irq	= gic_handle_irq,
.init_machine	= board_init,
.timer		= &omap4_timer,
.restart	= omap_prcm_restart,
MACHINE_END

//MACHINE_START(ARCHOS_A80S, "ARCHOS A80S board")
//	/* Maintainer: David Anders - Texas Instruments Inc */
//	.atag_offset	= 0x100,
//	.reserve	= omap_tablet_reserve,
//	.map_io		= omap4_map_io,
//	.init_early	= omap4430_init_early,
//	.init_irq	= gic_init_irq,
//	.handle_irq	= gic_handle_irq,
//	.init_machine	= board_init,
//	.timer		= &omap4_timer,
//	.restart	= omap_prcm_restart,
//	.fixup		= fixup_archos,
//MACHINE_END
