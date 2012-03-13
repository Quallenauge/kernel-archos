/*
 * linux/include/asm-arm/arch-omap/board-archos.h
 *
 * Hardware definitions for TI ARCHOS boards.
 *
 * Copyright (C) 2008 Niklas Schroeter, Archos S.A.,
 *
 * Derived from mach-omap2/board-omap3evm.h
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __MACH_BOARD_ARCHOS_H
#define __MACH_BOARD_ARCHOS_H

#include <plat/archos-gpio.h>
#include <plat/board.h>
#include <linux/err.h>
//#include <linux/mma7660fc.h>
#include <linux/mma8453q.h>
#include <linux/akm8975.h>
#include <linux/mag3110.h>
#include <linux/i2c/twl.h>
#include <linux/input/cypress-tma340.h>
#include <linux/input/pixcir_i2c_tsp.h>
#include <linux/input/tr16c0-i2c.h>
#include <linux/input/cpt_i2c_tsp.h>
#include <asm/mach-types.h>

struct omap_dss_device;
struct omap_dss_platform_data;
struct machine_desc;
struct tag;
struct meminfo;
struct i2c_client;

/* max. number of hardware revisions (change if necessary) */
#define MAX_HWREVS	16
/*
 * board config tags, use with omap_get_config()
 * to distribute board configuration to init
 * code and drivers.
 */
struct archos_tsp_conf
{
	int	irq_gpio;
	int	pwr_gpio;
	int bus_num;
	u16 x_plate_ohms;
	u16 pressure_max;
	int inversion_flags;
};

struct archos_tsp_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_tsp_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_TSP	0x4e01

struct archos_vbus_config
{
	int	nrev;	/* number of hardware revisions */
	int rev[MAX_HWREVS];
};
#define ARCHOS_TAG_VBUS0 0x4e02

struct archos_pwm_conf
{
	enum pwm_src { 
		OMAP_DM_PWM	= 0,
		TWL6030_PWM	= 1,
	} src;
	int timer;
	int mux_cfg;
	char *signal;
	char *signal_off;
};

struct archos_spi_conf
{
	int spi_clk;
	int spi_data;
	int spi_cs;
};

struct archos_disp_conf
{
	int lcd_pwon;
	int lcd_rst;
	int lcd_pci;
	int lvds_en;
	int lcd_stdby;
	int lcd_avdd_en;
	int hdmi_pwr;
	int hdmi_int;
	struct archos_pwm_conf vcom_pwm;
	struct archos_spi_conf spi;
	int cvbs_on;
	int bkl_en;
	int bkl_pwr;
	int use_fixed_bkl;
	int do_not_use_pll;
};

struct archos_display_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_disp_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_DISPLAY	0x4e03

struct archos_keys_conf
{
	int power;
};
struct archos_keys_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_keys_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_KEYS	0x4e04

struct archos_usb_conf
{
	int		usb_pwroff;
	unsigned int	usb_max_power;
	int		enable_usb_ehci;
	int		enable_usb_hub;
	int		usb_hub_rst;
	int		enable_5v;
};

struct archos_usb_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_usb_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_USB		0x4e05

struct archos_charge_conf
{
	int charge_enable;
	int charge_high;
	int charge_low;
	int gpio_dc_detect;
	int dc_detect_invert;
	enum charger_type_t { 
		CHARGER_LX2208=0,	// USB charger 100/500/1000mA
		CHARGER_DCIN,		// autonomous charger using DC-in
		CHARGER_RT9502,		// USB charger 100/500mA + DC-in
		CHARGER_TWL6030USB,	// USB charger built into TWL6030
		CHARGER_TWL6030USB_DC,	// USB charger built into TWL6030, used with DC-IN plug.
		CHARGER_ISL9220		// USB charger 100/500/1500mA
	} charger_type;
};

struct archos_charge_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_charge_conf rev[MAX_HWREVS];
};
#define ARCHOS_TAG_CHARGE 0x4e07

struct archos_usbhdd_conf
{
	int hub_power;
	int hdd_power;
};

struct archos_usbhdd_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_usbhdd_conf rev[MAX_HWREVS];
};
#define ARCHOS_TAG_USBHDD 0x4e08

struct archos_audio_conf
{
	int spdif;
	int hp_on;
	int headphone_plugged;
	int vamp_vbat;
	int vamp_dc;
	int clk_mux;
};

struct archos_audio_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_audio_conf rev[MAX_HWREVS];
};
#define ARCHOS_TAG_AUDIO 0x4e09

enum {
	PA_TYPE_TQM67002,
	PA_TYPE_TQM67002_NEW_BOM,
	PA_TYPE_TQM67002A,
	PA_TYPE_RF3482,
};

struct archos_wifi_bt_config {
	int nrev; /* number of hardware revisions */
	struct archos_wifi_bt_dev_conf {
		int wifi_power;
		char *wifi_power_signal;
		int wifi_irq;
		char *wifi_irq_signal;
		int bt_power;
		int fm_power;
		int gps_power;
		unsigned int wifi_pa_type;
	} rev[MAX_HWREVS];
};

struct archos_hdmi_in_config {
	 int nrev;
	 struct archos_hdmi_in_dev_conf {
		char *hdmi_in_pwron_signal;
		int hdmi_in_pwron;
		char *hdmi_in_detect_signal;
		int hdmi_in_detect;
	} rev[MAX_HWREVS];
};

#define ARCHOS_TAG_WIFI_BT 0x4e0a
struct archos_sata_conf
{
	int hdd_power;
	const char *hdd_power_mux;
	int sata_power;
	const char *sata_power_mux;
	int sata_ready;
	const char *sata_ready_mux;
	unsigned usb_suspend:1;
};
struct archos_sata_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_sata_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_SATA		0x4e0b

struct archos_sd_conf
{
	int sd_power;
	int sd_detect;
	int sd_prewarn;
};
struct archos_sd_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_sd_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_SD	0x4e0c

struct archos_gps_conf
{
	int gps_enable;
	int gps_reset;
	int gps_int;
};
struct archos_gps_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_gps_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_GPS	0x4e0d

struct archos_irblaster_conf
{
	struct archos_pwm_conf irblaster_pwm;
	int irblaster_pwm_disable;
	struct archos_pwm_conf irblaster_ctrl_timer;
};

struct archos_irblaster_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_irblaster_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_IRBLASTER	0x4e0e

struct archos_irremote_conf
{
	struct archos_pwm_conf irremote_timer;
	int irremote_timer_disable;
};

struct archos_irremote_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_irremote_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_IRREMOTE	0x4e0f

struct archos_uart3_conf
{
	int uart3_rx_mux;
	int uart3_tx_mux;
	int uart3_rx_disable;
	int uart3_tx_disable;
};

struct archos_uart3_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_uart3_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_UART3	0x4e10

struct archos_accel_conf
{
	int accel_int1;
	int accel_int2;
};

struct archos_accel_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_accel_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_ACCEL	0x4e11

struct archos_atmega_config {
	const char *name;
	int irq;
};
#define ARCHOS_TAG_ATMEGA	0x4e12

struct archos_modem_idcc_conf
{
	int pwron;
	int wwan_reset;
	int wwan_disable;
	int master_rdy;
	int slave_rdy;
};

struct archos_modem_idcc_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_modem_idcc_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_MODEM_IDCC	0x4e13

struct archos_modem_huawei_conf
{
	int pwron;
	int wwan_reset;
	int wwan_disable;
	int wake_module;
	int wake_omap;
	int enable_usb2;
	int gps_enable;
};

struct archos_modem_huawei_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_modem_huawei_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_MODEM_HUAWEI	0x4e14

struct archos_vibrator_conf
{
	int vibrator;
};

struct archos_vibrator_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_vibrator_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_VIBRATOR	0x4e15

struct archos_compass_conf
{
	int data_ready;
};

struct archos_compass_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_compass_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_COMPASS	0x4e16

struct archos_camera_conf
{
	int pwr_down;
	const char *pwr_down_mux;
	int reset;
	const char *reset_mux;
};

struct archos_camera_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_camera_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_CAMERA	0x4e17

struct archos_leds_conf
{
	int power_led;
	struct archos_pwm_conf backlight_led;
	int backlight_power;
	char *bkl_regulator_name;
	unsigned int bkl_freq;
	unsigned char bkl_max;
	unsigned char bkl_min;
	unsigned bkl_invert:1;
	unsigned pwr_invert:1;
	int status_led;
};

struct archos_leds_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_leds_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_LEDS		0x4e18

struct archos_fsusb_conf
{
	int	suspend;
	int	enable_usb_ohci;
};

struct archos_fsusb_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_fsusb_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_FSUSB	0x4e19

struct archos_usb_tsp_conf
{
	int	enable;
	int	reset;
	int 			suspend_flags;
	int 			x_scale;
	int 			x_offset;
	int 			y_scale;
	int 			y_offset;
};

struct archos_usb_tsp_config
{
	int	nrev;
	struct archos_usb_tsp_conf rev[MAX_HWREVS];
};
#define ARCHOS_TAG_USB_TSP 0x4e1a

#define USB_TSP_FLAGS_POWER_OFF		0x1
#define USB_TSP_FLAGS_EARLY_POWER_OFF	0x2
#define USB_TSP_FLAGS_RELEASE_WA 	0x4
#define USB_TSP_FLAGS_SLOW_EVENT_RATE 	0x8

struct usb_tsp_platform_data {
	unsigned long flags;
	void (*panel_power)(int on_off);
	int x_scale, x_offset;
	int y_scale, y_offset;
};

struct archos_i2c_tsp_conf
{
	int irq_gpio;
	char *irq_signal;
	int pwr_gpio;
	char *pwr_signal;
	int shtdwn_gpio;
	char *shtdwn_signal;
};

struct archos_i2c_tsp_config
{
	int	nrev;
	struct archos_i2c_tsp_conf rev[MAX_HWREVS];
};

#define ARCHOS_TAG_I2C_TSP 0x4e1d

struct archos_audio_twl6040_conf
{
	int	power_on;
};

struct archos_audio_twl6040_config
{
	int	nrev;
	struct archos_audio_twl6040_conf rev[MAX_HWREVS];
};
#define ARCHOS_TAG_AUDIO_TWL6040 0x4e1b

struct archos_supply5V_conf
{
	int pwr_on;
};

struct archos_supply5V_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_supply5V_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};

struct archos_core_platform_data {
	struct gpio_keys_platform_data *gpio_keys;
	int *codes;
	int ncodes;
};


struct archos_hdmi_conf {
	int hdmi_irq;
	int hdmi_dac;
	int disp_sel;
};

struct regulator;
struct archos_hdmi_platform_data {
	void (*hdmi_pwr)(struct i2c_client *client, int on_off);
	int pwr_gpio;
	struct regulator *regulator;
};

struct archos_extdac_conf {
	int hdmi_dac;
	int disp_sel;
	int cpld_aux;
};

struct archos_clocks_dpll4 {
	unsigned long rate;
	unsigned long m2_rate;
	unsigned long m3_rate;
	unsigned long m4_rate;
	unsigned long m5_rate;
	unsigned long m6_rate;	
};

struct archos_clocks {
	struct archos_clocks_dpll4 dpll4;
};

struct archos_musb_conf
{
	int gpio_vbus_detect;
	int gpio_vbus_flag;
	int gpio_id;
};

struct archos_musb_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_musb_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};

#define ARCHOS_TAG_MUSB		0x4e1c

struct archos_usb_gadget_config
{
	int	nrev;
	struct archos_usb_gadget_conf {
		char product_name[32];
		int product_id;
		int ums_luns;
	} rev[MAX_HWREVS];
};
#define ARCHOS_TAG_USB_GADGET	0x4e1e

struct archos_temp_duty_cycle_config
{
	unsigned int nitro_interval;		// max. time we spend at OPP_NITRO
	unsigned int cooling_interval;		// time we spend at cooling OPP
	unsigned int nitro_rate;			// clock rate for OPP_NITRO
	unsigned int cooling_rate;			// clock rate used for cooling
	unsigned int extra_cooling_rate;	// clock rate used for extra cooling
	bool	inhibit_charging;		/* during extra cooling turn off charging */

	struct duty_cycle_temp_table {		// table of parameters per temperature range
		int temperature;
		int heating_budget;
		bool cooling_needed;
	} duty_cycle_temp_table[10];
};
#define ARCHOS_TAG_TEMP_DUTY_CYCLE	0x4e1f

struct archos_3g_config
{
	int	nrev;
	struct archos_3g_conf {
		int enable;
	} rev[MAX_HWREVS];
};
#define ARCHOS_TAG_3G 0x4e20

//extern int __init archos_accel_mma7660fc_init(struct mma7660fc_pdata *pdata);
extern int __init archos_accel_mma8453q_init(struct mma8453q_pdata *pdata);

extern int __init ads7846_dev_init(void);
extern void __init fixup_archos(struct machine_desc *,
		struct tag *, char **, struct meminfo *);

extern void archos_power_off(void);

extern int __init panel_boe_wsvga_10_init(struct omap_dss_device *);
extern int __init panel_lg_wvga_7_init(struct omap_dss_device *);
extern int __init panel_a35_init(struct omap_dss_device *);
extern int __init panel_cpt_xga_8_init(struct omap_dss_device *);
extern int __init panel_auo_wxga_10_init(struct omap_dss_device *);
extern int __init panel_claa_wsvga_7_init(struct omap_dss_device *);
extern int __init panel_ivo_wxga_116_init(struct omap_dss_device *);
extern int __init panel_virtual_init(struct omap_dss_device *);

extern int __init archos_compass_init(struct akm8975_platform_data *pdata);
extern int __init archos_compass_mag3110_init(struct mag3110_platform_data *pdata);

extern int __init archos_touchscreen_tm340_init(struct cypress_tma340_platform_data *pdata);
extern int __init archos_touchscreen_pixcir_init(struct pixcir_platform_data *pdata);
extern int __init archos_touchscreen_tr16c0_init(struct tr16c0_platform_data *pdata);
extern int __init archos_touchscreen_cpt_i2c_tsp_init(struct cpt_i2c_tsp_platform_data *pdata);

extern int __init archos_camera_mt9m114_init(void);
extern int __init archos_camera_mt9d113_init(void);
extern int __init archos_camera_ov7675_init(void);

extern void __init archos_reserve(void);
extern int __init archos_memory_init(void);

//extern int __init archos_leds_init(void);

extern int __init archos_battery_twl4030_bci_init(struct twl4030_bci_platform_data *pdata);

struct gpio_vbus_mach_info;
extern int __init archos_usb_musb_init(struct gpio_vbus_mach_info *info);

extern void archos_omap3_ehci_init(void);
extern void archos_omap4_ehci_init(void);
extern int __init archos_omap3_uhhtll_init(void);

extern unsigned get_last_off_on_transaction_id(struct device *dev);

struct hardware_component {
	unsigned tps62361:1;
	unsigned hdd:1;
	unsigned gps:1;
};

extern struct hardware_component hardware_comp;

static inline bool machine_has_tps62361(void) {
	return !!hardware_comp.tps62361;
}
static inline int machine_has_usbhost_plug(void)
{
	/* no current device has dedicated usb host plug */
	return 0;
}

#define hwrev_ptr(hwconf_ptr, revno) 	\
({					\
	const typeof((*(hwconf_ptr)).rev[0]) *_conf;	\
	if (NULL == hwconf_ptr || IS_ERR(hwconf_ptr)) { \
		_conf = ERR_PTR(-EINVAL); 		\
	} else { 					\
		if ((*(hwconf_ptr)).nrev <= revno) {	\
			_conf = ERR_PTR(-EINVAL);	\
		} else {				\
			_conf = &((*(hwconf_ptr)).rev[revno]);\
		}					\
	}						\
	_conf;						\
})							\


#endif /*  __MACH_BOARD_ARCHOS_H */

