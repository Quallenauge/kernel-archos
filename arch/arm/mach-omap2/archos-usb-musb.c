#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/usb/gpio_vbus.h>
#include <linux/err.h>
#include "mux.h"

#include <mach/board-archos.h>

int __init archos_usb_musb_init(struct gpio_vbus_mach_info *vbus_info)
{
	const struct archos_musb_config *musb_cfg;
	const struct archos_musb_conf *conf;

	musb_cfg = omap_get_config( ARCHOS_TAG_MUSB, struct archos_musb_config );
	/* might be NULL */
	if (musb_cfg == NULL) {
		printk( "archos-usb-musb: no board configuration found\n");
		return -ENODEV;
	}
	conf = hwrev_ptr(musb_cfg, system_rev);
	if (IS_ERR(conf)) {
		pr_err("%s: no device configuration for hardware_rev %i\n",
				__func__, system_rev);
		return -ENODEV;
	}

	if (gpio_is_valid(conf->gpio_vbus_detect))
		omap_mux_init_gpio(conf->gpio_vbus_detect, OMAP_PIN_INPUT);
	if (gpio_is_valid(conf->gpio_id))
		omap_mux_init_gpio(conf->gpio_id, OMAP_PIN_INPUT_PULLUP);
	if (gpio_is_valid(conf->gpio_vbus_flag))
		omap_mux_init_gpio(conf->gpio_vbus_flag, OMAP_PIN_INPUT_PULLUP);
	
	vbus_info->gpio_vbus = conf->gpio_vbus_detect;
	vbus_info->gpio_vbus_flag = conf->gpio_vbus_flag;
	vbus_info->gpio_vbus_flag_inverted = true;
	vbus_info->gpio_id = conf->gpio_id;
	vbus_info->gpio_pullup = -1;
	
	return 0;
}

