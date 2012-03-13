/*
 * Freescale MAG3110 eCompass board configuration
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mag3110.h>

#include <mach/gpio.h>
#include <plat/archos-gpio.h>
#include <asm/mach-types.h>
#include <plat/board.h>
#include <mach/board-archos.h>


int __init archos_compass_mag3110_init(struct mag3110_platform_data *pdata)
{
	struct archos_compass_conf compass_gpio;
	const struct archos_compass_config *compass_cfg;
	
	compass_cfg = omap_get_config( ARCHOS_TAG_COMPASS, struct archos_compass_config );
	if (compass_cfg == NULL) {
		printk(KERN_DEBUG "archos_compass_mag3110_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( system_rev >= compass_cfg->nrev ) {
		printk(KERN_DEBUG "archos_compass_mag3110_init: system_rev (%i) >= nrev (%i)\n",
			system_rev, compass_cfg->nrev);
		return -ENODEV;
	}

	compass_gpio = compass_cfg->rev[system_rev];

	/* irq needed by the driver */
	if (compass_gpio.data_ready != -1)
		pdata->irq = gpio_to_irq(compass_gpio.data_ready);
	else
		pdata->irq = -1;

	archos_gpio_init_input(compass_gpio.data_ready, "data_ready");
	printk("archos_compass_mag3110_init: irq %d\n",pdata->irq);

	/* Choose the right position */
	pdata->position = 6;

	return 0;
}
