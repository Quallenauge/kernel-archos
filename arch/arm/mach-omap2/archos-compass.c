/*
 * AKM8975 Compass board configuration
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/akm8975.h>

#include <mach/gpio.h>
#include <plat/archos-gpio.h>
#include <asm/mach-types.h>
#include <plat/board.h>
#include <mach/board-archos.h>

#define PROJECT_NAME	"AKM8975 Compass"


int __init archos_compass_init(struct akm8975_platform_data *pdata)
{
	struct archos_compass_conf compass_gpio;
	const struct archos_compass_config *compass_cfg;
	
	compass_cfg = omap_get_config( ARCHOS_TAG_COMPASS, struct archos_compass_config );
	if (compass_cfg == NULL) {
		printk(KERN_DEBUG "archos_compass_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( system_rev >= compass_cfg->nrev ) {
		printk(KERN_DEBUG "archos_compass_init: system_rev (%i) >= nrev (%i)\n",
			system_rev, compass_cfg->nrev);
		return -ENODEV;
	}

	compass_gpio = compass_cfg->rev[system_rev];

	/* irq needed by the driver */
	if (compass_gpio.data_ready != -1)
		pdata->irq_DRDY = gpio_to_irq(compass_gpio.data_ready);
	else
		pdata->irq_DRDY = -1;

	archos_gpio_init_input(compass_gpio.data_ready, "data_ready");
	printk("archos_compass_init: irq %d\n",pdata->irq_DRDY);

	/* NOTE: Currently, the driver does not use these two fields */
	strcpy(pdata->project_name, PROJECT_NAME);
	pdata->layout = 0;

	return 0;
}
