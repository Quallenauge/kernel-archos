/*
 * MMA8453Q Accelerometer board configuration
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mma8453q.h>

#include <mach/gpio.h>
#include <asm/mach-types.h>
#include <plat/board.h>
#include <mach/board-archos.h>

#include "mux.h"

int __init archos_accel_mma8453q_init(struct mma8453q_pdata *pdata)
{
	struct archos_accel_conf accel_gpio;
	const struct archos_accel_config *accel_cfg;
	
	accel_cfg = omap_get_config( ARCHOS_TAG_ACCEL, struct archos_accel_config );
	if (accel_cfg == NULL) {
		printk(KERN_DEBUG "archos_accel_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( system_rev >= accel_cfg->nrev ) {
		printk(KERN_DEBUG "archos_accel_init: system_rev (%i) >= nrev (%i)\n",
			system_rev, accel_cfg->nrev);
		return -ENODEV;
	}

	pdata->irq1 = -1;
	pdata->irq2 = -1;

	accel_gpio = accel_cfg->rev[system_rev];

	if (accel_gpio.accel_int1 != UNUSED_GPIO) {
		int ret = gpio_request(accel_gpio.accel_int1, "accel_int1");
		if (ret) {
			pr_err("%s: accel_int1: Cannot request GPIO %d", __FUNCTION__, accel_gpio.accel_int1);
		} else {
			omap_mux_init_gpio(accel_gpio.accel_int1, OMAP_PIN_INPUT_PULLUP);
			gpio_direction_input(accel_gpio.accel_int1);
			/* irq needed by the driver */
			pdata->irq1 = gpio_to_irq(accel_gpio.accel_int1);
		}
	}

	if (accel_gpio.accel_int2 != UNUSED_GPIO) {
		int ret = gpio_request(accel_gpio.accel_int2, "accel_int1");
		if (ret) {
			pr_err("%s: accel_int2: Cannot request GPIO %d", __FUNCTION__, accel_gpio.accel_int2);
		} else {
			omap_mux_init_gpio(accel_gpio.accel_int2, OMAP_PIN_INPUT_PULLUP);
			gpio_direction_input(accel_gpio.accel_int2);
			/* irq needed by the driver */
			pdata->irq2 = gpio_to_irq(accel_gpio.accel_int2);
		}
	}

	printk("archos_accel_init: irq1 %d, irq2 %d\n", pdata->irq1, pdata->irq2);

	return 0;
}
