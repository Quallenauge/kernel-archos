/*
 * archos-nl5550.c
 *
 *  Created on: Feb 4, 2011
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <asm/gpio.h>

#include <mach/board-archos.h>

#include "mux.h"

#define DEVICE_NAME	"nl5550.0"


static struct nl5550_device {
	struct platform_device *pdev;
	struct regulator *gps_1v8;
	struct archos_gps_conf gps_conf;
	bool enabled;
} nl5550_device;

static void archos_nl5550_enable(struct nl5550_device *dev, bool en)
{
	if (dev->enabled == en)
		return;

	if (en) 
		regulator_enable(dev->gps_1v8);
	else
		regulator_disable(dev->gps_1v8);
		
	dev->enabled = en;
}

static int archos_nl5550_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct nl5550_device *dev = platform_get_drvdata(pdev);
	if (dev->enabled)
		regulator_disable(dev->gps_1v8);

	return 0;
}

static int archos_nl5550_resume(struct platform_device *pdev)
{
	struct nl5550_device *dev = platform_get_drvdata(pdev);
	if (dev->enabled)
		regulator_enable(dev->gps_1v8);
	return 0;
}

static struct platform_driver archos_nl5550_driver = {
#ifdef HWTEST
	.suspend		= archos_nl5550_suspend,
	.resume			= archos_nl5550_resume,
#else
	.suspend		= NULL,
	.resume 		= NULL,
#endif
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
	}
};

static int __init nl5550_init(void)
{
	struct nl5550_device *dev = &nl5550_device;
	struct platform_device *pdev;
	const struct archos_gps_config *gps_cfg;
	const struct archos_gps_conf *conf;
	int ret;
	
	gps_cfg = omap_get_config(ARCHOS_TAG_GPS,
			struct archos_gps_config);
	if (NULL == gps_cfg)
		return -ENODEV;
	
	conf = hwrev_ptr(gps_cfg, system_rev);
	if (IS_ERR(conf)) {
		pr_err("%s: no device configuration for hardware_rev %i\n",
				__func__, system_rev);	
		return -ENODEV;
	}
	
	dev->gps_conf = *conf;
	
	ret = platform_driver_register(&archos_nl5550_driver);
	if (ret)
		return ret;

	pdev = platform_device_alloc(DEVICE_NAME, -1);
	if (!pdev) {
		ret = -ENOMEM;
		goto initfail1;
	}

	ret = gpio_request(dev->gps_conf.gps_enable, "gps_nshutdown");
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
			dev->gps_conf.gps_enable);
		goto initfail2;
	}
	omap_mux_init_gpio(dev->gps_conf.gps_enable, 
			OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
	gpio_direction_output(dev->gps_conf.gps_enable, 0);
	
	if (gpio_is_valid(dev->gps_conf.gps_int)) {
		ret = gpio_request(dev->gps_conf.gps_int, "gps_int");
		if (IS_ERR_VALUE(ret)) {
			pr_err("%s: can't reserve GPIO: %d\n", __func__,
				dev->gps_conf.gps_int);
			goto initfail3;			
		}
		omap_mux_init_gpio(dev->gps_conf.gps_int, OMAP_PIN_INPUT_PULLDOWN);
		gpio_direction_input(dev->gps_conf.gps_int);
	}

	ret = platform_device_add(pdev);
	if (ret)
		goto initfail4;

	gpio_export(dev->gps_conf.gps_enable, false);
	gpio_export_link(&pdev->dev, "gps_nshutdown",
			dev->gps_conf.gps_enable);

	if (gpio_is_valid(dev->gps_conf.gps_int)) {
		gpio_export(dev->gps_conf.gps_int, false);
		gpio_export_link(&pdev->dev, "gps_int",
				dev->gps_conf.gps_int);
	}
	
	omap_mux_init_signal("mcspi1_cs1.uart1_rx", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("uart3_cts_rctx.uart1_tx", OMAP_PIN_OUTPUT);

	/* make sure 1V8S power rail is on */
	dev->gps_1v8 = regulator_get(&pdev->dev, "GPS_1V8");
	if (IS_ERR(dev->gps_1v8))
		goto initfail5;
	
	archos_nl5550_enable(dev, 1);
	
	dev->pdev = pdev;
	platform_set_drvdata(pdev, dev);
	
	return ret;
	
initfail5:
	platform_device_del(pdev);
initfail4:
	if (gpio_is_valid(dev->gps_conf.gps_int))
		gpio_free(dev->gps_conf.gps_int);
initfail3:
	gpio_free(dev->gps_conf.gps_enable);
initfail2:
	platform_device_put(pdev);
initfail1:
	platform_driver_unregister(&archos_nl5550_driver);
	return ret;

}

device_initcall(nl5550_init);
