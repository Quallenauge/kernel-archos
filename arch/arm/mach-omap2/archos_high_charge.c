/*
 * archos_high_charge.c
 *
 *  Created on: May 11, 2012
 *      Author: Bastian M?ller
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/usb/otg.h>

#include <linux/gpio.h>

#include "mux.h"

#include <mach/board-archos.h>

#include <plat/board.h>


#define DEVICE_NAME "archos_high_charge"

static struct archos_high_charge_device {
	struct platform_device* pdev;
	unsigned int enable_high_charge;
	struct notifier_block nb;
	struct otg_transceiver* otg;
	int allow_high_charge;
} archos_high_charge_dev;

static struct platform_driver archos_high_charge_driver = {

		.driver = {
			.name = DEVICE_NAME,
			.owner = THIS_MODULE,
		}
};

static void archos_high_charge_set(struct archos_high_charge_device *dev)
{
	if ((dev->otg->last_event == USB_EVENT_CHARGER) && (dev->allow_high_charge == 1)) {
		gpio_set_value(dev->enable_high_charge, 1);
	} else {
		gpio_set_value(dev->enable_high_charge, 0);
	}
}

static int archos_high_charge_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct archos_high_charge_device* dev = &archos_high_charge_dev;

	archos_high_charge_set(dev);

	return NOTIFY_OK;
}

static ssize_t high_charge_show(struct device* dev,
		    struct device_attribute *attr, char* buf)
{
	struct archos_high_charge_device* d = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%i\n", d->allow_high_charge);
}
static ssize_t high_charge_store(struct device* dev,
		    struct device_attribute *attr, const char* buf, size_t len)
{
	struct archos_high_charge_device* d = dev_get_drvdata(dev);

	d->allow_high_charge = 1;

	archos_high_charge_set(d);

	return len;
}
static DEVICE_ATTR(high_charge, S_IRUGO|S_IWUSR, high_charge_show, high_charge_store);

static int __init archos_high_charge_init(void) {
	const struct archos_ext_power_config* config;
	const struct archos_ext_power_conf* conf;
	struct archos_high_charge_device* dev = &archos_high_charge_dev;
	struct platform_device *pdev;
	int ret;

	dev->enable_high_charge = UNUSED_GPIO;

	config = omap_get_config(ARCHOS_TAG_POWER_EXT,
			struct archos_ext_power_config);
	if (config == NULL)
		return -ENODEV;

	conf = hwrev_ptr(config, system_rev);
	if (IS_ERR(conf)) {
		pr_err("%s: no device configuration for hardware_rev %i\n",
				__func__, system_rev);
		return -ENODEV;
	}

	if (conf->charger_type == CHARGER_TWL6030USB)
		return -ENODEV;

	ret = platform_driver_register(&archos_high_charge_driver);
	if (ret)
		return ret;

	pdev = platform_device_alloc(DEVICE_NAME, -1);
	if (!pdev) {
		ret = -ENOMEM;
		goto initfail1;
	}

	ret = gpio_request( conf->usb_high_charge, "enable_high_charge");
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
				conf->usb_high_charge);
		goto initfail2;
	}
	omap_mux_init_gpio(conf->usb_high_charge, OMAP_PIN_OUTPUT);
	gpio_direction_output(conf->usb_high_charge, 0);
	gpio_set_value(conf->usb_high_charge, 0);

	ret = platform_device_add(pdev);
	if (ret)
		goto initfail3;

	gpio_export(conf->usb_high_charge, false);

	dev->enable_high_charge = conf->usb_high_charge;
	dev->pdev = pdev;
	dev->nb.notifier_call = archos_high_charge_notifier_call;
	dev->allow_high_charge = 0;

	dev->otg = otg_get_transceiver();
	if (dev->otg) {
		ret = otg_register_notifier(dev->otg, &dev->nb);
	} else {
		pr_err("Could not register archos_charger_notifier with OTG module\n");
		goto initfail4;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_high_charge);
	if (ret < 0) {
		pr_err("Could not register high_charge sysfs\n");
		goto initfail5;
	}

	platform_set_drvdata(pdev, dev);

	archos_high_charge_set(dev);

	return ret;

initfail5:
	otg_put_transceiver(dev->otg);
initfail4:
	platform_device_del(pdev);
initfail3:
	gpio_free(conf->usb_high_charge);
initfail2:
	platform_device_put(pdev);
initfail1:
	platform_driver_unregister(&archos_high_charge_driver);
	return ret;
}

MODULE_LICENSE("GPL");

late_initcall(archos_high_charge_init);

