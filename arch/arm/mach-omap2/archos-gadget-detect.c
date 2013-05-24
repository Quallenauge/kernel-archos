/*
 * archos-gadget-detect.c
 *
 *  Created on: Apr 24, 2012
 *      Author: bmueller
 *
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c/twl6030-gpadc.h>
#include <linux/kobject.h>

#include <asm/gpio.h>

#include <mach/board-archos.h>

#include <linux/kthread.h>

#include "mux.h"

#define DEVICE_NAME	"archos_gadget_detect"

static struct archos_gadget_detect_device {
	int dc_out_enable;
	unsigned int adc_channel;
	unsigned int shorted;
	unsigned int open;
	unsigned int i_sense;
	struct platform_device* pdev;
	int state;
} archos_gadget_detect_dev;

static struct platform_driver archos_gadget_detect_driver = {
	.suspend = NULL,
	.resume = NULL,
	.driver = {
			.name = DEVICE_NAME,
			.owner = THIS_MODULE,
	},
};

struct task_struct* archos_gadget_detect_ts;

static int archos_gadget_detect_threadfcn(void* dat) {
	struct twl6030_gpadc_request req;
	struct archos_gadget_detect_device* dev = &archos_gadget_detect_dev;
	int i=0;
	int old = 0;
	req.channels = (1 << dev->adc_channel);
	req.method = TWL6030_GPADC_SW2;
	req.type = TWL6030_GPADC_WAIT;
	while (1) {
		if (kthread_should_stop())
			break;
		old = 0;
		twl6030_gpadc_set_chan3_current(dev->i_sense);
		twl6030_gpadc_conversion(&req);
		for (i=0; (i<=100) && ((req.rbuf[dev->adc_channel]-old) >10); i++){
			old = req.rbuf[dev->adc_channel];
			twl6030_gpadc_conversion(&req);
		}
		//printk("GPADC RESULT = %i\n", req.rbuf[dev->adc_channel]);
		if ((old >= dev->shorted) && (old <= dev->open)) {
			if (!dev->state) {
				printk("Gadget connected\n");
				gpio_set_value(dev->dc_out_enable, 1);
				dev->state = 1;
				kobject_uevent(&dev->pdev->dev.kobj, KOBJ_ONLINE);
			}
		} else {
			if (dev->state) {
				printk("Gadget disconnected\n");
				gpio_set_value(dev->dc_out_enable, 0);
				dev->state = 0;
				kobject_uevent(&dev->pdev->dev.kobj, KOBJ_OFFLINE);
			}
		}
		msleep(1000);
	}
	return 0;
}

static ssize_t _show_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct archos_gadget_detect_device* d = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", d->state);
}

static DEVICE_ATTR(state, 00444, _show_state, NULL);

static int __init archos_gadget_detect_init(void) {
	const struct archos_gadget_detect_config* pconfig;
	const struct archos_gadget_detect_conf* pconf;
	struct archos_gadget_detect_device* dev = &archos_gadget_detect_dev;
	struct platform_device* pdev;
	int ret = -ENODEV;

	pconfig = omap_get_config(ARCHOS_TAG_GADGET_DETECT,
			struct archos_gadget_detect_config);
	if (pconfig == NULL)
		goto initfail1;

	pconf = hwrev_ptr(pconfig, system_rev);
	if (IS_ERR(pconf)) {
		pr_err("%s: No configuration found for rev.%ui\n", __func__, system_rev);
		goto initfail1;
	}

	if (!gpio_is_valid(pconf->dc_out_enable)) {
		pr_err("%s: Invalid GPIO %i\n",__func__, pconf->dc_out_enable);
		goto initfail1;
	}

	ret = platform_driver_register(&archos_gadget_detect_driver);
	if (ret) {
		pr_err("%s: Can't register platform driver\n", __func__);
		goto initfail1;
	}

	pdev = platform_device_alloc(DEVICE_NAME, -1);
	if (!pdev) {
		ret = -ENOMEM;
		goto initfail2;
	}

	ret = gpio_request(pconf->dc_out_enable, "dc_out_enable");
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s: Error requesting GPIO %i\n", __func__, pconf->dc_out_enable);
		goto initfail3;
	}

	if (pconf->adc_channel > TWL6030_GPADC_MAX_CHANNELS) {
		pr_err("%s: Invalid adc channel\n",__func__);
		goto initfail3;
	}

	dev->dc_out_enable = pconf->dc_out_enable;
	dev->adc_channel = pconf->adc_channel;
	dev->open = pconf->open;
	dev->shorted = pconf->shorted;
	dev->i_sense = pconf->i_sense;
	dev->state = 0;

	omap_mux_init_gpio(dev->dc_out_enable, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
	gpio_direction_output(dev->dc_out_enable, 0);

	ret = platform_device_add(pdev);
	if (ret)
		goto initfail4;

	gpio_export(dev->dc_out_enable,false);
	gpio_export_link(&pdev->dev, "dc_out_enable", dev->dc_out_enable);

	archos_gadget_detect_ts = kthread_run(&archos_gadget_detect_threadfcn, NULL, "archos_gadget_detect");

	dev->pdev = pdev;
	platform_set_drvdata(pdev, dev);

	ret = device_create_file(&dev->pdev->dev, &dev_attr_state);

	return ret;

initfail4:
	gpio_free(dev->dc_out_enable);
initfail3:
	platform_device_put(pdev);
initfail2:
	platform_driver_unregister(&archos_gadget_detect_driver);
initfail1:
	return ret;
}

void __exit archos_gadget_detect_exit(void) {
	return;
}

device_initcall(archos_gadget_detect_init);

