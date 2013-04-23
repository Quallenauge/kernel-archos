/*
 *  HID driver for some archos "special" devices
 *
 *  Copyright (c) 2012 Archos SA
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/hidraw.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>

#include <linux/sched.h>
#include <linux/platform_device.h>

#include "hid-ids.h"

#define STATE_DISCONNECTED		0
#define STATE_CONNECTED_SLEEPING	1
#define STATE_CONNECTED_ENABLED		2

static struct archos_kbd_state_struct {
	struct platform_device *pdev;
	int mask_removal;
	int intf;
	int state;
	struct hid_input *hidinput;
} archos_kbd_state;

struct archos_kbd_data {
	unsigned long quirks;
};

static unsigned char ptr[] = "disconnected";

void hidinput_configure_usage(struct hid_input *hidinput, struct hid_field *field, struct hid_usage *usage);

static const struct hid_device_id archos_kbd_devices[] = {
	{ HID_USB_DEVICE(0x1c40, 0x04d2) },
	{ }
};
MODULE_DEVICE_TABLE(hid, archos_kbd_devices);

static int archos_kbd_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
	unsigned long quirks = id->driver_data;
	struct archos_kbd_data *akd;

	hid_info(hdev, "%s\n", __func__);

	if (archos_kbd_state.intf)
		return -ENODEV;

	archos_kbd_state.intf++;

	akd = kzalloc(sizeof(*akd), GFP_KERNEL);
	if (akd == NULL) {
		hid_err(hdev, "can't alloc archos keyboard descriptor\n");
		return -ENOMEM;
	}

	akd->quirks = quirks;
	hid_set_drvdata(hdev, akd);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		goto err_free;
	}

	if (archos_kbd_state.state == STATE_DISCONNECTED) {
		ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	} else {
		struct hid_input * hidinput = archos_kbd_state.hidinput;
		int i, j, k;

		hid_info(hdev, "was sleeping, replug\n");

		archos_kbd_state.hidinput = NULL;

		hdev->quirks |= HID_QUIRK_SKIP_INPUT_REPORTS |
			HID_QUIRK_SKIP_OUTPUT_REPORTS;
		ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
		hdev->quirks &= ~HID_QUIRK_SKIP_INPUT_REPORTS;

		input_set_drvdata(hidinput->input, hdev);

		hidinput->input->name = hdev->name;
		hidinput->input->phys = hdev->phys;
		hidinput->input->uniq = hdev->uniq;
		hidinput->input->dev.parent = hdev->dev.parent;

		list_add_tail(&hidinput->list, &hdev->inputs);

		// XXX refactor
		for (k = HID_INPUT_REPORT; k <= HID_OUTPUT_REPORT; k++) {
			struct hid_report *report;

			list_for_each_entry(report, &hdev->report_enum[k].report_list, list) {

			if (!report->maxfield)
				continue;

			for (i = 0; i < report->maxfield; i++)
				for (j = 0; j < report->field[i]->maxusage; j++)
					hidinput_configure_usage(hidinput, report->field[i],
								 report->field[i]->usage + j);
			}
		}

		hdev->ll_driver->open(hdev);
	}


	if (ret) {
		hid_err(hdev, "hw start failed\n");
		goto err_free;
	}

	archos_kbd_state.state = STATE_CONNECTED_ENABLED;
	return 0;

	hid_hw_stop(hdev);
err_free:
	kfree(akd);
	return ret;
}

static void archos_kbd_remove(struct hid_device *hdev)
{
	archos_kbd_state.state = STATE_DISCONNECTED;

	archos_kbd_state.intf = 0;

	if (archos_kbd_state.mask_removal) {
		hid_info(hdev, "goto sleep\n");

		archos_kbd_state.state = STATE_CONNECTED_SLEEPING;

		archos_kbd_state.hidinput = list_first_entry(&hdev->inputs, struct hid_input, list);

		archos_kbd_state.hidinput->input->name = ptr;
		archos_kbd_state.hidinput->input->phys = ptr;
		archos_kbd_state.hidinput->input->uniq = ptr;

		hdev->claimed &= ~HID_CLAIMED_INPUT;
		hid_disconnect(hdev);
		hdev->claimed |= HID_CLAIMED_INPUT;

		hdev->ll_driver->stop(hdev);
	} else {
		hid_hw_stop(hdev);
	}

	kfree(hid_get_drvdata(hdev));
}

static void archos_kbd_cleanup(void)
{
	if (archos_kbd_state.state != STATE_CONNECTED_SLEEPING)
		return;

	if (!archos_kbd_state.hidinput)
		return;

	archos_kbd_state.state = STATE_DISCONNECTED;

	archos_kbd_state.hidinput->input->close = NULL;
	input_unregister_device(archos_kbd_state.hidinput->input);
	kfree(archos_kbd_state.hidinput);
	archos_kbd_state.hidinput = NULL;

	return;
}

static struct hid_driver archos_kbd_driver = {
	.name = "Archos",
	.id_table = archos_kbd_devices,
	.probe = archos_kbd_probe,
	.remove = archos_kbd_remove,
};

static ssize_t _show_mask_removal(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", archos_kbd_state.mask_removal);
}


static ssize_t _store_mask_removal(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long v;

	if (strict_strtoul(buf, 10, &v) < 0)
		return -EINVAL;

	archos_kbd_state.mask_removal = v;

	pr_info("mask_removal = %d\n", archos_kbd_state.mask_removal);

	return count;
}
static DEVICE_ATTR(mask_removal, S_IWUSR | S_IRUGO, _show_mask_removal, _store_mask_removal);

static ssize_t _store_force_removal(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long v;

	pr_info("%s\n", __func__);

	if (strict_strtoul(buf, 10, &v) < 0)
		return -EINVAL;

	archos_kbd_cleanup();

	return count;
}
static DEVICE_ATTR(force_removal, S_IWUSR | S_IRUGO, NULL, _store_force_removal);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_mask_removal.attr,
	&dev_attr_force_removal.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = sysfs_attrs,
};

static int __init archos_kbd_init(void)
{
	archos_kbd_state.state = STATE_DISCONNECTED;
	archos_kbd_state.intf = 0;
	archos_kbd_state.mask_removal = 0;

	archos_kbd_state.pdev = platform_device_register_simple("hid_archos_ctrl", -1, NULL, 0);
	if (IS_ERR(archos_kbd_state.pdev))
		return -ENODEV;

	if (sysfs_create_group(&archos_kbd_state.pdev->dev.kobj, &attr_group) < 0)
		return -ENODEV;

	return hid_register_driver(&archos_kbd_driver);
}

static void __exit archos_kbd_exit(void)
{
	archos_kbd_cleanup();

	sysfs_remove_group(&archos_kbd_state.pdev->dev.kobj, &attr_group);
	platform_device_unregister(archos_kbd_state.pdev);

	hid_unregister_driver(&archos_kbd_driver);
}

module_init(archos_kbd_init);
module_exit(archos_kbd_exit);
MODULE_LICENSE("GPL");
