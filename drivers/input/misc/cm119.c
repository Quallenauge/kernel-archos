/*
 * Driver for the CM119 based Archos IR audio dock.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on cm109.c driver
 *
 * Small trick to use platform_data into this usb_driver :
 * We use a platform_driver to set a static cm119_platform_data. Therefore
 * cm119_usb_probe can use specific board file related data.
 *
 */

//#define DEBUG

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/rwsem.h>
#include <linux/usb/input.h>
#include <linux/platform_device.h>
#include <linux/input/cm119.h>
#include <linux/module.h>

#include "../../hid/hid-ids.h"

#define CM119_AMP_CONTROL_NAME	"cm119_amp_control"

#define DRIVER_AUTHOR  "Guillaume Revaillot"
#define DRIVER_DESC    "CM119 driver"

enum {
	/* HID Registers */
	HID_IR0 = 0x00, /* Record/Playback-mute button, Volume up/down  */
	HID_IR1 = 0x01, /* GPI, generic registers or EEPROM_DATA0       */
	HID_IR2 = 0x02, /* Generic registers or EEPROM_DATA1            */
	HID_IR3 = 0x03, /* Generic registers or EEPROM_CTRL             */

	HID_OR0 = 0x00, /* Mapping control, buzzer, SPDIF (offset 0x04) */
	HID_OR1 = 0x01, /* GPO - General Purpose Output                 */
	HID_OR2 = 0x02, /* Set GPIO to input/output mode                */
	HID_OR3 = 0x03, /* SPDIF status channel or EEPROM_CTRL          */
};

/* CM119 protocol packet */
struct cm119_ctl_packet {
	u8 byte[4];
} __attribute__ ((packed));

enum { USB_PKT_LEN = sizeof(struct cm119_ctl_packet) };

enum {
	RESET_SETUP = 1 << 0,
	NO_DEBOUNCE = 1 << 1,
	SHORT_DEBOUNCE = 1 << 2,
};

struct cm119_keymap {
	int val;
	int key;
	int flag;
};

static struct cm119_keymap keymap[] = {
	{ 1,	KEY_UNKNOWN, 		RESET_SETUP},	// switch from standby to running
	{ 2,	KEY_PREVIOUSSONG, 	0},		// previous soung (remote)
	{ 3,	KEY_NEXTSONG,		0},		// next song (remote)
	{ 4,	KEY_PLAYPAUSE,		0},		// play pause (remote)
	{ 5,	KEY_VOLUMEDOWN,		SHORT_DEBOUNCE},	// volume down key (remote and boombox)
	{ 6,	KEY_VOLUMEUP,		SHORT_DEBOUNCE},	// volume up key (remote and boombox)
	{ 7,	KEY_MUTE,		0},		// mute key
	{ 8,	KEY_PAUSE,		0},		// switch to standby mode
	{ 9,	KEY_PAUSE,		0},		// switch to aux source
	{ 10,	KEY_UNKNOWN, 		RESET_SETUP},	// switch to tablet source
};

/* CM119 device structure */
struct cm119_dev {
	struct input_dev *idev;	 /* input device */
	struct usb_device *udev; /* usb device */
	struct usb_interface *intf;
	struct platform_device *pdev;

	/* control output channel */
	struct cm119_ctl_packet *ctl_data;

	dma_addr_t ctl_dma;
	struct usb_ctrlrequest *ctl_req;
	struct urb *urb_ctl;
	struct completion urb_ctl_complete;

	/* irq input channel */
	struct cm119_ctl_packet *irq_data;
	dma_addr_t irq_dma;
	struct urb *urb_irq;
	struct completion urb_irq_complete;

	struct mutex pm_mutex;

	/* report work (debounce..) */
	struct delayed_work work;

	int last_key;
	int key_flag;

	long amp_volume;

	char phys[64];		/* physical device path */
};

static struct cm119_platform_data *cm119_pdata;

static int cm119_setup(struct cm119_dev * dev);

static void cm119_urb_irq_callback(struct urb *urb)
{
	struct cm119_dev *dev = urb->context;
	const int status = urb->status;
	int delay = 200;
	int i;

	dev_dbg(&urb->dev->dev, "%s: urb status %d", __func__, status);

	switch (urb->status) {
		case 0:			/* success */
			break;
		case -ECONNRESET:	/* unlink */
		case -ENOENT:
		case -ESHUTDOWN:
			return;
			/* -EPIPE:  should clear the halt */
		default:		/* error */
			goto resubmit;
	}

	dev_dbg(&urb->dev->dev, "### URB IRQ: [0x%02x 0x%02x 0x%02x 0x%02x]\n",
			dev->irq_data->byte[0],
			dev->irq_data->byte[1],
			dev->irq_data->byte[2],
			dev->irq_data->byte[3]);

	if (dev->irq_data->byte[1]) {
		// custom mcu keycode

		for (i = 0; i < ARRAY_SIZE(keymap); i++) {
			if (keymap[i].val == dev->irq_data->byte[1]) {
				dev->last_key = keymap[i].key;
				dev->key_flag = keymap[i].flag;
				break;
			}
		}

	} else if (dev->irq_data->byte[0]) {
		// regular keys
		switch (dev->irq_data->byte[0]) {
			case 0x01: dev->last_key = KEY_VOLUMEUP; break;
			case 0x02: dev->last_key = KEY_VOLUMEDOWN; break;
			case 0x04: dev->last_key = KEY_MUTE; break;
		}
	}

	if (dev->key_flag == NO_DEBOUNCE) {
		delay = 0;
		dev->key_flag = 0;
	} else if (dev->key_flag == SHORT_DEBOUNCE) {
		delay = 40;
		dev->key_flag = 0;
	}

	schedule_delayed_work(&dev->work, msecs_to_jiffies(delay));

	complete(&dev->urb_irq_complete);

resubmit:
	usb_submit_urb (urb, GFP_ATOMIC);
}

static void cm119_urb_ctl_callback(struct urb *urb)
{
	struct cm119_dev *dev = urb->context;
	const int status = urb->status;

	dev_dbg(&urb->dev->dev, "%s: urb status %d", __func__, status);

	switch (urb->status) {
		case 0:			/* success */
			break;
		case -ECONNRESET:	/* unlink */
		case -ENOENT:
		case -ESHUTDOWN:
			return;
			/* -EPIPE:  should clear the halt */
		default:		/* error */
			goto resubmit;
	}

	dev_dbg(&urb->dev->dev, "### URB CTL: [0x%02x 0x%02x 0x%02x 0x%02x]\n",
			dev->ctl_data->byte[0], dev->ctl_data->byte[1],
			dev->ctl_data->byte[2], dev->ctl_data->byte[3]);

	complete(&dev->urb_ctl_complete);

	return;

resubmit:
	usb_submit_urb (urb, GFP_ATOMIC);
}

static void cm119_stop_traffic(struct cm119_dev *dev)
{
	usb_kill_urb(dev->urb_ctl);
	usb_kill_urb(dev->urb_irq);
}
static int cm119_setup(struct cm119_dev * dev)
{
	int error;

	INIT_COMPLETION(dev->urb_ctl_complete);

	dev->ctl_data->byte[HID_OR0] = 1 << 6;
	dev->ctl_data->byte[HID_OR1] = dev->amp_volume;
	dev->ctl_data->byte[HID_OR2] = 0;
	dev->ctl_data->byte[HID_OR3] = 0;

	error = usb_submit_urb(dev->urb_ctl, GFP_KERNEL);
	if (error) {
		err("%s: usb_submit_urb (urb_ctl) failed %d\n", __func__, error);
		goto abort;
	} 

	error = wait_for_completion_interruptible_timeout(
			&dev->urb_ctl_complete,
			msecs_to_jiffies(500));

	if (error <= 0) {
		err("%s: usb_submit_urb (urb_ctl) timeout %d\n", __func__, error);
		goto abort;
	}

	return 0;
abort:
	return error;
}

static ssize_t _show_amp_volume(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cm119_dev * d = (struct cm119_dev *) dev->platform_data;

	return sprintf(buf, "%lu\n", d->amp_volume);
}

static ssize_t _store_amp_volume(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct cm119_dev * d = (struct cm119_dev *) dev->platform_data;
	unsigned long v;

	if (strict_strtoul(buf, 10, &v))
		return -EINVAL;

	if (v >= 0x20)
		return -EINVAL;

	d->amp_volume = v;

	cm119_setup(d);

	return count;
}
static DEVICE_ATTR(amp_volume, 00777, _show_amp_volume, _store_amp_volume);

static int cm119_input_open(struct input_dev *idev)
{
	struct cm119_dev *dev = input_get_drvdata(idev);
	int error;

	error = usb_autopm_get_interface(dev->intf);
	if (error < 0) {
		err("%s - cannot autoresume, result %d",
		    __func__, error);
		return error;
	}

	dev->amp_volume = 0x00;

	if ((error = cm119_setup(dev)))
		goto abort;

	error = usb_submit_urb(dev->urb_irq, GFP_ATOMIC);
	if (error)
		err("%s: usb_submit_urb (urb_irq) failed %d", __func__, error);

abort:
	if (error)
		usb_autopm_put_interface(dev->intf);

	return error;
}

static void cm119_input_close(struct input_dev *idev)
{
	struct cm119_dev *dev = input_get_drvdata(idev);

	mutex_lock(&dev->pm_mutex);

	cm119_stop_traffic(dev);

	mutex_unlock(&dev->pm_mutex);

	usb_autopm_put_interface(dev->intf);
}

static void cm119_report_func(struct work_struct *work)
{
	struct cm119_dev *dev =
		container_of(to_delayed_work(work), struct cm119_dev, work);
	struct input_dev *idev = dev->idev;
	int key = dev->last_key;

	dev->last_key = 0;

	dev_dbg(&idev->dev, "%s : %d\n", __func__, key);

	if (dev->key_flag == RESET_SETUP) {
		cm119_setup(dev);
		dev->key_flag = 0;
	}

	if (!key)
		return;

	input_report_key(idev, key, 1);
	input_sync(idev);

	input_report_key(idev, key, 0);
	input_sync(idev);
}

struct driver_info {
	char *name;
};

static const struct driver_info info_cm119 = {
	.name = "CM119 USB driver",
};

static const struct usb_device_id cm119_usb_table[] = {
	{
		.match_flags = USB_DEVICE_ID_MATCH_DEVICE |
				USB_DEVICE_ID_MATCH_INT_INFO,
		.idVendor =  USB_VENDOR_ID_CMEDIA,
		.idProduct = USB_DEVICE_ID_CM119,
		.bInterfaceClass = USB_CLASS_HID,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.driver_info = (kernel_ulong_t) &info_cm119
	},
	{ }
};

static void cm119_usb_cleanup(struct cm119_dev *dev)
{
	kfree(dev->ctl_req);
	if (dev->ctl_data)
		usb_free_coherent(dev->udev, USB_PKT_LEN,
				  dev->ctl_data, dev->ctl_dma);
	if (dev->irq_data)
		usb_free_coherent(dev->udev, USB_PKT_LEN,
				  dev->irq_data, dev->irq_dma);

	usb_free_urb(dev->urb_irq);	/* parameter validation in core/urb */
	usb_free_urb(dev->urb_ctl);	/* parameter validation in core/urb */
	kfree(dev);
}

static void cm119_usb_disconnect(struct usb_interface *interface)
{
	struct cm119_dev *dev = usb_get_intfdata(interface);

	device_remove_file(&dev->pdev->dev, &dev_attr_amp_volume);

	dev->pdev->dev.platform_data = NULL;

	platform_device_unregister(dev->pdev);

	usb_set_intfdata(interface, NULL);
	input_unregister_device(dev->idev);
	cm119_usb_cleanup(dev);
}

static int __init cm119_whitelist_probe(struct platform_device *pdev)
{
	cm119_pdata = pdev->dev.platform_data;

	if (!cm119_pdata) {
		printk(KERN_ERR "cm119_whitelist_probe: No platform data !!\n");
		return -ENODEV;
	}

	return 0;
}

static int cm119_usb_probe(struct usb_interface *intf,
			   const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct driver_info *nfo = (struct driver_info *)id->driver_info;
	struct usb_host_interface *interface;
	struct usb_endpoint_descriptor *endpoint;
	struct cm119_dev *dev;
	struct input_dev *input_dev = NULL;
	const char *path;
	int ret, pipe;
	int error = -ENOMEM;
	int i;

	/* load the CM119 driver only for the whitelisted path
	 * (see board file for platform data)
	 */
	path = kobject_get_path(&intf->dev.kobj, GFP_KERNEL);
	if (strcmp(path, cm119_pdata->path) != 0)
		return -ENODEV;

	interface = intf->cur_altsetting;
	endpoint = &interface->endpoint[0].desc;

	if (!usb_endpoint_is_int_in(endpoint))
		return -ENODEV;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->pm_mutex);

	dev->udev = udev;
	dev->intf = intf;

	dev->idev = input_dev = input_allocate_device();
	if (!input_dev)
		goto err_out;

	/* allocate usb buffers */
	dev->irq_data = usb_alloc_coherent(udev, USB_PKT_LEN,
					   GFP_KERNEL, &dev->irq_dma);
	if (!dev->irq_data)
		goto err_out;

	dev->ctl_data = usb_alloc_coherent(udev, USB_PKT_LEN,
					   GFP_KERNEL, &dev->ctl_dma);
	if (!dev->ctl_data)
		goto err_out;

	dev->ctl_req = kmalloc(sizeof(*(dev->ctl_req)), GFP_KERNEL);
	if (!dev->ctl_req)
		goto err_out;

	/* allocate urb structures */
	dev->urb_irq = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->urb_irq)
		goto err_out;

	dev->urb_ctl = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->urb_ctl)
		goto err_out;

	init_completion(&dev->urb_irq_complete);
	init_completion(&dev->urb_ctl_complete);

	/* get a handle to the interrupt data pipe */
	pipe = usb_rcvintpipe(udev, endpoint->bEndpointAddress);
	ret = usb_maxpacket(udev, pipe, usb_pipeout(pipe));
	if (ret != USB_PKT_LEN)
		err("invalid payload size %d, expected %d", ret, USB_PKT_LEN);

	usb_fill_int_urb(dev->urb_irq, udev, pipe, dev->irq_data,
			 USB_PKT_LEN,
			 cm119_urb_irq_callback, dev, endpoint->bInterval);
	dev->urb_irq->transfer_dma = dev->irq_dma;
	dev->urb_irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	dev->urb_irq->dev = udev;

	dev->ctl_req->bRequestType = USB_TYPE_CLASS | USB_RECIP_INTERFACE; // |
	//				USB_DIR_OUT;
	dev->ctl_req->bRequest = USB_REQ_SET_CONFIGURATION;
	dev->ctl_req->wValue = cpu_to_le16(0x200);
	dev->ctl_req->wIndex = cpu_to_le16(interface->desc.bInterfaceNumber);
	dev->ctl_req->wLength = cpu_to_le16(USB_PKT_LEN);

	usb_fill_control_urb(dev->urb_ctl, udev, usb_sndctrlpipe(udev, 0),
			     (void *)dev->ctl_req, dev->ctl_data, USB_PKT_LEN,
			     cm119_urb_ctl_callback, dev);

	dev->urb_ctl->transfer_dma = dev->ctl_dma;
	dev->urb_ctl->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	dev->urb_ctl->dev = udev;

	usb_make_path(udev, dev->phys, sizeof(dev->phys));
	strlcat(dev->phys, "/input0", sizeof(dev->phys));

	input_dev->name = nfo->name;
	input_dev->phys = dev->phys;
	usb_to_input_id(udev, &input_dev->id);
	input_dev->dev.parent = &intf->dev;

	input_set_drvdata(input_dev, dev);
	input_dev->open = cm119_input_open;
	input_dev->close = cm119_input_close;

	input_dev->evbit[0] = BIT_MASK(EV_KEY);

	// hardware keys
	__set_bit(KEY_VOLUMEUP, input_dev->keybit);
	__set_bit(KEY_VOLUMEDOWN, input_dev->keybit);
	__set_bit(KEY_MUTE, input_dev->keybit);

	for (i = 0; i < ARRAY_SIZE(keymap); i++)
		__set_bit(keymap[i].key, input_dev->keybit);

	INIT_DELAYED_WORK(&dev->work, cm119_report_func);

	usb_set_intfdata(intf, dev);

	error = input_register_device(dev->idev);
	if (error)
		goto err_out;

	dev->pdev = platform_device_register_simple(CM119_AMP_CONTROL_NAME, -1, NULL, 0);
	if (IS_ERR(dev->pdev))
		return ret;

	dev->pdev->dev.platform_data = dev;

	error = device_create_file(&dev->pdev->dev, &dev_attr_amp_volume);

	if (error) {
		dev_info(&dev->pdev->dev, "%s: could not create sysfs (%d))\n", __func__, error);
		goto err_out;
	}

	return 0;

 err_out:
	input_free_device(input_dev);
	cm119_usb_cleanup(dev);
	return error;
}

static int cm119_usb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct cm119_dev *dev = usb_get_intfdata(intf);

	dev_info(&intf->dev, "cm119: usb_suspend (event=%d)\n", message.event);

	mutex_lock(&dev->pm_mutex);
	cm119_stop_traffic(dev);
	mutex_unlock(&dev->pm_mutex);

	return 0;
}

static int cm119_usb_resume(struct usb_interface *intf)
{
	return 0;
}

static int cm119_usb_pre_reset(struct usb_interface *intf)
{
	struct cm119_dev *dev = usb_get_intfdata(intf);

	mutex_lock(&dev->pm_mutex);

	cm119_stop_traffic(dev);

	return 0;
}

static int cm119_usb_post_reset(struct usb_interface *intf)
{
	struct cm119_dev *dev = usb_get_intfdata(intf);

	mutex_unlock(&dev->pm_mutex);

	return 0;
}

static struct platform_driver archos_cm119_whitelist_driver = {
	.probe		= cm119_whitelist_probe,
	.driver = {
		.name		= "cm119-whitelist",
	},
};
static struct usb_driver cm119_driver = {
	.name		= "cm119",
	.probe		= cm119_usb_probe,
	.disconnect	= cm119_usb_disconnect,
	.suspend	= cm119_usb_suspend,
	.resume		= cm119_usb_resume,
	.reset_resume	= cm119_usb_resume,
	.pre_reset	= cm119_usb_pre_reset,
	.post_reset	= cm119_usb_post_reset,
	.id_table	= cm119_usb_table,
	.supports_autosuspend = 1,
};

static int cm119_init(void)
{
	return usb_register(&cm119_driver);
}

static int __init cm119_whitelist_init(void)
{
	int err;
	if ((err = platform_driver_register(&archos_cm119_whitelist_driver)) < 0)
		return err;

	return cm119_init();
}

static void cm119_exit(void)
{
	usb_deregister(&cm119_driver);
}

static void __exit cm119_whitelist_exit(void)
{
	cm119_exit();
}

module_init(cm119_whitelist_init);
module_exit(cm119_whitelist_exit);

MODULE_DEVICE_TABLE(usb, cm119_usb_table);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
