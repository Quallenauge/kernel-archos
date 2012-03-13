/*
 * archos_usb_xceiv - Archos specific USB transceiver driver, 
 *                    talking to OMAP OTG controller plus Archos Gen9 stuff
 *
 * Copyright (C) 2011 Archos S.A.
 * Contact: Niklas Schroeter <schroeter@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/usb.h>
#include <linux/usb/gpio_vbus.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/switch.h>
#include <linux/power_supply.h>

#include <plat/usb.h>

#define ARCHOS_USB_XCEIV_IRQ_FLAGS \
	( IRQF_SAMPLE_RANDOM | IRQF_SHARED \
	| IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING )

/* usb register definitions*/
#define USB_VENDOR_ID_LSB		0x00
#define USB_VENDOR_ID_MSB		0x01
#define USB_PRODUCT_ID_LSB		0x02
#define USB_PRODUCT_ID_MSB		0x03
#define USB_VBUS_CTRL_SET		0x04
#define USB_VBUS_CTRL_CLR		0x05
#define USB_ID_CTRL_SET			0x06
#define USB_ID_CTRL_CLR			0x07
#define USB_VBUS_INT_SRC		0x08
#define USB_VBUS_INT_LATCH_SET		0x09
#define USB_VBUS_INT_LATCH_CLR		0x0A
#define USB_VBUS_INT_EN_LO_SET		0x0B
#define USB_VBUS_INT_EN_LO_CLR		0x0C
#define USB_VBUS_INT_EN_HI_SET		0x0D
#define USB_VBUS_INT_EN_HI_CLR		0x0E
#define USB_ID_INT_SRC			0x0F
#define ID_GND				(1 << 0)
#define ID_FLOAT			(1 << 4)

#define USB_ID_INT_LATCH_SET		0x10
#define USB_ID_INT_LATCH_CLR		0x11


#define USB_ID_INT_EN_LO_SET		0x12
#define USB_ID_INT_EN_LO_CLR		0x13
#define USB_ID_INT_EN_HI_SET		0x14
#define USB_ID_INT_EN_HI_CLR		0x15
#define USB_OTG_ADP_CTRL		0x16
#define USB_OTG_ADP_HIGH		0x17
#define USB_OTG_ADP_LOW			0x18
#define USB_OTG_ADP_RISE		0x19
#define USB_OTG_REVISION		0x1A

/* to be moved to LDO */
#define TWL6030_MISC2			0xE5
#define TWL6030_CFG_LDO_PD2		0xF5
#define TWL6030_BACKUP_REG		0xFA

#define STS_HW_CONDITIONS		0x21
#define STS_USB_ID			(1 << 2)

#define USB_VBUS_CTRL_SET		0x04
#define USB_ID_CTRL_SET			0x06

/* In module TWL6030_MODULE_PM_RECEIVER */
#define VUSB_CFG_GRP			0x70
#define VUSB_CFG_TRANS			0x71
#define VUSB_CFG_STATE			0x72
#define VUSB_CFG_VOLTAGE		0x73

/* in module archos_twl6030_MODULE_MAIN_CHARGE*/

#define CHARGERUSB_CTRL1		0x08
#define CHARGERUSB_CTRL2		0x09
#define CHARGERUSB_CTRL3		0x0a

enum state {
	STATE_UNKNOWN = -1,
	STATE_NONE = 0,
	STATE_GND = 1,
	STATE_VBUS = 2,
	STATE_FLOAT = 3,
};

struct archos_usb_xceiv {
	struct otg_transceiver	otg;
	struct device		*dev;

	/* for vbus reporting with irqs disabled */
	spinlock_t		lock;

	struct regulator	*usb3v3;

	u8			asleep;

	struct work_struct	work;
	struct gpio_vbus_mach_info *vbus_info;
	bool 			vbus_enabled;
	enum state		state;
	
	struct regulator       *vbus_draw;
	
	struct switch_dev 	usb_switch;
};

/* internal define on top of container_of */
#define xceiv_to_archos_xceiv(x)	container_of((x), struct archos_usb_xceiv, otg);
#define switchdev_to_archos_xceiv(x)	container_of((x), struct archos_usb_xceiv, usb_switch);

static void __iomem *ctrl_base;

/*-------------------------------------------------------------------------*/

static inline int twl6030_writeb(struct archos_usb_xceiv *xceiv, u8 module,
						u8 data, u8 address)
{
	int ret = 0;

	ret = twl_i2c_write_u8(module, data, address);
	if (ret < 0)
		dev_dbg(xceiv->dev,
			"TWL6030:USB:Write[0x%x] Error %d\n", address, ret);
	return ret;
}

static inline int twl6030_readb(struct archos_usb_xceiv *xceiv, u8 module, u8 address)
{
	u8 data = 0;
	int ret = 0;

	ret = twl_i2c_read_u8(module, &data, address);
	if (ret >= 0)
		ret = data;
	else
		dev_dbg(xceiv->dev,
			"TWL6030:readb[0x%x,0x%x] Error %d\n",
					module, address, ret);

	return ret;
}

/*-------------------------------------------------------------------------*/

static int is_vbus_powered(struct gpio_vbus_mach_info *pdata)
{
	int vbus;

	vbus = gpio_get_value(pdata->gpio_vbus);
	if (pdata->gpio_vbus_inverted)
		vbus = !vbus;

	return vbus;
}

static int is_vbus_overcurrent(struct gpio_vbus_mach_info *pdata)
{
	int vbus;

	if (!gpio_is_valid(pdata->gpio_id))
		return 0;

	vbus = gpio_get_value(pdata->gpio_vbus_flag);
	if (pdata->gpio_vbus_flag_inverted)
		vbus = !vbus;

	return vbus;
}

static bool is_id_gnd(struct gpio_vbus_mach_info *pdata)
{
	int id;

	if (!gpio_is_valid(pdata->gpio_id))
		return 0;

	id = !gpio_get_value(pdata->gpio_id);

	if (pdata->gpio_id_inverted)
		id = !id;

	return (bool)id;
}

/*-------------------------------------------------------------------------*/

static int archos_twl6030_set_phy_clk(struct otg_transceiver *x, int on)
{
	struct archos_usb_xceiv *xceiv;
	struct device *dev;
	struct twl4030_usb_data *pdata;

	xceiv = xceiv_to_archos_xceiv(x);
	dev  = xceiv->dev;
	pdata = dev->platform_data;

	pdata->phy_set_clock(xceiv->dev, on);

	return 0;
}

static int archos_usb_xceiv_phy_init(struct otg_transceiver *x)
{
	struct archos_usb_xceiv *xceiv;
	struct device *dev;
	struct twl4030_usb_data *pdata;
	struct gpio_vbus_mach_info *info;
	
	xceiv = xceiv_to_archos_xceiv(x);
	info = xceiv->vbus_info;
	dev  = xceiv->dev;
	pdata = dev->platform_data;

	if (is_id_gnd(info))
		pdata->phy_power(xceiv->dev, 1, 1);
	else
		pdata->phy_power(xceiv->dev, 0, 1);

	return 0;
}

static void archos_usb_xceiv_phy_shutdown(struct otg_transceiver *x)
{
	struct archos_usb_xceiv *xceiv;
	struct device *dev;
	struct twl4030_usb_data *pdata;

	xceiv = xceiv_to_archos_xceiv(x);
	dev  = xceiv->dev;
	pdata = dev->platform_data;
	pdata->phy_power(xceiv->dev, 0, 0);
}

static int _usb_ldo_init(struct archos_usb_xceiv *xceiv)
{
	/* Set to OTG_REV 1.3 and turn on the ID_WAKEUP_COMP */
	twl6030_writeb(xceiv, TWL6030_MODULE_ID0 , 0x1, TWL6030_BACKUP_REG);

	/* Program CFG_LDO_PD2 register and set VUSB bit */
	twl6030_writeb(xceiv, TWL6030_MODULE_ID0 , 0x1, TWL6030_CFG_LDO_PD2);

	/* Program MISC2 register and set bit VUSB_IN_VBAT */
	twl6030_writeb(xceiv, TWL6030_MODULE_ID0 , 0x10, TWL6030_MISC2);

	if (!xceiv->usb3v3) {
		xceiv->usb3v3 = regulator_get(xceiv->dev, "vusb");
		if (IS_ERR(xceiv->usb3v3))
			return -ENODEV;
	}

	return 0;
}

static ssize_t archos_usb_xceiv_vbus_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct archos_usb_xceiv *xceiv = dev_get_drvdata(dev);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&xceiv->lock, flags);
	ret = sprintf(buf, "%s\n",
			(xceiv->state == STATE_VBUS) ? "on" : "off");
	spin_unlock_irqrestore(&xceiv->lock, flags);

	return ret;
}
static DEVICE_ATTR(vbus, 0444, archos_usb_xceiv_vbus_show, NULL);

/*
 * switch class handlers
 */

#define USB_SW_NAME "USB_SWITCH" 
static ssize_t usb_switch_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", sdev->name);
}

static ssize_t usb_switch_print_state(struct switch_dev *sdev, char *buf)
{
	ssize_t buflen = 0;
	struct archos_usb_xceiv *xceiv = switchdev_to_archos_xceiv(sdev);

	buflen = 0;
	switch (xceiv->state) {
	case STATE_UNKNOWN:
	default:
		buflen = sprintf(buf, "UNKNOWN\n");
		break;
	case STATE_VBUS:
		buflen = sprintf(buf, "ATTACHED_VBUS\n");
		break;
	case STATE_GND:
		buflen = sprintf(buf, "ATTACHED_HOST\n");
		break;
	case STATE_NONE:
		buflen = sprintf(buf, "DETACHED\n");
		break;
	}
	return buflen;
}

static void gpio_vbus_work(struct work_struct *work)
{
	struct archos_usb_xceiv *xceiv = container_of(work, struct archos_usb_xceiv, work);
	struct gpio_vbus_mach_info *info = xceiv->vbus_info;
	unsigned charger_type = POWER_SUPPLY_TYPE_MAINS;

#if 0
	printk("gpio_vbus_work: gnd: %d\n", is_id_gnd(info));
	printk("gpio_vbus_work: vbus_powered: %d\n", is_vbus_powered(info));
	printk("gpio_vbus_work: is_vbus_overcurrent: %d\n", is_vbus_overcurrent(info));
#endif

	/* force transition to floating mode if state before is unknown */
	if (xceiv->state == STATE_UNKNOWN) {
printk("STATE_UNKNOWN -> STATE_NONE\n");
		xceiv->state = STATE_NONE;
		xceiv->otg.last_event = USB_EVENT_NONE;

		atomic_notifier_call_chain(&xceiv->otg.notifier, USB_EVENT_NONE,
				&charger_type);

		xceiv->otg.default_a = false;
		xceiv->otg.state = OTG_STATE_UNDEFINED;

		if (xceiv->asleep) {
			regulator_disable(xceiv->usb3v3);
			xceiv->asleep = 0;
		}

	}
	
	if (xceiv->state == STATE_VBUS) {
		if (!is_vbus_powered(info)) {
printk("STATE_VBUS -> STATE_NONE\n");
			xceiv->state = STATE_NONE;
			xceiv->otg.last_event = USB_EVENT_NONE;

			atomic_notifier_call_chain(&xceiv->otg.notifier, USB_EVENT_NONE,
					&charger_type);

			if (xceiv->asleep) {
				regulator_disable(xceiv->usb3v3);
				xceiv->asleep = 0;
			}

			switch_set_state(&xceiv->usb_switch, xceiv->state);
			sysfs_notify(&xceiv->dev->kobj, NULL, "vbus");
		}
	} else if (xceiv->state == STATE_GND) {
		if (!is_id_gnd(info)) {
printk("STATE_GND -> STATE_NONE\n");
			xceiv->state = STATE_NONE;
			xceiv->otg.last_event = USB_EVENT_NONE;

			xceiv->otg.default_a = false;
			xceiv->otg.state = OTG_STATE_B_IDLE;

			atomic_notifier_call_chain(&xceiv->otg.notifier, USB_EVENT_NONE,
					&charger_type);
			switch_set_state(&xceiv->usb_switch, xceiv->state);

			if (xceiv->asleep) {
				regulator_disable(xceiv->usb3v3);
				xceiv->asleep = 0;
			}
		}	
	} else { 
		if (is_id_gnd(info)) {
printk("STATE_NONE -> STATE_GND\n");
			_usb_ldo_init(xceiv);
			regulator_enable(xceiv->usb3v3);
			xceiv->asleep = 1;

			xceiv->state = STATE_GND;
			xceiv->otg.last_event = USB_EVENT_ID;

			xceiv->otg.default_a = true;
			xceiv->otg.state = OTG_STATE_A_IDLE;

			atomic_notifier_call_chain(&xceiv->otg.notifier,
					USB_EVENT_ID, xceiv->otg.gadget);
		
			switch_set_state(&xceiv->usb_switch, xceiv->state);
		} else if (is_vbus_powered(info)) {
printk("STATE_NONE -> STATE_VBUS\n");
			_usb_ldo_init(xceiv);
			regulator_enable(xceiv->usb3v3);
			xceiv->asleep = 1;
			xceiv->otg.state = OTG_STATE_B_IDLE;

			xceiv->state = STATE_VBUS;
			xceiv->otg.last_event = USB_EVENT_VBUS;

			atomic_notifier_call_chain(&xceiv->otg.notifier,
					USB_EVENT_VBUS, &charger_type);
			sysfs_notify(&xceiv->dev->kobj, NULL, "vbus");
			switch_set_state(&xceiv->usb_switch, xceiv->state);
		}	
	}
}

/* VBUS change IRQ handler */
static irqreturn_t gpio_vbus_irq(int irq, void *data)
{
	struct archos_usb_xceiv *xceiv = data;
	schedule_work(&xceiv->work);
	return IRQ_HANDLED;
}

static int archos_usb_xceiv_set_suspend(struct otg_transceiver *x, int suspend)
{
	struct archos_usb_xceiv *xceiv = xceiv_to_archos_xceiv(x);
	xceiv->state = STATE_UNKNOWN;
	return 0;
}

static int archos_usb_xceiv_set_peripheral(struct otg_transceiver *x,
		struct usb_gadget *gadget)
{
	struct archos_usb_xceiv *xceiv;

	if (!x)
		return -ENODEV;

	xceiv = xceiv_to_archos_xceiv(x);
	xceiv->otg.gadget = gadget;
	if (!gadget)
		xceiv->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int archos_usb_xceiv_enable_irq(struct otg_transceiver *x)
{
	struct archos_usb_xceiv *xceiv = xceiv_to_archos_xceiv(x);

	xceiv->state = STATE_UNKNOWN;
	gpio_vbus_irq(-1, xceiv);
	return 0;
}

static int archos_usb_xceiv_set_vbus(struct otg_transceiver *x, bool enabled)
{
	struct regulator *vbus_draw;
	struct archos_usb_xceiv *xceiv = xceiv_to_archos_xceiv(x);

	vbus_draw = xceiv->vbus_draw;

	if (!vbus_draw)
		return -1;

	if (enabled) {
		if (!xceiv->vbus_enabled) {
			regulator_enable(vbus_draw);
			xceiv->vbus_enabled = true;
		}
	} else {
		if (xceiv->vbus_enabled) {
			regulator_disable(vbus_draw);
			xceiv->vbus_enabled = false;
		}
	}

	return 0;
}

static int archos_usb_xceiv_set_host(struct otg_transceiver *x, struct usb_bus *host)
{
	struct archos_usb_xceiv *xceiv;

	if (!x)
		return -ENODEV;

	xceiv = xceiv_to_archos_xceiv(x);
	xceiv->otg.host = host;
	if (!host)
		xceiv->otg.state = OTG_STATE_UNDEFINED;
	return 0;
}

static int __devinit archos_usb_xceiv_probe(struct platform_device *pdev)
{
	struct twl4030_usb_data *pdata;
	struct archos_usb_xceiv	*xceiv;
	int err, gpio, irq;
	
	xceiv = kzalloc(sizeof *xceiv, GFP_KERNEL);
	if (!xceiv)
		return -ENOMEM;

	pdata = pdev->dev.platform_data;
	if (!pdata || !pdata->platform)
		return -EINVAL;
	
	xceiv->vbus_info = (struct gpio_vbus_mach_info*)pdata->platform;
		
	xceiv->dev			= &pdev->dev;
	xceiv->otg.dev			= xceiv->dev;
	xceiv->otg.label		= "archos_xceiv";
	xceiv->otg.set_host		= archos_usb_xceiv_set_host;
	xceiv->otg.set_peripheral	= archos_usb_xceiv_set_peripheral;
	xceiv->otg.set_vbus		= archos_usb_xceiv_set_vbus;
	xceiv->otg.init			= archos_usb_xceiv_phy_init;
	xceiv->otg.shutdown		= archos_usb_xceiv_phy_shutdown;
	xceiv->otg.enable_irq		= archos_usb_xceiv_enable_irq;
	
	xceiv->state = STATE_UNKNOWN;
	
	/* init spinlock for workqueue */
	spin_lock_init(&xceiv->lock);

	err = _usb_ldo_init(xceiv);
	if (err) {
		dev_err(&pdev->dev, "ldo init failed\n");
		kfree(xceiv);
		return err;
	}

#if 0
	printk("phy usb vendor id: %02x%02x, product: %02x%02x\n", 
			twl6030_readb(xceiv, TWL_MODULE_USB, USB_VENDOR_ID_MSB),
			twl6030_readb(xceiv, TWL_MODULE_USB, USB_VENDOR_ID_LSB),
			twl6030_readb(xceiv, TWL_MODULE_USB, USB_PRODUCT_ID_MSB),
			twl6030_readb(xceiv, TWL_MODULE_USB, USB_PRODUCT_ID_LSB));
#endif

	platform_set_drvdata(pdev, xceiv);
	if (device_create_file(&pdev->dev, &dev_attr_vbus))
		dev_warn(&pdev->dev, "could not create sysfs file\n");

	ATOMIC_INIT_NOTIFIER_HEAD(&xceiv->otg.notifier);

	gpio = xceiv->vbus_info->gpio_vbus;
	err = gpio_request(gpio, "vbus_detect");
	if (err) {
		dev_err(&pdev->dev, "can't request vbus gpio %d, err: %d\n",
			gpio, err);
		goto err_gpio_vbus;
	}
	gpio_direction_input(gpio);

	//debounce gpio 1ms
	gpio_set_debounce(gpio, 1000);

	irq = gpio_to_irq(gpio);
	err = request_irq(irq, gpio_vbus_irq, ARCHOS_USB_XCEIV_IRQ_FLAGS,
			"vbus_detect", xceiv);
	if (err) {
		dev_err(&pdev->dev, "can't request irq %i, err: %d\n",
			irq, err);
		gpio_free(gpio);
		goto err_gpio_vbus;
	}

	gpio = xceiv->vbus_info->gpio_id;
	if (gpio_is_valid(gpio)) {
		err = gpio_request(gpio, "usb_id");
		if (err) {
			dev_err(&pdev->dev, "can't request id gpio %d, err: %d\n",
					gpio, err);
				goto err_gpio_id;
		}
		gpio_direction_input(gpio);

		irq = gpio_to_irq(gpio);
		err = request_irq(irq, gpio_vbus_irq, ARCHOS_USB_XCEIV_IRQ_FLAGS,
				"usb_id", xceiv);
		if (err) {
			dev_err(&pdev->dev, "can't request irq %i, err: %d\n",
				irq, err);
			gpio_free(gpio);
			goto err_gpio_id;
		}
		
	}

	gpio = xceiv->vbus_info->gpio_vbus_flag;
	if (gpio_is_valid(gpio)) {
		err = gpio_request(gpio, "vbus_flag");
		if (err) {
			dev_err(&pdev->dev, "can't request vbus flag gpio %d, err: %d\n",
					gpio, err);
				goto err_gpio_id;
		}
		gpio_direction_input(gpio);

		irq = gpio_to_irq(gpio);
		err = request_irq(irq, gpio_vbus_irq, ARCHOS_USB_XCEIV_IRQ_FLAGS,
				"vbus_flag", xceiv);
		if (err) {
			dev_err(&pdev->dev, "can't request irq %i, err: %d\n",
				irq, err);
			gpio_free(gpio);
			goto err_gpio_vbus_flag;
		}
	}

	INIT_WORK(&xceiv->work, gpio_vbus_work);

	/* only active when a gadget is registered */
	err = otg_set_transceiver(&xceiv->otg);
	if (err) {
		dev_err(&pdev->dev, "can't register transceiver, err: %d\n",
			err);
		goto err_otg;
	}

	xceiv->vbus_draw = regulator_get(&pdev->dev, "vbus_musb");
	if (IS_ERR(xceiv->vbus_draw)) {
		dev_dbg(&pdev->dev, "can't get vbus_draw regulator, err: %ld\n",
			PTR_ERR(xceiv->vbus_draw));
		xceiv->vbus_draw = NULL;
		goto err_otg;
	}

	/* initialize switches */
	xceiv->usb_switch.name = USB_SW_NAME;
	xceiv->usb_switch.print_name = usb_switch_print_name;
	xceiv->usb_switch.print_state = usb_switch_print_state;
	if (switch_dev_register(&xceiv->usb_switch) < 0) {
		dev_err(&pdev->dev, "Error creating USB switch\n");
		goto err_switch;
	}

	xceiv->asleep = 0;
	pdata->phy_init(xceiv->dev);
	archos_usb_xceiv_enable_irq(&xceiv->otg);
	
	dev_info(&pdev->dev, "Initialized Archos tranceiver module\n");
	return 0;

err_switch:
	regulator_put(xceiv->vbus_draw);
err_otg:
	free_irq(gpio_to_irq(xceiv->vbus_info->gpio_vbus_flag), &pdev->dev);
	if (gpio_is_valid(xceiv->vbus_info->gpio_vbus_flag)) {
		free_irq(gpio_to_irq(xceiv->vbus_info->gpio_vbus_flag), &pdev->dev);
		gpio_free(xceiv->vbus_info->gpio_vbus_flag);
	}
err_gpio_vbus_flag:
	free_irq(gpio_to_irq(xceiv->vbus_info->gpio_id), &pdev->dev);
	if (gpio_is_valid(xceiv->vbus_info->gpio_id)) {
		free_irq(gpio_to_irq(xceiv->vbus_info->gpio_id), &pdev->dev);
		gpio_free(xceiv->vbus_info->gpio_id);
	}
err_gpio_id:
	free_irq(gpio_to_irq(xceiv->vbus_info->gpio_vbus), &pdev->dev);
	gpio_free(xceiv->vbus_info->gpio_vbus);
err_gpio_vbus:
	kfree(xceiv);
	return err;
}

static int __exit archos_usb_xceiv_remove(struct platform_device *pdev)
{
	struct archos_usb_xceiv *xceiv = platform_get_drvdata(pdev);

	switch_dev_unregister(&xceiv->usb_switch);
		
	free_irq(gpio_to_irq(xceiv->vbus_info->gpio_vbus_flag), &pdev->dev);
	if (gpio_is_valid(xceiv->vbus_info->gpio_vbus_flag)) {
		free_irq(gpio_to_irq(xceiv->vbus_info->gpio_vbus_flag), &pdev->dev);
		gpio_free(xceiv->vbus_info->gpio_vbus_flag);
	}

	if (gpio_is_valid(xceiv->vbus_info->gpio_id)) {
		free_irq(gpio_to_irq(xceiv->vbus_info->gpio_id), xceiv);
		gpio_free(xceiv->vbus_info->gpio_id);
	}

	free_irq(gpio_to_irq(xceiv->vbus_info->gpio_vbus), xceiv);
	gpio_free(xceiv->vbus_info->gpio_vbus);

	regulator_put(xceiv->vbus_draw);
	regulator_put(xceiv->usb3v3);

	device_remove_file(xceiv->dev, &dev_attr_vbus);

	kfree(xceiv);

	return 0;
}

static struct platform_driver archos_usb_xceiv_driver = {
	.probe		= archos_usb_xceiv_probe,
	.remove		= __exit_p(archos_usb_xceiv_remove),
	.driver		= {
		.name	= "archos_usb_xceiv",
		.owner	= THIS_MODULE,
	},
};

static int __init archos_usb_xceiv_init(void)
{
	return platform_driver_register(&archos_usb_xceiv_driver);
}
subsys_initcall(archos_usb_xceiv_init);

static void __exit archos_usb_xceiv_exit(void)
{
	platform_driver_unregister(&archos_usb_xceiv_driver);
}
module_exit(archos_usb_xceiv_exit);

MODULE_ALIAS("platform:archos_usb_xceiv");
MODULE_AUTHOR("Archos S.A.");
MODULE_DESCRIPTION("Archos specific USB transceiver driver");
MODULE_LICENSE("GPL");
