/*
 * archos_twl6030_usb_xceiv - Archos specific USB transceiver driver, 
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
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/switch.h>

#include <linux/power_supply.h>

#include <plat/usb.h>

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


/* In module archos_twl6030_MODULE_PM_MASTER */
#define PROTECT_KEY			0x0E
#define STS_HW_CONDITIONS		0x21

#define VUSB_CFG_GRP			0x70
#define VUSB_CFG_TRANS			0x71
#define VUSB_CFG_STATE			0x72
#define VUSB_CFG_VOLTAGE		0x73

/* in module archos_twl6030_MODULE_MAIN_CHARGE*/

#define CHARGERUSB_CTRL1		0x08
#define CHARGERUSB_CTRL2		0x09
#define CHARGERUSB_CTRL3		0x0a

#define CONTROLLER_STAT1		0x03
#define	VBUS_DET			(1 << 2)
#define CHRG_DET_N			(1 << 5)

enum state {
	STATE_UNKNOWN = -1,
	STATE_NONE = 0,
	STATE_GND = 1,
	STATE_VBUS = 2,
	STATE_VBUS_TRANS = 3,
	STATE_FLOAT = 4,
	STATE_CHARGER = 5,
};

struct archos_twl6030_usb {
	struct otg_transceiver	otg;
	struct device		*dev;

	/* for vbus reporting with irqs disabled */
	spinlock_t		lock;

	struct regulator	*usb3v3;

	int			irq1;
	int			irq2;	
	unsigned int		usb_cinlimit_mA;
	u8			linkstat;
	u8			asleep;
	u8 			phy_powered;
	bool			irq_enabled;
	int			prev_vbus;

	struct delayed_work	work;
	enum state		state;
	struct regulator       *vbus_draw;	
	bool 			vbus_enabled;
	struct switch_dev 	usb_switch;
	unsigned long		features;
};

/* internal define on top of container_of */
#define xceiv_to_twl(x)		container_of((x), struct archos_twl6030_usb, otg);
#define switchdev_to_archos_xceiv(x)	container_of((x), struct archos_twl6030_usb, usb_switch);

/*-------------------------------------------------------------------------*/

static inline int archos_twl6030_writeb(struct archos_twl6030_usb *twl, u8 module,
						u8 data, u8 address)
{
	int ret = 0;

	ret = twl_i2c_write_u8(module, data, address);
	if (ret < 0)
		dev_dbg(twl->dev,
			"TWL6030:USB:Write[0x%x] Error %d\n", address, ret);
	return ret;
}

static inline int archos_twl6030_readb(struct archos_twl6030_usb *twl, u8 module, u8 address)
{
	u8 data = 0;
	int ret = 0;

	ret = twl_i2c_read_u8(module, &data, address);
	if (ret >= 0)
		ret = data;
	else
		dev_dbg(twl->dev,
			"TWL6030:readb[0x%x,0x%x] Error %d\n",
					module, address, ret);

	return ret;
}

/*-------------------------------------------------------------------------*/
static int archos_twl6030_set_phy_clk(struct otg_transceiver *x, int on)
{
	struct archos_twl6030_usb *twl;
	struct device *dev;
	struct twl4030_usb_data *pdata;

	twl = xceiv_to_twl(x);
	dev  = twl->dev;
	pdata = dev->platform_data;

	pdata->phy_set_clock(twl->dev, on);

	return 0;
}

static int archos_twl6030_phy_init(struct otg_transceiver *x)
{
	u8 hw_state;
	struct archos_twl6030_usb *twl;
	struct device *dev;
	struct twl4030_usb_data *pdata;

	twl = xceiv_to_twl(x);
	dev  = twl->dev;
	pdata = dev->platform_data;

	if (!twl->asleep) {
		regulator_enable(twl->usb3v3);
		twl->asleep = 1;
	}
	hw_state = archos_twl6030_readb(twl, TWL6030_MODULE_ID0, STS_HW_CONDITIONS);

	if (!twl->phy_powered) {
		if (hw_state & STS_USB_ID)
			pdata->phy_power(twl->dev, 1, 1);
		else
			pdata->phy_power(twl->dev, 0, 1);
		twl->phy_powered = 1;
		pr_err("PHY ON\n");
	}
	return 0;
}

static void archos_twl6030_phy_shutdown(struct otg_transceiver *x)
{
	struct archos_twl6030_usb *twl;
	struct device *dev;
	struct twl4030_usb_data *pdata;

	twl = xceiv_to_twl(x);

	if (!twl->phy_powered)
		return;
		
	/* USB_VBUS_CTRL_CLR */
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0xFF, 0x05);
	/* USB_ID_CRTL_CLR */
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0xFF, 0x07);
        /* CHARGERUSB_CTRL3 */
        twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x21, 0xEA);


	dev  = twl->dev;
	pdata = dev->platform_data;
	pdata->phy_power(twl->dev, 0, 0);
	pr_err("PHY OFF\n");

	twl->phy_powered = 0;
}

static int archos_twl6030_usb_ldo_init(struct archos_twl6030_usb *twl)
{
	char *regulator_name;

	if (twl->features & TWL6032_SUBCLASS)
		regulator_name = "ldousb";
	else
		regulator_name = "vusb";

	/* Set to OTG_REV 1.3 and turn on the ID_WAKEUP_COMP */
	archos_twl6030_writeb(twl, TWL6030_MODULE_ID0 , 0x1, TWL6030_BACKUP_REG);

	/* Program CFG_LDO_PD2 register and set VUSB bit */
	archos_twl6030_writeb(twl, TWL6030_MODULE_ID0 , 0x1, TWL6030_CFG_LDO_PD2);

	/* Program MISC2 register and set bit VUSB_IN_VBAT */
	archos_twl6030_writeb(twl, TWL6030_MODULE_ID0 , 0x10, TWL6030_MISC2);

	if (!twl->usb3v3) {
		twl->usb3v3 = regulator_get(twl->dev, regulator_name);
		if (IS_ERR(twl->usb3v3))
			return -ENODEV;
	}

	/* Program the USB_VBUS_CTRL_SET and set VBUS_ACT_COMP bit */
	archos_twl6030_writeb(twl, TWL_MODULE_USB, 0x4, USB_VBUS_CTRL_SET);

	/*
	 * Program the USB_ID_CTRL_SET register to enable GND drive
	 * and the ID comparators
	 */
	archos_twl6030_writeb(twl, TWL_MODULE_USB, 0x14, USB_ID_CTRL_SET);

	/* CHARGERUSB_CTRL3 */
	archos_twl6030_writeb(twl, TWL6030_MODULE_ID1, 0x01, 0xEA);

	return 0;
}

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
	struct archos_twl6030_usb *xceiv = switchdev_to_archos_xceiv(sdev);

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
	case STATE_CHARGER:
		buflen = sprintf(buf, "ATTACHED_CHARGER\n");
		break;
	}
	return buflen;
}

static ssize_t archos_twl6030_usb_vbus_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct archos_twl6030_usb *twl = dev_get_drvdata(dev);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&twl->lock, flags);
	ret = sprintf(buf, "%s\n",
			(twl->linkstat == USB_EVENT_VBUS) ? "on" : "off");
	spin_unlock_irqrestore(&twl->lock, flags);

	return ret;
}

static DEVICE_ATTR(vbus, 0444, archos_twl6030_usb_vbus_show, NULL);

static void linkstat_work(struct work_struct *work)
{
	u8 vbus_state, hw_state;
	int status = USB_EVENT_NONE;
	unsigned charger_type;
	struct archos_twl6030_usb *twl = container_of(work, struct archos_twl6030_usb, work.work);
	
	hw_state = archos_twl6030_readb(twl, TWL6030_MODULE_ID0, STS_HW_CONDITIONS);	

	vbus_state = archos_twl6030_readb(twl, TWL_MODULE_MAIN_CHARGE,
						CONTROLLER_STAT1);
		
//printk("hw_state: %x\n", hw_state);
//printk("vbus_state: %x\n", vbus_state);
	/* force transition to floating mode if state before is unknown */
	if (twl->state == STATE_UNKNOWN) {
printk("STATE_UNKNOWN -> STATE_NONE\n");
		twl->state = STATE_NONE;
		twl->otg.last_event = status;

		if (twl->asleep) {
			regulator_disable(twl->usb3v3);
			twl->asleep = 0;
		}

		atomic_notifier_call_chain(&twl->otg.notifier, USB_EVENT_NONE,
				twl->otg.gadget);
	}

		
/*	if (twl->state == STATE_VBUS_TRANS) {
		if (!(vbus_state & VBUS_DET)) {
			twl->state = STATE_NONE;
			archos_twl6030_phy_shutdown(&twl->otg);			
			if (twl->asleep) {
				regulator_disable(twl->usb3v3);
				twl->asleep = 0;
			}
		} else if (vbus_state & CHRG_DET_N) {
printk("STATE_VBUS_TRANS -> STATE_CHARGER\n");
			twl->state = STATE_CHARGER;
			switch_set_state(&twl->usb_switch, twl->state);
			sysfs_notify(&twl->dev->kobj, NULL, "vbus");
		} else {
		
printk("STATE_VBUS_TRANS -> STATE_VBUS\n");
			twl->otg.state = OTG_STATE_B_IDLE;
		
			twl->state = STATE_VBUS;
			twl->linkstat = USB_EVENT_VBUS;
			
			atomic_notifier_call_chain(&twl->otg.notifier,
					USB_EVENT_VBUS, twl->otg.gadget);
			switch_set_state(&twl->usb_switch, twl->state);
			sysfs_notify(&twl->dev->kobj, NULL, "vbus");
		}
	}	
*/
	if (twl->state == STATE_VBUS || twl->state == STATE_CHARGER) {
		if (!(vbus_state & VBUS_DET)) {
			enum state state = twl->state;
printk("STATE_VBUS -> STATE_NONE\n");
			twl->state = STATE_NONE;
			twl->linkstat = USB_EVENT_NONE;
			twl->otg.last_event = status;

			atomic_notifier_call_chain(&twl->otg.notifier, USB_EVENT_NONE,
					twl->otg.gadget);

			if (twl->asleep) {
				regulator_disable(twl->usb3v3);
				twl->asleep = 0;
			}
			
			switch_set_state(&twl->usb_switch, twl->state);
			sysfs_notify(&twl->dev->kobj, NULL, "vbus");

			archos_twl6030_phy_shutdown(&twl->otg);
			
		}
	} else if (twl->state == STATE_GND) {
		if (!(hw_state & STS_USB_ID)) {
printk("STATE_GND -> STATE_NONE\n");
			archos_twl6030_writeb(twl, TWL_MODULE_USB, 0x10, USB_ID_INT_EN_HI_CLR);
			archos_twl6030_writeb(twl, TWL_MODULE_USB,  0x1, USB_ID_INT_EN_HI_SET);

			twl->state = STATE_NONE;
			twl->otg.last_event = status;

			atomic_notifier_call_chain(&twl->otg.notifier, USB_EVENT_NONE,
					twl->otg.gadget);
			switch_set_state(&twl->usb_switch, twl->state);

			if (twl->asleep) {
				regulator_disable(twl->usb3v3);
				twl->asleep = 0;
			}
		}	
	} else { 
		if (hw_state & STS_USB_ID) {
printk("STATE_NONE -> STATE_GND\n");
			archos_twl6030_usb_ldo_init(twl);
			regulator_enable(twl->usb3v3);
			twl->asleep = 1;
			archos_twl6030_writeb(twl, TWL_MODULE_USB,  0x1, USB_ID_INT_EN_HI_CLR);
			archos_twl6030_writeb(twl, TWL_MODULE_USB, 0x10, USB_ID_INT_EN_HI_SET);

			twl->state = STATE_GND;
			twl->otg.last_event = USB_EVENT_ID;
			
			twl->otg.default_a = true;
			twl->otg.state = OTG_STATE_A_IDLE;
			atomic_notifier_call_chain(&twl->otg.notifier,
					USB_EVENT_ID, twl->otg.gadget);
		
			switch_set_state(&twl->usb_switch, twl->state);
		} else if (vbus_state & VBUS_DET) {
			archos_twl6030_usb_ldo_init(twl);
			regulator_enable(twl->usb3v3);
			//archos_twl6030_phy_init(&twl->otg);
			twl->asleep = 1;
			//twl->state = STATE_VBUS_TRANS;
			charger_type = omap4_charger_detect();
			if ((charger_type == POWER_SUPPLY_TYPE_USB_CDP) 
				|| (charger_type == POWER_SUPPLY_TYPE_USB)) {
printk("STATE_NONE -> STATE_VBUS\n");
				status = USB_EVENT_VBUS;
				twl->otg.last_event = status;
				twl->otg.default_a = false;
				twl->otg.state = OTG_STATE_B_IDLE;
				twl->linkstat = USB_EVENT_VBUS;
				twl->state = STATE_VBUS;
				twl->usb_cinlimit_mA = 500;
			} else if (charger_type == POWER_SUPPLY_TYPE_USB_DCP) {
printk("STATE_NONE -> STATE_CHARGER\n");
				status = USB_EVENT_CHARGER;
				twl->otg.last_event = status;
				twl->usb_cinlimit_mA = 1500;
				twl->state = STATE_CHARGER;
			}
			twl->prev_vbus = (vbus_state & VBUS_DET);
			switch_set_state(&twl->usb_switch, twl->state);
			atomic_notifier_call_chain(&twl->otg.notifier, status, &charger_type);
			sysfs_notify(&twl->dev->kobj, NULL, "vbus");
		}	
	}
	
	archos_twl6030_writeb(twl, TWL_MODULE_USB, status, USB_ID_INT_LATCH_CLR);
	twl->linkstat = status;

	if (twl->state == STATE_GND)
		schedule_delayed_work(&twl->work, HZ/10);
}

static irqreturn_t archos_twl6030_usb_irq(int irq, void *_twl)
{
	u8 vbus_state;
	struct archos_twl6030_usb *twl = _twl;
	
	schedule_delayed_work(&twl->work, 0);
	
	return IRQ_HANDLED;
}

static irqreturn_t archos_twl6030_usbotg_irq(int irq, void *_twl)
{
	struct archos_twl6030_usb *twl = _twl;
	schedule_delayed_work(&twl->work, 0);
	
	return IRQ_HANDLED;
}

static int archos_twl6030_set_peripheral(struct otg_transceiver *x,
		struct usb_gadget *gadget)
{
	struct archos_twl6030_usb *twl;

	if (!x)
		return -ENODEV;

	twl = xceiv_to_twl(x);
	twl->otg.gadget = gadget;
	if (!gadget)
		twl->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int archos_twl6030_enable_irq(struct otg_transceiver *x)
{
	struct archos_twl6030_usb *twl = xceiv_to_twl(x);
	twl->state = STATE_UNKNOWN;
	
	archos_twl6030_writeb(twl, TWL_MODULE_USB, USB_ID_INT_EN_HI_SET, 0x1);
	twl6030_interrupt_unmask(0x05, REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(0x05, REG_INT_MSK_STS_C);

	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_STS_C);

	archos_twl6030_usb_irq(twl->irq2, twl);
	return 0;
}

unsigned int archos_twl6030_get_usb_max_power(struct otg_transceiver *x)
{
       struct archos_twl6030_usb *twl = xceiv_to_twl(x);

       return twl->usb_cinlimit_mA;
}

static int archos_twl6030_set_vbus(struct otg_transceiver *x, bool enabled)
{
	struct regulator *vbus_draw;
	struct archos_twl6030_usb *twl = xceiv_to_twl(x);

	vbus_draw = twl->vbus_draw;
	
	if (!vbus_draw)
		return -1;

	if (enabled) {
		if (!twl->vbus_enabled) {
			regulator_enable(vbus_draw);
			twl->vbus_enabled = true;
		}
	} else {
		if (twl->vbus_enabled) {
			regulator_disable(vbus_draw);
			twl->vbus_enabled = false;
		}
	}

	return 0;
}

static int archos_twl6030_set_power(struct otg_transceiver *x, unsigned int mA)
{
       struct archos_twl6030_usb *twl = xceiv_to_twl(x);

       twl->usb_cinlimit_mA = mA;
       atomic_notifier_call_chain(&twl->otg.notifier, USB_EVENT_ENUMERATED,
                               &twl->usb_cinlimit_mA);
       return 0;
}

static int archos_twl6030_set_host(struct otg_transceiver *x, struct usb_bus *host)
{
	struct archos_twl6030_usb *twl;

	if (!x)
		return -ENODEV;

	twl = xceiv_to_twl(x);
	twl->otg.host = host;
	if (!host)
		twl->otg.state = OTG_STATE_UNDEFINED;
	return 0;
}

static int __devinit archos_twl6030_usb_probe(struct platform_device *pdev)
{
	struct archos_twl6030_usb	*twl;
	int			status, err;
	struct twl4030_usb_data *pdata;
	struct device *dev = &pdev->dev;
	pdata = dev->platform_data;

	twl = kzalloc(sizeof *twl, GFP_KERNEL);
	if (!twl)
		return -ENOMEM;

	twl->dev		= &pdev->dev;
	twl->irq1		= platform_get_irq(pdev, 0);
	twl->irq2		= platform_get_irq(pdev, 1);
	twl->otg.dev		= twl->dev;
	twl->otg.label		= "twl6030";
	twl->otg.set_host	= archos_twl6030_set_host;
	twl->otg.set_peripheral	= archos_twl6030_set_peripheral;
	twl->asleep		= 1;
	twl->otg.set_vbus	= archos_twl6030_set_vbus;
	twl->otg.init           = archos_twl6030_phy_init;
	twl->otg.shutdown       = archos_twl6030_phy_shutdown;
	twl->otg.enable_irq	= archos_twl6030_enable_irq;
	twl->features		= pdata->features;
	
	twl->state = STATE_UNKNOWN;

	/* init spinlock for workqueue */
	spin_lock_init(&twl->lock);

	err = archos_twl6030_usb_ldo_init(twl);
	if (err) {
		dev_err(&pdev->dev, "ldo init failed\n");
		kfree(twl);
		return err;
	}

	platform_set_drvdata(pdev, twl);
	if (device_create_file(&pdev->dev, &dev_attr_vbus))
		dev_warn(&pdev->dev, "could not create sysfs file\n");

	ATOMIC_INIT_NOTIFIER_HEAD(&twl->otg.notifier);

	twl->irq_enabled = true;
	status = request_irq(twl->irq1, archos_twl6030_usbotg_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"archos_twl6030_usb", twl);
	if (status < 0) {
		dev_dbg(&pdev->dev, "can't get IRQ %d, err %d\n",
			twl->irq1, status);
		device_remove_file(twl->dev, &dev_attr_vbus);
		kfree(twl);
		return status;
	}

	status = request_irq(twl->irq2, archos_twl6030_usb_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"archos_twl6030_usb", twl);
	if (status < 0) {
		dev_dbg(&pdev->dev, "can't get IRQ %d, err %d\n",
			twl->irq2, status);
		free_irq(twl->irq1, twl);
		device_remove_file(twl->dev, &dev_attr_vbus);
		kfree(twl);
		return status;
	}

	INIT_DELAYED_WORK(&twl->work, linkstat_work);
	
	/* only active when a gadget is registered */
	err = otg_set_transceiver(&twl->otg);
	if (err) {
		dev_err(&pdev->dev, "can't register transceiver, err: %d\n",
			err);
		goto err_otg;
	}

	twl->vbus_draw = regulator_get(&pdev->dev, "vbus_musb");
	if (IS_ERR(twl->vbus_draw)) {
		dev_dbg(&pdev->dev, "can't get vbus_draw regulator, err: %ld\n",
			PTR_ERR(twl->vbus_draw));
		twl->vbus_draw = NULL;
		goto err_otg;
	}

	/* initialize switches */
	twl->usb_switch.name = USB_SW_NAME;
	twl->usb_switch.print_name = usb_switch_print_name;
	twl->usb_switch.print_state = usb_switch_print_state;
	if (switch_dev_register(&twl->usb_switch) < 0) {
		dev_err(&pdev->dev, "Error creating USB switch\n");
		goto err_switch;
	}

	twl->asleep = 0;
	pdata->phy_init(dev);
	archos_twl6030_enable_irq(&twl->otg);
	
	dev_err(&pdev->dev, "Initialized Archos TWL6030 USB module\n");
	return 0;
err_switch:
	regulator_put(twl->vbus_draw);
err_otg:
	device_remove_file(twl->dev, &dev_attr_vbus);
	free_irq(twl->irq1, twl);
	free_irq(twl->irq2, twl);
	kfree(twl);
	return err;
}

static int __exit archos_twl6030_usb_remove(struct platform_device *pdev)
{
	struct archos_twl6030_usb *twl = platform_get_drvdata(pdev);
	struct twl4030_usb_data *pdata;
	struct device *dev = &pdev->dev;
	pdata = dev->platform_data;

	switch_dev_unregister(&twl->usb_switch);
	
	twl6030_interrupt_mask(TWL6030_USBOTG_INT_MASK,
		REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_USBOTG_INT_MASK,
			REG_INT_MSK_STS_C);
	
	free_irq(twl->irq1, twl);
	free_irq(twl->irq2, twl);
	regulator_put(twl->vbus_draw);
	regulator_put(twl->usb3v3);
	pdata->phy_exit(twl->dev);
	device_remove_file(twl->dev, &dev_attr_vbus);
	kfree(twl);

	return 0;
}

static struct platform_driver archos_twl6030_usb_driver = {
	.probe		= archos_twl6030_usb_probe,
	.remove		= __exit_p(archos_twl6030_usb_remove),
	.driver		= {
		.name	= "archos_twl6030_usb_xceiv",
		.owner	= THIS_MODULE,
	},
};

static int __init archos_twl6030_usb_init(void)
{
	return platform_driver_register(&archos_twl6030_usb_driver);
}
subsys_initcall(archos_twl6030_usb_init);

static void __exit archos_twl6030_usb_exit(void)
{
	platform_driver_unregister(&archos_twl6030_usb_driver);
}
module_exit(archos_twl6030_usb_exit);

MODULE_ALIAS("platform:archos_twl6030_usb_xceiv");
MODULE_AUTHOR("Archos S.A.");
MODULE_DESCRIPTION("Archos specific USB transceiver driver");
MODULE_LICENSE("GPL");
