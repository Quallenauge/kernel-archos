/*
 * TI LP855x Backlight Driver
 *
 *                     Copyright (C) 2011 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/lp855x.h>

/* Registers */
#define BRIGHTNESS_CTRL			(0x00)
#define DEVICE_CTRL		(0x01)

#define BUF_SIZE		20
#define DEFAULT_BL_NAME			"lcd-backlight"
#define MAX_BRIGHTNESS		255

struct lp855x {
	const char *chipid;
	struct i2c_client *client;
	struct backlight_device *bl;
	struct led_classdev led;
	struct device *dev;
	struct mutex xfer_lock;
	struct lp855x_platform_data *pdata;
};

static int lp855x_read_byte(struct lp855x *lp, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&lp->xfer_lock);
	ret = i2c_smbus_read_byte_data(lp->client, reg);
	if (ret < 0) {
		dev_err(lp->dev, "failed to read 0x%.2x\n", reg);
		return ret;
	}
	mutex_unlock(&lp->xfer_lock);

	*data = (u8)ret;
	return 0;
}

static int lp855x_write_byte(struct lp855x *lp, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&lp->xfer_lock);
	ret = i2c_smbus_write_byte_data(lp->client, reg, data);
	mutex_unlock(&lp->xfer_lock);

	return ret;
}

static int lp855x_set_brightness(struct lp855x *lp, u8 val)
{
	int ret;

	if (lp->pdata->max_brightness)
		val = ( (val * lp->pdata->max_brightness ) / 255);

	ret = lp855x_write_byte(lp, BRIGHTNESS_CTRL, val);

	return ret;
}

static int lp855x_is_valid_rom_area(struct lp855x *lp, u8 addr)
{
	const char *id = lp->chipid;
	u8 start, end;

	if (strstr(id, "lp8550") || strstr(id, "lp8551")
	    || strstr(id, "lp8552") || strstr(id, "lp8553")) {
		start = EEPROM_START;
		end = EEPROM_END;
	} else if (strstr(id, "lp8556")) {
		start = EPROM_START;
		end = EPROM_END;
	}

	return (addr >= start && addr <= end) ? 1 : 0;
}

static int lp855x_init_registers(struct lp855x *lp)
{
	u8 val, addr;
	int i, ret;
	struct lp855x_platform_data *pd = lp->pdata;

	val = pd->initial_brightness;
	ret = lp855x_set_brightness(lp, val);
	if (ret)
		return ret;

	val = pd->device_control;
	ret = lp855x_write_byte(lp, DEVICE_CTRL, val);
	if (ret)
		return ret;

	if (pd->load_new_rom_data && pd->size_program) {
		for (i = 0; i < pd->size_program; i++) {
			addr = pd->rom_data[i].addr;
			val = pd->rom_data[i].val;
			if (!lp855x_is_valid_rom_area(lp, addr))
				continue;

			ret = lp855x_write_byte(lp, addr, val);
			if (ret)
				return ret;
		}
	}

	return ret;
}

static int lp855x_bl_update_status(struct backlight_device *bl)
{
	struct lp855x *lp = bl_get_data(bl);
	enum lp855x_brightness_ctrl_mode mode = lp->pdata->mode;

	if (bl->props.state & BL_CORE_SUSPENDED)
		bl->props.brightness = 0;

	if (mode == PWM_BASED) {
		struct lp855x_pwm_data *pd = &lp->pdata->pwm_data;
		int br = bl->props.brightness;
		int max_br = bl->props.max_brightness;

		if (pd->pwm_set_intensity)
			pd->pwm_set_intensity(br, max_br);

	} else if (mode == REGISTER_BASED) {
		u8 val = bl->props.brightness;
		lp855x_set_brightness(lp, val);
	}

	lp->led.brightness = bl->props.brightness;

	return (bl->props.brightness);
}

static int lp855x_bl_get_brightness(struct backlight_device *bl)
{
	struct lp855x *lp = bl_get_data(bl);
	enum lp855x_brightness_ctrl_mode mode = lp->pdata->mode;

	if (mode == PWM_BASED) {
		struct lp855x_pwm_data *pd = &lp->pdata->pwm_data;
		int max_br = bl->props.max_brightness;

		if (pd->pwm_get_intensity)
			bl->props.brightness = pd->pwm_get_intensity(max_br);

	} else if (mode == REGISTER_BASED) {
		u8 val = 0;

		lp855x_read_byte(lp, BRIGHTNESS_CTRL, &val);
		bl->props.brightness = val;
	}

	return (bl->props.brightness);
}

static const struct backlight_ops lp855x_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lp855x_bl_update_status,
	.get_brightness = lp855x_bl_get_brightness,
};

static int lp855x_backlight_register(struct lp855x *lp)
{
	struct backlight_device *bl;
	struct backlight_properties props;
	struct lp855x_platform_data *pdata = lp->pdata;
	char *name = pdata->name ? : DEFAULT_BL_NAME;

	props.type = BACKLIGHT_PLATFORM;

	if (pdata->max_brightness)
		props.max_brightness = pdata->max_brightness;
	else
		props.max_brightness = MAX_BRIGHTNESS;

	if (pdata->initial_brightness > props.max_brightness)
		pdata->initial_brightness = props.max_brightness;

	props.brightness = pdata->initial_brightness;

	bl = backlight_device_register(name, lp->dev, lp,
					&lp855x_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	lp->bl = bl;

	return 0;
}

static void lp855x_backlight_unregister(struct lp855x *lp)
{
	if (lp->bl)
		backlight_device_unregister(lp->bl);
}

static void led_set(struct led_classdev *led_cdev,
			     enum led_brightness value)
{
	struct lp855x *lp = container_of(led_cdev, struct lp855x, led);

	lp->bl->props.brightness = value;
	lp855x_bl_update_status(lp->bl);
}

static int lp855x_led_register(struct lp855x *lp)
{
	struct lp855x_platform_data *pdata = lp->pdata;
	int ret;

	lp->led.default_trigger = pdata->default_trigger;
	lp->led.name = pdata->name ? : DEFAULT_BL_NAME;
	lp->led.brightness =  pdata->initial_brightness;
	lp->led.brightness_set = led_set;

	ret = led_classdev_register(lp->dev, &lp->led);
	if (ret < 0)
		return ret;

	return 0;
}

static void lp855x_led_unregister(struct lp855x *lp)
{
	led_classdev_unregister(&lp->led);
}

static ssize_t lp855x_get_chip_id(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lp855x *lp = dev_get_drvdata(dev);
	return scnprintf(buf, BUF_SIZE, "%s\n", lp->chipid);
}

static ssize_t lp855x_get_bl_ctl_mode(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct lp855x *lp = dev_get_drvdata(dev);
	enum lp855x_brightness_ctrl_mode mode = lp->pdata->mode;
	char *strmode = NULL;

	if (mode == PWM_BASED)
		strmode = "pwm based";
	else if (mode == REGISTER_BASED)
		strmode = "register based";

	return scnprintf(buf, BUF_SIZE, "%s\n", strmode);
}

static DEVICE_ATTR(chip_id, S_IRUGO, lp855x_get_chip_id, NULL);
static DEVICE_ATTR(bl_ctl_mode, S_IRUGO, lp855x_get_bl_ctl_mode, NULL);

static struct attribute *lp855x_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_bl_ctl_mode.attr,
	NULL,
};

static const struct attribute_group lp855x_attr_group = {
	.attrs = lp855x_attributes,
};

static int lp855x_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct lp855x *lp;
	struct lp855x_platform_data *pdata = cl->dev.platform_data;
	int ret;

	if (!i2c_check_functionality(cl->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		goto err_io;

	lp = kzalloc(sizeof(struct lp855x), GFP_KERNEL);
	if (!lp)
		goto err_mem;

	lp->client = cl;
	lp->dev = &cl->dev;
	lp->pdata = pdata;
	lp->chipid = id->name;
	i2c_set_clientdata(cl, lp);

	mutex_init(&lp->xfer_lock);

	ret = lp855x_init_registers(lp);
	if (ret)
		goto err_i2c;

	ret = lp855x_backlight_register(lp);
	if (ret)
		goto err_bldev;

	ret = lp855x_led_register(lp);
	if (ret)
		goto err_leddev;

	ret = sysfs_create_group(&lp->dev->kobj, &lp855x_attr_group);
	if (ret)
		goto err_sysfs;

	backlight_update_status(lp->bl);
	return ret;

err_io:
	return -EIO;
err_mem:
	return -ENOMEM;
err_i2c:
	dev_err(lp->dev, "i2c communication err: %d", ret);
	kfree(lp);
	return ret;
err_bldev:
	dev_err(lp->dev, "can not register backlight device. err: %d\n", ret);
	kfree(lp);
	return ret;
err_leddev:
	lp855x_backlight_unregister(lp);
	dev_err(lp->dev, "can not register led device. err: %d\n", ret);
	kfree(lp);
	return ret;
err_sysfs:
	dev_err(lp->dev, "can not register sysfs. err: %d\n", ret);
	lp855x_backlight_unregister(lp);
	lp855x_led_unregister(lp);
	kfree(lp);
	return ret;
}

static int __devexit lp855x_remove(struct i2c_client *cl)
{
	struct lp855x *lp = i2c_get_clientdata(cl);

	lp->bl->props.brightness = 0;
	backlight_update_status(lp->bl);
	sysfs_remove_group(&lp->dev->kobj, &lp855x_attr_group);
	lp855x_led_unregister(lp);
	lp855x_backlight_unregister(lp);
	kfree(lp);

	return 0;
}

static const struct i2c_device_id lp855x_ids[] = {
	{"lp8550", LP8550},
	{"lp8551", LP8551},
	{"lp8552", LP8552},
	{"lp8553", LP8553},
	{"lp8556", LP8556},
	{},
};
MODULE_DEVICE_TABLE(i2c, lp855x_ids);

static struct i2c_driver lp855x_driver = {
	.driver = {
		   .name = "lp855x",
		   },
	.probe = lp855x_probe,
	.remove = __devexit_p(lp855x_remove),
	.id_table = lp855x_ids,
};

static int __init lp855x_init(void)
{
	return i2c_add_driver(&lp855x_driver);
}

static void __exit lp855x_exit(void)
{
	i2c_del_driver(&lp855x_driver);
}

module_init(lp855x_init);
module_exit(lp855x_exit);

MODULE_DESCRIPTION("Texas Instruments LP855x Backlight driver");
MODULE_AUTHOR("Milo Kim <milo.kim@ti.com>");
MODULE_LICENSE("GPL");
