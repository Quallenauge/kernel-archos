/*
 *    goodix-gt8105.c : 16/05/2012
 *    g.revaillot, revaillot@archos.com
 *
 *    Goodix GT8105 Solution Driver (GT8105*2 + GTM802)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

// #define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <linux/interrupt.h>
#include <linux/i2c.h>

#include <linux/input.h>
#include <linux/input/mt.h>

#include <linux/input/goodix-gt8105.h>

#include "gt8105.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gt8105_early_suspend(struct early_suspend *h);
static void gt8105_late_resume(struct early_suspend *h);
#endif

static void gt8105_power(struct i2c_client *client, int on_off)
{
	struct gt8105_priv *priv = i2c_get_clientdata(client);
	static int state = -1;

	if (state == on_off)
		return;

	dev_dbg(&priv->client->dev, "%s %s\n",
			__FUNCTION__, on_off ? "on" : "off");

	state = on_off;

	if (priv->reset)
		priv->reset(0);

	if (on_off) {
		regulator_enable(priv->regulator);

		if (priv->reset)
			priv->reset(1);

		if (priv->demux_i2c)
			priv->demux_i2c(0);
	} else {
		if (priv->demux_i2c)
			priv->demux_i2c(1);

		regulator_disable(priv->regulator);
	}
}

int gt8105_write(struct i2c_client * client, u8 addr, u8 *value, u8 len)
{
	struct i2c_msg msg;
	int ret;

	char *buff = kzalloc(sizeof(addr) + len, GFP_KERNEL);

	if (!buff)
		return -ENOMEM;

	*buff = addr;

	memcpy(buff + sizeof(addr), value, len);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = buff;
	msg.len = sizeof(addr) + len;

	ret = i2c_transfer(client->adapter, &msg, 1);

	kfree(buff);

	if (ret <= 0)
		return -EIO;

	return len;
}

int gt8105_write_u8(struct i2c_client * client, u8 addr, u8 value) {
	return gt8105_write(client, addr, &value, sizeof(u8));
}

int gt8105_read(struct i2c_client * client, u8 addr, u8 *value, u8 len)
{
	struct i2c_msg msg[2];
	int ret;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &addr;
	msg[0].len = sizeof(addr);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = value;
	msg[1].len = len;

	ret = i2c_transfer(client->adapter, msg, 2);

	if (ret == 2)
		return len;
	else if (ret >= 0){
		return -EIO;
	} else {
		return ret;
	}

}

int gt8105_read_u8(struct i2c_client * client, u8 addr, u8 *value) {
	return gt8105_read(client, addr, value, sizeof(u8));
}

int gt8105_read_version(struct i2c_client *client, char *version, int vlen)
{
	char buf[17] = {0,};
	int len, ret = -1;

	if (vlen < sizeof(vlen))
		return 1;

	memset(version, 0, vlen);

	ret = gt8105_read(client, 240, buf, sizeof(buf) - 1);

	if (ret <= 0)
		return ret;

	len = strlen(buf);

	if (len <= 2)
		return 1;

	strncpy(version, buf, len);

	return 0;
}

int gt8105_cycle_and_startup_sequence(struct i2c_client * client, int retry)
{
	struct gt8105_priv *priv = i2c_get_clientdata(client);
	char v[32];

	uint8_t config_info[] = {
		0x00,0x04,0x00,0x03,0x00,0x05,0x61,0x01,0x00,0x0F,0x20,0x03,0x10,0x10,0x00,0x00,
		0x2D,0x00,0x00,0xA8,0x10,0x10,0x11,0x37,0x00,0x00,0x0B,0x0A,0x09,0x08,0x07,0x06,
		0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0x0B,0x0A,0x09,0x08,0x07,0x06,
		0x05,0x04,0x03,0x02,0x01,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x32,0x5F,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20
	};

	int ret;

	gt8105_power(client, 1);

	msleep(100);

	while ((ret = gt8105_write(client, 0x65, config_info, sizeof(config_info)) != sizeof(config_info))) {
		if (!--retry)
			break;
		dev_info(&client->dev, "ping ? (%d) (%02x)\n", retry, client->addr);
		msleep(100);
	}

	if (ret < 0) {
		dev_err(&client->dev, "%s: failed\n", __func__);
		return ret;
	}

	if (!gt8105_read_version(client, v, sizeof(v)))
		dev_info(&client->dev, "Up, FW %s\n", v);

	priv->state = ST_IDLING;

	return 0;
}

static irqreturn_t gt8105_thread_fn(int irq, void *v)
{
	struct gt8105_priv * priv = v;

	struct pointer_t {
		u8 x_h;
		u8 x_l;
		u8 y_h;
		u8 y_l;
		u8 w;
	} __attribute__ ((packed));

	unsigned char data[3 * sizeof(u8) + 10 * sizeof(struct pointer_t) + 1 * sizeof(u8)];
	u16 finger_status = 0;
	int ret;
	int i;

	struct pointer_t * p = (struct pointer_t *) (data + 3);

	if (priv->state != ST_RUNNING)
		return IRQ_HANDLED;

	ret = gt8105_read(priv->client, 0, (unsigned char *)data, sizeof(data));

	if (ret < 0) {
		dev_err(&priv->client->dev, "%s: failed\n", __func__);
		goto exit_work;
	}

	finger_status = data[1] | (data[2] << 8);

	for (i = 0; i < priv->max_fingers; i++) {
		if (finger_status & (1 << i)) {
			if (!(priv->old_finger_status & (1 << i)))
				dev_dbg(&priv->client->dev, "press %d\n", i);

			priv->old_finger_status &= ~(1 << i);

			input_mt_slot(priv->input_dev, i);
			input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, true);

			input_report_abs(priv->input_dev, ABS_MT_POSITION_X, p->x_l | (p->x_h << 8));
			input_report_abs(priv->input_dev, ABS_MT_POSITION_Y, p->y_l | (p->y_h << 8));
			input_report_abs(priv->input_dev, ABS_MT_TOUCH_MAJOR, p->w);

			dev_dbg(&priv->client->dev, "%d : %d / %d : %d\n",
					i,
					p->x_l | (p->x_h << 8),
					p->y_l | (p->y_h << 8),
					p->w);
		}

		if (priv->old_finger_status & (1 << i)) {
			input_mt_slot(priv->input_dev, i);
			input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, false);
			dev_dbg(&priv->client->dev, "release %d\n", i);
		}

		p++;
	}

	priv->old_finger_status = finger_status;

	input_sync(priv->input_dev);

exit_work:
	return IRQ_HANDLED;
}

static struct attribute *sysfs_attrs[] = {
	NULL
};

static struct attribute_group attr_group = {
	.attrs = sysfs_attrs,
};

static int gt8105_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct goodix_gt8105_platform_data *pdata = client->dev.platform_data;
	struct gt8105_priv *priv;
	int ret = -1;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);

	priv->client = client;

	priv->x_max = 1024;
	priv->y_max = 768;
	priv->max_fingers = 10;

	if (pdata) {
		priv->irq = pdata->irq;
		priv->flags = pdata->flags;

		//priv->reset = pdata->reset;
		//priv->get_irq_level= pdata->get_irq_level;
		//priv->demux_i2c = pdata->demux_i2c;
	} else {
		ret = -ENODEV;
		goto err_no_pdata;
	}

	priv->input_dev = input_allocate_device();
	if (priv->input_dev == NULL) {
		ret = -ENOMEM;
		goto err_input_alloc_failed;
	}

	priv->regulator = regulator_get(&client->dev, pdata->regulator);
	if (IS_ERR(priv->regulator)) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed to get regulator\n");
		goto err_regulator_get;
	}

	if (gt8105_cycle_and_startup_sequence(priv->client, 5) < 0) {
		ret = -ENODEV;
		dev_err(&client->dev, "could not detect tsp.\n");
		goto err_detect_failed;
	}

	dev_dbg(&client->dev, "%s: tsp online @ 0x%02x\n",
			__func__, client->addr);

	priv->input_dev->name = id->name;

	set_bit(EV_SYN, priv->input_dev->evbit);
	set_bit(EV_ABS, priv->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, priv->input_dev->evbit);

	set_bit(ABS_MT_POSITION_X, priv->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, priv->input_dev->absbit);

	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_X, 0, priv->x_max, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_Y, 0, priv->y_max, 0, 0);

	input_mt_init_slots(priv->input_dev, priv->max_fingers);

	ret = input_register_device(priv->input_dev);
	if (ret) {
		ret = -ENODEV;
		dev_err(&client->dev, "%s: Unable to register %s input device\n",
				__FUNCTION__, priv->input_dev->name);
		goto err_input_register_failed;
	}

	ret = request_threaded_irq(priv->irq,
			NULL,
			gt8105_thread_fn,
			IRQF_TRIGGER_FALLING,
			client->name, priv);

	if (ret) {
		ret = -ENODEV;
		dev_err(&client->dev, "request_irq failed\n");
		goto err_irq_request_failed;
	}

	enable_irq_wake(priv->irq);
	device_init_wakeup(&client->dev, true);

	ret = sysfs_create_group(&client->dev.kobj, &attr_group);

	gt8105_register_dfu(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	priv->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	priv->early_suspend.suspend = gt8105_early_suspend;
	priv->early_suspend.resume = gt8105_late_resume;

	register_early_suspend(&priv->early_suspend);
#endif

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;

err_irq_request_failed:
err_input_register_failed:

err_detect_failed:
	gt8105_power(client, 0);
	regulator_put(priv->regulator);

err_regulator_get:
	input_free_device(priv->input_dev);

err_input_alloc_failed:

err_no_pdata:
	kfree(priv);

	return ret;
}

static int gt8105_remove(struct i2c_client *client)
{
	struct gt8105_priv *priv = i2c_get_clientdata(client);

	if (priv->state != ST_SUSPEND) {
		disable_irq_wake(priv->irq);
		disable_irq(priv->irq);
	}

	cancel_work_sync(&priv->work);

	gt8105_release_dfu(client);

	sysfs_remove_group(&client->dev.kobj, &attr_group);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&priv->early_suspend);
#endif
	free_irq(priv->irq, priv);

	input_unregister_device(priv->input_dev);

	gt8105_power(client, 0);

	regulator_put(priv->regulator);

	kfree(priv);
	return 0;
}

static int gt8105_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct gt8105_priv *priv = i2c_get_clientdata(client);

	priv->state = ST_SUSPEND;

	disable_irq_wake(priv->irq);
	disable_irq(priv->irq);

	gt8105_power(client, 0);

	return 0;
}

static int gt8105_resume(struct i2c_client *client)
{
	struct gt8105_priv *priv = i2c_get_clientdata(client);

	enable_irq(priv->irq);
	enable_irq_wake(priv->irq);

	if (gt8105_cycle_and_startup_sequence(priv->client, 5) < 0)
		dev_err(&client->dev, "%s: failed ?\n", __FUNCTION__);

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gt8105_early_suspend(struct early_suspend *h)
{
	struct gt8105_priv *priv =
		container_of(h, struct gt8105_priv, early_suspend);
	gt8105_suspend(priv->client, PMSG_SUSPEND);
}

static void gt8105_late_resume(struct early_suspend *h)
{
	struct gt8105_priv *priv =
		container_of(h, struct gt8105_priv, early_suspend);
	gt8105_resume(priv->client);
}
#endif

static const struct i2c_device_id gt8105_id[] = {
	{ GOODIX_GT8105_NAME, 0 },
	{ }
};

static struct i2c_driver gt8105_driver = {
	.probe		= gt8105_probe,
	.remove		= gt8105_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= gt8105_suspend,
	.resume		= gt8105_resume,
#endif
	.id_table	= gt8105_id,
	.driver = {
		.name	= GOODIX_GT8105_NAME,
	},
};

static int __init gt8105_init(void)
{
	return i2c_add_driver(&gt8105_driver);
}

static void __exit gt8105_exit(void)
{
	i2c_del_driver(&gt8105_driver);
}

module_init(gt8105_init);
module_exit(gt8105_exit);

MODULE_AUTHOR("Guillaume Revaillot <revaillot@archos.com>");
MODULE_DESCRIPTION("Goodix GT8105 Touchscreen Driver");
MODULE_LICENSE("GPL");
