/*
 *    ft5606.c : 31/05/2012
 *    g.revaillot, revaillot@archos.com
 *
 *    Focaltech FT5606 TSP driver
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

//#define DEBUG

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

#include <linux/input/ft5606.h>

#define MAX_FINGER 10

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5606_early_suspend(struct early_suspend *h);
static void ft5606_late_resume(struct early_suspend *h);
#endif


#ifdef CONFIG_TOUCHSCREEN_FT5606_DFU
int ft5x0x_create_sysfs(struct i2c_client *client);
void ft5x0x_release_sysfs(struct i2c_client *client);
#endif

struct ft5606_priv {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct platform_device *pdev;

	struct work_struct  work;
	struct regulator *regulator;

	int irq;

	int x_max;
	int y_max;
	int max_fingers;

	int flags;

	void (*reset)(int);
	int (*get_irq_level)(void);
	int (*demux_i2c)(int);

	u16 old_finger_status;
	int state;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

enum {
	ST_IDLING = 0,		// std mode, not allowed to report events.
	ST_RUNNING = 1,		// std mode, reporting invents.
	ST_SUSPEND = 2,		// suspend mode, not powered.
	ST_MFG = 3,		// dfu / mfg mode
	ST_UNKNOWN = 4,		// unconfigured mode
};

static void ft5606_power(struct i2c_client *client, int on_off)
{
	struct ft5606_priv *priv = i2c_get_clientdata(client);
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

static int ft5606_write(struct i2c_client * client, u8 addr, u8 *value, u8 len)
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

static inline int ft5606_write_u8(struct i2c_client * client, u8 addr, u8 value) {
	return ft5606_write(client, addr, &value, sizeof(u8));
}

static int ft5606_read(struct i2c_client * client, u8 addr, u8 *value, u8 len)
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

static inline int ft5606_read_u8(struct i2c_client * client, u8 addr, u8 *value) {
	return ft5606_read(client, addr, value, sizeof(u8));
}

int ft5606_cycle_and_startup_sequence(struct i2c_client * client, int retry)
{
	u8 dump[32];
	u8 v;

	int ret;

	ft5606_power(client, 1);

	msleep(500);

	while ((ret = ft5606_read(client, 0x00, dump, sizeof(dump))) != sizeof(dump)) {
		if (!--retry)
			break;
		dev_info(&client->dev, "ping ? (%d) (%02x)\n", retry, client->addr);
		msleep(100);
	}

	if (ret < 0) {
		dev_err(&client->dev, "%s: failed\n", __func__);
		return ret;
	}

	print_hex_dump_bytes(KERN_ERR, 0, dump, sizeof(dump));

	ret = ft5606_read_u8(client, 0xa6, &v);

	if (ret < 0) {
		dev_err(&client->dev, "%s: read version failed\n", __func__);
		return ret;
	}

	dev_info(&client->dev, "%s : %d online\n", __func__, v);

	return 0;
}

static irqreturn_t ft5606_thread_fn(int irq, void *v)
{
	struct ft5606_priv * priv = v;
	int finger_status = 0;
	int i;

	struct {
		u16 pad;
		u8 fingers;
		struct pointer_t {
			u8 st_x_h;
			u8 x_l;
			u8 id_y_h;
			u8 y_l;
			u16 pad;
		} __attribute__ ((packed)) p[10];
	} __attribute__ ((packed)) map;

	int ret;

	if (priv->state == ST_MFG)
		return IRQ_HANDLED;

	ret = ft5606_read(priv->client, 0, (unsigned char *)&map, sizeof(map));

	if (ret < 0) {
		dev_err(&priv->client->dev, "%s: failed\n", __func__);
		goto exit_work;
	}

	for (i = 0; i < map.fingers; i++) {
		int st = map.p[i].st_x_h >> 4;
		int id = map.p[i].id_y_h >> 4;

		int x = ((map.p[i].st_x_h & 0x0f) << 8) | map.p[i].x_l;
		int y = ((map.p[i].id_y_h & 0x0f) << 8) | map.p[i].y_l;

		if (!(priv->old_finger_status & (1 << i)))
			dev_dbg(&priv->client->dev, "press %d\n", id);


		finger_status |= (1 << id);

		priv->old_finger_status &= ~(1 << id);

		input_mt_slot(priv->input_dev, id);
		input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, true);

		input_report_abs(priv->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(priv->input_dev, ABS_MT_POSITION_Y, y);

		dev_dbg(&priv->client->dev, "%d : %d / %d : %d\n", id, x, y, st);
	}

	for (i = 0; i < MAX_FINGER; i++) {
		if (!(priv->old_finger_status & (1 << i)))
			continue;

		input_mt_slot(priv->input_dev, i);
		input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, false);

		dev_dbg(&priv->client->dev, "release %d\n", i);
	}

	priv->old_finger_status = finger_status;

	input_sync(priv->input_dev);

exit_work:
	return IRQ_HANDLED;
}

static ssize_t _store_calibration(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct ft5606_priv *priv = i2c_get_clientdata(client);
	unsigned char i;

	u8 stat;

	if (priv->state != ST_RUNNING)
		return -ENODEV;

	priv->state = ST_IDLING;

	dev_info(&priv->client->dev, "%s\n", __func__);

	ft5606_write_u8(client, 0, 0x40);

	msleep(100);

	ft5606_write_u8(client, 2, 0x4);

	msleep(300);

	for( i=0; i<200; i++) {
		dev_info(&priv->client->dev, "%s (%d)\n", __func__, i);

		ft5606_read_u8(client, 0, &stat);

		if (!(stat & 0x70) >> 4)
			break;

		msleep(100);
	}

	msleep(300);

	ft5606_write_u8(client, 0, 0x40);  //goto factory mode
	msleep(100);

	ft5606_write_u8(client, 2, 0x5);  //store CLB result

	msleep(300);

	ft5606_write_u8(client, 0, 0x0); //return to normal mode

	msleep(300);

	priv->state = ST_RUNNING;

	dev_info(&priv->client->dev, "%s : ok\n", __FUNCTION__);

	return count;
}
static DEVICE_ATTR(calibration_trigger, S_IRUGO | S_IWUGO, NULL, _store_calibration);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_calibration_trigger.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = sysfs_attrs,
};

static int ft5606_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct focaltech_ft5606_platform_data *pdata = client->dev.platform_data;
	struct ft5606_priv *priv;
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

	if (ft5606_cycle_and_startup_sequence(priv->client, 5) < 0) {
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
			ft5606_thread_fn,
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

	priv->pdev = platform_device_register_simple("archos_touchscreen", -1, NULL, 0);
	if (IS_ERR(priv->pdev))
		return ret;

	ret = sysfs_create_link(&priv->pdev->dev.kobj, &client->dev.kobj, "tsp");

#ifdef CONFIG_TOUCHSCREEN_FT5606_DFU
	ft5x0x_create_sysfs(client);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	priv->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	priv->early_suspend.suspend = ft5606_early_suspend;
	priv->early_suspend.resume = ft5606_late_resume;

	register_early_suspend(&priv->early_suspend);
#endif

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;

err_irq_request_failed:
err_input_register_failed:

err_detect_failed:
	ft5606_power(client, 0);
	regulator_put(priv->regulator);

err_regulator_get:
	input_free_device(priv->input_dev);

err_input_alloc_failed:

err_no_pdata:
	kfree(priv);

	return ret;
}

static int ft5606_remove(struct i2c_client *client)
{
	struct ft5606_priv *priv = i2c_get_clientdata(client);

	if (priv->state != ST_SUSPEND) {
		disable_irq_wake(priv->irq);
		disable_irq(priv->irq);
	}

	cancel_work_sync(&priv->work);

#ifdef CONFIG_TOUCHSCREEN_FT5606_DFU
	ft5x0x_release_sysfs(client);
#endif

	sysfs_remove_link(&priv->pdev->dev.kobj, "tsp");

	platform_device_unregister(priv->pdev);

	sysfs_remove_group(&client->dev.kobj, &attr_group);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&priv->early_suspend);
#endif
	free_irq(priv->irq, priv);

	input_unregister_device(priv->input_dev);

	ft5606_power(client, 0);

	regulator_put(priv->regulator);

	kfree(priv);
	return 0;
}

static int ft5606_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ft5606_priv *priv = i2c_get_clientdata(client);

	priv->state = ST_SUSPEND;

	disable_irq_wake(priv->irq);
	disable_irq(priv->irq);

	ft5606_power(client, 0);

	return 0;
}

static int ft5606_resume(struct i2c_client *client)
{
	struct ft5606_priv *priv = i2c_get_clientdata(client);

	enable_irq(priv->irq);
	enable_irq_wake(priv->irq);

	if (ft5606_cycle_and_startup_sequence(priv->client, 5) < 0)
		dev_err(&client->dev, "%s: failed ?\n", __FUNCTION__);

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5606_early_suspend(struct early_suspend *h)
{
	struct ft5606_priv *priv =
		container_of(h, struct ft5606_priv, early_suspend);
	ft5606_suspend(priv->client, PMSG_SUSPEND);
}

static void ft5606_late_resume(struct early_suspend *h)
{
	struct ft5606_priv *priv =
		container_of(h, struct ft5606_priv, early_suspend);
	ft5606_resume(priv->client);
}
#endif

static const struct i2c_device_id ft5606_id[] = {
	{ FOCALTECH_FT5606_NAME, 0 },
	{ }
};

static struct i2c_driver ft5606_driver = {
	.probe		= ft5606_probe,
	.remove		= ft5606_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= ft5606_suspend,
	.resume		= ft5606_resume,
#endif
	.id_table	= ft5606_id,
	.driver = {
		.name	= FOCALTECH_FT5606_NAME,
	},
};

static int __init ft5606_init(void)
{
	return i2c_add_driver(&ft5606_driver);
}

static void __exit ft5606_exit(void)
{
	i2c_del_driver(&ft5606_driver);
}

module_init(ft5606_init);
module_exit(ft5606_exit);

MODULE_AUTHOR("Guillaume Revaillot <revaillot@archos.com>");
MODULE_DESCRIPTION("Focaltech FT8606 Touchscreen Driver");
MODULE_LICENSE("GPL");

