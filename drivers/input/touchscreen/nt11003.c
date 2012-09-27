 /* drivers/input/touchscreen/nt11003.c
 *
 *    nt11003.c : 05/05/2012
 *    g.revaillot, revaillot@archos.com
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
#include <linux/cdev.h>

#include <linux/input.h>
#include <linux/input/mt.h>

#include <asm/uaccess.h>

#include <linux/input/nt11003.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
static void nt11003_early_suspend(struct early_suspend *h);
static void nt11003_late_resume(struct early_suspend *h);
#endif

#define MAX(a,b) ((a) < (b) ? (b) : (a))
#define MIN(a,b) ((a) > (b) ? (b) : (a))

#define DEVICE_NAME	"NVTflash"

#define RESET_IS_NRESET	1

#define IOCTL_CALIBRATION 0
#define IOCTL_SOFTWARERESET 1
#define IOCTL_TEST_MODE	3
#define IOCTL_NORMAL_MODE 5

struct nt11003_priv {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	struct regulator *regulator;

	int old_finger_status;
	int green_wake_mode;

	int irq;

	int x_max;
	int y_max;
	int max_fingers;
	int flags;

	void (*reset)(int);
	int (*get_irq_level)(void);
	int (*demux_i2c)(int);

	dev_t devid;
	struct class *class;
	struct cdev cdev;
	int pre_mfg_state;
	int state;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

enum {
	ST_IDLING = 0,		// std mode, not allowed to report events.
	ST_RUNNING = 1,		// std mode, reporting invents.
	ST_SUSPEND = 2,		// suspend mode, not powered.
	ST_MFG = 3,		// manuf mode.
	ST_UNKNOWN = 4,		// unconfigured mode
};

static void nt11003_power(struct i2c_client *client, int on_off)
{
	struct nt11003_priv *priv = i2c_get_clientdata(client);
	static int state = 0;

	if (state == on_off)
		return;

	dev_dbg(&priv->client->dev, "%s %s\n",
			__FUNCTION__, on_off ? "on" : "off");

	state = on_off;


	if (on_off) {
		regulator_enable(priv->regulator);

		if (priv->demux_i2c)
			priv->demux_i2c(0);

		if (priv->reset)
			priv->reset(RESET_IS_NRESET);

		msleep(8);

		if (priv->reset)
			priv->reset(!RESET_IS_NRESET);

		msleep(500);

		if (priv->reset)
			priv->reset(RESET_IS_NRESET);
	} else {
		if (priv->reset)
			priv->reset(!RESET_IS_NRESET);

		if (priv->demux_i2c)
			priv->demux_i2c(1);

		regulator_disable(priv->regulator);
	}
}

static int nt11003_write(struct i2c_client * client, u8 addr, u8 *value, u8 len)
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

static inline int nt11003_write_u8(struct i2c_client * client, u8 addr, u8 value) {
	return nt11003_write(client, addr, &value, sizeof(u8));
}

static inline int nt11003_write_cmd(struct i2c_client * client, u16 cmd) {
	return nt11003_write(client, 0xff, (char *) &cmd, sizeof(u16));
}

static int nt11003_read(struct i2c_client * client, u8 addr, u8 *value, u8 len)
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

static inline int nt11003_read_u8(struct i2c_client * client, u8 addr, u8 *value) {
	return nt11003_read(client, addr, value, sizeof(u8));
}

int nt11003_cycle_and_startup_sequence(struct i2c_client * client, int retry)
{
	struct nt11003_priv *priv = i2c_get_clientdata(client);
	unsigned char cmd[2] = {0x8b, 0x00};
	unsigned char v[2];
	u8 tsp_mode;
	int ret;

	nt11003_power(client, 1);

	msleep(200);

	while ((ret = nt11003_read_u8(client, 0x7e, &tsp_mode)) < 0) {
		if (!--retry)
			break;
		dev_err(&client->dev, "could not read tsp info, retry (%d)\n", retry);
		msleep(100);
	}

	if (ret < 0) {
		dev_err(&client->dev, "%s: failed\n", __func__);
		return ret;
	}

	priv->green_wake_mode = (tsp_mode == 0x05);

	ret = nt11003_write(client, 0xff, cmd, sizeof(cmd));
	ret = nt11003_read(client, 0x3c, v, sizeof(v));

	dev_info(&client->dev, "%s: mode %d / v 0x%02x 0x%02x\n", __func__, tsp_mode, v[0], v[1]);

	return 0;
}

static irqreturn_t nt11003_thread_fn(int irq, void *p)
{
	struct nt11003_priv *priv = p;

	struct finger {
		u8 id_status;
		u8 x_h;
		u8 y_h;
		u8 xy_l;
		u8 area;
		u8 pressure;
	} __attribute__ ((packed));

	struct finger data[20];
	int finger_status = 0;
	int i, ret;

	int read_len = sizeof(struct finger) * priv->max_fingers;

	dev_dbg(&priv->client->dev,"%s\n", __func__);

	if (unlikely(priv->state == ST_MFG))
		goto exit_work;

	if ((ret = nt11003_read(priv->client, 0, (unsigned char*) data, read_len)) != read_len) {
		dev_err(&priv->client->dev, "%s FAIL %d\n", __FUNCTION__, ret);
		goto exit_work;
	}

	for (i = 0; i < priv->max_fingers; i++) {
		struct finger *f = &data[i];

		int st = f->id_status & 0x3;
		int id = f->id_status >> 3;
		int x = (f->x_h << 4) + (f->xy_l >> 4);
		int y = (f->y_h << 4) + (f->xy_l & 0x0f);

		if (st == 3)
			continue;

		dev_dbg(&priv->client->dev, "%s:%d : %d / %d : %04d / %04d : %d / %d\n",
				__func__, i, st, id, x, y, f->pressure, f->area);

		input_mt_slot(priv->input_dev, id);
		input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, true);
		input_report_abs(priv->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(priv->input_dev, ABS_MT_POSITION_Y, y);

		input_report_abs(priv->input_dev, ABS_MT_TOUCH_MAJOR, f->pressure);

		finger_status |= (1<<id);
	}

	for (i = 0; i < priv->max_fingers; i++) {
		if (!(priv->old_finger_status & (1<<i)))
			continue;

		if (finger_status & (1<<i))
			continue;

		dev_dbg(&priv->client->dev, "%s:%d release\n", __func__, i);

		input_mt_slot(priv->input_dev, i);
		input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, false);

		priv->old_finger_status &= ~(1<<i);
	}

	priv->old_finger_status = finger_status;

	input_sync(priv->input_dev);

exit_work:

	return IRQ_HANDLED;
}

int nt11003_flash_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
	struct nt11003_priv *priv = file->private_data;
	unsigned char * buf;
	unsigned char * data;
	int retries = 0;
	u8 adr, len, reg;
	int ret;

	if ((data = kmalloc(64, GFP_KERNEL)) == NULL)
		return -ENOMEM;

	if (copy_from_user(data, buff, count)) {
		kfree(data);
		return -EFAULT;
	}
	
	adr = data[0];
	len = data[1] - 1;
	reg = data[2];
	buf = &data[3];

	priv->client->addr = adr;

	dev_info(&priv->client->dev, "%s: @%02x / r%02x l%d\n\n",
			__func__, adr, reg, len);

	print_hex_dump_bytes(KERN_ERR, 0, data, len + 3);

	while (retries < 20) {
		ret = nt11003_write(priv->client, reg, buf, len);

		if (ret < 0)
			dev_err(&priv->client->dev, "%s %d (%d)\n", __func__, retries, ret);
		else
			break;

		retries++;
	}
	kfree(data);
	return ret;
}

int nt11003_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	struct nt11003_priv *priv = file->private_data;
	unsigned char * buf;
	unsigned char * data;
	int retries = 0;
	u8 adr, len, reg;
	int ret;

	if ((data = kmalloc(64, GFP_KERNEL)) == NULL)
		return -ENOMEM;

	if (copy_from_user(data, buff, count)) {
		kfree(data);
		return -EFAULT;
	}
	adr = data[0];
	len = data[1] - 1;
	reg = data[2];
	buf = &data[3];

	priv->client->addr = adr;

	dev_info(&priv->client->dev, "%s: @%02x / r%02x l%d\n\n",
			__func__, adr, reg, len);

	while (retries < 20) {
		ret = nt11003_read(priv->client, reg, buf, len);

		if (ret < 0)
			dev_err(&priv->client->dev, "%s %d (%d)\n", __func__, retries, ret);
		else
			break;

		retries++;
	}

	print_hex_dump_bytes(KERN_ERR, 0, data, len + 3);

	ret = copy_to_user(buff, data, count);
	
	kfree(data);
	return ret;

}

static long nt11003_flash_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct nt11003_priv *priv = file->private_data;

	dev_info(&priv->client->dev, "%s: %d\n", __func__, cmd);

	switch (cmd) {
		case IOCTL_TEST_MODE:
			disable_irq_nosync(priv->irq);
			break;

		case IOCTL_NORMAL_MODE:
			enable_irq(priv->irq);
			break;

		default:
			break;
	}
	return 0;
}

int nt11003_flash_open(struct inode *inode, struct file *file)
{
	struct nt11003_priv *priv = container_of(inode->i_cdev, struct nt11003_priv, cdev);
	file->private_data = priv;

	priv->pre_mfg_state = priv->state;
	priv->state = ST_MFG;

	return 0;
}

int nt11003_flash_close(struct inode *inode, struct file *file)
{
	struct nt11003_priv *priv = file->private_data;

	priv->state = priv->pre_mfg_state;;
	file->private_data = NULL;

	return 0;
}

struct file_operations nt11003_fops = {
	.owner = THIS_MODULE,
	.open = nt11003_flash_open,
	.release = nt11003_flash_close,
	.unlocked_ioctl = nt11003_flash_ioctl,
	.write = nt11003_flash_write,
	.read = nt11003_flash_read,
};

static int nt11003_register_cdev(struct i2c_client *client)
{
	struct nt11003_priv *priv = i2c_get_clientdata(client);
	struct device *dev;
	int ret;

	cdev_init(&priv->cdev, &nt11003_fops);
	priv->cdev.owner = THIS_MODULE;

	ret = alloc_chrdev_region(&priv->devid, 0, 1, DEVICE_NAME);
	if (ret)
		return ret;

	ret = cdev_add(&priv->cdev, priv->devid, 1);
	if (ret)
		goto err_cdev_add;

	priv->class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(priv->class))
		goto err_class_create;

	if (IS_ERR(dev = device_create(priv->class, NULL, priv->devid, NULL, DEVICE_NAME))) {
		ret = PTR_ERR(dev);
		goto err_device_create;
	}

	return 0;

err_device_create:
	class_destroy(priv->class);

err_class_create:
err_cdev_add:
	unregister_chrdev_region(priv->devid, 1);
	cdev_del(&priv->cdev);

	return ret;
}

static void nt11003_unregister_cdev(struct i2c_client *client)
{
	struct nt11003_priv *priv = i2c_get_clientdata(client);

	device_destroy(priv->class, priv->devid);
	class_destroy(priv->class);
	unregister_chrdev_region(priv->devid, 1);
	cdev_del(&priv->cdev);
}

static int nt11003_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct nt11003_platform_data *pdata = client->dev.platform_data;
	struct nt11003_priv *priv;
	int ret = -1;

	struct {
		u16 x_res;
		u16 y_res;
		u8 max_fingers;
		u8 int_type;
	} tsp_info;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);

	priv->client = client;

	if (pdata) {
		priv->irq = pdata->irq;
		priv->flags = pdata->flags;

		priv->x_max = pdata->x_max;
		priv->y_max = pdata->y_max;
		priv->max_fingers = pdata->max_fingers;

		priv->reset = pdata->reset;
		priv->get_irq_level= pdata->get_irq_level;
		priv->demux_i2c = pdata->demux_i2c;
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

	if (priv->demux_i2c) {
		priv->demux_i2c(1);
		msleep(100);
	}

	if (nt11003_cycle_and_startup_sequence(priv->client, 5) < 0) {
		ret = -ENODEV;
		dev_err(&client->dev, "could not detect tsp.\n");
		goto err_detect_failed;
	}

	dev_dbg(&client->dev, "%s: tsp online @ 0x%02x\n",
			__func__, client->addr);

	priv->input_dev->name = id->name;

	if (nt11003_read(client, 0x78, (char *) &tsp_info, sizeof(tsp_info)) < 0) {
		ret = -ENODEV;
		dev_err(&client->dev, "could not read tsp info.\n");
		goto err_detect_failed;
	}

	print_hex_dump_bytes(KERN_ERR, 0, (char *) &tsp_info, sizeof(tsp_info));

	if (!priv->x_max && !priv->y_max) {
		priv->x_max = be16_to_cpu(tsp_info.x_res);
		priv->y_max = be16_to_cpu(tsp_info.y_res);
		priv->max_fingers = tsp_info.max_fingers;

		if (priv->x_max == 0xffff)
			priv->x_max = 1280;

		if (priv->y_max == 0xffff)
			priv->y_max = 800;

		if (priv->max_fingers == 0xff)
			priv->max_fingers = 10;
	}

	dev_info(&client->dev, "%s: %d / %d / %d\n",
			__func__, priv->x_max, priv->y_max, priv->max_fingers);

	set_bit(EV_SYN, priv->input_dev->evbit);
	set_bit(EV_ABS, priv->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, priv->input_dev->evbit);

	set_bit(ABS_MT_POSITION_X, priv->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, priv->input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, priv->input_dev->absbit);

	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_X, 0, priv->x_max, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_Y, 0, priv->y_max, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_TOUCH_MAJOR, 70, 240, 0, 0);

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
			nt11003_thread_fn,
			IRQF_TRIGGER_FALLING,
			client->name, priv);

	if (ret) {
		ret = -ENODEV;
		dev_err(&client->dev, "request_irq failed\n");
		goto err_irq_request_failed;
	}

	enable_irq_wake(priv->irq);
	device_init_wakeup(&client->dev, true);

	nt11003_register_cdev(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	priv->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	priv->early_suspend.suspend = nt11003_early_suspend;
	priv->early_suspend.resume = nt11003_late_resume;

	register_early_suspend(&priv->early_suspend);
#endif

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;

err_irq_request_failed:
err_input_register_failed:

err_detect_failed:
	nt11003_power(client, 0);
	regulator_put(priv->regulator);

err_regulator_get:
	input_free_device(priv->input_dev);

err_input_alloc_failed:

err_no_pdata:
	kfree(priv);

	return ret;
}

static int nt11003_remove(struct i2c_client *client)
{
	struct nt11003_priv *priv = i2c_get_clientdata(client);

	if (priv->state != ST_SUSPEND) {
		disable_irq_wake(priv->irq);
		disable_irq(priv->irq);
	}

	nt11003_unregister_cdev(client);

	cancel_work_sync(&priv->work);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&priv->early_suspend);
#endif
	free_irq(priv->irq, priv);

	input_unregister_device(priv->input_dev);

	nt11003_power(client, 0);

	regulator_put(priv->regulator);

	kfree(priv);
	return 0;
}
static int nt11003_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct nt11003_priv *priv = i2c_get_clientdata(client);

	priv->state = ST_SUSPEND;

	disable_irq_wake(priv->irq);
	disable_irq(priv->irq);

	nt11003_power(client, 0);

	return 0;
}

static int nt11003_resume(struct i2c_client *client)
{
	struct nt11003_priv *priv = i2c_get_clientdata(client);

	enable_irq(priv->irq);
	enable_irq_wake(priv->irq);

	if (nt11003_cycle_and_startup_sequence(priv->client, 5) < 0)
		dev_err(&client->dev, "%s: failed ?\n", __FUNCTION__);

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void nt11003_early_suspend(struct early_suspend *h)
{
	struct nt11003_priv *priv =
		container_of(h, struct nt11003_priv, early_suspend);
	nt11003_suspend(priv->client, PMSG_SUSPEND);
}

static void nt11003_late_resume(struct early_suspend *h)
{
	struct nt11003_priv *priv =
		container_of(h, struct nt11003_priv, early_suspend);
	nt11003_resume(priv->client);
}
#endif

static const struct i2c_device_id nt11003_id[] = {
	{ NT11003_NAME, 0 },
	{ }
};

static struct i2c_driver nt11003_driver = {
	.probe		= nt11003_probe,
	.remove		= nt11003_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= nt11003_suspend,
	.resume		= nt11003_resume,
#endif
	.id_table	= nt11003_id,
	.driver = {
		.name	= NT11003_NAME,
	},
};

static int __init nt11003_init(void)
{
	return i2c_add_driver(&nt11003_driver);
}

static void __exit nt11003_exit(void)
{
	i2c_del_driver(&nt11003_driver);
}

module_init(nt11003_init);
module_exit(nt11003_exit);

MODULE_AUTHOR("Guillaume Revaillot <revaillot@archos.com>");
MODULE_DESCRIPTION("Novatek NT11003 Touchscreen Driver");
MODULE_LICENSE("GPL");

