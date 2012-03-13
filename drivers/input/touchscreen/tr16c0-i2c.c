/*
 *    tr16c0-i2c.c : 04/10/2011
 *    g.revaillot, revaillot@archos.com
 *
 *    Touchplus TR16C0 + Tango F45 ctsp driver
 */

//#define DEBUG

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <linux/input/tr16c0-i2c.h>

#define DELAY_MS	1
#define POLL_MS		4
#define POLL_MS_V2	4
#define MAX_FINGERS	10

#define CALIBRATION_SHORT_DELAY_MS	200
#define CALIBRATION_LONG_DELAY_MS	60000

#define MIN(a,b) 	(((a) < (b)) ? (a) : (b))

static const struct reg_maps {
	u8 touch_base;
	u8 version;
	u8 specop;
	u8 int_mode;
	u8 x_raw;
	u8 y_raw;
	u8 internal_enable;
	u8 cmd_mode;
} map[] = {
	{
		.touch_base = 0,
		.version = 48,
		.int_mode = 46,
		.specop = 55,
		.x_raw = 61,
		.y_raw = 125,
		.internal_enable = 194,
		.cmd_mode = 0, // XXX ?
	},
	{
		.touch_base = 0,
		.version = 65,
		.int_mode = 62,
		.specop = 55,
		.x_raw = 61,
		.y_raw = 125,
		.internal_enable = 0, // XXX
		.cmd_mode = 56,
	}
};

struct tr16c0_priv {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct completion irq_trigged;
	struct regulator *regulator;
	struct workqueue_struct *wq;
	struct delayed_work work;

	int irq;

	int x_max;
	int y_max;
	int flags;

	unsigned long poll_ms;
	unsigned long raw_mode;
	void (*reset)(int);
	int (*get_irq_level)(void);

	int finger_cnt;
	int point_state;

	int state;
	int map_version;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

enum {
	ST_IDLING = 0,		// std mode, not allowed to report events.
	ST_RUNNING = 1,		// std mode, reporting invents.
	ST_SUSPEND = 2,		// suspend mode, not powered.
	ST_BOOTLOADER = 3,	// bootloader mode.
	ST_UNKNOWN = 4,		// unconfigured mode
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tr16c0_early_suspend(struct early_suspend *h);
static void tr16c0_late_resume(struct early_suspend *h);
#endif

static void tr16c0_power(struct i2c_client *client, int on_off)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	static int state = -1;

	if (state == on_off)
		return;

	state = on_off;

	if (priv->reset)
		priv->reset(1);

	if (on_off) {
		regulator_enable(priv->regulator);

		if (priv->reset) {
			msleep(10);
			priv->reset(0);
		}
	} else {
		regulator_disable(priv->regulator);
	}
}


static int tr16c0_write(struct i2c_client * client, u8 addr, u8 *value, u8 len)
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

static inline int tr16c0_write_u8(struct i2c_client * client, u8 addr, u8 value) {
	return tr16c0_write(client, addr, &value, sizeof(u8));
}

static int tr16c0_read(struct i2c_client * client, u8 addr, u8 *value, u8 len)
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

static irqreturn_t tr16c0_irq_handler(int irq, void * p)
{
	struct tr16c0_priv *priv = p;

	dev_dbg(&priv->client->dev, "%s\n", __func__);

	switch (priv->state) {
		case ST_RUNNING:
			schedule_delayed_work(&priv->work,
					msecs_to_jiffies(DELAY_MS));
			break;

		default:
			complete(&priv->irq_trigged);
			break;

	}

	return IRQ_HANDLED;
}

static void tr16c0_irq_worker(struct work_struct *work)
{
	struct finger {
		u16 x;
		u16 y;
		u8 id;
	} __attribute__ ((packed));

	struct tr16c0_priv *priv =
		container_of(to_delayed_work(work), struct tr16c0_priv, work);

	u8 data[MAX_FINGERS * sizeof(struct finger) + 2];
	int new_point_state = 0;
	int read_len, i;

	read_len = 2 + priv->finger_cnt * sizeof(struct finger);

	if (tr16c0_read(priv->client, map[priv->map_version].touch_base, data, read_len) != read_len) {
		dev_err(&priv->client->dev,
				"%s: could not read output data.\n",
				__FUNCTION__);
		goto exit_work;
	}

	for (i = 0; i < MIN(priv->finger_cnt, data[0]); i++) {
		struct finger * f = (struct finger *) (data + i * sizeof(*f) + 2);
		dev_dbg(&priv->client->dev, "f%i of %d, id%d @ %d/%d\n", i, data[0], f->id, f->x, f->y);

		if (priv->flags & TR16C0_FLAGS_INV_X)
			f->x = priv->x_max - f->x;

		if (priv->flags & TR16C0_FLAGS_INV_Y)
			f->y = priv->y_max - f->y;

		input_mt_slot(priv->input_dev, f->id - 1);
		input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, true);

		input_report_abs(priv->input_dev, ABS_MT_POSITION_X, f->x);
		input_report_abs(priv->input_dev, ABS_MT_POSITION_Y, f->y);

		new_point_state |= (1 << f->id);
	}

	for (i=0; i < MAX_FINGERS; i++) {
		if ((priv->point_state & (1 << i)) && !(new_point_state & (1 << i))) {
			input_mt_slot(priv->input_dev, i-1);
			input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, false);
		}
	}

	priv->point_state = new_point_state;

	input_sync(priv->input_dev);

	if (!priv->get_irq_level())
		schedule_delayed_work(&priv->work,
				msecs_to_jiffies(priv->poll_ms));

	priv->finger_cnt = data[0];

exit_work:
	return;
}

static int tr16c0_startup_sequence(struct i2c_client *client)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	int retry = 5;
	int ret;

	struct version {
		u8 major;
		u8 minor;
		u16 svn;
	} __attribute__ ((packed)) v;

	tr16c0_power(client, 1);

	msleep(100);

	while ((ret = tr16c0_read(client, map[priv->map_version].version, (unsigned char *)&v, sizeof(v))) != sizeof(v)) {
		if (!--retry)
			break;
		dev_info(&client->dev, "ping ? (%d)\n", retry);
		msleep(50);
	}

	if (ret < 0)
		goto fail;

	dev_info(&client->dev, "v%d.%d.%d (r%d)\n",
			v.major & 0x0f, v.minor & 0xf, v.minor >> 4, v.svn);

	if (tr16c0_write_u8(client, map[priv->map_version].int_mode, 0x0a) < 1)
		goto fail;

	priv->state = ST_IDLING;

	return 0;
fail:
    dev_info(&client->dev, "could not talk to tsp.)\n");
	tr16c0_power(client, 0);
	return -ENODEV;
}

static ssize_t _store_calibration(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	int timeout = CALIBRATION_LONG_DELAY_MS / 1000;

	int ret;

	if (priv->state != ST_RUNNING)
		return -ENODEV;

	priv->state = ST_IDLING;

	INIT_COMPLETION(priv->irq_trigged);

	if (priv->map_version > 0)
		tr16c0_write_u8(client, map[priv->map_version].cmd_mode, 0x00);

	ret = tr16c0_write_u8(client, map[priv->map_version].specop, 0x03);

	if (ret < 0) {
		count = -ENODEV;
		goto fail;
	}

	if (wait_for_completion_interruptible_timeout(
				&priv->irq_trigged,
				msecs_to_jiffies(CALIBRATION_SHORT_DELAY_MS)) <= 0) {

		dev_err(&client->dev,
				"%s : missed irq falling edge, continue\n",
				__FUNCTION__);
	}

	while (priv->get_irq_level() == 0 && (--timeout > 0)) {
		dev_info(&client->dev, "%s : running (%d)...",
				__func__, timeout);
		msleep(1000);
	}

	if (timeout <= 0) {
		dev_err(&client->dev,
				"%s : calibration stuck\n",
				__FUNCTION__);
		count = -ENODEV;
	}



fail:
	if (priv->map_version > 0)
		tr16c0_write_u8(client, map[priv->map_version].cmd_mode, 0x00);

	priv->state = ST_RUNNING;

	return count;
}
static DEVICE_ATTR(calibration_trigger, S_IWUSR | S_IRUGO, NULL, _store_calibration);

#ifdef DEBUG

static ssize_t _show_poll_ms(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	return sprintf(buf, "%lu\n", priv->poll_ms);
}

static ssize_t _store_poll_ms(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	long v;

	if(strict_strtoul(buf, 10, &v))
		return -EINVAL;

	priv->poll_ms = v;

	return count;
}
static DEVICE_ATTR(poll_ms, S_IWUSR | S_IRUGO, _show_poll_ms, _store_poll_ms);

static ssize_t _show_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	int read_len = 64;
	char data[read_len];

	if (tr16c0_read(priv->client, map[priv->map_version].touch_base, data, read_len) != read_len) {
		return -ENODEV;
	}
	print_hex_dump_bytes(KERN_ERR, 0, data, sizeof(data));
	return 0;
}
static DEVICE_ATTR(data, S_IWUSR | S_IRUGO, _show_data, NULL);

struct raw {
	unsigned char data[48];
};

static int tr16c0_get_raw(struct i2c_client *client, u8 offset, struct raw * r)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);

	if (tr16c0_read(priv->client, offset, r->data, sizeof(r->data)) != sizeof(r->data))
		return -ENODEV;

	return 0;
}

static ssize_t _show_raw_x_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	struct raw x;
	int i, p = 0;

	if (tr16c0_get_raw(client, map[priv->map_version].x_raw, &x) < 0)
		return -ENODEV;

	for (i=sizeof(x.data) - 2; i>0; i-=2)
		p += sprintf(buf+p, "%+04d ", (signed char)x.data[i]);

	return p + sprintf(buf+p, "\n");
}

static DEVICE_ATTR(raw_x, S_IWUSR | S_IRUGO, _show_raw_x_data, NULL);

static ssize_t _show_raw_y_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	struct raw y;
	int i, p = 0;

	if (tr16c0_get_raw(client, map[priv->map_version].y_raw, &y) < 0)
		return -ENODEV;

	for (i=sizeof(y.data) - 2; i>0; i-=2)
		p += sprintf(buf+p, "%+04d ", (signed char)y.data[i]);

	return p + sprintf(buf+p, "\n");
}

static DEVICE_ATTR(raw_y, S_IWUSR | S_IRUGO, _show_raw_y_data, NULL);

static ssize_t _store_raw_mode(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct tr16c0_priv *priv = i2c_get_clientdata(client);

	if (!map[priv->map_version].internal_enable) {
		dev_err(&client->dev, "missing info.\n");
		return -ENODEV;
	}

	if (strict_strtoul(buf, 10, &priv->raw_mode) < 0)
		return -EINVAL;

	tr16c0_write_u8(client, map[priv->map_version].internal_enable, priv->raw_mode);

	return count;
}
static DEVICE_ATTR(raw_mode, S_IWUSR | S_IRUGO, NULL, _store_raw_mode);


#endif // DEBUG

static struct attribute *sysfs_attrs[] = {
	&dev_attr_calibration_trigger.attr,
#ifdef DEBUG
	&dev_attr_poll_ms.attr,
	&dev_attr_data.attr,
	&dev_attr_raw_x.attr,
	&dev_attr_raw_y.attr,
	&dev_attr_raw_mode.attr,
#endif
	NULL
};

static struct attribute_group attr_group = {
	.attrs = sysfs_attrs,
};

static int tr16c0_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tr16c0_platform_data *pdata = client->dev.platform_data;
	struct tr16c0_priv *priv;
	int retry = 2;

	int ret = 0;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);

	priv->client = client;
	client->addr = TR16C0_ADDR;

	if (pdata) {
		priv->irq = pdata->irq;
		priv->x_max = pdata->x_max;
		priv->y_max = pdata->y_max;
		priv->flags = pdata->flags;

		priv->reset = pdata->reset;
		priv->get_irq_level= pdata->get_irq_level;
	} else {
		ret = -ENODEV;
		goto err_no_pdata;
	}

	priv->poll_ms = POLL_MS;

	priv->input_dev = input_allocate_device();
	if (priv->input_dev == NULL) {
		ret = -ENOMEM;
		goto err_input_alloc_failed;
	}

	INIT_DELAYED_WORK(&priv->work, tr16c0_irq_worker);

	init_completion(&priv->irq_trigged);

	priv->regulator = regulator_get(&client->dev, "tsp_vcc");
	if (IS_ERR(priv->regulator)) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed to get regulator\n");
		goto err_regulator_get;
	}

	do {
		ret = tr16c0_startup_sequence(client);

		if (ret >= 0)
			break;

		msleep(100);
		tr16c0_power(client, 0);
	} while (retry--);

	if (ret < 0) {
		retry = 10;
		client->addr = 0x55;
		priv->poll_ms = POLL_MS_V2;
		priv->map_version = 1;

		dev_info(&client->dev, "trying v2 fw");
		do {
			ret = tr16c0_startup_sequence(client);

			if (ret >= 0)
				break;

			msleep(100);
			tr16c0_power(client, 0);
		} while (retry--);
	}

	if (ret < 0) {
		ret = -ENODEV;
		dev_err(&client->dev, "could not detect tsp in app mode.\n");
		goto err_detect_failed;
	}

	dev_dbg(&client->dev, "%s: tsp online @ 0x%02x, map v%d\n",
			__func__, client->addr, priv->map_version);

	priv->input_dev->name = id->name;

	set_bit(EV_SYN, priv->input_dev->evbit);
	set_bit(EV_ABS, priv->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, priv->input_dev->evbit);

	set_bit(ABS_MT_POSITION_X, priv->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, priv->input_dev->absbit);

	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_X, 0, priv->x_max, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_Y, 0, priv->y_max, 0, 0);

	input_mt_init_slots(priv->input_dev, MAX_FINGERS);

	ret = input_register_device(priv->input_dev);
	if (ret) {
		ret = -ENODEV;
		dev_err(&client->dev, "%s: Unable to register %s input device\n",
				__FUNCTION__, priv->input_dev->name);
		goto err_input_register_failed;
	}

	ret = request_irq(priv->irq,
			tr16c0_irq_handler,
			IRQF_TRIGGER_FALLING,
			client->name, priv);

	if (ret) {
		ret = -ENODEV;
		dev_err(&client->dev, "request_irq failed\n");
		goto err_irq_request_failed;
	}

	ret = sysfs_create_group(&client->dev.kobj, &attr_group);

#ifdef CONFIG_HAS_EARLYSUSPEND
	priv->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	priv->early_suspend.suspend = tr16c0_early_suspend;
	priv->early_suspend.resume = tr16c0_late_resume;

	register_early_suspend(&priv->early_suspend);
#endif

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;

err_irq_request_failed:
err_input_register_failed:

err_detect_failed:
	tr16c0_power(client, 0);
	regulator_put(priv->regulator);

err_regulator_get:
	input_free_device(priv->input_dev);

err_input_alloc_failed:

err_no_pdata:
	kfree(priv);

	return ret;
}

static int tr16c0_remove(struct i2c_client *client)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &attr_group);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&priv->early_suspend);
#endif
	free_irq(priv->irq, priv);

	input_unregister_device(priv->input_dev);

	tr16c0_power(client, 0);

	regulator_put(priv->regulator);

	kfree(priv);
	return 0;
}

static int tr16c0_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);

	priv->state = ST_SUSPEND;

	disable_irq(priv->irq);

	tr16c0_power(client, 0);

	return 0;
}

static int tr16c0_resume(struct i2c_client *client)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	int retry;
	int ret;

	enable_irq(priv->irq);

	retry = 5;
	do {
		ret = tr16c0_startup_sequence(client);

		if (ret >= 0) {
			dev_dbg(&client->dev, "%s: tsp online.\n", __FUNCTION__);
			break;
		}

		msleep(100);
		tr16c0_power(client, 0);
	} while (retry--);

	if (ret < 0)
		dev_err(&client->dev, "%s: failed ?\n", __FUNCTION__);

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tr16c0_early_suspend(struct early_suspend *h)
{
	struct tr16c0_priv *priv =
		container_of(h, struct tr16c0_priv, early_suspend);
	tr16c0_suspend(priv->client, PMSG_SUSPEND);
}

static void tr16c0_late_resume(struct early_suspend *h)
{
	struct tr16c0_priv *priv =
		container_of(h, struct tr16c0_priv, early_suspend);
	tr16c0_resume(priv->client);
}
#endif

static const struct i2c_device_id tr16c0_id[] = {
	{ TR16C0_NAME, 0 },
	{ }
};

static struct i2c_driver tr16c0_driver = {
	.probe		= tr16c0_probe,
	.remove		= tr16c0_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= tr16c0_suspend,
	.resume		= tr16c0_resume,
#endif
	.id_table	= tr16c0_id,
	.driver = {
		.name	= TR16C0_NAME,
	},
};

static int __init tr16c0_init(void)
{
	return i2c_add_driver(&tr16c0_driver);
}

static void __exit tr16c0_exit(void)
{
	i2c_del_driver(&tr16c0_driver);
}

module_init(tr16c0_init);
module_exit(tr16c0_exit);

MODULE_AUTHOR("Guillaume Revaillot <revaillot@archos.com>");
MODULE_DESCRIPTION("TR16C0 Touchscreen Driver");
MODULE_LICENSE("GPL");

