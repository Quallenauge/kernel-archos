/*
 *    cpt_i2c_tsp.c : 06/12/2011
 *    g.revaillot, revaillot@archos.com
 *	
 * CPT i2c tsp - novatek chip.
 *
 */

//#define DEBUG
//#define DEBUG_DUMP

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

#include <linux/input/cpt_i2c_tsp.h>

#define MS_TO_NS(x)	(x * 1E6L)

#define MAX_FINGERS	8
#define RESET_CMD	0x5a

struct cpt_i2c_tsp_priv {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct regulator *regulator;

	void (*reset)(int);

	int irq;

	int x_min;
	int y_min;
	int x_max;
	int y_max;
	int x_scale;
	int y_scale;
	int x_offset;
	int y_offset;

	int point_state;

	int state;

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

enum {
	READ_FW	= 0,
	READ_CURSOR = 1,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cpt_i2c_tsp_early_suspend(struct early_suspend *h);
static void cpt_i2c_tsp_late_resume(struct early_suspend *h);
#endif

static void cpt_i2c_tsp_power(struct i2c_client *client, int on_off)
{
	struct cpt_i2c_tsp_priv *priv = i2c_get_clientdata(client);
	static int state = -1;

	if (state == on_off)
		return;

	state = on_off;

	if (on_off) {
		regulator_enable(priv->regulator);
	} else {
		regulator_disable(priv->regulator);
	}
}


static int cpt_i2c_tsp_write(struct i2c_client * client, u8 addr, u8 *value, u8 len)
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

static inline int cpt_i2c_tsp_write_u8(struct i2c_client * client, u8 addr, u8 value) {
	return cpt_i2c_tsp_write(client, addr, &value, sizeof(u8));
}

static int cpt_i2c_tsp_read(struct i2c_client * client, int mode, u8 addr, u8 *value, u8 len)
{
	struct i2c_msg msg[2];
	int chip_addr = client->addr;
	int ret;
	
	if (mode == READ_CURSOR)
		chip_addr = CPT_I2C_TSP_CURSOR_ADDR;

	msg[0].addr = chip_addr;
	msg[0].flags = 0;
	msg[0].buf = &addr;
	msg[0].len = sizeof(addr);

	msg[1].addr = chip_addr;
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

static int scale(int value, int min, int max, int scale, int offset)
{
	value = value + ((value - max / 2) * scale) / 1000 + offset;

	if (value < min) value = min;

	if (value > max) value = max;

	return value;
}

#define scale_x(v, p) scale(v, p->x_min, p->x_max, p->x_scale, p->x_offset)
#define scale_y(v, p) scale(v, p->y_min, p->y_max, p->y_scale, p->y_offset)

static irqreturn_t cpt_i2c_tsp_irq_handler(int irq, void * p)
{
	struct cpt_i2c_tsp_priv *priv = p;
	int new_point_state = 0;
	int ret;
	int i;

	struct slot {
		u8 id_stat;
		u8 x_h;
		u8 y_h;
		u8 xy_l;
		u8 area;
		u8 pressure;
	} __attribute__ ((packed));

	enum state {
		CURSOR_UNKNOWN = 0,
		CURSOR_DOWN = 2,
		CURSOR_UP = 3,
	};

	struct slot slots[MAX_FINGERS];

	ret = cpt_i2c_tsp_read(priv->client, READ_CURSOR, 0, (char*) slots, sizeof(slots));

	if (ret < 0) {
		dev_info(&priv->client->dev, "could not talk to tsp.)\n");
		goto exit_work;
	}

#ifdef DEBUG_DUMP
	print_hex_dump_bytes(KERN_ERR, 0, (char*)slots, sizeof(slots));
#endif

	for (i=0; i < ARRAY_SIZE(slots); i++) {
		struct slot * s = &slots[i];

		int id = s->id_stat >> 3;
		int st = s->id_stat & 0x3;

		if (st == CURSOR_DOWN) {
			int x = scale_x((s->x_h << 4) + (s->xy_l >> 4), priv);
			int y = scale_y((s->y_h << 4) + (s->xy_l & 0x0f), priv);

			dev_dbg(&priv->client->dev, "%i d = %d, st = %d,x=%d, y=%d, p=%d, a=%d\n", i, id, st, x, y, s->pressure, s->area);

			input_mt_slot(priv->input_dev, id);
			input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, true);

			input_report_abs(priv->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(priv->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(priv->input_dev, ABS_MT_TOUCH_MAJOR, s->pressure);
			input_report_abs(priv->input_dev, ABS_MT_WIDTH_MAJOR, s->area);

			new_point_state |= (1 << id);
		}
	}

	for (i=1; i < MAX_FINGERS; i++) {
		if ((priv->point_state & (1 << i)) && !(new_point_state & (1 << i))) {
			input_mt_slot(priv->input_dev, i);
			input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, false);
		}
	}

	priv->point_state = new_point_state;

	input_sync(priv->input_dev);

exit_work:
	return IRQ_HANDLED;
}

static int cpt_i2c_tsp_startup_sequence(struct i2c_client *client)
{
	struct cpt_i2c_tsp_priv *priv = i2c_get_clientdata(client);
	int retry = 5;
	int ret;

	if (priv->reset)
		priv->reset(1);

	msleep(100);

	if (priv->reset)
		priv->reset(0);

	cpt_i2c_tsp_power(client, 1);

	while ((ret = cpt_i2c_tsp_write_u8(client, 0, RESET_CMD)) < 0) {
		if (!--retry)
			break;
		dev_info(&client->dev, "ping (%d)\n", retry);
		msleep(100);
	}

	if (ret < 0) {
		dev_info(&client->dev, "could not talk to tsp.)\n");
		goto fail;
	}

	priv->state = ST_IDLING;

	return 0;
fail:
	cpt_i2c_tsp_power(client, 0);

	return -ENODEV;
} 

#ifdef DEBUG
static ssize_t _show_x_scale(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct cpt_i2c_tsp_priv *priv = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", priv->x_scale);
}

static ssize_t _store_x_scale(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct cpt_i2c_tsp_priv *priv = i2c_get_clientdata(client);

	strict_strtol(buf, 10, (long*)&priv->x_scale);

	return count;
}
static DEVICE_ATTR(x_scale, S_IWUSR | S_IRUGO, _show_x_scale, _store_x_scale);

static ssize_t _show_y_scale(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct cpt_i2c_tsp_priv *priv = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", priv->y_scale);
}

static ssize_t _store_y_scale(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct cpt_i2c_tsp_priv *priv = i2c_get_clientdata(client);

	strict_strtol(buf, 10, (long*)&priv->y_scale);

	return count;
}
static DEVICE_ATTR(y_scale, S_IWUSR | S_IRUGO, _show_y_scale, _store_y_scale);

static ssize_t _show_x_offset(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct cpt_i2c_tsp_priv *priv = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", priv->x_offset);
}

static ssize_t _store_x_offset(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct cpt_i2c_tsp_priv *priv = i2c_get_clientdata(client);

	strict_strtol(buf, 10, (long*)&priv->x_offset);

	return count;
}
static DEVICE_ATTR(x_offset, S_IWUSR | S_IRUGO, _show_x_offset, _store_x_offset);

static ssize_t _show_y_offset(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct cpt_i2c_tsp_priv *priv = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", priv->y_offset);
}

static ssize_t _store_y_offset(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct cpt_i2c_tsp_priv *priv = i2c_get_clientdata(client);

	strict_strtol(buf, 10, (long*)&priv->y_offset);

	return count;
}
static DEVICE_ATTR(y_offset, S_IWUSR | S_IRUGO, _show_y_offset, _store_y_offset);

#endif // DEBUG

static struct attribute *sysfs_attrs[] = {
#ifdef DEBUG
	&dev_attr_x_scale.attr,
	&dev_attr_y_scale.attr,
	&dev_attr_x_offset.attr,
	&dev_attr_y_offset.attr,
#endif
	NULL
};

static struct attribute_group attr_group = {
	.attrs = sysfs_attrs,
};

static int cpt_i2c_tsp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cpt_i2c_tsp_platform_data *pdata = client->dev.platform_data;
	struct cpt_i2c_tsp_priv *priv;
	int retry = 2;

	int ret = 0;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);

	priv->client = client;

	if (pdata) {
		priv->irq = pdata->irq;
		priv->reset = pdata->reset;

		priv->x_max = pdata->x_max;
		priv->y_max = pdata->y_max;

		priv->x_scale = pdata->x_scale;
		priv->y_scale = pdata->y_scale;

		priv->x_offset = pdata->x_offset;
		priv->y_offset = pdata->y_offset;
	} else {
		ret = -ENODEV;
		goto err_no_pdata;
	}

	priv->input_dev = input_allocate_device();
	if (priv->input_dev == NULL) {
		ret = -ENOMEM;
		goto err_input_alloc_failed;
	}

	priv->regulator = regulator_get(&client->dev, "tsp_vcc");
	if (IS_ERR(priv->regulator)) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed to get regulator\n");
		goto err_regulator_get;
	}

	do {
		ret = cpt_i2c_tsp_startup_sequence(client);

		if (ret >= 0) {
			dev_dbg(&client->dev, "%s: tsp online\n", __FUNCTION__);
			break;
		}

		msleep(100);
		cpt_i2c_tsp_power(client, 0);

	} while (retry--);

	if (ret < 0) {
		ret = -ENODEV;
		dev_err(&client->dev, "could not detect tsp in app mode.\n");
		goto err_detect_failed;
	}

	priv->input_dev->name = id->name;

	set_bit(EV_SYN, priv->input_dev->evbit);
	set_bit(EV_ABS, priv->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, priv->input_dev->evbit);

	set_bit(ABS_MT_POSITION_X, priv->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, priv->input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, priv->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, priv->input_dev->absbit);

	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_X, priv->x_min, priv->x_max, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_Y, priv->y_min, priv->y_max, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xff, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_WIDTH_MAJOR, 0, 0xff, 0, 0);

	input_mt_init_slots(priv->input_dev, MAX_FINGERS);

	ret = input_register_device(priv->input_dev);
	if (ret) {
		ret = -ENODEV;
		dev_err(&client->dev, "%s: Unable to register %s input device\n",
				__FUNCTION__, priv->input_dev->name);
		goto err_input_register_failed;
	}

	ret = request_threaded_irq(priv->irq,
			NULL, cpt_i2c_tsp_irq_handler,
			IRQF_TRIGGER_FALLING,
			client->name, priv);

	if (ret) {
		ret = -ENODEV;
		dev_err(&client->dev, "request_threaded_irq failed\n");
		goto err_irq_request_failed;
	}

	ret = sysfs_create_group(&client->dev.kobj, &attr_group);

#ifdef CONFIG_HAS_EARLYSUSPEND
	priv->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	priv->early_suspend.suspend = cpt_i2c_tsp_early_suspend;
	priv->early_suspend.resume = cpt_i2c_tsp_late_resume;

	register_early_suspend(&priv->early_suspend);
#endif

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;

err_irq_request_failed:
err_input_register_failed:

err_detect_failed:
	cpt_i2c_tsp_power(client, 0);
	regulator_put(priv->regulator);

err_regulator_get:
	input_free_device(priv->input_dev);

err_input_alloc_failed:

err_no_pdata:
	kfree(priv);

	return ret;
}

static int cpt_i2c_tsp_remove(struct i2c_client *client)
{
	struct cpt_i2c_tsp_priv *priv = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &attr_group);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&priv->early_suspend);
#endif
	free_irq(priv->irq, priv);

	input_unregister_device(priv->input_dev);

	cpt_i2c_tsp_power(client, 0);

	regulator_put(priv->regulator);

	kfree(priv);
	return 0;
}

static int cpt_i2c_tsp_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct cpt_i2c_tsp_priv *priv = i2c_get_clientdata(client);

	priv->state = ST_SUSPEND;

	disable_irq(priv->irq);

	cpt_i2c_tsp_power(client, 0);

	return 0;
}

static int cpt_i2c_tsp_resume(struct i2c_client *client)
{
	struct cpt_i2c_tsp_priv *priv = i2c_get_clientdata(client);
	int retry;
	int ret;

	enable_irq(priv->irq);

	retry = 5;
	do {
		ret = cpt_i2c_tsp_startup_sequence(client);

		if (ret >= 0) {
			dev_dbg(&client->dev, "%s: tsp online.\n", __FUNCTION__);
			break;
		}

		msleep(100);
		cpt_i2c_tsp_power(client, 0);
	} while (retry--);

	if (ret < 0)
		dev_err(&client->dev, "%s: failed ?\n", __FUNCTION__);

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cpt_i2c_tsp_early_suspend(struct early_suspend *h)
{
	struct cpt_i2c_tsp_priv *priv =
		container_of(h, struct cpt_i2c_tsp_priv, early_suspend);
	cpt_i2c_tsp_suspend(priv->client, PMSG_SUSPEND);
}

static void cpt_i2c_tsp_late_resume(struct early_suspend *h)
{
	struct cpt_i2c_tsp_priv *priv =
		container_of(h, struct cpt_i2c_tsp_priv, early_suspend);
	cpt_i2c_tsp_resume(priv->client);
}
#endif

static const struct i2c_device_id cpt_i2c_tsp_id[] = {
	{ CPT_I2C_TSP_NAME, 0 },
	{ }
};

static struct i2c_driver cpt_i2c_tsp_driver = {
	.probe		= cpt_i2c_tsp_probe,
	.remove		= cpt_i2c_tsp_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= cpt_i2c_tsp_suspend,
	.resume		= cpt_i2c_tsp_resume,
#endif
	.id_table	= cpt_i2c_tsp_id,
	.driver = {
		.name	= CPT_I2C_TSP_NAME,
	},
};

static int __init cpt_i2c_tsp_init(void)
{
	return i2c_add_driver(&cpt_i2c_tsp_driver);
}

static void __exit cpt_i2c_tsp_exit(void)
{
	i2c_del_driver(&cpt_i2c_tsp_driver);
}

module_init(cpt_i2c_tsp_init);
module_exit(cpt_i2c_tsp_exit);

MODULE_AUTHOR("Guillaume Revaillot <revaillot@archos.com>");
MODULE_DESCRIPTION("CPT i2c Touchscreen Driver");
MODULE_LICENSE("GPL");

