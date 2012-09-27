/*
 *    tr16c0-i2c.c : 04/10/2011
 *    g.revaillot, revaillot@archos.com
 *
 *    Touchplus TR16C0 + Tango F45 ctsp driver
 */

//#define DEBUG
#define AUTOLOAD_FIRMWARE
#define DEBUG_INTERFACE

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
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/wakelock.h>
#include <linux/crc32.h>

#include <asm/uaccess.h>

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
		.version = 52,
		.int_mode = 62,
		.specop = 55,
		.x_raw = 61,
		.y_raw = 125,
		.internal_enable = 0, // XXX
		.cmd_mode = 56,
	},
	{
		.touch_base = 0,
		.version = 52,
		.int_mode = 62,
		.specop = 55,
		.x_raw = 61,
		.y_raw = 125,
		.internal_enable = 0, // XXX
		.cmd_mode = 56,
	}
};

struct finger {
	u16 x;
	u16 y;
	u8 id;
} __attribute__ ((packed));

struct tr16c0_priv {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct platform_device *pdev;

	struct completion irq_trigged;
	struct regulator *regulator;
	struct delayed_work work;
	struct wake_lock wakelock;

#ifdef DEBUG_INTERFACE
	struct class *class;
	struct cdev cdev;
	dev_t devid;
	int interrupt_flag;
	int autotune_status;
	int runmode;
	int temp_val;
	unsigned long raw_mode;
#endif

	char data[MAX_FINGERS * sizeof(struct finger) + 2];

	int irq;

	int x_max;
	int y_max;
	int flags;

	struct tr16c0_fw_version latest_fw_version;
	struct tr16c0_fw_version current_fw_version;

	int map_version;
	int no_poll;
	unsigned long poll_ms;

	void (*reset)(int);
	int (*get_irq_level)(void);
	int (*demux_i2c)(int);
	struct tr16c0_dead_areas * dead_areas;

	int finger_cnt;
	int point_state;

	// XXX rework, mutexify
	int state;
	int pre_mfg_state;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

enum {
	ST_IDLING = 0,		// std mode, not allowed to report events.
	ST_RUNNING = 1,		// std mode, reporting invents.
	ST_SUSPEND = 2,		// suspend mode, not powered.
	ST_BOOTLOADER = 3,	// bootloader mode.
	ST_MFG = 4,		// manuf mode.
	ST_UNKNOWN = 5,		// unconfigured mode
};

enum {
	CMD_NORMAL = 0,
	CMD_SELF_RAWDATA = 1,
	CMD_MUTUAL_RAWDATA = 2,
	CMD_CONFIG_READ = 3,
	CMD_CONFIG_WRITE = 4,
	CMD_CRC_CHECK = 5,
	CMD_PROGRAM_WRITE = 6,
	CMD_SELF_DIST_RAWDATA = 7,
	CMD_MUTUAL_DIST_RAWDATA = 8,
};

enum {
	IOCTL_AUTOTUNE_SET_ID = 1,
	IOCTL_AUTOTUNE_GET_ID,
	IOCTL_ENABLE_IRQ_ID,
	IOCTL_DISABLE_IRQ_ID,
	IOCTL_CMDMODE_SET_ID,
	IOCTL_MSIREG_SET_ID,
	IOCTL_MSIREG_GET_ID,
	IOCTL_BOOTLOADER_ID,
	IOCTL_RESET_TSP_ID,
	IOCTL_ID_INVALID,
};

#define IOCTL_MAGIC_NUMBER   0x55

#define IOCTL_CMD_AUTOTUNE_SET    _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_AUTOTUNE_SET_ID,1)
#define IOCTL_CMD_AUTOTUNE_GET    _IOC(_IOC_READ,   IOCTL_MAGIC_NUMBER, IOCTL_AUTOTUNE_GET_ID,1)
#define IOCTL_CMD_ENABLE_IRQ      _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_ENABLE_IRQ_ID,1)
#define IOCTL_CMD_DISABLE_IRQ     _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_DISABLE_IRQ_ID,1)
#define IOCTL_CMD_CMDMODE_SET     _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_CMDMODE_SET_ID,1)
#define IOCTL_CMD_MSIREG_SET      _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_MSIREG_SET_ID,1)
#define IOCTL_CMD_MSIREG_GET      _IOC(_IOC_READ,   IOCTL_MAGIC_NUMBER, IOCTL_MSIREG_GET_ID,1)
#define IOCTL_CMD_BOOTLOADER_SET  _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_BOOTLOADER_ID,1)
#define IOCTL_CMD_RESET_TSP_SET   _IOC(_IOC_WRITE,   IOCTL_MAGIC_NUMBER, IOCTL_RESET_TSP_ID,1)

#define TOUCHPLUS_BOOTLOADER_I2C_ADDRESS  0x5D    // 7-bit addressing
#define TOUCHPLUS_NORMAL_I2C_ADDRESS      0x55    // 7-bit addressing

#define MSI_REPORT                      0x00
#define MSI_VERSION                     52
#define MSI_SPECOP                      (MSI_VERSION	+ 3)    // 55
#define MSI_CMD_MODE                    (MSI_SPECOP + 1)      // 56
#define MSI_X_NBR                       (MSI_CMD_MODE + 1)    // 57
#define MSI_Y_NBR                       (MSI_X_NBR + 1)       // 58
#define MSI_KEY_NBR                     (MSI_Y_NBR + 1)       // 59
#define MSI_UNLOCK                      (MSI_KEY_NBR    + 1)  // 60
#define MSI_POWER_MODE                  (MSI_UNLOCK     + 1)  // 61
#define MSI_INT_MODE                    (MSI_POWER_MODE + 1)  // 62
#define MSI_INT_WIDTH                   (MSI_INT_MODE   + 1)  // 63
#define MSI_X_RES                       (MSI_INT_WIDTH  + 1)  // 64
#define MSI_Y_RES                       (MSI_X_RES      + 2)  // 66
#define MSI_UPDATE_POS	                (MSI_Y_RES      + 2)  // 68

#define ISP_UNLOCK_CODE        0x5d
#define AUTOTUNE_SUCCESS       0x55
#define CMD_AUTOTUNE           0x03
#define PACK_SIZE              144    // tpf encryption length
#define FACTORY_DATA_BUF_SIZE  144    // raw data buffer maximum  size
#define FACTORY_DATA_BUF_ADDR  96     // raw data address

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tr16c0_early_suspend(struct early_suspend *h);
static void tr16c0_late_resume(struct early_suspend *h);
#endif

static void tr16c0_power(struct i2c_client *client, int on_off)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	static int state = -1;

	pr_debug("%s: %d\n", __func__, on_off);

	if (state == on_off)
		return;

	state = on_off;

	if (priv->reset)
		priv->reset(0);

	if (on_off) {
		regulator_enable(priv->regulator);

		if (priv->demux_i2c)
			priv->demux_i2c(0);

		if (priv->reset) {
			priv->reset(1);
			msleep(10);
			priv->reset(0);
			msleep(10);
			priv->reset(1);
		}
	} else {
		if (priv->demux_i2c)
			priv->demux_i2c(1);

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

static inline int tr16c0_read_u8(struct i2c_client * client, u8 addr, u8 *value) {
	return tr16c0_read(client, addr, value, sizeof(u8));
}

static irqreturn_t tr16c0_irq_handler(int irq, void * p)
{
	struct tr16c0_priv *priv = p;

	dev_dbg(&priv->client->dev, "%s (%d)\n", __func__, priv->state);

#ifdef DEBUG_INTERFACE
	priv->interrupt_flag = 1;
#endif

	switch (priv->state) {
		case ST_RUNNING:
			schedule_work(&priv->work.work);
			break;

		case ST_MFG:
			return IRQ_HANDLED;

		default:
			complete(&priv->irq_trigged);
			break;

	}

	return IRQ_HANDLED;
}

int is_bad(struct tr16c0_priv * priv, struct finger * f)
{
	struct tr16c0_dead_areas * d = priv->dead_areas;
	int i;

	if (d == NULL)
		return 0;

	for (i = 0; i < d->num; i++) {
		int x1 = d->areas[i].x;
		int y1 = d->areas[i].y;
		int x2 = d->areas[i].x + d->areas[i].w;
		int y2 = d->areas[i].y + d->areas[i].h;

		if ((f->x >= x1) && (f->x < x2) &&
		    (f->y >= y1) && (f->y < y2)) {
			printk("%s : %d / %d BAD (%d)\n", __func__, f->x, f->y, i);
			return 1;
		}
	}

	return 0;
}

static void tr16c0_irq_worker(struct work_struct *work)
{
	struct tr16c0_priv *priv =
		container_of(to_delayed_work(work), struct tr16c0_priv, work);

	int new_point_state = 0;
	int read_len, i;

	read_len = 2 + priv->finger_cnt * sizeof(struct finger);

	if (tr16c0_read(priv->client, map[priv->map_version].touch_base, priv->data, read_len) != read_len) {
		dev_err(&priv->client->dev,
				"%s: could not read output data.\n",
				__FUNCTION__);
		goto exit_work;
	}

	for (i = 0; i < MIN(priv->finger_cnt, priv->data[0]); i++) {
		struct finger * f = (struct finger *) (priv->data + i * sizeof(*f) + 2);
		dev_dbg(&priv->client->dev, "f%i of %d, id%d @ %d/%d\n", i, priv->data[0], f->id, f->x, f->y);

		if (priv->flags & TR16C0_FLAGS_INV_X)
			f->x = priv->x_max - f->x;

		if (priv->flags & TR16C0_FLAGS_INV_Y)
			f->y = priv->y_max - f->y;

		if (is_bad(priv, f))
			continue;

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

	if (!priv->no_poll && !priv->get_irq_level())
		schedule_delayed_work(&priv->work,
				msecs_to_jiffies(priv->poll_ms));

	priv->finger_cnt = (priv->data[0]) ? priv->data[0] : 1;

exit_work:
	return;
}

#define DATA_LEN 128

static int tr16c0_get_data_hash(struct i2c_client *client, int *hash)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	char data[DATA_LEN] = {0,};
	int ret;

	ret = tr16c0_write_u8(priv->client, MSI_CMD_MODE, CMD_CONFIG_READ);
	if (ret < 0)
		return -ENODEV;

	mdelay(30);

	while (priv->get_irq_level());

	ret = tr16c0_read(priv->client, FACTORY_DATA_BUF_ADDR, data, DATA_LEN);
	if (ret < 0)
		return -ENODEV;

	*hash = crc32(0, data, DATA_LEN);

	print_hex_dump_bytes(KERN_ERR, 0, (unsigned char *) data, sizeof(data));

	ret = tr16c0_write_u8(priv->client, MSI_CMD_MODE, CMD_NORMAL);
	if (ret < 0)
		return -ENODEV;

	return 0;
}


static int tr16c0_startup_sequence(struct i2c_client *client)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	int retry = 10;
	int ret;

	struct version {
		u8 major;
		u8 minor;
		u8 svn;
	} __attribute__ ((packed)) v;

	tr16c0_power(client, 1);

	msleep(200);

	while ((ret = tr16c0_read(client, map[priv->map_version].version, (unsigned char *)&v, sizeof(v))) != sizeof(v)) {
		if (!--retry)
			break;
		dev_info(&client->dev, "ping ? (%d)\n", retry);
		msleep(50);
	}

	if (ret < 0)
		goto fail;

	priv->current_fw_version.major = v.major >> 4;
	priv->current_fw_version.minor = v.major & 0x0f;
	priv->current_fw_version.release = (v.minor & 0x0f) + 10 * (v.minor >> 4) + 100 * v.svn;

	print_hex_dump_bytes(KERN_ERR, 0, (unsigned char *) &v, sizeof(v));

	tr16c0_get_data_hash(client, &priv->current_fw_version.config_hash);

	dev_info(&client->dev, "%s: v%d.%d.%d %d - 0x%08x\n", __func__,
			priv->current_fw_version.major,
			priv->current_fw_version.minor,
			priv->current_fw_version.release, v.svn,
			priv->current_fw_version.config_hash);

	if (tr16c0_write_u8(client, map[priv->map_version].int_mode, 0x0a) < 1)
		goto fail;

	priv->state = ST_IDLING;

	return 0;
fail:
	dev_info(&client->dev, "could not talk to tsp.)\n");
	tr16c0_power(client, 0);
	return -ENODEV;
}

static int tr16c0_run_calibration(struct i2c_client *client)
{
	int timeout = CALIBRATION_LONG_DELAY_MS / 1000;
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	int ret;

	INIT_COMPLETION(priv->irq_trigged);

	if (priv->map_version > 0)
		tr16c0_write_u8(client, map[priv->map_version].cmd_mode, 0x00);

	ret = tr16c0_write_u8(client, map[priv->map_version].specop, 0x03);

	if (ret < 0) {
		ret = -ENODEV;
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
		ret = -ENODEV;
		goto fail;
	}

	ret = 0;

fail:
	if (priv->map_version > 0)
		tr16c0_write_u8(client, map[priv->map_version].cmd_mode, 0x00);

	return ret;
}

static ssize_t _store_calibration(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	int ret;

	if (priv->state != ST_RUNNING)
		return -ENODEV;

	priv->state = ST_IDLING;

	ret = tr16c0_run_calibration(client);

	priv->state = ST_RUNNING;

	if (ret < 0)
		count = ret;

	return count;
}
static DEVICE_ATTR(calibration_trigger, S_IRUGO | S_IWUGO, NULL, _store_calibration);

#ifdef DEBUG_INTERFACE
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

	if (strict_strtoul(buf, 10, &v))
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


#endif // DEBUG_INTERFACE

static struct attribute *sysfs_attrs[] = {
	&dev_attr_calibration_trigger.attr,
#ifdef DEBUG_INTERFACE
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


int tr16c0_enter_bootloader(struct i2c_client * client)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	unsigned char buf;
	int i = 0;

	if (priv->flags & TR16C0_FLAGS_TSP_USE_RESET && priv->reset) {

		priv->reset(1);
		priv->reset(0);
		mdelay(10);
		priv->reset(1);

		priv->client->addr = TOUCHPLUS_BOOTLOADER_I2C_ADDRESS;

		while ((tr16c0_read_u8(priv->client, 0, &buf) < 0) && (i < 20)) {
			dev_dbg(&client->dev, "%s: hard reset / waiting.. %i\n", __func__, i);
			mdelay(10);
			i++;
		}

		if (i < 20)
			return 0;

	}

	priv->client->addr = TOUCHPLUS_NORMAL_I2C_ADDRESS;

	tr16c0_write_u8(priv->client, MSI_UNLOCK, ISP_UNLOCK_CODE);
	tr16c0_write_u8(priv->client, MSI_CMD_MODE, CMD_PROGRAM_WRITE);

	dev_info(&client->dev, "software reset to LDROM\n");

	priv->client->addr = TOUCHPLUS_BOOTLOADER_I2C_ADDRESS;

	i = 0;
	while ((tr16c0_read_u8(priv->client, 0, &buf) < 0) && (i < 20)) {
		dev_dbg(&client->dev, "%s: soft reset / waiting.. %i\n", __func__, i);
		mdelay(10);
		i++;
	}

	if (i < 20)
		return 0;

	priv->client->addr = TOUCHPLUS_NORMAL_I2C_ADDRESS;
	dev_err(&client->dev, "%s: timeout\n", __func__);

	return -1;
}

static void tr16c0_flash_fw(struct i2c_client * client, const struct firmware * fw)
{
	struct fw_packet {
		char opcode;
		char data[143];
	} __attribute__ ((packed));

	struct fw_packet * fw_packet_ptr = (struct fw_packet *) fw->data;
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	int ret;

	int fw_packet_count = 0;
	int retry = 0;

	dev_info(&client->dev, "%s\n", __func__);

	if (fw->size % sizeof(struct fw_packet)) {
		dev_err(&client->dev, "%s: wrong firmware size (%d)", __func__, fw->size);
		return;
	}

	if (tr16c0_enter_bootloader(client) < 0) {
		dev_err(&client->dev, "%s: could not enter bootloader", __func__);
		return;
	}

	while (fw_packet_count < (fw->size / sizeof(struct fw_packet))) {
		char buf;

		dev_info(&client->dev, "%s: flashing packet %d / %d\n",
				__func__, fw_packet_count, fw->size / sizeof(*fw_packet_ptr));

		if (retry) {
			// XXX reset
		}

		if (retry > 20) {
			// XXX abort
		}

#ifdef DEBUG
		print_hex_dump_bytes(KERN_DEBUG, 0, (unsigned char *) fw_packet_ptr, sizeof(*fw_packet_ptr));
#endif

		ret = i2c_master_send(priv->client, (unsigned char *)fw_packet_ptr, sizeof(*fw_packet_ptr));

		if (ret != sizeof(*fw_packet_ptr)) {
			dev_err(&client->dev, "%s: flash packet %d tx error (%d), retry %d\n",
					__func__, fw_packet_count, ret, retry);
			retry++;
			continue;
		}

		if (fw_packet_ptr->opcode != 0x01) {

			mdelay(1);

			while (priv->get_irq_level());

			mdelay(1);

			ret = i2c_master_recv(client, &buf, 1);
			if (ret != 1) {
				dev_err(&client->dev, "%s: flash packet %d timeout error (%d), retry %d\n",
						__func__, fw_packet_count, ret, retry);
				retry++;
				continue;
			}

			if (buf & 0x80) {
				dev_err(&client->dev, "%s: flash packet %d 0x80 error (0x%02x), retry %d\n",
						__func__, fw_packet_count, buf, retry);
				retry++;
				continue;
			}

		} else {
			dev_info(&client->dev, "%s: done %d\n",
					__func__, fw_packet_count);
		}

		fw_packet_ptr++;
		fw_packet_count++;
		retry = 0;
	}

	dev_dbg(&client->dev, "%s: waiting for tsp to restart.\n", __FUNCTION__);

	priv->client->addr = TOUCHPLUS_NORMAL_I2C_ADDRESS;

	if (priv->flags & TR16C0_FLAGS_TSP_USE_RESET && priv->reset) {
		priv->reset(1);
		priv->reset(0);
		mdelay(10);
		priv->reset(1);
	}

	mdelay(500);

	retry = 5;
	do {
		ret = tr16c0_startup_sequence(client);

		if (ret >= 0) {
			dev_dbg(&client->dev, "%s: tsp online.\n", __FUNCTION__);
			break;
		}

		msleep(150);
		tr16c0_power(client, 0);
	} while (retry--);

	if (ret < 0)
		dev_err(&client->dev, "%s: failed ?\n", __FUNCTION__);

	tr16c0_run_calibration(client);

	dev_info(&client->dev, "%s done.\n", __func__);

	return;
}

static void tr16c0_fw_upgrade_cb(const struct firmware *fw, void *context)
{
	struct i2c_client * client = (struct i2c_client *) context;
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	int old_state;

	dev_info(&client->dev, "%s\n", __func__);

	if (!fw)
		return;

	switch (priv->state) {
		case ST_RUNNING:
		case ST_IDLING:
			break;

		default:
			dev_info(&client->dev, "%s: abort\n", __func__);
			goto release;
	}

	old_state = priv->state;

	priv->state = ST_BOOTLOADER;

	cancel_delayed_work_sync(&priv->work);

	wake_lock(&priv->wakelock);

	tr16c0_flash_fw(client, fw);

	wake_unlock(&priv->wakelock);

	priv->state = old_state;

release:
	release_firmware(fw);

	return;
}

enum {
	GLASS_UNKNOWN = 0,
	GLASS_05MM = 1,
	GLASS_07MM = 2,
};
struct tr16c0_tsp_fw_fingerprint {
	int glass_thickness;
	int major;
	int minor;
	int release;
	int config_hash;
} fw_fingerprints[] = {
	{GLASS_07MM, 5, 4, 515, 0xec9f356f},
	{GLASS_07MM, 5, 4, 533, 0xdd53306f},
	{GLASS_07MM, 5, 4, 1,   0xdd53306f},
	{GLASS_05MM, 5, 4, 1,   0x10f38c3f},
};

int tr16c0_guess_glass_thickness(struct tr16c0_fw_version * fw)
{
	int i;

	for (i=0; i < ARRAY_SIZE(fw_fingerprints); i++) {
		if ((fw->major == fw_fingerprints[i].major) &&
		    (fw->minor == fw_fingerprints[i].minor) &&
		    (fw->release == fw_fingerprints[i].release) &&
		    (fw->config_hash == fw_fingerprints[i].config_hash))
		return fw_fingerprints[i].glass_thickness;
	}

	// fallback to 05MM, new fw will improve tsp anyway.
	return GLASS_05MM;
}

static void tr16c0_request_firmware_nowait(struct i2c_client *client)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	static char fw_name[128];
	char fw_thickness_postfix[8] = {0,};

	if ((priv->current_fw_version.major == priv->latest_fw_version.major) &&
	    (priv->current_fw_version.minor == priv->latest_fw_version.minor) &&
	    (priv->current_fw_version.release == priv->latest_fw_version.release))
		return;

	if (tr16c0_guess_glass_thickness(&priv->current_fw_version) == GLASS_07MM)
		strncat(fw_thickness_postfix, "_07mm", sizeof(fw_thickness_postfix));

	snprintf(fw_name, sizeof(fw_name), "tr16c0_%d.%d.%d%s.bin",
			priv->latest_fw_version.major,
			priv->latest_fw_version.minor,
			priv->latest_fw_version.release,
			fw_thickness_postfix);

	dev_info(&client->dev, "%s: request %s\n", __func__, fw_name);

	request_firmware_nowait(
			THIS_MODULE, 1,
			fw_name,
			&client->dev,
			GFP_KERNEL,
			(void *) client,
			tr16c0_fw_upgrade_cb);

	return;
}

#ifdef DEBUG_INTERFACE
static int touchplus_ts_open(struct inode *inode, struct file *file)
{
	struct tr16c0_priv *priv = container_of(inode->i_cdev, struct tr16c0_priv, cdev);
	file->private_data = priv;

	priv->pre_mfg_state = priv->state;
	priv->state = ST_MFG;

	cancel_delayed_work_sync(&priv->work);

	return 0;
}

static int touchplus_ts_release(struct inode *inode, struct file *file)
{
	struct tr16c0_priv *priv = (struct tr16c0_priv *) file->private_data;
	priv->state = priv->pre_mfg_state;;
	file->private_data = NULL;
	return 0;
}

ssize_t touchplus_ts_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	struct tr16c0_priv *priv = file->private_data;
	char *data = kzalloc(MIN(count, PACK_SIZE), GFP_KERNEL);
	unsigned char buff[2];
	int opcode;
	int ret = 0;
	int i;

	switch (priv->runmode) {
		case CMD_PROGRAM_WRITE:
			memset(data, 0, PACK_SIZE);
			if (copy_from_user(data, buf, count)) {
				pr_debug("COPY FAIL ");
				ret = -EFAULT;
				break;
			}

			priv->client->addr = TOUCHPLUS_BOOTLOADER_I2C_ADDRESS;
			pr_debug("i2c slave address: %x : %d len %d\n", priv->client->addr, data[0], count);
			opcode = data[0];
			ret = i2c_master_send(priv->client,data,count);
			if (ret != count)
			{
				printk("143 bytes write error, ret = %d\n",ret);
				ret = -1;
				break;
			}
			priv->temp_val++;
			pr_debug("packet no = %3d\n",priv->temp_val);
			printk("%s:%d\n", __func__, __LINE__);
			if (opcode != 0x01) {  // not jump line

				mdelay(1);
				while (priv->get_irq_level()); // wait touch controller processing data to flash. INT is low means done.
				mdelay(1);
				ret = i2c_master_recv(priv->client,buff,1);
				if (ret != 1)
				{
					printk("read IIC slave status error,ret = %d\n",ret);
					ret = -1;
					break;
				}
			} else { // file download finished. wait touch screen controller jump to APROM
				if (priv->flags & TR16C0_FLAGS_TSP_USE_RESET) {
					priv->reset(1);
					priv->reset(0);
					mdelay(10);
					priv->reset(1);
				}

				mdelay(500);
				priv->client->addr = TOUCHPLUS_NORMAL_I2C_ADDRESS;
				pr_debug("i2c slave address: %x\n", priv->client->addr);
				i=0;
				buff[0] = 0;
				while(1)  // probe i2c address 0x55
				{
					ret = i2c_master_send(priv->client,buff,1);
					if (ret == 1) {ret = 0; break; }//probe sucess
					if (i++ > 10) {
						printk("Time out to APROM!!\n");
						ret = -1; // timer out, may be flash update fail
						break;
					}
					mdelay(500);
				}
				priv->runmode = CMD_NORMAL;
				printk("Jump to APROM!!\n");
				ret = 0; //success
				break;
			}

			printk("%s:%d\n", __func__, __LINE__);
			if ((buff[0]&0x80)&&(opcode != 0x01))
			{
				printk("download failed\n");
				ret = -1;
				break;
			}
			printk("%s:%d\n", __func__, __LINE__);
			break;

		default:
			break;
	}

	kfree(data);

	return ret;
}

ssize_t touchplus_ts_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	struct tr16c0_priv *priv = file->private_data;
	char *data = kzalloc(count, GFP_KERNEL);
	int ret = 1;

	switch (priv->runmode) {
		case CMD_NORMAL:
			if (priv->interrupt_flag) {
				priv->interrupt_flag = 0;
				ret = copy_to_user(buf, priv->data, count);
				if (!ret) {
					int i;
					for (i = 0; i < sizeof(priv->data); i++)
						if (0)
							pr_debug("report data: %d ", priv->data[i]);
				} else {
					ret = -1;
				}
			}
			break;

		case CMD_SELF_RAWDATA:
			{
				priv->temp_val = count;

				if ((!priv->temp_val) || (priv->temp_val > FACTORY_DATA_BUF_SIZE)) {ret = -1; break;}
				while(priv->get_irq_level());

				ret = tr16c0_read(priv->client, FACTORY_DATA_BUF_ADDR, data, priv->temp_val);
				if (ret != priv->temp_val) {
					pr_err("i2c self read error\n");
					ret = -1;
					break;
				}
				ret = copy_to_user(buf, data, priv->temp_val);
				if (ret)
					ret = -1;
			}
			break;
		case CMD_CONFIG_READ:
		case CMD_MUTUAL_RAWDATA:
			{
				priv->temp_val = count;
				if ((!priv->temp_val) || (priv->temp_val > FACTORY_DATA_BUF_SIZE)) {
					ret = -1;
					break;
				}

				while (priv->get_irq_level());

				ret = tr16c0_read(priv->client, FACTORY_DATA_BUF_ADDR, data, priv->temp_val);
				if (ret != priv->temp_val) {
					pr_err("i2c mutual read error\n");
					ret = -1;
					break;
				}

				ret = copy_to_user(buf, data, priv->temp_val);
				if (ret)
					ret = -1;
			}
			break;

		default:
			break;
	}

	kfree(data);

	return ret;
}

static int touchplus_ts_autotune_status(struct i2c_client *client)
{
	char buf[2];
	int ret = 0;

	ret = tr16c0_read(client, MSI_SPECOP, buf, 1);
	if (ret < 0) {
		pr_err("I2C read error!\n");
		return ret;
	}

	if (buf[0] == AUTOTUNE_SUCCESS) {
		pr_debug("autotune done\n");
		return 0;
	}

	return 1;
}

static int touchplus_ts_autotune(struct i2c_client *client)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	int ret = 0;
	int cnt = 0;

	priv->autotune_status = 1;

	if (tr16c0_write_u8(client, MSI_SPECOP, CMD_AUTOTUNE) < 0) {
		ret = -1;
		goto abort;
	}

	while (1) {
		ret = touchplus_ts_autotune_status(client);
		pr_info("%s:%d...\n", __func__, cnt);

		if (ret < 0)
			goto abort;

		if (ret == 0 )
			break;

		if (cnt++ > 100) {
			pr_err("autotune timer out!\n");
			return -1;
		}
		mdelay(1000);
	}

abort:
	priv->autotune_status = 0;

	return ret;
}

static long touchplus_ts_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	typedef struct {
		unsigned char addr;
		unsigned char val;
	} msi_reg;

	typedef struct {
		unsigned char addr;
		unsigned char len;
		unsigned char buf[250];
	} msi_regs;

	struct tr16c0_priv *priv = file->private_data;
	unsigned char iob[64];
	int retval = 0;
	msi_regs regs;
	msi_reg reg;

	if (_IOC_TYPE(cmd) != IOCTL_MAGIC_NUMBER)
		return -EFAULT;
	if (_IOC_NR(cmd) >= IOCTL_ID_INVALID)
		return -EFAULT;

	if (priv->client == NULL)
		return -EFAULT;

	pr_debug("IOCTL command: %08x\n", cmd);

	switch (cmd) {
		case IOCTL_CMD_AUTOTUNE_SET:
			pr_debug("touch screen autotune start!\n");
			retval = touchplus_ts_autotune(priv->client);
			break;

		case IOCTL_CMD_AUTOTUNE_GET:
			pr_debug("touch screnn autotune status!\n");
			retval = touchplus_ts_autotune_status(priv->client);
			break;

		case IOCTL_CMD_ENABLE_IRQ:
			pr_debug("enable IRQ!\n");
			//enable_irq(priv->irq);
			break;

		case IOCTL_CMD_DISABLE_IRQ:
			pr_debug("disable IRQ!\n");
			//disable_irq_nosync(priv->irq);
			break;

		case IOCTL_CMD_RESET_TSP_SET:
			pr_debug("Reset touch screen\n");

			if (priv->flags & TR16C0_FLAGS_TSP_USE_RESET) {
				priv->reset(1);
				priv->reset(0);
				mdelay(10);
				priv->reset(1);
			} else {
				priv->client->addr = TOUCHPLUS_NORMAL_I2C_ADDRESS;
				tr16c0_write_u8(priv->client, MSI_UNLOCK, ISP_UNLOCK_CODE);

				// software reset
				tr16c0_write_u8(priv->client, MSI_CMD_MODE, CMD_PROGRAM_WRITE);
				pr_info("software reset \n");
			}

			mdelay(300);
			break;

		case IOCTL_CMD_CMDMODE_SET:
			if (copy_from_user(&iob, (void *)arg, 1))
				return -EFAULT;

			pr_debug("CMD MODE: %d\n", iob[0]);

			priv->runmode = iob[0];

			if (priv->runmode == CMD_PROGRAM_WRITE) {
				cancel_delayed_work_sync(&priv->work);

				priv->temp_val = 0;

				if (tr16c0_enter_bootloader(priv->client) < 0)
					return -1;

			} else if (priv->runmode == CMD_SELF_RAWDATA) {
				cancel_delayed_work_sync(&priv->work);

				retval = tr16c0_write_u8(priv->client, MSI_CMD_MODE,iob[0]);
				if (retval < 0) {
					printk("switch self mode error!!\n");
					return -1;
				} else {
					retval = 0;
				}
				mdelay(30);
			} else if ((priv->runmode == CMD_MUTUAL_RAWDATA) || (priv->runmode == CMD_CONFIG_READ)) {
				cancel_delayed_work_sync(&priv->work);
				retval = tr16c0_write_u8(priv->client, MSI_CMD_MODE,iob[0]);
				if (retval < 0) { printk("switch mutual mode error!!\n"); return -1; }
				else retval = 0;
				mdelay(30);
			} else {
				retval = tr16c0_write_u8(priv->client, MSI_CMD_MODE,iob[0]);
				if (retval < 0) { printk("switch command mode error!!\n"); return -1; }
				else retval = 0;
			}
			break;

		case IOCTL_CMD_MSIREG_GET: /* read  MSI registers */
			pr_debug("read registers\n");
			if (copy_from_user( &regs, (void *)arg, sizeof(msi_regs)))
				return -EFAULT;

			pr_debug("register addr: %d,len: %d\n", regs.addr, regs.len);
			retval = tr16c0_read(priv->client, regs.addr, iob, regs.len);
			mdelay(regs.len);

			memcpy(regs.buf, iob, regs.len);
			if (copy_to_user( (void *) arg, &regs, sizeof(msi_regs)))
				return -EFAULT;
			break;

		case IOCTL_CMD_MSIREG_SET: /* write a single MSI registers */
			pr_debug("write registers\n");
			if (copy_from_user( &reg, (void *)arg, sizeof(msi_reg)))
				return -EFAULT;
			retval = tr16c0_write_u8(priv->client, reg.addr, reg.val);
			break;

		default:
			pr_err("unknown ioctl\n");
			retval = -EFAULT;
			break;

	}

	return retval;
}

static struct file_operations touchplus_fops = {
	.owner		= THIS_MODULE,
	.open		= touchplus_ts_open,
	.read		= touchplus_ts_read,
	.write		= touchplus_ts_write,
	.unlocked_ioctl	= touchplus_ts_ioctl,
	.release	= touchplus_ts_release,
};

static int touchplus_register_cdev(struct i2c_client *client)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	int ret;

	cdev_init(&priv->cdev, &touchplus_fops);
	priv->cdev.owner = THIS_MODULE;

	alloc_chrdev_region(&priv->devid, 0, 1, "touchplus");

	ret = cdev_add(&priv->cdev, priv->devid, 1);
	if (ret)
		goto err_cdev_add;

	priv->class = class_create(THIS_MODULE, "touchplus_class");
	if (IS_ERR(priv->class))
		goto err_class_create;

	if (IS_ERR(device_create(priv->class, NULL, priv->devid, NULL,
			"touchplus_i2c_ts" "%d", MINOR(priv->devid))))
		goto err_device_create;

	return 0;

err_device_create:
	class_destroy(priv->class);

err_class_create:
err_cdev_add:
	unregister_chrdev_region(priv->devid, 1);
	cdev_del(&priv->cdev);

	return ret;
}

static void touchplus_unregister_cdev(struct i2c_client *client)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);

	device_destroy(priv->class, priv->devid);
	class_destroy(priv->class);
	unregister_chrdev_region(priv->devid, 1);
	cdev_del(&priv->cdev);
}
#endif // DEBUG_INTERFACE

static int tr16c0_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tr16c0_platform_data *pdata = client->dev.platform_data;
	struct tr16c0_priv *priv;
	int retry = 2;
	int ret = -1;

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
		priv->demux_i2c = pdata->demux_i2c;
		priv->dead_areas = pdata->dead_areas;

		memcpy(&priv->latest_fw_version, &pdata->latest_fw_version, sizeof(struct tr16c0_fw_version));
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

	priv->regulator = regulator_get(&client->dev, pdata->regulator);
	if (IS_ERR(priv->regulator)) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed to get regulator\n");
		goto err_regulator_get;
	}

	ret = -1;

	do {
		if (priv->flags & TR16C0_FLAGS_TSP_V2) {
			break;
		}

		ret = tr16c0_startup_sequence(client);

		if (ret >= 0)
			break;

		msleep(150);
		tr16c0_power(client, 0);
	} while (retry--);

	if ((ret < 0))  {
		retry = 3;
		client->addr = 0x55;
		priv->poll_ms = POLL_MS_V2;
		priv->map_version = 1;

		if (priv->flags & TR16C0_FLAGS_TSP_V2)
			priv->map_version = 2;

		dev_info(&client->dev, "trying v2 fw\n");
		do {
			ret = tr16c0_startup_sequence(client);

			if (ret >= 0)
				break;

			msleep(150);
			tr16c0_power(client, 0);
		} while (retry--);
	}

	if (priv->flags & TR16C0_FLAGS_TSP_NO_POLL)
		priv->no_poll = 1;

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
	enable_irq_wake(priv->irq);
	device_init_wakeup(&client->dev, true);

#ifdef DEBUG_INTERFACE
	ret = touchplus_register_cdev(client);
	if (ret)
		dev_err(&client->dev, "touchplus_register_cdev failed\n");
#endif

	ret = sysfs_create_group(&client->dev.kobj, &attr_group);

	priv->pdev = platform_device_register_simple("archos_touchscreen", -1, NULL, 0);
	if (IS_ERR(priv->pdev))
		return ret;

	ret = sysfs_create_link(&priv->pdev->dev.kobj, &client->dev.kobj, "tsp");

#ifdef CONFIG_HAS_EARLYSUSPEND
	priv->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	priv->early_suspend.suspend = tr16c0_early_suspend;
	priv->early_suspend.resume = tr16c0_late_resume;

	register_early_suspend(&priv->early_suspend);

	wake_lock_init(&priv->wakelock, WAKE_LOCK_SUSPEND, "tr16c0");
#endif

#ifdef AUTOLOAD_FIRMWARE
	tr16c0_request_firmware_nowait(client);
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

	if (priv->state != ST_SUSPEND) {
		disable_irq_wake(priv->irq);
		disable_irq(priv->irq);
	}

	cancel_delayed_work_sync(&priv->work);

#ifdef DEBUG_INTERFACE
	touchplus_unregister_cdev(client);
#endif

	sysfs_remove_link(&priv->pdev->dev.kobj, "tsp");

	platform_device_unregister(priv->pdev);

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

	switch (priv->state) {
		case ST_BOOTLOADER:
		case ST_MFG:
			return 0;
	}

	priv->state = ST_SUSPEND;

	disable_irq_wake(priv->irq);
	disable_irq(priv->irq);

	tr16c0_power(client, 0);

	return 0;
}

static int tr16c0_resume(struct i2c_client *client)
{
	struct tr16c0_priv *priv = i2c_get_clientdata(client);
	int retry;
	int ret;

	if (priv->state != ST_SUSPEND)
		return 0;

	enable_irq(priv->irq);
	enable_irq_wake(priv->irq);

	retry = 5;
	do {
		ret = tr16c0_startup_sequence(client);

		if (ret >= 0) {
			dev_dbg(&client->dev, "%s: tsp online.\n", __FUNCTION__);
			break;
		}

		msleep(150);
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

