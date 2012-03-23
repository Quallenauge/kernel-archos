/*
 *    pixcir_i2c_tsp.c : 18/04/2011
 *    g.revaillot, revaillot@archos.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// check refresh scan rate

//#define BENCH
//#define DEBUG

// do not allow pm : i2c transferts are not possible while chip is
// sleeping, preventing us to start calibration manually.
// XXX -> calibration needs to trigger a tsp restart to be safe.
#undef ALLOW_PM

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/kernel.h>

#include <linux/input/pixcir_i2c_tsp.h>

#define MCU_TYPE_NUC100       0x0101
#define MCU_TYPE_M16C         0x0201
#define MCU_TYPE_M16C_I2C     0x0211
#define MCU_TYPE_R8C3GA_I2C   0x0411
#define MCU_TYPE_R8C3MU_I2C   0x0412

#define MAX_FINGERS           4

#define MIN(a,b) 	(((a) < (b)) ? (a) : (b))

static const struct register_map_m48 {
	u8 touch;
	u8 old_touch;
	struct {
		u8 x_h;
		u8 x_l;
		u8 y_h;
		u8 y_l;
		u8 x_w;
		u8 y_w;
	} p[2];
	u8 power_mode;
	u8 int_mode;
	u8 int_width;
	u8 firmware_version_base;
	u8 specop;
	u8 eeprom_read_addr;
} map_m48 = {
	.touch = 0x00,
	.old_touch = 0x01,
	.p = {
		{ 0x03, 0x02, 0x05, 0x04, 0x0a, 0x0b},
		{ 0x07, 0x06, 0x09, 0x08, 0x0c, 0x0d},
	},
	.power_mode = 0x14,
	.int_mode = 0x15,
	.int_width = 0x16,
	.firmware_version_base = 0x30,
	.specop = 0x37,
	.eeprom_read_addr = 0x38,
};

static const struct register_map_m45 {
	u8 touch;
	u8 x_max_l;
	u8 y_max_l;
	u8 power_mode;
	u8 int_mode;
	u8 int_width;
	u8 cmd_resp;
	u8 raw_noX;
	u8 raw_noY;
	u8 raw_data1;
	u8 raw_data2;
	u8 raw_data3;
	u8 raw_data4;
	u8 maint_end;
	u8 maint_cmd;
} map_m45 = {
	.touch = 0x00,
	.x_max_l = 47,
	.y_max_l = 49,
	.int_mode = 52,
	.int_width = 53,
	.cmd_resp = 91,
	.raw_noX = 93,
	.raw_noY = 93,
	.raw_data1 = 94,
	.raw_data2 = 95,
	.raw_data3 = 96,
	.raw_data4 = 97,
	.maint_end = 246,
	.maint_cmd = 247,
};

enum {
	APP_MODE = 0,
	BOOT_MODE = 1,
};

enum {
	ST_IDLING = 0,		// std mode, not allowed to report events.
	ST_RUNNING = 1,		// std mode, reporting invents.
	ST_SUSPEND = 2,		// suspend mode, not powered.
	ST_BOOTLOADER = 3,	// bootloader mode.
	ST_UNKNOWN = 4,		// unconfigured mode
};

enum {
	DFUCMD_CALIBRATION	 = 0x03, // Trigger Calibration
	DFUCMD_GET_BOOT_SELECT	 = 0x04, // Get operation mode
	DFUCMD_SET_BOOT_SELECT	 = 0x05, // Change of operation mode
	DFUCMD_ERASE_BLOCK	 = 0x06, // Specific Block erase
	DFUCMD_WRITE		 = 0x07, // Write to specific address
	DFUCMD_READ		 = 0x08, // Read from specific address
	DFUCMD_SET_SCAN		 = 0x09, // Permit/forbid of scan
};

struct pixcir_priv {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct completion irq_trigged;
	struct regulator *regulator;
	struct regulator *regulator_supply;

	int is_m45;

#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
	int down;
#endif

	int irq;

	// current finger count.
	int finger_cnt;
	int state;

	int flags;

	// min/max;
	int x_min;
	int x_max;
	int y_min;
	int y_max;

	// scales and offset, needed to adapt to lcd.
	int x_scale;
	int y_scale;
	int x_offset;
	int y_offset;

#ifdef BENCH
	struct hrtimer timer;
	int irq_count;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void pixcir_early_suspend(struct early_suspend *h);
static void pixcir_late_resume(struct early_suspend *h);
#endif

static int pixcir_cycle_and_startup_sequence(struct i2c_client *client, int mode, int retry);

static void pixcir_power(struct i2c_client *client, int on_off)
{
	struct pixcir_priv *priv = i2c_get_clientdata(client);
	static int state = -1;

	if (state == on_off)
		return;

	dev_dbg(&priv->client->dev, "%s %s\n",
			__FUNCTION__, on_off ? "on" : "off");

	state = on_off;

	if (on_off) {
		regulator_enable(priv->regulator_supply);
		msleep(2);
		regulator_enable(priv->regulator);
	} else {
		regulator_disable(priv->regulator);
		regulator_disable(priv->regulator_supply);
	}
}

static int pixcir_write(struct i2c_client * client, u8 addr, u8 *value, u8 len)
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

static inline int pixcir_write_u8(struct i2c_client * client, u8 addr, u8 value) {
	return pixcir_write(client, addr, &value, sizeof(u8));
}

static inline int pixcir_write_u16(struct i2c_client * client, u8 addr, u16 value) {
	return pixcir_write(client, addr, (u8 *)&value, sizeof(u16));
}

static int pixcir_read(struct i2c_client * client, u8 addr, u8 *value, u8 len)
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

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));

	if (ret == 2)
		return len;
	else if (ret >= 0){
		return -EIO;
	} else {
		return ret;
	}
}

static inline int pixcir_read_u8(struct i2c_client * client, u8 addr, u8 *value) {
	return pixcir_read(client, addr, value, sizeof(u8));
}

static inline int pixcir_read_u16(struct i2c_client * client, u8 addr, u16 *value) {
	return pixcir_read(client, addr, (u8 *)value, sizeof(u16));
}

static int pixcir_cmd(struct i2c_client * client, u8 cmd, int wait_irq_timeout)
{
	struct pixcir_priv *priv = i2c_get_clientdata(client);
	u8 stat;

	dev_dbg(&client->dev, "%s %d\n", __FUNCTION__, cmd);

	if (priv->is_m45) {
		pixcir_write_u8(client, map_m45.maint_cmd, cmd);

		INIT_COMPLETION(priv->irq_trigged);

		if (wait_irq_timeout && wait_for_completion_interruptible_timeout(
					&priv->irq_trigged,
					msecs_to_jiffies(wait_irq_timeout)) <= 0) {
			dev_err(&client->dev, "missed irq pulse ?\n");
		}

		pixcir_read_u8(client, map_m45.maint_end, &stat);

		pixcir_write_u8(client, map_m45.maint_end, 0x00);
		pixcir_write_u8(client, map_m45.maint_cmd, 0xff);

		dev_dbg(&client->dev, "%s stat : %d\n", __FUNCTION__, stat);

		return stat;

	} else {
		return pixcir_write_u8(client, map_m48.specop, cmd);
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

static irqreturn_t pixcir_m48_irq(int irq, void *p)
{
	struct pixcir_priv *priv = p;

	u8 data[0x10];
	u8 i;

	if (pixcir_read(priv->client, 0, data, sizeof(data)) != sizeof(data)) {
		dev_err(&priv->client->dev, "%s FAIL\n", __FUNCTION__);
		goto exit_work;
	}

#ifdef DEBUG
	print_hex_dump_bytes(KERN_ERR, 0, data, sizeof(data));
#endif

	for (i = 0; i < 2; i++) {
		int x = 0, y = 0, w = 0, h = 0;
#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
		if (i)
			break;
#endif

		if (i < data[map_m48.touch]) {
			w = data[map_m48.p[i].x_w];
			h = data[map_m48.p[i].y_w];

			if (priv->flags & PIXCIR_FLAGS_XY_SWAP) {
				x = (data[map_m48.p[i].x_h] << 8) + data[map_m48.p[i].x_l];
				y = (data[map_m48.p[i].y_h] << 8) + data[map_m48.p[i].y_l];
			} else {
				x = (data[map_m48.p[i].y_h] << 8) + data[map_m48.p[i].y_l];
				y = (data[map_m48.p[i].x_h] << 8) + data[map_m48.p[i].x_l];
			}

			x = scale_x(x, priv);
			y = scale_y(y, priv);

			if (priv->flags & PIXCIR_FLAGS_INV_X)
				x = priv->x_min + (priv->x_max - x);

			if (priv->flags & PIXCIR_FLAGS_INV_Y)
				y = priv->y_min + (priv->y_max - y);

#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
			input_report_abs(priv->input_dev, ABS_X, x);
			input_report_abs(priv->input_dev, ABS_Y, y);
			input_report_abs(priv->input_dev, ABS_PRESSURE, w);

			if (priv->down) {
				input_report_key(priv->input_dev, BTN_TOUCH, 1);
				priv->down=0;
			}
#else
			input_mt_slot(priv->input_dev, i);
			input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(priv->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(priv->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(priv->input_dev, ABS_MT_TOUCH_MAJOR, w);
			input_report_abs(priv->input_dev, ABS_MT_TOUCH_MINOR, h);
#endif
		} else if (i < data[map_m48.old_touch] ||
				(data[map_m48.old_touch] == 0 && data[map_m48.touch] == 0) ) {
#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
			input_report_abs(priv->input_dev, ABS_PRESSURE, 0);
			input_report_key(priv->input_dev, BTN_TOUCH, 0);
			priv->down=1;
#else 
			input_mt_slot(priv->input_dev, i);
			input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, false);
#endif
		} else {
			//
		}

	}

	input_sync(priv->input_dev);

exit_work:
	//enable_irq(priv->irq);
	return IRQ_HANDLED;
}

#define LOW_BAT_WA

static irqreturn_t pixcir_m45_irq(int irq, void *p)
{
	struct finger {
		u8 status;
		u16 x;
		u16 y;
		u8 xw;
		u8 yw;
		u8 xz;
		u8 yz;
	} __attribute__ ((packed));

	struct pixcir_priv *priv = p;
	u8 data[4 * sizeof(struct finger) + 1];
	int read_len, i;

	read_len = 1 + (priv->finger_cnt * sizeof(struct finger));

	if (pixcir_read(priv->client, 0, data, read_len) != read_len) {
		dev_err(&priv->client->dev, "%s FAIL\n", __FUNCTION__);
		goto exit_work;
	}
#ifdef LOW_BAT_WA
	if (unlikely(!data[0])) {
		dev_err(&priv->client->dev, "%s DEAAAAD !\n", __FUNCTION__);

		priv->state = ST_IDLING;

		if (pixcir_cycle_and_startup_sequence(priv->client, APP_MODE, 5) < 0)
			dev_err(&priv->client->dev, "%s __DEAD__ ?!\n", __FUNCTION__);

		if (priv->state == ST_IDLING)
			priv->state = ST_RUNNING;

		goto exit_work;
	}
#endif
#ifdef DEBUG
	print_hex_dump_bytes(KERN_ERR, 0, data, read_len);
#endif

	for (i = 0; i < MIN(priv->finger_cnt, data[0]); i++) {
		struct finger * f = (struct finger *) (data + i * sizeof(*f) + 1);
		int id = (f->status >> 4);
		int up = (f->status & 1);
		int dw = (f->status & 2);

#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
		if (id)
			break;
#endif

		if (dw) {
#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
			input_report_abs(priv->input_dev, ABS_X, f->x);
			input_report_abs(priv->input_dev, ABS_Y, f->y);
			input_report_abs(priv->input_dev, ABS_PRESSURE, f->xz);

			if (priv->down) {
				input_report_key(priv->input_dev, BTN_TOUCH, 1);
				priv->down=1;
			}
#else
			input_mt_slot(priv->input_dev, id);
			input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(priv->input_dev, ABS_MT_POSITION_X, f->x);
			input_report_abs(priv->input_dev, ABS_MT_POSITION_Y, f->y);
			input_report_abs(priv->input_dev, ABS_MT_TOUCH_MAJOR, f->xz);
			input_report_abs(priv->input_dev, ABS_MT_WIDTH_MAJOR, f->xw);
#endif
		} else if (up) {
#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
			input_report_abs(priv->input_dev, ABS_PRESSURE, 0);
			input_report_key(priv->input_dev, BTN_TOUCH, 0);
			priv->down=0;
#else
			input_mt_slot(priv->input_dev, id);
			input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, false);
#endif
		}
	}

	if (data[0])
		input_sync(priv->input_dev);

	priv->finger_cnt = data[0];

exit_work:
	//enable_irq(priv->irq);
	return IRQ_HANDLED;
}

#ifdef BENCH
#define MS_TO_NS(x)	(x * 1E6L)

static enum hrtimer_restart irq_counter_func(struct hrtimer *timer)
{
	struct pixcir_priv *priv =
		container_of(timer, struct pixcir_priv, timer);

	static int max = 0;

	if (priv->irq_count)
		dev_info(&priv->client->dev,
				"%s:%d / max %d\n",
				__FUNCTION__, priv->irq_count, max);

	if (priv->irq_count > max)
		max = priv->irq_count;

	priv->irq_count = 0;

	hrtimer_start(&priv->timer,
		      ktime_set( 0, MS_TO_NS(1000)),
		      HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}
#endif

static irqreturn_t pixcir_hardirq(int irq, void * p)
{
	struct pixcir_priv *priv = p;

	switch (priv->state) {
		case ST_RUNNING:
#ifdef BENCH
			priv->irq_count++;
#endif
			//disable_irq_nosync(priv->irq);
			return IRQ_WAKE_THREAD;
		default:
			complete(&priv->irq_trigged);
			break;
	}


	return IRQ_HANDLED;
}

static int pixcir_startup_sequence(struct i2c_client *client, int mode)
{
	struct pixcir_priv *priv = i2c_get_clientdata(client);
	int retry = 6;
	int ret;
	u8 stat;

	pixcir_power(client, 1);

	if (mode == BOOT_MODE) {
		client->addr = PIXCIR_BOOT_ADDR;
		msleep(10);
	} else {
		client->addr = PIXCIR_ADDR;
		msleep(300);
	}

	while (((ret = pixcir_read_u8(client, 0, &stat)) < 0) && (retry)) {
		retry--;
		dev_info(&client->dev, "ping ? (%d)\n", retry);
		msleep(50);
	}

	printk("%s : got 0x%02x (%d)\n", __FUNCTION__, stat, ret);

	if (!retry) {
		dev_info(&client->dev, "could not talk to tsp %s.)\n",
				mode == BOOT_MODE ? "boot" : "app");
		goto fail;
	}

	if (mode == APP_MODE) {
		if (priv->is_m45) {

			if (1) {
				u8 buf[255];

				if (pixcir_read(client, 0, buf, sizeof(buf)) < sizeof(buf))
					goto fail;

				print_hex_dump_bytes(KERN_ERR, 0, buf, sizeof(buf));
			}

			// irq active high, it mode enabled, report enabled
			if (pixcir_write_u8(client,
						map_m45.int_mode,
						(1<<4) | (1<<3) | (1<<2)) < 1)
				goto fail;
		} else {
#ifdef ALLOW_PM
			pixcir_read_u8(client, map_m48.power_mode, &stat);
			printk("PM:%02x\n", stat);
			// enable entering sleep
			pixcir_write_u8(client,
					map_m48.power_mode,
					(1<<10) | (1<<2));
#endif

			// irq active high, triggered when tsp has something to report
			if (pixcir_write_u8(client,
						map_m48.int_mode,
						(1<<3) | (1<<2) | (1<<0)) < 1)
				goto fail;
		}

		priv->state = ST_IDLING;
	} else {
		priv->state = ST_BOOTLOADER;
	}


	return 0;
fail:
	pixcir_power(client, 0);
	return -ENODEV;
}

static int pixcir_cycle_and_startup_sequence(struct i2c_client *client, int mode, int retry)
{
	int ret;

	do {
		msleep(100);
		ret = pixcir_startup_sequence(client, mode);

		if (ret >= 0) {
			dev_dbg(&client->dev, "%s: tsp online.\n", __FUNCTION__);
			return 0;
		}

		msleep(100);
		pixcir_power(client, 0);
	} while (retry--);

	dev_err(&client->dev, "%s: failed, abort.\n", __FUNCTION__);

	return -ENODEV;
}

static ssize_t _show_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct pixcir_priv *priv = i2c_get_clientdata(client);

	switch(priv->state) {
		case ST_IDLING: return sprintf(buf, "IDLING\n");
		case ST_RUNNING: return sprintf(buf, "RUNNING\n");
		case ST_SUSPEND: return sprintf(buf, "SUSPEND\n");
		case ST_BOOTLOADER: return sprintf(buf, "BOOTLOADER\n");
	}
	return sprintf(buf, "UNKNOWN\n");
}
static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, _show_state , NULL);

static ssize_t _store_calibration(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct pixcir_priv *priv = i2c_get_clientdata(client);
	int retry = 3;
	int ret;

	if (priv->state != ST_RUNNING)
		return -ENODEV;

	priv->state = ST_IDLING;

	do {
		pixcir_power(client, 0);

		msleep(100);

		pixcir_startup_sequence(client, APP_MODE);

		msleep(100);

		ret = pixcir_cmd(client, DFUCMD_CALIBRATION, 10000);

		if (!priv->is_m45) {
			// no way to check calibration.
			break;
		} else if (ret == 0x01) {
			// ack from tsp
			break;
		}

		dev_err(&priv->client->dev, "%s retry (%d)\n", __FUNCTION__, ret);
	} while (--retry);

	priv->state = ST_RUNNING;

	if (!retry)
		return -EIO;

	dev_info(&priv->client->dev, "%s : ok\n", __FUNCTION__);

	return count;
}
static DEVICE_ATTR(calibration_trigger, S_IWUSR | S_IRUGO, NULL, _store_calibration);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_calibration_trigger.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = sysfs_attrs,
};

static int pixcir_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct pixcir_platform_data *pdata = client->dev.platform_data;
	struct pixcir_priv *priv;
	irq_handler_t pixcir_thread_fn;
	int ret;

	u8 tsp_version[4];

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);

	priv->client = client;

	if (pdata) {
		priv->irq = pdata->irq;
		priv->flags = pdata->flags;

		priv->x_min = pdata->x_min;
		priv->x_max = (pdata->x_max) ? pdata->x_max : 1280;

		priv->y_min = pdata->y_min;
		priv->y_max = (pdata->y_max) ? pdata->y_max : 800;

		priv->x_scale = pdata->x_scale;
		priv->y_scale = pdata->y_scale;

		priv->x_offset = pdata->x_offset;
		priv->y_offset = pdata->y_offset;
	}

#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
	priv->down = 0;
#endif

	init_completion(&priv->irq_trigged);

	priv->regulator_supply = regulator_get(&client->dev, "5V");
	if (IS_ERR(priv->regulator_supply)) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed to get regulator_supply\n");
		goto err_regulator_get;
	}

	priv->regulator = regulator_get(&client->dev, "tsp_vcc");
	if (IS_ERR(priv->regulator)) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed to get regulator\n");
		goto err_regulator_get;
	}

	ret = pixcir_cycle_and_startup_sequence(priv->client, BOOT_MODE, 1);

	if (ret < 0) {
		dev_err(&client->dev, "could not detect tsp in boot mode.\n");
		priv->is_m45 = 1;
	}

	ret = pixcir_cycle_and_startup_sequence(priv->client, APP_MODE, 5);

	if (ret < 0) {
		ret = -ENODEV;
		dev_err(&client->dev, "could not detect tsp in app mode.\n");
		goto err_detect_failed;
	}

	if (priv->is_m45) {
		dev_info(&client->dev, "M45!\n");
		pixcir_thread_fn =  pixcir_m45_irq;
	} else {
		pixcir_read(client, map_m48.firmware_version_base, tsp_version, sizeof(tsp_version));
		dev_info(&client->dev, "M8 fw version %x:%x:%x - rev r%d\n",
				tsp_version[2], tsp_version[1], tsp_version[0], tsp_version[3]);

		pixcir_thread_fn = pixcir_m48_irq;
	}

	ret = request_threaded_irq(priv->irq, pixcir_hardirq, pixcir_thread_fn,
			IRQF_TRIGGER_RISING, client->name, priv);
	if (ret) {
		ret = -ENODEV;
		dev_err(&client->dev, "request_irq failed\n");
		goto err_irq_request_failed;
	}
	enable_irq_wake(priv->irq);
	device_init_wakeup(&client->dev, true);
	
	dev_info(&client->dev, "x_min/max : %d/%d - y_min/max : %d/%d\n",
			priv->x_min, priv->x_max, priv->y_min, priv->y_max);

	priv->input_dev = input_allocate_device();
	if (priv->input_dev == NULL) {
		ret = -ENOMEM;
		goto err_input_alloc_failed;
	}

	priv->input_dev->name = id->name;

	set_bit(EV_SYN, priv->input_dev->evbit);
	set_bit(EV_ABS, priv->input_dev->evbit);

#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
	set_bit(EV_KEY, priv->input_dev->evbit);
	set_bit(BTN_TOUCH, priv->input_dev->keybit);
#else
	set_bit(ABS_MT_POSITION_X, priv->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, priv->input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, priv->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, priv->input_dev->absbit);


	if (!priv->is_m45)
		set_bit(ABS_MT_TOUCH_MINOR, priv->input_dev->absbit);

	input_mt_init_slots(priv->input_dev, MAX_FINGERS);
#endif

#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
	input_set_abs_params(priv->input_dev, ABS_X, priv->x_min, priv->x_max, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_Y, priv->y_min, priv->y_max, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_PRESSURE, 0, 0x80, 0, 0);
#else
	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_X, priv->x_min, priv->x_max, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_Y, priv->y_min, priv->y_max, 0, 0);

	input_set_abs_params(priv->input_dev, ABS_MT_TOUCH_MAJOR, 0, 0x80, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_WIDTH_MAJOR, 0, 0x80, 0, 0);

	if (!priv->is_m45)
		input_set_abs_params(priv->input_dev, ABS_MT_TOUCH_MINOR, 0, 0x80, 0, 0);
#endif

	ret = input_register_device(priv->input_dev);
	if (ret) {
		ret = -ENODEV;
		dev_err(&client->dev, "%s: Unable to register %s input device\n",
				__FUNCTION__, priv->input_dev->name);
		goto err_input_register_failed;
	}

	ret = sysfs_create_group(&client->dev.kobj, &attr_group);

#ifdef CONFIG_HAS_EARLYSUSPEND
	priv->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	priv->early_suspend.suspend = pixcir_early_suspend;
	priv->early_suspend.resume = pixcir_late_resume;

	register_early_suspend(&priv->early_suspend);
#endif

#ifdef BENCH
	hrtimer_init(&priv->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	priv->timer.function = irq_counter_func;
	hrtimer_start(&priv->timer, ktime_set( 0, MS_TO_NS(1000)), HRTIMER_MODE_REL);
	priv->irq_count = 0;
#endif

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;

err_input_register_failed:
	input_free_device(priv->input_dev);

err_input_alloc_failed:
	disable_irq_wake(priv->irq);
	free_irq(priv->irq, priv);

	pixcir_power(client, 0);

err_irq_request_failed:

err_detect_failed:
	regulator_put(priv->regulator);
	regulator_put(priv->regulator_supply);

err_regulator_get:
	kfree(priv);

	return ret;
}

static int pixcir_remove(struct i2c_client *client)
{
	struct pixcir_priv *priv = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &attr_group);

#ifdef BENCH
	hrtimer_cancel(&priv->timer);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&priv->early_suspend);
#endif
	disable_irq_wake(priv->irq);
	free_irq(priv->irq, priv);

	input_unregister_device(priv->input_dev);

	pixcir_power(client, 0);

	regulator_put(priv->regulator);
	regulator_put(priv->regulator_supply);

	kfree(priv);
	return 0;
}

static int pixcir_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct pixcir_priv *priv = i2c_get_clientdata(client);

	priv->state = ST_SUSPEND;

	disable_irq(priv->irq);

	pixcir_power(client, 0);

	return 0;
}

static int pixcir_resume(struct i2c_client *client)
{
	struct pixcir_priv *priv = i2c_get_clientdata(client);

	enable_irq(priv->irq);

	if (pixcir_cycle_and_startup_sequence(priv->client, APP_MODE, 5) < 0)
		dev_err(&client->dev, "%s: failed ?\n", __FUNCTION__);

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void pixcir_early_suspend(struct early_suspend *h)
{
	struct pixcir_priv *priv =
		container_of(h, struct pixcir_priv, early_suspend);
	pixcir_suspend(priv->client, PMSG_SUSPEND);
}

static void pixcir_late_resume(struct early_suspend *h)
{
	struct pixcir_priv *priv =
		container_of(h, struct pixcir_priv, early_suspend);
	pixcir_resume(priv->client);
}
#endif

static const struct i2c_device_id pixcir_id[] = {
	{ PIXCIR_NAME, 0 },
	{ }
};

static struct i2c_driver pixcir_driver = {
	.probe		= pixcir_probe,
	.remove		= pixcir_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= pixcir_suspend,
	.resume		= pixcir_resume,
#endif
	.id_table	= pixcir_id,
	.driver = {
		.name	= PIXCIR_NAME,
	},
};

static int __init pixcir_init(void)
{
	return i2c_add_driver(&pixcir_driver);
}

static void __exit pixcir_exit(void)
{
	i2c_del_driver(&pixcir_driver);
}

module_init(pixcir_init);
module_exit(pixcir_exit);

MODULE_AUTHOR("Guillaume Revaillot <revaillot@archos.com>");
MODULE_DESCRIPTION("Pixcir i2c touchscreen driver");
MODULE_LICENSE("GPL");

