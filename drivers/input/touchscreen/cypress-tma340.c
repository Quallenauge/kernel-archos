/*
 *    cypress_tma340.c : 17/03/2011
 *    g.revaillot, revaillot@archos.com
 */

//#define BENCH

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/ihex.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/completion.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>

#include <linux/input/cypress-tma340.h>

#define BYD_FW_FILENAME "BYDAKS_8INCH.bin"

#define BYD_KEY { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 }

static const struct tma340_operation_register_map {
	u8 hst_mode;
	u8 tt_mode;
	u8 tt_stat;
	struct {
		u8 x_h;
		u8 x_l;
		u8 y_h;
		u8 y_l;
		u8 pressure;
		u8 id;
	} p[4];

	u8 gest_cnt;
	u8 gest_id;
	u8 gest_set;
	u8 magic_calib;
} dop = {
	.hst_mode = 0x00,
	.tt_mode = 0x01,
	.tt_stat = 0x02,
	.p = {
		{ 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 },
		{ 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x08 },
		{ 0x10, 0x11, 0x12, 0x13, 0x14, 0x15 },
		{ 0x16, 0x17, 0x18, 0x19, 0x1a, 0x15 },
	},
	.gest_cnt = 0x0e,
	.gest_id = 0x0f,
	.gest_set = 0x1e,
	.magic_calib = 0x1c,
};

static const struct tma340_sysinfo_register_map {
	u8 hst_mode;
	u8 bl_ver_hi;
	u8 tts_ver_hi;
	u8 app_id_hi;
	u8 app_ver_hi;
} dsys = {
	.hst_mode = 0x00,
	.bl_ver_hi =  0x0f,
	.tts_ver_hi = 0x11,
	.app_id_hi = 0x14,
	.app_ver_hi = 0x13,
};

static const struct tma340_bootloader_register_map {
	u8 bl_file;
	u8 bl_status;
	u8 bl_error;
	u8 bl_ver_hi;
	u8 bl_bld_ver_hi;
	u8 tts_ver_hi;
	u8 app_id_hi;
	u8 app_ver_hi;
} dbo = {
	.bl_file = 0x00,
	.bl_status = 0x01,
	.bl_error = 0x02,
	.bl_ver_hi =  0x03,
	.bl_bld_ver_hi =  0x05,
	.tts_ver_hi = 0x07,
	.app_id_hi = 0x0b,
	.app_ver_hi = 0x09,
};

struct tma340_version {
	u16 boot_version;
	u16 tts_version;
	u16 app_version;
	u16 app_id;
};

struct cypress_tma340_priv {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct completion irq_trigged;
	struct regulator *regulator;
	struct wake_lock wakelock;

	int state;
	int point_state;
	int upgrade_state;

#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
	int down;
#endif

	struct tma340_version version;

	int irq;

	int flags;

	u8 finger_max;
	u16 x_max;
	u16 y_max;

#ifdef BENCH
	struct hrtimer bench_timer;
	int irq_count;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

static u8 key[] = BYD_KEY;

enum {
	ST_IDLING = 0,		// std mode, not allowed to report events.
	ST_RUNNING = 1,		// std mode, reporting invents.
	ST_SUSPEND = 2,		// suspend mode, not powered.
	ST_BOOTLOADER = 3,	// bootloader mode.
	ST_UNKNOWN = 4,		// unconfigured mode
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cypress_tma340_early_suspend(struct early_suspend *h);
static void cypress_tma340_late_resume(struct early_suspend *h);
#endif

static void cypress_tma340_power(struct i2c_client *client, int on_off)
{
	struct cypress_tma340_priv *priv = i2c_get_clientdata(client);
	static int state = -1;

	if (state == on_off)
		return;

	dev_info(&client->dev, "%s %s\n",
			__FUNCTION__, on_off ? "on" : "off");

	state = on_off;

	if (on_off) {
		regulator_enable(priv->regulator);
	} else {
		regulator_disable(priv->regulator);
	}
}

static int cypress_tma340_write(struct i2c_client * client, u8 addr, u8 *value, u8 len)
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

static inline int cypress_tma340_write_u8(struct i2c_client * client, u8 addr, u8 value) {
	return cypress_tma340_write(client, addr, &value, sizeof(u8));
}

static int cypress_tma340_read(struct i2c_client * client, u8 addr, u8 *value, u8 len)
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

static inline int cypress_tma340_read_u8(struct i2c_client * client, u8 addr, u8 *value) {
	return cypress_tma340_read(client, addr, value, sizeof(u8));
}

static inline int cypress_tma340_read_u16(struct i2c_client * client, u8 addr, u16 *value)
{
	u8 buf[2];
	int ret;

	ret = cypress_tma340_read(client, addr, buf, sizeof(buf));

	*value = (buf[0] << 8) | buf[1];

	return ret;
}

static int cypress_tma340_write_cmd(struct i2c_client * client, u8 cmd)
{
	u8 stat;
	int retry = 5;

	cypress_tma340_write_u8(client, 0x02, cmd);

	do {
		msleep(10);

		cypress_tma340_read_u8(client, 0x01, &stat);

		dev_dbg(&client->dev, "%s: stat : 0x%02x\n",
				__FUNCTION__, stat);

		if (!retry--)
			break;

	} while (!(stat & (1 << 1)));

	return !(stat & (1 << 1));
}

static int cypress_tma340_write_boot_cmd(struct i2c_client * client, u8 cmd)
{
	struct boot_cmd {
		u8 file;
		u8 seed;
		u8 cmd;
		u8 key[8];
	} __attribute__ ((packed));

	struct boot_cmd buf = {
		.file = 0x00,
		.seed = 0xff,
		.cmd = cmd,
		.key = BYD_KEY,
	};

	return cypress_tma340_write(client, 0, (u8 *) &buf, sizeof(buf));
}

static irqreturn_t cypress_tma340_irq(int irq, void *p)
{
	struct cypress_tma340_priv *priv = p;
	
	int nu_point_state = 0;
	int i = 0;

	u8 b[0x20];

	if (cypress_tma340_read(priv->client, 0, b, sizeof(b)) != sizeof(b)) {
		dev_err(&priv->client->dev, "%s FAIL\n", __FUNCTION__);
		goto exit_work;
	}

	dev_dbg(&priv->client->dev,
			"hst_mode : %02x; mode : %02x; "
			"stat: %02x; id12 %02x ; id34 %02x\n",
			b[dop.hst_mode], b[dop.tt_mode], b[dop.tt_stat],
			b[dop.p[0].id], b[dop.p[2].id]);

	if (unlikely(b[dop.hst_mode] & (1 << 4))) {
		dev_info(&priv->client->dev,
				"%s: sysinfo mode, skip (%02x)\n",
				__FUNCTION__, b[dop.hst_mode]);
		goto exit_work;
	}

	if (unlikely(b[dop.tt_mode] & 0x20)) {
		dev_info(&priv->client->dev,
				"%s: invalid report, skip (%02x)\n",
				__FUNCTION__, b[dop.tt_mode]);
		goto exit_work;
	}

	for (i=0; i<priv->finger_max; i++) {
		int id = i&1 ? (b[dop.p[i].id] & 0x0f) : (b[dop.p[i].id] >> 4);

#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
		if (i)
			break;
#endif

		if (i < (b[dop.tt_stat] & 7)) {
			int x,y,p;

			if (priv->flags & CYPRESS_TMA340_FLAGS_XY_SWAP) {
				x = (b[dop.p[i].y_h] << 8) + b[dop.p[i].y_l];
				y = (b[dop.p[i].x_h] << 8) + b[dop.p[i].x_l];
			} else {
				x = (b[dop.p[i].x_h] << 8) + b[dop.p[i].x_l];
				y = (b[dop.p[i].y_h] << 8) + b[dop.p[i].y_l];
			}

			if (priv->flags & CYPRESS_TMA340_FLAGS_INV_X)
				x = priv->x_max - x;

			if (priv->flags & CYPRESS_TMA340_FLAGS_INV_Y)
				y = priv->y_max - y;

			p = b[dop.p[i].pressure];

#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
			input_report_abs(priv->input_dev, ABS_X, x);
			input_report_abs(priv->input_dev, ABS_Y, y);
			input_report_abs(priv->input_dev, ABS_PRESSURE, p);

			if (priv->down) {
				input_report_key(priv->input_dev, BTN_TOUCH, 1);
				priv->down=0;
			}
#else
			input_mt_slot(priv->input_dev, id - 1);
			input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(priv->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(priv->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(priv->input_dev, ABS_MT_TOUCH_MAJOR, p);
#endif

			nu_point_state |= (1 << id);
		}
	}

	for (i=0; i<16; i++) {
		if ((priv->point_state & (1 << i)) && !(nu_point_state & (1 << i))) {
#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
			input_report_abs(priv->input_dev, ABS_PRESSURE, 0);
			input_report_key(priv->input_dev, BTN_TOUCH, 0);
			priv->down=1;
#else
			input_mt_slot(priv->input_dev, i - 1);
			input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, false);
#endif
		}
	}

	cypress_tma340_write_u8(priv->client, dop.hst_mode, b[dop.hst_mode] ^ (1 << 7));

	priv->point_state = nu_point_state;

	input_sync(priv->input_dev);

exit_work:
	//enable_irq(priv->irq);
	return IRQ_HANDLED;
}

#ifdef BENCH
#define MS_TO_NS(x)	(x * 1E6L)

static enum hrtimer_restart irq_counter_func(struct hrtimer *timer)
{
	struct cypress_tma340_priv *priv =
		container_of(timer, struct cypress_tma340_priv, bench_timer);

	static int max = 0;

	if (priv->irq_count)
		dev_info(&priv->client->dev,
				"%s:%d / max %d\n",
				__FUNCTION__, priv->irq_count, max);

	if (priv->irq_count > max)
		max = priv->irq_count;

	priv->irq_count = 0;

	hrtimer_start(&priv->bench_timer, ktime_set( 0, MS_TO_NS(1000)), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}
#endif

static irqreturn_t cypress_tma340_hardirq(int irq, void * p)
{
	struct cypress_tma340_priv *priv = p;

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

static int cypress_tma340_set_mode(struct i2c_client *client, int mode)
{
	u8 stat, stat2;

	if (cypress_tma340_read_u8(client, dop.hst_mode, &stat) < 0)
		return -ENODEV;

	if (mode == (stat & 0x70) >> 4) {
		dev_dbg(&client->dev, "%s: %d : current %d : skip\n",
				__FUNCTION__,
				(stat & 0x70) >> 4,
				mode);
		return 0;
	}

	if (cypress_tma340_write_u8(client, dop.hst_mode,
				((mode << 4) & 0x70) | (1 << 7)) < 0)
		return -ENODEV;

	// XXX check if irq signaling works
	msleep(50);

	if (cypress_tma340_read_u8(client, dop.hst_mode, &stat2) < 0)
		return -ENODEV;

	dev_dbg(&client->dev, "mode: %d -> %d (0x%02x)\n",
			(stat & 0x70) >> 4, (stat2 & 0x70) >> 4, stat2);

	cypress_tma340_write_u8(client, dop.hst_mode, (stat2 & ~(1 << 7)));

	return 0;
}

static int cypress_tma340_read_version_info(struct i2c_client * client, struct tma340_version * v)
{
	u8 stat;

	cypress_tma340_read_u8(client, dbo.bl_status, &stat);

	if (stat & 0x10) {
		// boot ctx
		cypress_tma340_read_u16(client, dbo.bl_ver_hi, &v->boot_version);
		cypress_tma340_read_u16(client, dbo.tts_ver_hi, &v->tts_version);
		cypress_tma340_read_u16(client, dbo.app_id_hi, &v->app_id);
		cypress_tma340_read_u16(client, dbo.app_ver_hi, &v->app_version);
	} else {
		// app ctx : switch to sysinfo
		cypress_tma340_set_mode(client, 1);
		cypress_tma340_read_u16(client, dsys.bl_ver_hi, &v->boot_version);
		cypress_tma340_read_u16(client, dsys.tts_ver_hi, &v->tts_version);
		cypress_tma340_read_u16(client, dsys.app_id_hi, &v->app_id);
		cypress_tma340_read_u16(client, dsys.app_ver_hi, &v->app_version);
		cypress_tma340_set_mode(client, 0);
	}

	dev_info(&client->dev, "BOOT %04x / TTS %04x / APP %04x / ID %04x\n",
			v->boot_version, v->tts_version, v->app_version, v->app_id);

	return 0;
}

static int cypress_tma340_enter_bootloader(struct i2c_client *client)
{
	struct cypress_tma340_priv *priv = i2c_get_clientdata(client);
	int retry = 10;
	u8 stat;

	priv->state = ST_UNKNOWN;

	INIT_COMPLETION(priv->irq_trigged);

	cypress_tma340_write_u8(client, dop.hst_mode, 0x01);

	if (wait_for_completion_interruptible_timeout(
				&priv->irq_trigged,
				msecs_to_jiffies(500)) <= 0) {
		dev_err(&client->dev,
				"%s : missed irq pulse ?\n",
				__FUNCTION__);
		return -ENODEV;
	}

	while ((cypress_tma340_read_u8(client, dbo.bl_status, &stat) < 0) && (--retry)) {
		dev_info(&client->dev, "ping ? (%d)\n", retry);
		msleep(20);
	}

	if (!retry) {
		dev_err(&client->dev, "could not talk to tsp.)\n");
		return -ENODEV;
	}

	if (!(stat & 0x10)) {
		dev_err(&client->dev, "could not enter boot.)\n");
		priv->state = ST_RUNNING;
		return -ENODEV;
	}

	cypress_tma340_read_version_info(client, &priv->version);

	priv->state = ST_BOOTLOADER;

	return 0;
}

static int cypress_tma340_exit_bootloader(struct i2c_client *client)
{
	struct cypress_tma340_priv *priv = i2c_get_clientdata(client);
	int retry = 10;
	u8 stat;

	cypress_tma340_read_u8(client, dbo.bl_status, &stat);

	if (!(stat & 0x01)) {
		dev_err(&client->dev, "app wrong checksum. error 0x%02x\n", stat);
		goto boot_exit_fail;
	}

	cypress_tma340_write_boot_cmd(client, 0xa5);

	while ((cypress_tma340_read_u8(client, dbo.bl_status, &stat) < 0) && (--retry))
		msleep(20);

	if (!retry) {
		dev_err(&client->dev, "caramba, still in boot mode. error: 0x%02x)\n", stat);
		goto boot_exit_fail;
	}

	priv->state = ST_IDLING;

	return cypress_tma340_set_mode(client, 0);

boot_exit_fail:
	// stay as it, waiting for firmware fix.
	dev_info(&client->dev, "tsp stuck in boot mode.\n");
	return -EINVAL;;
}
static int cypress_tma340_pm_setup(struct i2c_client * client)
{
	u8 stat;

	if (cypress_tma340_read_u8(client, dop.hst_mode, &stat) < 0)
		return -ENODEV;

	if (stat & 0x04) {
		dev_dbg(&client->dev, "%s LP enabled : 0x%02x\n", __FUNCTION__, stat);
		cypress_tma340_write_u8(client, dop.hst_mode, stat & ~(0x04));
	}

	return 0;
}

static int cypress_tma340_startup_sequence(struct i2c_client *client)
{
	struct cypress_tma340_priv *priv = i2c_get_clientdata(client);
	int retry = 10;
	u8 stat;

	INIT_COMPLETION(priv->irq_trigged);

	priv->state = ST_UNKNOWN;

	cypress_tma340_power(client, 1);

	if (wait_for_completion_interruptible_timeout(
				&priv->irq_trigged,
				msecs_to_jiffies(900)) <= 0) {
		dev_err(&client->dev,
				"%s : missed irq pulse ?\n",
				__FUNCTION__);
		goto fail;
	}

	while ((cypress_tma340_read_u8(client, dop.hst_mode, &stat) < 0) && (retry)) {
		retry--;

		dev_info(&client->dev, "ping ? (%d)\n", retry);
		msleep(50);
	}

	if (!retry) {
		dev_err(&client->dev, "could not talk to tsp.)\n");
		goto fail;
	}

	cypress_tma340_read_version_info(client, &priv->version);

	if (!priv->version.app_version) {
		dev_err(&client->dev, "tsp in bad version mood, restart...)\n");
		goto fail;
	}

	// check if tsp is in bootloader mode.
	cypress_tma340_read_u8(client, dbo.bl_status, &stat);

	dev_info(&client->dev, "tsp woke up in %s mode.",
			(stat & 0x10) ? "bootloader" : "app");

	// and ajust state - we'll switch mode when needed.
	if (stat & 0x10)
		priv->state = ST_BOOTLOADER;
	else
		priv->state = ST_IDLING;

	return 0;
fail:
	cypress_tma340_power(client, 0);
	return -ENODEV;
}

static int cypress_tma340_check_boot_cmd(struct i2c_client *client)
{
	char bl_status[2] = {0,};
	int ret = 0;

	ret = cypress_tma340_read(client, 1, bl_status, 2);

	if (ret != sizeof(bl_status)) {
		dev_err(&client->dev, "get status failed - (0x%02x)\n", ret);
		return -EINVAL;
	}

	if ((bl_status[0] != 0x10) || (bl_status[1] != 0x20)) {
		dev_err(&client->dev, "write failed - 0x%02x/0x%02x\n",
				bl_status[0], bl_status[1]);
		return -EINVAL;
	}

	return 0;
}

static void cypress_tma340_trigger_calibration(struct i2c_client *client)
{
	int retry = 10;

	while (--retry) {
		u8 stat;

		cypress_tma340_enter_bootloader(client);
		cypress_tma340_exit_bootloader(client);

		msleep(100);

		cypress_tma340_write_u8(client, dop.magic_calib, 1);

		msleep(100);

		cypress_tma340_read_u8(client, dop.magic_calib, &stat);

		if (!stat)
			return;
	}

	return;
}

static int cypress_tma340_get_fw_version(const struct firmware *fw, struct tma340_version *v)
{
	const struct ihex_binrec * record;

	record = (const struct ihex_binrec *)fw->data;
	do {
		int block = be32_to_cpu(record->addr) / 64;
		int len = be16_to_cpu(record->len);

		if ((block == 30) && (len == 64)) {
			v->tts_version = (record->data[32] << 8) | record->data[33];
			v->app_id      = (record->data[34] << 8) | record->data[35];
			v->app_version = (record->data[36] << 8) | record->data[37];
			return 0;
		}
	} while ((record = ihex_next_binrec(record)));

	return -ENOENT;
}

static int cypress_tma340_fw_flash(struct i2c_client *client, struct firmware *fw)
{
	struct cypress_tma340_priv *priv = i2c_get_clientdata(client);
	const struct ihex_binrec * record;
	int block_cnt, block_current;
	int ret;

	// check firmware validify - we need an even number of valid records
	// since blocks are flashed by packet of two.
	record = (const struct ihex_binrec *)fw->data;
	block_cnt = 0;
	do {
		int block = be32_to_cpu(record->addr) / 64;
		int len = be16_to_cpu(record->len);

		// skip mysterious segments.
		if (block > 511)
			continue;

		// skip protected boot segment, but keep checksum blocks.
		if ((block < 44) && (block != 30) && (block != 31)) {
			continue;
		}

		if (len != 64) {
			dev_err(&client->dev,
					"%s: invalid block size (%d) at %d\n",
					__FUNCTION__, len, block_cnt);
			ret = -EINVAL;
			goto exit;
		}

		block_cnt++;

	} while ((record = ihex_next_binrec(record)));

	if (block_cnt % 2) {
		dev_err(&client->dev, "%s: invalid block count (%d)\n",
				__FUNCTION__, block_cnt);
		ret = -EINVAL;
		goto exit;
	}

	dev_info(&client->dev, "got valid firmware, %d bytes / %d blocks\n",
			fw->size, block_cnt);

	// start flash process.
	cypress_tma340_write_boot_cmd(client, 0x38);

	INIT_COMPLETION(priv->irq_trigged);
	// wait 12 secs - or better, an irq
	if (wait_for_completion_interruptible_timeout(
				&priv->irq_trigged,
				msecs_to_jiffies(12000)) <= 0) {
		dev_err(&client->dev, "missed update start irq pulse ?\n");
		ret = -ETIMEDOUT;
		goto exit;
	}

	if ((ret = cypress_tma340_check_boot_cmd(client)))
		goto exit;

	// start back
	record = (const struct ihex_binrec *)fw->data;
	block_current = 0;
	do {
		int block = be32_to_cpu(record->addr) / 64;
		int data_offset = 0;
		u8 sum_header = 0;
		u8 sum_payload = 0;
		int packet;

		if (block > 511)
			continue;

		if ((block < 44) && (block != 30) && (block != 31))
			continue;

		block_current++;

		priv->upgrade_state = (100 * block_current) / block_cnt;

		pr_debug("%s: writing block %d of %d : %d\n",
				__FUNCTION__, block_current, block_cnt,
				priv->upgrade_state);

		for (packet=0; packet < 5; packet++) {
			char *data = (char *)record->data;
			char buf[17] = {0,};
			int packet_len = sizeof(buf);
			int k;

			// fix packet offset
			buf[0] = packet * 0x10;

			switch (packet) {
			case 0:
				// build header
				buf[1] = 0xff;
				buf[2] = 0x39;
				memcpy(buf+3, key, 8);
				buf[11] = block >> 8;
				buf[12] = 0xff & block;

				// compute checksum for the header
				for (k = 1; k<13; k++)
					sum_header += buf[k];

				// push first 4 payload bytes
				memcpy(buf+13, &data[data_offset], 4);

				// compute payload checksum
				for (k = 0; k<4; k++, data_offset++)
					sum_payload += data[data_offset];

				break;

			default:
				// push next 16 payload bytes
				memcpy(buf+1, &data[data_offset], 16);

				// compute payload checksum
				for (k = 0; k<16; k++, data_offset++)
					sum_payload += data[data_offset];

				break;

			case 4:
				// push last 12 payload bytes
				memcpy(buf+1, &data[data_offset], 12);

				// compute end of payload checksum
				for (k = 0; k<12; k++, data_offset++)
					sum_payload += data[data_offset];

				// fix payload checksum
				buf[13] = sum_payload;
				// fix full packet checksum
				buf[14] = sum_header + sum_payload + buf[13];

				// fix specific shorter last packet size
				packet_len = 15;

				break;
			}

			// push it !
			INIT_COMPLETION(priv->irq_trigged);
			cypress_tma340_write(client, 0, buf, packet_len);
		}

		// stay quiet 20msec
		msleep(20);

		// wait - or better, wait for an irq.
		if (wait_for_completion_interruptible_timeout(
					&priv->irq_trigged,
					msecs_to_jiffies(200)) <= 0) {
			dev_err(&client->dev,
					"%s : missed write irq pulse ?\n",
					__FUNCTION__);
			// wait a bit more, and check status.
			msleep(100);
		}

		if ((ret = cypress_tma340_check_boot_cmd(client))) {
			dev_err(&client->dev, "write fail. (%d)\n", ret);
			goto exit;
		}

	} while ((record = ihex_next_binrec(record)));

	dev_info(&client->dev, "successfully pushed %d blocks\n", block_current);

	// tell tsp upgrade is complete.
	cypress_tma340_write_boot_cmd(client, 0x3b);

	// cheap reset.
	cypress_tma340_enter_bootloader(client);
	cypress_tma340_exit_bootloader(client);

	cypress_tma340_trigger_calibration(client);

	if (priv->state == ST_IDLING) {
		cypress_tma340_pm_setup(client);
		priv->state = ST_RUNNING;
	}


	return ret;

exit:
	priv->upgrade_state = ret;

	return ret;
}

void cypress_tma340_fw_upgrade_cb(const struct firmware *fw, void *context)
{
	struct i2c_client * client = (struct i2c_client *) context;
	struct cypress_tma340_priv *priv = i2c_get_clientdata(client);
	struct tma340_version v;
	struct mutex m;

	mutex_init(&m);

	if (!fw)
		return;

	if (priv->state == ST_SUSPEND)
		return;

	if (!mutex_trylock(&m))
		goto release;

	if (ihex_validate_fw(fw) < 0)
		goto abort;

	wake_lock(&priv->wakelock);

	cypress_tma340_read_version_info(client, &priv->version);

	if (priv->version.app_version == 0)
		goto abort;

	if ( cypress_tma340_get_fw_version(fw, &v) < 0) {
		dev_err(&client->dev, "%s: could not get firmware version",
				__FUNCTION__);
		goto abort;
	}

	if ((v.app_version == priv->version.app_version)) {
		priv->upgrade_state = 100;
		goto abort;
	}

	if (priv->state != ST_BOOTLOADER) {
		if (cypress_tma340_enter_bootloader(client) < 0)
			goto abort;
	}

	cypress_tma340_fw_flash(client, (struct firmware *) fw);

abort:
	wake_unlock(&priv->wakelock);

	mutex_unlock(&m);

release:
	release_firmware(fw);

	return;
}

static void cypress_tma340_fw_upgrade_nowait(struct i2c_client *client)
{
	request_firmware_nowait(
			THIS_MODULE, 1,
			BYD_FW_FILENAME,
			&client->dev,
			GFP_KERNEL,
			(void *) client,
			cypress_tma340_fw_upgrade_cb);
}

static ssize_t _show_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct cypress_tma340_priv *priv = i2c_get_clientdata(client);

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
	struct cypress_tma340_priv *priv = i2c_get_clientdata(client);

	if (priv->state != ST_RUNNING)
		return count;

	cypress_tma340_trigger_calibration(client);

	if (priv->state == ST_IDLING) {
		cypress_tma340_pm_setup(client);
		priv->state = ST_RUNNING;
	}

	return count;
}
static DEVICE_ATTR(calibration_trigger, S_IWUSR | S_IRUGO, NULL, _store_calibration);

static ssize_t _show_upgrade_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct cypress_tma340_priv *priv = i2c_get_clientdata(client);

	printk("%s:%d\n", __FUNCTION__, priv->upgrade_state);

	return sprintf(buf, "%d\n", priv->upgrade_state);
}
static DEVICE_ATTR(upgrade_state, S_IWUSR | S_IRUGO, _show_upgrade_state , NULL);

static ssize_t _show_upgrade(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct cypress_tma340_priv *priv = i2c_get_clientdata(client);
	return sprintf(buf, "0x%04x\n", priv->version.app_version);
}

static ssize_t _store_upgrade(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	cypress_tma340_fw_upgrade_nowait(client);
	return count;
}
static DEVICE_ATTR(upgrade_trigger, S_IWUSR | S_IRUGO, _show_upgrade, _store_upgrade);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_calibration_trigger.attr,
	&dev_attr_upgrade_state.attr,
	&dev_attr_upgrade_trigger.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = sysfs_attrs,
};

static int cypress_tma340_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cypress_tma340_platform_data *pdata = client->dev.platform_data;
	struct cypress_tma340_priv *priv;
	int retry;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);

	priv->client = client;

	// setup manually fallback calues to WA early tsps,
	// no able to switch to sysinfo mode.
	priv->finger_max = 2;
	priv->x_max = 2048;
	priv->y_max = 2048;

	if (pdata) {
		priv->irq = pdata->irq;
		priv->flags = pdata->flags;
	}

#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
	priv->down = 0;
#endif

	init_completion(&priv->irq_trigged);

	priv->regulator = regulator_get(&client->dev, "tsp_vcc");
	if (IS_ERR(priv->regulator)) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed to get regulator\n");
		goto err_regulator_get;
	}

	ret = request_threaded_irq(priv->irq, cypress_tma340_hardirq, cypress_tma340_irq,
			IRQF_TRIGGER_FALLING, client->name, priv);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_irq_request_failed;
	}
	enable_irq_wake(priv->irq);
	device_init_wakeup(&client->dev, true);
	
	retry = 5;
	do {
		msleep(10);
		ret = cypress_tma340_startup_sequence(client);

		if (ret >= 0)
			break;

		cypress_tma340_power(client, 0);
	} while (retry--);

	if (ret < 0) {
		dev_err(&client->dev, "could not detect tsp.\n");
		goto err_detect_failed;
	}

	// exit boot if needed.
	if (priv->state == ST_BOOTLOADER)
		cypress_tma340_exit_bootloader(client);

	if (priv->state == ST_BOOTLOADER) {
		dev_err(&client->dev, "probing tsp in bootloader mode...\n");
	} else {
		cypress_tma340_set_mode(client, 1);

		if (!cypress_tma340_write_cmd(client, 0x0f)) {
			cypress_tma340_read_u16(client, 0x05, &priv->x_max);
			cypress_tma340_read_u16(client, 0x07, &priv->y_max);
			cypress_tma340_read_u8(client, 0x09, &priv->finger_max);

			dev_info(&client->dev, "x_max : %d - y_max : %d - finger_max : %d\n",
					priv->x_max, priv->y_max, priv->finger_max);
		} else {
			dev_err(&client->dev, "failed to get tsp info.\n");
		}
		cypress_tma340_set_mode(client, 0);
	}

	priv->input_dev = input_allocate_device();
	if (priv->input_dev == NULL) {
		ret = -ENOMEM;
		goto err_input_alloc_failed;
	}

	priv->input_dev->name = id->name;

	set_bit(EV_SYN, priv->input_dev->evbit);
	set_bit(EV_ABS, priv->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, priv->input_dev->evbit);

#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
	set_bit(EV_KEY, priv->input_dev->evbit);
	set_bit(BTN_TOUCH, priv->input_dev->keybit);
#else
	set_bit(ABS_MT_POSITION_X, priv->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, priv->input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, priv->input_dev->absbit);
#endif

#ifdef CONFIG_TOUCHSCREEN_ARCHOS_TS_MONOTOUCH_COMPAT
	if (priv->flags & CYPRESS_TMA340_FLAGS_XY_SWAP) {
		input_set_abs_params(priv->input_dev, ABS_Y, 0, priv->x_max, 0, 0);
		input_set_abs_params(priv->input_dev, ABS_X, 0, priv->y_max, 0, 0);
	} else {
		input_set_abs_params(priv->input_dev, ABS_X, 0, priv->x_max, 0, 0);
		input_set_abs_params(priv->input_dev, ABS_Y, 0, priv->y_max, 0, 0);
	}
	input_set_abs_params(priv->input_dev, ABS_PRESSURE, 0, 0x80, 0, 0);
#else
	input_mt_init_slots(priv->input_dev, 16);

	if (priv->flags & CYPRESS_TMA340_FLAGS_XY_SWAP) {
		input_set_abs_params(priv->input_dev, ABS_MT_POSITION_Y, 0, priv->x_max, 0, 0);
		input_set_abs_params(priv->input_dev, ABS_MT_POSITION_X, 0, priv->y_max, 0, 0);
	} else {
		input_set_abs_params(priv->input_dev, ABS_MT_POSITION_X, 0, priv->x_max, 0, 0);
		input_set_abs_params(priv->input_dev, ABS_MT_POSITION_Y, 0, priv->y_max, 0, 0);
	}

	input_set_abs_params(priv->input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xff, 0, 0);
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
	priv->early_suspend.suspend = cypress_tma340_early_suspend;
	priv->early_suspend.resume = cypress_tma340_late_resume;
	register_early_suspend(&priv->early_suspend);

	wake_lock_init(&priv->wakelock, WAKE_LOCK_SUSPEND, "cypress-tma340");
#endif

#ifdef BENCH
	hrtimer_init(&priv->bench_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	priv->bench_timer.function = irq_counter_func;
	hrtimer_start(&priv->bench_timer, ktime_set( 0, MS_TO_NS(1000)), HRTIMER_MODE_REL);
	priv->irq_count = 0;
#endif

	// now, we can release irq handler to chat with tsp and process events.
	// note that we only do that if tsp is not in boot state..
	if (priv->state == ST_IDLING) {
		cypress_tma340_pm_setup(client);
		priv->state = ST_RUNNING;
	}

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_TMA340_AUTOLOAD_FIRMWARE
	cypress_tma340_fw_upgrade_nowait(client);
#endif

	return 0;

err_input_register_failed:
	input_free_device(priv->input_dev);

err_input_alloc_failed:
err_detect_failed:
	disable_irq_wake(priv->irq);
	free_irq(priv->irq, priv);
	cypress_tma340_power(client, 0);

err_irq_request_failed:
	regulator_put(priv->regulator);

err_regulator_get:
	kfree(priv);

	return ret;
}

static int cypress_tma340_remove(struct i2c_client *client)
{
	struct cypress_tma340_priv *priv = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &attr_group);

#ifdef BENCH
	hrtimer_cancel(&priv->bench_timer);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	wake_lock_destroy(&priv->wakelock);

	unregister_early_suspend(&priv->early_suspend);
#endif
	disable_irq_wake(priv->irq);
	free_irq(priv->irq, priv);

	input_unregister_device(priv->input_dev);

	cypress_tma340_power(client, 0);

	regulator_put(priv->regulator);

	kfree(priv);
	return 0;
}

static int cypress_tma340_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct cypress_tma340_priv *priv = i2c_get_clientdata(client);

	// we dont care of early suspend when in boot... we're most likely
	// upgrading tsp firware.
	if (priv->state == ST_BOOTLOADER)
		return 0;

	priv->state = ST_SUSPEND;

	disable_irq(priv->irq);

	cypress_tma340_power(client, 0);

	return 0;
}

static int cypress_tma340_resume(struct i2c_client *client)
{
	struct cypress_tma340_priv *priv = i2c_get_clientdata(client);
	int retry = 5;
	int ret;

	if (priv->state != ST_SUSPEND)
		return 0;

	enable_irq(priv->irq);

	// wake up device
	do {
		msleep(10);

		ret = cypress_tma340_startup_sequence(client);

		if (ret >= 0)
			break;

		cypress_tma340_power(client, 0);
	} while (--retry);

	if (ret < 0)
		dev_err(&client->dev, "%s: failed ?\n", __FUNCTION__);

	// exit boot if needed
	if (priv->state == ST_BOOTLOADER)
		cypress_tma340_exit_bootloader(client);

	// and start reporting asap, priv data are already ok.
	if (priv->state == ST_IDLING) {
		cypress_tma340_pm_setup(client);
		priv->state = ST_RUNNING;
	}

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cypress_tma340_early_suspend(struct early_suspend *h)
{
	struct cypress_tma340_priv *priv =
		container_of(h, struct cypress_tma340_priv, early_suspend);
	cypress_tma340_suspend(priv->client, PMSG_SUSPEND);
}

static void cypress_tma340_late_resume(struct early_suspend *h)
{
	struct cypress_tma340_priv *priv =
		container_of(h, struct cypress_tma340_priv, early_suspend);
	cypress_tma340_resume(priv->client);
}
#endif

static const struct i2c_device_id cypress_tma340_id[] = {
	{ CYPRESS_TMA340_NAME, 0 },
	{ }
};

static struct i2c_driver cypress_tma340_driver = {
	.probe		= cypress_tma340_probe,
	.remove		= cypress_tma340_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= cypress_tma340_suspend,
	.resume		= cypress_tma340_resume,
#endif
	.id_table	= cypress_tma340_id,
	.driver = {
		.name	= CYPRESS_TMA340_NAME,
	},
};

static int __init cypress_tma340_init(void)
{
	return i2c_add_driver(&cypress_tma340_driver);
}

static void __exit cypress_tma340_exit(void)
{
	i2c_del_driver(&cypress_tma340_driver);
}

module_init(cypress_tma340_init);
module_exit(cypress_tma340_exit);

MODULE_AUTHOR("Guillaume Revaillot <revaillot@archos.com>");
MODULE_DESCRIPTION("Cypress TMA340 TrueTouch touchscreen driver");
MODULE_LICENSE("GPL");
