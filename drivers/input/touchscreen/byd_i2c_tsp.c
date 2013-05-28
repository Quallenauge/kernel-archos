/*
 *    byd_i2c_tsp.c : 18/04/2011
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

// #define DEBUG
// #define CALIB_AT_PROBE

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
#include <linux/mutex.h>
#include <linux/cdev.h>

#include <asm/uaccess.h>

#include <linux/input/byd_i2c_tsp.h>

#define MAX_SLOT_NUM		10

enum {
	APP_MODE,
	BOOT_MODE
};

static const struct register_map{
	u8 touch;
	u8 buttons;
	u8 power_mode;
	u8 int_mode;
	u8 int_width;
	u8 firmware_version_base;
	u8 specop;
	u8 eeprom_read_addr;
} map_reg= {
	.touch = 0x00,
	.buttons = 0x01,
	.power_mode = 51,
	.int_mode = 52,
	.int_width = 53,
	.firmware_version_base = 64,
	.specop = 58,
	.eeprom_read_addr = 59,
};

enum {
	ST_IDLING = 0,		// std mode, not allowed to report events.
	ST_RUNNING = 1,		// std mode, reporting invents.
	ST_SUSPEND = 2,		// suspend mode, not powered.
	ST_BOOTLOADER = 3,	// bootloader mode.
	ST_MFG = 4,		// manuf mode.
	ST_UNKNOWN = 5,		// unconfigured mode
};

struct byd_priv {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct platform_device *pdev;

	struct regulator *regulator;

	void (*reset)(int);
	int (*get_irq_level)(void);

	int irq;

	// finger state
	struct point_node_t {
		unsigned char 	state ;
		unsigned char	finger_id;
		unsigned int	posx;
		unsigned int	posy;
	} point_slot[MAX_SLOT_NUM];

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

	dev_t devid;
	struct class *class;
	struct cdev cdev;
	int pre_mfg_state;
	int status_reg;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void byd_early_suspend(struct early_suspend *h);
static void byd_late_resume(struct early_suspend *h);
#endif

static int byd_cycle_and_startup_sequence(struct i2c_client *client, int mode, int retry);

static void byd_power(struct i2c_client *client, int on_off)
{
	struct byd_priv *priv = i2c_get_clientdata(client);
	static int state = -1;

	if (state == on_off)
		return;

	if (priv->reset)
		priv->reset(0);

	dev_dbg(&priv->client->dev, "%s %s\n",
			__FUNCTION__, on_off ? "on" : "off");

	state = on_off;

	if (on_off) {
		regulator_enable(priv->regulator);
	} else {
		regulator_disable(priv->regulator);
	}
}

static int byd_write(struct i2c_client * client, u8 addr, u8 *value, u8 len)
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

static inline int byd_write_u8(struct i2c_client * client, u8 addr, u8 value) {
	return byd_write(client, addr, &value, sizeof(u8));
}

static int byd_read(struct i2c_client * client, u8 addr, u8 *value, u8 len)
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

static inline int byd_read_u8(struct i2c_client * client, u8 addr, u8 *value) {
	return byd_read(client, addr, value, sizeof(u8));
}

static irqreturn_t byd_thread_fn(int irq, void *v)
{
	struct byd_priv *priv = v;

	unsigned char touch, pix_id, slot_id;
	unsigned char rdbuf[27];
	unsigned char *p = &rdbuf[2];
	int ret, i;

	ret = byd_read(priv->client, 0, rdbuf, sizeof(rdbuf));

	touch = rdbuf[0] & 0x07;

	pr_debug("%s:%02x\n", __func__, touch);

	for (i=0; i<touch; i++)	{
		pix_id = (*(p+4));
		slot_id = ((pix_id & 7)<<1) | ((pix_id & 8)>>3);
		priv->point_slot[slot_id].state = 1;
		priv->point_slot[slot_id].finger_id = pix_id;
		priv->point_slot[slot_id].posx = (*(p+1)<<8)+(*(p));
		priv->point_slot[slot_id].posy = (*(p+3)<<8)+(*(p+2));
		p+=5;
	}

	for (i=0; i < MAX_SLOT_NUM; i++) {

		switch (priv->point_slot[i].state) {
			// idle pointer, pass
			case 0:
				break;

			// new or refreshed pointer in slot, report
			case 1:
				input_mt_slot(priv->input_dev, i);
				input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, true);
				input_report_abs(priv->input_dev, ABS_MT_POSITION_X, priv->point_slot[i].posx);
				input_report_abs(priv->input_dev, ABS_MT_POSITION_Y, priv->point_slot[i].posy);

				pr_debug("slot=%d : %d x=%d,y=%d\n", i,
						priv->point_slot[i].state,
						priv->point_slot[i].posx,
						priv->point_slot[i].posy);

				priv->point_slot[i].state = 2;
				break;

			// pointer previously reported but not refreshed, release
			case 2:

				input_mt_slot(priv->input_dev, i);
				input_mt_report_slot_state(priv->input_dev, MT_TOOL_FINGER, false);

				pr_debug("slot=%d released\n", i);

				priv->point_slot[i].state = 0;
				break;
		}
	}

	input_sync(priv->input_dev);

	return IRQ_HANDLED;
}

static int byd_startup_sequence(struct i2c_client *client, int mode)
{
	struct byd_priv *priv = i2c_get_clientdata(client);
	u8 tsp_version[4];
	int retry = 6;
	int ret;

	byd_power(client, 1);

	if (priv->reset) {
		priv->reset(1);
		msleep(10);
		priv->reset(0);
	}

	msleep(300);

	while (((ret = byd_read(client, map_reg.firmware_version_base, tsp_version, sizeof(tsp_version))) < 0) && (retry)) {
		retry--;
		dev_info(&client->dev, "ping ? (%d)\n", retry);
		msleep(50);
	}

	dev_info(&client->dev, "M8 fw version %x:%x:%x - rev r%d\n",
				tsp_version[2], tsp_version[1], tsp_version[0], tsp_version[3]);

	if (!retry) {
		dev_info(&client->dev, "could not talk to tsp\n");
		goto fail;
	}

	byd_write_u8(client, map_reg.int_mode, 0x9);

	priv->state = ST_IDLING;

	return 0;
fail:
	byd_power(client, 0);
	return -ENODEV;
}

static int byd_cycle_and_startup_sequence(struct i2c_client *client, int mode, int retry)
{
	int ret;

	do {
		msleep(100);
		ret = byd_startup_sequence(client, mode);

		if (ret >= 0) {
			dev_dbg(&client->dev, "%s: tsp online.\n", __FUNCTION__);
			return 0;
		}

		msleep(100);
		byd_power(client, 0);
	} while (retry--);

	dev_err(&client->dev, "%s: failed, abort.\n", __FUNCTION__);

	return -ENODEV;
}

static ssize_t _show_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client * client = container_of(dev, struct i2c_client, dev);
	struct byd_priv *priv = i2c_get_clientdata(client);

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
	struct byd_priv *priv = i2c_get_clientdata(client);
	int timeout = 10;
	int retry = 2;

	int ret = count;

	u8 stat;

	if (priv->state != ST_RUNNING)
		return -ENODEV;

	priv->state = ST_IDLING;

	do {
		byd_power(client, 0);

		msleep(100);

		byd_startup_sequence(client, APP_MODE);

		msleep(100);

		byd_write_u8(client, 58, 0x03);

		do {
			if (byd_read_u8(client, 58, &stat) < 0) {
				ret = -ENODEV;
				goto error;
			}

			dev_info(&client->dev, "specop %x (%d...) \n", stat, timeout);

			msleep(1000);
			timeout--;

		} while ((stat != 0) && (timeout));

		if (stat == 0x00) {
			// ack from tsp
			break;
		}

		dev_err(&priv->client->dev, "%s retry (%d)\n", __FUNCTION__, retry);
	} while (--retry);

	if (!retry)
		ret = -EIO;

error:
	priv->state = ST_RUNNING;

	dev_info(&priv->client->dev, "%s : ok\n", __FUNCTION__);

	return ret;
}
static DEVICE_ATTR(calibration_trigger, S_IRUGO | S_IWUGO, NULL, _store_calibration);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_calibration_trigger.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = sysfs_attrs,
};

#define SLAVE_ADDR		0x5c
#define	BOOTLOADER_ADDR		0x5d

#ifndef I2C_MAJOR
#define I2C_MAJOR 		125
#endif

#define I2C_MINORS 		256

#define	CALIBRATION_FLAG	1
#define	BOOTLOADER		7
#define RESET_TP		9

#define	ENABLE_IRQ		10
#define	DISABLE_IRQ		11
#define	BOOTLOADER_STU		16
#define ATTB_VALUE		17

#define	MAX_FINGER_NUM		5

static int byd_fops_open(struct inode *inode, struct file *file)
{
	struct byd_priv *priv = container_of(inode->i_cdev, struct byd_priv, cdev);
	file->private_data = priv;

	priv->pre_mfg_state = priv->state;
	priv->state = ST_MFG;

	return 0;
}

static long byd_fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct byd_priv *priv = file->private_data;
	struct i2c_client *client = priv->client;
	static int irq_flag = 0;

	printk("byd_ioctl(),cmd = %d, arg = %ld\n", cmd, arg);

	switch (cmd)
	{
	case CALIBRATION_FLAG:	//CALIBRATION_FLAG = 1
		client->addr = SLAVE_ADDR;
		priv->status_reg = CALIBRATION_FLAG;
		break;

	case BOOTLOADER:	//BOOTLOADER = 7
		client->addr = BOOTLOADER_ADDR;
		priv->status_reg = BOOTLOADER;

		if (priv->reset) {
			priv->reset(1);
			msleep(10);
			priv->reset(0);
		}

		mdelay(5);
		break;

	case RESET_TP:		//RESET_TP = 9
		if (priv->reset) {
			priv->reset(1);
			msleep(10);
			priv->reset(0);
		}

		break;

	case ENABLE_IRQ:	//ENABLE_IRQ = 10
		priv->status_reg = 0;
		if(irq_flag == 0) {
			irq_flag = 1;
			enable_irq(priv->irq);
		}
		break;

	case DISABLE_IRQ:	//DISABLE_IRQ = 11
		if(irq_flag == 1) {
			irq_flag = 0;
			disable_irq_nosync(priv->irq);
		}
		break;

	case BOOTLOADER_STU:	//BOOTLOADER_STU = 16
		client->addr = BOOTLOADER_ADDR;
		priv->status_reg = BOOTLOADER_STU;

		if (priv->reset) {
			priv->reset(1);
			msleep(10);
			priv->reset(0);
		}

		mdelay(5);
		break;

	case ATTB_VALUE:	//ATTB_VALUE = 17
		client->addr = SLAVE_ADDR;
		priv->status_reg = ATTB_VALUE;
		break;

	default:
		client->addr = SLAVE_ADDR;
		priv->status_reg = 0;
		break;
	}
	return 0;
}

static ssize_t byd_fops_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	struct byd_priv *priv = file->private_data;
	struct i2c_client *client = priv->client;

	unsigned char *tmp, bootloader_stu[4], attb_value[1];
	int ret = 0;

	switch(priv->status_reg)
	{
	case BOOTLOADER_STU:
		i2c_master_recv(client, bootloader_stu, sizeof(bootloader_stu));
		if (ret!=sizeof(bootloader_stu)) {
			dev_err(&client->dev,
				"%s: BOOTLOADER_STU: i2c_master_recv() failed, ret=%d\n",
				__func__, ret);
			return -EFAULT;
		}

		if (copy_to_user(buf, bootloader_stu, sizeof(bootloader_stu))) {
			dev_err(&client->dev,
				"%s: BOOTLOADER_STU: copy_to_user() failed.\n",	__func__);
			return -EFAULT;
		} else {
			ret = 4;
		}
		break;

	case ATTB_VALUE:
		attb_value[0] = priv->get_irq_level();
		if(copy_to_user(buf, attb_value, sizeof(attb_value))) {
			dev_err(&client->dev,
				"%s: ATTB_VALUE: copy_to_user() failed.\n", __func__);
			return -EFAULT;
		} else {
			ret = 1;
		}
		break;

	default:
		tmp = kmalloc(count, GFP_KERNEL);
		if (tmp==NULL)
			return -ENOMEM;

		ret = i2c_master_recv(client, tmp, count);
		if (ret != count) {
			dev_err(&client->dev,
				"%s: default: i2c_master_recv() failed, ret=%d\n",
				__func__, ret);
			return -EFAULT;
		}

		if(copy_to_user(buf, tmp, count)) {
			dev_err(&client->dev,
				"%s: default: copy_to_user() failed.\n", __func__);
			kfree(tmp);
			return -EFAULT;
		}
		kfree(tmp);
		break;
	}
	return ret;
}

static ssize_t byd_fops_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct byd_priv *priv = file->private_data;
	struct i2c_client *client = priv->client;

	unsigned char *tmp, bootload_data[143];
	int ret = 0, time_out = 0;

	switch(priv->status_reg)
	{
	case CALIBRATION_FLAG:	//CALIBRATION_FLAG=1
		tmp = kmalloc(count, GFP_KERNEL);
		if (tmp == NULL)
			return -ENOMEM;

		if (copy_from_user(tmp, buf, count)) {
			dev_err(&client->dev,
				"%s: CALIBRATION_FLAG: copy_from_user() failed.\n", __func__);
			kfree(tmp);
			return -EFAULT;
		}

		ret = i2c_master_send(client, tmp, count);
		if (ret != count) {
			dev_err(&client->dev,
				"%s: CALIBRATION: i2c_master_send() failed, ret=%d\n",
				__func__, ret);
			kfree(tmp);
			return -EFAULT;
		}

		kfree(tmp);
		break;

	case BOOTLOADER:
		memset(bootload_data, 0, sizeof(bootload_data));

		if (copy_from_user(bootload_data, buf, count)) {
			dev_err(&client->dev,
				"%s: BOOTLOADER: copy_from_user() failed.\n", __func__);
			return -EFAULT;
		}
		time_out = 0;
		while (priv->get_irq_level()) {
			if(time_out > 100)
				break;
			else {
				time_out++;
				mdelay(1);
			}
		}
		ret = i2c_master_send(client, bootload_data, count);
		if (ret != count) {
			dev_err(&client->dev,
				"%s: BOOTLOADER: i2c_master_send() failed, ret = %d\n",
				__func__, ret);
			return -EFAULT;
		}
		time_out = 0;
		while (!priv->get_irq_level()) {
			if(time_out > 100)
				break;
			else {
				time_out++;
				mdelay(1);
			}
		}

		time_out = 0;
		while(priv->get_irq_level()) {
			if(time_out > 100)
				break;
			else {
				time_out++;
				mdelay(1);
			}
		}

		break;
	default:
		tmp = kmalloc(count, GFP_KERNEL);
		if (tmp == NULL)
			return -ENOMEM;

		if (copy_from_user(tmp, buf, count)) {
			dev_err(&client->dev,
				"%s: default: copy_from_user() failed.\n", __func__);
			kfree(tmp);
			return -EFAULT;
		}

		ret = i2c_master_send(client,tmp,count);
		if (ret != count) {
			dev_err(&client->dev,
				"%s: default: i2c_master_send() failed, ret=%d\n",
				__func__, ret);
			kfree(tmp);
			return -EFAULT;
		}
		kfree(tmp);
		break;
	}
	return ret;
}

static int byd_fops_release(struct inode *inode, struct file *file)
{
	struct byd_priv *priv = file->private_data;
	priv->state = priv->pre_mfg_state;;
	file->private_data = NULL;
	return 0;
}

static const struct file_operations byd_fops =
{	.owner		= THIS_MODULE,
	.open		= byd_fops_open,
	.unlocked_ioctl = byd_fops_ioctl,
	.read		= byd_fops_read,
	.write		= byd_fops_write,
	.release	= byd_fops_release,
};

int byd_register_cdev(struct i2c_client *client)
{
	struct byd_priv *priv = i2c_get_clientdata(client);
	struct device *dev;
	int ret;

	cdev_init(&priv->cdev, &byd_fops);
	priv->cdev.owner = THIS_MODULE;

	ret = alloc_chrdev_region(&priv->devid, 0, 1, "pixcir_i2c_ts");
	if (ret)
		return ret;

	ret = cdev_add(&priv->cdev, priv->devid, 1);
	if (ret)
		goto err_cdev_add;

	priv->class = class_create(THIS_MODULE, "pixcir_i2c_dev");
	if (IS_ERR(priv->class))
		goto err_class_create;

	if (IS_ERR(dev = device_create(priv->class, NULL, priv->devid, NULL,
			"pixcir_i2c_ts" "%d", MINOR(priv->devid)))) {
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

static void byd_unregister_cdev(struct i2c_client *client)
{
	struct byd_priv *priv = i2c_get_clientdata(client);

	device_destroy(priv->class, priv->devid);
	class_destroy(priv->class);
	unregister_chrdev_region(priv->devid, 1);
	cdev_del(&priv->cdev);
}

static int byd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct byd_platform_data *pdata = client->dev.platform_data;
	struct byd_priv *priv;

	int ret;

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

		priv->reset = pdata->reset;
		priv->get_irq_level= pdata->get_irq_level;
	} else {
		ret = -ENODEV;
		goto err_no_pdata;
	}

	priv->regulator = regulator_get(&client->dev, pdata->regulator);
	if (IS_ERR(priv->regulator)) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed to get regulator\n");
		goto err_regulator_get;
	}

	ret = byd_cycle_and_startup_sequence(priv->client, APP_MODE, 5);

	if (ret < 0) {
		ret = -ENODEV;
		dev_err(&client->dev, "could not detect tsp in app mode.\n");
		goto err_detect_failed;
	}

	ret = request_threaded_irq(priv->irq, NULL, byd_thread_fn,
			IRQF_TRIGGER_FALLING, client->name, priv);
	if (ret) {
		ret = -ENODEV;
		dev_err(&client->dev, "request_irq failed\n");
		goto err_irq_request_failed;
	}

	device_init_wakeup(&client->dev, true);

	dev_info(&client->dev, "x_min/max : %d/%d - y_min/max : %d/%d\n",
			priv->x_min, priv->x_max, priv->y_min, priv->y_max);

	priv->input_dev = input_allocate_device();
	if (priv->input_dev == NULL) {
		ret = -ENOMEM;
		goto err_input_alloc_failed;
	}

	priv->input_dev->name = id->name;

	set_bit(EV_ABS, priv->input_dev->evbit);
	set_bit(EV_SYN, priv->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, priv->input_dev->evbit);

	set_bit(ABS_MT_POSITION_X, priv->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, priv->input_dev->absbit);

	input_mt_init_slots(priv->input_dev, MAX_SLOT_NUM);

	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_X, priv->x_min, priv->x_max, 0, 0);
	input_set_abs_params(priv->input_dev, ABS_MT_POSITION_Y, priv->y_min, priv->y_max, 0, 0);

	ret = input_register_device(priv->input_dev);

	if (ret) {
		ret = -ENODEV;
		dev_err(&client->dev, "%s: Unable to register %s input device\n",
				__FUNCTION__, priv->input_dev->name);
		goto err_input_register_failed;
	}

	ret = sysfs_create_group(&client->dev.kobj, &attr_group);

	priv->pdev = platform_device_register_simple("archos_touchscreen", -1, NULL, 0);
	if (IS_ERR(priv->pdev))
		return ret;

	ret = sysfs_create_link(&priv->pdev->dev.kobj, &client->dev.kobj, "tsp");


#ifdef CONFIG_HAS_EARLYSUSPEND
	priv->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	priv->early_suspend.suspend = byd_early_suspend;
	priv->early_suspend.resume = byd_late_resume;

	register_early_suspend(&priv->early_suspend);
#endif

#ifdef CALIB_AT_PROBE
	{
		int timeout = 5;
		u8 stat;

		byd_write_u8(client, 58, 0x03);

		do {
			if (byd_read_u8(client, 58, &stat) < 0) {
				ret = -ENODEV;
				break;
			}

			dev_info(&client->dev, "specop %x (%d...) \n", stat, timeout);

			msleep(1000);
			timeout--;

		} while ((stat != 0) && (timeout));
	}
#endif

	enable_irq_wake(priv->irq);
	device_init_wakeup(&client->dev, true);


	byd_register_cdev(client);

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;

err_input_register_failed:
	input_free_device(priv->input_dev);

err_input_alloc_failed:
	disable_irq_wake(priv->irq);
	free_irq(priv->irq, priv);

	byd_power(client, 0);

err_irq_request_failed:

err_detect_failed:
	regulator_put(priv->regulator);

err_regulator_get:
err_no_pdata:
	kfree(priv);

	return ret;
}

static int byd_remove(struct i2c_client *client)
{
	struct byd_priv *priv = i2c_get_clientdata(client);

	byd_unregister_cdev(client);

	sysfs_remove_link(&priv->pdev->dev.kobj, "tsp");

	platform_device_unregister(priv->pdev);

	sysfs_remove_group(&client->dev.kobj, &attr_group);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&priv->early_suspend);
#endif
	disable_irq_wake(priv->irq);
	free_irq(priv->irq, priv);

	input_unregister_device(priv->input_dev);

	byd_power(client, 0);

	regulator_put(priv->regulator);

	kfree(priv);
	return 0;
}

static int byd_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct byd_priv *priv = i2c_get_clientdata(client);

	priv->state = ST_SUSPEND;

	disable_irq_wake(priv->irq);
	disable_irq(priv->irq);

	byd_power(client, 0);

	return 0;
}

static int byd_resume(struct i2c_client *client)
{
	struct byd_priv *priv = i2c_get_clientdata(client);

	enable_irq(priv->irq);
	enable_irq_wake(priv->irq);

	if (byd_cycle_and_startup_sequence(priv->client, APP_MODE, 5) < 0)
		dev_err(&client->dev, "%s: failed ?\n", __FUNCTION__);

	if (priv->state == ST_IDLING)
		priv->state = ST_RUNNING;

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void byd_early_suspend(struct early_suspend *h)
{
	struct byd_priv *priv =
		container_of(h, struct byd_priv, early_suspend);
	byd_suspend(priv->client, PMSG_SUSPEND);
}

static void byd_late_resume(struct early_suspend *h)
{
	struct byd_priv *priv =
		container_of(h, struct byd_priv, early_suspend);
	byd_resume(priv->client);
}
#endif

static const struct i2c_device_id byd_id[] = {
	{ BYD_NAME, 0 },
	{ }
};

static struct i2c_driver byd_driver = {
	.probe		= byd_probe,
	.remove		= byd_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= byd_suspend,
	.resume		= byd_resume,
#endif
	.id_table	= byd_id,
	.driver = {
		.name	= BYD_NAME,
	},
};

static int __init byd_init(void)
{
	return i2c_add_driver(&byd_driver);
}

static void __exit byd_exit(void)
{
	i2c_del_driver(&byd_driver);
}

module_init(byd_init);
module_exit(byd_exit);

MODULE_AUTHOR("Y Robic <robic@archos.com>");
MODULE_DESCRIPTION("byd i2c touchscreen driver");
MODULE_LICENSE("GPL");

