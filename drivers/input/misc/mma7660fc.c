/*
 *   MMA7660FC Accelerometer driver
 *
 *   Copyright (c) by Jean-Christophe Rona <rona@archos.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/mma7660fc.h>
#include <linux/earlysuspend.h>
#include <linux/regulator/machine.h>

#undef FILTER_ENABLED
#undef DEBUG_REPORT
#undef DEBUG_IOCTL
#undef DEBUG_IRQ
#undef DEBUG_REG

//#define DEBUG_REPORT
#define DEBUG_IOCTL
//#define DEBUG_IRQ
//#define DEBUG_REG

#define MMA7660FC_VERSION		"0.1"
#define MMA7660FC_DATE			"19 January 2010"

/*
 * If the polling delay is bigger that this threshold,
 * it means that the accuracy does not really matter.
 * So no needed to double the HW samplerate,
 * and save some mA instead.
 */
#define AVERAGE_DELAY_THRESHOLD		500
#define SAMPLES_TO_AVERAGE		1

#define SAMPLES_TO_SKIP			1

//#define FILTER_ENABLED
#define FILTER_DEPTH			5
#define FILTER_DENOMINATOR		FILTER_DEPTH
#define FILER_DELAY_THRESHOLD		100

static struct regulator *accel_1v8;
static struct regulator *accel_vcc;

static inline int mma7660fc_write(struct i2c_client *client, int reg, int value);
static inline int mma7660fc_read(struct i2c_client *client, int reg);

struct mma7660fc_data {
	struct input_dev *input_dev;
	struct delayed_work polling_work;
	struct work_struct irq_work;
	struct mma7660fc_pdata *pdata;
	/* Delay in ms between two acceleration reports */
	unsigned long poll_delay;
	int opencount;
	struct early_suspend early_suspend;
	unsigned standby:1;
	int sample_number;
	struct accel_data tmp_values;
	int samples_to_skip;
#ifdef FILTER_ENABLED
	struct accel_data circular_buffer[FILTER_DEPTH];
	unsigned int circular_index;
#endif
};

/* I2C client handle */
static struct i2c_client *this_client;

/* Filter coefficients */
static int filter_coeff[FILTER_DEPTH] = {1, 1, 1, 1, 1};

/*
 * MMA7660FC stuff
 */

static int mma7660fc_set_mode(short mode)
{
	int ret = 0;
	u8 data;
	
	data = mma7660fc_read(this_client, REG_MODE) & ~MMA7660FC_MODE_MASK;
	ret = mma7660fc_write(this_client, REG_MODE, data |
				(mode & MMA7660FC_MODE_MASK));
	return ret;
}

static short mma7660fc_get_mode(void)
{
	u8 data;
	
	data = mma7660fc_read(this_client, REG_MODE) & MMA7660FC_MODE_MASK;
	return data;
}

static int mma7660fc_set_pulse_debounce(short samples)
{
	int ret = 0;
	u8 data;
	
	data = mma7660fc_read(this_client, REG_PDET) & ~MMA7660FC_PULSE_DEBOUNCE_MASK;
	ret = mma7660fc_write(this_client, REG_PDET, data |
				((samples - 1)& MMA7660FC_PULSE_DEBOUNCE_MASK));
	return ret;
}

static short mma7660fc_get_pulse_debounce(void)
{
	u8 data;
	
	data = mma7660fc_read(this_client, REG_PDET) & MMA7660FC_PULSE_DEBOUNCE_MASK;
	return data;
}

static int mma7660fc_set_pulse_threshold(short trshld)
{
	int ret = 0;
	u8 data;
	
	data = mma7660fc_read(this_client, REG_PD) & ~MMA7660FC_PULSE_THRESHOLD_MASK;
	ret = mma7660fc_write(this_client, REG_PD, data |
				(trshld & MMA7660FC_PULSE_THRESHOLD_MASK));
	return ret;
}

static short mma7660fc_get_pulse_threshold(void)
{
	u8 data;
	
	data = mma7660fc_read(this_client, REG_PD) & MMA7660FC_PULSE_THRESHOLD_MASK;
	return data;
}

static int mma7660fc_set_pulse_axis(short axis)
{
	int ret = 0;
	u8 data;
	
	data = mma7660fc_read(this_client, REG_PD) & ~MMA7660FC_PULSE_AXIS_MASK;
	ret = mma7660fc_write(this_client, REG_PD, data |
				(axis & MMA7660FC_PULSE_AXIS_MASK));
	return ret;
}

static short mma7660fc_get_pulse_axis(void)
{
	u8 data;
	
	data = mma7660fc_read(this_client, REG_PD) & MMA7660FC_PULSE_AXIS_MASK;
	return data;
}

static int mma7660fc_interrupt_setup(short events)
{
	int ret = 0;
	u8 data;
	
	data = mma7660fc_read(this_client, REG_INTSU) & ~MMA7660FC_INT_SETUP_MASK;
	ret = mma7660fc_write(this_client, REG_INTSU, events |
				(events & MMA7660FC_INT_SETUP_MASK));
	return ret;
}

static short mma7660fc_get_interrupt_list(void)
{
	u8 data;
	
	data = mma7660fc_read(this_client, REG_INTSU) & MMA7660FC_INT_SETUP_MASK;
	return data;
}

static short mma7660fc_get_tilt_status(void)
{
	u8 data;
	
	while ((data = mma7660fc_read(this_client, REG_TILT) & MMA7660FC_TILT_STATUS_MASK) & MMA7660FC_INVALID_FLAG);
	return data;
}

static int mma7660fc_set_sleep_count(short count)
{
	int ret = 0;
	u8 data;
	
	data = mma7660fc_read(this_client, REG_SPCNT) & ~MMA7660FC_SLEEP_COUNT_MASK;
	ret = mma7660fc_write(this_client, REG_SPCNT, data |
				(count & MMA7660FC_SLEEP_COUNT_MASK));
	return ret;
}

static short mma7660fc_get_sleep_count(void)
{
	u8 data;
	
	data = mma7660fc_read(this_client, REG_SPCNT) & MMA7660FC_SLEEP_COUNT_MASK;
	return data;
}

static int mma7660fc_set_auto_wake_samplerate(short samplerate)
{
	int ret = 0;
	u8 data;
	
	data = mma7660fc_read(this_client, REG_SR) & ~MMA7660FC_SR_AUTO_WAKE_MASK;
	ret = mma7660fc_write(this_client, REG_SR, data |
				(samplerate & MMA7660FC_SR_AUTO_WAKE_MASK));
	return ret;
}

static int mma7660fc_set_auto_wake_delay(int delay)
{
	if (delay > 999) {
		return mma7660fc_set_auto_wake_samplerate(MMA7660FC_SR_AUTO_WAKE_1);
	} else if (delay > 499) {
		return mma7660fc_set_auto_wake_samplerate(MMA7660FC_SR_AUTO_WAKE_2);
	} else if (delay > 249) {
		return mma7660fc_set_auto_wake_samplerate(MMA7660FC_SR_AUTO_WAKE_4);
	} else if (delay > 124) {
		return mma7660fc_set_auto_wake_samplerate(MMA7660FC_SR_AUTO_WAKE_8);
	} else if (delay > 62) {
		return mma7660fc_set_auto_wake_samplerate(MMA7660FC_SR_AUTO_WAKE_16);
	} else if (delay > 31) {
		return mma7660fc_set_auto_wake_samplerate(MMA7660FC_SR_AUTO_WAKE_32);
	} else if (delay > 15) {
		return mma7660fc_set_auto_wake_samplerate(MMA7660FC_SR_AUTO_WAKE_64);
	} else
		return mma7660fc_set_auto_wake_samplerate(MMA7660FC_SR_AUTO_WAKE_120);
}

static short mma7660fc_get_auto_wake_samplerate(void)
{
	u8 data;
	
	data = mma7660fc_read(this_client, REG_SR) & MMA7660FC_SR_AUTO_WAKE_MASK;
	return data;
}

static int mma7660fc_set_auto_sleep_samplerate(short samplerate)
{
	int ret = 0;
	u8 data;
	
	data = mma7660fc_read(this_client, REG_SR) & ~MMA7660FC_SR_AUTO_SLEEP_MASK;
	ret = mma7660fc_write(this_client, REG_SR, data |
				(samplerate & MMA7660FC_SR_AUTO_SLEEP_MASK));
	return ret;
}

static short mma7660fc_get_auto_sleep_samplerate(void)
{
	u8 data;
	
	data = mma7660fc_read(this_client, REG_SR) & MMA7660FC_SR_AUTO_SLEEP_MASK;
	return data;
}

static int mma7660fc_set_sample_debounce(short samples)
{
	int ret = 0;
	u8 data;
	
	data = mma7660fc_read(this_client, REG_SR) & ~MMA7660FC_SR_DEBOUNCE_MASK;
	ret = mma7660fc_write(this_client, REG_SR, data |
				(((samples - 1) << MMA7660FC_SR_DEBOUNCE_SHIFT) & MMA7660FC_SR_DEBOUNCE_MASK));
	return ret;
}

static short mma7660fc_get_sample_debounce(void)
{
	u8 data;
	
	data = mma7660fc_read(this_client, REG_SR) & MMA7660FC_SR_DEBOUNCE_MASK;
	return (data >> MMA7660FC_SR_DEBOUNCE_SHIFT);
}

static int mma7660fc_enable_auto_wake(int on)
{
	int ret = 0;
	u8 data;

	if (!on)
		data = mma7660fc_read(this_client, REG_MODE) | MMA7660FC_MODE_AUTO_WAKE;
	else
		data = mma7660fc_read(this_client, REG_MODE) & ~MMA7660FC_MODE_AUTO_WAKE;

	ret = mma7660fc_write(this_client, REG_MODE, data);
	return ret;
}

static int mma7660fc_enable_auto_sleep(int on)
{
	int ret = 0;
	u8 data;

	if (!on)
		data = mma7660fc_read(this_client, REG_MODE) | MMA7660FC_MODE_AUTO_SLEEP;
	else
		data = mma7660fc_read(this_client, REG_MODE) & ~MMA7660FC_MODE_AUTO_SLEEP;

	ret = mma7660fc_write(this_client, REG_MODE, data);
	return ret;
}

static int mma7660fc_set_prescaler(int divisor)
{
	int ret = 0;
	u8 data;

	if (!divisor)
		data = mma7660fc_read(this_client, REG_MODE) | MMA7660FC_MODE_PRESCALER;
	else
		data = mma7660fc_read(this_client, REG_MODE) & ~MMA7660FC_MODE_PRESCALER;

	ret = mma7660fc_write(this_client, REG_MODE, data);
	return ret;
}

static int mma7660fc_set_int_pin_type(int type)
{
	int ret = 0;
	u8 data;

	if (!type)
		data = mma7660fc_read(this_client, REG_MODE) | MMA7660FC_MODE_INT_TYPE;
	else
		data = mma7660fc_read(this_client, REG_MODE) & ~MMA7660FC_MODE_INT_TYPE;

	ret = mma7660fc_write(this_client, REG_MODE, data);
	return ret;
}

static int mma7660fc_set_int_pin_active_level(int level)
{
	int ret = 0;
	u8 data;

	if (!level)
		data = mma7660fc_read(this_client, REG_MODE) | MMA7660FC_MODE_INT_ACTIVE_LEVEL;
	else
		data = mma7660fc_read(this_client, REG_MODE) & ~MMA7660FC_MODE_INT_ACTIVE_LEVEL;

	ret = mma7660fc_write(this_client, REG_MODE, data);
	return ret;
}

static void mma7660fc_get_values(struct accel_data * values)
{
	unsigned char tmp;

	if (!values) {
		printk("mma7660fc_get_values: values = NULL !!");
		return;
	}

	/* x, y and z are 6 bits signed values */
	/* Wait for the data to be ready */
	while ((tmp = mma7660fc_read(this_client, REG_XOUT)) & MMA7660FC_INVALID_FLAG);
	values->x = ((signed char) (tmp << 2))/4;

	/* Wait for the data to be ready */
	while ((tmp = mma7660fc_read(this_client, REG_YOUT)) & MMA7660FC_INVALID_FLAG);
	values->y = ((signed char) (tmp << 2))/4;

	/* Wait for the data to be ready */
	while ((tmp = mma7660fc_read(this_client, REG_ZOUT)) & MMA7660FC_INVALID_FLAG);
	values->z = ((signed char) (tmp << 2))/4;
}

static void mma7660fc_report_values(void)
{
	struct accel_data values;
	struct mma7660fc_data *data = i2c_get_clientdata(this_client);
	int i;

	mma7660fc_get_values(&values);

	/* First samples are null (usually 1) and need to be skipped */
	if(data->samples_to_skip > 0) {
		data->samples_to_skip--;
		return;
	}

	data->tmp_values.x += values.x;
	data->tmp_values.y += values.y;
	data->tmp_values.z += values.z;
	data->sample_number++;

	/* Do we have enough samples ? */
	if(data->sample_number < SAMPLES_TO_AVERAGE && data->poll_delay < AVERAGE_DELAY_THRESHOLD)
		return;

#ifdef FILTER_ENABLED
	memcpy(&data->circular_buffer[data->circular_index], &data->tmp_values, sizeof(struct accel_data));

	data->circular_index = (data->circular_index + 1) % FILTER_DEPTH;

	if (data->poll_delay < FILER_DELAY_THRESHOLD) {
		values.x = 0;
		values.y = 0;
		values.z = 0;

		for (i = 0; i < FILTER_DEPTH; i++) {
			values.x += data->circular_buffer[i].x * filter_coeff[(FILTER_DEPTH + data->circular_index - i) % FILTER_DEPTH];
			values.y += data->circular_buffer[i].y * filter_coeff[(FILTER_DEPTH + data->circular_index - i) % FILTER_DEPTH];
			values.z += data->circular_buffer[i].z * filter_coeff[(FILTER_DEPTH + data->circular_index - i) % FILTER_DEPTH];
		}
	
		values.x /= FILTER_DENOMINATOR * data->sample_number;
		values.y /= FILTER_DENOMINATOR * data->sample_number;
		values.z /= FILTER_DENOMINATOR * data->sample_number;
	} else
#endif
	{
		values.x = data->tmp_values.x/data->sample_number;
		values.y = data->tmp_values.y/data->sample_number;
		values.z = data->tmp_values.z/data->sample_number;
	}

#ifdef DEBUG_REPORT
	printk("mma7660fc_report_values: X = %d, Y = %d, Z = %d\n", values.x, values.y, values.z);
#endif
	input_report_abs(data->input_dev, ABS_X, (short) values.x);
	input_report_abs(data->input_dev, ABS_Y, (short) values.y);
	input_report_abs(data->input_dev, ABS_Z, (short) values.z);

	data->tmp_values.x = 0;
	data->tmp_values.y = 0;
	data->tmp_values.z = 0;
	data->sample_number = 0;

	input_sync(data->input_dev);
	return;
}

static void mma7660fc_general_purpose_event(int code)
{
	struct mma7660fc_data *data = i2c_get_clientdata(this_client);

	input_report_abs(data->input_dev, ABS_TILT_X, code);
	return;
}

static int mma7660fc_configure(struct mma7660fc_data *data)
{
	int ret = 0;
	
	/* Set Standby mode, Measurement mode will
		be set when the device will be open */
	ret = mma7660fc_set_mode(MMA7660FC_MODE_STANDBY);
	if (ret) {
		printk(KERN_ERR "mma7660fc_configure: mma7660fc_set_mode failed\n");
		return ret;
	}
	data->standby = 1;

	/* TODO: Decide what kind of event may be useful for us */
	/* We don't want anything to trigger an interrupt for now */
	ret = mma7660fc_interrupt_setup(MMA7660FC_INT_NONE);
	if (ret) {
		printk(KERN_ERR "mma7660fc_configure: mma7660fc_interrupt_setup failed\n");
		return ret;
	}

	/* Pulse (tap) disabled for X, Y and Z */
	ret = mma7660fc_set_pulse_axis(MMA7660FC_PULSE_AXIS_NONE);
	if (ret) {
		printk(KERN_ERR "mma7660fc_configure: mma7660fc_set_pulse_enabled_axis failed\n");
		return ret;
	}

	/* Arbitrary set pulse threshold to 31 */
	ret = mma7660fc_set_pulse_threshold(31);
	if (ret) {
		printk(KERN_ERR
			"mma7660fc_configure: mma7660fc_set_pulse_threshold failed\n");
		return ret;
	}

	/* Arbitrary set pulse debounce to 10 samples */
	ret = mma7660fc_set_pulse_debounce(10);
	if (ret) {
		printk(KERN_ERR
			"mma7660fc_configure: mma7660fc_set_pulse_debounce failed\n");
		return ret;
	}

	/* Sleep count to 0 */
	ret = mma7660fc_set_sleep_count(0);
	if (ret) {
		printk(KERN_ERR
			"mma7660fc_configure: mma7660fc_set_sleep_count failed\n");
		return ret;
	}

	/* Max samplerate in wake mode */
	ret = mma7660fc_set_auto_wake_samplerate(MMA7660FC_SR_AUTO_WAKE_120);
	if (ret) {
		printk(KERN_ERR
			"mma7660fc_configure: mma7660fc_set_auto_wake_samplerate failed\n");
		return ret;
	}

	/* Max samplerate in sleep mode */
	ret = mma7660fc_set_auto_sleep_samplerate(MMA7660FC_SR_AUTO_SLEEP_32);
	if (ret) {
		printk(KERN_ERR
			"mma7660fc_configure: mma7660fc_set_auto_sleep_samplerate failed\n");
		return ret;
	}

	/* No debounce for portrait/landscape detection */
	ret = mma7660fc_set_sample_debounce(1);
	if (ret) {
		printk(KERN_ERR
			"mma7660fc_configure: mma7660fc_set_sample_debounce failed\n");
		return ret;
	}

	/* TODO: power consumption improvement using sleep/wake mode */
	/* Disable auto wake for now */
	ret = mma7660fc_enable_auto_wake(OFF);
	if (ret) {
		printk(KERN_ERR
			"mma7660fc_configure: mma7660fc_enable_auto_wake failed\n");
		return ret;
	}

	/* Disable auto sleep for now */
	ret = mma7660fc_enable_auto_sleep(OFF);
	if (ret) {
		printk(KERN_ERR
			"mma7660fc_configure: mma7660fc_enable_auto_sleep failed\n");
		return ret;
	}

	/* No prescaler */
	ret = mma7660fc_set_prescaler(DIVIDE_BY_1);
	if (ret) {
		printk(KERN_ERR
			"mma7660fc_configure: mma7660fc_set_prescaler failed\n");
		return ret;
	}

	/* INT is open-drain */
	ret = mma7660fc_set_int_pin_type(OPEN_DRAIN);
	if (ret) {
		printk(KERN_ERR
			"mma7660fc_configure: mma7660fc_set_int_pin_type failed\n");
		return ret;
	}

	/* ~INT is active low, so INT is active high */
	ret = mma7660fc_set_int_pin_active_level(ACTIVE_LOW);
	if (ret) {
		printk(KERN_ERR
			"mma7660fc_configure: mma7660fc_set_int_pin_active_level failed\n");
		return ret;
	}

	return 0;
}

static int mma7660fc_open(struct input_dev *dev)
{
	int ret = 0;
	struct mma7660fc_data *data = i2c_get_clientdata(this_client);

	data->opencount++;
	if (data->opencount > 1)
		return 0;
	
	data->samples_to_skip = SAMPLES_TO_SKIP;

	/* Set Measurement mode */
	ret = mma7660fc_set_mode(MMA7660FC_MODE_MEASURE);
	if (ret) {
		printk(KERN_ERR
			"mma7660fc_open: mma7660fc_set_mode failed\n");
		return ret;
	}
	data->standby = 0;

	/* Everything is OK, enabling polling (if needed) */
	if (data->poll_delay > 0)
		schedule_delayed_work(&data->polling_work,0);
	
	return ret;
}

static void mma7660fc_close(struct input_dev *dev)
{
	struct mma7660fc_data *data = i2c_get_clientdata(this_client);

	data->opencount--;
	if (data->opencount > 0)
		return;
	
	/* Cancel remaining work */
	cancel_delayed_work_sync(&data->polling_work);
	/* Put the chip in Standby mode */
	mma7660fc_set_mode(MMA7660FC_MODE_STANDBY);
	data->standby = 1;
	return;
}

static void mma7660fc_polling_work_func(struct work_struct *work)
{
	struct mma7660fc_data *data = i2c_get_clientdata(this_client);

	/* Schedule the next work in data->poll_delay ms */
	if (data->poll_delay < AVERAGE_DELAY_THRESHOLD)
		schedule_delayed_work(&data->polling_work, (HZ * data->poll_delay) / (SAMPLES_TO_AVERAGE * 1000));
	else
		schedule_delayed_work(&data->polling_work, (HZ * data->poll_delay) / 1000);

	mma7660fc_report_values();
}

static void mma7660fc_irq_work_func(struct work_struct *work)
{
	struct mma7660fc_data *data = i2c_get_clientdata(this_client);

	input_report_abs(data->input_dev, ABS_MISC, mma7660fc_get_tilt_status());
//	input_sync(data->input_dev);
}

static irqreturn_t mma7660fc_isr(int irq, void *dev_id)
{
	struct mma7660fc_data *data = i2c_get_clientdata(this_client);

#ifdef DEBUG_IRQ
	printk("mma7660fc_isr: got irq %d\n", irq);
#endif

	schedule_work(&data->irq_work);

	return IRQ_HANDLED;
}

static void mma7456_early_suspend(struct early_suspend *h)
{
	struct mma7660fc_data *data = 
		container_of(h, struct mma7660fc_data, early_suspend);
	
	if (data->standby)
		return;
	
	if (data->poll_delay > 0)
		cancel_delayed_work(&data->polling_work);
	mma7660fc_set_mode(MMA7660FC_MODE_STANDBY);
	data->standby = 1;
}

static void mma7456_early_resume(struct early_suspend *h)
{
	struct mma7660fc_data *data = 
		container_of(h, struct mma7660fc_data, early_suspend);

	if (!data->standby)
		return;
	
	data->samples_to_skip = SAMPLES_TO_SKIP;
	mma7660fc_set_mode(MMA7660FC_MODE_MEASURE);
	data->standby = 0;
	if (data->poll_delay > 0)
		schedule_delayed_work(&data->polling_work,0);
}

/*
 * Control device stuff
 */

static int mma7660fc_ctrl_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int mma7660fc_ctrl_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long
mma7660fc_ctrl_ioctl(struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	struct mma7660fc_data *data = i2c_get_clientdata(this_client);
	void __user *argp = (void __user *)arg;
	short short_data;
	struct accel_data accel_values;
	int int_data;

	switch (cmd) {
	case MMA7660FC_IOCTL_S_MODE:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
		switch(short_data) {
#ifdef DEBUG_IOCTL
		printk("mma7660fc_ctrl_ioctl: Set mode %d\n", short_data);
#endif
		case MMA7660FC_MODE_STANDBY:
			/* Disable polling first, then the chip
				can be put in standby mode */
			cancel_delayed_work_sync(&data->polling_work);
			mma7660fc_set_mode(MMA7660FC_MODE_STANDBY);
			data->standby = 1;
			break;
		case MMA7660FC_MODE_MEASURE:
			data->samples_to_skip = SAMPLES_TO_SKIP;
			mma7660fc_set_mode(MMA7660FC_MODE_MEASURE);
			data->standby = 0;
			/* Enabling polling if needed */
			if (data->poll_delay > 0)
				schedule_delayed_work(&data->polling_work,0);
			break;
		case MMA7660FC_MODE_TEST:
			cancel_delayed_work_sync(&data->polling_work);
			mma7660fc_set_mode(MMA7660FC_MODE_TEST);
			data->standby = 0;
			break;
		default:
			return -EINVAL;
		}
		break;
	case MMA7660FC_IOCTL_G_MODE:
		short_data = mma7660fc_get_mode();
#ifdef DEBUG_IOCTL
		printk("mma7660fc_ctrl_ioctl: Get mode %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
	case MMA7660FC_IOCTL_S_POLL_DELAY:
		if (copy_from_user(&int_data, argp, sizeof(int_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma7660fc_ctrl_ioctl: Set polling delay %d\n", int_data);
#endif
		if (int_data < 0) {
			return -EINVAL;
		} else if (int_data == 0) {
			if (data->poll_delay == 0)
				break;
			/* Disabling polling */
			data->poll_delay = 0;
			cancel_delayed_work_sync(&data->polling_work);
			/* Optimize HW samplerate */
			mma7660fc_set_auto_wake_delay(1000);
		} else {
			/* Set the polling delay and optimize HW samplerate */
			if (mma7660fc_get_mode() == MMA7660FC_MODE_MEASURE) {
				if (data->poll_delay > 0)
					cancel_delayed_work_sync(&data->polling_work);
				data->poll_delay = int_data;
				mma7660fc_set_mode(MMA7660FC_MODE_STANDBY);
				if (data->poll_delay < AVERAGE_DELAY_THRESHOLD)
					mma7660fc_set_auto_wake_delay(int_data/SAMPLES_TO_AVERAGE);
				else
					mma7660fc_set_auto_wake_delay(int_data);
				data->samples_to_skip = SAMPLES_TO_SKIP;
				mma7660fc_set_mode(MMA7660FC_MODE_MEASURE);
				/* Schedule the next work in data->poll_delay ms */
				if (data->poll_delay < AVERAGE_DELAY_THRESHOLD)
					schedule_delayed_work(&data->polling_work, (HZ * data->poll_delay) / (SAMPLES_TO_AVERAGE * 1000));
				else
					schedule_delayed_work(&data->polling_work, (HZ * data->poll_delay) / 1000);
			} else {
				data->poll_delay = int_data;
				if (data->poll_delay < AVERAGE_DELAY_THRESHOLD)
					mma7660fc_set_auto_wake_delay(int_data/SAMPLES_TO_AVERAGE);
				else
					mma7660fc_set_auto_wake_delay(int_data);
			}
		}
		break;
	case MMA7660FC_IOCTL_G_POLL_DELAY:
		int_data = (int) data->poll_delay;
#ifdef DEBUG_IOCTL
		printk("mma7660fc_ctrl_ioctl: Get polling delay %d\n", int_data);
#endif
		if (copy_to_user(argp, &int_data, sizeof(int_data)))
			return -EFAULT;
		break;
	case MMA7660FC_IOCTL_S_PULSE_DEBOUNCE:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma7660fc_ctrl_ioctl: Set pulse duration %d\n", short_data);
#endif
		if (short_data < 1 || short_data > 256)
			return -EINVAL;
		mma7660fc_set_pulse_debounce(short_data);
		break;
	case MMA7660FC_IOCTL_G_PULSE_DEBOUNCE:
		short_data = mma7660fc_get_pulse_debounce();
#ifdef DEBUG_IOCTL
		printk("mma7660fc_ctrl_ioctl: Get pulse duration %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
	case MMA7660FC_IOCTL_S_PULSE_THRSHOLD:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma7660fc_ctrl_ioctl: Set pulse threshold %d\n", short_data);
#endif
		if (short_data < 0 || short_data > 0x1F)
			return -EINVAL;
		mma7660fc_set_pulse_threshold(short_data);
		break;
	case MMA7660FC_IOCTL_G_PULSE_THRSHOLD:
		short_data = mma7660fc_get_pulse_threshold();
#ifdef DEBUG_IOCTL
		printk("mma7660fc_ctrl_ioctl: Get pulse threshold %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
	case MMA7660FC_IOCTL_S_PULSE_AXIS:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma7660fc_ctrl_ioctl: Set pulse axis %d\n", short_data);
#endif
		if (short_data < 0 || short_data > 0x03)
			return -EINVAL;
		mma7660fc_set_pulse_axis(short_data);
		break;
	case MMA7660FC_IOCTL_G_PULSE_AXIS:
		short_data = mma7660fc_get_pulse_axis();
#ifdef DEBUG_IOCTL
		printk("mma7660fc_ctrl_ioctl: Get pulse axis %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
	case MMA7660FC_IOCTL_GP_EVENT:
		if (copy_from_user(&int_data, argp, sizeof(int_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma7660fc_ctrl_ioctl: General purpose event %d\n", int_data);
#endif
		if (int_data < 0 || int_data > 65535)
			return -EINVAL;
		mma7660fc_general_purpose_event(int_data);
		break;
	case MMA7660FC_IOCTL_G_ACCEL_DATA:
		mma7660fc_get_values(&accel_values);
#ifdef DEBUG_IOCTL
		printk("mma7660fc_ctrl_ioctl: Get accel values x=%d, y=%d, z=%d\n", accel_values.x, accel_values.y, accel_values.z);
#endif
		if (copy_to_user(argp, &accel_values, sizeof(struct accel_data)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct file_operations mma7660fc_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = mma7660fc_ctrl_open,
	.release = mma7660fc_ctrl_release,
	.unlocked_ioctl = mma7660fc_ctrl_ioctl,
};

static struct miscdevice mma7660fc_ctrl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "accel_ctrl",
	.fops = &mma7660fc_ctrl_fops,
};


/*
 * I2C Stuff
 */

static inline int mma7660fc_write(struct i2c_client *client, int reg, int value)
{
#ifdef DEBUG_REG
	int ret;
	ret = i2c_smbus_write_byte_data(client, reg, value);
	printk("mma7660fc_write: Reg = 0x%02X, Value = 0x%02X, Ret = %d\n", reg, value, ret);
	return ret;
#else
	return i2c_smbus_write_byte_data(client, reg, value);
#endif
}

static inline int mma7660fc_read(struct i2c_client *client, int reg)
{
#ifdef DEBUG_REG
	int value;
	value = i2c_smbus_read_byte_data(client, reg);;
	printk("mma7660fc_read: Reg = 0x%02X, Value = 0x%02X\n", reg, value);
	return value;
#else
	return i2c_smbus_read_byte_data(client, reg);
#endif
}

static int mma7660fc_probe(struct i2c_client *client, 
		const struct i2c_device_id *id)
{
	int err;
	struct mma7660fc_data *i2c_data;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s functinality check failed\n", id->name);
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	printk(KERN_INFO "MMA7660FC Init\n");

	i2c_data = kzalloc(sizeof(struct mma7660fc_data), GFP_KERNEL);
	if (!i2c_data) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_data->pdata = (struct mma7660fc_pdata*) client->dev.platform_data;
	if (!i2c_data->pdata) {
		printk(KERN_ERR "mma7660fc_probe: No platform data !!\n");
		err = -ENODEV;
		goto exit_plat_data_failed;
	}

	err = request_irq(i2c_data->pdata->irq, mma7660fc_isr, IRQF_TRIGGER_RISING,
			    "mma7660fc-int", client);
	if(err) {
		printk(KERN_ERR "mma7660fc_probe: Irq request failed (irq %d)!!\n", i2c_data->pdata->irq);
		goto exit_req_irq_failed;
	}

	INIT_DELAYED_WORK(&i2c_data->polling_work, mma7660fc_polling_work_func);
	INIT_WORK(&i2c_data->irq_work, mma7660fc_irq_work_func);
	i2c_set_clientdata(client, i2c_data);
	this_client = client;

	/* regulators */
	accel_1v8 = regulator_get(&client->dev, "ACCEL_1V8");
	if (IS_ERR(accel_1v8))
		dev_dbg(&client->dev, "no ACCEL_1V8 for this accelerometer\n");
	else
		regulator_enable(accel_1v8);
	accel_vcc = regulator_get(&client->dev, "ACCEL_VCC");
	if (IS_ERR(accel_vcc))
		dev_dbg(&client->dev, "no ACCEL_VCC for this accelerometer\n");
	else
		regulator_enable(accel_vcc);

	i2c_data->input_dev = input_allocate_device();
	/* Set the polling delay to 0 ms, disabling polling */
	i2c_data->poll_delay = 0;
	/* initialize reference counting */
	i2c_data->opencount = 0;
	i2c_data->sample_number = 0;
	i2c_data->samples_to_skip = SAMPLES_TO_SKIP;

	memset(&i2c_data->tmp_values, 0, sizeof(struct accel_data));

#ifdef FILTER_ENABLED
	i2c_data->circular_index = 0;
	memset(i2c_data->circular_buffer, 0, sizeof(struct accel_data) * FILTER_DEPTH);
#endif

	if (!i2c_data->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "mma7660fc_probe: Failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	set_bit(EV_ABS, i2c_data->input_dev->evbit);
	/* x-axis acceleration */
	input_set_abs_params(i2c_data->input_dev, ABS_X, -32, 31, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(i2c_data->input_dev, ABS_Y, -32, 31, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(i2c_data->input_dev, ABS_Z, -32, 31, 0, 0);
	/* pulse event */
	input_set_abs_params(i2c_data->input_dev, ABS_MISC, 0, 255, 0, 0);
	/* GP event */
	input_set_abs_params(i2c_data->input_dev, ABS_TILT_X, 0, 65535, 0, 0);

	i2c_data->input_dev->name = "MMA7660FC Accelerometer";
	i2c_data->input_dev->open = mma7660fc_open;
	i2c_data->input_dev->close = mma7660fc_close;

	err = misc_register(&mma7660fc_ctrl_device);
	if (err) {
		printk(KERN_ERR
		       "mma7660fc_probe: Unable to register control misc device\n");
		goto exit_misc_device_register_failed;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	/* register early suspend handler */
	i2c_data->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	i2c_data->early_suspend.suspend = mma7456_early_suspend;
	i2c_data->early_suspend.resume = mma7456_early_resume;
	register_early_suspend(&i2c_data->early_suspend);
#endif
	/* Configure the chip */
	err = mma7660fc_configure(i2c_data);
	if (err) {
		printk(KERN_ERR
		       "mma7660fc_probe: mma7660fc initialization failed\n");
		goto exit_init_failed;
	}
	
	err = input_register_device(i2c_data->input_dev);
	if (err) {
		printk(KERN_ERR
		       "mma7660fc_probe: Unable to register input device: %s\n",
		       i2c_data->input_dev->name);
		goto exit_input_register_device_failed;
	}

	return 0;

exit_input_register_device_failed:
exit_init_failed:
	unregister_early_suspend(&i2c_data->early_suspend);
	misc_deregister(&mma7660fc_ctrl_device);
exit_misc_device_register_failed:
	input_free_device(i2c_data->input_dev);
exit_input_dev_alloc_failed:
	if (!IS_ERR(accel_1v8)) {
		regulator_disable(accel_1v8);
		regulator_put(accel_1v8);
	}
	if (!IS_ERR(accel_vcc)) {
		regulator_disable(accel_vcc);
		regulator_put(accel_vcc);
	}
	free_irq(i2c_data->pdata->irq, client);
exit_req_irq_failed:
exit_plat_data_failed:
	kfree(i2c_data);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
	
static int __exit mma7660fc_remove(struct i2c_client *client)
{
	struct mma7660fc_data *data = i2c_get_clientdata(client);
	misc_deregister(&mma7660fc_ctrl_device);
	input_unregister_device(data->input_dev);
	unregister_early_suspend(&data->early_suspend);
	if (!IS_ERR(accel_1v8)) {
		regulator_disable(accel_1v8);
		regulator_put(accel_1v8);
	}
	if (!IS_ERR(accel_vcc)) {
		regulator_disable(accel_vcc);
		regulator_put(accel_vcc);
	}
	free_irq(data->pdata->irq, client);
	kfree(data);
	return 0;
}

static int mma7660fc_suspend(struct i2c_client *client, pm_message_t mesg)
{
//	struct mma7660fc_data *data = i2c_get_clientdata(client);
//	if (data->opencount) {
//		cancel_delayed_work_sync(&data->polling_work);
//		mma7660fc_set_mode(MMA7660FC_MODE_STANDBY);
//	}

	if (!IS_ERR(accel_1v8))
		regulator_disable(accel_1v8);
	if (!IS_ERR(accel_vcc))
		regulator_disable(accel_vcc);

	return 0;
}

static int mma7660fc_resume(struct i2c_client *client)
{
	if (!IS_ERR(accel_1v8))
		regulator_enable(accel_1v8);
	if (!IS_ERR(accel_vcc))
		regulator_enable(accel_vcc);

//	struct mma7660fc_data *data = i2c_get_clientdata(client);
//	if (data->opencount) {
//		mma7660fc_set_mode(MMA7660FC_MODE_MEASURE);
//		if (data->poll_delay > 0)
//			schedule_delayed_work(&data->polling_work,0);
//	}
	return 0;
}


static const struct i2c_device_id mma7660fc_id[] = {
	{"mma7660fc", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mma7660fc_id);

static struct i2c_driver mma7660fc_driver = {

	.driver = {
		.owner	= THIS_MODULE,
		.name	= "mma7660fc",
	},
	.probe		= mma7660fc_probe,
	.suspend	= mma7660fc_suspend,
	.resume		= mma7660fc_resume,
	.remove		= __exit_p(mma7660fc_remove),
	.id_table	= mma7660fc_id,
};

static int __init mma7660fc_init(void)
{
	int res;
	
	if ((res = i2c_add_driver(&mma7660fc_driver))) {
		printk("mma7660fc: Driver registration failed, module not inserted.\n");
		return res;
	}

	printk("MMA7660FC driver version %s (%s)\n", MMA7660FC_VERSION, MMA7660FC_DATE);
	
	return 0;
}

static void __exit mma7660fc_exit(void)
{
	i2c_del_driver(&mma7660fc_driver);
}

MODULE_AUTHOR("Jean-Christophe Rona <rona@archos.com>");
MODULE_DESCRIPTION("Input device driver for MMA7660FC accelerometer");
MODULE_LICENSE("GPL");

module_init(mma7660fc_init)
module_exit(mma7660fc_exit)
