/*
 *   MMA8453Q Accelerometer driver
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
#include <linux/clk.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/mma8453q.h>
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

#define MMA8453Q_VERSION		"0.1"
#define MMA8453Q_DATE			"1 April 2011"

/*
 * If the polling delay is bigger that this threshold,
 * it means that the accuracy does not really matter.
 * So no needed to double the HW samplerate,
 * and save some mA instead.
 */
#define AVERAGE_DELAY_THRESHOLD		500
#define SAMPLES_TO_AVERAGE		1
#define MIN_SUPPORTED_POLLING_DELAY_MS	10

#define SAMPLES_TO_SKIP			1

//#define FILTER_ENABLED
#define FILTER_DEPTH			5
#define FILTER_DENOMINATOR		FILTER_DEPTH
#define FILER_DELAY_THRESHOLD		100

static struct regulator *accel_1v8;
static struct regulator *accel_vcc;

static inline int mma8453q_write(struct i2c_client *client, int reg, int value);
static inline int mma8453q_read(struct i2c_client *client, int reg);
static inline int mma8453q_read_multiple(struct i2c_client *client, int reg, int length, u8* values);

struct mma8453q_data {
	struct input_dev *input_dev;
	struct delayed_work polling_work;
	struct work_struct irq_work;
	struct mma8453q_pdata *pdata;
	/* Delay in ms between two acceleration reports */
	unsigned long poll_delay;
	int opencount;
	struct early_suspend early_suspend;
	unsigned standby:1;
	int sample_number;
	struct mma8453q_accel_data tmp_values;
	int samples_to_skip;
	struct clk * auxclk1;
	int suspended;
#ifdef FILTER_ENABLED
	struct mma8453q_accel_data circular_buffer[FILTER_DEPTH];
	unsigned int circular_index;
#endif
};

/* I2C client handle */
static struct i2c_client *this_client;

/* Filter coefficients */
//static int filter_coeff[FILTER_DEPTH] = {1, 1, 1, 1, 1};

/*
 * MMA8453Q stuff
 */

static short mma8453q_get_id(void)
{
	u8 data;
	
	data = mma8453q_read(this_client, REG_WHO_AM_I) & MMA8453Q_ID_MASK;
	return data;
}

static int mma8453q_set_mode(short mode)
{
	int ret = 0;
	u8 data;
	
	data = mma8453q_read(this_client, REG_CTRL_REG1) & ~MMA8453Q_MODE_MASK;
	ret = mma8453q_write(this_client, REG_CTRL_REG1, data |
				(mode & MMA8453Q_MODE_MASK));
	return ret;
}

static short mma8453q_get_mode(void)
{
	u8 data;
	
	data = mma8453q_read(this_client, REG_CTRL_REG1) & MMA8453Q_MODE_MASK;
	return data;
}

#if 0
static short mma8453q_get_sysmode(void)
{
	u8 data;
	
	data = mma8453q_read(this_client, REG_SYSMOD) & MMA8453Q_SYSMODE_MASK;
	return data;
}
#endif

static int mma8453q_set_range(short range)
{
	int ret = 0;
	u8 data;
	
	data = mma8453q_read(this_client, REG_XYZ_DATA_CFG) & ~MMA8453Q_RANGE_MASK;
	ret = mma8453q_write(this_client, REG_XYZ_DATA_CFG, data |
				(range & MMA8453Q_RANGE_MASK));
	return ret;
}

static short mma8453q_get_range(void)
{
	u8 data;
	
	data = mma8453q_read(this_client, REG_XYZ_DATA_CFG) & MMA8453Q_RANGE_MASK;
	return data;
}

static int mma8453q_set_fast_read(int on)
{
	int ret = 0;
	u8 data;

	if (!on)
		data = mma8453q_read(this_client, REG_CTRL_REG1) & ~MMA8453Q_FAST_READ;
	else
		data = mma8453q_read(this_client, REG_CTRL_REG1) | MMA8453Q_FAST_READ;

	ret = mma8453q_write(this_client, REG_CTRL_REG1, data);
	return ret;
}

static int mma8453q_get_fast_read(void)
{
	u8 data;

	data = mma8453q_read(this_client, REG_CTRL_REG1) & MMA8453Q_FAST_READ;
	return ((data > 0) ? MMA8453Q_F_READ_ON : data);
}

static int mma8453q_set_int_sources(int sources)
{
	int ret = 0;
	u8 data;

	data = mma8453q_read(this_client, REG_INT_SOURCE) & ~MMA8453Q_INT_SRC_MASK;
	ret = mma8453q_write(this_client, REG_INT_SOURCE, data |
				(sources & MMA8453Q_INT_SRC_MASK));
	return ret;
}

static int mma8453q_set_int_ppod(int on)
{
	int ret = 0;
	u8 data;

	if (!on)
		data = mma8453q_read(this_client, REG_CTRL_REG3) & ~MMA8453Q_INT_PP_OD;
	else
		data = mma8453q_read(this_client, REG_CTRL_REG3) | MMA8453Q_INT_PP_OD;

	ret = mma8453q_write(this_client, REG_CTRL_REG3, data);
	return ret;
}

#if 0
static int mma8453q_get_int_ppod(void)
{
	u8 data;

	data = mma8453q_read(this_client, REG_CTRL_REG3) & MMA8453Q_INT_PP_OD;
	return ((data > 0) ? MMA8453Q_OPEN_DRAIN : data);
}
#endif

static int mma8453q_set_int_polarity(int on)
{
	int ret = 0;
	u8 data;

	if (!on)
		data = mma8453q_read(this_client, REG_CTRL_REG3) & ~MMA8453Q_INT_POLARITY;
	else
		data = mma8453q_read(this_client, REG_CTRL_REG3) | MMA8453Q_INT_POLARITY;

	ret = mma8453q_write(this_client, REG_CTRL_REG3, data);
	return ret;
}

#if 0
static int mma8453q_get_int_polarity(void)
{
	u8 data;

	data = mma8453q_read(this_client, REG_CTRL_REG3) & MMA8453Q_INT_POLARITY;
	return ((data > 0) ? MMA8453Q_ACTIVE_HIGH : data);
}

static int mma8453q_set_pulse_debounce(short samples)
{
	int ret = 0;
	u8 data;
	
	data = mma8453q_read(this_client, REG_PDET) & ~MMA8453Q_PULSE_DEBOUNCE_MASK;
	ret = mma8453q_write(this_client, REG_PDET, data |
				((samples - 1)& MMA8453Q_PULSE_DEBOUNCE_MASK));
	return ret;
}

static short mma8453q_get_pulse_debounce(void)
{
	u8 data;
	
	data = mma8453q_read(this_client, REG_PDET) & MMA8453Q_PULSE_DEBOUNCE_MASK;
	return data;
}

static int mma8453q_set_pulse_threshold(short trshld)
{
	int ret = 0;
	u8 data;
	
	data = mma8453q_read(this_client, REG_PD) & ~MMA8453Q_PULSE_THRESHOLD_MASK;
	ret = mma8453q_write(this_client, REG_PD, data |
				(trshld & MMA8453Q_PULSE_THRESHOLD_MASK));
	return ret;
}

static short mma8453q_get_pulse_threshold(void)
{
	u8 data;
	
	data = mma8453q_read(this_client, REG_PD) & MMA8453Q_PULSE_THRESHOLD_MASK;
	return data;
}

static int mma8453q_set_pulse_axis(short axis)
{
	int ret = 0;
	u8 data;
	
	data = mma8453q_read(this_client, REG_PD) & ~MMA8453Q_PULSE_AXIS_MASK;
	ret = mma8453q_write(this_client, REG_PD, data |
				(axis & MMA8453Q_PULSE_AXIS_MASK));
	return ret;
}

static short mma8453q_get_pulse_axis(void)
{
	u8 data;
	
	data = mma8453q_read(this_client, REG_PD) & MMA8453Q_PULSE_AXIS_MASK;
	return data;
}

static int mma8453q_interrupt_setup(short events)
{
	int ret = 0;
	u8 data;
	
	data = mma8453q_read(this_client, REG_INTSU) & ~MMA8453Q_INT_SETUP_MASK;
	ret = mma8453q_write(this_client, REG_INTSU, events |
				(events & MMA8453Q_INT_SETUP_MASK));
	return ret;
}

static short mma8453q_get_interrupt_list(void)
{
	u8 data;
	
	data = mma8453q_read(this_client, REG_INTSU) & MMA8453Q_INT_SETUP_MASK;
	return data;
}

static short mma8453q_get_tilt_status(void)
{
	u8 data;
	
	while ((data = mma8453q_read(this_client, REG_TILT) & MMA8453Q_TILT_STATUS_MASK) & MMA8453Q_INVALID_FLAG);
	return data;
}

static int mma8453q_set_sleep_count(short count)
{
	int ret = 0;
	u8 data;
	
	data = mma8453q_read(this_client, REG_SPCNT) & ~MMA8453Q_SLEEP_COUNT_MASK;
	ret = mma8453q_write(this_client, REG_SPCNT, data |
				(count & MMA8453Q_SLEEP_COUNT_MASK));
	return ret;
}

static short mma8453q_get_sleep_count(void)
{
	u8 data;
	
	data = mma8453q_read(this_client, REG_SPCNT) & MMA8453Q_SLEEP_COUNT_MASK;
	return data;
}

static int mma8453q_set_auto_wake_samplerate(short samplerate)
{
	int ret = 0;
	u8 data;
	
	data = mma8453q_read(this_client, REG_SR) & ~MMA8453Q_SR_AUTO_WAKE_MASK;
	ret = mma8453q_write(this_client, REG_SR, data |
				(samplerate & MMA8453Q_SR_AUTO_WAKE_MASK));
	return ret;
}

static int mma8453q_set_auto_wake_delay(int delay)
{
	if (delay > 999) {
		return mma8453q_set_auto_wake_samplerate(MMA8453Q_SR_AUTO_WAKE_1);
	} else if (delay > 499) {
		return mma8453q_set_auto_wake_samplerate(MMA8453Q_SR_AUTO_WAKE_2);
	} else if (delay > 249) {
		return mma8453q_set_auto_wake_samplerate(MMA8453Q_SR_AUTO_WAKE_4);
	} else if (delay > 124) {
		return mma8453q_set_auto_wake_samplerate(MMA8453Q_SR_AUTO_WAKE_8);
	} else if (delay > 62) {
		return mma8453q_set_auto_wake_samplerate(MMA8453Q_SR_AUTO_WAKE_16);
	} else if (delay > 31) {
		return mma8453q_set_auto_wake_samplerate(MMA8453Q_SR_AUTO_WAKE_32);
	} else if (delay > 15) {
		return mma8453q_set_auto_wake_samplerate(MMA8453Q_SR_AUTO_WAKE_64);
	} else
		return mma8453q_set_auto_wake_samplerate(MMA8453Q_SR_AUTO_WAKE_120);
}

static short mma8453q_get_auto_wake_samplerate(void)
{
	u8 data;
	
	data = mma8453q_read(this_client, REG_SR) & MMA8453Q_SR_AUTO_WAKE_MASK;
	return data;
}

static int mma8453q_set_auto_sleep_samplerate(short samplerate)
{
	int ret = 0;
	u8 data;
	
	data = mma8453q_read(this_client, REG_SR) & ~MMA8453Q_SR_AUTO_SLEEP_MASK;
	ret = mma8453q_write(this_client, REG_SR, data |
				(samplerate & MMA8453Q_SR_AUTO_SLEEP_MASK));
	return ret;
}

static short mma8453q_get_auto_sleep_samplerate(void)
{
	u8 data;
	
	data = mma8453q_read(this_client, REG_SR) & MMA8453Q_SR_AUTO_SLEEP_MASK;
	return data;
}

static int mma8453q_set_sample_debounce(short samples)
{
	int ret = 0;
	u8 data;
	
	data = mma8453q_read(this_client, REG_SR) & ~MMA8453Q_SR_DEBOUNCE_MASK;
	ret = mma8453q_write(this_client, REG_SR, data |
				(((samples - 1) << MMA8453Q_SR_DEBOUNCE_SHIFT) & MMA8453Q_SR_DEBOUNCE_MASK));
	return ret;
}

static short mma8453q_get_sample_debounce(void)
{
	u8 data;
	
	data = mma8453q_read(this_client, REG_SR) & MMA8453Q_SR_DEBOUNCE_MASK;
	return (data >> MMA8453Q_SR_DEBOUNCE_SHIFT);
}

static int mma8453q_enable_auto_wake(int on)
{
	int ret = 0;
	u8 data;

	if (!on)
		data = mma8453q_read(this_client, REG_MODE) | MMA8453Q_MODE_AUTO_WAKE;
	else
		data = mma8453q_read(this_client, REG_MODE) & ~MMA8453Q_MODE_AUTO_WAKE;

	ret = mma8453q_write(this_client, REG_MODE, data);
	return ret;
}

static int mma8453q_enable_auto_sleep(int on)
{
	int ret = 0;
	u8 data;

	if (!on)
		data = mma8453q_read(this_client, REG_MODE) | MMA8453Q_MODE_AUTO_SLEEP;
	else
		data = mma8453q_read(this_client, REG_MODE) & ~MMA8453Q_MODE_AUTO_SLEEP;

	ret = mma8453q_write(this_client, REG_MODE, data);
	return ret;
}

static int mma8453q_set_prescaler(int divisor)
{
	int ret = 0;
	u8 data;

	if (!divisor)
		data = mma8453q_read(this_client, REG_MODE) | MMA8453Q_MODE_PRESCALER;
	else
		data = mma8453q_read(this_client, REG_MODE) & ~MMA8453Q_MODE_PRESCALER;

	ret = mma8453q_write(this_client, REG_MODE, data);
	return ret;
}

static int mma8453q_set_int_pin_type(int type)
{
	int ret = 0;
	u8 data;

	if (!type)
		data = mma8453q_read(this_client, REG_MODE) | MMA8453Q_MODE_INT_TYPE;
	else
		data = mma8453q_read(this_client, REG_MODE) & ~MMA8453Q_MODE_INT_TYPE;

	ret = mma8453q_write(this_client, REG_MODE, data);
	return ret;
}

static int mma8453q_set_int_pin_active_level(int level)
{
	int ret = 0;
	u8 data;

	if (!level)
		data = mma8453q_read(this_client, REG_MODE) | MMA8453Q_MODE_INT_ACTIVE_LEVEL;
	else
		data = mma8453q_read(this_client, REG_MODE) & ~MMA8453Q_MODE_INT_ACTIVE_LEVEL;

	ret = mma8453q_write(this_client, REG_MODE, data);
	return ret;
}
#endif

static int mma8453q_get_values(struct mma8453q_accel_data * values)
{
	u8 tmp[6];
	bool ready = false;
	unsigned long timeout;
	
	if (!values) {
		printk("mma8453q_get_values: values = NULL !!");
		return -EINVAL;
	}

	/* x, y and z are 10 bits signed values */
	/* Wait for the data to be ready */
	timeout = jiffies + msecs_to_jiffies(100);
	while (time_before(jiffies, timeout)) {
		if (mma8453q_read(this_client, REG_STATUS) & MMA8453Q_STATUS_ZYXRDY) {
			ready = true;
			break;
		}
	}
	if (!ready)
		return -ETIMEDOUT;
	
	/* Read all 6 bytes of acceleration data in a single I2C transfert */
	if (mma8453q_read_multiple(this_client, REG_OUT_X_MSB, 6, tmp) != 6) {
		printk(KERN_ERR "mma8453q_get_values: Failed to read acceleration values !\n");
		values->x = values->y = values->z = 0;
		return -EIO;
	}

	values->x = (((signed char)tmp[0]) * 4) | ((tmp[1] >> 6) & 0x3);
	values->y = (((signed char)tmp[2]) * 4) | ((tmp[3] >> 6) & 0x3);
	values->z = (((signed char)tmp[4]) * 4) | ((tmp[5] >> 6) & 0x3);
	
	return 0;
}

static void mma8453q_report_values(void)
{
	struct mma8453q_accel_data values;
	struct mma8453q_data *data = i2c_get_clientdata(this_client);
	int err;
	
	err = mma8453q_get_values(&values);
	if (IS_ERR_VALUE(err))
		return;

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
	memcpy(&data->circular_buffer[data->circular_index], &data->tmp_values, sizeof(struct mma8453q_accel_data));

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
	printk("mma8453q_report_values: X = %d, Y = %d, Z = %d\n", values.x, values.y, values.z);
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

static void mma8453q_general_purpose_event(int code)
{
	struct mma8453q_data *data = i2c_get_clientdata(this_client);

	input_report_abs(data->input_dev, ABS_TILT_X, code);
	return;
}

static int mma8453q_configure(struct mma8453q_data *data)
{
	int ret = 0;
	
	/* Set Standby mode, Measurement mode will
		be set when the device will be open */
	ret = mma8453q_set_mode(MMA8453Q_MODE_STANDBY);
	if (ret) {
		printk(KERN_ERR "mma8453q_configure: mma8453q_set_mode failed\n");
		return ret;
	}
	data->standby = 1;

	/* Set range to 2G */
	ret = mma8453q_set_range(MMA8453Q_RANGE_2G);
	if (ret) {
		printk(KERN_ERR "mma8453q_configure: mma8453q_set_range failed\n");
		return ret;
	}

	/* Disable Fast Read mode (set output to 10 bits instead of 8 bits) */
	ret = mma8453q_set_fast_read(MMA8453Q_F_READ_OFF);
	if (ret) {
		printk(KERN_ERR "mma8453q_configure: mma8453q_set_fast_read failed\n");
		return ret;
	}

	/* TODO: Decide what kind of event may be useful for us */
	/* We don't want anything to trigger an interrupt for now */
	ret = mma8453q_set_int_sources(MMA8453Q_INT_SRC_NONE);
	if (ret) {
		printk(KERN_ERR "mma8453q_configure: mma8453q_set_int_sources failed\n");
		return ret;
	}

	/* INT is push-pull */
	ret = mma8453q_set_int_ppod(MMA8453Q_PUSH_PULL);
	if (ret) {
		printk(KERN_ERR
			"mma8453q_configure: mma8453q_set_int_ppod failed\n");
		return ret;
	}

	/* INT is active high */
	ret = mma8453q_set_int_polarity(MMA8453Q_ACTIVE_HIGH);
	if (ret) {
		printk(KERN_ERR
			"mma8453q_configure: mma8453q_set_int_polarity failed\n");
		return ret;
	}

#if 0
	/* Pulse (tap) disabled for X, Y and Z */
	ret = mma8453q_set_pulse_axis(MMA8453Q_PULSE_AXIS_NONE);
	if (ret) {
		printk(KERN_ERR "mma8453q_configure: mma8453q_set_pulse_enabled_axis failed\n");
		return ret;
	}

	/* Arbitrary set pulse threshold to 31 */
	ret = mma8453q_set_pulse_threshold(31);
	if (ret) {
		printk(KERN_ERR
			"mma8453q_configure: mma8453q_set_pulse_threshold failed\n");
		return ret;
	}

	/* Arbitrary set pulse debounce to 10 samples */
	ret = mma8453q_set_pulse_debounce(10);
	if (ret) {
		printk(KERN_ERR
			"mma8453q_configure: mma8453q_set_pulse_debounce failed\n");
		return ret;
	}

	/* Sleep count to 0 */
	ret = mma8453q_set_sleep_count(0);
	if (ret) {
		printk(KERN_ERR
			"mma8453q_configure: mma8453q_set_sleep_count failed\n");
		return ret;
	}

	/* Max samplerate in wake mode */
	ret = mma8453q_set_auto_wake_samplerate(MMA8453Q_SR_AUTO_WAKE_120);
	if (ret) {
		printk(KERN_ERR
			"mma8453q_configure: mma8453q_set_auto_wake_samplerate failed\n");
		return ret;
	}

	/* Max samplerate in sleep mode */
	ret = mma8453q_set_auto_sleep_samplerate(MMA8453Q_SR_AUTO_SLEEP_32);
	if (ret) {
		printk(KERN_ERR
			"mma8453q_configure: mma8453q_set_auto_sleep_samplerate failed\n");
		return ret;
	}

	/* No debounce for portrait/landscape detection */
	ret = mma8453q_set_sample_debounce(1);
	if (ret) {
		printk(KERN_ERR
			"mma8453q_configure: mma8453q_set_sample_debounce failed\n");
		return ret;
	}

	/* TODO: power consumption improvement using sleep/wake mode */
	/* Disable auto wake for now */
	ret = mma8453q_enable_auto_wake(OFF);
	if (ret) {
		printk(KERN_ERR
			"mma8453q_configure: mma8453q_enable_auto_wake failed\n");
		return ret;
	}

	/* Disable auto sleep for now */
	ret = mma8453q_enable_auto_sleep(OFF);
	if (ret) {
		printk(KERN_ERR
			"mma8453q_configure: mma8453q_enable_auto_sleep failed\n");
		return ret;
	}

	/* No prescaler */
	ret = mma8453q_set_prescaler(DIVIDE_BY_1);
	if (ret) {
		printk(KERN_ERR
			"mma8453q_configure: mma8453q_set_prescaler failed\n");
		return ret;
	}
#endif

#if 0
	data->poll_delay = 10;
	data->samples_to_skip = SAMPLES_TO_SKIP;
	mma8453q_set_mode(MMA8453Q_MODE_ACTIVE);
	data->standby = 0;
	/* Enabling polling if needed */
	if (data->poll_delay > 0)
		schedule_delayed_work(&data->polling_work,0);
#endif

	return 0;
}

static int mma8453q_open(struct input_dev *dev)
{
	int ret = 0;
	struct mma8453q_data *data = i2c_get_clientdata(this_client);

	data->opencount++;
	if (data->opencount > 1)
		return 0;
	
	data->samples_to_skip = SAMPLES_TO_SKIP;

	/* Set Measurement mode */
	ret = mma8453q_set_mode(MMA8453Q_MODE_ACTIVE);
	if (ret) {
		printk(KERN_ERR
			"mma8453q_open: mma8453q_set_mode failed\n");
		return ret;
	}
	data->standby = 0;

	/* Everything is OK, enabling polling (if needed) */
	if (data->poll_delay > 0)
		schedule_delayed_work(&data->polling_work,0);
	
	return ret;
}

static void mma8453q_close(struct input_dev *dev)
{
	struct mma8453q_data *data = i2c_get_clientdata(this_client);

	data->opencount--;
	if (data->opencount > 0)
		return;
	
	/* Cancel remaining work */
	cancel_delayed_work_sync(&data->polling_work);
	/* Put the chip in Standby mode */
	mma8453q_set_mode(MMA8453Q_MODE_STANDBY);
	data->standby = 1;
	return;
}

static void mma8453q_polling_work_func(struct work_struct *work)
{
	struct mma8453q_data *data = i2c_get_clientdata(this_client);

	/* Schedule the next work in data->poll_delay ms */
	if (data->poll_delay < AVERAGE_DELAY_THRESHOLD)
		schedule_delayed_work(&data->polling_work, (HZ * data->poll_delay) / (SAMPLES_TO_AVERAGE * 1000));
	else
		schedule_delayed_work(&data->polling_work, (HZ * data->poll_delay) / 1000);

	mma8453q_report_values();
}

static void mma8453q_irq_work_func(struct work_struct *work)
{
//	struct mma8453q_data *data = i2c_get_clientdata(this_client);

//	input_report_abs(data->input_dev, ABS_MISC, mma8453q_get_tilt_status());
//	input_sync(data->input_dev);
}

static irqreturn_t mma8453q_isr(int irq, void *dev_id)
{
	struct mma8453q_data *data = i2c_get_clientdata(this_client);

#ifdef DEBUG_IRQ
	printk("mma8453q_isr: got irq %d\n", irq);
#endif

	schedule_work(&data->irq_work);

	return IRQ_HANDLED;
}

static void mma7456_early_suspend(struct early_suspend *h)
{
	struct mma8453q_data *data = 
		container_of(h, struct mma8453q_data, early_suspend);
	
	if (data->standby)
		return;
	
	if (data->poll_delay > 0)
		cancel_delayed_work(&data->polling_work);
	mma8453q_set_mode(MMA8453Q_MODE_STANDBY);
	data->standby = 1;
}

static void mma7456_early_resume(struct early_suspend *h)
{
	struct mma8453q_data *data = 
		container_of(h, struct mma8453q_data, early_suspend);

	if (!data->standby)
		return;
	
	data->samples_to_skip = SAMPLES_TO_SKIP;
	mma8453q_set_mode(MMA8453Q_MODE_ACTIVE);
	data->standby = 0;
	if (data->poll_delay > 0)
		schedule_delayed_work(&data->polling_work,0);
}

/*
 * Control device stuff
 */

static int mma8453q_ctrl_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int mma8453q_ctrl_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long
mma8453q_ctrl_ioctl(struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	struct mma8453q_data *data = i2c_get_clientdata(this_client);
	void __user *argp = (void __user *)arg;
	short short_data;
	struct mma8453q_accel_data accel_values;
	int int_data;
	int err;

	switch (cmd) {
	case MMA8453Q_IOCTL_S_MODE:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
		switch(short_data) {
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Set mode %d\n", short_data);
#endif
		case MMA8453Q_MODE_STANDBY:
			/* Disable polling first, then the chip
				can be put in standby mode */
			cancel_delayed_work_sync(&data->polling_work);
			mma8453q_set_mode(MMA8453Q_MODE_STANDBY);
			data->standby = 1;
			break;
		case MMA8453Q_MODE_ACTIVE:
			data->samples_to_skip = SAMPLES_TO_SKIP;
			mma8453q_set_mode(MMA8453Q_MODE_ACTIVE);
			data->standby = 0;
			/* Enabling polling if needed */
			if (data->poll_delay > 0)
				schedule_delayed_work(&data->polling_work,0);
			break;
		default:
			return -EINVAL;
		}
		break;
	case MMA8453Q_IOCTL_G_MODE:
		short_data = mma8453q_get_mode();
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Get mode %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
	case MMA8453Q_IOCTL_S_POLL_DELAY:
		if (copy_from_user(&int_data, argp, sizeof(int_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Set polling delay %d\n", int_data);
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
//			mma8453q_set_auto_wake_delay(1000);
		} else {
			/* Set the polling delay and optimize HW samplerate */
			if (mma8453q_get_mode() == MMA8453Q_MODE_ACTIVE) {
				if (data->poll_delay > 0)
					cancel_delayed_work_sync(&data->polling_work);
				data->poll_delay = (int_data < MIN_SUPPORTED_POLLING_DELAY_MS) ? MIN_SUPPORTED_POLLING_DELAY_MS : int_data;
				mma8453q_set_mode(MMA8453Q_MODE_STANDBY);
//				if (data->poll_delay < AVERAGE_DELAY_THRESHOLD)
//					mma8453q_set_auto_wake_delay(data->poll_delay/SAMPLES_TO_AVERAGE);
//				else
//					mma8453q_set_auto_wake_delay(data->poll_delay);
				data->samples_to_skip = SAMPLES_TO_SKIP;
				mma8453q_set_mode(MMA8453Q_MODE_ACTIVE);
				/* Schedule the next work in data->poll_delay ms */
				if (data->poll_delay < AVERAGE_DELAY_THRESHOLD)
					schedule_delayed_work(&data->polling_work, (HZ * data->poll_delay) / (SAMPLES_TO_AVERAGE * 1000));
				else
					schedule_delayed_work(&data->polling_work, (HZ * data->poll_delay) / 1000);
			} else {
				data->poll_delay = (int_data < MIN_SUPPORTED_POLLING_DELAY_MS) ? MIN_SUPPORTED_POLLING_DELAY_MS : int_data;
//				if (data->poll_delay < AVERAGE_DELAY_THRESHOLD)
//					mma8453q_set_auto_wake_delay(data->poll_delay/SAMPLES_TO_AVERAGE);
//				else
//					mma8453q_set_auto_wake_delay(data->poll_delay);
			}
		}
		break;
	case MMA8453Q_IOCTL_G_POLL_DELAY:
		int_data = (int) data->poll_delay;
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Get polling delay %d\n", int_data);
#endif
		if (copy_to_user(argp, &int_data, sizeof(int_data)))
			return -EFAULT;
		break;
#if 0
	case MMA8453Q_IOCTL_S_PULSE_DEBOUNCE:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Set pulse duration %d\n", short_data);
#endif
		if (short_data < 1 || short_data > 256)
			return -EINVAL;
		mma8453q_set_pulse_debounce(short_data);
		break;
	case MMA8453Q_IOCTL_G_PULSE_DEBOUNCE:
		short_data = mma8453q_get_pulse_debounce();
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Get pulse duration %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
	case MMA8453Q_IOCTL_S_PULSE_THRSHOLD:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Set pulse threshold %d\n", short_data);
#endif
		if (short_data < 0 || short_data > 0x1F)
			return -EINVAL;
		mma8453q_set_pulse_threshold(short_data);
		break;
	case MMA8453Q_IOCTL_G_PULSE_THRSHOLD:
		short_data = mma8453q_get_pulse_threshold();
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Get pulse threshold %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
	case MMA8453Q_IOCTL_S_PULSE_AXIS:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Set pulse axis %d\n", short_data);
#endif
		if (short_data < 0 || short_data > 0x03)
			return -EINVAL;
		mma8453q_set_pulse_axis(short_data);
		break;
	case MMA8453Q_IOCTL_G_PULSE_AXIS:
		short_data = mma8453q_get_pulse_axis();
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Get pulse axis %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
#endif
	case MMA8453Q_IOCTL_G_RANGE:
		short_data = mma8453q_get_range();
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Get range %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
	case MMA8453Q_IOCTL_S_RANGE:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Set range %d\n", short_data);
#endif
		if (short_data < 0 || short_data > 0x1F)
			return -EINVAL;
		mma8453q_set_range(short_data);
		break;
	case MMA8453Q_IOCTL_G_FAST_MODE:
		int_data = mma8453q_get_fast_read();
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Get fast mode %d\n", int_data);
#endif
		if (copy_to_user(argp, &int_data, sizeof(int_data)))
			return -EFAULT;
		break;
	case MMA8453Q_IOCTL_S_FAST_MODE:
		if (copy_from_user(&int_data, argp, sizeof(int_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Set fast mode %d\n", int_data);
#endif
		if (int_data < 0 || int_data > 1)
			return -EINVAL;
		mma8453q_set_fast_read(int_data);
		break;
	case MMA8453Q_IOCTL_GP_EVENT:
		if (copy_from_user(&int_data, argp, sizeof(int_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: General purpose event %d\n", int_data);
#endif
		if (int_data < 0 || int_data > 65535)
			return -EINVAL;
		mma8453q_general_purpose_event(int_data);
		break;
	case MMA8453Q_IOCTL_G_ACCEL_DATA:
		err = mma8453q_get_values(&accel_values);
		if (IS_ERR_VALUE(err))
			return err;
#ifdef DEBUG_IOCTL
		printk("mma8453q_ctrl_ioctl: Get accel values x=%d, y=%d, z=%d\n", accel_values.x, accel_values.y, accel_values.z);
#endif
		if (copy_to_user(argp, &accel_values, sizeof(struct mma8453q_accel_data)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct file_operations mma8453q_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = mma8453q_ctrl_open,
	.release = mma8453q_ctrl_release,
	.unlocked_ioctl = mma8453q_ctrl_ioctl,
};

static struct miscdevice mma8453q_ctrl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "accel_ctrl",
	.fops = &mma8453q_ctrl_fops,
};


/*
 * I2C Stuff
 */

static inline int mma8453q_write(struct i2c_client *client, int reg, int value)
{
#ifdef DEBUG_REG
	int ret;
	ret = i2c_smbus_write_byte_data(client, reg, value);
	printk("mma8453q_write: Reg = 0x%02X, Value = 0x%02X, Ret = %d\n", reg, value, ret);
	return ret;
#else
	return i2c_smbus_write_byte_data(client, reg, value);
#endif
}

static inline int mma8453q_read(struct i2c_client *client, int reg)
{
#ifdef DEBUG_REG
	int value;
	value = i2c_smbus_read_byte_data(client, reg);
	printk("mma8453q_read: Reg = 0x%02X, Value = 0x%02X\n", reg, value);
	return value;
#else
	return i2c_smbus_read_byte_data(client, reg);
#endif
}

static inline int mma8453q_read_multiple(struct i2c_client *client, int reg, int length, u8* values)
{
#ifdef DEBUG_REG
	int read_len;
	read_len = i2c_smbus_read_i2c_block_data(client, reg, length, values);
	printk("mma8453q_read_multiple: Reg = 0x%02X, Length = 0x%02X\n", reg, read_len);
	return value;
#else
	return i2c_smbus_read_i2c_block_data(client, reg, length, values);
#endif
}

static int mma8453q_probe(struct i2c_client *client, 
		const struct i2c_device_id *id)
{
	const char *aux_name = "auxclk1_ck";
	int err;
	int chip_id;
	
	struct mma8453q_data *i2c_data;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s functinality check failed\n", id->name);
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	i2c_data = kzalloc(sizeof(struct mma8453q_data), GFP_KERNEL);
	if (!i2c_data) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_data->pdata = (struct mma8453q_pdata*) client->dev.platform_data;
	if (!i2c_data->pdata) {
		printk(KERN_ERR "mma8453q_probe: No platform data !!\n");
		err = -ENODEV;
		goto exit_plat_data_failed;
	}

	err = request_irq(i2c_data->pdata->irq1, mma8453q_isr, IRQF_TRIGGER_RISING,
			    "mma8453q-int", client);
	if(err) {
		printk(KERN_ERR "mma8453q_probe: Irq request failed (irq %d)!!\n", i2c_data->pdata->irq1);
		goto exit_req_irq1_failed;
	}

	if (i2c_data->pdata->irq2 != -1) {
		err = request_irq(i2c_data->pdata->irq2, mma8453q_isr, IRQF_TRIGGER_RISING,
				"mma8453q-int", client);
		if(err) {
			printk(KERN_ERR "mma8453q_probe: Irq request failed (irq %d)!!\n", i2c_data->pdata->irq2);
			goto exit_req_irq2_failed;
		}
	}

	INIT_DELAYED_WORK(&i2c_data->polling_work, mma8453q_polling_work_func);
	INIT_WORK(&i2c_data->irq_work, mma8453q_irq_work_func);
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

	if ((chip_id = mma8453q_get_id()) != 0)
		printk(KERN_INFO "Found MMA8453Q accelerometer with ID 0x%02X\n", chip_id);
	else {
		printk(KERN_INFO "No MMA8453Q accelerometer found !\n");
		goto exit_detection_failed;
	}

	/* Auxclk1 workaround TODO: the mma8453q should not need auxclk1 so this hack is voodoo / cargo code */
	if (cpu_is_omap44xx()) {
		i2c_data->auxclk1 = clk_get(NULL, aux_name);
		if (IS_ERR(i2c_data->auxclk1)) {
			pr_err("%s: auxclk1 not available\n", __func__);
			goto exit_auxclk1_failed;
		}
		clk_enable(i2c_data->auxclk1);
	} else {
		i2c_data->auxclk1 = NULL;
	}

	i2c_data->suspended = 0;

	i2c_data->input_dev = input_allocate_device();
	/* Set the polling delay to 0 ms, disabling polling */
	i2c_data->poll_delay = 0;
	/* initialize reference counting */
	i2c_data->opencount = 0;
	i2c_data->sample_number = 0;
	i2c_data->samples_to_skip = SAMPLES_TO_SKIP;

	memset(&i2c_data->tmp_values, 0, sizeof(struct mma8453q_accel_data));

#ifdef FILTER_ENABLED
	i2c_data->circular_index = 0;
	memset(i2c_data->circular_buffer, 0, sizeof(struct mma8453q_accel_data) * FILTER_DEPTH);
#endif

	if (!i2c_data->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "mma8453q_probe: Failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	set_bit(EV_ABS, i2c_data->input_dev->evbit);
	/* x-axis acceleration */
	input_set_abs_params(i2c_data->input_dev, ABS_X, -512, 512, 0, 0);
	/* y-axis acceleration */
	input_set_abs_params(i2c_data->input_dev, ABS_Y, -512, 512, 0, 0);
	/* z-axis acceleration */
	input_set_abs_params(i2c_data->input_dev, ABS_Z, -512, 512, 0, 0);
	/* pulse event */
	input_set_abs_params(i2c_data->input_dev, ABS_MISC, 0, 255, 0, 0);
	/* GP event */
	input_set_abs_params(i2c_data->input_dev, ABS_TILT_X, 0, 65535, 0, 0);

	i2c_data->input_dev->name = "MMA8453Q Accelerometer";
	i2c_data->input_dev->open = mma8453q_open;
	i2c_data->input_dev->close = mma8453q_close;

	err = misc_register(&mma8453q_ctrl_device);
	if (err) {
		printk(KERN_ERR
		       "mma8453q_probe: Unable to register control misc device\n");
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
	err = mma8453q_configure(i2c_data);
	if (err) {
		printk(KERN_ERR
		       "mma8453q_probe: mma8453q initialization failed\n");
		goto exit_init_failed;
	}
	
	err = input_register_device(i2c_data->input_dev);
	if (err) {
		printk(KERN_ERR
		       "mma8453q_probe: Unable to register input device: %s\n",
		       i2c_data->input_dev->name);
		goto exit_input_register_device_failed;
	}

	return 0;

exit_input_register_device_failed:
exit_init_failed:
	unregister_early_suspend(&i2c_data->early_suspend);
	misc_deregister(&mma8453q_ctrl_device);
exit_misc_device_register_failed:
	input_free_device(i2c_data->input_dev);
exit_input_dev_alloc_failed:
exit_auxclk1_failed:
exit_detection_failed:
	if (!IS_ERR(accel_1v8)) {
		regulator_disable(accel_1v8);
		regulator_put(accel_1v8);
	}
	if (!IS_ERR(accel_vcc)) {
		regulator_disable(accel_vcc);
		regulator_put(accel_vcc);
	}
	if (i2c_data->pdata->irq2 != -1)
		free_irq(i2c_data->pdata->irq2, client);
exit_req_irq2_failed:
	free_irq(i2c_data->pdata->irq1, client);
exit_req_irq1_failed:
exit_plat_data_failed:
	kfree(i2c_data);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
	
static int __exit mma8453q_remove(struct i2c_client *client)
{
	struct mma8453q_data *data = i2c_get_clientdata(client);
	misc_deregister(&mma8453q_ctrl_device);
	input_unregister_device(data->input_dev);
	unregister_early_suspend(&data->early_suspend);
	if (data->auxclk1) {
		if (!data->suspended)
			clk_disable(data->auxclk1);
		clk_put(data->auxclk1);
		data->auxclk1 = NULL;
	}
	if (!IS_ERR(accel_1v8)) {
		regulator_disable(accel_1v8);
		regulator_put(accel_1v8);
	}
	if (!IS_ERR(accel_vcc)) {
		regulator_disable(accel_vcc);
		regulator_put(accel_vcc);
	}

	if (data->pdata->irq2 != -1)
		free_irq(data->pdata->irq2, client);

	free_irq(data->pdata->irq1, client);
	kfree(data);
	return 0;
}

static int mma8453q_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct mma8453q_data *data = i2c_get_clientdata(client);
	printk("%s\n", __func__);
	if (data->auxclk1)
		clk_disable(data->auxclk1);
	data->suspended = 1;

//	if (data->opencount) {
//		cancel_delayed_work_sync(&data->polling_work);
//		mma8453q_set_mode(MMA8453Q_MODE_STANDBY);
//	}

	if (!IS_ERR(accel_1v8))
		regulator_disable(accel_1v8);
	if (!IS_ERR(accel_vcc))
		regulator_disable(accel_vcc);

	return 0;
}

static int mma8453q_resume(struct i2c_client *client)
{
	struct mma8453q_data *data = i2c_get_clientdata(client);
	printk("%s\n", __func__);

	if (!IS_ERR(accel_1v8))
		regulator_enable(accel_1v8);
	if (!IS_ERR(accel_vcc))
		regulator_enable(accel_vcc);

	if (data->auxclk1)
		clk_enable(data->auxclk1);
	data->suspended = 0;
//	if (data->opencount) {
//		mma8453q_set_mode(MMA8453Q_MODE_ACTIVE);
//		if (data->poll_delay > 0)
//			schedule_delayed_work(&data->polling_work,0);
//	}
	return 0;
}


static const struct i2c_device_id mma8453q_id[] = {
	{"mma8453q", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mma8453q_id);

static struct i2c_driver mma8453q_driver = {

	.driver = {
		.owner	= THIS_MODULE,
		.name	= "mma8453q",
	},
	.probe		= mma8453q_probe,
	.suspend	= mma8453q_suspend,
	.resume		= mma8453q_resume,
	.remove		= __exit_p(mma8453q_remove),
	.id_table	= mma8453q_id,
};

static int __init mma8453q_init(void)
{
	int res;
	
	if ((res = i2c_add_driver(&mma8453q_driver))) {
		printk("mma8453q: Driver registration failed, module not inserted.\n");
		return res;
	}

	printk("MMA8453Q driver version %s (%s)\n", MMA8453Q_VERSION, MMA8453Q_DATE);
	
	return 0;
}

static void __exit mma8453q_exit(void)
{
	i2c_del_driver(&mma8453q_driver);
}

MODULE_AUTHOR("Jean-Christophe Rona <rona@archos.com>");
MODULE_DESCRIPTION("Input device driver for MMA8453Q accelerometer");
MODULE_LICENSE("GPL");

module_init(mma8453q_init)
module_exit(mma8453q_exit)
