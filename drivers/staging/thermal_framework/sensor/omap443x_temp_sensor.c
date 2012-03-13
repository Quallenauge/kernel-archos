/*
 * OMAP4 Temperature sensor driver file
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: J Keerthy <j-keerthy@ti.com>
 * Author: Moiz Sonasath <m-sonasath@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/stddef.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/reboot.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <plat/common.h>
/* TO DO: This needs to be fixed */
#include "../../../../arch/arm/mach-omap2/control.h"
/* #include <plat/control.h> */
#include <plat/temperature_sensor.h>
#include <plat/omap_device.h>
#include <plat/omap-pm.h>
#include <mach/ctrl_module_core_44xx.h>
#include <linux/gpio.h>


#include <linux/thermal_framework.h>

/* This DEBUG flag is used to enable the sysfs entries */

#define TEMP_DEBUG 1

#define BGAP_THRESHOLD_T_HOT		73000	/* 73 deg C */
#define BGAP_THRESHOLD_T_COLD		71000	/* 71 deg C */
#define OMAP_ADC_START_VALUE		0
#define OMAP_ADC_END_VALUE			95
#define OMAP_MIN_TEMP				-40000	/* sensor starts at -40 deg C */

/* OMAP443X CONTROL_TEMP_SENSOR register */
#define BGAP_TEMPSOFF_MASK			(1<<12)
#define BGAP_TEMP_SENSOR_CONTCONV	(1<<10)
#define BGAP_TEMP_SENSOR_SOC		(1<<9)
#define BGAP_TEMP_SENSOR_EOCZ		(1<<8)
#define BGAP_TEMP_SENSOR_DTEMP_MASK 0xFF

#define MEASUREMENTS	3

typedef enum { UNDEFINED=0, BELOW_T_LOW, BETWEEN_T_LOW_AND_T_HIGH, ABOVE_T_HIGH} temp_state_t;
/*
 * omap_temp_sensor structure
 * @pdev - Platform device pointer
 * @dev - device pointer
 * @clock - Clock pointer
 * @sensor_mutex - Mutex for sysfs, irq and PM
 * @tshut_irq -  Thermal shutdown IRQ
 * @phy_base - Physical base of the temp I/O
 * @is_efuse_valid - Flag to determine if eFuse is valid or not
 * @clk_on - Manages the current clock state
 * @clk_rate - Holds current clock rate
 */
struct omap_temp_sensor {
	struct platform_device *pdev;
	struct device *dev;
	struct clk *clock;
	struct mutex sensor_mutex;
	struct spinlock lock;
	struct thermal_dev *therm_fw;
	unsigned int tshut_irq;
	unsigned long phy_base;
	int is_efuse_valid;
	u8 clk_on;
	int debug;
	int debug_temp;
	struct delayed_work work;
	int averaged_current_temperature;
	struct {
		int temperature;
		struct timeval timestamp;
	} measurement_history[MEASUREMENTS];
	int	num_measurements;
	int temp_threshold_high;
	int temp_threshold_low;
	int period;
	temp_state_t state;
};

#ifdef CONFIG_PM
static struct omap_temp_sensor *temp_sensor_pm;
#endif

#ifdef TEMP_DEBUG
static uint32_t debug_thermal;
module_param_named(temp_sensor_debug, debug_thermal, uint, 0664);
#endif
/*
 * Temperature values in milli degrees celsius ADC code values from 530 to 923
 */
/*
 *OMAP443x Temperature values in milli degress celsius ADC code values from 0 to 128
 */
static const int adc_to_temp[] = {
	-40000,	-40000,	-40000,	-40000, -40000, -40000, -40000, -40000,
	-40000,	-40000,	-40000,	-40000,	-40000,	-39000,	-36500,	-34500,
	-33000,	-31000,	-29000,	-27000,	-25000,	-23000,	-21000,	-19250,
	-17750,	-16000,	-14250,	-12750,	-11000,	 -9000,	 -7250,	 -5750,
	 -4250,	 -2500,   -750,   1000,	  2750,	  4250,	  5750,	  7500,
	  9250,	 11000,	 12750,	 14250,	 16000,	 18000,	 20000,	 22000,
	 24000,	 26000,	 27750,	 29250,	 31000,	 32750,	 34250,	 36000,
	 37750,	 39250,	 41000,	 42750,	 44250,	 46000,	 47750,	 49250,
	 51000,	 52750,	 54250,	 56000,	 57750,	 59250,	 61000,	 63000,
	 65000,	 67000,	 69000,	 70750,	 72500,	 74250,	 76000,	 77750,
	 79250,	 81000,	 82750,	 84250,	 86000,	 87750,	 89250,	 91000,
	 92750,	 94250,	 96000,	 97750,	 99250,	101000,	102750,	104250,
	106000,	108000,	110000,	112000,	114000,	116000,	117750,	119250,
	121000,	122750,	124250,	125000,	125000,	125000,	125000,	125000,
	125000, 125000,	125000,	125000,	125000,	125000,	125000,	125000,
	125000,	125000,	125000,	125000,	125000,	125000,	125000,	125000
};

static unsigned long omap_temp_sensor_readl(struct omap_temp_sensor
					    *temp_sensor, u32 reg)
{
	return omap_ctrl_readl(temp_sensor->phy_base + reg);
}

static void omap_temp_sensor_writel(struct omap_temp_sensor *temp_sensor,
				    u32 val, u32 reg)
{
	omap_ctrl_writel(val, (temp_sensor->phy_base + reg));
}

static int adc_to_temp_conversion(int adc_val)
{
	if (adc_val < OMAP_ADC_START_VALUE || adc_val > OMAP_ADC_END_VALUE) {
		pr_err("%s:Temp read is invalid %i\n", __func__, adc_val);
		return -EINVAL;
	}

	return adc_to_temp[adc_val - OMAP_ADC_START_VALUE];
}

static int temp_to_adc_conversion(long temp)
{
	int i;

	for (i = 0; i <= OMAP_ADC_END_VALUE - OMAP_ADC_START_VALUE; i++)
		if (temp < adc_to_temp[i])
			return OMAP_ADC_START_VALUE + i - 1;
	return -EINVAL;
}

static void omap443x_start_conversion(struct omap_temp_sensor *temp_sensor)
{
	omap_temp_sensor_writel( temp_sensor,
			omap_temp_sensor_readl(temp_sensor,	TEMP_SENSOR_CTRL_OFFSET) | BGAP_TEMP_SENSOR_SOC,
			TEMP_SENSOR_CTRL_OFFSET);
}

static void omap443x_end_conversion(struct omap_temp_sensor *temp_sensor)
{
	omap_temp_sensor_writel( temp_sensor,
			omap_temp_sensor_readl(temp_sensor,	TEMP_SENSOR_CTRL_OFFSET) & ~BGAP_TEMP_SENSOR_SOC,
			TEMP_SENSOR_CTRL_OFFSET);
}

static int omap443x_wait_for_end_of_conversion(struct omap_temp_sensor *temp_sensor)
{
	unsigned long temp;
	unsigned long timeout = jiffies + HZ/10;
	msleep(2); /* a conversion takes about 1.6ms, be conservative */
	do {
		temp = omap_temp_sensor_readl(temp_sensor,
				TEMP_SENSOR_CTRL_OFFSET);
		if (!(temp & BGAP_TEMP_SENSOR_EOCZ))
			return 0;
		cpu_relax();
	} while( time_before(jiffies, timeout));

printk("EOC timeout\n");
	return -ETIMEDOUT;
}

static int omap443x_wait_for_start_of_conversion(struct omap_temp_sensor *temp_sensor)
{
    unsigned long temp;
	unsigned long timeout = jiffies + HZ/10;
	do {
		temp = omap_temp_sensor_readl(temp_sensor,
				TEMP_SENSOR_CTRL_OFFSET);
		if ((temp & BGAP_TEMP_SENSOR_EOCZ))
			return 0;
		cpu_relax();
	} while( time_before(jiffies, timeout));

printk("SOC timeout\n");
	return -ETIMEDOUT;
}

static int omap_read_current_temp(struct omap_temp_sensor *temp_sensor,
		int *temperature, temp_state_t *state)
{
	int temp, ret;

	/* busy waiting here for the temperature sensor takes in the order of 1.6ms */
	omap443x_start_conversion(temp_sensor);
	ret = omap443x_wait_for_start_of_conversion(temp_sensor);
	omap443x_end_conversion(temp_sensor);
	if (ret)
		return ret;
	if ((ret = omap443x_wait_for_end_of_conversion(temp_sensor)))
		return ret;
	temp = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);
	temp &= (BGAP_TEMP_SENSOR_DTEMP_MASK);

	if (!temp_sensor->is_efuse_valid)
		pr_err_once("Invalid EFUSE, Non-trimmed BGAP, \
			Temp not accurate\n");

	if (temp < OMAP_ADC_START_VALUE || temp > OMAP_ADC_END_VALUE) {
		pr_err("%s:Invalid adc code reported by the sensor %d",
			__func__, temp);
		return -EINVAL;
	} else {
		if (temperature)
			*temperature = adc_to_temp[temp - OMAP_ADC_START_VALUE];
		if (state) {
			if (temp<temp_sensor->temp_threshold_low)
				*state = BELOW_T_LOW;
			else if (temp<temp_sensor->temp_threshold_high)
				*state = BETWEEN_T_LOW_AND_T_HIGH;
			else
				*state = ABOVE_T_HIGH;
		}
		return 0;
	}
}

static void temp_measure_work(struct work_struct *work)
{
	struct omap_temp_sensor *temp_sensor = container_of(work, struct omap_temp_sensor, work.work);
	int temp, ret;
	struct timeval current_time;
	temp_state_t state;

	if ( !temp_sensor->clk_on ) {
		pr_info("temp_sensor not enabled\n");
		goto out;
	}
	mutex_lock(&temp_sensor->sensor_mutex);
	do_gettimeofday( &current_time);
	ret = omap_read_current_temp(temp_sensor, &temp, &state);
	mutex_unlock(&temp_sensor->sensor_mutex);

	if (!ret) {
		/* move history */
		memmove( &temp_sensor->measurement_history[1], &temp_sensor->measurement_history[0],
				sizeof(temp_sensor->measurement_history[0]) * (MEASUREMENTS-1));
		temp_sensor->measurement_history[0].temperature = temp;
		temp_sensor->measurement_history[0].timestamp = current_time;
		temp_sensor->num_measurements++;
		{
			int i;
			for(i=0; i<(temp_sensor->num_measurements > MEASUREMENTS ? MEASUREMENTS : temp_sensor->num_measurements); i++)
				pr_debug("%d: %d %ld\n", i, temp_sensor->measurement_history[i].temperature,
						temp_sensor->measurement_history[i].timestamp.tv_sec*1000 + temp_sensor->measurement_history[i].timestamp.tv_usec/1000);
		}
		// FIXME: use more clever averaging, compute estimate
		if (temp < temp_sensor->averaged_current_temperature + 35000)
			temp_sensor->averaged_current_temperature = (temp * 90 + temp_sensor->averaged_current_temperature * 10)/100;
		else
			temp_sensor->averaged_current_temperature = (temp * 70 + temp_sensor->averaged_current_temperature * 30)/100;
		//if ( temp>80000 )
			pr_debug("temp now: %d agv: %d state %d\n", temp, temp_sensor->averaged_current_temperature, state);
		if (state != temp_sensor->state) { // crossed threshold
			pr_debug("crossed - was %d\n", temp_sensor->state);
			temp_sensor->therm_fw->current_temp = temp_sensor->averaged_current_temperature;
			temp_sensor->state = state;
			thermal_sensor_set_temp(temp_sensor->therm_fw);
			kobject_uevent(&temp_sensor->dev->kobj, KOBJ_CHANGE);
		}
	}

out:
	schedule_delayed_work(&temp_sensor->work, msecs_to_jiffies(temp_sensor->period));
}

static int omap_report_temp(struct thermal_dev *tdev)
{
	struct platform_device *pdev = to_platform_device(tdev->dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	return temp_sensor->averaged_current_temperature;

	/*** FIX -- 4430 thermal sensor does not like to be read more than once at a time
	int ret;

	mutex_lock(&temp_sensor->sensor_mutex);
	ret = omap_read_current_temp(temp_sensor, &temp_sensor->therm_fw->current_temp, NULL);
	mutex_unlock(&temp_sensor->sensor_mutex);

	if (!ret) {
		thermal_sensor_set_temp(temp_sensor->therm_fw);
		kobject_uevent(&temp_sensor->dev->kobj, KOBJ_CHANGE);
		return temp_sensor->therm_fw->current_temp;
	} else
		return -EINVAL;
	***/
}

static void omap_configure_temp_sensor_thresholds(struct omap_temp_sensor
						  *temp_sensor)
{
	u32 t_hot, t_cold;

	t_hot = temp_to_adc_conversion(BGAP_THRESHOLD_T_HOT);
	t_cold = temp_to_adc_conversion(BGAP_THRESHOLD_T_COLD);

	if ((t_hot == -EINVAL) || (t_cold == -EINVAL)) {
		pr_err("%s:Temp thresholds out of bounds\n", __func__);
		return;
	}
	printk("%s -> %d %d  %d %d\n", __FUNCTION__, t_hot, t_cold, BGAP_THRESHOLD_T_HOT, BGAP_THRESHOLD_T_COLD);
	temp_sensor->temp_threshold_high = t_hot;
	temp_sensor->temp_threshold_low = t_cold;
}

//FIXME: not sure continuous mode really works?
static void omap_enable_continuous_mode(struct omap_temp_sensor *temp_sensor,
					bool enable)
{
	u32 val;

	/* wait for ADC idle */
	omap443x_wait_for_end_of_conversion(temp_sensor);

	val = omap_temp_sensor_readl(temp_sensor, TEMP_SENSOR_CTRL_OFFSET);

	if (enable)
		val = val | BGAP_TEMP_SENSOR_CONTCONV;
	else
		val = val & ~(BGAP_TEMP_SENSOR_CONTCONV);

	omap_temp_sensor_writel(temp_sensor, val, TEMP_SENSOR_CTRL_OFFSET);
}

static int omap_set_thresholds(struct omap_temp_sensor *temp_sensor,
					int min, int max)
{
	int new_cold;
	int new_hot;

	/* A too low value is not acceptable for the thresholds */
	if ((min < OMAP_MIN_TEMP) || (max < OMAP_MIN_TEMP)) {
		pr_err("%s:Min or Max is invalid\n", __func__);
		return -EINVAL;
	}

	if (max < min) {
		pr_err("%s:Min is greater then the max\n", __func__);
		return -EINVAL;
	}

	new_hot = temp_to_adc_conversion(max);
	new_cold = temp_to_adc_conversion(min);

	if ((new_hot == -EINVAL) || (new_cold == -EINVAL)) {
		pr_err("%s: New thresh value is out of range\n", __func__);
		return -EINVAL;
	}
	if ( temp_sensor->temp_threshold_high == new_hot &&
			temp_sensor->temp_threshold_low == new_cold	)
		return 0;

	printk("%s %d %d  %d %d\n", __FUNCTION__, new_hot, new_cold, max, min);
	temp_sensor->temp_threshold_high = new_hot;
	temp_sensor->temp_threshold_low = new_cold;

	/* trigger measurement now */
	cancel_delayed_work(&temp_sensor->work);
	schedule_delayed_work(&temp_sensor->work, 0);

	return 0;
}

static int omap_set_temp_thresh(struct thermal_dev *tdev, int min, int max)
{
	struct platform_device *pdev = to_platform_device(tdev->dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	int ret;

	mutex_lock(&temp_sensor->sensor_mutex);
	ret = omap_set_thresholds(temp_sensor, min, max);
	mutex_unlock(&temp_sensor->sensor_mutex);

	return ret;
}

static int omap_update_measure_rate(struct omap_temp_sensor *temp_sensor,
					int rate)
{
	temp_sensor->period = rate;
	return rate;
}

static int omap_set_measuring_rate(struct thermal_dev *tdev, int rate)
{
	struct platform_device *pdev = to_platform_device(tdev->dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	int set_rate;

	mutex_lock(&temp_sensor->sensor_mutex);
	set_rate = omap_update_measure_rate(temp_sensor, rate);
	mutex_unlock(&temp_sensor->sensor_mutex);

	return set_rate;
}

/*
 * sysfs hook functions
 */
static ssize_t show_temp_thresholds(struct device *dev,
			struct device_attribute *devattr,
			char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	int min_temp;
	int max_temp;

	max_temp = adc_to_temp_conversion(temp_sensor->temp_threshold_high);
	min_temp = adc_to_temp_conversion(temp_sensor->temp_threshold_low);

	return sprintf(buf, "Min %d\nMax %d\n", min_temp, max_temp);
}

static ssize_t set_temp_thresholds(struct device *dev,
				 struct device_attribute *devattr,
				 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	unsigned first_val;
	unsigned second_val;
	int min_thresh = 0;
	int max_thresh = 0;
	char first[30];
	char second[30];

	if (sscanf(buf, "%s %i %s %i", first, &first_val,
			second, &second_val) != 4) {
		pr_err("%s:unable to parse input\n", __func__);
		return -EINVAL;
	}
	if (!strcmp(first, "min"))
		min_thresh = first_val;
	if (!strcmp(second, "max"))
		max_thresh = second_val;

	pr_info("%s: Min thresh is %i Max thresh is %i\n",
		__func__, min_thresh, max_thresh);
	mutex_lock(&temp_sensor->sensor_mutex);
	omap_set_thresholds(temp_sensor, min_thresh, max_thresh);
	mutex_unlock(&temp_sensor->sensor_mutex);

	return count;
}

static ssize_t show_update_rate(struct device *dev,
				struct device_attribute *devattr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", temp_sensor->period );
}

static ssize_t set_update_rate(struct device *dev,
			       struct device_attribute *devattr,
			       const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	long val;

	if (strict_strtol(buf, 10, &val)) {
		count = -EINVAL;
		goto out;
	}

	mutex_lock(&temp_sensor->sensor_mutex);

	omap_update_measure_rate(temp_sensor, (int)val);

out:
	mutex_unlock(&temp_sensor->sensor_mutex);
	return count;
}

static int omap_temp_sensor_read_temp(struct device *dev,
				      struct device_attribute *devattr,
				      char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	int temp = 0, ret;

	temp =  temp_sensor->averaged_current_temperature; //FIX only driver thread read the sensor

#ifdef TEMP_DEBUG
	if (temp_sensor->debug) {
		temp = temp_sensor->debug_temp;
		goto out;
	}
#endif
	/*** FIX -- 4430 thermal sensor does not like to be read more than once at a time 
	mutex_lock(&temp_sensor->sensor_mutex);
	ret = omap_read_current_temp(temp_sensor, &temp, NULL);
	mutex_unlock(&temp_sensor->sensor_mutex);
	***/
out:
	return sprintf(buf, "%d\n", temp);
}

#ifdef TEMP_DEBUG
static ssize_t show_temp_user_space(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", temp_sensor->debug_temp);
}

static ssize_t set_temp_user_space(struct device *dev,
			struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	long val;

	if (strict_strtol(buf, 10, &val)) {
		count = -EINVAL;
		goto out;
	}

	if (!temp_sensor->debug && debug_thermal) {
		pr_info("%s: Going into debug mode\n", __func__);
#if 0
//FIXME
		disable_irq_nosync(temp_sensor->irq);
#endif
		temp_sensor->debug = 1;
	} else if (temp_sensor->debug && !debug_thermal) {
		pr_info("%s:Reenable temp sensor dbg mode %i\n",
			__func__, temp_sensor->debug);
#if 0
//FIXME
		enable_irq(temp_sensor->irq);
#endif
		temp_sensor->debug = 0;
		temp_sensor->debug_temp = 0;
	} else if ((temp_sensor->debug == 0) &&
			(debug_thermal == 0)) {
		pr_info("%s:Not in debug mode\n", __func__);
		goto out;
	} else {
		pr_info("%s:Debug mode %i and setting temp to %li\n",
			__func__, temp_sensor->debug, val);
	}

	/* Set new temperature */
	temp_sensor->debug_temp = val;

	temp_sensor->therm_fw->current_temp = val;
	thermal_sensor_set_temp(temp_sensor->therm_fw);
	/* Send a kobj_change */
	kobject_uevent(&temp_sensor->dev->kobj, KOBJ_CHANGE);

out:
	return count;
}
#endif


#ifdef TEMP_DEBUG
static DEVICE_ATTR(debug_user, S_IWUSR | S_IRUGO, show_temp_user_space,
			  set_temp_user_space);
#endif

static DEVICE_ATTR(temp1_input, S_IRUGO, omap_temp_sensor_read_temp,
			  NULL);
static DEVICE_ATTR(temp_thresh, S_IWUSR | S_IRUGO, show_temp_thresholds,
			  set_temp_thresholds);
static DEVICE_ATTR(update_rate, S_IWUSR | S_IRUGO, show_update_rate,
			  set_update_rate);

static struct attribute *omap_temp_sensor_attributes[] = {
	&dev_attr_temp1_input.attr,
	&dev_attr_temp_thresh.attr,
#ifdef TEMP_DEBUG
	&dev_attr_debug_user.attr,
#endif
	&dev_attr_update_rate.attr,
	NULL
};

static const struct attribute_group omap_temp_sensor_group = {
	.attrs = omap_temp_sensor_attributes,
};

static int omap_temp_sensor_enable(struct omap_temp_sensor *temp_sensor)
{
	u32 temp;
	u32 ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&temp_sensor->lock, flags);

	if (temp_sensor->clk_on) {
		pr_debug("%s:clock already on\n", __func__);
		goto out;
	}

	ret = pm_runtime_get_sync(&temp_sensor->pdev->dev);
	if (ret) {
		pr_err("%s:get sync failed\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	temp = omap_temp_sensor_readl(temp_sensor,
					TEMP_SENSOR_CTRL_OFFSET);
	temp &= ~(BGAP_TEMPSOFF_MASK);

	/* write BGAP_TEMPSOFF should be reset to 0 */
	omap_temp_sensor_writel(temp_sensor, temp,
				TEMP_SENSOR_CTRL_OFFSET);
	temp_sensor->clk_on = 1;

out:
	spin_unlock_irqrestore(&temp_sensor->lock, flags);
	return ret;
}


static int omap_temp_sensor_disable(struct omap_temp_sensor *temp_sensor)
{
	u32 temp;
	u32 ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&temp_sensor->lock, flags);

	if (!temp_sensor->clk_on) {
		pr_debug("%s:clock already off\n", __func__);
		goto out;
	}
	temp = omap_temp_sensor_readl(temp_sensor,
				TEMP_SENSOR_CTRL_OFFSET);
	temp |= BGAP_TEMPSOFF_MASK;

	/* write BGAP_TEMPSOFF should be set to 1 before gating clock */
	omap_temp_sensor_writel(temp_sensor, temp,
				TEMP_SENSOR_CTRL_OFFSET);

	/* FIXME wait till the clean stop bit is set */
	omap443x_wait_for_end_of_conversion(temp_sensor);

	/* Gate the clock */
	ret = pm_runtime_put_sync_suspend(&temp_sensor->pdev->dev);
	if (ret) {
		pr_err("%s:put sync failed\n", __func__);
		ret = -EINVAL;
		goto out;
	}
	temp_sensor->clk_on = 0;

out:
	spin_unlock_irqrestore(&temp_sensor->lock, flags);
	return ret;
}

static irqreturn_t omap_tshut_irq_handler(int irq, void *data)
{
	struct omap_temp_sensor *temp_sensor = (struct omap_temp_sensor *)data;

	/* Need to handle thermal mgmt in bootloader
	 * to avoid restart again at kernel level
	 */
	if (temp_sensor->is_efuse_valid) {
		pr_emerg("%s: Thermal shutdown reached rebooting device\n",
			__func__);
		kernel_restart(NULL);
	} else {
		pr_err("%s:Invalid EFUSE, Non-trimmed BGAP\n", __func__);
	}

	return IRQ_HANDLED;
}



static struct thermal_dev_ops omap_sensor_ops = {
	.report_temp = omap_report_temp,
	.set_temp_thresh = omap_set_temp_thresh,
	.set_temp_report_rate = omap_set_measuring_rate,
};

static int __devinit omap_temp_sensor_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct omap_temp_sensor_pdata *pdata = pdev->dev.platform_data;
	struct omap_temp_sensor *temp_sensor;
	struct resource *mem;
	int ret = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "%s: platform data missing\n", __func__);
		return -EINVAL;
	}

	temp_sensor = kzalloc(sizeof(struct omap_temp_sensor), GFP_KERNEL);
	if (!temp_sensor)
		return -ENOMEM;

	spin_lock_init(&temp_sensor->lock);
	mutex_init(&temp_sensor->sensor_mutex);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "%s:no mem resource\n", __func__);
		ret = -EINVAL;
		goto plat_res_err;
	}

	ret = gpio_request_one(OMAP_TSHUT_GPIO, GPIOF_DIR_IN,
		"thermal_shutdown");
	if (ret) {
		dev_err(&pdev->dev, "%s: Could not get tshut_gpio\n",
			__func__);
		goto tshut_gpio_req_err;
	}

	temp_sensor->tshut_irq = gpio_to_irq(OMAP_TSHUT_GPIO);
	if (temp_sensor->tshut_irq < 0) {
		dev_err(&pdev->dev, "%s:Cannot get thermal shutdown irq\n",
			__func__);
		ret = -EINVAL;
		goto get_tshut_irq_err;
	}


	temp_sensor->phy_base = pdata->offset;
	temp_sensor->pdev = pdev;
	temp_sensor->dev = dev;

	pm_runtime_enable(dev);
	pm_runtime_irq_safe(dev);


	kobject_uevent(&pdev->dev.kobj, KOBJ_ADD);
	platform_set_drvdata(pdev, temp_sensor);


	/*
	 * check if the efuse has a non-zero value if not
	 * it is an untrimmed sample and the temperatures
	 * may not be accurate */
	if (omap_readl(OMAP4_CTRL_MODULE_CORE +
			OMAP4_CTRL_MODULE_CORE_STD_FUSE_OPP_BGAP))
		temp_sensor->is_efuse_valid = 1;

	temp_sensor->clock = clk_get(&temp_sensor->pdev->dev, "fclk");
	if (IS_ERR(temp_sensor->clock)) {
		ret = PTR_ERR(temp_sensor->clock);
		pr_err("%s:Unable to get fclk: %d\n", __func__, ret);
		ret = -EINVAL;
		goto clk_get_err;
	}

	ret = omap_temp_sensor_enable(temp_sensor);
	if (ret) {
		dev_err(&pdev->dev, "%s:Cannot enable temp sensor\n", __func__);
		goto sensor_enable_err;
	}

	temp_sensor->therm_fw = kzalloc(sizeof(struct thermal_dev), GFP_KERNEL);
	if (temp_sensor->therm_fw) {
		temp_sensor->therm_fw->name = "omap443x_ondie_sensor";
		temp_sensor->therm_fw->domain_name = "cpu";
		temp_sensor->therm_fw->dev = temp_sensor->dev;
		temp_sensor->therm_fw->dev_ops = &omap_sensor_ops;
		thermal_sensor_dev_register(temp_sensor->therm_fw);
	} else {
		dev_err(&pdev->dev, "%s:Cannot alloc memory for thermal fw\n",
			__func__);
		ret = -ENOMEM;
		goto therm_fw_alloc_err;
	}

	omap_enable_continuous_mode(temp_sensor, 0);
	omap_configure_temp_sensor_thresholds(temp_sensor);

	/* Wait till the first conversion is done wait for at least 1ms */
	mdelay(2);

	/* Read the temperature once */
	omap_report_temp(temp_sensor->therm_fw);

	/* Set 2 seconds time as default counter */
	temp_sensor->period = 2000;

	ret = sysfs_create_group(&pdev->dev.kobj,
				 &omap_temp_sensor_group);
	if (ret) {
		dev_err(&pdev->dev, "could not create sysfs files\n");
		goto sysfs_create_err;
	}

	ret = request_threaded_irq(temp_sensor->tshut_irq, NULL,
			omap_tshut_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"tshut", (void *)temp_sensor);
	if (ret) {
		dev_err(&pdev->dev, "Request threaded irq failed for TSHUT.\n");
		goto tshut_irq_req_err;
	}

	INIT_DELAYED_WORK(&temp_sensor->work, temp_measure_work);
	schedule_delayed_work(&temp_sensor->work, msecs_to_jiffies(temp_sensor->period));

	dev_info(&pdev->dev, "%s : '%s'\n", temp_sensor->therm_fw->name,
			pdata->name);

	temp_sensor_pm = temp_sensor;
	return 0;

tshut_irq_req_err:
	kobject_uevent(&temp_sensor->dev->kobj, KOBJ_REMOVE);
	sysfs_remove_group(&temp_sensor->dev->kobj, &omap_temp_sensor_group);
sysfs_create_err:
	thermal_sensor_dev_unregister(temp_sensor->therm_fw);
	kfree(temp_sensor->therm_fw);
	if (temp_sensor->clock)
		clk_put(temp_sensor->clock);
	platform_set_drvdata(pdev, NULL);
therm_fw_alloc_err:
	omap_temp_sensor_disable(temp_sensor);
sensor_enable_err:
	clk_put(temp_sensor->clock);
clk_get_err:
	pm_runtime_disable(&pdev->dev);
get_tshut_irq_err:
	gpio_free(OMAP_TSHUT_GPIO);
tshut_gpio_req_err:
plat_res_err:
	mutex_destroy(&temp_sensor->sensor_mutex);
	kfree(temp_sensor);
	return ret;
}

static int __devexit omap_temp_sensor_remove(struct platform_device *pdev)
{
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	thermal_sensor_dev_unregister(temp_sensor->therm_fw);
	kfree(temp_sensor->therm_fw);
	kobject_uevent(&temp_sensor->dev->kobj, KOBJ_REMOVE);
	sysfs_remove_group(&temp_sensor->dev->kobj, &omap_temp_sensor_group);
	omap_temp_sensor_disable(temp_sensor);
	if (temp_sensor->clock)
		clk_put(temp_sensor->clock);
	platform_set_drvdata(pdev, NULL);
	mutex_destroy(&temp_sensor->sensor_mutex);
	kfree(temp_sensor);

	return 0;
}

#ifdef CONFIG_PM

static int omap_temp_sensor_suspend(struct platform_device *pdev,
				    pm_message_t state)
{
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	omap_temp_sensor_disable(temp_sensor);

	return 0;
}

static int omap_temp_sensor_resume(struct platform_device *pdev)
{
	struct omap_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	omap_temp_sensor_enable(temp_sensor);

	return 0;
}

void omap443x_temp_sensor_idle(int idle_state)
{
	if (!cpu_is_omap446x())
		return;

	if (idle_state)
		omap_temp_sensor_disable(temp_sensor_pm);
	else
		omap_temp_sensor_enable(temp_sensor_pm);
}

#else
omap_temp_sensor_suspend NULL
omap_temp_sensor_resume NULL

#endif /* CONFIG_PM */

static int omap_temp_sensor_runtime_suspend(struct device *dev)
{
	struct omap_temp_sensor *temp_sensor =
			platform_get_drvdata(to_platform_device(dev));

	// TODO context?
	return 0;
}

static int omap_temp_sensor_runtime_resume(struct device *dev)
{
	struct omap_temp_sensor *temp_sensor =
			platform_get_drvdata(to_platform_device(dev));
	if (omap_pm_was_context_lost(dev)) {
	// TODO context?
		
	}
	return 0;
}

static const struct dev_pm_ops omap_temp_sensor_dev_pm_ops = {
	.runtime_suspend = omap_temp_sensor_runtime_suspend,
	.runtime_resume = omap_temp_sensor_runtime_resume,
};

static struct platform_driver omap_temp_sensor_driver = {
	.probe = omap_temp_sensor_probe,
	.remove = omap_temp_sensor_remove,
	.suspend = omap_temp_sensor_suspend,
	.resume = omap_temp_sensor_resume,
	.driver = {
		.name = "omap_temp_sensor",
		.pm = &omap_temp_sensor_dev_pm_ops,
	},
};

int __init omap443x_temp_sensor_init(void)
{
	if (!cpu_is_omap443x())
		return 0;

	return platform_driver_register(&omap_temp_sensor_driver);
}

static void __exit omap443x_temp_sensor_exit(void)
{
	platform_driver_unregister(&omap_temp_sensor_driver);
}

module_init(omap443x_temp_sensor_init);
module_exit(omap443x_temp_sensor_exit);

MODULE_DESCRIPTION("OMAP443X Temperature Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
