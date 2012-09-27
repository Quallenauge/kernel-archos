/*
 * drivers/thermal/omap_die_governor.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Dan Murphy <DMurphy@ti.com>
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
*/

#include <linux/err.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>

#include <linux/thermal_framework.h>
#include <plat/tmp102_temp_sensor.h>
#include <plat/board.h>
#include <plat/cpu.h>

/* CPU Zone information */
#define FATAL_ZONE	5
#define PANIC_ZONE	4
#define ALERT_ZONE	3
#define MONITOR_ZONE	2
#define SAFE_ZONE	1
#define NO_ACTION	0

/* this are defaults and can be overwritten in the board file */
#define OMAP_FATAL_TEMP 125000
#define OMAP_PANIC_TEMP 110000
#define OMAP_ALERT_TEMP 100000
#define OMAP_MONITOR_TEMP 85000
#define OMAP_SAFE_TEMP  25000

/* no cooling level beyond this - if not enough, we are in trouble */
#define MAX_COOLING_LEVEL 4

#define HYSTERESIS_VALUE 2000
#define NORMAL_TEMP_MONITORING_RATE 1000
#define FAST_TEMP_MONITORING_RATE 250
#define DECREASE_MPU_FREQ_PERIOD 2000

/* parameters for extrapolation of hot-spot temperature */
#define OMAP_GRADIENT_SLOPE_4430    500	/* FIXME, get better parameters */
#define OMAP_GRADIENT_CONST_4430 -12500 //30000 at 85deg. - nothing at 25deg.
#define OMAP_GRADIENT_SLOPE_4460    348
#define OMAP_GRADIENT_CONST_4460  -9301
#define OMAP_GRADIENT_SLOPE_4470    308
#define OMAP_GRADIENT_CONST_4470  -7896

/* PCB sensor calculation constants */
#define OMAP_GRADIENT_SLOPE_W_PCB_4460  1142
#define OMAP_GRADIENT_CONST_W_PCB_4460  -393
#define OMAP_GRADIENT_SLOPE_W_PCB_4470  1063
#define OMAP_GRADIENT_CONST_W_PCB_4470  -477
#define AVERAGE_NUMBER	      20

struct omap_die_governor {
	struct thermal_dev *temp_sensor;
	void (*update_temp_thresh) (struct thermal_dev *, int min, int max);
	struct mutex governor_mutex;
	int report_rate;
	int panic_zone_reached;
	int decrease_mpu_freq_period;
	int cooling_level;
	int hotspot_temp_upper;
	int hotspot_temp_lower;
	int pcb_panic_zone_reached;
	int pcb_cooling_level;
	int pcb_temp_upper;
	int pcb_temp_lower;
	int pcb_temp;
	int sensor_temp;
	int absolute_delta;
	int average_period;
	int avg_cpu_sensor_temp;
	int avg_is_valid;
	int use_separate_pcb_zone_definition;
	int pcb_panic_guard;
	int pcb_panic_guard_treshold;
	struct delayed_work average_cpu_sensor_work;
	struct delayed_work decrease_mpu_freq_work;

	int fatal_temp;
	int panic_temp;
	int alert_temp;
	int monitor_temp;
	int safe_temp;

	int pcb_fatal_temp;
	int pcb_panic_temp;
	int pcb_alert_temp;
	int pcb_monitor_temp;
	int pcb_safe_temp;

	int hysteresis_value;
	int pcb_hysteresis_value;
	int normal_temp_monitoring_rate;
	int fast_temp_monitoring_rate;

	int gradient_slope;
	int gradient_const;
	int gradient_slope_w_pcb;
	int gradient_const_w_pcb;
};

static struct thermal_dev *therm_fw;
static struct omap_die_governor *omap_gov;
static struct thermal_dev *pcb_sensor;
static int cpu_sensor_temp_table[AVERAGE_NUMBER];
static struct kobject * sysfs_kobj;

static LIST_HEAD(cooling_agents);

/**
 * DOC: Introduction
 * =================
 * The OMAP On-Die Temperature governor maintains the policy for the CPU
 * on die temperature sensor.  The main goal of the governor is to receive
 * a temperature report from a thermal sensor and calculate the current
 * thermal zone.  Each zone will sort through a list of cooling agents
 * passed in to determine the correct cooling stategy that will cool the
 * device appropriately for that zone.
 *
 * The temperature that is reported by the temperature sensor is first
 * calculated to an OMAP hot spot temperature.
 * This takes care of the existing temperature gradient between
 * the OMAP hot spot and the on-die temp sensor.
 * Note: The "slope" parameter is multiplied by 1000 in the configuration
 * file to avoid using floating values.
 * Note: The "offset" is defined in milli-celsius degrees.
 *
 * Next the hot spot temperature is then compared to thresholds to determine
 * the proper zone.
 *
 * There are 5 zones identified:
 *
 * FATAL_ZONE: This zone indicates that the on-die temperature has reached
 * a point where the device needs to be rebooted and allow ROM or the bootloader
 * to run to allow the device to cool.
 *
 * PANIC_ZONE: This zone indicates a near fatal temperature is approaching
 * and should impart all neccessary cooling agent to bring the temperature
 * down to an acceptable level.
 *
 * ALERT_ZONE: This zone indicates that die is at a level that may need more
 * agressive cooling agents to keep or lower the temperature.
 *
 * MONITOR_ZONE: This zone is used as a monitoring zone and may or may not use
 * cooling agents to hold the current temperature.
 *
 * SAFE_ZONE: This zone is optimal thermal zone.  It allows the device to
 * run at max levels without imparting any cooling agent strategies.
 *
 * NO_ACTION: Means just that.  There was no action taken based on the current
 * temperature sent in.
*/

/**
 * omap_update_report_rate() - Updates the temperature sensor monitoring rate
 *
 * @new_rate: New measurement rate for the temp sensor
 *
 * No return
 */
static void omap_update_report_rate(struct thermal_dev *temp_sensor,
				int new_rate)
{
	if (omap_gov->report_rate == -EOPNOTSUPP) {
		pr_err("%s:Updating the report rate is not supported\n",
			__func__);
		return;
	}

	if (omap_gov->report_rate != new_rate)
		omap_gov->report_rate =
			thermal_update_temp_rate(temp_sensor, new_rate);
}

/*
 * convert_omap_sensor_temp_to_hotspot_temp() -Convert the temperature from the
 *		OMAP on-die temp sensor into OMAP hot spot temperature.
 *		This takes care of the existing temperature gradient between
 *		the OMAP hot spot and the on-die temp sensor.
 *		When PCB sensor is used, the temperature gradient is computed
 *		from PCB and averaged on-die sensor temperatures.
 *
 * @sensor_temp: Raw temperature reported by the OMAP die temp sensor
 *
 * Returns the calculated hot spot temperature for the zone calculation
 */
static signed int convert_omap_sensor_temp_to_hotspot_temp(int sensor_temp)
{
	int absolute_delta;

	if (!omap_gov->use_separate_pcb_zone_definition && pcb_sensor && (omap_gov->avg_is_valid == 1)) {
		omap_gov->pcb_temp = thermal_request_temp(pcb_sensor);
		if (omap_gov->pcb_temp < 0)
			return sensor_temp + omap_gov->absolute_delta;

		absolute_delta = (
			((omap_gov->avg_cpu_sensor_temp - omap_gov->pcb_temp) *
			omap_gov->gradient_slope_w_pcb / 1000) +
			omap_gov->gradient_const_w_pcb);

		/* Ensure that this formula never returns negative value */
		if (absolute_delta < 0)
			absolute_delta = 0;
	} else {
		absolute_delta = (
			(sensor_temp * omap_gov->gradient_slope / 1000) +
			omap_gov->gradient_const);
	}

	omap_gov->absolute_delta = absolute_delta;
	pr_debug("%s:sensor %d avg sensor %d pcb %d, delta %d hot spot %d\n",
			__func__, sensor_temp, omap_gov->avg_cpu_sensor_temp,
			omap_gov->pcb_temp, omap_gov->absolute_delta,
			sensor_temp + absolute_delta);

	return sensor_temp + absolute_delta;
}

/*
 * hotspot_temp_to_omap_sensor_temp() - Convert the temperature from
 *		the OMAP hot spot temperature into the OMAP on-die temp sensor.
 *		This is useful to configure the thresholds at OMAP on-die
 *		sensor level. This takes care of the existing temperature
 *		gradient between the OMAP hot spot and the on-die temp sensor.
 *
 * @hot_spot_temp: Hot spot temperature to the be calculated to CPU on-die
 *		temperature value.
 *
 * Returns the calculated cpu on-die temperature.
 */

static signed hotspot_temp_to_sensor_temp(int hot_spot_temp)
{
	if (!omap_gov->use_separate_pcb_zone_definition && pcb_sensor && (omap_gov->avg_is_valid == 1))
		return hot_spot_temp - omap_gov->absolute_delta;
	else
		return ((hot_spot_temp - omap_gov->gradient_const) * 1000) /
			(1000 + omap_gov->gradient_slope);
}

static void start_panic_guard(void)
{
	cancel_delayed_work(&omap_gov->decrease_mpu_freq_work);
	if (omap_gov->decrease_mpu_freq_period)
		schedule_delayed_work(&omap_gov->decrease_mpu_freq_work,
		      msecs_to_jiffies(omap_gov->decrease_mpu_freq_period));
}

/**
 * omap_safe_zone() - THERMAL "Safe Zone" definition:
 *  - No constraint about Max CPU frequency
 *  - No constraint about CPU freq governor
 *  - Normal temperature monitoring rate
 *
 * @cooling_list: The list of cooling devices available to cool the zone
 * @cpu_temp:	The current adjusted CPU temperature
 *
 * Returns 0 on success and -ENODEV for no cooling devices available to cool
 */
static int omap_safe_zone(struct list_head *cooling_list, int cpu_temp)
{
	int die_temp_upper = 0;
	int die_temp_lower = 0;

	pr_debug("%s:hot spot temp %d\n", __func__, cpu_temp);

	omap_gov->cooling_level = 0;
	omap_gov->hotspot_temp_lower = omap_gov->safe_temp;
	omap_gov->hotspot_temp_upper = omap_gov->monitor_temp;
	die_temp_lower = hotspot_temp_to_sensor_temp(
		omap_gov->hotspot_temp_lower);
	die_temp_upper = hotspot_temp_to_sensor_temp(
		omap_gov->hotspot_temp_upper);
	thermal_update_temp_thresholds(omap_gov->temp_sensor,
		die_temp_lower, die_temp_upper);
	omap_update_report_rate(omap_gov->temp_sensor,
		omap_gov->fast_temp_monitoring_rate);
	if (pcb_sensor)
		omap_gov->average_period = omap_gov->normal_temp_monitoring_rate;

	omap_gov->panic_zone_reached = 0;
	cancel_delayed_work(&omap_gov->decrease_mpu_freq_work);


	return 0;
}

/**
 * omap_monitor_zone() - Current device is in a situation that requires
 *			monitoring and may impose agents to keep the device
 *			at a steady state temperature or moderately cool the
 *			device.
 *
 * @cooling_list: The list of cooling devices available to cool the zone
 * @cpu_temp:	The current adjusted CPU temperature
 *
 * Returns 0 on success and -ENODEV for no cooling devices available to cool
 */
static int omap_monitor_zone(struct list_head *cooling_list, int cpu_temp)
{
	int die_temp_upper = 0;
	int die_temp_lower = 0;

	pr_debug("%s:hot spot temp %d\n", __func__, cpu_temp);
    
	omap_gov->cooling_level = 0;
	omap_gov->hotspot_temp_lower =
		(omap_gov->monitor_temp - omap_gov->hysteresis_value);
	omap_gov->hotspot_temp_upper = omap_gov->alert_temp;
	die_temp_lower = hotspot_temp_to_sensor_temp(
		omap_gov->hotspot_temp_lower);
	die_temp_upper = hotspot_temp_to_sensor_temp(
		omap_gov->hotspot_temp_upper);
	thermal_update_temp_thresholds(omap_gov->temp_sensor,
		die_temp_lower, die_temp_upper);
	omap_update_report_rate(omap_gov->temp_sensor,
		omap_gov->fast_temp_monitoring_rate);
	if (pcb_sensor)
		omap_gov->average_period = omap_gov->fast_temp_monitoring_rate;

	omap_gov->panic_zone_reached = 0;
	cancel_delayed_work(&omap_gov->decrease_mpu_freq_work);

	return 0;
}
/**
 * omap_alert_zone() - "Alert Zone" definition:
 *	- If the Panic Zone has never been reached, then
 *	- Define constraint about Max CPU frequency
 *		if Current frequency < Max frequency,
 *		then Max frequency = Current frequency
 *	- Else keep the constraints set previously until
 *		temperature falls to safe zone
 *
 * @cooling_list: The list of cooling devices available to cool the zone
 * @cpu_temp:	The current adjusted CPU temperature
 *
 * Returns 0 on success and -ENODEV for no cooling devices available to cool
 */
static int omap_alert_zone(struct list_head *cooling_list, int cpu_temp)
{
	int die_temp_upper = 0;
	int die_temp_lower = 0;

	pr_debug("%s:hot spot temp %d\n", __func__, cpu_temp);

	if (omap_gov->panic_zone_reached == 0) {
		/* Temperature rises and enters into alert zone */
		omap_gov->cooling_level = 0;
	} else {
		if (omap_gov->panic_zone_reached > 0)
			omap_gov->panic_zone_reached--;
		if (omap_gov->cooling_level > 0) {
			omap_gov->cooling_level--;
			pr_info("lowering CPU cooling level to %d\n", omap_gov->cooling_level);
		}
	}

	omap_gov->hotspot_temp_lower =
		(omap_gov->alert_temp - omap_gov->hysteresis_value);
	omap_gov->hotspot_temp_upper = omap_gov->panic_temp;
	die_temp_lower = hotspot_temp_to_sensor_temp(
		omap_gov->hotspot_temp_lower);
	die_temp_upper = hotspot_temp_to_sensor_temp(
		omap_gov->hotspot_temp_upper);
	thermal_update_temp_thresholds(omap_gov->temp_sensor,
		die_temp_lower, die_temp_upper);
	omap_update_report_rate(omap_gov->temp_sensor,
		omap_gov->fast_temp_monitoring_rate);
	if (pcb_sensor)
		omap_gov->average_period = omap_gov->fast_temp_monitoring_rate;

	cancel_delayed_work(&omap_gov->decrease_mpu_freq_work);

	return 0;
}

/**
 * omap_panic_zone() - Force CPU frequency to a "safe frequency"
 *     . Force the CPU frequency to a “safe” frequency
 *     . Limit max CPU frequency to the “safe” frequency
 *
 * @cooling_list: The list of cooling devices available to cool the zone
 * @cpu_temp:	The current adjusted CPU temperature
 *
 * Returns 0 on success and -ENODEV for no cooling devices available to cool
 */
static int omap_panic_zone(struct list_head *cooling_list, int cpu_temp)
{
	int die_temp_upper = 0;
	int die_temp_lower = 0;

	pr_info("%s:hot spot temp %d\n", __func__, cpu_temp);

	omap_gov->cooling_level++;
	if (omap_gov->cooling_level > MAX_COOLING_LEVEL)
		omap_gov->cooling_level = MAX_COOLING_LEVEL;
	omap_gov->panic_zone_reached++;
	pr_info("%s: CPU Panic zone reached %i times\n",
		__func__, omap_gov->panic_zone_reached);
	omap_gov->hotspot_temp_lower =
		(omap_gov->panic_temp - omap_gov->hysteresis_value);

	/*
	 * Set the threshold window to below fatal.  This way the
	 * governor can manage the thermal if the temp should rise
	 * while throttling.  We need to be aggressive with throttling
	 * should we reach this zone.
	 */
	omap_gov->hotspot_temp_upper = ( (10000 / MAX_COOLING_LEVEL) *
		omap_gov->cooling_level) + omap_gov->panic_temp;

	if ( cpu_temp > omap_gov->hotspot_temp_upper) {
		int cooling_level_needed;
		cooling_level_needed = (cpu_temp - omap_gov->panic_temp) /
				(10000 / MAX_COOLING_LEVEL) + 1;
		omap_gov->cooling_level = cooling_level_needed;
		if (omap_gov->cooling_level > MAX_COOLING_LEVEL)
			omap_gov->cooling_level = MAX_COOLING_LEVEL;
		omap_gov->hotspot_temp_upper = ( (10000 / MAX_COOLING_LEVEL) *
			omap_gov->cooling_level) + omap_gov->panic_temp;
	}
	if (omap_gov->hotspot_temp_upper >= omap_gov->fatal_temp) {
		pr_err("hotspot_temp_upper exceeds fatal_temp limit %d %d\n", omap_gov->hotspot_temp_upper, omap_gov->fatal_temp);
		omap_gov->hotspot_temp_upper = omap_gov->fatal_temp;
	}

	/* finally set cooling level */
	pr_info("set CPU cooling level to %d\n", omap_gov->cooling_level);

	die_temp_lower = hotspot_temp_to_sensor_temp(
		omap_gov->hotspot_temp_lower);
	die_temp_upper = hotspot_temp_to_sensor_temp(
		omap_gov->hotspot_temp_upper);
	thermal_update_temp_thresholds(omap_gov->temp_sensor,
		die_temp_lower, die_temp_upper);
	omap_update_report_rate(omap_gov->temp_sensor,
		omap_gov->fast_temp_monitoring_rate);
	if (pcb_sensor)
		omap_gov->average_period = omap_gov->fast_temp_monitoring_rate;
	
	if (omap_gov->panic_zone_reached)
		start_panic_guard();
	else
		cancel_delayed_work(&omap_gov->decrease_mpu_freq_work);

	return 0;
}

/**
 * omap_fatal_zone() - Shut-down the system to ensure OMAP Junction
 *			temperature decreases enough
 *
 * @cpu_temp:	The current adjusted CPU temperature
 *
 * No return forces a restart of the system
 */
static void omap_fatal_zone(int cpu_temp)
{
	pr_emerg("%s:FATAL ZONE (hot spot temp: %i)\n", __func__, cpu_temp);

	kernel_restart(NULL);
}

static int omap_cpu_thermal_manager(struct list_head *cooling_list, int temp)
{
	struct thermal_dev *cooling_dev, *tmp;
	int cpu_temp;
	int ret = NO_ACTION;

	cpu_temp = convert_omap_sensor_temp_to_hotspot_temp(temp);

	mutex_lock(&omap_gov->governor_mutex);
	pr_debug("%s:sensor %d avg sensor %d pcb %d, delta %d hot spot %d\n",
			__func__, temp, omap_gov->avg_cpu_sensor_temp,
			omap_gov->pcb_temp, omap_gov->absolute_delta,
			cpu_temp);

	/* TO DO: need to build an algo to find the right cooling agent */
	list_for_each_entry_safe(cooling_dev, tmp, cooling_list, node) {
		if (cooling_dev->dev_ops &&
			cooling_dev->dev_ops->cool_device) {
			/* TO DO: Add cooling agents to a list here */
			list_add(&cooling_dev->node, &cooling_agents);
			break;
		} else {
			pr_info("%s:Cannot find cool_device for %s\n",
				__func__, cooling_dev->name);
		}
	}

	if (list_empty(&cooling_agents)) {
		pr_err("%s: No Cooling devices registered\n",
			__func__);
		mutex_unlock(&omap_gov->governor_mutex);
		return -ENODEV;
	}

	if (cpu_temp >= omap_gov->fatal_temp) {
		omap_fatal_zone(cpu_temp);
		ret = FATAL_ZONE;
	} else if (cpu_temp >= omap_gov->panic_temp) {
		omap_panic_zone(cooling_list, cpu_temp);
		ret = PANIC_ZONE;
	} else if (cpu_temp < (omap_gov->panic_temp - omap_gov->hysteresis_value)) {
		if (cpu_temp >= omap_gov->alert_temp) {
			omap_alert_zone(cooling_list, cpu_temp);
			ret = ALERT_ZONE;
		} else if (cpu_temp < (omap_gov->alert_temp - omap_gov->hysteresis_value)) {
			if (cpu_temp >= omap_gov->monitor_temp) {
				omap_monitor_zone(cooling_list, cpu_temp);
				ret = MONITOR_ZONE;
			} else {
				/*
				 * this includes the case where :
				 * (omap_gov->monitor_temp - omap_gov->hysteresis_value) <= T
				 * && T < omap_gov->monitor_temp
				 */
				omap_safe_zone(cooling_list, cpu_temp);
				ret = SAFE_ZONE;
			}
		} else {
			/*
			 * this includes the case where :
			 * (omap_gov->alert_temp - omap_gov->hysteresis_value) <= T
			 * && T < omap_gov->alert_temp
			 */
			omap_monitor_zone(cooling_list, cpu_temp);
			ret = MONITOR_ZONE;
		}
	} else {
		/*
		 * this includes the case where :
		 * (omap_gov->panic_temp - omap_gov->hysteresis_value) <= T < omap_gov->panic_temp
		 */
		omap_alert_zone(cooling_list, cpu_temp);
		ret = ALERT_ZONE;
	}
	
	/* The effective cooling level will be the highest one between the CPU and the PCB */
	thermal_cooling_set_level(&cooling_agents,
				(omap_gov->cooling_level > omap_gov->pcb_cooling_level) ?
				omap_gov->cooling_level : omap_gov->pcb_cooling_level);
	list_del_init(&cooling_agents);

	mutex_unlock(&omap_gov->governor_mutex);
	return ret;
}

static int omap_pcb_thermal_manager(struct list_head *cooling_list, int temp)
{
	struct thermal_dev *cooling_dev, *tmp;
	int needed_pcb_cooling_level;
	int ret = NO_ACTION;

	mutex_lock(&omap_gov->governor_mutex);
	pr_debug("%s:die %d, pcb %d, current colling level %d\n",
			__func__, omap_gov->sensor_temp, temp, omap_gov->pcb_cooling_level);

	if (temp >= omap_gov->pcb_temp_lower && temp < omap_gov->pcb_temp_upper) {
		if (omap_gov->pcb_panic_zone_reached > 0) {
			omap_gov->pcb_panic_guard++;
		}
		if (omap_gov->pcb_panic_zone_reached == 0 || omap_gov->pcb_panic_guard < omap_gov->pcb_panic_guard_treshold) {
			/* Still in the same zone, nothing to do */
			mutex_unlock(&omap_gov->governor_mutex);
			return ret;
		}
	}

	/* TO DO: need to build an algo to find the right cooling agent */
	list_for_each_entry_safe(cooling_dev, tmp, cooling_list, node) {
		if (cooling_dev->dev_ops &&
			cooling_dev->dev_ops->cool_device) {
			/* TO DO: Add cooling agents to a list here */
			list_add(&cooling_dev->node, &cooling_agents);
			break;
		} else {
			pr_info("%s:Cannot find cool_device for %s\n",
				__func__, cooling_dev->name);
		}
	}

	if (list_empty(&cooling_agents)) {
		pr_err("%s: No Cooling devices registered\n",
			__func__);
		mutex_unlock(&omap_gov->governor_mutex);
		return -ENODEV;
	}

	/* Compute the desired cooling level for the PCB */
	if (temp >= omap_gov->pcb_fatal_temp) {
		omap_gov->pcb_cooling_level = MAX_COOLING_LEVEL;
		pr_info("PCB fatal zone reached, temp %d !\n", temp);
		omap_gov->pcb_temp_lower = omap_gov->pcb_fatal_temp;
		omap_gov->pcb_temp_upper = 999;
		ret = FATAL_ZONE;
	} else if (temp >= omap_gov->pcb_panic_temp) {
		omap_gov->pcb_cooling_level++;
		omap_gov->pcb_panic_zone_reached++;
		omap_gov->pcb_panic_guard = 0;
		pr_info("%s: PCB Panic zone reached %i times, temp %d\n",
			__func__, omap_gov->pcb_panic_zone_reached, temp);
		if (omap_gov->pcb_cooling_level > MAX_COOLING_LEVEL)
			omap_gov->pcb_cooling_level = MAX_COOLING_LEVEL;
		needed_pcb_cooling_level = (temp - omap_gov->pcb_panic_temp) /
				((omap_gov->pcb_fatal_temp - omap_gov->pcb_panic_temp) / 4) + 1;
		if (needed_pcb_cooling_level > omap_gov->pcb_cooling_level)
			omap_gov->pcb_cooling_level = needed_pcb_cooling_level;
		if (omap_gov->pcb_cooling_level > MAX_COOLING_LEVEL)
			omap_gov->pcb_cooling_level = MAX_COOLING_LEVEL;
		pr_info("increasing PCB cooling level to %d\n", omap_gov->pcb_cooling_level);
		omap_gov->pcb_temp_lower = omap_gov->pcb_panic_temp - omap_gov->pcb_hysteresis_value;
		omap_gov->pcb_temp_upper = (
			((omap_gov->pcb_fatal_temp - omap_gov->pcb_panic_temp) / 4) *
			omap_gov->pcb_cooling_level) + omap_gov->pcb_panic_temp;
		ret = PANIC_ZONE;
	} else if (temp >= omap_gov->pcb_alert_temp) {
		if (omap_gov->pcb_panic_zone_reached == 0) {
			/* Temperature rises and enters into alert zone */
			omap_gov->pcb_cooling_level = 0;
		} else {
			if (omap_gov->pcb_panic_zone_reached > 0)
				omap_gov->pcb_panic_zone_reached = 0;
			if (omap_gov->pcb_cooling_level > 0) {
				omap_gov->pcb_cooling_level--;
				pr_info("lowering PCB cooling level to %d\n", omap_gov->pcb_cooling_level);
			}
		}
		omap_gov->pcb_temp_lower = omap_gov->pcb_alert_temp - omap_gov->pcb_hysteresis_value;
		omap_gov->pcb_temp_upper = omap_gov->pcb_panic_temp;
		ret = ALERT_ZONE;
	} else if (temp >= omap_gov->pcb_monitor_temp) {
		omap_gov->pcb_cooling_level = 0;
		omap_gov->pcb_temp_lower = omap_gov->pcb_monitor_temp - omap_gov->pcb_hysteresis_value;
		omap_gov->pcb_temp_upper = omap_gov->pcb_alert_temp;
		omap_gov->pcb_panic_zone_reached = 0;
		ret = MONITOR_ZONE;
	} else {
		omap_gov->pcb_cooling_level = 0;
		omap_gov->pcb_temp_lower = omap_gov->pcb_safe_temp;
		omap_gov->pcb_temp_upper = omap_gov->pcb_monitor_temp;
		omap_gov->pcb_panic_zone_reached = 0;
		ret = SAFE_ZONE;
	}
	
	/* The effective cooling level will be the highest one between the CPU and the PCB */
	thermal_cooling_set_level(&cooling_agents,
				(omap_gov->cooling_level > omap_gov->pcb_cooling_level) ?
				omap_gov->cooling_level : omap_gov->pcb_cooling_level);
	list_del_init(&cooling_agents);

	mutex_unlock(&omap_gov->governor_mutex);
	return ret;
}

static void decrease_mpu_freq_fn(struct work_struct *work)
{
	struct omap_die_governor *omap_gov;

	omap_gov = container_of(work, struct omap_die_governor,
				decrease_mpu_freq_work.work);

	omap_gov->sensor_temp = thermal_request_temp(omap_gov->temp_sensor);
	thermal_sensor_set_temp(omap_gov->temp_sensor);
}

/*
 * Make an average of the OMAP on-die temperature
 * this is helpful to handle burst activity of OMAP when extrapolating
 * the OMAP hot spot temperature from on-die sensor and PCB temperature
 * Re-evaluate the temperature gradient between hot spot and on-die sensor
 * (See absolute_delta) and reconfigure the thresholds if needed
 */
static void average_on_die_temperature(void)
{
	int i;
	int die_temp_lower = 0;
	int die_temp_upper = 0;

	if (omap_gov->temp_sensor == NULL)
		return;

	/* Read current temperature */
	omap_gov->sensor_temp = thermal_request_temp(omap_gov->temp_sensor);

	/* if on-die sensor does not report a correct value, then return */
	if (omap_gov->sensor_temp == -EINVAL)
		return;

	/* Update historical buffer */
	for (i = 1; i < AVERAGE_NUMBER; i++) {
		cpu_sensor_temp_table[AVERAGE_NUMBER - i] =
		cpu_sensor_temp_table[AVERAGE_NUMBER - i - 1];
	}
	cpu_sensor_temp_table[0] = omap_gov->sensor_temp;

	if (cpu_sensor_temp_table[AVERAGE_NUMBER - 1] == 0)
		omap_gov->avg_is_valid = 0;
	else
		omap_gov->avg_is_valid = 1;

	/* Compute the new average value */
	omap_gov->avg_cpu_sensor_temp = 0;
	for (i = 0; i < AVERAGE_NUMBER; i++)
		omap_gov->avg_cpu_sensor_temp += cpu_sensor_temp_table[i];

	omap_gov->avg_cpu_sensor_temp =
		(omap_gov->avg_cpu_sensor_temp / AVERAGE_NUMBER);

	/*
	 * Reconfigure the current temperature thresholds according
	 * to the current PCB temperature
	 */
	convert_omap_sensor_temp_to_hotspot_temp(omap_gov->sensor_temp);
	/* take the governor lock here, otherwise
	   thermal_update_temp_thresholds() may race against
	   changes done by the governor code */
	mutex_lock(&omap_gov->governor_mutex);
	die_temp_lower = hotspot_temp_to_sensor_temp(
		omap_gov->hotspot_temp_lower);
	die_temp_upper = hotspot_temp_to_sensor_temp(
		omap_gov->hotspot_temp_upper);
	thermal_update_temp_thresholds(omap_gov->temp_sensor,
		die_temp_lower, die_temp_upper);
	mutex_unlock(&omap_gov->governor_mutex);

	return;
}

static void average_cpu_sensor_delayed_work_fn(struct work_struct *work)
{
	struct omap_die_governor *omap_gov =
				container_of(work, struct omap_die_governor,
					     average_cpu_sensor_work.work);

	average_on_die_temperature();

	schedule_delayed_work(&omap_gov->average_cpu_sensor_work,
				msecs_to_jiffies(omap_gov->average_period));
}

static int omap_process_cpu_temp(struct list_head *cooling_list,
				struct thermal_dev *temp_sensor,
				int temp)
{
	if (!strcmp(temp_sensor->name, "pcb_sensor") ||
		!strcmp(temp_sensor->name, TMP102_SENSOR_NAME)) {
		if (pcb_sensor == NULL || pcb_sensor != temp_sensor) {
			pr_info("%s: Setting %s pointer\n",
				__func__, temp_sensor->name);
			pcb_sensor = temp_sensor;
		}
		omap_gov->pcb_temp = temp;

		if (omap_gov->use_separate_pcb_zone_definition)
			return omap_pcb_thermal_manager(cooling_list, omap_gov->pcb_temp);
		else
			return 0;
	} else {
		if (omap_gov->temp_sensor == NULL) {
			pr_info("%s: Setting %s pointer\n",
				__func__, temp_sensor->name);
			omap_gov->temp_sensor = temp_sensor;
		}
		omap_gov->sensor_temp = temp;
	}

	return omap_cpu_thermal_manager(cooling_list, omap_gov->sensor_temp);
}

static struct thermal_dev_ops omap_gov_ops = {
	.process_temp = omap_process_cpu_temp,
};

static ssize_t show_omap_zone_thresholds(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "Fatal Temp:   %d\n"
			    "Panic Temp:   %d\n"
			    "Alert Temp:   %d\n"
			    "Monitor Temp: %d\n"
			    "Safe Temp:    %d\n"
			    "Hysteresis:   %d\n", omap_gov->fatal_temp, omap_gov->panic_temp,
			    omap_gov->alert_temp, omap_gov->monitor_temp, omap_gov->safe_temp,
			    omap_gov->hysteresis_value);
}

static ssize_t store_omap_zone_thresholds(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int fatal_temp, panic_temp, alert_temp, monitor_temp, safe_temp, hysteresis;

	if (sscanf(buf, "%d %d %d %d %d %d", &fatal_temp, &panic_temp, &alert_temp, &monitor_temp, &safe_temp, &hysteresis) != 6) {
		count = -EINVAL;
	} else {
		mutex_lock(&omap_gov->governor_mutex);

		omap_gov->fatal_temp = fatal_temp;
		omap_gov->panic_temp = panic_temp;
		omap_gov->alert_temp = alert_temp;
		omap_gov->monitor_temp = monitor_temp;
		omap_gov->safe_temp = safe_temp;
		omap_gov->hysteresis_value = hysteresis;

		/* Reset everything */
		omap_gov->cooling_level = 0;
		omap_gov->panic_zone_reached = 0;
		omap_gov->avg_is_valid = 0;
		omap_gov->absolute_delta = 0;
		omap_gov->average_period = 0;
		omap_gov->avg_cpu_sensor_temp = 0;
		omap_gov->average_period = omap_gov->normal_temp_monitoring_rate;

		mutex_unlock(&omap_gov->governor_mutex);

		if (omap_gov && omap_gov->temp_sensor) {
			/* The OMAP sensor is properly registered */
			omap_gov->sensor_temp = thermal_request_temp(omap_gov->temp_sensor);
			thermal_sensor_set_temp(omap_gov->temp_sensor);
		}
	}

	return count;
}

static ssize_t show_pcb_zone_thresholds(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "Fatal Temp:   %d\n"
			    "Panic Temp:   %d\n"
			    "Alert Temp:   %d\n"
			    "Monitor Temp: %d\n"
			    "Safe Temp:    %d\n"
			    "Hysteresis:   %d\n", omap_gov->pcb_fatal_temp, omap_gov->pcb_panic_temp,
			    omap_gov->pcb_alert_temp, omap_gov->pcb_monitor_temp, omap_gov->pcb_safe_temp,
			    omap_gov->pcb_hysteresis_value);
}

static ssize_t store_pcb_zone_thresholds(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int fatal_temp, panic_temp, alert_temp, monitor_temp, safe_temp, hysteresis;

	if (sscanf(buf, "%d %d %d %d %d %d", &fatal_temp, &panic_temp, &alert_temp, &monitor_temp, &safe_temp, &hysteresis) != 6) {
		count = -EINVAL;
	} else {
		mutex_lock(&omap_gov->governor_mutex);

		omap_gov->pcb_fatal_temp = fatal_temp;
		omap_gov->pcb_panic_temp = panic_temp;
		omap_gov->pcb_alert_temp = alert_temp;
		omap_gov->pcb_monitor_temp = monitor_temp;
		omap_gov->pcb_safe_temp = safe_temp;
		omap_gov->pcb_hysteresis_value = hysteresis;

		/* Reset everything */
		omap_gov->pcb_cooling_level = 0;
		omap_gov->pcb_panic_zone_reached = 0;
		omap_gov->pcb_temp_lower = 0;
		omap_gov->pcb_temp_upper = 0;

		mutex_unlock(&omap_gov->governor_mutex);

		if (pcb_sensor && pcb_sensor->dev_ops) {
			/*
			 * The PCB sensor is properly registered (pcb_sensor->dev_ops
			 * is set to NULL when the PCB sensor has been unregistered)
			 */
			omap_gov->pcb_temp = thermal_request_temp(pcb_sensor);
			thermal_sensor_set_temp(pcb_sensor);
		}
	}

	return count;
}

static struct kobj_attribute attr_pcb_zone_thresholds = __ATTR(pcb_zone_thresholds, S_IWUSR | S_IRUGO,
							       show_pcb_zone_thresholds, store_pcb_zone_thresholds);
static struct kobj_attribute attr_omap_zone_thresholds =  __ATTR(omap_zone_thresholds, S_IWUSR | S_IRUGO,
								 show_omap_zone_thresholds, store_omap_zone_thresholds);

static struct attribute *omap_die_governor_group_attributes[] = {
	&attr_omap_zone_thresholds.attr,
	&attr_pcb_zone_thresholds.attr,
	NULL
};

static const struct attribute_group omap_die_governor_group = {
	.attrs = omap_die_governor_group_attributes,
};

static int __init omap_die_governor_init(void)
{
	struct thermal_dev *thermal_fw;
	const struct omap_die_governor_config *die_gov_cfg;
	int ret;

	omap_gov = kzalloc(sizeof(struct omap_die_governor), GFP_KERNEL);
	if (!omap_gov) {
		pr_err("%s:Cannot allocate memory\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&omap_gov->governor_mutex);
	/* copy from board data and initialize configuration */
	die_gov_cfg= omap_get_config(OMAP_TAG_DIE_GOVERNOR,
			struct omap_die_governor_config);
	omap_gov->fatal_temp = (die_gov_cfg && die_gov_cfg->fatal_temp) ?
			die_gov_cfg->fatal_temp : OMAP_FATAL_TEMP;
	omap_gov->panic_temp = (die_gov_cfg && die_gov_cfg->panic_temp) ?
			die_gov_cfg->panic_temp : OMAP_PANIC_TEMP;
	omap_gov->alert_temp = (die_gov_cfg && die_gov_cfg->alert_temp) ?
			die_gov_cfg->alert_temp : OMAP_ALERT_TEMP;
	omap_gov->monitor_temp = (die_gov_cfg && die_gov_cfg->monitor_temp) ?
			die_gov_cfg->monitor_temp : OMAP_MONITOR_TEMP;
	omap_gov->safe_temp = (die_gov_cfg && die_gov_cfg->safe_temp) ?
			die_gov_cfg->safe_temp : OMAP_SAFE_TEMP;

	omap_gov->pcb_fatal_temp = (die_gov_cfg && die_gov_cfg->pcb_fatal_temp) ?
			die_gov_cfg->pcb_fatal_temp : OMAP_FATAL_TEMP;
	omap_gov->pcb_panic_temp = (die_gov_cfg && die_gov_cfg->pcb_panic_temp) ?
			die_gov_cfg->pcb_panic_temp : OMAP_PANIC_TEMP;
	omap_gov->pcb_alert_temp = (die_gov_cfg && die_gov_cfg->pcb_alert_temp) ?
			die_gov_cfg->pcb_alert_temp : OMAP_ALERT_TEMP;
	omap_gov->pcb_monitor_temp = (die_gov_cfg && die_gov_cfg->pcb_monitor_temp) ?
			die_gov_cfg->pcb_monitor_temp : OMAP_MONITOR_TEMP;
	omap_gov->pcb_safe_temp = (die_gov_cfg && die_gov_cfg->pcb_safe_temp) ?
			die_gov_cfg->pcb_safe_temp : OMAP_SAFE_TEMP;

	omap_gov->hysteresis_value = (die_gov_cfg && die_gov_cfg->hysteresis_value) ? die_gov_cfg->hysteresis_value : HYSTERESIS_VALUE;
	omap_gov->pcb_hysteresis_value = (die_gov_cfg && die_gov_cfg->pcb_hysteresis_value) ? die_gov_cfg->pcb_hysteresis_value : HYSTERESIS_VALUE;
	omap_gov->normal_temp_monitoring_rate = (die_gov_cfg && die_gov_cfg->normal_temp_monitoring_rate) ?
			die_gov_cfg->normal_temp_monitoring_rate : NORMAL_TEMP_MONITORING_RATE;
	omap_gov->fast_temp_monitoring_rate = (die_gov_cfg && die_gov_cfg->fast_temp_monitoring_rate) ?
			die_gov_cfg->fast_temp_monitoring_rate : FAST_TEMP_MONITORING_RATE;
	omap_gov->use_separate_pcb_zone_definition = (die_gov_cfg && die_gov_cfg->use_separate_pcb_zone_definition) ?
			die_gov_cfg->use_separate_pcb_zone_definition : 0;
	omap_gov->pcb_panic_guard_treshold = (die_gov_cfg && die_gov_cfg->pcb_panic_guard_treshold) ?
			die_gov_cfg->pcb_panic_guard_treshold : 3;

	thermal_fw = kzalloc(sizeof(struct thermal_dev), GFP_KERNEL);
	if (thermal_fw) {
		thermal_fw->name = "omap_ondie_governor";
		thermal_fw->domain_name = "cpu";
		thermal_fw->dev_ops = &omap_gov_ops;
		thermal_governor_dev_register(thermal_fw);
		therm_fw = thermal_fw;
	} else {
		pr_err("%s: Cannot allocate memory\n", __func__);
		kfree(omap_gov);
		return -ENOMEM;
	}

	pcb_sensor = NULL;
	omap_gov->temp_sensor = NULL;

	omap_gov->cooling_level = 0;
	omap_gov->pcb_cooling_level = 0;
	omap_gov->pcb_temp_lower = 0;
	omap_gov->pcb_temp_upper = 0;
	omap_gov->pcb_panic_guard = 0;

	/* the extrapolation of the CPU hot-spot temperature given the on-die sensor measurement
	 * is a simple 1st order model. It is dependent on CPU type (location of on-die sensor)
	 */
	if (cpu_is_omap443x()) {
		omap_gov->gradient_slope = OMAP_GRADIENT_SLOPE_4430;
		omap_gov->gradient_const = OMAP_GRADIENT_CONST_4430;
		omap_gov->gradient_slope_w_pcb = OMAP_GRADIENT_SLOPE_W_PCB_4460; /* FIXME */
		omap_gov->gradient_const_w_pcb = OMAP_GRADIENT_CONST_W_PCB_4460; /* FIXME */
	} else if (cpu_is_omap446x()) {
		omap_gov->gradient_slope = OMAP_GRADIENT_SLOPE_4460;
		omap_gov->gradient_const = OMAP_GRADIENT_CONST_4460;
		omap_gov->gradient_slope_w_pcb = OMAP_GRADIENT_SLOPE_W_PCB_4460;
		omap_gov->gradient_const_w_pcb = OMAP_GRADIENT_CONST_W_PCB_4460;
	} else if (cpu_is_omap447x()) {
		omap_gov->gradient_slope = OMAP_GRADIENT_SLOPE_4470;
		omap_gov->gradient_const = OMAP_GRADIENT_CONST_4470;
		omap_gov->gradient_slope_w_pcb = OMAP_GRADIENT_SLOPE_W_PCB_4470;
		omap_gov->gradient_const_w_pcb = OMAP_GRADIENT_CONST_W_PCB_4470;
	} else {
		omap_gov->gradient_slope = 0;
		omap_gov->gradient_const = 0;
		omap_gov->gradient_slope_w_pcb = 0;
		omap_gov->gradient_const_w_pcb = 0;
	}

	/* Init delayed work to average on-die temperature */
	INIT_DELAYED_WORK(&omap_gov->average_cpu_sensor_work,
			  average_cpu_sensor_delayed_work_fn);
	INIT_DELAYED_WORK(&omap_gov->decrease_mpu_freq_work,
			  decrease_mpu_freq_fn);

	omap_gov->average_period = omap_gov->normal_temp_monitoring_rate;
	/* OMAP4460 and OMAP4470 react quite fast to cooling, so
	   use the safe_guard after some time of no change in panic zone */
	if (cpu_is_omap446x()||cpu_is_omap447x())
		omap_gov->decrease_mpu_freq_period = DECREASE_MPU_FREQ_PERIOD;
	omap_gov->avg_is_valid = 0;
	schedule_delayed_work(&omap_gov->average_cpu_sensor_work,
			msecs_to_jiffies(0));

	sysfs_kobj = kobject_create_and_add("omap_die_governor", &platform_bus.kobj);
	ret = sysfs_create_group(sysfs_kobj,
				 &omap_die_governor_group);
	if (ret) {
		pr_err("could not create sysfs files\n");
		kfree(therm_fw);
		kfree(omap_gov);
		return ret;
	}

	return 0;
}

static void __exit omap_die_governor_exit(void)
{
	if (omap_gov) {
		sysfs_remove_group(sysfs_kobj, &omap_die_governor_group);
		cancel_delayed_work_sync(&omap_gov->average_cpu_sensor_work);
		cancel_delayed_work_sync(&omap_gov->decrease_mpu_freq_work);
		thermal_governor_dev_unregister(therm_fw);
		mutex_destroy(&omap_gov->governor_mutex);
		kfree(therm_fw);
		kfree(omap_gov);
	}
}

module_init(omap_die_governor_init);
module_exit(omap_die_governor_exit);

MODULE_AUTHOR("Dan Murphy <DMurphy@ti.com>");
MODULE_DESCRIPTION("OMAP on-die thermal governor");
MODULE_LICENSE("GPL");
