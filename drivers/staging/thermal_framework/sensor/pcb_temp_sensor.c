/*
 * PCB Temperature sensor driver file
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
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

#include <linux/err.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/stddef.h>
#include <linux/sysfs.h>
#include <linux/thermal_framework.h>
#include <linux/types.h>
#include <linux/mutex.h>

#include <plat/common.h>
#include <plat/pcb_temperature_sensor.h>
#include <linux/i2c/twl6030-gpadc.h>

#define PCB_REPORT_DELAY_MS	1000

/*
 * Set this to 0 to disable the registration of the PCB
 * sensor against the thermal framework. The sensor will
 * have no effect on the device behaviour, but its
 * value will be available using the proper sysfs.
 * This can be enabled/disabled at runtime using the
 * thermal_fw_report sysfs.
 */
#define REPORT_TO_THERMAL_FW	1

/*
 * pcb_temp_sensor structure
 * @pdev - Platform device pointer
 * @dev - device pointer
 * @sensor_mutex - Mutex for sysfs, irq and PM
 */
struct pcb_temp_sensor {
	struct platform_device *pdev;
	struct device *dev;
	struct mutex sensor_mutex;
	struct spinlock lock;
	struct thermal_dev *therm_fw;
	struct delayed_work pcb_sensor_work;
	int work_delay;
	int debug_temp;
	struct pcb_temp_sensor_pdata *pdata;
	int thermal_fw_report_enabled;
};

static void pcb_enable_thermal_fw_report(struct pcb_temp_sensor *temp_sensor, int on);


static int pcb_read_current_temp_raw(struct pcb_temp_sensor *temp_sensor)
{
	struct twl6030_gpadc_request req;
	int val;

	req.channels = (1 << temp_sensor->pdata->gpadc_channel);
	req.method = TWL6030_GPADC_SW2;
	req.func_cb = NULL;
	req.type = TWL6030_GPADC_WAIT;
	val = twl6030_gpadc_conversion(&req);
	if (val < 0) {
		pr_err("%s:TWL6030_GPADC conversion is invalid %d\n",
			__func__, val);
		return -EINVAL;
	}

	return req.rbuf[temp_sensor->pdata->gpadc_channel];
}

static int pcb_read_current_temp(struct pcb_temp_sensor *temp_sensor)
{
	int val;

	val = pcb_read_current_temp_raw(temp_sensor);
	msleep(100);
	if (val < 0)
		return val;

	return temp_sensor->pdata->adc_to_temp_conversion(val);
}

static int pcb_get_temp(struct thermal_dev *tdev)
{
	struct platform_device *pdev = to_platform_device(tdev->dev);
	struct pcb_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	temp_sensor->therm_fw->current_temp =
			pcb_read_current_temp(temp_sensor);

	return temp_sensor->therm_fw->current_temp;
}

static void pcb_report_fw_temp(struct thermal_dev *tdev)
{
	struct platform_device *pdev = to_platform_device(tdev->dev);
	struct pcb_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	int ret;

	pcb_get_temp(tdev);
	if (temp_sensor->therm_fw->current_temp != -EINVAL) {
		ret = thermal_sensor_set_temp(temp_sensor->therm_fw);
		if (ret == -ENODEV)
			pr_err("%s:thermal_sensor_set_temp reports error\n",
				__func__);

		kobject_uevent(&temp_sensor->dev->kobj, KOBJ_CHANGE);
	}
}

/*
 * sysfs hook functions
 */
static ssize_t show_temp_user_space(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcb_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", temp_sensor->debug_temp);
}

static ssize_t set_temp_user_space(struct device *dev,
			struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcb_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	long val;

	if (strict_strtol(buf, 10, &val)) {
		count = -EINVAL;
		goto out;
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

static int pcb_temp_sensor_read_temp(struct device *dev,
				      struct device_attribute *devattr,
				      char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcb_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	int temp = 0;

	temp = pcb_read_current_temp(temp_sensor);

	return sprintf(buf, "%d\n", temp);
}

static int pcb_temp_sensor_read_temp_raw(struct device *dev,
				      struct device_attribute *devattr,
				      char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcb_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	int val = 0;

	val = pcb_read_current_temp_raw(temp_sensor);

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_thermal_fw_report(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcb_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", temp_sensor->thermal_fw_report_enabled);
}

static ssize_t store_thermal_fw_report(struct device *dev,
			struct device_attribute *devattr,
			const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pcb_temp_sensor *temp_sensor = platform_get_drvdata(pdev);
	long val;

	if (strict_strtol(buf, 10, &val))
		count = -EINVAL;
	else
		pcb_enable_thermal_fw_report(temp_sensor, val);
	
	return count;
}

static DEVICE_ATTR(debug_user, S_IWUSR | S_IRUGO, show_temp_user_space,
			  set_temp_user_space);
static DEVICE_ATTR(temp1_input, S_IRUGO, pcb_temp_sensor_read_temp,
			  NULL);
static DEVICE_ATTR(temp1_input_raw, S_IRUGO, pcb_temp_sensor_read_temp_raw,
			  NULL);
static DEVICE_ATTR(thermal_fw_report, S_IWUSR | S_IRUGO, show_thermal_fw_report,
			  store_thermal_fw_report);

static struct attribute *pcb_temp_sensor_attributes[] = {
	&dev_attr_thermal_fw_report.attr,
	&dev_attr_temp1_input_raw.attr,
	&dev_attr_temp1_input.attr,
	&dev_attr_debug_user.attr,
	NULL
};

static const struct attribute_group pcb_temp_sensor_group = {
	.attrs = pcb_temp_sensor_attributes,
};

static struct thermal_dev_ops pcb_sensor_ops = {
	.report_temp = pcb_get_temp,
};

static void pcb_enable_thermal_fw_report(struct pcb_temp_sensor *temp_sensor, int on)
{
	if (temp_sensor->thermal_fw_report_enabled == !!on)
		return;

	temp_sensor->thermal_fw_report_enabled = !!on;

	if (temp_sensor->thermal_fw_report_enabled) {
		temp_sensor->therm_fw->dev_ops = &pcb_sensor_ops;
		thermal_sensor_dev_register(temp_sensor->therm_fw);
		schedule_delayed_work(&temp_sensor->pcb_sensor_work,
				msecs_to_jiffies(0));
	} else {
		cancel_delayed_work_sync(&temp_sensor->pcb_sensor_work);
		temp_sensor->therm_fw->current_temp = 0;
		thermal_sensor_set_temp(temp_sensor->therm_fw);
		thermal_sensor_dev_unregister(temp_sensor->therm_fw);
		temp_sensor->therm_fw->dev_ops = NULL;
	}
}

static void pcb_sensor_delayed_work_fn(struct work_struct *work)
{
	struct pcb_temp_sensor *temp_sensor =
				container_of(work, struct pcb_temp_sensor,
					     pcb_sensor_work.work);

	pcb_report_fw_temp(temp_sensor->therm_fw);
	schedule_delayed_work(&temp_sensor->pcb_sensor_work,
				msecs_to_jiffies(temp_sensor->work_delay));
}

static int __devinit pcb_temp_sensor_probe(struct platform_device *pdev)
{
	struct pcb_temp_sensor_pdata *pdata = pdev->dev.platform_data;
	struct pcb_temp_sensor *temp_sensor;
	int ret = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "%s: platform data missing\n", __func__);
		return -EINVAL;
	}

	temp_sensor = kzalloc(sizeof(struct pcb_temp_sensor), GFP_KERNEL);
	if (!temp_sensor)
		return -ENOMEM;

	/* Init delayed work for PCB sensor temperature */
	INIT_DELAYED_WORK(&temp_sensor->pcb_sensor_work,
			  pcb_sensor_delayed_work_fn);

	spin_lock_init(&temp_sensor->lock);
	mutex_init(&temp_sensor->sensor_mutex);

	temp_sensor->pdev = pdev;
	temp_sensor->dev = &pdev->dev;

	kobject_uevent(&pdev->dev.kobj, KOBJ_ADD);
	platform_set_drvdata(pdev, temp_sensor);

	temp_sensor->pdata = pdata;

	temp_sensor->thermal_fw_report_enabled = REPORT_TO_THERMAL_FW;

	temp_sensor->therm_fw = kzalloc(sizeof(struct thermal_dev), GFP_KERNEL);
	if (temp_sensor->therm_fw) {
		temp_sensor->therm_fw->name = "pcb_sensor";
		temp_sensor->therm_fw->domain_name = "cpu";
		temp_sensor->therm_fw->dev = temp_sensor->dev;
		temp_sensor->therm_fw->dev_ops = &pcb_sensor_ops;
		if (temp_sensor->thermal_fw_report_enabled)
			thermal_sensor_dev_register(temp_sensor->therm_fw);
	} else {
		dev_err(&pdev->dev, "%s:Cannot alloc memory for thermal fw\n",
			__func__);
		ret = -ENOMEM;
		goto therm_fw_alloc_err;
	}

	ret = sysfs_create_group(&pdev->dev.kobj,
				 &pcb_temp_sensor_group);
	if (ret) {
		dev_err(&pdev->dev, "could not create sysfs files\n");
		goto sysfs_create_err;
	}

	temp_sensor->work_delay = PCB_REPORT_DELAY_MS;
	if (temp_sensor->thermal_fw_report_enabled)
		schedule_delayed_work(&temp_sensor->pcb_sensor_work,
				msecs_to_jiffies(0));

	dev_info(&pdev->dev, "%s : '%s'\n", temp_sensor->therm_fw->name,
			pdata->name);

	return 0;

sysfs_create_err:
	if (temp_sensor->thermal_fw_report_enabled)
		thermal_sensor_dev_unregister(temp_sensor->therm_fw);
	kfree(temp_sensor->therm_fw);
	platform_set_drvdata(pdev, NULL);
therm_fw_alloc_err:
	mutex_destroy(&temp_sensor->sensor_mutex);
	kfree(temp_sensor);
	return ret;
}

static int __devexit pcb_temp_sensor_remove(struct platform_device *pdev)
{
	struct pcb_temp_sensor *temp_sensor = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &pcb_temp_sensor_group);
	if (temp_sensor->thermal_fw_report_enabled) {
		cancel_delayed_work_sync(&temp_sensor->pcb_sensor_work);
		thermal_sensor_dev_unregister(temp_sensor->therm_fw);
	}
	kfree(temp_sensor->therm_fw);
	kobject_uevent(&temp_sensor->dev->kobj, KOBJ_REMOVE);
	platform_set_drvdata(pdev, NULL);
	mutex_destroy(&temp_sensor->sensor_mutex);
	kfree(temp_sensor);

	return 0;
}

static int pcb_temp_sensor_runtime_suspend(struct device *dev)
{
	return 0;
}

static int pcb_temp_sensor_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops pcb_temp_sensor_dev_pm_ops = {
	.runtime_suspend = pcb_temp_sensor_runtime_suspend,
	.runtime_resume = pcb_temp_sensor_runtime_resume,
};

static struct platform_driver pcb_temp_sensor_driver = {
	.probe = pcb_temp_sensor_probe,
	.remove = pcb_temp_sensor_remove,
	.driver = {
		.name = "pcb_temp_sensor",
		.pm = &pcb_temp_sensor_dev_pm_ops,
	},
};

int __init pcb_temp_sensor_init(void)
{
	if (!cpu_is_omap446x() && !cpu_is_omap447x())
		return 0;

	return platform_driver_register(&pcb_temp_sensor_driver);
}

static void __exit pcb_temp_sensor_exit(void)
{
	platform_driver_unregister(&pcb_temp_sensor_driver);
}

module_init(pcb_temp_sensor_init);
module_exit(pcb_temp_sensor_exit);

MODULE_DESCRIPTION("PCB Temperature Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
