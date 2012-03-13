/*
 *  drivers/switch/switch_suspend.c
 *
 * Copyright (C) 2011 Archos S.A.
 * Author: Jean-Christophe Rona <rona@archos.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/switch.h>


#define SUSPEND_SW_NAME		"suspend_switch"

enum {
	STATE_RUNNING,
	STATE_ENTER_SUSPEND,
	STATE_SUSPENDING,
	STATE_ENTER_RESUME,
	STATE_RESUMING,
};

static ssize_t print_suspend_name(struct switch_dev *sdev, char *buf);
static ssize_t print_suspend_state(struct switch_dev *sdev, char *buf);


static struct switch_dev suspend_sw_dev = {
	.name = SUSPEND_SW_NAME,
	.print_name  = print_suspend_name,
	.print_state = print_suspend_state,
};

static ssize_t print_suspend_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", SUSPEND_SW_NAME);
}

static ssize_t print_suspend_state(struct switch_dev *sdev, char *buf)
{
	ssize_t buflen=0;

	switch(sdev->state)
	{
		case STATE_RUNNING:
			buflen = sprintf(buf, "Running\n");
			break;
		case STATE_SUSPENDING:
		case STATE_ENTER_SUSPEND:
			buflen = sprintf(buf, "Suspending\n");
			break;
		case STATE_RESUMING:
		case STATE_ENTER_RESUME:
			buflen = sprintf(buf, "Resuming\n");
			break;
		default:
			buflen = sprintf(buf, "Unknown state\n");
			break;
	}

	/*
	 * HACK: These intermediate STATE_ENTER_* states are needed
	 * because the switch_set_state() function will call the
	 * print_suspend_state() function to set some environment variables.
	 * That way, we are sure there will be no infinite loop,
	 * that the state won't be switched to "Running" prematurely,
	 * and that the environment variables will always reflect
	 * the current state.
	 */
	switch(sdev->state)
	{
		case STATE_SUSPENDING:
		case STATE_RESUMING:
			/*
			 * One the suspend state is read from user space,
			 * we go back to running state
			 */
			switch_set_state(&suspend_sw_dev, STATE_RUNNING);
			break;
		case STATE_ENTER_SUSPEND:
			sdev->state = STATE_SUSPENDING;
			break;
		case STATE_ENTER_RESUME:
			sdev->state = STATE_RESUMING;
			break;
	}

	return buflen;
}

static int suspend_switch_suspend(struct platform_device *pdev, pm_message_t state)
{
	switch_set_state(&suspend_sw_dev, STATE_ENTER_SUSPEND);

	return 0;
}

static int suspend_switch_resume(struct platform_device *pdev)
{
	switch_set_state(&suspend_sw_dev, STATE_ENTER_RESUME);

	return 0;
}

static int __init suspend_switch_probe(struct platform_device *pdev)
{
	int  err;

	err = switch_dev_register(&suspend_sw_dev);
	if(err < 0)
		return err;

	switch_set_state(&suspend_sw_dev, STATE_RUNNING);

	return 0;
}

static int suspend_switch_remove(struct platform_device *pdev)
{
	switch_dev_unregister(&suspend_sw_dev);

	return 0;
}

static struct platform_device suspend_switch_device = {
	.name		= SUSPEND_SW_NAME,
	.id		= -1,
	.dev            = {
		.platform_data = NULL,
	},
};

static struct platform_driver suspend_switch_driver = {
	.probe		= suspend_switch_probe,
	.remove		= suspend_switch_remove,
	.suspend	= suspend_switch_suspend,
	.resume		= suspend_switch_resume,
	.driver		= {
		.name	= SUSPEND_SW_NAME,
	},
};

static int __devinit suspend_switch_init(void)
{
	int  err;

	err = platform_device_register(&suspend_switch_device);
	if(err < 0)
		return err;

	err = platform_driver_register(&suspend_switch_driver);
	if(err < 0)
		goto driver_register_error;

	return 0;

driver_register_error:
	platform_device_unregister(&suspend_switch_device);

	return err;
}

static void __exit suspend_switch_exit(void)
{
	platform_driver_unregister(&suspend_switch_driver);
	platform_device_unregister(&suspend_switch_device);
}


module_init(suspend_switch_init);
module_exit(suspend_switch_exit);

MODULE_AUTHOR("Jean-Christophe Rona <rona@archos.com>");
MODULE_DESCRIPTION("Suspend/Resume Switch driver");
MODULE_LICENSE("GPL");
