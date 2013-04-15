/*
 * Reboot reason special handler
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/platform_device.h>
#include <linux/init.h>
#include <asm/setup.h>
#include <linux/reboot.h>
#include <linux/notifier.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include "omap4-sar-layout.h"

static int omap_reboot_notifier_call(struct notifier_block *this,
					unsigned long code, void *cmd)
{
	void __iomem *sar_base;
	char *reason = "android";

	sar_base = omap4_get_sar_ram_base();

	if (!sar_base)
		return notifier_from_errno(-ENOMEM);
	
	/* Save reboot mode in scratch memory */
	if (code == SYS_RESTART && cmd != NULL && strlen(cmd))
		reason = cmd;
	else if (code == SYS_POWER_OFF)
		reason = "off";
	printk("omap_reboot_notifier_call: code: %lu %s\n",code,reason);
	strncpy(sar_base + OMAP_REBOOT_REASON_OFFSET,
			reason, OMAP_REBOOT_REASON_SIZE);
	return NOTIFY_DONE;
}

static struct notifier_block omap_reboot_notifier = {
	.notifier_call = omap_reboot_notifier_call,
};

static int __init omap_reboot_reason_init(void)
{
	void __iomem *sar_base = omap4_get_sar_ram_base();
	if (sar_base){
		char android_reboot_reason[OMAP_REBOOT_REASON_SIZE] ; 
		strncpy(android_reboot_reason,sar_base + OMAP_REBOOT_REASON_OFFSET, OMAP_REBOOT_REASON_SIZE);
		if(strlen(android_reboot_reason)>0)
			printk("android_reboot_reason:%s\n\toffset:%x size:%d\n",android_reboot_reason,OMAP_REBOOT_REASON_OFFSET, OMAP_REBOOT_REASON_SIZE);
		if(!strncmp(android_reboot_reason,"android",OMAP_REBOOT_REASON_SIZE)) {
			strlcat(saved_command_line," androidboot.mode=android",COMMAND_LINE_SIZE);
		}else{
			strlcat(saved_command_line," androidboot.mode=recovery",COMMAND_LINE_SIZE);		
		}
	}else{ // noram , what the fuck are you thinking
			strlcat(saved_command_line," androidboot.mode=recovery",COMMAND_LINE_SIZE);		
	}
	return register_reboot_notifier(&omap_reboot_notifier);
}
core_initcall(omap_reboot_reason_init);
