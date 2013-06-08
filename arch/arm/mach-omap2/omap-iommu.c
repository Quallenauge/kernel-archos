/*
 * omap iommu: omap device registration
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Written by Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>

#include <plat/iommu.h>
#include <plat/irqs.h>
#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>

static struct omap_device_pm_latency omap_iommu_latency[] = {
       {
               .deactivate_func = omap_device_shutdown_hwmods,
               .activate_func = omap_device_enable_hwmods,
               .flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
       },
};

static int __init omap_iommu_dev_init(struct omap_hwmod *oh, void *unused)
{
	struct platform_device *pdev;
	struct iommu_platform_data pdata;
	struct omap_mmu_dev_attr *a = (struct omap_mmu_dev_attr *)oh->dev_attr;
	static int i;

	printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);

	pdata.name = oh->name;
	pdata.nr_tlb_entries = a->nr_tlb_entries;
	pdata.da_start = a->da_start;
	pdata.da_end = a->da_end;
	pdata.has_bus_err_back = a->has_bus_err_back;
	pdata.pm_constraint = a->pm_constraint;

	pdev = omap_device_build("omap-iommu", i, oh, &pdata, sizeof(pdata),
                       omap_iommu_latency, ARRAY_SIZE(omap_iommu_latency), 0);
	if (IS_ERR(pdev)) {
		pr_err("%s: device build error: %ld\n",
				__func__, PTR_ERR(pdev));
		return PTR_ERR(pdev);
	}

	i++;

	return 0;
}

static int __init omap_iommu_init(void)
{
	printk("%s:%s:%d\n",__FILE__,__FUNCTION__,__LINE__);
	return omap_hwmod_for_each_by_class("mmu", omap_iommu_dev_init, NULL);
}
/* must be ready before omap3isp is probed */
subsys_initcall(omap_iommu_init);

static void __exit omap_iommu_exit(void)
{
	/* Do nothing */
}
module_exit(omap_iommu_exit);

MODULE_AUTHOR("Hiroshi DOYU");
MODULE_DESCRIPTION("omap iommu: omap device registration");
MODULE_LICENSE("GPL v2");
