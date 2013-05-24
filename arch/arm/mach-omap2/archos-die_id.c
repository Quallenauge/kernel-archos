#include <linux/types.h>
#include <linux/device.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <plat/cpu.h>
#include <plat/usb.h>
#include <plat/omap44xx.h>
#include <mach/archos-dieid.h>

#include <asm/io.h>
#include "iomap.h"

/* OMAP3xxx  */
#define CONTROL_DIE_ID_REG	0x4830A218
#define CONTROL_PROD_ID_REG	0x4830A208
#define CONTROL_IDCODE_REG	0x4830A204

/* OMAP44xx */
#define DIE_ID_REG_BASE		(L4_44XX_PHYS + 0x2000)
#define DIE_ID_REG_OFFSET	0x200


void get_dieid(u32 *die_id)
{
	u32 reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;
	if (cpu_is_omap44xx()) {
		die_id[3]=omap_readl(reg);
		die_id[2]=omap_readl(reg + 0x08);
		die_id[1]=omap_readl(reg + 0x0c);
		die_id[0]=omap_readl(reg + 0x10);
	} else if (cpu_is_omap34xx()) {
		die_id[3]=omap_readl(CONTROL_DIE_ID_REG);
		die_id[2]=omap_readl(CONTROL_DIE_ID_REG + 0x04);
		die_id[1]=omap_readl(CONTROL_DIE_ID_REG + 0x08);
		die_id[0]=omap_readl(CONTROL_DIE_ID_REG + 0x0c);		
	}
}

static ssize_t show_dieid(struct kobject *kobj,
	    struct kobj_attribute *attr, char *buf)
{
	u32 die_id[4];
	get_dieid(die_id);
	return sprintf(buf, "%08x%08x%08x%08x\n", 
			die_id[0],die_id[1],die_id[2],die_id[3]); 
}

void get_prodid(u32 *prod_id)
{
	if (cpu_is_omap44xx()) {
		u32 reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;
		prod_id[1] = omap_readl(reg + 0x14);
		prod_id[0] = omap_readl(reg + 0x18);
	} else if (cpu_is_omap34xx()) {
		prod_id[1] = omap_readl(CONTROL_PROD_ID_REG + 0xc);
		prod_id[0] = omap_readl(CONTROL_PROD_ID_REG + 0x8);
	}
}

static ssize_t show_prodid(struct kobject *kobj,
	    struct kobj_attribute *attr, char *buf)
{
	u32 prod_id[2];
	get_prodid(prod_id);
	return sprintf(buf, "%08x%08x\n", 
			prod_id[0],prod_id[1]); 
}

static ssize_t show_idcode(struct kobject *kobj,
	    struct kobj_attribute *attr, char *buf)
{
	u32 idcode;
	if (cpu_is_omap44xx())
		idcode = 0;
	else if(cpu_is_omap34xx())
		idcode = omap_readl(CONTROL_IDCODE_REG);
	return sprintf(buf, "%08x\n", idcode);
}

#define OMAP_SOC_ATTR_RO(_name, _show) \
	struct kobj_attribute omap_soc_prop_attr_##_name = \
		__ATTR(_name, S_IRUGO, _show, NULL)

static OMAP_SOC_ATTR_RO(die_id, show_dieid);
static OMAP_SOC_ATTR_RO(idcode, show_idcode);
static OMAP_SOC_ATTR_RO(prodid, show_prodid);

static struct attribute *omap_soc_prop_attrs[] = {
	&omap_soc_prop_attr_die_id.attr,
	&omap_soc_prop_attr_idcode.attr,
	&omap_soc_prop_attr_prodid.attr,
	NULL,
};

static struct attribute_group omap_soc_prop_attr_group = {
	.attrs = omap_soc_prop_attrs,
};

static int __init archos_get_dieid_init(void)
{
	struct kobject *board_props_kobj;
		int ret = 0;

		//TODO Archos: Place die_id sysfs entries in drivers/system folder.

		board_props_kobj = kobject_create_and_add("die_id", NULL);
		if (!board_props_kobj)
			goto err_board_obj;

		ret = sysfs_create_group(board_props_kobj, &omap_soc_prop_attr_group);
		if (ret)
			goto err_sysfs_create;

		return 1;

	err_sysfs_create:
	err_board_obj:
		if (!board_props_kobj || ret)
			pr_err("failed to create board_properties\n");
		return 0;
}

late_initcall(archos_get_dieid_init);
