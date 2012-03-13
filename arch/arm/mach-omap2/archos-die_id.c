#include <linux/types.h>
#include <linux/sysdev.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <mach/archos-dieid.h>

#include <asm/io.h>

/* OMAP3xxx  */
#define CONTROL_DIE_ID_REG	0x4830A218
#define CONTROL_PROD_ID_REG	0x4830A208
#define CONTROL_IDCODE_REG	0x4830A204

/* OMAP44xx */
#define DIE_ID_REG_BASE		(L4_44XX_PHYS + 0x2000)
#define DIE_ID_REG_OFFSET	0x200


static struct sysdev_class die_id_sysclass = {
	.name		= "die_id",
};

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

static ssize_t show_dieid(struct sysdev_class* cls, 
		struct sysdev_class_attribute *attr, char* buf)
{
	u32 die_id[4];
	get_dieid(die_id);
	return sprintf(buf, "%08x%08x%08x%08x\n", 
			die_id[0],die_id[1],die_id[2],die_id[3]); 
}
static SYSDEV_CLASS_ATTR(dieid, S_IRUGO, show_dieid, NULL);

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

static ssize_t show_prodid(struct sysdev_class *cls, 
		struct sysdev_class_attribute *attr, char* buf)
{
	u32 prod_id[2];
	get_prodid(prod_id);
	return sprintf(buf, "%08x%08x\n", 
			prod_id[0],prod_id[1]); 
}
static SYSDEV_CLASS_ATTR(prodid, S_IRUGO, show_prodid, NULL);

static ssize_t show_idcode(struct sysdev_class *cls, 
		struct sysdev_class_attribute *attr, char* buf)
{
	u32 idcode;
	if (cpu_is_omap44xx())
		idcode = 0;
	else if(cpu_is_omap34xx())
		idcode = omap_readl(CONTROL_IDCODE_REG);
	return sprintf(buf, "%08x\n", idcode);
}
static SYSDEV_CLASS_ATTR(idcode, S_IRUGO, show_idcode, NULL);

static int __init archos_get_dieid_init(void)
{
	int ret;
	
	ret = sysdev_class_register(&die_id_sysclass);
	if (ret >= 0) {
		sysdev_class_create_file(&die_id_sysclass, &attr_dieid);	
		sysdev_class_create_file(&die_id_sysclass, &attr_prodid);
		sysdev_class_create_file(&die_id_sysclass, &attr_idcode);
	}
	return ret;
}

late_initcall(archos_get_dieid_init);
