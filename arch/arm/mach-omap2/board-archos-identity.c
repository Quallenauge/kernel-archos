/* OMAP Identity file for Archos boards.
 *
 * Copyright (C) 2012 Archos
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2010 Texas Instruments
 *
 * Based on mach-omap2/board-44xx-identity.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/id.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/board-archos.h>
#include <plat/board.h>

#include "control.h"

struct board_props {
    int screen_rotation;
} board_properties;

static ssize_t archos_soc_family_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "OMAP%04x\n", GET_OMAP_TYPE);
}

static ssize_t archos_soc_revision_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "ES%d.%d\n", (GET_OMAP_REVISION() >> 4) & 0xf,
		       GET_OMAP_REVISION() & 0xf);
}

static ssize_t archos_board_screen_rotation_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", board_properties.screen_rotation);
}

#define ARCHOS_SOC_ATTR_RO(_name, _show) \
	struct kobj_attribute archos_soc_prop_attr_##_name = \
		__ATTR(_name, S_IRUGO, _show, NULL)

#define ARCHOS_BOARD_ATTR_RO(_name, _show) \
	struct kobj_attribute archos_board_prop_attr_##_name = \
		__ATTR(_name, S_IRUGO, _show, NULL)


static ARCHOS_SOC_ATTR_RO(family, archos_soc_family_show);
static ARCHOS_SOC_ATTR_RO(revision, archos_soc_revision_show);

static ARCHOS_BOARD_ATTR_RO(screen_rotation, archos_board_screen_rotation_show);

static struct attribute *archos_soc_prop_attrs[] = {
	&archos_soc_prop_attr_family.attr,
	&archos_soc_prop_attr_revision.attr,
	NULL,
};

static struct attribute *archos_board_prop_attrs[] = {
	&archos_board_prop_attr_screen_rotation.attr,
	NULL,
};

static struct attribute_group archos_soc_prop_attr_group = {
	.attrs = archos_soc_prop_attrs,
};

static struct attribute_group archos_board_prop_attr_group = {
	.attrs = archos_board_prop_attrs,
};

void __init fill_board_properties(struct board_props *props)
{
	const struct archos_display_config *disp_cfg;
	const struct archos_disp_conf * conf;

	disp_cfg = omap_get_config( ARCHOS_TAG_DISPLAY, struct archos_display_config );
	conf = hwrev_ptr(disp_cfg, system_rev);

	if (IS_ERR(conf)) props->screen_rotation = 0;
	else props->screen_rotation = conf->panel_rotation;
}

void __init archos_create_board_props(void)
{
	struct kobject *board_props_kobj;
	struct kobject *soc_kobj = NULL;
	struct kobject *board_kobj = NULL;
	int ret = 0;

	board_props_kobj = kobject_create_and_add("board_properties", NULL);
	if (!board_props_kobj)
		goto err_board_props_obj;

	soc_kobj = kobject_create_and_add("soc", board_props_kobj);
	if (!soc_kobj)
		goto err_soc_obj;

	board_kobj = kobject_create_and_add("board", board_props_kobj);
	if (!soc_kobj)
		goto err_board_obj;

	ret = sysfs_create_group(soc_kobj, &archos_soc_prop_attr_group);
	if (ret)
		goto err_soc_sysfs_create;

	ret = sysfs_create_group(board_kobj, &archos_board_prop_attr_group);
	if (ret)
		goto err_board_sysfs_create;

	fill_board_properties(&board_properties);
	return;

err_board_sysfs_create:
	sysfs_remove_group(soc_kobj, &archos_soc_prop_attr_group);
err_soc_sysfs_create:
	kobject_put(board_kobj);
err_board_obj:
        kobject_put(soc_kobj);
err_soc_obj:
	kobject_put(board_props_kobj);
err_board_props_obj:
	if (!board_props_kobj || !soc_kobj || !board_kobj || ret)
		pr_err("failed to create board_properties\n");
}
