/*
 * hdmi_panel.c
 *
 * HDMI library support functions for TI OMAP4 processors.
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors:	Mythri P k <mythripk@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <video/omapdss.h>
#include <linux/omapfb.h>
#include <linux/switch.h>
#include <linux/spinlock.h>

#include "dss.h"

#include <video/hdmi_ti_4xxx_ip.h>

static int overscan_percent = 10;

static struct {
	struct mutex hdmi_lock;
	struct switch_dev hdmi_switch;
	struct switch_dev underscan_switch;
	spinlock_t phy_lock;
	int connected;
	int initialized;
	struct completion phy_completion;
	bool edid_valid;
} hdmi;

enum hdmi_switch_value{
	HDMI_NOT_AVAILABLE = 0,
	HDMI_AVAILABLE = 1,
	HDMI_EDID_AVAILABLE = 2,
};

static ssize_t hdmi_deepcolor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r;
	r = omapdss_hdmi_get_deepcolor();
	return snprintf(buf, PAGE_SIZE, "%d\n", r);
}

static ssize_t hdmi_deepcolor_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned long deep_color;
	int r = kstrtoul(buf, 0, &deep_color);
	if (r || deep_color > 2)
		return -EINVAL;
	omapdss_hdmi_set_deepcolor(deep_color);
	return size;
}

static ssize_t hdmi_edid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return omapdss_hdmi_get_edid(buf);
}

static ssize_t hdmi_s3d_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r;
	ssize_t size;
	r = omapdss_hdmi_get_s3d_mode();
	switch (r) {
	case HDMI_FRAME_PACKING:
		size = snprintf(buf, PAGE_SIZE, "frame_packing\n");
		break;
	case HDMI_FIELD_ALTERNATIVE:
		size = snprintf(buf, PAGE_SIZE, "field_alternative\n");
		break;
	case HDMI_LINE_ALTERNATIVE:
		size = snprintf(buf, PAGE_SIZE, "line_alternative\n");
		break;
	case HDMI_SIDE_BY_SIDE_FULL:
		size = snprintf(buf, PAGE_SIZE, "side_by_side_full\n");
		break;
	case HDMI_L_DEPTH:
		size = snprintf(buf, PAGE_SIZE, "l_depth\n");
		break;
	case HDMI_L_DEPTH_GFX_GFX_DEPTH:
		size = snprintf(buf, PAGE_SIZE, "l_depth_gfx_depth\n");
		break;
	case HDMI_TOPBOTTOM:
		size = snprintf(buf, PAGE_SIZE, "top_bottom\n");
		break;
	case HDMI_SIDE_BY_SIDE_HALF:
		size = snprintf(buf, PAGE_SIZE, "side_by_side_half\n");
		break;
	default:
		return -EINVAL;
	}
	return size;
}

static ssize_t hdmi_s3d_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned long s3d_mode;
	int r = kstrtoul(buf, 0, &s3d_mode);
	if (r)
		return -EINVAL;
	switch (s3d_mode) {
	case HDMI_FRAME_PACKING:
	case HDMI_FIELD_ALTERNATIVE:
	case HDMI_LINE_ALTERNATIVE:
	case HDMI_SIDE_BY_SIDE_FULL:
	case HDMI_L_DEPTH:
	case HDMI_L_DEPTH_GFX_GFX_DEPTH:
	case HDMI_TOPBOTTOM:
	case HDMI_SIDE_BY_SIDE_HALF:
		omapdss_hdmi_set_s3d_mode(s3d_mode);
		break;
	default:
		return -EINVAL;
	}
	return size;
}

static ssize_t hdmi_s3d_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int enable;

	int r = kstrtoint(buf, 0, &enable);
	if (r)
		return -EINVAL;
	enable = !!enable;

	omapdss_hdmi_enable_s3d(enable);

	return size;
}

static ssize_t hdmi_s3d_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int r;
	r = omapdss_hdmi_get_s3d_enable();
	return snprintf(buf, PAGE_SIZE, "%d\n", r);
}

static DEVICE_ATTR(s3d_enable, S_IRUGO | S_IWUSR, hdmi_s3d_enable_show,
							hdmi_s3d_enable_store);
static DEVICE_ATTR(s3d_type, S_IRUGO | S_IWUSR, hdmi_s3d_mode_show,
							hdmi_s3d_mode_store);
static DEVICE_ATTR(edid, S_IRUGO, hdmi_edid_show, NULL);
static DEVICE_ATTR(deepcolor, S_IRUGO | S_IWUSR, hdmi_deepcolor_show,
							hdmi_deepcolor_store);

static struct attribute *hdmi_panel_attrs[] = {
	&dev_attr_s3d_enable.attr,
	&dev_attr_s3d_type.attr,
	&dev_attr_edid.attr,
	&dev_attr_deepcolor.attr,
	NULL,
};

static struct attribute_group hdmi_panel_attr_group = {
	.attrs = hdmi_panel_attrs,
};

static int hdmi_panel_probe(struct omap_dss_device *dssdev)
{
	DSSDBG("ENTER hdmi_panel_probe\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT |
			OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS;

	/*
	 * Initialize the timings to 640 * 480
	 * This is only for framebuffer update not for TV timing setting
	 * Setting TV timing will be done only on enable
	 */
	dssdev->panel.timings.x_res = 640;
	dssdev->panel.timings.y_res = 480;

	/* sysfs entry to provide user space control to set deepcolor mode */
	if (sysfs_create_group(&dssdev->dev.kobj, &hdmi_panel_attr_group))
		DSSERR("failed to create sysfs entries\n");

	DSSDBG("hdmi_panel_probe x_res= %d y_res = %d\n",
		dssdev->panel.timings.x_res,
		dssdev->panel.timings.y_res);

	hdmi.initialized = true;
	if (hdmi_get_current_hpd())
		hdmi_panel_hpd_handler(true);

	return 0;
}

static void hdmi_panel_remove(struct omap_dss_device *dssdev)
{
	sysfs_remove_group(&dssdev->dev.kobj, &hdmi_panel_attr_group);
}

static int hdmi_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;
	DSSDBG("ENTER hdmi_panel_enable\n");

	mutex_lock(&hdmi.hdmi_lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		r = -EINVAL;
		goto err;
	}

	r = omapdss_hdmi_display_enable(dssdev);
	if (r) {
		DSSERR("failed to power on\n");
		goto err;
	}

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
err:
	mutex_unlock(&hdmi.hdmi_lock);

	return r;
}

static void hdmi_panel_disable(struct omap_dss_device *dssdev)
{
	mutex_lock(&hdmi.hdmi_lock);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		omapdss_hdmi_display_disable(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	mutex_unlock(&hdmi.hdmi_lock);
}

void hdmi_panel_early_resume(struct omap_dss_device *dssdev)
{
	hdmi_panel_enable(dssdev);
        if (hdmi.edid_valid)
		switch_set_state(&hdmi.hdmi_switch, HDMI_EDID_AVAILABLE);
}

void hdmi_panel_early_suspend(struct omap_dss_device *dssdev)
{
	hdmi_panel_disable(dssdev);
}


static int hdmi_panel_suspend(struct omap_dss_device *dssdev)
{
	int r = 0;

	mutex_lock(&hdmi.hdmi_lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = -EINVAL;
		goto err;
	}

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	omapdss_hdmi_display_disable(dssdev);
err:
	mutex_unlock(&hdmi.hdmi_lock);

	return r;
}

static int hdmi_panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	mutex_lock(&hdmi.hdmi_lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		r = -EINVAL;
		goto err;
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
err:
	mutex_unlock(&hdmi.hdmi_lock);

	hdmi_panel_hpd_handler(hdmi_get_current_hpd());

	return r;
}

enum {
	HPD_STATE_OFF,
	HPD_STATE_START,
	HPD_STATE_EDID_TRYLAST = HPD_STATE_START + 5,
};

static struct hpd_worker_data {
	struct delayed_work dwork;
	atomic_t state;
} hpd_work;
static struct workqueue_struct *my_workq;

static void hdmi_hotplug_detect_worker(struct work_struct *work)
{
	struct hpd_worker_data *d = container_of(work, typeof(*d), dwork.work);
	struct omap_dss_device *dssdev = NULL;
	int state = atomic_read(&d->state);

	int match(struct omap_dss_device *dssdev, void *arg)
	{
		return sysfs_streq(dssdev->name , "hdmi");
	}
	dssdev = omap_dss_find_device(NULL, match);

	pr_err("in hpd work %d, state=%d\n", state, dssdev->state);
	if (dssdev == NULL || !hdmi.initialized)
		return;

	mutex_lock(&hdmi.hdmi_lock);
	if (state == HPD_STATE_OFF) {
		hdmi.edid_valid = false;
		switch_set_state(&hdmi.hdmi_switch, HDMI_NOT_AVAILABLE);
		switch_set_state(&hdmi.underscan_switch, 0);
		if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
			mutex_unlock(&hdmi.hdmi_lock);
			dssdev->driver->disable(dssdev);
			omapdss_hdmi_enable_s3d(false);
			mutex_lock(&hdmi.hdmi_lock);
		}
		goto done;
	} else {
		if (state == HPD_STATE_START) {
			unsigned long flags;

			mutex_unlock(&hdmi.hdmi_lock);
			dssdev->driver->enable(dssdev);
			mutex_lock(&hdmi.hdmi_lock);

			spin_lock_irqsave(&hdmi.phy_lock, flags);
			if (!hdmi.connected) {
				spin_unlock_irqrestore(&hdmi.phy_lock, flags);

				if (!wait_for_completion_timeout(&hdmi.phy_completion,
										msecs_to_jiffies(100)))
				DSSERR("timeout waiting for hdmi phy\n");
			} else
				spin_unlock_irqrestore(&hdmi.phy_lock, flags);

		} else if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE ||
			   (hdmi.hdmi_switch.state != HDMI_NOT_AVAILABLE)) {
			/* powered down after enable - skip EDID read */
			goto done;
		} else if (hdmi_read_edid(&dssdev->panel.timings)) {
			/* get monspecs from edid */
			hdmi_get_monspecs(&dssdev->panel.monspecs);
			pr_info("panel size %d by %d\n",
					dssdev->panel.monspecs.max_x,
					dssdev->panel.monspecs.max_y);
			dssdev->panel.width_in_um =
					dssdev->panel.monspecs.max_x * 10000;
			dssdev->panel.height_in_um =
					dssdev->panel.monspecs.max_y * 10000;
			hdmi.edid_valid = true;
			switch_set_state(&hdmi.hdmi_switch, HDMI_EDID_AVAILABLE);
			switch_set_state(&hdmi.underscan_switch, (dssdev->panel.monspecs.misc & FB_MISC_UNDERSCAN)?1:0);
			goto done;
		} else if (state == HPD_STATE_EDID_TRYLAST){
			pr_info("Failed to read EDID after %d times. Giving up.", state - HPD_STATE_START);
			goto done;
		}
		if (atomic_add_unless(&d->state, 1, HPD_STATE_OFF))
			queue_delayed_work(my_workq, &d->dwork, msecs_to_jiffies(60));
	}
done:
	mutex_unlock(&hdmi.hdmi_lock);
}

int hdmi_panel_hpd_handler(int hpd)
{
	__cancel_delayed_work(&hpd_work.dwork);
	atomic_set(&hpd_work.state, hpd ? HPD_STATE_START : HPD_STATE_OFF);
	queue_delayed_work(my_workq, &hpd_work.dwork, msecs_to_jiffies(hpd ? 40 : 30));
	return 0;
}

static void hdmi_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	mutex_lock(&hdmi.hdmi_lock);

	*timings = dssdev->panel.timings;

	mutex_unlock(&hdmi.hdmi_lock);
}

static int hdmi_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	int r = 0;

	DSSDBG("hdmi_check_timings\n");

	mutex_lock(&hdmi.hdmi_lock);

	r = omapdss_hdmi_display_check_timing(dssdev, timings);
	if (r) {
		DSSERR("Timing cannot be applied\n");
		goto err;
	}
err:
	mutex_unlock(&hdmi.hdmi_lock);
	return r;
}

static int hdmi_get_modedb(struct omap_dss_device *dssdev,
			   struct fb_videomode *modedb, int modedb_len)
{
	struct fb_monspecs *specs = &dssdev->panel.monspecs;
	if (specs->modedb_len < modedb_len)
		modedb_len = specs->modedb_len;
	memcpy(modedb, specs->modedb, sizeof(*modedb) * modedb_len);
	return modedb_len;
}

int omapdss_hdmi_panel_display_set_mode(struct omap_dss_device *dssdev,
				  struct fb_videomode *vm) {
	if (!omapdss_hdmi_display_set_mode(dssdev, vm)) {
		switch_set_state(&hdmi.hdmi_switch, HDMI_AVAILABLE);
		return 0;
	}
	return -1;
}

static void hdmi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	struct fb_videomode t;
	omapfb_dss2fb_timings(timings, &t);
	omapdss_hdmi_panel_display_set_mode(dssdev, &t);
}


static struct omap_dss_driver hdmi_driver = {
	.probe		= hdmi_panel_probe,
	.remove		= hdmi_panel_remove,
	.enable		= hdmi_panel_enable,
	.disable	= hdmi_panel_disable,
	.suspend	= hdmi_panel_suspend,
	.resume		= hdmi_panel_resume,
	.get_timings	= hdmi_get_timings,
	.set_timings	= hdmi_set_timings,
	.check_timings	= hdmi_check_timings,
	.get_modedb	= hdmi_get_modedb,
	.set_mode	= omapdss_hdmi_panel_display_set_mode,
	.driver			= {
		.name   = "hdmi_panel",
		.owner  = THIS_MODULE,
	},
};

int hdmi_panel_init(void)
{
	hdmi.connected = 0;
	spin_lock_init(&hdmi.phy_lock);
	init_completion(&hdmi.phy_completion);
	mutex_init(&hdmi.hdmi_lock);
	hdmi.hdmi_switch.name = "hdmi";
	hdmi.underscan_switch.name = "underscan";
	hdmi.edid_valid = false;
	switch_dev_register(&hdmi.hdmi_switch);
	switch_dev_register(&hdmi.underscan_switch);

	my_workq = create_singlethread_workqueue("hdmi_hotplug");
	INIT_DELAYED_WORK(&hpd_work.dwork, hdmi_hotplug_detect_worker);
	omap_dss_register_driver(&hdmi_driver);

	return 0;
}

void hdmi_panel_exit(void)
{
	destroy_workqueue(my_workq);
	omap_dss_unregister_driver(&hdmi_driver);

	switch_dev_unregister(&hdmi.underscan_switch);
	switch_dev_unregister(&hdmi.hdmi_switch);
}

void hdmi_panel_phy_connected(void)
{
	DSSDBG("phy_connection_completion\n");
	spin_lock(&hdmi.phy_lock);
	hdmi.connected = 1;
	complete(&hdmi.phy_completion);
	spin_unlock(&hdmi.phy_lock);
}

void hdmi_panel_phy_disconnected(void)
{
	spin_lock(&hdmi.phy_lock);
	hdmi.connected = 0;
	spin_unlock(&hdmi.phy_lock);
}

module_param (overscan_percent, int, 0644);
