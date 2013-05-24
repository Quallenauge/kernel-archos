/*
 * Toshiba TC358766XBG DSI-to-DP bridge
 *
 * Copyright (C) Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
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

#define DEBUG

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <plat/mcspi.h>

#include <video/omapdss.h>
#include <video/omap-panel-tc358766xbg.h>
#include <video/mipi_display.h>

#include "panel-tc358766xbg.h"

#define W(reg, val) \
	tc358766xbg_spi_write(reg, val); \
	pr_debug("WRITE %20s (0x%04x) = 0x%08x\n", # reg, reg, val);

#define R(reg, val) \
	tc358766xbg_spi_read(reg, (u32*)&val); \
	pr_debug("READ  %20s (0x%04x) = 0x%08x\n", # reg, reg, (u32)(*((u32*)&val)));

static int start_link(struct omap_dss_device *dssdev);
static struct omap_video_timings tc358766xbg_timings;

/* device private data structure */
struct tc358766xbg_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;

	int config_channel;
	int pixel_channel;
};

struct tc358766xbg_spi {
	struct spi_device *spi;
	struct mutex mutex;
} *tc_spi;

static int showColorBar = 0;

static uint32_t
pack_aux(uint8_t *src, int src_bytes)
{
	int	i;
	uint32_t v = 0;

	if (src_bytes > 4)
		src_bytes = 4;
	for (i = 0; i < src_bytes; i++)
		v |= ((uint32_t) src[i]) << (i * 8);
	return v;
}

static void
unpack_aux(uint32_t src, uint8_t *dst, int dst_bytes)
{
	int i;
	if (dst_bytes > 4)
		dst_bytes = 4;
	for (i = 0; i < dst_bytes; i++)
		dst[i] = src >> (i * 8);
}

static int tc358766_read_block(u16 reg, u8 *data, int len)
{
	struct spi_message	m;
	struct spi_transfer	*x, xfer[2];
	int			r;

	u16 buff[3];
	u8 recept[6];
	BUG_ON(tc_spi->spi == NULL);

	spi_message_init(&m);

	memset(xfer, 0, sizeof(xfer));

	x = &xfer[0];

	buff[0] =  reg | 1 ;
	buff[1] =  0;
	buff[2] =  0;

	x->tx_buf = buff;
	x->bits_per_word = 16;
	x->len = 3;
	x->rx_buf = recept;

	spi_message_add_tail(x, &m);

	r = spi_sync(tc_spi->spi, &m);
	if (r < 0)
		dev_dbg(&tc_spi->spi->dev, "spi_sync %d\n", r);

	spi_message_init(&m);
	
	x++;
	buff[0] = 0;
	x->tx_buf = buff;
	x->bits_per_word = 8;
	x->rx_buf = recept;
	x->len = 6;
	spi_message_add_tail(x, &m);

	r = spi_sync(tc_spi->spi, &m);
	if (r < 0)
		dev_dbg(&tc_spi->spi->dev, "spi_sync %d\n", r);

	data[0] = recept[2];
	data[1] = recept[3];
	data[2] = recept[4];
	data[3] = recept[5];
	
	return len;
}

static int tc358766xbg_spi_read(u16 reg, u32 *val)
{
	int r;
	u8 data[4];
	data[0] = data[1] = data[2] = data[3] = 0;

	r = tc358766_read_block(reg, data, 4);
	if (r != 4)
		return -EIO;

	*val = ((int)data[0] << 24) | ((int)(data[1]) << 16) |
	    ((int)(data[2]) << 8) | ((int)(data[3]));
	return 0;
}

static int tc358766xbg_spi_write(u16 reg, u32 val)
{
	struct spi_message	m;
	struct spi_transfer	xfer;
	int			r;

	u8 buff[6];
	u8 recept[6];
	BUG_ON(tc_spi->spi == NULL);

	spi_message_init(&m);

	memset(&xfer, 0, sizeof(xfer));

	buff[0] =  (reg >> 8) & 0xFF;
	buff[1] =  (reg >> 0) & 0xFE;
	buff[2] =  (val >> 24) & 0xFF;
	buff[3] =  (val >> 16) & 0xFF;
	buff[4] =  (val >> 8) & 0xFF;
	buff[5] =  (val >> 0) & 0xFF;

	xfer.tx_buf = buff;
	xfer.bits_per_word = 8;
	xfer.len = 6;
	xfer.rx_buf = recept;
	
	spi_message_add_tail(&xfer, &m);

	r = spi_sync(tc_spi->spi, &m);

	if (r < 0)
		dev_dbg(&tc_spi->spi->dev, "spi_sync %d\n", r);

	return 0;
}

static void tc358766xbg_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void tc358766xbg_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	if (dssdev->type != OMAP_DISPLAY_TYPE_DSI)
		dpi_set_timings(dssdev, timings);
}

static int tc358766xbg_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	if (tc358766xbg_timings.x_res != timings->x_res ||
			tc358766xbg_timings.y_res != timings->y_res ||
			tc358766xbg_timings.pixel_clock != timings->pixel_clock ||
			tc358766xbg_timings.hsw != timings->hsw ||
			tc358766xbg_timings.hfp != timings->hfp ||
			tc358766xbg_timings.hbp != timings->hbp ||
			tc358766xbg_timings.vsw != timings->vsw ||
			tc358766xbg_timings.vfp != timings->vfp ||
			tc358766xbg_timings.vbp != timings->vbp)
		return -EINVAL;

	return 0;
}

static void tc358766xbg_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = tc358766xbg_timings.x_res;
	*yres = tc358766xbg_timings.y_res;
}

static int tc358766xbg_hw_reset(struct omap_dss_device *dssdev)
{
	if (dssdev == NULL)
		return 0;
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	return 0;
}

static ssize_t tc358766xbg_num_errors_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	u32 val;

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI) {
		dsi_bus_lock(dssdev);
		dsi_video_mode_disable(dssdev);
	}

	R(SYSCTRL, val);

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI) {
		switch (dssdev->ctrl.pixel_size) {
			case 18:
				dsi_video_mode_enable(dssdev, 0x1e);
				break;
			case 24:
				dsi_video_mode_enable(dssdev, 0x3e);
				break;
			default:
				dev_warn(&dssdev->dev, "not expected pixel size: %d\n",
						dssdev->ctrl.pixel_size);
		}

		dsi_bus_unlock(dssdev);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static void tc358766xbg_disable(struct omap_dss_device *dssdev);
static int tc358766xbg_enable(struct omap_dss_device *dssdev);

static ssize_t colorbar_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	int enable = 0;
	int r;
	r = sscanf(buf, "%d", &enable);
	if (r != 1) {
		printk("sscanf failed: %d\n", r);
		return -EINVAL;
	}
	if (enable) showColorBar = 1;
	else showColorBar = 0;
	tc358766xbg_disable(dssdev);
	tc358766xbg_enable(dssdev);
	return count;
}

static ssize_t wr_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	u16 reg;
	u32 value;
	int r;
	char s1[64], s2[64];

	msleep(50);

	r = sscanf(buf, "%s %s", s1, s2);
	if (r != 2) {
		printk("sscanf failed: %d\n", r);
		return -EINVAL;
	}

	r = kstrtou16(s1, 0, &reg);
	if (r) {
		printk("parse reg failed\n");
		return r;
	}

	r = kstrtou32(s2, 0, &value);
	if (r) {
		printk("parse value failed\n");
		return r;
	}

	r = tc358766xbg_spi_write(reg, value);
	if (r) {
		printk("spi write failed: %d\n", r);
		return r;
	}

	printk("\nreg write 0x%04x = %08x\n", reg, value);

	return count;
}

static ssize_t rd_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	u16 reg;
	u32 value;
	int r;

	msleep(50);

	r = kstrtou16(buf, 0, &reg);
	if (r) {
		printk("parse failed\n");
		return r;
	}

	r = R(reg, value);
	if (r) {
		printk("spi read failed: %d\n", r);
		return r;
	}

	printk("\nreg read 0x%04x = %08x\n", reg, value);

	return count;
}

static ssize_t dwr_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	u16 reg;
	u32 value;
	int r;
	char s1[64], s2[64];

	msleep(50);

	r = sscanf(buf, "%s %s", s1, s2);
	if (r != 2) {
		printk("sscanf failed: %d\n", r);
		return -EINVAL;
	}

	r = kstrtou16(s1, 0, &reg);
	if (r) {
		printk("parse reg failed\n");
		return r;
	}

	r = kstrtou32(s2, 0, &value);
	if (r) {
		printk("parse value failed\n");
		return r;
	}

	mutex_lock(&d2d->lock);
	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
		dsi_bus_lock(dssdev);

	r = W(reg, value);
	/*
	if (r) {
		printk("write failed\n");
	} else {
		r = dsi_vc_send_bta_sync(dssdev, d2d->config_channel);
		if (r)
			printk("bta failed\n");
	}
*/
	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
		dsi_bus_unlock(dssdev);

	mutex_unlock(&d2d->lock);

	if (r) {
		printk("reg write failed\n");
		return r;
	}

//	printk("\nreg write 0x%04x = %08x\n", reg, value);

	return count;
}

static ssize_t drd_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	u16 reg;
	u32 value;
	int r;

	msleep(50);

	r = kstrtou16(buf, 0, &reg);
	if (r) {
		printk("parse failed\n");
		return r;
	}

	mutex_lock(&d2d->lock);

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
		dsi_bus_lock(dssdev);

	r = R(reg, value);

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
		dsi_bus_unlock(dssdev);

	mutex_unlock(&d2d->lock);

	if (r) {
		printk("dsi read failed: %d\n", r);
		return r;
	}

	printk("\nreg read 0x%04x = %08x\n", reg, value);

	return count;
}

static ssize_t reg_dump_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	u32 value;
	int count = 0;

	mutex_lock(&d2d->lock);

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
		dsi_bus_lock(dssdev);

#define DUMP(reg) \
	R(reg, value); \
	count += sprintf(buf + count, "%20s = 0x%08x\n", # reg, value);

	DUMP(DSI_BUSYDSI);
	DUMP(DSI_INTSTATUS);
	DUMP(PPI_BUSYPPI);
	DUMP(VPCTRL0);
	DUMP(HTIM01);
	DUMP(HTIM02);
	DUMP(VTIM01);
	DUMP(VTIM02);
	DUMP(VFUEN0);
	DUMP(SYSCTRL);
	DUMP(SYSSTAT);
	DUMP(DP0Ctl);
	DUMP(DP0_VIDMNGEN0);
	DUMP(DP0_VIDMNGEN1);
	DUMP(DP0_VIDMNGENSTATUS);
	DUMP(DP0_VIDSYNCDELAY);
	DUMP(DP0_TOTALVAL);
	DUMP(DP0_STARTVAL);
	DUMP(DP0_ACTIVEVAL);
	DUMP(DP0_SYNCVAL);
	DUMP(DP0_MISC);
	DUMP(DP0_AUXCFG0);
	DUMP(DP0_AUXCFG1);
	DUMP(DP0_AUXADDR);
	DUMP(DP0_AUXWDATA(0));
	DUMP(DP0_AUXRDATA(0));
	DUMP(DP0_AUXRDATA(1));
	DUMP(DP0_AUXSTATUS);
	DUMP(DP0_AUXI2CADR);
	DUMP(DP0_SRCCTRL);
	DUMP(DP0_LTSTAT);
	DUMP(DP0_LTLOOPCTRL);
	DUMP(DP0_SNKLTCTRL);
	DUMP(DP_PHY_CTRL);
	DUMP(DP0_PLLCTRL);
	DUMP(DP1_PLLCTRL);
	DUMP(PXL_PLLCTRL);
	DUMP(PXL_PLLPARAM);
	DUMP(SYS_PLLPARAM);
	DUMP(D2DPTSTCTL);
	DUMP(DSI_LANESTATUS0);
	DUMP(DSI_LANESTATUS1);
	DUMP(DSI_INTSTATUS);


	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
		dsi_bus_unlock(dssdev);
	mutex_unlock(&d2d->lock);

	return count;
}

static DEVICE_ATTR(num_dsi_errors, S_IRUGO, tc358766xbg_num_errors_show, NULL);
static DEVICE_ATTR(wr, S_IWUSR, NULL, wr_store);
static DEVICE_ATTR(colorbar, S_IWUSR, NULL, colorbar_store);
static DEVICE_ATTR(rd, S_IWUSR, NULL, rd_store);
static DEVICE_ATTR(dwr, S_IWUSR, NULL, dwr_store);
static DEVICE_ATTR(drd, S_IWUSR, NULL, drd_store);
static DEVICE_ATTR(reg_dump, S_IRUGO, reg_dump_show, NULL);

static struct attribute *tc358766xbg_attrs[] = {
	&dev_attr_num_dsi_errors.attr,
	&dev_attr_wr.attr,
	&dev_attr_colorbar.attr,
	&dev_attr_rd.attr,
	&dev_attr_dwr.attr,
	&dev_attr_drd.attr,
	&dev_attr_reg_dump.attr,
	NULL,
};

static struct attribute_group tc358766xbg_attr_group = {
	.attrs = tc358766xbg_attrs,
};

static int tc358766xbg_probe(struct omap_dss_device *dssdev)
{
	struct tc358766xbg_data *d2d;
	int r = 0;

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	tc358766xbg_timings = dssdev->panel.timings;

	d2d = kzalloc(sizeof(*d2d), GFP_KERNEL);
	if (!d2d) {
		r = -ENOMEM;
		goto err0;
	}

	d2d->dssdev = dssdev;

	mutex_init(&d2d->lock);

	dev_set_drvdata(&dssdev->dev, d2d);

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI) {
		r = omap_dsi_request_vc(dssdev, &d2d->pixel_channel);
		if (r) {
			printk( "failed to get virtual channel for"
					" transmitting pixel data\n");
			goto err0;
		}

		r = omap_dsi_set_vc_id(dssdev, d2d->pixel_channel, 0);
		if (r) {
			printk( "failed to set VC_ID for pixel data"
					" virtual channel\n");
			goto err1;
		}

		r = omap_dsi_request_vc(dssdev, &d2d->config_channel);
		if (r) {
			printk( "failed to get virtual channel for"
					"configuring bridge\n");
			goto err1;
		}

		r = omap_dsi_set_vc_id(dssdev, d2d->config_channel, 1);
		if (r) {
			printk( "failed to set VC_ID for config"
					" channel\n");
			goto err2;
		}
	} else {
		// setup timing configs for dpi

	}

	r = sysfs_create_group(&dssdev->dev.kobj, &tc358766xbg_attr_group);
	if (r) {
		printk( "failed to create sysfs files\n");
		//goto err_vc_id;
	}

	dev_dbg(&dssdev->dev, "%s probe done.\n", __func__);

	return 0;

err2:
	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
		omap_dsi_release_vc(dssdev, d2d->config_channel);
err1:
	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
		omap_dsi_release_vc(dssdev, d2d->pixel_channel);
err0:
	kfree(d2d);

	return r;
}

static void tc358766xbg_remove(struct omap_dss_device *dssdev)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);

	sysfs_remove_group(&dssdev->dev.kobj, &tc358766xbg_attr_group);

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI) {
		omap_dsi_release_vc(dssdev, d2d->pixel_channel);
		omap_dsi_release_vc(dssdev, d2d->config_channel);
	}

	kfree(d2d);
}

static void read_lt_status(struct omap_dss_device *dssdev)
{
	u32 value;

	R(DP0_LTSTAT, value);

	printk("\tLT status %x, ", (value >> 8) & 0x1f);
	if ((value & (1 << 0)) != 0) printk("CRDone0 ");
	if ((value & (1 << 1)) != 0) printk("EQDone0 ");
	if ((value & (1 << 2)) != 0) printk("SymLck0 ");
	if ((value & (1 << 3)) != 0) printk("ILAlign ");
	if ((value & (1 << 4)) != 0) printk("CRDone1 ");
	if ((value & (1 << 5)) != 0) printk("EQDone1 ");
	if ((value & (1 << 6)) != 0) printk("SymLck1 ");
	if ((value & (1 << 13)) != 0) printk("LoopDone ");
	printk("\n");
}

struct auxstatus
{
	bool busy:1;
	bool timeout:1;
	unsigned reserved1:2;
	unsigned status:4;
	unsigned bytes:8;
	bool err_sync:1;
	bool err_stop:1;
	bool err_align:1;
	unsigned reserved2:1;
	unsigned retry:3;
	unsigned reserved3:9;
} __attribute__ ((__packed__));

static int aux_nat_read(struct omap_dss_device *dssdev, u32 addr, u8 *buf,
		int len)
{
	const u32 native_read_cmd = 0x9;
	int r;
	int retries = 10;
	struct auxstatus auxstat;
	int n;

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	r = W(DP0_AUXADDR, addr);
	r = W(DP0_AUXCFG0,
			((len - 1) << 8) | native_read_cmd);

	while (--retries > 0) {
		mdelay(1);

		r = R(DP0_AUXSTATUS, auxstat);

		if ((auxstat.busy == false) && 
		    (auxstat.err_sync == false) &&
		    (auxstat.err_stop == false) &&
		    (auxstat.err_align == false))
			break;
	}

	if (retries == 0) {
		printk("failed\n");
		return -EIO;
	}

	if (auxstat.timeout) {
		printk("timeout\n");
		return -EIO;
	}

	if (auxstat.err_sync || auxstat.err_stop || auxstat.err_align) {
		printk("AUX error on read: ");

		if (auxstat.err_sync)
			printk("ErrSync ");
		if (auxstat.err_stop)
			printk("ErrStop ");
		if (auxstat.err_align)
			printk("ErrAlign ");

		printk("\n");

		return -EIO;
	}

	if (auxstat.status != 0) {
		printk("bad aux status %x\n", auxstat.status);
		return -EIO;
	}

	if (len != auxstat.bytes) {
		printk("bad read len %d\n", auxstat.bytes);
		return -EIO;
	}

	for (n = 0; n < len; n+=4)
	{
		u32 val;
		r = R(DP0_AUXRDATA(n/4), val); //min(4, len - n * 4));
		unpack_aux(val, &buf[n], len - n);
	}

	return 0;
}

static int aux_nat_write(struct omap_dss_device *dssdev, u32 addr, u8 *buf,
		int len)
{
	const u32 native_write_cmd = 0x8;
	int r;
	int retries = 10;
	struct auxstatus auxstat;
	int n;
	int i;

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	for (i = 0; i < len; ++i)
		printk("%02x ", buf[i]);

	printk("\n");

	r = W(DP0_AUXADDR, addr);

	for (n = 0; n < len; n+=4)
	{
		r = W(DP0_AUXWDATA(n/4), pack_aux(buf + n, len - n));
	}

	r = W(DP0_AUXCFG0,
			((len - 1) << 8) | native_write_cmd);

	while (--retries > 0) {
		mdelay(1);

		r = R(DP0_AUXSTATUS,auxstat);

		if ((auxstat.busy == false) && 
		    (auxstat.err_sync == false) &&
		    (auxstat.err_stop == false) &&
		    (auxstat.err_align == false))
			break;
	}

	if (retries == 0) {
		printk("failed\n");
		return -EIO;
	}

	if (auxstat.timeout) {
		printk("timeout\n");
		return -EIO;
	}

	if (auxstat.err_sync || auxstat.err_stop || auxstat.err_align) {
		printk("AUX error on write: ");

		if (auxstat.err_sync)
			printk("ErrSync ");
		if (auxstat.err_stop)
			printk("ErrStop ");
		if (auxstat.err_align)
			printk("ErrAlign ");

		printk("\n");

		return -EIO;
	}

	if (auxstat.status != 0) {
		printk("bad aux status %x\n", auxstat.status);
		return -EIO;
	}

	if (len != auxstat.bytes) {
		printk("bad write len %d\n", auxstat.bytes);
		return -EIO;
	}

	return 0;
}

static int aux_write_1(struct omap_dss_device *dssdev, u32 addr, u8 b1)
{
	u8 buf[] = { b1 };

	return aux_nat_write(dssdev, addr, buf, 1);
}

static int aux_write_2(struct omap_dss_device *dssdev, u32 addr, u8 b1, u8 b2)
{
	u8 buf[] = { b1, b2 };

	return aux_nat_write(dssdev, addr, buf, 2);
}

static void aux_dump(struct omap_dss_device *dssdev, u32 addr, int len)
{
	u8 buf[4*4];
	int i;
	int r;

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	r = aux_nat_read(dssdev, addr, buf, len);

	if (r) {
		printk("AUX READ  0x%04x FAILED\n", addr);
		return;
	}

	printk("AUX READ  0x%04x = ", addr);

	for (i = 0; i < len; ++i)
		printk("%02x ", buf[i]);

	printk("\n");
}

static int do_link_training(struct omap_dss_device *dssdev)
{
	u32 value;

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	// default values
	W(DP0_AUXCFG1,	0x32 |	// timer
			(0x7 << 8) |  // threshold
			(1 << 16)); // filter enable

	// MAX_LINK_RATE, 0x6 = 1.62 Gbps
	aux_dump(dssdev, 0x1, 1);

	// MAX_LANE_COUNT (0x84, 4 lanes)
	aux_dump(dssdev, 0x2, 1);

	// LINK_BW_SET & LANE_COUNT_SET
	aux_write_2(dssdev, 0x100, 0xA, 0x2); // 2.7Gbps, two lane

	aux_dump(dssdev, 0x100, 2);

	// MAIN_LINK_CHANNEL_CODING_SET
	aux_write_1(dssdev, 0x108, 0x1); // SET_ANSI 8B10B

	aux_dump(dssdev, 0x108, 1);


	// clear LT irq
	W(INTSTS_G, (1 << 1));


	W(DP0_LTLOOPCTRL,
			(0xd << 0) | // Loop timer delay
			(0x6 << 24) | // LoopIter
			(0xf << 28)); // DeferIter

	// Data to be written to Sink LT Control register at address 0x00102
	W(DP0_SNKLTCTRL, 0x21);
	W(DP0_SRCCTRL,	(1 << 0) | // auto correct
			(1 << 1) | // 2.7Gbps
			(1 << 2) | // two main channel lanes
			(1 << 7) | // lane skew
			(1 << 8) | // training pattern 1
			(1 << 12) | // EN810B
			(1 << 13)); // SCRMBL

	W(DP0Ctl, 0x1); // dp_en
	
	{ 
		int i;
		u32 val;
		for (i = 0; i < 10; ++i) {
			R(DP0_LTSTAT, val);
			if (val & 0x11)
				break;
			mdelay(1);
		}
		if (i == 10)
			printk("LT timeout 1\n");
	}
	read_lt_status(dssdev);

	// SINK_COUNT + 5
	aux_dump(dssdev, 0x200, 5);


	// clear LT irq
	W(INTSTS_G, (1 << 1));

	// Data to be written to Sink LT Control register at address 0x00102
	W(DP0_SNKLTCTRL, 0x22);
	W(DP0_SRCCTRL,	(1 << 0) | // auto correct
			(1 << 1) | // 2.7Gbps
			(1 << 2) | // two main channel lanes
			(1 << 7) | // lane skew
			(2 << 8) | // training pattern 2
			(1 << 12) | // EN810B
			(1 << 13)); // SCRMBL

	{
		int i;
		u32 val;
		for (i = 0; i < 10; ++i) {
			R(DP0_LTSTAT, val);
			if (val & 0x77)
				break;
			mdelay(1);
		}
		if (i == 10)
			printk("LT timeout 2\n");
	}

	// clear LT irq
	W(INTSTS_G, (1 << 1));


	R(DP0_LTSTAT, value);
	read_lt_status(dssdev);

	// SINK_COUNT + 5
	aux_dump(dssdev, 0x200, 5);

	// TRAINING_PATTERN_SET
	aux_write_1(dssdev, 0x102, 0);

	W(DP0_SRCCTRL,	(1 << 0) | // auto correct
			(1 << 1) | // 2.7Gbps
			(1 << 2) | // two main channel lanes
			(1 << 7) | // lane skew
			(1 << 12)); // EN810B

	// SINK_COUNT + 5
	aux_dump(dssdev, 0x200, 5);
	printk("Expected = 01 00 77 00 00\n");

	return 0;
}

static int dpi_config(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	W(DPIPXLFMT, (1 << 10) | // vsync active low
		     (1 << 9)); // hsync active low
				// RGB888 / DE active hi

	return 0;
}

static int dsi_config(struct omap_dss_device *dssdev)
{
	const int lp_tx_cnt = 6;

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	W(PPI_LPTXCNT, lp_tx_cnt);	// LPTXTIMECNT
	W(PPI_TX_RX_TA, (((5*lp_tx_cnt -3)/4) << 16) | (int)(1.5*lp_tx_cnt)); // TXTAGOCNT & TXTASURECNT

	W(PPI_D0S_CLRSIPOCOUNT, 10);
	W(PPI_D1S_CLRSIPOCOUNT, 10);
	W(PPI_D2S_CLRSIPOCOUNT, 10);
	W(PPI_D3S_CLRSIPOCOUNT, 10);

	W(PPI_LANEENABLE, 0x1f);	// PPI enable 4 + 1 lanes

	W(DSI_LANEENABLE, 0x1f);	// DSI enable 4 + 1 lanes

	W(PPI_STARTPPI, 1);
	W(DSI_STARTDSI, 1);

	return 0;
}

static int main_config(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);
	
	W(DP0_SRCCTRL,	(1 << 7) | // lane skew
			(1 << 1) | // 2.7 Gbps
			(1 << 2) | // two main channel lanes
			(1 << 12) | // EN810B
			(1 << 13)); // SCRMBL

	W(DP1_SRCCTRL,	(1 << 1)); // 2.7 Gbps

	W(SYS_PLLPARAM,	(1 << 0) | // LS_CLK_DIV = 2
			(0 << 4) | // use LS CLK
			(1 << 8)); // RefClk freq = 19.2

	msleep(100);

	return 0;
}

struct dp_timings
{
	unsigned x_res, hfp, hsw, hbp;
	unsigned y_res, vfp, vsw, vbp;
	bool hs_pol, vs_pol;
};

static struct dp_timings dp_timings_cmi_1920x1080 = { 
	.x_res = 1920,
	.hfp = 70,
	.hsw = 20,
	.hbp = 70,
	.hs_pol = true,

	.y_res = 1080,
	.vfp = 13,
	.vsw = 5,
	.vbp = 13,
	.vs_pol = true,
};

static int config_dp0_timings(struct omap_dss_device *dssdev,
		struct dp_timings *t)
{
	unsigned htot, hstart;
	unsigned vtot, vstart;
	unsigned thresh_dly, vid_sync_dly;

	htot = t->x_res + t->hfp + t->hsw + t->hbp;
	hstart = t->hsw + t->hbp;

	vtot = t->y_res + t->vfp + t->vsw + t->vbp;
	vstart = t->vsw + t->vbp;

	thresh_dly = 31;
	vid_sync_dly = t->hsw + t->hbp + t->x_res;

	W(DP0_VIDSYNCDELAY, (thresh_dly << 16) | vid_sync_dly);
	W(DP0_TOTALVAL, (vtot << 16) | htot);
	W(DP0_STARTVAL, (vstart << 16) | hstart);
	W(DP0_ACTIVEVAL, (t->y_res << 16) | t->x_res);
	W(DP0_SYNCVAL,	((t->vs_pol ? 1 : 0) << 31) | (t->vsw << 16) |
			((t->hs_pol ? 1 : 0) << 15) | t->hsw);

	return 0;
}

static int config_vtgen(struct omap_dss_device *dssdev,
		struct dp_timings *t)
{
	W(HTIM01, (t->hbp << 16) | t->hsw);
	W(HTIM02, (t->hfp << 16) | t->x_res);
	W(VTIM01, (t->vbp << 16) | t->vsw);
	W(VTIM02, (t->vfp << 16) | t->y_res);
	W(VFUEN0, 1); // VFUEN

	return 0;
}

static unsigned long dp_pclk; // XXX

struct dp_divs
{
	unsigned fbd;
	unsigned pre_div;
	unsigned ext_pre_div;
	unsigned ext_post_div;
	unsigned in_sel;
};

static struct dp_divs dp_divs_cmi_1920x1080 = {
	 //Pxl clock = 19.2*53/8
	.fbd = 53,
	.pre_div = 4,

	.ext_post_div = 2,
	.ext_pre_div = 2,
	.in_sel = 0,
};

static int config_pxl_pll(struct omap_dss_device *dssdev)
{
	unsigned long refclk = 19200000; // RefClk
	struct dp_divs *d;
	unsigned long vco;
	u32 v;

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	d = &dp_divs_cmi_1920x1080;

	vco = (refclk / 1000) * d->fbd / d->ext_pre_div / d->pre_div;
	vco /= 1000;

	dp_pclk = vco / d->ext_post_div;

	dev_dbg(&dssdev->dev, "DP PCLK %lu Hz\n", dp_pclk);

	v = 0;
	v |= d->fbd == 128 ? 0 : d->fbd;
	v |= (d->pre_div == 16 ? 0 : d->pre_div) << 8;
	v |= d->in_sel << 14;
	v |= d->ext_post_div << 16;
	v |= d->ext_pre_div << 20;
	v |= (vco >= 300 ? 1 : 0) << 24;

	W(PXL_PLLPARAM,	v);

	W(PXL_PLLCTRL,	(1 << 0) |	// Enable PLL
			(1 << 2));	// PLL UPDATE
	msleep(100);

	return 0;
}

static int start_link(struct omap_dss_device *dssdev)
{
	struct dp_timings *dpt;
	int retry;
	u32 value;

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	dpt = &dp_timings_cmi_1920x1080;

	W(DP0_SRCCTRL,	(1 << 7) | // lane skew
			(1 << 1) | // 2.7 Gbps
			(1 << 2) | // two main channel lanes
			(1 << 12) | // EN810B
			(1 << 13)); // SCRMBL

	W(DP1_SRCCTRL,	(1 << 1)); // 2.7 Gbps
	

	W(DP_PHY_CTRL,	(1 << 0) | // PHY Main Channel0 enable
			(1 << 1) | // PHY Aux Channel0 enable
			(1 << 2) | // LaneSel, dual channel
			(1 << 24) | // PHY Power Switch Enable
			(1 << 25)); // AUX PHY BGR Enable

	W(DP0_PLLCTRL,	(1 << 0) |	// Enable PLL
			(1 << 2));	// PLL UPDATE
	W(DP1_PLLCTRL,	(1 << 0) |	// Enable PLL
			(1 << 2));	// PLL UPDATE

	msleep(100);

	config_pxl_pll(dssdev);


	R(DP_PHY_CTRL, value);

	W(DP_PHY_CTRL,	(1 << 0) | // PHY Main Channel0 enable
			(1 << 1) | // PHY Aux Channel0 enable
			(1 << 2) | // LaneSel, dual channel
			(1 << 24) | // PHY Power Switch Enable
			(1 << 25) | // AUX PHY BGR Enable
			(1 << 8) | // PHY Main Channel0 Reset
			(1 << 12) | // PHY Main Channel1 Reset
			(1 << 28)); // DP PHy Global Soft Reset

	W(DP_PHY_CTRL,	(1 << 0) | // PHY Main Channel0 enable
			(1 << 1) | // PHY Aux Channel0 enable
			(1 << 2) | // LaneSel, dual channel
			(1 << 24) | // PHY Power Switch Enable
			(1 << 25)); // AUX PHY BGR Enable

	value = 0;
	retry = 50000;
	// wait for approx 1sec.
	while (!(value & 0x10000) && retry) {
		udelay(100);
		R(DP_PHY_CTRL, value);
		retry--;
	}

	if (!retry)
		dev_err(&dssdev->dev, "%s: abort\n", __func__);

	do_link_training(dssdev);

	msleep(100);

	if (showColorBar) {
		W(D2DPTSTCTL,	2 | // color bar
			(1 << 4) | // i2c filter
			(0x63 << 8) |	// B
			(0x14 << 16) |	// G
			(0x78 << 24));	// R
	} else {
 		W(D2DPTSTCTL,0);
	}


	W(VPCTRL0,	(0x45 << 20) | // VSDELAY)
			(1 << 8) |	// RGB666/RGB888
			(1 << 4)); // enable/disable VTGen


	config_vtgen(dssdev, dpt);
	config_dp0_timings(dssdev, dpt);

	W(DP0_MISC,	(0 << 0) | // sync_m
			(1 << 5) | // 8 bits per color component RGB888
			(0x3f << 16) |	// tu_size
			(0x3e << 24));	// max_tu_symbol


	msleep(100);
	W(DP0Ctl,	(1 << 0) | // dp_en
			(1 << 6)); // vid_mn_gen
	W(DP0Ctl,	(1 << 0) | // dp_en
			(1 << 1) | // vid_en
			(1 << 6)); // vid_mn_gen

	if (showColorBar) {
		W(SYSCTRL, 3);	// DP0_VidSrc = Color bar
	} else if (dssdev->type == OMAP_DISPLAY_TYPE_DSI) {
		W(SYSCTRL, 1);	// DP0_VidSrc = DSI
	} else {
		W(SYSCTRL, 2);	// DP0_VidSrc = DPI
	}

	return 0;
}

static int tc358766xbg_power_on(struct omap_dss_device *dssdev)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;
	u32 value;

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	/* At power on the first vsync has not been received yet */
	dssdev->first_vsync = false;
	
	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI) {
		r = omapdss_dsi_display_enable(dssdev);
	} else {
		r = omapdss_dpi_display_enable(dssdev);
	}

	if (r) {
		dev_err(&dssdev->dev, "%s: failed to enable omap dss display.\n", __func__);
		goto err_disp_enable;
	}

	/* reset tc358766xbg bridge */
// Seems not needed anymore, to check
//	tc358766xbg_hw_reset(dssdev);

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
		omapdss_dsi_vc_enable_hs(dssdev, d2d->pixel_channel, true);

	main_config(dssdev);

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
		dsi_config(dssdev);
	else
		dpi_config(dssdev);

	value = 0;
	R( IDREG, value);
	dev_dbg(&dssdev->dev, "%s: %x\n", __func__, value);

	R( IDREG, value);
	dev_dbg(&dssdev->dev, "%s: %x\n", __func__, value);

	R(SYSSTAT, value);
	if (value != 0)
		dev_err(&dssdev->dev, "%s: SYSTAT ERROR.\n", __func__);

	/* GPIOs */
	//W(INTGP0LCNT, 0x2); // GPIO0 irq low count
	W(GPIOM, 0);	// pins are GPIOs
	W(GPIOC, 0);	// GPIOs are inputs
	W(INTCTL_G, 0);
	W(INTSTS_G, 0xffffffff);
	W(INTCTL_G,	(1 << 16) | // SYS ERR
			(1 << 2) | (1 << 3)); // enable GPIO interrupts

	W(DP0_AUXI2CADR, 0xA5);

	R(IDREG, value);

	{
	//Wait for 100 ms to detect the HPD of the screen
		int i;
		for (i = 0; i < 10; ++i) {
			R(GPIOI, value);
			if (value & 1)
				break;
			mdelay(10);
		}
		if (i == 10) {
			dev_err(&dssdev->dev, "%s: HPD timeout.\n", __func__);
			goto err_write_init;
		}
	}

	start_link(dssdev);

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI) {
		omapdss_dsi_vc_enable_hs(dssdev, d2d->config_channel, true);

		switch (dssdev->ctrl.pixel_size) {
			case 18:
				dsi_video_mode_enable(dssdev, 0x1e);
				break;
			case 24:
				dsi_video_mode_enable(dssdev, 0x3e);
				break;
			default:
				dev_warn(&dssdev->dev, "not expected pixel size: %d\n",
						dssdev->ctrl.pixel_size);
		}
	}

	dev_dbg(&dssdev->dev, "%s: power_on done\n", __func__);
	return 0;

err_write_init:
	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI) {
		omapdss_dsi_display_disable(dssdev, false, false);
	} else {
		omapdss_dpi_display_disable(dssdev);
	}

err_disp_enable:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	return r;
}

static void tc358766xbg_power_off(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "%s\n", __func__);

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI) {
		dsi_video_mode_disable(dssdev);
		omapdss_dsi_display_disable(dssdev, false, false);
	} else {
		omapdss_dpi_display_disable(dssdev);
	}

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static void tc358766xbg_disable(struct omap_dss_device *dssdev)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

		mutex_lock(&d2d->lock);
		if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
			dsi_bus_lock(dssdev);

		tc358766xbg_power_off(dssdev);

		if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
			dsi_bus_unlock(dssdev);

		mutex_unlock(&d2d->lock);
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int tc358766xbg_enable(struct omap_dss_device *dssdev)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	mutex_lock(&d2d->lock);

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
		dsi_bus_lock(dssdev);

	r = tc358766xbg_power_on(dssdev);

	if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
		dsi_bus_unlock(dssdev);

	if (r) {
		dev_err(&dssdev->dev, "%s\n", __func__);
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	}

	mutex_unlock(&d2d->lock);

	return r;
}

static int tc358766xbg_suspend(struct omap_dss_device *dssdev)
{
	struct tc358766xbg_data *d2d = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "%s\n", __func__);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		
		dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

		mutex_lock(&d2d->lock);

		if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
			dsi_bus_lock(dssdev);

		tc358766xbg_power_off(dssdev);

		if (dssdev->type == OMAP_DISPLAY_TYPE_DSI)
			dsi_bus_unlock(dssdev);

		mutex_unlock(&d2d->lock);
	}

	return 0;
}

static int tc358766xbg_resume(struct omap_dss_device *dssdev)
{
	tc358766xbg_enable(dssdev);
	return 0;
}

static struct omap_dss_driver tc358766xbg_driver = {
	.probe		= tc358766xbg_probe,
	.remove		= tc358766xbg_remove,

	.enable		= tc358766xbg_enable,
	.disable	= tc358766xbg_disable,
	.suspend	= tc358766xbg_suspend,
	.resume		= tc358766xbg_resume,

	.get_resolution	= tc358766xbg_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= tc358766xbg_get_timings,
	.set_timings	= tc358766xbg_set_timings,
	.check_timings	= tc358766xbg_check_timings,

	.driver         = {
		.name   = "tc358766xbg",
		.owner  = THIS_MODULE,
	},
};
/*--------------------SPI probe-------------------------*/

static int tc358766xbg_spi_probe(struct spi_device *spi)
{

	tc_spi = kzalloc(sizeof(struct tc358766xbg_spi), GFP_KERNEL);
	if (tc_spi == NULL)
		return -ENOMEM;
	tc_spi->spi = spi;
	mutex_init(&tc_spi->mutex);
	dev_set_drvdata(&spi->dev, tc_spi);

	return 0;
}

static int tc358766xbg_spi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver tc358766xbg_spi_driver = {
	.driver = {
		.name	= "tc358766xbg_spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= tc358766xbg_spi_probe,
	.remove	= __devexit_p(tc358766xbg_spi_remove),
};

static int __init tc358766xbg_init(void)
{
	int r = 0;
	tc_spi = NULL;

	r = spi_register_driver(&tc358766xbg_spi_driver);
	if (r < 0) {
		pr_err("DSI/DPI to DP spi driver registration failed\n");
		return r;
	}
	omap_dss_register_driver(&tc358766xbg_driver);
	return 0;
}

static void __exit tc358766xbg_exit(void)
{
	omap_dss_unregister_driver(&tc358766xbg_driver);

	spi_unregister_driver(&tc358766xbg_spi_driver);
}

module_init(tc358766xbg_init);
module_exit(tc358766xbg_exit);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ti.com>");
MODULE_DESCRIPTION("TC358766XBG DSI/DPI-2-LVDS Driver");
MODULE_LICENSE("GPL");
