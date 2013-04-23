/*
 * Toshiba TC358766 DSI-to-Display port chip driver
 *
 * Copyright (C) Texas Instruments
 * Author: @archos.com>
 *
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

#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <plat/mcspi.h>

#include <video/omapdss.h>
#include <video/omap-panel-tc358766.h>
#include "panel-tc358766.h"

#include <drm/drm_dp_helper.h>

static struct omap_video_timings tc358766_timings;

static struct tc358766_board_data *get_board_data(struct omap_dss_device
					*dssdev) __attribute__ ((unused));

/* device private data structure */
struct tc358766_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;

	int channel0;
	int channel1;
};

struct tc358766_spi {
	struct spi_device *spi;
	struct mutex mutex;
} *tc_spi;


#ifdef CONFIG_TC358766_DEBUG
struct {
	struct device *dev;
	struct dentry *dir;
} tc358766_debug;

struct tc358766_reg {
	const char *name;
	u16 reg;
	u8 perm:2;
} tc358766_regs[] = {
	/* DSI D-PHY Layer Registers */
	{ "D0W_DPHYCONTTX", D0W_DPHYCONTTX, A_RW },
	{ "CLW_DPHYCONTRX", CLW_DPHYCONTRX, A_RW },
	{ "D0W_DPHYCONTRX", D0W_DPHYCONTRX, A_RW },
	{ "D1W_DPHYCONTRX", D1W_DPHYCONTRX, A_RW },
	{ "D2W_DPHYCONTRX", D2W_DPHYCONTRX, A_RW },
	{ "D3W_DPHYCONTRX", D3W_DPHYCONTRX, A_RW },
	{ "COM_DPHYCONTRX", COM_DPHYCONTRX, A_RW },
	{ "CLW_CNTRL", CLW_CNTRL, A_RW },
	{ "D0W_CNTRL", D0W_CNTRL, A_RW },
	{ "D1W_CNTRL", D1W_CNTRL, A_RW },
	{ "D2W_CNTRL", D2W_CNTRL, A_RW },
	{ "D3W_CNTRL", D3W_CNTRL, A_RW },
	{ "DFTMODE_CNTRL", DFTMODE_CNTRL, A_RW },
	/* DSI PPI Layer Registers */
	{ "PPI_STARTPPI", PPI_STARTPPI, A_RW },
	{ "PPI_BUSYPPI", PPI_BUSYPPI, A_RO },
	{ "PPI_LINEINITCNT", PPI_LINEINITCNT, A_RW },
	{ "PPI_LPTXTIMECNT", PPI_LPTXTIMECNT, A_RW },
	{ "PPI_LANEENABLE", PPI_LANEENABLE, A_RW },
	{ "PPI_TX_RX_TA", PPI_TX_RX_TA, A_RW },
	{ "PPI_CLS_ATMR", PPI_CLS_ATMR, A_RW },
	{ "PPI_D0S_ATMR", PPI_D0S_ATMR, A_RW },
	{ "PPI_D1S_ATMR", PPI_D1S_ATMR, A_RW },
	{ "PPI_D2S_ATMR", PPI_D2S_ATMR, A_RW },
	{ "PPI_D3S_ATMR", PPI_D3S_ATMR, A_RW },
	{ "PPI_D0S_CLRSIPOCOUNT", PPI_D0S_CLRSIPOCOUNT, A_RW },
	{ "PPI_D1S_CLRSIPOCOUNT", PPI_D1S_CLRSIPOCOUNT, A_RW },
	{ "PPI_D2S_CLRSIPOCOUNT", PPI_D2S_CLRSIPOCOUNT, A_RW },
	{ "PPI_D3S_CLRSIPOCOUNT", PPI_D3S_CLRSIPOCOUNT, A_RW },
	{ "CLS_PRE", CLS_PRE, A_RW },
	{ "D0S_PRE", D0S_PRE, A_RW },
	{ "D1S_PRE", D1S_PRE, A_RW },
	{ "D2S_PRE", D2S_PRE, A_RW },
	{ "D3S_PRE", D3S_PRE, A_RW },
	{ "CLS_PREP", CLS_PREP, A_RW },
	{ "D0S_PREP", D0S_PREP, A_RW },
	{ "D1S_PREP", D1S_PREP, A_RW },
	{ "D2S_PREP", D2S_PREP, A_RW },
	{ "D3S_PREP", D3S_PREP, A_RW },
	{ "CLS_ZERO", CLS_ZERO, A_RW },
	{ "D0S_ZERO", D0S_ZERO, A_RW },
	{ "D1S_ZERO", D1S_ZERO, A_RW },
	{ "D2S_ZERO", D2S_ZERO, A_RW  },
	{ "D3S_ZERO", D3S_ZERO, A_RW },
	{ "PPI_CLRFLG", PPI_CLRFLG, A_RW },
	{ "PPI_CLRSIPO", PPI_CLRSIPO, A_RW },
	{ "PPI_HSTimeout", PPI_HSTimeout, A_RW },
	{ "PPI_HSTimeoutEnable", PPI_HSTimeoutEnable, A_RW },
	/* DSI Protocol Layer Registers */
	{ "DSI_STARTDSI", DSI_STARTDSI, A_WO },
	{ "DSI_BUSYDSI", DSI_BUSYDSI, A_RO },
	{ "DSI_LANEENABLE", DSI_LANEENABLE, A_RW },
	{ "DSI_LANESTATUS0", DSI_LANESTATUS0, A_RO },
	{ "DSI_LANESTATUS1", DSI_LANESTATUS1, A_RO },
	{ "DSI_INTSTATUS", DSI_INTSTATUS, A_RO },
	{ "DSI_INTMASK", DSI_INTMASK, A_RW },
	{ "DSI_INTCLR", DSI_INTCLR, A_WO },
	{ "DSI_LPTXTO", DSI_LPTXTO, A_RW },
	/* DSI General Registers */
	{ "DSIERRCNT", DSIERRCNT, A_RW },
	/* DSI Application Layer Registers */
	{ "APLCTRL", APLCTRL, A_RW },
	{ "RDPKTLN", RDPKTLN, A_RW },
	/* DPI */
	{ "DPIPXLFMT", DPIPXLFMT, A_RW },
	/* Parallel Output */
	{ "POCTRL", POCTRL, A_RW },
	/* Video Path0 Registers */
	{ "VPCTRL0",  VPCTRL0, A_RW },
	{ "HTIM01", HTIM01, A_RW },
	{ "HTIM02", HTIM02, A_RW },
	{ "VTIM01", VTIM01, A_RW },
	{ "VTIM02", VTIM02, A_RW },
	{ "VFUEN0", VFUEN0, A_RW },
	/* Video Path1 Registers */
	{ "VPCTRL1",  VPCTRL1, A_RW },
	{ "HTIM11", HTIM11, A_RW },
	{ "HTIM12", HTIM12, A_RW },
	{ "VTIM11", VTIM11, A_RW },
	{ "VTIM12", VTIM12, A_RW },
	{ "VFUEN1", VFUEN1, A_RW },
	/* System Registers */
	{ "IDREG", IDREG, A_RO },
	{ "SYSBOOT", SYSBOOT, A_RW },
	{ "SYSSTAT", SYSSTAT, A_RO },
	{ "SYSRSTENB", SYSRSTENB, A_RW },
	{ "SYSCTRL", SYSCTRL, A_RO },
	/* I2C Registers */
	{ "I2TIMCTTTRL", I2TIMCTTTRL, A_WO },
	/* GPIO Registers */
	{ "GPIOM", GPIOM, A_RW },
	{ "GPIOC", GPIOC, A_RW },
	{ "GPIOO", GPIOO, A_RW },
	{ "GPIOI", GPIOI, A_RO },
	/*  Interrupt registers */
	{ "INTCTL_G", INTCTL_G, A_RW },
	{ "INTSTS_G", INTSTS_G, A_RW },
	{ "INTCTL_S", INTCTL_S, A_WO },
	{ "INTSTS_S", INTSTS_S, A_WO },
	{ "INT_GP0_LNCT", INT_GP0_LNCT, A_RO },
	{ "INT_GP1_LNCT", INT_GP1_LNCT, A_RO },
	/* Display port control */
	{ "DP0Ctl", DP0Ctl, A_RW },
	{ "DP1Ctl", DP1Ctl, A_RW },
	
	/* PLL */
	{ "DP0_PLLCTRL", DP0_PLLCTRL, A_RW },
	{ "DP1_PLLCTRL", DP1_PLLCTRL, A_RW },
	{ "PXL_PLLCTRL", PXL_PLLCTRL, A_RW },
	{ "PXL_PLLPARAM", PXL_PLLPARAM, A_RW },
	{ "SYS_PLLPARAM", SYS_PLLPARAM, A_RW },
	{ "DP_PHY_CTRL", DP_PHY_CTRL, A_RW },
	/* Debug Registers */
	{ "D2DPTSTCTL", D2DPTSTCTL, A_RW },
	{ "PLL_DBG", PLL_DBG, A_RW },
	/* Aux Registers*/
	{ "DP0_AuxCfg0", DP0_AuxCfg0, A_RW },
	{ "DP0_AuxCfg1", DP0_AuxCfg1, A_RW },
	{ "DP0_AuxAddr", DP0_AuxAddr, A_RW },
	{ "DP0_AuxWdata0", DP0_AuxWdata0, A_RW },
	{ "DP0_AuxWdata1", DP0_AuxWdata1, A_RW },
	{ "DP0_AuxWdata2", DP0_AuxWdata2, A_RW },
	{ "DP0_AuxWdata3", DP0_AuxWdata3, A_RW },
	{ "DP0_AuxRdata0", DP0_AuxRdata0, A_RO },
	{ "DP0_AuxRdata1", DP0_AuxRdata1, A_RO },
	{ "DP0_AuxRdata2", DP0_AuxRdata2, A_RO },
	{ "DP0_AuxRdata3", DP0_AuxRdata3, A_RO },
	{ "DP0_AuxStatus", DP0_AuxStatus, A_RO },
	{ "DP0_AuxI2CAdr", DP0_AuxI2CAdr, A_RW },
	{ "DP0_SrcCtrl", DP0_SrcCtrl, A_RW },
	{ "DP0_LTStat", DP0_LTStat, A_RO },
	{ "DP0_SnkLTChReq", DP0_SnkLTChReq, A_RO },
	{ "DP0_LTLoopCtrl", DP0_LTLoopCtrl, A_RW },
	{ "DP0_SnkLTCtrl", DP0_SnkLTCtrl, A_RW },
	{ "DP0_TPatDat0", DP0_TPatDat0, A_RW },
	{ "DP0_TPatDat1", DP0_TPatDat1, A_RW },
	{ "DP0_TPatDat2", DP0_TPatDat2, A_RW },
	{ "DP0_TPatDat3", DP0_TPatDat3, A_RW },
};

#endif

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

static int tc358766_spi_read(u16 reg, u32 *val)
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

static int tc358766_spi_write(u16 reg, u32 val)
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

static int tc358766_dsi_read(struct omap_dss_device *dssdev, u16 reg, u32 *val)
{
	struct tc358766_data *d2d = dev_get_drvdata(&dssdev->dev);
	u8 buf[4];
	int r;

	r = dsi_vc_gen_read_2(dssdev, d2d->channel1, reg, buf, 4);
	if (r < 0) {
		dev_err(&dssdev->dev, "0x%x read failed with status %d\n",
								reg, r);
		return r;
	}

	*val = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
	return 0;
}

static int tc358766_read_register(struct omap_dss_device *dssdev,
					u16 reg, u32 *val)
{
	int ret = 0;
	pm_runtime_get_sync(&dssdev->dev);
	/* I2C is preferred way of reading, but fall back to DSI
	 * if I2C didn't got initialized
	*/
	if (tc_spi)
		ret = tc358766_spi_read(reg, val);
	else
		ret = tc358766_dsi_read(dssdev, reg, val);
	pm_runtime_put_sync(&dssdev->dev);
	return ret;
}

static int tc358766_write_register(struct omap_dss_device *dssdev, u16 reg,
		u32 value)
{
	struct tc358766_data *d2d = dev_get_drvdata(&dssdev->dev);
	u8 buf[6];
	int r;

	buf[0] = (reg >> 0) & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = (value >> 0) & 0xff;
	buf[3] = (value >> 8) & 0xff;
	buf[4] = (value >> 16) & 0xff;
	buf[5] = (value >> 24) & 0xff;

	r = dsi_vc_gen_write_nosync(dssdev, d2d->channel1, buf, 6);
	if (r)
		dev_err(&dssdev->dev, "reg write reg(%x) val(%x) failed: %d\n",
			       reg, value, r);
	return r;
}

/****************************
********* DEBUG *************
****************************/
#ifdef CONFIG_TC358766_DEBUG

static int tc358766_registers_show(struct seq_file *seq, void *pos)
{
	struct device *dev = tc358766_debug.dev;
	unsigned i, reg_count;
	uint value = 0;

	if (!tc_spi) {
		dev_warn(dev,
			"failed to read register: SPI not initialized\n");
		return -ENODEV;
	}

	reg_count = sizeof(tc358766_regs) / sizeof(tc358766_regs[0]);
	pm_runtime_get_sync(dev);
	for (i = 0; i < reg_count; i++) {
		if (tc358766_regs[i].perm & A_RO) {
			tc358766_spi_read(tc358766_regs[i].reg, &value);
			seq_printf(seq, "%-20s = 0x%02X\n",
					tc358766_regs[i].name, value);
		}
	}
	pm_runtime_put_sync(dev);
	return 0;
}

static ssize_t tc358766_seq_write(struct file *filp, const char __user *ubuf,
						size_t size, loff_t *ppos)
{
	struct device *dev = tc358766_debug.dev;
	unsigned i, reg_count;
	u32 value = 0;
	int error = 0;
	char name[30];
	char buf[50];

	if (size >= sizeof(buf))
		size = sizeof(buf);

	if (copy_from_user(&buf, ubuf, size))
		return -EFAULT;

	buf[size-1] = '\0';
	if (sscanf(buf, "%s %x", name, &value) != 2) {
		dev_err(dev, "%s: unable to parse input\n", __func__);
		return -1;
	}

	if (!tc_spi) {
		dev_warn(dev,
			"failed to write register: SPI not initialized\n");
		return -ENODEV;
	}

	reg_count = sizeof(tc358766_regs) / sizeof(tc358766_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, tc358766_regs[i].name)) {
			if (!(tc358766_regs[i].perm & A_WO)) {
				dev_err(dev, "%s is write-protected\n", name);
				return -EACCES;
			}

			error = tc358766_spi_write(tc358766_regs[i].reg, value);

			if (error) {
				dev_err(dev, "%s: failed to write %s\n",
					__func__, name);
				return -1;
			}

			return size;
		}
	}

	dev_err(dev, "%s: no such register %s\n", __func__, name);

	return size;
}

static ssize_t tc358766_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, tc358766_registers_show, inode->i_private);
}

static const struct file_operations tc358766_debug_fops = {
	.open		= tc358766_seq_open,
	.read		= seq_read,
	.write		= tc358766_seq_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int tc358766_initialize_debugfs(struct omap_dss_device *dssdev)
{
	tc358766_debug.dir = debugfs_create_dir("tc358766", NULL);
	if (IS_ERR(tc358766_debug.dir)) {
		int ret = PTR_ERR(tc358766_debug.dir);
		tc358766_debug.dir = NULL;
		return ret;
	}

	tc358766_debug.dev = &dssdev->dev;
	debugfs_create_file("registers", S_IRWXU, tc358766_debug.dir,
			dssdev, &tc358766_debug_fops);
	return 0;
}

static void tc358766_uninitialize_debugfs(void)
{
	if (tc358766_debug.dir)
		debugfs_remove_recursive(tc358766_debug.dir);
	tc358766_debug.dir = NULL;
	tc358766_debug.dev = NULL;
}

#else
static int tc358766_initialize_debugfs(struct omap_dss_device *dssdev)
{
	return 0;
}

static void tc358766_uninitialize_debugfs(void)
{
}
#endif

static struct tc358766_board_data *get_board_data(struct omap_dss_device
								*dssdev)
{
	return (struct tc358766_board_data *)dssdev->data;
}

static void tc358766_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void tc358766_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dev_info(&dssdev->dev, "set_timings() not implemented\n");
}

static int tc358766_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	if (tc358766_timings.x_res != timings->x_res ||
			tc358766_timings.y_res != timings->y_res ||
			tc358766_timings.pixel_clock != timings->pixel_clock ||
			tc358766_timings.hsw != timings->hsw ||
			tc358766_timings.hfp != timings->hfp ||
			tc358766_timings.hbp != timings->hbp ||
			tc358766_timings.vsw != timings->vsw ||
			tc358766_timings.vfp != timings->vfp ||
			tc358766_timings.vbp != timings->vbp)
		return -EINVAL;

	return 0;
}

static void tc358766_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = tc358766_timings.x_res;
	*yres = tc358766_timings.y_res;
}

static int tc358766_hw_reset(struct omap_dss_device *dssdev)
{
	if (dssdev == NULL || dssdev->reset_gpio == -1)
		return 0;
// FIXME timings
	gpio_set_value(dssdev->reset_gpio, 1);
	udelay(100);
	/* reset the panel */
	gpio_set_value(dssdev->reset_gpio, 0);
	/* assert reset */
	udelay(100);
	gpio_set_value(dssdev->reset_gpio, 1);

	/* wait after releasing reset */
	msleep(100);

	return 0;
}

static int tc358766_probe(struct omap_dss_device *dssdev)
{
	struct tc358766_data *d2d;
	int r = 0;

	dev_dbg(&dssdev->dev, "tc358766_probe\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	tc358766_timings = dssdev->panel.timings;
	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;

	d2d = kzalloc(sizeof(*d2d), GFP_KERNEL);
	if (!d2d) {
		r = -ENOMEM;
		goto err;
	}
	
	d2d->dssdev = dssdev;

	mutex_init(&d2d->lock);

	dev_set_drvdata(&dssdev->dev, d2d);

	r = omap_dsi_request_vc(dssdev, &d2d->channel0);
	if (r) {
		dev_err(&dssdev->dev, "failed to get virtual channel0\n");
		goto err;
	}

	r = omap_dsi_set_vc_id(dssdev, d2d->channel0, 0);
	if (r) {
		dev_err(&dssdev->dev, "failed to set VC_ID0\n");
		goto err;
	}

	r = omap_dsi_request_vc(dssdev, &d2d->channel1);
	if (r) {
		dev_err(&dssdev->dev, "failed to get virtual channel1\n");
		goto err;
	}

	r = omap_dsi_set_vc_id(dssdev, d2d->channel1, 0);
	if (r) {
		dev_err(&dssdev->dev, "failed to set VC_ID1\n");
		goto err;
	}

	r = tc358766_initialize_debugfs(dssdev);
	if (r)
		dev_warn(&dssdev->dev, "failed to create sysfs files\n");

	dev_dbg(&dssdev->dev, "tc358766_probe done\n");
	return 0;
err:
	kfree(d2d);
	return r;
}

static void tc358766_remove(struct omap_dss_device *dssdev)
{
	struct tc358766_data *d2d = dev_get_drvdata(&dssdev->dev);

	tc358766_uninitialize_debugfs();

	omap_dsi_release_vc(dssdev, d2d->channel0);
	omap_dsi_release_vc(dssdev, d2d->channel1);

	kfree(d2d);
}

static int tc358766_init_ppi(struct omap_dss_device *dssdev)
{
	u32 go_cnt, sure_cnt, val = 0;
	u8 lanes = 0;
	int ret = 0;
	struct tc358766_board_data *board_data = get_board_data(dssdev);

	/* this register setting is required only if host wishes to
	 * perform DSI read transactions
	 */
	go_cnt = (board_data->lp_time * 5 - 3) / 4;
	sure_cnt = DIV_ROUND_UP(board_data->lp_time * 3, 2);
	val = FLD_MOD(val, go_cnt, 26, 16);
	val = FLD_MOD(val, sure_cnt, 10, 0);
	ret |= tc358766_spi_write(PPI_TX_RX_TA, val);

	/* SYSLPTX Timing Generation Counter */
	ret |= tc358766_spi_write(PPI_LPTXTIMECNT,
					board_data->lp_time);

	/* D*S_CLRSIPOCOUNT = [(THS-SETTLE + THS-ZERO) /
					HS_byte_clock_period ] */

	if (dssdev->phy.dsi.clk_lane)
		lanes |= (1 << 0);

	if (dssdev->phy.dsi.data1_lane) {
		lanes |= (1 << 1);
		ret |= tc358766_spi_write(PPI_D0S_CLRSIPOCOUNT,
							board_data->clrsipo);
	}
	if (dssdev->phy.dsi.data2_lane) {
		lanes |= (1 << 2);
		ret |= tc358766_spi_write(PPI_D1S_CLRSIPOCOUNT,
							board_data->clrsipo);
	}
	if (dssdev->phy.dsi.data3_lane) {
		lanes |= (1 << 3);
		ret |= tc358766_spi_write(PPI_D2S_CLRSIPOCOUNT,
							board_data->clrsipo);
	}
	if (dssdev->phy.dsi.data4_lane) {
		lanes |= (1 << 4);
		ret |= tc358766_spi_write(PPI_D3S_CLRSIPOCOUNT,
							board_data->clrsipo);
	}

	ret |= tc358766_spi_write(PPI_LANEENABLE, lanes);
	ret |= tc358766_spi_write(DSI_LANEENABLE, lanes);

	return ret;
}

static uint32_t
pack_aux(uint8_t *src, int src_bytes)
{
	int	i;
	uint32_t v = 0;

	if (src_bytes > 4)
		src_bytes = 4;
	for (i = 0; i < src_bytes; i++)
		v |= ((uint32_t) src[i]) << ((3-i) * 8);
	return v;
}

static void
unpack_aux(uint32_t src, uint8_t *dst, int dst_bytes)
{
	int i;
	if (dst_bytes > 4)
		dst_bytes = 4;
	for (i = 0; i < dst_bytes; i++)
		dst[i] = src >> ((3-i) * 8);
}

static int
tc358766_aux_ch(struct omap_dss_device *dssdev,
		uint8_t *send, int send_bytes,
		uint8_t *recv, int recv_size)
{
	int i = 0;
	int try = 0;
	int ret = 0;
	u32 val = 0;

	if ((send_bytes < 3) || (send_bytes > 19))return -EINVAL;
	/* Must try at least 3 times according to DP spec */
	for (try = 0; try < 5; try++) {
		/*Setup the aDdress*/
		u32 config =  send[0];
		ret = tc358766_spi_write(DP0_AuxAddr, ((send[1] << 8) | send[2]));
	
		if (send_bytes == 3) {
			//Read Command
			config |= (recv_size << BSIZE_SHIFT);
		} else {
			//Write command
			config |= ((send_bytes - 3) << BSIZE_SHIFT);
			for (i = 3; i < send_bytes; i += 4)
				ret |= tc358766_spi_write(DP0_AuxWdata0 + i, pack_aux(send + i, send_bytes - i));
		}
		ret |= tc358766_spi_write(DP0_AuxCfg0, config); //Trigger the Aux command
		for (;;) {
		  	udelay(100);
			ret |= tc358766_spi_read(DP0_AuxStatus, &val); //Trigger the Aux command
			if (ret || (val & AUX_BUSY_FLAG) == 0) {
				if (!(val & AUX_TIMEOUT_FLAG)) {
					printk("passed 0x%x\n", val);
					goto passed;
				}
				else {
					printk("fail 0x%x\n", val);
					ret = -EIO;
					goto end_return;
				}
			}
			printk("delay\n");
		}
		if (try == 4) {
		  ret = -EIO;
		  goto end_return;
		}
	}
passed:
	if (val && AUX_NATIVE_REPLY_MASK != AUX_NATIVE_REPLY_ACK)
		return -EIO;
	for (i=0; i < recv_size; i+=4) {
		ret |= tc358766_spi_read(DP0_AuxRdata0 + i, &val); //Trigger the Aux command
		unpack_aux(val, &recv[i], recv_size - i);
	}
end_return:
	return ret;
}
/* read bytes from a native aux channel */
static int
tc358766_aux_native_read(struct omap_dss_device *dssdev, uint16_t address, uint8_t *recv, int recv_bytes)
{
	uint8_t msg[3];
	int msg_bytes;
	int ret;

	msg[0] = AUX_NATIVE_READ;
	msg[1] = address >> 8;
	msg[2] = address & 0xff;

	msg_bytes = 3;

	ret = tc358766_aux_ch(dssdev, msg, msg_bytes,recv, recv_bytes);
	return ret;
}

static int tc358766_init_training_start(struct omap_dss_device *dssdev)
{
	int ret = 0;
	uint8_t dpcd[4];

	ret = tc358766_aux_native_read(dssdev, 0x000, dpcd, sizeof (dpcd));
	if (!ret) {
		printk("DPCD read : 0x%x 0x%x 0x%x 0x%x\n", dpcd[0],dpcd[1],dpcd[2],dpcd[3]);
	} else {
		printk("DPCD read failed\n");
	}
	return ret;
}

static int tc358766_init_dpout(struct omap_dss_device *dssdev)
{
	int ret = 0;
	u32 val = 0;

	//Aux should be low swing to be compliant with the panel
	ret = tc358766_spi_write(DP0_AUX_PHY_CTRL, 0x0);

	//We can enable the DP PHY
	ret |= tc358766_spi_read(DP_PHY_CTRL, &val);
	val = FLD_MOD(val, 0x1, 2, 2); //PHY 0 is in dual mode (2*2.7Gbps)
	val = FLD_MOD(val, 0x1, 1, 1); //Enable PHY0 Aux 
	val = FLD_MOD(val, 0x1, 0, 0); //Enable PHY0 Main
	ret |= tc358766_spi_write(DP_PHY_CTRL, val);
	//Wait for PHY ready
	val = 0;
	while (!((val >> 16) & 0x1)) {
		ret = tc358766_spi_read(DP_PHY_CTRL, &val);
		if (ret != 0) return ret;
	}

	tc358766_init_training_start(dssdev);
	// Set DP0 and DP1 source as DSI RX and no audio
	ret |= tc358766_spi_read(SYSCTRL, &val);
	val = FLD_MOD(val, 0x1, 6, 4);
	val = FLD_MOD(val, 0x1, 2, 0);
	ret |= tc358766_spi_write(SYSCTRL, val);

	ret |= tc358766_spi_read(DP0Ctl, &val);
	val = FLD_MOD(val, 0x1, 1, 1); //Enable DP video Tx
	val = FLD_MOD(val, 0x1, 0, 0); //Enable Main DP TX
	ret |= tc358766_spi_write(DP0Ctl, val);

	return ret;
}

static int tc358766_init_clock(struct omap_dss_device *dssdev)
{
	u32 val = 0;
	int ret = 0;

	//Be sure no video/audio is sent
	ret = tc358766_spi_read(SYSRSTENB, &val);
	val = FLD_MOD(val, 0x0, 2,2);
	val = FLD_MOD(val, 0x0, 6,6);
	val = FLD_MOD(val, 0x0, 7,7);
	ret |= tc358766_spi_write(SYSRSTENB, val);
	// b) Program updated PLL parameters in appropriate D2DP registers.
	ret |= tc358766_spi_read(SYS_PLLPARAM, &val);
	val = FLD_MOD(val, 0x0, 0, 0); //LSCLK_DIV = 0
	val = FLD_MOD(val, 0x0, 4, 4); //Use LSCLK as source
	val = FLD_MOD(val, 0x1, 9, 8); //REFCLK is 19.2MHz
	ret |= tc358766_spi_write(SYS_PLLPARAM, val);

	ret |= tc358766_spi_read(PXL_PLLPARAM, &val);
	val = FLD_MOD(val, 0x0, 24, 24); //VCO < 300 MHz
	val = FLD_MOD(val, 1, 22, 20); //Use Pre Divider 1
	val = FLD_MOD(val, 1, 18, 16); //No post Divider
	val = FLD_MOD(val, 0x0, 15, 14); //Input is RefClk
	val = FLD_MOD(val, 9, 11, 8); //PLL Divider = 9
	val = FLD_MOD(val, 65, 6, 0); //PLL Multiplier = 65
	//Result is 38.4/2 * (65/9) = 138.667 Mhz
	ret |= tc358766_spi_write(PXL_PLLPARAM, val);

	//Setup the DP0_PLL0
	ret |= tc358766_spi_read(DP0_PLLCTRL, &val);

        //c) Disable appropriate PLL by writing „0‟ to the “PLLEn” bit in the appropriate PLLCtl register.
	val &= ~ENABLE_PLL;
	val |= BYPASS_PLL;
	ret |= tc358766_spi_write(DP0_PLLCTRL, val);
	//d) Copy PLL parameters from PLL shadow registers in D2DP register space to current PLL
	//   parameters register in PLL control block using PLL_REF by writing to the “Pllupdate” bit in
	//   the PLLCtl register.
	ret |= tc358766_spi_write(DP0_PLLCTRL, val | PLL_UPDATE);

	//e) Enable appropriate PLL by writing „1‟ to the “PLLEn” bit in the appropriate PLLCtl register.
	val |= ENABLE_PLL;
	ret |= tc358766_spi_write(DP0_PLLCTRL, val);


        //Setup the PXL_PLL
	ret |= tc358766_spi_read(PXL_PLLCTRL, &val);
	
	val &= ~ENABLE_PLL;
	val |= BYPASS_PLL;
	ret |= tc358766_spi_write(PXL_PLLCTRL, val);
	//d) Copy PLL parameters from PLL shadow registers in D2DP register space to current PLL
	//   parameters register in PLL control block using PLL_REF by writing to the “Pllupdate” bit in
	//   the PLLCtl register.
	ret |= tc358766_spi_write(PXL_PLLCTRL, val | PLL_UPDATE);

	//e) Enable appropriate PLL by writing „1‟ to the “PLLEn” bit in the appropriate PLLCtl register.
	val |= ENABLE_PLL;
	ret |= tc358766_spi_write(PXL_PLLCTRL, val);

	// wait pll lock, check where????
	//f) All clocks from CG are enabled after LOCKDET (frequency lock detect true signal from PLL) is asserted.
	msleep(200); //Wait a bit for the PLL to lock 

	val &= ~BYPASS_PLL;
	ret |= tc358766_spi_write(DP0_PLLCTRL, val);
	val &= ~BYPASS_PLL;
	ret |= tc358766_spi_write(PXL_PLLCTRL, val);

	ret |= tc358766_spi_read(SYSRSTENB, &val);
	val = FLD_MOD(val, 0x1, 2,2); //Enable the LCD0 interface
	ret |= tc358766_spi_write(SYSRSTENB, val);
	
	return ret;
}

static int tc358766_init_video_timings(struct omap_dss_device *dssdev)
{
	u32 val;
	struct tc358766_board_data *board_data = get_board_data(dssdev);
	int ret;
	ret = tc358766_spi_read(VPCTRL0, &val);
	if (ret < 0) {
		dev_warn(&dssdev->dev,
			"couldn't access VPCTRL0, going on with reset value\n");
		val = 0;
	}

	if (dssdev->ctrl.pixel_size == 18) {
		/* Magic Square FRC available for RGB666 only */
		val = FLD_MOD(val, board_data->msf, 0, 0);
		val = FLD_MOD(val, 0, 8, 8);
	} else {
		val = FLD_MOD(val, 1, 8, 8);
	}

	val = FLD_MOD(val, board_data->vtgen, 4, 4);
	val = FLD_MOD(val, board_data->evtmode, 5, 5);
	val = FLD_MOD(val, board_data->vsdelay, 31, 20);

	ret = tc358766_spi_write(VPCTRL0, val);

	ret |= tc358766_spi_write(HTIM01,
		(tc358766_timings.hbp << 16) | tc358766_timings.hsw);
	ret |= tc358766_spi_write(HTIM02,
		((tc358766_timings.hfp << 16) | tc358766_timings.x_res));
	ret |= tc358766_spi_write(VTIM01,
		((tc358766_timings.vbp << 16) |	tc358766_timings.vsw));
	ret |= tc358766_spi_write(VTIM02,
		((tc358766_timings.vfp << 16) |	tc358766_timings.y_res));
	if (ret) return ret;
		/* commit video configuration */
	ret = tc358766_spi_write(VFUEN0, 0x1);
	if (ret)
		dev_err(&dssdev->dev, "failed to latch video timings\n");
	return ret;
}

static int tc358766_write_init_config(struct omap_dss_device *dssdev)
{
//	struct tc358766_board_data *board_data = get_board_data(dssdev);

	int r;
// FIXME
	/* HACK: dummy read: if we read via DSI, first reads always fail */
//	tc358765_read_register(dssdev, DSI_INTSTATUS, &val);

	dev_dbg(&dssdev->dev,"tc358766_write_init_config\n");

	r = tc358766_init_ppi(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to initialize PPI layer\n");
		return r;
	}

	r = tc358766_spi_write(PPI_STARTPPI, 0x1);
	if (r) {
		dev_err(&dssdev->dev, "failed to start PPI-TX\n");
		return r;
	}

	r = tc358766_spi_write(DSI_STARTDSI, 0x1);
	if (r) {
		dev_err(&dssdev->dev, "failed to start DSI-RX\n");
		return r;
	}

	r = tc358766_init_clock(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to initialize clocks\n");
		return r;
	}

	// Add here dp out config
	r = tc358766_init_dpout(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to initialize DPout\n");
		return r;
	}

	r = tc358766_init_video_timings(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to initialize video path layer\n");
		return r;
	}

	return r;
}

static int tc358766_power_on(struct omap_dss_device *dssdev)
{
	struct tc358766_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;

	/* At power on the first vsync has not been received yet */
	dssdev->first_vsync = false;

	dev_dbg(&dssdev->dev, "power_on\n");

	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err_disp_enable;
	}

	omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, true);

	/* reset tc358766 bridge */
	tc358766_hw_reset(dssdev);

	/* do extra job to match kozio registers (???) */
	dsi_videomode_panel_preinit(dssdev);

	/* Need to wait a certain time - Toshiba Bridge Constraint */
	/* msleep(400); */

	/* configure D2L chip DSI-RX configuration registers */
	r = tc358766_write_init_config(dssdev);
	if (r)
		goto err_write_init;

	omapdss_dsi_vc_enable_hs(dssdev, d2d->channel1, true);

	/* 0x0e - 16bit
	 * 0x1e - packed 18bit
	 * 0x2e - unpacked 18bit
	 * 0x3e - 24bit
	 */
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

	dev_dbg(&dssdev->dev, "power_on done\n");

	return r;

err_write_init:
	omapdss_dsi_display_disable(dssdev, false, false);
err_disp_enable:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	return r;
}

static void tc358766_power_off(struct omap_dss_device *dssdev)
{
	dsi_video_mode_disable(dssdev);

	omapdss_dsi_display_disable(dssdev, false, false);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static void tc358766_disable(struct omap_dss_device *dssdev)
{
// FIXME
	struct tc358766_data *d2d = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		mutex_lock(&d2d->lock);
		dsi_bus_lock(dssdev);

		tc358766_power_off(dssdev);

		dsi_bus_unlock(dssdev);
		mutex_unlock(&d2d->lock);
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int tc358766_enable(struct omap_dss_device *dssdev)
{
// FIXME
	struct tc358766_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	mutex_lock(&d2d->lock);
	dsi_bus_lock(dssdev);

	r = tc358766_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	}

	mutex_unlock(&d2d->lock);

	return r;
}

static int tc358766_suspend(struct omap_dss_device *dssdev)
{
	/* Disable the panel and return 0;*/
	tc358766_disable(dssdev);
	return 0;
}

static struct omap_dss_driver tc358766_driver = {
	.probe		= tc358766_probe,
	.remove		= tc358766_remove,

	.enable		= tc358766_enable,
	.disable	= tc358766_disable,
	.suspend	= tc358766_suspend,
	.resume		= NULL,

	.get_resolution	= tc358766_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= tc358766_get_timings,
	.set_timings	= tc358766_set_timings,
	.check_timings	= tc358766_check_timings,

	.driver         = {
		.name   = "tc358766",
		.owner  = THIS_MODULE,
	},
};

/*--------------------SPI probe-------------------------*/

static int tc358766_spi_probe(struct spi_device *spi)
{

	dev_dbg(&spi->dev, "%s\n", __func__);

	tc_spi = kzalloc(sizeof(struct tc358766_spi), GFP_KERNEL);
	if (tc_spi == NULL)
		return -ENOMEM;
	tc_spi->spi = spi;
	mutex_init(&tc_spi->mutex);
	dev_set_drvdata(&spi->dev, tc_spi);

	return 0;
}

static int tc358766_spi_remove(struct spi_device *spi)
{

	dev_dbg(&spi->dev, "%s\n", __func__);

	return 0;
}

static struct spi_driver tc358766_spi_driver = {
	.driver = {
		.name	= "tc358766_spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= tc358766_spi_probe,
	.remove	= __devexit_p(tc358766_spi_remove),
};


static int __init tc358766_init(void)
{
	int r;
	tc_spi = NULL;

	r = spi_register_driver(&tc358766_spi_driver);
	if (r < 0) {
		printk(KERN_WARNING "DSI to DP spi driver registration failed\n");
		return r;
	}
	omap_dss_register_driver(&tc358766_driver);
	return 0;
}

static void __exit tc358766_exit(void)
{
	omap_dss_unregister_driver(&tc358766_driver);

	spi_unregister_driver(&tc358766_spi_driver);
}

module_init(tc358766_init);
module_exit(tc358766_exit);

MODULE_AUTHOR("@archos.com>");
MODULE_DESCRIPTION("TC358766 DSI-2-DP Driver");
MODULE_LICENSE("GPL");
