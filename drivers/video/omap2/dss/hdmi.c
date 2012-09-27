/*
 * hdmi.c
 *
 * HDMI interface DSS driver setting for TI's OMAP4 family of processor.
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Authors: Yong Zhi
 *	Mythri pk <mythripk@ti.com>
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

#define DSS_SUBSYS_NAME "HDMI"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <video/omapdss.h>
#include <video/hdmi_ti_4xxx_ip.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/omapfb.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#include <asm/mach-types.h>
//#define USE_EARLYSUSPEND
#endif

#include "dss.h"
#include "dss_features.h"

#ifdef CONFIG_OMAP_PM
#include <linux/pm_qos_params.h>
static struct pm_qos_request_list pm_qos_handle;
#endif

#define HDMI_WP			0x0
#define HDMI_CORE_SYS		0x400
#define HDMI_CORE_AV		0x900
#define HDMI_PLLCTRL		0x200
#define HDMI_PHY		0x300
/* HDMI EDID Length move this */
#define HDMI_EDID_MAX_LENGTH			512
#define EDID_TIMING_DESCRIPTOR_SIZE		0x12
#define EDID_DESCRIPTOR_BLOCK0_ADDRESS		0x36
#define EDID_DESCRIPTOR_BLOCK1_ADDRESS		0x80
#define EDID_SIZE_BLOCK0_TIMING_DESCRIPTOR	4
#define EDID_SIZE_BLOCK1_TIMING_DESCRIPTOR	4

#define OMAP_HDMI_TIMINGS_NB			34

enum power_state {
	HDMI_POWER_OFF = 0,
	HDMI_POWER_MIN,
	HDMI_POWER_FULL,
};

enum connection_state {
	HDMI_DISCONNECT = 1,
	HDMI_CONNECT = 2,
};

static struct hdmi_data {
	struct mutex lock;
	struct omap_display_platform_data *pdata;
	struct platform_device *pdev;
	struct omap_dss_device *dssdev;
	struct hdmi_ip_data hdmi_data;
	int code;
	int mode;
	u8 edid[HDMI_EDID_MAX_LENGTH];
	u8 edid_set;
	bool can_do_hdmi;

	bool custom_set;
	enum hdmi_deep_color_mode deep_color;
	struct hdmi_config cfg;
	struct regulator *hdmi_reg, *hdmi_5v;

	int hdmi_irq;
	struct clk *sys_clk;
	struct clk *hdmi_clk;

	int runtime_count;
	int enabled;
	bool set_mode;
	bool wp_reset_done;

	u8 s3d_mode;
	bool s3d_enable;

	int main_runtime_count;
	enum power_state power_state;
	struct work_struct work;
	spinlock_t work_lock;
	enum connection_state connection_state;
	enum connection_state connection_trans;

        int source_physical_address;
	void (*hdmi_start_frame_cb)(void);
	void (*hdmi_irq_cb)(int);
	bool (*hdmi_power_on_cb)(void);
	void (*hdmi_cec_power_on_cb)(int phy_addr, int state);
	void (*hdmi_cec_irq_cb)(int);
} hdmi;

static struct hdmi_5v_work_data {
	struct delayed_work dwork;
	atomic_t current_state;
	atomic_t req_state;
} hdmi_5v_work;
static struct workqueue_struct *hdmi_5v_workq;

void hdmi_panel_phy_connected(void);
void hdmi_panel_phy_disconnected(void);

static const u8 edid_header[8] = {0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0};

static int hdmi_main_runtime_get(void)
{
	int r;
	if (hdmi.main_runtime_count++ == 0) {
		r = dss_runtime_get();
		if (r)
			return r;

		r = pm_runtime_get_sync(&hdmi.pdev->dev);
		WARN_ON(r);
		if (r < 0)
			goto err_runtime_get;
	}

	return 0;
err_runtime_get:
	dss_runtime_put();
	return r;
}

static int hdmi_main_runtime_put(void)
{
	if (--hdmi.main_runtime_count == 0) {
		int r = pm_runtime_put_sync(&hdmi.pdev->dev);
		WARN_ON(r);
		dss_runtime_put();
	}

	return 0;
}

static int hdmi_runtime_get(void)
{
	int r;

	DSSDBG("hdmi_runtime_get\n");

	if (hdmi.runtime_count++ == 0) {
		r = dispc_runtime_get();
		if (r)
			goto err_get_dispc;

		clk_enable(hdmi.sys_clk);
		clk_enable(hdmi.hdmi_clk);
	}

	return 0;

	clk_disable(hdmi.sys_clk);
	clk_disable(hdmi.hdmi_clk);
err_get_dispc:
	return r;
}

static void hdmi_runtime_put(void)
{
	DSSDBG("hdmi_runtime_put\n");

	if (--hdmi.runtime_count == 0) {
		clk_disable(hdmi.sys_clk);
		clk_disable(hdmi.hdmi_clk);

		dispc_runtime_put();
	}
}

int hdmi_init_display(struct omap_dss_device *dssdev)
{
	DSSDBG("init_display\n");

	return 0;
}

static int relaxed_fb_mode_is_equal(const struct fb_videomode *mode1,
				    const struct fb_videomode *mode2)
{
	u32 ratio1 = mode1->flag & (FB_FLAG_RATIO_4_3 | FB_FLAG_RATIO_16_9);
	u32 ratio2 = mode2->flag & (FB_FLAG_RATIO_4_3 | FB_FLAG_RATIO_16_9);
	u32 vtotal1 = mode1->vsync_len + mode1->upper_margin + mode1->lower_margin;
	u32 vtotal2 = mode2->vsync_len + mode2->upper_margin + mode2->lower_margin;

	if (mode2->vmode & FB_VMODE_INTERLACED) vtotal2 /= 2;
	return (mode1->xres         == mode2->xres &&
		mode1->yres         == mode2->yres &&
		mode1->pixclock     <= mode2->pixclock * 201 / 200 &&
		mode1->pixclock     >= mode2->pixclock * 200 / 201 &&
		mode1->hsync_len + mode1->left_margin + mode1->right_margin ==
		mode2->hsync_len + mode2->left_margin + mode2->right_margin &&
		vtotal1 == vtotal2 && 
		(!ratio1 || !ratio2 || ratio1 == ratio2) &&
		(mode1->vmode & FB_VMODE_INTERLACED) ==
		(mode2->vmode & FB_VMODE_INTERLACED));
}

static int hdmi_set_timings(struct fb_videomode *vm, bool check_only)
{
	int i = 0;
	DSSDBG("hdmi_get_code\n");

	if (!vm->xres || !vm->yres || !vm->pixclock)
		goto fail;

	for (i = 0; i < CEA_MODEDB_SIZE; i++) {
		if (relaxed_fb_mode_is_equal(cea_modes + i, vm)) {
			*vm = cea_modes[i];
			if (check_only)
				return 1;
			hdmi.cfg.cm.code = i;
			hdmi.cfg.cm.mode = HDMI_HDMI;
			hdmi.cfg.timings = cea_modes[hdmi.cfg.cm.code];
			goto done;
		}
	}

	for (i = 0; i < VESA_MODEDB_SIZE; i++) {
		if (relaxed_fb_mode_is_equal(vesa_modes + i, vm)) {
			*vm = vesa_modes[i];
			if (check_only)
				return 1;
			hdmi.cfg.cm.code = i;
			hdmi.cfg.cm.mode = HDMI_DVI;
			hdmi.cfg.timings = vesa_modes[hdmi.cfg.cm.code];
			goto done;
		}
	}

fail:
	if (check_only)
		return 0;
// 	hdmi.cfg.cm.code = 1;
// 	hdmi.cfg.cm.mode = HDMI_HDMI;
// 	hdmi.cfg.timings = cea_modes[hdmi.cfg.cm.code];

	i = -1;
done:

	DSSDBG("%s-%d\n", hdmi.cfg.cm.mode ? "CEA" : "VESA", hdmi.cfg.cm.code);
	return i >= 0;
}

void hdmi_get_monspecs(struct fb_monspecs *specs)
{
	int i, j;
	char *edid = (char *) hdmi.edid;

	memset(specs, 0x0, sizeof(*specs));
	if (!hdmi.edid_set)
		return;

	fb_edid_to_monspecs(edid, specs);
	if (specs->modedb == NULL)
		return;
	
	for (i = 1; i <= edid[0x7e] && i * 128 < HDMI_EDID_MAX_LENGTH; i++) {
		if (edid[i * 128] == 0x2)
			fb_edid_add_monspecs(edid + i * 128, specs);
	}

#ifdef CONFIG_SUPPORT_PANEL_EXTRA_MODE
	fb_extrapolate_modedb(specs);
#endif

	hdmi.can_do_hdmi = specs->misc & FB_MISC_HDMI;

	/* filter out resolutions we don't support */
	for (i = j = 0; i < specs->modedb_len; i++) {
		u32 max_pclk = hdmi.dssdev->clocks.hdmi.max_pixclk_khz;
		if (!hdmi_set_timings(&specs->modedb[i], true))
			continue;

		if (max_pclk &&
		    max_pclk < PICOS2KHZ(specs->modedb[i].pixclock))
			continue;

		if (specs->modedb[i].flag & FB_FLAG_PIXEL_REPEAT)
			continue;

		specs->modedb[j++] = specs->modedb[i];
	}
	specs->modedb_len = j;
	/*Find out the Source Physical address for the CEC*/
	i = 128;
	while (i < (HDMI_EDID_MAX_LENGTH - 5)) {
		if ((edid[i] == 0x03) && (edid[i+1] == 0x0c) &&
			(edid[i+2] == 0x00)) {
			hdmi.source_physical_address = 0;
			hdmi.source_physical_address = edid[i+3];
			hdmi.source_physical_address =
				(hdmi.source_physical_address << 8) |
				edid[i+4];
			break;
		}
		i++;

	}
}

u8 *hdmi_read_edid(struct omap_video_timings *dp)
{
	int ret = 0, i;

	if (hdmi.edid_set)
		return hdmi.edid;

	memset(hdmi.edid, 0, HDMI_EDID_MAX_LENGTH);

	ret = read_ti_4xxx_edid(&hdmi.hdmi_data, hdmi.edid,
						HDMI_EDID_MAX_LENGTH);

	for (i = 0; i < HDMI_EDID_MAX_LENGTH; i += 16)
		pr_info("edid[%03x] = %02x %02x %02x %02x %02x %02x %02x %02x "
			 "%02x %02x %02x %02x %02x %02x %02x %02x\n", i,
			hdmi.edid[i], hdmi.edid[i + 1], hdmi.edid[i + 2],
			hdmi.edid[i + 3], hdmi.edid[i + 4], hdmi.edid[i + 5],
			hdmi.edid[i + 6], hdmi.edid[i + 7], hdmi.edid[i + 8],
			hdmi.edid[i + 9], hdmi.edid[i + 10], hdmi.edid[i + 11],
			hdmi.edid[i + 12], hdmi.edid[i + 13], hdmi.edid[i + 14],
			hdmi.edid[i + 15]);

	if (ret) {
		DSSWARN("failed to read E-EDID\n");
		return NULL;
	}

	if (memcmp(hdmi.edid, edid_header, sizeof(edid_header)))
		return NULL;

	hdmi.edid_set = true;
	return hdmi.edid;
}

static void hdmi_compute_pll(struct omap_dss_device *dssdev, int phy,
		struct hdmi_pll_info *pi)
{
	unsigned long clkin, refclk;
	u32 mf;

	clkin = clk_get_rate(hdmi.sys_clk) / 10000;
	/*
	 * Input clock is predivided by N + 1
	 * out put of which is reference clk
	 */
	pi->regn = dssdev->clocks.hdmi.regn;
	refclk = clkin / (pi->regn + 1);

	/*
	 * multiplier is pixel_clk/ref_clk
	 * Multiplying by 100 to avoid fractional part removal
	 */
	pi->regm = (phy * 100 / (refclk)) / 100;
	pi->regm2 = dssdev->clocks.hdmi.regm2;

	/*
	 * fractional multiplier is remainder of the difference between
	 * multiplier and actual phy(required pixel clock thus should be
	 * multiplied by 2^18(262144) divided by the reference clock
	 */
	mf = (phy - pi->regm * refclk) * 262144;
	pi->regmf = mf / (refclk);

	/*
	 * Dcofreq should be set to 1 if required pixel clock
	 * is greater than 1000MHz
	 */
	pi->dcofreq = phy > 1000 * 100;
	pi->regsd = ((pi->regm * clkin / 10) / ((pi->regn + 1) * 250) + 5) / 10;

	DSSDBG("M = %d Mf = %d\n", pi->regm, pi->regmf);
	DSSDBG("range = %d sd = %d\n", pi->dcofreq, pi->regsd);
}

static void hdmi_load_hdcp_keys(struct omap_dss_device *dssdev)
{
	int aksv;
	DSSDBG("hdmi_load_hdcp_keys\n");
	/* load the keys and reset the wrapper to populate the AKSV registers*/
	if (hdmi.hdmi_power_on_cb) {
		aksv = hdmi_ti_4xx_check_aksv_data(&hdmi.hdmi_data);
		if ((aksv == HDMI_AKSV_ZERO) &&
		    hdmi.custom_set &&
		    hdmi.hdmi_power_on_cb()) {
			hdmi_ti_4xxx_set_wait_soft_reset(&hdmi.hdmi_data);
			/* HDCP keys are available in the AKSV registers 2ms after
			 * the RESET# rising edge, hence the delay before reading
			 * the registers*/
			mdelay(10);
			aksv = hdmi_ti_4xx_check_aksv_data(&hdmi.hdmi_data);
			hdmi.wp_reset_done = (aksv == HDMI_AKSV_VALID) ?
				true : false;
			DSSINFO("HDMI_WRAPPER RESET DONE\n");
		} else if (aksv == HDMI_AKSV_VALID)
			hdmi.wp_reset_done = true;
		else if (aksv == HDMI_AKSV_ERROR)
			hdmi.wp_reset_done = false;

		if (!hdmi.wp_reset_done)
			DSSERR("*** INVALID AKSV: %d "
				"Do not perform HDCP AUTHENTICATION\n", aksv);
	}

}

/* Set / Release c-state constraints */
static void hdmi_set_l3_cstr(struct omap_dss_device *dssdev, bool enable)
{
#ifdef CONFIG_OMAP_PM
	DSSINFO("%s c-state constraint for HDMI\n\n",
		enable ? "Set" : "Release");

	if (enable)
		pm_qos_add_request(&pm_qos_handle, PM_QOS_CPU_DMA_LATENCY, 100);
	else
		pm_qos_remove_request(&pm_qos_handle);
#else
	DSSINFO("C-STATE Constraints require COMFIG_OMAP_PM to be set\n");
#endif
}

static void hdmi_power_off(struct omap_dss_device *dssdev);
static int hdmi_power_min(struct omap_dss_device *dssdev);

static int hdmi_power_on(struct omap_dss_device *dssdev)
{
	int r;
	struct hdmi_pll_info pll_data;
	struct omap_video_timings *p;
	unsigned long phy;

	if (hdmi.power_state == HDMI_POWER_OFF) {
		r = hdmi_power_min(dssdev);
		if (r)
			return r;
	}

	r = hdmi_runtime_get();
	if (r) {
		if (hdmi.power_state == HDMI_POWER_OFF)
			hdmi_power_off(dssdev);
		return r;
	}

	hdmi_set_l3_cstr(dssdev, true);

	/* Load the HDCP keys if not already loaded*/
	hdmi_load_hdcp_keys(dssdev);

	hdmi_ti_4xxx_wp_video_start(&hdmi.hdmi_data, 0);

	dispc_enable_channel(OMAP_DSS_CHANNEL_DIGIT, dssdev->type, 0);

	p = &dssdev->panel.timings;

	DSSDBG("hdmi_power_on x_res= %d y_res = %d\n",
		dssdev->panel.timings.x_res,
		dssdev->panel.timings.y_res);

	if (!hdmi.custom_set) {
		struct fb_videomode vesa_vga = vesa_modes[4];
		hdmi_set_timings(&vesa_vga, false);
	}

	omapfb_fb2dss_timings(&hdmi.cfg.timings, &dssdev->panel.timings);

	phy = p->pixel_clock;

	switch (hdmi.deep_color) {
	case HDMI_DEEP_COLOR_30BIT:
		phy = (p->pixel_clock * 125) / 100 ;
		hdmi.cfg.deep_color = HDMI_DEEP_COLOR_30BIT;
		break;
	case HDMI_DEEP_COLOR_36BIT:
		if (p->pixel_clock == 148500) {
			printk(KERN_ERR "36 bit deep color not supported");
			goto err;
		}

		phy = (p->pixel_clock * 150) / 100;
		hdmi.cfg.deep_color = HDMI_DEEP_COLOR_36BIT;
		break;
	case HDMI_DEEP_COLOR_24BIT:
	default:
		phy = p->pixel_clock;
		hdmi.cfg.deep_color = HDMI_DEEP_COLOR_24BIT;
		break;
	}

	hdmi_compute_pll(dssdev, phy, &pll_data);

	/* config the PLL and PHY hdmi_set_pll_pwrfirst */
	r = hdmi_ti_4xxx_pll_program(&hdmi.hdmi_data, &pll_data);
	if (r) {
		DSSDBG("Failed to lock PLL\n");
		goto err;
	}

	r = hdmi_ti_4xxx_phy_init(&hdmi.hdmi_data, 0x1);
	if (r) {
		DSSDBG("Failed to start PHY\n");
		goto err;
	}

	hdmi.cfg.cm.mode = hdmi.can_do_hdmi ? HDMI_HDMI : hdmi.mode;
	hdmi.cfg.cm.code = hdmi.code;
	hdmi_ti_4xxx_basic_configure(&hdmi.hdmi_data, &hdmi.cfg);
	if (hdmi.s3d_enable) {
		struct hdmi_core_vendor_specific_infoframe config;
		config.enable = hdmi.s3d_enable;
		config.s3d_structure = hdmi.s3d_mode;
		if (config.s3d_structure == 8)
			config.s3d_ext_data = 1;
		hdmi_core_vsi_config(&hdmi.hdmi_data, &config);
	}
	/* Make selection of HDMI in DSS */
	dss_select_hdmi_venc_clk_source(DSS_HDMI_M_PCLK);

	/*
	 * Select the DISPC clock source as PRCM clock in case when both LCD
	 * panels are disabled and we cannot use DSI PLL for this purpose.
	 */
	if (!dispc_is_channel_enabled(OMAP_DSS_CHANNEL_LCD) &&
	    !dispc_is_channel_enabled(OMAP_DSS_CHANNEL_LCD2))
		dss_select_dispc_clk_source(dssdev->clocks.dispc.dispc_fclk_src);

	/* bypass TV gamma table */
	dispc_enable_gamma_table(0);

	/* tv size */
	dispc_set_digit_size(dssdev->panel.timings.x_res,
			dssdev->panel.timings.y_res);

	dispc_enable_channel(OMAP_DSS_CHANNEL_DIGIT, dssdev->type, 1);

	hdmi_ti_4xxx_wp_video_start(&hdmi.hdmi_data, 1);

	if (hdmi.hdmi_start_frame_cb &&
	    hdmi.custom_set &&
	    hdmi.wp_reset_done)
		(*hdmi.hdmi_start_frame_cb)();

	hdmi.power_state = HDMI_POWER_FULL;
	return 0;
err:
	hdmi_set_l3_cstr(dssdev, false);
	hdmi_runtime_put();
	hdmi.power_state = HDMI_POWER_OFF;
	return -EIO;
}

static int hdmi_power_min(struct omap_dss_device *dssdev)
{
	int r;
	if (hdmi.power_state == HDMI_POWER_FULL)  {
		r = hdmi_ti_4xxx_phy_init(&hdmi.hdmi_data, 0);
		if (r) {
			DSSDBG("Failed to switch off PHY TX_ON\n");
		}

		if (hdmi.hdmi_irq_cb)
			hdmi.hdmi_irq_cb(HDMI_HPD_LOW);

		hdmi_ti_4xxx_wp_video_start(&hdmi.hdmi_data, 0);

		dispc_enable_channel(OMAP_DSS_CHANNEL_DIGIT, dssdev->type, 0);
		hdmi_ti_4xxx_set_pll_pwr(&hdmi.hdmi_data, HDMI_PLLPWRCMD_ALLOFF);
		hdmi_set_l3_cstr(dssdev, false);
		hdmi_runtime_put();
		hdmi.deep_color = HDMI_DEEP_COLOR_24BIT;

		hdmi.power_state = HDMI_POWER_MIN;
	} else if (hdmi.power_state == HDMI_POWER_OFF) {

		r = hdmi_main_runtime_get();
		if (r)
			goto err;
		r = hdmi_runtime_get();
		if (r)
			goto err2;

		r = hdmi_ti_4xxx_phy_init(&hdmi.hdmi_data, 0);

		hdmi_runtime_put();

		if (r) {
			DSSDBG("Failed to start PHY\n");
			goto err2;
		}
	}

	hdmi.power_state = HDMI_POWER_MIN;
	return 0;

//err3:
	hdmi_ti_4xxx_phy_off(&hdmi.hdmi_data, hdmi.set_mode);
err2:
	hdmi_main_runtime_put();
err:
	hdmi.power_state = HDMI_POWER_OFF;
	return -EIO;
}

//TODO: Nikis evaluate after merge
static void hdmi_power_off(struct omap_dss_device *dssdev)
{
	enum hdmi_pwrchg_reasons reason = HDMI_PWRCHG_DEFAULT;

	if (hdmi.set_mode)
		reason = reason | HDMI_PWRCHG_MODE_CHANGE;
	if (dssdev->sync_lost_error)
		reason = reason | HDMI_PWRCHG_RESYNC;

	if (hdmi.power_state == HDMI_POWER_FULL) {
		if (hdmi.hdmi_irq_cb)
			hdmi.hdmi_irq_cb(HDMI_HPD_LOW);

		hdmi_ti_4xxx_wp_video_start(&hdmi.hdmi_data, 0);

		dispc_enable_channel(OMAP_DSS_CHANNEL_DIGIT, dssdev->type, 0);

		hdmi_ti_4xxx_set_pll_pwr(&hdmi.hdmi_data, HDMI_PLLPWRCMD_ALLOFF);
		hdmi_set_l3_cstr(dssdev, false);
		hdmi_runtime_put();
	}

	hdmi.deep_color = HDMI_DEEP_COLOR_24BIT;

	hdmi_ti_4xxx_phy_off(&hdmi.hdmi_data, reason);
	hdmi_main_runtime_put();

	hdmi.power_state = HDMI_POWER_OFF;
}

int omapdss_hdmi_get_pixel_clock(void)
{
	return PICOS2KHZ(hdmi.cfg.timings.pixclock);
}

int omapdss_hdmi_get_mode(void)
{
	return hdmi.mode;
}

int omapdss_hdmi_register_hdcp_callbacks(void (*hdmi_start_frame_cb)(void),
					 void (*hdmi_irq_cb)(int status),
					 bool (*hdmi_power_on_cb)(void))
{
	hdmi.hdmi_start_frame_cb = hdmi_start_frame_cb;
	hdmi.hdmi_irq_cb = hdmi_irq_cb;
	hdmi.hdmi_power_on_cb = hdmi_power_on_cb;

	return hdmi_ti_4xxx_wp_get_video_state(&hdmi.hdmi_data);
}
EXPORT_SYMBOL(omapdss_hdmi_register_hdcp_callbacks);
int omapdss_hdmi_register_cec_callbacks(void (*hdmi_cec_power_on_cb)
					(int phy_addr, int state),
					void (*hdmi_cec_irq_cb)(int))
{
	hdmi.hdmi_cec_power_on_cb = hdmi_cec_power_on_cb;
	hdmi.hdmi_cec_irq_cb = hdmi_cec_irq_cb;
	if (hdmi.hdmi_cec_power_on_cb && (hdmi.source_physical_address != 0) && (hdmi.connection_state == HDMI_CONNECT))
				(*hdmi.hdmi_cec_power_on_cb)(hdmi.source_physical_address, 1);
	return 0;
}
EXPORT_SYMBOL(omapdss_hdmi_register_cec_callbacks);

int omapdss_hdmi_is_auto_displayed(void)
{
    return hdmi.dssdev->cec_auto_switch;
}
EXPORT_SYMBOL(omapdss_hdmi_is_auto_displayed);

void omapdss_hdmi_set_deepcolor(int val)
{
	hdmi.deep_color = val;
}

int omapdss_hdmi_get_deepcolor(void)
{
	return hdmi.deep_color;
}

ssize_t omapdss_hdmi_get_edid(char *edid_buffer)
{
	ssize_t size = hdmi.enabled ? HDMI_EDID_MAX_LENGTH : 0;
	memcpy(edid_buffer, hdmi.edid, size);
	return size;
}

void omapdss_hdmi_set_s3d_mode(int val)
{
	hdmi.s3d_mode = val;
}

int omapdss_hdmi_get_s3d_mode(void)
{
	return hdmi.s3d_mode;
}

void omapdss_hdmi_enable_s3d(bool enable)
{
	hdmi.s3d_enable = enable;
	if (hdmi.enabled)
		omapdss_hdmi_display_set_timing(hdmi.dssdev);
}

int omapdss_hdmi_get_s3d_enable(void)
{
	return hdmi.s3d_enable;
}

int hdmi_get_current_hpd()
{
	return gpio_get_value(hdmi.dssdev->hpd_gpio);
}

static void hdmi_5V_worker(struct work_struct *work)
{
	struct hdmi_5v_work_data *d = container_of(work, typeof(*d), dwork.work);
	int req_state = atomic_read(&d->req_state);
	int current_state = atomic_read(&d->current_state);

	if (hdmi.hdmi_5v) {
		 if (req_state && !current_state) {
			regulator_enable(hdmi.hdmi_5v);
			atomic_set(&d->current_state, req_state);
			return;
		 }
		 if (!req_state && current_state) {
			regulator_disable(hdmi.hdmi_5v);
			atomic_set(&d->current_state, req_state);
			return;
		 }
	}
}

static void update5V_state(void) {
	int cur_state =  atomic_read(&hdmi_5v_work.current_state);
	int req_state = atomic_read(&hdmi_5v_work.req_state);

	if (((cur_state != 1) || (req_state != 1))&& (hdmi_get_current_hpd() || (hdmi.connection_state == HDMI_CONNECT))) {
		if (!req_state) {
			__cancel_delayed_work(&hdmi_5v_work.dwork);
			atomic_set(&hdmi_5v_work.req_state, 1);
			queue_delayed_work(hdmi_5v_workq, &hdmi_5v_work.dwork, msecs_to_jiffies(40));
		}
		return;
	}

	if (((cur_state != 0) || (req_state != 0)) && (!hdmi_get_current_hpd() && (hdmi.connection_state == HDMI_DISCONNECT))) {
		if (req_state) {
			__cancel_delayed_work(&hdmi_5v_work.dwork);
			atomic_set(&hdmi_5v_work.req_state, 0);
			queue_delayed_work(hdmi_5v_workq, &hdmi_5v_work.dwork, msecs_to_jiffies(2000));
		}
		return;
	}
}

static irqreturn_t hpd_irq_handler(int irq, void *ptr)
{
	int hpd = hdmi_get_current_hpd();
	pr_info("hpd %d\n", hpd);
	update5V_state();
	hdmi_panel_hpd_handler(hpd);

	return IRQ_HANDLED;
}

static void hdmi_irq_worker(struct work_struct *work)
{
	struct hdmi_data *data = container_of(work, typeof(struct hdmi_data), work);
	int trans, state = 0;
	unsigned long flags;

	spin_lock_irqsave(&data->work_lock, flags);
	trans = hdmi.connection_trans;
	hdmi.connection_trans = 0;
	spin_unlock_irqrestore(&data->work_lock, flags);

	if (trans == HDMI_CONNECT) {
pr_err("HDMI LINK CONNECT\n");
		state = HDMI_CONNECT;
		hdmi_panel_phy_connected();
	} else if (trans == HDMI_DISCONNECT) {
pr_err("HDMI LINK DISCONNECT\n");
		hdmi_panel_phy_disconnected();
		state = HDMI_DISCONNECT;
		if (hdmi.hdmi_cec_power_on_cb)
				(*hdmi.hdmi_cec_power_on_cb)(hdmi.source_physical_address, 0);
	}

	if (state) {
		spin_lock_irqsave(&data->work_lock, flags);
		hdmi.connection_state = state;
		spin_unlock_irqrestore(&data->work_lock, flags);

		update5V_state();
	}
}

static irqreturn_t hdmi_irq_handler(int irq, void *arg)
{
	int r = 0;

	if (hdmi.hdmi_cec_irq_cb)
		hdmi.hdmi_cec_irq_cb(irq);

	r = hdmi_ti_4xxx_irq_handler(&hdmi.hdmi_data);

	DSSDBG("Received HDMI IRQ = %08x\n", r);

	if (r & (HDMI_LINK_DISCONNECT | HDMI_LINK_CONNECT)) {
pr_err("HDMI LINK try dis-/connect\n");
		spin_lock(&hdmi.work_lock);

		if (r & HDMI_LINK_DISCONNECT) {
			if (hdmi.connection_trans == HDMI_CONNECT) /* last transition still not handled -> do nothing */
				hdmi.connection_trans = 0;
			else {
				hdmi.connection_trans = HDMI_DISCONNECT;
				schedule_work(&hdmi.work);
			}
		} else if (r & HDMI_LINK_CONNECT) {
			if (hdmi.connection_trans == HDMI_DISCONNECT) /* last transition still not handled -> do nothing */
				hdmi.connection_trans = 0;
			else {
				hdmi.connection_trans = HDMI_CONNECT;
				schedule_work(&hdmi.work);
			}
		}


		spin_unlock(&hdmi.work_lock);
	} else if (hdmi.hdmi_irq_cb)
		hdmi.hdmi_irq_cb(r);

	return IRQ_HANDLED;
}

int omapdss_hdmi_display_check_timing(struct omap_dss_device *dssdev,
					struct omap_video_timings *timings)
{
	struct fb_videomode t;

	omapfb_dss2fb_timings(timings, &t);

	/* also check interlaced timings */
	if (!hdmi_set_timings(&t, true)) {
		t.yres *= 2;
		t.vmode |= FB_VMODE_INTERLACED;
	}
	if (!hdmi_set_timings(&t, true))
		return -EINVAL;
	return 0;
}

int omapdss_hdmi_display_set_mode(struct omap_dss_device *dssdev,
				  struct fb_videomode *vm)
{
	int r1, r2;
	DSSINFO("Enter omapdss_hdmi_display_set_mode\n");
	/* turn the hdmi off and on to get new timings to use */
	hdmi.set_mode = true;
	dssdev->driver->disable(dssdev);
	hdmi.set_mode = false;
	r1 = hdmi_set_timings(vm, false) ? 0 : -EINVAL;
	hdmi.custom_set = 1;
	hdmi.code = hdmi.cfg.cm.code;
	hdmi.mode = hdmi.can_do_hdmi ? HDMI_HDMI : hdmi.cfg.cm.mode;
	r2 = dssdev->driver->enable(dssdev);

	if (!r1 && !r2 && hdmi.hdmi_cec_power_on_cb)
		(*hdmi.hdmi_cec_power_on_cb)(hdmi.source_physical_address, 1);
	return r1 ? : r2;
}

void omapdss_hdmi_display_set_timing(struct omap_dss_device *dssdev)
{
	struct fb_videomode t;

	omapfb_dss2fb_timings(&dssdev->panel.timings, &t);
	/* also check interlaced timings */
	if (!hdmi_set_timings(&t, true)) {
		t.yres *= 2;
		t.vmode |= FB_VMODE_INTERLACED;
	}

	omapdss_hdmi_display_set_mode(dssdev, &t);
}

int omapdss_hdmi_display_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	DSSINFO("ENTER hdmi_display_enable\n");

	mutex_lock(&hdmi.lock);

	if (hdmi.enabled)
		goto err0;

	r = omap_dss_start_device(dssdev);
	if (r) {
		DSSERR("failed to start device\n");
		goto err0;
	}

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r) {
			DSSERR("failed to enable GPIO's\n");
			goto err1;
		}
	}

	hdmi.hdmi_reg = regulator_get(&hdmi.pdev->dev, "hdmi_vref");
	if (IS_ERR_OR_NULL(hdmi.hdmi_reg)) {
		DSSERR("Failed to get hdmi_vref regulator\n");
		r = PTR_ERR(hdmi.hdmi_reg) ? : -ENODEV;
		goto err2;
	}

	r = regulator_enable(hdmi.hdmi_reg);
	if (r) {
		DSSERR("failed to enable hdmi_vref regulator\n");
		goto err3;
	}

	r = hdmi_power_on(dssdev);
	if (r) {
		DSSERR("failed to power on device\n");
		goto err4;
	}

	hdmi.enabled = true;

	mutex_unlock(&hdmi.lock);
	return 0;

err4:
	regulator_disable(hdmi.hdmi_reg);
err3:
	regulator_put(hdmi.hdmi_reg);
err2:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
err1:
	omap_dss_stop_device(dssdev);
err0:
	mutex_unlock(&hdmi.lock);
	return r;
}

void omapdss_hdmi_display_disable(struct omap_dss_device *dssdev)
{
	DSSINFO("Enter hdmi_display_disable\n");

	mutex_lock(&hdmi.lock);

	if (!hdmi.enabled)
		goto done;

	hdmi.enabled = false;
	hdmi.wp_reset_done = false;

	hdmi_power_min(dssdev);
	if (dssdev->sync_lost_error == 0)
		//FIXME read edid after suspend too because you can plug another tv during suspend
		if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
			/* clear EDID and mode on disable only */
			hdmi.edid_set = false;
			hdmi.custom_set = 0;
			pr_info("hdmi: clearing EDID info\n");
		}
	regulator_disable(hdmi.hdmi_reg);

	regulator_put(hdmi.hdmi_reg);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omap_dss_stop_device(dssdev);
done:
	mutex_unlock(&hdmi.lock);
}

static void omapdss_hdmi_suspend(struct omap_dss_device *dssdev)
{
	pr_debug("ENTER omapdss_hdmi_suspend\n");
#if 0
	if (hdmi.connection_state == HDMI_CONNECT) {
		if (hdmi.hdmi_5v) {
			int r = regulator_disable(hdmi.hdmi_5v);
			if (r)
				DSSERR("failed to disable hdmi_5v regulator\n");
			else
				hdmi.connection_state = HDMI_DISCONNECT;
		}
	}
#endif
	hdmi_power_off(dssdev);
}

static void omapdss_hdmi_resume(struct omap_dss_device *dssdev)
{
	pr_debug("ENTER omapdss_hdmi_resume\n");
	hdmi_power_min(dssdev);
}

static int hdmi_get_clocks(struct platform_device *pdev)
{
	struct clk *clk;

	clk = clk_get(&pdev->dev, "sys_clk");
	if (IS_ERR(clk)) {
		DSSERR("can't get sys_clk\n");
		return PTR_ERR(clk);
	}

	hdmi.sys_clk = clk;

	clk = clk_get(&pdev->dev, "hdmi_clk");
	if (IS_ERR(clk)) {
		DSSERR("can't get hdmi_clk\n");
		clk_put(hdmi.sys_clk);
		return PTR_ERR(clk);
	}

	hdmi.hdmi_clk = clk;

	return 0;
}

static void hdmi_put_clocks(void)
{
	if (hdmi.sys_clk)
		clk_put(hdmi.sys_clk);
	if (hdmi.hdmi_clk)
		clk_put(hdmi.hdmi_clk);
}

static int omapdss_hdmihw_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	omapdss_hdmi_suspend(hdmi.dssdev);
	return 0;
}

static int omapdss_hdmihw_resume(struct platform_device *pdev)
{
	omapdss_hdmi_resume(hdmi.dssdev);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void omapdss_early_suspend(struct early_suspend *h)
{
	hdmi_panel_early_suspend(hdmi.dssdev);
}

static void omapdss_early_resume(struct early_suspend *h)
{
	hdmi_panel_early_resume(hdmi.dssdev);
}

static struct early_suspend hdmi_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
	.suspend = omapdss_early_suspend,
	.resume = omapdss_early_resume,
};
#endif

/* HDMI HW IP initialisation */
static int omapdss_hdmihw_probe(struct platform_device *pdev)
{
	struct resource *hdmi_mem;
	struct omap_dss_board_info *board_data;
	int r;

	hdmi.pdata = pdev->dev.platform_data;
	hdmi.pdev = pdev;

	mutex_init(&hdmi.lock);

	hdmi_5v_workq = create_singlethread_workqueue("hdmi_5v_work");
	INIT_DELAYED_WORK(&hdmi_5v_work.dwork, hdmi_5V_worker);
	atomic_set(&hdmi_5v_work.current_state, 0);
	atomic_set(&hdmi_5v_work.req_state, 0);

	INIT_WORK(&hdmi.work, hdmi_irq_worker);
	spin_lock_init(&hdmi.work_lock);
	hdmi.connection_state = HDMI_DISCONNECT;
	hdmi.connection_trans = 0;

	/* save reference to HDMI device */
	board_data = hdmi.pdata->board_data;
	for (r = 0; r < board_data->num_devices; r++) {
		if (board_data->devices[r]->type == OMAP_DISPLAY_TYPE_HDMI)
			hdmi.dssdev = board_data->devices[r];
	}
	if (!hdmi.dssdev) {
		DSSERR("can't get HDMI device\n");
		return -EINVAL;
	}

	hdmi_mem = platform_get_resource(hdmi.pdev, IORESOURCE_MEM, 0);
	if (!hdmi_mem) {
		DSSERR("can't get IORESOURCE_MEM HDMI\n");
		return -EINVAL;
	}

	/* Base address taken from platform */
	hdmi.hdmi_data.base_wp = ioremap(hdmi_mem->start,
						resource_size(hdmi_mem));
	if (!hdmi.hdmi_data.base_wp) {
		DSSERR("can't ioremap WP\n");
		return -ENOMEM;
	}

	hdmi.hdmi_5v = regulator_get(&hdmi.pdev->dev, "hdmi_5v");
	if (IS_ERR_OR_NULL(hdmi.hdmi_5v)) {
		DSSERR("Failed to get \"hdmi_5v\" regulator\n");
		r = PTR_ERR(hdmi.hdmi_5v) ? : -ENODEV;
		hdmi.hdmi_5v = NULL;
		//goto reg_err; FixMe: Currently it continues without 5V
	}

	r = hdmi_get_clocks(pdev);
	if (r) {
		iounmap(hdmi.hdmi_data.base_wp);
		goto err_clks;
	}

	pm_runtime_enable(&pdev->dev);

	hdmi.hdmi_irq = platform_get_irq(pdev, 0);


	hdmi.hdmi_data.hdmi_core_sys_offset = HDMI_CORE_SYS;
	hdmi.hdmi_data.hdmi_core_av_offset = HDMI_CORE_AV;
	hdmi.hdmi_data.hdmi_pll_offset = HDMI_PLLCTRL;
	hdmi.hdmi_data.hdmi_phy_offset = HDMI_PHY;

	hdmi.wp_reset_done = false;

	hdmi_panel_init();

	hdmi_power_min(hdmi.dssdev);

	r = request_irq(gpio_to_irq(hdmi.dssdev->hpd_gpio), hpd_irq_handler,
			IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"hpd", NULL);
	if (r < 0) {
		pr_err("hdmi: request_irq %d failed\n",
			gpio_to_irq(hdmi.dssdev->hpd_gpio));
		goto err_irq;
	}

	r = request_irq(hdmi.hdmi_irq, hdmi_irq_handler, 0, "OMAP HDMI", NULL);
	if (r < 0) {
		pr_err("hdmi: request_irq %s failed\n",
			pdev->name);
		goto err_irq2;
	}

	return 0;

err_irq2:
	free_irq(gpio_to_irq(hdmi.dssdev->hpd_gpio), hpd_irq_handler);
err_irq:
	pm_runtime_disable(&pdev->dev);
	hdmi_put_clocks();
err_clks:
	if (hdmi.hdmi_5v)
		regulator_put(hdmi.hdmi_5v);
//reg_err:
	iounmap(hdmi.hdmi_data.base_wp);
	return r;
}

static int omapdss_hdmihw_remove(struct platform_device *pdev)
{
	hdmi_panel_exit();

	free_irq(hdmi.hdmi_irq, hdmi_irq_handler);

	if (hdmi.dssdev)
		free_irq(gpio_to_irq(hdmi.dssdev->hpd_gpio), hpd_irq_handler);
	hdmi.dssdev = NULL;

	pm_runtime_disable(&pdev->dev);

	hdmi_put_clocks();

	if (hdmi.hdmi_5v)
		regulator_put(hdmi.hdmi_5v);

	iounmap(hdmi.hdmi_data.base_wp);

	return 0;
}

static struct platform_driver omapdss_hdmihw_driver = {
	.probe          = omapdss_hdmihw_probe,
	.remove         = omapdss_hdmihw_remove,
	.suspend 	= omapdss_hdmihw_suspend,
	.resume 	= omapdss_hdmihw_resume,
	.driver         = {
		.name   = "omapdss_hdmi",
		.owner  = THIS_MODULE,
	},
};

int hdmi_init_platform_driver(void)
{
	return platform_driver_register(&omapdss_hdmihw_driver);
}

void hdmi_uninit_platform_driver(void)
{
	return platform_driver_unregister(&omapdss_hdmihw_driver);
}

void hdmi_dump_regs(struct seq_file *s)
{
	if (hdmi_main_runtime_get())
		return;

	hdmi_ti_4xxx_dump_regs(&hdmi.hdmi_data, s);

	hdmi_main_runtime_put();
}
