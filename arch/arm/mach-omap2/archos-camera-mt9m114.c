/*
 * MT9M114 Board configuration
 *
 */

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/regulator/machine.h>
#include <asm/mach-types.h>
#include <linux/module.h>

#include <mach/gpio.h>
#include <plat/archos-gpio.h>
#include <mach/board-archos.h>

#include "mux.h"

#define DEVICE_NAME	"archos_camera_glue"

static struct regulator *cam_vdd;
static struct regulator *cam_1v8;
static int reset_gpio;

static void __init init_cam_clk(void)
{
	struct clk *auxclk2;
	struct clk *aux_src_clk;
	struct clk *aux_parent_clk;
	static const char *aux_name = "auxclk2_ck";
	static const char *aux_src_name = "auxclk2_src_ck";
	static const char *aux_parent_name = "dpll_per_m3x2_ck";
	
	auxclk2 = clk_get(NULL, aux_name);
	if (IS_ERR(auxclk2)) {
		pr_err("%s: auxclk2 not available\n", __func__);
		return;
	}

	aux_src_clk = clk_get(NULL, aux_src_name);
	if (IS_ERR(aux_src_clk)) {
		pr_err("%s: %s is not available\n", __func__, aux_src_name);
		goto err;
	}
	
	aux_parent_clk = clk_get(NULL, aux_parent_name);
	if (IS_ERR(aux_parent_clk)) {
		pr_err("%s: %s is not available\n", __func__, aux_parent_name);
		goto err1;
	}
	
	pr_debug("%s: %s freq is: %lu\n", 
			__func__, aux_parent_name, clk_get_rate(aux_parent_clk));
	
	if (IS_ERR_VALUE(clk_set_rate(aux_parent_clk, 192000000))) {
		pr_err("%s: cannot set %s to 192MHz\n", __func__, aux_parent_name);
		goto err2;
	}
	
	pr_debug("%s: auxclk2 freq is: %lu\n", 
			__func__, clk_get_rate(auxclk2));
	
	if (IS_ERR_VALUE(clk_set_parent(aux_src_clk, aux_parent_clk))) {
		pr_err("%s: cannot set auxclk2 clock parent\n", __func__);
		goto err2;
	}

	if (IS_ERR_VALUE(clk_set_rate(auxclk2, 12000000))) {
		pr_err("%s: cannot set auxclk2 clock rate\n", __func__);
		goto err2;
	}

	pr_debug("%s: auxclk2 now at %lu\n", __func__, clk_get_rate(auxclk2));

err2:
	clk_put(aux_parent_clk);
err1:
	clk_put(aux_src_clk);
err:
	clk_put(auxclk2);
}

#if 0
static void auxclk2_en(int enable)
{
	struct clk *auxclk2;

	auxclk2 = clk_get(NULL, "auxclk2");
	if (!IS_ERR(auxclk2)) {
		if (enable) {
			printk("Enabling CAM_CLK\n");
			if ( clk_enable(auxclk2) != 0) {
				printk(KERN_ERR "failed to enable auxclk2\n");
			}
		} else {
			printk("Disabling CAM_CLK\n");
			clk_disable(auxclk2);
		}
		
		clk_put(auxclk2);
	}
}
#endif


static void enable(int on_off)
{
	static int state = 0;

	if (state == on_off)
		return;

	if (on_off) {

		regulator_enable(cam_vdd);

		if (!IS_ERR(cam_1v8)) {
			regulator_enable(cam_1v8);
		}

		msleep(10);

		init_cam_clk();

	} else {
		if (!IS_ERR(cam_1v8))
			regulator_disable(cam_1v8);

		regulator_disable(cam_vdd);
	}

	state = on_off;
}

static int archos_camera_glue_probe(struct platform_device * pdev)
{
	cam_vdd = regulator_get(&pdev->dev, "cam_vdd");
	if (IS_ERR(cam_vdd)) {
		dev_err(&pdev->dev, "no cam_vdd rail, abort.\n");
		return -ENODEV;
	}

	regulator_force_disable(cam_vdd);

	cam_1v8 = regulator_get(&pdev->dev, "cam_1v8");
	if (IS_ERR(cam_1v8))
		dev_info(&pdev->dev, "no cam_1v8 rail, skip.\n");
	else
		regulator_force_disable(cam_1v8);

	// assert that power rails are low
	msleep(20);

	enable(1);

	return 0;
}

static struct platform_driver archos_camera_glue_driver= {
	.probe = archos_camera_glue_probe,
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
	}
};

int __init archos_camera_mt9m114_init(void)
{
	const struct archos_camera_config *camera_cfg;
	struct platform_device *pdev;
	int ret;

	pr_debug("%s\n", __FUNCTION__);

	camera_cfg = omap_get_config( ARCHOS_TAG_CAMERA, struct archos_camera_config );
	if (camera_cfg == NULL) {
		printk(KERN_ERR "archos_camera_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( system_rev >= camera_cfg->nrev ) {
		printk(KERN_ERR "archos_camera_init: system_rev (%i) >= nrev (%i)\n",
			system_rev, camera_cfg->nrev);
		return -ENODEV;
	}

	if (gpio_is_valid(camera_cfg->rev[system_rev].reset)) {
		reset_gpio = camera_cfg->rev[system_rev].reset;
		omap_mux_init_gpio(reset_gpio, OMAP_PIN_INPUT);
		gpio_set_value(reset_gpio, 0);
	} else {
		ret = -ENODEV;
		return ret;
	}

	ret = platform_driver_register(&archos_camera_glue_driver);
	if (ret)
		return ret;

	pdev = platform_device_alloc(DEVICE_NAME, -1);
	if (!pdev) {
		ret = -ENOMEM;
		return ret;
	}

	ret = platform_device_add(pdev);
	if (ret)
		return ret;

	omap_mux_init_signal("fref_clk2_out", OMAP_PIN_OUTPUT);
	
	omap_mux_init_signal("csi21_dx0.csi21_dx0", OMAP_PIN_INPUT);
	omap_mux_init_signal("csi21_dy0.csi21_dy0", OMAP_PIN_INPUT);
	omap_mux_init_signal("csi21_dx1.csi21_dx1", OMAP_PIN_INPUT);
	omap_mux_init_signal("csi21_dy1.csi21_dy1", OMAP_PIN_INPUT);

	return 0;
}

device_initcall(archos_camera_mt9m114_init);
