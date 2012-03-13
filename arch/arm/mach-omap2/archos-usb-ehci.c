/*
 *    archos-usb-ehci.c : 29/03/2011
 *    g.revaillot, revaillot@archos.com
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <plat/usb.h>
#include <mach/board-archos.h>
#include <linux/clk.h>

#include "mux.h"

static struct clk *aux_clk;

static int hub_rst = UNUSED_GPIO;
static int husb1_pwron = UNUSED_GPIO;
static int pwron_5v = UNUSED_GPIO;
static bool suspended;

int archos_ehci_bus_suspend(void)
{
	pr_debug("%s\n", __FUNCTION__);
	return 0;

	if (aux_clk && !suspended) {
		pr_debug("Disabling aux_clk\n");
		clk_disable(aux_clk);
		}
	udelay(100);

	suspended = true;

	return 0;
}

int archos_ehci_bus_resume(void)
{
	pr_debug("%s\n", __FUNCTION__);
	return 0;

	if (suspended) {
		if (aux_clk) {
			pr_debug("Enabling aux_clk\n");
			clk_enable(aux_clk);
		}
	}

	/* hard reset of phy (still more safe than stp wakeup) */
	if ((husb1_pwron > 0)) {
		msleep(100);
		gpio_set_value(husb1_pwron, 0);
		msleep(100);
		gpio_set_value(husb1_pwron, 1);
		pr_err("Phy is now ON\n");
	}

	suspended = false;

	msleep(100);

	return 0;
}

int archos_ehci_bus_disable(void)
{
	pr_debug("%s\n", __FUNCTION__);

	if (!suspended) {
		if (aux_clk) {
			pr_debug("Disabling aux_clk\n");
			clk_disable(aux_clk);
		}
	}

	if (gpio_is_valid(husb1_pwron)) {
		gpio_set_value(husb1_pwron, 0);
		pr_debug("Phy is now OFF\n");
	}
#if 0
	if (gpio_is_valid(hub_pwron))
		gpio_set_value(hub_pwron, 0);
#endif
	if (gpio_is_valid(hub_rst))
		gpio_set_value(hub_rst, 1);

	if (gpio_is_valid(pwron_5v))
		gpio_set_value(pwron_5v, 0);

	return 0;
}

int archos_ehci_bus_enable(void)
{
	pr_debug("%s\n", __FUNCTION__);

	if (aux_clk) {
		pr_debug("Enabling aux_clk\n");
		clk_enable(aux_clk);
	}
	msleep(10);
	if (gpio_is_valid(husb1_pwron)) {
		gpio_set_value(husb1_pwron, 1);
		msleep(100);
		gpio_set_value(husb1_pwron, 0);
		msleep(100);
		gpio_set_value(husb1_pwron, 1);
		pr_debug("Phy is now ON\n");

	}

	msleep(100);
#if 0
	if (gpio_is_valid(hub_pwron)){
		gpio_set_value(hub_pwron, 1);
		msleep(100);
	}
#endif
	// XXX check ^^ timing
	if (gpio_is_valid(hub_rst))
		gpio_set_value(hub_rst, 0);

	if (gpio_is_valid(pwron_5v))
		gpio_set_value(pwron_5v, 1);

	suspended = false;

	return 0;
}

#if 0
static struct usbhs_omap_platform_data usbhs_pdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset = false,
	.deep_bus_suspend = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL,
	.platform_bus_suspend = archos_ehci_bus_suspend,
	.platform_bus_resume = archos_ehci_bus_resume,
	.platform_bus_disable = archos_ehci_bus_disable,
	.platform_bus_enable = archos_ehci_bus_enable,
};
static struct usbhs_omap_platform_data usbhs_pdata_omap3 __initconst = {
	.port_mode[0] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset = false,
	.deep_bus_suspend = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL,
	.platform_bus_suspend = archos_ehci_bus_suspend,
	.platform_bus_resume = archos_ehci_bus_resume,
	.platform_bus_disable = archos_ehci_bus_disable,
	.platform_bus_enable = archos_ehci_bus_enable,
	.es2_compatibility = true,
#endif
static struct usbhs_omap_board_data usbhs_board_data __initconst = {
		.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
		.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
		.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
		.phy_reset = false,
		.platform_bus_suspend = archos_ehci_bus_suspend,
		.platform_bus_resume = archos_ehci_bus_resume,
		.platform_bus_disable = archos_ehci_bus_disable,
		.platform_bus_enable = archos_ehci_bus_enable,
};

static struct usbhs_omap_board_data usbhs_board_data_omap3 __initconst = {
		.port_mode[0] = OMAP_USBHS_PORT_MODE_UNUSED,
		.port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
		.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
		.phy_reset = false,
		.es2_compatibility = true,
};

static void __init init_ehci_clock_init(void)
{
	struct clk *aux_src_clk;
	struct clk *aux_parent_clk;
	static const char *aux_clk_name = "auxclk4_ck";
	static const char *aux_src_clk_name = "auxclk4_src_ck";
	static const char *aux_clk_ball = "fref_clk4_out";
	static const char *aux_parent_clk_name = "dpll_per_m3x2_ck";

	if (cpu_is_omap34xx())
		return;

	aux_clk = clk_get(NULL, aux_clk_name);
	if (IS_ERR(aux_clk)) {
		pr_err("%s: %s not available\n", __func__, aux_clk_name);
		return;
	}

	aux_src_clk = clk_get(NULL, aux_src_clk_name);
	if (IS_ERR(aux_src_clk)) {
		pr_err("%s: %s not available\n", __func__, aux_src_clk_name);
		goto err;
	}

	aux_parent_clk = clk_get(NULL, aux_parent_clk_name);
	if (IS_ERR(aux_parent_clk)) {
		pr_err("%s: %s is not available\n", __func__, aux_parent_clk_name);
		goto err1;
	}

	pr_debug("%s: %s freq is: %lu\n",
			__func__, aux_parent_clk_name, clk_get_rate(aux_parent_clk));

	if (IS_ERR_VALUE(clk_set_rate(aux_parent_clk, 192000000))) {
		pr_err("%s: cannot set %s to 192MHz\n", __func__, aux_parent_clk_name);
		goto err2;
	}

	pr_debug("%s: %s freq is: %lu\n",
			__func__, aux_clk_name, clk_get_rate(aux_clk));

	if (IS_ERR_VALUE(clk_set_parent(aux_src_clk, aux_parent_clk))) {
		pr_err("%s: cannot set %s clock parent\n", __func__, aux_clk_name);
		goto err2;
	}

	if (IS_ERR_VALUE(clk_set_rate(aux_clk, 12000000))) {
		pr_err("%s: cannot set %s clock rate\n", __func__, aux_clk_name);
		goto err2;
	}

	pr_info("%s: %s now at %lu\n", __func__, aux_clk_name, clk_get_rate(aux_clk));
	pr_err("Enabling aux_clk\n");
	clk_enable(aux_clk);

	omap_mux_init_signal(aux_clk_ball, OMAP_PIN_OUTPUT);

	clk_put(aux_src_clk);
	clk_put(aux_parent_clk);

	return;
err2:
	clk_put(aux_parent_clk);
err1:
	clk_put(aux_src_clk);
err:
	clk_put(aux_clk);

	return;
}

void __init archos_omap3_ehci_init(void)
{
	const struct archos_usb_config * usb_cfg;
	const struct archos_usb_conf * cfg;

	int ret;

	usb_cfg = omap_get_config( ARCHOS_TAG_USB, struct archos_usb_config );

	if (NULL == usb_cfg)
		return;

	cfg = hwrev_ptr(usb_cfg, system_rev);
	if (IS_ERR(cfg)) {
		pr_err("%s: no device configuration for hardware_rev %i\n",
				__func__, system_rev);
		return;
	}

#if 0
	// hub power
	if (gpio_is_valid((hub_pwron = cfg->enable_usb_hub))) {
		ret = gpio_request(hub_pwron, "HUB_PWRON");
		if (ret) {
			pr_err("Cannot request GPIO %d\n", hub_pwron);
			goto error;
		}
		omap_mux_init_gpio(hub_pwron, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);

		gpio_export(hub_pwron, false);
		gpio_direction_output(hub_pwron, 0);
	}
#endif

	// hub rst
	if (gpio_is_valid(hub_rst = cfg->usb_hub_rst)) {
		ret = gpio_request(hub_rst, "HUB_RST");
		if (ret) {
			pr_err("Cannot request GPIO %d\n", hub_rst);
			goto error;
		}

		omap_mux_init_gpio(hub_rst, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);

		gpio_export(hub_rst, false);
		gpio_direction_output(hub_rst, 0);
	}


	// phy
	if (gpio_is_valid(husb1_pwron = cfg->enable_usb_ehci)) {
		ret = gpio_request(husb1_pwron, "HUSB1_EN");
		if (ret) {
			pr_err("Cannot request GPIO %d\n", husb1_pwron);
			goto error;
		}
		omap_mux_init_gpio(husb1_pwron, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);

		gpio_export(husb1_pwron, false);
		gpio_direction_output(husb1_pwron, 0);
	}

	// go
	init_ehci_clock_init();

	udelay(500);


	if (gpio_is_valid(husb1_pwron))
		gpio_set_value(husb1_pwron, 1);

	if (!cpu_is_omap34xx())
		usbhs_init(&usbhs_board_data);
	else
		usbhs_init(&usbhs_board_data_omap3);
#if 0
	if (gpio_is_valid(hub_pwron))
		gpio_set_value(hub_pwron, 1);
#endif
	if (gpio_is_valid(hub_rst))
		gpio_set_value(hub_rst, 0);

 error:
	return;
}

void __init archos_omap4_ehci_init(void)
{
	const struct archos_usb_config * usb_cfg;
	const struct archos_usb_conf * cfg;

	int ret;

	usb_cfg = omap_get_config( ARCHOS_TAG_USB, struct archos_usb_config );

	if (NULL == usb_cfg)
		return;

	cfg = hwrev_ptr(usb_cfg, system_rev);
	if (IS_ERR(cfg)) {
		pr_err("%s: no device configuration for system_rev %i\n",
				__func__, system_rev);
		return;
	}
#if 0
	// hub pwr
	if (gpio_is_valid(hub_pwron = cfg->enable_usb_hub)) {
		ret = gpio_request(hub_pwron, "HUB_PWRON");
		if (ret) {
			pr_err("Cannot request GPIO %d\n", hub_pwron);
			goto error;
		}
		omap_mux_init_gpio(hub_pwron, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);

		gpio_export(hub_pwron, false);
		gpio_direction_output(hub_pwron, 0);
	}
#endif
	// hub rst
	if (gpio_is_valid(hub_rst = cfg->usb_hub_rst)) {
		ret = gpio_request(hub_rst, "HUB_RST");
		if (ret) {
			pr_err("Cannot request GPIO %d\n", hub_rst);
			goto error;
		}
		omap_mux_init_gpio(hub_rst, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);

		gpio_export(hub_rst, false);
		gpio_direction_output(hub_rst, 0);
	}

	// phy
	if (gpio_is_valid(husb1_pwron = cfg->enable_usb_ehci)) {
		ret = gpio_request(husb1_pwron, "HUSB1_EN");
		if (ret) {
			pr_err("Cannot request GPIO %d\n", husb1_pwron);
			goto error;
		}
		omap_mux_init_gpio(husb1_pwron, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);

		gpio_export(husb1_pwron, false);
		gpio_direction_output(husb1_pwron, 0);
	}

	// pwron_5v
	if (gpio_is_valid(pwron_5v = cfg->enable_5v)) {
		ret = gpio_request(pwron_5v, "5V_USB");
		if (ret) {
			pr_err("Cannot request GPIO %d\n", hub_rst);
			goto error;
		}
		omap_mux_init_gpio(pwron_5v, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);

		gpio_export(pwron_5v, false);
		gpio_direction_output(pwron_5v, 0);
	}

	// go
	init_ehci_clock_init();

	udelay(500);

	if (gpio_is_valid(husb1_pwron))
		gpio_set_value(husb1_pwron, 1);

	usbhs_board_data.reset_gpio_port[0] = husb1_pwron;
	usbhs_board_data.transceiver_clk[0] = aux_clk;

	usbhs_init(&usbhs_board_data);
#if 0
	if (gpio_is_valid(hub_pwron))
		gpio_set_value(hub_pwron, 1);
#endif
	if (gpio_is_valid(hub_rst))
		gpio_set_value(hub_rst, 0);

	if (gpio_is_valid(pwron_5v))
		gpio_set_value(pwron_5v, 1);

error:
	return;
}
