/*
 * OMAP and TPS6236x specific initialization
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Vishwanath BS
 * Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>

#include "pm.h"
#include "vc.h"
#include "mux.h"
#include "twl-common.h"
#include <plat/omap_hwmod.h>

/* Voltage limits supported */
#define MIN_VOLTAGE_TPS62360_62_UV	770000
#define MAX_VOLTAGE_TPS62360_62_UV	1400000

#define MIN_VOLTAGE_TPS62361_UV		500000
#define MAX_VOLTAGE_TPS62361_UV		1770000

#define MAX_VOLTAGE_RAMP_TPS6236X_UV	32000

/*
 * This is the voltage delta between 2 values in voltage register.
 * when switching voltage V1 to V2, TPS62361 can ramp up or down
 * initially with step sizes of 20mV with a last step of 10mV.
 * In the case of TPS6236[0|2], it is a constant 10mV steps
 * we choose the 10mV step for linearity when SR is configured.
 */
#define STEP_SIZE_TPS6236X		10000

/* I2C access parameters */
#define I2C_TPS6236X_SLAVE_ADDR		0x60

#define DEF_SET_REG(VSEL0, VSEL1)	(((VSEL1) << 1 | (VSEL0) << 0) & 0x3)
#define REG_TPS6236X_SET_0		0x00
#define REG_TPS6236X_SET_1		0x01
#define REG_TPS6236X_SET_2		0x02
#define REG_TPS6236X_SET_3		0x03
#define REG_TPS6236X_CTRL		0x04
#define REG_TPS6236X_TEMP		0x05
#define REG_TPS6236X_RAMP_CTRL		0x06
#define REG_TPS6236X_CHIP_ID0		0x08
#define REG_TPS6236X_CHIP_ID1		0x09

#define MODE_TPS6236X_AUTO_PFM_PWM	0x00
#define MODE_TPS6236X_FORCE_PWM		BIT(7)

/* We use Auto PFM/PWM mode currently seems to have the best trade off */
#define VOLTAGE_PFM_MODE_VAL		MODE_TPS6236X_AUTO_PFM_PWM

#define REG_TPS6236X_RAMP_CTRL_RMP_MASK	(0x7 << 5)
#define REG_TPS6236X_RAMP_CTRL_EN_DISC	BIT(2)
#define REG_TPS6236X_RAMP_CTRL_RAMP_PFM	BIT(1)

#define REG_TPS6236X_CTRL_PD_EN		BIT(7)
#define REG_TPS6236X_CTRL_PD_VSEL0	BIT(6)
#define REG_TPS6236X_CTRL_PD_VSEL1	BIT(5)

/* Which register do we use by default? */
static int __initdata default_reg = -1;

/* Do we need to setup internal pullups? */
static int __initdata pd_vsel0 = -1;
static int __initdata pd_vsel1 = -1;

static int __init _bd_setup(char *name, int gpio_vsel, int *pull, int *pd_vsel)
{
	int pull_dir;
	int r;

	if (gpio_vsel == -1) {
		if (*pull != -1) {
			*pd_vsel = (*pull == OMAP_PIN_OFF_OUTPUT_HIGH);
			*pull = *pd_vsel;
		} else {
			*pull = 0;
		}
		return 0;
	}

	/* if we have a pull gpio, with bad dir, pull low */
	if (*pull == -1 || (*pull != OMAP_PIN_OFF_OUTPUT_HIGH &&
			    *pull != OMAP_PIN_OFF_OUTPUT_LOW))
		*pull = OMAP_PIN_OFF_OUTPUT_LOW;

	r = omap_mux_init_gpio(gpio_vsel, *pull);
	if (r) {
		pr_err("%s: unable to mux gpio%d=%d\n", __func__,
		       gpio_vsel, r);
		goto out;
	}

	pull_dir = (*pull == OMAP_PIN_OFF_OUTPUT_HIGH);
	*pull = pull_dir;

	r = gpio_request(gpio_vsel, name);
	if (r) {
		pr_err("%s: unable to req gpio%d=%d\n", __func__,
		       gpio_vsel, r);
		goto out;
	}
	r = gpio_direction_output(gpio_vsel, pull_dir);
	if (r) {
		pr_err("%s: unable to pull[%d] gpio%d=%d\n", __func__,
		       gpio_vsel, pull_dir, r);
		gpio_free(gpio_vsel);
		goto out;
	}
out:
	return r;
}

static unsigned long tps6236x_vsel_to_uv(const u8 vsel);
static u8 tps6236x_uv_to_vsel(unsigned long uv);

static struct omap_voltdm_pmic omap4_mpu_pmic = {
	.slew_rate		= 32000,
	.step_size		= STEP_SIZE_TPS6236X,
	.startup_time		= 1000,
	.shutdown_time		= 1,
	.vddmin			= MIN_VOLTAGE_TPS62361_UV,
	.vddmax			= MAX_VOLTAGE_TPS62361_UV,
	.volt_setup_time	= 0,
	.switch_on_time		= 1000,
	.vp_erroroffset		= OMAP4_VP_CONFIG_ERROROFFSET,
	.vp_vstepmin		= OMAP4_VP_VSTEPMIN_VSTEPMIN,
	.vp_vstepmax		= OMAP4_VP_VSTEPMAX_VSTEPMAX,
	.vp_timeout_us		= OMAP4_VP_VLIMITTO_TIMEOUT_US,
	.i2c_slave_addr		= I2C_TPS6236X_SLAVE_ADDR,
	.volt_reg_addr		= REG_TPS6236X_SET_0,
	.cmd_reg_addr		= REG_TPS6236X_SET_0,
	.i2c_high_speed		= true,
	.vsel_to_uv		= tps6236x_vsel_to_uv,
	.uv_to_vsel		= tps6236x_uv_to_vsel,
};

static unsigned long tps6236x_vsel_to_uv(const u8 vsel)
{
	return omap4_mpu_pmic.vddmin +
		(STEP_SIZE_TPS6236X * (vsel & ~VOLTAGE_PFM_MODE_VAL));
}

static u8 tps6236x_uv_to_vsel(unsigned long uv)
{
	if (!uv)
		return 0;

	/* Round off requests to limits */
	if (uv > omap4_mpu_pmic.vddmax) {
		pr_err("%s:Request for overvoltage[%ld] than supported[%u]\n",
		       __func__, uv, omap4_mpu_pmic.vddmax);
		uv = omap4_mpu_pmic.vddmax;
	}
	if (uv < omap4_mpu_pmic.vddmin) {
		pr_err("%s:Request for undervoltage[%ld] than supported[%u]\n",
		       __func__, uv, omap4_mpu_pmic.vddmin);
		uv = omap4_mpu_pmic.vddmin;
	}
	return DIV_ROUND_UP(uv - omap4_mpu_pmic.vddmin, STEP_SIZE_TPS6236X) |
			VOLTAGE_PFM_MODE_VAL;
}

/* Convert the ramp voltage to ramp value. */
static u8 __init tps6236x_ramp_value(unsigned long uv)
{
	if (!uv)
		return 0;

	if (uv > MAX_VOLTAGE_RAMP_TPS6236X_UV) {
		pr_err("%s: uv%ld greater than max %d\n", __func__,
			uv, MAX_VOLTAGE_RAMP_TPS6236X_UV);
		uv = MAX_VOLTAGE_RAMP_TPS6236X_UV;
	}
	return fls(MAX_VOLTAGE_RAMP_TPS6236X_UV / uv) - 1;
}

/**
 * omap4_twl_tps62361_enable() - Enable tps chip
 *
 * This function enables TPS chip by associating SYSEN signal
 * to APE resource group of TWL6030.
 *
 * Returns 0 on sucess, error is returned if I2C read/write fails.
 */
static int __init omap4_twl_tps62361_enable(struct voltagedomain *voltdm)
{
	int ret = 0;
	u8 val;

	/* Dont trust the bootloader. start with max, pm will set to proper */
	val = voltdm->pmic->uv_to_vsel(voltdm->pmic->vddmax);
	ret = omap_vc_bypass_send_i2c_msg(voltdm, voltdm->pmic->i2c_slave_addr,
			default_reg, val);

	/* Setup Ramp */
	val = tps6236x_ramp_value(voltdm->pmic->slew_rate) <<
		__ffs(REG_TPS6236X_RAMP_CTRL_RMP_MASK);
	val &= REG_TPS6236X_RAMP_CTRL_RMP_MASK;

	/* We would like to ramp the voltage asap */
	val |= REG_TPS6236X_RAMP_CTRL_RAMP_PFM;

	/* We would like to ramp down the voltage asap as well*/
	val |= REG_TPS6236X_RAMP_CTRL_EN_DISC;

	ret = omap_vc_bypass_send_i2c_msg(voltdm, voltdm->pmic->i2c_slave_addr,
			REG_TPS6236X_RAMP_CTRL, val);
	if (ret)
		goto out;

	/* Setup the internal pulls to select if needed */
	if (pd_vsel0 != -1 || pd_vsel1 != -1) {
		val = REG_TPS6236X_CTRL_PD_EN;
		val |= (pd_vsel0) ? 0 : REG_TPS6236X_CTRL_PD_VSEL0;
		val |= (pd_vsel1) ? 0 : REG_TPS6236X_CTRL_PD_VSEL1;
		ret = omap_vc_bypass_send_i2c_msg(voltdm,
					voltdm->pmic->i2c_slave_addr,
					REG_TPS6236X_CTRL, val);
		if (ret)
			goto out;
	}

	/* Enable thermal shutdown - 0 is enable :) */
	ret = omap_vc_bypass_send_i2c_msg(voltdm,
				voltdm->pmic->i2c_slave_addr,
				REG_TPS6236X_TEMP, 0x0);
	if (ret)
		goto out;

out:
	if (ret)
		pr_err("%s: Error enabling TPS(%d)\n", __func__, ret);

	return ret;
}

static __initdata struct omap_pmic_map omap_tps_map[] = {
	{
		.name = "mpu",
		.cpu = PMIC_CPU_OMAP4460,
//		.omap_chip = OMAP_CHIP_INIT(CHIP_IS_OMAP44XX),
		.pmic_data = &omap4_mpu_pmic,
		.special_action = omap4_twl_tps62361_enable,
	},
	/* Terminator */
	{ .name = NULL, .pmic_data = NULL},
};

/**
 * omap_tps6236x_gpio_no_reset_wa() - prevent GPIO banks reset on init
 * @gpio_vsel0:	If using GPIO to control VSEL0, provide gpio number, else -1
 * @gpio_vsel1:	If using GPIO to control VSEL1, provide gpio number, else -1
 * @gpio_bank_width:	GPIO bank width
 *
 * On some platforms like OMAP4460, GPIO lines are used for controlling
 * the TPS modes VSEL0/VSEL1, these GPIO lines should not be reset during init
 * as reset will cause the TPS voltage to drop to 0.9 V. Hence configure
 * corresponding OMAP HWMODs as INIT_NO_RESET.
 *
 * This function should be called not latter then pure_initcall because
 * INIT_NO_RESET property of the hwmod should be configured before calling
 * its setup() function which is done from omap_hwmod_setup_all() routine
 * execution on core_initcall level.
 */
void __init omap_tps6236x_gpio_no_reset_wa(int gpio_vsel0, int gpio_vsel1,
					       int gpio_bank_width)
{
	int gpio_bank;
	struct omap_hwmod *oh;
	char hw_name[7];

	if (gpio_vsel0 != -1) {

		gpio_bank = (gpio_vsel0 / gpio_bank_width) + 1;

		snprintf(hw_name, sizeof(hw_name), "gpio%d", gpio_bank);

		pr_debug("%s: WA: set INIT_NO_RESET flag for the %s hwmod\n",
			 __func__, hw_name);

		oh = omap_hwmod_lookup(hw_name);
		if (oh)
			omap_hwmod_no_setup_reset(oh);
		else
			pr_warn("%s: %s hwmod not found\n", __func__, hw_name);
	}

	if (gpio_vsel1 != -1) {

		gpio_bank = (gpio_vsel1 / gpio_bank_width) + 1;

		snprintf(hw_name, sizeof(hw_name), "gpio%d", gpio_bank);

		pr_debug("%s: WA: set INIT_NO_RESET flag for the %s hwmod\n",
			 __func__, hw_name);

		oh = omap_hwmod_lookup(hw_name);
		if (oh)
			omap_hwmod_no_setup_reset(oh);
		else
			pr_warn("%s: %s hwmod not found\n", __func__, hw_name);
	}
}

int __init omap_tps6236x_init(void)
{
	struct omap_pmic_map *map;

	/* Without registers, I wont proceed */
	if (default_reg == -1)
		return -EINVAL;

	map = omap_tps_map;

	/* setup all the pmic's voltage addresses to the default one */
	while (map->name) {
		map->pmic_data->volt_reg_addr = default_reg;
		map->pmic_data->cmd_reg_addr = default_reg;
		map++;
	}

	return omap_pmic_register_data(omap_tps_map);
}

/**
 * omap_tps6236x_board_setup() - provide the board config for TPS connect
 * @use_62361:	Do we use TPS62361 variant?
 * @gpio_vsel0:	If using GPIO to control VSEL0, provide gpio number, else -1
 * @gpio_vsel1:	If using GPIO to control VSEL1, provide gpio number, else -1
 * @pull0:	If using GPIO, provide mux mode OMAP_PIN_OFF_OUTPUT_[HIGH|LOW]
 *		else provide any internal pull required, -1 if unused.
 * @pull1:	If using GPIO, provide mux mode OMAP_PIN_OFF_OUTPUT_[HIGH|LOW]
 *		else provide any internal pull required, -1 if unused.
 *
 * TPS6236x variants of PMIC can be hooked in numerous combinations on to the
 * board. Some platforms can choose to hardwire and save on a GPIO for other
 * uses, while others may hook a single line for GPIO control and may ground
 * the other line. support these configurations.
 *
 * WARNING: for platforms using GPIO, be careful to provide MUX setting
 * considering OFF mode configuration as well.
 */
int __init omap_tps6236x_board_setup(bool use_62361, int gpio_vsel0,
		int gpio_vsel1, int pull0, int pull1)
{
	int r;

	r = _bd_setup("tps6236x_vsel0", gpio_vsel0, &pull0, &pd_vsel0);
	if (r)
		goto out;
	r = _bd_setup("tps6236x_vsel1", gpio_vsel1, &pull1, &pd_vsel1);
	if (r) {
		if (gpio_vsel0 != -1)
			gpio_free(gpio_vsel0);
		goto out;
	}

	default_reg = ((pull1 & 0x1) << 1) | (pull0 & 0x1);

	if (!use_62361) {
		omap4_mpu_pmic.vddmin = MIN_VOLTAGE_TPS62360_62_UV;
		omap4_mpu_pmic.vddmax = MAX_VOLTAGE_TPS62360_62_UV;
	}
out:
	return r;
}
