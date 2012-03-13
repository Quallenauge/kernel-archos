/*
 * twl6030_pwm.c
 * Driver for PHOENIX (TWL6030) Pulse Width Modulator
 *
 * Copyright (C) 2010 Texas Instruments
 * Author: Hemanth V <hemanthv@ti.com>
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
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>
#include <linux/slab.h>

#define LED_PWM_CTRL1	0xF4
#define LED_PWM_CTRL2	0xF5

/* Max value for CTRL1 register */
#define PWM_CTRL1_MAX	255

/* Pull down disable */
#define PWM_CTRL2_DIS_PD	(1 << 6)

/* LED Current control */ 
#define PWM_CTRL2_CURR_02	(2 << 4) /* 2.5 milli Amps */
#define PWM_CTRL2_CURR_03	(3 << 4) /*   5 milli Amps */

/* LED supply source */
#define PWM_CTRL2_SRC_VBUS	(0 << 2)
#define PWM_CTRL2_SRC_VAC	(1 << 2)
#define PWM_CTRL2_SRC_EXT	(2 << 2)

/* LED modes */
#define PWM_CTRL2_MODE_HW	(0 << 0)
#define PWM_CTRL2_MODE_SW	(1 << 0)
#define PWM_CTRL2_MODE_DIS	(2 << 0)

#define PWM_CTRL2_MODE_MASK	0x3

/* Custom PWM control */
#define TOGGLE3_PWM1_RESET	(1 << 0)
#define TOGGLE3_PWM1_SET	(1 << 1)
#define TOGGLE3_PWM1_CLK_EN	(1 << 2)
#define TOGGLE3_PWM2_RESET	(1 << 3)
#define TOGGLE3_PWM2_SET	(1 << 4)
#define TOGGLE3_PWM2_CLK_EN	(1 << 5)

struct pwm_device {
	const char *label;
	unsigned int pwm_id;
};

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	u8 duty_cycle;
	int ret = 0;

	if (pwm == NULL || period_ns == 0 || duty_ns > period_ns)
		return -EINVAL;

	pr_debug("%s : %d : %s : %d/%d\n", __FUNCTION__,
			pwm->pwm_id, pwm->label,
			duty_ns, period_ns);

	switch (pwm->pwm_id) {
		case 3:
			/* Custom PWM2 */
			duty_cycle = 64 - (duty_ns * 64) / period_ns;
			ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, duty_cycle | (1 << 7), TWL6030_PWM2ON);
			if (ret < 0)
				goto fail;

			ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, 64, TWL6030_PWM2OFF);
			break;
		case 2:
			/* Custom PWM1 */
			duty_cycle = 64 - (duty_ns * 64) / period_ns;
			ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, duty_cycle | (1 << 7), TWL6030_PWM1ON);
			if (ret < 0)
				goto fail;

			ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, 64, TWL6030_PWM1OFF);
			break;
		case 1:
			/* LED PWM */
			duty_cycle = (duty_ns * PWM_CTRL1_MAX) / period_ns;

			ret = twl_i2c_write_u8(TWL6030_MODULE_ID1,
					duty_cycle,
					LED_PWM_CTRL1);

			break;
	}

	if (ret < 0)
		goto fail;

	return 0;
fail:
	pr_err("%s: Failed to configure PWM, Error %d\n",
			pwm->label, ret);
	return ret;
}
EXPORT_SYMBOL(pwm_config);

int pwm_enable(struct pwm_device *pwm)
{
	u8 val = 0;
	int ret = 0;

	switch (pwm->pwm_id) {
		case 3:
			/* Custom PWM2 */
			ret = twl_i2c_read_u8(TWL6030_MODULE_ID1,
					&val,
					TWL6030_TOGGLE3);

			if (ret < 0)
				goto fail;

			ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, 
					(val | TOGGLE3_PWM2_SET | TOGGLE3_PWM2_CLK_EN),
					TWL6030_TOGGLE3);
			break;
		case 2:
			/* Custom PWM1 */
			ret = twl_i2c_read_u8(TWL6030_MODULE_ID1,
					&val,
					TWL6030_TOGGLE3);

			if (ret < 0)
				goto fail;

			ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, 
					(val | TOGGLE3_PWM1_SET | TOGGLE3_PWM1_CLK_EN),
					TWL6030_TOGGLE3);
			break;
		case 1:
			/* LED PWM */
			ret = twl_i2c_read_u8(TWL6030_MODULE_ID1,
					&val,
					LED_PWM_CTRL2);
			if (ret < 0)
				goto fail;

			/*  LED PWM  */
			/* Change mode to software control */
			val &= ~PWM_CTRL2_MODE_MASK;
			val |= PWM_CTRL2_MODE_SW;

			ret = twl_i2c_write_u8(TWL6030_MODULE_ID1,
					val,
					LED_PWM_CTRL2);
			if (ret < 0)
				goto fail;

			ret = twl_i2c_read_u8(TWL6030_MODULE_ID1,
					&val,
					LED_PWM_CTRL2);
			break;
	}

	if (ret < 0)
		goto fail;

	return 0;

fail:
	pr_err("%s: Failed to enable PWM, Error %d\n", pwm->label, ret);
	return ret;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	u8 val = 0;
	int ret = 0;

	switch (pwm->pwm_id) {
		case 3:
			/* Custom PWM2 */
			ret = twl_i2c_read_u8(TWL6030_MODULE_ID1,
					&val,
					TWL6030_TOGGLE3);
			if (ret < 0)
				goto fail;

			ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, 
					(val | TOGGLE3_PWM2_RESET) & ~TOGGLE3_PWM2_CLK_EN,
					TWL6030_TOGGLE3);
			break;
		case 2:
			/* Custom PWM1 */
			ret = twl_i2c_read_u8(TWL6030_MODULE_ID1,
					&val,
					TWL6030_TOGGLE3);
			if (ret < 0)
				goto fail;

			ret = twl_i2c_write_u8(TWL6030_MODULE_ID1, 
					(val | TOGGLE3_PWM1_RESET) & ~TOGGLE3_PWM1_CLK_EN,
					TWL6030_TOGGLE3);
			break;
		case 1:
			/*  LED PWM  */
			ret = twl_i2c_read_u8(TWL6030_MODULE_ID1,
					&val,
					LED_PWM_CTRL2);
			if (ret < 0)
				goto fail;

			val &= ~PWM_CTRL2_MODE_MASK;
			val |= PWM_CTRL2_MODE_HW;

			ret = twl_i2c_write_u8(TWL6030_MODULE_ID1,
					val,
					LED_PWM_CTRL2);
			if (ret < 0)
				goto fail;

			break;
	}

	if (ret < 0)
		goto fail;

	return;

fail:
	pr_err("%s: Failed to disable PWM, Error %d\n",
			pwm->label, ret);
	return;
}
EXPORT_SYMBOL(pwm_disable);

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	u8 val;
	int ret;
	struct pwm_device *pwm;

	pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
	if (pwm == NULL) {
		pr_err("%s: failed to allocate memory\n", label);
		return NULL;
	}

	pwm->label = label;
	pwm->pwm_id = pwm_id;

	switch (pwm_id) {
		case 3:
		case 2:
			/* Custom PWM2 & Custom PWM1 */
			/* No need for specific init */
			break;

		case 1:
			/* LED PWM */
			/* Configure PWM */
			val = PWM_CTRL2_DIS_PD | PWM_CTRL2_CURR_03 |
				PWM_CTRL2_SRC_VBUS | PWM_CTRL2_MODE_HW;

			ret = twl_i2c_write_u8(TWL6030_MODULE_ID1,
					val,
					LED_PWM_CTRL2);

			if (ret < 0)
				goto fail;

			break;
	}

	return pwm;

fail:
	pr_err("%s: Failed to configure PWM, Error %d\n", pwm->label, ret);

	kfree(pwm);
	return NULL;
}
EXPORT_SYMBOL(pwm_request);

void pwm_free(struct pwm_device *pwm)
{
	pwm_disable(pwm);
	kfree(pwm);
}
EXPORT_SYMBOL(pwm_free);
