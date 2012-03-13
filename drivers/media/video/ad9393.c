/*
 *  ad9393 - ad9393 Low Power HDMI Display Interface
 *
 * Copyright (C) 2012 Francois Caron <caron@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <media/ad9393.h>

MODULE_DESCRIPTION("Analog Devices AD9393 Low Power HDMI Display Interface");
MODULE_AUTHOR("Francois CARON");
MODULE_LICENSE("GPL");

enum connection_state {
	HDMI_DISCONNECT = 1,
	HDMI_CONNECT = 2,
};

struct ad9393_config current_config;

static struct hdmi_data {
	struct i2c_client *pdev;
	struct regulator *vcc_reg, *v1_8_reg;

	struct ad9393_platform_data* pdata; 
	enum connection_state connection_state;

} hdmi;

static inline int ad9393_read(int reg)
{
	if (hdmi.pdev)
		return i2c_smbus_read_byte_data(hdmi.pdev, reg);
	else
		return -1;
}

static inline int ad9393_write(int reg, int value)
{
	if (hdmi.pdev)
		return i2c_smbus_write_byte_data(hdmi.pdev, reg, value);
	else
		return -1;
}

static int write_config(struct ad9393_config *config) 
{
	int i, r;
	if (config == NULL) return -EINVAL;
	for (i = 0; i < config->nb_reg; i++) {
		r = ad9393_write(config->reg[i].addr, config->reg[i].value);
		if (r) {
			printk(KERN_ERR "Error writing AD9393 value 0x%x in reg 0x%x\n", config->reg[i].value, config->reg[i].addr);
			return r;
		}
	}
	return 0;
}

static const struct ad9393_config default_ad9393_config = {
  .nb_reg = 56,
  .reg[0] =  { .addr = 0x01, .mask = 0xFF, .value = 1683 >> 4,},
  .reg[1] =  { .addr = 0x02, .mask = 0xFF, .value = (1683 & 0xF) << 4,},
  .reg[2] =  { .addr = 0x03, .mask = 0xFF, .value = (1 << 6) | (1 << 3) | (0 << 2),},
  .reg[3] =  { .addr = 0x11, .mask = 0xFF, .value = 0x0,},
  .reg[4] =  { .addr = 0x12, .mask = 0xFF, .value = (1 << 7) | (1 << 5),},
  .reg[5] =  { .addr = 0x22, .mask = 0xFF, .value = 4,},
  .reg[6] =  { .addr = 0x23, .mask = 0xFF, .value = 32,},
  .reg[7] =  { .addr = 0x24, .mask = 0xFF, .value = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4),},
  .reg[8] =  { .addr = 0x25, .mask = 0xFF, .value = (1 << 6) | (3 << 4) | (1 << 1),},
  .reg[9] =  { .addr = 0x26, .mask = 0xFF, .value = (1 << 3),},
  .reg[10] =  { .addr = 0x27, .mask = 0xFF, .value = (1 << 7),},
  .reg[11] =  { .addr = 0x28, .mask = 0xFF, .value = (24 << 2) | (1 << 0),},
  .reg[12] =  { .addr = 0x29, .mask = 0xFF, .value = 4,},
  .reg[13] =  { .addr = 0x2A, .mask = 0xFF, .value = 1280 >> 8,},
  .reg[14] =  { .addr = 0x2B, .mask = 0xFF, .value = (1280 & 0xFF),},
  .reg[15] =  { .addr = 0x2C, .mask = 0xFF, .value = 720 >> 8,},
  .reg[16] =  { .addr = 0x2D, .mask = 0xFF, .value = (720 & 0xFF),},
  .reg[17] =  { .addr = 0x2E, .mask = 0xFF, .value = 24 << 0,},
  .reg[18] =  { .addr = 0x34, .mask = 0xFF, .value = 0,},
  .reg[19] =  { .addr = 0x35, .mask = 0xFF, .value = (1 << 5) | (3154 >> 8),},
  .reg[20] =  { .addr = 0x36, .mask = 0xFF, .value = (3154 & 0xFF) & 0x1F,},
  .reg[21] =  { .addr = 0x37, .mask = 0xFF, .value = (2048 >> 8),},
  .reg[22] =  { .addr = 0x38, .mask = 0xFF, .value = (2048 & 0xFF) & 0x1F,},
  .reg[23] =  { .addr = 0x39, .mask = 0xFF, .value = (0 >> 8),},
  .reg[24] =  { .addr = 0x3A, .mask = 0xFF, .value = (0 & 0xFF) & 0x1F,},
  .reg[25] =  { .addr = 0x3B, .mask = 0xFF, .value = (-1577 >> 8) & 0x1F,},
  .reg[26] =  { .addr = 0x3C, .mask = 0xFF, .value = (-1577 & 0xFF),},
  .reg[27] =  { .addr = 0x3D, .mask = 0xFF, .value = (-940 >> 8) & 0x1F,},
  .reg[28] =  { .addr = 0x3E, .mask = 0xFF, .value = (-940 & 0xFF),},
  .reg[29] =  { .addr = 0x3F, .mask = 0xFF, .value = (2048 >> 8) & 0x1F,},
  .reg[30] =  { .addr = 0x40, .mask = 0xFF, .value = (2048 & 0xFF),},
  .reg[31] =  { .addr = 0x41, .mask = 0xFF, .value = (-375 >> 8) & 0x1F,},
  .reg[32] =  { .addr = 0x42, .mask = 0xFF, .value = (-375 & 0xFF),},
  .reg[33] =  { .addr = 0x43, .mask = 0xFF, .value = (658 >> 8) & 0x1F,},
  .reg[34] =  { .addr = 0x44, .mask = 0xFF, .value = (658 & 0xFF),},
  .reg[35] =  { .addr = 0x45, .mask = 0xFF, .value = (0 >> 8) & 0x1F,},
  .reg[36] =  { .addr = 0x46, .mask = 0xFF, .value = (0 & 0xFF),},
  .reg[37] =  { .addr = 0x47, .mask = 0xFF, .value = (2048 >> 8) & 0x1F,},
  .reg[38] =  { .addr = 0x48, .mask = 0xFF, .value = (2048 & 0xFF),},
  .reg[39] =  { .addr = 0x49, .mask = 0xFF, .value = (3719 >> 8) & 0x1F,},
  .reg[40] =  { .addr = 0x4A, .mask = 0xFF, .value = (3719 & 0xFF),},
  .reg[41] =  { .addr = 0x4B, .mask = 0xFF, .value = (-1859 >> 8) & 0x1F,},
  .reg[42] =  { .addr = 0x4C, .mask = 0xFF, .value = (-1859 & 0xFF),},
  .reg[43] =  { .addr = 0x4D, .mask = 0xFF, .value = 0x3C,},
  .reg[44] =  { .addr = 0x4E, .mask = 0xFF, .value = 0x3C,},
  .reg[45] =  { .addr = 0x4F, .mask = 0xFF, .value = 0x13,},
  .reg[46] =  { .addr = 0x50, .mask = 0xFF, .value = 0x90,},
  .reg[47] =  { .addr = 0x51, .mask = 0xFF, .value = 0x40,},
  .reg[48] =  { .addr = 0x52, .mask = 0xFF, .value = 0x01,},
  .reg[49] =  { .addr = 0x53, .mask = 0xFF, .value = 0x3B,},
  .reg[50] =  { .addr = 0x54, .mask = 0xFF, .value = 0x00,},
  .reg[51] =  { .addr = 0x55, .mask = 0xFF, .value = 0x01,},
  .reg[52] =  { .addr = 0x56, .mask = 0xFF, .value = 0x0F,},
  .reg[53] =  { .addr = 0x57, .mask = 0xFF, .value = 0x00,},
  .reg[54] =  { .addr = 0x58, .mask = 0xFF, .value = 0x00,},
  .reg[55] =  { .addr = 0x59, .mask = 0xFF, .value = 0x00,},
};

static const struct ad9393_config config2 = {
  .nb_reg = 21,
  
#if 0
  This part is still missing in the config 2 
  
  By the way it is just about color conversion on imge output so not useful for the moment
  
# Adjust CSC Coefficients
AD9393['CSC Coef A1'].setValue(3135) # 0xC3F
AD9393['CSC Coef A2'].setValue(2111) # 0x83F
AD9393['CSC Coef A4'].setValue(-1405) # 0x-57D
AD9393['CSC Coef B2'].setValue(2175) # 0x87F
AD9393['CSC Coef B3'].setValue(1151) # 0x47F
AD9393['CSC Coef C1'].setValue(191) # 0xBF
AD9393['CSC Coef C2'].setValue(2111) # 0x83F
AD9393['CSC Coef C4'].setValue(-1792) # 0x-700
#endif
   //fixed register
  .reg[0] =  { .addr = 0x11, .mask = 0xFF, .value = 0x3,},
  .reg[1] =  { .addr = 0x4D, .mask = 0xFF, .value = 0x3B,},
  .reg[2] =  { .addr = 0x4E, .mask = 0xFF, .value = 0x6D,},
  .reg[3] =  { .addr = 0x4F, .mask = 0xFF, .value = 0x54,},
  .reg[4] =  { .addr = 0x50, .mask = 0xFF, .value = 0x90,},
  .reg[5] =  { .addr = 0x53, .mask = 0xFF, .value = 0x3F,},
  
  //variable register
  .reg[6] =  { .addr = 0x01, .mask = 0xFF, .value = 0x00,},
  .reg[7] =  { .addr = 0x02, .mask = 0xF0, .value = 0x04 << 4,},  
  .reg[8] =  { .addr = 0x03, .mask = (0x7 << 3) | (1 << 2), .value = (2 << 3) | (1 << 2),},
  .reg[9] =  { .addr = 0x25, .mask = (0x3 << 4), .value = (2 << 4),},
  .reg[10] =  { .addr = 0x58, .mask = (0x1 << 7) | (0x7 << 4), .value = (1 << 7) | (1 << 4),},
  .reg[11] =  { .addr = 0x34, .mask = (0x1 << 5) | (0x1 << 1), .value = (0x1 << 5) | (0x1 << 1),},
  .reg[12] =  { .addr = 0x59, .mask = (0x1 << 5), .value = (0x1 << 5),},
  .reg[13] =  { .addr = 0x58, .mask = (0x7 << 0), .value = (0x1 << 0),},
  .reg[14] =  { .addr = 0x59, .mask = (0x1 << 6), .value = (0x0 << 6),},
  .reg[15] =  { .addr = 0x25, .mask = (0x3 << 2), .value = (0x0 << 2),},
  .reg[16] =  { .addr = 0x7B, .mask = 0xFF, .value = (140625 >> 12),},
  .reg[17] =  { .addr = 0x7C, .mask = 0xFF, .value = (140625 >> 4) & 0xFF,},
  .reg[18] =  { .addr = 0x7D, .mask = 0xFF, .value = ((140625  & 0xF) << 4) | ((11648 >> 16) & 0xF) ,},
  .reg[19] =  { .addr = 0x7E, .mask = 0xFF, .value = (11648 >> 8) & 0xFF,},
  .reg[20] =  { .addr = 0x7F, .mask = 0xFF, .value = (11648 & 0xFF),},
};

static int load_default_control_config(struct ad9393_config *config) 
{
	if (config == NULL) return -1;

	memcpy(config, &default_ad9393_config, sizeof(default_ad9393_config));

	return 0;
}

static int load_control_config(struct ad9393_config *dst, const struct ad9393_config *src)
{
	int i;
	if (!dst || !src) return -1;
	memset(dst->reg, 0, sizeof(dst->reg));
	dst->nb_reg = src->nb_reg;
	for (i = 0; i < src->nb_reg; i++) {
		dst->reg[i].addr = src->reg[i].addr;
		dst->reg[i].value = ad9393_read(src->reg[i].addr);
		if (dst->reg[i].value == -1) return -1;
		dst->reg[i].value = (dst->reg[i].value & ~src->reg[i].mask) | src->reg[i].value;
	}
	return 0;
}

int __init ad9393_set_platform_data(const struct ad9393_platform_data *data)
{
	if (hdmi.pdata)
		return -EBUSY;
	if (!data)
		return -EINVAL;

	hdmi.pdata = kmemdup(data, sizeof(*data), GFP_KERNEL);
	if (!hdmi.pdata)
		return -ENOMEM;

	return 0;
}

int hdmi_get_current_5V(void)
{
	return gpio_get_value(hdmi.pdata->detect_5V);
}

static irqreturn_t sink5V_irq_handler(int irq, void *ptr)
{
	int sink5V = hdmi_get_current_5V();
	pr_info("sink5V %d\n", sink5V);
	return IRQ_HANDLED;
}

static int ad9393_probe(struct i2c_client *pdev, 
		const struct i2c_device_id *id)
{
	int r = 0;

	hdmi.pdev = pdev;
	hdmi.connection_state = HDMI_DISCONNECT;

	hdmi.pdata = (struct ad9393_platform_data*)pdev->dev.platform_data;

	if (IS_ERR_OR_NULL(hdmi.pdata)) {
		r = PTR_ERR(hdmi.pdata);
		printk(KERN_ERR "missing ad9393 platform data: %d\n", r);
		goto out;
	}
	
	r = request_irq(gpio_to_irq(hdmi.pdata->detect_5V), sink5V_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"sink_5V", NULL);
	if (r < 0) {
		pr_err("ad9393: detect_5V %d failed\n",
			gpio_to_irq(hdmi.pdata->detect_5V));
		goto out;
	}
	
	hdmi.vcc_reg = regulator_get(&hdmi.pdev->dev, "hdmi_in_vcc");
	if (IS_ERR_OR_NULL(hdmi.vcc_reg)) {
		printk(KERN_ERR "Failed to get \"hdmi_in_vcc\" regulator\n");
		r = PTR_ERR(hdmi.vcc_reg) ? : -ENODEV;
		hdmi.vcc_reg = NULL;
		//goto reg_err; FixMe: Currently it continues without power supply
	}

	hdmi.v1_8_reg = regulator_get(&hdmi.pdev->dev, "hdmi_in_1_8");
	if (IS_ERR_OR_NULL(hdmi.v1_8_reg)) {
		printk(KERN_ERR "Failed to get \"hdmi_in_1_8\" regulator\n");
		r = PTR_ERR(hdmi.v1_8_reg) ? : -ENODEV;
		hdmi.v1_8_reg = NULL;
		//goto reg_err; FixMe: Currently it continues without power supply
	}

	gpio_direction_output(hdmi.pdata->power_enable, 1);

	msleep(200);

	load_default_control_config(&current_config);

	r = write_config(&current_config);

	load_control_config(&current_config, &config2);

	r = write_config(&current_config);

	return 0;
out:
	return r;
}

static int __exit ad9393_remove(struct i2c_client *client)
{
	gpio_set_value(hdmi.pdata->power_enable,0);
	if (hdmi.vcc_reg) regulator_put(hdmi.vcc_reg);
	if (hdmi.v1_8_reg) regulator_put(hdmi.v1_8_reg);
	return 0;
}

static const struct i2c_device_id ad9393_id[] = {
	{"ad9393_hdmi", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ad9393_id);

static struct i2c_driver ad9393_driver = {

	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ad9393_hdmi",
	},
	.probe		= ad9393_probe,
	.remove		= __exit_p(ad9393_remove),
	.id_table	= ad9393_id,
};

static __init int init_ad9393(void)
{
	int ret;

	ret = i2c_add_driver(&ad9393_driver);
	if (ret) {
		printk("ad9393: I2C driver registration failed, module not inserted.\n");
		return ret;
	}
	return 0;
}

static __exit void exit_ad9393(void)
{
	i2c_del_driver(&ad9393_driver);
}

module_init(init_ad9393);
module_exit(exit_ad9393);
