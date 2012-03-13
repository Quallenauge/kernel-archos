#ifndef _LINUX_AD9393_H
#define _LINUX_AD9393_H

#include <linux/err.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

struct ad9393_config {
      int nb_reg;
      struct ad9393_reg_config {
		u8 addr;
		u8 value;
		u8 mask;
      } reg[60];
};

struct ad9393_platform_data {
	int power_enable;
	int detect_5V;
};

#endif //_LINUX_AD9393_H
