/*
 *    pixcir_i2c_tsp.h : 15/04/2011
 *    g.revaillot, revaillot@archos.com
 */

#ifndef _LINUX_PIXCIR_I2C_TSP_H
#define _LINUX_PIXCIR_I2C_TSP_H

#define PIXCIR_NAME	"pixcir_i2c_tsp"
#define PIXCIR_ADDR		(0x5c)
#define PIXCIR_BOOT_ADDR	(0x5d)

enum {
	PIXCIR_FLAGS_INV_X	= 1 << 0,
	PIXCIR_FLAGS_INV_Y	= 1 << 1,
	PIXCIR_FLAGS_XY_SWAP	= 1 << 2,
};

struct pixcir_platform_data {
	int irq;
	int flags;

	// min/max, as should normally
	// reported by chip.
	int x_min;
	int x_max;
	int y_min;
	int y_max;

	// scales and offset, need to adapt to lcd.
	int x_scale;
	int y_scale;
	int x_offset;
	int y_offset;
};

#endif

