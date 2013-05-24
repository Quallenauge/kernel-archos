/*
 *    byd_i2c_tsp.h : 5/05/2012
 *    y.robic, robic@archos.com
 */

#ifndef _LINUX_BYD_I2C_TSP_H
#define _LINUX_BYD_I2C_TSP_H

#define BYD_NAME	"byd_i2c_tsp"
#define BYD_ADDR		(0x5c)
#define BYD_BOOT_ADDR	(0x5d)

enum {
	BYD_FLAGS_INV_X	= 1 << 0,
	BYD_FLAGS_INV_Y	= 1 << 1,
	BYD_FLAGS_XY_SWAP	= 1 << 2,
};

struct byd_platform_data {
	int irq;
	int flags;
	const char * regulator;

	void (*reset)(int);
	int (*get_irq_level)(void);

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

