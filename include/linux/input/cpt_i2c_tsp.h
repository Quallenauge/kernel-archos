/*
 *    cpt_i2c_tsp.h : 09/12/2011
 *    g.revaillot, revaillot@archos.com
 */

#ifndef _LINUX_CPT_I2C_TSP_H
#define _LINUX_CPT_I2C_TSP_H

#define CPT_I2C_TSP_NAME	"cpt_i2c_tsp"
#define CPT_I2C_TSP_ADDR	(0x7f)
#define CPT_I2C_TSP_CURSOR_ADDR	(0x01)

struct cpt_i2c_tsp_platform_data {
	int irq;
	void (*reset)(int);

	int x_max;
	int y_max;

	int x_scale;
	int y_scale;
	int x_offset;
	int y_offset;
};

#endif

