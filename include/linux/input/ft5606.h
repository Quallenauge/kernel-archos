/*
 *    ft5606.h : 31/05/2012
 *    g.revaillot, revaillot@archos.com
 */

#ifndef __LINUX_FT5606_H__
#define __LINUX_FT5606_H__

#define FOCALTECH_FT5606_NAME "ft5606"
#define FOCALTECH_FT5606_ADDR (0x38)

struct focaltech_ft5606_platform_data {
	int irq;
	int flags;
	const char * regulator;

	void (*reset)(int);
	int (*get_irq_level)(void);
	int (*demux_i2c)(int);
};

#endif

