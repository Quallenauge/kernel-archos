/*
 *    goodix-gt8105.h : 16/05/2012
 *    g.revaillot, revaillot@archos.com
 *
 *    Goodix GT8105 Solution Driver (GT8105*2 + GTM802)
 *
 */

#ifndef _LINUX_GOODIX_GT8105_H
#define _LINUX_GOODIX_GT8105_H

#define GOODIX_GT8105_NAME	"goodix-gt8105"
#define GOODIX_GT8105_ADDR	(0x55)

struct goodix_gt8105_platform_data {
	int irq;
	int flags;
	const char * regulator;

	void (*reset)(int);
	int (*get_irq_level)(void);
	int (*demux_i2c)(int);
};

#endif


