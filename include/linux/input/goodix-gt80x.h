/*
 *    goodix-gt80x.h : 17/02/2011
 *    g.revaillot, revaillot@archos.com
 */

#ifndef _LINUX_GOODIX_GT80X_H
#define _LINUX_GOODIX_GT80X_H

#define GOODIX_GT801_NAME	"goodix-gt801"
#define GOODIX_GT801_ADDR	(0x55)

enum {
	GOODIX_GT80X_ORIENTATION_NORMAL	= 0,
	GOODIX_GT80X_ORIENTATION_INV_X	= 1,
	GOODIX_GT80X_ORIENTATION_INV_YX	= 2,
	GOODIX_GT80X_ORIENTATION_INV_Y	= 3,
};

// software orientation flags.
enum {
	GOODIX_GT80X_FLAGS_INV_X	= (1 << 0),
	GOODIX_GT80X_FLAGS_INV_Y	= (1 << 1),
	GOODIX_GT80X_FLAGS_XY_SWAP	= (1 << 2),
};

struct goodix_gt80x_platform_data {
	int irq;

	int flags;
	int orientation;
	int init_version;

	void (*set_power)(int on_off);
	void (*set_shutdown)(int on_off);
};

#endif

