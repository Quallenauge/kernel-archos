/*
 *    tr16c0-i2c.h : 23/06/2011
 *    g.revaillot, revaillot@archos.com
 */

#ifndef _LINUX_TR16C0_H
#define _LINUX_TR16C0_H

#define TR16C0_NAME	"tr16c0_i2c_tsp"
#define TR16C0_ADDR		(0x5c)

enum {
	TR16C0_FLAGS_INV_X	= 1 << 0,
	TR16C0_FLAGS_INV_Y	= 1 << 1,
	TR16C0_FLAGS_XY_SWAP	= 1 << 2,
	TR16C0_FLAGS_TSP_V2	= 1 << 3,
	TR16C0_FLAGS_TSP_USE_RESET = 1 << 4,
	TR16C0_FLAGS_TSP_NO_POLL = 1 << 5,
};

struct tr16c0_dead_areas {
	struct area {
		int x;
		int y;
		int w;
		int h;
	} areas[16];
	int num;
};

struct tr16c0_fw_version {
	int major;
	int minor;
	int release;
	int config_hash;
};

struct tr16c0_platform_data {
	int irq;
	int flags;
	int x_max;
	int y_max;

	void (*reset)(int);
	int (*get_irq_level)(void);
	int (*demux_i2c)(int);
	const char * regulator;

	struct tr16c0_dead_areas * dead_areas;
	struct tr16c0_fw_version latest_fw_version;
};

#endif

