/*
 *    cypress-tma340.h : 17/03/2011
 *    g.revaillot, revaillot@archos.com
 */

#ifndef _LINUX_CYPRESS_TMA340_H
#define _LINUX_CYPRESS_TMA340_H

#define CYPRESS_TMA340_NAME	"cypress-tma340"
#define CYPRESS_TMA340_ADDR	(0x24)

enum {
	CYPRESS_TMA340_FLAGS_INV_X	= 1 << 0,
	CYPRESS_TMA340_FLAGS_INV_Y	= 1 << 1,
	CYPRESS_TMA340_FLAGS_XY_SWAP	= 1 << 2,
};

struct cypress_tma340_platform_data {
	int irq;
	int flags;
};

#endif

