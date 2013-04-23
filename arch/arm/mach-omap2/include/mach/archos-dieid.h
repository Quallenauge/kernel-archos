#ifndef _ARCH_ARCHOS_DIEID_H
#define _ARCH_ARCHOS_DIEID_H

/*
 *    archos-dieid.h : 19/11/2009
 *    g.revaillot, revaillot@archos.com
 */

#define CONTROL_DIE_ID_REG	0x4830A218
#define CONTROL_PROD_ID_REG	0x4830A208
#define CONTROL_IDCODE_REG	0x4830A204

void get_dieid(u32 *die_id);
void get_prodid(u32 *prod_id);

#endif
