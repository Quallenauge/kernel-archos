/*
 *   Definitions for MMA8453Q Accelerometer chip
 *
 *   Copyright (c) by Jean-Christophe Rona <rona@archos.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef MMA8453Q_H
#define MMA8453Q_H

#include <linux/ioctl.h>

/*
 * MMA8453Q registers
 */

#define REG_STATUS			0x00		/* RO */
#define REG_OUT_X_MSB			0x01		/* RO */
#define REG_OUT_X_LSB			0x02		/* RO */
#define REG_OUT_Y_MSB			0x03		/* RO */
#define REG_OUT_Y_LSB			0x04		/* RO */
#define REG_OUT_Z_MSB			0x05		/* RO */
#define REG_OUT_Z_LSB			0x06		/* RO */
/* Reserved				0x07 - 0x08 */
#define REG_SYSMOD			0x0B		/* RO */
#define REG_INT_SOURCE			0x0C		/* RO */
#define REG_WHO_AM_I			0x0D		/* RO */
#define REG_XYZ_DATA_CFG		0x0E		/* RW */
#define REG_HP_FILTER_CUTOFF		0x0F		/* RW */
#define REG_PL_STATUS			0x10		/* RO */
#define REG_PL_CFG			0x11		/* RW */
#define REG_PL_COUNT			0x12		/* RW */
#define REG_PL_BF_ZCOMP			0x13		/* RW */
#define REG_P_L_THS_REG			0x14		/* RW */
#define REG_FF_MT_CFG			0x15		/* RW */
#define REG_FF_MT_SRC			0x16		/* RO */
#define REG_FF_MT_THS			0x17		/* RW */
#define REG_FF_MT_COUNT			0x18		/* RW */
/* Reserved				0x19 - 0x1C */
#define REG_TRANSIENT_CFG		0x1D		/* RW */
#define REG_TRANSIENT_SRC		0x1E		/* RO */
#define REG_TRANSIENT_THS		0x1F		/* RW */
#define REG_TRANSIENT_COUNT		0x20		/* RW */
#define REG_PULSE_CFG			0x21		/* RW */
#define REG_PULSE_SRC			0x22		/* RO */
#define REG_PULSE_THSX			0x23		/* RW */
#define REG_PULSE_THSY			0x24		/* RW */
#define REG_PULSE_THSZ			0x25		/* RW */
#define REG_PULSE_TMLT			0x26		/* RW */
#define REG_PULSE_LTCY			0x27		/* RW */
#define REG_PULSE_WIND			0x28		/* RW */
#define REG_ASLP_COUNT			0x29		/* RW */
#define REG_CTRL_REG1			0x2A		/* RW */
#define REG_CTRL_REG2			0x2B		/* RW */
#define REG_CTRL_REG3			0x2C		/* RW */
#define REG_CTRL_REG4			0x2D		/* RW */
#define REG_CTRL_REG5			0x2E		/* RW */
#define REG_OFF_X			0x2F		/* RW */
#define REG_OFF_Y			0x30		/* RW */
#define REG_OFF_Z			0x31		/* RW */
/* Reserved				0x40 - 0x7F */
 

/*
 * Mask and bit definitions of MMA8453Q registers
 */

#define MMA8453Q_ID_MASK	0xFF

#define MMA8453Q_SYSMODE_SHIFT		0
#define MMA8453Q_SYSMODE_MASK		(0x3 << MMA8453Q_SYSMODE_SHIFT)
#define MMA8453Q_SYSMODE_STANDBY	(0x0 << MMA8453Q_SYSMODE_SHIFT)
#define MMA8453Q_SYSMODE_WAKE		(0x1 << MMA8453Q_SYSMODE_SHIFT)
#define MMA8453Q_SYSMODE_SLEEP		(0x2 << MMA8453Q_SYSMODE_SHIFT)

#define MMA8453Q_MODE_SHIFT		0
#define MMA8453Q_MODE_MASK		(0x1 << MMA8453Q_MODE_SHIFT)
#define MMA8453Q_MODE_STANDBY		(0x0 << MMA8453Q_MODE_SHIFT)
#define MMA8453Q_MODE_ACTIVE		(0x1 << MMA8453Q_MODE_SHIFT)

#define MMA8453Q_STATUS_XRDY		(0x1 << 0)
#define MMA8453Q_STATUS_YRDY		(0x1 << 1)
#define MMA8453Q_STATUS_ZRDY		(0x1 << 2)
#define MMA8453Q_STATUS_ZYXRDY		(0x1 << 3)

#define MMA8453Q_RANGE_SHIFT		0
#define MMA8453Q_RANGE_MASK		(0x3 << MMA8453Q_RANGE_SHIFT)
#define MMA8453Q_RANGE_2G		(0x0 << MMA8453Q_RANGE_SHIFT)
#define MMA8453Q_RANGE_4G		(0x1 << MMA8453Q_RANGE_SHIFT)
#define MMA8453Q_RANGE_8G		(0x2 << MMA8453Q_RANGE_SHIFT)

#define MMA8453Q_INT_SRC_MASK		(0xBD)
#define MMA8453Q_INT_SRC_NONE		(0x00)
#define MMA8453Q_INT_SRC_DRDY		(0x1 << 0)
#define MMA8453Q_INT_SRC_FF_MT		(0x1 << 2)
#define MMA8453Q_INT_SRC_PULSE		(0x1 << 3)
#define MMA8453Q_INT_SRC_LNDPRT		(0x1 << 4)
#define MMA8453Q_INT_SRC_TRANS		(0x1 << 5)
#define MMA8453Q_INT_SRC_ASLP		(0x1 << 7)

#define MMA8453Q_INT_PP_OD		(0x1 << 0)
#define		MMA8453Q_PUSH_PULL	0
#define		MMA8453Q_OPEN_DRAIN	1
#define MMA8453Q_INT_POLARITY		(0x1 << 1)
#define		MMA8453Q_ACTIVE_LOW	0
#define		MMA8453Q_ACTIVE_HIGH	1

#define MMA8453Q_FAST_READ		0x1 << 1
#define		MMA8453Q_F_READ_OFF	0
#define		MMA8453Q_F_READ_ON	1


struct mma8453q_accel_data {
	short x;
	short y;
	short z;
};


/*
 * IOCTLs
 */

#define MMAIO				0xA1

#define MMA8453Q_IOCTL_S_MODE		_IOW(MMAIO, 0x01, short)
#define MMA8453Q_IOCTL_G_MODE		_IOR(MMAIO, 0x02, short)
#define MMA8453Q_IOCTL_S_POLL_DELAY	_IOW(MMAIO, 0x03, int)
#define MMA8453Q_IOCTL_G_POLL_DELAY	_IOR(MMAIO, 0x04, int)
#define MMA8453Q_IOCTL_S_PULSE_DEBOUNCE	_IOW(MMAIO, 0x05, short)
#define MMA8453Q_IOCTL_G_PULSE_DEBOUNCE	_IOR(MMAIO, 0x06, short)
#define MMA8453Q_IOCTL_S_PULSE_THRSHOLD	_IOW(MMAIO, 0x07, short)
#define MMA8453Q_IOCTL_G_PULSE_THRSHOLD	_IOR(MMAIO, 0x08, short)
#define MMA8453Q_IOCTL_S_PULSE_AXIS	_IOW(MMAIO, 0x09, short)
#define MMA8453Q_IOCTL_G_PULSE_AXIS	_IOR(MMAIO, 0x0A, short)
#define MMA8453Q_IOCTL_S_RANGE		_IOW(MMAIO, 0x0B, short)
#define MMA8453Q_IOCTL_G_RANGE		_IOR(MMAIO, 0x0C, short)
#define MMA8453Q_IOCTL_S_FAST_MODE	_IOW(MMAIO, 0x0D, short)
#define MMA8453Q_IOCTL_G_FAST_MODE	_IOR(MMAIO, 0x0E, short)
#define MMA8453Q_IOCTL_GP_EVENT		_IOW(MMAIO, 0x0F, int)
#define MMA8453Q_IOCTL_G_ACCEL_DATA	_IOR(MMAIO, 0x10, struct mma8453q_accel_data)

struct mma8453q_pdata {
	int irq1;
	int irq2;
};

#endif
