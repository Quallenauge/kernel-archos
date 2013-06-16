/*
 *   Definitions for MMA7660FC Accelerometer chip
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

#ifndef MMA7660FC_H
#define MMA7660FC_H

#include <linux/ioctl.h>

/*
 * MMA7660FC registers
 */

#define REG_XOUT			0x00
#define REG_YOUT			0x01
#define REG_ZOUT			0x02
#define REG_TILT			0x03
#define REG_SRST			0x04
#define REG_SPCNT			0x05
#define REG_INTSU			0x06
#define REG_MODE			0x07
#define REG_SR				0x08
#define REG_PDET			0x09
#define REG_PD				0x0A
/* Reserved				0x0B - 0x1F */


/*
 * Mask and bit definitions of MMA7660FC registers
 */

#define MMA7660FC_INVALID_FLAG		(0x1 << 6)

#define MMA7660FC_TILT_STATUS_MASK	0xFF

#define MMA7660FC_TILT_FB_SHIFT		0
#define MMA7660FC_TILT_FB_MASK		(0x03 << MMA7660FC_TILT_FB_SHIFT)
#define MMA7660FC_TILT_FB_UNKNOWN	(0x00 << MMA7660FC_TILT_FB_SHIFT)
#define MMA7660FC_TILT_FB_FRONT		(0x01 << MMA7660FC_TILT_FB_SHIFT)
#define MMA7660FC_TILT_FB_BACK		(0x02 << MMA7660FC_TILT_FB_SHIFT)
#define MMA7660FC_TILT_PL_SHIFT		2
#define MMA7660FC_TILT_PL_MASK		(0x07 << MMA7660FC_TILT_PL_MASK)
#define MMA7660FC_TILT_PL_UNKNOWN	(0x00 << MMA7660FC_TILT_PL_MASK)
#define MMA7660FC_TILT_PL_LEFT		(0x01 << MMA7660FC_TILT_PL_MASK)
#define MMA7660FC_TILT_PL_RIGHT		(0x02 << MMA7660FC_TILT_PL_MASK)
#define MMA7660FC_TILT_PL_DOWN		(0x05 << MMA7660FC_TILT_PL_MASK)
#define MMA7660FC_TILT_PL_UP		(0x06 << MMA7660FC_TILT_PL_MASK)
#define MMA7660FC_TILT_TAP		(0x01 << 0x5)
#define MMA7660FC_TILT_SHAKE		(0x01 << 0x7)

#define MMA7660FC_SLEEP_COUNT_MASK	0xFF

#define MMA7660FC_SR_AUTO_WAKE_SHIFT	0
#define MMA7660FC_SR_AUTO_WAKE_MASK	(0x07 << MMA7660FC_SR_AUTO_WAKE_SHIFT)
#define MMA7660FC_SR_AUTO_WAKE_120	(0x00 << MMA7660FC_SR_AUTO_WAKE_SHIFT)
#define MMA7660FC_SR_AUTO_WAKE_64	(0x01 << MMA7660FC_SR_AUTO_WAKE_SHIFT)
#define MMA7660FC_SR_AUTO_WAKE_32	(0x02 << MMA7660FC_SR_AUTO_WAKE_SHIFT)
#define MMA7660FC_SR_AUTO_WAKE_16	(0x03 << MMA7660FC_SR_AUTO_WAKE_SHIFT)
#define MMA7660FC_SR_AUTO_WAKE_8	(0x04 << MMA7660FC_SR_AUTO_WAKE_SHIFT)
#define MMA7660FC_SR_AUTO_WAKE_4	(0x05 << MMA7660FC_SR_AUTO_WAKE_SHIFT)
#define MMA7660FC_SR_AUTO_WAKE_2	(0x06 << MMA7660FC_SR_AUTO_WAKE_SHIFT)
#define MMA7660FC_SR_AUTO_WAKE_1	(0x07 << MMA7660FC_SR_AUTO_WAKE_SHIFT)
#define MMA7660FC_SR_AUTO_SLEEP_SHIFT	3
#define MMA7660FC_SR_AUTO_SLEEP_MASK	(0x03 << MMA7660FC_SR_AUTO_SLEEP_SHIFT)
#define MMA7660FC_SR_AUTO_SLEEP_32	(0x00 << MMA7660FC_SR_AUTO_SLEEP_SHIFT)
#define MMA7660FC_SR_AUTO_SLEEP_16	(0x01 << MMA7660FC_SR_AUTO_SLEEP_SHIFT)
#define MMA7660FC_SR_AUTO_SLEEP_8	(0x02 << MMA7660FC_SR_AUTO_SLEEP_SHIFT)
#define MMA7660FC_SR_AUTO_SLEEP_1	(0x03 << MMA7660FC_SR_AUTO_SLEEP_SHIFT)
#define MMA7660FC_SR_DEBOUNCE_SHIFT	5
#define MMA7660FC_SR_DEBOUNCE_MASK	(0x7 << MMA7660FC_SR_DEBOUNCE_SHIFT)

#define MMA7660FC_INT_SETUP_MASK	0xFF
#define MMA7660FC_INT_NONE		0
#define MMA7660FC_INT_FB		(0x1 << 0)
#define MMA7660FC_INT_PL		(0x1 << 1)
#define MMA7660FC_INT_TAP		(0x1 << 2)
#define MMA7660FC_INT_AUTO_SLEEP	(0x1 << 3)
#define MMA7660FC_INT_MEASURE		(0x1 << 4)
#define MMA7660FC_INT_SHAKE_X		(0x1 << 5)
#define MMA7660FC_INT_SHAKE_Y		(0x1 << 6)
#define MMA7660FC_INT_SHAKE_Z		(0x1 << 7)

#define MMA7660FC_MODE_SHIFT		0
#define MMA7660FC_MODE_MASK		(0x07 << MMA7660FC_MODE_SHIFT)
#define MMA7660FC_MODE_STANDBY		(0x00 << MMA7660FC_MODE_SHIFT)
#define MMA7660FC_MODE_MEASURE		(0x01 << MMA7660FC_MODE_SHIFT)
#define MMA7660FC_MODE_TEST		(0x04 << MMA7660FC_MODE_SHIFT)
#define MMA7660FC_MODE_AUTO_WAKE	(0x01 << 3)
#define MMA7660FC_MODE_AUTO_SLEEP	(0x01 << 4)
#define OFF				0
#define ON				1
#define MMA7660FC_MODE_PRESCALER	(0x01 << 5)
#define DIVIDE_BY_1			0
#define DIVIDE_BY_16			1
#define MMA7660FC_MODE_INT_TYPE		(0x01 << 6)
#define OPEN_DRAIN			0
#define PUSH_PULL			1
#define MMA7660FC_MODE_INT_ACTIVE_LEVEL	(0x01 << 7)
#define ACTIVE_LOW			0
#define ACTIVE_HIGH			1

#define MMA7660FC_PULSE_DEBOUNCE_MASK	0xFF
#define MMA7660FC_PULSE_THRESHOLD_MASK	0x1F
#define MMA7660FC_PULSE_AXIS_MASK	0xE0
#define MMA7660FC_PULSE_AXIS_NONE	0
#define MMA7660FC_PULSE_AXIS_X		(0x1 << 5)
#define MMA7660FC_PULSE_AXIS_Y		(0x1 << 6)
#define MMA7660FC_PULSE_AXIS_Z		(0x1 << 7)


struct accel_data {
	short x;
	short y;
	short z;
};


/*
 * IOCTLs
 */

#define MMAIO				0xA1

#define MMA7660FC_IOCTL_S_MODE		_IOW(MMAIO, 0x01, short)
#define MMA7660FC_IOCTL_G_MODE		_IOR(MMAIO, 0x02, short)
#define MMA7660FC_IOCTL_S_POLL_DELAY	_IOW(MMAIO, 0x03, int)
#define MMA7660FC_IOCTL_G_POLL_DELAY	_IOR(MMAIO, 0x04, int)
#define MMA7660FC_IOCTL_S_PULSE_DEBOUNCE	_IOW(MMAIO, 0x05, short)
#define MMA7660FC_IOCTL_G_PULSE_DEBOUNCE	_IOR(MMAIO, 0x06, short)
#define MMA7660FC_IOCTL_S_PULSE_THRSHOLD	_IOW(MMAIO, 0x07, short)
#define MMA7660FC_IOCTL_G_PULSE_THRSHOLD	_IOR(MMAIO, 0x08, short)
#define MMA7660FC_IOCTL_S_PULSE_AXIS	_IOW(MMAIO, 0x09, short)
#define MMA7660FC_IOCTL_G_PULSE_AXIS	_IOR(MMAIO, 0x0A, short)
#define MMA7660FC_IOCTL_GP_EVENT	_IOW(MMAIO, 0x0B, int)
#define MMA7660FC_IOCTL_G_ACCEL_DATA	_IOR(MMAIO, 0x0C, struct accel_data)

struct mma7660fc_pdata {
	int irq;
};

#endif
