
#ifndef __PANEL_TC358766XBG_H__
#define __PANEL_TC358766XBG_H__

/* DSI PPI */
#define PPI_STARTPPI		0x104
#define PPI_BUSYPPI		0x108
#define PPI_LININITCNT		0x110
#define PPI_LPTXCNT		0x114
#define PPI_LANEENABLE		0x134
#define PPI_TX_RX_TA		0x13c
#define PPI_D0S_CLRSIPOCOUNT	0x164
#define PPI_D1S_CLRSIPOCOUNT	0x168
#define PPI_D2S_CLRSIPOCOUNT	0x16c
#define PPI_D3S_CLRSIPOCOUNT	0x170

/* DSI proto */
#define DSI_STARTDSI		0x204
#define DSI_BUSYDSI		0x208
#define DSI_LANEENABLE		0x210
#define DSI_LANESTATUS0		0x214
#define DSI_LANESTATUS1		0x218
#define DSI_INTSTATUS		0x220

/* Parallel out */
#define DPIPXLFMT		0x440
#define POCTRL			0x448

/* Video path DP Tx0 */
#define VPCTRL0			0x450
#define HTIM01			0x454
#define HTIM02			0x458
#define VTIM01			0x45c
#define VTIM02			0x460
#define VFUEN0			0x464

/* Video path DP Tx1 */
#define VPCTRL1			0x480

/* System */
#define IDREG			0x500
#define SYSCTRL			0x510
#define SYSSTAT			0x508

/* GPIO */
#define GPIOM			0x540
#define GPIOC			0x544
#define GPIOO			0x548
#define GPIOI			0x54c

/* Interrupt */
#define INTCTL_G		0x560
#define INTSTS_G		0x564
#define INTCTL_S		0x570
#define INTGP0LCNT		0x584

/* DP Control */
#define DP0Ctl			0x600
#define DP1Ctl			0x604

/* DP Clock */
#define DP0_VIDMNGEN0		0x610
#define DP0_VIDMNGEN1		0x614
#define DP0_VIDMNGENSTATUS	0x618

/* DP0 - Main Channel */
#define DP0_VIDSYNCDELAY	0x644
#define DP0_TOTALVAL		0x648
#define DP0_STARTVAL		0x64c
#define DP0_ACTIVEVAL		0x650
#define DP0_SYNCVAL		0x654
#define DP0_MISC		0x658

/* DP0 - Aux Channel */
#define DP0_AUXCFG0		0x660
#define DP0_AUXCFG1		0x664
#define DP0_AUXADDR		0x668
#define DP0_AUXWDATA(n)		(0x66c + 4 * n)
#define DP0_AUXRDATA(n)		(0x67c + 4 * n)
#define DP0_AUXSTATUS		0x68c
#define DP0_AUXI2CADR		0x698

/* DP0 - Link Trainin */
#define DP0_SRCCTRL		0x6a0
#define DP0_LTSTAT		0x6d0
#define DP0_LTLOOPCTRL		0x6d8
#define DP0_SNKLTCTRL		0x6e4

/*DP1 - Link Trainin */
#define DP1_SRCCTRL		0x7a0
#define DP1_LTSTAT		0x7d0
#define DP1_LTLOOPCTRL		0x7d8
#define DP1_SNKLTCTRL		0x7e4

/* DP PHY */
#define DP_PHY_CTRL		0x800

/* PLL */
#define DP0_PLLCTRL		0x900
#define DP1_PLLCTRL		0x904
#define PXL_PLLCTRL		0x908
#define PXL_PLLPARAM		0x914
#define SYS_PLLPARAM		0x918

/* Test */
#define D2DPTSTCTL		0xa00

#endif
