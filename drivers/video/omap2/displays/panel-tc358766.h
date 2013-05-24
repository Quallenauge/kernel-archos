
#ifndef __PANEL_TC358766_H__
#define __PANEL_TC358766_H__

/* DSI D-PHY Layer Registers */
#define	D0W_DPHYCONTTX		0x0004		/* Data Lane 0 DPHY TX */
#define	CLW_DPHYCONTRX		0x0020		/* Clock Lane DPHY RX */
#define	D0W_DPHYCONTRX		0x0024		/* Data Land 0 DPHY Rx */
#define	D1W_DPHYCONTRX		0x0028		/* Data Lane 1 DPHY Rx */
#define	D2W_DPHYCONTRX		0x002c		/* Data Lane 2 DPHY Rx */
#define	D3W_DPHYCONTRX		0x0030		/* Data Lane 3 DPHY Rx */
#define	COM_DPHYCONTRX		0x0038		/* DPHY Rx Common */
#define	CLW_CNTRL		0x0040		/* Clock Lane */
#define	D0W_CNTRL		0x0044		/* Data Lane 0 */
#define	D1W_CNTRL		0x0048		/* Data Lane 1 */
#define	D2W_CNTRL		0x004c		/* Data Lane 2 */
#define	D3W_CNTRL		0x0050		/* Data Lane 3 */
#define	DFTMODE_CNTRL		0x0054		/* DFT Mode */

/* DSI PPI Layer Registers */
#define	PPI_STARTPPI		0x0104		/* Start control bit */
#define	PPI_BUSYPPI		0x0108			/* Busy bit */
#define	PPI_LINEINITCNT		0x0110		/* Line In initialization */
#define	PPI_LPTXTIMECNT		0x0114		/* LPTX timing signal */
#define	PPI_LANEENABLE		0x0134		/* Lane Enable */
#define	PPI_TX_RX_TA		0x013c		/* BTA timing param */
#define	PPI_CLS_ATMR		0x0140		/* Analog timer fcn */
#define	PPI_D0S_ATMR		0x0144		/* Analog timer fcn Lane 0 */
#define	PPI_D1S_ATMR		0x0148		/* Analog timer fcn Lane 1 */
#define	PPI_D2S_ATMR		0x014c		/* Analog timer fcn Lane 2 */
#define	PPI_D3S_ATMR		0x0150		/* Analog timer fcn Lane 3 */
#define	PPI_D0S_CLRSIPOCOUNT	0x0164		/* Assertion timer Lane 0 */
#define	PPI_D1S_CLRSIPOCOUNT	0x0168		/* Assertion timer Lane 1 */
#define	PPI_D2S_CLRSIPOCOUNT	0x016c		/* Assertion timer Lane 1 */
#define	PPI_D3S_CLRSIPOCOUNT	0x0170		/* Assertion timer Lane 1 */
#define CLS_PRE			0x0180		/* PHY IO cntr */
#define D0S_PRE			0x0184		/* PHY IO cntr */
#define D1S_PRE			0x0188		/* PHY IO cntr */
#define D2S_PRE			0x018c		/* PHY IO cntr */
#define D3S_PRE			0x0190		/* PHY IO cntr */
#define CLS_PREP		0x01a0		/* PHY IO cntr */
#define D0S_PREP		0x01a4		/* PHY IO cntr */
#define D1S_PREP		0x01a8		/* PHY IO cntr */
#define D2S_PREP		0x01ac		/* PHY IO cntr */
#define D3S_PREP		0x01b0		/* PHY IO cntr */
#define CLS_ZERO		0x01c0		/* PHY IO cntr */
#define	D0S_ZERO		0x01c4		/* PHY IO cntr */
#define	D1S_ZERO		0x01c8		/* PHY IO cntr */
#define	D2S_ZERO		0x01cc		/* PHY IO cntr */
#define	D3S_ZERO		0x01d0		/* PHY IO cntr */
#define PPI_CLRFLG		0x01e0		/* PRE cntrs */
#define PPI_CLRSIPO		0x01e4		/* Clear SIPO */
#define PPI_HSTimeout		0x01f0		/* HS RX timeout */
#define PPI_HSTimeoutEnable	0x01f4		/* Enable HS Rx Timeout */

/* DSI Protocol Layer Registers */
#define DSI_STARTDSI		0x0204		/* DSI TX start bit */
#define DSI_BUSYDSI		0x0208		/* DSI busy bit */
#define DSI_LANEENABLE		0x0210		/* Lane enable */
#define DSI_LANESTATUS0		0x0214		/* HS Rx mode */
#define DSI_LANESTATUS1		0x0218		/* ULPS or STOP state */
#define DSI_INTSTATUS		0x0220		/* Interrupt status */
#define DSI_INTMASK		0x0224		/* Interrupt mask */
#define DSI_INTCLR		0x0228		/* Interrupt clear */
#define DSI_LPTXTO		0x0230		/* LP Tx Cntr */

/* DSI General Registers */
#define	DSIERRCNT		0x0300		/* DSI Error Count */

/* DSI Application Layer Registers */
#define APLCTRL			0x0400		/* Application Layer Cntrl */
#define RDPKTLN			0x0404		/* Packet length */

/* DPI */
#define DPIPXLFMT		0x440		// DPI input pixel format */

/* Parallel Output */
#define	POCTRL			0x448		// Parallel outpt control */

/* Video Path0 Registers */
#define	VPCTRL0			0x0450		/* Video Path */
#define HTIM01			0x0454		/* Horizontal Timing */
#define HTIM02			0x0458		/* Horizontal Timing */
#define VTIM01			0x045c		/* Vertical Timing */
#define VTIM02			0x0460		/* Vertical Timing */
#define VFUEN0			0x0464		/* Video Frame Timing */

/* Video Path1 Registers */
#define	VPCTRL1			0x0480		/* Video Path */
#define HTIM11			0x0484		/* Horizontal Timing */
#define HTIM12			0x0488		/* Horizontal Timing */
#define VTIM11			0x048c		/* Vertical Timing */
#define VTIM12			0x0490		/* Vertical Timing */
#define VFUEN1			0x0494		/* Video Frame Timing */

/* System Registers */
#define IDREG			0x0500		/* Chip and Revision ID */
#define SYSBOOT			0x0504		/* System Bootstrap status */
#define SYSSTAT			0x0508		/* System Status */
#define SYSRSTENB		0x050c		/* System Reset/enable */
#define SYSCTRL			0x0510		/* System Control */

/* I2C Registers */
#define I2TIMCTTTRL		0x0520		/* I2C IF timings Control */

/* GPIO Registers */
#define GPIOM			0x0540		/* GPIO Control */
#define GPIOC			0x0544		/* GPIO Control */
#define GPIOO			0x0548		/* GPIO Output */
#define GPIOI			0x054c		/* GPIO Input */

/* Interrupt registers */
#define INTCTL_G		0x0560		/* Gen irq Control */
#define INTSTS_G		0x0564		/* Gen irq Status */
#define INTCTL_S		0x0570		/* Sync irq Control */
#define INTSTS_S		0x0574		/* Syn irq Status */
#define INT_GP0_LNCT		0x0584		/* Int GPIO0 value */
#define INT_GP1_LNCT		0x0588		/* Int GPIO1 value */

/* Display port control */
#define DP0Ctl			0x0600		/* Display port0 control */
#define DP1Ctl			0x0604		/* Display port1 control */

/* Display port clock */
#define DP0_VidMNGen0		0x0610		/* Display port0 Video Force M */
#define DP0_VidMNGen1		0x0614		/* Display port0 Video Force N */
#define DP0_VMNGenStatus	0x0618		/* Display port0 Video Current M */
#define DP1_VidMNGen0		0x061c		/* Display port1 Video Force M */
#define DP1_VidMNGen1		0x0620		/* Display port1 Video Force N */
#define DP1_VMNGenStatus	0x0624		/* Display port1 Video Current M */
#define DP0_AudMNGen0		0x0628		/* Display port0 Audio Force M */
#define DP0_AudMNGen1		0x062c		/* Display port0 Audio Force N */
#define DP0_AMNGenStatus	0x0630		/* Display port0 Audio Current M */
#define DP1_AudMNGen0		0x0634		/* Display port1 Audio Force M */
#define DP1_AudMNGen1		0x0638		/* Display port1 Audio Force N */
#define DP1_AMNGenStatus	0x063c		/* Display port1 Audio Current M */

/* Display port0 Main channel registers */
#define DP0_SecSample		0x0640		/* Display port0 Audio sample count */
#define DP0_VidSyncDelay	0x0644		/* Display port0 Video Sync delay */
#define DP0_TotalValReg		0x0648		/* Display port0 Total frame size */
#define DP0_StartVal		0x064c		/* Display port0 Active linePixel start */
#define DP0_ActiveVal		0x0650		/* Display port0 Active linePixel count */
#define DP0_SyncVal		0x0654		/* Display port0 Sync width */
#define DP0_Misc		0x0658		/* Display port0 Misc MSA */

/* Aux registers */
#define DP0_AuxCfg0		0x0660
#define DP0_AuxCfg1		0x0664
#define DP0_AuxAddr		0x0668
#define DP0_AuxWdata0		0x066C
#define DP0_AuxWdata1		0x0670
#define DP0_AuxWdata2		0x0674
#define DP0_AuxWdata3		0x0678
#define DP0_AuxRdata0		0x067C
#define DP0_AuxRdata1		0x0680
#define DP0_AuxRdata2		0x0684
#define DP0_AuxRdata3		0x0688
#define DP0_AuxStatus		0x068C
#define DP0_AuxI2CAdr		0x0698
#define DP0_SrcCtrl		0x06A0
#define DP0_LTStat		0x06D0
#define DP0_SnkLTChReq		0x06D4
#define DP0_LTLoopCtrl		0x06D8
#define DP0_SnkLTCtrl		0x06E4
#define DP0_TPatDat0		0x06E8
#define DP0_TPatDat1		0x06EC
#define DP0_TPatDat2		0x06F0
#define DP0_TPatDat3		0x06F4

/* Display port1 Main channel registers */
#define DP1_SecSample		0x0740		/* Display port1 Audio sample count */
#define DP1_VidSyncDelay	0x0744		/* Display port1 Video Sync delay */
#define DP1_TotalValReg		0x0748		/* Display port1 Total frame size */
#define DP1_StartVal		0x074c		/* Display port1 Active linePixel start */
#define DP1_ActiveVal		0x0750		/* Display port1 Active linePixel count */
#define DP1_SyncVal		0x0754		/* Display port1 Sync width */
#define DP1_Misc		0x0758		/* Display port1 Misc MSA */

/* Display port PHY control registers*/
#define DP_PHY_CTRL		0x0800
#define DP0_AUX_PHY_CTRL	0x0820
#define DP1_AUX_PHY_CTRL	0x0824

/* I2S */


/* PLL */
#define DP0_PLLCTRL		0x0900		/* DP0 PLL control */
#define DP1_PLLCTRL		0x0904		/* DP1 PLL control */
#define PXL_PLLCTRL		0x0908		/* Pixel PLL control */
#define PXL_PLLPARAM		0x0914		/* Pixel PLL params */
#define SYS_PLLPARAM		0x0918		/* PLL SYSCLK input selection */


/* HDCP */



/* Debug Register */
#define D2DPTSTCTL			0x0A00		/* D2DP test control*/
#define PLL_DBG				0x0A04		/* DP PLL debug */

/* PLL commands */
#define ENABLE_PLL	0x01
#define BYPASS_PLL	0x02
#define PLL_UPDATE	0x04


/*DSI DCS commands */
#define DCS_READ_NUM_ERRORS     0x05
#define DCS_READ_POWER_MODE     0x0a
#define DCS_READ_MADCTL         0x0b
#define DCS_READ_PIXEL_FORMAT   0x0c
#define DCS_RDDSDR              0x0f
#define DCS_SLEEP_IN            0x10
#define DCS_SLEEP_OUT           0x11
#define DCS_DISPLAY_OFF         0x28
#define DCS_DISPLAY_ON          0x29
#define DCS_COLUMN_ADDR         0x2a
#define DCS_PAGE_ADDR           0x2b
#define DCS_MEMORY_WRITE        0x2c
#define DCS_TEAR_OFF            0x34
#define DCS_TEAR_ON             0x35
#define DCS_MEM_ACC_CTRL        0x36
#define DCS_PIXEL_FORMAT        0x3a
#define DCS_BRIGHTNESS          0x51
#define DCS_CTRL_DISPLAY        0x53
#define DCS_WRITE_CABC          0x55
#define DCS_READ_CABC           0x56
#define DCS_GET_ID1             0xda
#define DCS_GET_ID2             0xdb
#define DCS_GET_ID3             0xdc

#define ADDR_ONLY_SHIFT		0x4
#define BSIZE_SHIFT		0x8

#define AUX_BUSY_FLAG		0x1
#define AUX_TIMEOUT_FLAG	0x2

#define A_RO 0x1
#define A_WO 0x2
#define A_RW (A_RO|A_WO)

#define FLD_MASK(start, end)	(((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))
#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))

#endif
