/*
 * linux/arch/arm/mach-omap2/io.c
 *
 * OMAP2 I/O mapping code
 *
 * Copyright (C) 2005 Nokia Corporation
 * Copyright (C) 2007-2009 Texas Instruments
 *
 * Author:
 *	Juha Yrjola <juha.yrjola@nokia.com>
 *	Syed Khasim <x0khasim@ti.com>
 *
 * Added OMAP4 support - Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>

#include <asm/tlb.h>
#include <asm/mach/map.h>

#include <plat/sram.h>
#include <plat/sdrc.h>
#include <plat/serial.h>
#include <plat/omap-pm.h>
#include <plat/omap_hwmod.h>
#include <plat/multi.h>

#include "iomap.h"
#include "voltage.h"
#include "powerdomain.h"
#include "clockdomain.h"
#include "common.h"
#include "clock2xxx.h"
#include "clock3xxx.h"
#include "clock44xx.h"
#include "clock54xx.h"
#include "pm.h"

#ifdef CONFIG_OMAP_SO_MAP_FORCE
#define OMAP_IO_MAP_TYPE	MT_MEMORY_SO
#else
#define OMAP_IO_MAP_TYPE	MT_DEVICE
#endif

/*
 * The machine specific code may provide the extra mapping besides the
 * default mapping provided here.
 */

#if defined(CONFIG_SOC_OMAP2420) || defined(CONFIG_SOC_OMAP2430)
static struct map_desc omap24xx_io_desc[] __initdata = {
	{
		.virtual	= L3_24XX_VIRT,
		.pfn		= __phys_to_pfn(L3_24XX_PHYS),
		.length		= L3_24XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
	{
		.virtual	= L4_24XX_VIRT,
		.pfn		= __phys_to_pfn(L4_24XX_PHYS),
		.length		= L4_24XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
};

#ifdef CONFIG_SOC_OMAP2420
static struct map_desc omap242x_io_desc[] __initdata = {
	{
		.virtual	= DSP_MEM_2420_VIRT,
		.pfn		= __phys_to_pfn(DSP_MEM_2420_PHYS),
		.length		= DSP_MEM_2420_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
	{
		.virtual	= DSP_IPI_2420_VIRT,
		.pfn		= __phys_to_pfn(DSP_IPI_2420_PHYS),
		.length		= DSP_IPI_2420_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
	{
		.virtual	= DSP_MMU_2420_VIRT,
		.pfn		= __phys_to_pfn(DSP_MMU_2420_PHYS),
		.length		= DSP_MMU_2420_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
};

#endif

#ifdef CONFIG_SOC_OMAP2430
static struct map_desc omap243x_io_desc[] __initdata = {
	{
		.virtual	= L4_WK_243X_VIRT,
		.pfn		= __phys_to_pfn(L4_WK_243X_PHYS),
		.length		= L4_WK_243X_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
	{
		.virtual	= OMAP243X_GPMC_VIRT,
		.pfn		= __phys_to_pfn(OMAP243X_GPMC_PHYS),
		.length		= OMAP243X_GPMC_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
	{
		.virtual	= OMAP243X_SDRC_VIRT,
		.pfn		= __phys_to_pfn(OMAP243X_SDRC_PHYS),
		.length		= OMAP243X_SDRC_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
	{
		.virtual	= OMAP243X_SMS_VIRT,
		.pfn		= __phys_to_pfn(OMAP243X_SMS_PHYS),
		.length		= OMAP243X_SMS_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
};
#endif
#endif

#ifdef	CONFIG_ARCH_OMAP3
static struct map_desc omap34xx_io_desc[] __initdata = {
	{
		.virtual	= L3_34XX_VIRT,
		.pfn		= __phys_to_pfn(L3_34XX_PHYS),
		.length		= L3_34XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
	{
		.virtual	= L4_34XX_VIRT,
		.pfn		= __phys_to_pfn(L4_34XX_PHYS),
		.length		= L4_34XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
	{
		.virtual	= OMAP34XX_GPMC_VIRT,
		.pfn		= __phys_to_pfn(OMAP34XX_GPMC_PHYS),
		.length		= OMAP34XX_GPMC_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
	{
		.virtual	= OMAP343X_SMS_VIRT,
		.pfn		= __phys_to_pfn(OMAP343X_SMS_PHYS),
		.length		= OMAP343X_SMS_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
	{
		.virtual	= OMAP343X_SDRC_VIRT,
		.pfn		= __phys_to_pfn(OMAP343X_SDRC_PHYS),
		.length		= OMAP343X_SDRC_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
	{
		.virtual	= L4_PER_34XX_VIRT,
		.pfn		= __phys_to_pfn(L4_PER_34XX_PHYS),
		.length		= L4_PER_34XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
	{
		.virtual	= L4_EMU_34XX_VIRT,
		.pfn		= __phys_to_pfn(L4_EMU_34XX_PHYS),
		.length		= L4_EMU_34XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
#if defined(CONFIG_DEBUG_LL) &&							\
	(defined(CONFIG_MACH_OMAP_ZOOM2) || defined(CONFIG_MACH_OMAP_ZOOM3))
	{
		.virtual	= ZOOM_UART_VIRT,
		.pfn		= __phys_to_pfn(ZOOM_UART_BASE),
		.length		= SZ_1M,
		.type		= OMAP_IO_MAP_TYPE
	},
#endif
};
#endif

#ifdef CONFIG_SOC_OMAPTI81XX
static struct map_desc omapti81xx_io_desc[] __initdata = {
	{
		.virtual	= L4_34XX_VIRT,
		.pfn		= __phys_to_pfn(L4_34XX_PHYS),
		.length		= L4_34XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	}
};
#endif

#ifdef CONFIG_SOC_OMAPAM33XX
static struct map_desc omapam33xx_io_desc[] __initdata = {
	{
		.virtual	= L4_34XX_VIRT,
		.pfn		= __phys_to_pfn(L4_34XX_PHYS),
		.length		= L4_34XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	},
	{
		.virtual	= L4_WK_AM33XX_VIRT,
		.pfn		= __phys_to_pfn(L4_WK_AM33XX_PHYS),
		.length		= L4_WK_AM33XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE
	}
};
#endif

#ifdef	CONFIG_ARCH_OMAP4
static struct map_desc omap44xx_io_desc[] __initdata = {
	{
		.virtual	= L3_44XX_VIRT,
		.pfn		= __phys_to_pfn(L3_44XX_PHYS),
		.length		= L3_44XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE,
	},
	{
		.virtual	= L4_44XX_VIRT,
		.pfn		= __phys_to_pfn(L4_44XX_PHYS),
		.length		= L4_44XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE,
	},
	{
		.virtual	= L4_PER_44XX_VIRT,
		.pfn		= __phys_to_pfn(L4_PER_44XX_PHYS),
		.length		= L4_PER_44XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE,
	},
#ifdef CONFIG_OMAP4_ERRATA_I688
	{
		.virtual	= OMAP4_ERRATA_I688_SRAM_VA,
		.pfn		= __phys_to_pfn(OMAP4_ERRATA_I688_SRAM_PA),
		.length		= OMAP4_ERRATA_I688_SIZE,
		.type		= MT_MEMORY_SO,
	},
#endif

};
#endif

#ifdef	CONFIG_ARCH_OMAP5
static struct map_desc omap54xx_io_desc[] __initdata = {
	{
		.virtual	= L3_54XX_VIRT,
		.pfn		= __phys_to_pfn(L3_54XX_PHYS),
		.length		= L3_54XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE,
	},
	{
		.virtual	= L4_54XX_VIRT,
		.pfn		= __phys_to_pfn(L4_54XX_PHYS),
		.length		= L4_54XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE,
	},
	{
		.virtual	= L4_WK_54XX_VIRT,
		.pfn		= __phys_to_pfn(L4_WK_54XX_PHYS),
		.length		= L4_WK_54XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE,
	},
	{
		.virtual	= L4_PER_54XX_VIRT,
		.pfn		= __phys_to_pfn(L4_PER_54XX_PHYS),
		.length		= L4_PER_54XX_SIZE,
		.type		= OMAP_IO_MAP_TYPE,
	},
#ifdef CONFIG_OMAP4_ERRATA_I688
	{
		.virtual	= OMAP5_ERRATA_I688_SRAM_VA,
		.pfn		= __phys_to_pfn(OMAP5_ERRATA_I688_SRAM_PA),
		.length		= OMAP4_ERRATA_I688_SIZE,
		.type		= MT_MEMORY_SO,
	},
#endif
};
#endif

#ifdef CONFIG_SOC_OMAP2420
void __init omap242x_map_common_io(void)
{
	iotable_init(omap24xx_io_desc, ARRAY_SIZE(omap24xx_io_desc));
	iotable_init(omap242x_io_desc, ARRAY_SIZE(omap242x_io_desc));
}
#endif

#ifdef CONFIG_SOC_OMAP2430
void __init omap243x_map_common_io(void)
{
	iotable_init(omap24xx_io_desc, ARRAY_SIZE(omap24xx_io_desc));
	iotable_init(omap243x_io_desc, ARRAY_SIZE(omap243x_io_desc));
}
#endif

#ifdef CONFIG_ARCH_OMAP3
void __init omap34xx_map_common_io(void)
{
	iotable_init(omap34xx_io_desc, ARRAY_SIZE(omap34xx_io_desc));
}
#endif

#ifdef CONFIG_SOC_OMAPTI81XX
void __init omapti81xx_map_common_io(void)
{
	iotable_init(omapti81xx_io_desc, ARRAY_SIZE(omapti81xx_io_desc));
}
#endif

#ifdef CONFIG_SOC_OMAPAM33XX
void __init omapam33xx_map_common_io(void)
{
	iotable_init(omapam33xx_io_desc, ARRAY_SIZE(omapam33xx_io_desc));
}
#endif

#ifdef CONFIG_ARCH_OMAP4
void __init omap44xx_map_common_io(void)
{
	iotable_init(omap44xx_io_desc, ARRAY_SIZE(omap44xx_io_desc));
	omap_barriers_init();
}
#endif

#ifdef CONFIG_ARCH_OMAP5
void __init omap54xx_map_common_io(void)
{
	iotable_init(omap54xx_io_desc, ARRAY_SIZE(omap54xx_io_desc));
	omap_barriers_init();
}
#endif
/*
 * omap2_init_reprogram_sdrc - reprogram SDRC timing parameters
 *
 * Sets the CORE DPLL3 M2 divider to the same value that it's at
 * currently.  This has the effect of setting the SDRC SDRAM AC timing
 * registers to the values currently defined by the kernel.  Currently
 * only defined for OMAP3; will return 0 if called on OMAP2.  Returns
 * -EINVAL if the dpll3_m2_ck cannot be found, 0 if called on OMAP2,
 * or passes along the return value of clk_set_rate().
 */
static int __init _omap2_init_reprogram_sdrc(void)
{
	struct clk *dpll3_m2_ck;
	int v = -EINVAL;
	long rate;

	if (!cpu_is_omap34xx())
		return 0;

	dpll3_m2_ck = clk_get(NULL, "dpll3_m2_ck");
	if (IS_ERR(dpll3_m2_ck))
		return -EINVAL;

	rate = clk_get_rate(dpll3_m2_ck);
	pr_info("Reprogramming SDRC clock to %ld Hz\n", rate);
	v = clk_set_rate(dpll3_m2_ck, rate);
	if (v)
		pr_err("dpll3_m2_clk rate change failed: %d\n", v);

	clk_put(dpll3_m2_ck);

	return v;
}

static int _set_hwmod_postsetup_state(struct omap_hwmod *oh, void *data)
{
	return omap_hwmod_set_postsetup_state(oh, *(u8 *)data);
}

static void __init omap_common_init_early(void)
{
	omap_init_consistent_dma_size();
}

static void __init omap_hwmod_init_postsetup(void)
{
	u8 postsetup_state;

	/* Set the default postsetup state for all hwmods */
#ifdef CONFIG_PM_RUNTIME
	postsetup_state = _HWMOD_STATE_IDLE;
#else
	postsetup_state = _HWMOD_STATE_ENABLED;
#endif
	omap_hwmod_for_each(_set_hwmod_postsetup_state, &postsetup_state);

	/*
	 * Set the default postsetup state for unusual modules (like
	 * MPU WDT).
	 *
	 * The postsetup_state is not actually used until
	 * omap_hwmod_late_init(), so boards that desire full watchdog
	 * coverage of kernel initialization can reprogram the
	 * postsetup_state between the calls to
	 * omap2_init_common_infra() and omap_sdrc_init().
	 *
	 * XXX ideally we could detect whether the MPU WDT was currently
	 * enabled here and make this conditional
	 */
	postsetup_state = _HWMOD_STATE_DISABLED;
	omap_hwmod_for_each_by_class("wd_timer",
				     _set_hwmod_postsetup_state,
				     &postsetup_state);

	if (cpu_is_omap54xx())
		pr_err("FIXME: omap5 opp layer init\n");
	else
		omap_pm_if_early_init();
}
#ifdef CONFIG_OMAP_SO_MAP_FORCE
/* SO limits */
static unsigned long _min_so_phy_map, _max_so_phy_map;

void __iomem *__omap_ioremap_caller(unsigned long phys_addr, size_t size,
				   unsigned int mtype, void *caller)
{
	/* change type if needed */
	if (mtype == MT_DEVICE && _max_so_phy_map && _max_so_phy_map) {
		if (phys_addr >= _min_so_phy_map && phys_addr < _max_so_phy_map)
			mtype = MT_MEMORY_SO;
	}

	return __arm_ioremap_caller(phys_addr, size, mtype, caller);
}

static inline void _phy_map_set_so_range(unsigned long min, unsigned long max)
{
	_min_so_phy_map = min;
	_max_so_phy_map = max;
	arch_ioremap_caller = __omap_ioremap_caller;
}

#else
static inline void _phy_map_set_so_range(unsigned long min, unsigned long max)
{}
#endif

#ifdef CONFIG_SOC_OMAP2420
void __init omap2420_init_early(void)
{
	omap2_set_globals_242x();
	omap2xxx_check_revision();
	omap_common_init_early();
	omap2xxx_voltagedomains_init();
	omap242x_powerdomains_init();
	omap242x_clockdomains_init();
	omap2420_hwmod_init();
	omap_hwmod_init_postsetup();
	omap2420_clk_init();
}
#endif

#ifdef CONFIG_SOC_OMAP2430
void __init omap2430_init_early(void)
{
	omap2_set_globals_243x();
	omap2xxx_check_revision();
	omap_common_init_early();
	omap2xxx_voltagedomains_init();
	omap243x_powerdomains_init();
	omap243x_clockdomains_init();
	omap2430_hwmod_init();
	omap_hwmod_init_postsetup();
	omap2430_clk_init();
}
#endif

/*
 * Currently only board-omap3beagle.c should call this because of the
 * same machine_id for 34xx and 36xx beagle.. Will get fixed with DT.
 */
#ifdef CONFIG_ARCH_OMAP3
void __init omap3_init_early(void)
{
	omap2_set_globals_3xxx();
	omap3xxx_check_revision();
	omap3xxx_check_features();
	omap_common_init_early();
	omap3xxx_voltagedomains_init();
	omap3xxx_powerdomains_init();
	omap3xxx_clockdomains_init();
	omap3xxx_hwmod_init();
	omap_hwmod_init_postsetup();
	omap3xxx_clk_init();
}

void __init omap3430_init_early(void)
{
	omap3_init_early();
}

void __init omap35xx_init_early(void)
{
	omap3_init_early();
}

void __init omap3630_init_early(void)
{
	omap3_init_early();
}

void __init am35xx_init_early(void)
{
	omap3_init_early();
}

void __init ti81xx_init_early(void)
{
	omap2_set_globals_ti81xx();
	omap3xxx_check_revision();
	ti81xx_check_features();
	omap_common_init_early();
	omap3xxx_voltagedomains_init();
	omap3xxx_powerdomains_init();
	omap3xxx_clockdomains_init();
	omap3xxx_hwmod_init();
	omap_hwmod_init_postsetup();
	omap3xxx_clk_init();
}
#endif

#ifdef CONFIG_ARCH_OMAP4
void __init omap4430_init_early(void)
{
	printk("%s: %u\n", __FILE__, __LINE__);
	omap2_set_globals_443x();
	omap4xxx_check_revision();
	omap4xxx_check_features();
	omap_common_init_early();
	omap44xx_voltagedomains_init();
	omap44xx_powerdomains_init();
	omap44xx_clockdomains_init();
	omap44xx_hwmod_init();
	omap_hwmod_init_postsetup();
	omap4xxx_clk_init();
	omap4_opp_init();
}
#endif

#ifdef CONFIG_ARCH_OMAP5
void __init omap_5430evm_init_early(void)
{
	/*
	 * Set MT_MEMORY_SO for range 0x44000000-0x60000000 for ioremap
	 * calls.
	 */
	_phy_map_set_so_range(0x44000000, 0x60000000);

	omap2_set_globals_543x();
	omap5xxx_check_revision();
	omap5xxx_check_features();
	omap_common_init_early();
	omap54xx_voltagedomains_init();
	omap54xx_powerdomains_init();
	omap54xx_clockdomains_init();
	omap54xx_hwmod_init();
	omap_hwmod_init_postsetup();
	omap5xxx_clk_init();
	omap5_opp_init();
}
#endif

void __init omap_sdrc_init(struct omap_sdrc_params *sdrc_cs0,
				      struct omap_sdrc_params *sdrc_cs1)
{
	omap_sram_init();

	if (cpu_is_omap24xx() || omap3_has_sdrc()) {
		omap2_sdrc_init(sdrc_cs0, sdrc_cs1);
		_omap2_init_reprogram_sdrc();
	}
}
