#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/feature_list.h>

#include <linux/init.h>
#include <linux/ion.h>
#include <linux/memblock.h>
#include <linux/omap_ion.h>

#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/lpddr2-micron.h>

#include <plat/common.h>
#include <plat/remoteproc.h>

#include "omap4_ion.h"
#include "omap_ram_console.h"

//#define FORCE_TI_DEFAULT

static struct emif_device_details *emif_config;


#define ARCHOS_PHYS_ADDR_DUCATI_SIZE		(SZ_1M * 55)

#define ARCHOS_OMAP4_ION_HEAP_TILER_SIZE	(SZ_512K)
#define ARCHOS_OMAP4_ION_HEAP_SECURE_SIZE       (SZ_512K)
#define ARCHOS_PHYS_ADDR_DUCATI_MEM		(0x80000000 + SZ_512M - ARCHOS_PHYS_ADDR_DUCATI_SIZE)
#define ARCHOS_PHYS_ADDR_HEAP_TILER		(ARCHOS_PHYS_ADDR_DUCATI_MEM - ARCHOS_OMAP4_ION_HEAP_TILER_SIZE)
#define ARCHOS_PHYS_ADDR_HEAP_SECURE		(ARCHOS_PHYS_ADDR_HEAP_TILER - ARCHOS_OMAP4_ION_HEAP_SECURE_SIZE)
#define ARCHOS_PHYS_ADDR_OMAP_RAM_CONSOLE	(ARCHOS_PHYS_ADDR_HEAP_SECURE - ARCHOS_OMAP_RAM_CONSOLE_SIZE)
#define ARCHOS_OMAP_RAM_CONSOLE_SIZE 		(SZ_1M)

enum map_id{
  ARCHOS_POOL = 0,
  TI_POOL,
};

#ifdef CONFIG_ION_OMAP_DYNAMIC
#define MAP_POOL	ARCHOS_POOL
#else
#define MAP_POOL	TI_POOL
#endif

static int ducati_phys_addr_data[2] = {
	//512 MB device
	ARCHOS_PHYS_ADDR_DUCATI_MEM,
	//1GB device
	PHYS_ADDR_DUCATI_MEM,
};

static struct ion_platform_data ti_ion_heap = {
	.nr = 3,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = OMAP_ION_HEAP_SECURE_INPUT,
			.name = "secure_input",
			.base = PHYS_ADDR_SMC_MEM -
					OMAP4_ION_HEAP_SECURE_INPUT_SIZE,
			.size = OMAP4_ION_HEAP_SECURE_INPUT_SIZE,
		},
		{	.type = OMAP_ION_HEAP_TYPE_TILER,
			.id = OMAP_ION_HEAP_TILER,
			.name = "tiler",
			.base = PHYS_ADDR_DUCATI_MEM -
					OMAP4_ION_HEAP_TILER_SIZE,
			.size = OMAP4_ION_HEAP_TILER_SIZE,
		},
		{
			.type = OMAP_ION_HEAP_TYPE_TILER,
			.id = OMAP_ION_HEAP_NONSECURE_TILER,
			.name = "nonsecure_tiler",
			.base = 0x80000000 + SZ_512M + SZ_2M,
			.size = OMAP4_ION_HEAP_NONSECURE_TILER_SIZE,
		},
	},
};
//We need structure for mempool
// So declare at least one heap of each kind with 1MB size
static struct ion_platform_data archos_ion_heap = {
	.nr = 2,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = OMAP_ION_HEAP_SECURE_INPUT,
			.name = "secure_input",
			.base = ARCHOS_PHYS_ADDR_HEAP_SECURE,
			.size = ARCHOS_OMAP4_ION_HEAP_SECURE_SIZE,
		},
		{	.type = OMAP_ION_HEAP_TYPE_TILER,
			.id = OMAP_ION_HEAP_TILER,
			.name = "tiler",
			.base = ARCHOS_PHYS_ADDR_HEAP_TILER,
			.size = ARCHOS_OMAP4_ION_HEAP_TILER_SIZE,
		},
	},
};

static struct ion_platform_data *archos_ion_data[2] = {
	&archos_ion_heap,
	&ti_ion_heap,

};

/*
 * LPDDR2 Configuration Data
 * The memory organisation is as below :
 *	EMIF1 - CS0 -	2 Gb
 *		CS1 -
 *	EMIF2 - CS0 -	2 Gb
 *		CS1 -
 *	--------------------
 *	TOTAL -		4 Gb
 *
 *or 	EMIF1 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	EMIF2 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 *
 *or	EMIF1 - CS0 -	4 Gb
 *		CS1 -
 *	EMIF2 - CS0 -	4 Gb
 *		CS1 -
 *	--------------------
 *	TOTAL -		8 Gb

 * Same devices installed on EMIF1 and EMIF2
 */
static struct emif_device_details emif_devices[] = {
	{
	.cs0_device = &micron_2G_S4,
	.cs1_device = NULL
	},
	{
	.cs0_device = &lpddr2_elpida_2G_S4_dev,
	.cs1_device = NULL
	},
	{
	.cs0_device = &lpddr2_elpida_2G_S4_dev,
	.cs1_device = NULL
	},
	{
	.cs0_device = &lpddr2_elpida_2G_S4_dev,
	.cs1_device = &lpddr2_elpida_2G_S4_dev,
	},
	{
	.cs0_device = &lpddr2_elpida_4G_S4_dev,
	.cs1_device = NULL
	},
};

static void __init setup_archos_ion_data(int configId)
{
	omap_ion_set_platform_data(archos_ion_data[configId]);

	if (configId != TI_POOL) {
		/* do the static reservations first */
		memblock_remove(ducati_phys_addr_data[configId], PHYS_ADDR_DUCATI_SIZE);
		/* ipu needs to recognize secure input buffer area as well */
#ifdef CONFIG_ION_OMAP_DYNAMIC
		omap_ipu_set_static_mempool(ducati_phys_addr_data[configId],
						PHYS_ADDR_DUCATI_SIZE);
#else
		omap_ipu_set_static_mempool(ducati_phys_addr_data[configId],
						PHYS_ADDR_DUCATI_SIZE + ARCHOS_OMAP4_ION_HEAP_SECURE_SIZE);
#endif
	} else {
		/* do the static reservations first */
		memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
		memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
		/* ipu needs to recognize secure input buffer area as well */
		omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
						OMAP4_ION_HEAP_SECURE_INPUT_SIZE);
	}
}

static void __init board_memory_prepare(void)
{
	struct feature_tag_sdram *sdram;

	if (( sdram = get_feature_tag(FTAG_SDRAM, 
			feature_tag_size(feature_tag_sdram)))) 
	{
		if (!strcmp(sdram->product, "MT42L64M64D2")) {			// MICRON
			emif_config = &emif_devices[0];
			printk(KERN_INFO "DDR type micron\n");
		} else	if (!strcmp(sdram->product, "EDB4064B2PB")) {		// ELPIDA
			emif_config = &emif_devices[1];
			printk(KERN_INFO "DDR type elpida\n");
		} else	if (!strcmp(sdram->product, "H9TKNNN4KDMPQR")) {	// HYNIX
			printk(KERN_INFO "DDR type hynix\n");
			emif_config = &emif_devices[2];
		} else	if (!strcmp(sdram->product, "EDB8064B1PB")) {		// Elpida 1Go
			printk(KERN_INFO "DDR type elpida 1Go\n");
			emif_config = &emif_devices[3];
		} else	if (!strcmp(sdram->product, "NT6TL64T64AR")) {		// Nanya 
			printk(KERN_INFO "DDR type nanya\n");
			emif_config = &emif_devices[1];
		} else	if (!strcmp(sdram->product, "NT6TL128F64AR")) {		// Nanya 1Go
			printk(KERN_INFO "DDR type nanya 1Go\n");
			emif_config = &emif_devices[3];
		} else	if (!strcmp(sdram->product, "EDB8164B3PF")) {		// Elpida 1Go
			printk(KERN_INFO "DDR type elpida 1Go\n");
			emif_config = &emif_devices[4];
		} else {
			printk(KERN_INFO "DDR type default\n");
			emif_config = &emif_devices[0];
		}
	} else {
		printk(KERN_INFO "DDR type unknown\n");
		emif_config = &emif_devices[0];
	}
	setup_archos_ion_data(MAP_POOL);
}

int __init archos_memory_init(void)
{
	if (emif_config == NULL) {
		printk(KERN_ERR "archos_memory_init Failed : No valid emif_config!\n");
		return -EINVAL;
	}
	omap_emif_setup_device_details(emif_config, emif_config);

	omap4_register_ion();
	return 0;
}

void __init archos_reserve(void)
{
	//Check the ram chip
	board_memory_prepare();
	omap_ram_console_init(ARCHOS_PHYS_ADDR_OMAP_RAM_CONSOLE,
                        ARCHOS_OMAP_RAM_CONSOLE_SIZE);

#ifdef CONFIG_ION_OMAP
	omap_ion_init();
#endif
	omap_reserve();
}
