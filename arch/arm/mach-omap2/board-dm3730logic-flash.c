/*
 * linux/arch/arm/mach-omap2/board-dm3730logic.c
 *
 * Copyright (C) 2011 Logic Product Development
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>

#include <plat/board.h>
#include <plat/gpmc.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/nand.h>

#include <plat/control.h>
#include <plat/nand.h>

#include "board-dm3730logic.h"

#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define DM3730LOGIC_NORFLASH_CS 2

#define NAND_BLOCK_SIZE        SZ_128K

/* TARR - NAND structured to support dual image (LINUX/ROOTFS) */

static struct mtd_partition dm3730logic_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "x-loader",
		.offset         = 0,
		.size           = 4*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "u-boot",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 15*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "u-boot-env",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 1*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "kernel-A",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 32*(SZ_128K)
	},
	{
		.name           = "kernel-B",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 32*(SZ_128K)
	},
	{
		.name           = "rootfs-A",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 2000*(SZ_128K)
	},
	{
		.name           = "rootfs-B",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 2000*(SZ_128K)
	},
	{
		.name           = "misc",
		.offset         = MTDPART_OFS_APPEND,
		.size           = MTDPART_SIZ_FULL
	},
};

static struct omap_nand_platform_data dm3730logic_nand_data = {
	.options	= NAND_BUSWIDTH_16 | NAND_SKIP_BBTSCAN,
	.parts          = dm3730logic_nand_partitions,
	.nr_parts       = ARRAY_SIZE(dm3730logic_nand_partitions),
	.nand_setup     = NULL,
	.dma_channel    = -1,           /* disable DMA in OMAP NAND driver */
#if 1
	.dev_ready	= (void *)1, /* poll WAIT0 status */
#else
	.dev_ready      = NULL,
#endif
};

static struct resource dm3730logic_nand_resource = {
	.flags          = IORESOURCE_MEM,
};

static struct platform_device dm3730logic_nand_device = {
	.name           = "omap2-nand",
	.id             = 0,
	.dev            = {
		.platform_data  = &dm3730logic_nand_data,
	},
	.num_resources  = 1,
	.resource       = &dm3730logic_nand_resource,
};

static struct mtd_partition dm3730logic_nor_partitions[] = {
	{
		.name		= CONFIG_DM3730LOGIC_NOR_PARTITION_ONE_NAME,
		.offset		= 0,
#if CONFIG_DM3730LOGIC_NOR_PARTITION_ONE_SIZE != 0
		.size		= (CONFIG_DM3730LOGIC_NOR_PARTITION_ONE_SIZE) << 17,
#else
		.size		= MTDPART_SIZ_FULL,
#endif
#if defined(CONFIG_DM3730LOGIC_NOR_PARTITION_ONE_READONLY)
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
#endif
	},
#if defined(CONFIG_DM3730LOGIC_NOR_PARTITION_TWO_SIZE) && (CONFIG_DM3730LOGIC_NOR_PARTITION_ONE_SIZE != 0)
	{
		.name		= CONFIG_DM3730LOGIC_NOR_PARTITION_TWO_NAME,
		.offset		= 0,
#if CONFIG_DM3730LOGIC_NOR_PARTITION_TWO_SIZE != 0
		.size		= (CONFIG_DM3730LOGIC_NOR_PARTITION_TWO_SIZE) << 17,
#else
		.size		= MTDPART_SIZ_FULL,
#endif
#if defined(CONFIG_DM3730LOGIC_NOR_PARTITION_TWO_READONLY)
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
#endif
	},
#endif
#if defined (CONFIG_DM3730LOGIC_NOR_PARTITION_THREE_SIZE) && (CONFIG_DM3730LOGIC_NOR_PARTITION_TWO_SIZE != 0)
	{
		.name		= CONFIG_DM3730LOGIC_NOR_PARTITION_THREE_NAME,
		.offset		= 0,
#if CONFIG_DM3730LOGIC_NOR_PARTITION_THREE_SIZE != 0
		.size		= (CONFIG_DM3730LOGIC_NOR_PARTITION_THREE_SIZE) << 17,
#else
		.size		= MTDPART_SIZ_FULL,
#endif
#if defined(CONFIG_DM3730LOGIC_NOR_PARTITION_THREE_READONLY)
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
#endif
	},
#endif
};

static struct physmap_flash_data dm3730logic_nor_data = {
	.width		= 2,
	.parts		= dm3730logic_nor_partitions,
	.nr_parts	= ARRAY_SIZE(dm3730logic_nor_partitions),
};

static struct resource dm3730logic_nor_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device dm3730logic_nor_device = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data = &dm3730logic_nor_data,
	},
	.num_resources	= 1,
	.resource	= &dm3730logic_nor_resource,
};

void __init dm3730logic_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;
	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;
	int nor_cs;
	unsigned long cs_mem_base;
	int nor_size = 0;

#if 1
#warning "Need productID to determine if NOR present!"
	if (machine_is_dm3730_som_lv()) {
		printk("%s: Assuming NOR present (need productID!)\n", __FUNCTION__);
		nor_size = 23; /* assume 8MB */
	}
#else
	nor_size = dm3730logic_NOR0_size();
#endif

	if (nor_size > 0) {
		nor_cs = 2;
		if (gpmc_cs_request(nor_cs, SZ_8M, &cs_mem_base) < 0) {
			printk(KERN_ERR "Failed to request GPMC mem for NOR flash\n");
			return;
		}
		printk("%s: NOR cs_mem_base %#lx\n", __FUNCTION__, cs_mem_base);

		dm3730logic_nor_resource.start = cs_mem_base;
		dm3730logic_nor_resource.end = cs_mem_base + (1 << nor_size) - 1;
		if (platform_device_register(&dm3730logic_nor_device) < 0)
			printk(KERN_ERR "Unable to register NOR device\n");
	}

	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {

			/* Found it!! */
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration in GPMC\n");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		dm3730logic_nand_data.cs   = nandcs;
		dm3730logic_nand_data.gpmc_cs_baseaddr = (void *)(gpmc_base_add +
					GPMC_CS0_BASE + nandcs*GPMC_CS_SIZE);
		dm3730logic_nand_data.gpmc_baseaddr   = (void *) (gpmc_base_add);

		if (platform_device_register(&dm3730logic_nand_device) < 0) {
			printk(KERN_ERR "Unable to register NAND device\n");
		}
	}
}
