/*
 * SDRC register values for the Micron MT29C4G48MAZAPAKQ5
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARCH_ARM_MACH_OMAP2_SDRAM_MICRON_MT29C4G48MAZAPAKQ5
#define ARCH_ARM_MACH_OMAP2_SDRAM_MICRON_MT46C4G48MAZAPAKQ5

#include <plat/sdrc.h>

/* Micron MT29C4G48MAZAPAKQ-5 */
static struct omap_sdrc_params mt29c4g48mazapakq5_sdrc_params[] = {
	[0] = {
		.rate	     = 200000000,
		.actim_ctrla = 0x83264707,
		.actim_ctrlb = 0x00021623,
		.rfr_ctrl    = 0x00021623,
		.mr	     = 0x00000032,
	},
	[1] = {
		.rate	     = 166000000,
		.actim_ctrla = 0x6ae24707,
		.actim_ctrlb = 0x00011617,
		.rfr_ctrl    = 0x0003c701,
		.mr	     = 0x00000032,
	},
	[2] = {
		.rate	     = 0
	},
};

#endif
