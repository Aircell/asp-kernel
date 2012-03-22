/*
 * linux/arch/arm/plat-omap/include/plat/board-dm3730logic.h
 *
 * Copyright (C) 2011 Logic Product Devleopment, Inc.
 *
 * Initial code: Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

extern void dm3730logic_init_twl_external_mute(void);
extern void twl4030_set_ext_mute(int mute);
extern int twl4030_get_ext_mute(void);
extern void twl4030_set_path_mute(int mute);

struct dm3730logic_panel {
	char				*name;
	int				config;
	int				acb;
	char				data_lines;
	struct omap_video_timings	timing;
};
extern struct dm3730logic_panel dm3730logic_default_panel;

/* brt6300 Bluetooth define/prototypes */
#define TWL4030_BT_nSHUTDOWN	8
#define BT_IRQ_GPIO		157
