/*
 * linux/arch/arm/mach-omap2/board-omap3logic.h
 *
 * Copyright (C) 2010 Logic Product Development
 *
 * Initial code: Peter Barada
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

extern struct omap_dss_device lcd_device;
extern struct platform_device dss_device;
extern struct omap_sdrc_params *omap3logic_get_sdram_timings(void);
extern int omap_zoom3_wifi_set_carddetect(int);
extern void omap3logic_lcd_init(void);
extern void dump_omap3logic_timings(void);
