/*
 * linux/arch/arm/mach-omap2/board-dm3730logic.h
 *
 * Copyright (C) 2011 Logic Product Development
 *
 * Initial code: Peter Barada
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

extern struct omap_dss_device dm3730logic_lcd_device;
extern struct platform_device dm3730logic_dss_device;
extern struct omap_sdrc_params *dm3730logic_get_sdram_timings(void);
extern int omap_dm3730logic_wifi_set_carddetect(int);
extern void dm3730logic_lcd_init(void);
extern void dump_dm3730logic_timings(void);

extern int dm3730logic_twl4030_gpio_base;

extern volatile int dm3730logic_hang;

extern void __init dm3730logic_flash_init(void);
