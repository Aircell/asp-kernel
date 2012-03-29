
/* Modified for Aircell Cloudsurfer - 2011
 *
 * linux/arch/arm/mach-omap2/board-omap3logic-display.c
 *
 * Copyright (C) 2010 Li-Pro.Net
 * Stephan Linz <linz@li-pro.net>
 *
 * Copyright (C) 2009 Logic Product Development, Inc.
 * Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <linux/regulator/machine.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "mux.h"
#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>

#include <linux/spi/spi.h>
#include <plat/mcspi.h>
#include "cloudsurfer-gpio.h"

struct lcd_data {
	int	gpio_enable;
	int	gpio_backlight;
	int	lcd_enabled;
};

static struct lcd_data board_lcd_data = {
	.gpio_enable	= -EINVAL,
	.gpio_backlight	= -EINVAL,
};

int enable_lcd(struct omap_dss_device *dssdev)
{
	/* Bring up backlight */
	gpio_set_value(board_lcd_data.gpio_backlight, 1);

	board_lcd_data.lcd_enabled = 1;
	return 0;
};

void disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(board_lcd_data.gpio_backlight, 0);

	board_lcd_data.lcd_enabled = 0;
};

struct omap_dss_device lcd_device = {
	.name						= "Seiko 37VWFOHZ1",
	.driver_name				= "seiko_panel",
	.type						= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines			= 24,
	.panel.recommended_bpp		= 24,
	.panel.config				= OMAP_DSS_LCD_TFT,
//				| OMAP_DSS_LCD_IVS
//				| OMAP_DSS_LCD_IHS,
	.panel.timings.x_res		= 480,
	.panel.timings.y_res		= 800,
	.panel.timings.pixel_clock 	= 27000,
	.panel.timings.hfp			= 2,
	.panel.timings.hsw			= 2,
	.panel.timings.hbp			= 28,
	.panel.timings.vfp			= 2,
	.panel.timings.vsw			= 2,
	.panel.timings.vbp			= 10,
	.platform_enable			= enable_lcd,
	.platform_disable			= disable_lcd,
};

static struct omap_dss_device *dss_devices[] = {
	&lcd_device,
};

static struct omap_dss_board_info dss_data = {
	.devices	= dss_devices,
	.num_devices	= ARRAY_SIZE(dss_devices),
	.default_device	= &lcd_device,
};

struct platform_device backlight_device = {
	.name 		= "cloudsurfer-backlight",
	.id 		= -1,
};

struct platform_device dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &dss_data,
	},
};

static struct omap2_mcspi_device_config dss_lcd_mcspi_config =
{
	.turbo_mode     = 0,
	.single_channel = 1, /* 0: slave, 1: master */
};

static struct spi_board_info spi_seiko_panel =
{
	.modalias       = "seiko_panel",
	.bus_num        = 3,
	.chip_select    = 0,
	.max_speed_hz   = 500000, 
	.bits_per_word 	= 8,
	.mode         = SPI_MODE_3,
	.controller_data = &dss_lcd_mcspi_config,
};

void __init board_lcd_init(void)
{

	board_lcd_data.gpio_backlight = AIRCELL_BACKLIGHT_ENABLE;
	gpio_set_value(board_lcd_data.gpio_backlight, 0);

	/* Toggle the LCD reset */	
	gpio_set_value(AIRCELL_LCD_RESET, 0);
	mdelay(20);
	gpio_set_value(AIRCELL_LCD_RESET, 1);
	mdelay(20);

	spi_register_board_info(&spi_seiko_panel, 1);

	board_lcd_data.lcd_enabled = 0;

	platform_device_register(&dss_device);

	platform_device_register(&backlight_device);

	return;
}

int display_selection(char *s)
{
	printk("LCD display is %s using %s driver\n", 
			lcd_device.name,lcd_device.driver_name);
	return 0;
}

__setup("display=", display_selection);

