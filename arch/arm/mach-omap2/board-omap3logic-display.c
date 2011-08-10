/*
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
#include "aircell_gpio.h"

struct omap3logic_lcd_data {
	int	gpio_enable;
	int	gpio_backlight;
	int	lcd_enabled;
};

struct omap3logic_display {
	const char *name; /* used to match display setup from cmdline */
	struct omap_dss_device device;
};



static struct omap3logic_lcd_data board_lcd_data = {
	.gpio_enable	= -EINVAL,
	.gpio_backlight	= -EINVAL,
};

static int omap3logic_panel_power_enable(int enable)
{
	int ret;
	struct regulator *vpll2_reg;

	vpll2_reg = regulator_get(NULL, "vpll2");
	if (IS_ERR(vpll2_reg)) {
		pr_err("Unable to get vpll2 regulator\n");
		return PTR_ERR(vpll2_reg);
	}

	if (enable)
		ret = regulator_enable(vpll2_reg);
	else
		ret = regulator_disable(vpll2_reg);

	return ret;
}

int omap3logic_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(board_lcd_data.gpio_enable, 1);

	// Sleep for 300ms since the 4.3" display needs clocks before
	// turning off the syncs after turning off the backlight
	msleep(300);

	/* Bring up backlight */
	gpio_set_value(board_lcd_data.gpio_backlight, 1);

	board_lcd_data.lcd_enabled = 1;

	return 0;
};

void omap3logic_disable_lcd(struct omap_dss_device *dssdev)
{
	int ret;

	ret = omap3logic_panel_power_enable(0);
	if (ret < 0)
		BUG();

	gpio_set_value(board_lcd_data.gpio_backlight, 0);
	gpio_set_value(board_lcd_data.gpio_enable, 0);

	board_lcd_data.lcd_enabled = 0;
};

/* setup by omap3logic_display_selection() below */
struct omap_dss_device omap3logic_lcd_device;

static struct omap_dss_device *omap3logic_dss_devices[] = {
	&omap3logic_lcd_device,
};

static struct omap_dss_board_info omap3logic_dss_data = {
	.devices	= omap3logic_dss_devices,
	.num_devices	= ARRAY_SIZE(omap3logic_dss_devices),
	.default_device	= &omap3logic_lcd_device,
};

struct platform_device omap3logic_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &omap3logic_dss_data,
	},
};

static struct regulator_consumer_supply omap3logic_vpll2_supplies[] = {
	{
		.supply         = "vpll2",
		.dev		= &omap3logic_lcd_device.dev,
	},
	{
		.supply         = "vdds_dsi",
		.dev		= &omap3logic_dss_device.dev,
	}
};

struct regulator_consumer_supply omap3logic_vdac_supply = {
	.supply         = "vdda_dac",
	.dev            = &omap3logic_dss_device.dev,
};

struct regulator_init_data omap3logic_vdac = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
		                        | REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
		                        | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &omap3logic_vdac_supply,
};

struct regulator_init_data omap3logic_vpll2 = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(omap3logic_vpll2_supplies),
	.consumer_supplies      = omap3logic_vpll2_supplies,
};

static struct omap2_mcspi_device_config dss_lcd_mcspi_config =
{
	.turbo_mode     = 0,
	.single_channel = 1, /* 0: slave, 1: master */
};

static struct spi_board_info omap3logic_spi_ls038y7dx01_panel =
{
	.modalias       = "sharp_ls038y7dx01_panel-spi",
	.bus_num        = 3,
	.chip_select    = 0,
	.max_speed_hz   = 307500, //37500,
	//.mode         = SPI_MODE_1,
	.controller_data = &dss_lcd_mcspi_config,
};

void __init board_lcd_init(void)
{
	
	board_lcd_data.gpio_enable = AIRCELL_LCD_POWER_ENABLE;
	board_lcd_data.gpio_backlight = AIRCELL_BACKLIGHT_ENABLE;

	/* Toggle the LCD reset */	
	gpio_set_value(AIRCELL_LCD_RESET, 1);
	mdelay(6);
	gpio_set_value(AIRCELL_LCD_RESET, 0);
	mdelay(3);
	gpio_set_value(AIRCELL_LCD_RESET, 1);
	mdelay(6);

	spi_register_board_info(&omap3logic_spi_ls038y7dx01_panel, 1);

	gpio_set_value(board_lcd_data.gpio_enable, 0);
	gpio_set_value(board_lcd_data.gpio_backlight, 0);

	board_lcd_data.lcd_enabled = 0;

	platform_device_register(&omap3logic_dss_device);

	return;
}



static struct omap3logic_display omap3logic_displays[] = {
	{ 
		.name		= "16", /* 16 == LS038Y7DX01 TFT WQVGA (3.8) Sharp */
		.device	= {
			.name				= "lcd",
			.driver_name		= "sharp_ls038y7dx01_panel",
			.type				= OMAP_DISPLAY_TYPE_DPI,
			.phy.dpi.data_lines	= 16,
			.platform_enable	= omap3logic_enable_lcd,
			.platform_disable	= omap3logic_disable_lcd,
		},
	},
};

static struct omap_dss_device omap3logic_customer_lcd_device = {
	.name						= "lcd",
	.driver_name				= "customer_panel",
	.type						= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines			= 16,
	.panel.recommended_bpp		= 16,
	.panel.config				= OMAP_DSS_LCD_TFT
				| OMAP_DSS_LCD_IVS
				| OMAP_DSS_LCD_IHS,
	.panel.timings.x_res		= 480,
	.panel.timings.y_res		= 272,
	.panel.timings.pixel_clock 	= 10000,
	.panel.timings.hfp			= 1,
	.panel.timings.hsw			= 41,
	.panel.timings.hbp			= 1,
	.panel.timings.vfp			= 1,
	.panel.timings.vsw			= 2,
	.panel.timings.vbp			= 1,
	.platform_enable			= omap3logic_enable_lcd,
	.platform_disable			= omap3logic_disable_lcd,
};

struct omap3logic_custom_display_fields {
	char *field;
	enum {
		lcdt_void	= 0,
		lcdt_u8,
		lcdt_u16,
		lcdt_u32,
		lcdt_int,
		lcdt_opc,
	} cast;
	void *ptr;
};

static struct omap3logic_custom_display_fields
omap3logic_custom_display_fields[] = {
	{"xr",lcdt_u16,&omap3logic_customer_lcd_device.panel.timings.x_res},
	{"yr",lcdt_u16,&omap3logic_customer_lcd_device.panel.timings.y_res},
	{"lm",lcdt_u16,&omap3logic_customer_lcd_device.panel.timings.hbp},
	{"rm",lcdt_u16,&omap3logic_customer_lcd_device.panel.timings.hfp},
	{"tm",lcdt_u16,&omap3logic_customer_lcd_device.panel.timings.vbp},
	{"bm",lcdt_u16,&omap3logic_customer_lcd_device.panel.timings.vfp},
	{"hs",lcdt_u16,&omap3logic_customer_lcd_device.panel.timings.hsw},
	{"vs",lcdt_u16,&omap3logic_customer_lcd_device.panel.timings.vsw},
	{"pc",lcdt_u32,&omap3logic_customer_lcd_device.panel.timings.pixel_clock},
	{"cf",lcdt_opc,&omap3logic_customer_lcd_device.panel.config},
	{"bp",lcdt_u8, &omap3logic_customer_lcd_device.panel.recommended_bpp},
	{"dl",lcdt_u8, &omap3logic_customer_lcd_device.phy.dpi.data_lines},
};

int omap3logic_display_selection(char *s)
{
	int err, i, v;
	char *p, *q;
	int last;
	struct omap3logic_display *disp;
	struct omap3logic_custom_display_fields *f;

	printk("%s: TARR -\n", __FUNCTION__);

	err = 0;
	if (strchr(s, ':')) {

		/* Display is custom specification -- kernel parameter display=

		   Syntax:
		     display=:xr:yr:lm:rm:tm:bm:hs:vs:pc:cf:bp:dl

		   Synopsis:
		     xr	x-resolution
		     yr	y-resolution
		     lm	left margin (horizontal back porch)
		     rm	right margin (horizontal front porch)
		     tm	top margin (vertical back porch)
		     bm	bottom margin (vertical front porch)
		     hs	horizontal synchronizing length
		     vs	vertical synchronizing length
		     pc	pixel clock in kHz
		     cf	display configuration flags (see plat/display.h)
		     bp	bpp - bits per pixel
		     dl	data lines

		   Defaults:
		     xr = 480,	yr = 272,
		     lm = 1,		rm = 1,
		     tm = 1,		bm = 1,
		     hs = 41,	vs = 2,
		     pc = 10000,
		     cf = OMAP_DSS_LCD_TFT
			| OMAP_DSS_LCD_IVS
			| OMAP_DSS_LCD_IHS,
		     bp = 16,	dl = 16

		   Data types:
		     Each value have to be an integer.
		*/

		last = 0;
		p = s;

		for (i = 0, f = omap3logic_custom_display_fields;
		     !last && i < ARRAY_SIZE(omap3logic_custom_display_fields);
		     ++i, ++f) {

			q = strchr(p, ':');
			if (q)
				*q = '\0';
			else
				last = 1;

			if (sscanf(p, "%i", &v) != 1) {
				printk(KERN_ERR "Custom display field %s value "
					"'%s' invalid\n", f->field, p);
				err = 1;
				break;
			}

			switch (f->cast) {

				case lcdt_u8:
					*((u8 *)f->ptr) = v;
					break;

				case lcdt_u16:
					*((u16 *)f->ptr) = v;
					break;

				case lcdt_u32:
					*((u32 *)f->ptr) = v;
					break;

				case lcdt_int:
					*((int *)f->ptr) = v;
					break;

				case lcdt_opc:
					*((enum omap_panel_config *)f->ptr) = v;
					break;

				default:
					err = 1;
					break;
			}

			p = q + 1;
		}

		omap3logic_lcd_device = omap3logic_customer_lcd_device;

	} else {

		for (i = 0, disp = omap3logic_displays;
		     i < ARRAY_SIZE(omap3logic_displays);
		     ++i, ++disp) {

			if (!strcmp(disp->name, s)) {
				omap3logic_lcd_device = disp->device;
				break;
			}

			if (i == ARRAY_SIZE(omap3logic_displays)) {
				printk(KERN_ERR "display='%s' specified "
					"on commandline not found\n", s);
				err = 1;
			}
		}
	}

	if (!err) {

		printk("LCD display selection '%s:%d' in effect\n",
			omap3logic_lcd_device.driver_name,
			omap3logic_lcd_device.phy.dpi.data_lines);
	}

	return err;
}

__setup("display=", omap3logic_display_selection);

