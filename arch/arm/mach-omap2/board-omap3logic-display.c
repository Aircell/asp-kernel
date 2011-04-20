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

#define OMAP3530_LV_SOM_LCD_GPIO_ENA		155
#define OMAP3530_LV_SOM_LCD_GPIO_BL		8

#define OMAP3_TORPEDO_LCD_GPIO_ENA		155
#define OMAP3_TORPEDO_LCD_GPIO_BL		154

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

	printk("%s: Called\n", __FUNCTION__);

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

static int omap3logic_panel_pre_enable_lcd(struct omap_dss_device *dssdev)
{
	struct omap3logic_dss_board_info *pdata;
	int ret;

	printk("%s: Called\n", __FUNCTION__);

	ret = omap3logic_panel_power_enable(1);
	if (ret < 0)
		return ret;

	/* Allow the power to stablize */
	msleep(50);

	pdata = dssdev->dev.platform_data;

	gpio_set_value(board_lcd_data.gpio_enable, 1);

	msleep(300);

	return 0;
}

int omap3logic_enable_lcd(struct omap_dss_device *dssdev)
{
	wait_queue_head_t wait;

	printk("%s: Called\n", __FUNCTION__);

	gpio_set_value(board_lcd_data.gpio_enable, 1);

	// Sleep for 300ms since the 4.3" display needs clocks before
	// turning off the syncs after turning off the backlight
//	init_waitqueue_head (&wait);
//	wait_event_timeout(wait, 0, msecs_to_jiffies(300));
	msleep(300);

	/* Bring up backlight */
	gpio_set_value(board_lcd_data.gpio_backlight, 1);

	board_lcd_data.lcd_enabled = 1;

	return 0;
};

void omap3logic_disable_lcd(struct omap_dss_device *dssdev)
{
	int ret;

	printk("%s: Called\n", __FUNCTION__);

	ret = omap3logic_panel_power_enable(0);
	if (ret < 0)
		BUG();

	gpio_set_value(board_lcd_data.gpio_backlight, 0);
	gpio_set_value(board_lcd_data.gpio_enable, 0);

	board_lcd_data.lcd_enabled = 0;
};

/* setup by omap3logic_display_selection() below */
struct omap_dss_device omap3logic_lcd_device;



static int omap3logic_enable_tv(struct omap_dss_device *dssdev)
{
	return 0;
}

static void omap3logic_disable_tv(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device omap3logic_tv_device = {
	.name			= "tv",
	.driver_name		= "venv",
	.type			= OMAP_DISPLAY_TYPE_VENC,
	.phy.dpi.data_lines	= OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable	= omap3logic_enable_tv,
	.platform_disable	= omap3logic_disable_tv,
};



static struct omap_dss_device *omap3logic_dss_devices[] = {
	&omap3logic_lcd_device,
	&omap3logic_tv_device,
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

void __init board_lcd_init(void)
{

	if (!omap3logic_lcd_device.name) {
		printk(KERN_ERR "No 'display=' specified on commandline!\n");
		return;
	}

	/* OMAP3530 LV SOM board */
	if (machine_is_omap3530_lv_som()) {
		board_lcd_data.gpio_enable = OMAP3530_LV_SOM_LCD_GPIO_ENA;
		board_lcd_data.gpio_backlight = OMAP3530_LV_SOM_LCD_GPIO_BL;
		omap_mux_init_gpio(OMAP3530_LV_SOM_LCD_GPIO_ENA, 
				OMAP_PIN_OUTPUT);
		omap_mux_init_gpio(OMAP3530_LV_SOM_LCD_GPIO_BL,
				OMAP_PIN_OUTPUT);

	/* Torpedo board */
	} else if (machine_is_omap3_torpedo()) {
		board_lcd_data.gpio_enable = OMAP3_TORPEDO_LCD_GPIO_ENA;
		board_lcd_data.gpio_backlight = OMAP3_TORPEDO_LCD_GPIO_BL;
		omap_mux_init_gpio(OMAP3_TORPEDO_LCD_GPIO_ENA, 
				OMAP_PIN_OUTPUT);
		omap_mux_init_gpio(OMAP3_TORPEDO_LCD_GPIO_BL, 
				OMAP_PIN_OUTPUT);

	/* unsupported board */
	} else {
		printk(KERN_ERR "Unknown machine in board_lcd_init()\n");
		return;
	}

	omap_mux_init_signal("dss_pclk", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_hsync", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_vsync", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_acbias", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data1", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data2", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data3", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data4", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data5", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data6", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data7", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data8", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data9", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data10", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data11", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data12", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data13", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data14", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data15", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data16", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data17", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data18", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data19", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data20", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data21", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data22", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data23", OMAP_PIN_OUTPUT);

	if (gpio_request(board_lcd_data.gpio_enable, "LCD enable")) {
		printk(KERN_ERR "failed to get LCD enable on GPIO %d\n",
			board_lcd_data.gpio_enable);
		goto err0;
	}

	if (gpio_request(board_lcd_data.gpio_backlight, "LCD backlight")) {
		printk(KERN_ERR "failed to get LCD backlight on GPIO %d\n",
			board_lcd_data.gpio_backlight);
		goto err1;
	}

	gpio_direction_output(board_lcd_data.gpio_enable, 0);
	gpio_direction_output(board_lcd_data.gpio_backlight, 0);

	board_lcd_data.lcd_enabled = 0;

	platform_device_register(&omap3logic_dss_device);

	return;
err1:
	gpio_free(board_lcd_data.gpio_enable);
err0:
	return;
}



static struct omap3logic_display omap3logic_displays[] = {
	{
		.name		= "2", /* 2 == LQ121S1DG31 TFT SVGA (12.1) Sharp */
		.device	= {
			.name			= "lcd",
			.driver_name		= "sharp_lq121s1dg31_panel",
			.type			= OMAP_DISPLAY_TYPE_DPI,
			.phy.dpi.data_lines	= 16,
			.platform_enable	= omap3logic_enable_lcd,
			.platform_disable	= omap3logic_disable_lcd,
		},
	},
	{
		.name		= "3", /* 3 == LQ036Q1DA01 TFT QVGA (3.6) Sharp w/ASIC */
		.device	= {
			.name			= "lcd",
			.driver_name		= "sharp_lq036q1da01_panel",
			.type			= OMAP_DISPLAY_TYPE_DPI,
			.phy.dpi.data_lines	= 16,
			.platform_enable	= omap3logic_enable_lcd,
			.platform_disable	= omap3logic_disable_lcd,
		},
	},
	{
		.name		= "4", /* 4 == HV056WX1-100 TFT WXGA (5.6) BOEhydis */
		.device	= {
			.name			= "lcd",
			.driver_name		= "boehydis_hc056wx1_100_panel",
			.type			= OMAP_DISPLAY_TYPE_DPI,
			.phy.dpi.data_lines	= 16,
			.platform_enable	= omap3logic_enable_lcd,
			.platform_disable	= omap3logic_disable_lcd,
		},
	},
	{
		.name		= "5", /* 5 == LQ64D343 TFT VGA (6.4) Sharp */
		.device	= {
			.name			= "lcd",
			.driver_name		= "sharp_lq64d343_panel",
			.type			= OMAP_DISPLAY_TYPE_DPI,
			.phy.dpi.data_lines	= 16,
			.platform_enable	= omap3logic_enable_lcd,
			.platform_disable	= omap3logic_disable_lcd,
		},
	},
	{
		.name		= "7", /* 7 == LQ10D368 TFT VGA (10.4) Sharp */
		.device	= {
			.name			= "lcd",
			.driver_name		= "sharp_lq10d368_panel",
			.type			= OMAP_DISPLAY_TYPE_DPI,
			.phy.dpi.data_lines	= 16,
			.platform_enable	= omap3logic_enable_lcd,
			.platform_disable	= omap3logic_disable_lcd,
		},
	},
	{
		.name		= "15", /* 15 == LQ043T1DG01 TFT WQVGA (4.3) Sharp */
		.device	= {
			.name			= "lcd",
			.driver_name		= "sharp_lq043t1dg01_panel",
			.type			= OMAP_DISPLAY_TYPE_DPI,
			.phy.dpi.data_lines	= 16,
			.platform_pre_enable    = omap3logic_panel_pre_enable_lcd,
			.platform_enable	= omap3logic_enable_lcd,
			.platform_disable	= omap3logic_disable_lcd,
		},
	},
};

static struct omap_dss_device omap3logic_customer_lcd_device = {
	.name			= "lcd",
	.driver_name		= "customer_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 16,
	.panel.recommended_bpp	= 16,
	.panel.config		= OMAP_DSS_LCD_TFT
				| OMAP_DSS_LCD_IVS
				| OMAP_DSS_LCD_IHS,
	.panel.timings.x_res	= 480,
	.panel.timings.y_res	= 272,
	.panel.timings.pixel_clock = 10000,
	.panel.timings.hfp	= 1,
	.panel.timings.hsw	= 41,
	.panel.timings.hbp	= 1,
	.panel.timings.vfp	= 1,
	.panel.timings.vsw	= 2,
	.panel.timings.vbp	= 1,
	.platform_enable	= omap3logic_enable_lcd,
	.platform_disable	= omap3logic_disable_lcd,
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

