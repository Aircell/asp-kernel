/*
 * linux/arch/arm/mach-omap2/board-dm3730logic-display.c
 *
 * Copyright (C) 2011 Logic Product Development
 *
 * Modified from mach-omap2/board-omap3evm.c
 *
 * Initial code: Peter Barada
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
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/backlight.h>
#include <linux/regulator/machine.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>

#include <plat/control.h>
#include <plat/display.h>
#include <plat/board-dm3730logic.h>
#include "board-dm3730logic.h"
#include <plat/dmtimer.h>

#include "mux.h"
#include "mux34xx.h"

struct dm3730logic_dss_board_info {
	int gpio_flag;
	int lcd_gpio_enable;
	int lcd_gpio_backlight;
	int lcd_enabled;
	int dvi_enabled;
	int syncs_as_gpio;   /* !0 if hsync/vsync are gpio pins */
};

#if 0
#define LCDPRINTK(fmt, args...) printk(KERN_INFO fmt, ## args)
#else
#define LCDPRINTK(fmt, args...)
#endif

#if defined(CONFIG_OMAP2_DSS) || defined(CONFIG_OMAP2_DSS_MODULE)
static struct {
	struct mutex lock;
	int enabled;
	int intensity;
} dm3730logic_bl_data;

static struct dm3730logic_dss_board_info dm3730logic_dss_lcd_data = {
	.gpio_flag = 0,
};

#define MIN_BRIGHTNESS 45
#define MAX_BRIGHTNESS 255

#define MIN_SOM_LV_BRIGHTNESS 100
#define MAX_SOM_LV_BRIGHTNESS 127

struct omap_dss_device dm3730logic_lcd_device;

/* SOM LV Backlight intensity function */

/*
 * PWMA/B register offsets (TWL4030_MODULE_PWMA)
 */
#define TWL_INTBR_PMBR1	0xD
#define TWL_INTBR_GPBR1	0xC
#define TWL_LED_PWMON	0x0
#define TWL_LED_PWMOFF	0x1

static struct generic_bl_info dm3730logic_bl_info = {
	.name			= "dm3730logic",
	.max_intensity		= MAX_BRIGHTNESS,
	.default_intensity	= (MAX_BRIGHTNESS * 7) / 10,
	.set_bl_intensity	= NULL,
};

static struct platform_device dm3730logic_bl_device = {
	.name			= "generic-bl",
	.id			= 1,
	.dev = {
		.platform_data = &dm3730logic_bl_info,
	},
};

static void dm3730_som_lv_bl_set_intensity(int level)
{
	struct DM3730logic_dss_board_info *pdata;
	unsigned char c;

	u8 mux_pwm, enb_pwm;

	pdata = dm3730logic_lcd_device.dev.platform_data;

	LCDPRINTK("%s: level %d\n", __FUNCTION__, level);
	if (level > MAX_BRIGHTNESS)
		return;

	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &mux_pwm, TWL_INTBR_PMBR1);
	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &enb_pwm, TWL_INTBR_GPBR1);

	if (level == 0) {
		/* disable pwm0 output and clock */
		enb_pwm = enb_pwm & 0xFA;
		/* change pwm0 pin to gpio pin */
		mux_pwm = mux_pwm & 0xF3;
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					enb_pwm, TWL_INTBR_GPBR1);
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					mux_pwm, TWL_INTBR_PMBR1);
		LCDPRINTK("%s: turn off GPIO_%d as backlight!\n", __FUNCTION__, dm3730logic_dss_lcd_data.lcd_gpio_backlight);
		/* Turn off the backlight! */
		gpio_set_value(dm3730logic_dss_lcd_data.lcd_gpio_backlight, 0);

		return;
	}

	LCDPRINTK("%s: enb_pwm %02x mux_pwm %02x\n", __FUNCTION__, enb_pwm, mux_pwm);
	if (((enb_pwm & 0x5) != 0x5) || ((mux_pwm & 0x0c) != 0x4)) {
		/* change gpio pin to pwm0 pin */
		mux_pwm = (mux_pwm & 0xc) | 0x04;
		/* enable pwm0 output and clock*/
		enb_pwm = (enb_pwm & 0x5) | 0x05;
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					mux_pwm, TWL_INTBR_PMBR1);
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					enb_pwm, TWL_INTBR_GPBR1);
		LCDPRINTK("%s: turn on GPIO_%d as backlight!\n", __FUNCTION__, dm3730logic_dss_lcd_data.lcd_gpio_backlight);
		/* Turn on the backlight! */
		gpio_set_value(dm3730logic_dss_lcd_data.lcd_gpio_backlight, 1);
	}

	/* 100 -> 1, 1 -> 127 */
	c = MIN_SOM_LV_BRIGHTNESS - 
		(level * (MAX_SOM_LV_BRIGHTNESS -1) / MAX_SOM_LV_BRIGHTNESS);

	LCDPRINTK("%s: c %d (%d%% on)\n", __FUNCTION__, c, (((MAX_BRIGHTNESS+1)-c) * 100)/(MAX_BRIGHTNESS+1));
	twl_i2c_write_u8(TWL4030_MODULE_PWM0, 0x7F, TWL_LED_PWMOFF);
	twl_i2c_write_u8(TWL4030_MODULE_PWM0, c, TWL_LED_PWMON);
	return;
}

static struct omap_dm_timer *intensity_timer;

/* TORPEDO Backlight intensity function */
static void dm3730_torpedo_bl_set_intensity(int level)
{
	int m, c;
	LCDPRINTK("%s: level %d (%d%% on)\n", __FUNCTION__, level, (level * 100)/ MAX_BRIGHTNESS);

	/*
	 * Scale values with y = mx+c
	 * where
	 *   m = (MAX - MIN) / (MAX_incoming - MIN_incoming)
	 *   c = MIN - MIN_incoming * m
	 *
	 */

	if (level == 0) {
		LCDPRINTK("%s: turn off GPIO_%d as backlight!\n", __FUNCTION__, dm3730logic_dss_lcd_data.lcd_gpio_backlight);
		/* Turn off the backlight! */
		gpio_set_value(dm3730logic_dss_lcd_data.lcd_gpio_backlight, 0);
		omap_dm_timer_disable(intensity_timer);
	} else {
		LCDPRINTK("%s: turn on GPIO_%d as backlight!\n", __FUNCTION__, dm3730logic_dss_lcd_data.lcd_gpio_backlight);
		/* Turn on the backlight! */
		gpio_set_value(dm3730logic_dss_lcd_data.lcd_gpio_backlight, 1);

		omap_dm_timer_enable(intensity_timer);
		omap_dm_timer_set_source(intensity_timer, OMAP_TIMER_SRC_SYS_CLK);
		omap_dm_timer_set_load(intensity_timer, 1, 0xFFFF0000);
		omap_dm_timer_set_pwm(intensity_timer, 0, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);

		m = (MAX_BRIGHTNESS - MIN_BRIGHTNESS)/(100 - 7);
		c = MIN_BRIGHTNESS - 7 * m;
		level = m * level + c;

		LCDPRINTK("%s: Scaled level %d (%d%%)\n", __FUNCTION__, level, (level * 100) / MAX_BRIGHTNESS);

		omap_dm_timer_set_match(intensity_timer, 1,
					(0xffff0000) | (level<<8));
		omap_dm_timer_start(intensity_timer);
	}
}

static void dm3730logic_do_update(int level)
{
	if (machine_is_dm3730_som_lv())
		dm3730_som_lv_bl_set_intensity(level);
	else
		dm3730_torpedo_bl_set_intensity(level);
}

/* Common function called to set the backlight; defer if need be since
 * dm3730logic_bl_update() will handle it later */
static void dm3730logic_do_bl_intensity(int level)
{
	mutex_lock(&dm3730logic_bl_data.lock);
	dm3730logic_bl_data.intensity = level;
	if (dm3730logic_bl_data.enabled)
		dm3730logic_do_update(level);
	mutex_unlock(&dm3730logic_bl_data.lock);
}

static void dm3730logic_enable_backlight(void)
{
	mutex_lock(&dm3730logic_bl_data.lock);
	if (!dm3730logic_bl_data.enabled) {
		dm3730logic_bl_data.enabled = 1;
		if (dm3730logic_bl_data.intensity) {
			/* Update the backlight if turned on */
			dm3730logic_do_update(dm3730logic_bl_data.intensity);
		}
	}
	mutex_unlock(&dm3730logic_bl_data.lock);
}

static void dm3730logic_disable_backlight(void)
{
	mutex_lock(&dm3730logic_bl_data.lock);
	if (dm3730logic_bl_data.enabled) {
		if (dm3730logic_bl_data.intensity) {
			printk(KERN_ERR "%s: backlight (%d) is still enabled and non-zero!\n", __FUNCTION__, dm3730logic_bl_data.intensity);
			dm3730logic_do_update(0);
		}
		dm3730logic_bl_data.enabled = 0;
	}
	mutex_unlock(&dm3730logic_bl_data.lock);
}

static int dm3730logic_panel_power_enable(int enable)
{
	int ret;
	struct regulator *vpll2_reg;

	LCDPRINTK("%s: enable %d\n", __FUNCTION__, enable);

	vpll2_reg = regulator_get(NULL, "vdvi");
	if (IS_ERR(vpll2_reg)) {
		pr_err("Unable to get VDVI regulator\n");
		return PTR_ERR(vpll2_reg);
	}

	if (enable)
		ret = regulator_enable(vpll2_reg);
	else
		ret = regulator_disable(vpll2_reg);

	return ret;
}

static int dm3730logic_panel_pre_enable_lcd(struct omap_dss_device *dssdev)
{
	struct dm3730logic_dss_board_info *pdata;
	int ret;

	LCDPRINTK("%s: dssdev %p\n", __FUNCTION__, dssdev);

	pdata = dm3730logic_lcd_device.dev.platform_data;

	if (pdata->dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}

	/* if hsync/vsync are GPIOs then release them */
	if (pdata->syncs_as_gpio) {
		printk(KERN_INFO "Remux DSS sync pins as DSS sync pins\n");
		gpio_free(66);
		gpio_free(67);
		gpio_free(68);
		gpio_free(69);
		gpio_free(70);
		gpio_free(71);
		gpio_free(72);
		gpio_free(73);
		gpio_free(74);
		gpio_free(75);
		gpio_free(76);
		gpio_free(77);
		gpio_free(78);
		gpio_free(79);
		gpio_free(80);
		gpio_free(81);
		gpio_free(82);
		gpio_free(83);
		gpio_free(84);
		gpio_free(85);
		gpio_free(86);
		gpio_free(87);
		gpio_free(88);
		gpio_free(89);
		gpio_free(90);
		gpio_free(91);
		gpio_free(92);
		gpio_free(93);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_PCLK_OFFSET);
		omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_HSYNC_OFFSET);
		omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_VSYNC_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_ACBIAS_OFFSET);
		if (machine_is_dm3730_torpedo()) {
			omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA0_OFFSET);
			omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA1_OFFSET);
			omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA2_OFFSET);
			omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA3_OFFSET);
			omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA4_OFFSET);
			omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA5_OFFSET);
		} else {
			omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA0_OFFSET);
			omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA1_OFFSET);
			omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA2_OFFSET);
			omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA3_OFFSET);
			omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA4_OFFSET);
			omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA5_OFFSET);
		}
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA6_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA7_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA8_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA9_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA10_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA11_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA12_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA13_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA14_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE0, OMAP3_CONTROL_PADCONF_DSS_DATA15_OFFSET);
		omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA16_OFFSET);
		omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA17_OFFSET);
		if (machine_is_dm3730_torpedo()) {
			omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE3, OMAP3_CONTROL_PADCONF_DSS_DATA18_OFFSET);
			omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE3, OMAP3_CONTROL_PADCONF_DSS_DATA19_OFFSET);
			omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE3, OMAP3_CONTROL_PADCONF_DSS_DATA20_OFFSET);
			omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE3, OMAP3_CONTROL_PADCONF_DSS_DATA21_OFFSET);
			omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE3, OMAP3_CONTROL_PADCONF_DSS_DATA22_OFFSET);
			omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE3, OMAP3_CONTROL_PADCONF_DSS_DATA23_OFFSET);
		} else {
			omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA18_OFFSET);
			omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA19_OFFSET);
			omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA20_OFFSET);
			omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA21_OFFSET);
			omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA22_OFFSET);
			omap_mux_write(OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE7, OMAP3_CONTROL_PADCONF_DSS_DATA23_OFFSET);
		}
		pdata->syncs_as_gpio = 0;
	}

	ret = dm3730logic_panel_power_enable(1);
	if (ret < 0)
		return ret;

	/* Allow the power to stablize */
	msleep(50);

	pdata = dssdev->dev.platform_data;

	gpio_set_value(pdata->lcd_gpio_enable, 1);

	msleep(300);

	pdata->lcd_enabled = 1;

	return 0;
}

static int dm3730logic_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	struct dm3730logic_dss_board_info *pdata;

	LCDPRINTK("%s: dssdev %p\n", __FUNCTION__, dssdev);

	pdata = dssdev->dev.platform_data;

	/* Sleep for 300ms since the 4.3" display needs
	 * power before turning on the clocks */
	msleep(300);

	/* Bring up backlight */

	pdata->lcd_enabled = 1;

	dm3730logic_enable_backlight();

	return 0;
}

static void dm3730logic_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	struct dm3730logic_dss_board_info *pdata;
	int ret;

	LCDPRINTK("%s: dssdev %p\n", __FUNCTION__, dssdev);

	pdata = dssdev->dev.platform_data;

	/* disable the display */
	pdata->lcd_enabled = 0;

	/* prevent backlight updates from adjusting backlight */
	dm3730logic_disable_backlight();

	ret = dm3730logic_panel_power_enable(0);
	if (ret < 0)
		BUG();
	
	/* By now the panel enable code must have been called */
	if (!pdata->gpio_flag)
		BUG();


	gpio_set_value(pdata->lcd_gpio_enable, 0);

	/* Remux HSYNC/VSYNC to be GPIO's at low level */
	if (!pdata->syncs_as_gpio) {
		printk(KERN_INFO "Remux DSS sync pins as gpio and pull low\n");
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_PCLK_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_HSYNC_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_VSYNC_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_ACBIAS_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA0_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA1_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA2_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA3_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA4_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA5_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA6_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA7_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA8_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA9_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA10_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA11_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA12_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA13_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA14_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA15_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA16_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA17_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA18_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA19_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA20_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA21_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA22_OFFSET);
		omap_mux_write(OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW | OMAP_MUX_MODE4, OMAP3_CONTROL_PADCONF_DSS_DATA23_OFFSET);
		if (gpio_request(66, "DSS_PCLK") < 0)
			printk(KERN_ERR "Failed to request GPIO_66 for dss_hsync\n");
		else
			gpio_direction_output(66, 0);

		if (gpio_request(67, "DSS_HSYNC") < 0)
			printk(KERN_ERR "Failed to request GPIO_67 for dss_hsync\n");
		else
			gpio_direction_output(67, 0);

		if (gpio_request(68, "DSS_VSYNC") < 0)
			printk(KERN_ERR "Failed to request GPIO_68 for dss_vsync\n");
		else
			gpio_direction_output(68, 0);

		if (gpio_request(69, "DSS_ACBIAS") < 0)
			printk(KERN_ERR "Failed to request GPIO_69 for dss_hsync\n");
		else
			gpio_direction_output(69, 0);

		if (gpio_request(70, "DSS_DATA0") < 0)
			printk(KERN_ERR "Failed to request GPIO_70 for dss_hsync\n");
		else
			gpio_direction_output(70, 0);

		if (gpio_request(71, "DSS_DATA1") < 0)
			printk(KERN_ERR "Failed to request GPIO_71 for dss_hsync\n");
		else
			gpio_direction_output(71, 0);

		if (gpio_request(72, "DSS_DATA2") < 0)
			printk(KERN_ERR "Failed to request GPIO_72 for dss_hsync\n");
		else
			gpio_direction_output(72, 0);

		if (gpio_request(73, "DSS_DATA3") < 0)
			printk(KERN_ERR "Failed to request GPIO_73 for dss_hsync\n");
		else
			gpio_direction_output(73, 0);

		if (gpio_request(74, "DSS_DATA4") < 0)
			printk(KERN_ERR "Failed to request GPIO_74 for dss_hsync\n");
		else
			gpio_direction_output(74, 0);

		if (gpio_request(75, "DSS_DATA5") < 0)
			printk(KERN_ERR "Failed to request GPIO_75 for dss_hsync\n");
		else
			gpio_direction_output(75, 0);

		if (gpio_request(76, "DSS_DATA6") < 0)
			printk(KERN_ERR "Failed to request GPIO_76 for dss_hsync\n");
		else
			gpio_direction_output(76, 0);

		if (gpio_request(77, "DSS_DATA7") < 0)
			printk(KERN_ERR "Failed to request GPIO_77 for dss_hsync\n");
		else
			gpio_direction_output(77, 0);

		if (gpio_request(78, "DSS_DATA8") < 0)
			printk(KERN_ERR "Failed to request GPIO_78 for dss_hsync\n");
		else
			gpio_direction_output(78, 0);

		if (gpio_request(79, "DSS_DATA9") < 0)
			printk(KERN_ERR "Failed to request GPIO_79 for dss_hsync\n");
		else
			gpio_direction_output(79, 0);

		if (gpio_request(80, "DSS_DATA10") < 0)
			printk(KERN_ERR "Failed to request GPIO_80 for dss_hsync\n");
		else
			gpio_direction_output(80, 0);

		if (gpio_request(81, "DSS_DATA11") < 0)
			printk(KERN_ERR "Failed to request GPIO_81 for dss_hsync\n");
		else
			gpio_direction_output(81, 0);

		if (gpio_request(82, "DSS_DATA12") < 0)
			printk(KERN_ERR "Failed to request GPIO_82 for dss_hsync\n");
		else
			gpio_direction_output(82, 0);

		if (gpio_request(83, "DSS_DATA13") < 0)
			printk(KERN_ERR "Failed to request GPIO_83 for dss_hsync\n");
		else
			gpio_direction_output(83, 0);

		if (gpio_request(84, "DSS_DATA14") < 0)
			printk(KERN_ERR "Failed to request GPIO_84 for dss_hsync\n");
		else
			gpio_direction_output(84, 0);

		if (gpio_request(85, "DSS_DATA15") < 0)
			printk(KERN_ERR "Failed to request GPIO_85 for dss_hsync\n");
		else
			gpio_direction_output(85, 0);

		if (gpio_request(86, "DSS_DATA16") < 0)
			printk(KERN_ERR "Failed to request GPIO_86 for dss_hsync\n");
		else
			gpio_direction_output(86, 0);

		if (gpio_request(87, "DSS_DATA17") < 0)
			printk(KERN_ERR "Failed to request GPIO_87 for dss_hsync\n");
		else
			gpio_direction_output(87, 0);

		if (gpio_request(88, "DSS_DATA18") < 0)
			printk(KERN_ERR "Failed to request GPIO_88 for dss_hsync\n");
		else
			gpio_direction_output(88, 0);

		if (gpio_request(89, "DSS_DATA19") < 0)
			printk(KERN_ERR "Failed to request GPIO_89 for dss_hsync\n");
		else
			gpio_direction_output(89, 0);

		if (gpio_request(90, "DSS_DATA20") < 0)
			printk(KERN_ERR "Failed to request GPIO_90 for dss_hsync\n");
		else
			gpio_direction_output(90, 0);

		if (gpio_request(91, "DSS_DATA21") < 0)
			printk(KERN_ERR "Failed to request GPIO_91 for dss_hsync\n");
		else
			gpio_direction_output(91, 0);

		if (gpio_request(92, "DSS_DATA22") < 0)
			printk(KERN_ERR "Failed to request GPIO_92 for dss_hsync\n");
		else
			gpio_direction_output(92, 0);

		if (gpio_request(93, "DSS_DATA23") < 0)
			printk(KERN_ERR "Failed to request GPIO_93 for dss_hsync\n");
		else
			gpio_direction_output(93, 0);
		pdata->syncs_as_gpio = 1;
	}

	pdata->dvi_enabled = 0;
}


struct omap_dss_device dm3730logic_lcd_device = {
	.name			= "lcd",
	.driver_name		= "dm3730logic_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= -EINVAL,
	.platform_pre_enable	= dm3730logic_panel_pre_enable_lcd,
	.platform_enable	= dm3730logic_panel_enable_lcd,
	.platform_disable	= dm3730logic_panel_disable_lcd,
	.dev = {
		.platform_data = &dm3730logic_dss_lcd_data,
	},
};

static int dm3730logic_enable_tv(struct omap_dss_device *dssdev)
{
	printk("%s:%d\n", __FUNCTION__, __LINE__);
	return 0;
}

static void dm3730logic_disable_tv(struct omap_dss_device *dssdev)
{
	printk("%s:%d\n", __FUNCTION__, __LINE__);
}

static struct omap_dss_device dm3730logic_tv_device = {
	.name			= "tv",
	.driver_name		= "venc",
	.type			= OMAP_DISPLAY_TYPE_VENC,
#if defined(CONFIG_OMAP2_VENC_OUT_TYPE_SVIDEO)
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
#elif defined(CONFIG_OMAP2_VENC_OUT_TYPE_COMPOSITE)
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_COMPOSITE,
#endif
	.platform_enable	= dm3730logic_enable_tv,
	.platform_disable	= dm3730logic_disable_tv,
};

static int dm3730logic_enable_dvi(struct omap_dss_device *dssdev)
{
	struct dm3730logic_dss_board_info *pdata;
	pdata = dssdev->dev.platform_data;

	printk("%s:%d\n", __FUNCTION__, __LINE__);
	if (pdata->lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}

	pdata->dvi_enabled = 1;
	return 0;
}

static void dm3730logic_disable_dvi(struct omap_dss_device *dssdev)
{
	struct dm3730logic_dss_board_info *pdata;
	pdata = dssdev->dev.platform_data;

	printk("%s:%d\n", __FUNCTION__, __LINE__);
	pdata->dvi_enabled = 0;
}

static struct omap_dss_device dm3730logic_dvi_device = {
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.platform_enable	= dm3730logic_enable_dvi,
	.platform_disable	= dm3730logic_disable_dvi,
};

static struct omap_dss_device *dm3730logic_dss_devices[] = {
	&dm3730logic_lcd_device,
	&dm3730logic_tv_device,
	&dm3730logic_dvi_device,
};

static struct omap_dss_board_info dm3730logic_dss_data = {
	.num_devices	= ARRAY_SIZE(dm3730logic_dss_devices),
	.devices	= dm3730logic_dss_devices,
	.default_device	= &dm3730logic_lcd_device,
};

struct platform_device dm3730logic_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &dm3730logic_dss_data,
	},
};

static int force_old_dss_pinmux;
static int __init dm3730_force_old_dss_pinmux(char *str)
{
	force_old_dss_pinmux = 1;
	return 1;
}

__setup("old_dss_pinmux", dm3730_force_old_dss_pinmux);

static void dm3730logic_lcd_set_panel_mux(void)
{
	struct dm3730logic_dss_board_info *pdata;

	pdata = dm3730logic_lcd_device.dev.platform_data;

	pdata->lcd_gpio_enable = 155;
	if (machine_is_dm3730_som_lv()) {
		pdata->lcd_gpio_backlight = 8;
	} else if (machine_is_dm3730_torpedo()) {
		pdata->lcd_gpio_backlight = 154;
		omap_mux_init_signal("gpmc_ncs5.gpt10_pwm_evt", OMAP_PIN_INPUT_PULLDOWN);
	} else
		BUG();

	if (gpio_request(pdata->lcd_gpio_backlight, "LCD backlight"))
		pr_err("dm3730logic: can't request GPIO %d for LCD backlight\n", pdata->lcd_gpio_backlight);
	gpio_direction_output(pdata->lcd_gpio_backlight, 0);
	omap_mux_init_gpio(pdata->lcd_gpio_backlight, OMAP_PIN_OUTPUT);
	if (gpio_request(pdata->lcd_gpio_enable, "LCD enable"))
		pr_err("dm3730logic: can't request GPIO %d for LCD enable\n", pdata->lcd_gpio_enable);
	gpio_direction_output(pdata->lcd_gpio_enable, 0);
	omap_mux_init_gpio(pdata->lcd_gpio_enable, OMAP_PIN_OUTPUT);
	pdata->gpio_flag = 1;

	/* Mux common DSS signals */
	omap_mux_init_signal("dss_pclk.dss_pclk", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_hsync.dss_hsync", OMAP_PIN_INPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_vsync.dss_vsync", OMAP_PIN_INPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_acbias.dss_acbias", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);

	/* Mux the differing DSS signals (DATA0-DATA5). We assume
	   the Torpedo uses the "new" mapping.  If overridden then use the
	   old mapping */
	if (force_old_dss_pinmux || machine_is_dm3730_som_lv()) {
		omap_mux_init_signal("dss_data0.dss_data0", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data1.dss_data1", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data2.dss_data2", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data3.dss_data3", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data4.dss_data4", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data5.dss_data5", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	} else {
		omap_mux_init_signal("dss_data18.dss_data0", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data19.dss_data1", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data20.dss_data2", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data21.dss_data3", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data22.dss_data4", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
		omap_mux_init_signal("dss_data23.dss_data5", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	}

	omap_mux_init_signal("dss_data6.dss_data6", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data7.dss_data7", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data8.dss_data8", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data9.dss_data9", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data10.dss_data10", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data11.dss_data11", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data12.dss_data12", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data13.dss_data13", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data14.dss_data14", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dss_data15.dss_data15", OMAP_PIN_OUTPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW);

	if (dm3730logic_default_panel.data_lines == 16)
		return;

	/* DSS_DATA16/17 is common to SOM LV/Torpedo */
	omap_mux_init_signal("dss_data16.dss_data16", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_data17.dss_data17", OMAP_PIN_OUTPUT);

	/* Mux the HDMI signals */
	if (machine_is_dm3730_som_lv()) {
		printk(KERN_ERR "Warning: muxing DSS18-23 for SOM LV!\n");
		omap_mux_init_signal("dss_data18.dss_data18", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("dss_data19.dss_data19", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("dss_data20.dss_data20", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("dss_data21.dss_data21", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("dss_data22.dss_data22", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("dss_data23.dss_data23", OMAP_PIN_OUTPUT);
	} else if (machine_is_dm3730_torpedo()) {
		omap_mux_init_signal("sys_boot0.dss_data18", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("sys_boot1.dss_data19", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("sys_boot3.dss_data20", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("sys_boot4.dss_data21", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("sys_boot5.dss_data22", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("sys_boot6.dss_data23", OMAP_PIN_OUTPUT);
	} else
		BUG();
}

void dm3730logic_lcd_init(void)
{
	int result;

	/* If no valid "display=" was specified then no video wanted */
	if (dm3730logic_default_panel.name) {
		dm3730logic_lcd_set_panel_mux();

		printk("%s: name %s data_lines %d\n", __FUNCTION__, 
			dm3730logic_default_panel.name,
			dm3730logic_default_panel.data_lines);
		dm3730logic_lcd_device.name = dm3730logic_default_panel.name;
		dm3730logic_lcd_device.phy.dpi.data_lines = dm3730logic_default_panel.data_lines;
		result = platform_device_register(&dm3730logic_dss_device);
		if (result)
			printk("%s: platform device register of DSS2 device failed: %d\n", __FUNCTION__, result);
		mutex_init(&dm3730logic_bl_data.lock);
		if (machine_is_dm3730_som_lv()) {
			dm3730logic_bl_info.set_bl_intensity = dm3730logic_do_bl_intensity;
		} else if (machine_is_dm3730_torpedo()) {
			intensity_timer = omap_dm_timer_request_specific(10);
			if (intensity_timer == NULL) {
				printk("%s: can't get backlight timer!\n", __FUNCTION__);
			} else {
				omap_dm_timer_disable(intensity_timer);
				dm3730logic_bl_info.set_bl_intensity = dm3730logic_do_bl_intensity;
			}
		} else
			BUG();

#if defined(CONFIG_BACKLIGHT_GENERIC)
		if (machine_is_dm3730_som_lv()) {
			/* Override the default intensity for the SOM LV, since
			 * it uses a different range than the Torpedo.
			 */
			dm3730logic_bl_info.default_intensity =
				(MAX_SOM_LV_BRIGHTNESS * 7) / 10;
		}
		result = platform_device_register(&dm3730logic_bl_device);
		if (result)
			printk("%s: platform device register of backlight device failed: %d\n", __FUNCTION__, result);
#endif
	}
}
#else
void dm3730logic_lcd_init(void)
{
}
#endif
