/*
 * linux/arch/arm/mach-omap2/board-dm3730logic-audio.c
 *
 * Copyright (C) 2011 Logic Product Development
 *
 * Audio mute support for DM3730 SOM LV/Torpedo boards
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <plat/mux.h>
#include <plat/board.h>
#include <plat/gpio.h>
#include <plat/dm3730logic-productid.h>
#include "mux.h"

static int dm3730logic_extern_audio_mute = -EINVAL;
static void setup_mute_io_mux(void)
{
	if (gpio_is_valid(dm3730logic_extern_audio_mute)) {
		omap_mux_init_gpio(dm3730logic_extern_audio_mute, OMAP_PIN_OUTPUT);
		if (gpio_request(dm3730logic_extern_audio_mute, "audio mute") < 0) {
			printk(KERN_ERR "Failed to request GPIO%d for twl4030 mute\n",
			       dm3730logic_extern_audio_mute);
			return;
		}
		// Initial value is muted
		gpio_direction_output(dm3730logic_extern_audio_mute, 1);
	}
}

// The following function is used by the SOC code to set digital muting
// on startup/shutdown of the output path (as it comes in pairs, don't
// need to worry about mute showing up in the middle).
static int dm3730logic_audio_ext_enabled = 1;
// 1 = on, 0=mute
void twl4030_set_path_mute(int mute)
{
	if (gpio_is_valid(dm3730logic_extern_audio_mute) && dm3730logic_audio_ext_enabled) {
		gpio_set_value(dm3730logic_extern_audio_mute, !mute);
	}
}
EXPORT_SYMBOL(twl4030_set_path_mute);

int twl4030_set_ext_mute(int mute)
{
	dm3730logic_audio_ext_enabled = mute;
	if (gpio_is_valid(dm3730logic_extern_audio_mute))
		gpio_set_value(dm3730logic_extern_audio_mute, !mute);

	return 0;
}
EXPORT_SYMBOL(twl4030_set_ext_mute);

int twl4030_get_ext_mute(void)
{
	int mute;
	mute = dm3730logic_audio_ext_enabled;
	return mute;
}
EXPORT_SYMBOL(twl4030_get_ext_mute);

void setup_mcbsp2_mux(void)
{
	omap_mux_init_signal("mcbsp2_clkx.mcbsp2_clkx", OMAP_PIN_INPUT);
	omap_mux_init_signal("mcbsp2_dr.mcbsp2_dr", OMAP_PIN_INPUT);
	omap_mux_init_signal("mcbsp2_fsx.mcbsp2_fsx", OMAP_PIN_INPUT);
	omap_mux_init_signal("mcbsp2_dx.mcbsp2_dx", OMAP_PIN_OUTPUT);
}

void dm3730logic_init_twl_audio(void)
{
	/* Mux mcbps2 pins for I2S input/output */
	setup_mcbsp2_mux();

	/* Note that dm3730logic_external_mute is valid if a GPIO pin
	   is used for audio mute */
	dm3730logic_extern_audio_mute = dm3730logic_external_mute_gpio();
	setup_mute_io_mux();
}
