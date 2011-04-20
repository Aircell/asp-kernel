
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <plat/mux.h>
#include <plat/board.h>
#include <plat/gpio.h>
#include <plat/omap3logic-productid.h>
#include "mux.h"

// On the 1011880 boards (and newer), audio mute is on GPIO_177
#define TWL4030_AUDIO_MUTE_GPIO 177

static int omap3logic_has_extern_audio_mute;
static void setup_mute_io_mux(void)
{
	if (omap3logic_has_extern_audio_mute) {
		omap_mux_init_gpio(TWL4030_AUDIO_MUTE_GPIO, OMAP_PIN_OUTPUT);
		if (gpio_request(TWL4030_AUDIO_MUTE_GPIO, "audio mute") < 0) {
			printk(KERN_ERR "Failed to request GPIO%d for twl4030 mute\n",
			       TWL4030_AUDIO_MUTE_GPIO);
			return;
		}
		// Initial value is muted
		gpio_direction_output(TWL4030_AUDIO_MUTE_GPIO, 1);
	}
}

// The following function is used by the SOC code to set digital muting
// on startup/shutdown of the output path (as it comes in pairs, don't
// need to worry about mute showing up in the middle).
static int omap3logic_audio_ext_enabled = 1;
// 1 = on, 0=mute
void twl4030_set_path_mute(int mute)
{
	if (omap3logic_has_extern_audio_mute && omap3logic_audio_ext_enabled) {
		gpio_set_value(TWL4030_AUDIO_MUTE_GPIO, !mute);
	}
}
EXPORT_SYMBOL(twl4030_set_path_mute);

int twl4030_set_ext_mute(int mute)
{
	omap3logic_audio_ext_enabled = mute;
	if (omap3logic_has_extern_audio_mute)
		gpio_set_value(TWL4030_AUDIO_MUTE_GPIO, !mute);

	return 0;
}
EXPORT_SYMBOL(twl4030_set_ext_mute);

int twl4030_get_ext_mute(void)
{
	int mute;
	mute = omap3logic_audio_ext_enabled;
	return mute;
}
EXPORT_SYMBOL(twl4030_get_ext_mute);

void omap3logic_init_twl_external_mute(void)
{
	omap3logic_has_extern_audio_mute = omap3logic_has_external_mute();
	setup_mute_io_mux();
}

