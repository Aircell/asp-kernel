
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <plat/mux.h>
#include <plat/board.h>
#include <plat/gpio.h>
#include <plat/omap3logic-productid.h>
#include "mux.h"

#include "cloudsurfer-gpio.h"

static int omap3logic_extern_audio_mute = -EINVAL;

static void setup_mute_io_mux(void)
{
	gpio_direction_output(omap3logic_extern_audio_mute, 1);

}

int twl4030_get_headset_int(void) 
{
	return AIRCELL_HEADSET_DETECT;
}
EXPORT_SYMBOL(twl4030_get_headset_int);

int twl4030_get_headset_enable(void) 
{
	return AIRCELL_EARPIECE_ENABLE;
}
EXPORT_SYMBOL(twl4030_get_headset_enable);

int twl4030_get_ringer_enable(void) 
{
	return AIRCELL_RINGER_ENABLE;
}
EXPORT_SYMBOL(twl4030_get_ringer_enable);

// The following function is used by the SOC code to set digital muting
// on startup/shutdown of the output path (as it comes in pairs, don't
// need to worry about mute showing up in the middle).
static int omap3logic_audio_ext_enabled = 1;
// 1 = on, 0=mute
void twl4030_set_path_mute(int mute)
{
	if (gpio_is_valid(omap3logic_extern_audio_mute) && omap3logic_audio_ext_enabled) {
		gpio_set_value(omap3logic_extern_audio_mute, !mute);
	}
}
EXPORT_SYMBOL(twl4030_set_path_mute);

int twl4030_set_ext_mute(int mute)
{
	omap3logic_audio_ext_enabled = mute;
	if (gpio_is_valid(omap3logic_extern_audio_mute))
		gpio_set_value(omap3logic_extern_audio_mute, !mute);

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
	/* Note that omap3logic_external_mute is valid if a GPIO pin
	   is used for audio mute */
	//omap3logic_extern_audio_mute = omap3logic_external_mute_gpio();
	setup_mute_io_mux();
}

void omap3logic_init_audio_mux(void)
{
}

int twl4030_get_ext_ringer(void)
{
       return gpio_get_value(AIRCELL_RINGER_ENABLE);
}
EXPORT_SYMBOL(twl4030_get_ext_ringer);


int twl4030_set_ext_ringer(int ringer)
{
       printk(KERN_INFO "%s Ringer set to %d\n", __func__, ringer);
       gpio_set_value(AIRCELL_5VA_ENABLE, 1);
       if(ringer) {
               gpio_set_value(AIRCELL_EARPIECE_ENABLE, 0);
               gpio_set_value(AIRCELL_RINGER_ENABLE, 1);
       } else {
               gpio_set_value(AIRCELL_EARPIECE_ENABLE, 1);
               gpio_set_value(AIRCELL_RINGER_ENABLE, 0);
       }
       return 0;
}
EXPORT_SYMBOL(twl4030_set_ext_ringer);



