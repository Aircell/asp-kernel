
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <plat/mux.h>
#include <plat/board.h>
#include <plat/gpio.h>
#include <plat/omap3logic-productid.h>
#include "mux.h"

#include "cloudsurfer-gpio.h"

static int omap3logic_extern_audio_mute = -EINVAL;

void twl4030_mute(unsigned long data) {
	gpio_set_value(AIRCELL_MUTE, 1);
}
EXPORT_SYMBOL(twl4030_mute);

static void twl_do_unmute(unsigned long data) {
	pr_debug("Unmuting from the timer function\n");
	gpio_set_value(AIRCELL_MUTE, 0);
}

static void twl_do_amp(unsigned long data) {
	pr_debug("Powering the %s amp from the timer function\n", data?"ringer":"earpiece");
	if(data) {
		gpio_set_value(AIRCELL_RINGER_ENABLE, 1);
	} else {
		gpio_set_value(AIRCELL_EARPIECE_ENABLE, 1);
	}	
}
DEFINE_TIMER(mute_timer, twl_do_unmute, 0, 0);
DEFINE_TIMER(amp_timer, twl_do_amp, 0, 0);

void twl4030_unmute_delay(unsigned int delay) {
	mod_timer(&mute_timer, jiffies + delay);
}
EXPORT_SYMBOL(twl4030_unmute_delay);

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
	int headset = 0;
	headset = gpio_get_value(AIRCELL_HEADSET_DETECT);

	pr_debug("%s Ringer set to %d (headset %s)\n", __func__, ringer, headset?"in":"out");

	gpio_set_value(AIRCELL_EARPIECE_ENABLE, 0);
	gpio_set_value(AIRCELL_RINGER_ENABLE, 0);

	amp_timer.expires = jiffies + 50;
	amp_timer.data = ringer;
	add_timer(&amp_timer);

	return 0;
}
EXPORT_SYMBOL(twl4030_set_ext_ringer);



void cloudsurfer_init_audio(void) {
	init_timer(&mute_timer);
	add_timer(&mute_timer);

	init_timer(&amp_timer);
}





