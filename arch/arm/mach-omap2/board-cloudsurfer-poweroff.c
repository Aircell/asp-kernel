/*
 * Aircell - 
 * Updated for the AirCell CloudSurfer
 * Tarr - July 2011
 * Tarr - March 2012 - Support switch to DM3730
 */

/*
 * linux/arch/arm/mach-omap2/board-omap3logic.c
 *
 * Copyright (C) 2010 Logic Product Development
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
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>

#include "cloudsurfer-gpio.h"
#include <plat/board.h>
/*
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio_keys.h>
#include <linux/switch.h>
#include <linux/input/matrix_keypad.h>
#include <linux/leds.h>

#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>

#include <linux/i2c/qt602240_ts.h>
#include <linux/i2c/twl.h>
#include <linux/usb/otg.h>
#include <linux/smsc911x.h>

#include <linux/regulator/machine.h>
#include <linux/usb/android_composite.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include <linux/power/cloudsurfer-charger.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/usb.h>
#include <plat/common.h>
#include <plat/mcspi.h>
#include <plat/clock.h>
#include <plat/omap-pm.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/control.h>
#include <plat/display.h>
#include <plat/board-omap3logic.h>
#include <plat/dm3730logic-productid.h>
#include <plat/omap3logic-cf.h>
#include <plat/wifi_tiwlan.h>
#include <plat/dmtimer.h>

#include "mux.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "omap3-opp.h"
#include "board-cloudsurfer.h"
#include <plat/sdrc.h>
#include <linux/i2c/at24.h>
#include <linux/leds-pca9626.h>
*/

static void cs_suicide_watch(struct work_struct * );
static void cs_suicide_timer(unsigned long data);
DEFINE_TIMER(cs_volume_poweroff_timer, cs_suicide_timer, 0, 0);
static struct work_struct cs_volume_poweroff_work;
static struct input_dev *idev;
static int countdown = -1;

/* This is run from the timer interrupt, this function controls countdown */
static void cs_suicide_timer(unsigned long data) {
	countdown--;
	schedule_work(&cs_volume_poweroff_work);
}

/* This is called from the gpio-keys work queue.  This is write-only on idev,
   which serves as a flag that the countdown is active */
void cs_volume_poweroff(int code, int state, void *data) {

	if(state==1) {
		idev = (struct input_dev *)data; 
		schedule_work(&cs_volume_poweroff_work);
	} else {
		idev = NULL;
		schedule_work(&cs_volume_poweroff_work);
	}	
}

/* This is a work queue that is fed by the timer routine and by the qpio-keys work queue */
/* Manipulate timers only in this function */
static void cs_suicide_watch(struct work_struct *w) {
	static DEFINE_MUTEX(cs_suicide_mutex);

	
	if(idev) { 
		/* Timeout is active */
		switch(countdown) {
		case -1:
			countdown = 13;
			cs_volume_poweroff_timer.expires = jiffies + 800;
			add_timer(&cs_volume_poweroff_timer);
			return;
		case 12:
			printk(KERN_INFO "%s : Sending powerdown key\n", __func__);
			input_event(idev, EV_KEY, KEY_POWER, 1);
			input_sync(idev);
			break;
		case 11:
			input_event(idev, EV_KEY, KEY_POWER, 0);
			input_sync(idev);
			break;
		
		case 1:
			if(gpio_get_value(AIRCELL_BATTERY_POWERED)){
				printk(KERN_INFO "Powering off wifi phone\n");
				gpio_set_value(AIRCELL_BATTERY_CUT_ENABLE, 1);
				
			} else {

				printk(KERN_INFO "Power cycling corded phone\n");
				gpio_set_value(AIRCELL_SOFTWARE_RESET, 0);
			}
			break;
		case 0:
			panic("Forced reset at user request");
		default:
			break;
		}
		del_timer(&cs_volume_poweroff_timer);
		cs_volume_poweroff_timer.expires = jiffies + 100;
		add_timer(&cs_volume_poweroff_timer);
		printk(KERN_INFO "%s : %d\n", __func__, countdown);
	} else {
		/* Countdown aborted */
		printk(KERN_INFO "%s : Countdown stopped\n", __func__);
		if(countdown > -1) 
			del_timer(&cs_volume_poweroff_timer);
		countdown = -1;

	}
	return;	
}


void cs_poweroff_setup(void) {
	init_timer(&cs_volume_poweroff_timer);
	INIT_WORK(&cs_volume_poweroff_work, cs_suicide_watch);
}


