/*
 * pca9626.h - platform data structure for pca9626 led controller
 *
 * Copyright (C) 2011 AirCell - Tarr
 * Lots of stuff from the PCA9626 implementation
 *
 * Copyright (C) 2008 Riku Voipio <riku.voipio@movial.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * Datasheet: http://www.nxp.com/acrobat/datasheets/PCA9626_3.pdf
 *
 */

#ifndef __LINUX_PCA9626_H
#define __LINUX_PCA9626_H

#include <linux/leds.h>
#include <linux/workqueue.h>

#define PCA9626_MAX_LEDS 24

struct pca9626_led {
    u8 id;							/* This is the pin on the controller */
    struct i2c_client *client;		
    struct led_classdev ldev;
};

struct pca9626_platform_data {
	struct pca9626_led leds[PCA9626_MAX_LEDS];
};

#endif /* __LINUX_PCA9626_H */

