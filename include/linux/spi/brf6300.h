/*
 * BRF6300 bluetooth definitions
 *
 * Copyright: peterb@logicpd.com
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


struct brf6300_platform_data {
	int irq_gpio;		/* IRQ gpio */
	int shutdown_gpio;	/* nSHUTDOWN gpio */
};

#define BT_IRQ_GPIO 157

extern int brf6300_request_bt_nshutdown_gpio(void);
extern void brf6300_free_bt_nshutdown_gpio(void);
extern void brf6300_direction_bt_nshutdown_gpio(int direction, int value);
extern void brf6300_set_bt_nshutdown_gpio(int value);
extern int brf6300_get_bt_nshutdown_gpio(int value);
extern int omap3530lv_som_twl_gpio_setup(struct device *dev,
						unsigned gpio, unsigned ngpio);
