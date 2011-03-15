/*
 * Logic OMAP3 board definitions
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

extern void omap3logic_lcd_panel_init(int *p_gpio_enable, int *p_gpio_backlight);

extern void omap3logic_init_twl_external_mute(void);
extern void twl4030_set_ext_mute(int mute);
extern int twl4030_get_ext_mute(void);
extern void twl4030_set_path_mute(int mute);

/* brt6300 Bluetooth define/prototypes */
#define TWL4030_BT_nSHUTDOWN	8
#define BT_IRQ_GPIO		157
#if 0
extern void brf6300_request_bt_nshutdown_gpio(void);
extern void brf6300_free_bt_nshutdown_gpio(void);
extern void brf6300_direction_bt_nshutdown_gpio(int direcction, int value);
extern void brf6300_set_bt_nshutdown_gpio(int value);
extern int brf6300_get_bt_nshutdown_gpio(int value);
#endif
