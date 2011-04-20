/*
 * LCD panel driver for Sharp LQ036Q1DA01 -- TFT QVGA (3.6)
 *
 * Copyright (C) 2009 Li-Pro.Net
 * Author: Stephan Linz <linz@li-pro.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>

#include <plat/display.h>

static struct omap_video_timings sharp_lq036q1da01_timings = {
	.x_res	= 320,
	.y_res	= 240,

	.pixel_clock	= 24500,

	.hsw		= 20,
	.hfp		= 20,
	.hbp		= 20,

	.vsw		= 3,
	.vfp		= 3,
	.vbp		= 4,
};

static int sharp_lq036q1da01_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = sharp_lq036q1da01_timings;
	dssdev->panel.acb = 0; /* not used for TFT*/

	return 0;
}

static void sharp_lq036q1da01_panel_remove(struct omap_dss_device *dssdev)
{
}

static int sharp_lq036q1da01_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	if (dssdev->platform_enable)
		r = dssdev->platform_enable(dssdev);

	return r;
}

static void sharp_lq036q1da01_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static int sharp_lq036q1da01_panel_suspend(struct omap_dss_device *dssdev)
{
	sharp_lq036q1da01_panel_disable(dssdev);
	return 0;
}

static int sharp_lq036q1da01_panel_resume(struct omap_dss_device *dssdev)
{
	return sharp_lq036q1da01_panel_enable(dssdev);
}

static struct omap_dss_driver sharp_lq036q1da01_driver = {
	.probe		= sharp_lq036q1da01_panel_probe,
	.remove		= sharp_lq036q1da01_panel_remove,

	.enable		= sharp_lq036q1da01_panel_enable,
	.disable	= sharp_lq036q1da01_panel_disable,
	.suspend	= sharp_lq036q1da01_panel_suspend,
	.resume		= sharp_lq036q1da01_panel_resume,

	.driver         = {
		.name   = "sharp_lq036q1da01_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init sharp_lq036q1da01_panel_drv_init(void)
{
	return omap_dss_register_driver(&sharp_lq036q1da01_driver);
}

static void __exit sharp_lq036q1da01_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&sharp_lq036q1da01_driver);
}

module_init(sharp_lq036q1da01_panel_drv_init);
module_exit(sharp_lq036q1da01_panel_drv_exit);
MODULE_LICENSE("GPL");
