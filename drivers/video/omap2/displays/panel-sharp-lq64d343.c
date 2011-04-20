/*
 * LCD panel driver for Sharp LQ64D343 -- TFT VGA (6.4)
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

static struct omap_video_timings sharp_lq64d343_timings = {
	.x_res	= 640,
	.y_res	= 480,

	.pixel_clock	= 25180,/* LogicPD: 27000 */

	.hsw		= 96,	/* LogicPD: 48 */
	.hfp		= 32,	/* LogicPD: 24 */
	.hbp		= 32,	/* LogicPD: 135 */

	.vsw		= 1,	/* LogicPD: 1 */
	.vfp		= 22,	/* LogicPD: 34 */
	.vbp		= 22,	/* LogicPD: 34 */
};

static int sharp_lq64d343_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT
			     | OMAP_DSS_LCD_IVS  /* inverted for 480 line mode */
	     		     | OMAP_DSS_LCD_IHS; /* inverted for 480 line mode */
	dssdev->panel.timings = sharp_lq64d343_timings;
	dssdev->panel.acb = 0; /* not used for TFT*/

	return 0;
}

static void sharp_lq64d343_panel_remove(struct omap_dss_device *dssdev)
{
}

static int sharp_lq64d343_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	if (dssdev->platform_enable)
		r = dssdev->platform_enable(dssdev);

	return r;
}

static void sharp_lq64d343_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static int sharp_lq64d343_panel_suspend(struct omap_dss_device *dssdev)
{
	sharp_lq64d343_panel_disable(dssdev);
	return 0;
}

static int sharp_lq64d343_panel_resume(struct omap_dss_device *dssdev)
{
	return sharp_lq64d343_panel_enable(dssdev);
}

static struct omap_dss_driver sharp_lq64d343_driver = {
	.probe		= sharp_lq64d343_panel_probe,
	.remove		= sharp_lq64d343_panel_remove,

	.enable		= sharp_lq64d343_panel_enable,
	.disable	= sharp_lq64d343_panel_disable,
	.suspend	= sharp_lq64d343_panel_suspend,
	.resume		= sharp_lq64d343_panel_resume,

	.driver         = {
		.name   = "sharp_lq64d343_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init sharp_lq64d343_panel_drv_init(void)
{
	return omap_dss_register_driver(&sharp_lq64d343_driver);
}

static void __exit sharp_lq64d343_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&sharp_lq64d343_driver);
}

module_init(sharp_lq64d343_panel_drv_init);
module_exit(sharp_lq64d343_panel_drv_exit);
MODULE_LICENSE("GPL");
