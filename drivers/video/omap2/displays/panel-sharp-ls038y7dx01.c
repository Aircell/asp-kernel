/*
 * LCD panel driver for Sharp LS037V7DW01
 *
 * Copyright (C) 2011 Logic PD
 * Author: Mark Chung <mark.chung@logicpd.com>
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
#include <linux/device.h>
#include <linux/spi/spi.h> // mark.chung
#include <linux/err.h>

#include <plat/display.h>


static struct omap_video_timings sharp_ls038y7dx01_timings = {
	.x_res = 480,
	.y_res = 800,

	.pixel_clock	= 21000,

	.hsw		= 10,
	.hfp		= 10,
	.hbp		= 20,

	.vsw		= 2,
	.vfp		= 2,
	.vbp		= 2,
};

static int sharp_ls038y7dx01_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	//dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
	//	OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF;
	//dssdev->panel.acb = 0x28;
	dssdev->panel.timings = sharp_ls038y7dx01_timings;

	return 0;
}

static void sharp_ls038y7dx01_panel_remove(struct omap_dss_device *dssdev)
{
}

static int ls038y7dx01_write_reg(struct spi_device *spidev, unsigned char reg, unsigned char val) 
{
	int ret = 0;
    struct spi_message msg;
    struct spi_transfer index_xfer = { 
        .len        = 2,
        .cs_change  = 1,
    };
  
	unsigned short buffer = 0;
    spi_message_init(&msg);

	buffer = (reg << 8) | val;
    index_xfer.tx_buf = &buffer;
    spi_message_add_tail(&index_xfer, &msg);

    if (ret = spi_sync(spidev, &msg)) {
		printk(KERN_ERR "spi_sync failed: %d\n", ret);
	}
	
	mdelay(5);
	return ret;
}

static int init_sharp_ls038y7dx01_panel(struct spi_device *spidev)
{
	// Power on sequence for Sharp ls038y7dx01 display panel.

	ls038y7dx01_write_reg(spidev, 0x0f, 0x01);
	ls038y7dx01_write_reg(spidev, 0x05, 0x01);
#if 1
	ls038y7dx01_write_reg(spidev, 0x07, 0x10);
	ls038y7dx01_write_reg(spidev, 0x09, 0x1e);
	ls038y7dx01_write_reg(spidev, 0x0a, 0x04);
#else
	ls038y7dx01_write_reg(spidev, 0x07, 0x30);
	ls038y7dx01_write_reg(spidev, 0x09, 0x28);
	ls038y7dx01_write_reg(spidev, 0x0a, 0x06);
#endif
	ls038y7dx01_write_reg(spidev, 0x11, 0xff);
	ls038y7dx01_write_reg(spidev, 0x15, 0x8a);
	ls038y7dx01_write_reg(spidev, 0x16, 0x00);
	ls038y7dx01_write_reg(spidev, 0x17, 0x82);
	ls038y7dx01_write_reg(spidev, 0x18, 0x24);
	ls038y7dx01_write_reg(spidev, 0x19, 0x22);
	ls038y7dx01_write_reg(spidev, 0x1a, 0x6d);
	ls038y7dx01_write_reg(spidev, 0x1b, 0xeb);
	ls038y7dx01_write_reg(spidev, 0x1c, 0xb9);
	ls038y7dx01_write_reg(spidev, 0x1d, 0x3a);
	ls038y7dx01_write_reg(spidev, 0x31, 0x1a);
	ls038y7dx01_write_reg(spidev, 0x32, 0x16);
	ls038y7dx01_write_reg(spidev, 0x33, 0x05);
	ls038y7dx01_write_reg(spidev, 0x37, 0x7f);
	ls038y7dx01_write_reg(spidev, 0x38, 0x15);
	ls038y7dx01_write_reg(spidev, 0x39, 0x7b);
	ls038y7dx01_write_reg(spidev, 0x3c, 0x05);
	ls038y7dx01_write_reg(spidev, 0x3d, 0x0c);
	ls038y7dx01_write_reg(spidev, 0x3e, 0x80);
	ls038y7dx01_write_reg(spidev, 0x3f, 0x00);
	ls038y7dx01_write_reg(spidev, 0x5c, 0x90);
	ls038y7dx01_write_reg(spidev, 0x61, 0x01);
	ls038y7dx01_write_reg(spidev, 0x62, 0xff);
	ls038y7dx01_write_reg(spidev, 0x71, 0x11);
	ls038y7dx01_write_reg(spidev, 0x72, 0x02);
	ls038y7dx01_write_reg(spidev, 0x73, 0x08);
	ls038y7dx01_write_reg(spidev, 0x7b, 0xab);
	ls038y7dx01_write_reg(spidev, 0x7c, 0x04);
	ls038y7dx01_write_reg(spidev, 0x06, 0x02);
	ls038y7dx01_write_reg(spidev, 0x85, 0x00);
	ls038y7dx01_write_reg(spidev, 0x86, 0xfe);
	ls038y7dx01_write_reg(spidev, 0x87, 0x22);
	ls038y7dx01_write_reg(spidev, 0x88, 0x0b);
	ls038y7dx01_write_reg(spidev, 0x89, 0xff);
	ls038y7dx01_write_reg(spidev, 0x8a, 0x0f);
	ls038y7dx01_write_reg(spidev, 0x8b, 0x00);
	ls038y7dx01_write_reg(spidev, 0x8c, 0xfe);
	ls038y7dx01_write_reg(spidev, 0x8d, 0x22);
	ls038y7dx01_write_reg(spidev, 0x8e, 0x0b);
	ls038y7dx01_write_reg(spidev, 0x8f, 0xff);
	ls038y7dx01_write_reg(spidev, 0x90, 0x0f);
	ls038y7dx01_write_reg(spidev, 0x91, 0x00);
	ls038y7dx01_write_reg(spidev, 0x92, 0xfe);
	ls038y7dx01_write_reg(spidev, 0x93, 0x22);
	ls038y7dx01_write_reg(spidev, 0x94, 0x0b);
	ls038y7dx01_write_reg(spidev, 0x95, 0xff);
	ls038y7dx01_write_reg(spidev, 0x96, 0x0f);
	ls038y7dx01_write_reg(spidev, 0xca, 0x30);
	ls038y7dx01_write_reg(spidev, 0x1e, 0x01);
	ls038y7dx01_write_reg(spidev, 0x04, 0x01);
	ls038y7dx01_write_reg(spidev, 0x1f, 0x41);

	//mdelay(10);

	ls038y7dx01_write_reg(spidev, 0x1f, 0xc1);

	//mdelay(10);

	ls038y7dx01_write_reg(spidev, 0x1f, 0xd9);
	ls038y7dx01_write_reg(spidev, 0x1f, 0xdf);

	msleep(1000);

	return 0;
}

static int sharp_ls038y7dx01_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(50);

	if (dssdev->platform_enable)
		r = dssdev->platform_enable(dssdev);

	return r;
}

static void sharp_ls038y7dx01_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(100);
}

static int sharp_ls038y7dx01_panel_suspend(struct omap_dss_device *dssdev)
{
	sharp_ls038y7dx01_panel_disable(dssdev);
	return 0;
}

static int sharp_ls038y7dx01_panel_resume(struct omap_dss_device *dssdev)
{
	return sharp_ls038y7dx01_panel_enable(dssdev);
}

static struct omap_dss_driver sharp_ls038y7dx01_driver = {
	.probe		= sharp_ls038y7dx01_panel_probe,
	.remove		= sharp_ls038y7dx01_panel_remove,

	.enable		= sharp_ls038y7dx01_panel_enable,
	.disable	= sharp_ls038y7dx01_panel_disable,
	.suspend	= sharp_ls038y7dx01_panel_suspend,
	.resume		= sharp_ls038y7dx01_panel_resume,

	.driver         = {
		.name   = "sharp_ls038y7dx01_panel",
		.owner  = THIS_MODULE,
	},
};

static int __devinit ls038y7dx01_panel_spi_probe(struct spi_device *spidev)
{
	int ret; 

	spidev->mode = SPI_MODE_1; // | SPI_CS_HIGH;
	spidev->bits_per_word = 16;
	spi_setup(spidev);

	init_sharp_ls038y7dx01_panel(spidev);
	
	ret = omap_dss_register_driver(&sharp_ls038y7dx01_driver);
	if (ret != 0) 
		pr_err("ls038y7dx01: Unable to register panel driver: %d\n", ret);
	
	return ret;
}

static int __devinit ls038y7dx01_panel_spi_remove(struct spi_device *spidev)
{
	omap_dss_unregister_driver(&sharp_ls038y7dx01_driver);
	return 0;
}

static int ls038y7dx01_panel_spi_suspend(struct spi_device *spidev, pm_message_t msg)
{
	ls038y7dx01_write_reg(spidev, 0x04, 0x00);
	mdelay(20);
	ls038y7dx01_write_reg(spidev, 0x1f, 0xc1);
	mdelay(10);
	ls038y7dx01_write_reg(spidev, 0x1f, 0x00);
		
	mdelay(80);
	
	return 0;
}

static int ls038y7dx01_panel_spi_resume(struct spi_device *spidev)
{
	init_sharp_ls038y7dx01_panel(spidev);
	return 0;
}

static struct spi_driver ls038y7dx01_spi_driver = {
	.driver		= {
		.name	= "sharp_ls038y7dx01_panel-spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= ls038y7dx01_panel_spi_probe,
	.remove 	= __devexit_p (ls038y7dx01_panel_spi_remove),
	.suspend	= ls038y7dx01_panel_spi_suspend,
	.resume		= ls038y7dx01_panel_spi_resume,
};

static int __init sharp_ls038y7dx01_panel_drv_init(void)
{
	int ret;
	
	ret = spi_register_driver(&ls038y7dx01_spi_driver);
	if (ret != 0) 
		pr_err("ls038y7dx01: Unable to register SPI driver: %d\n", ret);

	return ret;
}

static void __exit sharp_ls038y7dx01_panel_drv_exit(void)
{
	spi_unregister_driver(&ls038y7dx01_spi_driver);
}

module_init(sharp_ls038y7dx01_panel_drv_init);
module_exit(sharp_ls038y7dx01_panel_drv_exit);
MODULE_LICENSE("GPL");
