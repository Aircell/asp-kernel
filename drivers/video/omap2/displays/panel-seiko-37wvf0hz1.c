/*
 * LCD panel driver for Seiko 37WVF0HZ1
 *
 * Copyright (C) 2011 AirCell
 * Author: Steven Tarr
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

/*
 * The Seiko 37WVF0HZ1 Panel is controlled by a LG 4573 LCD driver chip
 * using an SPI interface. Hence two documents are required/useful. One
 * is the LG 4573 Driver chip manual and the other the Seiko 37WVF0HZ1
 * Product Specification. 
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/spi/spi.h> 
#include <linux/err.h>

#include <plat/display.h>

struct lg_cmd {
	unsigned char data[2];
};

struct lg_cmd seiko_init_cmds[] = {
	{0x70,0x20},

	{0x70,0x3a},
	{0x72,0x50},

	{0x70,0xb1},
	{0x72,0x06},
	{0x72,0x1e},
	{0x72,0x0c}, 

	{0x70,0xb2},
	{0x72,0x10},
	{0x72,0xc8},
	
	{0x70,0xb3},
	{0x72,0x00}, 

	{0x70,0xb4},
	{0x72,0x04},

	{0x70,0xb5},
	{0x72,0x10}, 
	{0x72,0x20},
	{0x72,0x20}, 
	{0x72,0x00}, 
	{0x72,0x00}, 

	{0x70,0xb6},
	{0x72,0x01},
	{0x72,0x18},
	{0x72,0x02},
	{0x72,0x40},
	{0x72,0x10},
	{0x72,0x40},

	{0x70,0xc3},
	{0x72,0x03},
	{0x72,0x04},
	{0x72,0x03},
	{0x72,0x03},
	{0x72,0x03},

	{0x70,0xc4},
	{0x72,0x12},
	{0x72,0x22},
	{0x72,0x10},
	{0x72,0x0c},
	{0x72,0x03},
	{0x72,0x6C},

	{0x70,0xc5},
	{0x72,0x76},

	{0x70,0xf9},
	{0x72,0x40},

	{0x70,0xc6},
	{0x72,0x23},
	{0x72,0x50},

	{0x70,0xd0},
	{0x72,0x10},
	{0x72,0x72},
	{0x72,0x44},
	{0x72,0x10},
	{0x72,0x06},
	{0x72,0x03},
	{0x72,0x60},
	{0x72,0x16},
	{0x72,0x02},

	{0x70,0xd1},
	{0x72,0x10},
	{0x72,0x72},
	{0x72,0x44},
	{0x72,0x10},
	{0x72,0x06},
	{0x72,0x03},
	{0x72,0x60},
	{0x72,0x16},
	{0x72,0x02},

	{0x70,0xd2},
	{0x72,0x10},
	{0x72,0x72},
	{0x72,0x44},
	{0x72,0x10},
	{0x72,0x06},
	{0x72,0x03},
	{0x72,0x60},
	{0x72,0x16},
	{0x72,0x02},

	{0x70,0xd3},
	{0x72,0x10},
	{0x72,0x72},
	{0x72,0x44},
	{0x72,0x10},
	{0x72,0x06},
	{0x72,0x03},
	{0x72,0x60},
	{0x72,0x16},
	{0x72,0x02},

	{0x70,0xd4},
	{0x72,0x10},
	{0x72,0x72},
	{0x72,0x44},
	{0x72,0x10},
	{0x72,0x06},
	{0x72,0x03},
	{0x72,0x60},
	{0x72,0x16},
	{0x72,0x02},

	{0x70,0xd5},
	{0x72,0x10},
	{0x72,0x72},
	{0x72,0x44},
	{0x72,0x10},
	{0x72,0x06},
	{0x72,0x03},
	{0x72,0x60},
	{0x72,0x16},
	{0x72,0x02},
	{0x70,0x11},
	{0x00,0x00},
};

static struct omap_video_timings seiko_timings = {
	.x_res = 480,
	.y_res = 800,

	.pixel_clock	= 27000,

	.hsw		= 2,
	.hfp		= 2,
	.hbp		= 28,

	.vsw		= 2,
	.vfp		= 2,
	.vbp		= 10,
};

static int seiko_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	//dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
	//	OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF;
	//dssdev->panel.acb = 0x28;
	dssdev->panel.timings = seiko_timings;

	return 0;
}

static void seiko_panel_remove(struct omap_dss_device *dssdev)
{
}

static int seiko_status(struct spi_device *spidev) 
{
	int i;
	unsigned int ret;
	unsigned char rbuf[8];
	unsigned char tbuf[8];
	struct spi_message msg;
	struct spi_transfer yfer = {
        .cs_change  = 0,
    };
	spi_message_init(&msg);

	yfer.len = 2;
	tbuf[0] = 0x70;	
	tbuf[1] = 0xfa;

	yfer.tx_buf = &tbuf[0];
	yfer.rx_buf = NULL;

    spi_message_add_tail(&yfer, &msg);
    if ( (ret = spi_sync(spidev, &msg)) < 0 ) {
		printk(KERN_ERR "spi_sync failed: %d\n", ret);
		return ret;
	}
	
	mdelay(5);

	tbuf[0] = 0x73;	
	tbuf[1] = 0x00;
	tbuf[2] = 0x00;
	tbuf[3] = 0x00;
	tbuf[4] = 0x00;
	tbuf[5] = 0x00;

	yfer.len = 6;
	yfer.tx_buf = &tbuf[0];
	yfer.rx_buf = &rbuf[0];

	spi_message_init(&msg);
    spi_message_add_tail(&yfer, &msg);
    if ( (ret = spi_sync(spidev, &msg)) < 0 ) {
		printk(KERN_ERR "spi_sync failed: %d\n", ret);
		return ret;
	}
	
	mdelay(5);
	printk("TARR - SEIKO - status: ");
	for (i=0; i<6; i++) 
		printk("0x%2.2X ",rbuf[i]);
	printk("\n");
	return 0;

}
static int seiko_cmd(struct spi_device *spidev, struct lg_cmd *c) 
{
	int ret = 0;
    struct spi_message msg;
    struct spi_transfer index_xfer = { 
        .len        = 2,
        .cs_change  = 0,
    };
  
    spi_message_init(&msg);

    index_xfer.tx_buf = &c->data[0];
#ifdef TARR_TEST
	char *p = (char *)index_xfer.tx_buf;
	printk("TARR - SEIKO init message: ");
	for ( int i=0; i<index_xfer.len; i++,p++) {
			printk("0x%2.2X ",*p);
	}
	printk("\n");	
#endif
    spi_message_add_tail(&index_xfer, &msg);

    if ( (ret = spi_sync(spidev, &msg)) < 0 ) {
		printk(KERN_ERR "spi_sync failed: %d\n", ret);
	}
	
	mdelay(5);
	return ret;
}
static int init_seiko_panel(struct spi_device *spidev)
{
	struct lg_cmd *c = &seiko_init_cmds[0];
	struct lg_cmd on_cmd = {0x70,0x29};

	while (c->data[0] > 0 ) {
		if ( seiko_cmd(spidev,c)) {
			printk(KERN_ERR "seiko spi write failed\n");
			return -1;
		}
		c++;
	}

	/* Sleep for a while */
	mdelay(500);
	seiko_cmd(spidev,&on_cmd);

	seiko_status(spidev);
	return 0;
}

static int seiko_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(500);

	if (dssdev->platform_enable)
		r = dssdev->platform_enable(dssdev);

	return r;
}

static void seiko_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(100);
}

static int seiko_panel_suspend(struct omap_dss_device *dssdev)
{
	seiko_panel_disable(dssdev);
	return 0;
}

static int seiko_panel_resume(struct omap_dss_device *dssdev)
{
	return seiko_panel_enable(dssdev);
}

static struct omap_dss_driver seiko_driver = {
	.probe		= seiko_panel_probe,
	.remove		= seiko_panel_remove,

	.enable		= seiko_panel_enable,
	.disable	= seiko_panel_disable,
	.suspend	= seiko_panel_suspend,
	.resume		= seiko_panel_resume,

	.driver         = {
		.name   = "seiko_panel",
		.owner  = THIS_MODULE,
	},
};

static int __devinit seiko_panel_spi_probe(struct spi_device *spidev)
{
	int ret; 

	printk("Seiko Panel SPI Probe\n");
	spidev->mode = SPI_MODE_3; 
	spidev->bits_per_word = 8;
	spi_setup(spidev);

	init_seiko_panel(spidev);
	
	ret = omap_dss_register_driver(&seiko_driver);
	if (ret != 0) 
		pr_err("seiko: Unable to register panel driver: %d\n", ret);
	
	return ret;
}

static int __devinit seiko_panel_spi_remove(struct spi_device *spidev)
{
	omap_dss_unregister_driver(&seiko_driver);
	return 0;
}

static int seiko_panel_spi_suspend(struct spi_device *spidev, pm_message_t msg)
{
	return 0;
}

static int seiko_panel_spi_resume(struct spi_device *spidev)
{
	init_seiko_panel(spidev);
	return 0;
}

static struct spi_driver seiko_spi_driver = {
	.driver		= {
		.name	= "seiko_panel",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= seiko_panel_spi_probe,
	.remove 	= __devexit_p (seiko_panel_spi_remove),
	.suspend	= seiko_panel_spi_suspend,
	.resume		= seiko_panel_spi_resume,
};

static int __init seiko_panel_drv_init(void)
{
	int ret;

	printk(KERN_INFO "Seiko Panel Driver Init\n");	
	ret = spi_register_driver(&seiko_spi_driver);
	if (ret != 0) 
		pr_err("seiko: Unable to register SPI driver: %d\n", ret);

	return ret;
}

static void __exit seiko_panel_drv_exit(void)
{
	spi_unregister_driver(&seiko_spi_driver);
}

module_init(seiko_panel_drv_init);
module_exit(seiko_panel_drv_exit);
MODULE_LICENSE("GPL");
