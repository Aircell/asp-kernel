/*
 * Aircell - 
 * Updated for the AirCell CloudSurfer
 * Tarr - July 2011
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/leds.h>
#include <linux/interrupt.h>

#include <linux/spi/spi.h>
#include <linux/spi/brf6300.h>
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

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
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
#include <plat/omap3logic-productid.h>
#include <plat/omap3logic-cf.h>
#include <plat/wifi_tiwlan.h>

#include "mux.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "omap3-opp.h"
#include "board-omap3logic.h"
#include <plat/sdrc.h>

#include "aircell_gpio.h"

#define QT_I2C_ADDR			 0x4b
#define DIE_ID_REG_BASE      (L4_WK_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET    0x218
#define MAX_USB_SERIAL_NUM   17

#define OMAP3LOGIC_SMSC911X_CS		1
#define OMAP3LOGIC_LV_SOM_SMSC911X_GPIO		152	/* LV SOM LAN IRQ */
#define OMAP3LOGIC_TORPEDO_SMSC911X_GPIO	129	/* Torpedo LAN IRQ */

extern struct regulator_consumer_supply twl4030_vmmc1_supply;
extern struct regulator_consumer_supply twl4030_vsim_supply; 

extern struct regulator_init_data vmmc1_data;
extern struct regulator_init_data vsim_data;

static struct resource omap3logic_smsc911x_resources[] = {
	[0] =	{
		.flags	= IORESOURCE_MEM,
	},
	[1] =	{
		.flags	= (IORESOURCE_IRQ | IRQF_TRIGGER_LOW),
	},
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface  = PHY_INTERFACE_MODE_MII,
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags          = SMSC911X_USE_16BIT,
};

static struct platform_device omap3logic_smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap3logic_smsc911x_resources),
	.resource	= &omap3logic_smsc911x_resources[0],
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static inline void __init omap3logic_init_smsc911x(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	int eth_gpio = 0;

	eth_cs = OMAP3LOGIC_SMSC911X_CS;

	eth_gpio = OMAP3LOGIC_LV_SOM_SMSC911X_GPIO;

	omap_mux_init_gpio(eth_gpio, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);

	if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for smsc911x\n");
		return;
	}

	omap3logic_smsc911x_resources[0].start = cs_mem_base + 0x0;

	omap3logic_smsc911x_resources[0].end   = cs_mem_base + 0xff;
	omap3logic_smsc911x_resources[0].end   = cs_mem_base + 0xf;

	if (gpio_request(eth_gpio, "eth0") < 0) {
		printk(KERN_ERR "Failed to request GPIO_%d for smsc911x IRQ\n",
				eth_gpio);
		return;
	}
	gpio_direction_input(eth_gpio);

	omap3logic_smsc911x_resources[1].start = OMAP_GPIO_IRQ(eth_gpio);

	if (omap3logic_extract_lan_ethaddr(&smsc911x_config.mac[0])) {
		printk(KERN_INFO "smsc911x: using production MAC address\n");
	}

	if (platform_device_register(&omap3logic_smsc911x_device) < 0) {
		printk(KERN_ERR "Unable to register smsc911x device\n");
		return;
	}
}

static struct regulator_consumer_supply omap3logic_vaux1_supply = {
	.supply			= "vaux1",
};

static struct regulator_consumer_supply omap3logic_vpll2_supplies[] = {
	{
		.supply         = "vpll2",
		.dev		= &omap3logic_lcd_device.dev,
	},
	{
		.supply         = "vdds_dsi",
		.dev		= &omap3logic_dss_device.dev,
	}
};

static struct regulator_consumer_supply omap3logic_vdda_dac_supply = {
	.supply         = "vdda_dac",
	.dev		= &omap3logic_dss_device.dev,
};

static struct regulator_init_data omap3logic_vdda_dac = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &omap3logic_vdda_dac_supply,
};

static struct regulator_init_data omap3logic_vpll2 = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(omap3logic_vpll2_supplies),
	.consumer_supplies      = omap3logic_vpll2_supplies,
};

/* VAUX1 for mainboard (touch and productID) */
static struct regulator_init_data omap3logic_vaux1 = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.name			= "VAUX1_30",
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
#if 0
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
#endif
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3logic_vaux1_supply,
};


static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{},	/* Terminator (or MMC3 info for LV SOM) */
	{}	/* Terminator */
};

static struct twl4030_hsmmc_info mmc3 = {
	.mmc		= 3,
	.wires		= 4,
	.gpio_cd	= -EINVAL,
	.gpio_wp	= -EINVAL,
};

static struct gpio_led omap3logic_leds[] = {
	{
		.name			= "led1",	/* D1 on baseboard */
		.default_trigger	= "heartbeat",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= false,
	},
	{
		.name			= "led2",	/* D2 on baseboard */
		.default_trigger	= "none",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= false,
	},
	{
		.name			= "led3",	/* D1 on Torpedo module */
		.default_trigger	= "none",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};
 
static struct gpio_led_platform_data omap3logic_led_data = {
	.leds		= omap3logic_leds,
	.num_leds	= 0,	/* Initialized in omap3logic_led_init() */
 };
 
static struct platform_device omap3logic_led_device = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &omap3logic_led_data,
	},
};
 
/* GPIO.0 of TWL4030 */
static int twl4030_base_gpio;

#define GPIO_LED1_TORPEDO_POST_1013994 180
#define GPIO_LED2_TORPEDO_POST_1013994 179

#define GPIO_LED1_SOM 133
#define GPIO_LED2_SOM 11

static void omap3logic_led_init(void)
{
	u32 part_number;
	int gpio_led1 = -EINVAL;
	int gpio_led2 = -EINVAL;

	if (machine_is_omap3_torpedo()) {
		if (!twl4030_base_gpio) {
			printk(KERN_ERR "Huh?!? twl4030_base_gpio not set!\n");
			return;
		}
		if (!omap3logic_get_product_id_part_number (&part_number)) {

			if (part_number >= 1013994) {
				/* baseboard LEDs are MCSPIO2_SOMI, MCSPOI2_SIMO */
				gpio_led1 = GPIO_LED1_TORPEDO_POST_1013994;
				gpio_led2 = GPIO_LED2_TORPEDO_POST_1013994;
			} else {
				/* baseboard LEDS are BT_PCK_DR and BT_PCM_DX */
				gpio_led1 = twl4030_base_gpio + 16;
				gpio_led2 = twl4030_base_gpio + 17;
			}
		}

		/* twl4030 vibra_p is the LED on the module */
		omap3logic_leds[2].gpio = twl4030_base_gpio + TWL4030_GPIO_MAX + 1;
		omap3logic_led_data.num_leds = 3;
	} else {
		gpio_led1 = GPIO_LED1_SOM;
		omap3logic_leds[0].active_low = true;
		gpio_led2 = GPIO_LED2_SOM;
		omap3logic_leds[1].active_low = true;

		/* SOM has only two LEDs */
		omap3logic_led_data.num_leds = 2;
	}

	if (gpio_led2 < twl4030_base_gpio)
		omap_mux_init_gpio(gpio_led2, OMAP_PIN_OUTPUT);
	omap3logic_leds[1].gpio = gpio_led2;

	omap3logic_led_data.num_leds = 2;

	if (platform_device_register(&omap3logic_led_device) < 0)
		printk(KERN_ERR "Unable to register LED device\n");
}

int brf6300_bt_nshutdown_gpio;

int brf6300_request_bt_nshutdown_gpio(void)
{
	struct clk *sys_clkout1_clk;

	BUG_ON(!brf6300_bt_nshutdown_gpio);
	printk("%s: gpio %d\n", __FUNCTION__, brf6300_bt_nshutdown_gpio);

	/* Enable sys_clkout1 (uP_CLKOUT1_26Mhz used by brf6300 */
	sys_clkout1_clk = clk_get(NULL, "sys_clkout1");
	if (IS_ERR(sys_clkout1_clk)) {
		printk("%s: Can't get sys_clkout1\n", __FUNCTION__);
		return -1;
	}
	clk_enable(sys_clkout1_clk);

	return gpio_request(brf6300_bt_nshutdown_gpio, "BT_nSHUTDOWN");
}

void brf6300_free_bt_nshutdown_gpio(void)
{
	BUG_ON(!brf6300_bt_nshutdown_gpio);
	printk("%s\n", __FUNCTION__);
	gpio_free(brf6300_bt_nshutdown_gpio);
}

void brf6300_direction_bt_nshutdown_gpio(int direction, int value)
{
	BUG_ON(!brf6300_bt_nshutdown_gpio);
	printk(KERN_INFO "%s: direction %d value %d\n", __FUNCTION__, direction, value);
	if (!direction)
		gpio_direction_output(brf6300_bt_nshutdown_gpio, value);
	else
		gpio_direction_input(brf6300_bt_nshutdown_gpio);
}

void brf6300_set_bt_nshutdown_gpio(int value)
{
	BUG_ON(!brf6300_bt_nshutdown_gpio);
	printk("%s: value %d\n", __FUNCTION__, value);
	gpio_set_value(brf6300_bt_nshutdown_gpio, value);
}

int brf6300_get_bt_nshutdown_gpio(int value)
{
	int ret;
	BUG_ON(!brf6300_bt_nshutdown_gpio);
	ret = gpio_get_value(brf6300_bt_nshutdown_gpio);
	printk("%s: value %d\n", __FUNCTION__, ret);
	return ret;
}

static int omap3logic_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* LV-SOM 1010194/1009998 */
	mmc[0].gpio_cd = 110;
	omap_mux_init_gpio(110, OMAP_PIN_INPUT_PULLUP);

	/* For the LV SOM, add in the uf1050a MMC info to
	 * the MMC list (the 3rd slot is the terminator). */
	mmc[1] = mmc3;
	/* link regulators to MMC adapters */
	twl4030_vmmc1_supply.dev = mmc[0].dev;
	twl4030_vsim_supply.dev = mmc[0].dev;

	twl4030_base_gpio = gpio;

	/* TWL4030 GPIO for BT_nSHUTDOWN */
	brf6300_bt_nshutdown_gpio = gpio + TWL4030_BT_nSHUTDOWN;

	printk(KERN_INFO "%s: TWL4030 base gpio: %d\n", __FUNCTION__, gpio);

	twl4030_mmc_init(mmc);

	return 0;
}

static struct twl4030_gpio_platform_data omap3logic_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.setup		= omap3logic_twl_gpio_setup,
};

static struct twl4030_usb_data omap3logic_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static int qt_keymap[] = {
	KEY(0, 0, KEY_BACK),
	KEY(0, 1, KEY_HOME),
	KEY(0, 2, KEY_HOME),
	KEY(1, 0, KEY_1),
	KEY(1, 1, KEY_2),
	KEY(1, 2, KEY_3),
	KEY(2, 0, KEY_4),
	KEY(2, 1, KEY_5),
	KEY(2, 2, KEY_6),
	KEY(3, 0, KEY_7),
	KEY(3, 1, KEY_8),
	KEY(3, 2, KEY_9),
	KEY(4, 0, KEY_NUMERIC_STAR),
	KEY(4, 1, KEY_0),
	KEY(4, 2, KEY_NUMERIC_POUND),
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= qt_keymap,
	.keymap_size		= ARRAY_SIZE(qt_keymap),
};

static struct twl4030_keypad_data omap3logic_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 4,
	.cols		= 3,
	.rep		= 1,
};

static struct twl4030_madc_platform_data omap3logic_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_codec_audio_data omap3logic_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_vibra_data omap3logic_vibra_data = {
    .audio_mclk = 26000000,
};

static struct twl4030_codec_data omap3logic_codec_data = {
	.audio_mclk = 26000000,
	.audio = &omap3logic_audio_data,
	.vibra = &omap3logic_vibra_data,
};

static int omap3logic_batt_table[] = {
/* 0 C*/
	30800, 29500, 28300, 27100,
	26000, 24900, 23900, 22900, 22000, 21100, 20300, 19400, 18700, 17900,
	17200, 16500, 15900, 15300, 14700, 14100, 13600, 13100, 12600, 12100,
	11600, 11200, 10800, 10400, 10000, 9630,  9280,  8950,  8620,  8310,
	8020,  7730,  7460,  7200,  6950,  6710,  6470,  6250,  6040,  5830,
	5640,  5450,  5260,  5090,  4920,  4760,  4600,  4450,  4310,  4170,
	4040,  3910,  3790,  3670,  3550
};

static struct twl4030_bci_platform_data omap3logic_bci_data = {
	.battery_tmp_tbl	= omap3logic_batt_table,
	.bb_chen		= BB_CHARGE_ENABLED,
	.bb_current		= BB_CURRENT_150,
	.bb_voltage		= BB_VOLTAGE_31,
	.tblsize		= ARRAY_SIZE(omap3logic_batt_table),
};

static struct twl4030_platform_data omap3logic_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.bci        = &omap3logic_bci_data,
	.madc		= &omap3logic_madc_data,
	.usb		= &omap3logic_usb_data,
	.gpio		= &omap3logic_gpio_data,
	.codec		= &omap3logic_codec_data,

	.vaux1		= &omap3logic_vaux1,
	.vpll2		= &omap3logic_vpll2,
	.vdac		= &omap3logic_vdda_dac,
};

static struct i2c_board_info __initdata omap3logic_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &omap3logic_twldata,
	},
};

/*
 * 24LC128 EEPROM Support
 */ 
#include <linux/i2c/at24.h>

static struct at24_platform_data m24c128 = { 
            .byte_len       = 131072 / 8,
            .page_size      = 64, 
			.flags			= AT24_FLAG_ADDR16,
};
	
static struct i2c_board_info __initdata omap3logic_i2c2_boardinfo[] = {
    {    
        I2C_BOARD_INFO("eeprom", 0x50),
		.platform_data = &m24c128,
    },
};

/*
 * Touchscreen MultiTouch configuration 
 *
 * For Aircell CloudSurfer, only that part of the touchscreen that
 * overlays the LCD display is used for the MultiTouch.
 * The touchscreen is configured to map returning x and y touch coordinates
 * to the dsiplay coordinates. The touch screen is a matrix of 19 lines by
 * 11 lines of "channels". The LCD display is a 480 X 800 pixle display.
 * The display has the origin (0,0) in the upper left hand corner, (480,0)
 * in the upper right hand corner, (0,800) in the lower left hand corner and
 * (480,800) in the lower right hand corner. Since the since the touch screen 
 * also overlays the "keys" which are a different module, the size is smaller
 * than the x size is smaller that the full touch screen.
 *
 * Now for the really weird parts. The orientation field provides a 
 * reporting of the x,y touch coordinates to match the display. 
 * This makes the rest of the configuration stuff non intuitive.
 * What makes this even worse is that the LCD for P1+ versions
 * Have the display wired upside down......
 *
 */
struct qt602240_platform_data omap3logic_touchscreendata = {
    .x_line = 19,
    .y_line = 11,
    .x_size = 1170,  /* Tarr - scaleing the x use same pixel density of the 
						LCD across the entire length of the TouchScreen */
    .y_size = 480,   /* Tarr - y is just the number of LCD pixels */
    .blen = 23,
    .threshold = 80,
    .voltage = 600,
    .orient = QT602240_NORMAL
};

static struct platform_device omap3logic_touch_device = {
    .name       = "qt602240_ts",
    .id     	= 0, 
    .dev        = {
        .platform_data = &omap3logic_touchscreendata,
    },
};


static struct i2c_board_info __initdata omap3logic_i2c3_boardinfo[] = {
	{
		I2C_BOARD_INFO("qt602440_ts", QT_I2C_ADDR),
		.type	= "qt602240_ts",
		.platform_data = &omap3logic_touchscreendata,
		.irq = OMAP_GPIO_IRQ(AIRCELL_TOUCH_INTERRUPT),
	},
};

static void omap3logic_qt602240_init(void)
{
	printk("TARR - qt602240_init()\n");

    if (platform_device_register(&omap3logic_touch_device) < 0){
            printk(KERN_ERR "Unable to register touch device\n");
        return;
    }
	//omap_set_gpio_debounce(AIRCELL_TOUCH_INTERRUPT, 1);
    //omap_set_gpio_debounce_time(AIRCELL_TOUCH_INTERRUPT, 0xa);

	/* Take the touch screen out of reset */
	gpio_direction_output(AIRCELL_TOUCH_RESET, 1);
    omap3logic_i2c3_boardinfo[0].irq = gpio_to_irq(AIRCELL_TOUCH_INTERRUPT);

	return;
}

static int __init omap3logic_i2c_init(void)
{

	/*
	 * REVISIT: These entries can be set in omap3logic_twl_data
	 * after a merge with MFD tree
	 */
	omap3logic_twldata.vmmc1 = &vmmc1_data;
	omap3logic_twldata.vsim = &vsim_data;

	omap_register_i2c_bus(1, 2600, omap3logic_i2c_boardinfo,
			ARRAY_SIZE(omap3logic_i2c_boardinfo));
    omap_register_i2c_bus(2, 400, omap3logic_i2c2_boardinfo,
            ARRAY_SIZE(omap3logic_i2c2_boardinfo));
	omap_register_i2c_bus(3, 400, omap3logic_i2c3_boardinfo,
			ARRAY_SIZE(omap3logic_i2c3_boardinfo));
	return 0;
}

static struct omap_board_config_kernel omap3logic_config[] __initdata = {
};


static void __init omap3logic_init_irq(void)
{
	omap_board_config = omap3logic_config;
	omap_board_config_size = ARRAY_SIZE(omap3logic_config);
	omap2_init_common_hw(omap3logic_get_sdram_timings(), NULL, 
			     omap35x_mpu_rate_table, omap35x_dsp_rate_table, 
			     omap35x_l3_rate_table);
	omap_init_irq();
	omap_gpio_init();
}

static struct platform_device *omap3logic_devices[] __initdata = {
};

static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

/* Initialization code called from LCD panel_init to allow muxing */
void omap3logic_lcd_panel_init(int *p_gpio_enable, int *p_gpio_backlight)
{
	*p_gpio_enable = 0;
	*p_gpio_backlight = 0;
}

#define NAND_BLOCK_SIZE		SZ_128K
#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30
#define OMAP3LOGIC_NORFLASH_CS 2

static struct mtd_partition omap3logic_nor_partitions[] = {
	{
		.name		= CONFIG_OMAP3LOGIC_NOR_PARTITION_ONE_NAME,
		.offset		= 0,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct physmap_flash_data omap3logic_nor_data = {
	.width		= 2,
	.parts		= omap3logic_nor_partitions,
	.nr_parts	= ARRAY_SIZE(omap3logic_nor_partitions),
};

static struct resource omap3logic_nor_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device omap3logic_nor_device = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data = &omap3logic_nor_data,
	},
	.num_resources	= 1,
	.resource	= &omap3logic_nor_resource,
};

static struct mtd_partition omap3logic_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,	
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "Empty",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= 1280 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Userdata",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0xA680000 */
		.size		= 512 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Cache",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0xE680000 */
		.size		= 202 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "U-Boot Env-NAND",
		.offset		= 2046 << 17,	/* Offset = 0xffc0000 */
		.size		= 2*(64 * 2048),
	},

};

static struct omap_nand_platform_data omap3logic_nand_data = {
	.parts = omap3logic_nand_partitions,
	.nr_parts = ARRAY_SIZE(omap3logic_nand_partitions),
	.ecc_opt	= 0x2,	/* HW ECC in romcode layout */
	.devsize	= 1,	/* 16-bit device */
	.dev_ready	= (void *)1, /* poll WAIT0 status */
};

static struct resource omap3logic_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device omap3logic_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &omap3logic_nand_data,
	},
	.num_resources	= 1,
	.resource	= &omap3logic_nand_resource,
};


static void __init omap3logic_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;
	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;
	int nor_cs;
	unsigned long cs_mem_base;
	int nor_size;
		
	if ((nor_size = omap3logic_NOR0_size()) > 0) {
		nor_cs = 2;
		if (gpmc_cs_request(nor_cs, SZ_8M, &cs_mem_base) < 0) {
			printk(KERN_ERR "Failed to request GPMC mem for NOR flash\n");
			return;
		}

		omap3logic_nor_resource.start = cs_mem_base;
		omap3logic_nor_resource.end = cs_mem_base + (1 << nor_size) - 1;
		if (platform_device_register(&omap3logic_nor_device) < 0)
			printk(KERN_ERR "Unable to register NOR device\n");
	}

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		u32 nand0_size, part, part_size, delta;

		omap3logic_nand_data.cs = nandcs;
		omap3logic_nand_data.gpmc_cs_baseaddr = (void *)
			(gpmc_base_add + GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
		omap3logic_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);


		/* Find the size of the NAND device.  We want the u-boot
		   environment to be in the last two blocks of NAND,
		   so place it there and bump up the size of the previous
		   partiton to take up the slack */
		nand0_size = omap3logic_NAND0_size();
		if (nand0_size > 0) {
			nand0_size = 1 << nand0_size;
			part = ARRAY_SIZE(omap3logic_nand_partitions) - 1;
			part_size = omap3logic_nand_partitions[part].offset +
				omap3logic_nand_partitions[part].size;
			if (nand0_size > part_size && part) {
				delta = nand0_size - part_size;
				printk(KERN_INFO "Adjusting u-boot environment partition by %x\n", delta);
				omap3logic_nand_partitions[part].offset += delta;
				omap3logic_nand_partitions[part-1].size += delta;
			}
		}

		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		if (platform_device_register(&omap3logic_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}

static struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 4,
	.reset_gpio_port[2]  = -EINVAL
};

static void omap3logic_init_ehci(void)
{
	omap_mux_init_gpio(ehci_pdata.reset_gpio_port[1], OMAP_PIN_OUTPUT);
	usb_ehci_init(&ehci_pdata);
}

static void omap3logic_usb_init(void)
{
	omap3logic_init_ehci();
}

void kick_uf1050a_card_detect(void)
{
	/* Only the LV SOM has the uf1050a on it */
	if (!machine_is_omap3530_lv_som())
		return;

	omap_zoom3_wifi_set_carddetect(1);
}
EXPORT_SYMBOL(kick_uf1050a_card_detect);

static void omap3logic_init_uf1050a_mux(void)
{
	printk("%s: Config MMC3 pinmux\n", __FUNCTION__);

	// Setup the mux for mmc3
	omap_mux_init_signal("mcspi1_cs1.sdmmc3_cmd", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("mcspi1_cs2.sdmmc3_clk", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat4.sdmmc3_dat0", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat5.sdmmc3_dat1", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat6.sdmmc3_dat2", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat7.sdmmc3_dat3", OMAP_PIN_INPUT_PULLUP);

	// If we're ES3.1 or later, the MMC multi-block is fixed
	if (omap_type() == OMAP2_DEVICE_TYPE_GP &&
	    omap_rev() >= OMAP3430_REV_ES3_1)
		mmc3.no_multi_block = 0;
}

static void omap3logic_init_murata_mux(void)
{
	printk(KERN_INFO "%s: setup murata mux signals\n", __FUNCTION__);

	if (gpio_request(OMAP3LOGIC_MURATA_WIFI_EN_GPIO, "wifi_en") != 0)
		pr_err("GPIO %i request for murata_en failed\n", OMAP3LOGIC_MURATA_WIFI_EN_GPIO);
	gpio_direction_output(OMAP3LOGIC_MURATA_WIFI_EN_GPIO, 0);
	omap_mux_init_gpio(OMAP3LOGIC_MURATA_WIFI_EN_GPIO, OMAP_PIN_OUTPUT);
	gpio_export(OMAP3LOGIC_MURATA_WIFI_EN_GPIO, 0);

	/* Pull the enables out of reset */
	msleep(10);
	gpio_set_value(OMAP3LOGIC_MURATA_WIFI_EN_GPIO, 1);
	msleep(10);

	/* Put them back into reset (so they go into low-power mode) */
	gpio_set_value(OMAP3LOGIC_MURATA_WIFI_EN_GPIO, 0);

	// Setup the mux for mmc3
	omap_mux_init_signal("mcspi1_cs1.sdmmc3_cmd", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("mcspi1_cs2.sdmmc3_clk", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat4.sdmmc3_dat0", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat5.sdmmc3_dat1", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat6.sdmmc3_dat2", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat7.sdmmc3_dat3", OMAP_PIN_INPUT_PULLUP);

	// If we're ES3.1 or later, the MMC multi-block is fixed
	if (omap_type() == OMAP2_DEVICE_TYPE_GP &&
	    omap_rev() >= OMAP3430_REV_ES3_1)
		mmc3.no_multi_block = 0;
}

static void omap3logic_init_wifi_mux(void)
{
	if (omap3logic_has_murata_wifi_module()) {
		omap3logic_init_murata_mux();
	} else {
		/* Only the LV SOM has the uf1050a on it */
		if (machine_is_omap3530_lv_som())
			omap3logic_init_uf1050a_mux();
	}
}

static void omap3logic_musb_init(void)
{
	/* Set up the mux for musb */
	omap_mux_init_signal("hsusb0_clk", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_stp", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_dir", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_nxt", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data0", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data1", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data2", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data3", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data4", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data5", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data6", OMAP_PIN_INPUT);
	omap_mux_init_signal("hsusb0_data7", OMAP_PIN_INPUT);

	usb_musb_init();
}

#ifdef CONFIG_BT_HCIBRF6300_SPI
static struct omap2_mcspi_device_config brf6300_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,  /* 0: slave, 1: master */
};


/* Intialized in omap3logic_spi_init() */
struct brf6300_platform_data brf6300_config;

static struct spi_board_info omap3logic_spi_brf6300 =
{
	/*
	 * BRF6300 operates at a max freqency of 2MHz, so
	 * operate slightly below at 1.5MHz
	 */
	.modalias		= "brf6300",
	.bus_num		= 1,
	.chip_select		= 0,
	.max_speed_hz		= 12000000,
	.controller_data	= &brf6300_mcspi_config,
	.irq			= OMAP_GPIO_IRQ(BT_IRQ_GPIO),
	.platform_data		= &brf6300_config,
	.mode			= SPI_MODE_1,
	.bits_per_word		= 16,
	.quirks			= SPI_QUIRK_BRF6300,
};
#endif

#define USE_AT25_AS_EEPROM
#ifdef USE_AT25_AS_EEPROM
/* Access the AT25160AN chip on the Torpedo baseboard using eeprom driver */
static struct spi_eeprom at25160an_config = {
	.name		= "at25160an",
	.byte_len	= 2048,
	.page_size	= 32,
	.flags		= EE_ADDR2,
};

static struct spi_board_info omap3logic_spi_at25160an = {
	.modalias	= "at25",
	.max_speed_hz	= 30000,
	.bus_num	= 1,
	.chip_select	= 0,
	.platform_data	= &at25160an_config,
	.bits_per_word	= 8,
};

#else
/* Access the AT25160AN chip on the Torpedo baseboard using spidev driver */
static struct omap2_mcspi_device_config at25160an_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 0,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3logic_spi_at25160an = {
	/*
	 * SPI EEPROM on Torpedo baseboard
	 */
	.modalias		= "spidev",
	.bus_num		= 1,
	.chip_select		= 0,
	.max_speed_hz		= 19000000,
	.controller_data	= &at25160an_mcspi_config,
	.irq			= 0,
	.platform_data		= NULL,
	.bits_per_word		= 8,
};
#endif

static void omap3logic_spi_init(void)
{
	int num_spi_devices = 1;
	/* config MCSPI1 pins */
	omap_mux_init_signal("mcspi1_cs0", OMAP_PIN_INPUT);
	omap_mux_init_signal("mcspi1_clk", OMAP_PIN_INPUT);
	omap_mux_init_signal("mcspi1_simo", OMAP_PIN_INPUT);
	omap_mux_init_signal("mcspi1_somi", OMAP_PIN_INPUT);

	if (machine_is_omap3530_lv_som()) {
		/* LV SOM only has the brf6300 on SPI */
		spi_register_board_info(&omap3logic_spi_brf6300, num_spi_devices);
	} else if (machine_is_omap3_torpedo()) {
		/* Torpedo only has the AT25160AN spi EEPROM on baseboard */
		spi_register_board_info(&omap3logic_spi_at25160an, num_spi_devices);
	} else {
		printk(KERN_ERR "%s: Unknown machine\n", __FUNCTION__);
	}
}

static void brf6300_dev_init(void)
{
	/* Only the LV SOM has a BRF6300 */
	if (!machine_is_omap3530_lv_som())
		return;

	if (!twl4030_base_gpio) {
		printk(KERN_ERR "Huh?!? twl4030_base_gpio not set!\n");
		return;
	}

	omap_mux_init_gpio(BT_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP); /* GPIO_157 */
	if (gpio_request(BT_IRQ_GPIO, "BRF6300 IRQ") < 0)
		printk(KERN_ERR "can't get BRF6300 irq GPIO\n");

	gpio_direction_input(BT_IRQ_GPIO);

	brf6300_config.irq_gpio = BT_IRQ_GPIO;
	brf6300_config.shutdown_gpio = twl4030_base_gpio + TWL4030_BT_nSHUTDOWN;
}

extern void omap3logic_init_audio_mux(void);
extern void __init board_lcd_init(void);


static char device_serial[MAX_USB_SERIAL_NUM];

static char *usb_functions_adb[] = {
	"adb",
};

static char *usb_functions_all[] = {
	"adb",
};

static struct android_usb_product usb_products[] = {
	{
		.num_functions = ARRAY_SIZE(usb_functions_adb),
		.functions     = usb_functions_adb,
	},
};

static struct android_usb_platform_data andusb_plat = {
	.manufacturer_name     = "LogicPD",
	.product_name          = "OMAP3530 SOM LV",
	.serial_number         = device_serial,
	.functions             = usb_functions_all,
	.products              = usb_products,
	.num_functions         = ARRAY_SIZE(usb_functions_all),
};

static struct platform_device androidusb_device = {
	.name                  = "android_usb",
	.id                    = -1,
	.dev                   = {
		.platform_data = &andusb_plat,
	},
};

static void omap3logic_android_gadget_init(void)
{
	unsigned int val[2];
	unsigned int reg;

	reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;
	val[0] = omap_readl(reg);
	val[1] = omap_readl(reg + 4);

	snprintf(device_serial, MAX_USB_SERIAL_NUM, "%08X%08X", val[1], val[0]);

	platform_device_register(&androidusb_device);
}

static void aircell_gpio_init(void)
{

#ifdef CLOUDSURFER_P1
	gpio_request(AIRCELL_5V_ENABLE,"AIRCELL_5V_ENABLE");
	gpio_request(AIRCELL_33V_ENABLE,"AIRCELL_33V_ENABLE");
	gpio_request(AIRCELL_23V_ENABLE,"AIRCELL_23V_ENABLE");
	gpio_direction_output(AIRCELL_5V_ENABLE,1);
	gpio_direction_output(AIRCELL_33V_ENABLE,1);
	gpio_direction_output(AIRCELL_23V_ENABLE,1);
	gpio_export(AIRCELL_5V_ENABLE,0);
	gpio_export(AIRCELL_33V_ENABLE,0);
	gpio_export(AIRCELL_23V_ENABLE,0);
#endif

#ifdef CLOUDSURFER_P2
	gpio_request(AIRCELL_5VA_ENABLE,"AIRCELL_5VAENABLE");
	gpio_request(AIRCELL_5VD_ENABLE,"AIRCELL_5VD_ENABLE");
	gpio_request(AIRCELL_CAMERA_PWDN,"AIRCELL_CAMERA_PWDN");
	gpio_direction_output(AIRCELL_5VA_ENABLE,0);
	gpio_direction_output(AIRCELL_5VD_ENABLE,0);
	gpio_direction_output(AIRCELL_CAMERA_PWDN,1);
	gpio_export(AIRCELL_5VA_ENABLE,0);
	gpio_export(AIRCELL_5VD_ENABLE,0);
	gpio_export(AIRCELL_CAMERA_PWDN,0);
#endif

	gpio_request(AIRCELL_18V_ENABLE,"AIRCELL_18V_ENABLE");
	gpio_request(AIRCELL_SOFTWARE_RESET,"AIRCELL_SOFTWARE_RESET");
	gpio_request(AIRCELL_WIFI_ENABLE_DETECT,"AIRCELL_WIFI_ENABLE_DETECT");
	gpio_request(AIRCELL_LCD_RESET,"AIRCELL_LCD_RESET");
	gpio_request(AIRCELL_CRADLE_DETECT,"AIRCELL_CRADLE_DETECT");
	gpio_request(AIRCELL_BLUE_ENABLE,"AIRCELL_BLUE_ENABLE");
	gpio_request(AIRCELL_GREEN_ENABLE,"AIRCELL_GREEN_ENABLE");
	gpio_request(AIRCELL_RED_ENABLE,"AIRCELL_RED_ENABLE");
	gpio_request(AIRCELL_LED_ENABLE,"AIRCELL_LED_ENABLE");
	gpio_request(AIRCELL_EARPIECE_ENABLE,"AIRCELL_EARPIECE_ENABLE");
	gpio_request(AIRCELL_RINGER_ENABLE,"AIRCELL_RINGER_ENABLE");
	gpio_request(AIRCELL_VOLUME_UP_DETECT,"AIRCELL_VOLUME_UP_DETECT");
	gpio_request(AIRCELL_VOLUME_DOWN_DETECT,"AIRCELL_VOLUME_DOWN_DETECT");
	gpio_request(AIRCELL_HANDSET_DETECT,"AIRCELL_HANDSET_DETECT");
	gpio_request(AIRCELL_TOUCH_RESET,"AIRCELL_TOUCH_RESET");
	gpio_request(AIRCELL_PROX_INTERRUPT,"AIRCELL_PROX_INTERRUPT");
	gpio_request(AIRCELL_ACCEL_INTERRUPT,"AIRCELL_ACCEL_INTERRUPT");
	gpio_request(AIRCELL_TOUCH_INTERRUPT,"AIRCELL_TOUCH_INTERRUPT");

    gpio_direction_input(AIRCELL_WIFI_ENABLE_DETECT);
    gpio_direction_output(AIRCELL_LCD_RESET,0);
    gpio_direction_input(AIRCELL_CRADLE_DETECT);
    gpio_direction_output(AIRCELL_BLUE_ENABLE,0);
    gpio_direction_output(AIRCELL_GREEN_ENABLE,0);
    gpio_direction_output(AIRCELL_RED_ENABLE,0);
    gpio_direction_output(AIRCELL_LED_ENABLE,0);
    gpio_direction_output(AIRCELL_EARPIECE_ENABLE,0);
    gpio_direction_output(AIRCELL_RINGER_ENABLE,0);
    gpio_direction_input(AIRCELL_VOLUME_UP_DETECT);
    gpio_direction_input(AIRCELL_VOLUME_DOWN_DETECT);
    gpio_direction_input(AIRCELL_HANDSET_DETECT);
    gpio_direction_output(AIRCELL_TOUCH_RESET,0);
    gpio_direction_input(AIRCELL_PROX_INTERRUPT);

	gpio_export(AIRCELL_18V_ENABLE,0);
	gpio_export(AIRCELL_SOFTWARE_RESET,0);
	gpio_export(AIRCELL_WIFI_ENABLE_DETECT,0);
	gpio_export(AIRCELL_LCD_RESET,0);
	gpio_export(AIRCELL_CRADLE_DETECT,0);
	gpio_export(AIRCELL_BLUE_ENABLE	,0);
	gpio_export(AIRCELL_GREEN_ENABLE,0);
	gpio_export(AIRCELL_RED_ENABLE,0);
	gpio_export(AIRCELL_LED_ENABLE,0);
	gpio_export(AIRCELL_EARPIECE_ENABLE,0);
	gpio_export(AIRCELL_RINGER_ENABLE,0);
	gpio_export(AIRCELL_VOLUME_UP_DETECT,0);
	gpio_export(AIRCELL_VOLUME_DOWN_DETECT,0);
	gpio_export(AIRCELL_HANDSET_DETECT,0);
	gpio_export(AIRCELL_TOUCH_RESET,0);
	gpio_export(AIRCELL_PROX_INTERRUPT,0);
	gpio_export(AIRCELL_ACCEL_INTERRUPT,0);
	gpio_export(AIRCELL_TOUCH_INTERRUPT,0);

}
static void __init omap3logic_init(void)
{

#ifdef CLOUDSURFER_P1
	printk(KERN_INFO "Aircell CloudSurfer P1\n");
#else
	printk(KERN_INFO "Aircell CloudSurfer P2\n");
#endif

	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);

	aircell_gpio_init();

	omap3logic_qt602240_init();

	omap3logic_i2c_init();

	board_lcd_init();

	platform_add_devices(omap3logic_devices, ARRAY_SIZE(omap3logic_devices));

	omap_serial_init_port(0);

	omap_serial_init_port(1);

	omap3logic_musb_init();

	/* Check the SRAM for valid product_id data(put there by
	 * u-boot). If not, then it will be read later. */
	omap3logic_fetch_sram_product_id_data();

	omap3logic_flash_init();

	omap3logic_init_smsc911x();

	omap3logic_usb_init();

	omap3logic_init_wifi_mux();

	omap3logic_init_audio_mux();


	/* Must be here since on exit, omap2_init_devices(called later)
	 * setups up SPI devices - can't add boardinfo afterwards */
	omap3logic_spi_init();

	dump_omap3logic_timings();

	omap3logic_android_gadget_init();

}

void omap3logic_init_productid_specifics(void)
{
	omap3logic_init_twl_external_mute();
	omap3logic_led_init();
	brf6300_dev_init();
}

static void __init omap3logic_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP3530_LV_SOM, "OMAP Logic 3530 LV SOM board")
	/* Maintainer: Peter Barada <peterb@logicpd.com> */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3logic_map_io,
	.init_irq	= omap3logic_init_irq,
	.init_machine	= omap3logic_init,
	.timer		= &omap_timer,
MACHINE_END
