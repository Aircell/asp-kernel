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

#include <linux/i2c/tsc2004.h>
#include <linux/i2c/twl.h>
#include <linux/usb/otg.h>
#include <linux/smsc911x.h>

#include <linux/regulator/machine.h>

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

#define OMAP3LOGIC_SMSC911X_CS		1
#define OMAP3LOGIC_LV_SOM_SMSC911X_GPIO		152	/* LV SOM LAN IRQ */
#define OMAP3LOGIC_TORPEDO_SMSC911X_GPIO	129	/* Torpedo LAN IRQ */

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
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

/* Fix the PBIAS voltage for Torpedo MMC1 pins that
 * are used for other needs (IRQs, etc). */
static void omap3torpedo_fix_pbias_voltage(void)
{
	u16 control_pbias_offset = OMAP343X_CONTROL_PBIAS_LITE;
	static int pbias_fixed = 0;
	u32 reg;

	if (!pbias_fixed) {
		// Set the bias for the pin
		reg = omap_ctrl_readl(control_pbias_offset);

		reg &= ~OMAP343X_PBIASLITEPWRDNZ1;
		omap_ctrl_writel(reg, control_pbias_offset);

		/* 100ms delay required for PBIAS configuration */
		msleep(100);

		reg |= OMAP343X_PBIASLITEVMODE1;
		reg |= OMAP343X_PBIASLITEPWRDNZ1;
		omap_ctrl_writel(reg | 0x300, control_pbias_offset);

		pbias_fixed = 1;
	}
}

static inline void __init omap3logic_init_smsc911x(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	int eth_gpio = 0;

	printk("%s:%d\n", __FUNCTION__, __LINE__);

	eth_cs = OMAP3LOGIC_SMSC911X_CS;

	if (machine_is_omap3530_lv_som()) {
		eth_gpio = OMAP3LOGIC_LV_SOM_SMSC911X_GPIO;
	} else if (machine_is_omap3_torpedo()) {
		eth_gpio = OMAP3LOGIC_TORPEDO_SMSC911X_GPIO;

		/* On Torpedo, LAN9221 IRQ is an MMC1_DATA7 pin
		 * and IRQ1760 IRQ is MMC1_DATA4 pin - need
		 * to update PBIAS to get voltage to the device
		 * so the IRQs works correctly rather than float
		 * and cause an IRQ storm... */

		omap3torpedo_fix_pbias_voltage();
	}

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

#else
static inline void __init omap3logic_init_smsc911x(void) { return; }
#endif

static struct regulator_consumer_supply omap3logic_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply omap3logic_vsim_supply = {
	.supply			= "vmmc_aux",
};

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


/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data omap3logic_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3logic_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data omap3logic_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3logic_vsim_supply,
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

// Disable GPIO initialization for LED1 when using
// Sharp LS038Y7DX01 display panel. PIN sharing issue.
#if !defined(CONFIG_PANEL_SHARP_LS038Y7DX01)

	if (gpio_led1 < twl4030_base_gpio)
		omap_mux_init_gpio(gpio_led1, OMAP_PIN_OUTPUT);
	omap3logic_leds[0].gpio = gpio_led1;
#endif

	if (gpio_led2 < twl4030_base_gpio)
		omap_mux_init_gpio(gpio_led2, OMAP_PIN_OUTPUT);
	omap3logic_leds[1].gpio = gpio_led2;

	omap3logic_led_data.num_leds = 2;

	if (platform_device_register(&omap3logic_led_device) < 0)
		printk(KERN_ERR "Unable to register LED device\n");
}

#ifdef CONFIG_BT_HCIBRF6300_SPI

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
#else
int brf6300_request_bt_nshutdown_gpio(void) {
	return -ENOSYS;
}

void brf6300_free_bt_nshutdown_gpio(void) {
	return;
}

void brf6300_direction_bt_nshutdown_gpio(int direction, int value)
{
	return;
}

void brf6300_set_bt_nshutdown_gpio(int value)
{	
	return;
}

int brf6300_get_bt_nshutdown_gpio(int value)
{
	return -ENOSYS;
}
#endif


static int omap3logic_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	if (machine_is_omap3530_lv_som()) {
#ifndef CONFIG_INPUT_TWL4030_VIBRA
		/* LV-SOM 1010194/1009998 */
		mmc[0].gpio_cd = 110;
		omap_mux_init_gpio(110, OMAP_PIN_INPUT_PULLUP);
		mmc[0].gpio_wp = 126;
		omap_mux_init_gpio(126, OMAP_PIN_INPUT_PULLUP);

		/* For the LV SOM, add in the uf1050a MMC info to
		 * the MMC list (the 3rd slot is the terminator). */
		mmc[1] = mmc3;
#endif
	} else if (machine_is_omap3_torpedo()) {
		/* Torpedo 1013994/1013995 */
		omap3torpedo_fix_pbias_voltage();
		mmc[0].gpio_cd = 127;
		omap_mux_init_gpio(127, OMAP_PIN_INPUT_PULLUP);
		omap_set_gpio_debounce(127, 5);
		mmc[0].gpio_wp = -EINVAL;
	} else {
		printk(KERN_ERR "%s: Unknown machine\n", __FUNCTION__);
		return -1;
	}

	/* link regulators to MMC adapters */
	omap3logic_vmmc1_supply.dev = mmc[0].dev;
	omap3logic_vsim_supply.dev = mmc[0].dev;

	twl4030_base_gpio = gpio;

#ifdef CONFIG_BT_HCIBRF6300_SPI
	/* TWL4030 GPIO for BT_nSHUTDOWN */
	brf6300_bt_nshutdown_gpio = gpio + TWL4030_BT_nSHUTDOWN;
#endif

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

static int board_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_A),
	KEY(0, 3, KEY_B),
	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_E),
	KEY(1, 3, KEY_F),
	KEY(2, 0, KEY_ENTER),
	KEY(2, 1, KEY_I),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_K),
	KEY(3, 0, KEY_M),
	KEY(3, 1, KEY_N),
	KEY(3, 2, KEY_O),
	KEY(3, 3, KEY_P)
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data omap3logic_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 4,
	.cols		= 4,
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
#ifdef OMAP3LOGIC_BACKUP_BATTERY_ENABLE
	.bb_chen		= BB_CHARGE_ENABLED,
# ifdef OMAP3LOGIC_BACKUP_BATTERY_C25
	.bb_current		= BB_CURRENT_25,
#  elif OMAP3LOGIC_BACKUP_BATTERY_C150
	.bb_current		= BB_CURRENT_150,
#  elif OMAP3LOGIC_BACKUP_BATTERY_C500
	.bb_current		= BB_CURRENT_500,
#  elif OMAP3LOGIC_BACKUP_BATTERY_C1000
	.bb_current		= BB_CURRENT_1000,
# endif
# ifdef OMAP3LOGIC_BACKUP_BATTERY_V25
	.bb_voltage		= BB_VOLTAGE_25,
#  elif OMAP3LOGIC_BACKUP_BATTERY_V30
	.bb_voltage		= BB_VOLTAGE_30,
#  elif OMAP3LOGIC_BACKUP_BATTERY_V31
	.bb_voltage		= BB_VOLTAGE_31,
#  elif OMAP3LOGIC_BACKUP_BATTERY_V32
	.bb_voltage		= BB_VOLTAGE_32,
# endif
# else
	.bb_chen		= BB_CHARGE_DISABLED,
#endif
	.tblsize		= ARRAY_SIZE(omap3logic_batt_table),
};

static struct twl4030_platform_data omap3logic_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.bci            = &omap3logic_bci_data,
	.keypad		= &omap3logic_kp_data,
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
#ifdef CONFIG_EEPROM_AT24
#include <linux/i2c/at24.h>

static struct at24_platform_data m24c128 = { 
            .byte_len       = 131072 / 8,
            .page_size      = 64, 
			.flags			= AT24_FLAG_ADDR16,
};
	
static struct i2c_board_info __initdata omap3logic_i2c2_boardinfo[] = {
    {    
        //I2C_BOARD_INFO("24c128", 0x57),
        I2C_BOARD_INFO("24c128", 0x50),
		.platform_data = &m24c128,
    },
};
#endif

/*
 * TSC 2004 Support
 */
#define	GPIO_TSC2004_IRQ	153

static int tsc2004_init_irq(void)
{
	int ret = 0;

	omap_mux_init_gpio(GPIO_TSC2004_IRQ, OMAP_PIN_INPUT);
	ret = gpio_request(GPIO_TSC2004_IRQ, "tsc2004-irq");
	if (ret < 0) {
		printk(KERN_WARNING "failed to request GPIO#%d: %d\n",
				GPIO_TSC2004_IRQ, ret);
		return ret;
	}

	if (gpio_direction_input(GPIO_TSC2004_IRQ)) {
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
				"input\n", GPIO_TSC2004_IRQ);
		return -ENXIO;
	}

	omap_set_gpio_debounce(GPIO_TSC2004_IRQ, 1);
	omap_set_gpio_debounce_time(GPIO_TSC2004_IRQ, 0xa);
	return ret;
}

static void tsc2004_exit_irq(void)
{
	gpio_free(GPIO_TSC2004_IRQ);
}

static int tsc2004_get_irq_level(void)
{
	return gpio_get_value(GPIO_TSC2004_IRQ) ? 0 : 1;
}

struct tsc2004_platform_data omap3logic_tsc2004data = {
	.model = 2004,
	.x_plate_ohms = 180,
	.get_pendown_state = tsc2004_get_irq_level,
	.init_platform_hw = tsc2004_init_irq,
	.exit_platform_hw = tsc2004_exit_irq,
};

static struct i2c_board_info __initdata omap3logic_i2c3_boardinfo[] = {
	{
		I2C_BOARD_INFO("tsc2004", 0x48),
		.type		= "tsc2004",
		.platform_data = &omap3logic_tsc2004data,
		.irq = OMAP_GPIO_IRQ(GPIO_TSC2004_IRQ),
	},
};


static int __init omap3logic_i2c_init(void)
{

	/*
	 * REVISIT: These entries can be set in omap3logic_twl_data
	 * after a merge with MFD tree
	 */
	omap3logic_twldata.vmmc1 = &omap3logic_vmmc1;
	omap3logic_twldata.vsim = &omap3logic_vsim;

	omap_register_i2c_bus(1, 2600, omap3logic_i2c_boardinfo,
			ARRAY_SIZE(omap3logic_i2c_boardinfo));
#ifdef CONFIG_EEPROM_AT24
    omap_register_i2c_bus(2, 400, omap3logic_i2c2_boardinfo,
            ARRAY_SIZE(omap3logic_i2c2_boardinfo));
#else
	omap_register_i2c_bus(2, 400, NULL, 0);
#endif
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

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

/* Initialization code called from LCD panel_init to allow muxing */
void omap3logic_lcd_panel_init(int *p_gpio_enable, int *p_gpio_backlight)
{
	int gpio_enable, gpio_backlight;

	if (machine_is_omap3530_lv_som()) {
		gpio_backlight = 8;
		gpio_enable = 155;
	} else if (machine_is_omap3_torpedo()) {
		gpio_backlight = 154;
		gpio_enable = 155;
		if (gpio_request(56, "LCD mdisp"))
			pr_err("omap3logic: can't request GPIO %d for LCD mdisp\n", gpio_backlight);
		gpio_direction_output(56, 1);
		omap_mux_init_gpio(56, OMAP_PIN_OUTPUT);
		
	} else
		BUG();

	*p_gpio_enable = gpio_enable;
	*p_gpio_backlight = gpio_backlight;

	if (gpio_request(gpio_backlight, "LCD backlight"))
		pr_err("omap3logic: can't request GPIO %d for LCD backlight\n", gpio_backlight);
	gpio_direction_output(gpio_backlight, 0);
	omap_mux_init_gpio(gpio_backlight, OMAP_PIN_OUTPUT);
	if (gpio_request(gpio_enable, "LCD enable"))
		pr_err("omap3logic: can't request GPIO %d for LCD enable\n", gpio_enable);
	gpio_direction_output(gpio_enable, 1);
	omap_mux_init_gpio(gpio_enable, OMAP_PIN_OUTPUT);

	/* Sleep for 600ms since hte 4.3" display needs
	 * power before turning on the clocks */
	msleep(600);

	/* Bring up backlight */
	gpio_direction_output(gpio_backlight, 1);

}

#define NAND_BLOCK_SIZE		SZ_128K
#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30
#define OMAP3LOGIC_NORFLASH_CS 2

#ifdef CONFIG_MACH_OMAP3530_LV_SOM
static struct mtd_partition omap3logic_nor_partitions[] = {
	{
		.name		= CONFIG_OMAP3LOGIC_NOR_PARTITION_ONE_NAME,
		.offset		= 0,
#if CONFIG_OMAP3LOGIC_NOR_PARTITION_ONE_SIZE != 0
		.size		= (CONFIG_OMAP3LOGIC_NOR_PARTITION_ONE_SIZE) << 17,
#else
		.size		= MTDPART_SIZ_FULL,
#endif
#if defined(CONFIG_OMAP3LOGIC_NOR_PARTITION_ONE_READONLY)
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
#endif
	},
#if (CONFIG_OMAP3LOGIC_NOR_PARTITION_ONE_SIZE != 0)
	/* file system */
	{
		.name		= CONFIG_OMAP3LOGIC_NOR_PARTITION_TWO_NAME,
		.offset		= MTDPART_OFS_APPEND,
#if CONFIG_OMAP3LOGIC_NOR_PARTITION_TWO_SIZE != 0
		.size		= (CONFIG_OMAP3LOGIC_NOR_PARTITION_TWO_SIZE) << 17,
#else
		.size		= MTDPART_SIZ_FULL,
#endif
#if defined(CONFIG_OMAP3LOGIC_NOR_PARTITION_TWO_READONLY)
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
#endif
	},
#endif
#if (CONFIG_OMAP3LOGIC_NOR_PARTITION_ONE_SIZE != 0) && defined(CONFIG_OMAP3LOGIC_NOR_PARTITION_TWO_SIZE)
#if (CONFIG_OMAP3LOGIC_NOR_PARTITION_TWO_SIZE != 0)
	/* 3rd partition */
	{
		.name		= CONFIG_OMAP3LOGIC_NOR_PARTITION_THREE_NAME,
		.offset		= MTDPART_OFS_APPEND,
#if CONFIG_OMAP3LOGIC_NOR_PARTITION_THREE_SIZE != 0
		.size		= (CONFIG_OMAP3LOGIC_NOR_PARTITION_THREE_SIZE) << 17,
#else
		.size		= MTDPART_SIZ_FULL,
#endif
#if defined(CONFIG_OMAP3LOGIC_NOR_PARTITION_THREE_READONLY)
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
#endif
	},
#endif
#endif
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
#endif

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
#if 0
	{
		.name		= CONFIG_OMAP3LOGIC_NAND_PARTITION_ONE_NAME,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x240000 */
#if CONFIG_OMAP3LOGIC_NAND_PARTITION_ONE_SIZE != 0
		.size = (CONFIG_OMAP3LOGIC_NAND_PARTITION_ONE_SIZE) << 17,
#else
		.size = (2046 - 17 -1) << 17,
#endif
#if defined (CONFIG_OMAP3LOGIC_NAND_PARTITION_ONE_READONLY)
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
#endif
	},
#if (CONFIG_OMAP3LOGIC_NAND_PARTITION_ONE_SIZE != 0)
	{
		.name		= CONFIG_OMAP3LOGIC_NAND_PARTITION_TWO_NAME,
		.offset		= MTDPART_OFS_APPEND,
#if CONFIG_OMAP3LOGIC_NAND_PARTITION_TWO_SIZE != 0
		.size = (CONFIG_OMAP3LOGIC_NAND_PARTITION_TWO_SIZE) << 17,
#else
		.size		= (2046 - (CONFIG_OMAP3LOGIC_NAND_PARTITION_ONE_SIZE) - 17 - 1)*(64 * 2048),
#endif
#if defined (CONFIG_OMAP3LOGIC_NAND_PARTITION_TWO_READONLY)
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
#endif
	},
#endif
#if (CONFIG_OMAP3LOGIC_NAND_PARTITION_ONE_SIZE != 0) && defined(CONFIG_OMAP3LOGIC_NAND_PARTITION_TWO_SIZE)
#if (CONFIG_OMAP3LOGIC_NAND_PARTITION_TWO_SIZE != 0)
	{
		.name		= CONFIG_OMAP3LOGIC_NAND_PARTITION_THREE_NAME,
		.offset		= MTDPART_OFS_APPEND,
#if CONFIG_OMAP3LOGIC_NAND_PARTITION_THREE_SIZE != 0
		.size = (CONFIG_OMAP3LOGIC_NAND_PARTITION_THREE_SIZE) << 17,
#else
		.size		= (2046 - (CONFIG_OMAP3LOGIC_NAND_PARTITION_ONE_SIZE) - (CONFIG_OMAP3LOGIC_NAND_PARTITION_TWO_SIZE) - 17 - 1)*(64 * 2048),
#endif
#if defined (CONFIG_OMAP3LOGIC_NAND_PARTITION_THREE_READONLY)
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
#endif
	},
#endif
#endif
#endif /* if 0 */
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

#if defined(CONFIG_USB_ISP1760_HCD_OMAP3LOGIC) || defined(CONFIG_USB_ISP1760_HCD_OMAP3LOGIC_MODULE) || defined(CONFIG_USB_ISP1760_HCD) || defined(CONFIG_USB_ISP1760_HCD)

// ISP1760 USB interrupt
#define OMAP3530LV_SOM_ISP1760_IRQ_GPIO		4
#define OMAP3TORPEDO_ISP1760_IRQ_GPIO		126

// ISP1760 SUSPEND
#define OMAP3530LV_SOM_ISP1760_SUSPEND_GPIO 182

static struct resource omap3logic_isp1760_resources[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = -EINVAL,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device omap3logic_isp1760 = {
	.name		= "isp1760",
	.id		= -1,
	.dev		= {
		.platform_data	= NULL,
	},
	.num_resources = ARRAY_SIZE(omap3logic_isp1760_resources),
	.resource = omap3logic_isp1760_resources,
};


static void omap3logic_init_isp1760(void)
{
	unsigned long cs_mem_base;
	unsigned int irq_gpio;

	// ISP1760 IRQ is an MMC1 data pin - need to update PBIAS
	// to get voltage to the device so the IRQ works correctly rather
	// than float below logic 1 and cause IRQ storm...
	if (machine_is_omap3_torpedo())
		omap3torpedo_fix_pbias_voltage();

	if (gpmc_cs_request(6, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for ISP1760\n");
		return;
	}
	
	omap3logic_isp1760_resources[0].start = cs_mem_base;
	omap3logic_isp1760_resources[0].end = cs_mem_base + 0xffff;

	if (machine_is_omap3530_lv_som()) {
		irq_gpio = OMAP3530LV_SOM_ISP1760_IRQ_GPIO;
		omap_mux_init_gpio(irq_gpio, OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE);
	} else if (machine_is_omap3_torpedo()) {
		irq_gpio = OMAP3TORPEDO_ISP1760_IRQ_GPIO;

		/* Since GPIO126 is routed on the OMAP35x to
		 * both sdmmc1_dat4 and cam_strobe we have to
		 * mux sdmm1_dat4 as a GPIO by hand */
		omap_mux_init_signal("sdmmc1_dat4.gpio_126", OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE);
	} else {
		printk(KERN_ERR "%s: Unknown machine\n", __FUNCTION__);
		return;
	}

	// Setup ISP1760 IRQ pin
	if (gpio_request(irq_gpio, "isp1760_irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for isp1760 IRQ\n",
		irq_gpio);
		return;
	}
	omap3logic_isp1760_resources[1].start = OMAP_GPIO_IRQ(irq_gpio);

	// Set IRQ as an input
	gpio_direction_input(irq_gpio);

	// Setup ISP data3 pin as GPIO
	omap_mux_init_gpio(OMAP3530LV_SOM_ISP1760_SUSPEND_GPIO, OMAP_PIN_INPUT);
	if (gpio_request(OMAP3530LV_SOM_ISP1760_SUSPEND_GPIO, "isp1760_suspend") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for isp1760 SUSPEND\n",
		       OMAP3530LV_SOM_ISP1760_SUSPEND_GPIO);
		gpio_free(irq_gpio);
		gpio_free(OMAP3530LV_SOM_ISP1760_IRQ_GPIO);
		return;
	}

        // Set SUSPEND as an input...
	gpio_direction_input(OMAP3530LV_SOM_ISP1760_SUSPEND_GPIO);

	if (platform_device_register(&omap3logic_isp1760) < 0) {
		printk(KERN_ERR "Unable to register isp1760 device\n");
		gpio_free(irq_gpio);
		gpio_free(OMAP3530LV_SOM_ISP1760_IRQ_GPIO);
		return;
	} else {
		pr_info("registered isp1760 platform_device\n");
	}
}
#else
static void omap3logic_init_isp1760(void)
{
}
#endif

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
	if (omap3logic_has_isp1760())
		omap3logic_init_isp1760();
	else
		omap3logic_init_ehci();
}

#ifdef CONFIG_MACH_OMAP3530_LV_SOM

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
#else
static void omap3logic_init_uf1050a_mux(void)
{
}
#endif

#ifdef CONFIG_WLAN_1273
static void omap3logic_init_murata_mux(void)
{
	printk(KERN_INFO "%s: setup murata mux signals\n", __FUNCTION__);

	if (gpio_request(OMAP3LOGIC_MURATA_WIFI_EN_GPIO, "wifi_en") != 0)
		pr_err("GPIO %i request for murata_en failed\n", OMAP3LOGIC_MURATA_WIFI_EN_GPIO);
	gpio_direction_output(OMAP3LOGIC_MURATA_WIFI_EN_GPIO, 0);
	omap_mux_init_gpio(OMAP3LOGIC_MURATA_WIFI_EN_GPIO, OMAP_PIN_OUTPUT);
	gpio_export(OMAP3LOGIC_MURATA_WIFI_EN_GPIO, 0);
#if 0
	if (gpio_request(OMAP3LOGIC_MURATA_BT_EN_GPIO, "bt_en") != 0)
		pr_err("GPIO %i request for bt_en failed\n", OMAP3LOGIC_MURATA_BT_EN_GPIO);
	gpio_direction_output(OMAP3LOGIC_MURATA_BT_EN_GPIO, 0);
	omap_mux_init_gpio(OMAP3LOGIC_MURATA_BT_EN_GPIO, OMAP_PIN_OUTPUT);
	gpio_export(OMAP3LOGIC_MURATA_WIFI_EN_GPIO, 0);
#endif

	/* Pull the enables out of reset */
	msleep(10);
	gpio_set_value(OMAP3LOGIC_MURATA_WIFI_EN_GPIO, 1);
#if 0
	gpio_set_value(OMAP3LOGIC_MURATA_BT_EN_GPIO, 1);
#endif
	msleep(10);

	/* Put them back into reset (so they go into low-power mode) */
	gpio_set_value(OMAP3LOGIC_MURATA_WIFI_EN_GPIO, 0);
#if 0
	gpio_set_value(OMAP3LOGIC_MURATA_BT_EN_GPIO, 0);
#endif

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
#else
static void omap3logic_init_murata_mux(void)
{
}
#endif

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

#ifdef CONFIG_BT_HCIBRF6300_SPI
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
#else
static void brf6300_dev_init(void)
{
}
#endif

extern void omap3logic_init_audio_mux(void);
extern void __init board_lcd_init(void);

//mark.chung
static void __init aircell_mux_init(void)
{
	// gpios for power
#if 0
	omap_mux_init_signal("uart1_cts.gpio_150", OMAP_PIN_INPUT);
	omap_mux_init_signal("uart1_rts.gpio_149", OMAP_PIN_INPUT);
	omap_mux_init_signal("sdmmc2_dat2.gpio_134", OMAP_PIN_INPUT);
	omap_mux_init_signal("mcbsp1_dr.gpio_159", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_hs.gpio_94", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_vs.gpio_95", OMAP_PIN_INPUT);
#endif
	//omap_mux_init_signal("jtag_emu1.gpio_31", OMAP_PIN_OUTPUT);
#if 0	
	if (gpio_request(31, "master reset")) {
		printk(KERN_ERR "gpio_request on gpio 31 failed.\n");
	}
	
	// etc
	omap_mux_init_signal("gpmc_nbe1.gpio_61", OMAP_PIN_INPUT);
#endif
}


static void __init omap3logic_init(void)
{
	//aircell_mux_init(); // mark.chung
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);

	omap3logic_i2c_init();

	board_lcd_init();

	platform_add_devices(omap3logic_devices, ARRAY_SIZE(omap3logic_devices));
#if 1 // mark.chung //def CONFIG_OMAP3LOGIC_UART_A
#if 0
	omap_mux_init_signal("jtag_emu1.gpio_31", OMAP_PIN_OUTPUT);
	if (gpio_request(31, "master reset")) {
		printk(KERN_ERR "gpio_request on gpio 31 failed.\n");
	}
	gpio_direction_output(31, 0);
#endif
	printk(KERN_INFO "Setup pinmux and enable UART A\n");
	omap_mux_init_signal("uart1_tx.uart1_tx", OMAP_PIN_OUTPUT);
//	omap_mux_init_signal("uart1_rts.uart1_rts", OMAP_PIN_INPUT);
//	omap_mux_init_signal("uart1_cts.uart1_cts", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart1_rx.uart1_rx", OMAP_PIN_INPUT);
	omap_serial_init_port(0);
#endif

//#ifdef CONFIG_OMAP3LOGIC_UART_B
#if 0
	printk(KERN_INFO "Setup pinmux and enable UART B\n");
	omap_mux_init_signal("uart3_tx_irtx.uart3_tx_irtx", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart3_rts_sd.uart3_rts_sd", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart3_cts_rctx.uart3_cts_rctx", OMAP_PIN_INPUT);
	omap_mux_init_signal("uart3_rx_irrx.uart3_rx_irrx", OMAP_PIN_INPUT);
	omap_serial_init_port(2);
#endif

#ifdef CONFIG_OMAP3LOGIC_UART_C
	printk(KERN_INFO "Setup pinmux and enable UART C\n");
	omap_mux_init_signal("uart2_tx.uart2_tx", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart2_rts.uart2_rts", OMAP_PIN_INPUT);
	omap_mux_init_signal("uart2_cts.uart2_cts", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart2_rx.uart2_rx", OMAP_PIN_INPUT);
	omap_serial_init_port(1);
#endif

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
}

#if defined(CONFIG_OMAP3LOGIC_COMPACT_FLASH) || defined(CONFIG_OMAP3LOGIC_COMPACT_FLASH_MODULE)

#define OMAP_LV_SOM_CF_RESET_GPIO 6
#define OMAP_LV_SOM_CF_EN_GPIO 128
#define OMAP_LV_SOM_CF_CD_GPIO 154

static struct resource omap3logic_lv_som_cf_resources[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = OMAP_GPIO_IRQ(OMAP_LV_SOM_CF_CD_GPIO),
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct omap3logic_cf_data cf_data = {
	.gpio_reset = OMAP_LV_SOM_CF_RESET_GPIO,
	.gpio_en = OMAP_LV_SOM_CF_EN_GPIO,
	.gpio_cd = OMAP_LV_SOM_CF_CD_GPIO,
};

static struct platform_device omap3logic_lv_som_cf = {
	.name		= "omap3logic-cf",
	.id		= 0,
	.dev		= {
		.platform_data	= &cf_data,
	},
	.num_resources = ARRAY_SIZE(omap3logic_lv_som_cf_resources),
	.resource = omap3logic_lv_som_cf_resources,
};

void omap3logic_cf_init(void)
{
	unsigned long cs_mem_base;
	int result;

	/* Only the LV SOM SDK has a CF interface */
	if (!machine_is_omap3530_lv_som())
		return;

	/* Fix PBIAS to get USIM enough voltage to power up */
	omap3torpedo_fix_pbias_voltage();

	if (gpmc_cs_request(3, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for CF\n");
		return;
	}

	omap3logic_lv_som_cf_resources[0].start = cs_mem_base;
	omap3logic_lv_som_cf_resources[0].end = cs_mem_base + 0x1fff;
	

	omap_mux_init_signal("gpmc_ncs3", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_io_dir", OMAP_PIN_OUTPUT);

	omap_mux_init_gpio(OMAP_LV_SOM_CF_CD_GPIO, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(OMAP_LV_SOM_CF_CD_GPIO, "CF card detect") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for CompactFlash CD IRQ\n",
		OMAP_LV_SOM_CF_CD_GPIO);
		return;
	}
	omap_set_gpio_debounce(OMAP_LV_SOM_CF_CD_GPIO, 5);
	gpio_direction_input(OMAP_LV_SOM_CF_CD_GPIO);

	// Setup ComapctFlash Enable pin
	omap_mux_init_gpio(OMAP_LV_SOM_CF_EN_GPIO, OMAP_PIN_OUTPUT);
	if (gpio_request(OMAP_LV_SOM_CF_EN_GPIO, "CF enable") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for CompactFlash EN\n",
		OMAP_LV_SOM_CF_EN_GPIO);
		return;
	}
	gpio_direction_output(OMAP_LV_SOM_CF_EN_GPIO, 0);

	// Setup ComapctFlash Reset pin
	omap_mux_init_gpio(OMAP_LV_SOM_CF_RESET_GPIO, OMAP_PIN_OUTPUT);
	if (gpio_request(OMAP_LV_SOM_CF_RESET_GPIO, "CF reset") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for CompactFlash Reset\n",
		OMAP_LV_SOM_CF_RESET_GPIO);
		return;
	}
	gpio_direction_output(OMAP_LV_SOM_CF_RESET_GPIO, 0);

	result = platform_device_register(&omap3logic_lv_som_cf);
	if (result)
		printk("%s: platform device register of CompactFlash device failed: %d\n", __FUNCTION__, result);
}
#else
static void omap3logic_cf_init(void)
{
}
#endif


void omap3logic_init_productid_specifics(void)
{
	omap3logic_init_twl_external_mute();
	aircell_mux_init(); // mark.chung
	omap3logic_led_init();
	brf6300_dev_init();
	omap3logic_cf_init();
}

static void __init omap3logic_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP3_TORPEDO, "Logic OMAP3 Torpedo board")
	/* Maintainer: Peter Barada <peterb@logicpd.com> */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3logic_map_io,
	.init_irq	= omap3logic_init_irq,
	.init_machine	= omap3logic_init,
	.timer		= &omap_timer,
MACHINE_END

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
