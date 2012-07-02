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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/leds.h>
#include <linux/interrupt.h>

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
#include <plat/dm3730logic-productid.h>
#include <plat/omap3logic-cf.h>
#include <plat/wifi_tiwlan.h>

#include "mux.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "omap3-opp.h"
#include "board-cloudsurfer.h"
#include <plat/sdrc.h>
#include <linux/i2c/at24.h>
#include <linux/leds-pca9626.h>
#include "cloudsurfer-gpio.h"

extern void print_omap_clocks(void);

#define QT_I2C_ADDR			 0x4b
#define DIE_ID_REG_BASE		(L4_WK_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET	0x218
#define MAX_USB_SERIAL_NUM	17

#define OMAP3LOGIC_SMSC911X_CS		1
#define OMAP3LOGIC_LV_SOM_SMSC911X_GPIO		152	/* LV SOM LAN IRQ */
#define OMAP3LOGIC_TORPEDO_SMSC911X_GPIO	129	/* Torpedo LAN IRQ */

/* Using default VMMC1 and VSIM configurations */
extern struct regulator_consumer_supply twl4030_vmmc1_supply;
extern struct regulator_init_data vmmc1_data;
extern struct regulator_consumer_supply twl4030_vsim_supply; 
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
	.phy_interface	= PHY_INTERFACE_MODE_MII,
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type		= SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags			= (SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS)
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
	//omap3logic_smsc911x_resources[0].end	= cs_mem_base + 0xff;
	omap3logic_smsc911x_resources[0].end	= cs_mem_base + 0x0f;

	if (gpio_request(eth_gpio, "eth0") < 0) {
		printk(KERN_ERR "Failed to request GPIO_%d for smsc911x IRQ\n",
				eth_gpio);
		return;
	}
	gpio_direction_input(eth_gpio);

	omap3logic_smsc911x_resources[1].start = OMAP_GPIO_IRQ(eth_gpio);

	if (dm3730logic_extract_lan_ethaddr(&smsc911x_config.mac[0])) {
		printk(KERN_INFO "smsc911x: using production MAC address\n");
	}

	if (platform_device_register(&omap3logic_smsc911x_device) < 0) {
		printk(KERN_ERR "Unable to register smsc911x device\n");
		return;
	}
};

/* Power stuff */

static char *cloudsurfer_supplicants[] = {
    "bq27500"
};

static struct gpio_charger_platform_data cloudsurfer_charger_pdata = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.gpio = AIRCELL_POWER_APPLIED_DETECT,
	.gpio_active_low = 1,
	.supplied_to = cloudsurfer_supplicants,
	.num_supplicants = ARRAY_SIZE(cloudsurfer_supplicants),
};

static struct platform_device cloudsurfer_charger_device = {
	.name = "gpio-charger",
	.dev = {
		.platform_data = &cloudsurfer_charger_pdata,
	},
};

/*
 * GPIO Buttons
 */
static struct gpio_keys_button cs_volume_buttons[] = {
    {
        .gpio       = AIRCELL_VOLUME_UP_DETECT,
        .code       = KEY_VOLUMEUP,
        .desc       = "VOLUME_UP",
        .type       =  EV_KEY,
        .active_low = 1,
        .wakeup     = 1,
    },
    {
        .gpio       = AIRCELL_VOLUME_DOWN_DETECT,
        .code       = KEY_VOLUMEDOWN,
        .desc       = "VOLUME_DOWN",
        .type       =  EV_KEY,
        .active_low = 1,
        .wakeup     = 1,
    }
};

static struct gpio_keys_platform_data volume_button_data = {
    .buttons    = cs_volume_buttons,
    .nbuttons   = ARRAY_SIZE(cs_volume_buttons),
};

static struct platform_device volume_buttons = {
    .name       = "gpio-keys",
    .id     = -1,
    .num_resources  = 0,
    .dev        = {
        .platform_data  = &volume_button_data,
    }
};

static struct gpio_switch_platform_data headset_switch_data = {
	.name = "h2w",
	.gpio = AIRCELL_HEADSET_DETECT,
};

static struct platform_device headset_jack = {
    .name       = "switch-gpio",
    .id     = -1,
    .dev        = {
        .platform_data  = &headset_switch_data,
    }
};

static struct platform_device *cloudsurfer_devices[] __initdata = {
    &volume_buttons,
    &headset_jack,
};


/* Fix the PBIAS voltage for GPIO-129 */
static void fix_pbias_voltage(void)
{
	u16 control_pbias_offset = OMAP343X_CONTROL_PBIAS_LITE;
	static int pbias_fixed = 0;
	u32 reg;

	if (!pbias_fixed) {
		/* Set the bias for the pin */
		reg = omap_ctrl_readl(control_pbias_offset);

		//reg &= ~OMAP343X_PBIASLITEVMODE1;
		reg &= ~OMAP343X_PBIASLITEPWRDNZ1;
		omap_ctrl_writel(reg, control_pbias_offset);
		/* 100ms delay required for PBIAS configuration */
		msleep(100);

		reg &= ~OMAP343X_PBIASLITEVMODE1;
		//reg |= OMAP343X_PBIASLITEVMODE1;
		reg |= OMAP343X_PBIASLITEPWRDNZ1;
		omap_ctrl_writel(reg, control_pbias_offset);

		/* For DM3730, turn on GPIO_IO_PWRDNZ to connect input pads*/
		if (cpu_is_omap3630()) {
			reg = omap_ctrl_readl(OMAP343X_CONTROL_WKUP_CTRL);
			reg |= OMAP343X_GPIO_IO_PWRDNZ;
			omap_ctrl_writel(reg, OMAP343X_CONTROL_WKUP_CTRL);
			printk("%s:%d PKUP_CTRL %#x\n", __FUNCTION__, __LINE__, omap_ctrl_readl(OMAP343X_CONTROL_WKUP_CTRL));
		}
		pbias_fixed = 1;

	}
}

/* TARR - The OPP clock rates need to be specific to CloudSurfer as we 
 * are using the Extended TEmperature Range versio of the DM3730 that 
 * does not support the higher MPU/DSP clock rates
 */
static struct omap_opp cloudsurfer_mpu_rate_table[] = {
    {0, 0, 0},
    /*OPP1 (OPP50)*/
    {S125M, VDD1_OPP1, 0x1B},
    /*OPP2 (OPP100)*/
    {S300M, VDD1_OPP2, 0x28},
    /*OPP3 (OPP120)*/
    {S600M, VDD1_OPP3, 0x35},
    /*OPP4 (OPPTM)*/
    {S800M, VDD1_OPP4, 0x35},
};

struct omap_opp cloudsurfer_dsp_rate_table[] = {
    {0, 0, 0},
    /*OPP1 (OPP50) */
    {S90M, VDD1_OPP1, 0x1B},
    /*OPP2 (OPP100) */
    {S260M, VDD1_OPP2, 0x28},
    /*OPP3 (OPP120) */
    {S520M, VDD1_OPP3, 0x35},
    /*OPP4 (OPPTM) */
    {S660M, VDD1_OPP4, 0x3C},
};

struct omap_opp cloudsurfer_l3_rate_table[] = {
    {0, 0, 0},
    /*OPP1 (OPP50)  */
    {S100M, VDD2_OPP1, 0x1B},
    /*OPP2 (OPP100) */
    {S200M, VDD2_OPP2, 0x2B},
};


/* TARR - FIXME: These values need to be updated based on more profiling
 * Originals came from the 3430sdp
 */
static struct cpuidle_params cloudsurfer_cpuidle_params_table[] = {
	/* C1 */
	{1, 2, 2, 5},
	/* C2 */
	{1, 10, 10, 30},
	/* C3 */
	{1, 50, 50, 300},
	/* C4 */
	{1, 1500, 1800, 4000},
	/* C5 */
	{1, 2500, 7500, 12000},
	/* C6 */
	{1, 3000, 8500, 15000},
	/* C7 */
	{1, 10000, 30000, 300000},
};

static struct prm_setup_vc cloudsurfer_setuptime_table = {
	.clksetup = 0xff,
	.voltsetup_time1 = 0xfff,
	.voltsetup_time2 = 0xfff,
	.voltoffset = 0xff,
	.voltsetup2 = 0xff,
	.vdd0_on = 0x30,
	.vdd0_onlp = 0x20,
	.vdd0_ret = 0x1e,
	.vdd0_off = 0x00,
	.vdd1_on = 0x2c,
	.vdd1_onlp = 0x20,
	.vdd1_ret = 0x1e,
	.vdd1_off = 0x00,
};



/* VDDA_DAC needed for dss */
static struct regulator_consumer_supply omap3logic_vdda_dac_supply = {
	.supply		 = "vdda_dac",
	.dev		= &dss_device.dev,
};

static struct regulator_init_data omap3logic_vdda_dac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
		.valid_ops_mask	= REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies		= &omap3logic_vdda_dac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_consumer_supply omap3logic_vpll2_supplies[] = {
	{
		.supply	 = "vdvi",
		.dev		= &lcd_device.dev,
	},
	{
		.supply	 = "vdds_dsi",
		.dev		= &dss_device.dev,
	}
};

static struct regulator_init_data omap3logic_vpll2 = {
	.constraints = {
	 	.name			 = "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
		.valid_ops_mask	= REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(omap3logic_vpll2_supplies),
	.consumer_supplies		= omap3logic_vpll2_supplies,
};

/* VAUX1 for mainboard (touch and productID) */
static struct regulator_consumer_supply omap3logic_vaux1_supply = {
	.supply			= "vax1",
};

static struct regulator_init_data omap3logic_vaux1 = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.name			= "VAUX1_30",
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL |	REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3logic_vaux1_supply,
};

/* VAUX3 required to enable WiLink 26MHz clock */
static struct regulator_consumer_supply omap3logic_vaux3_supply = {
	.supply		 = "vaux3",
};

static struct regulator_init_data omap3logic_vaux3 = {
	.constraints = {
		.min_uV		 = 2800000,
		.max_uV		 = 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3logic_vaux3_supply,
};

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc		= 3,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{}	/* Terminator */
};

static int twl4030_base_gpio;

int omap3logic_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* SD card detect interrupt */
	mmc[0].gpio_cd = 110;
	omap_mux_init_gpio(110, OMAP_PIN_INPUT_PULLUP);
	/* Since GPIO126 is routed on the OMAP35x to
	 * both sdmmc1_dat4 and cam_strobe we have to
	 * mux cam_strome/gpio_126 as a GPIO by hand */
	//omap_mux_init_signal("cam_strobe.gpio_126", OMAP_PIN_INPUT_PULLUP);
	//mmc[0].gpio_wp = 126;


	/* link regulators to MMC adapters */
	twl4030_vmmc1_supply.dev = mmc[0].dev;
	twl4030_vsim_supply.dev = mmc[0].dev;

	twl4030_base_gpio = gpio;

	printk(KERN_INFO "%s: TWL4030 base gpio: %d\n", __FUNCTION__, gpio);

	twl4030_mmc_init(mmc);

	return 0;
}

static struct twl4030_gpio_platform_data omap3logic_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	//.use_leds	= true,
	.setup		= omap3logic_twl_gpio_setup,
};

#ifdef USE_USB
static struct twl4030_usb_data omap3logic_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};
#endif

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

static struct twl4030_platform_data omap3logic_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &omap3logic_madc_data,
#ifdef USE_USB
	.usb		= &omap3logic_usb_data,
#endif
	.gpio		= &omap3logic_gpio_data,
	.codec		= &omap3logic_codec_data,

	.vmmc1		= &vmmc1_data,
	.vaux1		= &omap3logic_vaux1,
	.vaux3		= &omap3logic_vaux3,
	.vpll2		= &omap3logic_vpll2,
	.vdac		= &omap3logic_vdda_dac,
};

static struct i2c_board_info __initdata omap3logic_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &omap3logic_twldata,
	},
};

/* 
 * Devices on I2C bus 2 are the EEPROM and the LED controller
 */

/* Name for the LED regions - The names are associated with
 * the LED number of the controller chip 
 */

static struct pca9626_platform_data cloud_pca9626_data = {
	.leds = {
		{	.id = 0,
			.ldev.name = "LED_ZONE_5",
			.ldev.brightness = 16,
		},
		{	.id = 1,
			.ldev.name = "LED_ZONE_8",
			.ldev.brightness = 16,
		},
		{	.id = 2,
			.ldev.name = "LED_ZONE_2",
			.ldev.brightness = 16,
		},
		{	.id = 3,
			.ldev.name = "LED_ZONE_6",
			.ldev.brightness = 16,
		},
		{	.id = 4,
			.ldev.name = "LED_ZONE_3",
			.ldev.brightness = 16,
		},
		{	.id = 5,
			.ldev.name = "LED_ZONE_MENU",
			.ldev.brightness = 16,
		},
		{	.id = 6,
			.ldev.name = "LED_ZONE_HOME",
			.ldev.brightness = 16,
		},
		{	.id = 11,
			.ldev.name = "LED_ZONE_RED",
			.ldev.brightness = 16,
		},
		{	.id = 12,
			.ldev.name = "LED_ZONE_BACK",
			.ldev.brightness = 16,
		},
		{	.id = 13,
			.ldev.name = "LED_ZONE_9",
			.ldev.brightness = 16,
		},
		{	.id = 14,
			.ldev.name = "LED_ZONE_POUND",
			.ldev.brightness = 16,
		},
		{	.id = 15,
			.ldev.name = "LED_ZONE_0",
			.ldev.brightness = 16,
		},
		{	.id = 16,
			.ldev.name = "LED_ZONE_ASTRIX",
			.ldev.brightness = 16,
		},
		{	.id = 17,
			.ldev.name = "LED_ZONE_7",
			.ldev.brightness = 16,
		},
		{	.id = 18,
			.ldev.name = "LED_ZONE_4",
			.ldev.brightness = 16,
		},
		{	.id = 19,
			.ldev.name = "LED_ZONE_1",
			.ldev.brightness = 16,
		},
		{	.id = 20,
			.ldev.name = "LED_ZONE_S1",
			.ldev.brightness = 16,
		},
		{	.id = 21,
			.ldev.name = "LED_ZONE_S2",
			.ldev.brightness = 16,
		},
		{	.id = 22,
			.ldev.name = "LED_ZONE_S3",
			.ldev.brightness = 16,
		},
		{	.id = 23,
			.ldev.name = "BACKLIGHT",
			.ldev.brightness = 16,
		},
	},
};

static struct at24_platform_data m24c128 = { 
			.byte_len		= 131072 / 8,
			.page_size		= 64, 
			.flags			= AT24_FLAG_ADDR16,
};

	
static struct i2c_board_info __initdata omap3logic_i2c2_boardinfo[] = {
	{	
		I2C_BOARD_INFO("at24", 0x50),
		.platform_data = &m24c128,
	},
	{	
		I2C_BOARD_INFO("pca9626", 0x12),
		.platform_data = &cloud_pca9626_data,
	},
	{
		/* Place holder for Battery Fuel Guage */
		//I2C_BOARD_INFO("bq27200", 0x55),
	},
	{
		/* Place holder for Battery Charger */
	},
};
/*
 * D2vices on I2C bus 3 are the touchscreen controller\
 */

/*
 * Touch Screen Support
 */

struct qt602240_platform_data omap3logic_touchscreendata = {
	.x_line = 19,
	.y_line = 11,
	.x_size = 1170,
	.y_size = 480,
	.blen = 23,
	.threshold = 60,
	.voltage = 600,
	.orient = QT602240_NORMAL
};

static struct platform_device omap3logic_touch_device = {
	.name		= "qt602240_ts",
	.id	 	= 0, 
	.dev		= {
		.platform_data = &omap3logic_touchscreendata,
	},
};


static struct i2c_board_info __initdata omap3logic_i2c3_boardinfo[] = {
	{
		I2C_BOARD_INFO("qt602440_ts", QT_I2C_ADDR),
		.type		= "qt602240_ts",
		.platform_data = &omap3logic_touchscreendata,
		.irq = OMAP_GPIO_IRQ(AIRCELL_TOUCH_INTERRUPT),
	},
};

static void omap3logic_qt602240_init(void)
{
	printk("QT602240 Init");
	if (platform_device_register(&omap3logic_touch_device) < 0){
			printk(KERN_ERR "Unable to register touch device\n");
		return;
	}
	omap_mux_init_gpio(AIRCELL_TOUCH_INTERRUPT, 
			OMAP_PIN_INPUT_PULLUP | OMAP_WAKEUP_EN | OMAP_MUX_MODE4);
	omap_set_gpio_debounce(AIRCELL_TOUCH_INTERRUPT, 1);
	omap_set_gpio_debounce_time(AIRCELL_TOUCH_INTERRUPT, 0xa);

	/* 5V Digital is required for the touchscreen controller */
	gpio_direction_output(AIRCELL_5VD_ENABLE, 1);

	/* Take the touch screen out of reset */
	gpio_direction_output(AIRCELL_TOUCH_RESET, 1);
	omap3logic_i2c3_boardinfo[0].irq = gpio_to_irq(AIRCELL_TOUCH_INTERRUPT);

	return;
}

int __init omap3logic_i2c_init(void)
{
	printk("Cloudsurfer I2C Init");
	omap_register_i2c_bus(1, 2600, omap3logic_i2c1_boardinfo,
			ARRAY_SIZE(omap3logic_i2c1_boardinfo));
	/* Check to see if we need to add in the Battery Charger Controller
     * and the Battery Fuel Gauge. This is done based on the 
     * AIRCELL_BATTERY_POWERED gpio. If the pin is high, it is a
     * battery powered phone and we need to add the two devices
     * to the I2C2 boardinfo
     */
	if ( gpio_get_value(AIRCELL_BATTERY_POWERED) == 1 ) {
		strcpy(&omap3logic_i2c2_boardinfo[2].type[0],"cloudsurfer-charger");
		omap3logic_i2c2_boardinfo[2].addr = 0x41;
		omap3logic_i2c2_boardinfo[2].platform_data = &cloudsurfer_charger_pdata;
		strcpy(&omap3logic_i2c2_boardinfo[3].type[0],"bq27500");
		omap3logic_i2c2_boardinfo[3].addr = 0x55;
		printk("Cloudsurfer is Battery Powered");
	} else {
		printk("Cloudsurfer is POE Powered");
	}
	omap_register_i2c_bus(2, 400, omap3logic_i2c2_boardinfo,
			ARRAY_SIZE(omap3logic_i2c2_boardinfo));
	omap_register_i2c_bus(3, 400, omap3logic_i2c3_boardinfo,
			ARRAY_SIZE(omap3logic_i2c3_boardinfo));
	return 0;
}

static struct omap_board_config_kernel omap3logic_config[] __initdata = {
};


void __init omap3logic_init_irq(void)
{
	omap_board_config = omap3logic_config;
	omap_board_config_size = ARRAY_SIZE(omap3logic_config);
	/* TARR HERE - Setup pwoer management tables */
	omap3_pm_init_cpuidle(cloudsurfer_cpuidle_params_table);
	omap3_pm_init_vc(&cloudsurfer_setuptime_table);

	omap2_init_common_hw(omap3logic_get_sdram_timings(), NULL, 
				 cloudsurfer_mpu_rate_table, cloudsurfer_dsp_rate_table, 
				 cloudsurfer_l3_rate_table);
	omap_init_irq();
	omap_gpio_init();
}

static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

/* Initialization code called from LCD panel_init to allow muxing */
void omap3logic_lcd_panel_init(int *p_gpio_enable, int *p_gpio_backlight)
{
	*p_gpio_enable = 0;
	*p_gpio_backlight = 0;
}

#ifdef USE_USB
static struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset	= true,
	.reset_gpio_port[0]	= -EINVAL,
	.reset_gpio_port[1]	= 4,
	.reset_gpio_port[2]	= -EINVAL
};

void omap3logic_init_ehci(void)
{
	omap_mux_init_gpio(ehci_pdata.reset_gpio_port[1], OMAP_PIN_OUTPUT);
	usb_ehci_init(&ehci_pdata);
}

void omap3logic_usb_init(void)
{
	omap3logic_init_ehci();
}

#endif

static void wifi_init(void)
{
	struct clk *sys_clkout1_clk;

	/* Pull the enables out of reset */
	msleep(10);
	gpio_set_value(OMAP_DM3730LOGIC_WIFI_PMENA_GPIO, 1);
	/* Put them back into reset (so they go into low-power mode) */
	msleep(10);
	gpio_set_value(OMAP_DM3730LOGIC_WIFI_PMENA_GPIO, 0);

	/* Enable sys_clkout1 (uP_CLKOUT1_26Mhz) */
	sys_clkout1_clk = clk_get(NULL, "sys_clkout1");
	if (IS_ERR(sys_clkout1_clk)) {
		printk("%s: Can't get sys_clkout1\n", __FUNCTION__);
	} else {
		clk_enable(sys_clkout1_clk);
	}
	// Setup the mux for mmc3
	omap_mux_init_signal("mcspi1_cs1.sdmmc3_cmd", OMAP_PIN_INPUT_PULLUP); /* McSPI1_CS1/ADPLLV2D_DITHERING_EN2/MMC3_CMD/GPIO_175 */
	omap_mux_init_signal("mcspi1_cs2.sdmmc3_clk", OMAP_PIN_INPUT_PULLUP); /* McSPI1_CS2/MMC3_CLK/GPIO_176 */
	omap_mux_init_signal("sdmmc2_dat4.sdmmc3_dat0", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT4/MMC2_DIR_DAT0/MMC3_DAT0/GPIO_136 */
	omap_mux_init_signal("sdmmc2_dat5.sdmmc3_dat1", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT5/MMC2_DIR_DAT1/CAM_GLOBAL_RESET/MMC3_DAT1/HSUSB3_TLL_STP/MM3_RXDP/GPIO_137 */
	omap_mux_init_signal("sdmmc2_dat6.sdmmc3_dat2", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT6/MMC2_DIR_CMD/CAM_SHUTTER/MMC3_DAT2/HSUSB3_TLL_DIR/GPIO_138 */
	omap_mux_init_signal("sdmmc2_dat7.sdmmc3_dat3", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT7/MMC2_CLKIN/MMC3_DAT3/HSUSB3_TLL_NXT/MM3_RXDM/GPIO_139 */

	omap_mux_init_signal("sys_boot0.gpio_2", OMAP_PIN_INPUT);

}

#ifdef USE_USB
void omap3logic_musb_init(void)
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
extern void omap3logic_init_audio_mux(void);
extern void __init board_lcd_init(void);


static char device_serial[MAX_USB_SERIAL_NUM];

#ifdef USE_USB
static char *usb_functions_adb[] = {
	"adb",
};

static char *usb_functions_all[] = {
	"adb",
};

static struct android_usb_product usb_products[] = {
	{
		.num_functions = ARRAY_SIZE(usb_functions_adb),
		.functions	 = usb_functions_adb,
	},
};

static struct android_usb_platform_data andusb_plat = {
	.manufacturer_name	 = "LogicPD",
	.product_name			= "DM3730 SOM LV",
	.serial_number		 = device_serial,
	.functions			 = usb_functions_all,
	.products				= usb_products,
	.num_functions		 = ARRAY_SIZE(usb_functions_all),
};

static struct platform_device androidusb_device = {
	.name					= "android_usb",
	.id					= -1,
	.dev					= {
		.platform_data = &andusb_plat,
	},
};

void omap3logic_android_gadget_init(void)
{
	unsigned int val[2];
	unsigned int reg;

	reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;
	val[0] = omap_readl(reg);
	val[1] = omap_readl(reg + 4);

	snprintf(device_serial, MAX_USB_SERIAL_NUM, "%08X%08X", val[1], val[0]);

	platform_device_register(&androidusb_device);
}
#endif

void cloudsurfer_gpio_init(void)
{

	/* Someone is dickering wiht the I2C3 pins... Fix it here */
#define I2C3_SCLK	184
#define I2C3_SDATA 185
	omap_mux_init_gpio(I2C3_SCLK, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(I2C3_SDATA, OMAP_PIN_INPUT_PULLUP);
    omap_mux_init_gpio(OMAP_DM3730LOGIC_WIFI_PMENA_GPIO, OMAP_PIN_OUTPUT);

	gpio_request(OMAP_DM3730LOGIC_WIFI_PMENA_GPIO, "wifi_en");
    gpio_direction_output(OMAP_DM3730LOGIC_WIFI_PMENA_GPIO, 0);
    gpio_export(OMAP_DM3730LOGIC_WIFI_PMENA_GPIO, 0);
	gpio_request(AIRCELL_5VA_ENABLE,"AIRCELL_5VA_ENABLE");
	gpio_request(AIRCELL_5VA_ENABLE,"AIRCELL_5VA_ENABLE");
	gpio_request(AIRCELL_5VD_ENABLE,"AIRCELL_5VD_ENABLE");
	gpio_direction_output(AIRCELL_5VA_ENABLE,0);
	gpio_direction_output(AIRCELL_5VD_ENABLE,0);
	gpio_export(AIRCELL_5VA_ENABLE,0);
	gpio_export(AIRCELL_5VD_ENABLE,0);
	gpio_request(AIRCELL_18V_ENABLE,"AIRCELL_18V_ENABLE");
	gpio_request(AIRCELL_SOFTWARE_RESET,"AIRCELL_SOFTWARE_RESET");
	gpio_request(AIRCELL_BATTERY_POWERED,"AIRCELL_BATTERY_POWERED");
	gpio_request(AIRCELL_LCD_RESET,"AIRCELL_LCD_RESET");
	gpio_request(AIRCELL_POWER_APPLIED_DETECT,"AIRCELL_POWER_APPLIED_DETECT");
	gpio_request(AIRCELL_LED_ENABLE,"AIRCELL_LED_ENABLE");
	gpio_request(AIRCELL_EARPIECE_ENABLE,"AIRCELL_EARPIECE_ENABLE");
	gpio_request(AIRCELL_RINGER_ENABLE,"AIRCELL_RINGER_ENABLE");
	gpio_request(AIRCELL_TOUCH_RESET,"AIRCELL_TOUCH_RESET");
	gpio_request(AIRCELL_PROX_INTERRUPT,"AIRCELL_PROX_INTERRUPT");
	gpio_request(AIRCELL_ACCEL_INTERRUPT,"AIRCELL_ACCEL_INTERRUPT");
	gpio_request(AIRCELL_TOUCH_INTERRUPT,"AIRCELL_TOUCH_INTERRUPT");
	gpio_request(AIRCELL_BATTERY_CUT_ENABLE,"AIRCELL_BATTERY_CUT_ENABLE");
	gpio_request(AIRCELL_BACKLIGHT_ENABLE,"AIRCELL_BACKLIGHT_ENABLE");
	gpio_request(AIRCELL_TOUCH_INTERRUPT,"AIRCELL_TOUCH_INTERRUPT");
	gpio_request(AIRCELL_MUTE,"AIRCELL_MUTE");

	gpio_direction_input(AIRCELL_BATTERY_POWERED);
	gpio_direction_output(AIRCELL_18V_ENABLE,1);
	gpio_direction_output(AIRCELL_SOFTWARE_RESET,1); /* active low */
	gpio_direction_output(AIRCELL_LCD_RESET,0);
	gpio_direction_input(AIRCELL_POWER_APPLIED_DETECT);
	gpio_direction_output(AIRCELL_LED_ENABLE,1);
	gpio_direction_output(AIRCELL_EARPIECE_ENABLE,0);
	gpio_direction_output(AIRCELL_RINGER_ENABLE,0);
	gpio_direction_output(AIRCELL_TOUCH_RESET,0);
	gpio_direction_output(AIRCELL_BATTERY_CUT_ENABLE,0);
	gpio_direction_output(AIRCELL_BACKLIGHT_ENABLE,0);
	gpio_direction_input(AIRCELL_PROX_INTERRUPT);
	gpio_direction_output(AIRCELL_MUTE,0);

	gpio_export(AIRCELL_18V_ENABLE,0);
	gpio_export(AIRCELL_SOFTWARE_RESET,0);
	gpio_export(AIRCELL_BATTERY_POWERED,0);
	gpio_export(AIRCELL_LCD_RESET,0);
	gpio_export(AIRCELL_POWER_APPLIED_DETECT,0);
	gpio_export(AIRCELL_LED_ENABLE,0);
	gpio_export(AIRCELL_EARPIECE_ENABLE,0);
	gpio_export(AIRCELL_RINGER_ENABLE,0);
	gpio_export(AIRCELL_TOUCH_RESET,0);
	gpio_export(AIRCELL_PROX_INTERRUPT,0);
	gpio_export(AIRCELL_ACCEL_INTERRUPT,0);
	gpio_export(AIRCELL_TOUCH_INTERRUPT,0);
	gpio_export(AIRCELL_BATTERY_CUT_ENABLE,0);
	gpio_export(AIRCELL_BACKLIGHT_ENABLE,0);
	gpio_export(AIRCELL_MUTE,0);

}

static void __init omap3logic_init(void)
{
	printk("Aircell CloudSurfer P3\n");

	omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);
	
	fix_pbias_voltage();

	cloudsurfer_gpio_init();

	omap3logic_qt602240_init();

	omap3logic_i2c_init();

	platform_add_devices(cloudsurfer_devices, ARRAY_SIZE(cloudsurfer_devices));

	board_lcd_init();

	omap_serial_init_port(0);
	omap_serial_init_port(1);

#ifdef USE_USB
	omap3logic_usb_init();
	omap3logic_musb_init();
	omap3logic_android_gadget_init();
#endif

	/* Check the SRAM for valid product_id data(put there by
	 * u-boot). If not, then it will be read later. */
	dm3730logic_fetch_sram_product_id_data();

	dm3730logic_flash_init();

	omap3logic_init_smsc911x();

    /* Check to see if we need to add in the WiFi controller
     * This is done based on the * AIRCELL_BATTERY_POWERED gpio. 
	 * If the pin is high, it is a battery powered phone and we need 
     * initialize the WiFi controller 
     */
    if ( gpio_get_value(AIRCELL_BATTERY_POWERED) == 1 ) {
		wifi_init();
	}

	omap3logic_init_audio_mux();

	/* Must be here since on exit, omap2_init_devices(called later)
	 * setups up SPI devices - can't add boardinfo afterwards */
	//omap3logic_spi_init();

	dump_omap3logic_timings();


	print_omap_clocks();
}

void cloudsurfer_init_productid_specifics(void)
{
	omap3logic_init_twl_external_mute();
}

static void __init omap3logic_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(DM3730_SOM_LV, "Aircell CloudSurfer DM3730")
	.phys_io		= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io			= omap3logic_map_io,
	.init_irq		= omap3logic_init_irq,
	.init_machine	= omap3logic_init,
	.timer			= &omap_timer,
MACHINE_END
