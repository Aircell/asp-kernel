/*
 * linux/arch/arm/mach-omap2/board-dm3730logic.c
 *
 * Copyright (C) 2011 Logic Product Development
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/input/matrix_keypad.h>
#include <linux/leds.h>
#include <linux/interrupt.h>
#include <linux/backlight.h>
#include <linux/pda_power.h>

#include <linux/spi/spi.h>
#include <linux/spi/brf6300.h>

#include <linux/i2c/twl.h>
#include <linux/i2c/tsc2004.h>
#include <linux/usb/otg.h>
#include <linux/usb/isp1763.h>
#include <linux/smsc911x.h>

#include <linux/regulator/machine.h>
#include <linux/usb/android_composite.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/gpmc.h>
#include <plat/usb.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/mcspi.h>
#include <plat/clock.h>
#include <plat/omap-pm.h>
#include <plat/display.h>
#include <plat/dm3730logic-cf.h>
#include <plat/wifi_tiwlan.h>

#include "mux.h"
#include "sdram-micron-mt29c4g48mazapakq-5.h"
#include "mmc-twl4030.h"
#include "twl4030-script.h"
#include "pm.h"
#include "prm-regbits-34xx.h"
#include "omap3-opp.h"
#include <plat/sdrc.h>
#include <plat/dm3730logic-productid.h>
#include <plat/board-dm3730logic.h>
#include <plat/board-dm3730logic-audio.h>
#include "board-dm3730logic.h"
#ifdef CONFIG_PRINTK_DEBUG
#include <plat/printk_debug.h>
#include <plat/sram.h>
#endif

#define DIE_ID_REG_BASE      (L4_WK_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET    0x218
#define MAX_USB_SERIAL_NUM   17

#define DM3730LOGIC_TS_GPIO	175

#define DM3730LOGIC_ETHR_START	0x2c000000
#define DM3730LOGIC_ETHR_SIZE	1024
#define DM3730LOGIC_ETHR_ID_REV	0x50

#define DM3730LOGIC_SOM_LV_ETHR_GPIO_IRQ	152
#define DM3730LOGIC_TORPEDO_ETHR_GPIO_IRQ	129
#define DM3730LOGIC_SMSC911X_CS	1

#ifdef CONFIG_TIWLAN_SDIO
static void dm3730logic_init_wilink_mux(void)
{
	struct clk *sys_clkout1_clk;

	printk(KERN_INFO "%s: setup wilink mux signals\n", __FUNCTION__);

	if (gpio_request(OMAP_DM3730LOGIC_WIFI_PMENA_GPIO, "wifi_en") != 0)
		pr_err("GPIO %i request for wilink_en failed\n", OMAP_DM3730LOGIC_WIFI_PMENA_GPIO);
	gpio_direction_output(OMAP_DM3730LOGIC_WIFI_PMENA_GPIO, 0);
	omap_mux_init_gpio(OMAP_DM3730LOGIC_WIFI_PMENA_GPIO, OMAP_PIN_OUTPUT);
	gpio_export(OMAP_DM3730LOGIC_WIFI_PMENA_GPIO, 0);
#ifdef CONFIG_WLAN_127x_BT
	if (gpio_request(OMAP_DM3730LOGIC_BT_EN_GPIO, "bt_en") != 0)
		pr_err("GPIO %i request for bt_en failed\n", OMAP_DM3730LOGIC_BT_EN_GPIO);
	gpio_direction_output(OMAP_DM3730LOGIC_BT_EN_GPIO, 0);
	omap_mux_init_gpio(OMAP_DM3730LOGIC_BT_EN_GPIO, OMAP_PIN_OUTPUT);
	gpio_export(OMAP_DM3730LOGIC_BT_EN_GPIO, 0);
#endif

	/* Pull the enables out of reset */
	msleep(10);
	gpio_set_value(OMAP_DM3730LOGIC_WIFI_PMENA_GPIO, 1);
#ifdef CONFIG_WLAN_127x_BT
	gpio_set_value(OMAP_DM3730LOGIC_BT_EN_GPIO, 1);
#endif
	msleep(10);

	/* Put them back into reset (so they go into low-power mode) */
	gpio_set_value(OMAP_DM3730LOGIC_WIFI_PMENA_GPIO, 0);
#ifdef CONFIG_WLAN_127x_BT
	gpio_set_value(OMAP_DM3730LOGIC_BT_EN_GPIO, 0);
#endif

	// Setup the mux for mmc3
	omap_mux_init_signal("mcspi1_cs1.sdmmc3_cmd", OMAP_PIN_INPUT_PULLUP); /* McSPI1_CS1/ADPLLV2D_DITHERING_EN2/MMC3_CMD/GPIO_175 */
	omap_mux_init_signal("mcspi1_cs2.sdmmc3_clk", OMAP_PIN_INPUT_PULLUP); /* McSPI1_CS2/MMC3_CLK/GPIO_176 */
	omap_mux_init_signal("sdmmc2_dat4.sdmmc3_dat0", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT4/MMC2_DIR_DAT0/MMC3_DAT0/GPIO_136 */
	omap_mux_init_signal("sdmmc2_dat5.sdmmc3_dat1", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT5/MMC2_DIR_DAT1/CAM_GLOBAL_RESET/MMC3_DAT1/HSUSB3_TLL_STP/MM3_RXDP/GPIO_137 */
	omap_mux_init_signal("sdmmc2_dat6.sdmmc3_dat2", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT6/MMC2_DIR_CMD/CAM_SHUTTER/MMC3_DAT2/HSUSB3_TLL_DIR/GPIO_138 */
	omap_mux_init_signal("sdmmc2_dat7.sdmmc3_dat3", OMAP_PIN_INPUT_PULLUP); /* MMC2_DAT7/MMC2_CLKIN/MMC3_DAT3/HSUSB3_TLL_NXT/MM3_RXDM/GPIO_139 */

	omap_mux_init_signal("sys_boot0.gpio_2", OMAP_PIN_INPUT);


#ifdef CONFIG_WLAN_127x_BT
	omap_mux_init_gpio(OMAP_DM3730LOGIC_BT_WAKEUP_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(OMAP_DM3730LOGIC_BT_HOST_WAKE_GPIO, OMAP_PIN_INPUT);

/* NOTE: SOM-LV is configured to use BRF6300 for Bluetooth instead of wl127x.
	If both WiFi and BT are to be enabled simultaneously, design should 
	consider using BT on wl127x instead it can handle BT/WLAN coexistance.
*/
	omap_mux_init_signal("uart2_cts.uart2_cts", OMAP_PIN_INPUT_PULLUP); /* UART2_CTS/MCBSP3_DX/GPT9_PWM_EVT/GPIO_144 */
	omap_mux_init_signal("uart2_rts.uart2_rts", OMAP_PIN_OUTPUT); /* UART2_RTS/MCBSP3_DR/GPT10_PWM_EVT/GPIO_145 */
	omap_mux_init_signal("uart2_tx.uart2_tx", OMAP_PIN_OUTPUT); /* UART2_TX/MCBSP3_CLKX/GPT11_PWM_EVT/GPIO_146 */
	omap_mux_init_signal("uart2_rx.uart2_rx", OMAP_PIN_INPUT); /* UART2_RX/MCBSP3_FSX/GPT8_PWM_EVT/GPIO_147 */
#endif

	/* Enable sys_clkout1 (uP_CLKOUT1_26Mhz) */
	sys_clkout1_clk = clk_get(NULL, "sys_clkout1");
	if (IS_ERR(sys_clkout1_clk)) {
		printk("%s: Can't get sys_clkout1\n", __FUNCTION__);
	} else {
		clk_enable(sys_clkout1_clk);
	}

	omap_mux_init_signal("mcbsp3_fsx.mcbsp3_fsx", OMAP_PIN_OUTPUT); /* McBSP3_FSX/UART2_RX/HSUSB3_TLL_DATA7/GPIO_143 */
	omap_mux_init_signal("mcbsp3_clkx.mcbsp3_clkx", OMAP_PIN_OUTPUT); /* McBSP3_CLKX/UART2_TX/HSUSB3_TLL_DATA6/GPIO_142 */
	omap_mux_init_signal("mcbsp3_dr.mcbsp3_dr", OMAP_PIN_OUTPUT); /* McBSP3_DR/UART2_RTS/HSUSB3_TLL_DATA5/GPIO_141 */
	omap_mux_init_signal("mcbsp3_dx.mcbsp3_dx", OMAP_PIN_OUTPUT); /* McBSP3_DX/UART21_CTS/HSUSB3_TLL_DATA4/GPIO_140 */

}
#else
static void dm3730logic_init_wilink_mux(void)
{
}
#endif

static void dm3730logic_init_wifi_mux(void)
{
	if (dm3730logic_has_wilink_wifi_module() && machine_is_dm3730_som_lv()) {
		dm3730logic_init_wilink_mux();
	}
}

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
/* Fix the PBIAS voltage for Torpedo MMC1 pins that
 * are used for other needs (IRQs, etc). */
static void dm3730torpedo_fix_pbias_voltage(void)
{
	u16 control_pbias_offset = OMAP343X_CONTROL_PBIAS_LITE;
	static int pbias_fixed = 0;
	u32 reg;

	if (!pbias_fixed) {
		/* Set the bias for the pin */
		reg = omap_ctrl_readl(control_pbias_offset);

		reg &= ~OMAP343X_PBIASLITEPWRDNZ1;
		omap_ctrl_writel(reg, control_pbias_offset);

		/* 100ms delay required for PBIAS configuration */
		msleep(100);

		reg |= OMAP343X_PBIASLITEVMODE1;
		reg |= OMAP343X_PBIASLITEPWRDNZ1;
		omap_ctrl_writel(reg | 0x300, control_pbias_offset);


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
#endif

/* First GPIO on twl4030 */
int dm3730logic_twl4030_gpio_base;


#if defined(CONFIG_NEW_LEDS) || defined(CONFIG_NEW_LEDS_MODULE)
static struct gpio_led dm3730logic_leds[] = {
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
 
static struct gpio_led_platform_data dm3730logic_led_data = {
	.leds		= dm3730logic_leds,
	.num_leds	= 0,	/* Initialized in dm3730logic_led_init() */
 };
 
static struct platform_device dm3730logic_led_device = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &dm3730logic_led_data,
	},
};

#define GPIO_LED1_SOM_LV	133
#define GPIO_LED2_SOM_LV	11

#define GPIO_LED1_TORPEDO	180
#define GPIO_LED2_TORPEDO	179

static void dm3730logic_led_init(void)
{
	int gpio_led1 = -EINVAL;
	int gpio_led2 = -EINVAL;

	if (machine_is_dm3730_torpedo()) {
		if (!dm3730logic_twl4030_gpio_base) {
			printk(KERN_ERR "Huh?!? twl4030_gpio_base not set!\n");
			return;
		}
		/* baseboard LEDs are MCSPIO2_SOMI, MCSPOI2_SIMO */
		gpio_led1 = GPIO_LED1_TORPEDO;
		gpio_led2 = GPIO_LED2_TORPEDO;

		/* twl4030 ledA is the LED on the module */
		dm3730logic_leds[2].gpio = dm3730logic_twl4030_gpio_base + TWL4030_GPIO_MAX + 0;
		dm3730logic_led_data.num_leds = 3;
	} else if (machine_is_dm3730_som_lv()) {
		gpio_led1 = GPIO_LED1_SOM_LV;
		dm3730logic_leds[0].active_low = true;
		gpio_led2 = GPIO_LED2_SOM_LV;
		dm3730logic_leds[1].active_low = true;

		/* SOM has only two LEDs */
		dm3730logic_led_data.num_leds = 2;
	}

	if (gpio_led1 < dm3730logic_twl4030_gpio_base)
		omap_mux_init_gpio(gpio_led1, OMAP_PIN_OUTPUT);
	dm3730logic_leds[0].gpio = gpio_led1;

	if (gpio_led2 < dm3730logic_twl4030_gpio_base)
		omap_mux_init_gpio(gpio_led2, OMAP_PIN_OUTPUT);
	dm3730logic_leds[1].gpio = gpio_led2;

	if (platform_device_register(&dm3730logic_led_device) < 0)
		printk(KERN_ERR "Unable to register LED device\n");
}
#else
static void dm3730logic_led_init(void)
{
}
#endif

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct resource dm3730logic_smsc911x_resources[] = {
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
	.flags          = (SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS),
};

static struct platform_device dm3730logic_smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(dm3730logic_smsc911x_resources),
	.resource	= &dm3730logic_smsc911x_resources[0],
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static inline void __init dm3730logic_init_smsc911x(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	int eth_gpio = 0;

	eth_cs = DM3730LOGIC_SMSC911X_CS;

	if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for smsc911x\n");
		return;
	}

	if (machine_is_dm3730_som_lv())
		eth_gpio = DM3730LOGIC_SOM_LV_ETHR_GPIO_IRQ;
	else {
		eth_gpio = DM3730LOGIC_TORPEDO_ETHR_GPIO_IRQ;

		/* On Torpedo, LAN9221 IRQ is an MMC1_DATA7 pin
		 * and IRQ1760 IRQ is MMC1_DATA4 pin - need
		 * to update PBIAS to get voltage to the device
		 * so the IRQs works correctly rather than float
		 * and cause an IRQ storm... */
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
		dm3730torpedo_fix_pbias_voltage();
#endif
	}

	if (gpio_request(eth_gpio, "SMSC911x irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smsc911x IRQ\n",
			eth_gpio);
		return;
	}

	omap_mux_init_gpio(eth_gpio, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
	gpio_direction_input(eth_gpio);

	dm3730logic_smsc911x_resources[1].start = OMAP_GPIO_IRQ(eth_gpio);
	dm3730logic_smsc911x_resources[1].end = OMAP_GPIO_IRQ(eth_gpio);

	dm3730logic_smsc911x_resources[0].start = cs_mem_base + 0x0;
	dm3730logic_smsc911x_resources[0].end = cs_mem_base + DM3730LOGIC_ETHR_SIZE - 1;

	platform_device_register(&dm3730logic_smsc911x_device);
}

#else
static inline void __init dm3730logic_init_smsc911x(void) { return; }
#endif

static struct regulator_consumer_supply dm3730logic_vaux1_supply = {
	.supply			= "vaux1",
};

/* VAUX1 for touch/product ID chip */
static struct regulator_init_data dm3730logic_vaux1 = {
	.constraints = {
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.apply_uV	= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
#if 1
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
#endif
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &dm3730logic_vaux1_supply,
};

static struct regulator_consumer_supply dm3730logic_vaux3_supply = {
	.supply			= "vaux3",
};

/* VAUX3 required to enable WiLink 26MHz clock */
static struct regulator_init_data dm3730logic_vaux3 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &dm3730logic_vaux3_supply,
};

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{}, /* Terminator (or MMC3 info for SOM LV) */
	{}	/* Terminator */
};

static struct twl4030_hsmmc_info mmc3 = {
	.mmc		= 3,
	.wires		= 4,
	.gpio_cd	= -EINVAL,
	.gpio_wp	= -EINVAL,
};


static struct regulator_consumer_supply dm3730logic_vmmc1_supply = {
	.supply			= "vmmc",
};

#ifdef CONFIG_PM
/*
 * Save the state of keypad
 *
 * TODO: This definition should ideally be in a header file, but
 *       matrix_keypad.h is not the right one. Also, plat/keypad.h
 *       is no longer used.
 */
struct omap_keypad_pm_state {
	void __iomem *wk_st;
	void __iomem *wk_en;
	u32 wk_mask;
	u32 padconf;
};

/*
 * Board specific hook for keypad suspend
 */
void dm3730logic_kp_suspend(void *ptr)
{
	struct omap_keypad_pm_state *pstate =
			(struct omap_keypad_pm_state *)ptr;

	if (pstate) {
		/*
		 * Set wake-enable bit
		 */
		if (pstate->wk_en && pstate->wk_mask) {
			u32 v = __raw_readl(pstate->wk_en);
			v |= pstate->wk_mask;
			__raw_writel(v, pstate->wk_en);
		}
		/*
		 * Set corresponding IOPAD wakeup-enable
		 */
		if (cpu_is_omap34xx() && pstate->padconf) {
			u16 v = omap_ctrl_readw(pstate->padconf);
			v |= OMAP3_PADCONF_WAKEUPENABLE0;
			omap_ctrl_writew(v, pstate->padconf);
		}
	}
}

/*
 * Board specific hook for keypad resume
 */
void dm3730logic_kp_resume(void *ptr)
{
	struct omap_keypad_pm_state *pstate =
			(struct omap_keypad_pm_state *)ptr;

	if (pstate) {
		/*
		 * Clear wake-enable bit
		 */
		if (pstate->wk_en && pstate->wk_mask) {
			u32 v = __raw_readl(pstate->wk_en);
			v &= ~pstate->wk_mask;
			__raw_writel(v, pstate->wk_en);
		}
		/*
		 * Clear corresponding IOPAD wakeup-enable
		 */
		if (cpu_is_omap34xx() && pstate->padconf) {
			u16 v = omap_ctrl_readw(pstate->padconf);
			v &= ~OMAP3_PADCONF_WAKEUPENABLE0;
			omap_ctrl_writew(v, pstate->padconf);
		}
	}
}

static struct omap_opp * _omap37x_mpu_rate_table	= omap37x_mpu_rate_table;
static struct omap_opp * _omap37x_dsp_rate_table	= omap37x_dsp_rate_table;
static struct omap_opp * _omap37x_l3_rate_table		= omap37x_l3_rate_table;
#else	/* CONFIG_PM */
static struct omap_opp * _omap37x_mpu_rate_table	= NULL;
static struct omap_opp * _omap37x_dsp_rate_table	= NULL;
static struct omap_opp * _omap37x_l3_rate_table		= NULL;
#endif	/* CONFIG_PM */

static struct gpio_keys_button gpio_buttons[] = {
	{
		.code = KEY_HOME,
		.gpio = -EINVAL,
		.desc = "home",
		.active_low = 1,
		.debounce_interval = 30,
		.wakeup = 0,
	},
	{
		.code = KEY_MENU,
		.gpio = -EINVAL,
		.desc = "menu",
		.active_low = 1,
		.debounce_interval = 30,
		.wakeup = 0,
	},
	{
		.code = KEY_BACK,
		.gpio = -EINVAL,
		.desc = "back",
		.active_low = 1,
		.debounce_interval = 30,
		.wakeup = 0,
	},
	{
		.code = KEY_SEARCH,
		.gpio = -EINVAL,
		.desc = "search",
		.active_low = 1,
		.debounce_interval = 30,
		.wakeup = 0,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons = gpio_buttons,
	.nbuttons = ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name = "gpio-keys",
	.id   = -1,
	.dev  = {
		.platform_data = &gpio_key_info,
	},
};

static int dm3730logic_is_ac_online(void)
{
	return 0;
}

static int dm3730logic_is_usb_online(void)
{
	return 0;
}

static char *dm3730logic_supplicants[] = {
	"bq27000"
};

static struct pda_power_pdata power_supply_info = {
	.is_ac_online     = dm3730logic_is_ac_online,
	.is_usb_online    = dm3730logic_is_usb_online,
	.supplied_to      = dm3730logic_supplicants,
	.num_supplicants  = ARRAY_SIZE(dm3730logic_supplicants),
};

static struct platform_device power_supply = {
	.name             = "pda-power",
	.id               = -1,
	.dev = {
		.platform_data = &power_supply_info,
	},
};


static struct platform_device *dm3730logic_devices[] __initdata = {
	&keys_gpio,
	&power_supply,
};

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
	printk("%s: direction %d value %d\n", __FUNCTION__, direction, value);
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
#endif //CONFIG_BT_HCIBRF6300_SPI

static int dm3730logic_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	int i;
	int dm3730_key_menu = -EINVAL;
	int dm3730_key_home = -EINVAL;
	int dm3730_key_back = -EINVAL;
	int dm3730_key_search = -EINVAL;

	dm3730logic_twl4030_gpio_base = gpio;

	if (machine_is_dm3730_torpedo()) {
		/* gpio_127 is mmc0_cd on Torpedo */
		omap_mux_init_gpio(127, OMAP_PIN_INPUT_PULLUP);
		mmc[0].gpio_cd = 127;

		dm3730_key_home = 181;
		dm3730_key_menu = 7;
		dm3730_key_back = 2;
		dm3730_key_search = 178;

	} else if (machine_is_dm3730_som_lv()) {
		mmc[0].gpio_cd = 110;
		omap_mux_init_gpio(110, OMAP_PIN_INPUT_PULLUP);
		/* Since GPIO126 is routed on the OMAP35x to
		 * both sdmmc1_dat4 and cam_strobe we have to
		 * mux cam_strome/gpio_126 as a GPIO by hand */
		omap_mux_init_signal("cam_strobe.gpio_126", OMAP_PIN_INPUT_PULLUP);
		mmc[0].gpio_wp = 126;

		/* For the SOM LV, add in the MMC info to
		 * the MMC list (the 3rd slot is the terminator). */
		mmc[1] = mmc3;

		dm3730_key_home = gpio + 7;
		dm3730_key_menu = gpio + 15;
		dm3730_key_back = 111;
		dm3730_key_search = gpio + 2;

	}

	for (i = 0; i < ARRAY_SIZE(gpio_buttons); i++) {
		if( KEY_MENU == gpio_buttons[i].code )
			gpio_buttons[i].gpio = dm3730_key_menu;
		if( KEY_HOME == gpio_buttons[i].code )
			gpio_buttons[i].gpio = dm3730_key_home;
		if( KEY_BACK == gpio_buttons[i].code )
			gpio_buttons[i].gpio = dm3730_key_back;
		if( KEY_SEARCH == gpio_buttons[i].code )
			gpio_buttons[i].gpio = dm3730_key_search;
		printk( KERN_INFO "gpio_buttons[%d] (%s) gpio=%d\n",i, gpio_buttons[i].desc, gpio_buttons[i].gpio );
	}

#ifdef CONFIG_BT_HCIBRF6300_SPI
	/* TWL4030 GPIO for BT_nSHUTDOWN */
	brf6300_bt_nshutdown_gpio = gpio + TWL4030_BT_nSHUTDOWN;
#endif

	twl4030_mmc_init(mmc);

	/* link regulator to MMC adapter */
	dm3730logic_vmmc1_supply.dev = mmc[0].dev;

	dm3730logic_lcd_init();

	dm3730logic_led_init();

	return 0;
}

static struct twl4030_gpio_platform_data dm3730logic_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.pullups	= BIT(2) | BIT(7) | BIT(15),
	.use_leds	= true,
	.setup		= dm3730logic_twl_gpio_setup,
};

#define STANDARD_OMAP	0
#define TEST_LOGIC	1

//#define TEST_GROUP	DEV_GRP_P1
#define TEST_GROUP	DEV_GRP_NULL

static struct twl4030_ins  sleep_on_seq[] = {
#if STANDARD_OMAP
	/* Broadcast message to put res to sleep (TYPE2 = 1, 2) */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_SLEEP), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_SLEEP), 2},
#else
	{MSG_SINGULAR(DEV_GRP_P1, RES_HFCLKOUT, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD1,     RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD2,     RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VPLL1,    RES_STATE_OFF), 2},
#if TEST_LOGIC
	{MSG_BROADCAST(TEST_GROUP, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_OFF), 2},
#endif
#endif
};

static struct twl4030_script sleep_on_script = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

static struct twl4030_ins wakeup_p12_seq[] = {
#if STANDARD_OMAP
	/*
	 * Broadcast message to put resources to active
	 *
	 * Since we are not using TYPE, resources which have TYPE2 configured
	 * as 1 will be targeted (VPLL1, VDD1, VDD2, REGEN, NRES_PWRON, SYSEN).
	 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_ACTIVE), 2},
#else
	{MSG_SINGULAR(DEV_GRP_P1, RES_HFCLKOUT, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD1,     RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VDD2,     RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_P1, RES_VPLL1,    RES_STATE_ACTIVE), 2},
#if TEST_LOGIC
	{MSG_BROADCAST(TEST_GROUP, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_ACTIVE), 2},
#endif
#endif
};

static struct twl4030_script wakeup_p12_script = {
	.script	= wakeup_p12_seq,
	.size	= ARRAY_SIZE(wakeup_p12_seq),
	.flags	= TWL4030_WAKEUP12_SCRIPT,
};

static struct twl4030_ins wakeup_p3_seq[] = {
#if STANDARD_OMAP
	/*
	 * Broadcast message to put resources to active
	 *
	 * Since we are not using TYPE, resources which have TYPE2 configured
	 * as 2 will be targeted
	 * (VINTANA1, VINTANA2, VINTDIG, VIO, CLKEN, HFCLKOUT).
	 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_ACTIVE), 2},
#else
	{MSG_SINGULAR(DEV_GRP_P1, RES_HFCLKOUT, RES_STATE_ACTIVE), 2},
#endif
};

static struct twl4030_script wakeup_p3_script = {
	.script = wakeup_p3_seq,
	.size   = ARRAY_SIZE(wakeup_p3_seq),
	.flags  = TWL4030_WAKEUP3_SCRIPT,
};

static struct twl4030_ins wrst_seq[] = {
#if STANDARD_OMAP
	/*
	 * As a workaround for OMAP Erratum  (ID: i537 - OMAP HS devices are
	 * not recovering from warm reset while in OFF mode)
	 * NRESPWRON is toggled to force a power on reset condition to OMAP
	 */
	/* Trun OFF NRES_PWRON */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_NRES_PWRON, RES_STATE_OFF), 2},
	/* Reset twl4030 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_OFF), 2},
	/* Reset MAIN_REF */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_MAIN_REF, RES_STATE_WRST), 2},
	/* Reset All type2_group2 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_WRST), 2},
	/* Reset VUSB_3v1 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_VUSB_3V1, RES_STATE_WRST), 2},
	/* Reset All type2_group1 */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_WRST), 2},
	/* Reset the Reset & Contorl_signals */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_WRST), 2},
	/* Re-enable twl4030 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_ACTIVE), 2},
	/* Trun ON NRES_PWRON */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_NRES_PWRON, RES_STATE_ACTIVE), 2},
#else
	/*
	 * Reset twl4030.
	 * Reset VDD1 regulator.
	 * Reset VDD2 regulator.
	 * Reset VPLL1 regulator.
	 * Enable sysclk output.
	 * Reenable twl4030.
	 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET,    RES_STATE_OFF),    2},
	{MSG_SINGULAR(DEV_GRP_P1,   RES_VDD1,     RES_STATE_WRST),   15},
	{MSG_SINGULAR(DEV_GRP_P1,   RES_VDD2,     RES_STATE_WRST),   15},
	{MSG_SINGULAR(DEV_GRP_P1,   RES_VPLL1,    RES_STATE_WRST),   0x60},
	{MSG_SINGULAR(DEV_GRP_P1,   RES_HFCLKOUT, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET,    RES_STATE_ACTIVE), 2},
#endif
};
static struct twl4030_script wrst_script = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] = {
	&wakeup_p12_script,
	&wakeup_p3_script,
	&sleep_on_script,
	&wrst_script,
};

static struct twl4030_resconfig twl4030_rconfig[] = {
#if STANDARD_OMAP
	{
		.resource = RES_NRES_PWRON,
		.devgroup = DEV_GRP_ALL,
		.type = 0,
		.type2 = 1,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_VINTANA2,
		.devgroup = DEV_GRP_ALL,
		.type = 0,
		.type2 = 2,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_HFCLKOUT,
		.devgroup = DEV_GRP_P3,
		.type = 0,
		.type2 = 2,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_VINTANA1,
		.devgroup = DEV_GRP_ALL,
		.type = 1,
		.type2 = 2,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_VINTDIG,
		.devgroup = DEV_GRP_ALL,
		.type = 1,
		.type2 = 2,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_REGEN,
		.devgroup = DEV_GRP_ALL,
		.type = 2,
		.type2 = 1,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_VIO,
		.devgroup = DEV_GRP_ALL,
		.type = 2,
		.type2 = 2,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_VPLL1,
		.devgroup = DEV_GRP_P1,
		.type = 3,
		.type2 = 1,
		.remap_sleep = RES_STATE_OFF
	},
	{
		.resource = RES_VDD2,
		.devgroup = DEV_GRP_P1,
		.type = 3,
		.type2 = 1,
		.remap_sleep = RES_STATE_OFF
	},
	{
		.resource = RES_CLKEN,
		.devgroup = DEV_GRP_ALL,
		.type = 3,
		.type2 = 2,
		.remap_sleep = RES_STATE_SLEEP
	},
	{
		.resource = RES_VDD1,
		.devgroup = DEV_GRP_P1,
		.type = 4,
		.type2 = 1,
		.remap_sleep = RES_STATE_OFF
	},
	{
		.resource = RES_SYSEN,
		.devgroup = DEV_GRP_ALL,
		.type = 6,
		.type2 = 1,
		.remap_sleep = RES_STATE_SLEEP
	},
#else
	{
		.resource = RES_HFCLKOUT,
		.devgroup = DEV_GRP_P3,
		.type = -1,
		.type2 = -1
	},
	{
		.resource = RES_VDD1,
		.devgroup = DEV_GRP_P1,
		.type = -1,
		.type2 = -1
	},
	{
		.resource = RES_VDD2,
		.devgroup = DEV_GRP_P1,
		.type = -1,
		.type2 = -1
	},
#if TEST_LOGIC
	{
		.resource = RES_VAUX1,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VAUX2,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VAUX3,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VAUX4,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VMMC1,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VMMC2,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VSIM,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VDAC,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
#if 0
	// Disabling these seems to to hose up the warm reset.  The system will
	// still come up from a cold start.
	{
		.resource = RES_VINTANA1,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VINTANA2,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
#endif
	{
		.resource = RES_VUSB_1V5,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VUSB_1V8,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
	{
		.resource = RES_VUSB_3V1,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
#if 1
	// No effect on power consumption when the system is in suspend.
	{
		.resource = RES_VUSBCP,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
#endif
	{
		.resource = RES_SYSEN,
		.devgroup = TEST_GROUP,
		.type = -1,
		.type2 = RES_TYPE2_R2,
	},
#endif
#endif
	{ 0, 0},
};

static struct twl4030_power_data dm3730logic_t2scripts_data = {
	.scripts	= twl4030_scripts,
	.num		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data dm3730logic_vmmc1 = {
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
	.consumer_supplies	= &dm3730logic_vmmc1_supply,
};

static struct twl4030_usb_data dm3730logic_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_madc_platform_data dm3730logic_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_codec_audio_data dm3730logic_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data dm3730logic_codec_data = {
	.audio_mclk = 26000000,
	.audio = &dm3730logic_audio_data,
};

#if defined(CONFIG_OMAP2_DSS) || defined(CONFIG_OMAP2_DSS_MODULE)
static struct regulator_consumer_supply dm3730logic_vdda_dac_supply = {
	.supply		= "vdda_dac",
	.dev		= &dm3730logic_dss_device.dev,
};

/* VDAC for DSS driving S-Video */
static struct regulator_init_data dm3730logic_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &dm3730logic_vdda_dac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_consumer_supply dm3730logic_vpll2_supplies[] = {
	{
		.supply	= "vdvi",
		.dev	= &dm3730logic_lcd_device.dev,
	},
	{
		.supply	= "vdds_dsi",
		.dev	= &dm3730logic_dss_device.dev,
	},
};

static struct regulator_init_data dm3730logic_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(dm3730logic_vpll2_supplies),
	.consumer_supplies	= dm3730logic_vpll2_supplies,
};
#endif

static struct twl4030_platform_data dm3730logic_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &dm3730logic_madc_data,
	.usb		= &dm3730logic_usb_data,
	.gpio		= &dm3730logic_gpio_data,
	.codec		= &dm3730logic_codec_data,
	.power		= &dm3730logic_t2scripts_data,

	.vmmc1		= &dm3730logic_vmmc1,
	.vaux1		= &dm3730logic_vaux1,
	.vaux3		= &dm3730logic_vaux3,

#if defined(CONFIG_OMAP2_DSS) || defined(CONFIG_OMAP2_DSS_MODULE)
	.vdac		= &dm3730logic_vdac,
	.vpll2		= &dm3730logic_vpll2,
#endif

	/* Disable pullups internal to the PMIC; recommended by TI since
	   DM3730 Logic boards have 470 ohm pullups on I2C1; if in parallel
	   with PMIC 3K ohm pullups then pullup is too hard */
	.disable_pmic_pullups = 1,
};

static struct i2c_board_info __initdata dm3730logic_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &dm3730logic_twldata,
	},
};

#ifdef CONFIG_TOUCHSCREEN_TSC2004

#define	GPIO_TSC2004_IRQ	153

static int tsc2004_pre_init(struct tsc2004_platform_data *pdata)
{
	int err;

	pdata->regulator_name = "vaux1";
	pdata->regulator = regulator_get(NULL, "vaux1");
	if (IS_ERR(pdata->regulator)) {
		pr_err("%s: unable to get %s regulator\n", __FUNCTION__, pdata->regulator_name);
		return -1;
	}

	err = regulator_enable(pdata->regulator);
	if (err) {
		pr_err("%s: unable to enable %s regulator\n", __FUNCTION__, pdata->regulator_name);
		regulator_put(pdata->regulator);
		pdata->regulator = NULL;
		return err;
	}
	return 0;
}

static int tsc2004_init_irq(void)
{
	int ret = 0;

	omap_mux_init_gpio(GPIO_TSC2004_IRQ, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
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

static void tsc2004_post_exit(struct tsc2004_platform_data *pdata)
{
	if (pdata->regulator && regulator_is_enabled(pdata->regulator)) {
		regulator_disable(pdata->regulator);
	}
}

static int tsc2004_get_irq_level(void)
{
	return gpio_get_value(GPIO_TSC2004_IRQ) ? 0 : 1;
}

struct tsc2004_platform_data dm3730logic_tsc2004data = {
	.model = 2004,
	.x_plate_ohms = 180,
	.get_pendown_state = tsc2004_get_irq_level,
	.pre_init_platform_hw = tsc2004_pre_init,
	.init_platform_hw = tsc2004_init_irq,
	.exit_platform_hw = tsc2004_exit_irq,
	.post_exit_platform_hw = tsc2004_post_exit,
	.regulator_name = "vaux1",
};

#endif

static struct i2c_board_info __initdata dm3730logic_i2c3_boardinfo[] = {
#ifdef CONFIG_TOUCHSCREEN_TSC2004
	{
		I2C_BOARD_INFO("tsc2004", 0x48),
		.type		= "tsc2004",
		.platform_data = &dm3730logic_tsc2004data,
		.irq = OMAP_GPIO_IRQ(GPIO_TSC2004_IRQ),
	},
#endif
};

static int __init dm3730logic_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, dm3730logic_i2c_boardinfo,
			ARRAY_SIZE(dm3730logic_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, dm3730logic_i2c3_boardinfo,
			ARRAY_SIZE(dm3730logic_i2c3_boardinfo));
	return 0;
}


struct spi_board_info dm3730logic_spi_board_info[] = {
};

static struct omap_board_config_kernel dm3730logic_config[] __initdata = {
};

static void __init dm3730logic_init_irq(void)
{
	omap_board_config = dm3730logic_config;
	omap_board_config_size = ARRAY_SIZE(dm3730logic_config);
	omap2_init_common_hw(mt29c4g48mazapakq5_sdrc_params,
			NULL,
			_omap37x_mpu_rate_table,
			_omap37x_dsp_rate_table,
			_omap37x_l3_rate_table);

	omap_init_irq();
	omap_gpio_init();
}


#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux omap36x_board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define omap36x_board_mux	NULL
#endif

#if defined(CONFIG_USB_ISP1763) || defined(CONFIG_USB_ISP1763_MODULE) || defined(CONFIG_USB_ISP1763_HCD_OMAP_TORPEDO)

/* ISP1763 USB interrupt */
#define OMAP3TORPEDO_ISP1763_IRQ_GPIO		128

static struct isp1763_platform_data dm3730logic_isp1763_pdata = {
	.bus_width_8		= 0,
	.port1_otg		= 0,
	.dack_polarity_high	= 0,
	.dreq_polarity_high	= 0,
	.intr_polarity_high	= 0,
	.intr_edge_trigger	= 0,
};

static struct resource dm3730logic_isp1763_resources[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = -EINVAL,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device dm3730logic_isp1763 = {
	.name		= "isp1763",
	.id		= -1,
	.dev		= {
		.platform_data	= &dm3730logic_isp1763_pdata,
	},
	.num_resources = ARRAY_SIZE(dm3730logic_isp1763_resources),
	.resource = dm3730logic_isp1763_resources,
};


static void dm3730logic_init_isp1763(void)
{
	unsigned long cs_mem_base;
	unsigned int irq_gpio;

	/* ISP1763 IRQ is an MMC1 data pin - need to update PBIAS
	 * to get voltage to the device so the IRQ works correctly rather
	 * than float below logic 1 and cause IRQ storm... */
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
	if (machine_is_dm3730_torpedo())
		dm3730torpedo_fix_pbias_voltage();
	else
		BUG();
#endif

	if (gpmc_cs_request(6, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for ISP1763\n");
		return;
	}
	
	dm3730logic_isp1763_resources[0].start = cs_mem_base;
	dm3730logic_isp1763_resources[0].end = cs_mem_base + 0xffff;

	irq_gpio = OMAP3TORPEDO_ISP1763_IRQ_GPIO;

	omap_mux_init_gpio(irq_gpio, OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE);
	/* Setup ISP1763 IRQ pin as input */
	if (gpio_request(irq_gpio, "isp1763_irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for isp1763 IRQ\n",
		irq_gpio);
		return;
	}
	gpio_direction_input(irq_gpio);
	dm3730logic_isp1763_resources[1].start = OMAP_GPIO_IRQ(irq_gpio);

	if (platform_device_register(&dm3730logic_isp1763) < 0) {
		printk(KERN_ERR "Unable to register isp1763 device\n");
		gpio_free(irq_gpio);
		return;
	} else {
		pr_info("registered isp1763 platform_device\n");
	}
}
#else
static void dm3730logic_init_isp1763(void)
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

static void dm3730logic_init_ehci(void)
{
	omap_mux_init_gpio(ehci_pdata.reset_gpio_port[1], OMAP_PIN_OUTPUT);
	usb_ehci_init(&ehci_pdata);
}

static void dm3730logic_usb_init(void)
{
	if (machine_is_dm3730_som_lv())
		dm3730logic_init_ehci();
	else
		dm3730logic_init_isp1763();
}

static void dm3730logic_musb_init(void)
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

static void dm3730logic_gpio_init(void)
{
	// Home, Menu, Back and Search buttons

	if (machine_is_dm3730_torpedo()) {
		omap_mux_init_signal("sys_boot0.gpio_2", OMAP_PIN_INPUT_PULLUP);
		omap_mux_init_signal("sys_boot5.gpio_7", OMAP_PIN_INPUT_PULLUP);
		omap_mux_init_signal("mcspi2_cs0.gpio_181", OMAP_PIN_INPUT_PULLUP);
		omap_mux_init_signal("mcspi2_clk.gpio_178", OMAP_PIN_INPUT_PULLUP);
	} else {
		omap_mux_init_signal("cam_xclkb.gpio_111", OMAP_PIN_INPUT_PULLUP); /* KEY_BACK */
	}

}

static void enable_board_wakeup_source(void)
{
	/* T2 interrupt line (keypad) */
	omap_mux_init_signal("sys_nirq",
		OMAP_WAKEUP_EN | OMAP_PIN_INPUT_PULLUP);
}

#ifdef CONFIG_BT_HCIBRF6300_SPI
static struct omap2_mcspi_device_config brf6300_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,  /* 0: slave, 1: master */
};


/* Intialized in omap3logic_spi_init() */
struct brf6300_platform_data brf6300_config;

static struct spi_board_info dm3730logic_spi_brf6300 =
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

static void brf6300_dev_init(void)
{
	/* Only the LV SOM has a BRF6300 */
	if (!machine_is_dm3730_som_lv())
		return;

	if (!dm3730logic_twl4030_gpio_base) {
		printk(KERN_ERR "Huh?!? dm3730logic_twl4030_gpio_base not set!\n");
		return;
	}

	omap_mux_init_gpio(BT_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP); /* GPIO_157 */
	if (gpio_request(BT_IRQ_GPIO, "BRF6300 IRQ") < 0)
		printk(KERN_ERR "can't get BRF6300 irq GPIO\n");

	gpio_direction_input(BT_IRQ_GPIO);

	brf6300_config.irq_gpio = BT_IRQ_GPIO;
	brf6300_config.shutdown_gpio = dm3730logic_twl4030_gpio_base + TWL4030_BT_nSHUTDOWN;
}
#else
static void brf6300_dev_init(void)
{
}
#endif //CONFIG_BT_HCIBRF6300_SPI

#if defined(CONFIG_DM3730LOGIC_SPI1_CS0) \
        || defined(CONFIG_DM3730LOGIC_SPI1_CS1) \
        || defined(CONFIG_DM3730LOGIC_SPI1_CS2) \
        || defined(CONFIG_DM3730LOGIC_SPI1_CS3) \
        || defined(CONFIG_DM3730LOGIC_SPI3_CS0) \
        || defined(CONFIG_DM3730LOGIC_SPI3_CS1)
static struct omap2_mcspi_device_config expansion_board_mcspi_config = {
        .turbo_mode     = 0,
        .single_channel = 1,    /* 0: slave, 1: master */ 
                                // Note: This doesn't actually seem to be connected 
	                        // anywhere in the code base.
};
#endif

#ifdef CONFIG_DM3730LOGIC_SPI1_CS0
static struct spi_board_info dm3730logic_spi1_expansion_board_cs0 = {
        /*
         * Generic SPI on expansion board, SPI1/CS0
         */
        .modalias               = "spidev",
        .bus_num                = 1,
        .chip_select            = 0,
        .max_speed_hz           = 48000000,
        .controller_data        = &expansion_board_mcspi_config,
        .irq                    = 0,
        .platform_data          = NULL,
        .bits_per_word          = 8,
};
#endif

#ifdef CONFIG_DM3730LOGIC_SPI1_CS1
static struct spi_board_info dm3730logic_spi1_expansion_board_cs1 = {
	/*
	 * Generic SPI on expansion board, SPI1/CS1
	 */
	.modalias		= "spidev",
	.bus_num		= 1,
	.chip_select		= 1,
	.max_speed_hz		= 48000000,
	.controller_data	= &expansion_board_mcspi_config,
	.irq			= 0,
	.platform_data		= NULL,
	.bits_per_word		= 8,
};
#endif

#ifdef CONFIG_DM3730LOGIC_SPI1_CS2
static struct spi_board_info dm3730logic_spi1_expansion_board_cs2 = {
	/*
	 * Generic SPI on expansion board, SPI1/CS2
	 */
	.modalias		= "spidev",
	.bus_num		= 1,
	.chip_select		= 2,
	.max_speed_hz		= 48000000,
	.controller_data	= &expansion_board_mcspi_config,
	.irq			= 0,
	.platform_data		= NULL,
	.bits_per_word		= 8,
};
#endif

#ifdef CONFIG_DM3730LOGIC_SPI1_CS3
static struct spi_board_info dm3730logic_spi1_expansion_board_cs3 = {
	/*
	 * Generic SPI on expansion board, SPI1/CS3
	 */
	.modalias		= "spidev",
	.bus_num		= 1,
	.chip_select		= 3,
	.max_speed_hz		= 48000000,
	.controller_data	= &expansion_board_mcspi_config,
	.irq			= 0,
	.platform_data		= NULL,
	.bits_per_word		= 8,
};
#endif

#ifdef CONFIG_DM3730LOGIC_SPI3_CS0
static struct spi_board_info dm3730logic_spi3_expansion_board_cs0 = {
	/*
	 * Generic SPI on expansion board, SPI3/CS0
	 */
	.modalias		= "spidev",
	.bus_num		= 3,
	.chip_select		= 0,
	.max_speed_hz		= 48000000,
	.controller_data	= &expansion_board_mcspi_config,
	.irq			= 0,
	.platform_data		= NULL,
	.bits_per_word		= 8,
};
#endif

#ifdef CONFIG_DM3730LOGIC_SPI3_CS1
static struct spi_board_info dm3730logic_spi3_expansion_board_cs1= {
	/*
	 * Generic SPI on expansion board, SPI3/CS1
	 */
	.modalias		= "spidev",
	.bus_num		= 3,
	.chip_select		= 1,
	.max_speed_hz		= 48000000,
	.controller_data	= &expansion_board_mcspi_config,
	.irq			= 0,
	.platform_data		= NULL,
	.bits_per_word		= 8,
};
#endif

/* SPI device entries we can have */
static struct spi_board_info dm3730logic_spi_devices[7];

static void dm3730logic_spi_init(void)
{
	int num_spi_devices = 0;
	int use_mcspi1 = 0;
	int use_mcspi3 = 0;

	if (machine_is_dm3730_som_lv()) {
		/* LV SOM only has the brf6300 on SPI */
#ifdef CONFIG_BT_HCIBRF6300_SPI
		dm3730logic_spi_devices[num_spi_devices++] = dm3730logic_spi_brf6300;
		omap_mux_init_signal("mcspi1_cs0", OMAP_PIN_INPUT);
		use_mcspi1 = 1;
#endif
	}
	
#ifdef CONFIG_DM3730LOGIC_SPI1_CS0
	/* SPIDEV on McSPI1/CS0 can only work if we aren't using it
	   for either the bfr6300 or at25160an, each of which would
	   set use_mcspi1 to non-zero. */
	if (!use_mcspi1) {
		dm3730logic_spi_devices[num_spi_devices++] = 
			dm3730logic_spi1_expansion_board_cs0;
		omap_mux_init_signal("mcspi1_cs0", OMAP_PIN_INPUT);
		use_mcspi1=1;
	}
#endif
#ifdef CONFIG_DM3730LOGIC_SPI1_CS1
	if (machine_is_dm3730_som_lv()) {
		printk("Can't setup SPIDEV SPI1/CS1 as McSPI1_CS1 not available on LV SOM\n");
	} else {
		dm3730logic_spi_devices[num_spi_devices++] = 
			dm3730logic_spi1_expansion_board_cs1;
		omap_mux_init_signal("mcspi1_cs1", OMAP_PIN_INPUT);
		use_mcspi1=1;
	}
#endif
#ifdef CONFIG_DM3730LOGIC_SPI1_CS2
	if (machine_is_dm3730_som_lv()) {
		printk("Can't setup SPIDEV SPI1/CS2 as McSPI1_CS2 not available on LV SOM\n");
	} else {
		dm3730logic_spi_devices[num_spi_devices++] = 
			dm3730logic_spi1_expansion_board_cs2;
		omap_mux_init_signal("mcspi1_cs2.mcspi1_cs2", OMAP_PIN_INPUT);
		use_mcspi1=1;
	}
#endif
#ifdef CONFIG_DM3730LOGIC_SPI1_CS3
	if (machine_is_dm3730_som_lv()) {
		printk("Can't setup SPIDEV SPI1/CS3 as McSPI1_CS3 not available on LV SOM\n");
	} else {
		dm3730logic_spi_devices[num_spi_devices++] = 
			dm3730logic_spi1_expansion_board_cs3;
		omap_mux_init_signal("mcspi1_cs3.mcspi1_cs3", OMAP_PIN_INPUT);
		use_mcspi1=1;
	}
#endif
#ifdef CONFIG_DM3730LOGIC_SPI3_CS0
	dm3730logic_spi_devices[num_spi_devices++] = 
		dm3730logic_spi3_expansion_board_cs0;
	if (machine_is_dm3730_som_lv())
		omap_mux_init_signal("sdmmc2_dat3.mcspi3_cs0", OMAP_PIN_INPUT); 
	else
		omap_mux_init_signal("etk_d2.mcspi3_cs0", OMAP_PIN_INPUT); 
	use_mcspi3=1;
#endif
#ifdef CONFIG_DM3730LOGIC_SPI3_CS1
	dm3730logic_spi_devices[num_spi_devices++] = 
		dm3730logic_spi3_expansion_board_cs1;
	if (machine_is_dm3730_som_lv())
		omap_mux_init_signal("sdmmc2_dat2.mcspi3_cs1", OMAP_PIN_INPUT); 
	else
		omap_mux_init_signal("etk_d7.mcspi3_cs1", OMAP_PIN_INPUT); 
	use_mcspi3=1;
#endif
	
	if (use_mcspi1) {
		/* config MCSPI1 pins */
		omap_mux_init_signal("mcspi1_clk", OMAP_PIN_INPUT);
		omap_mux_init_signal("mcspi1_simo", OMAP_PIN_INPUT);
		omap_mux_init_signal("mcspi1_somi", OMAP_PIN_INPUT);
	}
	
	if (use_mcspi3) {
		if (machine_is_dm3730_som_lv()) {
			omap_mux_init_signal("sdmmc2_clk.mcspi3_clk", 
					     OMAP_PIN_INPUT);
			omap_mux_init_signal("sdmmc2_cmd.mcspi3_simo", 
					     OMAP_PIN_INPUT);
			omap_mux_init_signal("sdmmc2_dat0.mcspi3_somi", 
					     OMAP_PIN_INPUT); 		
		} else {
			omap_mux_init_signal("etk_d3.mcspi3_clk", OMAP_PIN_INPUT);
			omap_mux_init_signal("etk_d0.mcspi3_simo", OMAP_PIN_INPUT);
			omap_mux_init_signal("etk_d1.mcspi3_somi", OMAP_PIN_INPUT);
		}
	}

	if (num_spi_devices)
		spi_register_board_info(dm3730logic_spi_devices, num_spi_devices);

}

#ifdef CONFIG_PRINTK_DEBUG
struct printk_debug *printk_debug;
#endif

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
	.product_name          = "DM3730 SOM LV/Torpedo",
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

static void dm3730logic_android_gadget_init(void)
{
	unsigned int val[2];
	unsigned int reg;

	reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;
	val[0] = omap_readl(reg);
	val[1] = omap_readl(reg + 4);

	snprintf(device_serial, MAX_USB_SERIAL_NUM, "%08X%08X", val[1], val[0]);

	platform_device_register(&androidusb_device);
}

static void __init dm3730logic_init(void)
{
	/* hang on start */
	while (dm3730logic_hang)
		;
#ifdef CONFIG_PRINTK_DEBUG
	printk_debug = (void *)PAGE_OFFSET;
	printk_debug->tag = PRINTK_DEBUG_COOKIE;
#endif

	omap3_mux_init(omap36x_board_mux, OMAP_PACKAGE_CBP);

	dm3730logic_i2c_init();

	platform_add_devices(dm3730logic_devices, ARRAY_SIZE(dm3730logic_devices));

#ifdef CONFIG_DM3730LOGIC_UART_A
	printk(KERN_INFO "Setup pinmux and enable UART A\n");
	omap_mux_init_signal("uart1_tx.uart1_tx", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart1_rts.uart1_rts", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart1_cts.uart1_cts", OMAP_PIN_INPUT);
	omap_mux_init_signal("uart1_rx.uart1_rx", OMAP_PIN_INPUT);
	omap_serial_init_port(0);
#endif

#ifdef CONFIG_DM3730LOGIC_UART_B
	printk(KERN_INFO "Setup pinmux and enable UART B\n");
	omap_mux_init_signal("uart3_tx_irtx.uart3_tx_irtx", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart3_rts_sd.uart3_rts_sd", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart3_cts_rctx.uart3_cts_rctx", OMAP_PIN_INPUT);
	omap_mux_init_signal("uart3_rx_irrx.uart3_rx_irrx", OMAP_PIN_INPUT);
	omap_serial_init_port(2);
#endif

#ifdef CONFIG_DM3730LOGIC_UART_C
	printk(KERN_INFO "Setup pinmux and enable UART C\n");
	omap_mux_init_signal("uart2_tx.uart2_tx", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart2_rts.uart2_rts", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart2_cts.uart2_cts", OMAP_PIN_INPUT);
	omap_mux_init_signal("uart2_rx.uart2_rx", OMAP_PIN_INPUT);
	omap_serial_init_port(1);
#endif

	spi_register_board_info(dm3730logic_spi_board_info,
				ARRAY_SIZE(dm3730logic_spi_board_info));

	/* Did u-boot write valid productID data? */
	dm3730logic_fetch_sram_product_id_data();

	dm3730logic_usb_init();
	dm3730logic_musb_init();

	dm3730logic_flash_init();
	dm3730logic_init_smsc911x();
	dm3730logic_gpio_init();
	enable_board_wakeup_source();
	dm3730logic_init_wifi_mux();

	dm3730logic_spi_init();

	dm3730logic_android_gadget_init();
}

#if defined(CONFIG_DM3730LOGIC_COMPACT_FLASH) || defined(CONFIG_DM3730LOGIC_COMPACT_FLASH_MODULE)

#define DM3730_SOM_LV_CF_RESET_GPIO 6
#define DM3730_SOM_LV_CF_EN_GPIO 128
#define DM3730_SOM_LV_CF_CD_GPIO 154

static struct resource dm3730logic_som_lv_cf_resources[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = OMAP_GPIO_IRQ(DM3730_SOM_LV_CF_CD_GPIO),
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct dm3730logic_cf_data cf_data = {
	.gpio_reset = DM3730_SOM_LV_CF_RESET_GPIO,
	.gpio_en = DM3730_SOM_LV_CF_EN_GPIO,
	.gpio_cd = DM3730_SOM_LV_CF_CD_GPIO,
};

static struct platform_device dm3730logic_som_lv_cf = {
	.name		= "dm3730logic-cf",
	.id		= 0,
	.dev		= {
		.platform_data	= &cf_data,
	},
	.num_resources = ARRAY_SIZE(dm3730logic_som_lv_cf_resources),
	.resource = dm3730logic_som_lv_cf_resources,
};

void dm3730logic_cf_init(void)
{
	unsigned long cs_mem_base;
	int result;

	/* Only the SOM LV SDK has a CF interface */
	if (!machine_is_dm3730_som_lv())
		return;

	/* Fix PBIAS to get USIM enough voltage to power up */
	dm3730torpedo_fix_pbias_voltage();

	if (gpmc_cs_request(3, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for CF\n");
		return;
	}

	dm3730logic_som_lv_cf_resources[0].start = cs_mem_base;
	dm3730logic_som_lv_cf_resources[0].end = cs_mem_base + 0x1fff;
	

	omap_mux_init_signal("gpmc_ncs3", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("gpmc_io_dir", OMAP_PIN_OUTPUT);

	omap_mux_init_gpio(DM3730_SOM_LV_CF_CD_GPIO, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(DM3730_SOM_LV_CF_CD_GPIO, "CF card detect") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for CompactFlash CD IRQ\n",
		DM3730_SOM_LV_CF_CD_GPIO);
		return;
	}
	omap_set_gpio_debounce(DM3730_SOM_LV_CF_CD_GPIO, 5);
	gpio_direction_input(DM3730_SOM_LV_CF_CD_GPIO);

	// Setup ComapctFlash Enable pin
	omap_mux_init_gpio(DM3730_SOM_LV_CF_EN_GPIO, OMAP_PIN_OUTPUT);
	if (gpio_request(DM3730_SOM_LV_CF_EN_GPIO, "CF enable") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for CompactFlash EN\n",
		DM3730_SOM_LV_CF_EN_GPIO);
		return;
	}
	gpio_direction_output(DM3730_SOM_LV_CF_EN_GPIO, 0);

	// Setup ComapctFlash Reset pin
	omap_mux_init_gpio(DM3730_SOM_LV_CF_RESET_GPIO, OMAP_PIN_OUTPUT);
	if (gpio_request(DM3730_SOM_LV_CF_RESET_GPIO, "CF reset") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for CompactFlash Reset\n",
		DM3730_SOM_LV_CF_RESET_GPIO);
		return;
	}
	gpio_direction_output(DM3730_SOM_LV_CF_RESET_GPIO, 0);

	result = platform_device_register(&dm3730logic_som_lv_cf);
	if (result)
		printk("%s: platform device register of CompactFlash device failed: %d\n", __FUNCTION__, result);
}
#else
static void dm3730logic_cf_init(void)
{
}
#endif


/*
 * dm3730logic_init_productid_specifics called IFF known valid product
 * ID data is found.
 */
void dm3730logic_init_productid_specifics(void)
{
	dm3730logic_init_twl_audio();
	brf6300_dev_init();
	dm3730logic_cf_init();
}

static void __init dm3730logic_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(DM3730_SOM_LV, "DM3730 SOM LV")
	/* Maintainer: Peter Barada - Logic Product Development */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= dm3730logic_map_io,
	.init_irq	= dm3730logic_init_irq,
	.init_machine	= dm3730logic_init,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(DM3730_TORPEDO, "DM3730 Torpedo")
	/* Maintainer: Peter Barada - Logic Product Development */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= dm3730logic_map_io,
	.init_irq	= dm3730logic_init_irq,
	.init_machine	= dm3730logic_init,
	.timer		= &omap_timer,
MACHINE_END
