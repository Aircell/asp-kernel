/* mach-omap2/board-dm3730logic-wifi.c
 *
 * Driver for dm3730logic 1273 WLAN. derived from board-zoom3-wifi.c
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * Authors:
 *	Ohad Ben-Cohen		<ohad@bencohen.org>
 *	BVijay			<bvijay@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>

#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>

#include <plat/wifi_tiwlan.h>
#include <asm/mach-types.h>

static int omap_dm3730logic_wifi_cd;	/* WIFI virtual 'card detect' status */
static void (*wifi_status_cb) (int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int wifi_irq_gpio;

int omap_wifi_status_register(void (*callback) (int card_present,
						void *dev_id), void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;

	wifi_status_cb = callback;

	wifi_status_cb_devid = dev_id;
	return 0;
}

int omap_wifi_status(int irq)
{
	return omap_dm3730logic_wifi_cd;
}

int omap_dm3730logic_wifi_set_carddetect(int val)
{
	omap_dm3730logic_wifi_cd = val;

	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);

	return 0;
}

static int omap_dm3730logic_wifi_power_state;

int omap_dm3730logic_wifi_power(int on)
{
	int err;
	struct regulator	*regl;

	gpio_set_value(OMAP_DM3730LOGIC_WIFI_PMENA_GPIO, on);

	omap_dm3730logic_wifi_power_state = on;

	regl = regulator_get(NULL, "vaux3");
	if (IS_ERR(regl)) {
		pr_err("%s: unable to get vaux3 regulator\n", __FUNCTION__);
		return -1;
	}

	if( on ) {
		mdelay(15);
		err = regulator_enable(regl);
	} else {
		err = 0;
		if( regulator_is_enabled(regl) )
			err = regulator_disable(regl);
		regulator_put(regl);
	}
	if (err) {
		pr_err("%s: unable to enable/disable (%i) vaux3 regulator\n", __FUNCTION__, on);
		regulator_put(regl);
		regl = NULL;
		return -1;
	}

	return 0;
}

static int omap_dm3730logic_wifi_reset_state;
int omap_dm3730logic_wifi_reset(int on)
{
	omap_dm3730logic_wifi_reset_state = on;
	return 0;
}

struct wifi_platform_data omap_dm3730logic_wifi_control = {
	.set_power = omap_dm3730logic_wifi_power,
	.set_reset = omap_dm3730logic_wifi_reset,
	.set_carddetect = omap_dm3730logic_wifi_set_carddetect,
};

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct resource omap_dm3730logic_wifi_resources[] = {
	[0] = {
	       .name = "device_wifi_irq",
	       .start = -EINVAL,
	       .end = -EINVAL,
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	       },
};

static struct platform_device omap_dm3730logic_wifi_device = {
	.name = "device_wifi",
	.id = 1,
	.num_resources = ARRAY_SIZE(omap_dm3730logic_wifi_resources),
	.resource = omap_dm3730logic_wifi_resources,
	.dev = {
		.platform_data = &omap_dm3730logic_wifi_control,
	},
};
#endif

static int __init omap_dm3730logic_wifi_init(void)
{
	int ret;

	printk("TARR - %s\n",__FUNCTION__);

	wifi_irq_gpio = OMAP_DM3730LOGIC_WIFI_IRQ_GPIO;

	ret = gpio_request(wifi_irq_gpio, "wifi_irq");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__, wifi_irq_gpio);
		goto out;
	}
	gpio_direction_input(wifi_irq_gpio);

	omap_dm3730logic_wifi_resources[0].start = OMAP_GPIO_IRQ(wifi_irq_gpio);
	omap_dm3730logic_wifi_resources[0].end = OMAP_GPIO_IRQ(wifi_irq_gpio);

#ifdef CONFIG_WIFI_CONTROL_FUNC
	ret = platform_device_register(&omap_dm3730logic_wifi_device);
#endif
out:
	return ret;
}

device_initcall(omap_dm3730logic_wifi_init);
