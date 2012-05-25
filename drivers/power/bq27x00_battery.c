/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <asm/unaligned.h>

#define DRIVER_VERSION			"1.0.0"

#ifdef CONFIG_BATTERY_BQ27000
#include "../w1/w1.h"
#endif
#ifdef CONFIG_BATTERY_BQ27200
#include <linux/i2c.h>
#endif

#define BQ27x00_REG_TEMP		0x06
#define BQ27x00_REG_VOLT		0x08
#define BQ27x00_REG_FLAGS		0x0A
#define BQ27x00_REG_RSOC		0x0B /* Relative State-of-Charge */
#define BQ27x00_REG_NAC         0x0C
#define BQ27x00_REG_LMD         0x12
#define BQ27x00_REG_AI			0x14
#define BQ27x00_REG_TTE         0x16
#define BQ27x00_REG_TTF         0x18
#define BQ27x00_REG_SAE         0x22
#define BQ27x00_REG_AP          0x24
#define BQ27x00_REG_TTECP       0x26
#define BQ27x00_REG_CSOC        0x2C
#define BQ27x00_REG_AR          0x02
#define BQ27x00_REG_MODE        0x01
#define BQ27x00_REG_CTRL        0x00
#define HIGH_BYTE(A)            ((A) << 8)

/* If the system has several batteries we need a different name for each
 * of them...
 */
#ifdef CONFIG_BATTERY_BQ27200
static DEFINE_IDR(battery_id);
#endif /* CONFIG_BATTERY_BQ27200 */
static DEFINE_MUTEX(battery_mutex);
static DEFINE_MUTEX(reg_lock);

struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27x00_device_info *di);
};

struct bq27x00_device_info {
	struct device 		*dev;
#ifdef CONFIG_BATTERY_BQ27200
	struct i2c_client	*client;
	int			id;
#endif
	//struct mutex reg_lock; /* protects data */

	struct bq27x00_access_methods	*bus;
	struct power_supply	bat;

	//struct i2c_client	*client;
};

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

/*
 * Common code for BQ27x00 devices
 */

static int bq27x00_read(u8 reg, int *rt_value, int b_single,
			struct bq27x00_device_info *di)
{
	int ret;
	
	mutex_lock(&reg_lock);
	ret = di->bus->read(reg, rt_value, b_single, di);
	mutex_unlock(&reg_lock);

	return ret;
}

/*
 * Return the battery temperature in Celsius degrees
 * Or < 0 if something fails.
 */
static int bq27x00_battery_temperature(struct bq27x00_device_info *di)
{
	int ret;
	int temp = 0;

	ret = bq27x00_read(BQ27x00_REG_TEMP, &temp, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	return (temp >> 2) - 273;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di)
{
	int ret;
	int volt = 0;

	ret = bq27x00_read(BQ27x00_REG_VOLT, &volt, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}

	return volt;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di)
{
	int ret;
	int curr = 0;
	int flags = 0;

	ret = bq27x00_read(BQ27x00_REG_AI, &curr, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}
	ret = bq27x00_read(BQ27x00_REG_FLAGS, &flags, 0, di);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return 0;
	}
	if ((flags & (1 << 7)) != 0) {
		dev_dbg(di->dev, "negative current!\n");
		return -curr;
	}
	return curr;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27x00_battery_rsoc(struct bq27x00_device_info *di)
{
	int ret;
	int rsoc = 0;

	ret = bq27x00_read(BQ27x00_REG_RSOC, &rsoc, 1, di);
	if (ret) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}

	/* if the capacity is below 5%, just report back 5% */
	if (rsoc < 5) {
		rsoc = 5;
	}

	return rsoc;
}

static int bq27x00_battery_timetofull(struct bq27x00_device_info *di)
{
	int ret;
	int ttf = 0;

	ret = bq27x00_read(BQ27x00_REG_TTF, &ttf, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading time to full\n");
		return ret;
	}

	return ttf;
}

static int bq27x00_battery_timetoempty(struct bq27x00_device_info *di)
{
	int ret;
	int tte = 0;

	ret = bq27x00_read(BQ27x00_REG_TTE, &tte, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading time to empty\n");
		return ret;
	}

	return tte;

}

static int bq27x00_battery_energy(struct bq27x00_device_info *di)
{
	int ret;
	int sae = 0;

	ret = bq27x00_read(BQ27x00_REG_SAE, &sae, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading available energy\n");
		return ret;
	}

	return sae;

}

static int bq27x00_battery_technology(struct bq27x00_device_info *di)
{
	int ret = POWER_SUPPLY_TECHNOLOGY_LION;

	return ret;
}

static int bq27x00_battery_status(struct bq27x00_device_info *di)
{
	int ret = POWER_SUPPLY_STATUS_UNKNOWN, r = 0;
	int flags = 0, rsoc = 0;

	ret = bq27x00_read(BQ27x00_REG_FLAGS, &flags, 0, di);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return 0;
	}

	if ((flags & 0xFF) & 0x80) {
		/* CHGS (bit 7) set means charging */
		ret = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		ret = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	r = bq27x00_read(BQ27x00_REG_RSOC, &rsoc, 1, di);
	if (r < 0) {
		dev_err(di->dev, "error reading rsoc\n");
	} else {
		if (rsoc == 100) {
			ret = POWER_SUPPLY_STATUS_FULL;
		}
	}
	
	return ret;
}

static int bq27x00_battery_health(struct bq27x00_device_info *di)
{
	int ret = POWER_SUPPLY_HEALTH_UNKNOWN;
	int temp = 0, volt = 0;

	temp = bq27x00_battery_temperature(di);
	if (temp > 50) 
		ret = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (temp < 10)
		ret = POWER_SUPPLY_HEALTH_COLD;
	else
		ret = POWER_SUPPLY_HEALTH_GOOD;

	volt = bq27x00_battery_voltage(di);
	if (volt > 4250)
		ret = POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	return ret;
}

#define to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, bat);

static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq27x00_battery_status(di);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq27x00_battery_health(di);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq27x00_battery_voltage(di);
		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq27x00_battery_current(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq27x00_battery_rsoc(di);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27x00_battery_temperature(di);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		val->intval = bq27x00_battery_timetoempty(di);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		val->intval = bq27x00_battery_timetofull(di);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		val->intval = bq27x00_battery_energy(di);
		break;	
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = bq27x00_battery_technology(di);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27x00_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.external_power_changed = NULL;
}

/*
 * BQ27000 specific code
 */
#ifdef CONFIG_BATTERY_BQ27000

extern int w1_bq27000_read(struct device *dev, u8 ret);

static int bq27000_read(u8 reg, int *rt_value, int b_single,
			struct bq27x00_device_info *di)
{
	u8 val;

	val = w1_bq27000_read(di->dev, reg);
	*rt_value = val;

	if (!b_single) {
		val = w1_bq27000_read(di->dev, reg + 1);
		*rt_value += HIGH_BYTE((int) val);
	}

	return 0;
}

static int bq27000_battery_probe(struct platform_device *pdev)
{
	struct bq27x00_device_info *di;
	struct bq27x00_access_methods *bus;
	int retval = 0;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&pdev->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&pdev->dev, "failed to allocate access method data\n");
		kfree(di);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, di);
 
	di->dev = &pdev->dev;
	di->dev = pdev->dev.parent;
	di->bat.name = "bq27000";
	bus->read = &bq27000_read;
	di->bus = bus;

	bq27x00_powersupply_init(di);

	//mutex_init(&di->reg_lock);

	retval = power_supply_register(&pdev->dev, &di->bat);
	if (retval) {
		dev_err(&pdev->dev, "failed to register battery\n");
		goto batt_failed;
	}


	printk("TARR - %s\n",__FUNCTION__);
	dev_info(&pdev->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;

batt_failed:
	kfree(bus);
	kfree(di);
	return retval;
}

static int bq27000_battery_remove(struct platform_device *pdev)
{
	struct bq27x00_device_info *di = platform_get_drvdata(pdev);

	power_supply_unregister(&di->bat);
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

#endif /* CONFIG_BATTERY_BQ27000 */

/*
 * BQ27200 specific code
 */

#ifdef CONFIG_BATTERY_BQ27200

static int bq27200_read(u8 reg, int *rt_value, int b_single,
			struct bq27x00_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_be16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

static int bq27200_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27x00_device_info *di;
	struct bq27x00_access_methods *bus;
	int num;
	int retval = 0;

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "bq27200-%d", num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->bat.name = name;
	bus->read = &bq27200_read;
	di->bus = bus;
	di->client = client;

	bq27x00_powersupply_init(di);

	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register battery\n");
		goto batt_failed_4;
	}

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;

batt_failed_4:
	kfree(bus);
batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27200_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	power_supply_unregister(&di->bat);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);

	return 0;
}

#endif /* CONFIG_BATTERY_BQ27200 */

/*
 * Module stuff
 */

#ifdef CONFIG_BATTERY_BQ27000

static struct platform_driver bq27000_battery_driver = {
	.probe = bq27000_battery_probe,
	.remove = bq27000_battery_remove,

	.driver = {
		.name = "bq27000-battery",
	},
};

#endif /* CONFIG_BATTERY_BQ27000 */

#ifdef CONFIG_BATTERY_BQ27200

static const struct i2c_device_id bq27200_id[] = {
	{ "bq27200", 0 },
	{},
};

static struct i2c_driver bq27200_battery_driver = {
	.driver = {
		.name = "bq27200-battery",
	},
	.probe = bq27200_battery_probe,
	.remove = bq27200_battery_remove,
	.id_table = bq27200_id,
};

#endif /* CONFIG_BATTERY_BQ27200 */

static int __init bq27x00_battery_init(void)
{
	int ret = 0;

#ifdef CONFIG_BATTERY_BQ27000
	ret = platform_driver_register(&bq27000_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27000 driver\n");
#endif
#ifdef CONFIG_BATTERY_BQ27200
	ret = i2c_add_driver(&bq27200_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27200 driver\n");
#endif

	return ret;
}
module_init(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
#ifdef CONFIG_BATTERY_BQ27000
	platform_driver_unregister(&bq27000_battery_driver);
#endif
#ifdef CONFIG_BATTERY_BQ27200
	i2c_del_driver(&bq27200_battery_driver);
#endif
}
module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
