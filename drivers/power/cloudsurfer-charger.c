/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Driver for chargers which report their online status through a GPIO pin
 *
 *  Copyright (C) 2012 Topher Cawlfield <ccawlfield@aircell.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/*
 *  Based on gpio-charger from Lars-Peter Clausen, with added i2c operations
 *  to initiate and regulate charging.
 *
 *  TARR - 25Oct2012 - Add sysfs functionality to modify the charge current
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <linux/power/cloudsurfer-charger.h>

/* local mnemomics */
#define PCA9536_INIT_REG	0x03
#define PCA9536_INIT_DATA	0x0c
#define CHARGE_RATE_REG		0x01
#define CHARGE_CURRENT_0	0
#define CHARGE_RATE_OFF		0x01
#define CHARGE_CURRENT_100	100
#define CHARGE_RATE_TRICKLE	0x00
#define CHARGE_CURRENT_500	500
#define CHARGE_RATE_FULL	0x02

#define OP_PLUG_EVENT		1
#define OP_SLEEP			2
#define OP_WAKE				3
#define DELAY_REPORT_SECS	5

struct gpio_charger {
	struct i2c_client *client;
	const struct gpio_charger_platform_data *pdata;
	unsigned int online;
	unsigned int charge_current;
	unsigned int irq;
	struct work_struct irq_work;
	struct delayed_work delayed_report;
	struct power_supply charger;
};

static struct gpio_charger *G_gpio_charger;

static int pca9536_i2c_write(struct i2c_client *client, u8 reg, u8 value)
{
	struct i2c_msg xfer;
	u8 buf[2];
	
	/*printk("PCA9536 - write %d to register %d\n", value, reg);*/
	buf[0] = reg;
	buf[1] = value;

	/* Write the register */
    xfer.addr = client->addr;
    xfer.flags = 0;
    xfer.len = 2;
    xfer.buf = buf;

    if (i2c_transfer(client->adapter, &xfer, 1) != 1) {
		dev_err(&client->dev, 
				"%s: i2c transfer failed -- battery not charging!\n", __func__);
        return -EIO;
    }
    return 0;
}

static inline struct gpio_charger *psy_to_gpio_charger(struct power_supply *psy)
{
	return container_of(psy, struct gpio_charger, charger);
}

static void set_charger_state(struct gpio_charger *gc, int operation_id) 
{
	int online;	
	int level;
	const struct gpio_charger_platform_data *pdata = gc->pdata;
	struct i2c_client *client = gc->client;

	online = gpio_get_value(pdata->gpio);
	online ^= pdata->gpio_active_low;
	
	if (!gc->online && !online) 
		return;
	gc->online = online;

	if (operation_id == OP_PLUG_EVENT) {
		pca9536_i2c_write(client, PCA9536_INIT_REG, PCA9536_INIT_DATA);
		if ( gc->online ) {
			level = CHARGE_RATE_TRICKLE;
			gc->charge_current = CHARGE_CURRENT_100;
		} else {
			level = CHARGE_RATE_OFF;
			gc->charge_current = CHARGE_CURRENT_0; 
		}
		pca9536_i2c_write(client, CHARGE_RATE_REG, level);
	} else if (operation_id == OP_SLEEP) {
		pca9536_i2c_write(client, CHARGE_RATE_REG, CHARGE_RATE_FULL);
		gc->charge_current = CHARGE_CURRENT_500;
	} else if (operation_id == OP_WAKE) {
		pca9536_i2c_write(client, PCA9536_INIT_REG, PCA9536_INIT_DATA);
		pca9536_i2c_write(client, CHARGE_RATE_REG, CHARGE_RATE_TRICKLE);
		gc->charge_current = CHARGE_CURRENT_100;
	}
}

static void set_charger_on_irq(struct work_struct *work)
{
	struct gpio_charger *gc =
		container_of(work, struct gpio_charger, irq_work);
	set_charger_state(gc, OP_PLUG_EVENT);
	/*power_supply_changed(&gc->charger);*/
	schedule_delayed_work(&gc->delayed_report, DELAY_REPORT_SECS * HZ);
}

/*
 * The fuel gauge takes 3-4 seconds before it shows the correct status.
 * We have to wait a bit before reporting power_supply_changed.
 */
static void delayed_change_report(struct work_struct *work)
{
	struct gpio_charger *gc =
		container_of(work, struct gpio_charger, delayed_report.work);
	power_supply_changed(&gc->charger);
}

static irqreturn_t gpio_charger_irq(int irq, void *devid)
{
	struct power_supply *charger = devid;
	struct gpio_charger *gc = psy_to_gpio_charger(charger);

	schedule_work(&gc->irq_work); /* will soon call set_charger_on_irq() above */

	return IRQ_HANDLED;
}

static int gpio_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct gpio_charger *gpio_charger = psy_to_gpio_charger(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = gpio_charger->online;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property gpio_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static ssize_t show_valid_charge_current(struct device *dev, struct device_attribute *attr, char *buf) 
{
	return sprintf(buf,"%d,%d,%d\n",CHARGE_CURRENT_0,CHARGE_CURRENT_100,
				CHARGE_CURRENT_500);
}

static DEVICE_ATTR(valid_charge_current,S_IRUGO,show_valid_charge_current,NULL);

static ssize_t show_charger(struct device *dev, struct device_attribute *attr, char *buf) 
{
	return sprintf(buf,"%d\n",G_gpio_charger->charge_current);
}

static ssize_t set_charger(struct device *dev, struct device_attribute *attr, char *buf,size_t count) 
{
	int tcurrent;
	int charge_rate = -1;
	if ( G_gpio_charger->online == 0 )
		return count;

	sscanf(buf,"%d",&tcurrent);
	switch ( tcurrent ) {
		case CHARGE_CURRENT_0:
			charge_rate = CHARGE_RATE_OFF;
			break;
		case CHARGE_CURRENT_100:
			charge_rate = CHARGE_RATE_TRICKLE;
			break;
		case CHARGE_CURRENT_500:
			charge_rate = CHARGE_RATE_FULL;
			break;
		default:
			charge_rate = -1;
			break;
	}
	if ( charge_rate != -1 ) {
		pca9536_i2c_write(G_gpio_charger->client, CHARGE_RATE_REG, charge_rate);
		G_gpio_charger->charge_current = tcurrent;
	}		
	return count;
}

static DEVICE_ATTR(charge_current,S_IRUGO| S_IWUSR,show_charger,set_charger);

static struct attribute *cloudsurfer_charger_sysfs_entries[] = {
	&dev_attr_charge_current.attr,
	&dev_attr_valid_charge_current.attr,
	NULL,
};

static struct attribute_group cloudsurfer_charger_attr_group = {
	.name 	= NULL,
	.attrs	= cloudsurfer_charger_sysfs_entries,
};

static int gpio_charger_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct gpio_charger *gpio_charger;
	const struct gpio_charger_platform_data *pdata = client->dev.platform_data;
	struct power_supply *charger;
	int ret;
	int irq;

	dev_info(&client->dev, "beginning gpio-charger probe");

	if (!pdata) {
		dev_err(&client->dev, "No platform data\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio)) {
		dev_err(&client->dev, "Invalid gpio pin\n");
		return -EINVAL;
	}

	gpio_charger = kzalloc(sizeof(*gpio_charger), GFP_KERNEL);
	if (!gpio_charger) {
		dev_err(&client->dev, "Failed to alloc driver structure\n");
		return -ENOMEM;
	}
	charger = &gpio_charger->charger;

	charger->name = pdata->name ? pdata->name : "charger";
	charger->type = pdata->type;
	charger->properties = gpio_charger_properties;
	charger->num_properties = ARRAY_SIZE(gpio_charger_properties);
	charger->get_property = gpio_charger_get_property;
	charger->supplied_to = pdata->supplied_to;
	charger->num_supplicants = pdata->num_supplicants;

	gpio_charger->client = client;

	/* already done in board-cloudsurfer
	ret = gpio_request(pdata->gpio, dev_name(&client->dev));
	if (ret) {
		dev_err(&client->dev, "Failed to request gpio pin %d: %d\n", pdata->gpio, ret);
		goto err_free;
	}
	ret = gpio_direction_input(pdata->gpio);
	if (ret) {
		dev_err(&client->dev, "Failed to set gpio to input: %d\n", ret);
		goto err_gpio_free;
	} */

	gpio_charger->pdata = pdata;

	INIT_WORK(&gpio_charger->irq_work, set_charger_on_irq);
	INIT_DELAYED_WORK(&gpio_charger->delayed_report, delayed_change_report);

	ret = power_supply_register(&client->dev, charger);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register power supply: %d\n",
			ret);
		goto err_gpio_free;
	}

	irq = 0;
	irq = gpio_to_irq(pdata->gpio);
	if (irq > 0) {
		ret = request_irq(irq, gpio_charger_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				dev_name(&client->dev), charger);
		if (ret < 0)
			dev_warn(&client->dev, "Failed to request irq: %d\n", ret);
		else
			gpio_charger->irq = irq;
	}

	/*platform_set_drvdata(pdev, gpio_charger);*/
	i2c_set_clientdata(client, gpio_charger);

	if ( sysfs_create_group(&charger->dev->kobj,&cloudsurfer_charger_attr_group)) 
		dev_err(&client->dev,"Failed to create sysfs entries\n");

	schedule_work(&gpio_charger->irq_work);
	G_gpio_charger = gpio_charger;
	return 0;

err_gpio_free:
	gpio_free(pdata->gpio);
//err_free:
	kfree(gpio_charger);
	return ret;
}

static int gpio_charger_remove(struct i2c_client *client)
{
	struct gpio_charger *gpio_charger = i2c_get_clientdata(client);

	if (gpio_charger->irq)
		free_irq(gpio_charger->irq, &gpio_charger->charger);

	power_supply_unregister(&gpio_charger->charger);

	gpio_free(gpio_charger->pdata->gpio);

	//platform_set_drvdata(pdev, NULL);
	i2c_set_clientdata(client, NULL);
	kfree(gpio_charger);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_charger_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_charger *gpio_charger = platform_get_drvdata(pdev);

	set_charger_state(gpio_charger, OP_WAKE);
	power_supply_changed(&gpio_charger->charger);

	return 0;
}

static int gpio_charger_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_charger *gpio_charger = platform_get_drvdata(pdev);

	set_charger_state(gpio_charger, OP_SLEEP);

	return 0;
}
#endif

static const struct i2c_device_id pca9536_id[] = {
	{ "cloudsurfer-charger", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c,pca9536_id);

static SIMPLE_DEV_PM_OPS(gpio_charger_pm_ops, gpio_charger_suspend, gpio_charger_resume);

static struct i2c_driver gpio_charger_driver = {
	.driver = {
		.name = "cloudsurfer-charger",
		.owner = THIS_MODULE,
		.pm = &gpio_charger_pm_ops,
	},
	.probe = gpio_charger_probe,
	.remove = gpio_charger_remove,
	.id_table = pca9536_id,
};

/*module_platform_driver(gpio_charger_driver);*/
/* The above macro is from a more recent kernel version.
 * it would have replaced the boiler-plate code below
 */

static int __init gpio_charger_init(void)
{
	/*return platform_driver_register(&gpio_charger_driver);*/
	return i2c_add_driver(&gpio_charger_driver);
}
module_init(gpio_charger_init);

static void __exit gpio_charger_exit(void)
{
	/*platform_driver_unregister(&gpio_charger_driver);*/
	i2c_del_driver(&gpio_charger_driver);
}
module_exit(gpio_charger_exit);


MODULE_AUTHOR("Topher Cawlfield <ccawlfield@aircell.com>");
MODULE_DESCRIPTION("Driver for ASP battery charger");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:charger");
