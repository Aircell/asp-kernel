/*
 * pca9626.c - LED controller for Aircell CloudSurfer
 *
 * Copyright (C) 2011 Aircell 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * Datasheet: http://www.nxp.com/acrobat/datasheets/PCA9532_3.pdf
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#define ldev_to_led(c)       container_of(c, struct pca9626_led, ldev)

/* local mnemomics */
#define PCA9626_MODE1		0x00
#define MODE1_RESET_VALUE	0x91

struct pca9626_data {
	struct i2c_client *client;
	struct mutex update_lock;
	struct input_dev *idev;
	struct work_struct work;
	u8 pwm[2];
	u8 psc[2];
};

static int pca9626_read(struct i2c_client *client, u8 reg, u8 *value)
{
	struct i2c_msg xfer[2];
	u8 buf[2];

	buf[0] = reg;

	/* Write the register */
    xfer[0].addr = client->addr;
    xfer[0].flags = 0;
    xfer[0].len = 2;
    xfer[0].buf = buf;

    /* Read data */
    xfer[1].addr = client->addr;
    xfer[1].flags = I2C_M_RD;
    xfer[1].len = 2;
    xfer[1].buf = buf;

    if (i2c_transfer(client->adapter, xfer, 2) != 2) {
        dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
        return -EIO;
    }

	printk("LED - read 0x%2.2x 0x%2.2x\n",buf[0],buf[1]);
    return 0;
}
static int pca9626_write(struct i2c_client *client, u8 reg, u8 value);

static int pca9626_present(struct i2c_client *client) 
{
	u8 value;
	if ( pca9626_read(client,PCA9626_MODE1,&value) < 0 ) {
		return 0;
	}
	if ( value != MODE1_RESET_VALUE )
		return 0;
	else
		return 1;
}

static int pca9626_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int pca9626_remove(struct i2c_client *client);

static const struct i2c_device_id pca9626_id[] = {
	{ "pca9626", 0 },
	{ }
};


static struct i2c_driver pca9626_driver = {
	.driver = {
		.name = "pca9626",
	},
	.probe = pca9626_probe,
	.remove = pca9626_remove,
	.id_table = pca9626_id,
};

static int pca9626_event(struct input_dev *dev, unsigned int type,
	unsigned int code, int value)
{
	return 0;
}

static void pca9626_input_work(struct work_struct *work)
{
	return;

}

static void pca9626_led_work(struct work_struct *work)
{
	return;
}

static int pca9626_configure(struct i2c_client *client,
	struct pca9626_data *data, struct pca9626_platform_data *pdata)
{
	return 0;
}

static int pca9626_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct pca9626_data *data = i2c_get_clientdata(client);
	struct pca9626_platform_data *pca9626_pdata = client->dev.platform_data;
	int err;

	printk("LED CONTROLLER Probe\n");

	/* Is the device there? */
	if ( !pca9626_present(client) ) {
		return -EIO;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev_info(&client->dev, "setting platform data\n");
	i2c_set_clientdata(client, data);
	data->client = client;
	mutex_init(&data->update_lock);

	return 0;
}

static int pca9626_remove(struct i2c_client *client)
{
	struct pca9626_data *data = i2c_get_clientdata(client);

	kfree(data);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static int __init pca9626_init(void)
{
	printk("LED CONTROLLER Init\n");
	return i2c_add_driver(&pca9626_driver);
}

static void __exit pca9626_exit(void)
{
	i2c_del_driver(&pca9626_driver);
}

MODULE_AUTHOR("Steve Tarr");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PCA 9626 LED Controller");

module_init(pca9626_init);
module_exit(pca9626_exit);

