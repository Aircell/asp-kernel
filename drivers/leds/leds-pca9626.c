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
#include <linux/leds-pca9626.h>

#define ldev_to_led(c)       container_of(c, struct pca9626_led, ldev)

/* local mnemomics */
#define PCA9626_MODE1		0x00
#define MODE1_RESET_VALUE	0x11
#define LED_REG_OFFSET		0x02
#define LED_PWM_CONTROL		0x1d

struct pca9626_data {
	struct i2c_client *client;
	struct pca9626_led *leds;
};

static struct pca9626_data *my_data;

static int pca9626_read(struct i2c_client *client, u8 reg, u8 *value)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	
	//printk("LED - read from address %d\n",client->addr);
	buf[0] = reg;

	/* Write the register */
    xfer[0].addr = client->addr;
    xfer[0].flags = 0;
    xfer[0].len = 1;
    xfer[0].buf = buf;

    /* Read data */
    xfer[1].addr = client->addr;
    xfer[1].flags = I2C_M_RD;
    xfer[1].len = 1;
    xfer[1].buf = buf;

    if (i2c_transfer(client->adapter, xfer, 2) != 2) {
        dev_err(&client->dev, "%s: i2c read failed\n", __func__);
        return -EIO;
    }
	//printk("LED - read 0x%2.2x 0x%2.2x\n",buf[0],buf[1]);
	*value = buf[0];
    return 0;
}
static int pca9626_write(struct i2c_client *client, u8 reg, u8 value)
{
	struct i2c_msg xfer;
	u8 buf[2];
	
	//printk("LED - write %d to register %d\n",client->addr);
	buf[0] = reg;
	buf[1] = value;

	/* Write the register */
    xfer.addr = client->addr;
    xfer.flags = 0;
    xfer.len = 2;
    xfer.buf = buf;

    if (i2c_transfer(client->adapter, &xfer, 1) != 1) {
        dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
        return -EIO;
    }
    return 0;
}

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

static void pca9626_set_brightness(struct led_classdev *led_cdev, 
		enum led_brightness value)
{
	struct pca9626_led *led;
		
	led = ldev_to_led(led_cdev);
	if ( pca9626_write(led->client,led->id+LED_REG_OFFSET,(u8)value) < 0 ) {
		printk("LED - write failed\n");
		return;
	}
	led->ldev.brightness = value;
	return;
}


static const struct i2c_device_id pca9626_id[] = {
	{ "pca9626", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c,pca9626_id);

static int pca9626_configure(struct i2c_client *client,
			struct pca9626_data *ld, 
			struct pca9626_platform_data *pd)
{
	int i;
	u8 value;
	struct led_classdev *led_class;

	/* set up the leds PWM control */
	value = 0xaa;	
	for (i=0; i<6; i++) {
		if ( pca9626_write(client,i+LED_PWM_CONTROL,value) < 0 ) {
			printk("LED - PWM control write failed\n");
			return -EIO;
		}
	}
	
	for (i=0; i<PCA9626_MAX_LEDS; i++ ) {
		pd->leds[i].client = client;			
		led_class = &pd->leds[i].ldev;
		led_class->brightness_set = pca9626_set_brightness;
		if ( led_classdev_register(&client->dev, led_class) < 0 ) {
			printk("LED - failed to register LED %d\n",i);
			return -ENOMEM;
		}
		/* Set initital brightness */
		pca9626_set_brightness(led_class,led_class->brightness);
	}
	return 0;
}

static int pca9626_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct pca9626_data *local_data = i2c_get_clientdata(client);
	struct pca9626_platform_data *platform_data = client->dev.platform_data;


	/* Is the device there? */
	if ( !pca9626_present(client) ) {
		return -EIO;
	}

	if ( !(local_data = kzalloc(sizeof(*local_data), GFP_KERNEL)) )
		return -ENOMEM;

	local_data->leds = platform_data->leds;
	local_data->client = client;
	if ( pca9626_configure(client,local_data,platform_data) < 0 ) {
		kfree(local_data);
		return -ENOMEM;
	}
	my_data = local_data;
	i2c_set_clientdata(client, local_data);

	/* Take the controller out of sleep mode */
	if ( pca9626_write(client,0x00,0x01) < 0 ) {
		printk(KERN_ERR "LED - PWM control write failed\n");
		return -EIO;
	}
	
	return 0;
}

static int pca9626_remove(struct i2c_client *client)
{
	struct pca9626_data *data = i2c_get_clientdata(client);

	kfree(data);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static struct i2c_driver pca9626_driver = {
	.driver = {
		.name = "pca9626",
		.owner = THIS_MODULE,
	},
	.probe = pca9626_probe,
	.remove = pca9626_remove,
	.id_table = pca9626_id,
};

static int __init pca9626_init(void)
{
	printk(KERN_INFO "LED CONTROLLER PCA9626  Init\n");
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

