/*
    Copyright (C) 1998, 1999  Frodo Looijaard <frodol@dds.nl> and
			       Philip Edelbrock <phil@netroedge.com>
    Copyright (C) 2003 Greg Kroah-Hartman <greg@kroah.com>
    Copyright (C) 2003 IBM Corp.
    Copyright (C) 2004 Jean Delvare <khali@linux-fr.org>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
//#include <linux/i2c.h>
#include <linux/mutex.h>

#include <linux/fs.h>


#define DEVICE_NAME 	"logicEEPROM"
#define LOGIC_MAJOR		240
#define EEPROM_SIZE		256

char buff[EEPROM_SIZE];

static int logic_eeprom_open(struct inode *, struct file *);
static int logic_eeprom_release(struct inode *, struct file *);
static ssize_t logic_eeprom_read(struct file *, char *, size_t, loff_t *);
static ssize_t logic_eeprom_write(struct file *, const char *, size_t, loff_t *);


static int logic_eeprom_open(struct inode *inode, struct file *fd)
{
	printk(KERN_INFO "^^^^^^^^^^^^^^^^^^^EEPROM open\n");
	return 0;
}

static int logic_eeprom_release(struct inode *inode, struct file *fd)
{
	printk(KERN_INFO "^^^^^^^^^^^^^^^^^^^EEPROM release\n");
	return 0;
}

static ssize_t logic_eeprom_read(struct file *fd, char __user *data, size_t count, loff_t *f_pos)
{
	printk(KERN_INFO "^^^^^^^^^^^^^^^^^^^EEPROM read\n");
	return 0;
}

static ssize_t logic_eeprom_write(struct file *fd, const char __user *data, size_t count, loff_t *f_pos)
{
	printk(KERN_INFO "^^^^^^^^^^^^^^^^^^^EEPROM write\n");
	return 0;
}


static struct file_operations eeprom_fops = {
		.owner	= THIS_MODULE,
		.read	= logic_eeprom_read,
		.write	= logic_eeprom_write,
		.open	= logic_eeprom_open,
		.release = logic_eeprom_release,
};

/*
static int logic_eeprom_probe(struct device *dev)
{
	return 0;
}

static int logic_eeprom_remove(struct device *dev)
{
	return 0;
}

static struct device_driver logic_eeprom_driver = {
		.name		= "logicEEPROM",
		.owner		= THIS_MODULE,
		.bus		= &platform_bus_type,
		.probe		= logic_eeprom_probe,
		.remove		= logic_eeprom_remove,
};
*/

static int __init logic_eeprom_init(void)
{
	int ret = register_chrdev(LOGIC_MAJOR, DEVICE_NAME, &eeprom_fops);

	if (ret < 0) {
		printk(KERN_ERR "*******************failed to register logic EEPROM char driver.\n");
		return ret;
	}

/*
	ret = driver_register(&logic_eeprom_driver);

	if (ret < 0) {
		printk(KERN_ERR "*******************failed to register logic EEPROM platform driver.\n");
		return ret;
	}
*/

	printk(KERN_INFO "^^^^^^^^^^^^^^^^^^^^logic EEPROM registered.\n");
	return 0;
}

static void __exit logic_eeprom_exit(void)
{
	unregister_chrdev(LOGIC_MAJOR, DEVICE_NAME);
//	driver_unregister(&logic_eeprom_driver);
	printk(KERN_INFO "^^^^^^^^^^^^^^^^^^^^logic EEPROM unregistered.\n");
}

module_init(logic_eeprom_init);
module_exit(logic_eeprom_exit);

MODULE_AUTHOR("Mark Chung <mark.chung@logicpd.com>");
MODULE_DESCRIPTION("I2C EEPROM driver");
MODULE_LICENSE("GPL");


