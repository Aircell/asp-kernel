#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/err.h>
#include <linux/delay.h>

#include "ehci-omap-usb3320-hack.h"

#include <plat/../../../mach-omap2/mux.h>

static struct device *usb3320_hack_dev = NULL;
static irq_handler_t usb3320_hack_return_call = NULL;

static int usb3320_hack_reset_gpio = 4;
static int usb3320_hack_wakeups_enabled = 0;

struct usb3320_hack_gpio_list
{
	int cleaned;
	int gpio;
	int direction;
	int old_mux;
	const char *name;
	int last_state;
	int wakeup;
};

static struct usb3320_hack_gpio_list usb3320_hack_signals[] =
{
	{
		.gpio = 28, 
		.direction = 0,
		.name = "USB_DAT0",
	},
	{
		.gpio = 29, 
		.direction = 0,
		.name = "USB_DAT1",
	},
	{
		.gpio = 177, 
		.direction = 0,
		.name = "USB_DAT2",
	},
	{
		.gpio = 182, 
		.direction = 0,
		.name = "USB_DAT3",
	},
	{
		.gpio = 179, 
		.direction = 0,
		.name = "USB_DAT4",
	},
	{
		.gpio = 180,
		.direction = 0,
		.name = "USB_DAT5",
	},
	{
		.gpio = 181, 
		.direction = 0,
		.name = "USB_DAT6",
	},
	{
		.gpio = 178, 
		.direction = 0,
		.name = "USB_DAT7",
	},
	{
		.gpio = 27,
		.direction = 0,
		.name = "USB_NXT",
	},
	{
		.gpio = 26,
		.direction = 0,
		.name = "USB_DIR",
	}, 
	{
		.gpio = 25,
		.direction = 1,
		.name = "USB_STP",
	},
	{
		.gpio = 24, 
		.direction = 1,
		.name = "USB_CLK",
	},
};

static void usb3320_hack_drive_exit_condition(void)
{
	int i;
	struct usb3320_hack_gpio_list *dir = usb3320_hack_signals + 9;
	struct usb3320_hack_gpio_list *stp = usb3320_hack_signals + 10;
	struct usb3320_hack_gpio_list *clk = usb3320_hack_signals + 11;

//	printk(KERN_INFO "dir = %s\n", dir->name);
//	printk(KERN_INFO "clk = %s\n", clk->name);
//	printk(KERN_INFO "stp = %s\n", stp->name);

	if(stp->cleaned == 0 && clk->cleaned == 0 && dir->cleaned == 0)
	{
		if(gpio_get_value(dir->gpio) == 0)
		{
//			printk(KERN_INFO "Drive exit condition - DIR is 0 already??\n");
			gpio_set_value(stp->gpio, 1);
			return;
		}
		gpio_set_value(usb3320_hack_reset_gpio, 0);
		gpio_set_value(clk->gpio, 0);
		gpio_set_value(stp->gpio, 1);
		for(i=0;i<100;++i)
		{
			if(i > 30)
			{
				gpio_set_value(usb3320_hack_reset_gpio, 1);
			}
			gpio_set_value(clk->gpio, 1);
			gpio_set_value(clk->gpio, 0);

			if(gpio_get_value(dir->gpio) == 0)
			{
				gpio_request(clk->gpio, clk->name);

//				printk(KERN_INFO "Successfully removed PHY from suspend state\n");
				gpio_set_value(stp->gpio, 0);
				if(usb3320_hack_return_call)
				{
					usb3320_hack_return_call(0, usb3320_hack_dev);
				}
				return;
			}
		}
	}
	// printk(KERN_INFO "Drive exit condition unable to cause PHY to resume\n");
}

static void usb3320_hack_check_exit_condition(void)
{
	if(usb3320_hack_signals[0].last_state == 0 && usb3320_hack_signals[1].last_state == 1)
	{
		// printk(KERN_INFO "usb_usb3320_hack detected a remote wakeup request; pulling out\n");
		usb3320_hack_drive_exit_condition();
	}
}

static irqreturn_t usb3320_hack_handler(int irq, void *device)
{
	int gpio = irq_to_gpio(irq);
	int i;

	for(i=0;i < ARRAY_SIZE(usb3320_hack_signals);++i)
	{
		struct usb3320_hack_gpio_list *d = usb3320_hack_signals + i;
	
		if(d->gpio == gpio)
		{
			d->last_state = gpio_get_value(gpio);
			// printk(KERN_INFO "GPIO IRQ - %s is now %i\n", d->name, d->last_state);
		}
	}
	
	usb3320_hack_check_exit_condition();
	
	return IRQ_HANDLED;
}

static int usb3320_hack_setup_gpio(struct usb3320_hack_gpio_list *d)
{
	int irq = -1;
	int ret = 0;
	u16 mux_opt;

	d->old_mux = omap_mux_get_gpio(d->gpio);

	mux_opt = ((d->direction == 0) ? OMAP_PIN_INPUT_PULLDOWN : OMAP_PIN_OUTPUT);

	if(usb3320_hack_wakeups_enabled)
	{
		mux_opt |= ((d->direction == 0) ? (OMAP_PIN_OFF_WAKEUPENABLE | OMAP_OFF_EN) : (OMAP_PIN_OFF_OUTPUT_LOW));
	}

	mux_opt |= OMAP_MUX_MODE4;

	omap_mux_set_gpio(mux_opt, d->gpio);

	d->cleaned = 0;
	d->wakeup = 0;

	if((ret = gpio_request(d->gpio, d->name)) < 0)
	{
		// printk(KERN_INFO "Failed to request GPIO %i\n", d->gpio);
		goto cleanup1;
	}

	if(d->direction == 1)
	{
		if(d->gpio == 25)
		{
			gpio_direction_output(d->gpio, 0);
		} else {
			gpio_direction_output(d->gpio, 0);
		}
	} else {
		gpio_direction_input(d->gpio);

		irq = gpio_to_irq(d->gpio);

		d->last_state = gpio_get_value(d->gpio);
		// printk(KERN_INFO "GPIO %s is now %i\n", d->name, d->last_state);

		if(irq < 0)
		{
			ret = irq;
			// printk(KERN_INFO "Failed to call gpio_to_irq\n");
			goto cleanup1;
		}

		if((ret = request_irq(irq, usb3320_hack_handler, IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, d->name, usb3320_hack_dev)) < 0)
		{
			// printk(KERN_INFO "Faied to request IRQ #%i\n", irq);
			goto cleanup2;
		}
		if(usb3320_hack_wakeups_enabled)
		{
			if(enable_irq_wake(irq) == 0)
			{
				d->wakeup = 1;
			}
		}

	}

	return ret;

cleanup2:
	gpio_free(d->gpio);

cleanup1:
	omap_mux_set_gpio(d->old_mux, d->gpio);
	return ret;
}

static int usb3320_hack_cleanup_gpio(struct usb3320_hack_gpio_list *d)
{
	int irq = -1;
	if(d->cleaned == 0)
	{	
		irq = gpio_to_irq(d->gpio);

		if(d->wakeup)
		{
			disable_irq_wake(irq);
			d->wakeup = 0;
		}
		if(d->direction == 0)
		{
			free_irq(irq, usb3320_hack_dev);
		}
		omap_mux_set_gpio(d->old_mux, d->gpio);
		gpio_free(d->gpio);
		d->cleaned = 1;
	}
	return 0;
}

int usb3320_hack_install(void)
{
	int i;
	int ret = 0;
	struct usb3320_hack_gpio_list *dat0 = usb3320_hack_signals + 0;
	struct usb3320_hack_gpio_list *dat1 = usb3320_hack_signals + 1;

	usb3320_hack_wakeups_enabled = device_may_wakeup(usb3320_hack_dev);

	for(i=0;i < ARRAY_SIZE(usb3320_hack_signals);++i)
	{
		if((ret = usb3320_hack_setup_gpio(usb3320_hack_signals + i)) < 0)
		{
			goto cleanup;
		}
	}

	// Note: We're using the cached values of the state upon GPIO initialization
	// Sometimes that'll clear the 0 state of the DAT0 pin in certain conditions...
	if(dat0->cleaned == 0 && dat1->cleaned == 0)
	{
		if(dat0->last_state != 1 || dat1->last_state != 0)
		{
			if(usb3320_hack_return_call)
			{
				usb3320_hack_return_call(0, usb3320_hack_dev);
			}
		}
	}


	return 0;

cleanup:
	for(--i;i>=0;--i)
	{
		usb3320_hack_cleanup_gpio(usb3320_hack_signals + i);
	}
	return ret;
}

int usb3320_hack_remove(void)
{
	int i;

	for(i=0;i < ARRAY_SIZE(usb3320_hack_signals);++i)
	{
		usb3320_hack_cleanup_gpio(usb3320_hack_signals + i);
	}
	return 0;
}

int usb3320_hack_init(irq_handler_t return_call, struct device *dev)
{
	int i;
	
	if(!(usb3320_hack_dev == NULL && usb3320_hack_return_call == NULL))
	{
	    return -ENODEV;
	}
	
	usb3320_hack_dev = dev;
	usb3320_hack_return_call = return_call;
	
	for(i=0;i < ARRAY_SIZE(usb3320_hack_signals);++i)
	{
		usb3320_hack_signals[i].cleaned = 1;
		usb3320_hack_signals[i].wakeup = 0;
	}
	
	return 0;
}

int usb3320_hack_deinit(void)
{
	usb3320_hack_remove();
	usb3320_hack_dev = NULL;
	usb3320_hack_return_call = NULL;
	
	return 0;
}

int usb3320_hack_set_wakeup(int wakeup)
{
	int i;
	usb3320_hack_wakeups_enabled = wakeup;

	for(i=0;i < ARRAY_SIZE(usb3320_hack_signals);++i)
	{
		struct usb3320_hack_gpio_list *d = usb3320_hack_signals + i;

		if(d->cleaned == 0)
		{
			u16 mux_opt;
			int irq = gpio_to_irq(d->gpio);

			mux_opt = ((d->direction == 0) ? OMAP_PIN_INPUT_PULLDOWN : OMAP_PIN_OUTPUT);

			if(usb3320_hack_wakeups_enabled)
			{
				mux_opt |= ((d->direction == 0) ? (OMAP_PIN_OFF_WAKEUPENABLE | OMAP_OFF_EN) : (OMAP_PIN_OFF_OUTPUT_LOW));
			}

			mux_opt |= OMAP_MUX_MODE4;

			omap_mux_set_gpio(mux_opt, d->gpio);

			if(d->direction == 0)
			{
				if(d->wakeup == 0)
				{
					if(usb3320_hack_wakeups_enabled)
					{
						if(enable_irq_wake(irq) == 0)
						{
							d->wakeup = 1;
						}
					}
				} else {
					disable_irq_wake(irq);
					d->wakeup = 0;
				}
			}
		}
	}
	return 0;
}
