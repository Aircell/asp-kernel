/**
 * cloudsurfer-wakeup.c - Wakeup timer for cloudsurfer
 *
 * Copyright (C) 2012 Gogo/Aircell
 *
 * Written by Steve Tarr <tarr@boldersystemsdesign.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <plat/dmtimer.h>

static struct platform_device *cloudsurfer_wakeup_device;
static unsigned int wakeup_seconds = 30;
extern unsigned long clk_get_rate(struct clk *clk);

struct cloudsurfer_wakeup_struct {
	struct omap_dm_timer *timer;
	unsigned int irq;
};

static irqreturn_t cloudsurfer_wakeup_handler(int irq, void *_wkup_in)
{
	struct input_dev *wkup_in = _wkup_in;
	struct cloudsurfer_wakeup_struct *wkup = input_get_drvdata(wkup_in);
	omap_dm_timer_write_status(wkup->timer, OMAP_TIMER_INT_OVERFLOW);
	input_event(wkup_in,EV_MSC,1,0);
	input_sync(wkup_in);
	return IRQ_HANDLED;
}

static struct irqaction cloudsurfer_irq_data = {
    .name       = "cloudsurfer-wakeup",
    .flags      = IRQF_DISABLED | IRQF_TIMER ,
    .handler    = cloudsurfer_wakeup_handler,
};

static int __devinit cloudsurfer_wakeup_probe(struct platform_device *pdev)
{
	struct cloudsurfer_wakeup_struct *wkup;
	struct input_dev *wkup_in;
	u32 tick_rate, cycles;
	int err;

	/* Data structure allocation */
	wkup = kzalloc(sizeof(struct cloudsurfer_wakeup_struct), GFP_KERNEL);
	wkup_in = input_allocate_device();
	if (!wkup_in) {
		printk("CLOUDSURFER - Can't allocate wakeup timer input\n");
		err = -ENOMEM;
		goto error_input_dev;
	}

	/* Timer allocation and setup */
	if  ( (wkup->timer=omap_dm_timer_request_specific(1)) == NULL ) {
        printk("CLOUDSURFER - can't allocate wakeup timer\n");
        err = -ENODEV;
		goto error_timer;
    }
	wkup->irq = omap_dm_timer_get_irq(wkup->timer);
	omap_dm_timer_set_source(wkup->timer,OMAP_TIMER_SRC_32_KHZ);
	tick_rate = clk_get_rate(omap_dm_timer_get_fclk(wkup->timer));
	cycles = tick_rate * wakeup_seconds;
	omap_dm_timer_stop(wkup->timer);

	input_set_capability(wkup_in,EV_MSC,1);
	input_set_capability(wkup_in,EV_MSC,2);

	wkup_in->name = "cloudsurfer-wakeup";
	wkup_in->phys = "cloudsurfer-wakeup/input0";
	wkup_in->dev.parent = &pdev->dev;

	cloudsurfer_irq_data.dev_id = wkup_in;
	setup_irq(wkup->irq,&cloudsurfer_irq_data);
	input_set_drvdata(wkup_in, wkup);

	err = input_register_device(wkup_in);
	if (err) {
		printk("CLOUDSURFER -Can't register wakeup timer: %d\n", err);
		goto error_register;
	}
	
	omap_dm_timer_set_int_enable(wkup->timer,OMAP_TIMER_INT_OVERFLOW);
    omap_dm_timer_set_load_start(wkup->timer, 1, 0xffffffff - cycles);

	platform_set_drvdata(pdev, wkup_in);

	return 0;

error_register:
	free_irq(wkup->irq, NULL);
error_timer:
	input_free_device(wkup_in);
error_input_dev:
	kfree(wkup);
	return err;
}

static int __devexit cloudsurfer_wakeup_remove(struct platform_device *pdev)
{
	struct input_dev *wkup_in = platform_get_drvdata(pdev);
	struct cloudsurfer_wakeup_struct *wkup = input_get_drvdata(wkup_in);
	int irq = omap_dm_timer_get_irq(wkup->timer);

	free_irq(irq, wkup);
	kfree(wkup);
	input_unregister_device(wkup_in);
	return 0;
}

struct platform_driver cloudsurfer_wakeup_driver = {
	.probe		= cloudsurfer_wakeup_probe,
	.remove		= __devexit_p(cloudsurfer_wakeup_remove),
	.driver		= {
		.name	= "cloudsurfer-wakeup",
		.owner	= THIS_MODULE,
	},
};

static int __init cloudsurfer_wakeup_init(void)
{
	if ( platform_driver_register(&cloudsurfer_wakeup_driver) ) {
		printk("CLOUDSURFER - failed to register wakeup driver\n");
		return -ENOMEM;
	}
	cloudsurfer_wakeup_device = platform_device_alloc("cloudsurfer-wakeup", -1);
	if ( cloudsurfer_wakeup_device == NULL ) {
		printk("CLOUDSURFER - failed to allocate wakeup device\n");
		platform_driver_unregister(&cloudsurfer_wakeup_driver);
		return -ENOMEM;
	}
	if ( platform_device_add(cloudsurfer_wakeup_device) ) {
		printk("CLOUDSURFER - failed to add wakeup device\n");
		platform_device_put(cloudsurfer_wakeup_device);		
		platform_driver_unregister(&cloudsurfer_wakeup_driver);
		return -ENOMEM;
	}
	return 0;
}
module_init(cloudsurfer_wakeup_init);

static void __exit cloudsurfer_wakeup_exit(void)
{
	platform_driver_unregister(&cloudsurfer_wakeup_driver);
}
module_exit(cloudsurfer_wakeup_exit);

MODULE_DESCRIPTION("Cloudsurfer Wakeup Timer");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Steve Tarrr <tarr@bouldersystemsdesign.com>");

