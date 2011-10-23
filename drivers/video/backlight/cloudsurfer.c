/*
 * Aircell - Copyright 2011 
 * 
 * The backlight controller for the Aircell CouldSurfer
 */

#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>


struct cloudsurfer {
	int base;
};

static void backlight_high(void)
{
	/* Set the backlight PWM to full on */
		
}
static int cloudsurfer_backlight_update_status(struct backlight_device *bldev)
{
	return 0;
}

static int cloudsurfer_backlight_get_brightness(struct backlight_device *bldev)
{
	return 30;
}

static int cloudsurfer_backlight_check_fb(struct fb_info *fb)
{
	return 1;
}

static struct backlight_ops cloudsurfer_backlight_ops = {
	.options = 0,
	.update_status = cloudsurfer_backlight_update_status,
	.get_brightness = cloudsurfer_backlight_get_brightness,
	.check_fb = cloudsurfer_backlight_check_fb,
};

static int __devinit cloudsurfer_backlight_probe(struct platform_device *pdev)
{
	struct backlight_device *bldev;
	struct resource *res;
	struct cloudsurfer *cl;
	int ret = 0;
	u8 val;

	printk("BACKLIGHT PROBE\n");

	cl = devm_kzalloc(&pdev->dev, sizeof(*cl), GFP_KERNEL);
	if (!cl) {
		printk("BACKLIGHT - Failed kzalloc\n");
		ret = -ENOMEM;
		goto out;
	}

	printk("BACKLIGHT - Start PWM\n");
	/* The backlight uses PMW0 from the TSP65950 chip */
	/* The PWM to maximum */
	if ( twl_i2c_write_u8(TWL4030_MODULE_PWM0,0x02,0x00) < 0 ) 
		printk("BACKLIGHT - PWM i2c write 1 failed\n");
	if ( twl_i2c_write_u8(TWL4030_MODULE_PWM0,0x78,0x01) < 0 )
		printk("BACKLIGHT - PWM i2c write 2 failed\n");
	/* Connect the PWM0 to the BGA pin */
	twl_i2c_read_u8(TWL4030_MODULE_INTBR,&val,0x0d);
	val |= 0x04;
	if ( twl_i2c_write_u8(TWL4030_MODULE_INTBR,val,0x0d) < 0 ) 
		printk("BACKLIGHT - PWM i2c write 3 failed\n");
	/* Enable the PWM0 */
	twl_i2c_read_u8(TWL4030_MODULE_INTBR,&val,0x0c);
	val |= 0x05;
	if ( twl_i2c_write_u8(TWL4030_MODULE_INTBR,val,0x0c) < 0 ) 
		printk("BACKLIGHT - PWM i2c write 4 failed\n");

	
	bldev = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, cl,
			&cloudsurfer_backlight_ops);
	if (!bldev) {
		ret = -ENOMEM;
		goto out;
	}

	
	bldev->props.max_brightness = 0xff;
	bldev->props.brightness = 0xff;
	bldev->props.power = FB_BLANK_UNBLANK;

	platform_set_drvdata(pdev, bldev);

out:
	return ret;
}

static int __devexit cloudsurfer_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bldev;
	int ret = 0;

	bldev = platform_get_drvdata(pdev);
	bldev->props.power = FB_BLANK_UNBLANK;
	bldev->props.brightness = 0xff;
	backlight_update_status(bldev);
	backlight_device_unregister(bldev);
	platform_set_drvdata(pdev, NULL);

	return ret;
}

#ifdef CONFIG_PM
static int cloudsurfer_backlight_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}

static int cloudsurfer_backlight_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define cloudsurfer_backlight_suspend NULL
#define cloudsurfer_backlight_resume NULL
#endif

static struct platform_driver cloudsurfer_backlight_driver = {
	.probe = cloudsurfer_backlight_probe,
	.remove = __devexit_p(cloudsurfer_backlight_remove),
	.suspend = cloudsurfer_backlight_suspend,
	.resume = cloudsurfer_backlight_resume,
	.driver = {
		.name = "cloudsurfer-backlight",
		.owner = THIS_MODULE,
	},
};

static int __init cloudsurfer_backlight_init(void)
{
	printk("BACKLIGHT INIT\n");
	return platform_driver_register(&cloudsurfer_backlight_driver);
}

static void __exit cloudsurfer_backlight_exit(void)
{
	platform_driver_unregister(&cloudsurfer_backlight_driver);
}

module_init(cloudsurfer_backlight_init);
module_exit(cloudsurfer_backlight_exit);

MODULE_AUTHOR("Steven Tarr <tarr@bouldersystemsdesign.com>");
MODULE_DESCRIPTION("AirCell CloudSurfer Backlight Driver");
MODULE_LICENSE("GPL v2");
