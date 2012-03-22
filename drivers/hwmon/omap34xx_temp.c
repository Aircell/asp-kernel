/*
 * omap34xx_temp.c - Linux kernel module for OMAP34xx hardware monitoring
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Written by Peter De Schrijver <[EMAIL PROTECTED]>
 *
 * Inspired by k8temp.c
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

#include <linux/clk.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <plat/control.h>

struct omap34xx_data {
	struct device *hwmon_dev;
	struct clk *clk_32k;
	const char *name;
	char valid;
	unsigned long last_updated;
	u32 temp;
	u32 bit_CONTCONV;	/* CONTCONF bit in CONTROL_TEMP_SENSOR */
	u32 bit_SOC;		/* SOC bit in CONTROL_TEMP_SENSOR */
	u32 bit_EOCZ;		/* EOCZ bit in CONTROL_TEMP_SENSOR */
	short *adc_table;	/* ADC table used */
	u32 adc_bitmask;	/* bitmask of valid bits in ADC table */
};

static struct platform_device omap34xx_temp_device = {
	.name   = "temp_sensor",
	.id     = -1,
};

static int __devinit omap34xx_temp_probe(struct platform_device *pdev);
static int __devexit omap34xx_temp_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM
static int omap34xx_temp_suspend(struct device *dev)
{
	struct omap34xx_data *data = dev_get_drvdata(dev);
	clk_disable(data->clk_32k);
	return 0;
}
static int omap34xx_temp_resume(struct device *dev)
{
	struct omap34xx_data *data = dev_get_drvdata(dev);
	clk_enable(data->clk_32k);
	return 0;
}

static const struct dev_pm_ops omap34xx_temp_pm_ops = {
	.suspend	= omap34xx_temp_suspend,
	.resume		= omap34xx_temp_resume,
};

#define OMAP34XX_TEMP_PM_OPS (&omap34xx_temp_pm_ops)
#else
#define OMAP34XX_TEMP_PM_OPS NULL
#endif

static struct platform_driver omap34xx_temp_driver = {
	.probe	= omap34xx_temp_probe,
	.remove	= __devexit_p(omap34xx_temp_remove),
	.driver = {
		.name	= "temp_sensor",
		.owner	= THIS_MODULE,
		.pm	= OMAP34XX_TEMP_PM_OPS,
	},
};

/* ADC conversion table for OMAP3530/AM3530, multiplied by 10 */
static short adc_34xx_to_temp[129] = {
	-400, -400, -400, -400,	-400, -400, -389, -375, -361, -333,
	-318, -304, -290, -275, -261, -247, -233, -219, -205, -191,
	-177, -163, -149, -134, -120, -106, -92, -78, -64, -50,
	-35, -21, -7, +8, +23, +37, +51, +66, +80, +94,
	+108, +123, +137, +151, +165, +179, +194, +208, +222, +236,
	+251, +265, +279, +293, +307, +321, +335, +349, +364, +378,
	+392, +406, +420, +424, +449, +463, +447, +491, +505, +519,
	+533, +546, +560, +574, +588, +602, +616, +630, +644, +657,
	+671, +685, +699, +713, +727, +741, +755, +769, +783, +797,
	+811, +823, +838, +852, +865, +879, +893, +906, +920, +934,
	+947, +961, +975, +989, +1002, +1016, +1030, +1043, +1057, +1071,
	+1085, +1098, +1112, +1126, +1140, +1153, +1167, +1181, +1194, +1208,
	+1222, +1235, +1249, +1250, +1250, +1250, +1250, +1250, +1250,
};

/* ADC conversion table for OMAP3730/AM3730, multiplied by 10 */
static short adc_37xx_to_temp[129] = {
	-400, -400, -400, -400, -400, -400, -400, -400, -400, -400,
	-400, -400, -400, -400, -380, -350, -340, -320, -300, -280,
	-260, -240, -220, -200, -185, -170, -150, -135, -120, -100,
	-80, -65, -50, -35, -15, +0, +20, +35, +50, +65,
	85, +100, +120, +135, +150, +170, +190, +210, +230, +250,
	270, +285, +300, +320, +335, +350, +370, +385, +400, +420,
	435, +450, +470, +485, +500, +520, +535, +550, +570, +585,
	600, +620, +640, +660, +680, +700, +715, +735, +750, +770,
	785, +800, +820, +835, +850, +870, +885, +900, +920, +935,
	950, +970, +985, +1000, +1020, +1035, +1050, +1070, +1090, +1110,
	1130, +1150, +1170, +1185, +1200, +1220, +1235, +1250, +1250, +1250,
	1250, +1250, +1250, +1250, +1250, +1250, +1250, +1250, +1250, +1250,
	1250, +1250, +1250, +1250, +1250, +1250, +1250, +1250, +1250
};


static void omap34xx_update(struct omap34xx_data *data)
{
	data->temp = omap_ctrl_readl(OMAP343X_CONTROL_TEMP_SENSOR);
	data->temp &= data->adc_bitmask;
}

static ssize_t show_name(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct omap34xx_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", data->name);
}

static ssize_t show_temp_raw(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct omap34xx_data *data = dev_get_drvdata(dev);

	omap34xx_update(data);

	return sprintf(buf, "%d\n", data->temp);
}

static ssize_t show_temp(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct omap34xx_data *data = dev_get_drvdata(dev);
	u32 temp_val;
     
	omap34xx_update(data);

	/* Average the temp value in table (table has 1 extra entry) */
	temp_val = data->adc_table[data->temp];
	temp_val += data->adc_table[data->temp+1];
	temp_val /= 2;

	return sprintf(buf, "%d.%1d\n", temp_val/10, temp_val%10);
}

static SENSOR_DEVICE_ATTR_2(celsius, S_IRUGO, show_temp, NULL, 0, 0);
static SENSOR_DEVICE_ATTR_2(raw, S_IRUGO, show_temp_raw,
			NULL, 0, 0);
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

static int __devinit omap34xx_temp_probe(struct platform_device *pdev)
{
	int err;
	struct omap34xx_data *data;

	data = kzalloc(sizeof(struct omap34xx_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit_platform;
	}

	dev_set_drvdata(&omap34xx_temp_device.dev, data);
	data->name = "temp_sensor";

	data->clk_32k = clk_get(&omap34xx_temp_device.dev, "ts_fck");
	if (IS_ERR(data->clk_32k)) {
		err = PTR_ERR(data->clk_32k);
		printk(KERN_ERR "Failed to get ts_fck; returned %d\n", err);
		goto exit_free;
	}
	err = clk_enable(data->clk_32k);
	if (err) {
		printk(KERN_ERR "clk_enable(ts_clk) failed with %d\n", err);
		goto clock_free;
	}

	err = device_create_file(&omap34xx_temp_device.dev,
				&sensor_dev_attr_celsius.dev_attr);
	if (err)
		goto clock_disable;

	err = device_create_file(&omap34xx_temp_device.dev,
				&sensor_dev_attr_raw.dev_attr);
	if (err)
		goto exit_remove;

	err = device_create_file(&omap34xx_temp_device.dev, &dev_attr_name);
	if (err)
		goto exit_remove_raw;

	data->hwmon_dev = hwmon_device_register(&omap34xx_temp_device.dev);

	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove_all;
	}

	if (cpu_is_omap3430()) {
		/* OMAP34xx/OMAP35xx */
		data->bit_CONTCONV = BIT(9);
		data->bit_SOC = BIT(8);
		data->bit_EOCZ = BIT(7);
		data->adc_table = adc_34xx_to_temp;
	} else if (cpu_is_omap3630()) {
		/* OMAP36xx/OMAp37xx */
		data->bit_CONTCONV = BIT(10);
		data->bit_SOC = BIT(9);
		data->bit_EOCZ = BIT(8);
		data->adc_table = adc_37xx_to_temp;
	}
	data->adc_bitmask = 0x7f;
	omap_ctrl_writel(data->bit_CONTCONV|data->bit_SOC, OMAP343X_CONTROL_TEMP_SENSOR);

	return 0;

exit_remove_all:
	device_remove_file(&omap34xx_temp_device.dev,
                        &dev_attr_name);
exit_remove_raw:
	device_remove_file(&omap34xx_temp_device.dev,
                        &sensor_dev_attr_raw.dev_attr);
exit_remove:
	device_remove_file(&omap34xx_temp_device.dev,
                        &sensor_dev_attr_celsius.dev_attr);
clock_disable:
	clk_disable(data->clk_32k);

clock_free:
	clk_put(data->clk_32k);

exit_free:
	kfree(data);
exit_platform:
	/* platform_device_unregister(&omap34xx_temp_device); */
/* exit: */
	return err;
}

static int __init omap34xx_temp_init(void)
{
	int err;

	err = platform_driver_register(&omap34xx_temp_driver);
	if (err)
		return err;
	return platform_device_register(&omap34xx_temp_device);
}

static void __exit omap34xx_temp_exit(void)
{
	struct omap34xx_data *data =
		dev_get_drvdata(&omap34xx_temp_device.dev);

	clk_disable(data->clk_32k);
	clk_put(data->clk_32k);
	hwmon_device_unregister(data->hwmon_dev);
	device_remove_file(&omap34xx_temp_device.dev,
                        &sensor_dev_attr_celsius.dev_attr);
	device_remove_file(&omap34xx_temp_device.dev,
                        &sensor_dev_attr_raw.dev_attr);
	device_remove_file(&omap34xx_temp_device.dev, &dev_attr_name);
	kfree(data);
	/* platform_device_unregister(&omap34xx_temp_device); */
}

MODULE_AUTHOR("Peter De Schrijver");
MODULE_DESCRIPTION("Omap34xx/37xx temperature sensor");
MODULE_LICENSE("GPL");

module_init(omap34xx_temp_init)
module_exit(omap34xx_temp_exit)
