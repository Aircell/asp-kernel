/*
 * linux/arch/arm/mach-omap2/board-cloud-camera.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mm.h>

#include <linux/regulator/consumer.h>

#include <asm/io.h>

#include <mach/gpio.h>
#ifdef CONFIG_OMAP_PM_SRF
#include <mach/omap-pm.h>
#endif

static int cam_inited;

static struct device *cloudcam_dev;

#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#define DEBUG_BASE		0x08000000

#define REG_SDP3430_FPGA_GPIO_2 (0x50)
#define FPGA_SPR_GPIO1_3v3	(0x1 << 14)
#define FPGA_GPIO6_DIR_CTRL	(0x1 << 6)

#define CAMZOOM2_USE_XCLKBA  	0

/* Sensor specific GPIO signals */
#define OV7692_PWDN_GPIO  	95

static struct regulator *cloud_ov7692_reg1;
static struct regulator *cloud_ov7692_reg2;

#if defined(CONFIG_VIDEO_OV7692) || defined(CONFIG_VIDEO_OV7692_MODULE)
#include <media/ov7692.h>
#include <../drivers/media/video/isp/ispcsi2.h>
#define OV7692_CSI2_CLOCK_POLARITY	0	/* +/- pin order */
#define OV7692_CSI2_DATA0_POLARITY	0	/* +/- pin order */
#define OV7692_CSI2_DATA1_POLARITY	0	/* +/- pin order */
#define OV7692_CSI2_CLOCK_LANE		1	 /* Clock lane position: 1 */
#define OV7692_CSI2_DATA0_LANE		2	 /* Data0 lane position: 2 */
#define OV7692_CSI2_PHY_THS_TERM	2
#define OV7692_CSI2_PHY_THS_SETTLE	23
#define OV7692_CSI2_PHY_TCLK_TERM	0
#define OV7692_CSI2_PHY_TCLK_MISS	1
#define OV7692_CSI2_PHY_TCLK_SETTLE	14
#define OV7692_BIGGEST_FRAME_BYTE_SIZE	PAGE_ALIGN(640 * 480 * 2)
#endif



#if defined(CONFIG_VIDEO_OV7692) || defined(CONFIG_VIDEO_OV7692_MODULE)

static struct omap34xxcam_sensor_config ov7692_hwc = {
	.sensor_isp  = 0,
	.capture_mem = OV7692_BIGGEST_FRAME_BYTE_SIZE * 32,
	.ival_default	= { 1, 10 },
};

static int ov7692_sensor_set_prv_data(struct v4l2_int_device *s, void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.sensor_isp = ov7692_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = ov7692_hwc.capture_mem;
	hwc->dev_index		= 2;
	hwc->dev_minor		= 0;
	hwc->dev_type		= OMAP34XXCAM_SLAVE_SENSOR;

	return 0;
}

static struct isp_interface_config ov7692_if_config = {
	.ccdc_par_ser 		= ISP_CSIA,   /* Type of interface , CSI1, CSI2,..) */
	.dataline_shift 	= 0x0,	      /* Bitshift reduction enroute to CCDC */
	.hsvs_syncdetect 	= ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe 		= 0x0,
	.prestrobe 		= 0x0,
	.shutter 		= 0x0,
	.wenlog 		= ISPCCDC_CFG_WENLOG_AND,
	.wait_hs_vs		= 0,
	.u.csi.crc 		= 0x0,
	.u.csi.mode 		= 0x0, /* ? Probably CSI1 */
	.u.csi.edge 		= 0x0, /* ? Probably CSI1 */
	.u.csi.signalling 	= 0x0, /* CSI1 mode only */
	.u.csi.strobe_clock_inv = 0x0, /* CSI1 mode only */
	.u.csi.vs_edge 		= 0x0,
	.u.csi.channel 		= 0x0,
	.u.csi.vpclk 		= 1, //@vp_out_ctrl: Divider value for setting videoport clock frequency based on
 				       // OCP port frequency, valid dividers are between 1 and 4.
	.u.csi.data_start 	= 0x0, 
	.u.csi.data_size 	= 0x0, // CSI1 
	//.u.csi.format 		= V4L2_PIX_FMT_SGRBG10,
	.u.csi.format 		= V4L2_PIX_FMT_YUYV,
};


static int ov7692_sensor_power_set(struct v4l2_int_device *s, enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_csi2_lanes_cfg lanecfg;
	struct isp_csi2_phy_cfg phyconfig;
	static enum v4l2_power previous_power = V4L2_POWER_OFF;
	int err = 0;

#if 0
	if (!cam_inited) {
		printk(KERN_ERR "MT9P012: Unable to control board GPIOs!\n");
		return -EFAULT;
	}
#endif

	/*
	 * Plug regulator consumer to respective VAUX supply
	 * if not done before.
	 */

#ifdef CONFIG_CLOUD_REGULATOR
	cloudcam_dev = vdev->cam->isp;

	if (!cloud_ov7692_reg1 && !cloud_ov7692_reg2) {
		cloud_ov7692_reg1 = regulator_get(cloudcam_dev, "vaux2_1");
		if (IS_ERR(cloud_ov7692_reg1)) {
			dev_err(cloudcam_dev, "vaux2_1 regulator missing\n");
			return PTR_ERR(cloud_ov7692_reg1);
		}
		cloud_ov7692_reg2 = regulator_get(cloudcam_dev, "vaux4_1");
		if (IS_ERR(cloud_ov7692_reg2)) {
			dev_err(cloudcam_dev, "vaux4_1 regulator missing\n");
			regulator_put(cloud_ov7692_reg1);
			return PTR_ERR(cloud_ov7692_reg2);
		}
	}
#endif

	switch (power) {
	case V4L2_POWER_ON:
		/* Power Up Sequence */
		printk(KERN_DEBUG "ov7692_sensor_power_set(ON)\n");

		/* Through-put requirement:
		 * 3280 x 2464 x 2Bpp x 7.5fps x 3 memory ops = 355163 KByte/s
		 */
#ifdef CONFIG_OMAP_PM_SRF
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 355163);
#endif

		/* Turn off camera during CSI2 Reset */
		gpio_set_value(OV7692_PWDN_GPIO, 1);
		err = isp_csi2_reset();
		if(err)
			goto err_out;

		lanecfg.clk.pol = OV7692_CSI2_CLOCK_POLARITY;
		lanecfg.clk.pos = OV7692_CSI2_CLOCK_LANE;
		lanecfg.data[0].pol = OV7692_CSI2_DATA0_POLARITY;
		lanecfg.data[0].pos = OV7692_CSI2_DATA0_LANE;
		lanecfg.data[1].pol = 0;
		lanecfg.data[1].pos = 0;
		lanecfg.data[2].pol = 0;
		lanecfg.data[2].pos = 0;
		lanecfg.data[3].pol = 0;
		lanecfg.data[3].pos = 0;
		isp_csi2_complexio_lanes_config(&lanecfg);
		isp_csi2_complexio_lanes_update(true);

		isp_csi2_ctrl_config_ecc_enable(true);

		phyconfig.ths_term = OV7692_CSI2_PHY_THS_TERM;
		phyconfig.ths_settle = OV7692_CSI2_PHY_THS_SETTLE;
		phyconfig.tclk_term = OV7692_CSI2_PHY_TCLK_TERM;
		phyconfig.tclk_miss = OV7692_CSI2_PHY_TCLK_MISS;
		phyconfig.tclk_settle = OV7692_CSI2_PHY_TCLK_SETTLE;
		isp_csi2_phy_config(&phyconfig);
		isp_csi2_phy_update(true);
		isp_csi2_complexio_init();
		
		isp_configure_interface(vdev->cam->isp, &ov7692_if_config);

		/* Start the camera */
		gpio_set_value(OV7692_PWDN_GPIO, 0);

		if (previous_power == V4L2_POWER_OFF) {
			/* POWER is active LOW. set HIGH to release power */
			gpio_set_value(OV7692_PWDN_GPIO, 0);

#ifdef CONFIG_CLOUD_REGULATOR
			/* turn on analog power */
			regulator_enable(cloud_ov7692_reg1);
			regulator_enable(cloud_ov7692_reg2);
			udelay(100);

#endif
		}
		break;
	case V4L2_POWER_OFF:
		printk(KERN_DEBUG "ov7692_sensor_power_set(OFF)\n");
		/* POWER is active LOW. set HIGH to release power */
		gpio_set_value(OV7692_PWDN_GPIO, 1);

		/* Power Down Sequence */
		isp_csi2_complexio_power(ISP_CSI2_POWER_OFF);

#ifdef CONFIG_CLOUD_REGULATOR
		if (regulator_is_enabled(cloud_ov7692_reg1))
			regulator_disable(cloud_ov7692_reg1);
		if (regulator_is_enabled(cloud_ov7692_reg2))
			regulator_disable(cloud_ov7692_reg2);
#endif

#ifdef CONFIG_OMAP_PM_SRF
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 0);
#endif
		break;
	case V4L2_POWER_STANDBY:
		printk(KERN_DEBUG "ov7692_sensor_power_set(STANDBY)\n");
		isp_csi2_complexio_power(ISP_CSI2_POWER_OFF);
		/*TODO*/
#ifdef CONFIG_OMAP_PM_SRF
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 0);
#endif
		break;
	}

	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;

err_out:
	return err;
}

static u32 ov7692_sensor_set_xclk(struct v4l2_int_device *s, u32 xclkfreq)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

	return isp_set_xclk(vdev->cam->isp, xclkfreq, CAMZOOM2_USE_XCLKBA);
}

struct ov7692_platform_data cloud_ov7692_platform_data = {
	.power_set            = ov7692_sensor_power_set,
	.priv_data_set        = ov7692_sensor_set_prv_data,
	.set_xclk             = ov7692_sensor_set_xclk,
	.csi2_lane_count      = isp_csi2_complexio_lanes_count,
	.csi2_cfg_vp_out_ctrl = isp_csi2_ctrl_config_vp_out_ctrl,
	.csi2_ctrl_update     = isp_csi2_ctrl_update,
	.csi2_cfg_virtual_id  = isp_csi2_ctx_config_virtual_id,
	.csi2_ctx_update      = isp_csi2_ctx_update,
	.csi2_calc_phy_cfg0   = isp_csi2_calc_phy_cfg0,
};
#endif

void cloud_cam_init(void)
{

	printk(KERN_INFO "cloud_cam_init() called\n");

}

static int cloud_cam_remove(struct platform_device *pdev)
{
#ifdef CONFIG_CLOUD_REGULATOR
	if (regulator_is_enabled(cloud_ov7692_reg1))
		regulator_disable(cloud_ov7692_reg1);
	regulator_put(cloud_ov7692_reg1);
	if (regulator_is_enabled(cloud_ov7692_reg2))
		regulator_disable(cloud_ov7692_reg2);
	regulator_put(cloud_ov7692_reg2);
#endif

	return 0;
}



