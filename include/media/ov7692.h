/*
 * ov7692.h - Shared settings for the OV7692 CameraChip.
 *
 * Contributors:
 *   Dominic Curran <dcurran@ti.com>
 *
 * Copyright (C) 2008 Hewlett Packard.
 * Copyright (C) 2009 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef OV7692_H
#define OV7692_H

#define OV7692_I2C_ADDR		0x3c

/* Average black level */
#define OV7692_BLACK_LEVEL_AVG	64

/**
 * struct ov7692_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @default_regs: Default registers written after power-on or reset.
 * @ifparm: Interface parameters access function
 * @priv_data_set: device private data (pointer) access function
 */
struct ov7692_platform_data {
	int (*power_set)(struct v4l2_int_device *s, enum v4l2_power power);
	int (*ifparm)(struct v4l2_ifparm *p);
	int (*priv_data_set)(struct v4l2_int_device *s, void *);
	u32 (*set_xclk)(struct v4l2_int_device *s, u32 xclkfreq);
	int (*cfg_interface_bridge)(u32);
	int (*csi2_lane_count)(int count);
	int (*csi2_cfg_vp_out_ctrl)(u8 vp_out_ctrl);
	int (*csi2_ctrl_update)(bool);
	int (*csi2_cfg_virtual_id)(u8 ctx, u8 id);
	int (*csi2_ctx_update)(u8 ctx, bool);
	int (*csi2_calc_phy_cfg0)(u32, u32, u32);
};

#endif /* ifndef OV7692_H */
