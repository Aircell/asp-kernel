/*
 * drivers/media/video/ov7692.c
 *
 * Sony ov7692 sensor driver
 *
 *
 * Copyright (C) 2008 Hewlett Packard
 *
 * Leverage mt9p012.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/i2c.h>
#include <linux/delay.h>

#include <media/v4l2-int-device.h>
#include <media/ov7692.h>

#include "ov7692_regs.h"
#include "omap34xxcam.h"
#include "isp/isp.h"
#include "isp/ispcsi2.h"


static int test=0;
module_param(test, int, S_IRUGO);

#define OV7692_DRIVER_NAME  "ov7692"
#define OV7692_MOD_NAME "OV7692: "

#define PEL(x) printk(KERN_INFO "%s at %s line %d\n", #x, __FILE__, __LINE__)
//#define PEL(x) do{}while(0) 

#define I2C_M_WR 0

/*============================================================================
							DATA DECLARATIONS
============================================================================*/
/*  96MHz PCLK @ 24MHz MCLK */
const static struct ov7692_reg initial_list[] = {
    {0x12, 0x00}, // Set  YUV
    {0x3e, 0x30}, // MIPI clock selection in YUV 
    {0xff, 0x01}, // MIPI Bank select
    {0xb4, 0xc0}, // Enable MIPI, 8-bit
  {0x80, 0x14}, // Send short pkts, Free running clock, LP11 idle data 
    {0xff, 0x00}, // MIPI Bank deselect
    {0x61, 0x70}, // Test pattern output, 8 bit pattern 2
    {0xFF, 0xFF}, // END
//    {0x12, 0x80},
    {0x0e, 0x08},
    {0x69, 0x52},
    {0x1e, 0xb3},
    {0x48, 0x42},
    {0xff, 0x01},
    {0xae, 0xa0},
    {0xa8, 0x26},
    {0xb4, 0xc0},
    {0xb5, 0x40},
    {0xff, 0x00},
    {0x0c, 0x00},
    {0x62, 0x10},
    {0x12, 0x00},
    {0x17, 0x65},
    {0x18, 0xa4},
    {0x19, 0x0a},
    {0x1a, 0xf6},
    {0x3e, 0x30},
    {0x64, 0x0a},
    {0xff, 0x01},
    {0xb4, 0xc0},
    {0xff, 0x00},
    {0x67, 0x20},
    {0x81, 0x3f},
    {0xcc, 0x02},
    {0xcd, 0x80},
    {0xce, 0x01},
    {0xcf, 0xe0},
    {0xc8, 0x02},
    {0xc9, 0x80},
    {0xca, 0x01},
    {0xcb, 0xe0},
    {0xd0, 0x48},
    {0x82, 0x03},
    {0x0e, 0x00},
    {0x70, 0x00},
    {0x71, 0x34},
    {0x74, 0x28},
    {0x75, 0x98},
    {0x76, 0x00},
    {0x77, 0x64},
    {0x78, 0x01},
    {0x79, 0xc2},
    {0x7a, 0x4e},
    {0x7b, 0x1f},
    {0x7c, 0x00},
    {0x11, 0x00},
    {0x20, 0x00},
    {0x21, 0x23},
    {0x50, 0x9a},
    {0x51, 0x80},
    {0x4c, 0x7d},
    {0x0e, 0x00},
    {0x80, 0x7f},
    {0x85, 0x10},
    {0x86, 0x00},
    {0x87, 0x00},
    {0x88, 0x00},
    {0x89, 0x2a},
    {0x8a, 0x26},
    {0x8b, 0x22},
    {0xbb, 0x7a},
    {0xbc, 0x69},
    {0xbd, 0x11},
    {0xbe, 0x13},
    {0xbf, 0x81},
    {0xc0, 0x96},
    {0xc1, 0x1e},
    {0xb7, 0x05},
    {0xb8, 0x09},
    {0xb9, 0x00},
    {0xba, 0x18},
    {0x5a, 0x1f},
    {0x5b, 0x9f},
    {0x5c, 0x6a},
    {0x5d, 0x42},
    {0x24, 0x78},
    {0x25, 0x68},
    {0x26, 0xb3},
    {0xa3, 0x0b},
    {0xa4, 0x15},
    {0xa5, 0x2a},
    {0xa6, 0x51},
    {0xa7, 0x63},
    {0xa8, 0x74},
    {0xa9, 0x83},
    {0xaa, 0x91},
    {0xab, 0x9e},
    {0xac, 0xaa},
    {0xad, 0xbe},
    {0xae, 0xce},
    {0xaf, 0xe5},
    {0xb0, 0xf3},
    {0xb1, 0xfb},
    {0xb2, 0x06},
    {0x8c, 0x5c},
    {0x8d, 0x11},
    {0x8e, 0x12},
    {0x8f, 0x19},
    {0x90, 0x50},
    {0x91, 0x20},
    {0x92, 0x96},
    {0x93, 0x80},
    {0x94, 0x13},
    {0x95, 0x1b},
    {0x96, 0xff},
    {0x97, 0x00},
    {0x98, 0x3d},
    {0x99, 0x36},
    {0x9a, 0x51},
    {0x9b, 0x43},
    {0x9c, 0xf0},
    {0x9d, 0xf0},
    {0x9e, 0xf0},
    {0x9f, 0xff},
    {0xa0, 0x68},
    {0xa1, 0x62},
    {0xa2, 0x0e},
    {0xFF, 0xFF}, // END
};



/**
 * struct ov7692_sensor - main structure for storage of sensor information
 * @pdata: access functions and data for platform level information
 * @v4l2_int_device: V4L2 device structure structure
 * @i2c_client: iic client device structure
 * @pix: V4L2 pixel format information structure
 * @timeperframe: time per frame expressed as V4L fraction
 * @scaler:
 * @ver: ov7692 chip version
 * @fps: frames per second value
 */
struct ov7692_sensor {
	const struct ov7692_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	int scaler;
	int ver;
	int fps;
	int state;
	bool resuming;
};

static struct ov7692_sensor ov7692;
static struct i2c_driver ov7692sensor_i2c_driver;
static unsigned long xclk_current = OV7692_XCLK_NOM_1;
static enum ov7692_image_size isize_current = VGA;

/* list of image formats supported by ov7692 sensor */
const static struct v4l2_fmtdesc ov7692_formats[] = {
	{
		.description	= "YUV",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
	},
	{
		.description	= "Bayer 10 bit BGGR",
		.pixelformat	= V4L2_PIX_FMT_SGRBG10,
	},
};

#define NUM_CAPTURE_FORMATS ARRAY_SIZE(ov7692_formats)

static u32 min_exposure_time = 1000;
static u32 max_exposure_time = 128000;
static enum v4l2_power current_power_state;

/* Structure of Sensor settings that change with image size */
static struct ov7692_sensor_settings sensor_settings[] = {
	 /* NOTE: must be in same order as image_size array */

	/* VGA */
	{
		.clk = {
			.pre_pll_div = 1,
			.pll_mult = 1,
			.post_pll_div = 1,
			.vt_pix_clk_div = 10,
			.vt_sys_clk_div = 1,
		},
		.mipi = {
			.data_lanes = 1,
			.ths_prepare = 2,
			.ths_zero = 5,
			.ths_settle_lower = 8,
			.ths_settle_upper = 28,
		},
		.frame = {
			.frame_len_lines_min = 200,
			.line_len_pck = 640,
			.x_addr_start = 0,
			.x_addr_end = 640,
			.y_addr_start = 0,
			.y_addr_end = 480,
			.x_output_size = 640,
			.y_output_size = 480,
			.x_even_inc = 9,
			.x_odd_inc = 7,
			.y_even_inc = 9,
			.y_odd_inc = 7,
			.v_mode_add = 0,
			.h_mode_add = 0,
			.h_add_ave = 1,
		},
	},
};

static struct ov7692_clock_freq current_clk;

struct i2c_list {
	struct i2c_msg *reg_list;
	unsigned int list_size;
};

/**
 * struct vcontrol - Video controls
 * @v4l2_queryctrl: V4L2 VIDIOC_QUERYCTRL ioctl structure
 * @current_value: current value of this control
 */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} ov7692sensor_video_control[] = {
	{
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = OV7692_MIN_EXPOSURE,
			.maximum = OV7692_MAX_EXPOSURE,
			.step = OV7692_EXPOSURE_STEP,
			.default_value = OV7692_DEF_EXPOSURE,
		},
		.current_value = OV7692_DEF_EXPOSURE,
	},
	{
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Analog Gain",
			.minimum = OV7692_EV_MIN_GAIN,
			.maximum = OV7692_EV_MAX_GAIN,
			.step = OV7692_EV_GAIN_STEP,
			.default_value = OV7692_EV_DEF_GAIN,
		},
		.current_value = OV7692_EV_DEF_GAIN,
	},
	{
		{
			.id = V4L2_CID_TEST_PATTERN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Test Pattern",
			.minimum = OV7692_MIN_TEST_PATT_MODE,
			.maximum = OV7692_MAX_TEST_PATT_MODE,
			.step = OV7692_MODE_TEST_PATT_STEP,
			.default_value = OV7692_MIN_TEST_PATT_MODE,
		},
		.current_value = OV7692_MIN_TEST_PATT_MODE,
	}
};

/**
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array for
 *
 * Returns the index of the requested ID from the control structure array
 */
static int
find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(ov7692sensor_video_control) - 1); i >= 0; i--)
		if (ov7692sensor_video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/**
 * ov7692_read_reg - Read a value from a register in an ov7692 sensor device
 * @client: i2c driver client structure
 * @data_length: length of data to be read
 * @reg: register address / offset
 * @val: stores the value that gets read
 *
 * Read a value from a register in an ov7692 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */

static int
ov7692_read_reg(struct i2c_client *client, u8 reg)
{
	
	return i2c_smbus_read_byte_data(client, reg);
	
#if 0
	int err;
	struct i2c_msg msg[1];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;
	if (data_length != I2C_8BIT && data_length != I2C_16BIT
			&& data_length != I2C_32BIT)
		return -EINVAL;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;

	/* Write addr - high byte goes out first */
	data[0] = (u8) (reg >> 8);;
	data[1] = (u8) (reg & 0xff);
	err = i2c_transfer(client->adapter, msg, 1);

	/* Read back data */
	if (err >= 0) {
		msg->len = data_length;
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		*val = 0;
		/* high byte comes first */
		if (data_length == I2C_8BIT)
			*val = data[0];
		else if (data_length == I2C_16BIT)
			*val = data[1] + (data[0] << 8);
		else
			*val = data[3] + (data[2] << 8) +
				(data[1] << 16) + (data[0] << 24);
		return 0;
	}
	v4l_err(client, "read from offset 0x%x error %d", reg, err);
	return err;
#endif
}
/**
 * Write a value to a register in ov7692 sensor device.
 * @client: i2c driver client structure.
 * @reg: Address of the register to read value from.
 * @val: Value to be written to a specific register.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov7692_write_reg(struct i2c_client *client, u8 addr,
						u8 val)
{

	return i2c_smbus_write_byte_data(client, addr, val);

}

/**
 * Initialize a list of ov7692 registers.
 * The list of registers is terminated by the pair of values
 * {OV3640_REG_TERM, OV3640_VAL_TERM}.
 * @client: i2c driver client structure.
 * @reglist[]: List of address of the registers to write data.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov7692_write_regs(struct i2c_client *client,
					const struct ov7692_reg reglist[])
{
	s32 err = 0;
	const struct ov7692_reg *list = reglist;
	int 	i;

	PEL(ov7692_write_regs);

	for(i=0; i<test; i++) 
	{
		while (!((list->reg == I2C_REG_TERM)
			&& (list->val == I2C_VAL_TERM))) {
			list++;
		}
	}
	while (!((list->reg == I2C_REG_TERM)
		&& (list->val == I2C_VAL_TERM))) {
		err = i2c_smbus_write_byte_data(client, list->reg, list->val);
		if (err<0)
			return err;
		list++;
	}
	return 0;
}

/**
 * ov7692_find_size - Find the best match for a requested image capture size
 * @width: requested image width in pixels
 * @height: requested image height in pixels
 *
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 * Since the available sizes are subsampled in the vertical direction only,
 * the routine will find the size with a height that is equal to or less
 * than the requested height.
 */
static enum ov7692_image_size ov7692_find_size(unsigned int width,
							unsigned int height)
{
	enum ov7692_image_size isize;

	PEL(ov7692_find_size);

	for (isize = VGA; isize <= VGA; isize++) {
		if ((ov7692_sizes[isize].height >= height) &&
			(ov7692_sizes[isize].width >= width)) {
			break;
		}
	}

	printk(KERN_DEBUG "ov7692_find_size: Req Size=%dx%d, "
			"Calc Size=%dx%d\n",
			width, height, (int)ov7692_sizes[isize].width,
			(int)ov7692_sizes[isize].height);

	return isize;
}

/**
 * Set CSI2 Virtual ID.
 * @client: i2c client driver structure
 * @id: Virtual channel ID.
 *
 * Sets the channel ID which identifies data packets sent by this device
 * on the CSI2 bus.
 **/
static int ov7692_set_virtual_id(struct i2c_client *client, u32 id)
{
	u8 curr;	
	s32 err;

	if((err = i2c_smbus_read_byte_data(client, OV7692_REG_MIPICTRL14))<0)
	{
		return err;
	}	
	curr = err & 0xFF;
	curr &= ~(MIPICTRL14_MASK_VC<<MIPICTRL14_SHFT_VC);
	curr |= ((MIPICTRL14_MASK_VC & id)<<MIPICTRL14_SHFT_VC);
	return i2c_smbus_write_byte_data(client, 
		OV7692_REG_MIPICTRL14, curr);
	
}

/**
 * ov7692_set_framerate - Sets framerate by adjusting frame_length_lines reg.
 * @s: pointer to standard V4L2 device structure
 * @fper: frame period numerator and denominator in seconds
 *
 * The maximum exposure time is also updated since it is affected by the
 * frame rate.
 **/
static int ov7692_set_framerate(struct v4l2_int_device *s,
						struct v4l2_fract *fper)
{
	int err = 0;

#if 0
	u16 isize = isize_current;
	u32 frame_length_lines, line_time_q8;
	struct ov7692_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct ov7692_sensor_settings *ss;

	if ((fper->numerator == 0) || (fper->denominator == 0)) {
		/* supply a default nominal_timeperframe */
		fper->numerator = 1;
		fper->denominator = OV7692_DEF_FPS;
	}

	sensor->fps = fper->denominator / fper->numerator;
	if (sensor->fps < OV7692_MIN_FPS) {
		sensor->fps = OV7692_MIN_FPS;
		fper->numerator = 1;
		fper->denominator = sensor->fps;
	} else if (sensor->fps > OV7692_MAX_FPS) {
		sensor->fps = OV7692_MAX_FPS;
		fper->numerator = 1;
		fper->denominator = sensor->fps;
	}

	ss = &sensor_settings[isize_current];

	line_time_q8 = ((u32)ss->frame.line_len_pck * 1000000) /
		(current_clk.vt_pix_clk >> 8); /* usec's */

	frame_length_lines = (((u32)fper->numerator * 1000000 * 256 /
		fper->denominator)) / line_time_q8;

	/* Range check frame_length_lines */
	if (frame_length_lines > OV7692_MAX_FRAME_LENGTH_LINES)
		frame_length_lines = OV7692_MAX_FRAME_LENGTH_LINES;
	else if (frame_length_lines < ss->frame.frame_len_lines_min)
		frame_length_lines = ss->frame.frame_len_lines_min;

	ov7692_write_reg(client, OV7692_REG_FRAME_LEN_LINES,
					frame_length_lines, I2C_16BIT);

	sensor_settings[isize].frame.frame_len_lines = frame_length_lines;

	/* Update max exposure time */
	max_exposure_time = (line_time_q8 * (frame_length_lines - 1)) >> 8;

	printk(KERN_DEBUG "OV7692 Set Framerate: fper=%d/%d, "
		"frame_len_lines=%d, max_expT=%dus\n", fper->numerator,
		fper->denominator, frame_length_lines, max_exposure_time);
#endif
	return err;
}

/**
 * ov7692sensor_calc_xclk - Calculate the required xclk frequency
 *
 * Xclk is not determined from framerate for the OV7692
 */
static unsigned long ov7692sensor_calc_xclk(void)
{
	xclk_current = OV7692_XCLK_NOM_1;

	return xclk_current;
}

/**
 * Sets the correct orientation based on the sensor version.
 *   IU046F2-Z   version=2  orientation=3
 *   IU046F4-2D  version>2  orientation=0
 */
static int ov7692_set_orientation(struct i2c_client *client, u32 ver)
{
	int err = 0;

#if 0
	u8 orient;

	orient = (ver <= 0x2) ? 0x3 : 0x0;
	err = ov7692_write_reg(client, OV7692_REG_IMAGE_ORIENTATION,
				orient, I2C_8BIT);
#endif
	return err;
}

/**
 * ov7692sensor_set_exposure_time - sets exposure time per input value
 * @exp_time: exposure time to be set on device (in usec)
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in ov7692sensor_video_controls array
 *
 * If the requested exposure time is within the allowed limits, the HW
 * is configured to use the new exposure time, and the
 * ov7692sensor_video_control[] array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int ov7692sensor_set_exposure_time(u32 exp_time, struct v4l2_int_device *s,
							struct vcontrol *lvc)
{
	int err = 0, i;

#if 0
	struct ov7692_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	u16 coarse_int_time = 0;
	u32 line_time_q8 = 0;
	struct ov7692_sensor_settings *ss;

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		if (exp_time < min_exposure_time) {
			v4l_err(client, "Exposure time %d us not within"
					" the legal range.\n", exp_time);
			v4l_err(client, "Exposure time must be between"
					" %d us and %d us\n",
					min_exposure_time, max_exposure_time);
			exp_time = min_exposure_time;
		}

		if (exp_time > max_exposure_time) {
			v4l_err(client, "Exposure time %d us not within"
					" the legal range.\n", exp_time);
			v4l_err(client, "Exposure time must be between"
					" %d us and %d us\n",
					min_exposure_time, max_exposure_time);
			exp_time = max_exposure_time;
		}

		ss = &sensor_settings[isize_current];

		line_time_q8 =
			((u32)ss->frame.line_len_pck * 1000000) /
			(current_clk.vt_pix_clk >> 8); /* usec's */

		coarse_int_time = ((exp_time * 256) + (line_time_q8 >> 1)) /
				 line_time_q8;

		if (coarse_int_time > ss->frame.frame_len_lines - 2)
			coarse_int_time = ss->frame.frame_len_lines - 2;

		err = ov7692_write_reg(client, OV7692_REG_COARSE_INT_TIME,
					coarse_int_time, I2C_16BIT);
	}

	if (err) {
		v4l_err(client, "Error setting exposure time: %d", err);
	} else {
		i = find_vctrl(V4L2_CID_EXPOSURE);
		if (i >= 0) {
			lvc = &ov7692sensor_video_control[i];
			lvc->current_value = exp_time;
		}
	}

#endif
	return err;
}

/**
 * This table describes what should be written to the sensor register for each
 * gain value. The gain(index in the table) is in terms of 0.1EV, i.e. 10
 * indexes in the table give 2 time more gain
 *
 * Elements in TS2_8_GAIN_TBL doesn't comply linearity. This is because
 * there is nonlinear dependecy between analogue_gain_code_global and real gain
 * value: Gain_analog = 256 / (256 - analogue_gain_code_global)
 */

const u16 OV7692_EV_GAIN_TBL[OV7692_EV_TABLE_GAIN_MAX + 1] = {
	/* Gain x1 */
	0,  16, 33, 48,
	62, 74, 88, 98,
	109, 119,

	/* Gain x2 */
	128, 136, 144, 152,
	159, 165, 171, 177,
	182, 187,

	/* Gain x4 */
	192, 196, 200, 204,
	208, 211, 214, 216,
	219, 222,

	/* Gain x8 */
	224
};

/**
 * ov7692sensor_set_gain - sets sensor analog gain per input value
 * @gain: analog gain value to be set on device
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 analog gain entry in ov7692sensor_video_control array
 *
 * If the requested analog gain is within the allowed limits, the HW
 * is configured to use the new gain value, and the ov7692sensor_video_control
 * array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int ov7692sensor_set_gain(u16 lineargain, struct v4l2_int_device *s,
							struct vcontrol *lvc)
{
	int err = 0, i;

#if 0
	u16 reg_gain = 0;
	struct ov7692_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;

	if (current_power_state == V4L2_POWER_ON || sensor->resuming) {

		if (lineargain < OV7692_EV_MIN_GAIN) {
			lineargain = OV7692_EV_MIN_GAIN;
			v4l_err(client, "Gain out of legal range.");
		}
		if (lineargain > OV7692_EV_MAX_GAIN) {
			lineargain = OV7692_EV_MAX_GAIN;
			v4l_err(client, "Gain out of legal range.");
		}

		reg_gain = OV7692_EV_GAIN_TBL[lineargain];

		err = ov7692_write_reg(client, OV7692_REG_ANALOG_GAIN_GLOBAL,
					reg_gain, I2C_16BIT);
	}

	if (err) {
		v4l_err(client, "Error setting analog gain: %d", err);
	} else {
		i = find_vctrl(V4L2_CID_GAIN);
		if (i >= 0) {
			lvc = &ov7692sensor_video_control[i];
			lvc->current_value = lineargain;
		}
	}
#endif
	return err;
}

/**
 * ov7692_update_clocks - calcs sensor clocks based on sensor settings.
 * @isize: image size enum
 */
int ov7692_update_clocks(u32 xclk, enum ov7692_image_size isize)
{

	PEL(ov7692_update_clocks);

	current_clk.vco_clk =
			xclk * sensor_settings[isize].clk.pll_mult /
			sensor_settings[isize].clk.pre_pll_div /
			sensor_settings[isize].clk.post_pll_div;

	current_clk.vt_pix_clk = current_clk.vco_clk * 2 /
			(sensor_settings[isize].clk.vt_pix_clk_div *
			sensor_settings[isize].clk.vt_sys_clk_div);

	if (sensor_settings[isize].mipi.data_lanes == 2)
		current_clk.mipi_clk = current_clk.vco_clk;
	else
		current_clk.mipi_clk = current_clk.vco_clk / 2;

	current_clk.mipi_clk = xclk;
	current_clk.ddr_clk = current_clk.mipi_clk / 2;

	printk(KERN_DEBUG "OV7692: xclk=%u, vco_clk=%u, "
		"vt_pix_clk=%u,  mipi_clk=%u,  ddr_clk=%u\n",
		xclk, current_clk.vco_clk, current_clk.vt_pix_clk,
		current_clk.mipi_clk, current_clk.ddr_clk);

	return 0;
}

/**
 * ov7692_setup_pll - initializes sensor PLL registers.
 * @c: i2c client driver structure
 * @isize: image size enum
 */
int ov7692_setup_pll(struct i2c_client *client, enum ov7692_image_size isize)
{
	PEL(ov7692_setup_pll);
#if 0
	u32 rgpltd_reg;
	u32 rgpltd[3] = {2, 0, 1};

	ov7692_write_reg(client, OV7692_REG_PRE_PLL_CLK_DIV,
		sensor_settings[isize].clk.pre_pll_div, I2C_16BIT);

	ov7692_write_reg(client, OV7692_REG_PLL_MULTIPLIER,
		sensor_settings[isize].clk.pll_mult, I2C_16BIT);

	ov7692_read_reg(client, I2C_8BIT, OV7692_REG_RGPLTD_RGCLKEN,
		&rgpltd_reg);
	rgpltd_reg &= ~RGPLTD_MASK;
	rgpltd_reg |= rgpltd[sensor_settings[isize].clk.post_pll_div >> 1];
	ov7692_write_reg(client, OV7692_REG_RGPLTD_RGCLKEN,
		rgpltd_reg, I2C_8BIT);

	ov7692_write_reg(client, OV7692_REG_VT_PIX_CLK_DIV,
		sensor_settings[isize].clk.vt_pix_clk_div, I2C_16BIT);

	ov7692_write_reg(client, OV7692_REG_VT_SYS_CLK_DIV,
		sensor_settings[isize].clk.vt_sys_clk_div, I2C_16BIT);

	printk(KERN_DEBUG "OV7692: pre_pll_clk_div=%u, pll_mult=%u, "
		"rgpltd=0x%x, vt_pix_clk_div=%u, vt_sys_clk_div=%u\n",
		sensor_settings[isize].clk.pre_pll_div,
		sensor_settings[isize].clk.pll_mult, rgpltd_reg,
		sensor_settings[isize].clk.vt_pix_clk_div,
		sensor_settings[isize].clk.vt_sys_clk_div);
#endif

	return 0;
}

/**
 * ov7692_setup_mipi - initializes sensor & isp MIPI registers.
 * @c: i2c client driver structure
 * @isize: image size enum
 */
int ov7692_setup_mipi(struct ov7692_sensor *sensor,
			enum ov7692_image_size isize)
{
	
	struct i2c_client *client = sensor->i2c_client;
	PEL(ov7692_setup_mipi);
	/* NOTE: Make sure ov7692_update_clocks is called 1st */
#if 0
	/* Enable MIPI */
	ov7692_write_reg(client, OV7692_REG_RGOUTSEL1, 0x00, I2C_8BIT);
	ov7692_write_reg(client, OV7692_REG_TESTDI, 0x04, I2C_8BIT);

	/* Set sensor Mipi timing params */
	ov7692_write_reg(client, OV7692_REG_RGTHSTRAIL, 0x06, I2C_8BIT);

	ov7692_write_reg(client, OV7692_REG_RGTHSPREPARE,
		sensor_settings[isize].mipi.ths_prepare, I2C_8BIT);

	ov7692_write_reg(client, OV7692_REG_RGTHSZERO,
		sensor_settings[isize].mipi.ths_zero, I2C_8BIT);

	/* Set number of lanes in sensor */
	if (sensor_settings[isize].mipi.data_lanes == 2)
		ov7692_write_reg(client, OV7692_REG_RGLANESEL, 0x00, I2C_8BIT);
	else
		ov7692_write_reg(client, OV7692_REG_RGLANESEL, 0x01, I2C_8BIT);
#endif

	/* Set number of lanes in isp */
	sensor->pdata->csi2_lane_count(sensor_settings[isize].mipi.data_lanes);

	/* Send settings to ISP-CSI2 Receiver PHY */
	sensor->pdata->csi2_calc_phy_cfg0(current_clk.mipi_clk,
		sensor_settings[isize].mipi.ths_settle_lower,
		sensor_settings[isize].mipi.ths_settle_upper);

	/* Dump some registers for debug purposes */
	printk(KERN_DEBUG "ov7692SPREPARE=0x%02X\n",
		sensor_settings[isize].mipi.ths_prepare);
	printk(KERN_DEBUG "imx:THSZERO=0x%02X\n",
		sensor_settings[isize].mipi.ths_zero);
	printk(KERN_DEBUG "imx:LANESEL=0x%02X\n",
		(sensor_settings[isize].mipi.data_lanes == 2) ? 0 : 1);

	return 0;
}

/**
 * ov7692_configure_frame - initializes image frame registers
 * @c: i2c client driver structure
 * @isize: image size enum
 */
int ov7692_configure_frame(struct i2c_client *client,
			enum ov7692_image_size isize)
{
	u32 val = 0;

#if 0
	ov7692_write_reg(client, OV7692_REG_FRAME_LEN_LINES,
		sensor_settings[isize].frame.frame_len_lines_min, I2C_16BIT);

	ov7692_write_reg(client, OV7692_REG_LINE_LEN_PCK,
		sensor_settings[isize].frame.line_len_pck, I2C_16BIT);

	ov7692_write_reg(client, OV7692_REG_X_ADDR_START,
		sensor_settings[isize].frame.x_addr_start, I2C_16BIT);

	ov7692_write_reg(client, OV7692_REG_X_ADDR_END,
		sensor_settings[isize].frame.x_addr_end, I2C_16BIT);

	ov7692_write_reg(client, OV7692_REG_Y_ADDR_START,
		sensor_settings[isize].frame.y_addr_start, I2C_16BIT);

	ov7692_write_reg(client, OV7692_REG_Y_ADDR_END,
		sensor_settings[isize].frame.y_addr_end, I2C_16BIT);

	ov7692_write_reg(client, OV7692_REG_X_OUTPUT_SIZE,
		sensor_settings[isize].frame.x_output_size, I2C_16BIT);

	ov7692_write_reg(client, OV7692_REG_Y_OUTPUT_SIZE,
		sensor_settings[isize].frame.y_output_size, I2C_16BIT);

	ov7692_write_reg(client, OV7692_REG_X_EVEN_INC,
		sensor_settings[isize].frame.x_even_inc, I2C_16BIT);

	ov7692_write_reg(client, OV7692_REG_X_ODD_INC,
		sensor_settings[isize].frame.x_odd_inc, I2C_16BIT);

	ov7692_write_reg(client, OV7692_REG_Y_EVEN_INC,
		sensor_settings[isize].frame.y_even_inc, I2C_16BIT);

	ov7692_write_reg(client, OV7692_REG_Y_ODD_INC,
		sensor_settings[isize].frame.y_odd_inc, I2C_16BIT);

	ov7692_read_reg(client, I2C_8BIT, OV7692_REG_PGACUR_VMODEADD, &val);
	val &= ~VMODEADD_MASK;
	val |= sensor_settings[isize].frame.v_mode_add << VMODEADD_SHIFT;
	ov7692_write_reg(client, OV7692_REG_PGACUR_VMODEADD, val, I2C_8BIT);

	ov7692_read_reg(client, I2C_8BIT, OV7692_REG_HMODEADD, &val);
	val &= ~HMODEADD_MASK;
	val |= sensor_settings[isize].frame.h_mode_add << HMODEADD_SHIFT;
	ov7692_write_reg(client, OV7692_REG_HMODEADD, val, I2C_8BIT);

	ov7692_read_reg(client, I2C_8BIT, OV7692_REG_HADDAVE, &val);
	val &= ~HADDAVE_MASK;
	val |= sensor_settings[isize].frame.h_add_ave << HADDAVE_SHIFT;
	ov7692_write_reg(client, OV7692_REG_HADDAVE, val, I2C_8BIT);
#endif

	return 0;
}

 /**
 * ov7692_configure_test_pattern - Configure 3 possible test pattern modes
 * @ mode: Test pattern mode. Possible modes : 1 , 2 and 4.
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in ov7692sensor_video_controls array
 *
 * If the requested test pattern mode is within the allowed limits, the HW
 * is configured for that particular test pattern, and the
 * ov7692sensor_video_control[] array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int ov7692_configure_test_pattern(int mode, struct v4l2_int_device *s,
							struct vcontrol *lvc)
{
#if 0
	struct ov7692_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {

		switch (mode) {
		case OV7692_TEST_PATT_COLOR_BAR:
		case OV7692_TEST_PATT_PN9:
			/* red */
			ov7692_write_reg(client, OV7692_REG_TEST_PATT_RED,
							0x07ff, I2C_16BIT);
			/* green-red */
			ov7692_write_reg(client, OV7692_REG_TEST_PATT_GREENR,
							0x00ff,	I2C_16BIT);
			/* blue */
			ov7692_write_reg(client, OV7692_REG_TEST_PATT_BLUE,
							0x0000, I2C_16BIT);
			/* green-blue */
			ov7692_write_reg(client, OV7692_REG_TEST_PATT_GREENB,
							0x0000,	I2C_16BIT);
			break;
		case OV7692_TEST_PATT_SOLID_COLOR:
			/* red */
			ov7692_write_reg(client, OV7692_REG_TEST_PATT_RED,
				(OV7692_BLACK_LEVEL_AVG & 0x00ff), I2C_16BIT);
			/* green-red */
			ov7692_write_reg(client, OV7692_REG_TEST_PATT_GREENR,
				(OV7692_BLACK_LEVEL_AVG & 0x00ff), I2C_16BIT);
			/* blue */
			ov7692_write_reg(client, OV7692_REG_TEST_PATT_BLUE,
				(OV7692_BLACK_LEVEL_AVG & 0x00ff), I2C_16BIT);
			/* green-blue */
			ov7692_write_reg(client, OV7692_REG_TEST_PATT_GREENB,
				(OV7692_BLACK_LEVEL_AVG & 0x00ff), I2C_16BIT);
			break;
		}
		/* test-pattern mode */
		ov7692_write_reg(client, OV7692_REG_TEST_PATT_MODE,
						(mode & 0x7), I2C_16BIT);
		/* Disable sensor ISP processing */
		ov7692_write_reg(client, OV7692_REG_TESBYPEN,
					(mode == 0) ? 0x0 : 0x10, I2C_8BIT);
	}
	lvc->current_value = mode;
#endif
	return 0;
}
/**
 * ov7692_configure - Configure the ov7692 for the specified image mode
 * @s: pointer to standard V4L2 device structure
 *
 * Configure the ov7692 for a specified image size, pixel format, and frame
 * period.  xclk is the frequency (in Hz) of the xclk input to the ov7692.
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int ov7692_configure(struct v4l2_int_device *s)
{
	struct ov7692_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &sensor->pix;
	struct i2c_client *client = sensor->i2c_client;
	enum ov7692_image_size isize;
	int err, i;
	struct vcontrol *lvc = NULL;
	
	PEL(ov7692_configure);

	isize = ov7692_find_size(pix->width, pix->height);
	isize_current = isize;

	err = i2c_smbus_write_byte_data(client, OV7692_REG_REG12, REG12_RESET);
	mdelay(5);

	ov7692_write_regs(client, initial_list);

	ov7692_update_clocks(xclk_current, isize);
	ov7692_setup_pll(client, isize);

	ov7692_setup_mipi(sensor, isize);

	/* configure image size and pixel format */
	ov7692_configure_frame(client, isize);

	/* Setting of frame rate */
	err = ov7692_set_framerate(s, &sensor->timeperframe);

	ov7692_set_orientation(client, sensor->ver);

	sensor->pdata->csi2_cfg_vp_out_ctrl(2);
	sensor->pdata->csi2_ctrl_update(false);

	sensor->pdata->csi2_cfg_virtual_id(0, OV7692_CSI2_VIRTUAL_ID);
	sensor->pdata->csi2_ctx_update(0, true);
	ov7692_set_virtual_id(client, OV7692_CSI2_VIRTUAL_ID);

	/* Set initial exposure and gain */
	i = find_vctrl(V4L2_CID_EXPOSURE);
	if (i >= 0) {
		lvc = &ov7692sensor_video_control[i];
		ov7692sensor_set_exposure_time(lvc->current_value,
					sensor->v4l2_int_device, lvc);
	}

	i = find_vctrl(V4L2_CID_GAIN);
	if (i >= 0) {
		lvc = &ov7692sensor_video_control[i];
		ov7692sensor_set_gain(lvc->current_value,
				sensor->v4l2_int_device, lvc);
	}

	i = find_vctrl(V4L2_CID_TEST_PATTERN);
	if (i >= 0) {
		lvc = &ov7692sensor_video_control[i];
		ov7692_configure_test_pattern(lvc->current_value,
				sensor->v4l2_int_device, lvc);
	}
	/* configure streaming ON */
#if 0
	err = ov7692_write_reg(client, OV7692_REG_MODE_SELECT, 0x01, I2C_8BIT);
#endif
	mdelay(1);

	return err;
}

/**
 * ov7692_detect - Detect if an ov7692 is present, and if so which revision
 * @client: pointer to the i2c client driver structure
 *
 * Detect if an ov7692 is present, and if so which revision.
 * A device is considered to be detected if the manufacturer ID (MIDH and MIDL)
 * and the product ID (PID) registers match the expected values.
 * Any value of the version ID (VER) register is accepted.
 * Returns a negative error number if no device is detected, or the
 * non-negative value of the version ID register if a device is detected.
 */
static int
ov7692_detect(struct i2c_client *client)
{
	u32 model_id, mfr_id, rev;
	struct ov7692_sensor *sensor;

	if (!client)
		return -ENODEV;

	sensor = i2c_get_clientdata(client);

	if ((model_id=i2c_smbus_read_byte_data(client, OV7692_REG_PIDH)) < 0)
	{
		return -ENODEV;
	}
	if ((rev=i2c_smbus_read_byte_data(client, OV7692_REG_PIDL)) <0)
	{
		return -ENODEV;
	}
	model_id <<= 8;
	model_id += rev;
	if ((mfr_id=i2c_smbus_read_byte_data(client, OV7692_REG_MIDH)) <0)
	{
		return -ENODEV;
	}
	if ((rev=i2c_smbus_read_byte_data(client, OV7692_REG_MIDL)) <0)
	{
		return -ENODEV;
	}
	mfr_id <<= 8;
	mfr_id += rev;
	v4l_info(client, "model id detected 0x%x mfr 0x%x\n",
			model_id, mfr_id);
	if ((model_id != OV7692_MOD_ID) || (mfr_id != OV7692_MFR_ID)) {
		/* We didn't read the values we expected, so
		 * this must not be an OV7692.
		 */
		v4l_warn(client, "model id mismatch 0x%x mfr 0x%x\n",
							model_id, mfr_id);

		return -ENODEV;
	}
	return model_id;
}

/**
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the ov7692sensor_video_control[] array.
 * Otherwise, returns -EINVAL if the control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
				struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = ov7692sensor_video_control[i].qc;
	return 0;
}

/**
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the ov7692sensor_video_control[] array.
 * Otherwise, returns -EINVAL if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	struct vcontrol *lvc;
	int i;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &ov7692sensor_video_control[i];

	switch (vc->id) {
	case  V4L2_CID_EXPOSURE:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_GAIN:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_TEST_PATTERN:
		vc->value = lvc->current_value;
		break;
	}

	return 0;
}

/**
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the ov7692sensor_video_control[] array).
 * Otherwise, * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	int retval = -EINVAL;
	int i;
	struct vcontrol *lvc;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &ov7692sensor_video_control[i];

	switch (vc->id) {
	case V4L2_CID_EXPOSURE:
		retval = ov7692sensor_set_exposure_time(vc->value, s, lvc);
		break;
	case V4L2_CID_GAIN:
		retval = ov7692sensor_set_gain(vc->value, s, lvc);
		break;
	case V4L2_CID_TEST_PATTERN:
		retval = ov7692_configure_test_pattern(vc->value, s, lvc);
		break;
	}

	return retval;
}

/**
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
				   struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= NUM_CAPTURE_FORMATS)
			return -EINVAL;
	break;
	default:
		return -EINVAL;
	}

	fmt->flags = ov7692_formats[index].flags;
	strlcpy(fmt->description, ov7692_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = ov7692_formats[index].pixelformat;

	return 0;
}

/**
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int ioctl_try_fmt_cap(struct v4l2_int_device *s,
			     struct v4l2_format *f)
{
	enum ov7692_image_size isize;
	int ifmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct ov7692_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix2 = &sensor->pix;

	isize = ov7692_find_size(pix->width, pix->height);

	pix->width = ov7692_sizes[isize].width;
	pix->height = ov7692_sizes[isize].height;
	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == ov7692_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;
	pix->pixelformat = ov7692_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	*pix2 = *pix;
	return 0;
}

/**
 * ioctl_s_fmt_cap - V4L2 sensor interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
static int ioctl_s_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct ov7692_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rval;

	rval = ioctl_try_fmt_cap(s, f);
	if (rval)
		return rval;
	else
		sensor->pix = *pix;


	return rval;
}

/**
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct ov7692_sensor *sensor = s->priv;
	f->fmt.pix = sensor->pix;

	return 0;
}

/**
 * ioctl_g_pixclk - V4L2 sensor interface handler for ioctl_g_pixclk
 * @s: pointer to standard V4L2 device structure
 * @pixclk: pointer to unsigned 32 var to store pixelclk in HZ
 *
 * Returns the sensor's current pixel clock in HZ
 */
static int ioctl_priv_g_pixclk(struct v4l2_int_device *s, u32 *pixclk)
{
	*pixclk = xclk_current;

	return 0;
}

/**
 * ioctl_g_activesize - V4L2 sensor interface handler for ioctl_g_activesize
 * @s: pointer to standard V4L2 device structure
 * @pix: pointer to standard V4L2 v4l2_pix_format structure
 *
 * Returns the sensor's current active image basesize.
 */
static int ioctl_priv_g_activesize(struct v4l2_int_device *s,
			      struct v4l2_pix_format *pix)
{
	struct ov7692_sensor *sensor = s->priv;

	pix->width = sensor->pix.width;
	pix->height = sensor->pix.height;

	return 0;
}

/**
 * ioctl_g_fullsize - V4L2 sensor interface handler for ioctl_g_fullsize
 * @s: pointer to standard V4L2 device structure
 * @pix: pointer to standard V4L2 v4l2_pix_format structure
 *
 * Returns the sensor's biggest image basesize.
 */
static int ioctl_priv_g_fullsize(struct v4l2_int_device *s,
			    struct v4l2_pix_format *pix)
{
	pix->width = ov7692_sizes[NUM_IMAGE_SIZES - 1].width;
	pix->height = ov7692_sizes[NUM_IMAGE_SIZES - 1].height;

	return 0;
}

/**
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	struct ov7692_sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe = sensor->timeperframe;

	return 0;
}

/**
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	struct ov7692_sensor *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;

	sensor->timeperframe = *timeperframe;
	ov7692sensor_calc_xclk();
	*timeperframe = sensor->timeperframe;

	return 0;
}


/**
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct ov7692_sensor *sensor = s->priv;

	return sensor->pdata->priv_data_set(s, p);

}

/**
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct ov7692_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct vcontrol *lvc;
	int rval, i;

	if ((on == V4L2_POWER_STANDBY) && (sensor->state == SENSOR_DETECTED)) {
		/* ov7692_write_regs(c, stream_off_list,
						I2C_STREAM_OFF_LIST_SIZE); */
	}
	
	if (on != V4L2_POWER_ON)
		sensor->pdata->set_xclk(s, xclk_current);
		//sensor->pdata->set_xclk(s, 0);
	else 
	{
		sensor->pdata->set_xclk(s, xclk_current);
	}


	rval = sensor->pdata->power_set(s, on);
	if (rval < 0) {
		v4l_err(client, "Unable to set the power state: "
			OV7692_DRIVER_NAME " sensor\n");
		sensor->pdata->set_xclk(s, 0);
		return rval;
	}
	printk( KERN_INFO "JFK POWER SET %s:%d\n", __FILE__, __LINE__);



	if ((current_power_state == V4L2_POWER_STANDBY) &&
					(on == V4L2_POWER_ON) &&
					(sensor->state == SENSOR_DETECTED)) {
		sensor->resuming = true;
		ov7692_configure(s);
	}

	if ((on == V4L2_POWER_ON) && (sensor->state == SENSOR_NOT_DETECTED)) {
		rval = ov7692_detect(client);
		if (rval < 0) {
			v4l_err(client, "Unable to detect "
					OV7692_DRIVER_NAME " sensor\n");
			sensor->state = SENSOR_NOT_DETECTED;
			return rval;
		}
		sensor->state = SENSOR_DETECTED;
		sensor->ver = rval;
		v4l_info(client, OV7692_DRIVER_NAME
			" chip version 0x%02x detected\n", sensor->ver);
	}

	if (on == V4L2_POWER_OFF) {
		/* Reset defaults for controls */
		i = find_vctrl(V4L2_CID_GAIN);
		if (i >= 0) {
			lvc = &ov7692sensor_video_control[i];
			lvc->current_value = OV7692_EV_DEF_GAIN;
		}
		i = find_vctrl(V4L2_CID_EXPOSURE);
		if (i >= 0) {
			lvc = &ov7692sensor_video_control[i];
			lvc->current_value = OV7692_DEF_EXPOSURE;
		}
		i = find_vctrl(V4L2_CID_TEST_PATTERN);
		if (i >= 0) {
			lvc = &ov7692sensor_video_control[i];
			lvc->current_value = OV7692_MIN_TEST_PATT_MODE;
		}
	}

	sensor->resuming = false;
	current_power_state = on;
	return 0;
}

/**
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call ov7692_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the dev. at slave detach.  The complement of ioctl_dev_init.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.  Returns 0 if
 * ov7692 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct ov7692_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	int err;

	err = ov7692_detect(client);
	if (err < 0) {
		v4l_err(client, "Unable to detect " OV7692_DRIVER_NAME
				" sensor\n");
		return err;
	}

	sensor->ver = err;
	v4l_info(client, OV7692_DRIVER_NAME " chip version "
			"0x%02x detected\n", sensor->ver);

	return 0;
}

/**
 * ioctl_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 **/
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
					struct v4l2_frmsizeenum *frms)
{
	int ifmt;

	PEL(ioctl_enum_framesizes);

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frms->pixel_format == ov7692_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Check that the index we are being asked for is not
	   out of bounds. */
	if (frms->index >= ARRAY_SIZE(ov7692_sizes))
		return -EINVAL;

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = ov7692_sizes[frms->index].width;
	frms->discrete.height = ov7692_sizes[frms->index].height;

	return 0;
}

const struct v4l2_fract ov7692_frameintervals[] = {
	{ .numerator = 3, .denominator = 30 },
	{ .numerator = 1, .denominator = 30 },
};

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					struct v4l2_frmivalenum *frmi)
{
	int ifmt;

	/* Check that the requested format is one we support */
	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frmi->pixel_format == ov7692_formats[ifmt].pixelformat)
			break;
	}

	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Check that the index we are being asked for is not
	   out of bounds. */
	if (frmi->index >= ARRAY_SIZE(ov7692_frameintervals))
		return -EINVAL;

	/* Make sure that the 8MP size reports a max of 10fps */
	if (frmi->width == 3280 && frmi->height == 2464) {
		if (frmi->index != 0)
			return -EINVAL;
	}

	frmi->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	frmi->discrete.numerator =
				ov7692_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
				ov7692_frameintervals[frmi->index].denominator;

	return 0;
}

static struct v4l2_int_ioctl_desc ov7692_ioctl_desc[] = {
	{ .num = vidioc_int_enum_framesizes_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{ .num = vidioc_int_enum_frameintervals_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_frameintervals},
	{ .num = vidioc_int_dev_init_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_dev_init},
	{ .num = vidioc_int_dev_exit_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_dev_exit},
	{ .num = vidioc_int_s_power_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_power },
	{ .num = vidioc_int_g_priv_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_priv },
	{ .num = vidioc_int_init_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_init },
	{ .num = vidioc_int_enum_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
	{ .num = vidioc_int_try_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_try_fmt_cap },
	{ .num = vidioc_int_g_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
	{ .num = vidioc_int_s_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_fmt_cap },
	{ .num = vidioc_int_g_parm_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ .num = vidioc_int_s_parm_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_parm },
	{ .num = vidioc_int_queryctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_queryctrl },
	{ .num = vidioc_int_g_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ .num = vidioc_int_s_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_ctrl },
	{ .num = vidioc_int_priv_g_pixclk_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_priv_g_pixclk },
	{ .num = vidioc_int_priv_g_activesize_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_priv_g_activesize },
	{ .num = vidioc_int_priv_g_fullsize_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_priv_g_fullsize },
};

static struct v4l2_int_slave ov7692_slave = {
	.ioctls = ov7692_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov7692_ioctl_desc),
};

static struct v4l2_int_device ov7692_int_device = {
	.module = THIS_MODULE,
	.name = OV7692_DRIVER_NAME,
	.priv = &ov7692,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ov7692_slave,
	},
};

/**
 * ov7692_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int __devinit ov7692_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct ov7692_sensor *sensor = &ov7692;
	int err;

	if (i2c_get_clientdata(client))
		return -EBUSY;

	sensor->pdata = client->dev.platform_data;

	if (!sensor->pdata) {
		v4l_err(client, "no platform data?\n");
		return -ENODEV;
	}

	sensor->v4l2_int_device = &ov7692_int_device;
	sensor->i2c_client = client;

	i2c_set_clientdata(client, sensor);

	/* Make the default capture format QCIF V4L2_PIX_FMT_SRGGB10 */
	sensor->pix.width = OV7692_IMAGE_WIDTH_MAX;
	sensor->pix.height = OV7692_IMAGE_HEIGHT_MAX;
	sensor->pix.pixelformat = V4L2_PIX_FMT_SRGGB10;

	err = v4l2_int_device_register(sensor->v4l2_int_device);
	if (err)
		i2c_set_clientdata(client, NULL);

	return 0;
}

/**
 * ov7692_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of ov7692_probe().
 */
static int __exit
ov7692_remove(struct i2c_client *client)
{
	struct ov7692_sensor *sensor = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(sensor->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id ov7692_id[] = {
	{ OV7692_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ov7692_id);

static struct i2c_driver ov7692sensor_i2c_driver = {
	.driver = {
		.name = OV7692_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = ov7692_probe,
	.remove = __exit_p(ov7692_remove),
	.id_table = ov7692_id,
};

static struct ov7692_sensor ov7692 = {
	.timeperframe = {
		.numerator = 1,
		.denominator = 30,
	},
	.state = SENSOR_NOT_DETECTED,
};

/**
 * ov7692sensor_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init ov7692sensor_init(void)
{
	int err;

	err = i2c_add_driver(&ov7692sensor_i2c_driver);
	if (err) {
		printk(KERN_ERR "Failed to register" OV7692_DRIVER_NAME ".\n");
		return err;
	}
	return 0;
}
late_initcall(ov7692sensor_init);

/**
 * ov7692sensor_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of ov7692sensor_init.
 */
static void __exit ov7692sensor_cleanup(void)
{
	i2c_del_driver(&ov7692sensor_i2c_driver);
}
module_exit(ov7692sensor_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ov7692 camera sensor driver");
