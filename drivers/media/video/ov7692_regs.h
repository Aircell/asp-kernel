/*
 * ov7692_regs.h
 *
 * Register definitions for the OV7692 Sensor.
 *
 * Leverage MT9P012.h
 *
 * Copyright (C) 2008 Hewlett Packard.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef OV7692_REGS_H
#define OV7692_REGS_H

#define V4L2_CID_TEST_PATTERN		(V4L2_CID_PRIVATE_BASE + 1)

/* The ID values we are looking for */
#define OV7692_MOD_ID			0x7692
#define OV7692_MFR_ID			0x7FA2


/* Register definitions */
#define OV7692_REG_PIDH		0x0A
#define OV7692_REG_PIDL		0x0B 
#define OV7692_REG_REG0C	0x0C 
#define OV7692_REG_AECH		0x0F 
#define OV7692_REG_AECL		0x10 
#define OV7692_REG_CLKRC	0x11 
#define OV7692_REG_REG12	0x12 
# define REG12_RESET 0x80
#define OV7692_REG_REG14	0x14 
#define OV7692_REG_REG15	0x15 
#define OV7692_REG_REG16	0x16 
#define OV7692_REG_HSTART	0x17 
#define OV7692_REG_HSIZE	0x18 
#define OV7692_REG_VSTART	0x19 #define OV7692_REG_VSIZE	0x1A 	
#define OV7692_REG_SHFT		0x1B 
#define OV7692_REG_MIDH		0x1C 
#define OV7692_REG_MIDL		0x1D 
#define OV7692_REG_REG61	0x61
#   define REG61_TESTPATTERN_ENABLE (1<<6)
#   define REG61_SHFT_PATTERNTYPE (1<<4)
#   define REG61_MASK_PATTERNTYPE (0x3)
#define OV7692_REG_RSTCTRL	0x63 
#define OV7692_REG_RSTCTRL	0x63 
#define OV7692_REG_REGC8	0xC8 
#define OV7692_REG_REGC9	0xC9 
#define OV7692_REG_REGCA	0xCA 
#define OV7692_REG_REGCB	0xCB 
#define OV7692_REG_REGCC	0xCC 
#define OV7692_REG_REGCD	0xCD 
#define OV7692_REG_REGCE	0xCE 
#define OV7692_REG_REGCF	0xCF 
#define OV7692_REG_REGDD	0xDD 
#define OV7692_REG_MIPIBANK	0xFF 
# define MIPIBANK_ENABLE 1
#define OV7692_REG_MIPICTRL00	0x80 
#    define MIPICTRL00_GATECLK	(1<<5); 
#    define MIPICTRL00_IDLE1	(1<<2); 
#define OV7692_REG_MIPICTRL01	0x81 
#define OV7692_REG_MIPICTRL02	0x82 
#define OV7692_REG_MIPICTRL03	0x83 
#define OV7692_REG_MIPICTRL04	0x84 
#define OV7692_REG_MIPICTRL05	0x85 
#define OV7692_REG_MIPICTRL06	0x86 
#define OV7692_REG_MIPICTRL07	0x87 
#define OV7692_REG_MIPICTRL08	0x88 
#define OV7692_REG_MIPICTRL09	0x89 
#define OV7692_REG_MIPICTRL14	0x8E 
#  define MIPICTRL14_SHFT_VC 6
#  define MIPICTRL14_MASK_VC 0x3 
#define OV7692_REG_MIPIFCNTH	0x8A 
#define OV7692_REG_MIPIFCNTL	0x8B 
#define OV7692_REG_MIPISPKTH	0x8C 
#define OV7692_REG_MIPISPKTL	0x8D 
#define OV7692_REG_MIPICOM0	0xB5 
#   define MIPICOM0_ENABLE   (1<<7);
#   define MIPICOM0_8BITMODE (1<<6);
#   define MIPICOM0_MIPITEST (1<<5);
#   define MIPICOM0_MASK_PCLKDIV (0x1F);
#   define MIPICOM0_SHFT_PCLKDIV (0);
#define OV7692_REG_RMIPI0	0xB5 
#   define MIPI0_POWERDOWN   (1<<5);

#define OV7692_REG_RMIPI1	0xB6 


/* Terminating list entry for reg */
#define I2C_REG_TERM		0xFF
/* Terminating list entry for val */
#define I2C_VAL_TERM		0xFF

/* terminating token for reg list */
#define OV7692_TOK_TERM 		0xFF

/* delay token for reg list */
#define OV7692_TOK_DELAY		100

/* CSI2 Virtual ID */
#define OV7692_CSI2_VIRTUAL_ID	0x0

#define OV7692_CLKRC			0x11

/* Used registers */
#define OV7692_REG_MODEL_ID				0x0000
#define OV7692_REG_REV_NUMBER			0x0002
#define OV7692_REG_MFR_ID				0x0003

#define OV7692_REG_MODE_SELECT			0x0100
#define OV7692_REG_IMAGE_ORIENTATION	0x0101
#define OV7692_REG_SW_RESET				0x0103
#define OV7692_REG_GROUPED_PAR_HOLD		0x0104
#define OV7692_REG_CCP2_CHANNEL_ID		0x0110

#define OV7692_REG_FINE_INT_TIME		0x0200
#define OV7692_REG_COARSE_INT_TIME		0x0202

#define OV7692_REG_ANALOG_GAIN_GLOBAL	0x0204
#define OV7692_REG_ANALOG_GAIN_GREENR	0x0206
#define OV7692_REG_ANALOG_GAIN_RED		0x0208
#define OV7692_REG_ANALOG_GAIN_BLUE		0x020A
#define OV7692_REG_ANALOG_GAIN_GREENB	0x020C
#define OV7692_REG_DIGITAL_GAIN_GREENR	0x020E
#define OV7692_REG_DIGITAL_GAIN_RED		0x0210
#define OV7692_REG_DIGITAL_GAIN_BLUE	0x0212
#define OV7692_REG_DIGITAL_GAIN_GREENB	0x0214

#define OV7692_REG_VT_PIX_CLK_DIV		0x0300
#define OV7692_REG_VT_SYS_CLK_DIV		0x0302
#define OV7692_REG_PRE_PLL_CLK_DIV		0x0304
#define OV7692_REG_PLL_MULTIPLIER		0x0306
#define OV7692_REG_OP_PIX_CLK_DIV		0x0308
#define OV7692_REG_OP_SYS_CLK_DIV		0x030A

#define OV7692_REG_FRAME_LEN_LINES		0x0340
#define OV7692_REG_LINE_LEN_PCK			0x0342

#define OV7692_REG_X_ADDR_START			0x0344
#define OV7692_REG_Y_ADDR_START			0x0346
#define OV7692_REG_X_ADDR_END			0x0348
#define OV7692_REG_Y_ADDR_END			0x034A
#define OV7692_REG_X_OUTPUT_SIZE		0x034C
#define OV7692_REG_Y_OUTPUT_SIZE		0x034E
#define OV7692_REG_X_EVEN_INC			0x0380
#define OV7692_REG_X_ODD_INC			0x0382
#define OV7692_REG_Y_EVEN_INC			0x0384
#define OV7692_REG_Y_ODD_INC			0x0386

#define OV7692_REG_HMODEADD				0x3001
#define HMODEADD_SHIFT					7
#define HMODEADD_MASK  					(0x1 << HMODEADD_SHIFT)
#define OV7692_REG_OPB_CTRL				0x300C
#define OV7692_REG_Y_OPBADDR_START_DI	0x3014
#define OV7692_REG_Y_OPBADDR_END_DI		0x3015
#define OV7692_REG_PGACUR_VMODEADD		0x3016
#define VMODEADD_SHIFT					6
#define VMODEADD_MASK  					(0x1 << VMODEADD_SHIFT)
#define OV7692_REG_CHCODE_OUTCHSINGLE	0x3017
#define OV7692_REG_OUTIF				0x301C
#define OV7692_REG_RGPLTD_RGCLKEN		0x3022
#define RGPLTD_MASK						0x3
#define OV7692_REG_RGPOF_RGPOFV2		0x3031
#define OV7692_REG_CPCKAUTOEN			0x3040
#define OV7692_REG_RGCPVFB				0x3041
#define OV7692_REG_RGAZPDRV				0x3051
#define OV7692_REG_RGAZTEST				0x3053
#define OV7692_REG_RGVSUNLV				0x3055
#define OV7692_REG_CLPOWER				0x3060
#define OV7692_REG_CLPOWERSP			0x3065
#define OV7692_REG_ACLDIRV_TVADDCLP		0x30AA
#define OV7692_REG_TESTDI				0x30E5
#define OV7692_REG_HADDAVE				0x30E8
#define HADDAVE_SHIFT					7
#define HADDAVE_MASK  					(0x1 << HADDAVE_SHIFT)

#define OV7692_REG_RAW10CH2V2P_LO		0x31A4
#define OV7692_REG_RAW10CH2V2D_LO		0x31A6
#define OV7692_REG_COMP8CH1V2P_LO		0x31AC
#define OV7692_REG_COMP8CH1V2D_LO		0x31AE
#define OV7692_REG_RAW10CH1V2P_LO		0x31B4
#define OV7692_REG_RAW10CH1V2D_LO		0x31B6

#define OV7692_REG_RGOUTSEL1			0x3300
#define OV7692_REG_RGLANESEL			0x3301
#define OV7692_REG_RGTLPX 				0x3304
#define OV7692_REG_RGTCLKPREPARE 		0x3305
#define OV7692_REG_RGTCLKZERO 			0x3306
#define OV7692_REG_RGTCLKPRE 			0x3307
#define OV7692_REG_RGTCLKPOST 			0x3308
#define OV7692_REG_RGTCLKTRAIL 			0x3309
#define OV7692_REG_RGTHSEXIT 			0x330A
#define OV7692_REG_RGTHSPREPARE			0x330B
#define OV7692_REG_RGTHSZERO			0x330C
#define OV7692_REG_RGTHSTRAIL 			0x330D

#define OV7692_REG_TESBYPEN		0x30D8
#define OV7692_REG_TEST_PATT_MODE	0x0600
#define OV7692_REG_TEST_PATT_RED	0x0602
#define OV7692_REG_TEST_PATT_GREENR	0x0604
#define OV7692_REG_TEST_PATT_BLUE	0x0606
#define OV7692_REG_TEST_PATT_GREENB	0x0608

/*
 * The nominal xclk input frequency of the OV7692 is 18MHz, maximum
 * frequency is 45MHz, and minimum frequency is 6MHz.
 */
#define OV7692_XCLK_MIN   	6000000
#define OV7692_XCLK_MAX   	54000000
#define OV7692_XCLK_NOM_1 	24000000
#define OV7692_XCLK_NOM_2 	24000000

/* FPS Capabilities */
#define OV7692_MIN_FPS		7
#define OV7692_DEF_FPS		15
#define OV7692_MAX_FPS		30

#define I2C_RETRY_COUNT		5

/* Still capture 8 MP */
#define OV7692_IMAGE_WIDTH_MAX	640
#define OV7692_IMAGE_HEIGHT_MAX	480

/* Analog gain values */
#define OV7692_EV_MIN_GAIN		0
#define OV7692_EV_MAX_GAIN		30
#define OV7692_EV_DEF_GAIN		21
#define OV7692_EV_GAIN_STEP		1
/* maximum index in the gain EVT */
#define OV7692_EV_TABLE_GAIN_MAX	30

/* Exposure time values */
#define OV7692_MIN_EXPOSURE		250
#define OV7692_MAX_EXPOSURE		128000
#define OV7692_DEF_EXPOSURE	    33000
#define OV7692_EXPOSURE_STEP	50

/* Test Pattern Values */
#define OV7692_MIN_TEST_PATT_MODE	0
#define OV7692_MAX_TEST_PATT_MODE	4
#define OV7692_MODE_TEST_PATT_STEP	1

#define OV7692_TEST_PATT_SOLID_COLOR 	1
#define OV7692_TEST_PATT_COLOR_BAR	2
#define OV7692_TEST_PATT_PN9		4

#define OV7692_MAX_FRAME_LENGTH_LINES	0xFFFF

#define SENSOR_DETECTED		1
#define SENSOR_NOT_DETECTED	0

#define NUM_IMAGE_SIZES ARRAY_SIZE(ov7692_sizes)
/**
 * struct ov7692_reg - ov7692 register format
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 * @length: length of the register
 *
 * Define a structure for OV7692 register initialization values
 */
struct ov7692_reg {
	u16 	reg;
	u8 	val;
};

enum ov7692_image_size {
	VGA,
};

#define NUM_IMAGE_SIZES ARRAY_SIZE(ov7692_sizes)
/**
 * struct ov7692_capture_size - image capture size information
 * @width: image width in pixels
 * @height: image height in pixels
 */
struct ov7692_capture_size {
	unsigned long width;
	unsigned long height;
};

/**
 * struct struct clk_settings - struct for storage of sensor
 * clock settings
 */
struct ov7692_clk_settings {
	u16	pre_pll_div;
	u16	pll_mult;
	u16  post_pll_div;
	u16	vt_pix_clk_div;
	u16	vt_sys_clk_div;
};

/**
 * struct struct mipi_settings - struct for storage of sensor
 * mipi settings
 */
struct ov7692_mipi_settings {
	u16	data_lanes;
	u16	ths_prepare;
	u16	ths_zero;
	u16	ths_settle_lower;
	u16	ths_settle_upper;
};

/**
 * struct struct frame_settings - struct for storage of sensor
 * frame settings
 */
struct ov7692_frame_settings {
	u16	frame_len_lines_min;
	u16	frame_len_lines;
	u16	line_len_pck;
	u16	x_addr_start;
	u16	x_addr_end;
	u16	y_addr_start;
	u16	y_addr_end;
	u16	x_output_size;
	u16	y_output_size;
	u16	x_even_inc;
	u16	x_odd_inc;
	u16	y_even_inc;
	u16	y_odd_inc;
	u16	v_mode_add;
	u16	h_mode_add;
	u16	h_add_ave;
};

/**
 * struct struct ov7692_sensor_settings - struct for storage of
 * sensor settings.
 */
struct ov7692_sensor_settings {
	struct ov7692_clk_settings clk;
	struct ov7692_mipi_settings mipi;
	struct ov7692_frame_settings frame;
};

/**
 * struct struct ov7692_clock_freq - struct for storage of sensor
 * clock frequencies
 */
struct ov7692_clock_freq {
	u32 vco_clk;
	u32 mipi_clk;
	u32 ddr_clk;
	u32 vt_pix_clk;
};

/**
 * Array of image sizes supported by OV7692.  These must be ordered from
 * smallest image size to largest.
 */
const static struct ov7692_capture_size ov7692_sizes[] = {
	{ 640, 480 },	/* VGA - 1/8 Vertical Elim */
};

/* PLL settings for ov7692 */
enum ov7692_pll_type {
	PLL_QUART_MP = 0,
	PLL_0_5MP,
	PLL_2MP,
	PLL_8MP,
};

#endif /* ifndef OV7692_REGS_H */
