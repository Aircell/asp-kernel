/* Copyright (c) 2011 - Aircel
 * Modified for the CloudSurfer by Tarr in 2011 
 * This is a little different and probably not a kosker design, but....
 *
 * The touchscreen is divided into two regions. The first acts as a standard
 * touchscreen that is overlying the display. The second acts as a keypad.
 * The full touch screen uses the Atmel xMT224 feature that has the controller
 * returning touch locations extrapolated to a value with in a configured range.
 * In this particular case, that range is calcuated based on the ppi density
 * of the display extended across the full length of the touch screen. In the
 * case of the specifics of the P2 Cloudsurfer, the display is a 800X480
 * screen. Extenteding it out to cover the full touchscreen yeilds a 1170X480
 * range. The interrupt handle determines if the touch was in the "touchscreen"
 * region or the keypad region. If it is in the "touchscreen" region, the
 * location is reported on up the ladder. If it is in the "keypad" region,
 * the handler determines which "key" was touched by wizing through a 
 * keypad layout structure. The identified "key" is then reported up the
 * ladder.
 *
 * The really poor part of this is that the range for the full touchscreen
 * is set in the board specific code (arch/arm/mach-omap2/board-omap3logic.c).
 * However, the keylayout map is defined here and is highly specific to
 * the display parameter and the touchscreen's size. My humble apology.
 *
 * Added note: The Atmel's Key area feature was tried and found completely
 * in adequate.
 * 
 */

/*
 * AT42QT602240/ATMXT224 Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#define DEBUG
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/qt602240_ts.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

/* Version */
#define QT602240_VER_20			20
#define QT602240_VER_21			21
#define QT602240_VER_22			22

/* Slave addresses */
#define QT602240_APP_LOW		0x4a
#define QT602240_APP_HIGH		0x4b
#define QT602240_BOOT_LOW		0x24
#define QT602240_BOOT_HIGH		0x25

/* Firmware */
#define QT602240_FW_NAME		"qt602240.fw"

/* Registers */
#define QT602240_FAMILY_ID		0x00
#define QT602240_VARIANT_ID		0x01
#define QT602240_VERSION		0x02
#define QT602240_BUILD			0x03
#define QT602240_MATRIX_X_SIZE	0x04
#define QT602240_MATRIX_Y_SIZE	0x05
#define QT602240_OBJECT_NUM		0x06
#define QT602240_OBJECT_START	0x07

#define QT602240_OBJECT_SIZE		6

/* Object types */
#define QT602240_DEBUG_DIAGNOSTIC	37
#define QT602240_GEN_MESSAGE		5
#define QT602240_GEN_COMMAND		6
#define QT602240_GEN_POWER			7
#define QT602240_GEN_ACQUIRE		8
#define QT602240_TOUCH_MULTI		9
#define QT602240_TOUCH_KEYARRAY		15
#define QT602240_TOUCH_PROXIMITY	23
#define QT602240_PROCI_GRIPFACE		20
#define QT602240_PROCG_NOISE		22
#define QT602240_PROCI_ONETOUCH		24
#define QT602240_PROCI_TWOTOUCH		27
#define QT602240_SPT_COMMSCONFIG	18	/* firmware ver 21 over */
#define QT602240_SPT_GPIOPWM		19
#define QT602240_SPT_SELFTEST		25
#define QT602240_SPT_CTECONFIG		28
#define QT602240_SPT_USERDATA		38	/* firmware ver 21 over */

/* QT602240_GEN_COMMAND field */
#define QT602240_COMMAND_RESET		0
#define QT602240_COMMAND_BACKUPNV	1
#define QT602240_COMMAND_CALIBRATE	2
#define QT602240_COMMAND_REPORTALL	3
#define QT602240_COMMAND_DIAGNOSTIC	5

/* QT602240_GEN_POWER field */
#define QT602240_POWER_IDLEACQINT	0
#define QT602240_POWER_ACTVACQINT	1
#define QT602240_POWER_ACTV2IDLETO	2

/* QT602240_GEN_ACQUIRE field */
#define QT602240_ACQUIRE_CHRGTIME	0
#define QT602240_ACQUIRE_TCHDRIFT	2
#define QT602240_ACQUIRE_DRIFTST	3
#define QT602240_ACQUIRE_TCHAUTOCAL	4
#define QT602240_ACQUIRE_SYNC		5
#define QT602240_ACQUIRE_ATCHCALST	6
#define QT602240_ACQUIRE_ATCHCALSTHR	7

/* QT602240_TOUCH_MULTI field */
#define QT602240_TOUCH_CTRL			0
#define QT602240_TOUCH_XORIGIN		1
#define QT602240_TOUCH_YORIGIN		2
#define QT602240_TOUCH_XSIZE		3
#define QT602240_TOUCH_YSIZE		4
#define QT602240_TOUCH_BLEN			6
#define QT602240_TOUCH_TCHTHR		7
#define QT602240_TOUCH_TCHDI		8
#define QT602240_TOUCH_ORIENT		9
#define QT602240_TOUCH_MOVHYSTI		11
#define QT602240_TOUCH_MOVHYSTN		12
#define QT602240_TOUCH_NUMTOUCH		14
#define QT602240_TOUCH_MRGHYST		15
#define QT602240_TOUCH_MRGTHR		16
#define QT602240_TOUCH_AMPHYST		17
#define QT602240_TOUCH_XRANGE_LSB	18
#define QT602240_TOUCH_XRANGE_MSB	19
#define QT602240_TOUCH_YRANGE_LSB	20
#define QT602240_TOUCH_YRANGE_MSB	21
#define QT602240_TOUCH_XLOCLIP		22
#define QT602240_TOUCH_XHICLIP		23
#define QT602240_TOUCH_YLOCLIP		24
#define QT602240_TOUCH_YHICLIP		25
#define QT602240_TOUCH_XEDGECTRL	26
#define QT602240_TOUCH_XEDGEDIST	27
#define QT602240_TOUCH_YEDGECTRL	28
#define QT602240_TOUCH_YEDGEDIST	29
#define QT602240_TOUCH_JUMPLIMIT	30	/* firmware ver 22 over */

/* QT602240_PROCI_GRIPFACE field */
#define QT602240_GRIPFACE_CTRL		0
#define QT602240_GRIPFACE_XLOGRIP	1
#define QT602240_GRIPFACE_XHIGRIP	2
#define QT602240_GRIPFACE_YLOGRIP	3
#define QT602240_GRIPFACE_YHIGRIP	4
#define QT602240_GRIPFACE_MAXTCHS	5
#define QT602240_GRIPFACE_SZTHR1	7
#define QT602240_GRIPFACE_SZTHR2	8
#define QT602240_GRIPFACE_SHPTHR1	9
#define QT602240_GRIPFACE_SHPTHR2	10
#define QT602240_GRIPFACE_SUPEXTTO	11

/* QT602240_PROCI_NOISE field */
#define QT602240_NOISE_CTRL			0
#define QT602240_NOISE_OUTFLEN		1
#define QT602240_NOISE_GCAFUL_LSB	3
#define QT602240_NOISE_GCAFUL_MSB	4
#define QT602240_NOISE_GCAFLL_LSB	5
#define QT602240_NOISE_GCAFLL_MSB	6
#define QT602240_NOISE_ACTVGCAFVALID	7
#define QT602240_NOISE_NOISETHR		8
#define QT602240_NOISE_FREQHOPSCALE	10
#define QT602240_NOISE_FREQ0		11
#define QT602240_NOISE_FREQ1		12
#define QT602240_NOISE_FREQ2		13
#define QT602240_NOISE_FREQ3		14
#define QT602240_NOISE_FREQ4		15
#define QT602240_NOISE_IDLEGCAFVALID	16

/* QT602240_SPT_COMMSCONFIG */
#define QT602240_COMMS_CTRL		0
#define QT602240_COMMS_CMD		1

/* QT602240_SPT_CTECONFIG field */
#define QT602240_CTE_CTRL		0
#define QT602240_CTE_CMD		1
#define QT602240_CTE_MODE		2
#define QT602240_CTE_IDLEGCAFDEPTH	3
#define QT602240_CTE_ACTVGCAFDEPTH	4
#define QT602240_CTE_VOLTAGE		5	/* firmware ver 21 over */

#define QT602240_VOLTAGE_DEFAULT	2700000
#define QT602240_VOLTAGE_STEP		10000

/* Define for QT602240_GEN_COMMAND */
#define QT602240_BOOT_VALUE		0xa5
#define QT602240_BACKUP_VALUE		0x55
#define QT602240_BACKUP_TIME		25	/* msec */
#define QT602240_RESET_TIME		65	/* msec */

#define QT602240_FWRESET_TIME		175	/* msec */

/* Command to unlock bootloader */
#define QT602240_UNLOCK_CMD_MSB		0xaa
#define QT602240_UNLOCK_CMD_LSB		0xdc

/* Bootloader mode status */
#define QT602240_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define QT602240_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define QT602240_FRAME_CRC_CHECK	0x02
#define QT602240_FRAME_CRC_FAIL		0x03
#define QT602240_FRAME_CRC_PASS		0x04
#define QT602240_APP_CRC_FAIL		0x40	/* valid 7 8 bit only */
#define QT602240_BOOT_STATUS_MASK	0x3f

/* Touch status */
#define QT602240_UNGRIP			(1 << 0)
#define QT602240_SUPPRESS		(1 << 1)
#define QT602240_AMP			(1 << 2)
#define QT602240_VECTOR			(1 << 3)
#define QT602240_MOVE			(1 << 4)
#define QT602240_RELEASE		(1 << 5)
#define QT602240_PRESS			(1 << 6)
#define QT602240_DETECT			(1 << 7)

/* Touchscreen absolute values */
#define QT602240_MAX_XC			0x3ff
#define QT602240_MAX_YC			0x3ff
#define QT602240_MAX_AREA		0xff

#define QT602240_MAX_FINGER		10

#define DISPLAY_START_OFFSET	17

/* Modified to support 19X11 touchscreen based on email from
 * Tu T. Phan at Touch Internaltional to AirCell on 24 Aug 2011
 */
static const u8 init_vals_ver_22[] = {
	/* QT602240_GEN_COMMAND(6) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* QT602240_GEN_POWER(7) */
	0xff, 0xff, 0xff,
	/* QT602240_GEN_ACQUIRE(8) */
	0x08, 0x05, 0x01, 0x01, 0x32, 0x00, 0x00, 0x00,
	/* QT602240_TOUCH_MULTI(9) */
	0x83, 0x00, 0x00, 0x13, 0x0b, 0x00, 0x17, 0x50, 0x02, 0x01,
	0x00, 0x01, 0x01, 0x00, 0x04, 0x10, 0x10, 0x10, 0xFF, 0x03,
	0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	/* QT602240_TOUCH_KEYARRAY(15) */
	0x00, 0x00, 0x00, 0x05, 0x03, 0x00, 0x41, 0x50, 0x02, 0x00,
	0x00,
	/* QT602240_SPT_GPIOPWM(19) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* QT602240_PROCI_GRIPFACE(20) */
	0x00, 0x64, 0x64, 0x64, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00,
	/* QT602240_PROCG_NOISE(22) */
	0x09, 0x00, 0x00, 0x0a, 0x00, 0x0a, 0x00, 0x04, 0x08, 0x00,
	0x01, 0x0a, 0x0f, 0x14, 0x19, 0x1e, 0x04,
	/* QT602240_TOUCH_PROXIMITY(23) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	/* QT602240_PROCI_ONETOUCH(24) */
	0x03, 0x0a, 0xFF, 0x03, 0x00, 0x64, 0x64, 0x01, 0x0a, 0x14,
	0x28, 0x00, 0x4b, 0x00, 0x02, 0x00, 0x64, 0x00, 0x19,
	/* QT602240_SPT_SELFTEST(25) */
	0x00, 0x00, 0xe0, 0x2e, 0x58, 0x1b, 0xB0, 0x36, 0xf4, 0x01,
	0x00, 0x00, 0x00, 0x00,
	/* QT602240_PROCI_TWOTOUCH(27) */
	0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* QT602240_SPT_CTECONFIG(28) */
	0x00, 0x00, 0x03, 0x04, 0x08, 0x00,
};

struct qt602240_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct qt602240_object {
	u8 type;
	u16 start_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;

	/* to map object and message */
	u8 max_reportid;
};

struct qt602240_message {
	u8 reportid;
	u8 message[7];
	u8 checksum;
};

struct key {
	int start_x;
	int end_x;
	int start_y;
	int end_y;
	int valid;
	int status;
	int code;
	char character;
};

struct key keypad[] = {
	{852, 964,0,153,1,0,KEY_BACK,'B'}, 		/* BACK key */
	{852, 964,170,307,1,0,KEY_HOME,'H'},	/* HOME key */
	{852, 964,323,479,1,0,KEY_MENU,'M'},	/* MENU key */

	{977, 1032,0,153,1,0,KEY_1,'1'}, 		/* 1 key */
	{977, 1032,170,307,1,0,KEY_2,'2'},		/* 2 key */
	{977, 1032,323,479,1,0,KEY_3,'3'},		/* 3 key */

	{1045, 1102,0,153,1,0,KEY_4,'4'}, 		/* 4 key */
	{1045, 1102,170,307,1,0,KEY_5,'5'},		/* 5 key */
	{1045, 1102,323,479,1,0,KEY_6,'6'},		/* 6 key */

	{1115, 1175,0,153,1,0,KEY_7,'7'},		/* 7 key */
	{1115, 1175,170,307,1,0,KEY_8,'8'},		/* 8 key */
	{1115, 1175,323,479,1,0,KEY_9,'9'},		/* 9 key */

	{1188, 1252,0,153,1,0,KEY_NUMERIC_STAR,'*'},/* * key */
	{1188, 1252,170,307,1,0,KEY_0,'0'},			/* 0 key */
	{1188, 1252,323,479,1,0,KEY_NUMERIC_POUND,'#'},	/* # key*/
	{0,0,0,0,0,0,0,0},					/* End of keys */
};

enum ACTIVE_DIVISION {
	UNTOUCHED = 0,
	ON_TOUCHSCREEN,
	ON_KEYPAD
};

struct qt602240_finger {
	int status;
	int x;
	int y;
	int area;
	struct key *k;
	enum ACTIVE_DIVISION where;
};

/* Each client has this additional data */
struct qt602240_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct qt602240_platform_data *pdata;
	struct qt602240_object *object_table;
	struct qt602240_info info;
	struct qt602240_finger finger[QT602240_MAX_FINGER];
	unsigned int irq;
};

/* 
 * Local function prototype
 */
static void qt602240_start(struct qt602240_data *data);


static bool qt602240_object_readable(unsigned int type)
{
	switch (type) {
	case QT602240_GEN_MESSAGE:
	case QT602240_GEN_COMMAND:
	case QT602240_GEN_POWER:
	case QT602240_GEN_ACQUIRE:
	case QT602240_TOUCH_MULTI:
	case QT602240_TOUCH_KEYARRAY:
	case QT602240_TOUCH_PROXIMITY:
	case QT602240_PROCI_GRIPFACE:
	case QT602240_PROCG_NOISE:
	case QT602240_PROCI_ONETOUCH:
	case QT602240_PROCI_TWOTOUCH:
	case QT602240_SPT_COMMSCONFIG:
	case QT602240_SPT_GPIOPWM:
	case QT602240_SPT_SELFTEST:
	case QT602240_SPT_CTECONFIG:
	case QT602240_SPT_USERDATA:
		return true;
	default:
		return false;
	}
}

static bool qt602240_object_writable(unsigned int type)
{
	switch (type) {
	case QT602240_GEN_COMMAND:
	case QT602240_GEN_POWER:
	case QT602240_GEN_ACQUIRE:
	case QT602240_TOUCH_MULTI:
	case QT602240_TOUCH_KEYARRAY:
	case QT602240_TOUCH_PROXIMITY:
	case QT602240_PROCI_GRIPFACE:
	case QT602240_PROCG_NOISE:
	case QT602240_PROCI_ONETOUCH:
	case QT602240_PROCI_TWOTOUCH:
	case QT602240_SPT_GPIOPWM:
	case QT602240_SPT_SELFTEST:
	case QT602240_SPT_CTECONFIG:
		return true;
	default:
		return false;
	}
}

static void qt602240_dump_message(struct device *dev,
				  struct qt602240_message *message)
{
	printk("QT - reportid:\t0x%x\n", message->reportid);
	printk("QT - message1:\t0x%x\n", message->message[0]);
	printk("QT - message2:\t0x%x\n", message->message[1]);
	printk("QT - message3:\t0x%x\n", message->message[2]);
	printk("QT - message4:\t0x%x\n", message->message[3]);
	printk("QT - message5:\t0x%x\n", message->message[4]);
	printk("QT - message6:\t0x%x\n", message->message[5]);
	printk("QT - message7:\t0x%x\n", message->message[6]);
	printk("QT - checksum:\t0x%x\n", message->checksum);
}

static int qt602240_check_bootloader(struct i2c_client *client,
				     unsigned int state)
{
	u8 val;

recheck:
	if (i2c_master_recv(client, &val, 1) != 1) {
		dev_err(&client->dev, "%s: i2c recv failed\n", __func__);
		return -EIO;
	}

	switch (state) {
	case QT602240_WAITING_BOOTLOAD_CMD:
	case QT602240_WAITING_FRAME_DATA:
		val &= ~QT602240_BOOT_STATUS_MASK;
		break;
	case QT602240_FRAME_CRC_PASS:
		if (val == QT602240_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(&client->dev, "Unvalid bootloader mode state\n");
		return -EINVAL;
	}

	return 0;
}

static int qt602240_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = QT602240_UNLOCK_CMD_LSB;
	buf[1] = QT602240_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int qt602240_fw_write(struct i2c_client *client,
			     const u8 *data, unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int __qt602240_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	if (i2c_transfer(client->adapter, xfer, 2) != 2) {
		dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int qt602240_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	return __qt602240_read_reg(client, reg, 1, val);
}

static int qt602240_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = val;

	if (i2c_master_send(client, buf, 3) != 3) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int qt602240_read_object_table(struct i2c_client *client,
				      u16 reg, u8 *object_buf)
{
	return __qt602240_read_reg(client, reg, QT602240_OBJECT_SIZE,
				   object_buf);
}

static struct qt602240_object *qt602240_get_object(struct qt602240_data *data, 
			u8 type)
{
	struct qt602240_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	//dev_err(&data->client->dev, "Invalid object type\n");
	return NULL;
}

#ifdef QT_DEBUG
static int qt602240_dump_object(struct qt602240_data *data, u8 type)
{
	u16 reg;
	u8 val;
	int i;

	struct qt602240_object *object;
	object = qt602240_get_object(data, type);
	if ( object == NULL )
		return 0;
	printk("QT - Object %d dump\n\t",type);
	reg = object->start_address;
	for ( i=0; i<=object->size; i++ ) {
		__qt602240_read_reg(data->client, reg + i, 1, &val);
		printk("0x%2.2X ",val);
	}
	printk("\n");
	return 1;
}
#endif	
		
static int qt602240_read_message(struct qt602240_data *data,
				 struct qt602240_message *message)
{
	struct qt602240_object *object;
	u16 reg;

	object = qt602240_get_object(data, QT602240_GEN_MESSAGE);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __qt602240_read_reg(data->client, reg,
			sizeof(struct qt602240_message), message);
}

static int qt602240_read_object(struct qt602240_data *data,
				u8 type, u8 offset, u8 *val)
{
	struct qt602240_object *object;
	u16 reg;

	object = qt602240_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __qt602240_read_reg(data->client, reg + offset, 1, val);
}

static int qt602240_write_object(struct qt602240_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct qt602240_object *object;
	u16 reg;

	object = qt602240_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return qt602240_write_reg(data->client, reg + offset, val);
}

static void qt602240_input_report(struct qt602240_data *data, int single_id)
{
	struct qt602240_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;
	int status = finger[single_id].status;
	int finger_num = 0;
	int id;

	for (id = 0; id < QT602240_MAX_FINGER; id++) {
		if (finger[id].where != ON_TOUCHSCREEN)
			continue;
#ifdef TARR_DEBUG
		printk("QT - [%d] x: %d y: %d %s\n",id,
			finger[id].x,finger[id].y,
			status & QT602240_MOVE ? "moved" : "pressed");
#endif
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
				finger[id].status != QT602240_RELEASE ?
				finger[id].area + 1 : 0);
		input_report_abs(input_dev, ABS_MT_POSITION_X,
				finger[id].x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y,
				finger[id].y);
		input_mt_sync(input_dev);

		if (finger[id].status == QT602240_RELEASE) {
			finger[id].status = 0;
			finger[id].where = UNTOUCHED;
		} else {
			finger_num++;
		}
	}

	input_report_key(input_dev, BTN_TOUCH, finger_num > 0);

	if (status != QT602240_RELEASE) {
		input_report_abs(input_dev, ABS_X, finger[single_id].x);
		input_report_abs(input_dev, ABS_Y, finger[single_id].y);
	}

	input_sync(input_dev);
}

/*
 * keypad_input looks up the key by raw x and y values, and calls report_key
 */
static struct key* find_key_on_keypad(int rawx, int rawy)
{
	struct key *k = &keypad[0];
	int index = 0;

	/* Walk through the keypad definition table to find the key */	
	while ( k->start_x != 0 ) {
		if ( k->start_x <= rawx && k->end_x >= rawx &&
				 k->start_y <= rawy && k->end_y >= rawy ) {
			// Is it an active key?
			if ( k->valid ) {
				return k; // Yep
			}
			return NULL; // Nope
		}
		index++;
		k++;
	}
	return NULL;
}

static void report_key(struct qt602240_data *data,
					   struct qt602240_finger *finger)
{
	struct input_dev *input_dev = data->input_dev;

	if (finger->status == QT602240_PRESS) {
		if (finger->k != NULL) {
			// Should not happen -- press not preceded by release
			input_report_key(input_dev, finger->k->code, 0);
			input_sync(input_dev);
			finger->k = NULL;
		}
		finger->k = find_key_on_keypad(finger->x, finger->y);
		if (finger->k != NULL) {
			input_report_key(input_dev, finger->k->code, 1);
			input_sync(input_dev);
			printk("KEY - %c pressed\n",finger->k->character);
		}

	} else if (finger->status == QT602240_MOVE) {
		// If a key is held down, report key up if we move outside the key's box
		if (finger->k != NULL) {
			struct key *newkey = find_key_on_keypad(finger->x, finger->y);
			if (newkey != finger->k) {
				input_report_key(input_dev, finger->k->code, 0);
				input_sync(input_dev);
				finger->k = NULL;
			}
		}

	} else if (finger->status == QT602240_RELEASE) {
		if (finger->k != NULL) {
			input_report_key(input_dev, finger->k->code, 0);
			input_sync(input_dev);
		}
		finger->k = NULL;
		finger->where = UNTOUCHED;

	} // No other possibilities for finger->status
}

static void qt602240_input_touchevent(struct qt602240_data *data,
				      struct qt602240_message *message, int id)
{
	struct qt602240_finger *finger = data->finger + id;
	//struct input_dev *input_dev = data->input_dev;
	u8 status = message->message[0];
	int x;
	int y;
	int area;

#ifdef TARR_DEBUG
	printk("QT - input [%d] %2.2X:%2.2X:%2.2X:%2.2X:%2.2X\n",
			id,
			message->message[0], message->message[1],
			message->message[2], message->message[3],
			message->message[4]);
#endif

	/* Tarr - x is reported in 12 bits, y is reported in 10 due to 
       scaling size difference */
	x = (message->message[1] << 4) | ((message->message[3] & ~0x0f) >> 4);
	/* Tarr - Because the touchscreen got "flipped" in the P3, need to
     * adjust x accordingly
	 */
    x = 1252 - x;
	y = (message->message[2] << 2) | ((message->message[3] & ~0xf3) >> 2);
	area = message->message[4];

	//printk("touch at %dx%d\n",x,y);
	//TARR - Touch screen starts before display

	// Manage fingers data, where each finger is tracked separately

	// Simplify message status
	if (status & (QT602240_RELEASE | QT602240_SUPPRESS)) {
		finger->status = QT602240_RELEASE;
	} else if (status & (QT602240_UNGRIP | QT602240_PRESS)) {
		finger->status = QT602240_PRESS;
	} else if (status & QT602240_MOVE) {
		finger->status = QT602240_MOVE;
	} else {
		// We don't care
		return;
	}

	//printk("Finger %d status %x where %d key %p\n", id, status, finger->where, finger->k);

	if (finger->where == UNTOUCHED) {
		// first report on this finger since up -- call it keypad or touchscreen
		// If a finger starts in one area, it keeps reporting there until up.
		if (x > 800+DISPLAY_START_OFFSET) {
			/* TARR - the keypad is at the "lower" portion of the screen */
			finger->where = ON_KEYPAD;
			//printk("qt602240 - finger %d on keypad\n", id);
		} else {
			finger->where = ON_TOUCHSCREEN;
			//printk("qt602240 - finger %d on touchscreen\n", id);
		}
	}

	if (finger->where == ON_KEYPAD) {
		// Store raw X and Y values
		finger->x = x;
		finger->y = y;
		report_key(data, finger);
	} else {
		// Must be ON_TOUCHSCREEN
		
		//TARR - need to remove an offset
		if (x > DISPLAY_START_OFFSET) {
			x -= DISPLAY_START_OFFSET;
		} else {
			x = 0;
		}

		/* TARR - P1 wierdness due to display rotated 180 */
		finger->x = y;
		//finger->x = 480-y;
		finger->y = x;
		//finger->y = 800-x;
		finger->area = area;

		qt602240_input_report(data, id);
	}
}

static irqreturn_t qt602240_interrupt(int irq, void *dev_id)
{
	struct qt602240_data *data = dev_id;
	struct qt602240_message message;
	struct qt602240_object *object;
	struct device *dev = &data->client->dev;
	int id;
	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;

	do {
		if (qt602240_read_message(data, &message)) {
			dev_err(dev, "Failed to read message\n");
			goto end;
		}

		reportid = message.reportid;

		/* whether reportid is thing of QT602240_TOUCH_MULTI */
		object = qt602240_get_object(data, QT602240_TOUCH_MULTI);
		if (!object)
			goto end;

		max_reportid = object->max_reportid;
		min_reportid = max_reportid - object->num_report_ids + 1;
		id = reportid - min_reportid;

		if (reportid >= min_reportid && reportid <= max_reportid) {
			qt602240_input_touchevent(data, &message, id);
		} else {
#ifdef TARR_DEBUG
			printk("QT - interrupt [%d] %2.2X:%2.2X:%2.2X:%2.2X:%2.2X\n",
				id,
				message.message[0], message.message[1],
				message.message[2], message.message[3],
				message.message[4])
#endif
			;
		}
	} while (reportid != 0xff);

end:
	return IRQ_HANDLED;
}

static int qt602240_check_reg_init(struct qt602240_data *data)
{
	struct qt602240_object *object;
	struct qt602240_message status;
	struct device *dev = &data->client->dev;
	int index = 0;
	int i, j;
	u8 version = data->info.version;
	u8 *init_vals;

	switch (version) {
	case QT602240_VER_22:
		init_vals = (u8 *)init_vals_ver_22;
		break;
	default:
		dev_err(dev, "Firmware version %d not supported\n", version);
		return -EINVAL;
	}
	/* Dump all current messages */
	for ( i=0; i<4; i++ ) {
		qt602240_read_message(data,&status);
	}
		

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		if (!qt602240_object_writable(object->type))
			continue;

		for (j = 0; j < object->size + 1; j++) {
			qt602240_write_object(data, object->type, j,
					init_vals[index + j]);

			/* Not supposed ot poll for messages, however
 			 * don't want to enable the interrupt during configuration.....
			 * So, we poll
			 */
			qt602240_read_message(data,&status);
			if (status.reportid == 0xFF )
				continue;
			printk("QT - configuration error for object type %d\n",object->type);
			qt602240_dump_message(dev,&status);	

		}
		index += object->size + 1;
	}

	return 0;
}

static int qt602240_check_matrix_size(struct qt602240_data *data)
{
	const struct qt602240_platform_data *pdata = data->pdata;
	struct device *dev = &data->client->dev;
	int mode = -1;
	int error;
	u8 val;

	dev_dbg(dev, "Number of X lines: %d\n", pdata->x_line);
	dev_dbg(dev, "Number of Y lines: %d\n", pdata->y_line);

	switch (pdata->x_line) {
	case 0 ... 15:
		if (pdata->y_line <= 14)
			mode = 0;
		break;
	case 16:
		if (pdata->y_line <= 12)
			mode = 1;
		if (pdata->y_line == 13 || pdata->y_line == 14)
			mode = 0;
		break;
	case 17:
		if (pdata->y_line <= 11)
			mode = 2;
		if (pdata->y_line == 12 || pdata->y_line == 13)
			mode = 1;
		break;
	case 18:
		if (pdata->y_line <= 10)
			mode = 3;
		if (pdata->y_line == 11 || pdata->y_line == 12)
			mode = 2;
		break;
	case 19:
		if (pdata->y_line <= 9)
			mode = 4;
		if (pdata->y_line == 10 || pdata->y_line == 11)
			mode = 3;
		break;
	case 20:
		mode = 4;
	}

	if (mode < 0) {
		dev_err(dev, "Invalid X/Y lines\n");
		return -EINVAL;
	}

	error = qt602240_read_object(data, QT602240_SPT_CTECONFIG,
				QT602240_CTE_MODE, &val);
	if (error)
		return error;

	if (mode == val)
		return 0;

	/* Change the CTE configuration */
	qt602240_write_object(data, QT602240_SPT_CTECONFIG,
			QT602240_CTE_CTRL, 1);
	qt602240_write_object(data, QT602240_SPT_CTECONFIG,
			QT602240_CTE_MODE, mode);
	qt602240_write_object(data, QT602240_SPT_CTECONFIG,
			QT602240_CTE_CTRL, 0);

	return 0;
}

static int qt602240_make_highchg(struct qt602240_data *data)
{
	qt602240_write_object(data, QT602240_SPT_COMMSCONFIG,
			1, 2);

#ifdef ORIGINAL
	struct device *dev = &data->client->dev;
	int count = 10;
	int error;
	u8 val;

	/* Read dummy message to make high CHG pin */
	do {
		error = qt602240_read_object(data, QT602240_GEN_MESSAGE, 0, &val);
		if (error)
			return error;
	} while ((val != 0xff) && --count);

	if (!count) {
		dev_err(dev, "CHG pin isn't cleared\n");
		return -EBUSY;
	}
#endif
	return 0;
}

static void qt602240_handle_pdata(struct qt602240_data *data)
{
	const struct qt602240_platform_data *pdata = data->pdata;
	u8 voltage;

	/* Set touchscreen lines */
	qt602240_write_object(data, QT602240_TOUCH_MULTI, QT602240_TOUCH_XSIZE,
			pdata->x_line);
	qt602240_write_object(data, QT602240_TOUCH_MULTI, QT602240_TOUCH_YSIZE,
			pdata->y_line);

	/* Set touchscreen orient */
	qt602240_write_object(data, QT602240_TOUCH_MULTI, QT602240_TOUCH_ORIENT,
			pdata->orient);

	/* Set touchscreen burst length */
	qt602240_write_object(data, QT602240_TOUCH_MULTI,
			QT602240_TOUCH_BLEN, pdata->blen);

	/* Set touchscreen threshold */
	qt602240_write_object(data, QT602240_TOUCH_MULTI,
			QT602240_TOUCH_TCHTHR, pdata->threshold);

	/* Set touchscreen resolution */
	qt602240_write_object(data, QT602240_TOUCH_MULTI,
			QT602240_TOUCH_XRANGE_LSB, (pdata->x_size - 1) & 0xff);
	qt602240_write_object(data, QT602240_TOUCH_MULTI,
			QT602240_TOUCH_XRANGE_MSB, (pdata->x_size - 1) >> 8);
	qt602240_write_object(data, QT602240_TOUCH_MULTI,
			QT602240_TOUCH_YRANGE_LSB, (pdata->y_size - 1) & 0xff);
	qt602240_write_object(data, QT602240_TOUCH_MULTI,
			QT602240_TOUCH_YRANGE_MSB, (pdata->y_size - 1) >> 8);

	/* Set touchscreen voltage */
	if (data->info.version >= QT602240_VER_21 && pdata->voltage) {
		if (pdata->voltage < QT602240_VOLTAGE_DEFAULT) {
			voltage = (QT602240_VOLTAGE_DEFAULT - pdata->voltage) /
				QT602240_VOLTAGE_STEP;
			voltage = 0xff - voltage + 1;
		} else
			voltage = (pdata->voltage - QT602240_VOLTAGE_DEFAULT) /
				QT602240_VOLTAGE_STEP;

		qt602240_write_object(data, QT602240_SPT_CTECONFIG,
				QT602240_CTE_VOLTAGE, voltage);
	}
}

static int qt602240_get_info(struct qt602240_data *data)
{
	struct i2c_client *client = data->client;
	struct qt602240_info *info = &data->info;
	int error;
	u8 val;

	error = qt602240_read_reg(client, QT602240_FAMILY_ID, &val);
	if (error)
		return error;
	info->family_id = val;

	error = qt602240_read_reg(client, QT602240_VARIANT_ID, &val);
	if (error)
		return error;
	info->variant_id = val;

	error = qt602240_read_reg(client, QT602240_VERSION, &val);
	if (error)
		return error;
	info->version = val;

	error = qt602240_read_reg(client, QT602240_BUILD, &val);
	if (error)
		return error;
	info->build = val;

	error = qt602240_read_reg(client, QT602240_OBJECT_NUM, &val);
	if (error)
		return error;
	info->object_num = val;

	return 0;
}

static int qt602240_get_object_table(struct qt602240_data *data)
{
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[QT602240_OBJECT_SIZE];

	for (i = 0; i < data->info.object_num; i++) {
		struct qt602240_object *object = data->object_table + i;

		reg = QT602240_OBJECT_START + QT602240_OBJECT_SIZE * i;
		error = qt602240_read_object_table(data->client, reg, buf);
		if (error)
			return error;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		object->size = buf[3];
		object->instances = buf[4];
		object->num_report_ids = buf[5];

		if (object->num_report_ids) {
			reportid += object->num_report_ids *
					(object->instances + 1);
			object->max_reportid = reportid;
		}
	}

	return 0;
}

static int qt602240_initialize(struct qt602240_data *data)
{
	struct i2c_client *client = data->client;
	struct qt602240_info *info = &data->info;
	int error;
	u8 val;

	//printk("QT - qt602240 initialize()\n");

	error = qt602240_get_info(data);
	if (error)
		return error;

	data->object_table = kcalloc(info->object_num,
				     sizeof(struct qt602240_object), GFP_KERNEL);

	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = qt602240_get_object_table(data);
	if (error)
		return error;

	/* Check register init values */
	error = qt602240_check_reg_init(data);
	if (error)
		return error;

	/* Check X/Y matrix size */
	error = qt602240_check_matrix_size(data);
	if (error)
		return error;

	error = qt602240_make_highchg(data);
	if (error)
		return error;

	qt602240_handle_pdata(data);

	/* Backup to memory */
	qt602240_write_object(data, QT602240_GEN_COMMAND,
			QT602240_COMMAND_BACKUPNV,
			QT602240_BACKUP_VALUE);
	msleep(QT602240_BACKUP_TIME);

	/* Soft reset */
	qt602240_write_object(data, QT602240_GEN_COMMAND,
			QT602240_COMMAND_RESET, 1);
	msleep(QT602240_RESET_TIME);

	/* Update matrix size at info struct */
	error = qt602240_read_reg(client, QT602240_MATRIX_X_SIZE, &val);
	if (error)
		return error;
	info->matrix_xsize = val;

	error = qt602240_read_reg(client, QT602240_MATRIX_Y_SIZE, &val);
	if (error)
		return error;
	info->matrix_ysize = val;

	dev_info(&client->dev,
			"Family ID: %d Variant ID: %d Version: %d Build: %d\n",
			info->family_id, info->variant_id, info->version,
			info->build);

	dev_info(&client->dev,
			"Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);

    qt602240_read_object(data, QT602240_TOUCH_MULTI,
                QT602240_TOUCH_CTRL, &val);

	return 0;
}

static ssize_t qt602240_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct qt602240_data *data = dev_get_drvdata(dev);
	struct qt602240_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 val;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		count += sprintf(buf + count,
				"Object Table Element %d(Type %d)\n",
				i + 1, object->type);

		if (!qt602240_object_readable(object->type)) {
			count += sprintf(buf + count, "\n");
			continue;
		}

		for (j = 0; j < object->size + 1; j++) {
			error = qt602240_read_object(data,
						object->type, j, &val);
			if (error)
				return error;

			count += sprintf(buf + count,
					"  Byte %d: 0x%x (%d)\n", j, val, val);
		}

		count += sprintf(buf + count, "\n");
	}

	return count;
}

static int qt602240_load_fw(struct device *dev, const char *fn)
{
	struct qt602240_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	int ret;

	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		return ret;
	}

	/* Change to the bootloader mode */
	qt602240_write_object(data, QT602240_GEN_COMMAND,
			QT602240_COMMAND_RESET, QT602240_BOOT_VALUE);
	msleep(QT602240_RESET_TIME);

	/* Change to slave address of bootloader */
	if (client->addr == QT602240_APP_LOW)
		client->addr = QT602240_BOOT_LOW;
	else
		client->addr = QT602240_BOOT_HIGH;

	ret = qt602240_check_bootloader(client, QT602240_WAITING_BOOTLOAD_CMD);
	if (ret)
		goto out;

	/* Unlock bootloader */
	qt602240_unlock_bootloader(client);

	while (pos < fw->size) {
		ret = qt602240_check_bootloader(client,
						QT602240_WAITING_FRAME_DATA);
		if (ret)
			goto out;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* We should add 2 at frame size as the the firmware data is not
		 * included the CRC bytes.
		 */
		frame_size += 2;

		/* Write one frame to device */
		qt602240_fw_write(client, fw->data + pos, frame_size);

		ret = qt602240_check_bootloader(client,
						QT602240_FRAME_CRC_PASS);
		if (ret)
			goto out;

		pos += frame_size;

		dev_dbg(dev, "Updated %d bytes / %zd bytes\n", pos, fw->size);
	}

out:
	release_firmware(fw);

	/* Change to slave address of application */
	if (client->addr == QT602240_BOOT_LOW)
		client->addr = QT602240_APP_LOW;
	else
		client->addr = QT602240_APP_HIGH;

	return ret;
}

static ssize_t qt602240_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct qt602240_data *data = dev_get_drvdata(dev);
	unsigned int version;
	int error;

	if (sscanf(buf, "%u", &version) != 1) {
		dev_err(dev, "Invalid values\n");
		return -EINVAL;
	}

	if (data->info.version < QT602240_VER_21 || version < QT602240_VER_21) {
		dev_err(dev, "FW update supported starting with version 21\n");
		return -EINVAL;
	}

	disable_irq(data->irq);

	error = qt602240_load_fw(dev, QT602240_FW_NAME);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_dbg(dev, "The firmware update succeeded\n");

		/* Wait for reset */
		msleep(QT602240_FWRESET_TIME);

		kfree(data->object_table);
		data->object_table = NULL;

		qt602240_initialize(data);
	}

	enable_irq(data->irq);

	return count;
}

static DEVICE_ATTR(object, 0444, qt602240_object_show, NULL);
static DEVICE_ATTR(update_fw, 0664, NULL, qt602240_update_fw_store);

static struct attribute *qt602240_attrs[] = {
	&dev_attr_object.attr,
	&dev_attr_update_fw.attr,
	NULL
};

static const struct attribute_group qt602240_attr_group = {
	.attrs = qt602240_attrs,
};

static void qt602240_start(struct qt602240_data *data)
{
	int i;
	struct qt602240_message status;

	// Initialize parts of finger array
	for (i = 0; i < QT602240_MAX_FINGER; i++) {
		data->finger[i].where = UNTOUCHED;
		data->finger[i].k = NULL;
	}

	//printk("QT - Touchscreen START\n");

	/* MultiTouch start  */
	qt602240_write_object(data,
			QT602240_TOUCH_MULTI, QT602240_TOUCH_CTRL, 0x83);
	qt602240_write_object(data,
			QT602240_PROCI_ONETOUCH, QT602240_TOUCH_CTRL, 0x03);

#ifdef TARR
	qt602240_write_object(data,
			QT602240_TOUCH_KEYARRAY, QT602240_TOUCH_CTRL, 0x03);
#endif

	msleep(100);
	   /* Dump all current messages */
    for ( i=0; i<4; i++ ) {
        qt602240_read_message(data,&status);
        //qt602240_dump_message(data,&status);
    }

#ifdef QT_DEBUG
	printk("Dumping Touch Configration\n");
	for (i=0; i<=38; i++ ) {
		qt602240_dump_object(data,i);
	}	
#endif
	
}

static void qt602240_stop(struct qt602240_data *data)
{
	/* Touch disable */
	qt602240_write_object(data,
			QT602240_TOUCH_MULTI, QT602240_TOUCH_CTRL, 0);
}

static int qt602240_input_open(struct input_dev *dev)
{
	struct qt602240_data *data = input_get_drvdata(dev);

	qt602240_start(data);

	return 0;
}

static void qt602240_input_close(struct input_dev *dev)
{
	struct qt602240_data *data = input_get_drvdata(dev);

	qt602240_stop(data);
}

static int __devinit qt602240_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct qt602240_data *data;
	struct input_dev *input_dev;
	int error;
	struct key *k;

	if (!client->dev.platform_data)
		return -EINVAL;

	data = kzalloc(sizeof(struct qt602240_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	input_dev->name = "AT42QT602240-Touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = qt602240_input_open;
	input_dev->close = qt602240_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->keybit);
	__set_bit(EV_REL, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	/* Setup the keypad */
	for (k=&keypad[0]; k->start_x != 0; k++ ) {
		__set_bit(k->code,input_dev->keybit);
	}	

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, QT602240_MAX_XC, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, QT602240_MAX_YC, 0, 0);

	/* For multi touch */
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, QT602240_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, QT602240_MAX_XC, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, QT602240_MAX_YC, 0, 0);

	input_set_drvdata(input_dev, data);

	data->client = client;
	data->input_dev = input_dev;
	data->pdata = client->dev.platform_data;
	data->irq = client->irq;

	i2c_set_clientdata(client, data);

	error = qt602240_initialize(data);
	if (error)
		goto err_free_object;

	error = request_threaded_irq(client->irq, NULL, qt602240_interrupt,
			IRQF_TRIGGER_FALLING, client->dev.driver->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}

	error = input_register_device(input_dev);
	if (error)
		goto err_free_irq;

	error = sysfs_create_group(&client->dev.kobj, &qt602240_attr_group);
	if (error)
		goto err_unregister_device;

	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, data);
err_free_object:
	kfree(data->object_table);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
	return error;
}

static int __devexit qt602240_remove(struct i2c_client *client)
{
	struct qt602240_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &qt602240_attr_group);
	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
	kfree(data->object_table);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int qt602240_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	printk("TARR - %s\n",__FUNCTION__);
#ifdef TARR
	struct qt602240_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		qt602240_stop(data);

	mutex_unlock(&input_dev->mutex);
#else
    set_irq_wake(client->irq, 1);
    disable_irq(client->irq);
#endif
	return 0;
}

static int qt602240_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	printk("TARR - %s\n",__FUNCTION__);
#ifdef TARR
	struct qt602240_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	/* Soft reset */
	qt602240_write_object(data, QT602240_GEN_COMMAND,
			QT602240_COMMAND_RESET, 1);

	msleep(QT602240_RESET_TIME);

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		qt602240_start(data);

	mutex_unlock(&input_dev->mutex);
#else
    enable_irq(client->irq);
    set_irq_wake(client->irq, 0);
#endif
	return 0;
}

static const struct dev_pm_ops qt602240_pm_ops = {
	.suspend	= qt602240_suspend,
	.resume		= qt602240_resume,
};
#endif

static const struct i2c_device_id qt602240_id[] = {
	{ "qt602240_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, qt602240_id);

static struct i2c_driver qt602240_driver = {
	.driver = {
		.name	= "qt602240_ts",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &qt602240_pm_ops,
#endif
	},
	.probe		= qt602240_probe,
	.remove		= __devexit_p(qt602240_remove),
	.id_table	= qt602240_id,
};

static int __init qt602240_init(void)
{
	return i2c_add_driver(&qt602240_driver);
}

static void __exit qt602240_exit(void)
{
	i2c_del_driver(&qt602240_driver);
}

module_init(qt602240_init);
module_exit(qt602240_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("AT42QT602240/ATMXT224 Touchscreen driver");
MODULE_LICENSE("GPL");
