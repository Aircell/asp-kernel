/*
 * Copyright © 2004 Texas Instruments, Jian Zhang <jzhang@ti.com>
 * Copyright © 2004 Micron Technology Inc.
 * Copyright © 2004 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/proc_fs.h>
#include <linux/leds.h>

#include <plat/dma.h>
#include <plat/gpmc.h>
#include <plat/nand.h>

#define GPMC_IRQ_STATUS		0x18
#define GPMC_IRQ_ENABLE		0x1C
#define GPMC_ECC_CONFIG		0x1F4
#define GPMC_ECC_CONTROL	0x1F8
#define GPMC_ECC_SIZE_CONFIG	0x1FC
#define GPMC_ECC1_RESULT	0x200

#define	DRIVER_NAME	"omap2-nand"

#define	NAND_WP_OFF	0
#define NAND_WP_BIT	0x00000010

#define	GPMC_BUF_FULL	0x00000001
#define	GPMC_BUF_EMPTY	0x00000000

#define NAND_Ecc_P1e		(1 << 0)
#define NAND_Ecc_P2e		(1 << 1)
#define NAND_Ecc_P4e		(1 << 2)
#define NAND_Ecc_P8e		(1 << 3)
#define NAND_Ecc_P16e		(1 << 4)
#define NAND_Ecc_P32e		(1 << 5)
#define NAND_Ecc_P64e		(1 << 6)
#define NAND_Ecc_P128e		(1 << 7)
#define NAND_Ecc_P256e		(1 << 8)
#define NAND_Ecc_P512e		(1 << 9)
#define NAND_Ecc_P1024e		(1 << 10)
#define NAND_Ecc_P2048e		(1 << 11)

#define NAND_Ecc_P1o		(1 << 16)
#define NAND_Ecc_P2o		(1 << 17)
#define NAND_Ecc_P4o		(1 << 18)
#define NAND_Ecc_P8o		(1 << 19)
#define NAND_Ecc_P16o		(1 << 20)
#define NAND_Ecc_P32o		(1 << 21)
#define NAND_Ecc_P64o		(1 << 22)
#define NAND_Ecc_P128o		(1 << 23)
#define NAND_Ecc_P256o		(1 << 24)
#define NAND_Ecc_P512o		(1 << 25)
#define NAND_Ecc_P1024o		(1 << 26)
#define NAND_Ecc_P2048o		(1 << 27)

#define TF(value)	(value ? 1 : 0)

#define P2048e(a)	(TF(a & NAND_Ecc_P2048e)	<< 0)
#define P2048o(a)	(TF(a & NAND_Ecc_P2048o)	<< 1)
#define P1e(a)		(TF(a & NAND_Ecc_P1e)		<< 2)
#define P1o(a)		(TF(a & NAND_Ecc_P1o)		<< 3)
#define P2e(a)		(TF(a & NAND_Ecc_P2e)		<< 4)
#define P2o(a)		(TF(a & NAND_Ecc_P2o)		<< 5)
#define P4e(a)		(TF(a & NAND_Ecc_P4e)		<< 6)
#define P4o(a)		(TF(a & NAND_Ecc_P4o)		<< 7)

#define P8e(a)		(TF(a & NAND_Ecc_P8e)		<< 0)
#define P8o(a)		(TF(a & NAND_Ecc_P8o)		<< 1)
#define P16e(a)		(TF(a & NAND_Ecc_P16e)		<< 2)
#define P16o(a)		(TF(a & NAND_Ecc_P16o)		<< 3)
#define P32e(a)		(TF(a & NAND_Ecc_P32e)		<< 4)
#define P32o(a)		(TF(a & NAND_Ecc_P32o)		<< 5)
#define P64e(a)		(TF(a & NAND_Ecc_P64e)		<< 6)
#define P64o(a)		(TF(a & NAND_Ecc_P64o)		<< 7)

#define P128e(a)	(TF(a & NAND_Ecc_P128e)		<< 0)
#define P128o(a)	(TF(a & NAND_Ecc_P128o)		<< 1)
#define P256e(a)	(TF(a & NAND_Ecc_P256e)		<< 2)
#define P256o(a)	(TF(a & NAND_Ecc_P256o)		<< 3)
#define P512e(a)	(TF(a & NAND_Ecc_P512e)		<< 4)
#define P512o(a)	(TF(a & NAND_Ecc_P512o)		<< 5)
#define P1024e(a)	(TF(a & NAND_Ecc_P1024e)	<< 6)
#define P1024o(a)	(TF(a & NAND_Ecc_P1024o)	<< 7)

#define P8e_s(a)	(TF(a & NAND_Ecc_P8e)		<< 0)
#define P8o_s(a)	(TF(a & NAND_Ecc_P8o)		<< 1)
#define P16e_s(a)	(TF(a & NAND_Ecc_P16e)		<< 2)
#define P16o_s(a)	(TF(a & NAND_Ecc_P16o)		<< 3)
#define P1e_s(a)	(TF(a & NAND_Ecc_P1e)		<< 4)
#define P1o_s(a)	(TF(a & NAND_Ecc_P1o)		<< 5)
#define P2e_s(a)	(TF(a & NAND_Ecc_P2e)		<< 6)
#define P2o_s(a)	(TF(a & NAND_Ecc_P2o)		<< 7)

#define P4e_s(a)	(TF(a & NAND_Ecc_P4e)		<< 0)
#define P4o_s(a)	(TF(a & NAND_Ecc_P4o)		<< 1)

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL };
#endif

#ifdef CONFIG_MTD_NAND_OMAP_PREFETCH
static int use_prefetch = 1;

/* "modprobe ... use_prefetch=0" etc */
module_param(use_prefetch, bool, 0);
MODULE_PARM_DESC(use_prefetch, "enable/disable use of PREFETCH");

#ifdef CONFIG_MTD_NAND_OMAP_PREFETCH_DMA
static int use_dma = 1;

/* "modprobe ... use_dma=0" etc */
module_param(use_dma, bool, 0);
MODULE_PARM_DESC(use_dma, "enable/disable use of DMA");
#else
const int use_dma;
#endif
#else
const int use_prefetch;
const int use_dma;
#endif

struct omap_nand_info {
	struct nand_hw_control		controller;
	struct omap_nand_platform_data	*pdata;
	struct mtd_info			mtd;
	struct mtd_partition		*parts;
	struct nand_chip		nand;
	struct platform_device		*pdev;

	int				gpmc_cs;
	unsigned long			phys_base;
	void __iomem			*gpmc_cs_baseaddr;
	void __iomem			*gpmc_baseaddr;
	void __iomem			*nand_pref_fifo_add;
	struct completion		comp;
	int				dma_ch;
};

static struct nand_ecclayout hw_x8_romcode_oob_64 = {
	.eccbytes = 12,
	.eccpos = {
		    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12
		  },
	.oobfree = {
			{.offset = 13,
			 .length = 51}
		   }
};

/* Define some generic bad / good block scan pattern which are used
 * while scanning a device for factory marked good / bad blocks
 */
static uint8_t scan_ff_pattern[] = { 0xff };
static struct nand_bbt_descr bb_descrip_flashbased = {
	.options = NAND_BBT_SCANEMPTY | NAND_BBT_SCANALLPAGES,
	.offs = 0,
	.len = 1,
	.pattern = scan_ff_pattern,
};

static struct nand_ecclayout hw_x16_romcode_oob_64 = {
	.eccbytes = 12,
	.eccpos = {
		    2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
		  },
	.oobfree = {
			{.offset = 14,
			 .length = 50}
		   }
};

/**
 * omap_nand_wp - This function enable or disable the Write Protect feature
 * @mtd: MTD device structure
 * @mode: WP ON/OFF
 */
static void omap_nand_wp(struct mtd_info *mtd, int mode)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);

	unsigned long config = __raw_readl(info->gpmc_baseaddr + GPMC_CONFIG);

	if (mode)
		config &= ~(NAND_WP_BIT);	/* WP is ON */
	else
		config |= (NAND_WP_BIT);	/* WP is OFF */

	__raw_writel(config, (info->gpmc_baseaddr + GPMC_CONFIG));
}

#ifdef CONFIG_MTD_NAND_OMAP_WAIT0_AS_RDY
 /**
 * omap_nand_wait_polarity - This function sets the active polarity of
 *   the NAND R/B# pin
 * @mtd: MTD device structure
 * @level: R/B# active level
 */
static void omap_nand_wait_polarity(struct mtd_info *mtd, int level)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);

	unsigned long config = __raw_readl(info->gpmc_baseaddr + GPMC_CONFIG);

	if (level)
		config |= (GPMC_CONFIG_WAIT0POL);	/* WAIT is active hi */
	else
		config &= ~(GPMC_CONFIG_WAIT0POL);	/* WAIT is active lo */

	__raw_writel(config, (info->gpmc_baseaddr + GPMC_CONFIG));
}

 /**
 * omap_nand_wait_polarity - This function enables thesets the edge
 *   detection status irq of the GPMC
 * @mtd: MTD device structure
 * @enable: on/off
 */
static void omap_nand_wait_irqenable(struct mtd_info *mtd, int enable)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	unsigned long gpmc_irq_enable = __raw_readl(info->gpmc_baseaddr + GPMC_IRQ_ENABLE);


	if (enable)
		gpmc_irq_enable |= (GPMC_IRQ_ENABLE_WAIT0EDGEDETECTIONSTATUS);
	else
		gpmc_irq_enable &= ~(GPMC_IRQ_ENABLE_WAIT0EDGEDETECTIONSTATUS);

	__raw_writel(gpmc_irq_enable, (info->gpmc_baseaddr + GPMC_IRQ_ENABLE));
}
#endif

/**
 * omap_hwcontrol - hardware specific access to control-lines
 * @mtd: MTD device structure
 * @cmd: command to device
 * @ctrl:
 * NAND_NCE: bit 0 -> don't care
 * NAND_CLE: bit 1 -> Command Latch
 * NAND_ALE: bit 2 -> Address Latch
 *
 * NOTE: boards may use different bits for these!!
 */
static void omap_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct omap_nand_info *info = container_of(mtd,
					struct omap_nand_info, mtd);
	switch (ctrl) {
	case NAND_CTRL_CHANGE | NAND_CTRL_CLE:
		info->nand.IO_ADDR_W = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_COMMAND;
		info->nand.IO_ADDR_R = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_DATA;
		break;

	case NAND_CTRL_CHANGE | NAND_CTRL_ALE:
		info->nand.IO_ADDR_W = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_ADDRESS;
		info->nand.IO_ADDR_R = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_DATA;
		break;

	case NAND_CTRL_CHANGE | NAND_NCE:
		info->nand.IO_ADDR_W = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_DATA;
		info->nand.IO_ADDR_R = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_DATA;
		break;
	}

	if (cmd != NAND_CMD_NONE) {
		/* clear WAIT edge detection to capture the WAIT edge */
		__raw_writel(GPMC_IRQ_STATUS_WAIT0EDGEDETECTIONSTATUS,
			info->gpmc_baseaddr + GPMC_IRQ_STATUS);
		__raw_writeb(cmd, info->nand.IO_ADDR_W);
	}
}

/**
 * omap_read_buf8 - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf8(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *nand = mtd->priv;

	ioread8_rep(nand->IO_ADDR_R, buf, len);
}

/**
 * omap_write_buf8 - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf8(struct mtd_info *mtd, const u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	u_char *p = (u_char *)buf;

	while (len--) {
		iowrite8(*p++, info->nand.IO_ADDR_W);
		while (GPMC_BUF_EMPTY == (readl(info->gpmc_baseaddr +
						GPMC_STATUS) & GPMC_BUF_FULL));
	}
}

/**
 * omap_read_buf16 - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf16(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *nand = mtd->priv;

	ioread16_rep(nand->IO_ADDR_R, buf, len / 2);
}

/**
 * omap_write_buf16 - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf16(struct mtd_info *mtd, const u_char * buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	u16 *p = (u16 *) buf;

	/* FIXME try bursts of writesw() or DMA ... */
	len >>= 1;

	while (len--) {
		iowrite16(*p++, info->nand.IO_ADDR_W);

		while (GPMC_BUF_EMPTY == (readl(info->gpmc_baseaddr +
						GPMC_STATUS) & GPMC_BUF_FULL))
			;
	}
}

/**
 * omap_read_buf_pref - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf_pref(struct mtd_info *mtd, u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	uint32_t pfpw_status = 0, r_count = 0;
	int ret = 0;
	u32 *p = (u32 *)buf;

	/* take care of subpage reads */
	if (len % 4) {
		if (info->nand.options & NAND_BUSWIDTH_16)
			omap_read_buf16(mtd, buf, len % 4);
		else
			omap_read_buf8(mtd, buf, len % 4);
		p = (u32 *) (buf + len % 4);
		len -= len % 4;
	}

	/* configure and start prefetch transfer */
	ret = gpmc_prefetch_enable(info->gpmc_cs, 0x0, len, 0x0);
	if (ret) {
		/* PFPW engine is busy, use cpu copy method */
		if (info->nand.options & NAND_BUSWIDTH_16)
			omap_read_buf16(mtd, buf, len);
		else
			omap_read_buf8(mtd, buf, len);
	} else {
		do {
			pfpw_status = gpmc_prefetch_status();
			r_count = ((pfpw_status >> 24) & 0x7F) >> 2;
			ioread32_rep(info->nand_pref_fifo_add, p, r_count);
			p += r_count;
			len -= r_count << 2;
		} while (len);

		/* disable and stop the PFPW engine */
		gpmc_prefetch_reset();
	}
}

/**
 * omap_write_buf_pref - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf_pref(struct mtd_info *mtd,
					const u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	uint32_t pfpw_status = 0, w_count = 0;
	int i = 0, ret = 0;
	u16 *p = (u16 *) buf;

	/* take care of subpage writes */
	if (len % 2 != 0) {
		writeb(*buf, info->nand.IO_ADDR_R);
		p = (u16 *)(buf + 1);
		len--;
	}

	/*  configure and start prefetch transfer */
	ret = gpmc_prefetch_enable(info->gpmc_cs, 0x0, len, 0x1);
	if (ret) {
		/* PFPW engine is busy, use cpu copy method */
		if (info->nand.options & NAND_BUSWIDTH_16)
			omap_write_buf16(mtd, buf, len);
		else
			omap_write_buf8(mtd, buf, len);
	} else {
		pfpw_status = gpmc_prefetch_status();
		while (pfpw_status & 0x3FFF) {
			w_count = ((pfpw_status >> 24) & 0x7F) >> 1;
			for (i = 0; (i < w_count) && len; i++, len -= 2)
				iowrite16(*p++, info->nand_pref_fifo_add);
			pfpw_status = gpmc_prefetch_status();
		}

		/* disable and stop the PFPW engine */
		gpmc_prefetch_reset();
	}
}

#ifdef CONFIG_MTD_NAND_OMAP_PREFETCH_DMA
/*
 * omap_nand_dma_cb: callback on the completion of dma transfer
 * @lch: logical channel
 * @ch_satuts: channel status
 * @data: pointer to completion data structure
 */
static void omap_nand_dma_cb(int lch, u16 ch_status, void *data)
{
	complete((struct completion *) data);
}

/*
 * omap_nand_dma_transfer: configer and start dma transfer
 * @mtd: MTD device structure
 * @addr: virtual address in RAM of source/destination
 * @len: number of data bytes to be transferred
 * @is_write: flag for read/write operation
 */
static inline int omap_nand_dma_transfer(struct mtd_info *mtd, void *addr,
					unsigned int len, int is_write)
{
	struct omap_nand_info *info = container_of(mtd,
					struct omap_nand_info, mtd);
	uint32_t prefetch_status = 0;
	enum dma_data_direction dir = is_write ? DMA_TO_DEVICE :
							DMA_FROM_DEVICE;
	dma_addr_t dma_addr;
	int ret;

	/* The fifo depth is 64 bytes. We have a sync at each frame and frame
	 * length is 64 bytes.
	 */
	int buf_len = len >> 6;

	if (addr >= high_memory) {
		struct page *p1;

		if (((size_t)addr & PAGE_MASK) !=
			((size_t)(addr + len - 1) & PAGE_MASK))
			goto out_copy;
		p1 = vmalloc_to_page(addr);
		if (!p1)
			goto out_copy;
		addr = page_address(p1) + ((size_t)addr & ~PAGE_MASK);
	}

	dma_addr = dma_map_single(&info->pdev->dev, addr, len, dir);
	if (dma_mapping_error(&info->pdev->dev, dma_addr)) {
		dev_err(&info->pdev->dev,
			"Couldn't DMA map a %d byte buffer\n", len);
		goto out_copy;
	}

	if (is_write) {
	    omap_set_dma_dest_params(info->dma_ch, 0, OMAP_DMA_AMODE_CONSTANT,
						info->phys_base, 0, 0);
	    omap_set_dma_src_params(info->dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
							dma_addr, 0, 0);
	    omap_set_dma_transfer_params(info->dma_ch, OMAP_DMA_DATA_TYPE_S32,
					0x10, buf_len, OMAP_DMA_SYNC_FRAME,
					OMAP24XX_DMA_GPMC, OMAP_DMA_DST_SYNC);
	} else {
	    omap_set_dma_src_params(info->dma_ch, 0, OMAP_DMA_AMODE_CONSTANT,
						info->phys_base, 0, 0);
	    omap_set_dma_dest_params(info->dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
							dma_addr, 0, 0);
	    omap_set_dma_transfer_params(info->dma_ch, OMAP_DMA_DATA_TYPE_S32,
					0x10, buf_len, OMAP_DMA_SYNC_FRAME,
					OMAP24XX_DMA_GPMC, OMAP_DMA_SRC_SYNC);
	}
	/*  configure and start prefetch transfer */
	ret = gpmc_prefetch_enable(info->gpmc_cs, 0x1, len, is_write);
	if (ret)
		/* PFPW engine is busy, use cpu copy methode */
		goto out_copy;

	init_completion(&info->comp);

	omap_start_dma(info->dma_ch);

	/* setup and start DMA using dma_addr */
	wait_for_completion(&info->comp);

	while (0x3fff & (prefetch_status = gpmc_prefetch_status()))
		;
	/* disable and stop the PFPW engine */
	gpmc_prefetch_reset();

	dma_unmap_single(&info->pdev->dev, dma_addr, len, dir);
	return 0;

out_copy:
	if (info->nand.options & NAND_BUSWIDTH_16)
		is_write == 0 ? omap_read_buf16(mtd, (u_char *) addr, len)
			: omap_write_buf16(mtd, (u_char *) addr, len);
	else
		is_write == 0 ? omap_read_buf8(mtd, (u_char *) addr, len)
			: omap_write_buf8(mtd, (u_char *) addr, len);
	return 0;
}
#else
static void omap_nand_dma_cb(int lch, u16 ch_status, void *data) {}
static inline int omap_nand_dma_transfer(struct mtd_info *mtd, void *addr,
					unsigned int len, int is_write)
{
	return 0;
}
#endif

/**
 * omap_read_buf_dma_pref - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf_dma_pref(struct mtd_info *mtd, u_char *buf, int len)
{
	if (len <= mtd->oobsize)
		omap_read_buf_pref(mtd, buf, len);
	else
		/* start transfer in DMA mode */
		omap_nand_dma_transfer(mtd, buf, len, 0x0);
}

/**
 * omap_write_buf_dma_pref - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf_dma_pref(struct mtd_info *mtd,
					const u_char *buf, int len)
{
	if (len <= mtd->oobsize)
		omap_write_buf_pref(mtd, buf, len);
	else
		/* start transfer in DMA mode */
		omap_nand_dma_transfer(mtd, (u_char *) buf, len, 0x1);
}

/**
 * omap_verify_buf - Verify chip data against buffer
 * @mtd: MTD device structure
 * @buf: buffer containing the data to compare
 * @len: number of bytes to compare
 */
static int omap_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	u16 *p = (u16 *) buf;

	len >>= 1;
	while (len--) {
		if (*p++ != cpu_to_le16(readw(info->nand.IO_ADDR_R)))
			return -EFAULT;
	}

	return 0;
}

/**
 * omap_hwecc_init - Initialize the HW ECC for NAND flash in GPMC controller
 * @mtd: MTD device structure
 */
static void omap_hwecc_init(struct mtd_info *mtd)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	struct nand_chip *chip = mtd->priv;
	unsigned long val = 0x0;

	/* Read from ECC Control Register */
	val = __raw_readl(info->gpmc_baseaddr + GPMC_ECC_CONTROL);
	/* Clear all ECC | Enable Reg1 */
	val = ((0x00000001<<8) | 0x00000001);
	__raw_writel(val, info->gpmc_baseaddr + GPMC_ECC_CONTROL);

	/* Read from ECC Size Config Register */
	val = __raw_readl(info->gpmc_baseaddr + GPMC_ECC_SIZE_CONFIG);
	/* ECCSIZE1=512 | Select eccResultsize[0-3] */
	val = ((((chip->ecc.size >> 1) - 1) << 22) | (0x0000000F));
	__raw_writel(val, info->gpmc_baseaddr + GPMC_ECC_SIZE_CONFIG);
}

/**
 * gen_true_ecc - This function will generate true ECC value
 * @ecc_buf: buffer to store ecc code
 *
 * This generated true ECC value can be used when correcting
 * data read from NAND flash memory core
 */
static void gen_true_ecc(u8 *ecc_buf)
{
	u32 tmp = ecc_buf[0] | (ecc_buf[1] << 16) |
		((ecc_buf[2] & 0xF0) << 20) | ((ecc_buf[2] & 0x0F) << 8);

	ecc_buf[0] = ~(P64o(tmp) | P64e(tmp) | P32o(tmp) | P32e(tmp) |
			P16o(tmp) | P16e(tmp) | P8o(tmp) | P8e(tmp));
	ecc_buf[1] = ~(P1024o(tmp) | P1024e(tmp) | P512o(tmp) | P512e(tmp) |
			P256o(tmp) | P256e(tmp) | P128o(tmp) | P128e(tmp));
	ecc_buf[2] = ~(P4o(tmp) | P4e(tmp) | P2o(tmp) | P2e(tmp) | P1o(tmp) |
			P1e(tmp) | P2048o(tmp) | P2048e(tmp));
}

/**
 * omap_compare_ecc - Detect (2 bits) and correct (1 bit) error in data
 * @ecc_data1:  ecc code from nand spare area
 * @ecc_data2:  ecc code from hardware register obtained from hardware ecc
 * @page_data:  page data
 *
 * This function compares two ECC's and indicates if there is an error.
 * If the error can be corrected it will be corrected to the buffer.
 */
static int omap_compare_ecc(u8 *ecc_data1,	/* read from NAND memory */
			    u8 *ecc_data2,	/* read from register */
			    u8 *page_data)
{
	uint	i;
	u8	tmp0_bit[8], tmp1_bit[8], tmp2_bit[8];
	u8	comp0_bit[8], comp1_bit[8], comp2_bit[8];
	u8	ecc_bit[24];
	u8	ecc_sum = 0;
	u8	find_bit = 0;
	uint	find_byte = 0;
	int	isEccFF;

	isEccFF = ((*(u32 *)ecc_data1 & 0xFFFFFF) == 0xFFFFFF);

	gen_true_ecc(ecc_data1);
	gen_true_ecc(ecc_data2);

	for (i = 0; i <= 2; i++) {
		*(ecc_data1 + i) = ~(*(ecc_data1 + i));
		*(ecc_data2 + i) = ~(*(ecc_data2 + i));
	}

	for (i = 0; i < 8; i++) {
		tmp0_bit[i]     = *ecc_data1 % 2;
		*ecc_data1	= *ecc_data1 / 2;
	}

	for (i = 0; i < 8; i++) {
		tmp1_bit[i]	 = *(ecc_data1 + 1) % 2;
		*(ecc_data1 + 1) = *(ecc_data1 + 1) / 2;
	}

	for (i = 0; i < 8; i++) {
		tmp2_bit[i]	 = *(ecc_data1 + 2) % 2;
		*(ecc_data1 + 2) = *(ecc_data1 + 2) / 2;
	}

	for (i = 0; i < 8; i++) {
		comp0_bit[i]     = *ecc_data2 % 2;
		*ecc_data2       = *ecc_data2 / 2;
	}

	for (i = 0; i < 8; i++) {
		comp1_bit[i]     = *(ecc_data2 + 1) % 2;
		*(ecc_data2 + 1) = *(ecc_data2 + 1) / 2;
	}

	for (i = 0; i < 8; i++) {
		comp2_bit[i]     = *(ecc_data2 + 2) % 2;
		*(ecc_data2 + 2) = *(ecc_data2 + 2) / 2;
	}

	for (i = 0; i < 6; i++)
		ecc_bit[i] = tmp2_bit[i + 2] ^ comp2_bit[i + 2];

	for (i = 0; i < 8; i++)
		ecc_bit[i + 6] = tmp0_bit[i] ^ comp0_bit[i];

	for (i = 0; i < 8; i++)
		ecc_bit[i + 14] = tmp1_bit[i] ^ comp1_bit[i];

	ecc_bit[22] = tmp2_bit[0] ^ comp2_bit[0];
	ecc_bit[23] = tmp2_bit[1] ^ comp2_bit[1];

	for (i = 0; i < 24; i++)
		ecc_sum += ecc_bit[i];

	switch (ecc_sum) {
	case 0:
		/* Not reached because this function is not called if
		 *  ECC values are equal
		 */
		return 0;

	case 1:
		/* Uncorrectable error */
		DEBUG(MTD_DEBUG_LEVEL0, "ECC UNCORRECTED_ERROR 1\n");
		return -1;

	case 11:
		/* UN-Correctable error */
		DEBUG(MTD_DEBUG_LEVEL0, "ECC UNCORRECTED_ERROR B\n");
		return -1;

	case 12:
		/* Correctable error */
		find_byte = (ecc_bit[23] << 8) +
			    (ecc_bit[21] << 7) +
			    (ecc_bit[19] << 6) +
			    (ecc_bit[17] << 5) +
			    (ecc_bit[15] << 4) +
			    (ecc_bit[13] << 3) +
			    (ecc_bit[11] << 2) +
			    (ecc_bit[9]  << 1) +
			    ecc_bit[7];

		find_bit = (ecc_bit[5] << 2) + (ecc_bit[3] << 1) + ecc_bit[1];

		DEBUG(MTD_DEBUG_LEVEL0, "Correcting single bit ECC error at "
				"offset: %d, bit: %d\n", find_byte, find_bit);

		page_data[find_byte] ^= (1 << find_bit);

		return 1;
	default:
		if (isEccFF) {
			if (ecc_data2[0] == 0 &&
			    ecc_data2[1] == 0 &&
			    ecc_data2[2] == 0)
				return 0;
		}
		DEBUG(MTD_DEBUG_LEVEL0, "UNCORRECTED_ERROR default %d\n", ecc_sum);
		return -1;
	}
}

/**
 * omap_correct_data - Compares the ECC read with HW generated ECC
 * @mtd: MTD device structure
 * @dat: page data
 * @read_ecc: ecc read from nand flash
 * @calc_ecc: ecc read from HW ECC registers
 *
 * Compares the ecc read from nand spare area with ECC registers values
 * and if ECC's mismached, it will call 'omap_compare_ecc' for error detection
 * and correction.
 */
static int omap_correct_data(struct mtd_info *mtd, u_char *dat,
				u_char *read_ecc, u_char *calc_ecc)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	int blockCnt = 0, i = 0, ret = 0;
	int corrected = 0;

	/* Ex NAND_ECC_HW12_2048 */
	if ((info->nand.ecc.mode == NAND_ECC_HW) &&
			(info->nand.ecc.size  == 2048))
		blockCnt = 4;
	else
		blockCnt = 1;

	for (i = 0; i < blockCnt; i++) {
		if (memcmp(read_ecc, calc_ecc, 3) != 0) {
			ret = omap_compare_ecc(read_ecc, calc_ecc, dat);
			if (ret < 0) {
				return ret;
			} else
				corrected += ret;
		}
		read_ecc += 3;
		calc_ecc += 3;
		dat      += 512;
	}
	return corrected;
}

/**
 * omap_calcuate_ecc - Generate non-inverted ECC bytes.
 * @mtd: MTD device structure
 * @dat: The pointer to data on which ecc is computed
 * @ecc_code: The ecc_code buffer
 *
 * Using noninverted ECC can be considered ugly since writing a blank
 * page ie. padding will clear the ECC bytes. This is no problem as long
 * nobody is trying to write data on the seemingly unused page. Reading
 * an erased page will produce an ECC mismatch between generated and read
 * ECC bytes that has to be dealt with separately.
 */
static int omap_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				u_char *ecc_code)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	unsigned long val = 0x0;
	unsigned long reg;

	/* Start Reading from HW ECC1_Result = 0x200 */
	reg = (unsigned long)(info->gpmc_baseaddr + GPMC_ECC1_RESULT);
	val = __raw_readl(reg);
	*ecc_code++ = val;          /* P128e, ..., P1e */
	*ecc_code++ = val >> 16;    /* P128o, ..., P1o */
	/* P2048o, P1024o, P512o, P256o, P2048e, P1024e, P512e, P256e */
	*ecc_code++ = ((val >> 8) & 0x0f) | ((val >> 20) & 0xf0);
	reg += 4;

	return 0;
}

/**
 * omap_enable_hwecc - This function enables the hardware ecc functionality
 * @mtd: MTD device structure
 * @mode: Read/Write mode
 */
static void omap_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	struct nand_chip *chip = mtd->priv;
	unsigned int dev_width = (chip->options & NAND_BUSWIDTH_16) ? 1 : 0;
	unsigned long val = __raw_readl(info->gpmc_baseaddr + GPMC_ECC_CONFIG);

	switch (mode) {
	case NAND_ECC_READ:
		__raw_writel(0x101, info->gpmc_baseaddr + GPMC_ECC_CONTROL);
		/* (ECC 16 or 8 bit col) | ( CS  )  | ECC Enable */
		val = (dev_width << 7) | (info->gpmc_cs << 1) | (0x1);
		break;
	case NAND_ECC_READSYN:
		 __raw_writel(0x100, info->gpmc_baseaddr + GPMC_ECC_CONTROL);
		/* (ECC 16 or 8 bit col) | ( CS  )  | ECC Enable */
		val = (dev_width << 7) | (info->gpmc_cs << 1) | (0x1);
		break;
	case NAND_ECC_WRITE:
		__raw_writel(0x101, info->gpmc_baseaddr + GPMC_ECC_CONTROL);
		/* (ECC 16 or 8 bit col) | ( CS  )  | ECC Enable */
		val = (dev_width << 7) | (info->gpmc_cs << 1) | (0x1);
		break;
	default:
		DEBUG(MTD_DEBUG_LEVEL0, "Error: Unrecognized Mode[%d]!\n",
					mode);
		break;
	}

	__raw_writel(val, info->gpmc_baseaddr + GPMC_ECC_CONFIG);
}

/**
 * omap_wait - wait until the command is done
 * @mtd: MTD device structure
 * @chip: NAND Chip structure
 *
 * Wait function is called during Program and erase operations and
 * the way it is called from MTD layer, we should wait till the NAND
 * chip is ready after the programming/erase operation has completed.
 *
 * Erase can take up to 400ms and program up to 20ms according to
 * general NAND and SmartMedia specs
 */
static int omap_wait(struct mtd_info *mtd, struct nand_chip *chip)
{
	struct nand_chip *this = mtd->priv;
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	unsigned long timeo = jiffies;
	int status = NAND_STATUS_FAIL, state = this->state;

	if (state == FL_ERASING)
		timeo += (HZ * 400) / 1000;
	else
		timeo += (HZ * 20) / 1000;

	this->IO_ADDR_W = (void *) info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_COMMAND;
	this->IO_ADDR_R = (void *) info->gpmc_cs_baseaddr + GPMC_CS_NAND_DATA;

	__raw_writeb(NAND_CMD_STATUS & 0xFF, this->IO_ADDR_W);

	while (time_before(jiffies, timeo)) {
		status = __raw_readb(this->IO_ADDR_R);
		if (status & NAND_STATUS_READY)
			break;
		cond_resched();
	}
	return status;
}

#ifdef CONFIG_MTD_NAND_OMAP_WAIT0_AS_RDY
/**
 * omap_dev_ready - calls the platform specific dev_ready function
 * @mtd: MTD device structure
 */
static int omap_dev_ready(struct mtd_info *mtd)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	int val;

	/* Now we can look looking for R/B# to be high (i.e. WAIT0 is
	   asserted). */
	val = __raw_readl(info->gpmc_baseaddr + GPMC_STATUS);
	if (val & GPMC_STATUS_WAIT0STATUS)
		return 1;
	return 0;
}

/*
 * For devices which display every fart in the system on a separate LED. Is
 * compiled away when LED support is disabled.
 */
DEFINE_LED_TRIGGER(omap_nand_led_trigger);

#ifdef CONFIG_MTD_NAND_OMAP_WAIT0_STATISTICS
struct wait_stats {
	/* Low -> High */
	unsigned int ltoh;
	unsigned int ltoh_sum;
	unsigned long long ltoh_sum_squares;
	/* High -> High */
	unsigned int htoh;
	unsigned int htoh_sum;
	unsigned long long htoh_sum_squares;
} wait_stats;

static void capture_stats(struct wait_stats *p, unsigned int wait_ready,
			unsigned int wait_edge, unsigned int first)
{
	/* If first then we waiting for edge to drop then wait for ready */
	if (first) {
		/* we found ready high to start with */
		wait_ready += wait_edge;
		p->htoh++;
		p->htoh_sum += wait_ready;
		p->htoh_sum_squares += ((unsigned long long)wait_ready)*wait_ready;
	} else {
		/* samples is total of wait for edge, then edge high */
		wait_ready += wait_edge;
		p->ltoh++;
		p->ltoh_sum += wait_ready;
		p->ltoh_sum_squares += ((unsigned long long)wait_ready)*wait_ready;
	}
}
#endif

/*
 * Wait for the ready pin, after a command
 * The timeout is catched later.
 */
static void omap_nand_wait_ready(struct mtd_info *mtd)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	int first, val, cnt=0, wait_cnt=0;
	/* struct nand_chip *chip = mtd->priv; */
	unsigned long timeo = jiffies + 2;

	/* Capture the ready line */
	first = __raw_readl(info->gpmc_baseaddr + GPMC_STATUS);

	led_trigger_event(omap_nand_led_trigger, LED_FULL);

	/* wait for an edge on WAIT0 */
	while ( ((__raw_readl(info->gpmc_baseaddr + GPMC_IRQ_STATUS) & GPMC_IRQ_STATUS_WAIT0EDGEDETECTIONSTATUS) == 0) && time_before(jiffies, timeo)) {
		wait_cnt++;
	}

	/* Now wait for ready to go high */
	do {
		val = __raw_readl(info->gpmc_baseaddr + GPMC_STATUS);
		if (val & GPMC_STATUS_WAIT0STATUS)
			break;
		touch_softlockup_watchdog();
		cnt++;
	} while (time_before(jiffies, timeo));

	led_trigger_event(omap_nand_led_trigger, LED_OFF);

#ifdef CONFIG_MTD_NAND_OMAP_WAIT0_STATISTICS
	capture_stats(&wait_stats, cnt, wait_cnt, first&GPMC_STATUS_WAIT0STATUS);
#endif		
}

/**
 * omap_nand_command_lp - Send command to NAND large page device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 *
 * Send command to NAND device. This is the version for the new large page
 * devices We dont have the separate regions as we have in the small page
 * devices.  We must emulate NAND_CMD_READOOB to keep the code compatible.
 */
static void omap_nand_command_lp(struct mtd_info *mtd, unsigned int command,
			    int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;

	/* Emulate NAND_CMD_READOOB */
	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	/* Command latch cycle */
	omap_hwcontrol(mtd, command & 0xff,
		       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	if (column != -1 || page_addr != -1) {
		int ctrl = NAND_CTRL_CHANGE | NAND_NCE | NAND_ALE;

		/* Serially input address */
		if (column != -1) {
			/* Adjust columns for 16 bit buswidth */
			if (chip->options & NAND_BUSWIDTH_16)
				column >>= 1;
			omap_hwcontrol(mtd, column, ctrl);
			ctrl &= ~NAND_CTRL_CHANGE;
			omap_hwcontrol(mtd, column >> 8, ctrl);
		}
		if (page_addr != -1) {
			omap_hwcontrol(mtd, page_addr, ctrl);
			omap_hwcontrol(mtd, page_addr >> 8,
				       NAND_NCE | NAND_ALE);
			/* One more address cycle for devices > 128MiB */
			if (chip->chipsize > (128 << 20))
				omap_hwcontrol(mtd, page_addr >> 16,
					       NAND_NCE | NAND_ALE);
		}
	}
	omap_hwcontrol(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * program and erase have their own busy handlers
	 * status, sequential in, and deplete1 need no delay
	 */
	switch (command) {

	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_RNDIN:
	case NAND_CMD_STATUS:
	case NAND_CMD_DEPLETE1:
		return;

		/*
		 * read error status commands require only a short delay
		 */
	case NAND_CMD_STATUS_ERROR:
	case NAND_CMD_STATUS_ERROR0:
	case NAND_CMD_STATUS_ERROR1:
	case NAND_CMD_STATUS_ERROR2:
	case NAND_CMD_STATUS_ERROR3:
		udelay(chip->chip_delay);
		return;

	case NAND_CMD_RESET:
		if (chip->dev_ready)
			break;
		udelay(chip->chip_delay);
		omap_hwcontrol(mtd, NAND_CMD_STATUS,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		omap_hwcontrol(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		while (!(chip->read_byte(mtd) & NAND_STATUS_READY)) ;
		return;

	case NAND_CMD_RNDOUT:
		/* No ready / busy check necessary */
		omap_hwcontrol(mtd, NAND_CMD_RNDOUTSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		omap_hwcontrol(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		return;

	case NAND_CMD_READ0:

		/* Send the read start */
		omap_hwcontrol(mtd, NAND_CMD_READSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		omap_hwcontrol(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}

	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	ndelay(100);

	omap_nand_wait_ready(mtd);
}
#else
/**
 * omap_dev_ready - calls the platform specific dev_ready function
 * @mtd: MTD device structure
 */
static int omap_dev_ready(struct mtd_info *mtd)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	unsigned int val = __raw_readl(info->gpmc_baseaddr + GPMC_IRQ_STATUS);

	if ((val & 0x100) == 0x100) {
		/* Clear IRQ Interrupt */
		val |= 0x100;
		val &= ~(0x0);
		__raw_writel(val, info->gpmc_baseaddr + GPMC_IRQ_STATUS);
	} else {
		unsigned int cnt = 0;
		while (cnt++ < 0x1FF) {
			if  ((val & 0x100) == 0x100)
				return 0;
			val = __raw_readl(info->gpmc_baseaddr +
							GPMC_IRQ_STATUS);
		}
	}

	return 1;
}
#endif

#if defined(CONFIG_PROC_FS) && defined(CONFIG_MTD_NAND_OMAP_WAIT0_STATISTICS)
static int wait_stats_read_proc (char *page, char **start, off_t off, int count,
			  int *eof, void *data_unused)
{
	int len;
	
	len = sprintf(page, "lo->hi: %u sum %u sum-squared %llu\nhi->hi: %u sum %u sum-sqared %llu\n",
		wait_stats.ltoh, wait_stats.ltoh_sum, wait_stats.ltoh_sum_squares,
		wait_stats.htoh, wait_stats.htoh_sum, wait_stats.htoh_sum_squares);

	return len;
}

static int wait_stats_write_proc(struct file *file, const char *buf,
				unsigned long count, void *data)
{
	memset(&wait_stats, 0, sizeof(wait_stats));
	return count;
}

static struct proc_dir_entry *proc_wait_stats;
#endif

static int __devinit omap_nand_probe(struct platform_device *pdev)
{
	struct omap_nand_info		*info;
	struct omap_nand_platform_data	*pdata;
	int				err;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct omap_nand_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	platform_set_drvdata(pdev, info);

	spin_lock_init(&info->controller.lock);
	init_waitqueue_head(&info->controller.wq);

	info->pdev = pdev;

	info->gpmc_cs		= pdata->cs;
	info->gpmc_baseaddr	= pdata->gpmc_baseaddr;
	info->gpmc_cs_baseaddr	= pdata->gpmc_cs_baseaddr;

	info->mtd.priv		= &info->nand;
	info->mtd.name		= dev_name(&pdev->dev);
	info->mtd.owner		= THIS_MODULE;

	info->nand.options	|= pdata->devsize ? NAND_BUSWIDTH_16 : 0;
	info->nand.options	|= NAND_SKIP_BBTSCAN;

	err = gpmc_cs_request(info->gpmc_cs, NAND_IO_SIZE, &info->phys_base);
	if (err < 0) {
		dev_err(&pdev->dev, "Cannot request GPMC CS\n");
		goto out_free_info;
	}

	/* NAND write protect off */
	omap_nand_wp(&info->mtd, NAND_WP_OFF);

	if (!request_mem_region(info->phys_base, NAND_IO_SIZE,
				pdev->dev.driver->name)) {
		err = -EBUSY;
		goto out_free_cs;
	}

	info->nand.IO_ADDR_R = ioremap(info->phys_base, NAND_IO_SIZE);
	if (!info->nand.IO_ADDR_R) {
		err = -ENOMEM;
		goto out_release_mem_region;
	}

	info->nand.controller = &info->controller;

	info->nand.IO_ADDR_W = info->nand.IO_ADDR_R;
	info->nand.cmd_ctrl  = omap_hwcontrol;

	/*
	 * If RDY/BSY line is connected to OMAP then use the omap ready
	 * funcrtion and the generic nand_wait function which reads the status
	 * register after monitoring the RDY/BSY line.Otherwise use a standard
	 * chip delay which is slightly more than tR (AC Timing) of the NAND
	 * device and read status register until you get a failure or success
	 */
	if (pdata->dev_ready) {
		info->nand.dev_ready = omap_dev_ready;
		info->nand.chip_delay = 0;
#ifdef CONFIG_MTD_NAND_OMAP_WAIT0_AS_RDY
		omap_nand_wait_polarity(&info->mtd, 0);
		omap_nand_wait_irqenable(&info->mtd, 1);
		info->nand.nand_wait_ready = omap_nand_wait_ready;
		if (info->nand.options & NAND_BUSWIDTH_16) {
			info->nand.cmdfunc = omap_nand_command_lp;
			printk(KERN_INFO "OMAP NAND: use WAIT0 as R/B# pin\n");
		}
		omap_nand_led_trigger = DEFINE_LED_GET_TRIGGER("nand-disk");
		if (!omap_nand_led_trigger)
			printk(KERN_ERR "NAND: could not find nand-disk LED trigger\n");
#endif
	} else {
		info->nand.waitfunc = omap_wait;
		info->nand.chip_delay = 50;
	}

	if (use_prefetch) {
		/* copy the virtual address of nand base for fifo access */
		info->nand_pref_fifo_add = info->nand.IO_ADDR_R;

		info->nand.read_buf   = omap_read_buf_pref;
		info->nand.write_buf  = omap_write_buf_pref;
		if (use_dma) {
			err = omap_request_dma(OMAP24XX_DMA_GPMC, "NAND",
				omap_nand_dma_cb, &info->comp, &info->dma_ch);
			if (err < 0) {
				info->dma_ch = -1;
				printk(KERN_WARNING "DMA request failed."
					" Non-dma data transfer mode\n");
			} else {
				omap_set_dma_dest_burst_mode(info->dma_ch,
						OMAP_DMA_DATA_BURST_16);
				omap_set_dma_src_burst_mode(info->dma_ch,
						OMAP_DMA_DATA_BURST_16);

				info->nand.read_buf   = omap_read_buf_dma_pref;
				info->nand.write_buf  = omap_write_buf_dma_pref;
			}
		}
	} else {
		if (info->nand.options & NAND_BUSWIDTH_16) {
			info->nand.read_buf   = omap_read_buf16;
			info->nand.write_buf  = omap_write_buf16;
		} else {
			info->nand.read_buf   = omap_read_buf8;
			info->nand.write_buf  = omap_write_buf8;
		}
	}
	info->nand.verify_buf = omap_verify_buf;

	if (pdata->ecc_opt & 0x3) {
		if (pdata->ecc_opt == 0x2) {
			if (info->nand.options & NAND_BUSWIDTH_16) {
				info->nand.ecc.layout = &hw_x16_romcode_oob_64;
			} else {
				info->nand.ecc.layout = &hw_x8_romcode_oob_64;
				info->nand.badblock_pattern =
							&bb_descrip_flashbased;
			}
		}
	info->nand.ecc.bytes		= 3;
	info->nand.ecc.size		= 512;
	info->nand.ecc.calculate	= omap_calculate_ecc;
	info->nand.ecc.hwctl		= omap_enable_hwecc;
	info->nand.ecc.correct		= omap_correct_data;
	info->nand.ecc.mode		= NAND_ECC_HW;

	/* init HW ECC */
	omap_hwecc_init(&info->mtd);
	} else {
	info->nand.ecc.mode = NAND_ECC_SOFT;
	}

#if defined(CONFIG_PROC_FS) && defined(CONFIG_MTD_NAND_OMAP_WAIT0_STATISTICS)
	if ((proc_wait_stats = create_proc_entry( "nand-wait-stats", S_IRUGO | S_IFREG, NULL))) {
		proc_wait_stats->write_proc = wait_stats_write_proc;
		proc_wait_stats->read_proc = wait_stats_read_proc;
		proc_wait_stats->data = NULL;
	} else
		return -ENOMEM;
#endif
	/* DIP switches on some boards change between 8 and 16 bit
	 * bus widths for flash.  Try the other width if the first try fails.
	 */
	if (nand_scan(&info->mtd, 1)) {
		info->nand.options ^= NAND_BUSWIDTH_16;
		if (nand_scan(&info->mtd, 1)) {
			err = -ENXIO;
			goto out_release_mem_region;
		}
	}

#ifdef CONFIG_MTD_PARTITIONS
	err = parse_mtd_partitions(&info->mtd, part_probes, &info->parts, 0);
	if (err > 0)
		add_mtd_partitions(&info->mtd, info->parts, err);
	else if (pdata->parts)
		add_mtd_partitions(&info->mtd, pdata->parts, pdata->nr_parts);
	else
#endif
		add_mtd_device(&info->mtd);

	platform_set_drvdata(pdev, &info->mtd);

	return 0;

out_release_mem_region:
	release_mem_region(info->phys_base, NAND_IO_SIZE);
out_free_cs:
	gpmc_cs_free(info->gpmc_cs);
out_free_info:
	kfree(info);

	return err;
}

static int omap_nand_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);

	platform_set_drvdata(pdev, NULL);
	if (use_dma)
		omap_free_dma(info->dma_ch);

	/* Release NAND device, its internal structures and partitions */
	nand_release(&info->mtd);
	iounmap(info->nand_pref_fifo_add);
	kfree(&info->mtd);
	return 0;
}

static struct platform_driver omap_nand_driver = {
	.probe		= omap_nand_probe,
	.remove		= omap_nand_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init omap_nand_init(void)
{
	printk(KERN_INFO "%s driver initializing\n", DRIVER_NAME);

	/* This check is required if driver is being
	 * loaded run time as a module
	 */
	if ((1 == use_dma) && (0 == use_prefetch)) {
		printk(KERN_INFO"Wrong parameters: 'use_dma' can not be 1 "
				"without use_prefetch'. Prefetch will not be"
				" used in either mode (mpu or dma)\n");
	}
	return platform_driver_register(&omap_nand_driver);
}

static void __exit omap_nand_exit(void)
{
#if defined(CONFIG_PROC_FS) && defined(CONFIG_MTD_NAND_OMAP_WAIT0_STATISTICS)
	if (proc_wait_stats)
		remove_proc_entry( "nand-wait-stats", NULL);
#endif
	platform_driver_unregister(&omap_nand_driver);
}

module_init(omap_nand_init);
module_exit(omap_nand_exit);

MODULE_ALIAS(DRIVER_NAME);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Glue layer for NAND flash on TI OMAP boards");
