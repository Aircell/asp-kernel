#define CONFIG_MMC_DEBUG 1
/*
 * drivers/mmc/host/omap_hsmmc.c
 *
 * Driver for OMAP2430/3430 MMC controller.
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * Authors:
 *	Syed Mohammed Khasim	<x0khasim@ti.com>
 *	Madhusudhan		<madhu.cr@ti.com>
 *	Mohit Jalori		<mjalori@ti.com>
 *  Support for Support for MMC3 controller TI WLAN by:
 *	San Mehat		<san@google.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <plat/dma.h>
#include <mach/hardware.h>
#include <plat/board.h>
#include <plat/mmc.h>
#include <plat/cpu.h>

/* OMAP HSMMC Host Controller Registers */
#define OMAP_HSMMC_SYSCONFIG	0x0010
#define OMAP_HSMMC_SYSSTATUS	0x0014
#define OMAP_HSMMC_CON		0x002C
#define OMAP_HSMMC_BLK		0x0104
#define OMAP_HSMMC_ARG		0x0108
#define OMAP_HSMMC_CMD		0x010C
#define OMAP_HSMMC_RSP10	0x0110
#define OMAP_HSMMC_RSP32	0x0114
#define OMAP_HSMMC_RSP54	0x0118
#define OMAP_HSMMC_RSP76	0x011C
#define OMAP_HSMMC_DATA		0x0120
#define OMAP_HSMMC_HCTL		0x0128
#define OMAP_HSMMC_SYSCTL	0x012C
#define OMAP_HSMMC_STAT		0x0130
#define OMAP_HSMMC_IE		0x0134
#define OMAP_HSMMC_ISE		0x0138
#define OMAP_HSMMC_CAPA		0x0140

#define VS18			(1 << 26)
#define VS30			(1 << 25)
#define SDVS18			(0x5 << 9)
#define SDVS30			(0x6 << 9)
#define SDVS33			(0x7 << 9)
#define SDVS_MASK		0x00000E00
#define SDVSCLR			0xFFFFF1FF
#define SDVSDET			0x00000400
#define AUTOIDLE		0x1
#define SIDLEMODE_IGNORE	0x08

#define SDBP			(1 << 8)
#define DTO			0xe
#define ICE			0x1
#define ICS			0x2
#define CEN			(1 << 2)
#define CLKD_MASK		0x0000FFC0
#define CLKD_SHIFT		6
#define DTO_MASK		0x000F0000
#define DTO_SHIFT		16
#define INT_EN_MASK		0x307F0033
#define BWR_ENABLE		(1 << 4)
#define BRR_ENABLE		(1 << 5)
#define INIT_STREAM		(1 << 1)
#define DP_SELECT		(1 << 21)
#define DDIR			(1 << 4)
#define DMA_EN			0x1
#define MSBS			(1 << 5)
#define BCE			(1 << 1)
#define FOUR_BIT		(1 << 1)
#define DW8			(1 << 5)
#define CC			0x1
#define TC			0x02
#define OD			0x1
#define ERR			(1 << 15)
#define CMD_TIMEOUT		(1 << 16)
#define DATA_TIMEOUT		(1 << 20)
#define CMD_CRC			(1 << 17)
#define DATA_CRC		(1 << 21)
#define CARD_ERR		(1 << 28)
#define STAT_CLEAR		0xFFFFFFFF
#define INIT_STREAM_CMD		0x00000000
#define DUAL_VOLT_OCR_BIT	7
#define SRC			(1 << 25)
#define SRD			(1 << 26)
#define SOFTRESET		(1 << 1)
#define RESETDONE		(1 << 0)
#define CIRQ		(1 << 8)
#define CIRQ_ENABLE	(1 << 8)
#define CTPL		(1 << 11)
#define CLKEXTFREE	(1 << 16)

/*
 * FIXME: Most likely all the data using these _DEVID defines should come
 * from the platform_data, or implemented in controller and slot specific
 * functions.
 */
#define OMAP_MMC1_DEVID		0
#define OMAP_MMC2_DEVID		1
#define OMAP_MMC3_DEVID		2
#define OMAP_MMC4_DEVID		3
#define OMAP_MMC5_DEVID		4

#define MMC_TIMEOUT_MS		20
#define OMAP_MMC_MASTER_CLOCK	96000000
#define DRIVER_NAME		"mmci-omap-hs"

/* Timeouts for entering power saving states on inactivity, msec */
#define OMAP_MMC_DISABLED_TIMEOUT	100
#define OMAP_MMC_SLEEP_TIMEOUT		1000
#define OMAP_MMC_OFF_TIMEOUT		8000

/*
 * One controller can have multiple slots, like on some omap boards using
 * omap.c controller driver. Luckily this is not currently done on any known
 * omap_hsmmc.c device.
 */
#define mmc_slot(host)		(host->pdata->slots[host->slot_id])

/*
 * MMC Host controller read/write API's
 */
#define OMAP_HSMMC_READ(base, reg)	\
	__raw_readl((base) + OMAP_HSMMC_##reg)

#define OMAP_HSMMC_WRITE(base, reg, val) \
	__raw_writel((val), (base) + OMAP_HSMMC_##reg)

struct omap_hsmmc_host {
	struct	device		*dev;
	struct	mmc_host	*mmc;
	struct	mmc_request	*mrq;
	struct	mmc_command	*cmd;
	struct	mmc_data	*data;
	struct	clk		*fclk;
	struct	clk		*iclk;
	struct	clk		*dbclk;
	struct	semaphore	sem;
	struct	work_struct	mmc_carddetect_work;
	void	__iomem		*base;
	resource_size_t		mapbase;
	spinlock_t		irq_lock; /* Prevent races with irq handler */
	unsigned long		flags;
	unsigned int		id;
	unsigned int		dma_len;
	unsigned int		dma_sg_idx;
	unsigned char		bus_mode;
	unsigned char		power_mode;
	u32			*buffer;
	u32			bytesleft;
	int			suspended;
	int			irq;
	int			use_dma, dma_ch;
	int			dma_line_tx, dma_line_rx;
	int			slot_id;
	int			got_dbclk;
	int			response_busy;
	int			context_loss;
	int			dpm_state;
	int			vdd;
	int			protect_card;
	int			reqs_blocked;
	int			sdio_int;
	int			carddetect;

	struct	omap_mmc_platform_data	*pdata;
};

/*
 * Stop clock to the card
 */
void omap_hsmmc_stop_clock(struct omap_hsmmc_host *host)
{
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) & ~CEN);
	if ((OMAP_HSMMC_READ(host->base, SYSCTL) & CEN) != 0x0)
		dev_dbg(mmc_dev(host->mmc), "MMC Clock is not stoped\n");
}

#ifdef CONFIG_PM

/*
 * Restore the MMC host context, if it was lost as result of a
 * power state change.
 */
int omap_hsmmc_context_restore(struct omap_hsmmc_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;
	struct omap_mmc_platform_data *pdata = host->pdata;
	int context_loss = 0;
	u32 hctl, capa, con;
	u16 dsor = 0;
	unsigned long timeout;

	if (pdata->get_context_loss_count) {
		context_loss = pdata->get_context_loss_count(host->dev);
		if (context_loss < 0)
			return 1;
	}

	dev_dbg(mmc_dev(host->mmc), "context was %slost\n",
		context_loss == host->context_loss ? "not " : "");
	if (host->context_loss == context_loss)
		return 1;

	/* Wait for hardware reset */
	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, SYSSTATUS) & RESETDONE) != RESETDONE
		&& time_before(jiffies, timeout))
		;

	/* Do software reset */
	OMAP_HSMMC_WRITE(host->base, SYSCONFIG, SOFTRESET);
	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, SYSSTATUS) & RESETDONE) != RESETDONE
		&& time_before(jiffies, timeout))
		;

	/* TARR - not sure about this */
	OMAP_HSMMC_WRITE(host->base, SYSCONFIG,
			//OMAP_HSMMC_READ(host->base, SYSCONFIG) | SIDLEMODE_IGNORE);
			OMAP_HSMMC_READ(host->base, SYSCONFIG) | AUTOIDLE);

	if (host->id == OMAP_MMC1_DEVID) {
		if (host->power_mode != MMC_POWER_OFF &&
		    (1 << ios->vdd) <= MMC_VDD_23_24)
			hctl = SDVS18;
		else
			hctl = SDVS30;
		capa = VS30 | VS18;
	} else {
		hctl = SDVS18;
		capa = VS18;
	}

	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | hctl);

	OMAP_HSMMC_WRITE(host->base, CAPA,
			OMAP_HSMMC_READ(host->base, CAPA) | capa);

	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDBP);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, HCTL) & SDBP) != SDBP
		&& time_before(jiffies, timeout))
		;

	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
	OMAP_HSMMC_WRITE(host->base, ISE, INT_EN_MASK);
	OMAP_HSMMC_WRITE(host->base, IE, INT_EN_MASK);

	/* Do not initialize card-specific things if the power is off */
	if (host->power_mode == MMC_POWER_OFF)
		goto out;

	con = OMAP_HSMMC_READ(host->base, CON);
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_8:
		OMAP_HSMMC_WRITE(host->base, CON, con | DW8);
		break;
	case MMC_BUS_WIDTH_4:
		OMAP_HSMMC_WRITE(host->base, CON, con & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | FOUR_BIT);
		break;
	case MMC_BUS_WIDTH_1:
		OMAP_HSMMC_WRITE(host->base, CON, con & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) & ~FOUR_BIT);
		break;
	}

	if (ios->clock) {
		dsor = OMAP_MMC_MASTER_CLOCK / ios->clock;
		if (dsor < 1)
			dsor = 1;

		if (OMAP_MMC_MASTER_CLOCK / dsor > ios->clock)
			dsor++;

		if (dsor > 250)
			dsor = 250;
	}

	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) & ~CEN);
	OMAP_HSMMC_WRITE(host->base, SYSCTL, (dsor << 6) | (DTO << 16));
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | ICE);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & ICS) != ICS
		&& time_before(jiffies, timeout))
		;

	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | CEN);

	con = OMAP_HSMMC_READ(host->base, CON);
	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
		OMAP_HSMMC_WRITE(host->base, CON, con | OD);
	else
		OMAP_HSMMC_WRITE(host->base, CON, con & ~OD);
out:
	host->context_loss = context_loss;

	dev_dbg(mmc_dev(host->mmc), "context is restored\n");
	return 0;
}

/*
 * Save the MMC host context (store the number of power state changes so far).
 */
void omap_hsmmc_context_save(struct omap_hsmmc_host *host)
{
	struct omap_mmc_platform_data *pdata = host->pdata;
	int context_loss;

	if (pdata->get_context_loss_count) {
		context_loss = pdata->get_context_loss_count(host->dev);
		if (context_loss < 0)
			return;
		host->context_loss = context_loss;
	}
}

#else

int omap_hsmmc_context_restore(struct omap_hsmmc_host *host)
{
	return 0;
}

void omap_hsmmc_context_save(struct omap_hsmmc_host *host)
{
}

#endif

/*
 * Send init stream sequence to card
 * before sending IDLE command
 */
static void send_init_stream(struct omap_hsmmc_host *host)
{
	int reg = 0;
	unsigned long timeout;

	if (host->protect_card)
		return;

	disable_irq(host->irq);
	OMAP_HSMMC_WRITE(host->base, CON,
		OMAP_HSMMC_READ(host->base, CON) | INIT_STREAM);
	OMAP_HSMMC_WRITE(host->base, CMD, INIT_STREAM_CMD);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((reg != CC) && time_before(jiffies, timeout))
		reg = OMAP_HSMMC_READ(host->base, STAT) & CC;

	OMAP_HSMMC_WRITE(host->base, CON,
		OMAP_HSMMC_READ(host->base, CON) & ~INIT_STREAM);

	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
	OMAP_HSMMC_READ(host->base, STAT);

	enable_irq(host->irq);
}

static inline
int omap_hsmmc_cover_is_closed(struct omap_hsmmc_host *host)
{
	int r = 1;

	if (mmc_slot(host).get_cover_state)
		r = mmc_slot(host).get_cover_state(host->dev, host->slot_id);
	return r;
}

static ssize_t
omap_hsmmc_show_cover_switch(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mmc_host *mmc = container_of(dev, struct mmc_host, class_dev);
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	return sprintf(buf, "%s\n",
			omap_hsmmc_cover_is_closed(host) ? "closed" : "open");
}

static DEVICE_ATTR(cover_switch, S_IRUGO, omap_hsmmc_show_cover_switch, NULL);

static ssize_t
omap_hsmmc_show_slot_name(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct mmc_host *mmc = container_of(dev, struct mmc_host, class_dev);
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	return sprintf(buf, "%s\n", mmc_slot(host).name);
}

static DEVICE_ATTR(slot_name, S_IRUGO, omap_hsmmc_show_slot_name, NULL);

/*
 * Configure the response type and send the cmd.
 */
static void
omap_hsmmc_start_command(struct omap_hsmmc_host *host, struct mmc_command *cmd,
	struct mmc_data *data)
{
	int cmdreg = 0, resptype = 0, cmdtype = 0;

	dev_dbg(mmc_dev(host->mmc), "%s: CMD%d, argument 0x%08x\n",
		mmc_hostname(host->mmc), cmd->opcode, cmd->arg);
	host->cmd = cmd;

	/*
	 * Clear status bits and enable interrupts
	 */
	OMAP_HSMMC_WRITE(host->base, STAT, STAT_CLEAR);
	OMAP_HSMMC_WRITE(host->base, ISE, INT_EN_MASK);

	if (host->use_dma)
		OMAP_HSMMC_WRITE(host->base, IE,
				 INT_EN_MASK & ~(BRR_ENABLE | BWR_ENABLE));
	else
		OMAP_HSMMC_WRITE(host->base, IE, INT_EN_MASK);

	if (host->sdio_int) {
		OMAP_HSMMC_WRITE(host->base, ISE,
			(OMAP_HSMMC_READ(host->base, ISE) | CIRQ_ENABLE));
		OMAP_HSMMC_WRITE(host->base, IE,
			(OMAP_HSMMC_READ(host->base, IE) | CIRQ_ENABLE));
	}

	host->response_busy = 0;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			resptype = 1;
		else if (cmd->flags & MMC_RSP_BUSY) {
			resptype = 3;
			host->response_busy = 1;
		} else
			resptype = 2;
	}

	/*
	 * Unlike OMAP1 controller, the cmdtype does not seem to be based on
	 * ac, bc, adtc, bcr. Only commands ending an open ended transfer need
	 * a val of 0x3, rest 0x0.
	 */
	if (cmd == host->mrq->stop)
		cmdtype = 0x3;

	cmdreg = (cmd->opcode << 24) | (resptype << 16) | (cmdtype << 22);

	if (data) {
		cmdreg |= DP_SELECT | MSBS | BCE;
		if (data->flags & MMC_DATA_READ)
			cmdreg |= DDIR;
		else
			cmdreg &= ~(DDIR);
	}

	if (host->use_dma)
		cmdreg |= DMA_EN;

	/*
	 * In an interrupt context (i.e. STOP command), the spinlock is unlocked
	 * by the interrupt handler, otherwise (i.e. for a new request) it is
	 * unlocked here.
	 */
	if (!in_interrupt())
		spin_unlock_irqrestore(&host->irq_lock, host->flags);

	OMAP_HSMMC_WRITE(host->base, ARG, cmd->arg);
	OMAP_HSMMC_WRITE(host->base, CMD, cmdreg);
}

static int
omap_hsmmc_get_dma_dir(struct omap_hsmmc_host *host, struct mmc_data *data)
{
	if (data->flags & MMC_DATA_WRITE)
		return DMA_TO_DEVICE;
	else
		return DMA_FROM_DEVICE;
}

/*
 * Notify the transfer complete to MMC core
 */
static void
omap_hsmmc_xfer_done(struct omap_hsmmc_host *host, struct mmc_data *data)
{
	if (!data) {
		struct mmc_request *mrq = host->mrq;

		/* TC before CC from CMD6 - don't know why, but it happens */
		if (host->cmd && host->cmd->opcode == 6 &&
		    host->response_busy) {
			host->response_busy = 0;
			return;
		}

		host->mrq = NULL;
		mmc_request_done(host->mmc, mrq);
		return;
	}

	host->data = NULL;

	if (host->use_dma && host->dma_ch != -1)
		dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->dma_len,
			omap_hsmmc_get_dma_dir(host, data));

	if (!data->error)
		data->bytes_xfered += data->blocks * (data->blksz);
	else
		data->bytes_xfered = 0;

	if (!data->stop) {
		host->mrq = NULL;
		mmc_request_done(host->mmc, data->mrq);
		return;
	}
	omap_hsmmc_start_command(host, data->stop, NULL);
}

/*
 * Notify the core about command completion
 */
static void
omap_hsmmc_cmd_done(struct omap_hsmmc_host *host, struct mmc_command *cmd)
{
	host->cmd = NULL;

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* response type 2 */
			cmd->resp[3] = OMAP_HSMMC_READ(host->base, RSP10);
			cmd->resp[2] = OMAP_HSMMC_READ(host->base, RSP32);
			cmd->resp[1] = OMAP_HSMMC_READ(host->base, RSP54);
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP76);
		} else {
			/* response types 1, 1b, 3, 4, 5, 6 */
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP10);
		}
	}
	if ((host->data == NULL && !host->response_busy) || cmd->error) {
		host->mrq = NULL;
		mmc_request_done(host->mmc, cmd->mrq);
	}
}

/*
 * DMA clean up for command errors
 */
static void omap_hsmmc_dma_cleanup(struct omap_hsmmc_host *host, int errno)
{
	host->data->error = errno;

	if (host->use_dma && host->dma_ch != -1) {
		dma_unmap_sg(mmc_dev(host->mmc), host->data->sg, host->dma_len,
			omap_hsmmc_get_dma_dir(host, host->data));
		omap_free_dma(host->dma_ch);
		host->dma_ch = -1;
		up(&host->sem);
	}
	host->data = NULL;
}

/*
 * Readable error output
 */
#ifdef CONFIG_MMC_DEBUG
static void omap_hsmmc_report_irq(struct omap_hsmmc_host *host, u32 status)
{
	/* --- means reserved bit without definition at documentation */
	static const char *omap_hsmmc_status_bits[] = {
		"CC", "TC", "BGE", "---", "BWR", "BRR", "---", "---", "CIRQ",
		"OBI", "---", "---", "---", "---", "---", "ERRI", "CTO", "CCRC",
		"CEB", "CIE", "DTO", "DCRC", "DEB", "---", "ACE", "---",
		"---", "---", "---", "CERR", "CERR", "BADA", "---", "---", "---"
	};
	char res[256];
	char *buf = res;
	int len, i;

	len = sprintf(buf, "MMC IRQ 0x%x :", status);
	buf += len;

	for (i = 0; i < ARRAY_SIZE(omap_hsmmc_status_bits); i++)
		if (status & (1 << i)) {
			len = sprintf(buf, " %s", omap_hsmmc_status_bits[i]);
			buf += len;
		}

	dev_dbg(mmc_dev(host->mmc), "%s\n", res);
}
#endif  /* CONFIG_MMC_DEBUG */

/*
 * MMC controller internal state machines reset
 *
 * Used to reset command or data internal state machines, using respectively
 *  SRC or SRD bit of SYSCTL register
 * Can be called from interrupt context
 */
static inline void omap_hsmmc_reset_controller_fsm(struct omap_hsmmc_host *host,
						   unsigned long bit)
{
	unsigned long i = 0;
	unsigned long limit = (loops_per_jiffy *
				msecs_to_jiffies(MMC_TIMEOUT_MS));

	OMAP_HSMMC_WRITE(host->base, SYSCTL,
			 OMAP_HSMMC_READ(host->base, SYSCTL) | bit);

	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & bit) &&
		(i++ < limit))
		cpu_relax();

	if (OMAP_HSMMC_READ(host->base, SYSCTL) & bit)
		dev_err(mmc_dev(host->mmc),
			"Timeout waiting on controller reset in %s\n",
			__func__);
}

/*
 * MMC controller IRQ handler
 */
static irqreturn_t omap_hsmmc_irq(int irq, void *dev_id)
{
	struct omap_hsmmc_host *host = dev_id;
	struct mmc_data *data;
	int end_cmd = 0, end_trans = 0, status;

	spin_lock(&host->irq_lock);

	data = host->data;
	status = OMAP_HSMMC_READ(host->base, STAT);
	dev_dbg(mmc_dev(host->mmc), "IRQ Status is %x\n", status);

	if (host->mmc->caps & MMC_CAP_SDIO_IRQ) {
		if (status & CIRQ) {
			dev_dbg(mmc_dev(host->mmc), "SDIO Card Interrupt\n");
			mmc_signal_sdio_irq(host->mmc);
		}
	}

	if (host->mrq == NULL) {
		OMAP_HSMMC_WRITE(host->base, STAT,
			OMAP_HSMMC_READ(host->base, STAT));
		/* Flush posted write */
		OMAP_HSMMC_READ(host->base, STAT);
		spin_unlock(&host->irq_lock);
		return IRQ_HANDLED;
	}

	if (status & ERR) {
#ifdef CONFIG_MMC_DEBUG
		omap_hsmmc_report_irq(host, status);
#endif
		if ((status & CMD_TIMEOUT) ||
			(status & CMD_CRC)) {
			if (host->cmd) {
				if (status & CMD_TIMEOUT) {
					omap_hsmmc_reset_controller_fsm(host,
									SRC);
					host->cmd->error = -ETIMEDOUT;
				} else {
					host->cmd->error = -EILSEQ;
				}
				end_cmd = 1;
			}
			if (host->data || host->response_busy) {
				if (host->data)
					omap_hsmmc_dma_cleanup(host,
								-ETIMEDOUT);
				host->response_busy = 0;
				omap_hsmmc_reset_controller_fsm(host, SRD);
			}
		}
		if ((status & DATA_TIMEOUT) ||
			(status & DATA_CRC)) {
			if (host->data || host->response_busy) {
				int err = (status & DATA_TIMEOUT) ?
						-ETIMEDOUT : -EILSEQ;

				if (host->data) {
					printk(KERN_ERR "TARR - %s:%d - %d (%d)\n",__FUNCTION__,
								__LINE__,err,host->dpm_state);
					omap_hsmmc_dma_cleanup(host, err);
				}
				else {
					printk(KERN_ERR "TARR - %s:%d - %d\n",__FUNCTION__,
								__LINE__,err);
					host->mrq->cmd->error = err;
				}
				host->response_busy = 0;
				omap_hsmmc_reset_controller_fsm(host, SRD);
				end_trans = 1;
			}
		}
		if (status & CARD_ERR) {
			dev_dbg(mmc_dev(host->mmc),
				"Ignoring card err CMD%d\n", host->cmd->opcode);
			if (host->cmd)
				end_cmd = 1;
			if (host->data)
				end_trans = 1;
		}
	}

	OMAP_HSMMC_WRITE(host->base, STAT, status);
	/* Flush posted write */
	OMAP_HSMMC_READ(host->base, STAT);

	if (end_cmd || ((status & CC) && host->cmd))
		omap_hsmmc_cmd_done(host, host->cmd);
	if ((end_trans || (status & TC)) && host->mrq)
		omap_hsmmc_xfer_done(host, data);

	spin_unlock(&host->irq_lock);

	return IRQ_HANDLED;
}

static void set_sd_bus_power(struct omap_hsmmc_host *host)
{
	unsigned long i;

	OMAP_HSMMC_WRITE(host->base, HCTL,
			 OMAP_HSMMC_READ(host->base, HCTL) | SDBP);
	for (i = 0; i < loops_per_jiffy; i++) {
		if (OMAP_HSMMC_READ(host->base, HCTL) & SDBP)
			break;
		cpu_relax();
	}
}

/*
 * Switch MMC interface voltage ... only relevant for MMC1.
 *
 * MMC2 and MMC3 use fixed 1.8V levels, and maybe a transceiver.
 * The MMC2 transceiver controls are used instead of DAT4..DAT7.
 * Some chips, like eMMC ones, use internal transceivers.
 */
static int omap_hsmmc_switch_opcond(struct omap_hsmmc_host *host, int vdd)
{
	u32 reg_val = 0;
	int ret;

	/* Disable the clocks */
	clk_disable(host->fclk);
	clk_disable(host->iclk);
	if (host->got_dbclk)
		clk_disable(host->dbclk);

	/* Turn the power off */
	ret = mmc_slot(host).set_power(host->dev, host->slot_id, 0, 0);

	/* Turn the power ON with given VDD 1.8 or 3.0v */
	if (!ret)
		ret = mmc_slot(host).set_power(host->dev, host->slot_id, 1,
					       vdd);
	clk_enable(host->iclk);
	clk_enable(host->fclk);
	if (host->got_dbclk)
		clk_enable(host->dbclk);

	if (ret != 0)
		goto err;

	OMAP_HSMMC_WRITE(host->base, HCTL,
		OMAP_HSMMC_READ(host->base, HCTL) & SDVSCLR);
	reg_val = OMAP_HSMMC_READ(host->base, HCTL);

	/*
	 * If a MMC dual voltage card is detected, the set_ios fn calls
	 * this fn with VDD bit set for 1.8V. Upon card removal from the
	 * slot, omap_hsmmc_set_ios sets the VDD back to 3V on MMC_POWER_OFF.
	 *
	 * Cope with a bit of slop in the range ... per data sheets:
	 *  - "1.8V" for vdds_mmc1/vdds_mmc1a can be up to 2.45V max,
	 *    but recommended values are 1.71V to 1.89V
	 *  - "3.0V" for vdds_mmc1/vdds_mmc1a can be up to 3.5V max,
	 *    but recommended values are 2.7V to 3.3V
	 *
	 * Board setup code shouldn't permit anything very out-of-range.
	 * TWL4030-family VMMC1 and VSIM regulators are fine (avoiding the
	 * middle range) but VSIM can't power DAT4..DAT7 at more than 3V.
	 */
	if ((1 << vdd) <= MMC_VDD_23_24)
		reg_val |= SDVS18;
	else
		reg_val |= SDVS30;

	OMAP_HSMMC_WRITE(host->base, HCTL, reg_val);
	set_sd_bus_power(host);

	return 0;
err:
	dev_dbg(mmc_dev(host->mmc), "Unable to switch operating voltage\n");
	return ret;
}

/* Protect the card while the cover is open */
static void omap_hsmmc_protect_card(struct omap_hsmmc_host *host)
{
	if (!mmc_slot(host).get_cover_state)
		return;

	host->reqs_blocked = 0;
	if (mmc_slot(host).get_cover_state(host->dev, host->slot_id)) {
		if (host->protect_card) {
			printk(KERN_INFO "%s: cover is closed, "
					 "card is now accessible\n",
					 mmc_hostname(host->mmc));
			host->protect_card = 0;
		}
	} else {
		if (!host->protect_card) {
			printk(KERN_INFO "%s: cover is open, "
					 "card is now inaccessible\n",
					 mmc_hostname(host->mmc));
			host->protect_card = 1;
		}
	}
}

/*
 * Work Item to notify the core about card insertion/removal
 */
static void omap_hsmmc_detect(struct work_struct *work)
{
	struct omap_hsmmc_host *host =
		container_of(work, struct omap_hsmmc_host, mmc_carddetect_work);
	struct omap_mmc_slot_data *slot = &mmc_slot(host);
	int carddetect;

	if (host->suspended)
		return;

	sysfs_notify(&host->mmc->class_dev.kobj, NULL, "cover_switch");

	if (slot->card_detect)
		carddetect = slot->card_detect(slot->card_detect_irq);
	else {
		omap_hsmmc_protect_card(host);
		carddetect = -ENOSYS;
	}

	if (carddetect) {
		mmc_detect_change(host->mmc, (HZ * 200) / 1000);
	} else {
		mmc_host_enable(host->mmc);
		omap_hsmmc_reset_controller_fsm(host, SRD);
		mmc_host_lazy_disable(host->mmc);

		mmc_detect_change(host->mmc, (HZ * 50) / 1000);
	}
}

/*
 * ISR for handling card insertion and removal
 */
static irqreturn_t omap_hsmmc_cd_handler(int irq, void *dev_id)
{
	struct omap_hsmmc_host *host = (struct omap_hsmmc_host *)dev_id;

	if (host->suspended)
		return IRQ_HANDLED;
	schedule_work(&host->mmc_carddetect_work);

	return IRQ_HANDLED;
}

static int omap_hsmmc_get_dma_sync_dev(struct omap_hsmmc_host *host,
				     struct mmc_data *data)
{
	int sync_dev;

	if (data->flags & MMC_DATA_WRITE)
		sync_dev = host->dma_line_tx;
	else
		sync_dev = host->dma_line_rx;
	return sync_dev;
}

static void omap_hsmmc_config_dma_params(struct omap_hsmmc_host *host,
				       struct mmc_data *data,
				       struct scatterlist *sgl)
{
	int blksz, nblk, dma_ch;

	dma_ch = host->dma_ch;
	if (data->flags & MMC_DATA_WRITE) {
		omap_set_dma_dest_params(dma_ch, 0, OMAP_DMA_AMODE_CONSTANT,
			(host->mapbase + OMAP_HSMMC_DATA), 0, 0);
		omap_set_dma_src_params(dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
			sg_dma_address(sgl), 0, 0);
	} else {
		omap_set_dma_src_params(dma_ch, 0, OMAP_DMA_AMODE_CONSTANT,
			(host->mapbase + OMAP_HSMMC_DATA), 0, 0);
		omap_set_dma_dest_params(dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
			sg_dma_address(sgl), 0, 0);
	}

	blksz = host->data->blksz;
	nblk = sg_dma_len(sgl) / blksz;

	omap_set_dma_transfer_params(dma_ch, OMAP_DMA_DATA_TYPE_S32,
			blksz / 4, nblk, OMAP_DMA_SYNC_FRAME,
			omap_hsmmc_get_dma_sync_dev(host, data),
			!(data->flags & MMC_DATA_WRITE));

	omap_start_dma(dma_ch);
}

/*
 * DMA call back function
 */
static void omap_hsmmc_dma_cb(int lch, u16 ch_status, void *data)
{
	struct omap_hsmmc_host *host = data;

	if (ch_status & OMAP2_DMA_MISALIGNED_ERR_IRQ)
		dev_dbg(mmc_dev(host->mmc), "MISALIGNED_ADRS_ERR\n");

	if (host->dma_ch < 0)
		return;

	host->dma_sg_idx++;
	if (host->dma_sg_idx < host->dma_len) {
		/* Fire up the next transfer. */
		omap_hsmmc_config_dma_params(host, host->data,
					   host->data->sg + host->dma_sg_idx);
		return;
	}

	omap_free_dma(host->dma_ch);
	host->dma_ch = -1;
	/*
	 * DMA Callback: run in interrupt context.
	 * mutex_unlock will throw a kernel warning if used.
	 */
	up(&host->sem);
}

/*
 * Routine to configure and start DMA for the MMC card
 */
static int omap_hsmmc_start_dma_transfer(struct omap_hsmmc_host *host,
					struct mmc_request *req)
{
	int dma_ch = 0, ret = 0, err = 1, i;
	struct mmc_data *data = req->data;

	/* Sanity check: all the SG entries must be aligned by block size. */
	for (i = 0; i < data->sg_len; i++) {
		struct scatterlist *sgl;

		sgl = data->sg + i;
		if (sgl->length % data->blksz)
			return -EINVAL;
	}
	if ((data->blksz % 4) != 0)
		/* REVISIT: The MMC buffer increments only when MSB is written.
		 * Return error for blksz which is non multiple of four.
		 */
		return -EINVAL;

	/*
	 * If for some reason the DMA transfer is still active,
	 * we wait for timeout period and free the dma
	 */
	if (host->dma_ch != -1) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(100);
		if (down_trylock(&host->sem)) {
			omap_free_dma(host->dma_ch);
			host->dma_ch = -1;
			up(&host->sem);
			return err;
		}
	} else {
		if (down_trylock(&host->sem))
			return err;
	}

	ret = omap_request_dma(omap_hsmmc_get_dma_sync_dev(host, data),
			       "MMC/SD", omap_hsmmc_dma_cb, host, &dma_ch);
	if (ret != 0) {
		dev_err(mmc_dev(host->mmc),
			"%s: omap_request_dma() failed with %d\n",
			mmc_hostname(host->mmc), ret);
		return ret;
	}

	host->dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg,
			data->sg_len, omap_hsmmc_get_dma_dir(host, data));
	host->dma_ch = dma_ch;
	host->dma_sg_idx = 0;

	omap_hsmmc_config_dma_params(host, data, data->sg);

	return 0;
}

static void set_data_timeout(struct omap_hsmmc_host *host,
			     unsigned int timeout_ns,
			     unsigned int timeout_clks)
{
	unsigned int timeout, cycle_ns;
	uint32_t reg, clkd, dto = 0;

	reg = OMAP_HSMMC_READ(host->base, SYSCTL);
	clkd = (reg & CLKD_MASK) >> CLKD_SHIFT;
	if (clkd == 0)
		clkd = 1;

	cycle_ns = 1000000000 / (clk_get_rate(host->fclk) / clkd);
	timeout = timeout_ns / cycle_ns;
	timeout += timeout_clks;
	if (timeout) {
		while ((timeout & 0x80000000) == 0) {
			dto += 1;
			timeout <<= 1;
		}
		dto = 31 - dto;
		timeout <<= 1;
		if (timeout && dto)
			dto += 1;
		if (dto >= 13)
			dto -= 13;
		else
			dto = 0;
		if (dto > 14)
			dto = 14;
	}

	reg &= ~DTO_MASK;
	reg |= dto << DTO_SHIFT;
	OMAP_HSMMC_WRITE(host->base, SYSCTL, reg);
}

/*
 * Configure block length for MMC/SD cards and initiate the transfer.
 */
static int
omap_hsmmc_prepare_data(struct omap_hsmmc_host *host, struct mmc_request *req)
{
	int ret;
	host->data = req->data;

	if (req->data == NULL) {
		OMAP_HSMMC_WRITE(host->base, BLK, 0);
		/*
		 * Set an arbitrary 100ms data timeout for commands with
		 * busy signal.
		 */
		if (req->cmd->flags & MMC_RSP_BUSY)
			set_data_timeout(host, 100000000U, 0);
		return 0;
	}

	OMAP_HSMMC_WRITE(host->base, BLK, (req->data->blksz)
					| (req->data->blocks << 16));
	set_data_timeout(host, req->data->timeout_ns, req->data->timeout_clks);

	if (host->use_dma) {
		ret = omap_hsmmc_start_dma_transfer(host, req);
		if (ret != 0) {
			dev_dbg(mmc_dev(host->mmc), "MMC start dma failure\n");
			return ret;
		}
	}
	return 0;
}

/*
 * Request function. for read/write operation
 */
static void omap_hsmmc_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	int err;

	/*
	 * Prevent races with the interrupt handler because of unexpected
	 * interrupts, but not if we are already in interrupt context i.e.
	 * retries.
	 */
	if (!in_interrupt()) {
		spin_lock_irqsave(&host->irq_lock, host->flags);
		/*
		 * Protect the card from I/O if there is a possibility
		 * it can be removed.
		 */
		if (host->protect_card) {
			if (host->reqs_blocked < 3) {
				/*
				 * Ensure the controller is left in a consistent
				 * state by resetting the command and data state
				 * machines.
				 */
				omap_hsmmc_reset_controller_fsm(host, SRD);
				omap_hsmmc_reset_controller_fsm(host, SRC);
				host->reqs_blocked += 1;
			}
			req->cmd->error = -EBADF;
			if (req->data)
				req->data->error = -EBADF;
			spin_unlock_irqrestore(&host->irq_lock, host->flags);
			mmc_request_done(mmc, req);
			return;
		} else if (host->reqs_blocked)
			host->reqs_blocked = 0;
	}
	WARN_ON(host->mrq != NULL);
	host->mrq = req;
	err = omap_hsmmc_prepare_data(host, req);
	if (err) {
		req->cmd->error = err;
		if (req->data)
			req->data->error = err;
		host->mrq = NULL;
		if (!in_interrupt())
			spin_unlock_irqrestore(&host->irq_lock, host->flags);
		mmc_request_done(mmc, req);
		return;
	}

	omap_hsmmc_start_command(host, req->cmd, req->data);
}

/* Routine to configure clock values. Exposed API to core */
void omap_hsmmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	u16 dsor = 0;
	unsigned long regval;
	unsigned long timeout;
	u32 con;
	int do_send_init_stream = 0;

	mmc_host_enable(host->mmc);

	if (ios->power_mode != host->power_mode) {
		switch (ios->power_mode) {
		case MMC_POWER_OFF:
			if (mmc_slot(host).set_power)
				mmc_slot(host).set_power(host->dev, 
					host->slot_id, 0, 0);
			break;
		case MMC_POWER_UP:
			if (mmc_slot(host).set_power)
				mmc_slot(host).set_power(host->dev, 
					host->slot_id, 1, ios->vdd);
			break;
		case MMC_POWER_ON:
			do_send_init_stream = 1;
			break;
		}
		host->power_mode = ios->power_mode;
	}

	/* FIXME: set registers based only on changes to ios */

	con = OMAP_HSMMC_READ(host->base, CON);
	switch (mmc->ios.bus_width) {
	case MMC_BUS_WIDTH_8:
		OMAP_HSMMC_WRITE(host->base, CON, con | DW8);
		break;
	case MMC_BUS_WIDTH_4:
		OMAP_HSMMC_WRITE(host->base, CON, con & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | FOUR_BIT);
		break;
	case MMC_BUS_WIDTH_1:
		OMAP_HSMMC_WRITE(host->base, CON, con & ~DW8);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) & ~FOUR_BIT);
		break;
	}

	if (host->id == OMAP_MMC1_DEVID) {
		/* Only MMC1 can interface at 3V without some flavor
		 * of external transceiver; but they all handle 1.8V.
		 */
		if ((OMAP_HSMMC_READ(host->base, HCTL) & SDVSDET) &&
			(ios->vdd == DUAL_VOLT_OCR_BIT)) {
				/*
				 * The mmc_select_voltage fn of the core does
				 * not seem to set the power_mode to
				 * MMC_POWER_UP upon recalculating the voltage.
				 * vdd 1.8v.
				 */
			if (omap_hsmmc_switch_opcond(host, ios->vdd) != 0)
				dev_dbg(mmc_dev(host->mmc),
						"Switch operation failed\n");
		}
	}

	if (ios->clock) {
		dsor = OMAP_MMC_MASTER_CLOCK / ios->clock;
		if (dsor < 1)
			dsor = 1;

		if (OMAP_MMC_MASTER_CLOCK / dsor > ios->clock)
			dsor++;

		if (dsor > 250)
			dsor = 250;
	}
	omap_hsmmc_stop_clock(host);
	regval = OMAP_HSMMC_READ(host->base, SYSCTL);
	regval = regval & ~(CLKD_MASK);
	regval = regval | (dsor << 6) | (DTO << 16);
	OMAP_HSMMC_WRITE(host->base, SYSCTL, regval);
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | ICE);

	/* Wait till the ICS bit is set */
	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & ICS) != ICS
		&& time_before(jiffies, timeout))
		msleep(1);

	OMAP_HSMMC_WRITE(host->base, SYSCTL,
		OMAP_HSMMC_READ(host->base, SYSCTL) | CEN);

	if (do_send_init_stream)
		send_init_stream(host);

	con = OMAP_HSMMC_READ(host->base, CON);
	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
		OMAP_HSMMC_WRITE(host->base, CON, con | OD);
	else
		OMAP_HSMMC_WRITE(host->base, CON, con & ~OD);

	if (host->power_mode == MMC_POWER_OFF)
		mmc_host_disable(host->mmc);
	else
		mmc_host_lazy_disable(host->mmc);
}

#ifdef CONFIG_MMC_EMBEDDED_SDIO
void omap_hsmmc_status_notify_cb(int card_present, void *dev_id)
{
       struct omap_hsmmc_host *host = dev_id;
       struct omap_mmc_slot_data *slot = &mmc_slot(host);

       printk(KERN_DEBUG "%s: card_present %d\n", mmc_hostname(host->mmc),
		card_present);
       host->carddetect = slot->card_detect(slot->card_detect_irq);

       sysfs_notify(&host->mmc->class_dev.kobj, NULL, "cover_switch");

	if (host->carddetect) {
		mmc_detect_change(host->mmc, (HZ * 200) / 1000);
	} else {
		mmc_host_enable(host->mmc);
		omap_hsmmc_reset_controller_fsm(host, SRD);
		mmc_host_lazy_disable(host->mmc);
		mmc_detect_change(host->mmc, (HZ * 50) / 1000);
	}
}
#endif

int omap_hsmmc_get_cd(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	int cd = -1;

	if (mmc_slot(host).card_detect) {
		cd = mmc_slot(host).card_detect(mmc_slot(host).card_detect_irq);
		return cd;
	} else {
	if (host->id == OMAP_MMC1_DEVID || host->id == OMAP_MMC2_DEVID
		|| host->id == OMAP_MMC3_DEVID)
			return 1;
	}
		return -ENOSYS;
}

static int omap_hsmmc_get_ro(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	if (!mmc_slot(host).get_ro)
		return -ENOSYS;
	return mmc_slot(host).get_ro(host->dev, 0);
}

static void omap_hsmmc_conf_bus_power(struct omap_hsmmc_host *host)
{
	u32 hctl, capa, value;

	//printk(KERN_INFO "TARR - %s\n",__FUNCTION__);
	/* Only MMC1 supports 3.0V */
	if (host->id == OMAP_MMC1_DEVID) {
		hctl = SDVS30;
		capa = VS30 | VS18;
	} else {
		hctl = SDVS18;
		capa = VS18;
	}

	value = OMAP_HSMMC_READ(host->base, HCTL) & ~SDVS_MASK;
	OMAP_HSMMC_WRITE(host->base, HCTL, value | hctl);

	value = OMAP_HSMMC_READ(host->base, CAPA);
	OMAP_HSMMC_WRITE(host->base, CAPA, value | capa);

	/* Set the controller to AUTO IDLE mode */
	/* TARR - More no sure of stuff */
	value = OMAP_HSMMC_READ(host->base, SYSCONFIG);
	OMAP_HSMMC_WRITE(host->base, SYSCONFIG, value | AUTOIDLE);
	//OMAP_HSMMC_WRITE(host->base, SYSCONFIG, value | SIDLEMODE_IGNORE);

	/* Set SD bus power bit */
	set_sd_bus_power(host);
}

/*
 * Dynamic power saving handling, FSM:
 *   ENABLED -> DISABLED -> CARDSLEEP / REGSLEEP -> OFF
 *     ^___________|          |                      |
 *     |______________________|______________________|
 *
 * ENABLED:   mmc host is fully functional
 * DISABLED:  fclk is off
 * CARDSLEEP: fclk is off, card is asleep, voltage regulator is asleep
 * REGSLEEP:  fclk is off, voltage regulator is asleep
 * OFF:       fclk is off, voltage regulator is off
 *
 * Transition handlers return the timeout for the next state transition
 * or negative error.
 */

enum {ENABLED = 0, DISABLED, CARDSLEEP, REGSLEEP, OFF};

/* Handler for [ENABLED -> DISABLED] transition */
int omap_hsmmc_enabled_to_disabled(struct omap_hsmmc_host *host)
{
	//printk(KERN_INFO "TARR - %s\n",__FUNCTION__);
	omap_hsmmc_context_save(host);
	clk_disable(host->fclk);
	host->dpm_state = DISABLED;

	dev_dbg(mmc_dev(host->mmc), "ENABLED -> DISABLED\n");

	if (host->power_mode == MMC_POWER_OFF)
		return 0;

	return msecs_to_jiffies(OMAP_MMC_SLEEP_TIMEOUT);
}

/* Handler for [DISABLED -> REGSLEEP / CARDSLEEP] transition */
int omap_hsmmc_disabled_to_sleep(struct omap_hsmmc_host *host)
{
	int err, new_state;

	//printk(KERN_INFO "TARR - %s\n",__FUNCTION__);
	if (!mmc_try_claim_host(host->mmc))
		return 0;

	clk_enable(host->fclk);
	omap_hsmmc_context_restore(host);
	if (mmc_card_can_sleep(host->mmc)) {
		err = mmc_card_sleep(host->mmc);
		if (err < 0) {
			clk_disable(host->fclk);
			mmc_release_host(host->mmc);
			return err;
		}
		new_state = CARDSLEEP;
	} else {
		new_state = REGSLEEP;
	}
	if (mmc_slot(host).set_sleep)
		mmc_slot(host).set_sleep(host->dev, host->slot_id, 1, 0,
					 new_state == CARDSLEEP);
	/* FIXME: turn off bus power and perhaps interrupts too */
	clk_disable(host->fclk);
	host->dpm_state = new_state;

	mmc_release_host(host->mmc);

	dev_dbg(mmc_dev(host->mmc), "DISABLED -> %s\n",
		host->dpm_state == CARDSLEEP ? "CARDSLEEP" : "REGSLEEP");

	if ((host->mmc->caps & MMC_CAP_NONREMOVABLE) ||
	    mmc_slot(host).card_detect ||
	    (mmc_slot(host).get_cover_state &&
	     mmc_slot(host).get_cover_state(host->dev, host->slot_id)))
		return msecs_to_jiffies(OMAP_MMC_OFF_TIMEOUT);

	return 0;
}

/* Handler for [REGSLEEP / CARDSLEEP -> OFF] transition */
int omap_hsmmc_sleep_to_off(struct omap_hsmmc_host *host)
{
	//printk(KERN_INFO "TARR - %s\n",__FUNCTION__);
	if (!mmc_try_claim_host(host->mmc))
		return 0;

	if (!((host->mmc->caps & MMC_CAP_NONREMOVABLE) ||
	      mmc_slot(host).card_detect ||
	      (mmc_slot(host).get_cover_state &&
	       mmc_slot(host).get_cover_state(host->dev, host->slot_id)))) {
		mmc_release_host(host->mmc);
		return 0;
	}

	mmc_slot(host).set_power(host->dev, host->slot_id, 0, 0);
	host->vdd = 0;
	host->power_mode = MMC_POWER_OFF;

	dev_dbg(mmc_dev(host->mmc), "%s -> OFF\n",
		host->dpm_state == CARDSLEEP ? "CARDSLEEP" : "REGSLEEP");

	host->dpm_state = OFF;

	mmc_release_host(host->mmc);

	return 0;
}

/* Handler for [DISABLED -> ENABLED] transition */
int omap_hsmmc_disabled_to_enabled(struct omap_hsmmc_host *host)
{
	int err;

	//printk(KERN_INFO "TARR - %s\n",__FUNCTION__);
	err = clk_enable(host->fclk);
	if (err < 0)
		return err;

	omap_hsmmc_context_restore(host);
	host->dpm_state = ENABLED;

	dev_dbg(mmc_dev(host->mmc), "DISABLED -> ENABLED\n");

	return 0;
}

/* Handler for [SLEEP -> ENABLED] transition */
int omap_hsmmc_sleep_to_enabled(struct omap_hsmmc_host *host)
{
	//printk(KERN_INFO "TARR - %s\n",__FUNCTION__);
	if (!mmc_try_claim_host(host->mmc))
		return 0;

	clk_enable(host->fclk);
	omap_hsmmc_context_restore(host);
	if (mmc_slot(host).set_sleep)
		mmc_slot(host).set_sleep(host->dev, host->slot_id, 0,
			 host->vdd, host->dpm_state == CARDSLEEP);
	if (mmc_card_can_sleep(host->mmc))
		mmc_card_awake(host->mmc);

	dev_dbg(mmc_dev(host->mmc), "%s -> ENABLED\n",
		host->dpm_state == CARDSLEEP ? "CARDSLEEP" : "REGSLEEP");

	host->dpm_state = ENABLED;

	mmc_release_host(host->mmc);

	return 0;
}

/* Handler for [OFF -> ENABLED] transition */
int omap_hsmmc_off_to_enabled(struct omap_hsmmc_host *host)
{
	//printk(KERN_INFO "TARR - %s\n",__FUNCTION__);
	clk_enable(host->fclk);

	omap_hsmmc_context_restore(host);
	omap_hsmmc_conf_bus_power(host);
	mmc_power_restore_host(host->mmc);

	host->dpm_state = ENABLED;

	dev_dbg(mmc_dev(host->mmc), "OFF -> ENABLED\n");

	return 0;
}

/*
 * Bring MMC host to ENABLED from any other PM state.
 */
int omap_hsmmc_enable(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	//printk(KERN_INFO "TARR - %s\n",__FUNCTION__);
	switch (host->dpm_state) {
	case DISABLED:
		return omap_hsmmc_disabled_to_enabled(host);
	case CARDSLEEP:
	case REGSLEEP:
		return omap_hsmmc_sleep_to_enabled(host);
	case OFF:
		return omap_hsmmc_off_to_enabled(host);
	default:
		dev_dbg(mmc_dev(host->mmc), "UNKNOWN state\n");
		return -EINVAL;
	}
}

/*
 * Bring MMC host in PM state (one level deeper).
 */
int omap_hsmmc_disable(struct mmc_host *mmc, int lazy)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	//printk(KERN_INFO "TARR - %s\n",__FUNCTION__);
	switch (host->dpm_state) {
	case ENABLED: {
		int delay;

		delay = omap_hsmmc_enabled_to_disabled(host);
		if (lazy || delay < 0)
			return delay;
		return 0;
	}
	case DISABLED:
		return omap_hsmmc_disabled_to_sleep(host);
	case CARDSLEEP:
	case REGSLEEP:
		return omap_hsmmc_sleep_to_off(host);
	default:
		dev_dbg(mmc_dev(host->mmc), "UNKNOWN state\n");
		return -EINVAL;
	}
}

int omap_hsmmc_enable_fclk(struct mmc_host *mmc)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	int err;

	//printk(KERN_INFO "TARR - %s - %s\n",__FUNCTION__,mmc_hostname(host->mmc));
	err = clk_enable(host->fclk);
	if (err)
		return err;
	dev_dbg(mmc_dev(host->mmc), "mmc_fclk: enabled\n");
	omap_hsmmc_context_restore(host);
	return 0;
}

int omap_hsmmc_disable_fclk(struct mmc_host *mmc, int lazy)
{
	struct omap_hsmmc_host *host = mmc_priv(mmc);

	//printk(KERN_INFO "TARR - %s - %s\n",__FUNCTION__,mmc_hostname(host->mmc));
	omap_hsmmc_context_save(host);
	//clk_disable(host->fclk);
	dev_dbg(mmc_dev(host->mmc), "mmc_fclk: disabled\n");
	return 0;
}

static const struct mmc_host_ops omap_hsmmc_ops = {
	.enable = omap_hsmmc_enable_fclk,
	.disable = omap_hsmmc_disable_fclk,
	.request = omap_hsmmc_request,
	.set_ios = omap_hsmmc_set_ios,
	.get_cd = omap_hsmmc_get_cd,
	.get_ro = omap_hsmmc_get_ro,
	/* NYET -- enable_sdio_irq */
};

static const struct mmc_host_ops omap_hsmmc_ps_ops = {
	.enable = omap_hsmmc_enable,
	.disable = omap_hsmmc_disable,
	.request = omap_hsmmc_request,
	.set_ios = omap_hsmmc_set_ios,
	.get_cd = omap_hsmmc_get_cd,
	.get_ro = omap_hsmmc_get_ro,
	/* NYET -- enable_sdio_irq */
};

#ifdef CONFIG_DEBUG_FS

int omap_hsmmc_regs_show(struct seq_file *s, void *data)
{
	struct mmc_host *mmc = s->private;
	struct omap_hsmmc_host *host = mmc_priv(mmc);
	int context_loss = 0;

	if (host->pdata->get_context_loss_count)
		context_loss = host->pdata->get_context_loss_count(host->dev);

	seq_printf(s, "mmc%d:\n"
			" enabled:\t%d\n"
			" dpm_state:\t%d\n"
			" nesting_cnt:\t%d\n"
			" ctx_loss:\t%d:%d\n"
			"\nregs:\n",
			mmc->index, mmc->enabled ? 1 : 0,
			host->dpm_state, mmc->nesting_cnt,
			host->context_loss, context_loss);

	if (host->suspended || host->dpm_state == OFF) {
		seq_printf(s, "host suspended, can't read registers\n");
		return 0;
	}

	if (clk_enable(host->fclk) != 0) {
		seq_printf(s, "can't read the regs\n");
		return 0;
	}

	seq_printf(s, "SYSCONFIG:\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, SYSCONFIG));
	seq_printf(s, "CON:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, CON));
	seq_printf(s, "HCTL:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, HCTL));
	seq_printf(s, "SYSCTL:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, SYSCTL));
	seq_printf(s, "IE:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, IE));
	seq_printf(s, "ISE:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, ISE));
	seq_printf(s, "CAPA:\t\t0x%08x\n",
			OMAP_HSMMC_READ(host->base, CAPA));

	clk_disable(host->fclk);

	return 0;
}

static int omap_hsmmc_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, omap_hsmmc_regs_show, inode->i_private);
}

static const struct file_operations mmc_regs_fops = {
	.open           = omap_hsmmc_regs_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static void omap_hsmmc_debugfs(struct mmc_host *mmc)
{
	if (mmc->debugfs_root)
		debugfs_create_file("regs", S_IRUSR, mmc->debugfs_root,
			mmc, &mmc_regs_fops);
}

#else

static void omap_hsmmc_debugfs(struct mmc_host *mmc)
{
}

#endif

static int __init omap_hsmmc_probe(struct platform_device *pdev)
{
	struct omap_mmc_platform_data *pdata = pdev->dev.platform_data;
	struct mmc_host *mmc;
	struct omap_hsmmc_host *host = NULL;
	struct resource *res;
	int ret = 0, irq;

	//printk(KERN_INFO "TARR - %s\n",__FUNCTION__);
	if (pdata == NULL) {
		dev_err(&pdev->dev, "Platform Data is missing\n");
		return -ENXIO;
	}

	if (pdata->nr_slots == 0) {
		dev_err(&pdev->dev, "No Slots\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (res == NULL || irq < 0)
		return -ENXIO;

	res = request_mem_region(res->start, res->end - res->start + 1,
							pdev->name);
	if (res == NULL)
		return -EBUSY;

	mmc = mmc_alloc_host(sizeof(struct omap_hsmmc_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto err;
	}

	host		= mmc_priv(mmc);
	host->mmc	= mmc;
	host->pdata	= pdata;
	host->dev	= &pdev->dev;
	host->use_dma	= 1;
	host->dev->dma_mask = &pdata->dma_mask;
	host->dma_ch	= -1;
	host->irq	= irq;
	host->id	= pdev->id;
	host->slot_id	= 0;
	host->mapbase	= res->start;
	host->base	= ioremap(host->mapbase, SZ_4K);

#ifdef CONFIG_MMC_EMBEDDED_SDIO
	if (pdev->id == CONFIG_TIWLAN_MMC_CONTROLLER - 1) {
		if (pdata->slots[0].embedded_sdio != NULL) {
			mmc_set_embedded_sdio_data(mmc,
						pdata->
						slots[0].embedded_sdio->cis,
						pdata->
						slots[0].embedded_sdio->cccr,
						pdata->
						slots[0].embedded_sdio->
						funcs,
						pdata->
						slots[0].embedded_sdio->
						num_funcs,
						pdata->
						slots[0].embedded_sdio->
						quirks);
		}
	}
#endif

	platform_set_drvdata(pdev, host);
	INIT_WORK(&host->mmc_carddetect_work, omap_hsmmc_detect);

	if (mmc_slot(host).power_saving)
		mmc->ops	= &omap_hsmmc_ps_ops;
	else
		mmc->ops	= &omap_hsmmc_ops;

	mmc->f_min	= 400000;
	mmc->f_max	= 52000000;

	sema_init(&host->sem, 1);
	spin_lock_init(&host->irq_lock);

	host->iclk = clk_get(&pdev->dev, "ick");
	if (IS_ERR(host->iclk)) {
		ret = PTR_ERR(host->iclk);
		host->iclk = NULL;
		goto err1;
	}
	host->fclk = clk_get(&pdev->dev, "fck");
	if (IS_ERR(host->fclk)) {
		ret = PTR_ERR(host->fclk);
		host->fclk = NULL;
		clk_put(host->iclk);
		goto err1;
	}

	omap_hsmmc_context_save(host);

	mmc->caps |= MMC_CAP_DISABLE;
	mmc_set_disable_delay(mmc, OMAP_MMC_DISABLED_TIMEOUT);
	/* we start off in DISABLED state */
	host->dpm_state = DISABLED;

	if (mmc_host_enable(host->mmc) != 0) {
		clk_put(host->iclk);
		clk_put(host->fclk);
		goto err1;
	}

	if (clk_enable(host->iclk) != 0) {
		mmc_host_disable(host->mmc);
		clk_put(host->iclk);
		clk_put(host->fclk);
		goto err1;
	}

	if (cpu_is_omap2430()) {
		host->dbclk = clk_get(&pdev->dev, "mmchsdb_fck");
		/*
		 * MMC can still work without debounce clock.
		 */
		if (IS_ERR(host->dbclk))
			dev_warn(mmc_dev(host->mmc),
				"Failed to get debounce clock\n");
		else
			host->got_dbclk = 1;

		if (host->got_dbclk)
			if (clk_enable(host->dbclk) != 0)
				dev_dbg(mmc_dev(host->mmc), "Enabling debounce"
							" clk failed\n");
	}

	/* Since we do only SG emulation, we can have as many segs
	 * as we want. */
	mmc->max_phys_segs = 1024;
	mmc->max_hw_segs = 1024;

	mmc->max_blk_size = 512;       /* Block Length at max can be 1024 */
	mmc->max_blk_count = 0xFFFF;    /* No. of Blocks is 16 bits */
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size = mmc->max_req_size;

	mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
		     MMC_CAP_WAIT_WHILE_BUSY;

	if (mmc_slot(host).wires >= 8)
		mmc->caps |= MMC_CAP_8_BIT_DATA;
	else if (mmc_slot(host).wires >= 4)
		mmc->caps |= MMC_CAP_4_BIT_DATA;

	if (mmc_slot(host).nonremovable)
		mmc->caps |= MMC_CAP_NONREMOVABLE;

#if defined(CONFIG_MACH_OMAP3530_LV_SOM) || defined(CONFIG_MACH_DM3730_SOM_LV)
	// set the "no multi block" flag if need be
	if (pdata->slots[host->slot_id].no_multi_block)
		mmc->caps |= MMC_CAP_NO_MULTI_BLOCK;
	// set the "funky writeb_f0_status" flag if slot has problems
	// with the status return from a byte-write to the address
	// space of SDIO function 0.
	if (pdata->slots[host->slot_id].funky_f0_writeb_status)
		mmc->caps |= MMC_CAP_FUNKY_F0_WRITEB_STATUS;
#endif

	omap_hsmmc_conf_bus_power(host);

	/* Select DMA lines */
	switch (host->id) {
	case OMAP_MMC1_DEVID:
		host->dma_line_tx = OMAP24XX_DMA_MMC1_TX;
		host->dma_line_rx = OMAP24XX_DMA_MMC1_RX;
		break;
	case OMAP_MMC2_DEVID:
		host->dma_line_tx = OMAP24XX_DMA_MMC2_TX;
		host->dma_line_rx = OMAP24XX_DMA_MMC2_RX;
		break;
	case OMAP_MMC3_DEVID:
		host->dma_line_tx = OMAP34XX_DMA_MMC3_TX;
		host->dma_line_rx = OMAP34XX_DMA_MMC3_RX;
		break;
	case OMAP_MMC4_DEVID:
		host->dma_line_tx = OMAP44XX_DMA_MMC4_TX;
		host->dma_line_rx = OMAP44XX_DMA_MMC4_RX;
		break;
	case OMAP_MMC5_DEVID:
		host->dma_line_tx = OMAP44XX_DMA_MMC5_TX;
		host->dma_line_rx = OMAP44XX_DMA_MMC5_RX;
		break;
	default:
		dev_err(mmc_dev(host->mmc), "Invalid MMC id\n");
		goto err_irq;
	}

	/* Request IRQ for MMC operations */
	ret = request_irq(host->irq, omap_hsmmc_irq, IRQF_DISABLED,
			mmc_hostname(mmc), host);
	if (ret) {
		dev_dbg(mmc_dev(host->mmc), "Unable to grab HSMMC IRQ\n");
		goto err_irq;
	}

	/* initialize power supplies, gpios, etc */
	if (pdata->init != NULL) {
		if (pdata->init(&pdev->dev) != 0) {
			dev_dbg(mmc_dev(host->mmc),
				"Unable to configure MMC IRQs\n");
			goto err_irq_cd_init;
		}
	}
	mmc->ocr_avail = mmc_slot(host).ocr_mask;

	/* Request IRQ for card detect */
	if ((mmc_slot(host).card_detect_irq)) {
		ret = request_irq(mmc_slot(host).card_detect_irq,
				  omap_hsmmc_cd_handler,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
					  | IRQF_DISABLED,
				  mmc_hostname(mmc), host);
		if (ret) {
			dev_dbg(mmc_dev(host->mmc),
				"Unable to grab MMC CD IRQ\n");
			goto err_irq_cd;
		}
	}
#ifdef CONFIG_MMC_EMBEDDED_SDIO
	else if (mmc_slot(host).register_status_notify) {
		if (pdev->id == CONFIG_TIWLAN_MMC_CONTROLLER - 1) {
			mmc_slot(host).register_status_notify
			 (omap_hsmmc_status_notify_cb, host);
		}
	}
#endif

	OMAP_HSMMC_WRITE(host->base, ISE, INT_EN_MASK);
	OMAP_HSMMC_WRITE(host->base, IE, INT_EN_MASK);

	mmc_host_lazy_disable(host->mmc);

	omap_hsmmc_protect_card(host);

	mmc_add_host(mmc);

	if (mmc_slot(host).name != NULL) {
		ret = device_create_file(&mmc->class_dev, &dev_attr_slot_name);
		if (ret < 0)
			goto err_slot_name;
	}
	if (mmc_slot(host).card_detect_irq && mmc_slot(host).get_cover_state) {
		ret = device_create_file(&mmc->class_dev,
					&dev_attr_cover_switch);
		if (ret < 0)
			goto err_cover_switch;
	}

	omap_hsmmc_debugfs(mmc);

	return 0;

err_cover_switch:
	device_remove_file(&mmc->class_dev, &dev_attr_cover_switch);
err_slot_name:
	mmc_remove_host(mmc);
err_irq_cd:
	free_irq(mmc_slot(host).card_detect_irq, host);
err_irq_cd_init:
	free_irq(host->irq, host);
err_irq:
	mmc_host_disable(host->mmc);
	clk_disable(host->iclk);
	clk_put(host->fclk);
	clk_put(host->iclk);
	if (host->got_dbclk) {
		clk_disable(host->dbclk);
		clk_put(host->dbclk);
	}

err1:
	iounmap(host->base);
err:
	dev_dbg(mmc_dev(host->mmc), "Probe Failed\n");
	release_mem_region(res->start, res->end - res->start + 1);
	if (host)
		mmc_free_host(mmc);
	return ret;
}

static int omap_hsmmc_remove(struct platform_device *pdev)
{
	struct omap_hsmmc_host *host = platform_get_drvdata(pdev);
	struct resource *res;

	//printk(KERN_INFO "TARR -- %s\n",__FUNCTION__);
	if (host) {
		mmc_host_enable(host->mmc);
		mmc_remove_host(host->mmc);
		if (host->pdata->cleanup)
			host->pdata->cleanup(&pdev->dev);
		free_irq(host->irq, host);
		if (mmc_slot(host).card_detect_irq)
			free_irq(mmc_slot(host).card_detect_irq, host);
		flush_scheduled_work();

		mmc_host_disable(host->mmc);
		clk_disable(host->iclk);
		clk_put(host->fclk);
		clk_put(host->iclk);
		if (host->got_dbclk) {
			clk_disable(host->dbclk);
			clk_put(host->dbclk);
		}

		mmc_free_host(host->mmc);
		iounmap(host->base);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start, res->end - res->start + 1);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int omap_hsmmc_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret = 0;
	struct omap_hsmmc_host *host = platform_get_drvdata(pdev);

	//printk(KERN_INFO "TARR - %s\n",__FUNCTION__);

	/* This stuff is commented outso that the Wifi stays alive through
     * a suspend/resume cycle
     */
#ifdef XTARR
	if (host && host->suspended)
		return 0;

	if (host) {
		host->suspended = 1;
		if (host->pdata->suspend) {
			ret = host->pdata->suspend(&pdev->dev,
							host->slot_id);
			if (ret) {
				dev_dbg(mmc_dev(host->mmc),
					"Unable to handle MMC board"
					" level suspend\n");
				host->suspended = 0;
				printk(KERN_INFO "TARR - %s:%d\n",__FUNCTION__,__LINE__);
				return ret;
			}
		}
		cancel_work_sync(&host->mmc_carddetect_work);
		mmc_host_enable(host->mmc);
		ret = mmc_suspend_host(host->mmc, state);
		if (ret == 0) {
			printk(KERN_INFO "TARR - %s:%d\n",__FUNCTION__,__LINE__);
			OMAP_HSMMC_WRITE(host->base, ISE, 0);
			OMAP_HSMMC_WRITE(host->base, IE, 0);
			OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) & ~SDBP);
			mmc_host_disable(host->mmc);
			clk_disable(host->iclk);
			if (host->got_dbclk)
				clk_disable(host->dbclk);
		} else {
			printk(KERN_INFO "TARR - %s:%d\n",__FUNCTION__,__LINE__);
			host->suspended = 0;
			if (host->pdata->resume) {
				ret = host->pdata->resume(&pdev->dev, host->slot_id);
				if (ret)
					dev_dbg(mmc_dev(host->mmc), "Unmask interrupt failed\n");
			}
			mmc_host_disable(host->mmc);
		}

	}
#endif
	return ret;
}

/* Routine to resume the MMC device */
static int omap_hsmmc_resume(struct platform_device *pdev)
{
	int ret = 0;
	int line;
	struct omap_hsmmc_host *host = platform_get_drvdata(pdev);

	//printk(KERN_INFO "TARR - %s - (%d)\n",__FUNCTION__,host->dpm_state);
	/* TARR - The following is commented out so the Wifi stays alive
     * through a suspend/resume cycle
     */
#ifdef XTARR
	if (host && !host->suspended) {
		line =  __LINE__;
		ret = 0;
		goto resume_done;
	}

	line = __LINE__;
	if (host) {
		ret = clk_enable(host->iclk);
		if (ret) {
			line = __LINE__;
			goto resume_done;
		}
		if (mmc_host_enable(host->mmc) != 0) {
			line = __LINE__;
			clk_disable(host->iclk);
			goto resume_done;
		}

		if (host->got_dbclk)
			clk_enable(host->dbclk);

		omap_hsmmc_conf_bus_power(host);

		if (host->pdata->resume) {
			ret = host->pdata->resume(&pdev->dev, host->slot_id);
			if (ret) {
				printk(KERN_INFO "TARR - %s:%d\n",__FUNCTION__,__LINE__);
				dev_dbg(mmc_dev(host->mmc), "Unmask interrupt failed\n");
			}
		}

		omap_hsmmc_protect_card(host);

		/* Notify the core to resume the host */
		ret = mmc_resume_host(host->mmc);
		if (ret == 0) {
			line = __LINE__;
			host->suspended = 0;
		}

		mmc_host_lazy_disable(host->mmc);
		line = __LINE__;
	}

resume_done:
	//printk(KERN_INFO "TARR - %s:%d - %d\n",__FUNCTION__,line,ret);
#endif
	return ret;
}

#else
#define omap_hsmmc_suspend	NULL
#define omap_hsmmc_resume		NULL
#endif

static struct platform_driver omap_hsmmc_driver = {
	.remove		= omap_hsmmc_remove,
	.suspend	= omap_hsmmc_suspend,
	.resume		= omap_hsmmc_resume,
	.driver		= {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

int __init omap_hsmmc_init(void)
{
	/* Register the MMC driver */
	//printk(KERN_INFO "TARR - %s\n",__FUNCTION__);
	return platform_driver_probe(&omap_hsmmc_driver, omap_hsmmc_probe);
}

static void __exit omap_hsmmc_cleanup(void)
{
	/* Unregister MMC driver */
	platform_driver_unregister(&omap_hsmmc_driver);
}

module_init(omap_hsmmc_init);
module_exit(omap_hsmmc_cleanup);

MODULE_DESCRIPTION("OMAP High Speed Multimedia Card driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
