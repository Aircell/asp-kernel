/***************************************************************************
**+----------------------------------------------------------------------+**
**|                                ****                                  |**
**|                                ****                                  |**
**|                                ******o***                            |**
**|                          ********_///_****                           |**
**|                           ***** /_//_/ ****                          |**
**|                            ** ** (__/ ****                           |**
**|                                *********                             |**
**|                                 ****                                 |**
**|                                  ***                                 |**
**|                                                                      |**
**|     Copyright (c) 1998 - 2010 Texas Instruments Incorporated         |**
**|                        ALL RIGHTS RESERVED                           |**
**|                                                                      |**
**| Permission is hereby granted to licensees of Texas Instruments       |**
**| Incorporated (TI) products to use this computer program for the sole |**
**| purpose of implementing a licensee product based on TI products.     |**
**| No other rights to reproduce, use, or disseminate this computer      |**
**| program, whether in part or in whole, are granted.                   |**
**|                                                                      |**
**| TI makes no representation or warranties with respect to the         |**
**| performance of this computer program, and specifically disclaims     |**
**| any responsibility for any damages, special or consequential,        |**
**| connected with the use of this program.                              |**
**|                                                                      |**
**+----------------------------------------------------------------------+**
***************************************************************************/
 
/** \file   SdioDrv.c 
 *  \brief  The OMAP3530 Linux SDIO driver (platform and OS dependent) 
 * 
 * The lower SDIO driver (BSP) for OMAP3530 on Linux OS.
 * Provides all SDIO commands and read/write operation methods.
 *  
 *  \see    SdioDrv.h
 */

/*
 * modification history
 * --------------------
 * \version xxx,DDMMMYY,yyy details of modification
 */
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22)
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
#include <linux/i2c/twl.h>
#include <plat/board.h>
#include <plat/clock.h>
#include <plat/dma.h>

#else
#include <mach/io.h>
#include <mach/hardware.h>
#include <linux/i2c/twl4030.h>
#include <mach/board.h>
#include <mach/clock.h>
#include <mach/dma.h>
#endif
#else
#include <asm/arch/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/twl4030.h>
#include <asm/arch/board.h>
#include <asm/arch/clock.h>
#include <asm/arch/dma.h>
#endif
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <plat/resource.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>

#include "SdioDrvDbg.h"
#include "SdioDrv.h"


#define TIWLAN_MMC_MAX_DMA          (8192)

#define SDIO_DRIVER_NAME 			"TIWLAN_SDIO" 
#define SDIO_TOTAL_FUNCS			(2)
#define SDIO_WLAN_FUNC				(2)
#define SDIO_BT_FUNC				(1)
#define SDIO_CTRL_FUNC				(0)

typedef struct OMAP3530_sdiodrv
{
	struct clk    *fclk, *iclk, *dbclk;
	int           dma_tx_channel;
	int           dma_rx_channel;
	void          (*BusTxnCB)(void* BusTxnHandle, int status);
	void*         BusTxnHandle;
	unsigned int  uBlkSize;
	unsigned int  uBlkSizeShift;
	char          *dma_buffer;
	void          *async_buffer;
	unsigned int  async_length;
	int           async_status;
	int (*wlanDrvIf_pm_resume)(void);
	int (*wlanDrvIf_pm_suspend)(void);
	struct device *dev;
	dma_addr_t dma_read_addr;
	size_t dma_read_size;
	dma_addr_t dma_write_addr;
	size_t dma_write_size;
} OMAP3530_sdiodrv_t;

module_param(g_sdio_debug_level, int, 0644);
MODULE_PARM_DESC(g_sdio_debug_level, "debug level");

int g_sdio_debug_level = SDIO_DEBUGLEVEL_INFO;
EXPORT_SYMBOL(g_sdio_debug_level);

OMAP3530_sdiodrv_t g_drv;
struct platform_device adhoc_mmc2;
static int sdiodrv_irq_requested = 0;
static int sdiodrv_mmc_power_ena = 0;
static int sdiodrv_dma_on = 0;
static int sdiodrv_iclk_enable = 0;
static int sdiodrv_fclk_enable = 0;
static int sdiodrv_suspend_enable = 0;
static int initCount = 0;

static struct sdio_func sdio_ctrl_func;
static struct sdio_func *tiwlan_func[1 + SDIO_TOTAL_FUNCS];

 

int sdioDrv_ConnectBus (void *       fCbFunc, 
                        void *       hCbArg, 
                        unsigned int uBlkSizeShift, 
                        unsigned int uSdioThreadPriority)
                       
{
    g_drv.BusTxnCB      = fCbFunc;
    g_drv.BusTxnHandle  = hCbArg;
    g_drv.uBlkSizeShift = uBlkSizeShift;
    g_drv.uBlkSize      = 1 << uBlkSizeShift;

    return 0;
}

int sdioDrv_DisconnectBus (void)
{
    return 0;
}

/*p.3609 cmd flow */
int sdioDrv_ExecuteCmd (unsigned int uCmd, 
                        unsigned int uArg, 
                        unsigned int uRespType, 
                        void *       pResponse, 
                        unsigned int uLen)
{
	struct mmc_command cmd;
	int err;

	PDEBUG("%s: cmd %d\n", __func__, uCmd);
	printk("%s: cmd %d\n", __func__, uCmd);

	BUG_ON(!tiwlan_func[SDIO_WLAN_FUNC]->card->host);

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = uCmd;
	cmd.arg = uArg;
	cmd.flags = uRespType;

	err = mmc_wait_for_cmd(tiwlan_func[SDIO_WLAN_FUNC]->card->host, &cmd, 3);
	if (err) {
		printk(KERN_ERR "%s: SDIO Command error, err = %d\n", __func__, err);
	    return -1;
	}

	if (uLen > 0 && uLen <= 4) {
		memcpy(pResponse, (char *)&cmd.resp[0], uLen);
		printk(KERN_DEBUG "%s: response = 0x%x\n", __func__, *(int *)pResponse);
	}

    return 0;
}

static int generic_read_bytes(unsigned int uFunc, unsigned int uHwAddr,
								unsigned char *pData, unsigned int uLen,
								unsigned int bIncAddr, unsigned int bMore)
{
	unsigned int i;
	int ret;

	PDEBUG("%s: uFunc %d uHwAddr %d pData %x uLen %d bIncAddr %d\n", __func__, uFunc, uHwAddr, (unsigned int)pData, uLen, bIncAddr);
//	printk("%s: uFunc %d uHwAddr %d pData %x uLen %d bIncAddr %d\n", __func__, uFunc, uHwAddr, (unsigned int)pData, uLen, bIncAddr);

	BUG_ON(uFunc != SDIO_CTRL_FUNC && uFunc != SDIO_WLAN_FUNC);
	
	for (i = 0; i < uLen; i++) {
		if (uFunc == 0)
			*pData = sdio_f0_readb(tiwlan_func[uFunc], uHwAddr, &ret);
		else
			*pData = sdio_readb(tiwlan_func[uFunc], uHwAddr, &ret);

		if (0 != ret) {
			printk(KERN_ERR "%s: function %d sdio error: %d\n", __func__, uFunc, ret);
            return -1;
        }

        pData++;
		if (bIncAddr)
			uHwAddr++;
    }

    return 0;
}

/* We know that tiwlan_sdio_probe not yet called - this means the
 * wl127x is either not on this board, or broken. */
static void tiwlan_warn_sdio_not_probed(const char *func)
{
	static int warned = 0;
	if (!warned) {
		warned = 1;
		printk("%s: wl127x either missing or broken(MMC enumeration missing)\n",__func__);
	}
}

static int generic_write_bytes(unsigned int uFunc, unsigned int uHwAddr,
								unsigned char *pData, unsigned int uLen,
								unsigned int bIncAddr, unsigned int bMore)
{
	unsigned int i;
	int ret = 0;

	PDEBUG("%s: uFunc %d uHwAddr %d pData %x uLen %d\n", __func__, uFunc, uHwAddr, (unsigned int) pData, uLen);
//	printk("%s: uFunc %d uHwAddr %d pData %x uLen %d\n", __func__, uFunc, uHwAddr, (unsigned int) pData, uLen);
	if(!tiwlan_func[uFunc]){
		tiwlan_warn_sdio_not_probed(__func__);
		return -1;
	}

	BUG_ON(uFunc != SDIO_CTRL_FUNC && uFunc != SDIO_WLAN_FUNC);
	
	for (i = 0; i < uLen; i++) {
	  if (uFunc == 0)
	    sdio_f0_writeb(tiwlan_func[uFunc], *pData, uHwAddr, &ret);
	  else
	    sdio_writeb(tiwlan_func[uFunc], *pData, uHwAddr, &ret);
	  
	  if (0 != ret) {
	    printk(KERN_ERR "%s: function %d sdio error: %d\n", __func__, uFunc, ret);
            return -1;
	  }
	  
	  pData++;
	  if (bIncAddr)
	    uHwAddr++;
	}
	
	return 0;
}

/*--------------------------------------------------------------------------------------*/

int sdioDrv_ReadSync (unsigned int uFunc, 
                      unsigned int uHwAddr, 
                      void *       pData, 
                      unsigned int uLen,
                      unsigned int bIncAddr,
                      unsigned int bMore)
{
	int ret = 0;

	PDEBUG("%s: uFunc %d uHwAddr %d pData %x uLen %d bIncAddr %d\n", __func__, uFunc, uHwAddr, (unsigned int)pData, uLen, bIncAddr);
//	printk("%s: uFunc %d uHwAddr %d pData %x uLen %d bIncAddr %d\n", __func__, uFunc, uHwAddr, (unsigned int)pData, uLen, bIncAddr);

	/* If request is either for sdio function 0 or not a multiple of 4 (OMAP DMA limit)
	    then we have to use CMD 52's */

	if(!tiwlan_func[uFunc]){
		tiwlan_warn_sdio_not_probed(__func__);
		return -1;
	}

	sdio_claim_host(tiwlan_func[SDIO_WLAN_FUNC]);
	if (uFunc == SDIO_CTRL_FUNC || uLen % 4 != 0)
	  ret = generic_read_bytes(uFunc, uHwAddr, pData, uLen, bIncAddr, bMore);
	else
	  if (bIncAddr)
	    ret = sdio_memcpy_fromio(tiwlan_func[uFunc], pData, uHwAddr, uLen);
	  else
	    ret = sdio_readsb(tiwlan_func[uFunc], pData, uHwAddr, uLen);
	sdio_release_host(tiwlan_func[SDIO_WLAN_FUNC]);
	
	if (ret) {
	  printk(KERN_ERR "%s: sdio error: %d\n", __func__, ret);
	  return -1;
	}

	return 0;
}

/*--------------------------------------------------------------------------------------*/
int sdioDrv_ReadAsync (unsigned int uFunc, 
                       unsigned int uHwAddr, 
                       void *       pData, 
                       unsigned int uLen, 
                       unsigned int bBlkMode,
                       unsigned int bIncAddr,
                       unsigned int bMore)
{
#if 1
	PERR("%s not yet supported!\n", __func__);

	return -1;
#else
	int          iStatus;
	unsigned int uCmdArg;
   unsigned int uNumBlks;
   unsigned int uDmaBlockCount;
   unsigned int uNumOfElem;
	void         *dma_buffer;
	dma_addr_t   dma_bus_address;

	//printk(KERN_INFO "in sdioDrv_ReadAsync\n");
	
    if (bBlkMode)
    {
        /* For block mode use number of blocks instead of length in bytes */
        uNumBlks = uLen >> g_drv.uBlkSizeShift;
        uDmaBlockCount = uNumBlks;
        /* due to the DMA config to 32Bit per element (OMAP_DMA_DATA_TYPE_S32) the division is by 4 */ 
        uNumOfElem = g_drv.uBlkSize >> 2;
    }
    else
    {	
        uNumBlks = uLen;
        uDmaBlockCount = 1;
        uNumOfElem = (uLen + 3) >> 2;
    }

	if (((u32)pData & 3) == 0) /* 4 bytes aligned */
	{  
	  dma_buffer         = pData;
	}
	else                      /* 2 bytes aligned */
	{
	  dma_buffer         = g_drv.dma_buffer;
	  g_drv.async_buffer = pData;
	  g_drv.async_length = uLen;
	}

    uCmdArg = SDIO_CMD53_READ(0, uFunc, bBlkMode, bIncAddr, uHwAddr, uNumBlks);

    iStatus = sdiodrv_send_data_xfer_commad(OMAP_HSMMC_CMD53_READ_DMA, uCmdArg, uNumBlks, BRE, bBlkMode);

    if (!(iStatus & BRE)) 
    {
        PERR("sdioDrv_ReadAsync() buffer disabled! length = %d BLK = 0x%x PSTATE = 0x%x, BlkMode = %d\n", 
              uLen, OMAP_HSMMC_READ(BLK), iStatus, bBlkMode);
	goto err;
    }

	PDEBUG("sdiodrv_read_async() dma_ch=%d \n",g_drv.dma_rx_channel);
	printk("sdiodrv_read_async() dma_ch=%d \n",g_drv.dma_rx_channel);

	dma_bus_address = dma_map_single(g_drv.dev, dma_buffer, uLen, DMA_FROM_DEVICE);
	if (!dma_bus_address) {
		PERR("sdioDrv_ReadAsync: dma_map_single failed\n");
		goto err;
	}		

	if (g_drv.dma_read_addr != 0) {
//		constraint_remove(omap3430_sdio_cnstr);
		printk(KERN_ERR "sdioDrv_ReadAsync: previous DMA op is not finished!\n");
		BUG();
	}
	
	g_drv.dma_read_addr = dma_bus_address;
	g_drv.dma_read_size = uLen;

	omap_set_dma_dest_params(g_drv.dma_rx_channel,
									0,			// dest_port is only for OMAP1
									OMAP_DMA_AMODE_POST_INC,
									dma_bus_address,
									0, 0);

	omap_set_dma_transfer_params(g_drv.dma_rx_channel, OMAP_DMA_DATA_TYPE_S32, uNumOfElem , uDmaBlockCount , OMAP_DMA_SYNC_FRAME, OMAP24XX_DMA_MMC2_RX, OMAP_DMA_SRC_SYNC);

	omap_start_dma(g_drv.dma_rx_channel);

    /* Continued at sdiodrv_irq() after DMA transfer is finished */
	return 0;
err:
//	constraint_remove(omap3430_sdio_cnstr);
	return -1;
#endif
}


/*--------------------------------------------------------------------------------------*/

int sdioDrv_WriteSync (unsigned int uFunc, 
                       unsigned int uHwAddr, 
                       void *       pData, 
                       unsigned int uLen,
                       unsigned int bIncAddr,
                       unsigned int bMore)
{
	int ret;

	PDEBUG("%s: uFunc %d uHwAddr %d pData %x uLen %d bIncAddr %d\n", __func__, uFunc, uHwAddr, (unsigned int)pData, uLen, bIncAddr);
//	printk("%s: uFunc %d uHwAddr %d pData %x uLen %d bIncAddr %d\n", __func__, uFunc, uHwAddr, (unsigned int)pData, uLen, bIncAddr);

	/* If request is either for sdio function 0 or not a multiple of 4 (OMAP DMA limit)
	    then we have to use CMD 52's */

	if(!tiwlan_func[uFunc]){
		tiwlan_warn_sdio_not_probed(__func__);
		return -1;
	}
        sdio_claim_host(tiwlan_func[SDIO_WLAN_FUNC]);
	if (uFunc == SDIO_CTRL_FUNC || uLen % 4 != 0)
		ret = generic_write_bytes(uFunc, uHwAddr, pData, uLen, bIncAddr, bMore);
	else
	if (bIncAddr)
		ret = sdio_memcpy_toio(tiwlan_func[uFunc], uHwAddr, pData, uLen);
	else
		ret = sdio_writesb(tiwlan_func[uFunc], uHwAddr, pData, uLen);

     
        sdio_release_host(tiwlan_func[SDIO_WLAN_FUNC]);
	if (ret) {
		printk(KERN_ERR "%s: sdio error: %d\n", __func__, ret);
		return -1;
	}

	return 0;
}

/*--------------------------------------------------------------------------------------*/
int sdioDrv_WriteAsync (unsigned int uFunc, 
                        unsigned int uHwAddr, 
                        void *       pData, 
                        unsigned int uLen, 
                        unsigned int bBlkMode,
                        unsigned int bIncAddr,
                        unsigned int bMore)
{
#if 1
	PERR("%s not yet supported!\n", __func__);

	return -1;
#else
   int          iStatus;
   unsigned int uCmdArg;
   unsigned int uNumBlks;
   unsigned int uDmaBlockCount;
   unsigned int uNumOfElem;
	dma_addr_t   dma_bus_address;

//	printk(KERN_INFO "in sdioDrv_WriteAsync\n");
    if (bBlkMode)
    {

        /* For block mode use number of blocks instead of length in bytes */
        uNumBlks = uLen >> g_drv.uBlkSizeShift;
        uDmaBlockCount = uNumBlks;
        /* due to the DMA config to 32Bit per element (OMAP_DMA_DATA_TYPE_S32) the division is by 4 */ 
        uNumOfElem = g_drv.uBlkSize >> 2;
    }
    else
    {
	
        uNumBlks = uLen;
        uDmaBlockCount = 1;
        uNumOfElem = (uLen + 3) >> 2;
    }

    uCmdArg = SDIO_CMD53_WRITE(1, uFunc, bBlkMode, bIncAddr, uHwAddr, uNumBlks);

    iStatus = sdiodrv_send_data_xfer_commad(OMAP_HSMMC_CMD53_WRITE_DMA, uCmdArg, uNumBlks, BWE, bBlkMode);
    if (!(iStatus & BWE)) 
    {
        PERR("sdioDrv_WriteAsync() buffer disabled! length = %d, BLK = 0x%x, Status = 0x%x\n", 
             uLen, OMAP_HSMMC_READ(BLK), iStatus);
	goto err;
    }

	OMAP_HSMMC_WRITE(ISE, TC);

	dma_bus_address = dma_map_single(g_drv.dev, pData, uLen, DMA_TO_DEVICE);
	if (!dma_bus_address) {
		PERR("sdioDrv_WriteAsync: dma_map_single failed\n");
		goto err;
	}

	if (g_drv.dma_write_addr != 0) {
//		constraint_remove(omap3430_sdio_cnstr);
		PERR("sdioDrv_WriteAsync: previous DMA op is not finished!\n");
		BUG();
	}
	
	g_drv.dma_write_addr = dma_bus_address;
	g_drv.dma_write_size = uLen;

	omap_set_dma_src_params(g_drv.dma_tx_channel,
									0,			// src_port is only for OMAP1
									OMAP_DMA_AMODE_POST_INC,
									dma_bus_address,
									0, 
                           0);

	omap_set_dma_transfer_params(g_drv.dma_tx_channel, OMAP_DMA_DATA_TYPE_S32, uNumOfElem, uDmaBlockCount, OMAP_DMA_SYNC_FRAME, OMAP24XX_DMA_MMC2_TX, OMAP_DMA_DST_SYNC);

	omap_start_dma(g_drv.dma_tx_channel);


    /* Continued at sdiodrv_irq() after DMA transfer is finished */
	return 0;
err:
//	constraint_remove(omap3430_sdio_cnstr);
	return -1;
#endif
}

/*--------------------------------------------------------------------------------------*/

int sdioDrv_ReadSyncBytes (unsigned int  uFunc, 
                           unsigned int  uHwAddr, 
                           unsigned char *pData, 
                           unsigned int  uLen, 
                           unsigned int  bMore)
{
	int err = 0;
	PDEBUG("%s: uFunc %d uHwAddr %d pData %x uLen %d\n", __func__, uFunc, uHwAddr, (unsigned int)pData, uLen);
//	printk("%s: uFunc %d uHwAddr %d pData %x uLen %d\n", __func__, uFunc, uHwAddr, (unsigned int)pData, uLen);

	if(!tiwlan_func[uFunc]){
		tiwlan_warn_sdio_not_probed(__func__);
		return -1;
	}
        sdio_claim_host(tiwlan_func[SDIO_WLAN_FUNC]);
	err = generic_read_bytes(uFunc, uHwAddr, pData, uLen, 1, bMore);
        sdio_release_host(tiwlan_func[SDIO_WLAN_FUNC]);

	return err;
}

/*--------------------------------------------------------------------------------------*/

int sdioDrv_WriteSyncBytes (unsigned int  uFunc, 
                            unsigned int  uHwAddr, 
                            unsigned char *pData, 
                            unsigned int  uLen, 
                            unsigned int  bMore)
{
	int err = 0;
	PDEBUG("%s: uFunc %d uHwAddr %d pData %x uLen %d\n", __func__, uFunc, uHwAddr, (unsigned int) pData, uLen);
//	printk("%s: uFunc %d uHwAddr %d pData %x uLen %d\n", __func__, uFunc, uHwAddr, (unsigned int) pData, uLen);

	if(!tiwlan_func[uFunc]){
		tiwlan_warn_sdio_not_probed(__func__);
		return -1;
	}

        sdio_claim_host(tiwlan_func[SDIO_WLAN_FUNC]);
        err = generic_write_bytes(uFunc, uHwAddr, pData, uLen, 1, bMore);
        sdio_release_host(tiwlan_func[SDIO_WLAN_FUNC]);

	return err;
}

/*------------------------------------------------------------------*/
void sdioDrv_PrintRegisters(void)
{
#if 0
	printk(" OMAP MMC controller registers dump:\n");
	printk(" OMAP MMC Register Dump:\n");
	printk("SCONF =  0x%08x \n", OMAP_HSMMC_READ(SYSCONFIG));
	printk("SSTAT =  0x%08x \n", OMAP_HSMMC_READ(SYSSTATUS));
	printk(" CSRE =  0x%08x \n", OMAP_HSMMC_READ(CSRE));
	printk("STEST =  0x%08x \n", OMAP_HSMMC_READ(SYSTEST));
	printk(" CON  =  0x%08x \n", OMAP_HSMMC_READ(CON));
	printk(" BLK  =  0x%08x \n", OMAP_HSMMC_READ(BLK));
	printk(" ARG  =  0x%08x \n", OMAP_HSMMC_READ(ARG));
	printk(" CMD  =  0x%08x \n", OMAP_HSMMC_READ(CMD));
	printk("RSP10 =  0x%08x \n", OMAP_HSMMC_READ(RSP10));
	printk("RSP32 =  0x%08x \n", OMAP_HSMMC_READ(RSP32));
	printk("RSP54 =  0x%08x \n", OMAP_HSMMC_READ(RSP54));
	printk("RSP76 =  0x%08x \n", OMAP_HSMMC_READ(RSP76));
	printk(" DATA =  0x%08x \n", OMAP_HSMMC_READ(DATA));
	printk("PSTATE=  0x%08x \n", OMAP_HSMMC_READ(PSTATE));
	printk(" HCTL =  0x%08x \n", OMAP_HSMMC_READ(HCTL));
	printk("SYSCTL=  0x%08x \n", OMAP_HSMMC_READ(SYSCTL));
	printk(" STAT =  0x%08x \n", OMAP_HSMMC_READ(STAT));
	printk(" IE   =  0x%08x \n", OMAP_HSMMC_READ(IE));
	printk(" ISE  =  0x%08x \n", OMAP_HSMMC_READ(ISE));
	printk(" AC12 =  0x%08x \n", OMAP_HSMMC_READ(AC12));
	printk(" CAPA =  0x%08x \n", OMAP_HSMMC_READ(CAPA));
	printk("CCAPA =  0x%08x \n", OMAP_HSMMC_READ(CUR_CAPA));
	printk(" REV  =  0x%08x \n", OMAP_HSMMC_READ(REV));

	printk("\n  SDIO control, muxing registers:");

	printk("\n  CONTROL_PADCONF_MMC2_CLK = ");
	printk("  0x%08x", CONTROL_PADCONF_MMC2_CLK);
	printk("\n  CONTROL_PADCONF_MMC2_CMD = ");
	printk("  0x%08x", CONTROL_PADCONF_MMC2_DAT0);
	printk("\n  CONTROL_PADCONF_MMC2_DAT2 = ");
	printk("  0x%08x", CONTROL_PADCONF_MMC2_DAT2);
	printk("\n  CONTROL_DEVCONF1 = ");
	printk("  0x%08x", OMAP2_CONTROL_DEVCONF1);

	printk("\n  GPIO1 bank registers:");
	printk("\n PRCM_GPIO1_SYSCONFIG  = 0x%08x", omap_readl(0x48310010));
	printk("\n GPIO1_IRQSTATUS1  = 0x%08x", omap_readl(0x48310018));
	printk("\n GPIO1_IRQSTATUS2  = 0x%08x", omap_readl(0x48310028));
	printk("\n GPIO1_IRQENABLE1  = 0x%08x", omap_readl(0x4831001c));
	printk("\n GPIO1_WAKEUPENABLE  = 0x%08x", omap_readl(0x48310020));
	printk("\n GPIO1_SETIRQENABLE1 = 0x%08x", omap_readl(0x48310064));
	printk("\n GPIO1_SETWKUENA = 0x%08x", omap_readl(0x48310084));
	printk("\n GPIO1_FALLINGDETECT = 0x%08x", omap_readl(0x4831004c));

	printk("\n");
#endif
}/* end of sdioDrv_PrintRegisters() */

static void tiwlan_sdio_irq(struct sdio_func *func)
{
    PDEBUG("%s:\n", __func__);
}

int sdioDrv_DisableFunction(unsigned int uFunc)
{
    int err=0;	

	if(!tiwlan_func[uFunc]){
		tiwlan_warn_sdio_not_probed(__func__);
		return -1;
	}
	    PDEBUG("%s: func %d\n", __func__, uFunc);
	 printk("%s: func %d\n", __func__, uFunc);

	/* currently only wlan sdio function is supported */
	BUG_ON(uFunc != SDIO_WLAN_FUNC);
	BUG_ON(tiwlan_func[uFunc] == NULL);

        sdio_claim_host(tiwlan_func[SDIO_WLAN_FUNC]);
        err = sdio_disable_func(tiwlan_func[SDIO_WLAN_FUNC]);
        sdio_release_host(tiwlan_func[SDIO_WLAN_FUNC]);

	return err;
}

int sdioDrv_EnableInterrupt(unsigned int uFunc)
{
    PDEBUG("%s: func %d\n", __func__, uFunc);
    printk("%s: func %d\n", __func__, uFunc);

	/* currently only wlan sdio function is supported */
	BUG_ON(uFunc != SDIO_WLAN_FUNC);
	BUG_ON(tiwlan_func[uFunc] == NULL);

	return sdio_claim_irq(tiwlan_func[uFunc], tiwlan_sdio_irq);
}

int sdioDrv_DisableInterrupt(unsigned int uFunc)
{
    PDEBUG("%s: func %d\n", __func__, uFunc);
    printk("%s: func %d\n", __func__, uFunc);

	/* currently only wlan sdio function is supported */
	BUG_ON(uFunc != SDIO_WLAN_FUNC);
	BUG_ON(tiwlan_func[uFunc] == NULL);

	return sdio_release_irq(tiwlan_func[uFunc]);
}

int tiwlan_sdio_probe_called;
EXPORT_SYMBOL(tiwlan_sdio_probe_called);

static int tiwlan_sdio_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
    int err=0;	
    printk("%s\n",__func__);

    PDEBUG("TIWLAN: probed with vendor 0x%x, device 0x%x, class 0x%x\n",
           func->vendor, func->device, func->class);
    printk("tiwlan_sdio_probe called \n");

    if (func->vendor != SDIO_VENDOR_ID_TI ||
		func->device != SDIO_DEVICE_ID_TI_WL12xx ||
		func->class != SDIO_CLASS_WLAN) {
	printk("%s: >>>>>>>>>>>>>>>>>----- NOT MATCHING: %i, %i, %i \n",__func__, func->vendor, func->device, func->class);
        return -ENODEV;
	}

    tiwlan_sdio_probe_called = 1;

    printk(KERN_INFO "TIWLAN: Found TI/WLAN SDIO controller (vendor 0x%x, device 0x%x, class 0x%x)\n",
           func->vendor, func->device, func->class);
    
    tiwlan_func[SDIO_WLAN_FUNC] = func;
    tiwlan_func[SDIO_CTRL_FUNC] = func;
    sdio_claim_host(tiwlan_func[SDIO_WLAN_FUNC]);
    err = sdio_enable_func(tiwlan_func[SDIO_WLAN_FUNC]); 
    err = sdio_set_block_size(tiwlan_func[SDIO_WLAN_FUNC], 512);
    sdio_release_host(tiwlan_func[SDIO_WLAN_FUNC]);

    return 0;
}

static void tiwlan_sdio_remove(struct sdio_func *func)
{
    PDEBUG("%s\n", __func__);
    printk("%s\n", __func__);

    /* sdio_release_host(tiwlan_func[SDIO_WLAN_FUNC]); */
}

# if 1
static const struct sdio_device_id tiwl12xx_devices[] = {
       {.class = SDIO_CLASS_WLAN,
	   	.vendor = SDIO_VENDOR_ID_TI,
	   	.device = SDIO_DEVICE_ID_TI_WL12xx},
       {}
};
#else
static const struct sdio_device_id tiwl12xx_devices[] = {
       { SDIO_DEVICE(SDIO_VENDOR_ID_TI, SDIO_DEVICE_ID_TI_WL12xx) },
       {}
};
#endif
MODULE_DEVICE_TABLE(sdio, tiwl12xx_devices);

static struct sdio_driver tiwlan_sdio_drv = {
    .probe          = tiwlan_sdio_probe,
    .remove         = tiwlan_sdio_remove,
    .name           = "sdio_tiwlan",
    .id_table       = tiwl12xx_devices,
};

int sdioDrv_init(void)
{
	int ret;

	printk("%s\n", __func__);
	PDEBUG("%s: Debug mode\n", __func__);

#define CM_FCLKEN1_CORE                 0x48004A00
#define CM_ICLKEN1_CORE                 0x48004A10

    
	printk("setting SDIO F&I clock Configuration\n");
	omap_writel(omap_readl(CM_ICLKEN1_CORE) /*| (1<<25) */| (1<<30), CM_ICLKEN1_CORE);
	omap_writel(omap_readl(CM_FCLKEN1_CORE) /*| (1<<25) */| (1<<30), CM_FCLKEN1_CORE);
	printk("done setting SDIO F&I clock Configuration\n");

	memset(&g_drv, 0, sizeof(g_drv));

	g_drv.dma_buffer = kzalloc(TIWLAN_MMC_MAX_DMA, GFP_KERNEL|GFP_DMA);
	if (g_drv.dma_buffer == NULL) {
	  ret = -ENOMEM;
	  goto out;
	}

#if 0
	g_drv.fclk = clk_get(&pdev->dev, "mmchs_fck");
	if (IS_ERR(g_drv.fclk)) {
		rc = PTR_ERR(g_drv.fclk);
		PERR("clk_get(fclk) FAILED !!!\n");
		goto err;
	}
	sdiodrv_fclk_got = 1;

	g_drv.iclk	= clk_get(&pdev->dev, "mmchs_ick");
	if (IS_ERR(g_drv.iclk)) {
		rc = PTR_ERR(g_drv.iclk);
		PERR("clk_get(iclk) FAILED !!!\n");
		goto err;
	}
	sdiodrv_iclk_got = 1;

	rc = clk_enable(g_drv.iclk);
	if (rc) {
		PERR("clk_enable(iclk) FAILED !!!\n");
		goto err;
	}
	sdiodrv_iclk_ena = 1;

	rc = clk_enable(g_drv.fclk);
	if (rc) {
		PERR("clk_enable(fclk) FAILED !!!\n");
		goto err;
	}
	sdiodrv_fclk_ena = 1;

	gpio_set_value(129, 0);  // lvlshift
	gpio_set_value(3, 0); // toggle it (if it was already high )
	mdelay(100);
	gpio_set_value(3, 1); // set it high to enable wlan
#endif

    ret = sdio_register_driver(&tiwlan_sdio_drv);
    if (ret < 0) {
        printk(KERN_ERR "sdioDrv_init: sdio register failed: %d\n", ret);
		kfree(g_drv.dma_buffer);
		goto out;
    }

	printk(KERN_INFO "TI WiLink 1271 SDIO: Driver loaded\n");

out:
	return ret;
}

void sdioDrv_exit(void)
{
	sdio_unregister_driver(&tiwlan_sdio_drv);

	kzfree(g_drv.dma_buffer);

	printk(KERN_INFO "TI WiLink 1271 SDIO Driver unloaded\n");
}

#if 0
module_init(sdioDrv_init);
module_exit(sdioDrv_exit);

#ifdef CONFIG_PM
EXPORT_SYMBOL(sdioDrv_register_pm);
#endif
EXPORT_SYMBOL(sdioDrv_ConnectBus);
EXPORT_SYMBOL(sdioDrv_DisconnectBus);
EXPORT_SYMBOL(sdioDrv_ExecuteCmd);
EXPORT_SYMBOL(sdioDrv_ReadSync);
EXPORT_SYMBOL(sdioDrv_WriteSync);
EXPORT_SYMBOL(sdioDrv_ReadAsync);
EXPORT_SYMBOL(sdioDrv_WriteAsync);
EXPORT_SYMBOL(sdioDrv_ReadSyncBytes);
EXPORT_SYMBOL(sdioDrv_WriteSyncBytes);
//EXPORT_SYMBOL(sdioDrv_PrintRegisters);
MODULE_LICENSE("GPL");
#endif

