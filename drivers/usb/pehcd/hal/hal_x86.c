/************************************************************
 * ISP176x Linux Host Controller driver : pehcd
 * Hardware Abstraction Layer code file
 *
 * (c) 2009 ST-ERICSSON, All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * File Name: hal_x86.c
 *
 * Refering linux kernel version 2.6.9/11/20/28
 *
 * History:
 *
 * Date                Author                  Comments
 * ---------------------------------------------------------------------
 * 28 Jan 2009      ST-ERICSSON     Initial Creation   
 *
 **********************************************************************
 */
#ifdef LINUX_269
#include <linux/config.h>
#endif
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>  
#include <linux/usb.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>  
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <asm/dma.h>

#define USE_PLATFORM_DEVICE

/*--------------------------------------------------------------*
 *               linux system include files
 *--------------------------------------------------------------*/
#include "../hal/isp1761.h"
#include "hal_x86.h"
#include "../hal/hal_intf.h"

#ifdef SWAP
#define PTD_HED 1
#define PTD_PAY 2
#define LSB0(x) ((x) & 0xff) 
#define LSB1(x) (((x) >> 8) & 0xff) 
#define LSB2(x) (((x) >> 16) & 0xff) 
#define LSB3(x) (((x) >> 24) & 0xff) 
#define MSB16(x) (((x) >> 8 ) & 0xff)
#define LSB16(x) ((x) & 0xff)
#define SWAP32(x) ((LSB0(x) << 24) | (LSB1(x) << 16) | (LSB2(x) << 8) | (LSB3(x)))
#define SWAP16(x) (MSB16(x) | (LSB16(x) << 8))
static inline void writeb_b(unsigned char b, volatile void __iomem *addr)
{
	*(u8 *) addr = b;
}
static inline void writew_w(unsigned short b, volatile void __iomem *addr)
{
	*(u16 *) addr = b;
}
static inline void writel_l(unsigned int b, volatile void __iomem *addr)
{
	*(u32 *) addr = (b);
}
static inline void writew_swap(unsigned short b, volatile void __iomem *addr)
{
	*(u16 *) addr = SWAP16(b);
}
static inline void writel_swap(unsigned int b, volatile void __iomem *addr)
{
	*(u32 *) addr = SWAP32(b);
}
static inline unsigned char readb_b(const volatile void __iomem *addr)
{
	return (*(u8 *) addr);
}
static inline unsigned short readw_w(const volatile void __iomem *addr)
{
	return *(u16 *) addr;
}
static inline unsigned int readl_l(const volatile void __iomem *addr)
{
	return (*(u32 *) addr);
}
static inline unsigned int readl_swap(const volatile void __iomem *addr)
{
	return SWAP32(*(u32 *) addr);
}
#else
#endif
/*--------------------------------------------------------------*
 *               Local variable Definitions
 *--------------------------------------------------------------*/
struct isp1761_dev              isp1761_loc_dev[ISP1761_LAST_DEV];
static  struct isp1761_hal      hal_data;
#ifdef NON_PCI
#else
static  __u32             pci_io_base  = 0;
static   __u32           pci_mem_phy0 =0;
static    __u32           pci_mem_len = 0xffff;
static  int               isp1761_pci_latency;
#endif
void     *iobase = 0;
int      iolength = 0;

/*--------------------------------------------------------------*
 *               Local # Definitions
 *--------------------------------------------------------------*/
#define         PCI_ACCESS_RETRY_COUNT  20
#define         PLX_INT_CSR_REG         0x68
#define         PLX_LBRD0_REG           0x18
#define         PLX_LBRD0_WAIT_STATE_MASK       0x000000C3
#define         PLX_LBRD0_WAIT_STATES           0x00000003
#define         isp1761_driver_name     "1761-pci"

/*--------------------------------------------------------------*
 *               Local Function 
 *--------------------------------------------------------------*/
#ifdef NON_PCI
#ifdef USE_PLATFORM_DEVICE
static int __devinit isp1761_probe (struct platform_device *pdev);
static int __devexit isp1761_remove (struct platform_device *pdev);
static int isp1761_suspend (struct platform_device *pdev, pm_message_t level);
static int isp1761_resume (struct platform_device *pdev);
#else
static void __devexit isp1761_remove (struct device *dev);
static int __devinit isp1761_probe (struct device *dev);
static int isp1761_suspend (struct device *dev, u32 state, u32 level);
static int isp1761_resume (struct device *dev, u32 level);
#endif
static irqreturn_t  isp1761_non_pci_isr (int irq, void *dev_id
#if defined(LINUX_269) || defined(LINUX_2611)
					 , struct pt_regs *regs
#endif
					 );
#else /*PCI*/
static void __devexit isp1761_pci_remove (struct pci_dev *dev);
static int __devinit isp1761_pci_probe (struct pci_dev *dev, 
        const struct pci_device_id *id);
int isp1761_pci_suspend (struct pci_dev *dev, __u32 state);
int isp1761_pci_resume (struct pci_dev *dev);
static irqreturn_t  isp1761_pci_isr (int irq, void *dev_id, struct pt_regs *regs);
#endif

/*--------------------------------------------------------------*
 *              ISP 1761 interrupt locking functions
 *--------------------------------------------------------------*/
int             isp1761_hw_lock = 0;
int             isp1761_hw_isr = 0;

void isp1761_disable_interrupt(void) {
    
    struct isp1761_dev  *dev;
    u32 hw_mode_buff = 0;
    
    /* Process the Host Controller Driver */
    dev = &isp1761_loc_dev[ISP1761_HC];
    
    hw_mode_buff = isp1761_reg_read32(dev, HC_HW_MODE_REG, hw_mode_buff);

    /* Disable ISP176x global interrupt */
    hw_mode_buff &= ~(0x1);
        
    isp1761_reg_write32(dev, HC_HW_MODE_REG, hw_mode_buff);
}

void isp1761_enable_interrupt(void) {
    struct isp1761_dev  *dev;
    u32 hw_mode_buff = 0;
    
    /* Process the Host Controller Driver */
    dev = &isp1761_loc_dev[ISP1761_HC];
    
    hw_mode_buff = isp1761_reg_read32(dev, HC_HW_MODE_REG, hw_mode_buff);

    /* Enable ISP176x global interrupt */
    hw_mode_buff |= 0x1;
    
    isp1761_reg_write32(dev, HC_HW_MODE_REG, hw_mode_buff);
}




/*--------------------------------------------------------------*
 *               ISP1761 Interrupt Service Routine
 *--------------------------------------------------------------*/
/*Interrupt Service Routine for device controller*/
irqreturn_t isp1761_pci_dc_isr(int irq, void *data, struct pt_regs *r)
{
    struct isp1761_dev *dev;
    dev = &isp1761_loc_dev[ISP1761_DC];

    hal_entry("%s: Entered\n",__FUNCTION__);
    /*not ready yet*/
    if(dev->active == 0)
    {
        printk("isp1761_pci_dc_isr: dev->active is NULL \n");
        return IRQ_NONE;
    }

    /* Get the source of interrupts for Device Controller
     * Device Controller interrupts are cleared by the driver
     * during processing
     */

    /*unblock the device interrupt*/

    isp1761_reg_write32(dev, DEV_UNLOCK_REGISTER, 0xaa37);
    dev->int_reg = isp1761_reg_read32(dev, DEV_INTERRUPT_REGISTER, dev->int_reg);
    hal_int("isp1761_pci_dc_isr:INTERRUPT_REGISTER 0x%x\n",(unsigned int)dev->int_reg);
    /*clear the interrupt source*/
    isp1761_reg_write32(dev, 0x218, dev->int_reg);
    dev->int_reg &= 0x03fffdb9;
#if ((defined LINUX_269) || defined (LINUX_2611))    
    if(dev->int_reg)
        dev->handler(dev, dev->isr_data,r);
#else
	if(dev->int_reg)
        dev->handler(dev, dev->isr_data);
#endif
    hal_entry("%s: Exit\n",__FUNCTION__);
    return IRQ_HANDLED;
}

/* Interrupt Service Routine of isp1761                                   
 * Reads the source of interrupt and calls the corresponding driver's ISR.
 * Before calling the driver's ISR clears the source of interrupt.
 * The drivers can get the source of interrupt from the dev->int_reg field
 */
#ifdef NON_PCI
irqreturn_t     isp1761_non_pci_isr(int irq, void *__data
#if defined(LINUX_269) || defined(LINUX_2611)
				    , struct pt_regs *r
#endif
				    )
#else /*PCI*/
irqreturn_t     isp1761_pci_isr(int irq, void *__data, struct pt_regs *r)
#endif

{
    __u32               irq_mask = 0;
    struct isp1761_dev  *dev;
    hal_entry("%s: Entered\n",__FUNCTION__);
    isp1761_disable_interrupt();
    /* Process the Host Controller Driver */
    dev = &isp1761_loc_dev[ISP1761_HC];
    /* Get the source of interrupts for Host Controller*/
    dev->int_reg = isp1761_reg_read32(dev, HC_INTERRUPT_REG,dev->int_reg);
    isp1761_reg_write32(dev,HC_INTERRUPT_REG,dev->int_reg);
    irq_mask = isp1761_reg_read32(dev, HC_INTENABLE_REG,irq_mask);

    dev->int_reg &= irq_mask; /*shared irq ??*/
#if ((defined LINUX_269) || defined (LINUX_2611))
    /*call the Host Isr if any valid(minus otg)interrupt is present*/
    if(dev->int_reg & ~HC_OTG_INTERRUPT)                
        dev->handler(dev,dev->isr_data,r);
#else
	/*call the Host Isr if any valid(minus otg)interrupt is present*/
    if(dev->int_reg & ~HC_OTG_INTERRUPT)                
        dev->handler(dev,dev->isr_data);
#endif

#ifdef OTG
#ifndef MSEC_INT_BASED
    mdelay(1);
#endif
    /*process otg interrupt if there is any*/
    if(dev->int_reg & HC_OTG_INTERRUPT){
        u32     otg_int;        
#ifndef MSEC_INT_BASED
        mdelay(1);
#endif
        otg_int = (dev->int_reg & HC_OTG_INTERRUPT);
        /* Process OTG controller Driver 
         * Since OTG is part of  HC interrupt register, 
         * the interrupt source will be HC interrupt Register
         * */
        dev = &isp1761_loc_dev[ISP1761_OTG];
        /* Read the source of  OTG_INT and clear the
           interrupt source */
        dev->int_reg = otg_int; 
#if ((defined LINUX_269) || defined (LINUX_2611))		
        dev->handler(dev, dev->isr_data,r);
#else
		dev->handler(dev, dev->isr_data);
#endif
    }   
#endif

#if ((defined LINUX_269) || defined (LINUX_2611))		
    dev->handler(dev, dev->isr_data,r);
#else
	dev->handler(dev, dev->isr_data);
#endif
    isp1761_enable_interrupt();
    hal_entry("%s: Exit\n",__FUNCTION__);
    return IRQ_HANDLED;
} /* End of isp1362_pci_isr */

#ifdef NON_PCI
/*--------------------------------------------------------------*
 *               NON_PCI Driver Interface Functions
 *--------------------------------------------------------------*/

#define ISP1761_ID -1

/* Important fields to initialize for Non-PCI based driver*/

/* The base physical memory address assigned for the ISP176x */
#define ISP176x_MEM_BASE 0x1C100000

/* The memory range assigned for the ISP176x */
#define ISP176x_MEM_RANGE 0x10000

/* The IRQ number assigned to the ISP176x */
#define ISP176x_IRQ_NUM 35

static struct resource isp1761_resources[] = {
    [0] = {
        .start= ISP176x_MEM_BASE,
        .end= (ISP176x_MEM_BASE | ISP176x_MEM_RANGE),
        .flags= IORESOURCE_MEM,
    },
    [1] = {
        .start= ISP176x_IRQ_NUM,  
        .end= ISP176x_IRQ_NUM,
        .flags= IORESOURCE_IRQ,
    },
};

static void isp1761_device_release(struct device *dev)
{
    /* Keep this function empty. */
}

static struct platform_device isp1761_device = {
    .name = "isp176x_hal",
    .id = 0,  
    .dev		= {
        .release = isp1761_device_release,
    },

    .num_resources = ARRAY_SIZE(isp1761_resources),
    .resource      = isp1761_resources,
};

#ifdef USE_PLATFORM_DEVICE
static struct platform_driver isp1761_driver = {
	.driver = {
		.name		= "isp1760",
		.owner		= THIS_MODULE,
	},
#if 0
	.bus		= &platform_bus_type,
#endif
	.probe		= isp1761_probe,
	.remove		= isp1761_remove,
	.suspend	= isp1761_suspend,
	.resume		= isp1761_resume,
};
#else
static struct device_driver isp1761_driver = {
    .name		= "isp1760",  
    .bus		= &platform_bus_type,
    .probe		= isp1761_probe,
    .remove		= isp1761_remove,
    .suspend	        = isp1761_suspend,
    .resume		= isp1761_resume,
};
#endif
#else /*PCI*/


/*--------------------------------------------------------------*
 *               PCI Driver Interface Functions
 *--------------------------------------------------------------*/

static const struct pci_device_id __devinitdata isp1761_pci_ids [] = {
    {
        /* handle PCI BRIDE  manufactured by PLX */
class:          ((PCI_CLASS_BRIDGE_OTHER << 8) | (0x06 << 16)),
class_mask:     ~0,
                /* no matter who makes it */
vendor:         /*0x10B5,*/PCI_ANY_ID,
device:         /*0x5406,*/PCI_ANY_ID,
subvendor:      PCI_ANY_ID,
subdevice:      PCI_ANY_ID
    }, 
    { /* end: all zeroes */ }
};

MODULE_DEVICE_TABLE (pci, isp1761_pci_ids);

/* Pci driver interface functions */
static struct pci_driver isp1761_pci_driver = {
name:           "isp1761-hal",
id_table:       &isp1761_pci_ids[0],
probe:         isp1761_pci_probe,
remove:        isp1761_pci_remove,
};

#endif
/*--------------------------------------------------------------*
 *               ISP1761 Read write routine 
 *--------------------------------------------------------------*/

/* Write a 32 bit Register of isp1761 */
void isp1761_reg_write32(struct isp1761_dev *dev,__u16 reg,__u32 data)
{ 
#ifdef SWAP
    writel_swap(data,dev->baseaddress+reg);
#else /*NO_SWAP*/
    /* Write the 32bit to the register address given to us*/
	writel(data,dev->baseaddress+reg);
#endif
}


/* Read a 32 bit Register of isp1761 */
__u32 isp1761_reg_read32(struct isp1761_dev *dev,__u16 reg,__u32 data)
{ 
#ifdef SWAP
      data = readl_swap(dev->baseaddress+reg);
#else /*NO_SWAP*/
    data = readl(dev->baseaddress + reg);
#endif
    return data;
}


/* Read a 16 bit Register of isp1761 */
__u16 isp1761_reg_read16(struct isp1761_dev *dev,__u16 reg,__u16 data)
{ 
#ifdef SWAP
    data = readw_w(dev->baseaddress+reg);
#else /*NO_SWAP*/
    data = readw(dev->baseaddress+reg);
#endif
    return data;
}

/* Write a 16 bit Register of isp1761 */
void isp1761_reg_write16(struct isp1761_dev *dev,__u16 reg,__u16 data)
{ 
#ifdef SWAP
    writew_w(data,dev->baseaddress+reg);
#else /*NO_SWAP*/
    writew(data,dev->baseaddress+reg);
#endif
}

/*--------------------------------------------------------------*
 *  
 * Module dtatils: isp1761_mem_read
 *
 * Memory read using PIO method.
 *
 *  Input: struct isp1761_driver *drv  -->  Driver structure.
 *                      __u32 start_add     --> Starting address of memory 
 *              __u32 end_add     ---> End address 
 *              
 *              __u32 * buffer      --> Buffer pointer.
 *              __u32 length       ---> Length 
 *              __u16 dir          ---> Direction ( Inc or Dec)
 *                      
 *  Output     int Length  ----> Number of bytes read 
 *
 *  Called by: system function 
 * 
 * 
 *--------------------------------------------------------------*/
/* Memory read function PIO */
#ifdef SWAP
int     
isp1761_mem_read(struct isp1761_dev *dev, __u32 start_add, 
        __u32 end_add, __u32 * buffer, __u32 length, __u16 dir,u32 flag)
#else /*NO_SWAP*/
int     
isp1761_mem_read(struct isp1761_dev *dev, __u32 start_add, 
        __u32 end_add, __u32 * buffer, __u32 length, __u16 dir)
#endif       
{
    u8 *temp_base_mem = 0;      
    u8 *one = (u8 *) buffer;
    u16 *two = (u16 *) buffer;
    int a = (int)length;
    u32 w;

#ifdef LOGIC_OMAP3530_SOM_USB1760
    /* Read the first memory address multiple times 
       until its read value is stable. */
    if (length != 0)
    {
	u8 loop;
	u32 prev_w;

#ifdef NON_PCI
	temp_base_mem= (u8 *)(dev->baseaddress + start_add);
#else /*PCI*/
	temp_base_mem= (dev->baseaddress + start_add);
#endif
        /*initialize the Register 0x33C-used to manage Multiple threads */
#ifdef SWAP
	writel_swap(start_add,dev->baseaddress+0x33c);
#else /*NO_SWAP*/
	writel(start_add,dev->baseaddress+0x33c);
#endif
	udelay(1);  
	w = readl(temp_base_mem);

	for(loop=0; loop < 5; loop++) 
	{
#ifdef NON_PCI
		temp_base_mem= (u8 *)(dev->baseaddress + start_add);
#else /*PCI*/
		temp_base_mem= (dev->baseaddress + start_add);
#endif
		/*initialize the Register 0x33C-used to manage Multiple threads */
#ifdef SWAP
		writel_swap(start_add,dev->baseaddress+0x33c);
#else /*NO_SWAP*/
		writel(start_add,dev->baseaddress+0x33c);
#endif
		udelay(1); 

		prev_w = w;
		w = readl(temp_base_mem);
		if(prev_w == w)
			break;
	} 
    }
#endif

#ifdef NON_PCI
    temp_base_mem= (u8 *)(dev->baseaddress + start_add);
#else /*PCI*/
    temp_base_mem= (dev->baseaddress + start_add);
#endif
    /*initialize the Register 0x33C-used to manage Multiple threads */
#ifdef SWAP
   writel_swap(start_add,dev->baseaddress+0x33c);
#else /*NO_SWAP*/
    writel(start_add,dev->baseaddress+0x33c);
#endif

#ifdef LOGIC_OMAP3530_SOM_USB1760
    /* Time delay between memory bank configuration and the actual memory access. */ 
    udelay(1);  
#endif
	if(buffer == NULL)
	{
		printk("@@@@ In isp1761_mem_read The buffer is pointing to null\n");
		return 0;
	}
last:
#ifdef SWAP
	if(flag==PTD_HED) /* PTD header */
	{
	    w = readl_swap(temp_base_mem);
	}
	else if(flag ==PTD_PAY)
		{
	    w = readl_l(temp_base_mem);
		}
	else
	{
	    printk("illegal operation\n");
	}
    if(a == 1){
        *one=(u8)(w>>24);
        return 0;
    }
    if(a == 2){
        *two=(u16)(w>>16);
        return 0;
    }   
    if(a == 3){
        *two=(u16)(w>>8);
        two += 1;
        w >>= 24;
        *two = (u8)(w);
        return 0;
    }
#else /*NO_SWAP*/
    w = readl(temp_base_mem);
    if(a == 1){
        *one=(u8)w;
        return 0;
    }
    if(a == 2){
        *two=(u16)w;
        return 0;
    }   
    if(a == 3){
        *two=(u16)w;
        two += 1;
        w >>= 16;
        *two = (u8)(w);
        return 0;

    }
#endif
    while(a>0){
        *buffer = w;
        temp_base_mem = temp_base_mem+4;
        start_add +=4;
        a -= 4;
        if(a == 0)
            break;
        if(a < 4){
            buffer += 1;
            one = (u8 *)buffer;
            two = (u16 *)buffer;
            goto last;
        }
        buffer += 1;
#ifdef SWAP
	if(flag==PTD_HED)
	{
	    w = readl_swap(temp_base_mem);
	}
	else if(flag ==PTD_PAY)
		{
	    w = readl_l(temp_base_mem);
		}
	else
	{
	    printk("illegal operation\n");
	}
#else /*NO_SWAP*/
        w = readl(temp_base_mem); 
#endif

    }
    return ((a < 0) || (a == 0))?0:(-1);

}

/*--------------------------------------------------------------*
 *  
 * Module dtatils: isp1761_mem_write
 *
 * Memory write using PIO method.
 *
 *  Input: struct isp1761_driver *drv  -->  Driver structure.
 *                      __u32 start_add     --> Starting address of memory 
 *              __u32 end_add     ---> End address 
 *              
 *              __u32 * buffer      --> Buffer pointer.
 *              __u32 length       ---> Length 
 *              __u16 dir          ---> Direction ( Inc or Dec)
 *                      
 *  Output     int Length  ----> Number of bytes read 
 *
 *  Called by: system function 
 * 
 * 
 *--------------------------------------------------------------*/

/* Memory read function IO */
#ifdef SWAP

int     
isp1761_mem_write(struct isp1761_dev *dev, 
        __u32 start_add, __u32 end_add, 
        __u32 * buffer, __u32 length,
        __u16 dir, u32 flag)
#else /*NO_SWAP*/
int     
isp1761_mem_write(struct isp1761_dev *dev, 
        __u32 start_add, __u32 end_add, 
        __u32 * buffer, __u32 length,
        __u16 dir)
#endif
{
    u8 *temp_base_mem = 0;
    int a = length;
//    u8 *temp = (u8*)buffer;
    u8 one      =(u8 )(*buffer);
    u16 two     =(u16 )(*buffer);       
#ifdef NON_PCI
    temp_base_mem= (u8 *)(dev->baseaddress + start_add);
#else /*PCI*/
    temp_base_mem= (dev->baseaddress + start_add);
#endif

#ifdef SWAP
    if(a == 1){
        writeb_b(one,temp_base_mem);
        return 0;
    }
    if(a == 2){
        writew_swap(two,temp_base_mem);
        return 0;
    }
#else /*NO_SWAP*/
    if(a == 1){
        writeb(one,temp_base_mem);
        return 0;
    }
    if(a == 2){
        writew(two,temp_base_mem);
        return 0;
    }
#endif
    while(a>0){
#ifdef SWAP
	if(flag==PTD_HED)
	{
	    writel_swap(*buffer, temp_base_mem);
	}
	else if(flag ==PTD_PAY)
		{
	    writel_l(*buffer, temp_base_mem);
		}
	else
	{
	    printk("illegal operation\n");
	}
#else /*NO_SWAP*/
        writel(*buffer, temp_base_mem);
#endif
        temp_base_mem = temp_base_mem+4;
        start_add +=4;
        a -=4;
        if(a <= 0)
            break;
        buffer += 1;

    }
    return ((a < 0) || (a == 0))?0:(-1);

}
/*--------------------------------------------------------------*
 *  
 * Module dtatils: isp1761_check_mem_region
 *
 *  Check the memory region for Memory Mapping 
 *  Check with the system about the availability of the region,
 *  and returns success, if available.
 *
 *  Input: struct isp1761_driver *drv  --> Driver structure.
 *  
 *  Output result  
 *         
 *
 *  Called by: system function 
 * 
 * 
 *--------------------------------------------------------------*/

int isp1761_check_mem_region(struct isp1761_dev *dev)
{
    int ret;
    ret=check_mem_region(dev->start, dev->length);
    return ret;
}/* End of isp1761_check_mem_region */

/*--------------------------------------------------------------*
 *  
 * Module dtatils: isp1761_request_mem_region
 isp1761_release_mem_region
 isp1761_get_mem_params

 *
 *  If the check returns Success, we can request the region for 
 *  Memory mapping of our chip memory
 *
 *  Input: struct isp1761_driver *drv  --> Driver structure.
 *  
 *  Output result  
 *         
 *
 *  Called by: system function 
 * 
 * 
 *--------------------------------------------------------------*/

struct resource* isp1761_request_mem_region(struct isp1761_dev *dev)
{
    dev->mem_res = request_mem_region(dev->start, dev->length, "Isp1761_device");
    return dev->mem_res;
}/* End of isp1761_request_mem_region */

/* Release an already acquired memory region.
   It should be done at the rmmod of the module */
void isp1761_release_mem_region(struct isp1761_dev* dev)
{
    release_mem_region (dev->start, dev->length);
}

/* Get the start address and length of Mapped Memory */ 
void isp1761_get_mem_params(struct isp1761_dev *dev,struct isp1761_driver *drv)
{
    dev->start  =isp1761_loc_dev[drv->index].start;
    dev->length =isp1761_loc_dev[drv->index].length;
}/* End of isp1761_get_mem_params*/


/*--------------------------------------------------------------*
 *  
 * Module dtatils: isp1761_request_irq
 *
 * This function registers the ISR of driver with this driver.
 * Since there is only one interrupt line, when the first driver
 * is registerd, will call the system function request_irq. The PLX
 * bridge needs enabling of interrupt in the interrupt control register to 
 * pass the local interrupts to the PCI (cpu).
 * For later registrations will just update the variables. On ISR, this driver
 * will look for registered handlers and calls the corresponding driver's
 * ISR "handler" function with "isr_data" as parameter.
 *
 *  Input: struct 
 *              (void (*handler)(struct isp1761_dev *, void *)-->handler.
 *               isp1761_driver *drv  --> Driver structure.
 *  Output result 
 *         0= complete 
 *         1= error.
 *
 *  Called by: system function module_init 
 * 
 * 
 *--------------------------------------------------------------*/
#if ((defined LINUX_269) || defined (LINUX_2611))
int isp1761_request_irq(void (*handler)(struct isp1761_dev *, void *, struct pt_regs *),
        struct isp1761_dev *dev, void *isr_data) 
#else
int isp1761_request_irq(void (*handler)(struct isp1761_dev *, void *),
        struct isp1761_dev *dev, void *isr_data) 
#endif
{
    int result = 0;
    u32 intcsr = 0;
    hal_entry("%s: Entered\n",__FUNCTION__);
    hal_init("isp1761_request_irq: dev->index %x\n",dev->index);
    if(dev->index == ISP1761_DC){
#if ((defined LINUX_269) || (defined LINUX_2611) || (defined LINUX_2620))
        result = request_irq(dev->irq, isp1761_pci_dc_isr,
                SA_SHIRQ,
                dev->name,
                isr_data);
#else
		result = request_irq(dev->irq, isp1761_pci_dc_isr,
			IRQF_SHARED,
			dev->name,
			isr_data); 		
#endif

    }else {
#if ((defined LINUX_269) || (defined LINUX_2611) || (defined LINUX_2620))
#ifdef NON_PCI
        result= request_irq(dev->irq,isp1761_non_pci_isr,
#else /*PCI*/
        result= request_irq(dev->irq,isp1761_pci_isr,
#endif
                SA_SHIRQ,
                dev->name,
                isr_data);
#else
#ifdef NON_PCI
        result= request_irq(dev->irq,isp1761_non_pci_isr,
#else /*PCI*/
			result= request_irq(dev->irq,isp1761_pci_isr,
#endif
			IRQF_SHARED,
			dev->name,
			isr_data);		
#endif
    }
#ifdef NON_PCI
#else /*PCI*/
    /*CONFIGURE PCI/PLX interrupt*/
    intcsr = readl(iobase+0x68);
    intcsr |= 0x900;
    writel(intcsr,iobase+0x68);
#endif

    /*Interrupt handler routine*/
    dev->handler = handler;
    dev->isr_data = isr_data;
	if ((dev->handler) != NULL)
		hal_int("isp1761_request_irq: dev->handler %x\n",(unsigned int)dev->handler);
	if ((dev->isr_data) != NULL)
		hal_int("isp1761_request_irq: dev->isr_data %x\n",(unsigned int)dev->isr_data);
	
    hal_entry("%s: Exit\n",__FUNCTION__);
    return result;
} /* End of isp1761_request_irq */

/*--------------------------------------------------------------*
 *  
 * Module dtatils: isp1761_free_irq
 *
 * This function de-registers the ISR of driver with this driver.
 * Since there is only one interrupt line, when the last driver
 * is de-registerd, will call the system function free_irq. The PLX
 * bridge needs disabling of interrupt in the interrupt control register to 
 * block the local interrupts to the PCI (cpu).
 *
 *  Input: struct 
 *              (void (*handler)(struct isp1761_dev *, void *)-->handler.
 *               isp1761_driver *drv  --> Driver structure.
 *  Output result 
 *         0= complete 
 *         1= error.
 *
 *  Called by: system function module_init 
 * 
 * 
 *--------------------------------------------------------------*/

void isp1761_free_irq(struct isp1761_dev *dev, void *isr_data)
{
#ifdef NON_PCI
    free_irq(dev->irq,isr_data);
#else /*PCI*/
    __u32       intcsr;
	if ((dev != NULL) && (isr_data != NULL))
		hal_int(("isp1761_free_irq(dev=%p,isr_data=%p)\n",dev,isr_data));

    free_irq(dev->irq,isr_data);
    /*disable the plx/pci interrupt*/
    intcsr = readl(iobase+0x68);
    intcsr &= ~0x900;
    writel(intcsr,iobase+0x68);
#endif	
} /* isp1761_free_irq */



/* Allocate Fragmented kernel Memory */
void* isp_1761_kmalloc(size_t size,int flags)
{
    void* ret;
    ret =kmalloc(size,flags);
    return ret;
}

/* Free the memory allocated by kmalloc */
void isp_1761_kfree(const void* objp)
{
    kfree(objp);
}

/* Allocate Contiguous kernel Memory */
void* isp_1761_vmalloc(__u32 size, __u16 flags, pgprot_t prot)
{
    void* ret;
    ret =__vmalloc(size, flags, prot);
    return ret;
}

/* Free the memory allocated by vmalloc */
void isp_1761_vfree(const void* objp)
{
    kfree(objp);
}



/*--------------------------------------------------------------*
 *  
 * Module dtatils: isp1761_register_driver
 *
 * This function is used by top driver (OTG, HCD, DCD) to register
 * their communication functions (probe, remove, suspend, resume) using
 * the drv data structure.
 * This function will call the probe function of the driver if the ISP1761
 * corresponding to the driver is enabled
 *
 *  Input: struct isp1761_driver *drv  --> Driver structure.
 *  Output result 
 *         0= complete 
 *         1= error.
 *
 *  Called by: system function module_init 
 * 
 * 
 *--------------------------------------------------------------*/

int     isp1761_register_driver(struct isp1761_driver *drv) 
{
    struct isp1761_dev  *dev;
    int result;
    isp1761_id *id;

    hal_entry("%s: Entered\n",__FUNCTION__);

    if(!drv) return -EINVAL;
    pr_info("%s(drv=%p) drv->index %lu\n", __FUNCTION__, drv, drv->index);

    dev = &isp1761_loc_dev[drv->index];
    pr_info("%s: dev %p\n", __FUNCTION__, dev);
    if(drv->index == ISP1761_DC){/*FIX for device*/
	    hal_entry("%s:%d drv->probe %p\n",__FUNCTION__, __LINE__, drv->probe);
	    result = drv->probe(dev,drv->id);
	    hal_entry("%s:%d result %d\n",__FUNCTION__, __LINE__, result);
    }else{              
        id = drv->id;
	pr_info("id: %p drv->probe %p active %d\n", id, drv->probe, dev->active);
        if(dev->active)
		result = drv->probe(dev,id);
        else
		result = -ENODEV;
    }

    if(result >= 0 ) {
        printk(KERN_INFO __FILE__ ": Registered Driver %s\n",
                drv->name);
        dev->driver = drv;
    }
    hal_entry("%s: Exit\n",__FUNCTION__);
    return result;
} /* End of isp1761_register_driver */


/*--------------------------------------------------------------*
 *  
 * Module dtatils: isp1761_unregister_driver
 *
 * This function is used by top driver (OTG, HCD, DCD) to de-register
 * their communication functions (probe, remove, suspend, resume) using
 * the drv data structure.
 * This function will check whether the driver is registered or not and
 * call the remove function of the driver if registered
 *
 *  Input: struct isp1761_driver *drv  --> Driver structure.
 *  Output result 
 *         0= complete 
 *         1= error.
 *
 *  Called by: system function module_init 
 * 
 * 
 *--------------------------------------------------------------*/


void    isp1761_unregister_driver(struct isp1761_driver *drv)
{
    struct isp1761_dev  *dev;
    hal_entry("%s: Entered\n",__FUNCTION__);

    pr_info("isp1761_unregister_driver(drv=%p)\n",drv);
    dev = &isp1761_loc_dev[drv->index];
    if(dev->driver == drv) {
        /* driver registered is same as the requestig driver */
        drv->remove(dev);
        dev->driver = NULL;
        pr_info(": De-registered Driver %s\n",
                drv->name);
        return;
    }
    hal_entry("%s: Exit\n",__FUNCTION__);
} /* End of isp1761_unregister_driver */


/*--------------------------------------------------------------*
 *               ISP1761 PCI driver interface routine.
 *--------------------------------------------------------------*/


/*--------------------------------------------------------------*
 *
 *  Module dtatils: isp1761_pci_module_init
 *
 *  This  is the module initialization function. It registers to 
 *  PCI driver for a PLX PCI bridge device. And also resets the
 *  internal data structures before registering to PCI driver.
 *
 *  Input: void 
 *  Output result 
 *         0= complete 
 *         1= error.
 *
 *  Called by: system function module_init 
 * 
 * 
 * 
 -------------------------------------------------------------------*/
#ifdef NON_PCI
static int __init isp1761_module_init (void) 
#else /*PCI*/
static int __init isp1761_pci_module_init (void) 
#endif
{
    int result = 0;
    hal_entry("%s: Entered\n",__FUNCTION__);
    memset(isp1761_loc_dev,0,sizeof(isp1761_loc_dev));

#ifdef NON_PCI
    if((result = platform_driver_register(&isp1761_driver)) < 0) {
	    pr_err("%s: driver_register failed %d\n", __FUNCTION__, result);
	    return result;
    }
#else /*PCI*/
#if ((defined LINUX_269) || (defined LINUX_2611) || (defined LINUX_2620))
    printk("%s:%d\n", __FUNCTION__, __LINE__);
    if((result = pci_module_init(&isp1761_pci_driver)) < 0) {
        printk("PCI Iinitialization Fail(error = %d)\n",result);
        return result;
    }
    else
        pr_info(": %s PCI Initialization Success \n",isp1761_driver_name);
#else
	if((result = pci_register_driver(&isp1761_pci_driver)) < 0) {
		printk("PCI Iinitialization Fail(error = %d)\n",result);
		return result;
	}
	else
		pr_info(": %s PCI Initialization Success \n",isp1761_driver_name);	
#endif
#endif

    hal_entry("%s: Exit\n",__FUNCTION__);
    return result;
}

/*--------------------------------------------------------------*
 *
 *  Module dtatils: isp1761_pci_module_cleanup
 *
 * This  is the module cleanup function. It de-registers from 
 * PCI driver and resets the internal data structures.
 *
 *  Input: void 
 *  Output void
 *
 *  Called by: system function module_cleanup 
 * 
 * 
 * 
 --------------------------------------------------------------*/

#ifdef NON_PCI
static void __exit isp1761_module_cleanup (void) 
{
    printk("Hal Module Cleanup\n");
#ifdef USE_PLATFORM_DEVICE
    platform_driver_unregister(&isp1761_driver);
#else
    driver_unregister(&isp1761_driver);
#endif
    memset(isp1761_loc_dev,0,sizeof(isp1761_loc_dev));
} 
#else /*PCI*/
static void __exit isp1761_pci_module_cleanup (void) 
{
    printk("Hal Module Cleanup\n");
    pci_unregister_driver (&isp1761_pci_driver);
    memset(isp1761_loc_dev,0,sizeof(isp1761_loc_dev));
} 
#endif

/*--------------------------------------------------------------*
 *
 *  Module dtatils: isp1761_pci_probe
 *
 * PCI probe function of ISP1761
 * This function is called from PCI Driver as an initialization function
 * when it founds the PCI device. This functions initializes the information
 * for the 3 Controllers with the assigned resources and tests the register
 * access to these controllers and do a software reset and makes them ready
 * for the drivers to play with them.
 *
 *  Input: 
 *              struct pci_dev *dev                     ----> PCI Devie data structure 
 *      const struct pci_device_id *id  ----> PCI Device ID 
 *  Output void
 *
 *  Called by: system function module_cleanup 
 * 
 * 
 * 
 --------------------------------------------------------------**/
#ifdef NON_PCI

static int __devinit isp1761_probe (
#ifdef USE_PLATFORM_DEVICE
	struct platform_device *pdev
#else
	struct device *dev
#endif
)
{
#ifdef USE_PLATFORM_DEVICE
	struct device *dev = &pdev->dev;
#else
    struct platform_device *pdev = to_platform_device(dev);
#endif
    struct isp1761_dev  *loc_dev;
    void *address = 0;
    int status = 1;  
    unsigned long word32;
    unsigned long base_addr, temp, i;

    hal_entry("%s: Entered\n",__FUNCTION__);

    /* Get the Host Controller IO and INT resources
     */
    loc_dev = &(isp1761_loc_dev[ISP1761_HC]);

	if (loc_dev != NULL)
		hal_init("loc_dev: 0x%p\n", loc_dev);
	else
		hal_init("loc_dev is NULL");
	
	if (pdev != NULL)
	{
		hal_init("pdev->resource[0].start 0x%x\n", pdev->resource[0].start);
		hal_init("pdev->resource[0].end 0x%x\n", pdev->resource[0].end);
		hal_init("Range: 0x%x\n", (pdev->resource[0].end - pdev->resource[0].start -1));	
	}
	else
		hal_init("pdev is NULL");

    loc_dev->irq = platform_get_irq(pdev, 0);
	if (loc_dev != NULL)
    hal_init("loc_dev->irq: 0x%x\n", loc_dev->irq);
	else
		hal_init("loc_dev is NULL");

    loc_dev->io_base = pdev->resource[0].start;
    loc_dev->start   =  pdev->resource[0].start;
    loc_dev->length  = (pdev->resource[0].end - pdev->resource[0].start -1);
    loc_dev->io_len = (pdev->resource[0].end - pdev->resource[0].start -1); /*64K*/
    loc_dev->index = ISP1761_HC;/*zero*/

    loc_dev->io_len = ISP176x_MEM_RANGE; 
    if(check_mem_region(loc_dev->io_base, loc_dev->length) < 0) {
        hal_init("host controller already in use\n");
        return -EBUSY;
    }
    if(!request_mem_region(loc_dev->io_base, loc_dev->length, isp1761_driver_name)){
        hal_init("host controller already in use\n");
        return -EBUSY;
    }

    /*map available memory*/
    address = ioremap_nocache(loc_dev->io_base, loc_dev->length);
    if(address == NULL){
        err("memory map problem\n");
        release_mem_region(loc_dev->io_base, loc_dev->length);
        return -ENOMEM;
    } 
	if (loc_dev != NULL)
    hal_init("Base: 0x%lx with Range: 0x%lx remapped to 0x%p\n", loc_dev->io_base, loc_dev->length, address);
	else	
		hal_init("loc_dev is NULL");

    loc_dev->baseaddress = (u8*)address;
    loc_dev->dmabase = (u8*) 0; 

	if (loc_dev != NULL)
    hal_init("isp1761 HC MEM Base= %p irq = %d\n", 
                loc_dev->baseaddress,loc_dev->irq);
	else
		hal_init("loc_dev is NULL");

    /* Try to check whether we can access Scratch Register of
     * Host Controller or not. The initial PCI access is retried until 
     * local init for the PCI bridge is completed 
     */
    loc_dev = &(isp1761_loc_dev[ISP1761_HC]);



    hal_init("Initiating Scratch register test\n");

#ifdef LOGIC_OMAP3530_SOM_USB1760
    /* Set 16-bit bus width to access ISP1760 registers. */
    base_addr = (unsigned long)(address + 0x300);

#ifdef SWAP                
    temp = readl_swap(base_addr);
#else
    temp = readl(base_addr);
#endif

    temp = temp & ~0x100;  

#ifdef SWAP        
    writel_swap(temp, base_addr);  
#else
    writel(temp, base_addr);
#endif
#endif
	
    for(i = 0, word32 = 1; i < 32; i ++)
    {    
        word32 = (1 << i);      
        base_addr = (unsigned long)(address + 0x308);
#ifdef SWAP        
        writel_swap(word32, base_addr);  
#else
        writel(word32, base_addr);
#endif

        base_addr = (unsigned long)(address + 0x304);
#ifdef SWAP                
        temp = readl_swap(base_addr);
#else
        temp = readl(base_addr);
#endif

        if(temp != 0x00011761)
        {
	  hal_init("Chip ID mismatch after writing 0x%08lx (0x%lx)\n", word32, temp);
            goto clean;
        }

        base_addr = (unsigned long)(address + 0x308);
#ifdef SWAP                
        temp = readl_swap(base_addr);  
#else
        temp = readl(base_addr);
#endif
        if(temp != word32)
        {
            hal_init("ERROR ====> Writing 0x%08lx to Scrath Reg failed!\n", word32);
            goto clean;
        }
    }


	
    hal_init("Scratch test completed...have a nice day!\n");

    memcpy(loc_dev->name, isp1761_driver_name, sizeof(isp1761_driver_name));
    loc_dev->name[sizeof(isp1761_driver_name)] = 0;
    loc_dev->active = 1;

    loc_dev->dev = pdev; 

    hal_data.irq_usage = 0;
    dev_set_drvdata(dev, loc_dev);
    hal_init("Exiting HAL initialization....SUCCESS!!!\n");

    hal_entry("%s: Exit\n",__FUNCTION__);
    return 0;

clean:
    release_mem_region(loc_dev->io_base, loc_dev->io_len);
    iounmap(loc_dev->baseaddress);
    hal_entry("%s: Exit\n",__FUNCTION__);
    return status;
} /* End of isp1761_probe */
#else /*PCI*/
    static int __devinit
isp1761_pci_probe (struct pci_dev *dev, const struct pci_device_id *id)
{
    u8 latency, limit;
    __u32       reg_data = 0;
    int retry_count;
    struct isp1761_dev  *loc_dev;
    void *address = 0;
    int length = 0;
    int status = 1;
    hal_entry("%s: Entered\n",__FUNCTION__);

	if (dev != NULL)
		hal_init("isp1761_pci_probe(dev=%p)\n",(void*)dev);
	else
		hal_init("dev is NULL\n");

    if (pci_enable_device(dev) < 0){
        err("failed in enabing the device\n");
        return -ENODEV;
    }
    if (!dev->irq) {
        err("found ISP1761 device with no IRQ assigned."); 
        err("check BIOS settings!");
        return -ENODEV;
    }
    /* Grab the PLX PCI mem maped port start address we need  */
    pci_io_base = pci_resource_start(dev, 0);
    hal_init("isp1761 pci IO Base= %x\n", (unsigned int)pci_io_base);

    iolength = pci_resource_len(dev, 0);
    hal_init("isp1761 pci io length %d\n", iolength);
    if(!request_mem_region(pci_io_base,iolength,"ISP1761 IO MEM")){
        err("host controller already in use1\n");
        return -EBUSY;
    }   
    iobase = ioremap_nocache(pci_io_base, iolength);
    if(!iobase){
        err("can not map io memory to system memory\n");
        release_mem_region(pci_io_base,iolength);
        return -ENOMEM;
    }
    /* Grab the PLX PCI shared memory of the ISP 1761 we need  */
    pci_mem_phy0 = pci_resource_start (dev, 3);         
    hal_init("isp1761 pci base address = %x\n", pci_mem_phy0);

    /* Get the Host Controller IO and INT resources
     */
    loc_dev = &(isp1761_loc_dev[ISP1761_HC]);
    loc_dev->irq = dev->irq;
    loc_dev->io_base = pci_mem_phy0;
    loc_dev->start   =  pci_mem_phy0;
    loc_dev->length  = pci_mem_len;
    loc_dev->io_len = pci_mem_len; /*64K*/
    loc_dev->index = ISP1761_HC;/*zero*/

    length = pci_resource_len(dev,3);
    if(length < pci_mem_len){
        err("memory length for this resource is less than required\n");
        release_mem_region(pci_io_base, iolength);
        iounmap(iobase);
        return  -ENOMEM;                                

    }
    loc_dev->io_len = length; 
    if(check_mem_region(loc_dev->io_base,length)<0){
        err("host controller already in use\n");
        release_mem_region(pci_io_base, iolength);
        iounmap(iobase);
        return -EBUSY;
    }
    if(!request_mem_region(loc_dev->io_base, length,isp1761_driver_name)){
        err("host controller already in use\n");
        release_mem_region(pci_io_base, iolength);
        iounmap(iobase);
        return -EBUSY;

    }

    /*map available memory*/
    address = ioremap_nocache(pci_mem_phy0,length);
    if(address == NULL){
        err("memory map problem\n");
        release_mem_region(pci_io_base, iolength);
        iounmap(iobase);
        release_mem_region(loc_dev->io_base,length);
        return -ENOMEM;
    } 

    loc_dev->baseaddress = (u8*)address;
    loc_dev->dmabase = (u8*)iobase;

	if ((loc_dev != NULL) && ((loc_dev->baseaddress) != NULL))
		hal_init("isp1761 HC MEM Base= %p irq = %d\n", 
			        (void*)(loc_dev->baseaddress),loc_dev->irq);
	else
		hal_init("loc_dev is NULL\n");

#ifdef ISP1761_DEVICE   

    /*initialize device controller framework*/  
    loc_dev = &(isp1761_loc_dev[ISP1761_DC]);
    loc_dev->irq = dev->irq;
    loc_dev->io_base = pci_mem_phy0;
    loc_dev->start   = pci_mem_phy0;
    loc_dev->length  = pci_mem_len;
    loc_dev->io_len = pci_mem_len;
    loc_dev->index = ISP1761_DC;
    loc_dev->baseaddress = address;
    loc_dev->active = 1;
    memcpy(loc_dev->name,"isp1761_dev",11);
    loc_dev->name[12] = '\0';


    {
        u32 chipid = 0;
        chipid = readl(address + 0x270);
        pr_info("pid %04x, vid %04x\n", (chipid & 0xffff), (chipid >> 16));
    }   
	
	if (dev != NULL)
		hal_init("isp1761 DC MEM Base= %lx irq = %d\n", 
					loc_dev->io_base,loc_dev->irq);	
	else
		hal_init("dev is NULL\n");
	
    /* Get the OTG Controller IO and INT resources
     * OTG controller resources are same as Host Controller resources
     */
    loc_dev = &(isp1761_loc_dev[ISP1761_OTG]);
    loc_dev->irq = dev->irq; /*same irq also*/
    loc_dev->io_base = pci_mem_phy0;
    loc_dev->start   =  pci_mem_phy0;
    loc_dev->length  = pci_mem_len;     
    loc_dev->io_len = pci_mem_len;
    loc_dev->index = ISP1761_OTG; 
    loc_dev->baseaddress = address; /*having the same address as of host*/
    loc_dev->active = 1;
    memcpy(loc_dev->name,"isp1761_otg",11);
    loc_dev->name[12] = '\0';

	if (loc_dev != NULL)
		hal_init("isp1761 OTG MEM Base= %lx irq = %x\n", 
				loc_dev->io_base,loc_dev->irq);
	else
		hal_init("loc_dev is NULL\n");
    

#endif
    /* bad pci latencies can contribute to overruns */ 
    pci_read_config_byte (dev, PCI_LATENCY_TIMER, &latency);
    if (latency) {
        pci_read_config_byte (dev, PCI_MAX_LAT, &limit);
        if (limit && limit < latency) {
            dbg ("PCI latency reduced to max %d", limit);
            pci_write_config_byte (dev, PCI_LATENCY_TIMER, limit);
            isp1761_pci_latency = limit;
        } else {
            /* it might already have been reduced */
            isp1761_pci_latency = latency;
        }
    }

    /* Try to check whether we can access Scratch Register of
     * Host Controller or not. The initial PCI access is retried until 
     * local init for the PCI bridge is completed 
     */

    loc_dev = &(isp1761_loc_dev[ISP1761_HC]);
    retry_count = PCI_ACCESS_RETRY_COUNT;
    while((reg_data != 0xFACE) && retry_count) {
        /*by default host is in 16bit mode, so
         * io operations at this stage must be 16 bit
         * */
        isp1761_reg_write16(loc_dev, HC_SCRATCH_REG, 0xFACE);
        udelay(100);
        reg_data = isp1761_reg_read16(loc_dev, HC_SCRATCH_REG,reg_data);
        retry_count--;
    }

    /* Host Controller presence is detected by writing to scratch register
     * and reading back and checking the contents are same or not
     */
    if(reg_data != 0xFACE) {
        err("%s scratch register mismatch %x",
                isp1761_driver_name,reg_data);
        status = -ENODEV;
        goto clean;
    }

    memcpy(loc_dev->name, isp1761_driver_name, sizeof(isp1761_driver_name));
    loc_dev->name[sizeof(isp1761_driver_name)] = 0;
    loc_dev->active = 1;

    pr_info("controller address %p\n", &dev->dev);
    /*keep a copy of pcidevice*/
    loc_dev->pcidev = dev;


    pci_set_master(dev);
    hal_data.irq_usage = 0;
    pci_set_drvdata (dev, loc_dev);
    hal_entry("%s: Exit\n",__FUNCTION__);
    return 1;

clean:
    release_mem_region(pci_io_base, iolength);
    iounmap(iobase);
    release_mem_region(loc_dev->io_base, loc_dev->io_len);
    iounmap(loc_dev->baseaddress);
    hal_entry("%s: Exit\n",__FUNCTION__);
    return status;
} /* End of isp1761_pci_probe */
#endif
/*--------------------------------------------------------------*
 *
 *  Module dtatils: isp1761_pci_remove
 *
 * PCI cleanup function of ISP1761
 * This function is called from PCI Driver as an removal function
 * in the absence of PCI device or a de-registration of driver.
 * This functions checks the registerd drivers (HCD, DCD, OTG) and calls
 * the corresponding removal functions. Also initializes the local variables
 * to zero.
 *
 *  Input: 
 *              struct pci_dev *dev                     ----> PCI Devie data structure 
 *    
 *  Output void
 *
 *  Called by: system function module_cleanup 
 * 
 * 
 * 
 --------------------------------------------------------------*/
 #ifdef NON_PCI
static
#ifdef USE_PLATFORM_DEVICE
int
#else
void
#endif
__devexit isp1761_remove(
#ifdef USE_PLATFORM_DEVICE
	struct platform_device *pdev
#else
	struct device *dev
#endif
	)
{
#ifdef USE_PLATFORM_DEVICE
	struct device *dev = &pdev->dev;
#endif
    struct isp1761_dev  *loc_dev;

	if (dev != NULL)
		hal_init("isp1761_pci_remove(dev=%p)\n",dev);
	else
		hal_init("dev is NULL\n");

    /*Lets handle the host first*/
    loc_dev  = &isp1761_loc_dev[ISP1761_HC];

    /*free the memory occupied by host*/
    release_mem_region(loc_dev->io_base, loc_dev->io_len);      

    /*unmap the occupied memory resources*/
    iounmap(loc_dev->baseaddress);

    return
#ifdef USE_PLATFORM_DEVICE
	    0
#endif
	    ;
} /* End of isp1761_remove */
 #else /*PCI*/
    static void __devexit
isp1761_pci_remove (struct pci_dev *dev)
{
    struct isp1761_dev  *loc_dev;
	if (dev != NULL)
		hal_init("isp1761_pci_remove(dev=%p)\n",(void*)dev);
	else
		hal_init("dev is NULL\n");

    /*Lets handle the host first*/
    loc_dev  = &isp1761_loc_dev[ISP1761_HC];
    /*free the memory occupied by host*/
    release_mem_region(loc_dev->io_base, loc_dev->io_len);      
    release_mem_region(pci_io_base, iolength);
    /*unmap the occupied memory resources*/
    iounmap(loc_dev->baseaddress);
    /* unmap the occupied io resources*/
    iounmap(iobase); 
    return;
} /* End of isp1761_pci_remove */
#endif
/*--------------------------------------------------------------*
 *
 *  Module dtatils: isp1761_pci_suspend
 *
 * PCI suspend function of ISP1761
 * This function is called from PCI Driver.
 * This functions checks the registerd drivers (HCD, DCD, OTG) and calls
 * the corresponding suspend functions if present. 
 *  Input: 
 *              struct pci_dev *dev                     ----> PCI Devie data structure 
 *    
 *  Output void
 *
 *  Called by: system function 
 * 
 * 
 * 
 --------------------------------------------------------------*/
 #ifdef NON_PCI
static int isp1761_suspend (
#ifdef USE_PLATFORM_DEVICE
	struct platform_device *pdev,
	pm_message_t level
#else
	struct device *dev,
	u32 state,
	u32 level
#endif
	) 
{
#ifdef USE_PLATFORM_DEVICE
	struct device *dev = &pdev->dev;
#endif
    struct isp1761_dev  *loc_dev;

	if (dev != NULL)
#ifdef USE_PLATFORM_DEVICE
		hal_init("isp1761_pci_suspend(pdev=%p)\n",pdev);
#else
    hal_init("isp1761_pci_suspend(dev=%p, state = %x)\n",dev, state);
#endif
	else
		hal_init("dev is NULL\n");

    loc_dev = (struct isp1761_dev *) dev_get_drvdata(dev);

    return 0;
} /* End of isp1761_suspend */
 #else /*PCI*/
int isp1761_pci_suspend (struct pci_dev *dev, __u32 state) 
{
    struct isp1761_dev  *loc_dev;
    int                 index;

	if (dev != NULL)
		hal_init("isp1761_pci_suspend(dev=%p, state = %x)\n",(void*)dev, state);
	else
		hal_init("dev is NULL\n");


    loc_dev = (struct isp1761_dev *)pci_get_drvdata(dev);

    /* For each controller check whether driver is registerd
     * or not. If registerd call the suspend function if it is
     * present
     */
    for(index=ISP1761_1ST_DEV;index<ISP1761_LAST_DEV;(index++,loc_dev++)) {
        if(loc_dev->driver && loc_dev->driver->suspend) {
            loc_dev->driver->suspend(loc_dev);
        }
    }

    return 0;
} /* End of isp1761_pci_suspend */
#endif

/*--------------------------------------------------------------*
 *
 *  Module dtatils: isp1761_pci_suspend
 *
 *  PCI resume function of ISP1761
 * This function is called from PCI Driver.
 * This functions checks the registerd drivers (HCD, DCD, OTG) and calls
 * the corresponding resume functions if present.  
 *  Input: 
 *              struct pci_dev *dev                     ----> PCI Devie data structure 
 *    
 *  Output void
 *
 *  Called by: system function
 * 
 * 
 --------------------------------------------------------------*/
 #ifdef NON_PCI
static int isp1761_resume (
#ifdef USE_PLATFORM_DEVICE
	struct platform_device *pdev
#else
	struct device *dev,
	u32 level
#endif
)
{
#ifdef USE_PLATFORM_DEVICE
	struct device *dev = &pdev->dev;
#endif
    struct isp1761_dev  *loc_dev;

	if (dev != NULL)
		hal_init("isp1362_pci_resume(dev=%p)\n",dev);
	else
		hal_init("dev is NULL\n");

    loc_dev = (struct isp1761_dev *) dev_get_drvdata(dev);

    return 0;
} /* End of isp1362_resume */
 #else /*PCI*/
int isp1761_pci_resume (struct pci_dev *dev)
{
    struct isp1761_dev  *loc_dev;
    int                 index;

	if (dev != NULL)
		hal_init("isp1362_pci_resume(dev=%p)\n",(void*)dev);
	else
		hal_init("dev is NULL \n");

    loc_dev = (struct isp1761_dev *)pci_get_drvdata(dev);

    /* For each controller check whether driver is registerd
     * or not. If registerd call the resume function if it is
     * present
     */
    for(index=ISP1761_1ST_DEV;index<ISP1761_LAST_DEV;(index++,loc_dev++)) {
        if(loc_dev->driver && loc_dev->driver->resume) {
            loc_dev->driver->resume(loc_dev);
        }
    }

    return 0;

} /* End of isp1362_pci_resume */
#endif


EXPORT_SYMBOL(isp1761_reg_write16);
EXPORT_SYMBOL(isp1761_reg_read16);
EXPORT_SYMBOL(isp1761_reg_read32);
EXPORT_SYMBOL(isp1761_reg_write32);
EXPORT_SYMBOL(isp1761_request_irq);
EXPORT_SYMBOL(isp1761_mem_read);
EXPORT_SYMBOL(isp1761_mem_write);
EXPORT_SYMBOL(isp1761_free_irq);
EXPORT_SYMBOL(isp1761_register_driver);
EXPORT_SYMBOL(isp1761_unregister_driver);
EXPORT_SYMBOL(isp1761_disable_interrupt);
EXPORT_SYMBOL(isp1761_enable_interrupt);

MODULE_AUTHOR (DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE ("GPL");
#ifdef NON_PCI
module_init (isp1761_module_init);
module_exit (isp1761_module_cleanup);
#else /*PCI*/
module_init (isp1761_pci_module_init);
module_exit (isp1761_pci_module_cleanup);
#endif


