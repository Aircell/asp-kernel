/********************************************************************
 * ISP176x Linux Host Controller driver : pehcd
 * Debugging and Conditinal Compilation header file
 *
 * (c) 2009 ST-ERICSSON, All rights reserved. <wired.support@stnwireless.com>
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
 * File Name: isp1761.h
 *
 * Refering linux kernel version 2.6.9/11/20/28
 *
 * History:
 *
 * Date               Author                  Comments
 * --------------------------------------------------------------------
 * 28 Jan 2009      ST-ERICSSON               Initial Creation   
 *
 **********************************************************************
 */


/*******************START HOST CONTROLLER********************************/
/* Host controller conditional defines*/

#undef LINUX_2611		/* define if you use 2.6.11 kernel */
#undef LINUX_269		/* define if you use 2.6.9 kernel */
#define NON_PCI			/* define to Non PCI Host   */
#undef SWAP                     /* define for Big-Endian CPU */
#define MSEC_INT_BASED          /* define to run host on SOF interrupt */
#define PORT1_HOST		/* define if you want ISP1761 port 1 as Host */
#define ISP1761_SUDDEN_DISCONNECT_FIX  /* define to fix sudden disconnect problem */
#undef ISP1761_CONFIG_PM	/* define if power mgmt implemented */
#define LOGIC_OMAP3530_SOM_USB1760 /* ISP 1760 USB code specific to Logic's OMAP3530 SOM */

#ifdef PORT1_HOST    
#define OTG_CONTROL_REG		0x374
#define PORT1_HOST_VALUE	0x00800018	
#endif
/* Determines what method to use when doing NAK retry */
#define HW_NAK_RETRY		/*define if you use HW NAK Retry */
#undef SW_NAK_RETRY		/*define if you use SW NAL Retry */
//#undef HW_NAK_RETRY		/*define if you use HW NAK Retry */
//#define SW_NAK_RETRY		/*define if you use SW NAL Retry */

#ifdef HW_NAK_RETRY
#define RELOAD_CNT 0x0
#define NAK_CNT 0x0
#define CERR 0x2
#endif

#ifdef SW_NAK_RETRY
#define RELOAD_CNT 0xf
#define NAK_CNT 0xf
#define CERR 0x3
#endif

	/*--------------------------------------*/
		/* Start ISO support stuff*/

#ifdef MSEC_INT_BASED /* Iso support is only currently available in this mode */
#define CONFIG_ISO_SUPPORT /* Comment to remove isochronous transfer support */
#endif

#ifdef CONFIG_ISO_SUPPORT

#define ISO_DBG_ENTRY FALSE
#define ISO_DBG_EXIT FALSE
#define ISO_DBG_ADDR FALSE
#define ISO_DBG_DATA FALSE
#define ISO_DBG_ERR  TRUE
#define ISO_DBG_INFO FALSE

#if 0 /* Set to 1 to enable isochronous debugging */
#define iso_dbg(category, format, arg...) \
do \
{ \
    if(category) \
    { \
        printk(format, ## arg); \
    } \
} while(0)
#else
#define iso_dbg(category, format, arg...) while(0)
#endif

#endif /* CONFIG_ISO_SUPPORT */

		/*End ISO support stuff*/
	/*------------------------------------------*/

/*Debug For Entry/Exit of the functions */ 
#undef HCD_DEBUG_LEVEL1
//#define HCD_DEBUG_LEVEL1
#ifdef HCD_DEBUG_LEVEL1
#define pehci_entry(format, args... ) printk(format, ##args)
#else
#define pehci_entry(format, args...) do { } while(0)
#endif

/*Debug for Port Info and Errors */
#undef HCD_DEBUG_LEVEL2
//#define HCD_DEBUG_LEVEL2
#ifdef HCD_DEBUG_LEVEL2
#define pehci_print(format, args... ) printk(format, ##args)
#else
#define pehci_print(format, args...) do { } while(0)
#endif

/*Debug For the Port changes and Enumeration */
#undef HCD_DEBUG_LEVEL3
//#define HCD_DEBUG_LEVEL3
#ifdef HCD_DEBUG_LEVEL3
#define pehci_info(format,arg...) printk(format, ##arg)
#else
#define pehci_info(format,arg...) do {} while (0)
#endif

/*Debug For Transfer flow  */
#undef HCD_DEBUG_LEVEL4
//#define HCD_DEBUG_LEVEL4
#ifdef HCD_DEBUG_LEVEL4
#define pehci_check(format,args...) printk(format, ##args)
#else
#define pehci_check(format,args...)
#endif 

/*Debug For Root Hub flow  */
#undef HCD_DEBUG_LEVEL5
//#define HCD_DEBUG_LEVEL5
#ifdef HCD_DEBUG_LEVEL5
#define pehci_rhinfo(format,args...) printk(format, ##args)
#else
#define pehci_rhinfo(format,args...)
#endif 

/*******************END HOST CONTROLLER**********************************/



/*******************START DEVICE CONTROLLER******************************/

/* For MTP support */
#undef MTP_ENABLE /* Enable to add MTP support on ISP1761 Device Controller
                   * Requires MTP class driver 
                   */

/*Debug Entery/Exit of Function as well as some other Information(I'm not sure)*/
#undef DEV_DEBUG_LEVEL2
//#define DEV_DEBUG_LEVEL2
#ifdef DEV_DEBUG_LEVEL2
#define dev_print(format,arg...) printk(format, ##arg)
#else
#define dev_print(format,arg...) do {} while (0)
#endif

/*Debug for Interrupt , Registers , Device Enable/Disable and some other info */
#undef DEV_DEBUG_LEVEL3
//#define DEV_DEBUG_LEVEL3
#ifdef DEV_DEBUG_LEVEL3
#define isp1761_dev_info (format,arg...) printk(format, ##arg)
#else
#define isp1761_dev_info (format,arg...) do {} while (0)
#endif

/*Debug for Tranffer flow , Enumeration and Packet info */
#undef DEV_DEBUG_LEVEL4
//#define DEV_DEBUG_LEVEL4
#ifdef DEV_DEBUG_LEVEL4
#define dev_check(format,args...) printk(format, ##args)
#else
#define dev_check(format,args...) do{}while(0)
#endif
/*******************END DEVICE CONTROLLER********************************/


/*******************START MSCD*******************************************/
/*Debug Entery/Exit of Function as well as some other Information(I'm not sure)*/
#undef MSCD_DEBUG_LEVEL2
//#define MSCD_DEBUG_LEVEL2
#ifdef MSCD_DEBUG_LEVEL2
#define mscd_print(format,arg...) printk(format, ##arg)
#else
#define mscd_print(format,arg...) do {} while (0)
#endif

/*Debug for Info */
#undef MSCD_DEBUG_LEVEL3
//#define MSCD_DEBUG_LEVEL3
#ifdef MSCD_DEBUG_LEVEL3
#define mscd_info(format,arg...) printk(format, ##arg)
#else
#define mscd_info(format,arg...) do {} while (0)
#endif
/*******************END MSCD*********************************************/


/*******************START OTG CONTROLLER*********************************/
#undef OTG      /*undef for Host only and Device only */
#undef ALL_FSM_FLAGS
/*Debug for Entry/Exit and Info */
#undef OTG_DEBUG_LEVEL1
#ifdef OTG_DEBUG_LEVEL1
#define otg_entry(format, args... ) printk(format, ##args)
#else
#define otg_entry(format, args...) do { } while(0)
#endif

/*Debug for State Machine Flow */
#undef OTG_DEBUG_LEVEL2
#ifdef OTG_DEBUG_LEVEL2
#define otg_print(format,arg...) printk(format, ##arg)
#else
#define otg_print(format,arg...) do {} while (0)
#endif
/*Debug for Info */
#undef OTG_DEBUG_LEVEL3
#ifdef OTG_DEBUG_LEVEL3
#define otg_info(format,arg...) printk(format, ##arg)
#else
#define otg_info(format,arg...) do {} while (0)
#endif
/*******************END OTG CONTROLLER***********************************/



/*******************START FOR HAL ***************************************/
/*Debug For Entry and Exit of the functions */ 
#undef HAL_DEBUG_LEVEL1
//#define HAL_DEBUG_LEVEL1
#ifdef HAL_DEBUG_LEVEL1
#define hal_entry(format, args... ) printk(format, ##args)
#else
#define hal_entry(format, args...) do { } while(0)
#endif

/*Debug For Interrupt information */ 
#undef HAL_DEBUG_LEVEL2
//#define HAL_DEBUG_LEVEL2
#ifdef HAL_DEBUG_LEVEL2
#define hal_int(format, args... ) printk(format, ##args)
#else
#define hal_int(format, args...) do { } while(0)
#endif

/*Debug For HAL Initialisation and Mem Initialisation */ 
#undef HAL_DEBUG_LEVEL3
//#define HAL_DEBUG_LEVEL3
#ifdef HAL_DEBUG_LEVEL3
#define hal_init(format, args... ) printk(format, ##args)
#else
#define hal_init(format, args...) do { } while(0)
#endif
/*******************END FOR HAL*******************************************/



/*******************START FOR ALL CONTROLLERS*****************************/
#undef CONFIG_USB_OTG   /*undef for Host only and Device only */
#undef ISP1761_DEVICE   

#ifdef CONFIG_USB_DEBUG
#define DEBUG
#else
#undef DEBUG
#endif
/*******************END FOR ALL CONTROLLERS*******************************/

/*******************ISP1761_SUDDEN_DISCONNECT_FIX*************************/
#ifdef ISP1761_SUDDEN_DISCONNECT_FIX
#undef SUDDEN_DISCONNECT_DEBUG_LEVEL1
//#define SUDDEN_DISCONNECT_DEBUG_LEVEL1
#ifdef SUDDEN_DISCONNECT_DEBUG_LEVEL1
#define sdf_entry(format, args... ) printk(format, ##args)
#else
#define sdf_entry(format, args...) do { } while(0)
#endif

#undef SUDDEN_DISCONNECT_DEBUG_LEVEL2
//#define SUDDEN_DISCONNECT_DEBUG_LEVEL2
#ifdef SUDDEN_DISCONNECT_DEBUG_LEVEL2
#define sdf_info(format, args... ) printk(format, ##args)
#else
#define sdf_info(format, args...) do { } while(0)
#endif

#endif
/*******************END ISP1761_SUDDEN_DISCONNECT_FIX**********************/

