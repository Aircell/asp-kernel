/*
 *
 *  BRF6300 (SPI) Bluetooth driver
 *
 *  Copyright (C) 2008  Logic Product Development.
 *  Copyright (C) 2008  Xijian Chen <xijianc@logicpd.com>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/firmware.h>

#include <linux/spi/brf6300.h>

#include <linux/spi/spi.h>
#include <mach/gpio.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#undef   BRF6300_DEBUG_MSG
//#define  BRF6300_DEBUG_MSG

#if defined(BRF6300_DEBUG_MSG)
	#define BRF6300_DEBUG(fmt, arg...)	printk("Brf6300 debug: "fmt, ##arg)
#else
	#define BRF6300_DEBUG(fmt, arg...)
#endif 

#define VERSION "0.1"

#define 	ACTION_SEND_CMD          1
#define 	ACTION_WAIT_EVENT        2
#define 	ACTION_SERIAL_PORT       3
#define 	ACTION_DELAY             4
#define 	ACTION_RUN_SCRIPT        5
#define 	ACTION_REMARKS           6

#define 	BRF_6300_STATE_CLOSE    	0x00000001			
#define 	BRF_6300_STATE_SCRIPT_RUNNED	0x00000002
#define 	BRF_6300_STATE_READ_REQUEST     0x00000004
#define 	BRF_6300_STATE_WRITE_REQUEST    0x00000008
#define 	BRF_6300_STATE_ISR_ACTIVE       0x00000010
#define 	BRF_6300_STATE_IRQ_DISABLED     0x00000020

#define 	CMD_COMPLETE_EVENT_CODE         0x0e

struct brf6300_spi_struct 
{
	struct spi_device       *spi;
	struct hci_dev		*hdev;
	struct sk_buff_head	txq;
	struct task_struct 	*task;
	struct completion	task_completion;
	struct semaphore	sem;
	int			state;
};

const u8 HCI_VS_TI_SPI_CFG_CMD[] = 
{
	0x01,				/* HCI Command Packet Type */
	0x41,0xFD,			/* HCI_VS_TI_SPI_CFG Opode */
	0x11,				/* Command data length */
	0x00,0x01,0x00,0x00,0x00,	/* Command data */
	0x00,0x01,0x00,0x00,0x00,	/* Command data */
	0x00,0x00,0x00,0x00,0x00,	/* Command data */
	0x00,0x00			/* Command data */
};            

const u8 READ_HEADER[] = 
{
	0x03, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define BUF_MAX_LENGTH               (522)
static u8 rx_spi_data[BUF_MAX_LENGTH];	

enum
{
	HCI_COMMAND_PACKET_ID = 1,
	HCI_ACL_DATA_PACKET_ID,
	HCI_SCO_DATA_PACKET_ID,
	HCI_EVENT_PACKET_ID,
};

typedef struct
{
	u8	type;
	u8	event_code;
	u8 	length;
	u8	cmd_packets;
	u16	cmd_opcode;
	u8	status;
} hci_packet_cmd_complete_event;

static int txrx_command(const u8* tx_data, u16 tx_len, u8** rx_data, u16* rx_len, struct spi_device *spi);
static int run_script_entry(u16 code, u16 size, const u8* parse_ptr, struct spi_device *spi);
static int run_script(const u8* script_start_addr, size_t script_len, struct spi_device *spi);
static irqreturn_t brf6300_irq(int irq, void *v);
static int brf6300_thread(void *v);
static int send_frame(struct sk_buff *skb);
static int recv_frame(struct brf6300_spi_struct *brf6300_spi);

static irqreturn_t brf6300_irq(int irq, void *v)
{
	struct brf6300_spi_struct *brf6300_spi = v;

	if(gpio_get_value(BT_IRQ_GPIO) != 0)
	{
		BRF6300_DEBUG("Spurious bluetooth interrupt\n");
		return IRQ_HANDLED;
	}

	BRF6300_DEBUG("Interrupt occurs with irq number %d\n", irq);
	disable_irq_nosync(irq);
	brf6300_spi->state |= BRF_6300_STATE_IRQ_DISABLED;
	if(brf6300_spi->state & BRF_6300_STATE_ISR_ACTIVE)
	{
		brf6300_spi->state |= BRF_6300_STATE_READ_REQUEST;
		up(&brf6300_spi->sem);
		BRF6300_DEBUG("Bluetooth IRQ handler gives green light\n");
	}

	return IRQ_HANDLED;
}

static int recv_frame(struct brf6300_spi_struct *brf6300_spi)
{
	int ret;
	int loop;
	u16 rx_len;
	u8 *rx_data;
	struct sk_buff *skb;
	u8 temp;

	ret = spi_bt_write_then_read(brf6300_spi->spi,
		   	       READ_HEADER,
                               2,
                               rx_spi_data,
                               0x12);

	rx_len = (rx_spi_data[3] << 8) | rx_spi_data[2];
	rx_data = rx_spi_data + 4;

	for(loop = 0; loop < rx_len; loop += 2)
	{
		temp = rx_data[loop];
		rx_data[loop] = rx_data[loop + 1];
		rx_data[loop + 1] = temp;
	}

	skb = bt_skb_alloc(rx_len - 1, GFP_KERNEL);
	if(!skb)
	{
		BT_ERR("skb allocation error\n");
		return -ENOMEM;
	}

	skb_put(skb, rx_len - 1);
	skb_copy_to_linear_data(skb, rx_data + 1, rx_len - 1);
	brf6300_spi->hdev->stat.byte_rx += rx_len - 1;
	
	skb->dev = (void *)brf6300_spi->hdev;
	bt_cb(skb)->pkt_type = *rx_data;

	BRF6300_DEBUG("pkt type: 0x%x\n", bt_cb(skb)->pkt_type);
	BRF6300_DEBUG("pkt length: 0x%x\n", skb->len);
#if defined(BRF6300_DEBUG_MSG)
	for(loop = 0; loop < (rx_len - 1); loop++)
		BRF6300_DEBUG("pkt data[0x%x] = 0x%x\n", loop, skb->data[loop]);
#endif
	udelay(500); 
	ret = hci_recv_frame(skb);

	BRF6300_DEBUG("hci_recv_frame returns 0x%x\n", ret);

	return ret;
}

static int brf6300_thread(void *v)
{
	struct brf6300_spi_struct *brf6300_spi = v;
	struct sk_buff *skb;
	int ret;

	brf6300_spi->task = current;

	daemonize("brf6300d");
	allow_signal(SIGKILL);

	BRF6300_DEBUG("before sending task_completion\n");
	complete(&brf6300_spi->task_completion);
	BRF6300_DEBUG("after sending task_completion\n");

	do
	{
		BRF6300_DEBUG("before waiting on sem\n");
		if (down_interruptible(&brf6300_spi->sem))
		{
			BT_ERR("semaphore error\n");
			return -EINTR;
		}
		BRF6300_DEBUG("after waiting on sem\n");

		if(brf6300_spi->state & BRF_6300_STATE_CLOSE)
		{
			BRF6300_DEBUG("Exit brf6300d kernel thread\n");
			break;
		}

		if(brf6300_spi->state & BRF_6300_STATE_READ_REQUEST)
		{
			BRF6300_DEBUG("process read request\n");
			ret = recv_frame(brf6300_spi);
			if(ret < 0)
				brf6300_spi->hdev->stat.err_rx++;
				
			brf6300_spi->state &= ~BRF_6300_STATE_READ_REQUEST;
		}
		else if(brf6300_spi->state & BRF_6300_STATE_WRITE_REQUEST)
		{
			BRF6300_DEBUG("process write request\n");
			skb = skb_dequeue(&brf6300_spi->txq);
			if(skb != NULL)
			{
				ret = send_frame(skb);
				if(ret < 0)
				{
					brf6300_spi->hdev->stat.err_tx++;
					skb_queue_head(&brf6300_spi->txq, skb);
				}
			}
			brf6300_spi->state &= ~BRF_6300_STATE_WRITE_REQUEST;
		}

		if(brf6300_spi->state & BRF_6300_STATE_IRQ_DISABLED)
		{
			brf6300_spi->state &= ~BRF_6300_STATE_IRQ_DISABLED;
			enable_irq((brf6300_spi->spi)->irq);
			BRF6300_DEBUG("Enabled Bluetooth irq\n");
		}
		else
		{
			BRF6300_DEBUG("Bluetooth irq used to be enabled here\n");
		}

		if(brf6300_spi->state & BRF_6300_STATE_CLOSE)
		{
			BRF6300_DEBUG("Exit brf6300d kernel thread\n");
			break;
		}
	} while(!signal_pending(brf6300_spi->task));
	
	BRF6300_DEBUG("before complete and exit task_completion\n");
	complete_and_exit(&brf6300_spi->task_completion, 0);
	BRF6300_DEBUG("after complete and exit task_completion\n");
}

static int send_frame(struct sk_buff *skb)
{
	int loop;
	int ret;
	u8 pad_cnt;
	u16 spi_payload_len;
	u8 temp;
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;
	struct brf6300_spi_struct *brf6300_spi = hdev->driver_data;

	BRF6300_DEBUG("Send frame to BRF6300 chip\n");

	if(skb->len % 2)
		pad_cnt = 1;
	else
		pad_cnt = 0;

	spi_payload_len = skb->len + 1 + pad_cnt;

	if((spi_payload_len + 5) > BUF_MAX_LENGTH)
	{
		BT_ERR("send_frame buffer allocated is too small!\n");
		return -ENOMEM;
	}

	rx_spi_data[0] = 0x01;	/* Write op code */
	rx_spi_data[1] = (spi_payload_len >> 8)  & 0xff;
	rx_spi_data[2] = spi_payload_len & 0xff;
	rx_spi_data[3] = 0x0;	/* busy byte */
	rx_spi_data[4] = 0x0;	/* busy byte */
	rx_spi_data[5] = bt_cb(skb)->pkt_type;
	
	skb_copy_from_linear_data(skb, &rx_spi_data[6], skb->len);

	if(pad_cnt)
		rx_spi_data[skb->len + 6] = 0x0;     /* Pad byte value is 0. */

#if defined(BRF6300_DEBUG_MSG)
	for(loop = 0; loop < (spi_payload_len + 5); loop ++)
	{
		BRF6300_DEBUG("send_frame: rx_spi_data[0x%x] = 0x%x\n", loop, rx_spi_data[loop]);
	}
#endif

	for(loop = 0; loop < (spi_payload_len + 5); loop += 2)
	{
		temp = rx_spi_data[loop];
		rx_spi_data[loop] = rx_spi_data[loop + 1];
		rx_spi_data[loop + 1] = temp;
	}

	ret = spi_write(brf6300_spi->spi, 
                       rx_spi_data,
	               spi_payload_len + 5);

	if(ret < 0)
	{
		BT_ERR("SPI write failure\n");
		return ret;
	}
	
	brf6300_spi->hdev->stat.byte_tx += skb->len;

	return 0;
}

static int txrx_command(const u8* tx_data, u16 tx_len, u8** rx_data, u16* rx_len, struct spi_device *spi)
{
	int ret;
	u8 pad_cnt;
	u16 spi_payload_len;
	int loop;
	u8 temp;
	u32 timeout;

	if(tx_len % 2)
		pad_cnt = 0;
	else
		pad_cnt = 1;

	spi_payload_len = tx_len + pad_cnt;

	if((spi_payload_len + 5) > BUF_MAX_LENGTH)
	{
		BT_ERR("tx buffer allocated is too small!\n");
		return -ENOMEM;
	}

	rx_spi_data[0] = 0x01;	/* Write op code */
	rx_spi_data[1] = (spi_payload_len >> 8)  & 0xff;
	rx_spi_data[2] = spi_payload_len & 0xff;
	rx_spi_data[3] = 0x0;	/* busy byte */
	rx_spi_data[4] = 0x0;	/* busy byte */

	for(loop = 0; loop < tx_len; loop++)
		rx_spi_data[loop + 5] = tx_data[loop];

	if(pad_cnt)
		rx_spi_data[tx_len + 5] = 0x0;     /* pad byte value is 0. */

	for(loop = 0; loop < (spi_payload_len + 5); loop += 2)
	{
		temp = rx_spi_data[loop];
		rx_spi_data[loop] = rx_spi_data[loop + 1];
		rx_spi_data[loop + 1] = temp;
	}

	ret = spi_write(spi, 
                       rx_spi_data,
	               spi_payload_len + 5);

	if(ret < 0)
	{
		BT_ERR("SPI write failure\n");
		return ret;
	}

	timeout = 0;  
	while((gpio_get_value(BT_IRQ_GPIO) != 0) && (timeout < 1000000))
	{
		udelay(10);
		timeout += 10;
	}
	if(timeout >= 1000000)
	{
		BT_ERR("SPI write failure. No response from BT IRQ signal\n");
		return -EIO;
	}

	msleep(1);


	ret = spi_bt_write_then_read(spi,
		   	       READ_HEADER,
                               2,
                               rx_spi_data,
                               0x12);

	timeout = 0;  
	while((gpio_get_value(BT_IRQ_GPIO) != 1) && (timeout < 1000000))
	{
		udelay(10);
		timeout += 10;
	}
	if(timeout >= 1000000)
	{
		BT_ERR("SPI read failure. No response from BT IRQ signal\n");
		return -EIO;
	}

	*rx_len = (rx_spi_data[3] << 8) | rx_spi_data[2];
	*rx_data = rx_spi_data + 4;

	for(loop = 0; loop < *rx_len; loop += 2)
	{
		temp = (*rx_data)[loop];
		(*rx_data)[loop] = (*rx_data)[loop + 1];
		(*rx_data)[loop + 1] = temp;
	}

	return 0;
}

static int run_script_entry(u16 code, u16 size, const u8* parse_ptr, struct spi_device *spi)
{
	u32 time_delay;
	u16 cmd_id;
	int ret;
	u16 rx_len;
	hci_packet_cmd_complete_event *packet_ptr;
	u8* rx_data_ptr;

	switch(code)
	{
		case ACTION_SEND_CMD:
			if(*parse_ptr != HCI_COMMAND_PACKET_ID)
				return -ENOEXEC;
			cmd_id = (*(parse_ptr+2) << 8) | *(parse_ptr+1);
			if((size & 0xff00) || (*(parse_ptr+3) != (size-4)))
				return -ENOEXEC;
			
			ret = txrx_command(parse_ptr, size, &rx_data_ptr, &rx_len, spi);
			if(ret != 0)
				return ret;

			/* Check read result */
			packet_ptr = (hci_packet_cmd_complete_event *)rx_data_ptr;
			if((packet_ptr->type != HCI_EVENT_PACKET_ID) ||
			   (packet_ptr->event_code != CMD_COMPLETE_EVENT_CODE) ||
			   (packet_ptr->cmd_opcode != cmd_id) ||
			   (packet_ptr->status != 0))
				return -EIO;
			break;

		case ACTION_SERIAL_PORT:
			BRF6300_DEBUG("Serial port setup. Shouldn't run into since this driver supports SPI bus only\n");
			return -ENOEXEC;
			break;

		case ACTION_DELAY:
			time_delay = *parse_ptr |
                                     (*(parse_ptr+1) << 8) |
				     (*(parse_ptr+2) << 16) |
				     (*(parse_ptr+3) << 24);
			BRF6300_DEBUG("Add artificial time delay 0x%x\n", time_delay);
			msleep(time_delay);
			break;

		case ACTION_RUN_SCRIPT:
			BRF6300_DEBUG("Run script. No action taken\n");
			break;

		case ACTION_REMARKS:
		case ACTION_WAIT_EVENT:
			break;
		default:
			BRF6300_DEBUG("Unknown code: 0x%x\n", code);
			return -ENOEXEC;
			break;	
	}
	 	
	return 0;
}
static int run_script(const u8* script_start_addr, size_t script_len, struct spi_device *spi)
{
	const u8* script_upper_limit = script_start_addr + script_len - 1;
	const u8* parse_ptr = script_start_addr;
	u16 code, size;
	int ret;

	BRF6300_DEBUG("Bts script start address: %p\n", script_start_addr);
	BRF6300_DEBUG("Bts script length in bytes: 0x%x\n", script_len);
	BRF6300_DEBUG("Bts script upper limit: %p\n", script_upper_limit);

	if((*parse_ptr != 'B') ||
           (*(parse_ptr+1) != 'T') || 
           (*(parse_ptr+2) != 'S') || 	
           (*(parse_ptr+3) != 'B'))
		return -ENOEXEC;	 

	parse_ptr += 32;
	
	while(parse_ptr < script_upper_limit)
	{
		code = *parse_ptr | (*(parse_ptr +1) << 8);
		if(parse_ptr > script_upper_limit)
		{
			BT_ERR("Bts script execution error: cannot find code header field\n");
			return -ENOMEM;
		}
		parse_ptr += 2;

		size = *parse_ptr | (*(parse_ptr +1) << 8);
		if(parse_ptr > script_upper_limit)
		{
			BT_ERR("Bts script execution error: cannot find code header field\n");
			return -ENOMEM;
		}
		parse_ptr += 2;

		ret = run_script_entry(code, size, parse_ptr, spi);
		if(ret < 0)
		{
			BT_ERR("Bts script execution failure\n");
			return ret;
		}
		parse_ptr += size;
	}

	BRF6300_DEBUG("Bts script exeution completed successfully\n");
	return 0;
}

static int brf6300_spi_open(struct hci_dev *hdev)
{
	struct brf6300_spi_struct *brf6300_spi = hdev->driver_data;
	const struct firmware *fw_entry;
	int ret;
	hci_packet_cmd_complete_event *packet_ptr;
	u8* rx_data_ptr;
	u16 rx_data_len;
	int time_delay;

	BRF6300_DEBUG("brf6300_spi_open\n");

	if (test_and_set_bit(HCI_RUNNING, &hdev->flags)) {
		BRF6300_DEBUG("%s:%d\n", __FUNCTION__, __LINE__);
		return 0;
	}

	brf6300_spi->state &= ~BRF_6300_STATE_CLOSE;

	/* Initialize muxtex for synchronization. */
	init_MUTEX_LOCKED(&brf6300_spi->sem);
	
	init_completion(&brf6300_spi->task_completion);

	/* Set GPIO pin for BT_IRQ. */
	BRF6300_DEBUG("Bluetooth irq is %d\n", brf6300_spi->spi->irq);
	if(brf6300_spi->spi->irq)
	{
		ret = request_irq(brf6300_spi->spi->irq, 
                                  brf6300_irq, 
                                  IRQF_DISABLED | IRQF_TRIGGER_LOW,
                                  "brf6300",
                                  brf6300_spi);

		if(ret == 0)
		{
			BRF6300_DEBUG("Bluetooth IRQ is configured successfully\n");
			brf6300_spi->state &= ~BRF_6300_STATE_IRQ_DISABLED;
		}  
		else
		{
			BT_ERR("Failed to request Bluetooth IRQ\n");
			return ret;		
		}
	}

     	if(!(brf6300_spi->state & BRF_6300_STATE_SCRIPT_RUNNED))
     	{
		/* Toggle BT_nSHUTDOWN signal */ 
		brf6300_request_bt_nshutdown_gpio();

		brf6300_direction_bt_nshutdown_gpio(0, 1);

        	msleep(5);

		brf6300_direction_bt_nshutdown_gpio(0, 0);

		msleep(10);

		brf6300_set_bt_nshutdown_gpio(1);

		BRF6300_DEBUG("Successfully toggled BT_nSHUTDOWN signal\n");

		/* Wait until BT_IRQ low. */
		time_delay = 0;
		while((gpio_get_value(BT_IRQ_GPIO) != 0) &&
        	      (time_delay < 1000))
		{
			msleep(5);
			time_delay += 5;
		}

		if(gpio_get_value(BT_IRQ_GPIO) != 0)
		{
			BT_ERR("Timeout on waiting for BT_IRQ ready\n");
			return ret;
		}

		ret = txrx_command(HCI_VS_TI_SPI_CFG_CMD, 
				   sizeof HCI_VS_TI_SPI_CFG_CMD,
        	                   &rx_data_ptr,
        	                   &rx_data_len,
				   brf6300_spi->spi);

		if(ret < 0)
		{
			BT_ERR("tx initialization command failed\n");
			return ret;
		}

		packet_ptr = (hci_packet_cmd_complete_event *)rx_data_ptr;
		if((packet_ptr->type != HCI_EVENT_PACKET_ID) ||
		   (packet_ptr->event_code != CMD_COMPLETE_EVENT_CODE) ||
		   (packet_ptr->cmd_opcode != 0xFD41) ||
		   (packet_ptr->status != 0))
			return -EIO;

		ret = request_firmware(&fw_entry, "brf6300/brf61_4.2.38.bts", (void *)brf6300_spi->spi);
		if(ret < 0)
		{
			BT_ERR("Firmware not avaiable\n");
			return ret;
		}
		else
		{	
			ret = run_script(fw_entry->data, fw_entry->size, brf6300_spi->spi);
			if(ret < 0)
			{
				BT_ERR("Failed on running bts initialization script\n");
				return ret;
			}

			release_firmware(fw_entry);
			brf6300_spi->state |= BRF_6300_STATE_SCRIPT_RUNNED;
		}
	}	

	ret = kernel_thread(brf6300_thread, brf6300_spi, CLONE_KERNEL);
	if (ret >= 0) 
	{
		BRF6300_DEBUG("Before waiting for kernel_thread completion\n");
		wait_for_completion(&brf6300_spi->task_completion);
		BRF6300_DEBUG("After waiting for kernel_thread completion\n");
	}
	else
	{
		BT_ERR("Kernel threading error\n");
		return ret;
	}		

	brf6300_spi->state |= BRF_6300_STATE_ISR_ACTIVE;

	return 0;
}

static int brf6300_spi_close(struct hci_dev *hdev)
{
	struct brf6300_spi_struct *brf6300_spi = hdev->driver_data;

	BRF6300_DEBUG("brf6300_spi_close\n");

	brf6300_spi->state |= BRF_6300_STATE_CLOSE;

	if (brf6300_spi->task) 
	{
		send_sig(SIGKILL, brf6300_spi->task, 1);
		up(&brf6300_spi->sem);
		wait_for_completion(&brf6300_spi->task_completion);
	}

	free_irq(brf6300_spi->spi->irq, brf6300_spi);

	brf6300_free_bt_nshutdown_gpio();

	return 0;

}

static int brf6300_spi_flush(struct hci_dev *hdev)
{
	struct brf6300_spi_struct *brf6300_spi = hdev->driver_data;

	BRF6300_DEBUG("brf6300_spi_flush\n");

	skb_queue_purge(&brf6300_spi->txq);

	return 0;
}

static int brf6300_spi_send_frame(struct sk_buff *skb)
{
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;
	struct brf6300_spi_struct *brf6300_spi = hdev->driver_data;

	if(!test_bit(HCI_RUNNING, &hdev->flags))
		return -EBUSY;

	brf6300_spi->state |= BRF_6300_STATE_WRITE_REQUEST;
	up(&brf6300_spi->sem);

	switch (bt_cb(skb)->pkt_type) 
	{
		case HCI_COMMAND_PKT:
			hdev->stat.cmd_tx++;
			break;

		case HCI_ACLDATA_PKT:
			hdev->stat.acl_tx++;
			break;

		case HCI_SCODATA_PKT:
			hdev->stat.sco_tx++;
			break;

		default:
			return -EILSEQ;
	}

	skb_queue_tail(&brf6300_spi->txq, skb);

	return 0;
}

static void brf6300_spi_destruct(struct hci_dev *hdev)
{
	BRF6300_DEBUG("brf6300_spi_destruct");
}

static int __devinit brf6300_spi_probe(struct spi_device *spi)
{
	struct brf6300_spi_struct *brf6300_spi;
	struct hci_dev *hdev;
	int err;

	/* Sanity check in spi input data. */
        if ((spi->bits_per_word && spi->bits_per_word != 16)
		|| (spi->max_speed_hz > 13000000)) {
		printk("%s:%d spi %p bits_per_word %d max_speed %u\n", __FUNCTION__, __LINE__, spi, spi->bits_per_word, spi->max_speed_hz);
 		return -EINVAL;
	}

	/* Set up driver data. */
	brf6300_spi = kzalloc(sizeof *brf6300_spi, GFP_KERNEL);
	if(!brf6300_spi) {
		printk("%s:%d\n", __FUNCTION__, __LINE__);
		return -ENOMEM;
	}
	brf6300_spi->state = 0;

	brf6300_spi->spi = spi;
	spi_set_drvdata(spi, brf6300_spi);

	skb_queue_head_init(&brf6300_spi->txq);

	hdev = hci_alloc_dev();
	if(!hdev)
	{
		kfree(brf6300_spi);
		return -ENOMEM;
	}

	hdev->type = HCI_SPI;
	hdev->driver_data = brf6300_spi;

	brf6300_spi->hdev = hdev;

	SET_HCIDEV_DEV(hdev, &spi->dev);

	hdev->open = brf6300_spi_open;
	hdev->close = brf6300_spi_close;
	hdev->flush = brf6300_spi_flush;
	hdev->send = brf6300_spi_send_frame;
	hdev->destruct = brf6300_spi_destruct;

	hdev->owner = THIS_MODULE;

	err = hci_register_dev(hdev);
	if (err < 0)
	{
		hci_free_dev(hdev);
		kfree(brf6300_spi);
		return err;
	}

	BT_INFO("BRF6300 SPI Bluetooth driver is registered.");

	return 0;
}

static int __devexit brf6300_spi_remove(struct spi_device *spi)
{
	struct brf6300_spi_struct *brf6300_spi;
	struct hci_dev *hdev;

	brf6300_spi = spi_get_drvdata(spi);
	if(!brf6300_spi)
		return -ENODEV;

	hdev = brf6300_spi->hdev;

	spi_set_drvdata(spi, NULL);
	
	hci_unregister_dev(hdev);

	hci_free_dev(hdev);

	BT_INFO("BRF6300 SPI Bluetooth driver is removed.");

	return 0;
}

static struct spi_driver brf6300_spi_driver = {
        .driver = {
                 .name           = "brf6300",
                 .bus            = &spi_bus_type,
                 .owner          = THIS_MODULE,
         },

         .probe          = brf6300_spi_probe,
         .remove         = __devexit_p(brf6300_spi_remove),
};

static int __init brf6300_spi_init(void)
{
int err;
	BT_INFO("BRF6300 SPI Bluetooth driver ver %s", VERSION);

	err = spi_register_driver(&brf6300_spi_driver);
	return err;
}

static void __exit brf6300_spi_exit(void)
{
	spi_unregister_driver(&brf6300_spi_driver);
}

module_init(brf6300_spi_init);
module_exit(brf6300_spi_exit);

MODULE_AUTHOR("Xijian Chen <xijianc@logicpd.com>");
MODULE_DESCRIPTION("brf6300 SPI Bluetooth driver ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("ti_brf6300/brf61_4.2.38.bts");
