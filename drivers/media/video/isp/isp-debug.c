

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include "isp.h"

static struct proc_dir_entry *isp_dir;

typedef struct {
	char name[8];
	enum isp_mem_resources region;
	struct isp_device *isp;
} isp_proc_entry_data_t;

static isp_proc_entry_data_t isp_proc_entry_data[OMAP3_ISP_IOMEM_CSI2PHY+1];


static int isp_region_read(char *page, char **start,
                            off_t off, int count,
                            int *eof, void *pdata)
{
        int i, len;
        isp_proc_entry_data_t *data = pdata;
	struct isp_device *isp;
	enum isp_mem_resources e;
	unsigned long *base,*phys;

	isp = data->isp;
	e = data->region;
	base = isp->mmio_base[e];
	phys = isp->mmio_base_phys[e];

        len = sprintf(page, "phys: %lX mapped: %lX Size %d\n", 
		isp->mmio_base_phys[e], isp->mmio_base[e], isp->mmio_size[e]);
	for(i=0; i<isp->mmio_size[e]/4; i++) 
	{
		unsigned long out;
		out = __raw_readl(base+i);
		if(i%4==0) len += sprintf(page+len, "\n%08x:",phys+i);
		len += sprintf(page+len, "%08X ", out);
	}
	len += sprintf(page+len, "\n");
#if 0
#endif

        return len;
}



static void isp_create_entry(char *name, enum isp_mem_resources e, struct isp_device *isp)
{
	isp_proc_entry_data_t *data = isp_proc_entry_data+e;
	struct proc_dir_entry *entry = NULL;

	strlcat(data->name, name, 8);
	data->region = e;
	data->isp = isp;
	
	if((entry=create_proc_entry(name, 0644, isp_dir))==NULL) return;
	entry->data = data;
	entry->read_proc = isp_region_read;

		
}

#define CPE(x) isp_create_entry(#x, OMAP3_ISP_IOMEM_##x, isp)
#define RPE(x) remove_proc_entry(#x, isp_dir)

int init_isp_debug(struct isp_device *isp)
{

	isp_dir = proc_mkdir("isp", NULL);
	if(isp_dir==NULL) return -ENOMEM;

	isp_get();
	CPE(MAIN);	
        CPE(CBUFF);
        CPE(CCP2);
        CPE(CCDC);
        CPE(HIST);
        CPE(H3A);
        CPE(PREV);
        CPE(RESZ);
        CPE(SBL);
        CPE(CSI2A);
        CPE(CSI2PHY);
	return 0;
}

void cleanup_isp_debug(void)
{
	RPE(MAIN);	
        RPE(CBUFF);
        RPE(CCP2);
        RPE(CCDC);
        RPE(HIST);
        RPE(H3A);
        RPE(PREV);
        RPE(RESZ);
        RPE(SBL);
        RPE(CSI2A);
        RPE(CSI2PHY);
	remove_proc_entry("isp", NULL);
	isp_put();
	
}


#if 0
module_init(init_isp_proc);
module_exit(fini_isp_proc);

MODULE_AUTHOR("John Kelly");
MODULE_DESCRIPTION("Debugging procfs for isp");
MODULE_LICENSE("GPL");
#endif

