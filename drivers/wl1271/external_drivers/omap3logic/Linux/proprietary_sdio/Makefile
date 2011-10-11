#EXTRA_CFLAGS += -DOMAP3_EVM -DCONFIG_SDIO_ADDRESS_MAPPING_BY_APPLICATION

obj-m := SdioDrv.o
#obj-m += testsdio.o
#testsdio-objs := testdrv.o SdioAdapter.o

PWD := $(shell pwd)
all:
	$(MAKE) CROSS_COMPILE=$(CROSS_COMPILE) ARCH=$(ARCH) -C $(KERNEL_DIR) M=$(PWD)  modules

clean:
	rm -f *.o *~ *.~* core .depend dep $(SRCS_OBJ)
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions
	rm -f modules.order Module.symvers
