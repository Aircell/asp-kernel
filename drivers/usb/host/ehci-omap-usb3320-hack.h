#ifndef EHCI_OMAP_USB3320_HACK_H
#define EHCI_OMAP_USB3320_HACK_H

#include <linux/device.h>
#include <linux/interrupt.h>

/** Init the USB3320 hack; specify a return handler, and a device to callback to.
    Returns -ENODEV if the hack has already been initted to another device 
    
    Return handler is called when there is a remote wakeup request.
    */
int usb3320_hack_init(irq_handler_t handler, struct device *dev);
int usb3320_hack_deinit(void);

int usb3320_hack_install(void);
int usb3320_hack_remove(void);

int usb3320_hack_set_wakeup(int wakeup);

#endif // EHCI_OMAP_USB3320_HACK_H
