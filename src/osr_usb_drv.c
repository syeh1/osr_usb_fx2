/******************************************************************************
 * Copyright @ 2015 Sinclair Yeh
 *****************************************************************************/

#define LINUX

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#include "osr_usb_drv.h"



/**
 * Constants
 */
#define ENDPOINT_NUMBER_MASK  0xF



/**
 * Static Functions
 */
static int fx2_probe(struct usb_interface *intf,
		     const struct usb_device_id *id);
static void fx2_disconnect(struct usb_interface *intf);
static void fx2_debounce_work(struct work_struct *data);


/**
 * Global Variables
 */
static struct usb_device_id id_table[] = {
	{ USB_DEVICE(0x0547, 0x1002) },
	{},
};
MODULE_DEVICE_TABLE(usb, id_table);


/* Info to register with the USB sub system with */
static struct usb_driver osr_usb_fx2_driver = {
	.name       = "osr_usb_fx2",
	.probe      = fx2_probe,
	.disconnect = fx2_disconnect,
	.id_table   = id_table,
};



/**
 * fx2_debounce_work - handles debouncing of the toggle switch
 *
 * This function gets called at most once every 10 milliseconds,  by
 * which time the toggle switch state should have been settled.
 *
 * @data:  Used to get back dev priv
 */
void fx2_debounce_work(struct work_struct *data)
{
	struct osr_usb_fx2_dev *dev_priv;

	dev_priv = container_of(data, struct osr_usb_fx2_dev,
				debounce_work.work);

	spin_lock(&dev_priv->isr_lock);
	printk("WorkQueue: 0x%x\n", dev_priv->switch_val);
	spin_unlock(&dev_priv->isr_lock);
}



/**
 * fx2_isr - Interrupt Service Routine
 *
 * @urb: USB context 
 */
static void fx2_isr(struct urb *urb)
{
	struct osr_usb_fx2_dev *dev_priv = urb->context;
	int ret;


	if (urb->status) {
		if (urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN) {
			return;
		} else {
			printk("Non-Zero URB status\n");
		}
	} else if (urb->actual_length > 0) {
		spin_lock(&dev_priv->isr_lock);
		dev_priv->switch_val = dev_priv->int_buffer[0];
		spin_unlock(&dev_priv->isr_lock);

		schedule_delayed_work(&dev_priv->debounce_work, HZ/10);
	}

	if (dev_priv->udev) {
		/* Resubmit the interrupt request so USB core will
		 * call us again */
		ret = usb_submit_urb(dev_priv->int_urb, GFP_ATOMIC);
		if (ret) {
			printk("Failed to resubmit URB\n");
		}
	}

}



/**
 * fx2_probe - Called when USB core finds our device
 *
 * @intf: USB interface
 * @id: not used
 */
static int fx2_probe(struct usb_interface *intf,
		     const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_host_interface *host_interface;
	struct osr_usb_fx2_dev *dev_priv = NULL;
	int int_end_size;
	int ret = 0, i;


	printk("Calling fx_probe\n");

	if (!udev) {
		printk(KERN_ERR "No USB device\n");
		return -EINVAL;
	}

	host_interface = intf->cur_altsetting;

	dev_priv = kzalloc(sizeof(*dev_priv), GFP_KERNEL);
	if (!dev_priv) {
		printk(KERN_ERR "Cannot allocate device private structure\n");
		return -ENOMEM;
	}

	/* Initialize our private structure */
	dev_priv->udev = udev;
	spin_lock_init(&dev_priv->isr_lock);

	/* Initialize Attributes */
	osr_usb_attrib_init(&intf->dev);

	/* Set up the interrupt endpoint */
	for (i = 0; i < host_interface->desc.bNumEndpoints; i++) {
		struct usb_endpoint_descriptor *endpoint;

		endpoint = &host_interface->endpoint[i].desc;

		printk("Detected Endpoint Number: %d\n",
			endpoint->bEndpointAddress & ENDPOINT_NUMBER_MASK);

		if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) ==
			USB_DIR_IN &&
		    (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
			USB_ENDPOINT_XFER_INT) {
			if (dev_priv->int_endpoint)
				printk(KERN_ERR "More than one INT endpoint\n");

			dev_priv->int_endpoint = endpoint;
		}
	}

	if (!dev_priv->int_endpoint) {
		printk(KERN_ERR "No interrupt endpoint found\n");
		ret = -EINVAL;
		goto error;
	}

	/* Debounce work */
	INIT_DELAYED_WORK(&dev_priv->debounce_work, fx2_debounce_work);

	/**
	 * Initialize ISR
	 */

	/* 0 ISO packets because this is an interrupt EP */
	dev_priv->int_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev_priv->int_urb) {
		printk("Cannot allocate interrupt URB\n");
		ret = -ENOMEM;
		goto error;
	}

	/* Set up buffer to handle input from the board */
	int_end_size = le16_to_cpu(dev_priv->int_endpoint->wMaxPacketSize);
	dev_priv->int_buffer = kmalloc(int_end_size, int_end_size);
	if (!dev_priv->int_buffer) {
		printk("Cannot allocate interrupt buffer\n");
		ret = -ENOMEM;
		goto error;
	}

	usb_fill_int_urb(dev_priv->int_urb, dev_priv->udev,
		usb_rcvintpipe(dev_priv->udev,
			       dev_priv->int_endpoint->bEndpointAddress),
		dev_priv->int_buffer,
		int_end_size,
		fx2_isr,
		dev_priv,
		dev_priv->int_endpoint->bInterval);

	mb();

	ret = usb_submit_urb(dev_priv->int_urb, GFP_KERNEL);
	if (ret) {
		printk("Failed to submit interrupt URB\n");
		goto error;
	}

	/* Associate dev_priv with intf */
	usb_set_intfdata(intf, dev_priv);

	return ret;

error:
	fx2_disconnect(intf);
	return ret;
}



/**
 * fx2_disconnect - Called when the device is disconnected
 *
 * @intf: USB interface
 */
static void fx2_disconnect(struct usb_interface *intf)
{
	struct osr_usb_fx2_dev *dev_priv = usb_get_intfdata(intf);


	printk("fx2_disconnect\n");

	osr_usb_attrib_cleanup(&intf->dev);

	usb_set_intfdata(intf, NULL);

	if (dev_priv->int_urb)
		usb_free_urb(dev_priv->int_urb);

	if (dev_priv->int_buffer)
		kfree(dev_priv->int_buffer);

	kfree(dev_priv);
}



/**
 * init_module - Called when the module is loaded
 *
 * Registers this driver with the USB core.  This way the USB core knows
 * to call our probe function when a USB device with matching vendor/device
 * ID is connected to the system.
 */
int init_module(void)
{
	int ret;

	printk(KERN_INFO "osr_usb_fx2_driver loaded\n");
	ret = usb_register(&osr_usb_fx2_driver);
	if (ret) {
		printk(KERN_ERR "Cannot register driver\n");
		return ret;
	}

	return 0;
}



/**
 * cleanup_module - Called when the module is removed
 *
 * Undo whatever init_module has done
 */
void cleanup_module(void)
{
	usb_deregister(&osr_usb_fx2_driver);
}



MODULE_LICENSE("GPL and additional rights");
MODULE_DESCRIPTION("Driver for the OSR FX2 device");
MODULE_AUTHOR("syeh");

