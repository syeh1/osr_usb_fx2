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
#define BULK_FULL_SPEED_SIZE  64
#define BULK_HIGH_SPEED_SIZE  512

#ifdef CONFIG_USB_DYNAMIC_MINORS
#define FX2_MINOR_BASE 0
#else
#define FX2_MINOR_BASE 88
#endif



/**
 * Static Functions
 */
static int fx2_probe(struct usb_interface *intf,
		     const struct usb_device_id *id);
static void fx2_disconnect(struct usb_interface *intf);
static void fx2_debounce_work(struct work_struct *data);
static int fx2_bulk_open(struct inode *node, struct file *fp);
static int fx2_bulk_release(struct inode *node, struct file *fp);
static ssize_t fx2_bulk_write(struct file *file,
		const char __user *user_buf,
		size_t count,
		loff_t *ppos);
static ssize_t fx2_bulk_read(struct file *file,
		char __user *user_buf,
		size_t count,
		loff_t *ppos);


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


/* Bulk operation are handled via file operations */
static struct file_operations osr_usb_fx2_fops = {
	.owner   = THIS_MODULE,
	.write   = fx2_bulk_write,
	.read    = fx2_bulk_read,
	.open    = fx2_bulk_open,
	.release = fx2_bulk_release,
};


/* Used to register the device and get a minor number */
static struct usb_class_driver osr_usb_fx2_class = {
	.name = "osr_usb%d",
	.fops = &osr_usb_fx2_fops,
	.minor_base = FX2_MINOR_BASE,
};



/**
 * fx2_bulk_open - Does nothing in this case
 *
 * @node: not used
 * @fp: Used to store private structure
 *
 * Returns 0 on success
 */
static int fx2_bulk_open(struct inode *node, struct file *fp)

{
	int subminor;
	struct usb_interface *intf;
	struct osr_usb_fx2_dev *dev_priv = NULL;


	subminor = iminor(node);

	intf = usb_find_interface(&osr_usb_fx2_driver, subminor);
	if (!intf) {
		printk("fx2_bulk_open: Cannot find USB Interface\n");
		return -ENODEV;
	}

	dev_priv = usb_get_intfdata(intf);
	if (!dev_priv) {
		printk("fx2_bulk_open: Cannot find device private structure\n");
		return -ENODEV;
	}

	fp->private_data = dev_priv;

	switch (dev_priv->udev->speed) {
	case USB_SPEED_HIGH:
		dev_priv->bulk_buffer_size = BULK_HIGH_SPEED_SIZE;
		break;
	case USB_SPEED_FULL:
		dev_priv->bulk_buffer_size = BULK_FULL_SPEED_SIZE;
		break;
	default:
		printk("fx2_bulk_open: Unsupported speed\n");
		return -EINVAL;
	}

	/* Allocate bulk buffer */
	if (!dev_priv->bulk_buffer) {
		printk("fx2_bulk_open: Allocate %ld bytes\n",
			dev_priv->bulk_buffer_size);

		dev_priv->bulk_buffer = kmalloc(dev_priv->bulk_buffer_size,
						GFP_KERNEL);
	}

	return 0;
}



/**
 * fx2_bulk_release - Clean up after fx2_bulk_open
 *
 * @node: not used
 * @fp: Used to get private structure
 */
static int fx2_bulk_release(struct inode *node, struct file *fp)

{
	struct osr_usb_fx2_dev *dev_priv = fp->private_data;

	printk("fx2_bulk_release:\n");

	if (dev_priv->bulk_buffer) {
		kfree(dev_priv->bulk_buffer);
		dev_priv->bulk_buffer = NULL;
	}

	return 0;
}



/**
 * fx2_bulk_write - writes user data to bulk write port
 *
 * @fp:  Only needed to get the private data
 * @user_buf:  User data to write
 * @count: size of user buffer
 * @ppos:  Not used
 *
 * Returns: bytes written or an error code.
 */
static ssize_t fx2_bulk_write(struct file *fp,
		const char __user *user_buf,
		size_t count,
		loff_t *ppos)
{
	int ret = 0;
	struct osr_usb_fx2_dev *dev_priv = fp->private_data;
	int bytes_to_write = min(count, dev_priv->bulk_buffer_size);
	int bytes_written = 0;

	if (copy_from_user(dev_priv->bulk_buffer, user_buf, bytes_to_write))
	{
		return -EFAULT;
	}


	/* Write the data into the bulk endpoint */
	ret = usb_bulk_msg(dev_priv->udev,
			   usb_sndbulkpipe(dev_priv->udev,
				dev_priv->bulk_out_endpoint->bEndpointAddress),
			   dev_priv->bulk_buffer,
			   bytes_to_write,
			   &bytes_written, 3*HZ);
	if (ret)
	{
		/* FIXME:  If buffer is full, then perhaps clean out buffer
		 * and try to write again
		 */
		printk("fx2_bulk_write: Failed to write (%d)\n", ret);
		return ret;
	}

	if (bytes_to_write != bytes_written)
		printk("fx2_bulk_write:  Expected to write %d, written %d\n",
			bytes_to_write, bytes_written);

	return bytes_written;
}



/**
 * fx2_bulk_read - Read from bulk endpoint
 *
 * @fp:  Only needed to get the private data
 * @user_buf:  User buffer returned
 * @count: size of user buffer
 * @ppos:  Not used
 *
 * Returns: size of user buffer or an error code.  Maybe this should be
 *	min(dev_priv->bulk_buffer_size, count)?
 */
static ssize_t fx2_bulk_read(struct file *fp,
		char __user *user_buf,
		size_t count,
		loff_t *ppos)
{
	int ret;
	struct osr_usb_fx2_dev *dev_priv = fp->private_data;


	/* do a blocking bulk read to get data from the device */
	ret = usb_bulk_msg(dev_priv->udev,
		usb_rcvbulkpipe(dev_priv->udev,
			dev_priv->bulk_in_endpoint->bEndpointAddress),
		dev_priv->bulk_buffer,
		min(dev_priv->bulk_buffer_size, count),
		(int *) &count,
		3*HZ);

	if (!ret) {
		if (copy_to_user(user_buf, dev_priv->bulk_buffer, count))
			ret = -EFAULT;
		else
			ret = count;
	}

	return ret;
}



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


	printk("fx2_probe: Calling fx_probe\n");

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

	/* Set up interrupt and bulk endpoints */
	for (i = 0; i < host_interface->desc.bNumEndpoints; i++) {
		struct usb_endpoint_descriptor *endpoint;

		endpoint = &host_interface->endpoint[i].desc;

		if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) ==
			USB_DIR_IN &&
		    (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
			USB_ENDPOINT_XFER_INT) {

			printk("fx2_probe: Added Endpoint Number: %d, "
			       "Interrupt\n",
				endpoint->bEndpointAddress &
				ENDPOINT_NUMBER_MASK);

			if (dev_priv->int_endpoint)
				printk(KERN_ERR "More than one INT endpoint\n");

			dev_priv->int_endpoint = endpoint;
		} else if ((endpoint->bmAttributes &
			    USB_ENDPOINT_XFERTYPE_MASK) ==
				USB_ENDPOINT_XFER_BULK) {

			if (endpoint->bEndpointAddress & USB_DIR_IN) {
				printk("fx2_probe: Added Endpoint Number: %d, "
				       "Bulk In\n",
					endpoint->bEndpointAddress &
					ENDPOINT_NUMBER_MASK);

				dev_priv->bulk_in_endpoint = endpoint;
			} else {
				printk("fx2_probe: Added Endpoint Number: %d, "
				       "Bulk Out\n",
					endpoint->bEndpointAddress &
					ENDPOINT_NUMBER_MASK);

				dev_priv->bulk_out_endpoint = endpoint;
			}
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
		printk(KERN_ERR "Cannot allocate interrupt URB\n");
		ret = -ENOMEM;
		goto error;
	}

	/* Set up buffer to handle input from the board */
	int_end_size = le16_to_cpu(dev_priv->int_endpoint->wMaxPacketSize);
	dev_priv->int_buffer = kmalloc(int_end_size, int_end_size);
	if (!dev_priv->int_buffer) {
		printk(KERN_ERR "Cannot allocate interrupt buffer\n");
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
		printk(KERN_ERR "Failed to submit interrupt URB\n");
		goto error;
	}

	/* Associate dev_priv with intf */
	usb_set_intfdata(intf, dev_priv);

	/* Register the device */
	ret = usb_register_dev(intf, &osr_usb_fx2_class);
	if (ret) {
		printk(KERN_ERR "Failed to register OSR FX2 device\n");
		goto error;
	}

	dev_priv->minor = intf->minor;

	printk("fx2_probe: /dev/osr_usb%d\n",
		dev_priv->minor - FX2_MINOR_BASE);

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


	printk("fx2_disconnect /dev/osr_usb%d\n",
		dev_priv->minor - FX2_MINOR_BASE);

	osr_usb_attrib_cleanup(&intf->dev);

	usb_set_intfdata(intf, NULL);

	usb_deregister_dev(intf, &osr_usb_fx2_class);

	if (dev_priv->int_urb)
		usb_free_urb(dev_priv->int_urb);

	if (dev_priv->int_buffer)
		kfree(dev_priv->int_buffer);

	if (dev_priv->bulk_buffer)
		kfree(dev_priv->bulk_buffer);

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

