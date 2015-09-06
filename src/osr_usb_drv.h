/******************************************************************************
 * Copyright @ 2015 Sinclair Yeh
 *****************************************************************************/

/* Supported Requests */
#define READ_SEG_DISPLAY_REQ    0xD4
#define READ_SWITCHES_REQ       0xD6
#define READ_BAR_DISPLAY_REQ    0xD7
#define SET_BAR_DISPLAY_REQ     0xD8
#define READ_USB_SPEED_MODE_REQ 0xD9
#define SET_SEG_DISPLAY_REQ     0xDB



/**
 * Request type is a bit field
 *  [0..4]
 *    0000 = Device
 *    0001 = Interface
 *    0010 = Endpoint
 *    0011 = Other
 *    All other = reserved
 *  [5..6]
 *    00 = Standard
 *    01 = Class
 *    10 = Vendor
 *    11 = Reserved
 *  [7]
 *    0 = Host to Device
 *    1 = Devie to Host
 *
 * We will be sending/reading vendor-specific requests to the device, thus
 * the types are defined as such below.
 */
#define SEND_DATA_REQ_TYPE      0x40
#define READ_DATA_REQ_TYPE      0xC0


/**
 * osr_usb_fx2_dev:  Device private structure
 */
struct osr_usb_fx2_dev {
	struct usb_device *udev;
	struct usb_class_driver class;

	struct usb_endpoint_descriptor *int_endpoint;
	struct urb                     *int_urb;
	char                           *int_buffer;

	/* protected by isr_lock */
	int        switch_val;
	spinlock_t isr_lock;

	struct delayed_work debounce_work;
};


extern int osr_usb_attrib_init(struct device *dev);
extern void osr_usb_attrib_cleanup(struct device *dev);

