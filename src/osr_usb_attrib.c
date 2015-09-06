/******************************************************************************
 * Copyright @ 2015 Sinclair Yeh
 *****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/slab.h>

#include "osr_usb_drv.h"


/**
 * Constants
 */
#define BITS_PER_BYTE  8

/* LED bargraph is wired with bits shifted 3 positions from the usual places */
#define BARGRAPH_SHIFT 3



/**
 * Attributes
 */
static ssize_t write_bargraph_attr(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t read_bargraph_attr(struct device *dev,
		struct device_attribute *attr, char *buf);
static DEVICE_ATTR(bargraph, S_IRUGO | S_IWUGO,
		read_bargraph_attr, write_bargraph_attr);

static ssize_t write_7seg_attr(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t read_7seg_attr(struct device *dev,
		struct device_attribute *attr, char *buf);
static DEVICE_ATTR(7seg, S_IRUGO | S_IWUGO,
		read_7seg_attr, write_7seg_attr);

static ssize_t read_speed_attr(struct device *dev,
		struct device_attribute *attr, char *buf);
static DEVICE_ATTR(speed, S_IRUGO, read_speed_attr, NULL);

static ssize_t read_toggle_attr(struct device *dev,
		struct device_attribute *attr, char *buf);
static DEVICE_ATTR(toggle, S_IRUGO, read_toggle_attr, NULL);



/**
 * rotr: Circular rotate bits "shift" position to the right
 *
 * @value: value to be shifted
 * @shift: number of positions ot shift
 *
 * Return: returns "value" if shift is >= 8 bits.  Otherwise returns
 *         "value" circular rotated by "shift" position.
 */
static unsigned char rotr(unsigned char value, int shift)
{
	if (shift >= BITS_PER_BYTE)
		return value;

	return (value >> shift) | (value << (BITS_PER_BYTE - shift));
}



/**
 * rotl: Circular rotate bits "shift" position to the left
 *
 * @value: value to be shifted
 * @shift: number of positions ot shift
 *
 * Return: returns "value" if shift is >= 8 bits.  Otherwise returns
 *         "value" circular rotated by "shift" position.
 */
static unsigned char rotl(unsigned char value, int shift)
{
	if (shift >= BITS_PER_BYTE)
		return value;

	return (value << shift) | (value >> (BITS_PER_BYTE - shift));
}



/**
 * read_7seg_attr: outputs the 7 segment display value
 *
 * @dev: Device handle
 * @attr: Not used
 * @buf:  PAGE_SIZE buffer used to return a string
 *
 * Return: number of bytes used in buf
 */
static ssize_t read_7seg_attr(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	int ret;
	struct usb_interface *intf = to_usb_interface(dev);
	struct osr_usb_fx2_dev *dev_priv = usb_get_intfdata(intf);
	unsigned int value = 0;
	unsigned char *cmd_buf;

	cmd_buf = kmalloc(8, GFP_KERNEL);
	if (!cmd_buf) {
		printk(KERN_ERR "read_7seg_attr: Out of memory\n");
		return 0;
	}

	*cmd_buf = 0;

	ret = usb_control_msg(dev_priv->udev,
		usb_rcvctrlpipe(dev_priv->udev, 0),
		READ_SEG_DISPLAY_REQ,
		READ_DATA_REQ_TYPE,
		0, /* Value */
		0, /* Index */
		cmd_buf,
		8,
		2*HZ);

	value = *cmd_buf;

	kfree(cmd_buf);

	if (0 > ret) {
		printk(KERN_ERR "read_7seg_attr: usb_control_msg() "
			"returned %d\n", ret);
		return sprintf(buf, "Error\n");
	} else {
		return sprintf(buf, "0x%x\n", value);
	}
}



/**
 * write_7seg_attr: Writes to 7 segment display
 *
 * The 7 segment (plus a dot) are turned configured as follows
 *
 *	0x01
 *  0x40    0x02
 *	0x20
 *  0x10    0x04
 *	0x80
 *
 *  dot: 0x08
 *
 * @dev: handle to device
 * @attr: not used
 * @buf: input from the user
 * @count: size of buf
 *
 * Return:
 *   Number of bytes in buf used by this operation, which is "count" in our case
 *   because we use the entire buffer to get "value".
 */
static ssize_t write_7seg_attr(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct osr_usb_fx2_dev *dev_priv = usb_get_intfdata(intf);
	int ret;
	unsigned int value = simple_strtoul(buf, NULL, 10);
	unsigned char *cmd_buf;


	if (value > 255) {
		printk(KERN_ERR "write_7seg_attr: Invalid value\n");
		return count;
	}

	cmd_buf = kmalloc(1, GFP_KERNEL);
	if (!cmd_buf) {
		printk(KERN_ERR "write_bargraph_attr: Out of memory\n");
		return count;
	}

	*cmd_buf = (u8) (value & 0xFF);

	ret = usb_control_msg(dev_priv->udev,
		usb_sndctrlpipe(dev_priv->udev, 0),
		SET_SEG_DISPLAY_REQ,
		SEND_DATA_REQ_TYPE,
		0, /* Value */
		0, /* Index */
		cmd_buf,
		1,
		2*HZ);

	if (0 > ret) {
		printk(KERN_ERR "write_7seg_attr: usb_control_msg() "
			"returned %d\n", ret);
		return count;
	}

	printk("Writing: %d\n", value);

	kfree(cmd_buf);

	return count;
}



/**
 * read_toggle_attr: outputs the current toggle switch setting
 *
 * @dev: handle to device
 * @attr: Not used
 * @buf: PAGE_SIZE buffer used to return a string
 *
 * Return: number of bytes used in buf
 */
static ssize_t read_toggle_attr(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int ret;
	struct usb_interface *intf = to_usb_interface(dev);
	struct osr_usb_fx2_dev *dev_priv = usb_get_intfdata(intf);
	unsigned int value = 0;
	unsigned char *cmd_buf;

	cmd_buf = kmalloc(8, GFP_KERNEL);
	if (!cmd_buf) {
		printk(KERN_ERR "read_speed_attr: Out of memory\n");
		return 0;
	}

	*cmd_buf = 0;

	ret = usb_control_msg(dev_priv->udev,
		usb_rcvctrlpipe(dev_priv->udev, 0),
		READ_SWITCHES_REQ,
		READ_DATA_REQ_TYPE,
		0, /* Value */
		0, /* Index */
		cmd_buf,
		8,
		2*HZ);

	value = *cmd_buf;

	kfree(cmd_buf);

	if (0 > ret) {
		printk(KERN_ERR "read_toggle_attr: usb_control_msg() "
			"returned %d\n", ret);
		return sprintf(buf, "Error\n");
	} else {
		return sprintf(buf, "0x%x\n", value);
	}
}



/**
 * read_speed_attr: outputs the current speed setting (HIGH/FULL)
 *
 * @dev: handle to device
 * @attr: Not used
 * @buf: PAGE_SIZE buffer used to return a string
 *
 * Return: number of bytes used in buf
 */
static ssize_t read_speed_attr(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	int ret;
	struct usb_interface *intf = to_usb_interface(dev);
	struct osr_usb_fx2_dev *dev_priv = usb_get_intfdata(intf);
	unsigned int value = 0;
	unsigned char *cmd_buf;

	cmd_buf = kmalloc(8, GFP_KERNEL);
	if (!cmd_buf) {
		printk(KERN_ERR "read_speed_attr: Out of memory\n");
		return 0;
	}

	*cmd_buf = 0;

	ret = usb_control_msg(dev_priv->udev,
		usb_rcvctrlpipe(dev_priv->udev, 0),
		READ_USB_SPEED_MODE_REQ,
		READ_DATA_REQ_TYPE,
		0, /* Value */
		0, /* Index */
		cmd_buf,
		8,
		2*HZ);

	value = *cmd_buf;

	kfree(cmd_buf);

	/* "High" if device replies "1", "FULL" if "0".  "Error" otherwise. */
	if (0 > ret) {
		printk(KERN_ERR "read_speed_attr: usb_control_msg() "
			"returned %d\n", ret);
		return sprintf(buf, "Error\n");
	} else {
		return sprintf(buf, "%s\n", value ? "High" : "Full");
	}
}



/**
 * read_bargraph_attr: Prints the value of the bargraph
 *
 * This function will compensate for bargraph's bit shifted wiring.
 *
 * @buf: PAGE_SIZE buffer used to return a string
 *
 * Return: number of bytes used in buf
 */
static ssize_t read_bargraph_attr(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	int ret;
	struct usb_interface *intf = to_usb_interface(dev);
	struct osr_usb_fx2_dev *dev_priv = usb_get_intfdata(intf);
	unsigned int value = 0;
	unsigned char *cmd_buf;

	cmd_buf = kmalloc(8, GFP_KERNEL);
	if (!cmd_buf) {
		printk(KERN_ERR "read_bargraph_attr: Out of memory\n");
		return 0;
	}

	*cmd_buf = 0;

	ret = usb_control_msg(dev_priv->udev,
		usb_rcvctrlpipe(dev_priv->udev, 0),
		READ_BAR_DISPLAY_REQ,
		READ_DATA_REQ_TYPE,
		0, /* Value */
		0, /* Index */
		cmd_buf,
		8,
		2*HZ);

	/* Compensate for the way the LED Bargraph is wired */
	value = rotl(*cmd_buf, BARGRAPH_SHIFT);

	kfree(cmd_buf);

	if (0 > ret) {
		printk(KERN_ERR "read_bargraph_attr: usb_control_msg() "
			"returned %d\n", ret);
	}

	return sprintf(buf, "%d\n", value);
}



/**
 * write_bargraph_attr: Writes to LED bargraph
 *
 * Accepts one byte where each bit position represents one bar in the bargraph.
 *
 * @dev: handle to device
 * @attr: not used
 * @buf: input from the user
 * @count: size of buf
 *
 * Return:
 *   Number of bytes in buf used by this operation, which is "count" in our case
 *   because we use the entire buffer to get "value".
 */
static ssize_t write_bargraph_attr(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct osr_usb_fx2_dev *dev_priv = usb_get_intfdata(intf);
	int ret;
	unsigned int value = simple_strtoul(buf, NULL, 10);
	unsigned char *cmd_buf;


	if (value > 255) {
		printk(KERN_ERR "write_graph_attr: Invalid value\n");
		return count;
	}

	cmd_buf = kmalloc(1, GFP_KERNEL);
	if (!cmd_buf) {
		printk(KERN_ERR "write_bargraph_attr: Out of memory\n");
		return count;
	}

	/* Compensate for the way the LED Bargraph is wired */
	*cmd_buf = rotr((u8) (value & 0xFF), BARGRAPH_SHIFT);

	ret = usb_control_msg(dev_priv->udev,
		usb_sndctrlpipe(dev_priv->udev, 0),
		SET_BAR_DISPLAY_REQ,
		SEND_DATA_REQ_TYPE,
		0, /* Value */
		0, /* Index */
		cmd_buf,
		1,
		2*HZ);

	if (0 > ret) {
		printk(KERN_ERR "write_graph_attr: usb_control_msg() "
			"returned %d\n", ret);
		return count;
	}

	printk("Writing: %d\n", value);

	kfree(cmd_buf);

	return count;
}



/**
 * osr_usb_attrib_init
 *
 * Creates the attributes' sysfs entries
 *
 * @dev: Device handle
 *
 * Return: 0 on success
 */
int osr_usb_attrib_init(struct device *dev)
{

	/* Initialize attributes */
	if (device_create_file(dev, &dev_attr_bargraph)) {
		printk(KERN_ERR "Cannot create sysfs handle for"
			" bargraph attribute\n");
		return -EINVAL;
	}

	if (device_create_file(dev, &dev_attr_7seg)) {
		printk(KERN_ERR "Cannot create sysfs handle for"
			" 7 segment display attribute\n");
		return -EINVAL;
	}

	if (device_create_file(dev, &dev_attr_speed)) {
		printk(KERN_ERR "Cannot create sysfs handle for"
			" speed attribute\n");
		return -EINVAL;
	}

	if (device_create_file(dev, &dev_attr_toggle)) {
		printk(KERN_ERR "Cannot create sysfs handle for"
			" toggle attribute\n");
		return -EINVAL;
	}

	return 0;
}



/**
 * osr_usb_attrib_cleanup
 *
 * Undo things done in osr_usb_attrib_init, which is removing attributes'
 * sysfs entries
 *
 * @dev: Device handle
 */
void osr_usb_attrib_cleanup(struct device *dev)
{
	device_remove_file(dev, &dev_attr_bargraph);
	device_remove_file(dev, &dev_attr_speed);
	device_remove_file(dev, &dev_attr_7seg);
	device_remove_file(dev, &dev_attr_toggle);
}

