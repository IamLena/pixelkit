#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/usb.h>
#include <linux/uaccess.h>

#define USB_SKEL_MINOR_BASE	192
#define USB_SKEL_VENDOR_ID	0x2341
#define USB_SKEL_PRODUCT_ID	0x0043

static struct usb_device_id skel_table [] = {
	{ USB_DEVICE(USB_SKEL_VENDOR_ID, USB_SKEL_PRODUCT_ID) },
	{ }
};
MODULE_DEVICE_TABLE (usb, skel_table);

struct usb_skel {
	struct usb_device *	udev;
	struct kref		kref;
};

#define to_skel_dev(d) container_of(d, struct usb_skel, kref)

static struct usb_driver skel_driver;
static struct device mydevice;

static void skel_delete(struct kref *kref)
{
	struct usb_skel *dev = to_skel_dev(kref);
	usb_put_dev(dev->udev);
	usb_put_dev(mydevice);
	kfree (dev);
}

static struct file_operations skel_fops = {
	.owner =	THIS_MODULE,
};

static struct usb_class_driver skel_class = {
	.name = "usb/skel%d",
	.fops = &skel_fops,
	.minor_base = USB_SKEL_MINOR_BASE,
};

static int skel_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	printk(KERN_INFO "MY PROBE FUNCTION CALLED\n");
	struct usb_skel *dev = NULL;
	int retval = -ENOMEM;

	dev = kzalloc(sizeof(struct usb_skel), GFP_KERNEL);
	if (!dev) {
		pr_err("Out of memory");
		goto error;
	}
	kref_init(&dev->kref);
	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	mydevice = = usb_get_dev(interface_to_usbdev(interface));
	return 0;
error:
	if (dev)
		kref_put(&dev->kref, skel_delete);
	return retval;
}

static void skel_disconnect(struct usb_interface *interface)
{
	struct usb_skel *dev;
	int minor = interface->minor;

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);
	usb_deregister_dev(interface, &skel_class);
	kref_put(&dev->kref, skel_delete);
	dev_info(&interface->dev, "USB Skeleton #%d now disconnected", minor);
}

static struct usb_driver skel_driver = {
	.name = "skeleton",
	.id_table = skel_table,
	.probe = skel_probe,
	.disconnect = skel_disconnect,
};

static int __init usb_skel_init(void)
{
	printk(KERN_INFO "MY INIT\n");
	int result = usb_register(&skel_driver);
	if (result)
		pr_err("usb_register failed. Error number %d", result);
	return result;
}

static void __exit usb_skel_exit(void)
{
	usb_deregister(&skel_driver);
	printk(KERN_INFO "MY OUT\n");
}

module_init (usb_skel_init);
module_exit (usb_skel_exit);

MODULE_LICENSE("GPL");
