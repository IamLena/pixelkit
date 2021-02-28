#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/idr.h>


#define DRIVER_AUTHOR "Luchina Elena IU7 BMSTU"
#define DRIVER_DESC "driver for ardruino uno device to print messages"

#define MY_TTY_MAJOR		240		// major number identified with the driver
#define MY_TTY_MINORS		1		//minor numbers range for devices

static struct usb_driver my_usb_driver;
static struct tty_driver *my_tty_driver;

static DEFINE_IDR(dev_minors_idr);			// inits mapping to devices

// array of struct usb_device_id required by usb_driver
static const struct usb_device_id device_ids[] = {
	{ USB_DEVICE(0x2341, 0x0043) },			// arduino uno - idVendor; idProduct 0x0043
	{ }										// terminating
};


struct ard {
	struct usb_device *dev;				/* the corresponding usb device */
	struct usb_interface *control;		/* control interface */
	struct usb_interface *data;			/* data interface */
	unsigned in, out;					/* i/o pipes */
	struct tty_port port;			 	/* our tty port data */
	struct urb *ctrlurb;				/* urbs */
	u8 *ctrl_buffer;					/* buffers of urbs */
	dma_addr_t ctrl_dma;				/* dma handles of buffers */
	u8 *country_codes;					/* country codes from device */
	unsigned int country_code_size;		/* size of this buffer */
	unsigned int country_rel_date;		/* release date of version */
	struct ard_wb wb[ARD_NW];
	unsigned long read_urbs_free;
	struct urb *read_urbs[ARD_NR];
	struct ard_rb read_buffers[ARD_NR];
	int rx_buflimit;
	spinlock_t read_lock;
	u8 *notification_buffer;			/* to reassemble fragmented notifications */
	unsigned int nb_index;
	unsigned int nb_size;
	int transmitting;
	spinlock_t write_lock;
	struct mutex mutex;
	bool disconnected;
	unsigned long flags;
#		define EVENT_TTY_WAKEUP	0
#		define EVENT_RX_STALL	1
#		define ARD_THROTTLED	2
#		define ARD_ERROR_DELAY	3
	unsigned long urbs_in_error_delay;	/* these need to be restarted after a delay */
	struct usb_cdc_line_coding line;	/* bits, stop, parity */
	struct delayed_work dwork;			/* work queue entry for various purposes */
	unsigned int ctrlin;				/* input control lines (DCD, DSR, RI, break, overruns) */
	unsigned int ctrlout;				/* output control lines (DTR, RTS) */
	struct async_icount iocount;		/* counters for control line changes */
	struct async_icount oldcount;		/* for comparison of counter */
	wait_queue_head_t wioctl;			/* for ioctl */
	unsigned int writesize;				/* max packet size for the output bulk endpoint */
	unsigned int readsize,ctrlsize;		/* buffer sizes for freeing */
	unsigned int minor;					/* ard minor number */
	unsigned char clocal;				/* termios CLOCAL */
	unsigned int ctrl_caps;				/* control capabilities from the class specific header */
	unsigned int susp_count;			/* number of suspended interfaces */
	unsigned int combined_interfaces:1;	/* control and data collapsed */
	u8 bInterval;
	struct usb_anchor delayed;			/* writes queued for a device about to be woken */
};

static int my_usb_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct usb_cdc_parsed_header h;
	struct usb_cdc_union_desc *union_header;
	struct usb_cdc_call_mgmt_descriptor *cmgmd;
	int call_intf_num = -1;
	int data_intf_num = -1;
	struct usb_endpoint_descriptor *epctrl = NULL;
	struct usb_endpoint_descriptor *epread = NULL;
	struct usb_endpoint_descriptor *epwrite = NULL;
	int minor;
	u8 *buf;

	printk(KERN_INFO KBUILD_MODNAME ": my_usb_probe called\n");

	//check pointe to descriptors following this endpoint in the configuration
	if (!intf->altsetting->extra) {
		dev_err(&intf->dev, "Weird descriptor references\n");
		return -EINVAL;
	}

	// check length
	if (!intf->altsetting->extralen) {
		if (!(intf->cur_altsetting->endpoint &&
				intf->cur_altsetting->endpoint->extralen &&
				intf->cur_altsetting->endpoint->extra)) {
			dev_err(&intf->dev, "Zero length descriptor references\n");
			return -EINVAL;
	}

	// parse as communications device class
	cdc_parse_cdc_header(&h, intf, intf->cur_altsetting->endpoint->extra, intf->cur_altsetting->endpoint->extralen);
	union_header = h.usb_cdc_union_desc;
	cmgmd = h.usb_cdc_call_mgmt_descriptor;
	if (cmgmd)
		call_intf_num = cmgmd->bDataInterface;

	//set controll and data interfaces
	if (!union_header) {
		if (call_intf_num > 0) {
			dev_dbg(&intf->dev, "No union descriptor, using call management descriptor\n");
			data_intf_num = call_intf_num;
			data_interface = usb_ifnum_to_if(usb_dev, data_intf_num);
			control_interface = intf;
		} else {
			dev_dbg(&intf->dev, "No union descriptor, giving up\n");
			return -ENODEV;
		}
	} else {
		int class = -1;

		data_intf_num = union_header->bSlaveInterface0;
		control_interface = usb_ifnum_to_if(usb_dev, union_header->bMasterInterface0);
		data_interface = usb_ifnum_to_if(usb_dev, data_intf_num);

		if (control_interface)
			class = control_interface->cur_altsetting->desc.bInterfaceClass;
	}

	if (!control_interface || !data_interface) {
		dev_dbg(&intf->dev, "no interfaces founds\n");
		return -ENODEV;
	}

	// Accept probe requests only for the control interface
	if (intf != control_interface)
		return -ENODEV;

	// check if busy
	if (usb_interface_claimed(data_interface)) {
		dev_dbg(&intf->dev, "The data interface isn't available\n");
		return -EBUSY;
	}

	// set endpoints' descriptors
	if (data_interface->cur_altsetting->desc.bNumEndpoints < 2 ||
	    control_interface->cur_altsetting->desc.bNumEndpoints == 0)
		return -EINVAL;
	epctrl = &control_interface->cur_altsetting->endpoint[0].desc;
	epread = &data_interface->cur_altsetting->endpoint[0].desc;
	epwrite = &data_interface->cur_altsetting->endpoint[1].desc;

	// allocate struct ard
	ard = kzalloc(sizeof(struct ard), GFP_KERNEL);
	if (!ard)
		return -ENOMEM;

	// initiate tty-port
	tty_port_init(&ard->port);
	ard->port.ops = &ard_port_ops;
	ard->control = control_interface;
	ard->data = data_interface;

	// allocate minor
	minor = ard_alloc_minor(ard);
	if (minor < 0)
	{
		tty_port_put(&ard->port);
		return -ENOMEM;
	}
	ard->minor = minor;
	ard->dev = usb_dev;
	INIT_DELAYED_WORK(&ard->dwork, ard_softint);
	init_waitqueue_head(&ard->wioctl);
	spin_lock_init(&ard->write_lock);
	spin_lock_init(&ard->read_lock);
	mutex_init(&ard->mutex);

	// set endpoints as a ard struct memmbers
	if (usb_endpoint_xfer_int(epread)) {
		ard->bInterval = epread->bInterval;
		ard->in = usb_rcvintpipe(usb_dev, epread->bEndpointAddress);
	} else {
		ard->in = usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress);
	}
	if (usb_endpoint_xfer_int(epwrite))
		ard->out = usb_sndintpipe(usb_dev, epwrite->bEndpointAddress);
	else
		ard->out = usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress);
	init_usb_anchor(&ard->delayed);

	// allocate dma-consistent buffer for URB
	// DMA is the hardware mechanism that allows peripheral components to transfer their I/O data directly to and from main memory without the need to involve the system processor.
	buf = usb_alloc_coherent(usb_dev, ctrlsize, GFP_KERNEL, &ard->ctrl_dma);
	if (!buf)
		goto alloc_fail1;
	ard->ctrl_buffer = buf;
	ard->ctrlurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ard->ctrlurb)
		goto alloc_fail3;

	if (ard_write_buffers_alloc(ard) < 0)
		goto alloc_fail2;
}

static struct usb_driver my_usb_driver = {
	.name =		"my_usb_driver",
	.probe =	my_usb_probe,			// called when device is detected
	.disconnect =	my_usb_disconnect,	// called when device is disconnected or the driver is uninstalled
	.id_table =	device_ids,				// table of device to contor by this driver
// #ifdef CONFIG_PM
// 	.suspend =	ard_suspend,
// 	.resume =	ard_resume,
// 	.reset_resume =	ard_reset_resume,
// 	.supports_autosuspend = 1,
// #endif
};

static const struct tty_operations tty_ops = {
	.install =		my_tty_install,			// install a new tty into the tty driver internal tables
	.open =			my_tty_open,				// is called when a particular tty device is opened
	.close =		my_tty_close,				// is called when a particular tty device is closed
	.cleanup =		my_tty_cleanup,			// is called asynchronously when a particular tty device
												// is closed for the last time freeing up the resources.
	.write =		my_tty_write,				// is called to write a series of chars to the tty device
	.ioctl =		my_tty_ioctl,				// allows to implement device-specific ioctl's
	.set_termios =	my_tty_set_termios,		// notifies the driver tha termios settings have changed
};

static int __init my_module_init(void)	//funtion to init module
{
	int retval;

	printk(KERN_INFO KBUILD_MODNAME ": start to init the module\n");

	//allocate tty_driver
	my_tty_driver = alloc_tty_driver(MY_TTY_MINORS);
	if (!my_tty_driver)
		return -ENOMEM; //Cannot allocate memory

	// init my_tty_driver struct fields
	my_tty_driver->driver_name = "ard";
	my_tty_driver->name = "ttyARD";
	my_tty_driver->major = ARD_TTY_MAJOR;
	my_tty_driver->minor_start = 0;
	my_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	my_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	my_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;

	// init termios struct
	memset(&(my_tty_driver->init_termios), 0, sizeof(struct termios));
	my_tty_driver->init_termios.c_cflag = CREAD | CLOCAL | CS8 & ~CSIZE & ~CSTOB
	my_tty_driver->init_termios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	my_tty_driver->init_termios.c_iflag &= ~(INPCK);
	my_tty_driver->init_termios.c_oflag &= ~OPOST;
	//  .c_cc = INIT_C_CC ??

	tty_set_operations(my_tty_driver, &tty_ops);

	// register driver with the tty core
	retval = tty_register_driver(my_tty_driver);
	if (retval) {
		// cleans up a tty_driver structure that has not been
		// successfully registered with the tty core
		put_tty_driver(my_tty_driver);
		return retval;
	}

	// register usb driver
	retval = usb_register(&my_usb_driver);
	if (retval) {
		// in failure unregister tty_driver, clean up tty_driver struct
		tty_unregister_driver(my_tty_driver);
		put_tty_driver(my_tty_driver);
		return retval;
	}

	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_DESC "\n");
	return 0;
}

static void __exit my_module_exit(void)
{
	usb_deregister(&my_usb_driver);				// deregister usb_driver
	tty_unregister_driver(my_tty_driver);		// unregister tty_driver
	put_tty_driver(my_tty_driver);				// clean up tty_driver struct
	idr_destroy(&dev_minors_idr);				// free all IDs mapping to pointers
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(MY_TTY_MAJOR);

module_init(my_module_init);
module_exit(my_module_exit);
