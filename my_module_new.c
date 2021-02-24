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

static int my_usb_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	printk(KERN_INFO KBUILD_MODNAME ": my_usb_probe called\n");

	// struct usb_cdc_union_desc *union_header = NULL;
	// struct usb_cdc_call_mgmt_descriptor *cmgmd = NULL;
	// unsigned char *buffer = intf->altsetting->extra;
	// int buflen = intf->altsetting->extralen;
	// struct usb_interface *control_interface;
	// struct usb_interface *data_interface;
	// struct usb_endpoint_descriptor *epctrl = NULL;
	// struct usb_endpoint_descriptor *epread = NULL;
	// struct usb_endpoint_descriptor *epwrite = NULL;
	// struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct usb_cdc_parsed_header h;
	// struct acm *acm;
	// int minor;
	// int ctrlsize, readsize;
	// u8 *buf;
	// int call_intf_num = -1;
	// int data_intf_num = -1;
	// int num_rx_buf;
	// int i;
	// int combined_interfaces = 0;
	// struct device *tty_dev;
	// int rv = -ENOMEM;

	memset(&h, 0x00, sizeof(struct usb_cdc_parsed_header));
	num_rx_buf = ACM_NR;

	/* normal probing*/
	if (!buffer) {
		dev_err(&intf->dev, "Weird descriptor references\n");
		return -EINVAL;
	}

	if (!buflen) {
		if (intf->cur_altsetting->endpoint &&
				intf->cur_altsetting->endpoint->extralen &&
				intf->cur_altsetting->endpoint->extra) {
			dev_dbg(&intf->dev,
				"Seeking extra descriptors on endpoint\n");
			buflen = intf->cur_altsetting->endpoint->extralen;
			buffer = intf->cur_altsetting->endpoint->extra;
		} else {
			dev_err(&intf->dev,
				"Zero length descriptor references\n");
			return -EINVAL;
		}
	}

	cdc_parse_cdc_header(&h, intf, buffer, buflen);
	union_header = h.usb_cdc_union_desc;
	cmgmd = h.usb_cdc_call_mgmt_descriptor;
	if (cmgmd)
		call_intf_num = cmgmd->bDataInterface;

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
		dev_dbg(&intf->dev, "no interfaces\n");
		return -ENODEV;
	}

	if (data_intf_num != call_intf_num)
		dev_dbg(&intf->dev, "Separate call control interface. That is not fully supported.\n");

	/*workaround for switched interfaces */
	if (data_interface->cur_altsetting->desc.bInterfaceClass != USB_CLASS_CDC_DATA) {
		if (control_interface->cur_altsetting->desc.bInterfaceClass == USB_CLASS_CDC_DATA) {
			dev_dbg(&intf->dev,
				"Your device has switched interfaces.\n");
			swap(control_interface, data_interface);
		} else {
			return -EINVAL;
		}
	}

	/* Accept probe requests only for the control interface */
	if (!combined_interfaces && intf != control_interface)
		return -ENODEV;

	if (!combined_interfaces && usb_interface_claimed(data_interface)) {
		/* valid in this context */
		dev_dbg(&intf->dev, "The data interface isn't available\n");
		return -EBUSY;
	}


	if (data_interface->cur_altsetting->desc.bNumEndpoints < 2 ||
	    control_interface->cur_altsetting->desc.bNumEndpoints == 0)
		return -EINVAL;

	epctrl = &control_interface->cur_altsetting->endpoint[0].desc;
	epread = &data_interface->cur_altsetting->endpoint[0].desc;
	epwrite = &data_interface->cur_altsetting->endpoint[1].desc;


	/* workaround for switched endpoints */
	if (!usb_endpoint_dir_in(epread)) {
		/* descriptors are swapped */
		dev_dbg(&intf->dev,
			"The data interface has switched endpoints\n");
		swap(epread, epwrite);
	}

// made_compressed_probe:
	dev_dbg(&intf->dev, "interfaces are valid\n");

	acm = kzalloc(sizeof(struct acm), GFP_KERNEL);
	if (acm == NULL)
		goto alloc_fail;

	tty_port_init(&acm->port);
	acm->port.ops = &acm_port_ops;

	ctrlsize = usb_endpoint_maxp(epctrl);
	readsize = usb_endpoint_maxp(epread) * 2;
	acm->combined_interfaces = combined_interfaces;
	acm->writesize = usb_endpoint_maxp(epwrite) * 20;
	acm->control = control_interface;
	acm->data = data_interface;

	usb_get_intf(acm->control); /* undone in destruct() */

	minor = acm_alloc_minor(acm);
	if (minor < 0)
		goto alloc_fail1;

	acm->minor = minor;
	acm->dev = usb_dev;
	if (h.usb_cdc_acm_descriptor)
		acm->ctrl_caps = h.usb_cdc_acm_descriptor->bmCapabilities;
	acm->ctrlsize = ctrlsize;
	acm->readsize = readsize;
	acm->rx_buflimit = num_rx_buf;
	INIT_DELAYED_WORK(&acm->dwork, acm_softint);
	init_waitqueue_head(&acm->wioctl);
	spin_lock_init(&acm->write_lock);
	spin_lock_init(&acm->read_lock);
	mutex_init(&acm->mutex);
	if (usb_endpoint_xfer_int(epread)) {
		acm->bInterval = epread->bInterval;
		acm->in = usb_rcvintpipe(usb_dev, epread->bEndpointAddress);
	} else {
		acm->in = usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress);
	}
	if (usb_endpoint_xfer_int(epwrite))
		acm->out = usb_sndintpipe(usb_dev, epwrite->bEndpointAddress);
	else
		acm->out = usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress);
	init_usb_anchor(&acm->delayed);

	buf = usb_alloc_coherent(usb_dev, ctrlsize, GFP_KERNEL, &acm->ctrl_dma);
	if (!buf)
		goto alloc_fail1;
	acm->ctrl_buffer = buf;

	if (acm_write_buffers_alloc(acm) < 0)
		goto alloc_fail2;

	acm->ctrlurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!acm->ctrlurb)
		goto alloc_fail3;

	for (i = 0; i < num_rx_buf; i++) {
		struct acm_rb *rb = &(acm->read_buffers[i]);
		struct urb *urb;

		rb->base = usb_alloc_coherent(acm->dev, readsize, GFP_KERNEL,
								&rb->dma);
		if (!rb->base)
			goto alloc_fail4;
		rb->index = i;
		rb->instance = acm;

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb)
			goto alloc_fail4;

		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = rb->dma;
		if (usb_endpoint_xfer_int(epread))
			usb_fill_int_urb(urb, acm->dev, acm->in, rb->base,
					 acm->readsize,
					 acm_read_bulk_callback, rb,
					 acm->bInterval);
		else
			usb_fill_bulk_urb(urb, acm->dev, acm->in, rb->base,
					  acm->readsize,
					  acm_read_bulk_callback, rb);

		acm->read_urbs[i] = urb;
		__set_bit(i, &acm->read_urbs_free);
	}
	for (i = 0; i < ACM_NW; i++) {
		struct acm_wb *snd = &(acm->wb[i]);

		snd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (snd->urb == NULL)
			goto alloc_fail5;

		if (usb_endpoint_xfer_int(epwrite))
			usb_fill_int_urb(snd->urb, usb_dev, acm->out,
				NULL, acm->writesize, acm_write_bulk, snd, epwrite->bInterval);
		else
			usb_fill_bulk_urb(snd->urb, usb_dev, acm->out,
				NULL, acm->writesize, acm_write_bulk, snd);
		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		snd->instance = acm;
	}

	usb_set_intfdata(intf, acm);

	i = device_create_file(&intf->dev, &dev_attr_bmCapabilities);
	if (i < 0)
		goto alloc_fail5;

	if (h.usb_cdc_country_functional_desc) { /* export the country data */
		struct usb_cdc_country_functional_desc * cfd =
					h.usb_cdc_country_functional_desc;

		acm->country_codes = kmalloc(cfd->bLength - 4, GFP_KERNEL);
		if (!acm->country_codes)
			goto skip_countries;
		acm->country_code_size = cfd->bLength - 4;
		memcpy(acm->country_codes, (u8 *)&cfd->wCountyCode0,
							cfd->bLength - 4);
		acm->country_rel_date = cfd->iCountryCodeRelDate;

		i = device_create_file(&intf->dev, &dev_attr_wCountryCodes);
		if (i < 0) {
			kfree(acm->country_codes);
			acm->country_codes = NULL;
			acm->country_code_size = 0;
			goto skip_countries;
		}

		i = device_create_file(&intf->dev,
						&dev_attr_iCountryCodeRelDate);
		if (i < 0) {
			device_remove_file(&intf->dev, &dev_attr_wCountryCodes);
			kfree(acm->country_codes);
			acm->country_codes = NULL;
			acm->country_code_size = 0;
			goto skip_countries;
		}
	}

skip_countries:
	usb_fill_int_urb(acm->ctrlurb, usb_dev,
			 usb_rcvintpipe(usb_dev, epctrl->bEndpointAddress),
			 acm->ctrl_buffer, ctrlsize, acm_ctrl_irq, acm,
			 /* works around buggy devices */
			 epctrl->bInterval ? epctrl->bInterval : 16);
	acm->ctrlurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	acm->ctrlurb->transfer_dma = acm->ctrl_dma;
	acm->notification_buffer = NULL;
	acm->nb_index = 0;
	acm->nb_size = 0;

	dev_info(&intf->dev, "ttyACM%d: USB ACM device\n", minor);

	acm->line.dwDTERate = cpu_to_le32(9600);
	acm->line.bDataBits = 8;
	acm_set_line(acm, &acm->line);

	usb_driver_claim_interface(&acm_driver, data_interface, acm);
	usb_set_intfdata(data_interface, acm);

	tty_dev = tty_port_register_device(&acm->port, acm_tty_driver, minor,
			&control_interface->dev);
	if (IS_ERR(tty_dev)) {
		rv = PTR_ERR(tty_dev);
		goto alloc_fail6;
	}
	return 0;

alloc_fail6:
	if (acm->country_codes) {
		device_remove_file(&acm->control->dev,
				&dev_attr_wCountryCodes);
		device_remove_file(&acm->control->dev,
				&dev_attr_iCountryCodeRelDate);
		kfree(acm->country_codes);
	}
	device_remove_file(&acm->control->dev, &dev_attr_bmCapabilities);
alloc_fail5:
	usb_set_intfdata(intf, NULL);
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(acm->wb[i].urb);
alloc_fail4:
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(acm->read_urbs[i]);
	acm_read_buffers_free(acm);
	usb_free_urb(acm->ctrlurb);
alloc_fail3:
	acm_write_buffers_free(acm);
alloc_fail2:
	usb_free_coherent(usb_dev, ctrlsize, acm->ctrl_buffer, acm->ctrl_dma);
alloc_fail1:
	tty_port_put(&acm->port);
alloc_fail:
	return rv;
}

static struct usb_driver my_usb_driver = {
	.name =		"my_usb_driver",
	.probe =	my_usb_probe,			// called when device is detected
	.disconnect =	my_usb_disconnect,	// called when device is disconnected or the driver is uninstalled
	.id_table =	device_ids,				// table of device to contor by this driver
// #ifdef CONFIG_PM
// 	.suspend =	acm_suspend,
// 	.resume =	acm_resume,
// 	.reset_resume =	acm_reset_resume,
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
	my_tty_driver->major = ACM_TTY_MAJOR;
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
