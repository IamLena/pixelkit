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

// inits mapping to devices
static DEFINE_IDR(dev_minors_idr);

static const struct usb_device_id acm_ids[] = {
	{ USB_DEVICE(0x2341, 0x0043) },			// arduino uno - idVendor; idProduct 0x0043
	{ }										// terminating
};

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
