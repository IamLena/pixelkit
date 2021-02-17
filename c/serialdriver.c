// #include <linux/config.h> unknown

#ifdef CONFIG_USB_SERIAL_DEBUG
   static int debug = 1;
   #define DEBUG
#else
   static int debug;
#endif

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/serial/usb-serial.h>

#define MY_PRODUCT_ID	0x2341
#define MY_DEVICE_ID	0x0043

static struct usb_device_id id_table [] = {
   { USB_DEVICE(MY_PRODUCT_ID, MY_DEVICE_ID) },
   { }   /* Terminating entry */
};

/*
 * allow this driver to be automatically loaded
 * for these devices if they are present.
 */
MODULE_DEVICE_TABLE (usb, id_table);

static struct usb_driver tiny_driver = {
   .name =     "tiny",
   .probe = usb_serial_probe,
   .disconnect =  usb_serial_disconnect,
   .id_table = id_table,
};

/* All device info needed for the Tiny device */
static struct usb_serial_device_type tiny_device = {
   .owner =    THIS_MODULE,
   .name =        "Tiny USB serial",
   .short_name =     "tiny",
   .id_table =    id_table,
   .num_interrupt_in =  NUM_DONT_CARE,
   .num_bulk_in =    NUM_DONT_CARE,
   .num_bulk_out =      NUM_DONT_CARE,
   .num_ports =      1,
};

static int __init tiny_init (void)
{
   usb_serial_register (&tiny_device);
   usb_register (&tiny_driver);
   return 0;
}

static void __exit tiny_exit (void)
{
   usb_deregister (&tiny_driver);
   usb_serial_deregister (&tiny_device);
}

module_init(tiny_init);
module_exit(tiny_exit);

MODULE_LICENSE("GPL");
