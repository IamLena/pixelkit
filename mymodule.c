#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#define DRIVER_VERSION "v2.0"
#define DRIVER_AUTHOR "Luchina"
#define DRIVER_DESC "Tiny TTY driver"

/* Module information */
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#define DELAY_TIME		(HZ * 2)	/* 2 seconds per character */
#define TINY_DATA_CHARACTER	't'

#define TINY_TTY_MAJOR		240	/* experimental range */
#define TINY_TTY_MINORS		4	/* only have 4 devices */

static const struct tty_operations serial_ops = {
};
static struct tty_driver *tiny_tty_driver;
static struct tty_port tiny_tty_port[TINY_TTY_MINORS];

struct tiny_serial {
	struct tty_struct	*tty;		/* pointer to the tty for this device */
	int			open_count;	/* number of times this port has been opened */
	struct mutex	mutex;		/* locks this structure */
	struct timer_list	timer;

	/* for tiocmget and tiocmset functions */
	int			msr;		/* MSR shadow */
	int			mcr;		/* MCR shadow */

	/* for ioctl fun */
	struct serial_struct	serial;
	wait_queue_head_t	wait;
	struct async_icount	icount;
};

static struct tiny_serial *tiny_table[TINY_TTY_MINORS];	/* initially all NULL */

static int __init tiny_init(void)
{
	printk(KERN_DEBUG "init called");
	int retval;
	int i;

	/* allocate the tty driver */
	tiny_tty_driver = alloc_tty_driver(TINY_TTY_MINORS);
	if (!tiny_tty_driver)
		return -ENOMEM;

	/* initialize the tty driver */
	tiny_tty_driver->owner = THIS_MODULE;
	tiny_tty_driver->driver_name = "tiny_tty";
	tiny_tty_driver->name = "ttty";
	tiny_tty_driver->major = TINY_TTY_MAJOR,
	tiny_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	tiny_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	tiny_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV,
	tiny_tty_driver->init_termios = tty_std_termios;
	tiny_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_set_operations(tiny_tty_driver, &serial_ops);
	for (i = 0; i < TINY_TTY_MINORS; i++) {
		tty_port_init(tiny_tty_port + i);
		tty_port_link_device(tiny_tty_port + i, tiny_tty_driver, i);
	}

	/* register the tty driver */
	retval = tty_register_driver(tiny_tty_driver);
	if (retval) {
		pr_err("failed to register tiny tty driver");
		put_tty_driver(tiny_tty_driver);
		return retval;
	}

	for (i = 0; i < TINY_TTY_MINORS; ++i)
		tty_register_device(tiny_tty_driver, i, NULL);

	pr_info(DRIVER_DESC " " DRIVER_VERSION);
	return retval;
}

static void do_close(struct tiny_serial *tiny)
{
	mutex_lock(&tiny->mutex);

	if (!tiny->open_count) {
		/* port was never opened */
		goto exit;
	}

	--tiny->open_count;
	if (tiny->open_count <= 0) {
		/* The port is being closed by the last user. */
		/* Do any hardware specific stuff here */

		/* shut down our timer */
		// del_timer(&tiny->timer);
	}
exit:
	mutex_unlock(&tiny->mutex);
}

static void __exit tiny_exit(void)
{
	struct tiny_serial *tiny;
	int i;

	for (i = 0; i < TINY_TTY_MINORS; ++i)
		tty_unregister_device(tiny_tty_driver, i);
	tty_unregister_driver(tiny_tty_driver);

	/* shut down all of the timers and free the memory */
	for (i = 0; i < TINY_TTY_MINORS; ++i) {
		tiny = tiny_table[i];
		if (tiny) {
			/* close the port */
			while (tiny->open_count)
				do_close(tiny);

			/* shut down our timer and free the memory */
			// del_timer(&tiny->timer);
			kfree(tiny);
			tiny_table[i] = NULL;
		}
	}
}

module_init(tiny_init);
module_exit(tiny_exit);
