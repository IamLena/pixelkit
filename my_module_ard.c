#include <linux/kernel.h>
#include <linux/sched/signal.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/log2.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <linux/idr.h>
#include <linux/list.h>
#include <linux/string.h>

#ifndef CMSPAR
# define CMSPAR			0
#endif

#define ARD_TTY_MAJOR		240
#define ARD_TTY_MINORS		1

// Requests.
#define USB_RT_ARD		(USB_TYPE_CLASS | USB_RECIP_INTERFACE)

// Output control lines.
#define ARD_CTRL_DTR		0x01
#define ARD_CTRL_RTS		0x02

// Input control lines and line errors.
#define ARD_CTRL_DCD		0x01
#define ARD_CTRL_DSR		0x02
#define ARD_CTRL_BRK		0x04
#define ARD_CTRL_RI			0x08
#define ARD_CTRL_FRAMING	0x10
#define ARD_CTRL_PARITY		0x20
#define ARD_CTRL_OVERRUN	0x40

#define ARD_NW  16
#define ARD_NR  16

#define DRIVER_AUTHOR "Luchina"
#define DRIVER_DESC "driver for ardruino uno device to print messages"

/*
 * Internal driver structures.
 */

struct ard_wb {
	u8 *buf;
	dma_addr_t dmah;
	unsigned int len;
	struct urb		*urb;
	struct ard		*instance;
	bool use;
};

struct ard_rb {
	int			size;
	unsigned char		*base;
	dma_addr_t		dma;
	int			index;
	struct ard		*instance;
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

static struct usb_driver ard_driver;
static struct tty_driver *ard_tty_driver;

static DEFINE_IDR(ard_minors);
static DEFINE_MUTEX(ard_minors_lock);

static void ard_tty_set_termios(struct tty_struct *tty, struct ktermios *termios_old);

/*
 * ard_minors accessors
 */

/*
 * Look up an ARD structure by minor. If found and not disconnected, increment
 * its refcount and return it with its mutex held.
 */
static struct ard *ard_get_by_minor(unsigned int minor)
{
	struct ard *ard;
	printk(KERN_INFO KBUILD_MODNAME ": ard_get_by_minor called\n");

	mutex_lock(&ard_minors_lock);
	ard = idr_find(&ard_minors, minor);
	if (ard) {
		mutex_lock(&ard->mutex);
		if (ard->disconnected) {
			mutex_unlock(&ard->mutex);
			ard = NULL;
		} else {
			tty_port_get(&ard->port);
			mutex_unlock(&ard->mutex);
		}
	}
	mutex_unlock(&ard_minors_lock);
	return ard;
}

/*
 * Try to find an available minor number and if found, associate it with 'ard'.
 */
static int ard_alloc_minor(struct ard *ard)
{
	int minor;
	printk(KERN_INFO KBUILD_MODNAME ": ard_alloc_minor called\n");

	mutex_lock(&ard_minors_lock);
	minor = idr_alloc(&ard_minors, ard, 0, ARD_TTY_MINORS, GFP_KERNEL);
	mutex_unlock(&ard_minors_lock);

	return minor;
}

/* Release the minor number associated with 'ard'.  */
static void ard_release_minor(struct ard *ard)
{
	printk(KERN_INFO KBUILD_MODNAME ": ard_release_minor called\n");
	mutex_lock(&ard_minors_lock);
	idr_remove(&ard_minors, ard->minor);
	mutex_unlock(&ard_minors_lock);
}

/*
 * Functions for ARD control messages.
 */

static int ard_ctrl_msg(struct ard *ard, int request, int value,
							void *buf, int len)
{
	int retval;
	printk(KERN_INFO KBUILD_MODNAME ": ard_ctrl_msg called\n");

	retval = usb_autopm_get_interface(ard->control);
	if (retval)
		return retval;

	retval = usb_control_msg(ard->dev, usb_sndctrlpipe(ard->dev, 0),
		request, USB_RT_ARD, value,
		ard->control->altsetting[0].desc.bInterfaceNumber,
		buf, len, 5000);

	dev_dbg(&ard->control->dev,
		"%s - rq 0x%02x, val %#x, len %#x, result %d\n",
		__func__, request, value, len, retval);

	usb_autopm_put_interface(ard->control);

	return retval < 0 ? retval : 0;
}

/* devices aren't required to support these requests.
 * the cdc ard descriptor tells whether they do...
 */
static inline int ard_set_control(struct ard *ard, int control)
{
	printk(KERN_INFO KBUILD_MODNAME ": ard_set_control called\n");
	return ard_ctrl_msg(ard, USB_CDC_REQ_SET_CONTROL_LINE_STATE,
			control, NULL, 0);
}

#define ard_set_line(ard, line) \
	ard_ctrl_msg(ard, USB_CDC_REQ_SET_LINE_CODING, 0, line, sizeof *(line))
#define ard_send_break(ard, ms) \
	ard_ctrl_msg(ard, USB_CDC_REQ_SEND_BREAK, ms, NULL, 0)

static void ard_kill_urbs(struct ard *ard)
{
	int i;
	printk(KERN_INFO KBUILD_MODNAME ": ard_kill_urbs called\n");

	usb_kill_urb(ard->ctrlurb);
	for (i = 0; i < ARD_NW; i++)
		usb_kill_urb(ard->wb[i].urb);
	for (i = 0; i < ard->rx_buflimit; i++)
		usb_kill_urb(ard->read_urbs[i]);
}

/*
 * Write buffer management.
 * All of these assume proper locks taken by the caller.
 */

static int ard_wb_alloc(struct ard *ard)
{
	int i, wbn;
	struct ard_wb *wb;

	printk(KERN_INFO KBUILD_MODNAME ": ard_wb_alloc called\n");
	wbn = 0;
	i = 0;
	for (;;) {
		wb = &ard->wb[wbn];
		if (!wb->use) {
			wb->use = true;
			wb->len = 0;
			return wbn;
		}
		wbn = (wbn + 1) % ARD_NW;
		if (++i >= ARD_NW)
			return -1;
	}
}

/*
 * Finish write. Caller must hold ard->write_lock
 */
static void ard_write_done(struct ard *ard, struct ard_wb *wb)
{
	printk(KERN_INFO KBUILD_MODNAME ": ard_write_done called\n");
	wb->use = false;
	ard->transmitting--;
	usb_autopm_put_interface_async(ard->control);
}

/*
 * Poke write.
 *
 * the caller is responsible for locking
 */

static int ard_start_wb(struct ard *ard, struct ard_wb *wb)
{
	int rc;
	printk(KERN_INFO KBUILD_MODNAME ": ard_start_wb called\n");

	ard->transmitting++;

	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = ard->dev;

	rc = usb_submit_urb(wb->urb, GFP_ATOMIC);
	if (rc < 0) {
		dev_err(&ard->data->dev,
			"%s - usb_submit_urb(write bulk) failed: %d\n",
			__func__, rc);
		ard_write_done(ard, wb);
	}
	return rc;
}

/*
 * attributes exported through sysfs
 */
static ssize_t bmCapabilities_show
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf;
	struct ard *ard;

	printk(KERN_INFO KBUILD_MODNAME ": bmCapabilities_show called\n");
	intf = to_usb_interface(dev);
	ard = usb_get_intfdata(intf);
	return sprintf(buf, "%d", ard->ctrl_caps);
}
static DEVICE_ATTR_RO(bmCapabilities);

static ssize_t wCountryCodes_show
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf;
	struct ard *ard;

	printk(KERN_INFO KBUILD_MODNAME ": wCountryCodes_show called\n");
	intf = to_usb_interface(dev);
	ard = usb_get_intfdata(intf);
	memcpy(buf, ard->country_codes, ard->country_code_size);
	return ard->country_code_size;
}

static DEVICE_ATTR_RO(wCountryCodes);

static ssize_t iCountryCodeRelDate_show
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf;
	struct ard *ard;

	printk(KERN_INFO KBUILD_MODNAME ": iCountryCodeRelDate_show called\n");
	intf = to_usb_interface(dev);
	ard = usb_get_intfdata(intf);
	return sprintf(buf, "%d", ard->country_rel_date);
}

static DEVICE_ATTR_RO(iCountryCodeRelDate);

/* control interface reports status changes with "interrupt" transfers */
static void ard_ctrl_irq(struct urb *urb)
{
	struct ard *ard = urb->context;
	struct usb_cdc_notification *dr = urb->transfer_buffer;
	unsigned int current_size = urb->actual_length;
	unsigned int expected_size, copy_size, alloc_size;
	int retval;
	int status = urb->status;

	printk(KERN_INFO KBUILD_MODNAME ": ard_ctrl_irq called\n");
	switch (status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(&ard->control->dev,
			"%s - urb shutting down with status: %d\n",
			__func__, status);
		return;
	default:
		dev_dbg(&ard->control->dev,
			"%s - nonzero urb status received: %d\n",
			__func__, status);
		goto exit;
	}

	usb_mark_last_busy(ard->dev);

	if (ard->nb_index)
		dr = (struct usb_cdc_notification *)ard->notification_buffer;

	/* size = notification-header + (optional) data */
	expected_size = sizeof(struct usb_cdc_notification) +
					le16_to_cpu(dr->wLength);

	if (current_size < expected_size) {
		/* notification is transmitted fragmented, reassemble */
		if (ard->nb_size < expected_size) {
			u8 *new_buffer;
			alloc_size = roundup_pow_of_two(expected_size);
			/* Final freeing is done on disconnect. */
			new_buffer = krealloc(ard->notification_buffer,
					      alloc_size, GFP_ATOMIC);
			if (!new_buffer) {
				ard->nb_index = 0;
				goto exit;
			}

			ard->notification_buffer = new_buffer;
			ard->nb_size = alloc_size;
			dr = (struct usb_cdc_notification *)ard->notification_buffer;
		}

		copy_size = min(current_size,
				expected_size - ard->nb_index);

		memcpy(&ard->notification_buffer[ard->nb_index],
		       urb->transfer_buffer, copy_size);
		ard->nb_index += copy_size;
		current_size = ard->nb_index;
	}

	// if (current_size >= expected_size) {
	// 	/* notification complete */
	// 	ard_process_notification(ard, (unsigned char *)dr);
	// 	ard->nb_index = 0;
	// }

exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval && retval != -EPERM && retval != -ENODEV)
		dev_err(&ard->control->dev,
			"%s - usb_submit_urb failed: %d\n", __func__, retval);
	else
		dev_vdbg(&ard->control->dev,
			"control resubmission terminated %d\n", retval);
}

static int ard_submit_read_urb(struct ard *ard, int index, gfp_t mem_flags)
{
	int res;

	printk(KERN_INFO KBUILD_MODNAME ": ard_submit_read_urb called\n");
	if (!test_and_clear_bit(index, &ard->read_urbs_free))
		return 0;

	res = usb_submit_urb(ard->read_urbs[index], mem_flags);
	if (res) {
		if (res != -EPERM && res != -ENODEV) {
			dev_err(&ard->data->dev,
				"urb %d failed submission with %d\n",
				index, res);
		} else {
			dev_vdbg(&ard->data->dev, "intended failure %d\n", res);
		}
		set_bit(index, &ard->read_urbs_free);
		return res;
	} else {
		dev_vdbg(&ard->data->dev, "submitted urb %d\n", index);
	}

	return 0;
}

static int ard_submit_read_urbs(struct ard *ard, gfp_t mem_flags)
{
	int res;
	int i;

	printk(KERN_INFO KBUILD_MODNAME ": ard_submit_read_urbs called\n");
	for (i = 0; i < ard->rx_buflimit; ++i) {
		res = ard_submit_read_urb(ard, i, mem_flags);
		if (res)
			return res;
	}

	return 0;
}

static void ard_process_read_urb(struct ard *ard, struct urb *urb)
{
	printk(KERN_INFO KBUILD_MODNAME ": ard_process_read_urb called\n");
	if (!urb->actual_length)
		return;

	tty_insert_flip_string(&ard->port, urb->transfer_buffer,
			urb->actual_length);
	tty_flip_buffer_push(&ard->port);
}

static void ard_read_bulk_callback(struct urb *urb)
{
	struct ard_rb *rb = urb->context;
	struct ard *ard = rb->instance;
	int status = urb->status;
	bool stopped = false;
	bool stalled = false;
	bool cooldown = false;

	printk(KERN_INFO KBUILD_MODNAME ": ard_read_bulk_callback called\n");
	dev_vdbg(&ard->data->dev, "got urb %d, len %d, status %d\n",
		rb->index, urb->actual_length, status);

	if (!ard->dev) {
		dev_dbg(&ard->data->dev, "%s - disconnected\n", __func__);
		return;
	}

	switch (status) {
	case 0:
		usb_mark_last_busy(ard->dev);
		ard_process_read_urb(ard, urb);
		break;
	case -EPIPE:
		set_bit(EVENT_RX_STALL, &ard->flags);
		stalled = true;
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		dev_dbg(&ard->data->dev,
			"%s - urb shutting down with status: %d\n",
			__func__, status);
		stopped = true;
		break;
	case -EOVERFLOW:
	case -EPROTO:
		dev_dbg(&ard->data->dev,
			"%s - cooling babbling device\n", __func__);
		usb_mark_last_busy(ard->dev);
		set_bit(rb->index, &ard->urbs_in_error_delay);
		set_bit(ARD_ERROR_DELAY, &ard->flags);
		cooldown = true;
		break;
	default:
		dev_dbg(&ard->data->dev,
			"%s - nonzero urb status received: %d\n",
			__func__, status);
		break;
	}

	/*
	 * Make sure URB processing is done before marking as free to avoid
	 * racing with unthrottle() on another CPU. Matches the barriers
	 * implied by the test_and_clear_bit() in ard_submit_read_urb().
	 */
	smp_mb__before_atomic();
	set_bit(rb->index, &ard->read_urbs_free);
	/*
	 * Make sure URB is marked as free before checking the throttled flag
	 * to avoid racing with unthrottle() on another CPU. Matches the
	 * smp_mb() in unthrottle().
	 */
	smp_mb__after_atomic();

	if (stopped || stalled || cooldown) {
		if (stalled)
			schedule_delayed_work(&ard->dwork, 0);
		else if (cooldown)
			schedule_delayed_work(&ard->dwork, HZ / 2);
		return;
	}

	if (test_bit(ARD_THROTTLED, &ard->flags))
		return;

	ard_submit_read_urb(ard, rb->index, GFP_ATOMIC);
}

/* data interface wrote those outgoing bytes */
static void ard_write_bulk(struct urb *urb)
{
	struct ard_wb *wb = urb->context;
	struct ard *ard = wb->instance;
	unsigned long flags;
	int status = urb->status;

	printk(KERN_INFO KBUILD_MODNAME ": ard_write_bulk called\n");
	if (status || (urb->actual_length != urb->transfer_buffer_length))
		dev_vdbg(&ard->data->dev, "wrote len %d/%d, status %d\n",
			urb->actual_length,
			urb->transfer_buffer_length,
			status);

	spin_lock_irqsave(&ard->write_lock, flags);
	ard_write_done(ard, wb);
	spin_unlock_irqrestore(&ard->write_lock, flags);
	set_bit(EVENT_TTY_WAKEUP, &ard->flags);
	schedule_delayed_work(&ard->dwork, 0);
}

static void ard_softint(struct work_struct *work)
{
	int i;
	struct ard *ard = container_of(work, struct ard, dwork.work);

	printk(KERN_INFO KBUILD_MODNAME ": ard_softint called\n");
	if (test_bit(EVENT_RX_STALL, &ard->flags)) {
		smp_mb(); /* against ard_suspend() */
		if (!ard->susp_count) {
			for (i = 0; i < ard->rx_buflimit; i++)
				usb_kill_urb(ard->read_urbs[i]);
			usb_clear_halt(ard->dev, ard->in);
			ard_submit_read_urbs(ard, GFP_KERNEL);
			clear_bit(EVENT_RX_STALL, &ard->flags);
		}
	}

	if (test_and_clear_bit(ARD_ERROR_DELAY, &ard->flags)) {
		for (i = 0; i < ard->rx_buflimit; i++)
			if (test_and_clear_bit(i, &ard->urbs_in_error_delay))
				ard_submit_read_urb(ard, i, GFP_KERNEL);
	}

	if (test_and_clear_bit(EVENT_TTY_WAKEUP, &ard->flags))
		tty_port_tty_wakeup(&ard->port);
}

/*
 * TTY handlers
 */

static int ard_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct ard *ard;
	int retval;

	printk(KERN_INFO KBUILD_MODNAME ": ard_tty_install called\n");
	ard = ard_get_by_minor(tty->index);
	if (!ard)
		return -ENODEV;

	retval = tty_standard_install(driver, tty);
	if (retval)
		goto error_init_termios;

	tty->driver_data = ard;

	return 0;

error_init_termios:
	tty_port_put(&ard->port);
	return retval;
}

static int ard_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct ard *ard = tty->driver_data;

	printk(KERN_INFO KBUILD_MODNAME ": ard_tty_open called\n");
	return tty_port_open(&ard->port, tty, filp);
}

static void ard_port_dtr_rts(struct tty_port *port, int raise)
{
	struct ard *ard = container_of(port, struct ard, port);
	int val;
	int res;

	printk(KERN_INFO KBUILD_MODNAME ": ard_port_dtr_rts called\n");
	if (raise)
		val = ARD_CTRL_DTR | ARD_CTRL_RTS;
	else
		val = 0;

	/* FIXME: add missing ctrlout locking throughout driver */
	ard->ctrlout = val;

	res = ard_set_control(ard, val);
	if (res && (ard->ctrl_caps & USB_CDC_CAP_LINE))
		dev_err(&ard->control->dev, "failed to set dtr/rts\n");
}

static int ard_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct ard *ard = container_of(port, struct ard, port);
	int retval = -ENODEV;
	int i;

	printk(KERN_INFO KBUILD_MODNAME ": ard_port_activate called\n");
	mutex_lock(&ard->mutex);
	if (ard->disconnected)
		goto disconnected;

	retval = usb_autopm_get_interface(ard->control);
	if (retval)
		goto error_get_interface;

	/*
	 * FIXME: Why do we need this? Allocating 64K of physically contiguous
	 * memory is really nasty...
	 */
	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	ard->control->needs_remote_wakeup = 1;

	ard->ctrlurb->dev = ard->dev;
	retval = usb_submit_urb(ard->ctrlurb, GFP_KERNEL);
	if (retval) {
		dev_err(&ard->control->dev,
			"%s - usb_submit_urb(ctrl irq) failed\n", __func__);
		goto error_submit_urb;
	}

	ard_tty_set_termios(tty, NULL);

	/*
	 * Unthrottle device in case the TTY was closed while throttled.
	 */
	clear_bit(ARD_THROTTLED, &ard->flags);

	retval = ard_submit_read_urbs(ard, GFP_KERNEL);
	if (retval)
		goto error_submit_read_urbs;

	usb_autopm_put_interface(ard->control);

	mutex_unlock(&ard->mutex);

	return 0;

error_submit_read_urbs:
	for (i = 0; i < ard->rx_buflimit; i++)
		usb_kill_urb(ard->read_urbs[i]);
	usb_kill_urb(ard->ctrlurb);
error_submit_urb:
	usb_autopm_put_interface(ard->control);
error_get_interface:
disconnected:
	mutex_unlock(&ard->mutex);

	return usb_translate_errors(retval);
}

static void ard_port_destruct(struct tty_port *port)
{
	struct ard *ard = container_of(port, struct ard, port);

	printk(KERN_INFO KBUILD_MODNAME ": ard_port_destruct called\n");
	ard_release_minor(ard);
	usb_put_intf(ard->control);
	kfree(ard->country_codes);
	kfree(ard);
}

static void ard_port_shutdown(struct tty_port *port)
{
	struct ard *ard = container_of(port, struct ard, port);
	struct urb *urb;
	struct ard_wb *wb;

	printk(KERN_INFO KBUILD_MODNAME ": ard_port_shutdown called\n");
	/*
	 * Need to grab write_lock to prevent race with resume, but no need to
	 * hold it due to the tty-port initialised flag.
	 */
	spin_lock_irq(&ard->write_lock);
	spin_unlock_irq(&ard->write_lock);

	usb_autopm_get_interface_no_resume(ard->control);
	ard->control->needs_remote_wakeup = 0;
	usb_autopm_put_interface(ard->control);

	for (;;) {
		urb = usb_get_from_anchor(&ard->delayed);
		if (!urb)
			break;
		wb = urb->context;
		wb->use = false;
		usb_autopm_put_interface_async(ard->control);
	}

	ard_kill_urbs(ard);
}

static void ard_tty_cleanup(struct tty_struct *tty)
{
	struct ard *ard = tty->driver_data;

	printk(KERN_INFO KBUILD_MODNAME ": ard_tty_cleanup called\n");
	tty_port_put(&ard->port);
}

static void ard_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct ard *ard = tty->driver_data;

	printk(KERN_INFO KBUILD_MODNAME ": ard_tty_close called\n");
	tty_port_close(&ard->port, tty, filp);
}

static int ard_wb_is_avail(struct ard *ard)
{
	int i, n;
	unsigned long flags;

	n = ARD_NW;
	spin_lock_irqsave(&ard->write_lock, flags);
	for (i = 0; i < ARD_NW; i++)
		if(ard->wb[i].use)
			n--;
	spin_unlock_irqrestore(&ard->write_lock, flags);
	return n;
}

static int ard_tty_write_room(struct tty_struct *tty)
{
	struct ard *ard = tty->driver_data;
	/*
	 * Do not let the line discipline to know that we have a reserve,
	 * or it might get too enthusiastic.
	 */
	return ard_wb_is_avail(ard) ? ard->writesize : 0;
}

static int ard_tty_write(struct tty_struct *tty,
					const unsigned char *buf, int count)
{
	struct ard *ard = tty->driver_data;
	int stat;
	unsigned long flags;
	int wbn;
	struct ard_wb *wb;

	printk(KERN_INFO KBUILD_MODNAME ": ard_tty_write called\n");
	if (!count)
		return 0;

	dev_vdbg(&ard->data->dev, "%d bytes from tty layer\n", count);

	spin_lock_irqsave(&ard->write_lock, flags);
	wbn = ard_wb_alloc(ard);
	if (wbn < 0) {
		spin_unlock_irqrestore(&ard->write_lock, flags);
		return 0;
	}
	wb = &ard->wb[wbn];

	if (!ard->dev) {
		wb->use = false;
		spin_unlock_irqrestore(&ard->write_lock, flags);
		return -ENODEV;
	}

	count = (count > ard->writesize) ? ard->writesize : count;
	dev_vdbg(&ard->data->dev, "writing %d bytes\n", count);
	memcpy(wb->buf, buf, count);
	wb->len = count;

	stat = usb_autopm_get_interface_async(ard->control);
	if (stat) {
		wb->use = false;
		spin_unlock_irqrestore(&ard->write_lock, flags);
		return stat;
	}

	if (ard->susp_count) {
		usb_anchor_urb(wb->urb, &ard->delayed);
		spin_unlock_irqrestore(&ard->write_lock, flags);
		return count;
	}

	stat = ard_start_wb(ard, wb);
	spin_unlock_irqrestore(&ard->write_lock, flags);

	if (stat < 0)
		return stat;
	return count;
}

static int ard_tty_ioctl(struct tty_struct *tty,
					unsigned int cmd, unsigned long arg)
{
	struct ard *ard = tty->driver_data;
	int rv = -ENOIOCTLCMD;

	printk(KERN_INFO KBUILD_MODNAME ": ard_tty_ioctl called\n");
	switch (cmd) {
	case TIOCMIWAIT:
		rv = usb_autopm_get_interface(ard->control);
		if (rv < 0) {
			rv = -EIO;
			break;
		}
		// rv = wait_serial_change(ard, arg);
		usb_autopm_put_interface(ard->control);
		break;
	}

	return rv;
}

static void ard_tty_set_termios(struct tty_struct *tty,
						struct ktermios *termios_old)
{
	struct ard *ard = tty->driver_data;
	struct ktermios *termios = &tty->termios;
	struct usb_cdc_line_coding newline;
	int newctrl = ard->ctrlout;

	printk(KERN_INFO KBUILD_MODNAME ": ard_tty_set_termios called\n");
	newline.dwDTERate = cpu_to_le32(tty_get_baud_rate(tty));
	newline.bCharFormat = termios->c_cflag & CSTOPB ? 2 : 0;
	newline.bParityType = termios->c_cflag & PARENB ?
				(termios->c_cflag & PARODD ? 1 : 2) +
				(termios->c_cflag & CMSPAR ? 2 : 0) : 0;
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		newline.bDataBits = 5;
		break;
	case CS6:
		newline.bDataBits = 6;
		break;
	case CS7:
		newline.bDataBits = 7;
		break;
	case CS8:
	default:
		newline.bDataBits = 8;
		break;
	}
	/* FIXME: Needs to clear unsupported bits in the termios */
	ard->clocal = ((termios->c_cflag & CLOCAL) != 0);

	if (C_BAUD(tty) == B0) {
		newline.dwDTERate = ard->line.dwDTERate;
		newctrl &= ~ARD_CTRL_DTR;
	} else if (termios_old && (termios_old->c_cflag & CBAUD) == B0) {
		newctrl |=  ARD_CTRL_DTR;
	}

	if (newctrl != ard->ctrlout)
		ard_set_control(ard, ard->ctrlout = newctrl);

	if (memcmp(&ard->line, &newline, sizeof newline)) {
		memcpy(&ard->line, &newline, sizeof newline);
		dev_dbg(&ard->control->dev, "%s - set line: %d %d %d %d\n",
			__func__,
			le32_to_cpu(newline.dwDTERate),
			newline.bCharFormat, newline.bParityType,
			newline.bDataBits);
		ard_set_line(ard, &ard->line);
	}
}

static const struct tty_port_operations ard_port_ops = {
	.dtr_rts = ard_port_dtr_rts,
	.shutdown = ard_port_shutdown,
	.activate = ard_port_activate,
	.destruct = ard_port_destruct,
};

/*
 * USB probe and disconnect routines.
 */

/* Little helpers: write/read buffers free */
static void ard_write_buffers_free(struct ard *ard)
{
	int i;
	struct ard_wb *wb;

	printk(KERN_INFO KBUILD_MODNAME ": ard_write_buffers_free called\n");
	for (wb = &ard->wb[0], i = 0; i < ARD_NW; i++, wb++)
		usb_free_coherent(ard->dev, ard->writesize, wb->buf, wb->dmah);
}

static void ard_read_buffers_free(struct ard *ard)
{
	int i;

	printk(KERN_INFO KBUILD_MODNAME ": ard_read_buffers_free called\n");
	for (i = 0; i < ard->rx_buflimit; i++)
		usb_free_coherent(ard->dev, ard->readsize,
			  ard->read_buffers[i].base, ard->read_buffers[i].dma);
}

/* Little helper: write buffers allocate */
static int ard_write_buffers_alloc(struct ard *ard)
{
	int i;
	struct ard_wb *wb;

	printk(KERN_INFO KBUILD_MODNAME ": ard_write_buffers_alloc called\n");
	for (wb = &ard->wb[0], i = 0; i < ARD_NW; i++, wb++) {
		wb->buf = usb_alloc_coherent(ard->dev, ard->writesize, GFP_KERNEL,
		    &wb->dmah);
		if (!wb->buf) {
			while (i != 0) {
				--i;
				--wb;
				usb_free_coherent(ard->dev, ard->writesize,
				    wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	return 0;
}

static int ard_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_cdc_union_desc *union_header = NULL;
	struct usb_cdc_call_mgmt_descriptor *cmgmd = NULL;
	unsigned char *buffer = intf->altsetting->extra;
	int buflen = intf->altsetting->extralen;
	struct usb_interface *control_interface;
	struct usb_interface *data_interface;
	struct usb_endpoint_descriptor *epctrl = NULL;
	struct usb_endpoint_descriptor *epread = NULL;
	struct usb_endpoint_descriptor *epwrite = NULL;
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct usb_cdc_parsed_header h;
	struct ard *ard;
	int minor;
	int ctrlsize, readsize;
	u8 *buf;
	int call_intf_num = -1;
	int data_intf_num = -1;
	int num_rx_buf;
	int i;
	int combined_interfaces = 0;
	struct device *tty_dev;
	int rv = -ENOMEM;

	printk(KERN_INFO KBUILD_MODNAME ": ard_probe called\n");
	memset(&h, 0x00, sizeof(struct usb_cdc_parsed_header));
	num_rx_buf = ARD_NR;

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

	ard = kzalloc(sizeof(struct ard), GFP_KERNEL);
	if (ard == NULL)
		goto alloc_fail;

	tty_port_init(&ard->port);
	ard->port.ops = &ard_port_ops;

	ctrlsize = usb_endpoint_maxp(epctrl);
	readsize = usb_endpoint_maxp(epread) * 2;
	ard->combined_interfaces = combined_interfaces;
	ard->writesize = usb_endpoint_maxp(epwrite) * 20;
	ard->control = control_interface;
	ard->data = data_interface;

	usb_get_intf(ard->control); /* undone in destruct() */

	minor = ard_alloc_minor(ard);
	if (minor < 0)
		goto alloc_fail1;

	ard->minor = minor;
	ard->dev = usb_dev;
	// if (h.usb_cdc_ard_descriptor)
	// 	ard->ctrl_caps = h.usb_cdc_ard_descriptor->bmCapabilities;
	ard->ctrlsize = ctrlsize;
	ard->readsize = readsize;
	ard->rx_buflimit = num_rx_buf;
	INIT_DELAYED_WORK(&ard->dwork, ard_softint);
	init_waitqueue_head(&ard->wioctl);
	spin_lock_init(&ard->write_lock);
	spin_lock_init(&ard->read_lock);
	mutex_init(&ard->mutex);
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

	buf = usb_alloc_coherent(usb_dev, ctrlsize, GFP_KERNEL, &ard->ctrl_dma);
	if (!buf)
		goto alloc_fail1;
	ard->ctrl_buffer = buf;

	if (ard_write_buffers_alloc(ard) < 0)
		goto alloc_fail2;

	ard->ctrlurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ard->ctrlurb)
		goto alloc_fail3;

	for (i = 0; i < num_rx_buf; i++) {
		struct ard_rb *rb = &(ard->read_buffers[i]);
		struct urb *urb;

		rb->base = usb_alloc_coherent(ard->dev, readsize, GFP_KERNEL,
								&rb->dma);
		if (!rb->base)
			goto alloc_fail4;
		rb->index = i;
		rb->instance = ard;

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb)
			goto alloc_fail4;

		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = rb->dma;
		if (usb_endpoint_xfer_int(epread))
			usb_fill_int_urb(urb, ard->dev, ard->in, rb->base,
					 ard->readsize,
					 ard_read_bulk_callback, rb,
					 ard->bInterval);
		else
			usb_fill_bulk_urb(urb, ard->dev, ard->in, rb->base,
					  ard->readsize,
					  ard_read_bulk_callback, rb);

		ard->read_urbs[i] = urb;
		__set_bit(i, &ard->read_urbs_free);
	}
	for (i = 0; i < ARD_NW; i++) {
		struct ard_wb *snd = &(ard->wb[i]);

		snd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (snd->urb == NULL)
			goto alloc_fail5;

		if (usb_endpoint_xfer_int(epwrite))
			usb_fill_int_urb(snd->urb, usb_dev, ard->out,
				NULL, ard->writesize, ard_write_bulk, snd, epwrite->bInterval);
		else
			usb_fill_bulk_urb(snd->urb, usb_dev, ard->out,
				NULL, ard->writesize, ard_write_bulk, snd);
		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		snd->instance = ard;
	}

	usb_set_intfdata(intf, ard);

	i = device_create_file(&intf->dev, &dev_attr_bmCapabilities);
	if (i < 0)
		goto alloc_fail5;

	if (h.usb_cdc_country_functional_desc) { /* export the country data */
		struct usb_cdc_country_functional_desc * cfd =
					h.usb_cdc_country_functional_desc;

		ard->country_codes = kmalloc(cfd->bLength - 4, GFP_KERNEL);
		if (!ard->country_codes)
			goto skip_countries;
		ard->country_code_size = cfd->bLength - 4;
		memcpy(ard->country_codes, (u8 *)&cfd->wCountyCode0,
							cfd->bLength - 4);
		ard->country_rel_date = cfd->iCountryCodeRelDate;

		i = device_create_file(&intf->dev, &dev_attr_wCountryCodes);
		if (i < 0) {
			kfree(ard->country_codes);
			ard->country_codes = NULL;
			ard->country_code_size = 0;
			goto skip_countries;
		}

		i = device_create_file(&intf->dev,
						&dev_attr_iCountryCodeRelDate);
		if (i < 0) {
			device_remove_file(&intf->dev, &dev_attr_wCountryCodes);
			kfree(ard->country_codes);
			ard->country_codes = NULL;
			ard->country_code_size = 0;
			goto skip_countries;
		}
	}

skip_countries:
	usb_fill_int_urb(ard->ctrlurb, usb_dev,
			 usb_rcvintpipe(usb_dev, epctrl->bEndpointAddress),
			 ard->ctrl_buffer, ctrlsize, ard_ctrl_irq, ard,
			 /* works around buggy devices */
			 epctrl->bInterval ? epctrl->bInterval : 16);
	ard->ctrlurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	ard->ctrlurb->transfer_dma = ard->ctrl_dma;
	ard->notification_buffer = NULL;
	ard->nb_index = 0;
	ard->nb_size = 0;

	dev_info(&intf->dev, "ttyARD%d: USB ARD device\n", minor);

	ard->line.dwDTERate = cpu_to_le32(9600);
	ard->line.bDataBits = 8;
	ard_set_line(ard, &ard->line);

	usb_driver_claim_interface(&ard_driver, data_interface, ard);
	usb_set_intfdata(data_interface, ard);

	tty_dev = tty_port_register_device(&ard->port, ard_tty_driver, minor,
			&control_interface->dev);
	if (IS_ERR(tty_dev)) {
		rv = PTR_ERR(tty_dev);
		goto alloc_fail6;
	}
	return 0;

alloc_fail6:
	if (ard->country_codes) {
		device_remove_file(&ard->control->dev,
				&dev_attr_wCountryCodes);
		device_remove_file(&ard->control->dev,
				&dev_attr_iCountryCodeRelDate);
		kfree(ard->country_codes);
	}
	device_remove_file(&ard->control->dev, &dev_attr_bmCapabilities);
alloc_fail5:
	usb_set_intfdata(intf, NULL);
	for (i = 0; i < ARD_NW; i++)
		usb_free_urb(ard->wb[i].urb);
alloc_fail4:
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(ard->read_urbs[i]);
	ard_read_buffers_free(ard);
	usb_free_urb(ard->ctrlurb);
alloc_fail3:
	ard_write_buffers_free(ard);
alloc_fail2:
	usb_free_coherent(usb_dev, ctrlsize, ard->ctrl_buffer, ard->ctrl_dma);
alloc_fail1:
	tty_port_put(&ard->port);
alloc_fail:
	return rv;
}

static void ard_disconnect(struct usb_interface *intf)
{
	struct ard *ard = usb_get_intfdata(intf);
	struct tty_struct *tty;
	int i;

	printk(KERN_INFO KBUILD_MODNAME ": ard_disconnect called\n");
	/* sibling interface is already cleaning up */
	if (!ard)
		return;

	mutex_lock(&ard->mutex);
	ard->disconnected = true;
	if (ard->country_codes) {
		device_remove_file(&ard->control->dev,
				&dev_attr_wCountryCodes);
		device_remove_file(&ard->control->dev,
				&dev_attr_iCountryCodeRelDate);
	}
	wake_up_all(&ard->wioctl);
	device_remove_file(&ard->control->dev, &dev_attr_bmCapabilities);
	usb_set_intfdata(ard->control, NULL);
	usb_set_intfdata(ard->data, NULL);
	mutex_unlock(&ard->mutex);

	tty = tty_port_tty_get(&ard->port);
	if (tty) {
		tty_vhangup(tty);
		tty_kref_put(tty);
	}

	ard_kill_urbs(ard);
	cancel_delayed_work_sync(&ard->dwork);

	tty_unregister_device(ard_tty_driver, ard->minor);

	usb_free_urb(ard->ctrlurb);
	for (i = 0; i < ARD_NW; i++)
		usb_free_urb(ard->wb[i].urb);
	for (i = 0; i < ard->rx_buflimit; i++)
		usb_free_urb(ard->read_urbs[i]);
	ard_write_buffers_free(ard);
	usb_free_coherent(ard->dev, ard->ctrlsize, ard->ctrl_buffer, ard->ctrl_dma);
	ard_read_buffers_free(ard);

	kfree(ard->notification_buffer);

	if (!ard->combined_interfaces)
		usb_driver_release_interface(&ard_driver, intf == ard->control ?
					ard->data : ard->control);

	tty_port_put(&ard->port);
}

#ifdef CONFIG_PM
static int ard_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct ard *ard = usb_get_intfdata(intf);
	int cnt;

	printk(KERN_INFO KBUILD_MODNAME ": ard_suspend called\n");
	spin_lock_irq(&ard->write_lock);
	if (PMSG_IS_AUTO(message)) {
		if (ard->transmitting) {
			spin_unlock_irq(&ard->write_lock);
			return -EBUSY;
		}
	}
	cnt = ard->susp_count++;
	spin_unlock_irq(&ard->write_lock);

	if (cnt)
		return 0;

	ard_kill_urbs(ard);
	cancel_delayed_work_sync(&ard->dwork);
	ard->urbs_in_error_delay = 0;

	return 0;
}

static int ard_resume(struct usb_interface *intf)
{
	struct ard *ard = usb_get_intfdata(intf);
	struct urb *urb;
	int rv = 0;

	printk(KERN_INFO KBUILD_MODNAME ": ard_resume called\n");
	spin_lock_irq(&ard->write_lock);

	if (--ard->susp_count)
		goto out;

	if (tty_port_initialized(&ard->port)) {
		rv = usb_submit_urb(ard->ctrlurb, GFP_ATOMIC);

		for (;;) {
			urb = usb_get_from_anchor(&ard->delayed);
			if (!urb)
				break;

			ard_start_wb(ard, urb->context);
		}

		/*
		 * delayed error checking because we must
		 * do the write path at all cost
		 */
		if (rv < 0)
			goto out;

		rv = ard_submit_read_urbs(ard, GFP_ATOMIC);
	}
out:
	spin_unlock_irq(&ard->write_lock);

	return rv;
}

static int ard_reset_resume(struct usb_interface *intf)
{
	struct ard *ard = usb_get_intfdata(intf);

	printk(KERN_INFO KBUILD_MODNAME ": ard_reset_resume called\n");
	if (tty_port_initialized(&ard->port))
		tty_port_tty_hangup(&ard->port, false);

	return ard_resume(intf);
}

#endif /* CONFIG_PM */

static const struct usb_device_id ard_ids[] = {
	{ USB_DEVICE(0x2341, 0x0043) },
	{ }
};

MODULE_DEVICE_TABLE(usb, ard_ids);

static struct usb_driver ard_driver = {
	.name =		"my_cdc_ard",
	.probe =	ard_probe,
	.disconnect =	ard_disconnect,
	.id_table =	ard_ids,
#ifdef CONFIG_PM
	.suspend =	ard_suspend,
	.resume =	ard_resume,
	.reset_resume =	ard_reset_resume,
	.supports_autosuspend = 1,
#endif
};

static const struct tty_operations ard_ops = {
	.install =		ard_tty_install,
	.open =			ard_tty_open,
	.close =		ard_tty_close,
	.cleanup =		ard_tty_cleanup,
	.write =		ard_tty_write,
	.write_room =		ard_tty_write_room,
	.ioctl =		ard_tty_ioctl,
	.set_termios =	ard_tty_set_termios,
};

static int __init ard_init(void)
{
	int retval;
	printk(KERN_INFO KBUILD_MODNAME ": THIS IS MY MODULE\n");
	ard_tty_driver = alloc_tty_driver(ARD_TTY_MINORS);
	if (!ard_tty_driver)
		return -ENOMEM;
	ard_tty_driver->driver_name = "ard",
	ard_tty_driver->name = "ttyARD",
	ard_tty_driver->major = ARD_TTY_MAJOR,
	ard_tty_driver->minor_start = 0,
	ard_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	ard_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	ard_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	// ard_tty_driver->init_termios = tty_std_termios;
	memset(&(ard_tty_driver->init_termios), 0, sizeof(struct termios));
	ard_tty_driver->init_termios.c_cflag |= (CREAD | CLOCAL);
	ard_tty_driver->init_termios.c_cflag &= ~CSIZE;
	ard_tty_driver->init_termios.c_cflag |= CS8;
	ard_tty_driver->init_termios.c_cflag &= ~CSTOPB;

	ard_tty_driver->init_termios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	ard_tty_driver->init_termios.c_iflag &= ~(INPCK);
	ard_tty_driver->init_termios.c_oflag &= ~OPOST;

	tty_set_operations(ard_tty_driver, &ard_ops);

	printk(KERN_INFO KBUILD_MODNAME ": register tty\n");
	retval = tty_register_driver(ard_tty_driver);
	if (retval) {
		put_tty_driver(ard_tty_driver);
		return retval;
	}
	printk(KERN_INFO KBUILD_MODNAME ": regisration finished tty\n");
	printk(KERN_INFO KBUILD_MODNAME ": register usb\n");
	retval = usb_register(&ard_driver);
	if (retval) {
		tty_unregister_driver(ard_tty_driver);
		put_tty_driver(ard_tty_driver);
		return retval;
	}
	printk(KERN_INFO KBUILD_MODNAME ": regisration finished usb\n");

	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_DESC "\n");

	return 0;
}

static void __exit ard_exit(void)
{
	usb_deregister(&ard_driver);
	tty_unregister_driver(ard_tty_driver);
	put_tty_driver(ard_tty_driver);
	idr_destroy(&ard_minors);
}

module_init(ard_init);
module_exit(ard_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(ARD_TTY_MAJOR);
