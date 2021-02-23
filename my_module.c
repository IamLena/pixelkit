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

#define ACM_TTY_MAJOR		240
#define ACM_TTY_MINORS		1

// Requests.
#define USB_RT_ACM		(USB_TYPE_CLASS | USB_RECIP_INTERFACE)

// Output control lines.
#define ACM_CTRL_DTR		0x01
#define ACM_CTRL_RTS		0x02

// Input control lines and line errors.
#define ACM_CTRL_DCD		0x01
#define ACM_CTRL_DSR		0x02
#define ACM_CTRL_BRK		0x04
#define ACM_CTRL_RI			0x08
#define ACM_CTRL_FRAMING	0x10
#define ACM_CTRL_PARITY		0x20
#define ACM_CTRL_OVERRUN	0x40

/*
 * Internal driver structures.
 */

/*
 * The only reason to have several buffers is to accommodate assumptions
 * in line disciplines. They ask for empty space amount, receive our URB size,
 * and proceed to issue several 1-character writes, assuming they will fit.
 * The very first write takes a complete URB. Fortunately, this only happens
 * when processing onlcr, so we only need 2 buffers. These values must be
 * powers of 2.
 */
#define ACM_NW  16
#define ACM_NR  16

struct acm_wb {
	u8 *buf;
	dma_addr_t dmah;
	unsigned int len;
	struct urb		*urb;
	struct acm		*instance;
	bool use;
};

struct acm_rb {
	int			size;
	unsigned char		*base;
	dma_addr_t		dma;
	int			index;
	struct acm		*instance;
};

struct acm {
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
	struct acm_wb wb[ACM_NW];
	unsigned long read_urbs_free;
	struct urb *read_urbs[ACM_NR];
	struct acm_rb read_buffers[ACM_NR];
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
#		define ACM_THROTTLED	2
#		define ACM_ERROR_DELAY	3
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
	unsigned int minor;					/* acm minor number */
	unsigned char clocal;				/* termios CLOCAL */
	unsigned int ctrl_caps;				/* control capabilities from the class specific header */
	unsigned int susp_count;			/* number of suspended interfaces */
	unsigned int combined_interfaces:1;	/* control and data collapsed */
	u8 bInterval;
	struct usb_anchor delayed;			/* writes queued for a device about to be woken */
};

#define DRIVER_AUTHOR "Luchina"
#define DRIVER_DESC "driver for ardruino uno device to print messages"

static struct usb_driver acm_driver;
static struct tty_driver *acm_tty_driver;

static DEFINE_IDR(acm_minors);
static DEFINE_MUTEX(acm_minors_lock);

static void acm_tty_set_termios(struct tty_struct *tty, struct ktermios *termios_old);

/*
 * acm_minors accessors
 */

/*
 * Look up an ACM structure by minor. If found and not disconnected, increment
 * its refcount and return it with its mutex held.
 */
static struct acm *acm_get_by_minor(unsigned int minor)
{
	printk(KERN_INFO "acm_get_by_minor called\n");
	struct acm *acm;

	mutex_lock(&acm_minors_lock);
	acm = idr_find(&acm_minors, minor);
	if (acm) {
		mutex_lock(&acm->mutex);
		if (acm->disconnected) {
			mutex_unlock(&acm->mutex);
			acm = NULL;
		} else {
			tty_port_get(&acm->port);
			mutex_unlock(&acm->mutex);
		}
	}
	mutex_unlock(&acm_minors_lock);
	return acm;
}

/*
 * Try to find an available minor number and if found, associate it with 'acm'.
 */
static int acm_alloc_minor(struct acm *acm)
{
	printk(KERN_INFO "acm_alloc_minor called\n");
	int minor;

	mutex_lock(&acm_minors_lock);
	minor = idr_alloc(&acm_minors, acm, 0, ACM_TTY_MINORS, GFP_KERNEL);
	mutex_unlock(&acm_minors_lock);

	return minor;
}

/* Release the minor number associated with 'acm'.  */
static void acm_release_minor(struct acm *acm)
{
	printk(KERN_INFO "acm_release_minor called\n");
	mutex_lock(&acm_minors_lock);
	idr_remove(&acm_minors, acm->minor);
	mutex_unlock(&acm_minors_lock);
}

/*
 * Functions for ACM control messages.
 */

static int acm_ctrl_msg(struct acm *acm, int request, int value,
							void *buf, int len)
{
	printk(KERN_INFO "acm_ctrl_msg called\n");
	int retval;

	retval = usb_autopm_get_interface(acm->control);
	if (retval)
		return retval;

	retval = usb_control_msg(acm->dev, usb_sndctrlpipe(acm->dev, 0),
		request, USB_RT_ACM, value,
		acm->control->altsetting[0].desc.bInterfaceNumber,
		buf, len, 5000);

	dev_dbg(&acm->control->dev,
		"%s - rq 0x%02x, val %#x, len %#x, result %d\n",
		__func__, request, value, len, retval);

	usb_autopm_put_interface(acm->control);

	return retval < 0 ? retval : 0;
}

/* devices aren't required to support these requests.
 * the cdc acm descriptor tells whether they do...
 */
static inline int acm_set_control(struct acm *acm, int control)
{
	printk(KERN_INFO "acm_set_control called\n");
	return acm_ctrl_msg(acm, USB_CDC_REQ_SET_CONTROL_LINE_STATE,
			control, NULL, 0);
}

#define acm_set_line(acm, line) \
	acm_ctrl_msg(acm, USB_CDC_REQ_SET_LINE_CODING, 0, line, sizeof *(line))
#define acm_send_break(acm, ms) \
	acm_ctrl_msg(acm, USB_CDC_REQ_SEND_BREAK, ms, NULL, 0)

static void acm_kill_urbs(struct acm *acm)
{
	printk(KERN_INFO "acm_kill_urbs called\n");
	int i;

	usb_kill_urb(acm->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_kill_urb(acm->wb[i].urb);
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->read_urbs[i]);
}

/*
 * Write buffer management.
 * All of these assume proper locks taken by the caller.
 */

static int acm_wb_alloc(struct acm *acm)
{
	printk(KERN_INFO "acm_wb_alloc called\n");
	int i, wbn;
	struct acm_wb *wb;

	wbn = 0;
	i = 0;
	for (;;) {
		wb = &acm->wb[wbn];
		if (!wb->use) {
			wb->use = true;
			wb->len = 0;
			return wbn;
		}
		wbn = (wbn + 1) % ACM_NW;
		if (++i >= ACM_NW)
			return -1;
	}
}

// static int acm_wb_is_avail(struct acm *acm)
// {
// 	printk(KERN_INFO "acm_wb_is_avail called\n");
// 	int i, n;
// 	unsigned long flags;

// 	n = ACM_NW;
// 	spin_lock_irqsave(&acm->write_lock, flags);
// 	for (i = 0; i < ACM_NW; i++)
// 		if(acm->wb[i].use)
// 			n--;
// 	spin_unlock_irqrestore(&acm->write_lock, flags);
// 	return n;
// }

/*
 * Finish write. Caller must hold acm->write_lock
 */
static void acm_write_done(struct acm *acm, struct acm_wb *wb)
{
	printk(KERN_INFO "acm_write_done called\n");
	wb->use = false;
	acm->transmitting--;
	usb_autopm_put_interface_async(acm->control);
}

/*
 * Poke write.
 *
 * the caller is responsible for locking
 */

static int acm_start_wb(struct acm *acm, struct acm_wb *wb)
{
	printk(KERN_INFO "acm_start_wb called\n");
	int rc;

	acm->transmitting++;

	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = acm->dev;

	rc = usb_submit_urb(wb->urb, GFP_ATOMIC);
	if (rc < 0) {
		dev_err(&acm->data->dev,
			"%s - usb_submit_urb(write bulk) failed: %d\n",
			__func__, rc);
		acm_write_done(acm, wb);
	}
	return rc;
}

/*
 * attributes exported through sysfs
 */
// static ssize_t bmCapabilities_show
// (struct device *dev, struct device_attribute *attr, char *buf)
// {
// 	printk(KERN_INFO "bmCapabilities_show called\n");
// 	struct usb_interface *intf = to_usb_interface(dev);
// 	struct acm *acm = usb_get_intfdata(intf);

// 	return sprintf(buf, "%d", acm->ctrl_caps);
// }
// static DEVICE_ATTR_RO(bmCapabilities);

// static ssize_t wCountryCodes_show
// (struct device *dev, struct device_attribute *attr, char *buf)
// {
// 	printk(KERN_INFO "wCountryCodes_show called\n");
// 	struct usb_interface *intf = to_usb_interface(dev);
// 	struct acm *acm = usb_get_intfdata(intf);

// 	memcpy(buf, acm->country_codes, acm->country_code_size);
// 	return acm->country_code_size;
// }

// static DEVICE_ATTR_RO(wCountryCodes);

// static ssize_t iCountryCodeRelDate_show
// (struct device *dev, struct device_attribute *attr, char *buf)
// {
// 	printk(KERN_INFO "iCountryCodeRelDate_show called\n");
// 	struct usb_interface *intf = to_usb_interface(dev);
// 	struct acm *acm = usb_get_intfdata(intf);

// 	return sprintf(buf, "%d", acm->country_rel_date);
// }

// static DEVICE_ATTR_RO(iCountryCodeRelDate);
/*
 * Interrupt handlers for various ACM device responses
 */

// static void acm_process_notification(struct acm *acm, unsigned char *buf)
// {
// 	printk(KERN_INFO "acm_process_notification called\n");
// 	int newctrl;
// 	int difference;
// 	unsigned long flags;
// 	struct usb_cdc_notification *dr = (struct usb_cdc_notification *)buf;
// 	unsigned char *data = buf + sizeof(struct usb_cdc_notification);

// 	switch (dr->bNotificationType) {
// 	case USB_CDC_NOTIFY_NETWORK_CONNECTION:
// 		dev_dbg(&acm->control->dev,
// 			"%s - network connection: %d\n", __func__, dr->wValue);
// 		break;

// 	case USB_CDC_NOTIFY_SERIAL_STATE:
// 		if (le16_to_cpu(dr->wLength) != 2) {
// 			dev_dbg(&acm->control->dev,
// 				"%s - malformed serial state\n", __func__);
// 			break;
// 		}

// 		newctrl = get_unaligned_le16(data);
// 		dev_dbg(&acm->control->dev,
// 			"%s - serial state: 0x%x\n", __func__, newctrl);

// 		if (!acm->clocal && (acm->ctrlin & ~newctrl & ACM_CTRL_DCD)) {
// 			dev_dbg(&acm->control->dev,
// 				"%s - calling hangup\n", __func__);
// 			tty_port_tty_hangup(&acm->port, false);
// 		}

// 		difference = acm->ctrlin ^ newctrl;
// 		spin_lock_irqsave(&acm->read_lock, flags);
// 		acm->ctrlin = newctrl;
// 		acm->oldcount = acm->iocount;

// 		if (difference & ACM_CTRL_DSR)
// 			acm->iocount.dsr++;
// 		if (difference & ACM_CTRL_DCD)
// 			acm->iocount.dcd++;
// 		if (newctrl & ACM_CTRL_BRK)
// 			acm->iocount.brk++;
// 		if (newctrl & ACM_CTRL_RI)
// 			acm->iocount.rng++;
// 		if (newctrl & ACM_CTRL_FRAMING)
// 			acm->iocount.frame++;
// 		if (newctrl & ACM_CTRL_PARITY)
// 			acm->iocount.parity++;
// 		if (newctrl & ACM_CTRL_OVERRUN)
// 			acm->iocount.overrun++;
// 		spin_unlock_irqrestore(&acm->read_lock, flags);

// 		if (difference)
// 			wake_up_all(&acm->wioctl);

// 		break;

// 	default:
// 		dev_dbg(&acm->control->dev,
// 			"%s - unknown notification %d received: index %d len %d\n",
// 			__func__,
// 			dr->bNotificationType, dr->wIndex, dr->wLength);
// 	}
// }

/* control interface reports status changes with "interrupt" transfers */
static void acm_ctrl_irq(struct urb *urb)
{
	printk(KERN_INFO "acm_ctrl_irq called\n");
	struct acm *acm = urb->context;
	struct usb_cdc_notification *dr = urb->transfer_buffer;
	unsigned int current_size = urb->actual_length;
	unsigned int expected_size, copy_size, alloc_size;
	int retval;
	int status = urb->status;

	switch (status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(&acm->control->dev,
			"%s - urb shutting down with status: %d\n",
			__func__, status);
		return;
	default:
		dev_dbg(&acm->control->dev,
			"%s - nonzero urb status received: %d\n",
			__func__, status);
		goto exit;
	}

	usb_mark_last_busy(acm->dev);

	if (acm->nb_index)
		dr = (struct usb_cdc_notification *)acm->notification_buffer;

	/* size = notification-header + (optional) data */
	expected_size = sizeof(struct usb_cdc_notification) +
					le16_to_cpu(dr->wLength);

	if (current_size < expected_size) {
		/* notification is transmitted fragmented, reassemble */
		if (acm->nb_size < expected_size) {
			u8 *new_buffer;
			alloc_size = roundup_pow_of_two(expected_size);
			/* Final freeing is done on disconnect. */
			new_buffer = krealloc(acm->notification_buffer,
					      alloc_size, GFP_ATOMIC);
			if (!new_buffer) {
				acm->nb_index = 0;
				goto exit;
			}

			acm->notification_buffer = new_buffer;
			acm->nb_size = alloc_size;
			dr = (struct usb_cdc_notification *)acm->notification_buffer;
		}

		copy_size = min(current_size,
				expected_size - acm->nb_index);

		memcpy(&acm->notification_buffer[acm->nb_index],
		       urb->transfer_buffer, copy_size);
		acm->nb_index += copy_size;
		current_size = acm->nb_index;
	}

	// if (current_size >= expected_size) {
	// 	/* notification complete */
	// 	acm_process_notification(acm, (unsigned char *)dr);
	// 	acm->nb_index = 0;
	// }

exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval && retval != -EPERM && retval != -ENODEV)
		dev_err(&acm->control->dev,
			"%s - usb_submit_urb failed: %d\n", __func__, retval);
	else
		dev_vdbg(&acm->control->dev,
			"control resubmission terminated %d\n", retval);
}

static int acm_submit_read_urb(struct acm *acm, int index, gfp_t mem_flags)
{
	printk(KERN_INFO "acm_submit_read_urb called\n");
	int res;

	if (!test_and_clear_bit(index, &acm->read_urbs_free))
		return 0;

	res = usb_submit_urb(acm->read_urbs[index], mem_flags);
	if (res) {
		if (res != -EPERM && res != -ENODEV) {
			dev_err(&acm->data->dev,
				"urb %d failed submission with %d\n",
				index, res);
		} else {
			dev_vdbg(&acm->data->dev, "intended failure %d\n", res);
		}
		set_bit(index, &acm->read_urbs_free);
		return res;
	} else {
		dev_vdbg(&acm->data->dev, "submitted urb %d\n", index);
	}

	return 0;
}

static int acm_submit_read_urbs(struct acm *acm, gfp_t mem_flags)
{
	printk(KERN_INFO "acm_submit_read_urbs called\n");
	int res;
	int i;

	for (i = 0; i < acm->rx_buflimit; ++i) {
		res = acm_submit_read_urb(acm, i, mem_flags);
		if (res)
			return res;
	}

	return 0;
}

static void acm_process_read_urb(struct acm *acm, struct urb *urb)
{
	printk(KERN_INFO "acm_process_read_urb called\n");
	if (!urb->actual_length)
		return;

	tty_insert_flip_string(&acm->port, urb->transfer_buffer,
			urb->actual_length);
	tty_flip_buffer_push(&acm->port);
}

static void acm_read_bulk_callback(struct urb *urb)
{
	printk(KERN_INFO "acm_read_bulk_callback called\n");
	struct acm_rb *rb = urb->context;
	struct acm *acm = rb->instance;
	int status = urb->status;
	bool stopped = false;
	bool stalled = false;
	bool cooldown = false;

	dev_vdbg(&acm->data->dev, "got urb %d, len %d, status %d\n",
		rb->index, urb->actual_length, status);

	if (!acm->dev) {
		dev_dbg(&acm->data->dev, "%s - disconnected\n", __func__);
		return;
	}

	switch (status) {
	case 0:
		usb_mark_last_busy(acm->dev);
		acm_process_read_urb(acm, urb);
		break;
	case -EPIPE:
		set_bit(EVENT_RX_STALL, &acm->flags);
		stalled = true;
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		dev_dbg(&acm->data->dev,
			"%s - urb shutting down with status: %d\n",
			__func__, status);
		stopped = true;
		break;
	case -EOVERFLOW:
	case -EPROTO:
		dev_dbg(&acm->data->dev,
			"%s - cooling babbling device\n", __func__);
		usb_mark_last_busy(acm->dev);
		set_bit(rb->index, &acm->urbs_in_error_delay);
		set_bit(ACM_ERROR_DELAY, &acm->flags);
		cooldown = true;
		break;
	default:
		dev_dbg(&acm->data->dev,
			"%s - nonzero urb status received: %d\n",
			__func__, status);
		break;
	}

	/*
	 * Make sure URB processing is done before marking as free to avoid
	 * racing with unthrottle() on another CPU. Matches the barriers
	 * implied by the test_and_clear_bit() in acm_submit_read_urb().
	 */
	smp_mb__before_atomic();
	set_bit(rb->index, &acm->read_urbs_free);
	/*
	 * Make sure URB is marked as free before checking the throttled flag
	 * to avoid racing with unthrottle() on another CPU. Matches the
	 * smp_mb() in unthrottle().
	 */
	smp_mb__after_atomic();

	if (stopped || stalled || cooldown) {
		if (stalled)
			schedule_delayed_work(&acm->dwork, 0);
		else if (cooldown)
			schedule_delayed_work(&acm->dwork, HZ / 2);
		return;
	}

	if (test_bit(ACM_THROTTLED, &acm->flags))
		return;

	acm_submit_read_urb(acm, rb->index, GFP_ATOMIC);
}

/* data interface wrote those outgoing bytes */
static void acm_write_bulk(struct urb *urb)
{
	printk(KERN_INFO "acm_write_bulk called\n");
	struct acm_wb *wb = urb->context;
	struct acm *acm = wb->instance;
	unsigned long flags;
	int status = urb->status;

	if (status || (urb->actual_length != urb->transfer_buffer_length))
		dev_vdbg(&acm->data->dev, "wrote len %d/%d, status %d\n",
			urb->actual_length,
			urb->transfer_buffer_length,
			status);

	spin_lock_irqsave(&acm->write_lock, flags);
	acm_write_done(acm, wb);
	spin_unlock_irqrestore(&acm->write_lock, flags);
	set_bit(EVENT_TTY_WAKEUP, &acm->flags);
	schedule_delayed_work(&acm->dwork, 0);
}

static void acm_softint(struct work_struct *work)
{
	printk(KERN_INFO "acm_softint called\n");
	int i;
	struct acm *acm = container_of(work, struct acm, dwork.work);

	if (test_bit(EVENT_RX_STALL, &acm->flags)) {
		smp_mb(); /* against acm_suspend() */
		if (!acm->susp_count) {
			for (i = 0; i < acm->rx_buflimit; i++)
				usb_kill_urb(acm->read_urbs[i]);
			usb_clear_halt(acm->dev, acm->in);
			acm_submit_read_urbs(acm, GFP_KERNEL);
			clear_bit(EVENT_RX_STALL, &acm->flags);
		}
	}

	if (test_and_clear_bit(ACM_ERROR_DELAY, &acm->flags)) {
		for (i = 0; i < acm->rx_buflimit; i++)
			if (test_and_clear_bit(i, &acm->urbs_in_error_delay))
				acm_submit_read_urb(acm, i, GFP_KERNEL);
	}

	if (test_and_clear_bit(EVENT_TTY_WAKEUP, &acm->flags))
		tty_port_tty_wakeup(&acm->port);
}

/*
 * TTY handlers
 */

static int acm_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	printk(KERN_INFO "acm_tty_install called\n");
	struct acm *acm;
	int retval;

	acm = acm_get_by_minor(tty->index);
	if (!acm)
		return -ENODEV;

	retval = tty_standard_install(driver, tty);
	if (retval)
		goto error_init_termios;

	tty->driver_data = acm;

	return 0;

error_init_termios:
	tty_port_put(&acm->port);
	return retval;
}

static int acm_tty_open(struct tty_struct *tty, struct file *filp)
{
	printk(KERN_INFO "acm_tty_open called\n");
	struct acm *acm = tty->driver_data;

	return tty_port_open(&acm->port, tty, filp);
}

static void acm_port_dtr_rts(struct tty_port *port, int raise)
{
	printk(KERN_INFO "acm_port_dtr_rts called\n");
	struct acm *acm = container_of(port, struct acm, port);
	int val;
	int res;

	if (raise)
		val = ACM_CTRL_DTR | ACM_CTRL_RTS;
	else
		val = 0;

	/* FIXME: add missing ctrlout locking throughout driver */
	acm->ctrlout = val;

	res = acm_set_control(acm, val);
	if (res && (acm->ctrl_caps & USB_CDC_CAP_LINE))
		dev_err(&acm->control->dev, "failed to set dtr/rts\n");
}

static int acm_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	printk(KERN_INFO "acm_port_activate called\n");
	struct acm *acm = container_of(port, struct acm, port);
	int retval = -ENODEV;
	int i;

	mutex_lock(&acm->mutex);
	if (acm->disconnected)
		goto disconnected;

	retval = usb_autopm_get_interface(acm->control);
	if (retval)
		goto error_get_interface;

	/*
	 * FIXME: Why do we need this? Allocating 64K of physically contiguous
	 * memory is really nasty...
	 */
	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	acm->control->needs_remote_wakeup = 1;

	acm->ctrlurb->dev = acm->dev;
	retval = usb_submit_urb(acm->ctrlurb, GFP_KERNEL);
	if (retval) {
		dev_err(&acm->control->dev,
			"%s - usb_submit_urb(ctrl irq) failed\n", __func__);
		goto error_submit_urb;
	}

	acm_tty_set_termios(tty, NULL);

	/*
	 * Unthrottle device in case the TTY was closed while throttled.
	 */
	clear_bit(ACM_THROTTLED, &acm->flags);

	retval = acm_submit_read_urbs(acm, GFP_KERNEL);
	if (retval)
		goto error_submit_read_urbs;

	usb_autopm_put_interface(acm->control);

	mutex_unlock(&acm->mutex);

	return 0;

error_submit_read_urbs:
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->read_urbs[i]);
	usb_kill_urb(acm->ctrlurb);
error_submit_urb:
	usb_autopm_put_interface(acm->control);
error_get_interface:
disconnected:
	mutex_unlock(&acm->mutex);

	return usb_translate_errors(retval);
}

static void acm_port_destruct(struct tty_port *port)
{
	printk(KERN_INFO "acm_port_destruct called\n");
	struct acm *acm = container_of(port, struct acm, port);

	acm_release_minor(acm);
	usb_put_intf(acm->control);
	kfree(acm->country_codes);
	kfree(acm);
}

static void acm_port_shutdown(struct tty_port *port)
{
	printk(KERN_INFO "acm_port_shutdown called\n");
	struct acm *acm = container_of(port, struct acm, port);
	struct urb *urb;
	struct acm_wb *wb;

	/*
	 * Need to grab write_lock to prevent race with resume, but no need to
	 * hold it due to the tty-port initialised flag.
	 */
	spin_lock_irq(&acm->write_lock);
	spin_unlock_irq(&acm->write_lock);

	usb_autopm_get_interface_no_resume(acm->control);
	acm->control->needs_remote_wakeup = 0;
	usb_autopm_put_interface(acm->control);

	for (;;) {
		urb = usb_get_from_anchor(&acm->delayed);
		if (!urb)
			break;
		wb = urb->context;
		wb->use = false;
		usb_autopm_put_interface_async(acm->control);
	}

	acm_kill_urbs(acm);
}

static void acm_tty_cleanup(struct tty_struct *tty)
{
	printk(KERN_INFO "acm_tty_cleanup called\n");
	struct acm *acm = tty->driver_data;

	tty_port_put(&acm->port);
}

static void acm_tty_close(struct tty_struct *tty, struct file *filp)
{
	printk(KERN_INFO "acm_tty_close called\n");
	struct acm *acm = tty->driver_data;

	tty_port_close(&acm->port, tty, filp);
}

static int acm_tty_write(struct tty_struct *tty,
					const unsigned char *buf, int count)
{
	printk(KERN_INFO "acm_tty_write called\n");
	struct acm *acm = tty->driver_data;
	int stat;
	unsigned long flags;
	int wbn;
	struct acm_wb *wb;

	if (!count)
		return 0;

	dev_vdbg(&acm->data->dev, "%d bytes from tty layer\n", count);

	spin_lock_irqsave(&acm->write_lock, flags);
	wbn = acm_wb_alloc(acm);
	if (wbn < 0) {
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return 0;
	}
	wb = &acm->wb[wbn];

	if (!acm->dev) {
		wb->use = false;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return -ENODEV;
	}

	count = (count > acm->writesize) ? acm->writesize : count;
	dev_vdbg(&acm->data->dev, "writing %d bytes\n", count);
	memcpy(wb->buf, buf, count);
	wb->len = count;

	stat = usb_autopm_get_interface_async(acm->control);
	if (stat) {
		wb->use = false;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return stat;
	}

	if (acm->susp_count) {
		usb_anchor_urb(wb->urb, &acm->delayed);
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return count;
	}

	stat = acm_start_wb(acm, wb);
	spin_unlock_irqrestore(&acm->write_lock, flags);

	if (stat < 0)
		return stat;
	return count;
}

// static int acm_tty_write_room(struct tty_struct *tty)
// {
// 	printk(KERN_INFO "acm_tty_write_room called\n");
// 	struct acm *acm = tty->driver_data;
// 	/*
// 	 * Do not let the line discipline to know that we have a reserve,
// 	 * or it might get too enthusiastic.
// 	 */
// 	return acm_wb_is_avail(acm) ? acm->writesize : 0;
// }

// static int acm_tty_tiocmget(struct tty_struct *tty)
// {
// 	printk(KERN_INFO "acm_tty_tiocmget called\n");
// 	struct acm *acm = tty->driver_data;

// 	return (acm->ctrlout & ACM_CTRL_DTR ? TIOCM_DTR : 0) |
// 	       (acm->ctrlout & ACM_CTRL_RTS ? TIOCM_RTS : 0) |
// 	       (acm->ctrlin  & ACM_CTRL_DSR ? TIOCM_DSR : 0) |
// 	       (acm->ctrlin  & ACM_CTRL_RI  ? TIOCM_RI  : 0) |
// 	       (acm->ctrlin  & ACM_CTRL_DCD ? TIOCM_CD  : 0) |
// 	       TIOCM_CTS;
// }

// static int acm_tty_tiocmset(struct tty_struct *tty,
// 			    unsigned int set, unsigned int clear)
// {
// 	printk(KERN_INFO "acm_tty_tiocmset called\n");
// 	struct acm *acm = tty->driver_data;
// 	unsigned int newctrl;

// 	newctrl = acm->ctrlout;
// 	set = (set & TIOCM_DTR ? ACM_CTRL_DTR : 0) |
// 					(set & TIOCM_RTS ? ACM_CTRL_RTS : 0);
// 	clear = (clear & TIOCM_DTR ? ACM_CTRL_DTR : 0) |
// 					(clear & TIOCM_RTS ? ACM_CTRL_RTS : 0);

// 	newctrl = (newctrl & ~clear) | set;

// 	if (acm->ctrlout == newctrl)
// 		return 0;
// 	return acm_set_control(acm, acm->ctrlout = newctrl);
// }

// static int wait_serial_change(struct acm *acm, unsigned long arg)
// {
// 	printk(KERN_INFO "wait_serial_change called\n");
// 	int rv = 0;
// 	DECLARE_WAITQUEUE(wait, current);
// 	struct async_icount old, new;

// 	do {
// 		spin_lock_irq(&acm->read_lock);
// 		old = acm->oldcount;
// 		new = acm->iocount;
// 		acm->oldcount = new;
// 		spin_unlock_irq(&acm->read_lock);

// 		if ((arg & TIOCM_DSR) &&
// 			old.dsr != new.dsr)
// 			break;
// 		if ((arg & TIOCM_CD)  &&
// 			old.dcd != new.dcd)
// 			break;
// 		if ((arg & TIOCM_RI) &&
// 			old.rng != new.rng)
// 			break;

// 		add_wait_queue(&acm->wioctl, &wait);
// 		set_current_state(TASK_INTERRUPTIBLE);
// 		schedule();
// 		remove_wait_queue(&acm->wioctl, &wait);
// 		if (acm->disconnected) {
// 			if (arg & TIOCM_CD)
// 				break;
// 			else
// 				rv = -ENODEV;
// 		} else {
// 			if (signal_pending(current))
// 				rv = -ERESTARTSYS;
// 		}
// 	} while (!rv);



// 	return rv;
// }

static int acm_tty_ioctl(struct tty_struct *tty,
					unsigned int cmd, unsigned long arg)
{
	printk(KERN_INFO "acm_tty_ioctl called\n");
	struct acm *acm = tty->driver_data;
	int rv = -ENOIOCTLCMD;

	switch (cmd) {
	case TIOCMIWAIT:
		rv = usb_autopm_get_interface(acm->control);
		if (rv < 0) {
			rv = -EIO;
			break;
		}
		// rv = wait_serial_change(acm, arg);
		usb_autopm_put_interface(acm->control);
		break;
	}

	return rv;
}

static void acm_tty_set_termios(struct tty_struct *tty,
						struct ktermios *termios_old)
{
	printk(KERN_INFO "acm_tty_set_termios called\n");
	struct acm *acm = tty->driver_data;
	struct ktermios *termios = &tty->termios;
	struct usb_cdc_line_coding newline;
	int newctrl = acm->ctrlout;

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
	acm->clocal = ((termios->c_cflag & CLOCAL) != 0);

	if (C_BAUD(tty) == B0) {
		newline.dwDTERate = acm->line.dwDTERate;
		newctrl &= ~ACM_CTRL_DTR;
	} else if (termios_old && (termios_old->c_cflag & CBAUD) == B0) {
		newctrl |=  ACM_CTRL_DTR;
	}

	if (newctrl != acm->ctrlout)
		acm_set_control(acm, acm->ctrlout = newctrl);

	if (memcmp(&acm->line, &newline, sizeof newline)) {
		memcpy(&acm->line, &newline, sizeof newline);
		dev_dbg(&acm->control->dev, "%s - set line: %d %d %d %d\n",
			__func__,
			le32_to_cpu(newline.dwDTERate),
			newline.bCharFormat, newline.bParityType,
			newline.bDataBits);
		acm_set_line(acm, &acm->line);
	}
}

static const struct tty_port_operations acm_port_ops = {
	.dtr_rts = acm_port_dtr_rts,
	.shutdown = acm_port_shutdown,
	.activate = acm_port_activate,
	.destruct = acm_port_destruct,
};

/*
 * USB probe and disconnect routines.
 */

/* Little helpers: write/read buffers free */
static void acm_write_buffers_free(struct acm *acm)
{
	printk(KERN_INFO "acm_write_buffers_free called\n");
	int i;
	struct acm_wb *wb;

	for (wb = &acm->wb[0], i = 0; i < ACM_NW; i++, wb++)
		usb_free_coherent(acm->dev, acm->writesize, wb->buf, wb->dmah);
}

static void acm_read_buffers_free(struct acm *acm)
{
	printk(KERN_INFO "acm_read_buffers_free called\n");
	int i;

	for (i = 0; i < acm->rx_buflimit; i++)
		usb_free_coherent(acm->dev, acm->readsize,
			  acm->read_buffers[i].base, acm->read_buffers[i].dma);
}

/* Little helper: write buffers allocate */
static int acm_write_buffers_alloc(struct acm *acm)
{
	printk(KERN_INFO "acm_write_buffers_alloc called\n");
	int i;
	struct acm_wb *wb;

	for (wb = &acm->wb[0], i = 0; i < ACM_NW; i++, wb++) {
		wb->buf = usb_alloc_coherent(acm->dev, acm->writesize, GFP_KERNEL,
		    &wb->dmah);
		if (!wb->buf) {
			while (i != 0) {
				--i;
				--wb;
				usb_free_coherent(acm->dev, acm->writesize,
				    wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	return 0;
}

static int acm_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	printk(KERN_INFO "acm_probe called\n");
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
	struct acm *acm;
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

static void acm_disconnect(struct usb_interface *intf)
{
	printk(KERN_INFO "acm_disconnect called\n");
	struct acm *acm = usb_get_intfdata(intf);
	struct tty_struct *tty;
	int i;

	/* sibling interface is already cleaning up */
	if (!acm)
		return;

	mutex_lock(&acm->mutex);
	acm->disconnected = true;
	if (acm->country_codes) {
		device_remove_file(&acm->control->dev,
				&dev_attr_wCountryCodes);
		device_remove_file(&acm->control->dev,
				&dev_attr_iCountryCodeRelDate);
	}
	wake_up_all(&acm->wioctl);
	device_remove_file(&acm->control->dev, &dev_attr_bmCapabilities);
	usb_set_intfdata(acm->control, NULL);
	usb_set_intfdata(acm->data, NULL);
	mutex_unlock(&acm->mutex);

	tty = tty_port_tty_get(&acm->port);
	if (tty) {
		tty_vhangup(tty);
		tty_kref_put(tty);
	}

	acm_kill_urbs(acm);
	cancel_delayed_work_sync(&acm->dwork);

	tty_unregister_device(acm_tty_driver, acm->minor);

	usb_free_urb(acm->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(acm->wb[i].urb);
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_free_urb(acm->read_urbs[i]);
	acm_write_buffers_free(acm);
	usb_free_coherent(acm->dev, acm->ctrlsize, acm->ctrl_buffer, acm->ctrl_dma);
	acm_read_buffers_free(acm);

	kfree(acm->notification_buffer);

	if (!acm->combined_interfaces)
		usb_driver_release_interface(&acm_driver, intf == acm->control ?
					acm->data : acm->control);

	tty_port_put(&acm->port);
}

// #ifdef CONFIG_PM
// static int acm_suspend(struct usb_interface *intf, pm_message_t message)
// {
// 	printk(KERN_INFO "acm_suspend called\n");
// 	struct acm *acm = usb_get_intfdata(intf);
// 	int cnt;

// 	spin_lock_irq(&acm->write_lock);
// 	if (PMSG_IS_AUTO(message)) {
// 		if (acm->transmitting) {
// 			spin_unlock_irq(&acm->write_lock);
// 			return -EBUSY;
// 		}
// 	}
// 	cnt = acm->susp_count++;
// 	spin_unlock_irq(&acm->write_lock);

// 	if (cnt)
// 		return 0;

// 	acm_kill_urbs(acm);
// 	cancel_delayed_work_sync(&acm->dwork);
// 	acm->urbs_in_error_delay = 0;

// 	return 0;
// }

// static int acm_resume(struct usb_interface *intf)
// {
// 	printk(KERN_INFO "acm_resume called\n");
// 	struct acm *acm = usb_get_intfdata(intf);
// 	struct urb *urb;
// 	int rv = 0;

// 	spin_lock_irq(&acm->write_lock);

// 	if (--acm->susp_count)
// 		goto out;

// 	if (tty_port_initialized(&acm->port)) {
// 		rv = usb_submit_urb(acm->ctrlurb, GFP_ATOMIC);

// 		for (;;) {
// 			urb = usb_get_from_anchor(&acm->delayed);
// 			if (!urb)
// 				break;

// 			acm_start_wb(acm, urb->context);
// 		}

// 		/*
// 		 * delayed error checking because we must
// 		 * do the write path at all cost
// 		 */
// 		if (rv < 0)
// 			goto out;

// 		rv = acm_submit_read_urbs(acm, GFP_ATOMIC);
// 	}
// out:
// 	spin_unlock_irq(&acm->write_lock);

// 	return rv;
// }

// static int acm_reset_resume(struct usb_interface *intf)
// {
// 	printk(KERN_INFO "acm_reset_resume called\n");
// 	struct acm *acm = usb_get_intfdata(intf);

// 	if (tty_port_initialized(&acm->port))
// 		tty_port_tty_hangup(&acm->port, false);

// 	return acm_resume(intf);
// }

// #endif /* CONFIG_PM */

static const struct usb_device_id acm_ids[] = {
	{ USB_DEVICE(0x2341, 0x0043) },
	{ }
};

MODULE_DEVICE_TABLE(usb, acm_ids);

static struct usb_driver acm_driver = {
	.name =		"my_cdc_acm",
	.probe =	acm_probe,
	.disconnect =	acm_disconnect,
	.id_table =	acm_ids,
// #ifdef CONFIG_PM
// 	.suspend =	acm_suspend,
// 	.resume =	acm_resume,
// 	.reset_resume =	acm_reset_resume,
// 	.supports_autosuspend = 1,
// #endif
};

static const struct tty_operations acm_ops = {
	.install =		acm_tty_install,
	.open =			acm_tty_open,
	.close =		acm_tty_close,
	.cleanup =		acm_tty_cleanup,
	.write =		acm_tty_write,
	// .write_room =	acm_tty_write_room,
	.ioctl =		acm_tty_ioctl,
	.set_termios =	acm_tty_set_termios,
	// .tiocmget =		acm_tty_tiocmget,
	// .tiocmset =		acm_tty_tiocmset
};

static int __init acm_init(void)
{
	int retval;
	printk(KERN_INFO "THIS IS MY MODULE\n");
	acm_tty_driver = alloc_tty_driver(ACM_TTY_MINORS);
	if (!acm_tty_driver)
		return -ENOMEM;
	acm_tty_driver->driver_name = "acm",
	acm_tty_driver->name = "ttyACM",
	acm_tty_driver->major = ACM_TTY_MAJOR,
	acm_tty_driver->minor_start = 0,
	acm_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	acm_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	acm_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	// acm_tty_driver->init_termios = tty_std_termios;
	memset(&(acm_tty_driver->init_termios), 0, sizeof(struct termios));
	acm_tty_driver->init_termios.c_cflag |= (CREAD | CLOCAL);
	acm_tty_driver->init_termios.c_cflag &= ~CSIZE;
	acm_tty_driver->init_termios.c_cflag |= CS8;
	acm_tty_driver->init_termios.c_cflag &= ~CSTOPB;

	acm_tty_driver->init_termios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	acm_tty_driver->init_termios.c_iflag &= ~(INPCK);
	acm_tty_driver->init_termios.c_oflag &= ~OPOST;

	tty_set_operations(acm_tty_driver, &acm_ops);

	retval = tty_register_driver(acm_tty_driver);
	if (retval) {
		put_tty_driver(acm_tty_driver);
		return retval;
	}

	retval = usb_register(&acm_driver);
	if (retval) {
		tty_unregister_driver(acm_tty_driver);
		put_tty_driver(acm_tty_driver);
		return retval;
	}

	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_DESC "\n");

	return 0;
}

static void __exit acm_exit(void)
{
	usb_deregister(&acm_driver);
	tty_unregister_driver(acm_tty_driver);
	put_tty_driver(acm_tty_driver);
	idr_destroy(&acm_minors);
}

module_init(acm_init);
module_exit(acm_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(ACM_TTY_MAJOR);
