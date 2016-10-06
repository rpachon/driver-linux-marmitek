/*
 *  Marmitek remote x10
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/usb/input.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <media/rc-core.h>

/*
 * Module and Version Information, Module Parameters
 */

#define MARMITEK_REMOTE_VENDOR_ID		0x0bc7
#define MARMITEK_REMOTE_PRODUCT_ID		0x0005

#define DRIVER_VERSION		"2.2.1"
#define DRIVER_AUTHOR       "Remi Pachon <remi.pachon@gmail.com>"
#define DRIVER_DESC         "MARMITEK/X10 RF USB Remote Control"

#define NAME_BUFSIZE      80    /* size of product name, path buffers */
#define DATA_BUFSIZE      63    /* size of URB data buffers */

/*
 * Duplicate event filtering time.
 * Sequential, identical KIND_FILTERED inputs with less than
 * FILTER_TIME milliseconds between them are considered as repeat
 * events. The hardware generates 5 events for the first keypress
 * and we have to take this into account for an accurate repeat
 * behaviour.
 */
#define FILTER_TIME	60 /* msec */
#define REPEAT_DELAY	500 /* msec */

static unsigned long channel_mask;
module_param(channel_mask, ulong, 0644);
MODULE_PARM_DESC(channel_mask, "Bitmask of remote control channels to ignore");

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Enable extra debug messages and information");

static int repeat_filter = FILTER_TIME;
module_param(repeat_filter, int, 0644);
MODULE_PARM_DESC(repeat_filter, "Repeat filter time, default = 60 msec");

static int repeat_delay = REPEAT_DELAY;
module_param(repeat_delay, int, 0644);
MODULE_PARM_DESC(repeat_delay, "Delay before sending repeats, default = 500 msec");

static bool mouse = true;
module_param(mouse, bool, 0444);
MODULE_PARM_DESC(mouse, "Enable mouse device, default = yes");

#define dbginfo(dev, format, arg...) \
	do { if (debug) dev_info(dev , format , ## arg); } while (0)
#undef err
#define err(format, arg...) printk(KERN_ERR format , ## arg)

struct marmitek_receiver_type {
    /* either default_keymap or get_default_keymap should be set */
    const char *default_keymap;
    const char *(*get_default_keymap)(struct usb_interface *interface);
};

static const struct marmitek_receiver_type type_marmitek		= {
        .default_keymap = RC_MAP_MARMITEK_X10
};

static struct usb_device_id marmitek_remote_table[] = {
        {
                USB_DEVICE(MARMITEK_REMOTE_VENDOR_ID, MARMITEK_REMOTE_PRODUCT_ID),
                .driver_info = (unsigned long)&type_marmitek
        },
        {}	/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, marmitek_remote_table);

/* Get hi and low bytes of a 16-bits int */
#define HI(a)	((unsigned char)((a) >> 8))
#define LO(a)	((unsigned char)((a) & 0xff))

#define SEND_FLAG_IN_PROGRESS	1
#define SEND_FLAG_COMPLETE	2

/* Device initialization strings */
static char init1[] = { 0x01, 0x00, 0x20, 0x14 };
static char init2[] = { 0x01, 0x00, 0x20, 0x14, 0x20, 0x20, 0x20 };

struct marmitek_remote {
    struct input_dev *idev;
    struct rc_dev *rdev;
    struct usb_device *udev;
    struct usb_interface *interface;

    struct urb *irq_urb;
    struct urb *out_urb;
    struct usb_endpoint_descriptor *endpoint_in;
    struct usb_endpoint_descriptor *endpoint_out;
    unsigned char *inbuf;
    unsigned char *outbuf;
    dma_addr_t inbuf_dma;
    dma_addr_t outbuf_dma;

    int old_data;     /* Detect duplicate events */
    unsigned long old_jiffies;
    unsigned long acc_jiffies;  /* handle acceleration */
    unsigned long first_jiffies;

    unsigned int repeat_count;

    char rc_name[NAME_BUFSIZE];
    char rc_phys[NAME_BUFSIZE];
    char mouse_name[NAME_BUFSIZE];
    char mouse_phys[NAME_BUFSIZE];

    wait_queue_head_t wait;
    int send_flags;

    int users; /* 0-2, users are rc and input */
    struct mutex open_mutex;
};

/* "Kinds" of messages sent from the hardware to the driver. */
#define KIND_END        0
#define KIND_LITERAL    1   /* Simply pass to input system as EV_KEY */
#define KIND_FILTERED   2   /* Add artificial key-up events, drop keyrepeats */
#define KIND_ACCEL      3   /* Translate to EV_REL mouse-move events */

/* Translation table from hardware messages to input events. */
static const struct {
    unsigned char kind;
    unsigned char data;	/* Raw key code from remote */
    unsigned short code;	/* Input layer translation */
}  marmitek_remote_tbl[] = {
        /* Directional control pad axes.  Code is xxyy */
        {KIND_ACCEL,    0x70, 0xff00},	/* left */
        {KIND_ACCEL,    0x71, 0x0100},	/* right */
        {KIND_ACCEL,    0x72, 0x00ff},	/* up */
        {KIND_ACCEL,    0x73, 0x0001},	/* down */

        /* Directional control pad diagonals */
        {KIND_ACCEL,    0x74, 0xffff},	/* left up */
        {KIND_ACCEL,    0x75, 0x01ff},	/* right up */
        {KIND_ACCEL,    0x77, 0xff01},	/* left down */
        {KIND_ACCEL,    0x76, 0x0101},	/* right down */

        /* "Mouse button" buttons.  The code below uses the fact that the
         * lsbit of the raw code is a down/up indicator. */
        {KIND_LITERAL,  0x78, BTN_LEFT}, /* left btn down */
        {KIND_LITERAL,  0x79, BTN_LEFT}, /* left btn up */
        {KIND_LITERAL,  0x7c, BTN_RIGHT},/* right btn down */
        {KIND_LITERAL,  0x7d, BTN_RIGHT},/* right btn up */

        /* Artificial "doubleclick" events are generated by the hardware.
         * They are mapped to the "side" and "extra" mouse buttons here. */
        {KIND_FILTERED, 0x7a, BTN_SIDE}, /* left dblclick */
        {KIND_FILTERED, 0x7e, BTN_EXTRA},/* right dblclick */

        /* Non-mouse events are handled by rc-core */
        {KIND_END, 0x00, 0}
};

/*
 * marmitek_remote_dump_input
 */
static void marmitek_remote_dump(struct device *dev, unsigned char *data,
                                 unsigned int len)
{
    if (len == 1) {
        if (data[0] != (unsigned char)0xff && data[0] != 0x00)
            dev_warn(dev, "Weird byte 0x%02x\n", data[0]);
    }
    else if (len == 4)
        dev_warn(dev, "Weird key %*ph\n", 4, data);
    else if (len == 5)
        dev_warn(dev, "Weird key %*ph\n", 5, data);
    else
        dev_warn(dev, "Weird data, len=%d %*ph ...\n", len, 6, data);
}

/*
 * marmitek_remote_open
 */
static int marmitek_remote_open(struct marmitek_remote *marmitek_remote)
{
    int err = 0;

    mutex_lock(&marmitek_remote->open_mutex);

    if (marmitek_remote->users++ != 0)
        goto out; /* one was already active */

    /* On first open, submit the read urb which was set up previously. */
    marmitek_remote->irq_urb->dev = marmitek_remote->udev;
    if (usb_submit_urb(marmitek_remote->irq_urb, GFP_KERNEL)) {
        dev_err(&marmitek_remote->interface->dev,
                "%s: usb_submit_urb failed!\n", __func__);
        err = -EIO;
    }

    out:	mutex_unlock(&marmitek_remote->open_mutex);
    return err;
}

/*
 * marmitek_remote_close
 */
static void marmitek_remote_close(struct marmitek_remote *marmitek_remote)
{
    mutex_lock(&marmitek_remote->open_mutex);
    if (--marmitek_remote->users == 0)
        usb_kill_urb(marmitek_remote->irq_urb);
    mutex_unlock(&marmitek_remote->open_mutex);
}

static int marmitek_remote_input_open(struct input_dev *inputdev)
{
    struct marmitek_remote *marmitek_remote = input_get_drvdata(inputdev);
    return marmitek_remote_open(marmitek_remote);
}

static void marmitek_remote_input_close(struct input_dev *inputdev)
{
    struct marmitek_remote *marmitek_remote = input_get_drvdata(inputdev);
    marmitek_remote_close(marmitek_remote);
}

static int marmitek_remote_rc_open(struct rc_dev *rdev)
{
    struct marmitek_remote *marmitek_remote = rdev->priv;
    return marmitek_remote_open(marmitek_remote);
}

static void marmitek_remote_rc_close(struct rc_dev *rdev)
{
    struct marmitek_remote *marmitek_remote = rdev->priv;
    marmitek_remote_close(marmitek_remote);
}

/*
 * marmitek_remote_irq_out
 */
static void marmitek_remote_irq_out(struct urb *urb)
{
    struct marmitek_remote *marmitek_remote = urb->context;

    if (urb->status) {
        dev_dbg(&marmitek_remote->interface->dev, "%s: status %d\n",
                __func__, urb->status);
        return;
    }

    marmitek_remote->send_flags |= SEND_FLAG_COMPLETE;
    wmb();
    wake_up(&marmitek_remote->wait);
}

/*
 * marmitek_remote_sendpacket
 *
 * Used to send device initialization strings
 */
static int marmitek_remote_sendpacket(struct marmitek_remote *marmitek_remote, u16 cmd,
                                      unsigned char *data)
{
    int retval = 0;

    /* Set up out_urb */
    memcpy(marmitek_remote->out_urb->transfer_buffer + 1, data, LO(cmd));
    ((char *) marmitek_remote->out_urb->transfer_buffer)[0] = HI(cmd);

    marmitek_remote->out_urb->transfer_buffer_length = LO(cmd) + 1;
    marmitek_remote->out_urb->dev = marmitek_remote->udev;
    marmitek_remote->send_flags = SEND_FLAG_IN_PROGRESS;

    retval = usb_submit_urb(marmitek_remote->out_urb, GFP_ATOMIC);
    if (retval) {
        dev_dbg(&marmitek_remote->interface->dev,
                "sendpacket: usb_submit_urb failed: %d\n", retval);
        return retval;
    }

    wait_event_timeout(marmitek_remote->wait,
                       ((marmitek_remote->out_urb->status != -EINPROGRESS) ||
                        (marmitek_remote->send_flags & SEND_FLAG_COMPLETE)),
                       HZ);
    usb_kill_urb(marmitek_remote->out_urb);

    return retval;
}

/*
 * marmitek_remote_compute_accel
 *
 * Implements acceleration curve for directional control pad
 * If elapsed time since last event is > 1/4 second, user "stopped",
 * so reset acceleration. Otherwise, user is probably holding the control
 * pad down, so we increase acceleration, ramping up over two seconds to
 * a maximum speed.
 */
static int marmitek_remote_compute_accel(struct marmitek_remote *marmitek_remote)
{
    static const char accel[] = { 1, 2, 4, 6, 9, 13, 20 };
    unsigned long now = jiffies;
    int acc;

    if (time_after(now, marmitek_remote->old_jiffies + msecs_to_jiffies(250))) {
        acc = 1;
        marmitek_remote->acc_jiffies = now;
    }
    else if (time_before(now, marmitek_remote->acc_jiffies + msecs_to_jiffies(125)))
        acc = accel[0];
    else if (time_before(now, marmitek_remote->acc_jiffies + msecs_to_jiffies(250)))
        acc = accel[1];
    else if (time_before(now, marmitek_remote->acc_jiffies + msecs_to_jiffies(500)))
        acc = accel[2];
    else if (time_before(now, marmitek_remote->acc_jiffies + msecs_to_jiffies(1000)))
        acc = accel[3];
    else if (time_before(now, marmitek_remote->acc_jiffies + msecs_to_jiffies(1500)))
        acc = accel[4];
    else if (time_before(now, marmitek_remote->acc_jiffies + msecs_to_jiffies(2000)))
        acc = accel[5];
    else
        acc = accel[6];

    return acc;
}

/*
 * marmitek_remote_report_input
 */
static void marmitek_remote_input_report(struct urb *urb)
{
    struct marmitek_remote *marmitek_remote = urb->context;
    unsigned char *data= marmitek_remote->inbuf;
    struct input_dev *dev = marmitek_remote->idev;
    int index = -1;
    int remote_num;
    int scancode;
    u32 wheel_keycode = KEY_RESERVED;
    int i;

    /*
     * data[0] = 0x14
     * data[1] = data[2] + data[3] + 0xd5 (a checksum byte)
     * data[2] = the key code (with toggle bit in MSB with some models)
     * data[3] = channel << 4 (the low 4 bits must be zero)
     */

    if (urb->actual_length == 4) {
        /* Deal with strange looking inputs */
        if ( urb->actual_length != 4 || data[0] != 0x14 ||
             data[1] != (unsigned char)(data[2] + data[3] + 0xD5) ||
             (data[3] & 0x0f) != 0x00) {
            marmitek_remote_dump(&urb->dev->dev, data, urb->actual_length);
            return;
        }

        if (data[1] != ((data[2] + data[3] + 0xd5) & 0xff)) {
            dbginfo(&marmitek_remote->interface->dev,
                    "wrong checksum in input: %*ph\n", 4, data);
            return;
        }

        /* Mask unwanted remote channels.  */
        /* note: remote_num is 0-based, channel 1 on remote == 0 here */
        remote_num = (data[3] >> 4) & 0x0f;
        if (channel_mask & (1 << (remote_num + 1))) {
            dbginfo(&marmitek_remote->interface->dev,
                    "Masked input from channel 0x%02x: data %02x, "
                            "mask= 0x%02lx\n",
                    remote_num, data[2], channel_mask);
            return;
        }

        /*
         * MSB is a toggle code, though only used by some devices
         * (e.g. SnapStream Firefly)
         */
        scancode = data[2] & 0x7f;

        dbginfo(&marmitek_remote->interface->dev,
                "channel 0x%02x; key data %02x, scancode %02x\n",
                remote_num, data[2], scancode);

        if (scancode >= 0x70) {
            /*
             * This is either a mouse or scrollwheel event, depending on
             * the remote/keymap.
             * Get the keycode assigned to scancode 0x78/0x70. If it is
             * set, assume this is a scrollwheel up/down event.
             */
            wheel_keycode = rc_g_keycode_from_table(marmitek_remote->rdev,
                                                    scancode & 0x78);

            if (wheel_keycode == KEY_RESERVED) {
                /* scrollwheel was not mapped, assume mouse */

                /* Look up event code index in the mouse translation
                 * table.
                 */
                for (i = 0; marmitek_remote_tbl[i].kind != KIND_END; i++) {
                    if (scancode == marmitek_remote_tbl[i].data) {
                        index = i;
                        break;
                    }
                }
            }
        }

        if (index >= 0 && marmitek_remote_tbl[index].kind == KIND_LITERAL) {
            /*
             * The lsbit of the raw key code is a down/up flag.
             * Invert it to match the input layer's conventions.
             */
            input_event(dev, EV_KEY, marmitek_remote_tbl[index].code,
                        !(data[2] & 1));

            marmitek_remote->old_jiffies = jiffies;

        } else if (index < 0 || marmitek_remote_tbl[index].kind == KIND_FILTERED) {
            unsigned long now = jiffies;

            /* Filter duplicate events which happen "too close" together. */
            if (marmitek_remote->old_data == data[2] &&
                time_before(now, marmitek_remote->old_jiffies +
                                 msecs_to_jiffies(repeat_filter))) {
                marmitek_remote->repeat_count++;
            } else {
                marmitek_remote->repeat_count = 0;
                marmitek_remote->first_jiffies = now;
            }

            marmitek_remote->old_jiffies = now;

            /* Ensure we skip at least the 4 first duplicate events
             * (generated by a single keypress), and continue skipping
             * until repeat_delay msecs have passed.
             */
            if (marmitek_remote->repeat_count > 0 &&
                (marmitek_remote->repeat_count < 5 ||
                 time_before(now, marmitek_remote->first_jiffies +
                                  msecs_to_jiffies(repeat_delay))))
                return;

            if (index >= 0) {
                input_event(dev, EV_KEY, marmitek_remote_tbl[index].code, 1);
                input_event(dev, EV_KEY, marmitek_remote_tbl[index].code, 0);
            } else {
                /* Not a mouse event, hand it to rc-core. */
                int count = 1;

                if (wheel_keycode != KEY_RESERVED) {
                    /*
                     * This is a scrollwheel event, send the
                     * scroll up (0x78) / down (0x70) scancode
                     * repeatedly as many times as indicated by
                     * rest of the scancode.
                     */
                    count = (scancode & 0x07) + 1;
                    scancode &= 0x78;
                }

                while (count--) {
                    /*
                    * We don't use the rc-core repeat handling yet as
                    * it would cause ghost repeats which would be a
                    * regression for this driver.
                    */
                    rc_keydown_notimeout(marmitek_remote->rdev, RC_TYPE_OTHER,
                                         scancode, data[2]);
                    rc_keyup(marmitek_remote->rdev);
                }
                goto nosync;
            }

        } else if (marmitek_remote_tbl[index].kind == KIND_ACCEL) {
            signed char dx = marmitek_remote_tbl[index].code >> 8;
            signed char dy = marmitek_remote_tbl[index].code & 255;

            /*
             * Other event kinds are from the directional control pad, and
             * have an acceleration factor applied to them.  Without this
             * acceleration, the control pad is mostly unusable.
             */
            int acc = marmitek_remote_compute_accel(marmitek_remote);
            if (dx)
                input_report_rel(dev, REL_X, dx * acc);
            if (dy)
                input_report_rel(dev, REL_Y, dy * acc);
            marmitek_remote->old_jiffies = jiffies;

        } else {
            dev_dbg(&marmitek_remote->interface->dev, "marmitek_remote kind=%d\n",
                    marmitek_remote_tbl[index].kind);
            return;
        }
        input_sync(dev);
        nosync:
        marmitek_remote->old_data = data[2];
    }

        /*
         * data[0] = 0x20
         * data[1] = 0xEE
         * data[2] = 0x11 (data[1] + data[2] = 0xFF)
         * data[3] = scancode high
         * data[4] = scancode low (data[3] + data[4] = 0xFF)
         */

    else {
        /* Deal with strange looking inputs */
        if ((urb->actual_length != 5) || (data[0] != 0x20)) {
            marmitek_remote_dump(&urb->dev->dev, data, urb->actual_length);
            return;
        }

        if ((data[1] + data[2] != 0xFF) || (data[3] + data[4] != 0xFF)) {
            dbginfo(&marmitek_remote->interface->dev,
                    "wrong checksum in input: %*ph\n", 5, data);
            return;
        }

        scancode = data[3] * 0x100 + data[4];

        unsigned long now = jiffies;

        /* Filter duplicate events which happen "too close" together. */
        if (marmitek_remote->old_data == scancode &&
            time_before(now, marmitek_remote->old_jiffies +
                             msecs_to_jiffies(repeat_delay))) {
            marmitek_remote->repeat_count++;
        } else {
            marmitek_remote->repeat_count = 0;
            marmitek_remote->first_jiffies = now;
        }

        marmitek_remote->old_data = scancode;
        marmitek_remote->old_jiffies = now;

        /* Ensure we skip at least the 4 first duplicate events (generated
         * by a single keypress), and continue skipping until repeat_delay
         * msecs have passed
         */
        if (marmitek_remote->repeat_count > 0 &&
            (marmitek_remote->repeat_count < 5 ||
             time_before(now, marmitek_remote->first_jiffies +
                              msecs_to_jiffies(repeat_delay))))
            return;

        dbginfo(&marmitek_remote->interface->dev,
                "key data %02x %02x, scancode %02x\n",
                data[3], data[4], scancode);

        rc_keydown_notimeout(marmitek_remote->rdev, RC_TYPE_OTHER, scancode,
                             scancode);
        rc_keyup(marmitek_remote->rdev);
    }
}

/*
 * marmitek_remote_irq_in
 */
static void marmitek_remote_irq_in(struct urb *urb)
{
    struct marmitek_remote *marmitek_remote = urb->context;
    int retval;

    switch (urb->status) {
        case 0:			/* success */
            marmitek_remote_input_report(urb);
            break;
        case -ECONNRESET:	/* unlink */
        case -ENOENT:
        case -ESHUTDOWN:
            dev_dbg(&marmitek_remote->interface->dev,
                    "%s: urb error status, unlink?\n",
                    __func__);
            return;
        default:		/* error */
            dev_dbg(&marmitek_remote->interface->dev,
                    "%s: Nonzero urb status %d\n",
                    __func__, urb->status);
    }

    retval = usb_submit_urb(urb, GFP_ATOMIC);
    if (retval)
        dev_err(&marmitek_remote->interface->dev,
                "%s: usb_submit_urb()=%d\n",
                __func__, retval);
}

/*
 * marmitek_remote_alloc_buffers
 */
static int marmitek_remote_alloc_buffers(struct usb_device *udev,
                                         struct marmitek_remote *marmitek_remote)
{
    marmitek_remote->inbuf = usb_alloc_coherent(udev, DATA_BUFSIZE, GFP_ATOMIC,
                                                &marmitek_remote->inbuf_dma);
    if (!marmitek_remote->inbuf)
        return -1;

    marmitek_remote->outbuf = usb_alloc_coherent(udev, DATA_BUFSIZE, GFP_ATOMIC,
                                                 &marmitek_remote->outbuf_dma);
    if (!marmitek_remote->outbuf)
        return -1;

    marmitek_remote->irq_urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!marmitek_remote->irq_urb)
        return -1;

    marmitek_remote->out_urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!marmitek_remote->out_urb)
        return -1;

    return 0;
}

/*
 * marmitek_remote_free_buffers
 */
static void marmitek_remote_free_buffers(struct marmitek_remote *marmitek_remote)
{
    usb_free_urb(marmitek_remote->irq_urb);
    usb_free_urb(marmitek_remote->out_urb);

    usb_free_coherent(marmitek_remote->udev, DATA_BUFSIZE,
                      marmitek_remote->inbuf, marmitek_remote->inbuf_dma);

    usb_free_coherent(marmitek_remote->udev, DATA_BUFSIZE,
                      marmitek_remote->outbuf, marmitek_remote->outbuf_dma);
}

static void marmitek_remote_input_init(struct marmitek_remote *marmitek_remote)
{
    struct input_dev *idev = marmitek_remote->idev;
    int i;

    idev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REL);
    idev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) |
                                        BIT_MASK(BTN_RIGHT) | BIT_MASK(BTN_SIDE) | BIT_MASK(BTN_EXTRA);
    idev->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y);
    for (i = 0; marmitek_remote_tbl[i].kind != KIND_END; i++)
        if (marmitek_remote_tbl[i].kind == KIND_LITERAL ||
            marmitek_remote_tbl[i].kind == KIND_FILTERED)
            __set_bit(marmitek_remote_tbl[i].code, idev->keybit);

    input_set_drvdata(idev, marmitek_remote);

    idev->open = marmitek_remote_input_open;
    idev->close = marmitek_remote_input_close;

    idev->name = marmitek_remote->mouse_name;
    idev->phys = marmitek_remote->mouse_phys;

    usb_to_input_id(marmitek_remote->udev, &idev->id);
    idev->dev.parent = &marmitek_remote->interface->dev;
}

static void marmitek_remote_rc_init(struct marmitek_remote *marmitek_remote)
{
    struct rc_dev *rdev = marmitek_remote->rdev;

    rdev->priv = marmitek_remote;
    rdev->driver_type = RC_DRIVER_SCANCODE;
    rdev->allowed_protocols = RC_BIT_OTHER;
    rdev->driver_name = "marmitek_remote";

    rdev->open = marmitek_remote_rc_open;
    rdev->close = marmitek_remote_rc_close;

    rdev->input_name = marmitek_remote->rc_name;
    rdev->input_phys = marmitek_remote->rc_phys;

    usb_to_input_id(marmitek_remote->udev, &rdev->input_id);
    rdev->dev.parent = &marmitek_remote->interface->dev;
}

static int marmitek_remote_initialize(struct marmitek_remote *marmitek_remote)
{
    struct usb_device *udev = marmitek_remote->udev;
    int pipe, maxp;

    init_waitqueue_head(&marmitek_remote->wait);

    /* Set up irq_urb */
    pipe = usb_rcvintpipe(udev, marmitek_remote->endpoint_in->bEndpointAddress);
    maxp = usb_maxpacket(udev, pipe, usb_pipeout(pipe));
    maxp = (maxp > DATA_BUFSIZE) ? DATA_BUFSIZE : maxp;

    usb_fill_int_urb(marmitek_remote->irq_urb, udev, pipe, marmitek_remote->inbuf,
                     maxp, marmitek_remote_irq_in, marmitek_remote,
                     marmitek_remote->endpoint_in->bInterval);
    marmitek_remote->irq_urb->transfer_dma = marmitek_remote->inbuf_dma;
    marmitek_remote->irq_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

    /* Set up out_urb */
    pipe = usb_sndintpipe(udev, marmitek_remote->endpoint_out->bEndpointAddress);
    maxp = usb_maxpacket(udev, pipe, usb_pipeout(pipe));
    maxp = (maxp > DATA_BUFSIZE) ? DATA_BUFSIZE : maxp;

    usb_fill_int_urb(marmitek_remote->out_urb, udev, pipe, marmitek_remote->outbuf,
                     maxp, marmitek_remote_irq_out, marmitek_remote,
                     marmitek_remote->endpoint_out->bInterval);
    marmitek_remote->out_urb->transfer_dma = marmitek_remote->outbuf_dma;
    marmitek_remote->out_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

    /* send initialization strings */
    if ((marmitek_remote_sendpacket(marmitek_remote, 0x8004, init1)) ||
        (marmitek_remote_sendpacket(marmitek_remote, 0x8007, init2))) {
        dev_err(&marmitek_remote->interface->dev,
                "Initializing marmitek_remote hardware failed.\n");
        return -EIO;
    }

    return 0;
}

/*
 * marmitek_remote_probe
 */
static int marmitek_remote_probe(struct usb_interface *interface,
                                 const struct usb_device_id *id)
{
    struct usb_device *udev = interface_to_usbdev(interface);
    struct usb_host_interface *iface_host = interface->cur_altsetting;
    struct usb_endpoint_descriptor *endpoint_in, *endpoint_out;
    struct marmitek_receiver_type *type = (struct marmitek_receiver_type *)id->driver_info;
    struct marmitek_remote *marmitek_remote;
    struct input_dev *input_dev;
    struct rc_dev *rc_dev;
    int err = -ENOMEM;

    if (iface_host->desc.bNumEndpoints != 2) {
        err("%s: Unexpected desc.bNumEndpoints\n", __func__);
        return -ENODEV;
    }

    endpoint_in = &iface_host->endpoint[0].desc;
    endpoint_out = &iface_host->endpoint[1].desc;

    if (!usb_endpoint_is_int_in(endpoint_in)) {
        err("%s: Unexpected endpoint_in\n", __func__);
        return -ENODEV;
    }
    if (le16_to_cpu(endpoint_in->wMaxPacketSize) == 0) {
        err("%s: endpoint_in message size==0? \n", __func__);
        return -ENODEV;
    }

    marmitek_remote = kzalloc(sizeof (struct marmitek_remote), GFP_KERNEL);
    rc_dev = rc_allocate_device();
    if (!marmitek_remote || !rc_dev)
        goto exit_free_dev_rdev;

    /* Allocate URB buffers, URBs */
    if (marmitek_remote_alloc_buffers(udev, marmitek_remote))
        goto exit_free_buffers;

    marmitek_remote->endpoint_in = endpoint_in;
    marmitek_remote->endpoint_out = endpoint_out;
    marmitek_remote->udev = udev;
    marmitek_remote->rdev = rc_dev;
    marmitek_remote->interface = interface;

    usb_make_path(udev, marmitek_remote->rc_phys, sizeof(marmitek_remote->rc_phys));
    strlcpy(marmitek_remote->mouse_phys, marmitek_remote->rc_phys,
            sizeof(marmitek_remote->mouse_phys));

    strlcat(marmitek_remote->rc_phys, "/input0", sizeof(marmitek_remote->rc_phys));
    strlcat(marmitek_remote->mouse_phys, "/input1", sizeof(marmitek_remote->mouse_phys));

    if (udev->manufacturer)
        strlcpy(marmitek_remote->rc_name, udev->manufacturer,
                sizeof(marmitek_remote->rc_name));

    if (udev->product)
        snprintf(marmitek_remote->rc_name, sizeof(marmitek_remote->rc_name),
                 "%s %s", marmitek_remote->rc_name, udev->product);

    if (!strlen(marmitek_remote->rc_name))
        snprintf(marmitek_remote->rc_name, sizeof(marmitek_remote->rc_name),
                 DRIVER_DESC "(%04x,%04x)",
                 le16_to_cpu(marmitek_remote->udev->descriptor.idVendor),
                 le16_to_cpu(marmitek_remote->udev->descriptor.idProduct));

    snprintf(marmitek_remote->mouse_name, sizeof(marmitek_remote->mouse_name),
             "%s mouse", marmitek_remote->rc_name);

    rc_dev->map_name = RC_MAP_MARMITEK_X10; /* default map */

    /* set default keymap according to receiver model */
    if (type) {
        if (type->default_keymap)
            rc_dev->map_name = type->default_keymap;
        else if (type->get_default_keymap)
            rc_dev->map_name = type->get_default_keymap(interface);
    }

    marmitek_remote_rc_init(marmitek_remote);
    mutex_init(&marmitek_remote->open_mutex);

    /* Device Hardware Initialization - fills in marmitek_remote->idev from udev. */
    err = marmitek_remote_initialize(marmitek_remote);
    if (err)
        goto exit_kill_urbs;

    /* Set up and register rc device */
    err = rc_register_device(marmitek_remote->rdev);
    if (err)
        goto exit_kill_urbs;

    /* use our delay for rc_dev */
    marmitek_remote->rdev->input_dev->rep[REP_DELAY] = repeat_delay;

    /* Set up and register mouse input device */
    if (mouse) {
        input_dev = input_allocate_device();
        if (!input_dev) {
            err = -ENOMEM;
            goto exit_unregister_device;
        }

        marmitek_remote->idev = input_dev;
        marmitek_remote_input_init(marmitek_remote);
        err = input_register_device(input_dev);

        if (err)
            goto exit_free_input_device;
    }

    usb_set_intfdata(interface, marmitek_remote);
    return 0;

    exit_free_input_device:
    input_free_device(input_dev);
    exit_unregister_device:
    rc_unregister_device(rc_dev);
    rc_dev = NULL;
    exit_kill_urbs:
    usb_kill_urb(marmitek_remote->irq_urb);
    usb_kill_urb(marmitek_remote->out_urb);
    exit_free_buffers:
    marmitek_remote_free_buffers(marmitek_remote);
    exit_free_dev_rdev:
    rc_free_device(rc_dev);
    kfree(marmitek_remote);
    return err;
}

/*
 * marmitek_remote_disconnect
 */
static void marmitek_remote_disconnect(struct usb_interface *interface)
{
    struct marmitek_remote *marmitek_remote;

    marmitek_remote = usb_get_intfdata(interface);
    usb_set_intfdata(interface, NULL);
    if (!marmitek_remote) {
        dev_warn(&interface->dev, "%s - null device?\n", __func__);
        return;
    }

    usb_kill_urb(marmitek_remote->irq_urb);
    usb_kill_urb(marmitek_remote->out_urb);
    if (marmitek_remote->idev)
        input_unregister_device(marmitek_remote->idev);
    rc_unregister_device(marmitek_remote->rdev);
    marmitek_remote_free_buffers(marmitek_remote);
    kfree(marmitek_remote);
}

/* usb specific object to register with the usb subsystem */
static struct usb_driver marmitek_remote_driver = {
        .name         = "marmitek_remote",
        .probe        = marmitek_remote_probe,
        .disconnect   = marmitek_remote_disconnect,
        .id_table     = marmitek_remote_table,
};

module_usb_driver(marmitek_remote_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
