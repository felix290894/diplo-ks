/*
 * gps_usb.c
 *
 * Copyright (c) 2016 Victor Martinez	<mvictor619@live.com.mx>
 *
 * This work was derived form the cdc_acm driver, it was adjusted to work
 * with only one specific driver, GPS click of mikroe
 */

#define DEBUG
#define VERBOSE_DEBUG

#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/usb.h>
#include <linux/idr.h>

#define DRIVER_AUTHOR "Victor Martinez"
#define DRIVER_DESC "Test driver made for the GPS click of mikroe"


#define GPS_TTY_MAJOR		166
#define GPS_TTY_MINORS		256

#define GPS_NR			8

/* The usb and tty driver structures */
static struct usb_driver gps_driver;
static struct tty_driver *gps_tty_driver;


/* gps read buffer structure, used to fill the read callback
 * function and store information to be used in the callback function */
struct gps_rb {
	int			size;
	unsigned char		*base;
	dma_addr_t		dma;
	int			index;
	struct gps		*instance;
};

/* the main gps structure, needed by the tty functions to operate
 * and retrieve information needed to do their work */
struct gps {
	struct usb_device *dev;				/* the corresponding usb device */
	struct usb_interface *data;			/* data interface */
	struct usb_interface *control;			/* control interface */
	struct urb *read_urbs[GPS_NR];			/* the read URBs */
	struct gps_rb read_buffers[GPS_NR];		/* the read buffers */
	int rx_buflimit;				/* the number of read buffers */
	struct mutex mutex;				
	unsigned int readsize;				/* buffer sizes for freeing */
	unsigned int minor;				/* minor */
	struct tty_port port;			 	/* our tty port data */
};


/* IDR variable, used to store the gps structure by the minor number */
static DEFINE_IDR(acm_minors);
/* Glocal mutex, the mutex is declared and initialized */
static DEFINE_MUTEX(acm_minors_lock);

/*
 * acm_minors accessors
 */

/*
 * Look up an GPS structure by minor.
 */
static struct gps *acm_get_by_minor(unsigned int minor)
{
	struct gps *gps;

	mutex_lock(&acm_minors_lock);
	gps = idr_find(&acm_minors, minor);
	mutex_unlock(&acm_minors_lock);
	return gps;
}

/*
 * Try to find an available minor number and if found, associate it with 'gps'.
 */
static int acm_alloc_minor(struct gps *gps)
{
	int minor;

	mutex_lock(&acm_minors_lock);
	minor = idr_alloc(&acm_minors, gps, 0, GPS_TTY_MINORS, GFP_KERNEL);
	mutex_unlock(&acm_minors_lock);

	return minor;
}

/* Release the minor number associated with 'gps'.  */
static void acm_release_minor(struct gps *gps)
{
	mutex_lock(&acm_minors_lock);
	idr_remove(&acm_minors, gps->minor);
	mutex_unlock(&acm_minors_lock);
}

/*
 * Callbacks functions.
 */

/* Pass the data received from the device to the tty port */
static void acm_process_read_urb(struct gps *gps, struct urb *urb)
{
        if (!urb->actual_length)
                return;

	tty_insert_flip_string(&gps->port, urb->transfer_buffer,
			urb->actual_length);

	/* push the line flip buffer to the line dicipline */
	tty_flip_buffer_push(&gps->port);
}

/* Callback function to read the data receive from the bulk in endpoint */
static void gps_read_bulk_callback(struct urb *urb)
{
        struct gps_rb *rb = urb->context;
        struct gps *gps = rb->instance;
        int status = urb->status;

	// The device has been disconected */
        if (!gps->dev) {
                return;
        }

        if (status) {
                if ((status != -ENOENT) || (urb->actual_length == 0))
                        return;
        }

        acm_process_read_urb(gps, urb);

	
	// Resubmit the urb
        usb_submit_urb(gps->read_urbs[rb->index], GFP_ATOMIC);
}

/*
 * TTY handlers
 */

/* tty install function needed to initialice the data needed by the other tty functions */
static int gps_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct gps *gps;
	int retval;

	/* Get the gps structure by the minor number */
	gps = acm_get_by_minor(tty->index);
	if (!gps)
		return -ENODEV;

	/* Initialize termios among other tasks */
	retval = tty_standard_install(driver, tty);
	if (retval)
		goto error_init_termios;

	/* store the gps structure to use it in the other tty functions */
	tty->driver_data = gps;

	return 0;

error_init_termios:
	/* destruct the tty port */
	tty_port_put(&gps->port);
	return retval;
}

/* Function used by the tty port, when this is called the URB are registered */
static int gps_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct gps *gps = container_of(port, struct gps, port);
	int retval = -ENODEV;
	int i;

	mutex_lock(&gps->mutex);

	// submit the read urbs so the read callback can receive information
        for (i = 0; i < gps->rx_buflimit; ++i) {
        	if (usb_submit_urb(gps->read_urbs[i], GFP_KERNEL))
			goto error_submit_read_urbs;
        }

	mutex_unlock(&gps->mutex);

	return 0;

// If there is an error, all the urbs are killed
error_submit_read_urbs:
	for (i = 0; i < gps->rx_buflimit; i++)
		usb_kill_urb(gps->read_urbs[i]);
	mutex_unlock(&gps->mutex);

	return retval;
}

/* Function executed when the device node is opened */
static int gps_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct gps *gps = tty->driver_data;

	// Do the standar open, with this call the count is incremented,
	// the initialize function of the port is called
	return tty_port_open(&gps->port, tty, filp);
}

/* Function where the resourses are freed */
static void gps_port_destruct(struct tty_port *port)
{
	struct gps *gps = container_of(port, struct gps, port);

	// remove the data stored in the idr and free the gps struct
	acm_release_minor(gps);
	kfree(gps);
}

/* Function used by the tty port, this is called when the tty is being closed */
static void gps_port_shutdown(struct tty_port *port)
{
	struct gps *gps = container_of(port, struct gps, port);
	int i;

	// the urbs are killed here
	for (i = 0; i < gps->rx_buflimit; i++)
		usb_kill_urb(gps->read_urbs[i]);
}

/* Function called when the device node is closed */
static void gps_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct gps *gps = tty->driver_data;

	// Do the standart close, the shutdown function is called here
	tty_port_close(&gps->port, tty, filp);
}

/* Write function on the device node, is nos currently implemented */
static int gps_tty_write(struct tty_struct *tty,
					const unsigned char *buf, int count)
{
	if (!count)
		return 0;

	return count;
}

/*
 * TTY Port handlers
 */
static const struct tty_port_operations gps_port_ops = {
	.shutdown = gps_port_shutdown,
	.activate = gps_port_activate,
	.destruct = gps_port_destruct,
};



/*
 * USB driver functions operation.
 */

static int gps_probe(struct usb_interface *intf,
		     const struct usb_device_id *id)
{
	struct usb_interface *data_interface; // gps data interface
	struct usb_interface *control_interface; // gps control interface
	struct usb_endpoint_descriptor *epread = NULL; // read endpoint of the data interface
	struct usb_device *usb_dev = interface_to_usbdev(intf); // convert the usb_interface to usb_device
	struct gps *gps;
	int readsize;
	int num_rx_buf = GPS_NR; // number of read buffers
	int i;
	struct device *tty_dev;

	// Get the data and control interfaces
	// lsusb -v, show us the the interface number 1 is the data interface
	control_interface = usb_ifnum_to_if(usb_dev, 0);
	data_interface = usb_ifnum_to_if(usb_dev, 1);

	if (!control_interface || !data_interface) {
		return -ENODEV;
	}


	// Get the read endpoint of the data interface
	epread = &data_interface->cur_altsetting->endpoint[1].desc;

	/* Verify if the read endpoint is actually an IN endpoint */
	if (!usb_endpoint_dir_in(epread)) {
		epread = &data_interface->cur_altsetting->endpoint[0].desc;
	}

	// Allocate memory for the gps structure
	gps = kzalloc(sizeof(struct gps), GFP_KERNEL);
	if (gps == NULL)
		goto alloc_fail;


	// Get the max number of bytes that can be received
	readsize = usb_endpoint_maxp(epread);

	// Store this informatin to be used by the tty functions
	gps->data = data_interface;
	gps->control = control_interface;
	gps->dev = usb_dev;

	// Store the gps structure and get the index of where it was stored
	// We use this index as the minor number
	gps->minor = acm_alloc_minor(gps);
	if (gps->minor < 0)
		goto alloc_fail1;

	gps->readsize = readsize;
	gps->rx_buflimit = num_rx_buf;
	mutex_init(&gps->mutex);

	// Initialize the port structure
	tty_port_init(&gps->port);
	gps->port.ops = &gps_port_ops;

	// Register the callbacks for the IN data endpoint
	for (i = 0; i < num_rx_buf; i++) {
		struct gps_rb *rb = &(gps->read_buffers[i]);
		struct urb *urb;

		// Allocate a dma-consistent buffer
		rb->base = usb_alloc_coherent(usb_dev, readsize, GFP_KERNEL,
								&rb->dma);
		if (!rb->base)
			goto alloc_fail2;

		// Store the urb index, and store the gps structure to can be retrieve
		// with the container_of macro
		rb->index = i;
		rb->instance = gps;

		// Allocate the urb
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb)
			goto alloc_fail2;

		// the transfer_dma buffer is valid on submit
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = rb->dma;

		// Register the endpoint in the urb
		usb_fill_bulk_urb(urb, usb_dev,
				  usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress),
				  rb->base,
				  gps->readsize,
				  gps_read_bulk_callback, rb);

		// Store the urb on the gps structure
		gps->read_urbs[i] = urb;
	}

	// This is needed to associate the driver specific data with this interface
	usb_set_intfdata(intf, gps);

	// This is needed because this device has two interfaces, we only need to create one device node,
	usb_driver_claim_interface(&gps_driver, data_interface, gps);

	// Register the tty port
	tty_dev = tty_port_register_device(&gps->port, gps_tty_driver, gps->minor,
			&control_interface->dev);

	if (IS_ERR(tty_dev)) {
		goto alloc_fail3;
	}
	

	return 0;
// Manage the errors
alloc_fail3:
	usb_set_intfdata(intf, NULL);
alloc_fail2:
	for (i = 0; i < num_rx_buf; i++) {
		usb_free_urb(gps->read_urbs[i]);
                usb_free_coherent(usb_dev, gps->readsize,
                          gps->read_buffers[i].base, gps->read_buffers[i].dma);
	}
alloc_fail1:
	kfree(gps);
alloc_fail:
	return -ENODEV;
}

static void gps_disconnect(struct usb_interface *intf)
{
	struct gps *gps = usb_get_intfdata(intf);
	struct tty_struct *tty;
	int i;


	if (!gps)
		return;

	usb_set_intfdata(gps->data, NULL);

	tty = tty_port_tty_get(&gps->port);
	if (tty) {
		tty_vhangup(tty);
		// Release the reference to the tty
		tty_kref_put(tty);
	}
	
	// kill the urbs
	for (i = 0; i < gps->rx_buflimit; i++)
		usb_kill_urb(gps->read_urbs[i]);

	// unregister the tty driver
	tty_unregister_device(gps_tty_driver, gps->minor);

	// Free the memory allocated to manage the urbs
	for (i = 0; i < gps->rx_buflimit; i++) {
		usb_free_urb(gps->read_urbs[i]);
                usb_free_coherent(gps->dev, gps->readsize,
                          gps->read_buffers[i].base, gps->read_buffers[i].dma);
	}

	/* destruct the tty port */
	tty_port_put(&gps->port);
}

/*
 * USB driver structure.
 */

static const struct usb_device_id gps_ids[] = {
	{ USB_DEVICE(0x1546, 0x01A6) },
	{ }
};

MODULE_DEVICE_TABLE(usb, gps_ids);

static struct usb_driver gps_driver = {
	.name =		"gps_usb",
	.probe =	gps_probe,
	.disconnect =	gps_disconnect,
	.id_table =	gps_ids,
	.supports_autosuspend = 1,
	.disable_hub_initiated_lpm = 1,
};


/*
 * TTY driver structures.
 */

static const struct tty_operations gps_ops = {
	.install =		gps_tty_install,
	.open =			gps_tty_open,
	.close =		gps_tty_close,
	.write =		gps_tty_write,
};

/*
 * GPS init and exit functions.
 */

static int __init gps_init(void)
{
	int retval;

	// allicate memory for the tty structure, with the number of minors supported
	gps_tty_driver = alloc_tty_driver(GPS_TTY_MINORS);
	if (!gps_tty_driver)
		return -ENOMEM;

	// fill the tty structure to act as a serial
	gps_tty_driver->driver_name = "gps",
	gps_tty_driver->name = "ttyGPS",
	gps_tty_driver->major = GPS_TTY_MAJOR,
	gps_tty_driver->minor_start = 0,
	gps_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	gps_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	gps_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	gps_tty_driver->init_termios = tty_std_termios;
	gps_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD |
								HUPCL | CLOCAL;

	// set the operations supported by the tty
	tty_set_operations(gps_tty_driver, &gps_ops);

	retval = tty_register_driver(gps_tty_driver);
	if (retval) {
		// This function will destruct the tty driver
		put_tty_driver(gps_tty_driver);
		return retval;
	}

	// register the USB
	retval = usb_register(&gps_driver);
	if (retval) {
		tty_unregister_driver(gps_tty_driver);
		put_tty_driver(gps_tty_driver);
		return retval;
	}

	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_DESC "\n");

	return 0;
}

static void __exit gps_exit(void)
{
	usb_deregister(&gps_driver);
	tty_unregister_driver(gps_tty_driver);
	put_tty_driver(gps_tty_driver);
	idr_destroy(&acm_minors);
}

module_init(gps_init);
module_exit(gps_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(GPS_TTY_MAJOR);
