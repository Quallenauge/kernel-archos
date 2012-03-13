#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/ata.h>
#include <linux/hdreg.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/utsname.h>
#include <linux/sysfs.h>
#include <linux/pm_runtime.h>

#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <linux/usb/jm20329.h>
#include <linux/list.h>

#include "usb.h"
#include "transport.h"
#include "protocol.h"
#include "debug.h"
#include "scsiglue.h"
#include "../core/usb.h"

//#include <linux/usb.h>
#include <linux/usb/hcd.h>

static LIST_HEAD(jm20329_devices);

/*
 * The table of devices
 */
#define UNUSUAL_DEV(id_vendor, id_product, bcdDeviceMin, bcdDeviceMax, \
		    vendorName, productName, useProtocol, useTransport, \
		    initFunction, flags) \
{ USB_DEVICE_VER(id_vendor, id_product, bcdDeviceMin, bcdDeviceMax), \
  .driver_info = (flags)|(USB_US_TYPE_STOR<<24) }

struct usb_device_id jm20329_usb_ids[] = {
#	include "unusual_jm20329.h"
	{ }		/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, jm20329_usb_ids);

#undef UNUSUAL_DEV
#undef USUAL_DEV

/*
 * The flags table
 */
#define UNUSUAL_DEV(idVendor, idProduct, bcdDeviceMin, bcdDeviceMax, \
		    vendor_name, product_name, use_protocol, use_transport, \
		    init_function, Flags) \
{ \
	.vendorName = vendor_name,	\
	.productName = product_name,	\
	.useProtocol = use_protocol,	\
	.useTransport = use_transport,	\
	.initFunction = init_function,	\
}

static struct us_unusual_dev jm20329_unusual_dev_list[] = {
#	include "unusual_jm20329.h"
	{ }		/* Terminating entry */
};

static inline struct jm20329_device *find_device(const char *interface_string)
{
	struct jm20329_device *e;
	list_for_each_entry(e, &jm20329_devices, node) {
		if (e->pdrv->interface_string && !strcmp(interface_string, e->pdrv->interface_string))
			return e;
	}

	return NULL;
}
#define ATA_PASSTHROUGH_CMD_SIZE 12
#define JMICRON_ATA_PASSTHROUGH  0xDF
#define JM20329_SUSPEND_TIMEOUT 60000

static int bulk_cmd_data(struct us_data *us, u8 *cdb, int cdb_len, int is_write, u8 *buf, unsigned int data_len)
{
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	struct bulk_cs_wrap *bcs = (struct bulk_cs_wrap *) us->iobuf;
	unsigned int residue;
	unsigned int cswlen;
	unsigned int cbwlen = US_BULK_CB_WRAP_LEN;
	int result;
	int i;
	
	/* Take care of BULK32 devices; set extra byte to 0 */
	if ( unlikely(us->fflags & US_FL_BULK32)) {
		cbwlen = 32;
		us->iobuf[31] = 0;
	}

	/* set up the command wrapper */
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = cpu_to_le32(data_len);
	bcb->Flags = is_write ? 0 : 1 << 7;
	bcb->Tag = ++us->tag;
	bcb->Lun = 0; /* we don't know better */
	bcb->Length = cdb_len;

	/* copy the command payload */
	memset(bcb->CDB, 0, sizeof(bcb->CDB));
	memcpy(bcb->CDB, cdb, bcb->Length);

	/* send it to out endpoint */
	US_DEBUGP("Bulk Command S 0x%x T 0x%x L %d F %d Trg %d LUN %d CL %d\n",
			le32_to_cpu(bcb->Signature), bcb->Tag,
			le32_to_cpu(bcb->DataTransferLength), bcb->Flags,
			(bcb->Lun >> 4), (bcb->Lun & 0x0F), 
			bcb->Length);
			
	for (i = 0; i < bcb->Length; i++) {
		US_DEBUGPX(" %02x", bcb->CDB[i]);
	}
	US_DEBUGPX("\n");
	
	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe,
				bcb, cbwlen, NULL);
	if (result != USB_STOR_XFER_GOOD) {
		printk("Bulk command transfer result=%d\n", result);
		return USB_STOR_TRANSPORT_ERROR;
	}
	/* DATA STAGE */
	/* send/receive data payload, if there is any */

	if (data_len) {
		int act_len;
		unsigned int pipe = is_write ? us->send_bulk_pipe : us->recv_bulk_pipe;
		udelay(125);
		result = usb_stor_bulk_transfer_buf(us, pipe,
					buf, data_len, &act_len);
		US_DEBUGP("Bulk data transfer result 0x%x act_len %d\n", result, act_len);
		if (result == USB_STOR_XFER_ERROR || result == USB_STOR_XFER_LONG)
			return USB_STOR_TRANSPORT_ERROR;
	}

	/* get CSW for device status */
	US_DEBUGP("Attempting to get CSW...\n");
	result = usb_stor_bulk_transfer_buf(us, us->recv_bulk_pipe,
				bcs, US_BULK_CS_WRAP_LEN, &cswlen);

	/* Some broken devices add unnecessary zero-length packets to the
	 * end of their data transfers.  Such packets show up as 0-length
	 * CSWs.  If we encounter such a thing, try to read the CSW again.
	 */
	if (result == USB_STOR_XFER_SHORT && cswlen == 0) {
		US_DEBUGP("Received 0-length CSW; retrying...\n");
		result = usb_stor_bulk_transfer_buf(us, us->recv_bulk_pipe,
				bcs, US_BULK_CS_WRAP_LEN, &cswlen);
	}

	/* did the attempt to read the CSW fail? */
	if (result == USB_STOR_XFER_STALLED) {

		/* get the status again */
		US_DEBUGP("Attempting to get CSW (2nd try)...\n");
		result = usb_stor_bulk_transfer_buf(us, us->recv_bulk_pipe,
				bcs, US_BULK_CS_WRAP_LEN, NULL);
	}

	/* if we still have a failure at this point, we're in trouble */
	US_DEBUGP("Bulk status result = %d\n", result);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	/* check bulk status */
	residue = le32_to_cpu(bcs->Residue);
	US_DEBUGP("Bulk Status S 0x%x T 0x%x R %u Stat 0x%x\n",
			le32_to_cpu(bcs->Signature), bcs->Tag, 
			residue, bcs->Status);
	if (bcs->Tag != us->tag || bcs->Status > US_BULK_STAT_PHASE) {
		printk("Bulk logical error\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	/* Some broken devices report odd signatures, so we do not check them
	 * for validity against the spec. We store the first one we see,
	 * and check subsequent transfers for validity against this signature.
	 */
	if (!us->bcs_signature) {
		us->bcs_signature = bcs->Signature;
		if (us->bcs_signature != cpu_to_le32(US_BULK_CS_SIGN))
			US_DEBUGP("Learnt BCS signature 0x%08X\n",
					le32_to_cpu(us->bcs_signature));
	} else if (bcs->Signature != us->bcs_signature) {
		US_DEBUGP("Signature mismatch: got %08X, expecting %08X\n",
			  le32_to_cpu(bcs->Signature),
			  le32_to_cpu(us->bcs_signature));
		return USB_STOR_TRANSPORT_ERROR;
	}

	/* based on the status code, we report good or bad */
	switch (bcs->Status) {
	case US_BULK_STAT_OK:
		/* command good -- note that data could be short */
		return USB_STOR_TRANSPORT_GOOD;

	case US_BULK_STAT_FAIL:
		/* command failed */
		return USB_STOR_TRANSPORT_FAILED;

	case US_BULK_STAT_PHASE:
		/* phase error -- note that a transport reset will be
		 * invoked by the invoke_transport() function
		 */
		return USB_STOR_TRANSPORT_ERROR;
	}

	/* we should never get here, but if we do, we're in trouble */
	return USB_STOR_TRANSPORT_ERROR;
}

int bulk_cmd(struct us_data *us, u8 *cdb, int cdb_len, int is_write )
{
	return bulk_cmd_data( us, cdb, cdb_len, is_write, NULL, 0 );
}

/* Send an ATA command via the passthrough interface */
static int bulk_ata_passthrough_nondata(struct us_data *us, u8 *ata_cmd)
{
	u8 cmd_buf[ATA_PASSTHROUGH_CMD_SIZE];
	u16 data_len = 0;
		
	memset(cmd_buf, 0xFF, ATA_PASSTHROUGH_CMD_SIZE);
	
	cmd_buf[0] = JMICRON_ATA_PASSTHROUGH;
	cmd_buf[1] = 0x10;
	cmd_buf[3] = (data_len & 0xFF00) >> 8;
	cmd_buf[4] =  data_len & 0xFF;
	memcpy(&cmd_buf[5], ata_cmd, 7);

	return bulk_cmd(us, cmd_buf, ATA_PASSTHROUGH_CMD_SIZE, 0);
}

static int usb_stor_ata_sleep(struct us_data *us)
{
	u8 ata_cmd[7];
	
	US_DEBUGP("%s called\n", __FUNCTION__);

	memset(ata_cmd, 0, sizeof(ata_cmd));
	ata_cmd[6] = 0xE6; /* WIN_SLEEPNOW1 */
	
	return bulk_ata_passthrough_nondata(us, ata_cmd);
}

static inline int jm20329_wakeup(struct jm20329_device *dev)
{
	US_DEBUGP("-- %s\n", __func__);

	mutex_lock(&dev->dev_mutex);
	printk("%s\n", __func__);

	if (dev->pdrv->wakeup)
		dev->pdrv->wakeup(dev);

	if (dev->roothub)
		pm_runtime_get_sync(&dev->roothub->dev);

	dev->in_transition = JM20329_NONE;
	dev->state = JM20329_AWAKE;
	mutex_unlock(&dev->dev_mutex);
	return 0;
}

static inline int jm20329_do_sleep(struct jm20329_device *dev)
{
	US_DEBUGP("-- %s\n", __func__);

	mutex_lock(&dev->dev_mutex);

	printk("%s\n", __func__);

	if (dev->pdrv->disconnect)
		dev->pdrv->disconnect(dev);

	if (dev->roothub)
		pm_runtime_put_sync(&dev->roothub->dev);

	mutex_unlock(&dev->dev_mutex);

	return 0;
}

static bool jm20329_check_sleep_state(struct jm20329_device *dev)
{
	int i=0;
	mutex_lock(&dev->dev_mutex);
	if (dev->state == JM20329_ASLEEP) {
		while (dev->in_transition != JM20329_NONE) {
			mutex_unlock(&dev->dev_mutex);
			msleep(10);
			mutex_lock(&dev->dev_mutex);
			if ((i++>50) && (dev->roothub)) {
				printk("%s: USB disconnect missed? Triggering USB to find out.\n", __func__);
				i=0;
				pm_runtime_get_sync(&dev->roothub->dev);
				pm_runtime_put_sync(&dev->roothub->dev);
			}
		}
		dev->in_transition = JM20329_ONGOING;
		mutex_unlock(&dev->dev_mutex);
		jm20329_wakeup(dev);
		return true;
	} else if (dev->state == JM20329_DO_SLEEP) {
		dev->state = JM20329_ASLEEP;
		mutex_unlock(&dev->dev_mutex);
		jm20329_do_sleep(dev);
		return true;
	}
	mutex_unlock(&dev->dev_mutex);
	
	return false;
}

static void jm20329_suspend_resume_hook(struct us_data *us, int state)
{
	struct jm20329_device *jm20329 = us->extra;
	int result;

	US_DEBUGP("%s called\n", __FUNCTION__);

	if (state == US_SUSPEND) {
		mutex_lock(&jm20329->dev_mutex);

		jm20329->in_transition = JM20329_ONGOING;


		if (jm20329->state == JM20329_ASLEEP) {
			mutex_unlock(&jm20329->dev_mutex);
			return;
		}
		mutex_unlock(&jm20329->dev_mutex);

		//if (us->pusb_intf)
		//	usb_set_device_state(interface_to_usbdev(us->pusb_intf), USB_STATE_SUSPENDED);

		printk("Sending hdd to sleep\n");
		result = usb_stor_ata_sleep(us);
		if (result) {
			printk("%s: hdd sleep failed: %d\n", __func__, result);
		} else
			US_DEBUGP("Sent hdd to sleep\n");
		mutex_lock(&jm20329->dev_mutex);
		jm20329->state = JM20329_DO_SLEEP;
		mutex_unlock(&jm20329->dev_mutex);

		complete(&us->cmnd_ready);

	}
}

static int jm20329_control_thread(void * __us)
{
	struct us_data *us = (struct us_data *)__us;
	struct Scsi_Host *host = us_to_host(us);
	struct jm20329_device *dev = us->extra;
	long oldtime, time = jiffies_to_msecs(jiffies);
	long tmp;
	int was_in_suspend = 0;
	printk("JM20329 control thread started\n");

	mutex_lock(&dev->dev_mutex);
	dev->timeout = dev->timeout_reset;
	mutex_unlock(&dev->dev_mutex);

	for(;;) {
		US_DEBUGP("*** thread sleeping.\n");
		tmp = wait_for_completion_interruptible_timeout(&us->cmnd_ready, 100);
		if (tmp<0) 	// interrupted
			break;
		if (tmp == 0) {	// timed out			
			oldtime = time;
			time = jiffies_to_msecs(jiffies);
			mutex_lock(&dev->dev_mutex);
			if ((dev->state == JM20329_ASLEEP) || (dev->in_transition) || (was_in_suspend)) {
				dev->timeout = dev->timeout_reset;
				mutex_unlock(&dev->dev_mutex);
				// We don't count time wasted while sleeping
				oldtime = time;
			} else {
				if (dev->timeout<(time-oldtime)) {
					dev->timeout = dev->timeout_reset;
					dev->in_transition = JM20329_REQUESTED;
					mutex_unlock(&dev->dev_mutex);
					jm20329_suspend_resume_hook(dev->us, US_SUSPEND);
					was_in_suspend = 1;
					kobject_uevent(&dev->pdev->dev.kobj, KOBJ_OFFLINE);
				} else {
					dev->timeout -= (time-oldtime);
					mutex_unlock(&dev->dev_mutex);
				}
			}
			continue;	// Go and wait some more
		}

		if (was_in_suspend) {
			if (was_in_suspend == 2)
				kobject_uevent(&dev->pdev->dev.kobj,KOBJ_CHANGE);
			was_in_suspend++;
		}

		mutex_lock(&dev->dev_mutex);
		if (dev->state == JM20329_RELEASE) {	// We are done here
			mutex_unlock(&dev->dev_mutex);
			US_DEBUGP("-- exiting\n");
			break;
		}

		mutex_unlock(&dev->dev_mutex);

		US_DEBUGP("*** thread awakened.\n");

		if (jm20329_check_sleep_state(dev)) {
			mutex_lock(&dev->dev_mutex);
			time = jiffies_to_msecs(jiffies);
			oldtime = time;
			dev->timeout = dev->timeout_reset;	// We are working so let's reset the timeout
			mutex_unlock(&dev->dev_mutex);
			continue;
		}

		/* lock the device pointers */
		mutex_lock(&(us->dev_mutex));

		/* lock access to the state */
		scsi_lock(host);

		/* When we are called with no command pending, we're done */
		if (us->srb == NULL) {
			scsi_unlock(host);
			mutex_unlock(&us->dev_mutex);
			US_DEBUGP("-- exiting\n");
			break;
		}

		/* has the command timed out *already* ? */
		if (test_bit(US_FLIDX_TIMED_OUT, &us->dflags)) {
			us->srb->result = DID_ABORT << 16;
			goto SkipForAbort;
		}

		scsi_unlock(host);

		/* reject the command if the direction indicator 
		 * is UNKNOWN
		 */
		if (us->srb->sc_data_direction == DMA_BIDIRECTIONAL) {
			US_DEBUGP("UNKNOWN data direction\n");
			us->srb->result = DID_ERROR << 16;
		}

		/* reject if target != 0 or if LUN is higher than
		 * the maximum known LUN
		 */
		else if (us->srb->device->id && 
				!(us->fflags & US_FL_SCM_MULT_TARG)) {
			US_DEBUGP("Bad target number (%d:%d)\n",
				  us->srb->device->id, us->srb->device->lun);
			us->srb->result = DID_BAD_TARGET << 16;
		}

		else if (us->srb->device->lun > us->max_lun) {
			US_DEBUGP("Bad LUN (%d:%d)\n",
				  us->srb->device->id, us->srb->device->lun);
			us->srb->result = DID_BAD_TARGET << 16;
		}

		/* Handle those devices which need us to fake 
		 * their inquiry data */
		else if ((us->srb->cmnd[0] == INQUIRY) &&
			    (us->fflags & US_FL_FIX_INQUIRY)) {
			unsigned char data_ptr[36] = {
			    0x00, 0x80, 0x02, 0x02,
			    0x1F, 0x00, 0x00, 0x00};

			US_DEBUGP("Faking INQUIRY command\n");
			fill_inquiry_response(us, data_ptr, 36);
			us->srb->result = SAM_STAT_GOOD;
		}

		/* we've got a command, let's do it! */
		else {
			US_DEBUG(usb_stor_show_command(us->srb));
			us->proto_handler(us->srb, us);
		}

		/* lock access to the state */
		scsi_lock(host);

		/* indicate that the command is done */
		if (us->srb->result != DID_ABORT << 16) {
			US_DEBUGP("scsi cmd done, result=0x%x\n", 
				   us->srb->result);
			us->srb->scsi_done(us->srb);
		} else {
SkipForAbort:
			US_DEBUGP("scsi command aborted\n");
		}

		/* If an abort request was received we need to signal that
		 * the abort has finished.  The proper test for this is
		 * the TIMED_OUT flag, not srb->result == DID_ABORT, because
		 * the timeout might have occurred after the command had
		 * already completed with a different result code. */
		if (test_bit(US_FLIDX_TIMED_OUT, &us->dflags)) {
			complete(&(us->notify));

			/* Allow USB transfers to resume */
			clear_bit(US_FLIDX_ABORTING, &us->dflags);
			clear_bit(US_FLIDX_TIMED_OUT, &us->dflags);
		}

		/* finished working on this command */
		us->srb = NULL;
		scsi_unlock(host);

		/* unlock the device pointers */
		mutex_unlock(&us->dev_mutex);

		if (was_in_suspend) {
			kobject_uevent(&dev->pdev->dev.kobj,KOBJ_ONLINE);
			was_in_suspend = 0;
		}
		
		mutex_lock(&dev->dev_mutex);
		time = jiffies_to_msecs(jiffies);
		oldtime = time;
		dev->timeout = dev->timeout_reset;	// We are working so let's reset the timeout
		mutex_unlock(&dev->dev_mutex);

	} /* for (;;) */
	/* Wait until we are told to stop */
	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (kthread_should_stop())
			break;
		schedule();
	}
	__set_current_state(TASK_RUNNING);
	return 0;
}	

static void jm20329_usb_disconnect(struct usb_interface *intf)
{
	struct us_data *us = usb_get_intfdata(intf);
	struct jm20329_device *jm20329 = us->extra;

	US_DEBUGP("%s called\n", __FUNCTION__);

	if (jm20329) {
		mutex_lock(&jm20329->dev_mutex);
		if (jm20329->state == JM20329_ASLEEP) {
			/* neded to because of hub internal debouncing,
			 * an immediate resume could otherwise be ignored
			 */
			msleep(100);
			jm20329->in_transition = JM20329_NONE;
			mutex_unlock(&jm20329->dev_mutex);
			usb_set_device_state(interface_to_usbdev(intf), USB_STATE_SUSPENDED);
			jm20329->us->pusb_dev = NULL;
			return;
		} else {
			mutex_unlock(&jm20329->dev_mutex);
			printk("Warning: final disconnect\n");
			jm20329->us->pusb_dev = NULL;
		}
	}
	
	usb_stor_disconnect(intf);
}

/* Initialize all the dynamic resources we need */
static int jm20329_acquire_resources(struct us_data *us)
{
	int p;
	struct task_struct *th;

	us->current_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!us->current_urb) {
		US_DEBUGP("URB allocation failed\n");
		return -ENOMEM;
	}

	/* Just before we start our control thread, initialize
	 * the device if it needs initialization */
	if (us->unusual_dev->initFunction) {
		p = us->unusual_dev->initFunction(us);
		if (p)
			return p;
	}

	/* Start up our control thread */
	th = kthread_run(jm20329_control_thread, us, "usb-storage");
	if (IS_ERR(th)) {
		dev_warn(&us->pusb_intf->dev,
				"Unable to start control thread\n");
		return PTR_ERR(th);
	}
	us->ctl_thread = th;

	return 0;
}

/* Release all our dynamic resources */
static void jm20329_release_resources(struct us_data *us)
{
	struct jm20329_device* dev = us->extra;
	US_DEBUGP("-- %s\n", __func__);

	/* Tell the control thread to exit.  The SCSI host must
	 * already have been removed and the DISCONNECTING flag set
	 * so that we won't accept any more commands.
	 */
	US_DEBUGP("-- sending exit command to thread\n");
	mutex_lock(&dev->dev_mutex);
	dev->state=JM20329_RELEASE;
	mutex_unlock(&dev->dev_mutex);
	complete(&us->cmnd_ready);
	if (us->ctl_thread)
		kthread_stop(us->ctl_thread);

	/* Call the destructor routine, if it exists */
	if (us->extra_destructor) {
		US_DEBUGP("-- calling extra_destructor()\n");
		us->extra_destructor(us->extra);
	}

	/* Free the extra data and the URB */
	kfree(us->extra);
	usb_free_urb(us->current_urb);
}

/* Dissociate from the USB device */
static void jm20329_dissociate_dev(struct us_data *us)
{
	US_DEBUGP("-- %s\n", __func__);

	/* Free the buffers */
	kfree(us->cr);
	usb_free_coherent(us->pusb_dev, US_IOBUF_SIZE, us->iobuf, us->iobuf_dma);

	/* Remove our private data from the interface */
	usb_set_intfdata(us->pusb_intf, NULL);
}

/* Associate our private data with the USB device */
static int jm20329_associate_dev(struct us_data *us, struct usb_interface *intf)
{
	US_DEBUGP("-- %s\n", __func__);

	/* Fill in the device-related fields */
	us->pusb_dev = interface_to_usbdev(intf);
	us->pusb_intf = intf;
	us->ifnum = intf->cur_altsetting->desc.bInterfaceNumber;
	US_DEBUGP("Vendor: 0x%04x, Product: 0x%04x, Revision: 0x%04x\n",
			le16_to_cpu(us->pusb_dev->descriptor.idVendor),
			le16_to_cpu(us->pusb_dev->descriptor.idProduct),
			le16_to_cpu(us->pusb_dev->descriptor.bcdDevice));
	US_DEBUGP("Interface Subclass: 0x%02x, Protocol: 0x%02x\n",
			intf->cur_altsetting->desc.bInterfaceSubClass,
			intf->cur_altsetting->desc.bInterfaceProtocol);

	/* Store our private data in the interface */
	usb_set_intfdata(intf, us);

	if (!us->cr) {
		/* Allocate the control/setup and DMA-mapped buffers */
		us->cr = kmalloc(sizeof(*us->cr), GFP_KERNEL);
		if (!us->cr) {
			US_DEBUGP("usb_ctrlrequest allocation failed\n");
			return -ENOMEM;
		}

		us->iobuf = usb_alloc_coherent(us->pusb_dev, US_IOBUF_SIZE,
				GFP_KERNEL, &us->iobuf_dma);
		if (!us->iobuf) {
			US_DEBUGP("I/O buffer allocation failed\n");
			return -ENOMEM;
		}
	}
	return 0;
}

/* First stage of disconnect processing: stop SCSI scanning,
 * remove the host, and stop accepting new commands
 */
static void jm20329_quiesce_and_remove_host(struct us_data *us)
{
	struct Scsi_Host *host = us_to_host(us);

	/* If the device is really gone, cut short reset delays */
	if (us->pusb_dev->state == USB_STATE_NOTATTACHED)
		set_bit(US_FLIDX_DISCONNECTING, &us->dflags);

	/* Prevent SCSI-scanning (if it hasn't started yet)
	 * and wait for the SCSI-scanning thread to stop.
	 */
	set_bit(US_FLIDX_DONT_SCAN, &us->dflags);
	wake_up(&us->delay_wait);
	wait_for_completion(&us->scanning_done);

	/* Removing the host will perform an orderly shutdown: caches
	 * synchronized, disks spun down, etc.
	 */
	scsi_remove_host(host);

	/* Prevent any new commands from being accepted and cut short
	 * reset delays.
	 */
	scsi_lock(host);
	set_bit(US_FLIDX_DISCONNECTING, &us->dflags);
	scsi_unlock(host);
	wake_up(&us->delay_wait);
}

/* Second stage of disconnect processing: deallocate all resources */
static void jm20329_release_everything(struct us_data *us)
{
printk("jm20329_release_everything\n");
	jm20329_release_resources(us);
	jm20329_dissociate_dev(us);
}

/* Get the unusual_devs entries and the string descriptors */
static int jm20329_get_device_info(struct us_data *us, const struct usb_device_id *id,
		struct us_unusual_dev *unusual_dev)
{
	struct usb_device *dev = us->pusb_dev;
	struct usb_interface_descriptor *idesc =
		&us->pusb_intf->cur_altsetting->desc;
	struct device *pdev = &us->pusb_intf->dev;

	/* Store the entries */
	us->unusual_dev = unusual_dev;
	us->subclass = (unusual_dev->useProtocol == USB_SC_DEVICE) ?
			idesc->bInterfaceSubClass :
			unusual_dev->useProtocol;
	us->protocol = (unusual_dev->useTransport == USB_PR_DEVICE) ?
			idesc->bInterfaceProtocol :
			unusual_dev->useTransport;
	us->fflags = USB_US_ORIG_FLAGS(id->driver_info);

	if (us->fflags & US_FL_IGNORE_DEVICE) {
		dev_info(pdev, "device ignored\n");
		return -ENODEV;
	}

	/*
	 * This flag is only needed when we're in high-speed, so let's
	 * disable it if we're in full-speed
	 */
	if (dev->speed != USB_SPEED_HIGH)
		us->fflags &= ~US_FL_GO_SLOW;

	if (us->fflags)
		dev_info(pdev, "Quirks match for vid %04x pid %04x: %lx\n",
				le16_to_cpu(dev->descriptor.idVendor),
				le16_to_cpu(dev->descriptor.idProduct),
				us->fflags);

	/* Log a message if a non-generic unusual_dev entry contains an
	 * unnecessary subclass or protocol override.  This may stimulate
	 * reports from users that will help us remove unneeded entries
	 * from the unusual_devs.h table.
	 */
	if (id->idVendor || id->idProduct) {
		static const char *msgs[3] = {
			"an unneeded SubClass entry",
			"an unneeded Protocol entry",
			"unneeded SubClass and Protocol entries"};
		struct usb_device_descriptor *ddesc = &dev->descriptor;
		int msg = -1;

		if (unusual_dev->useProtocol != USB_SC_DEVICE &&
			us->subclass == idesc->bInterfaceSubClass)
			msg += 1;
		if (unusual_dev->useTransport != USB_PR_DEVICE &&
			us->protocol == idesc->bInterfaceProtocol)
			msg += 2;
		if (msg >= 0 && !(us->fflags & US_FL_NEED_OVERRIDE))
			dev_notice(pdev, "This device "
					"(%04x,%04x,%04x S %02x P %02x)"
					" has %s in unusual_devs.h (kernel"
					" %s)\n"
					"   Please send a copy of this message to "
					"<linux-usb@vger.kernel.org> and "
					"<usb-storage@lists.one-eyed-alien.net>\n",
					le16_to_cpu(ddesc->idVendor),
					le16_to_cpu(ddesc->idProduct),
					le16_to_cpu(ddesc->bcdDevice),
					idesc->bInterfaceSubClass,
					idesc->bInterfaceProtocol,
					msgs[msg],
					utsname()->release);
	}

	return 0;
}

/* Get the pipe settings */
static int jm20329_get_pipes(struct us_data *us)
{
	struct usb_host_interface *altsetting =
		us->pusb_intf->cur_altsetting;
	int i;
	struct usb_endpoint_descriptor *ep;
	struct usb_endpoint_descriptor *ep_in = NULL;
	struct usb_endpoint_descriptor *ep_out = NULL;
	struct usb_endpoint_descriptor *ep_int = NULL;

	/*
	 * Find the first endpoint of each type we need.
	 * We are expecting a minimum of 2 endpoints - in and out (bulk).
	 * An optional interrupt-in is OK (necessary for CBI protocol).
	 * We will ignore any others.
	 */
	for (i = 0; i < altsetting->desc.bNumEndpoints; i++) {
		ep = &altsetting->endpoint[i].desc;

		if (usb_endpoint_xfer_bulk(ep)) {
			if (usb_endpoint_dir_in(ep)) {
				if (!ep_in)
					ep_in = ep;
			} else {
				if (!ep_out)
					ep_out = ep;
			}
		}

		else if (usb_endpoint_is_int_in(ep)) {
			if (!ep_int)
				ep_int = ep;
		}
	}

	if (!ep_in || !ep_out || (us->protocol == USB_PR_CBI && !ep_int)) {
		US_DEBUGP("Endpoint sanity check failed! Rejecting dev.\n");
		return -EIO;
	}

	/* Calculate and store the pipe values */
	us->send_ctrl_pipe = usb_sndctrlpipe(us->pusb_dev, 0);
	us->recv_ctrl_pipe = usb_rcvctrlpipe(us->pusb_dev, 0);
	us->send_bulk_pipe = usb_sndbulkpipe(us->pusb_dev,
		usb_endpoint_num(ep_out));
	us->recv_bulk_pipe = usb_rcvbulkpipe(us->pusb_dev, 
		usb_endpoint_num(ep_in));
	if (ep_int) {
		us->recv_intr_pipe = usb_rcvintpipe(us->pusb_dev,
			usb_endpoint_num(ep_int));
		us->ep_bInterval = ep_int->bInterval;
	}
	return 0;
}

static unsigned int delay_use = 1;
/* Thread to carry out delayed SCSI-device scanning */
static int jm20329_usb_stor_scan_thread(void * __us)
{
	struct us_data *us = (struct us_data *)__us;
	struct device *dev = &us->pusb_intf->dev;

	dev_dbg(dev, "device found\n");

	set_freezable();
	/* Wait for the timeout to expire or for a disconnect */
	if (delay_use > 0) {
		dev_dbg(dev, "waiting for device to settle "
				"before scanning\n");
		wait_event_freezable_timeout(us->delay_wait,
				test_bit(US_FLIDX_DONT_SCAN, &us->dflags),
				delay_use * HZ);
	}

	/* If the device is still connected, perform the scanning */
	if (!test_bit(US_FLIDX_DONT_SCAN, &us->dflags)) {

		/* For bulk-only devices, determine the max LUN value */
		if (us->protocol == USB_PR_BULK &&
				!(us->fflags & US_FL_SINGLE_LUN)) {
			mutex_lock(&us->dev_mutex);
			us->max_lun = usb_stor_Bulk_max_lun(us);
			mutex_unlock(&us->dev_mutex);
		}
		scsi_scan_host(us_to_host(us));
		dev_dbg(dev, "scan complete\n");

		/* Should we unbind if no devices were detected? */
	}

	complete_and_exit(&us->scanning_done, 0);
}

/* Second part of general USB mass-storage probing */
int jm20329_usb_stor_probe2(struct us_data *us)
{
	struct task_struct *th;
	int result;
	struct device *dev = &us->pusb_intf->dev;

	/* fix for single-lun devices */
	if (us->fflags & US_FL_SINGLE_LUN)
		us->max_lun = 0;

	/* Acquire all the other resources and add the host */
	result = jm20329_acquire_resources(us);
	if (result)
		goto BadDevice;
	snprintf(us->scsi_name, sizeof(us->scsi_name), "usb-storage %s",
					dev_name(&us->pusb_intf->dev));
	result = scsi_add_host(us_to_host(us), dev);
	if (result) {
		dev_warn(dev,
				"Unable to add the scsi host\n");
		goto BadDevice;
	}

	/* Start up the thread for delayed SCSI-device scanning */
	th = kthread_create(jm20329_usb_stor_scan_thread, us, "usb-stor-scan");
	if (IS_ERR(th)) {
		dev_warn(dev,
				"Unable to start the device-scanning thread\n");
		complete(&us->scanning_done);
		jm20329_quiesce_and_remove_host(us);
		result = PTR_ERR(th);
		goto BadDevice;
	}

	wake_up_process(th);

	return 0;

	/* We come here if there are any problems */
BadDevice:
	printk("storage_probe() failed\n");
	jm20329_release_everything(us);
	return result;
}

/* The main probe routine for removable nonspecific devices */
/* mainly copied from storage/usb.c */
static int jm20329_removable_connect(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	struct us_data *us;
	int result;

	/*
	 * Call the general probe procedures.
	 *
	 * The unusual_dev_list array is parallel to the usb_storage_usb_ids
	 * table, so we use the index of the id entry to find the
	 * corresponding unusual_devs entry.
	 */
	result = usb_stor_probe1(&us, intf, id,
			(id - jm20329_usb_ids) + jm20329_unusual_dev_list);
	if (result)
		return result;

	us->extra = NULL;
	/* No special transport or protocol settings in the main module */

	result = usb_stor_probe2(us);
	return result;
}

static int jm20329_usb_connect(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	int result;
	struct us_data *us;
	struct jm20329_device *jm20329;
	struct usb_device *parent;

	jm20329 = find_device(dev_name(&intf->dev));
	if (!jm20329) {
		US_DEBUGP("Use generic storage for nonspecific devices: %s\n", dev_name(&intf->dev));
		return jm20329_removable_connect(intf, id);
	}
	
	us = jm20329->us;
	
	/* Associate the us_data structure with the USB device */
	result = jm20329_associate_dev(us, intf);
	if (result)
		goto BadDevice;

	/* Get the unusual_devs entries and the descriptors */
	result = jm20329_get_device_info(us, id, (id - jm20329_usb_ids) + jm20329_unusual_dev_list);
	if (result)
		goto BadDevice2;

	/* Find the endpoints and calculate pipe values */
	result = jm20329_get_pipes(us);
	if (result)
		goto BadDevice2;

	/* Figure out which roothub we are connected to */
	parent = jm20329->us->pusb_dev->parent;
	while (parent->parent)
		parent = parent->parent;
	mutex_lock(&jm20329->dev_mutex);
	jm20329->roothub = parent;
	mutex_unlock(&jm20329->dev_mutex);

	if (!jm20329->configured) {
		result = jm20329_usb_stor_probe2(us);
		if (result)
			goto BadDevice2;
		jm20329->configured = true;
	} else {
		usb_enable_autosuspend(us->pusb_dev);
		mutex_lock(&jm20329->dev_mutex);
		jm20329->state = JM20329_AWAKE;
		mutex_unlock(&jm20329->dev_mutex);
		complete(&us->cmnd_ready);	
	}
	
	
	return 0;
BadDevice2:
	jm20329_dissociate_dev(us);
BadDevice:
	return result;
}

static struct usb_driver jm20329_usb_driver = {
	.name 			= "ums-jm20329",
	.probe 			= jm20329_usb_connect,
	.disconnect 		= jm20329_usb_disconnect,
	.suspend 		= usb_stor_suspend,
	.resume 		= usb_stor_resume,
	.reset_resume 		= usb_stor_reset_resume,
	.pre_reset 		= usb_stor_pre_reset,
	.post_reset 		= usb_stor_post_reset,
	.id_table 		= jm20329_usb_ids,
	.soft_unbind 		= 1,
	.supports_autosuspend 	= 0,
//	.no_dynamic_id		= 1,
};

static int jm20329_probe(struct platform_device *pdev)
{
	struct Scsi_Host *host;
	struct us_data *us;
	int result;
	struct jm20329_device *jm20329;

	jm20329 = kmalloc(sizeof(struct jm20329_device), GFP_KERNEL);
	if (!jm20329)
		return -ENOMEM;
	
	memset(jm20329, 0, sizeof(struct jm20329_device));
	
	jm20329->pdrv = pdev->dev.platform_data;

	jm20329->udrv = &jm20329_usb_driver;
	
	spin_lock_init(&jm20329->state_lock);
	
	jm20329->pdev = pdev;
	if (jm20329->pdrv->init) {
		if ((result = jm20329->pdrv->init(jm20329)) < 0)
			goto failed;
	}
	host = scsi_host_alloc(&usb_stor_host_template, sizeof(struct us_data));
	if (!host) {
		dev_warn(&pdev->dev,
				"Unable to allocate the scsi host\n");
		result = -ENOMEM;
		goto failed2;
	}

	/*
	 * Allow 16-byte CDBs and thus > 2TB
	 */
	host->max_cmd_len = 16;
	host->sg_tablesize = SG_ALL;
	us = host_to_us(host);
	memset(us, 0, sizeof(struct us_data));
	mutex_init(&us->dev_mutex);
	init_completion(&us->cmnd_ready);
	init_completion(&(us->notify));
	init_waitqueue_head(&us->delay_wait);
	init_completion(&us->scanning_done);

	us->protocol_name = "JM20329 SATA";
	us->proto_handler = usb_stor_transparent_scsi_command;

	/* Get standard transport and protocol settings */
	us->transport_name = "Bulk";
	us->transport = usb_stor_Bulk_transport;
	us->transport_reset = usb_stor_Bulk_reset;
	us->suspend_resume_hook = jm20329_suspend_resume_hook;

	us->extra = jm20329;
	jm20329->us = us;

	mutex_init(&jm20329->dev_mutex);
	jm20329->timeout_reset = JM20329_SUSPEND_TIMEOUT;
	jm20329->timeout = jm20329->timeout_reset;
	jm20329->in_transition = 0;
	jm20329->roothub = NULL;
			
	platform_set_drvdata(pdev, jm20329);
	
	/* first usb device ? */
	if (list_empty(&jm20329_devices)) {
		result = usb_register(&jm20329_usb_driver);
		if (result)
			goto failed3;
	}
	
	list_add_tail(&jm20329->node, &jm20329_devices);

	return 0;

failed3:
	scsi_host_put(host);
failed2:
	if (jm20329->pdrv->release)
		jm20329->pdrv->release(jm20329);	
failed:
	kfree(jm20329);
	return result;
}

static int jm20329_remove(struct platform_device *pdev)
{
	struct jm20329_device *jm20329 = platform_get_drvdata(pdev);
	struct jm20329_device *e;
	
	list_for_each_entry(e, &jm20329_devices, node) {
		if (e == jm20329) {
			list_del(&jm20329->node);
			break;
		}
	}

	if (list_empty(&jm20329_devices)) {
		usb_deregister(&jm20329_usb_driver);
	}
	
	scsi_host_put(us_to_host(jm20329->us));
	
	if (jm20329->pdrv->release)
		jm20329->pdrv->release(jm20329);	

	kfree(jm20329);
	return 0;
}

static int jm20329_suspend(struct platform_device *pdev, pm_message_t message)
{
	struct jm20329_device *jm20329 = platform_get_drvdata(pdev);
	mutex_lock(&jm20329->dev_mutex);
	jm20329->timeout = 0;
	while (jm20329->state != JM20329_ASLEEP && jm20329->state != JM20329_RELEASE) {
		mutex_unlock(&jm20329->dev_mutex);
		schedule_timeout(HZ/10);
		mutex_lock(&jm20329->dev_mutex);
	}
	mutex_unlock(&jm20329->dev_mutex);
	return 0;
}

static void jm20329_shutdown(struct platform_device *pdev)
{
	struct jm20329_device *jm20329 = platform_get_drvdata(pdev);

	if (jm20329->us && jm20329->us->pusb_dev) {
		struct usb_device *udev = jm20329->us->pusb_dev;
		usb_lock_device(udev);
		//usb_try_autosuspend_device(udev);
		//usb_autosuspend_device(udev);
		usb_unlock_device(udev);
	}

	mutex_lock(&jm20329->dev_mutex);
	jm20329->timeout = 0;
	while (jm20329->state != JM20329_ASLEEP && jm20329->state != JM20329_RELEASE) {
		mutex_unlock(&jm20329->dev_mutex);
		schedule_timeout(HZ/10);
		mutex_lock(&jm20329->dev_mutex);
	}	
	mutex_unlock(&jm20329->dev_mutex);
}

static struct platform_driver jm20329_driver = {
	.probe			= jm20329_probe,
	.remove			= jm20329_remove,
	.shutdown		= jm20329_shutdown,
	.suspend		= jm20329_suspend,
	.driver = {
		.name = "jm20329",
		.owner = THIS_MODULE,
	}
};

static int __init jm20329_init(void)
{
	return platform_driver_register(&jm20329_driver);
}

static void __exit jm20329_exit(void)
{
	platform_driver_unregister(&jm20329_driver);
}

module_init(jm20329_init);
module_exit(jm20329_exit);
