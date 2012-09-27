#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/completion.h>
#include <linux/miscdevice.h>
#include <video/omapdss.h>
#include <linux/switch.h>

#include "dss.h"
#include "cec.h"

#define CEC_DEBUG 	0
#define DUMP_CEC if (CEC_DEBUG)

#if CEC_DEBUG
#define CEC_DBG(fmt, args...) printk(fmt,## args)
#else
#define CEC_DBG(fmt, args...)
#endif

#define CEC_ERROR(fmt, args...) printk(KERN_ERR fmt,## args)

static struct cec_t {
	void __iomem *hdmi_wp_base_addr;
	struct cec_rx_data rx_data;
	struct switch_dev rx_switch;
	struct mutex lock;
	bool cec_rx_data_valid;
	int phy_address;
	char vendor_id[3];
	int tv_vendorId;
	int dss_state;
	int switch_state;
	int power_on;
	char cec_source_power;
	char menu_mode;
	char active_source;
	int always_active_source;

	struct cec_dev dev;
	struct input_dev *input_dev;
	char input_name[32];
} cec;

#define CEC_BROADCAST	(cec_id_list[CEC_UNREGISTERED].ids[0])

#define HDMI_CORE_CEC_TIMEOUT 200
#define DSS_POWER
/* Driver */
static int __init cec_init(void);
static void __exit cec_exit(void);
static int reply(enum cec_command reply_cmd, char* op, int op_len, char dest_id);
static void request_vendor_id(void);
static void report_initial_status(void);

static void cec_request_dss(void)
{
#ifdef DSS_POWER
	cec.dss_state = dss_runtime_get();
#endif
}
static void cec_release_dss(void)
{
#ifdef DSS_POWER
	dss_runtime_put();
#endif
}

int register_cec_input_device(void)
{
	int r = 0;
	cec.input_dev = input_allocate_device();
	if (!cec.input_dev)
		return -ENOMEM;
	snprintf(cec.input_name, 32, "omap4_hdmi_cec");
	cec.input_dev->name = cec.input_name;
	cec.input_dev->id.bustype = BUS_USB;
	cec.input_dev->id.vendor = 0x0e79; //Archos USB ID
	cec.input_dev->id.product = 0x0001;
	cec.input_dev->id.version = 0x0001;

	input_set_drvdata(cec.input_dev, &cec);

	set_bit(EV_KEY, cec.input_dev->evbit);

	set_bit(BUTTON_OK, cec.input_dev->keybit);
	set_bit(DPAD_LEFT, cec.input_dev->keybit);
	set_bit(DPAD_RIGHT, cec.input_dev->keybit);
	set_bit(DPAD_UP, cec.input_dev->keybit);
	set_bit(DPAD_DOWN, cec.input_dev->keybit);
	set_bit(BUTTON_EXIT, cec.input_dev->keybit);
	set_bit(BUTTON_0, cec.input_dev->keybit);
	set_bit(BUTTON_1, cec.input_dev->keybit);
	set_bit(BUTTON_2, cec.input_dev->keybit);
	set_bit(BUTTON_3, cec.input_dev->keybit);
	set_bit(BUTTON_4, cec.input_dev->keybit);
	set_bit(BUTTON_5, cec.input_dev->keybit);
	set_bit(BUTTON_6, cec.input_dev->keybit);
	set_bit(BUTTON_7, cec.input_dev->keybit);
	set_bit(BUTTON_8, cec.input_dev->keybit);
	set_bit(BUTTON_9, cec.input_dev->keybit);
	set_bit(RED_BUTTON, cec.input_dev->keybit);
	set_bit(GREEN_BUTTON, cec.input_dev->keybit);
	set_bit(YELLOW_BUTTON, cec.input_dev->keybit);
	set_bit(BLUE_BUTTON, cec.input_dev->keybit);
	set_bit(PLAY_BUTTON, cec.input_dev->keybit);
	set_bit(STOP_BUTTON, cec.input_dev->keybit);
	set_bit(WIND_BUTTON, cec.input_dev->keybit);
	set_bit(REWIND_BUTTON, cec.input_dev->keybit);

	r = input_register_device(cec.input_dev);
	if (r) {
		input_free_device(cec.input_dev);
		return r;
	}
	return 0;
}

int unregister_cec_input_device(void)
{
	if (cec.input_dev) input_unregister_device(cec.input_dev);
	return 0;
}

int cec_read_rx_cmd(struct cec_rx_data *rx_data)
{
	int rx_byte_cnt;
	int temp;
	int i;
	int cec_cmd_cnt = 0;
	int r = 0;

	cec_request_dss();
	cec_cmd_cnt = RD_FIELD_32(cec.hdmi_wp_base_addr +
				HDMI_IP_CORE_CEC, HDMI_CEC_RX_COUNT, 6, 4);
	if (cec_cmd_cnt > 0) {
		/*Get the initiator and destination id*/
		temp = RD_REG_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC,
			HDMI_CEC_RX_CMD_HEADER);
		rx_data->init_device_id = FLD_GET(temp, 7, 4);
		rx_data->dest_device_id = FLD_GET(temp, 3, 0);

		/*get the command*/
		temp = RD_REG_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC,
			HDMI_CEC_RX_COMMAND);
		rx_data->rx_cmd = FLD_GET(temp, 7, 0);

		/*Get the rx command operands*/
		temp = RD_REG_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC,
			HDMI_CEC_RX_COUNT);
		rx_byte_cnt = FLD_GET(temp, 3, 0);
		rx_data->rx_count = rx_byte_cnt;

		for (i = 0; i < rx_byte_cnt; i++) {
			temp = RD_REG_32(cec.hdmi_wp_base_addr +
				HDMI_IP_CORE_CEC,
			HDMI_CEC_RX_OPERAND + (i * 4));
			rx_data->rx_operand[i] = FLD_GET(temp, 7, 0);
		}

		/* Clear the just read command */
		temp = WR_FIELD_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC,
			HDMI_CEC_RX_CONTROL, 0, 0, 1);
	} else {
		/*No cmd in the FIFO return error*/
		r = -EINVAL   ;
	}

	cec_release_dss();
	return r;
}
int cec_transmit_cmd(struct cec_tx_data *data, int *cmd_acked)
{
	int r = EINVAL;
	u32 timeout = HDMI_CORE_CEC_TIMEOUT;
	u32 temp, i = 0;

	if (data == NULL)
		goto error_exit;

	/* 1. Flush TX FIFO - required as change of initiator ID / destination
	ID while TX is in progress could result in courrupted message.
	2. Clear interrupt status registers for TX.
	3. Set initiator Address, set retry count
	4. Set Destination Address
	5. Clear TX interrupt flags - if required
	6. Set the command
	7. Transmit
	8. Check for NACK / ACK - report the same. */

	cec_request_dss();

	/* Clear TX FIFO */
	WR_FIELD_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC, HDMI_CEC_DBG_3,
			7, 7, 0x1);

	while (timeout) {
		temp = RD_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
				HDMI_CEC_DBG_3);
		if (FLD_GET(temp, 7, 7) == 0)
			break;
		timeout--;
	}
	if (timeout == 0x0) {
		printk(KERN_ERR "Could not clear TX FIFO");
		printk(KERN_ERR "\n FIFO Reset - timeouts  : %d - was %d\n",
			timeout, HDMI_CORE_CEC_TIMEOUT);
		goto error_exit;
	}

	/* Clear TX interrupts */
	WR_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC, HDMI_CEC_INT_STATUS_0,
		0x64);

	WR_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC, HDMI_CEC_INT_STATUS_1,
		0x2);

	/* Set the initiator addresses */

	temp = RD_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
			HDMI_CEC_TX_INIT);
	temp = FLD_MOD(temp, data->initiator_device_id, 3, 0);
	WR_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC, HDMI_CEC_TX_INIT,
		temp);
	/*Set destination id*/

	temp = RD_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
		HDMI_CEC_TX_DEST);
	temp = FLD_MOD(temp, data->dest_device_id, 3, 0);
	WR_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC, HDMI_CEC_TX_DEST,
		temp);


	/* Set the retry count */
	WR_FIELD_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC, HDMI_CEC_DBG_3, 6,
		4, data->retry_count);

	if (data->send_ping)
		goto send_ping;


	/* Setup command and arguments for the command */
	WR_FIELD_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC, HDMI_CEC_TX_COMMAND,
		7, 0, data->tx_cmd);


	for (i = 0; i < data->tx_count; i++) {
		temp = RD_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
			(HDMI_CEC_TX_OPERAND + (i * 4)));
		temp = FLD_MOD(temp, data->tx_operand[i], 7, 0);
		WR_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
			(HDMI_CEC_TX_OPERAND + (i * 4)), temp);
	}

	/* Operand count */
	WR_FIELD_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
		HDMI_CEC_TRANSMIT_DATA,	3, 0, data->tx_count);
	/* Transmit */
	WR_FIELD_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
		HDMI_CEC_TRANSMIT_DATA,	4, 4, 0x1);

	goto wait_for_ack_nack;

send_ping:
	CEC_DBG("cec_transmit_cmd use ping only\n");
	WR_FIELD_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC, HDMI_CEC_TX_DEST,
		7, 7, 0x1);

wait_for_ack_nack:
	CEC_DBG("cec_transmit_cmd wait for ack\n");
	timeout = HDMI_CORE_CEC_TIMEOUT + 200;
	*cmd_acked = -1;
	do {
		temp = RD_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
			HDMI_CEC_INT_STATUS_0);
		/* Look for TX change event */
		if (FLD_GET(temp, 5, 5) != 0) {
			*cmd_acked = 1;
			/* Clear interrupt status */
			WR_FIELD_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
				HDMI_CEC_INT_STATUS_0, 5, 5, 1);
			if (FLD_GET(temp, 2, 2) != 0) {
				/* Clear interrupt status */
				WR_FIELD_32(cec.hdmi_wp_base_addr +
					HDMI_IP_CORE_CEC,
					HDMI_CEC_INT_STATUS_0, 2, 2, 1);
			}

			r = 0;
			break;
		}
		/* Wait for re-transmits to expire */
		temp = RD_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
			HDMI_CEC_INT_STATUS_1);
		if (FLD_GET(temp, 1, 1) == 0) {
			/* Wait for 7 mSecs - As per CEC protocol
				nominal bit period is ~3 msec
				delay of >= 3 bit period before next attempt
			*/
			mdelay(10);

		} else {
			/* Nacked ensure to clear the status */
			temp = FLD_MOD(0x0, 1, 1, 1);
			WR_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
				HDMI_CEC_INT_STATUS_1, temp);
			*cmd_acked = 0;
			r = 0;
			break;
		}
		timeout--;
	} while (timeout);

	if (timeout == 0x0) {
		printk(KERN_ERR "\nCould not send\n");
		printk(KERN_ERR "\nNo ack / nack sensed\n");
		printk(KERN_ERR "\nResend did not complete in : %d\n",
			((HDMI_CORE_CEC_TIMEOUT - timeout) * 10));
	}

error_exit:
	cec_release_dss();
	return r;
}

static int try_to_register(struct cec_dev *dev, int current_ca_7_0, int current_ca_15_8)
{
	struct cec_tx_data tx_data;
	u32 regis_reg, temp,shift_cnt;
	int r;
	int associated = 0;
	int acked_nacked = 0xFF;

	CEC_DBG("cec_register_device try to register the device as %d\n",  dev->device_id);
	tx_data.initiator_device_id = dev->device_id;
	/* Register to receive messages intended for this device
	and broad cast messages */
	regis_reg = HDMI_CEC_CA_7_0;
	shift_cnt = dev->device_id;
	temp = 0;
	if (dev->device_id > 0x7) {
		regis_reg = HDMI_CEC_CA_15_8;
		shift_cnt -= 0x7;
	}
	if (dev->clear_existing_device == 0x1) {
		WR_REG_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC,
			HDMI_CEC_CA_7_0, 0);
		WR_REG_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC,
			HDMI_CEC_CA_15_8, 0);
	} else {
		temp = RD_REG_32(cec.hdmi_wp_base_addr +
			HDMI_IP_CORE_CEC, regis_reg);
	}
	temp |= 0x1 << shift_cnt;
	WR_REG_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC, regis_reg,
		temp);

	/* Report Association between physical address and device id */
	/* Report Physical address broadcast message */
	tx_data.dest_device_id = cec_id_list[CEC_UNREGISTERED].ids[0];
	tx_data.tx_cmd = cec_cmd_b_report_physical_address;
	tx_data.tx_count = 0x3;
	tx_data.tx_operand[0] = (cec.phy_address & 0xff00)>>8;
	tx_data.tx_operand[1] = cec.phy_address & 0xff;
	tx_data.tx_operand[2] = dev->device_id;;
	tx_data.retry_count = 5;
	tx_data.send_ping = 0;
	r = cec_transmit_cmd(&tx_data, &acked_nacked);
	if ((acked_nacked != 1) || (r != 0x0)) {
		/* Restore previous registration */
		WR_REG_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC,
			HDMI_CEC_CA_15_8, current_ca_15_8);
		WR_REG_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC,
			HDMI_CEC_CA_7_0, current_ca_7_0);
	} else {
		associated = 1;
		dev->phy_addr = cec.phy_address;
		CEC_DBG("cec_register_device device registered as %d\n",  dev->device_id);
	}
	return associated;
}

int cec_register_device(struct cec_dev *dev)
{
	int acked_nacked;
	struct cec_tx_data tx_data;
	int r;
	int current_ca_15_8;
	int current_ca_7_0;
	CEC_DBG("cec_register_device[%d]++\n", dev->device_id);

	/*  1. Send an Ping command
	2. If acked, return error
	3. Register to receive for registered initiator
	4. report physical address
	5. check for nacks
	6. set flag is_pa_set to TRUE indicating we are registered. */

	if (dev->device_id == cec_id_list[CEC_UNREGISTERED].ids[0]) {
		acked_nacked = 0;
		cec.power_on = 0;
		CEC_DBG("%s Device id is unreg hence no ping required\n",
			__func__);
		goto no_ping_required;
	}
	/*send ping message to the desired device id*/
	/*Initially set initiator id to 0xf */
	tx_data.initiator_device_id = cec_id_list[CEC_UNREGISTERED].ids[0];
	tx_data.dest_device_id = dev->device_id;
	/* cmd and no of arguments are not used for ping command */
	tx_data.tx_cmd = 0x0;
	tx_data.tx_count = 0x0;

	acked_nacked = 0xFF;

	tx_data.send_ping = 0x1;
	tx_data.retry_count = 5;

	r = cec_transmit_cmd(&tx_data, &acked_nacked);

	if (r != 0) {
		printk(KERN_ERR "\nCould not Ping device\n");
		return -1;
	}

no_ping_required:
	cec_request_dss();

	/*Store current device listning ids*/
	current_ca_15_8 = RD_REG_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC,
		HDMI_CEC_CA_15_8);
	current_ca_7_0 = RD_REG_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC,
		HDMI_CEC_CA_7_0);

	/* Check if device is already present */
	if (acked_nacked != 1) {
		int associated = 0;
		int nb_try = cec_id_list[dev->type].num_id;
		  //Try first with last known registration
		if (dev->device_id != cec_id_list[CEC_UNREGISTERED].ids[0]) {
			associated = try_to_register(dev, current_ca_7_0, current_ca_15_8);
		}

		while (!associated && nb_try!= 0)
		{
			cec.power_on = 0;
			dev->device_id = cec_id_list[dev->type].ids[cec_id_list[dev->type].num_id - nb_try--];
			associated = try_to_register(dev, current_ca_7_0, current_ca_15_8);
		}
		if (!associated) {
			  CEC_DBG("cec_register_device association failed\n");
			  switch_set_state(&cec.rx_switch, 0);
			  r = -1;
			  goto end;
		}
	} else {
		/* Device present */
		CEC_DBG("cec_register_device device[%d] already present\n",
			dev->device_id);
		r = -EEXIST;
	}
	request_vendor_id();
end:
	cec_release_dss();
	return r;
}

int cec_recv_cmd(struct cec_rx_data *data)
{
	int r = 0;

	r = cec_read_rx_cmd(data);

	return r;
}
long cec_ioctl(struct file *fd, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int r;
	if (cec.power_on == 0) {
		CEC_DBG("HDMI not connected ++\n");
		return -EFAULT;
	}
	switch (cmd) {
	case CEC_REGISTER_DEVICE:
	{
		r = copy_from_user(&cec.dev, argp, sizeof(struct cec_dev));
		if (!r) {
			mutex_lock(&cec.lock);
			r = cec_register_device(&cec.dev);
			mutex_unlock(&cec.lock);
		}

		if (!r)
			r = copy_to_user(argp, &cec.dev, sizeof(struct cec_dev));

		return r == 0 ? 0 : -EFAULT;
	}
	case CEC_TRANSMIT_CMD:
	{
		struct cec_tx_data tx_cmd;
		int cmd_acked;
		r = copy_from_user(&tx_cmd, argp, sizeof(struct cec_tx_data));
			if (!r) {
				mutex_lock(&cec.lock);
				r = cec_transmit_cmd(&tx_cmd, &cmd_acked);
				mutex_unlock(&cec.lock);
			}
			return r == 0 ? cmd_acked : -EFAULT;
	}
	case CEC_RECV_CMD:
	{
		struct cec_rx_data rx_cmd;
		mutex_lock(&cec.lock);
		r = cec_recv_cmd(&rx_cmd);
		mutex_unlock(&cec.lock);
		if (!r) {
			r = copy_to_user(argp, &rx_cmd,
				sizeof(struct cec_rx_data));
		}
		return r == 0 ? 0 : -EFAULT;
	}
	case CEC_GET_PHY_ADDR:
	{

		if (cec.power_on) {
			r = copy_to_user(argp, &cec.phy_address, sizeof(int));
			return 0;
		} else
			return -EFAULT;
	}
	default:
		return -ENOTTY;
	} /* End switch */
}


/******************************************************************************
 * CEC driver init/exit
 *****************************************************************************/


static const struct file_operations cec_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = cec_ioctl,
};

static struct miscdevice mdev;


int cec_module_init(void)
{
	return 0;
}

static struct key_worker_data {
	struct delayed_work dwork;
	atomic_t keycode;
} cec_key_work;

static struct cec_worker_data {
	struct delayed_work dwork;
	atomic_t state;
} cec_work;

static struct cec_disconnect_worker_data {
	struct delayed_work dwork;
} cec_disconnect_work;

static struct workqueue_struct *my_workq;
static struct workqueue_struct *my_key_workq;
static struct workqueue_struct *my_cec_workq;

static void dump_rx_data(struct cec_rx_data *rx_data) {
	int i;
	CEC_DBG("Received CEC packet on device 0x%x.%x.%x.%x state:%d:\n", (cec.phy_address & 0xF000) >> 12, (cec.phy_address & 0x0F00) >> 8, (cec.phy_address & 0x00F0) >> 4, (cec.phy_address & 0x00F), cec.power_on);
	CEC_DBG("\trx_cmd 0x%x \n\tinit_id %d \n\tdest_id %d \n\t broadcast:%d\n", rx_data->rx_cmd, rx_data->init_device_id, rx_data->dest_device_id,(rx_data->dest_device_id == cec_id_list[CEC_UNREGISTERED].ids[0])?1:0);
	if (rx_data->rx_count) {
	  CEC_DBG("\t parameter :");
	  for (i = 0; i < rx_data->rx_count; i++) CEC_DBG("0x%x ", rx_data->rx_operand[i]);
	}
	CEC_DBG("\n");
}

static int reply(enum cec_command reply_cmd, char* op, int op_len, char dest_id) {
	struct cec_tx_data tx_cmd;
	int r= 0, cmd_acked;
	CEC_DBG("Reply CEC : \n\tCmd=0x%x\n\tDest=0x%x\n\tByteLen=0x%x\n", reply_cmd, dest_id, op_len);
	if (op_len != 0) {
		int i = 0;
		CEC_DBG("\tByte Array= ");
		for (i=0; i<op_len; i++) CEC_DBG("0x%x ", op[i]);
		CEC_DBG("\n");
	}
	tx_cmd.dest_device_id = dest_id;
	tx_cmd.initiator_device_id = cec.dev.device_id;
	tx_cmd.retry_count = 5;
	tx_cmd.tx_cmd = reply_cmd;
	tx_cmd.tx_count = (op_len<=15)?op_len:15;
	memcpy(tx_cmd.tx_operand,op,tx_cmd.tx_count );
	tx_cmd.send_ping = 0;
	r = cec_transmit_cmd(&tx_cmd, &cmd_acked);
	if (cmd_acked != 1) {
		CEC_DBG("Error cmd 0x%x not acked\n", reply_cmd);
		r = -1;
	}
	return r;
}

static void set_current_active_source(int enabled) {
	if ((enabled != 0) || cec.always_active_source) {
		cec.active_source = enabled;
	}
	//in case on non automatic switch, wake-up the product
	switch_set_state(&cec.rx_switch, enabled);
}

static void request_vendor_id(void) {
	reply(cec_cmd_u_give_device_vendor_id, NULL, 0, cec_id_list[CEC_TV].ids[0]);
}

static void report_initial_status(void) {
	//Report initial status
	char phy[2];
	//power = 0 => cable disconnect
	//power = 1, active_source = 1 => Cable still connected and device is still the source
	if ((cec.power_on) && (!cec.active_source)) goto enable_switch;

	cec.power_on = 1;
	cec.cec_source_power = 0x0;
	cec.menu_mode = 0x0;
	phy[0] = (cec.phy_address & 0xff00)>>8;
	phy[1] = cec.phy_address & 0xff;

	reply(cec_cmd_u_image_view_on, NULL, 0, cec_id_list[CEC_TV].ids[0]);
	reply(cec_cmd_u_text_view_on, NULL, 0, cec_id_list[CEC_TV].ids[0]);
	if (!cec.active_source) {
		reply(cec_cmd_b_active_source, phy, 2, CEC_BROADCAST);
	}
	set_current_active_source(0x1);
	reply(cec_cmd_b_device_vendor_id, cec.vendor_id, 3, CEC_BROADCAST);
	reply(cec_cmd_u_menu_status, &cec.menu_mode, 1, cec_id_list[CEC_TV].ids[0]);
	phy[0] = cec_ui_command_power_on_function;
	reply(cec_cmd_u_user_control_pressed, phy, 1, cec_id_list[CEC_TV].ids[0]);
	reply(cec_cmd_u_user_control_released, phy, 1, cec_id_list[CEC_TV].ids[0]);

	if (cec.tv_vendorId == CEC_VENDOR_LG) {
		phy[0] = 0x3;
		phy[1] = 0x0;
		reply(cec_cmd_u_vendor_command, phy, 2, cec_id_list[CEC_TV].ids[0]);
	}
enable_switch:
	switch_set_state(&cec.rx_switch, 1);
}

static int process_cec_control(struct cec_rx_data *rx_data) {
	cec_buttons keycode = -1;

	switch(rx_data->rx_operand[0]) {
	  case cec_ui_command_power_on_function:
	  case cec_ui_command_power:
	  {
		char phy[2];
		phy[0] = (cec.phy_address & 0xff00)>>8;
		phy[1] = cec.phy_address & 0xff;
		//generate an event on power key
		cec.cec_source_power = 0x0;
		reply(cec_cmd_u_image_view_on, NULL, 0, cec_id_list[CEC_TV].ids[0]);
		reply(cec_cmd_u_text_view_on, NULL, 0, cec_id_list[CEC_TV].ids[0]);
		reply(cec_cmd_b_active_source, phy, 2, CEC_BROADCAST);
	  }
	  break;
	  case cec_ui_command_select:
		 keycode = BUTTON_OK;
	  break;
	  case cec_ui_command_up:
	  case cec_ui_command_forward:
		 keycode = DPAD_UP;
	  break;
	  case cec_ui_command_down:
	  case cec_ui_command_backward:
		 keycode = DPAD_DOWN;
	  break;
	  case cec_ui_command_left:
		 keycode = DPAD_LEFT;
	  break;
	  case cec_ui_command_right:
		 keycode = DPAD_RIGHT;
	  break;
	  case cec_ui_command_num_0_10:
	  case cec_ui_command_num_1:
	  case cec_ui_command_num_2:
	  case cec_ui_command_num_3:
	  case cec_ui_command_num_4:
	  case cec_ui_command_num_5:
	  case cec_ui_command_num_6:
	  case cec_ui_command_num_7:
	  case cec_ui_command_num_8:
	  case cec_ui_command_num_9:
		 keycode = BUTTON_0 + (rx_data->rx_operand[0]-cec_ui_command_num_0_10);
	  break;
	  case cec_ui_command_play:
	  case cec_ui_command_pause:
	  case cec_ui_command_play_function:
	  case cec_ui_command_pause_play_function:
		 keycode = PLAY_BUTTON;
	  break;
	  case cec_ui_command_stop:
	  case cec_ui_command_stop_function:
		 keycode = STOP_BUTTON;
	  break;
	  case cec_ui_command_rewind:
		 keycode = REWIND_BUTTON;
	  break;
	  case cec_ui_command_fast_forward:
	         keycode = WIND_BUTTON;
	  break;
	  case cec_ui_command_f1_blue:
		 keycode = BLUE_BUTTON;
	  break;
	  case cec_ui_command_f2_red:
		 keycode = RED_BUTTON;
	  break;
	  case cec_ui_command_f3_green:
		 keycode = GREEN_BUTTON;
	  break;
	  case cec_ui_command_f4_yellow:
		 keycode = YELLOW_BUTTON;
	  break;
	  case cec_ui_command_exit:
	    //Let use the red button as exit... Not true in fact
		 keycode = BUTTON_EXIT;
	  break;
	  default:
	    CEC_ERROR("Unsupported CEC UI command 0x%x\n", rx_data->rx_operand[0]);
	    break;
	}
	if (keycode != -1) {
		mutex_lock(&cec.lock);
		if (cec.dev.last_ui_button != -1) {
			__cancel_delayed_work(&cec_key_work.dwork);
			if (cec.dev.last_ui_button != keycode) {
				input_report_key(cec.input_dev, cec.dev.last_ui_button, 0);
				input_sync(cec.input_dev);
			}
		}
		if (cec.dev.last_ui_button != keycode) {
			//Report a press
			input_report_key(cec.input_dev, keycode, 1);
			input_sync(cec.input_dev);
		}
		cec.dev.last_ui_button = keycode;
		atomic_set(&cec_key_work.keycode, keycode);
		queue_delayed_work(my_key_workq, &cec_key_work.dwork, msecs_to_jiffies(300));
		mutex_unlock(&cec.lock);
	}
	return 0;
}

static int cec_process_cmd(struct cec_rx_data *rx_data) {
	int broadcast = (rx_data->dest_device_id == cec_id_list[CEC_UNREGISTERED].ids[0])?1:0;
	DUMP_CEC dump_rx_data(rx_data);
	if ((rx_data->dest_device_id == cec.dev.device_id)||broadcast) {
		switch (rx_data->rx_cmd) {
		  case cec_cmd_u_give_osd_name:
		  {
			char name[6] = "archos";
			if (broadcast) break;
			reply(cec_cmd_u_set_osd_name, name, 6 , rx_data->init_device_id);
		  }
		  break;
		  case cec_cmd_u_give_device_vendor_id:
		  {
			if (broadcast) break;
			reply(cec_cmd_b_device_vendor_id, cec.vendor_id, 3, CEC_BROADCAST);
		  }
		  break;
		  case cec_cmd_b_device_vendor_id:
		  {
			if (!broadcast || (rx_data->rx_count != 3)) break;
			cec.tv_vendorId = (rx_data->rx_operand[0] << 16) | (rx_data->rx_operand[1] << 8) | (rx_data->rx_operand[2]);
			switch (cec.tv_vendorId) {
			  case CEC_VENDOR_SONY:
			  {
				char activate_cmd[7];
				activate_cmd[0] = rx_data->rx_operand[0];
				activate_cmd[1] = rx_data->rx_operand[1];
				activate_cmd[2] = rx_data->rx_operand[2];
				activate_cmd[3] = 0x00;
				activate_cmd[4] = 0x06;
				activate_cmd[5] = 0x00;
				activate_cmd[6] = 0x01;
				reply(cec_cmd_u_vendor_command_with_id, activate_cmd, 7, rx_data->init_device_id);
			  }
			  break;
			  case CEC_VENDOR_LG:
			  {
				//LG tv only accept to talk to LG devices.
				cec.vendor_id[0] = CEC_VENDOR_LG >> 16;
				cec.vendor_id[1] = CEC_VENDOR_LG >> 8 & 0xFF;
				cec.vendor_id[2] = CEC_VENDOR_LG  & 0xFF;
				//send an abort without parameters to acknowledge
				reply(cec_cmd_u_feature_abort, NULL, 0, cec_id_list[CEC_TV].ids[0]);
			  }
			  break;
			  default:
			  break;
			}
			if (cec.tv_vendorId != CEC_VENDOR_LG) report_initial_status();
		  }
		  break;
		  case cec_cmd_u_user_control_pressed:
		  {
			if (broadcast) break;
			//The user pressed a button
			if (rx_data->rx_count == 1) {
				process_cec_control(rx_data);
			}
		  }
		  break;
		  case cec_cmd_u_B_vendor_remote_button_down:
		  {
			switch(cec.tv_vendorId) {
			  case CEC_VENDOR_SAMSUNG:
				if (rx_data->rx_count == 1) {
					switch (rx_data->rx_operand[0]) {
					  case 0x91:
						//Return key pressed -> fake a back key pressed
						rx_data->rx_operand[0] = cec_ui_command_exit;
						process_cec_control(rx_data);
					  break;
					  default:
					  break;
					}
				}
			break;
			default:
			break;
			}
		  }
		  break;
		  case cec_cmd_u_user_control_released:
		  case cec_cmd_u_B_vendor_remote_button_up:
		  {
			//Do not take care of this message if TV does not support long press
		  }
		  break;
		  case cec_cmd_u_play:
		  {
			if (broadcast || (rx_data->rx_count != 1)) break;
			//Fake a play/pause UI event
			rx_data->rx_operand[0] = cec_ui_command_pause_play_function;
			process_cec_control(rx_data);
		  }
		  break;
		  case cec_cmd_u_deck_control:
		  {
			enum cec_ui_command fake_ui_cmd = -1;
			if (broadcast || (rx_data->rx_count != 1)) break;

			switch (rx_data->rx_operand[0]) {
			  case 1:
			    fake_ui_cmd = cec_ui_command_forward;
			  break;
			  case 2:
			    fake_ui_cmd = cec_ui_command_backward;
			  break;
			  case 3:
			  case 4:
			    fake_ui_cmd = cec_ui_command_stop;
			  break;
			  default:
			  break;
			}
			if (fake_ui_cmd != -1) {
				rx_data->rx_operand[0] = fake_ui_cmd;
				process_cec_control(rx_data);
			}
		  }
		  break;
		  case cec_cmd_u_give_device_power_status:
		  {
			if (broadcast) break;
			CEC_DBG("Report power status = 0x%x\n", cec.cec_source_power);
		        reply(cec_cmd_u_report_power_status, &cec.cec_source_power , 1, rx_data->init_device_id);
		  }
		  break;
		  case cec_cmd_u_get_cec_version:
		  {
			char version = 0x4; //1.3a
			if (broadcast) break;
		        reply(cec_cmd_u_cec_version, &version, 1, rx_data->init_device_id);
		  }
		  break;
		  case cec_cmd_u_give_physical_address:
		  {
			char phy[3];
			if (broadcast) break;
			phy[0] = (cec.phy_address & 0xff00)>>8;
			phy[1] = cec.phy_address & 0xff;
			phy[2] = cec.dev.device_id;
			//generate an event on power key
			reply(cec_cmd_b_report_physical_address, phy, 3, CEC_BROADCAST);
		  }
		  break;
		  case cec_cmd_b_request_active_source:
		  {
			char phy[2];
			if (!broadcast) break;
			phy[0] = (cec.phy_address & 0xff00)>>8;
			phy[1] = cec.phy_address & 0xff;
			//generate an event on power key
			cec.cec_source_power = 0x0;
			if (cec.active_source)
				reply(cec_cmd_b_active_source, phy, 2, CEC_BROADCAST);
		  }
		  break;
		  case cec_cmd_b_active_source:
		  {
			if (!broadcast || (rx_data->rx_count != 2)) break;
			if ((rx_data->rx_operand[0] == 0x0) && (rx_data->rx_operand[1] == 0x0)) {
				//TV is now the source, request to be the active source
				CEC_DBG("Active source is the TV\n");
				set_current_active_source(0);
				request_vendor_id();
			} else if ((rx_data->rx_operand[0] != ((cec.phy_address & 0xff00)>>8)) && (rx_data->rx_operand[1] != (cec.phy_address & 0xff))) {
				CEC_DBG("Active source is not the TV\n");
				set_current_active_source(0);
			}
		  }
		  break;
		  case cec_cmd_b_routing_change:
		  {
			if (!broadcast || (rx_data->rx_count != 4)) break;
			if ((rx_data->rx_operand[2] != ((cec.phy_address & 0xff00)>>8)) || (rx_data->rx_operand[3] != (cec.phy_address & 0xff))) {
				CEC_DBG("Device route disabled\n");
				set_current_active_source(0);
			 } else {
				CEC_DBG("Device route enabled\n");
				set_current_active_source(1);
				request_vendor_id();
			 }
		  }
		  break;
		  case cec_cmd_b_routing_info:
		  {
			 if (!broadcast || (rx_data->rx_count != 2)) break;
			 if ((rx_data->rx_operand[0] != ((cec.phy_address & 0xff00)>>8)) || (rx_data->rx_operand[1] != (cec.phy_address & 0xff))) {
				CEC_DBG("Device route disabled\n");
				set_current_active_source(0);
			 } else {
				CEC_DBG("Device route enabled\n");
				set_current_active_source(1);
				request_vendor_id();
			 }
		  }
		  break;
		  case cec_cmd_u_menu_request:
		  {
			char cmd;
			if (broadcast || (rx_data->rx_count != 1)) break;
			cmd = rx_data->rx_operand[0];
			if (cmd > 2) break;
			cec.menu_mode = 0;
			reply(cec_cmd_u_menu_status, &cec.menu_mode, 1, rx_data->init_device_id);
		  }
		  break;
		  case cec_cmd_u_give_deck_status:
		  {
			char cmd, resp;
			if (broadcast || (rx_data->rx_count != 1)) break;
			cmd = rx_data->rx_operand[0];
			if (cmd ==0 || cmd > 3) break;
			if (cec.tv_vendorId == CEC_VENDOR_LG) resp = 0x20;
			else resp = 0x1A;
			reply(cec_cmd_u_deck_status, &resp, 1, rx_data->init_device_id);
			if (cec.tv_vendorId == CEC_VENDOR_LG) {
				if (cmd == 1) report_initial_status();
			}
		  }
		  break;
		  case cec_cmd_b_set_stream_path:
		  {
			char cmd[2];
			if (!broadcast || (rx_data->rx_count != 2)) break;
			cmd[0] = rx_data->rx_operand[0];
			cmd[1] = rx_data->rx_operand[1];
			if ((cmd[0] == ((cec.phy_address & 0xff00)>>8)) && (cmd[1] == (cec.phy_address & 0xff))) {
				cec.cec_source_power = 0x0;
				reply(cec_cmd_b_active_source, cmd, 2, CEC_BROADCAST);
				reply(cec_cmd_u_image_view_on, NULL, 0, cec_id_list[CEC_TV].ids[0]);
				reply(cec_cmd_u_text_view_on, NULL, 0, cec_id_list[CEC_TV].ids[0]);
			}
		  }
		  break;
		  case cec_cmd_u_feature_abort:
		  {
			if (broadcast) break;
			if (rx_data->rx_count > 2) break;
			if (rx_data->rx_operand[0] == cec_cmd_u_give_device_vendor_id) {
				report_initial_status();
			} else {
				switch(cec.tv_vendorId) {
				  case CEC_VENDOR_LG: {
					if (rx_data->rx_count == 0) {
					reply(cec_cmd_u_report_power_status, &cec.cec_source_power, 1, cec_id_list[CEC_TV].ids[0]);
					reply(cec_cmd_b_device_vendor_id, cec.vendor_id, 3, CEC_BROADCAST);
					}
				  }
				  break;
				  default:
				  break;
				}
			}
		  }
		  break;
		  case cec_cmd_u_vendor_command:
		  {
			if (broadcast) break;
			switch(cec.tv_vendorId) {
			  case CEC_VENDOR_LG:
				if ((rx_data->rx_count == 1) && (rx_data->rx_operand[0] == 0x1)) {
					//Handle LG Power up sequence
					char resp[2];
					resp[0] = 0x2; //Unknow error 2
					resp[1] = 0x5; //Command_type_HDDRecorder
					reply(cec_cmd_u_vendor_command, resp, 2, cec_id_list[CEC_TV].ids[0]);
				}
				if ((rx_data->rx_count == 2) && (rx_data->rx_operand[0] == 0x3)) {
					char resp[1];
					cec.cec_source_power = 0x2;
					resp[0] = cec.cec_source_power; //standby to on
					reply(cec_cmd_u_report_power_status, resp, 1, cec_id_list[CEC_TV].ids[0]);
					msleep(2);
					cec.cec_source_power = 0x0;
					resp[0] = cec.cec_source_power; //on
					reply(cec_cmd_u_report_power_status, resp, 1, cec_id_list[CEC_TV].ids[0]);
				}
				if ((rx_data->rx_count == 2) && (rx_data->rx_operand[0] == 0x4)) {
					char resp[1];
					resp[0] = 0x5;
					resp[1] = cec_id_list[CEC_RECORDING].ids[0];
					reply(cec_cmd_u_vendor_command, resp, 2, cec_id_list[CEC_TV].ids[0]);
				}
				if ((rx_data->rx_count == 1) && (rx_data->rx_operand[0] == 0xa0)) {
					char resp[1];
					cec.cec_source_power = 0x2;
					resp[0] = cec.cec_source_power; //standby to on
					reply(cec_cmd_u_report_power_status, resp, 1, cec_id_list[CEC_TV].ids[0]);
				}
			  break;
			  default: break;
			}
		  }
		  break;
		  default:
		  {
			  char error[2];
			  if (broadcast) break;
			  error[0] = rx_data->rx_cmd;
			  error[1] = 0;
			  reply(cec_cmd_u_feature_abort, error, 2, rx_data->init_device_id);
			  DUMP_CEC dump_rx_data(rx_data);
			  CEC_ERROR("Unsupported CEC command 0x%x\n", rx_data->rx_cmd);
		  }
		  break;
		}
	} else {
		CEC_DBG("CEC command 0x%x for other device\n", rx_data->rx_cmd);
	}
	return 0;
}

static void cec_rx_worker(struct work_struct *work)
{
	struct cec_worker_data *d = container_of(work, typeof(*d), dwork.work);
	struct cec_rx_data rx_data;
	CEC_DBG("cec_rx_worker ++\n");
	while (!cec_read_rx_cmd(&rx_data) ) {
		CEC_DBG("Receive command 0x%x\n", rx_data.rx_cmd);
		cec_process_cmd(&rx_data);
	}

}

static void cec_key_worker(struct work_struct *work)
{
	struct key_worker_data *d = container_of(work, typeof(*d), dwork.work);
	int keycode = -1;
	mutex_lock(&cec.lock);
	keycode = atomic_read(&cec_key_work.keycode);
	if (keycode != -1) {
		input_report_key(cec.input_dev, keycode, 0);
		input_sync(cec.input_dev);
		cec.dev.last_ui_button = -1;
	}
	mutex_unlock(&cec.lock);
}

static void cec_disconnect_worker(struct work_struct *work)
{
	struct cec_disconnect_worker_data *d = container_of(work, typeof(*d), dwork.work);
	mutex_lock(&cec.lock);
	cec.power_on = 0;
	mutex_unlock(&cec.lock);
}

void cec_irq_cb(int irq)
{
	u32 core_int = 0;
	u32 cec_int = 0;

	/*Check if core interrupt*/
	core_int = RD_FIELD_32(cec.hdmi_wp_base_addr, HDMI_WP_IRQSTATUS, 0, 0);
	if (core_int) {
		/*Check if came from CEC*/
		cec_int = RD_FIELD_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_SYSTEM,
			HDMI_CORE_SYS_INTR4, 3, 3);
		if (cec_int) {

			u32 cec_rx = 0;
			/*Currently only RX interrupt is handled*/
			cec_rx = RD_FIELD_32(cec.hdmi_wp_base_addr +
				HDMI_IP_CORE_CEC, HDMI_CEC_INT_STATUS_0, 1, 0);
			/*if command present in FIFO*/
			if (cec_rx) {

				__cancel_delayed_work(&cec_work.dwork);
				queue_delayed_work(my_workq, &cec_work.dwork,
					1);
				/*clear CEC RX interrupts*/
				WR_FIELD_32(cec.hdmi_wp_base_addr +
					HDMI_IP_CORE_CEC, HDMI_CEC_INT_STATUS_0,
					1, 0, cec_rx);
			}

			/*Clear CEC interrupt from core*/
			cec_int = WR_FIELD_32(cec.hdmi_wp_base_addr +
					HDMI_IP_CORE_SYSTEM,
					HDMI_CORE_SYS_INTR4, 3, 3, 1);

		}
		/*Leave core interrupt as is, this should be reset by HDMI
		 driver*/
	}
	return;
}
void cec_power_on_cb(int phy_addr, int status)
{
	int temp;
	CEC_DBG("cec_power_on_cb ++\n");
	if (status) {
		__cancel_delayed_work(&cec_disconnect_work.dwork);
	} else {
		//After 5 seconds in disconnect state, consider the cable as unplugged
		queue_delayed_work(my_workq, &cec_disconnect_work.dwork,msecs_to_jiffies(5000));
		switch_set_state(&cec.rx_switch, 0);
		return;
	}
	cec_request_dss();
	cec.phy_address = phy_addr;
	//On cable connect, consider we should be the active source.
	set_current_active_source(0);
	//Look at http://standards.ieee.org/develop/regauth/oui/oui.txt
	//Archos = 0x0016DC
	cec.vendor_id[0] = 0x00;
	cec.vendor_id[1] = 0x16;
	cec.vendor_id[2] = 0xDC;
	/*Clear TX FIFO*/
	WR_FIELD_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC, HDMI_CEC_DBG_3,
			7, 7, 1);
	/*Clear RX FIFO*/
	WR_REG_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC,
			HDMI_CEC_RX_CONTROL, 0x3);

	/*Clear CEC interrupts*/
	WR_REG_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC,
		HDMI_CEC_INT_STATUS_1, RD_REG_32(cec.hdmi_wp_base_addr +
		HDMI_IP_CORE_CEC, HDMI_CEC_INT_STATUS_1));
	WR_REG_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_CEC,
		HDMI_CEC_INT_STATUS_0, RD_REG_32(cec.hdmi_wp_base_addr +
		HDMI_IP_CORE_CEC, HDMI_CEC_INT_STATUS_0));

	/*Enable HDMI core interrupts*/
	WR_FIELD_32(cec.hdmi_wp_base_addr, HDMI_WP_IRQENABLE_SET,
			0, 0, 1);

	WR_FIELD_32(cec.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		HDMI_CORE_SYS_UMASK4, 3, 3, 0x1);




	/*Enable CEC interrupts*/
	/*command being received event*/
	WR_FIELD_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
		HDMI_CEC_INT_ENABLE_0, 0, 0, 1);
	/*RX fifo not empty event*/
	WR_FIELD_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
		HDMI_CEC_INT_ENABLE_0, 1, 1, 1);


	/*Initialize CEC clock divider*/
	/*CEC needs 2MHz clock hence set the devider to 24 to get
	48/24=2MHz clock*/
	WR_FIELD_32(cec.hdmi_wp_base_addr, HDMI_WP_WP_CLK, 5, 0, 0x18);

	/*Remove BYpass mode*/

	temp = RD_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
		HDMI_CEC_SETUP);
	if (FLD_GET(temp, 4, 4) != 0) {
		temp = FLD_MOD(temp, 0, 4, 4);
		WR_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
			HDMI_CEC_SETUP, temp);

		/* If we enabled CEC in middle of a CEC messages on CEC n/w,
			we will could have start bit irregularity and/or short
			pulse event. Clear them now */
		temp = RD_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
			HDMI_CEC_INT_STATUS_1);
		temp = FLD_MOD(0x0, 0x5, 2, 0);
		WR_REG_32(cec.hdmi_wp_base_addr+HDMI_IP_CORE_CEC,
			HDMI_CEC_INT_STATUS_1, temp);
	}
	cec_register_device(&cec.dev);
	cec_release_dss();
	CEC_DBG("cec_power_on_cb--\n");
	return;
}
static int __init cec_init(void)
{
	CEC_DBG("cec_init() %u", jiffies_to_msecs(jiffies));

	/* Map HDMI WP address */
	cec.hdmi_wp_base_addr = ioremap(HDMI_WP, 0x1000);
	CEC_DBG("hdmi_wp_addr=%p\n", cec.hdmi_wp_base_addr);
	if (!cec.hdmi_wp_base_addr) {
		printk(KERN_ERR "CEC: HDMI WP IOremap error\n");
		return -EFAULT;
	}
	cec.cec_rx_data_valid = 0;
	cec.power_on = 0;
	cec.dev.last_ui_button = -1;
	cec.dev.device_id = cec_id_list[CEC_UNREGISTERED].ids[0];
	cec.dev.type = CEC_PLAYBACK;
	mdev.minor = MISC_DYNAMIC_MINOR;
	mdev.name = "cec";
	mdev.mode = 0666;
	mdev.fops = &cec_fops;

	if (misc_register(&mdev)) {
		printk(KERN_ERR "CEC: Could not add character driver\n");
		goto err_register;
	}

	if (register_cec_input_device()) {
		printk(KERN_ERR "CEC: Could not add input driver\n");
		goto err_input_register;
	}
	printk(KERN_ERR "register call backs\n");
	cec.always_active_source = omapdss_hdmi_is_auto_displayed();
	omapdss_hdmi_register_cec_callbacks(&cec_power_on_cb, &cec_irq_cb);
	mutex_init(&cec.lock);
	cec.switch_state = 0;
	cec.rx_switch.name = "cec";
	switch_dev_register(&cec.rx_switch);

	my_workq = create_singlethread_workqueue("cec");
	INIT_DELAYED_WORK(&cec_work.dwork, cec_rx_worker);

	my_key_workq = create_singlethread_workqueue("cec_key");
	INIT_DELAYED_WORK(&cec_key_work.dwork, cec_key_worker);

	my_cec_workq = create_singlethread_workqueue("cec_disconnect");
	INIT_DELAYED_WORK(&cec_disconnect_work.dwork, cec_disconnect_worker);

	return 0;

err_input_register:
	misc_deregister(&mdev);
err_register:
	iounmap(cec.hdmi_wp_base_addr);

	return -EFAULT;
}

/*-----------------------------------------------------------------------------
 * Function: cec_exit
 *-----------------------------------------------------------------------------
 */
static void __exit cec_exit(void)
{
	CEC_DBG("cec_exit() %u", jiffies_to_msecs(jiffies));

	misc_deregister(&mdev);
	switch_dev_unregister(&cec.rx_switch);
	unregister_cec_input_device();

	/* Unmap HDMI WP / DESHDCP */
	iounmap(cec.hdmi_wp_base_addr);
}

/*-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
module_init(cec_init);
module_exit(cec_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OMAP CEC kernel module");
MODULE_AUTHOR("Muralidhar Dixit");
