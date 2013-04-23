/*
 *    goodix-gt8105.h : 07/06/2012
 *    g.revaillot, revaillot@archos.com
 */

#include <linux/earlysuspend.h>
#include <linux/proc_fs.h>

struct gt8105_priv {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	struct regulator *regulator;

	int irq;

	int x_max;
	int y_max;
	int max_fingers;

	int flags;

	void (*reset)(int);
	int (*get_irq_level)(void);
	int (*demux_i2c)(int);

	u16 old_finger_status;
	int state;

	struct proc_dir_entry * proc_entry;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

enum {
	ST_IDLING = 0,		// std mode, not allowed to report events.
	ST_RUNNING = 1,		// std mode, reporting invents.
	ST_SUSPEND = 2,		// suspend mode, not powered.
	ST_UNKNOWN = 3,		// unconfigured mode
	ST_ISP = 4,		// isp
};

#ifdef CONFIG_TOUCHSCREEN_GOODIX_GT8105_DFU
int gt8105_register_dfu(struct i2c_client *client);
void gt8105_release_dfu(struct i2c_client *client);
#else
int gt8105_register_dfu(struct i2c_client *client) { return 0; }
void gt8105_release_dfu(struct i2c_client *client) { return; }
#endif

int gt8105_write(struct i2c_client * client, u8 addr, u8 *value, u8 len);
int gt8105_write_u8(struct i2c_client * client, u8 addr, u8 value);

int gt8105_read(struct i2c_client * client, u8 addr, u8 *value, u8 len);
int gt8105_read_u8(struct i2c_client * client, u8 addr, u8 *value);

int gt8105_read_version(struct i2c_client *client, char *version, int vlen);
