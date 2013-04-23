#ifndef _JM20329_H
#define _JM20329_H

struct platform_device;
struct jm20329_device;
struct usb_device_id;
struct usb_driver;
struct us_data;

enum jm20329_status {
	JM20329_AWAKE = 0,
	JM20329_ASLEEP = 1,
	JM20329_DO_SLEEP = 2,
	JM20329_RELEASE = 3
};

enum jm20329_transition {
	JM20329_NONE = 0,
	JM20329_REQUESTED = 1,
	JM20329_ONGOING = 2,
};

struct jm20329_platform_driver {
	int (*disconnect)(struct jm20329_device *);
	int (*wakeup)(struct jm20329_device *);
	int (*probe)(struct jm20329_device *, const struct usb_device_id *);
	int (*init)(struct jm20329_device *);
	int (*release)(struct jm20329_device *);
	const char *interface_string;
};

struct jm20329_device {
	struct list_head node;
	struct platform_device *pdev;
	struct jm20329_platform_driver *pdrv;
	const struct usb_device_id *id;
	int state;
	struct usb_driver *udrv;
	bool configured;
	struct us_data *us;
	spinlock_t state_lock;
	long timeout;
	long timeout_reset;
	struct mutex dev_mutex;
	int in_transition;
	struct usb_device *parent;
	struct usb_device *roothub;
	void *data;
};

#endif /* _JM20329_H */
