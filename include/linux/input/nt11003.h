#ifndef 	_LINUX_NT11003_TOUCH_H
#define		_LINUX_NT11003_TOUCH_H

#define NT11003_NAME "nt11003"
#define NT11003_I2C_ADDR  (0x01)

struct nt11003_platform_data {
	int irq;
	int flags;
	const char * regulator;

	int x_max;
	int y_max;
	int max_fingers;

	void (*reset)(int);
	int (*get_irq_level)(void);
	int (*demux_i2c)(int);
};

#endif
