/*
 * archos-jm20329.c
 *
 *  Created on: Feb 8, 2011
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/usb/jm20329.h>

#include <mach/board-archos.h>

#include "mux.h"

#define DEVICE_NAME	"jm20329"
//#define JM20329_DEBUG

#define SATA_WAKEUP_RETRIES 5
#define SATA_SUSPEND_RETRIES 5

static struct jm20329_data {
	struct clk *auxclk1;
	struct clk *aux_src_clk;
	struct clk *aux_parent_clk;
	struct regulator *sata_vcc;
	struct regulator *hdd_5v;
	struct regulator *hub_vcc;
	int gpio_hdd_power;
	int gpio_sata_power;
	int gpio_sata_ready;
	const char *mux_hdd_power;
	const char *mux_sata_power;
	const char *mux_sata_ready;
} jm20329_data; 

static void remux_regulator_gpio(int gpio)
{
	if (gpio == jm20329_data.gpio_hdd_power && jm20329_data.mux_hdd_power)
		omap_mux_init_signal(jm20329_data.mux_hdd_power, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
	else if (gpio == jm20329_data.gpio_sata_power && jm20329_data.mux_sata_power)
		omap_mux_init_signal(jm20329_data.mux_sata_power, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
	else
		omap_mux_init_gpio(gpio, OMAP_PIN_INPUT|OMAP_PIN_OUTPUT);
}

/*
 * HDD_5V supply, consumer of VBATT (not a regulator!)
 */
static struct regulator_consumer_supply hdd_5v_consumer[] = {
	REGULATOR_SUPPLY("hdd_5v", "jm20329"),
};
static struct regulator_init_data fixed_reg_hdd_5v_initdata = {
	.constraints = {
		.min_uV = 5000000,
		.max_uV = 5000000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	
	.consumer_supplies = hdd_5v_consumer,
	.num_consumer_supplies = ARRAY_SIZE(hdd_5v_consumer),
};
static struct fixed_voltage_config fixed_reg_hdd_5v = {
	.supply_name	= "HDD_5V",
	.microvolts	= 5000000,
	.gpio		= -EINVAL,
	.enable_high	= 1,
	.enabled_at_boot= 0,
	.init_data	= &fixed_reg_hdd_5v_initdata,
	.remux		= remux_regulator_gpio,
};
static struct platform_device fixed_supply_hdd_5v = {
	.name 	= "reg-fixed-voltage",
	.id	= 98,
	.dev.platform_data = &fixed_reg_hdd_5v,
};

/* 
 * SATA_VCC supply, consumer of VCC
 */
static struct regulator_consumer_supply sata_vcc_consumer[] = {
	REGULATOR_SUPPLY("sata_vcc", "jm20329"),
};
static struct regulator_init_data fixed_reg_sata_vcc_initdata = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	
	.supply_regulator = "VCC",
	.consumer_supplies = sata_vcc_consumer,
	.num_consumer_supplies = ARRAY_SIZE(sata_vcc_consumer),
};
static struct fixed_voltage_config fixed_reg_sata_vcc = {
	.supply_name	= "SATA_VCC",
	.microvolts	= 3300000,
	.gpio		= -EINVAL,
	.enable_high	= 1,
	.enabled_at_boot= 0,
	.init_data	= &fixed_reg_sata_vcc_initdata,
	.remux		= remux_regulator_gpio,
};
static struct platform_device fixed_supply_sata_vcc = {
	.name 	= "reg-fixed-voltage",
	.id	= 97,
	.dev.platform_data = &fixed_reg_sata_vcc,
};

static int __init jm20329_platform_init(struct jm20329_device *jm20329);
static int jm20329_platform_release(struct jm20329_device *jm20329);
static int jm20329_platform_disconnect(struct jm20329_device *jm20329);
static int jm20329_platform_wakeup(struct jm20329_device *jm20329);

static struct jm20329_platform_driver jm20329 = {
	.wakeup = jm20329_platform_wakeup,
	.disconnect = jm20329_platform_disconnect,
	.init = jm20329_platform_init,
	.release = jm20329_platform_release,
	.interface_string = "1-1.1:1.0",
};

static struct platform_device jm20329_pdev = {
	.name			= "jm20329",
	.id			= -1,
	.dev.platform_data	= &jm20329,
};

static struct platform_device *sata_devices[] __initdata = {
	&fixed_supply_sata_vcc,
	&fixed_supply_hdd_5v,
};

static int __init init_sata_clk(struct jm20329_device *jm20329)
{
	struct jm20329_data *priv = jm20329->data;
	const char *aux_name = "auxclk1_ck";
	const char *aux_src_name = "auxclk1_src_ck";
	const char *aux_parent_name = "dpll_per_m3x2_ck";
	int ret = 0;
	
	if (cpu_is_omap34xx()) {
		priv->auxclk1 = clk_get(&jm20329_pdev.dev, "sys_clkout2");
		if (IS_ERR(priv->auxclk1)) {
			pr_err("%s: sys_clkout2 not available\n", __func__);
			return PTR_ERR(priv->auxclk1);
		}

		clk_enable(priv->auxclk1);

		return 0;
	}

	priv->auxclk1 = clk_get(&jm20329_pdev.dev, aux_name);
	if (IS_ERR(priv->auxclk1)) {
		pr_err("%s: auxclk1 not available\n", __func__);
		return PTR_ERR(priv->auxclk1);
	}
	
	priv->aux_src_clk = clk_get(&jm20329_pdev.dev, aux_src_name);
	if (IS_ERR(priv->aux_src_clk)) {
		ret = PTR_ERR(priv->aux_src_clk);
		pr_err("%s: %s is not available\n", __func__, aux_src_name);
		goto err;
	}

	priv->aux_parent_clk = clk_get(&jm20329_pdev.dev, aux_parent_name);
	if (IS_ERR(priv->aux_parent_clk)) {
		ret = PTR_ERR(priv->aux_parent_clk);
		pr_err("%s: %s is not available\n", __func__, aux_parent_name);
		goto err1;
	}
	
	pr_debug("%s: %s freq is: %lu\n", 
			__func__, aux_parent_name, clk_get_rate(priv->aux_parent_clk));
	
	if (IS_ERR_VALUE(clk_set_rate(priv->aux_parent_clk, 192000000))) {
		pr_err("%s: cannot set %s to 192MHz\n", __func__, aux_parent_name);
		goto err2;
	}
	
	pr_debug("%s: auxclk1 freq is: %lu\n", 
			__func__, clk_get_rate(priv->auxclk1));
	
	if (IS_ERR_VALUE(clk_set_parent(priv->aux_src_clk, priv->aux_parent_clk))) {
		pr_err("%s: cannot set auxclk1 clock parent\n", __func__);
		goto err2;
	}

	if (IS_ERR_VALUE(clk_set_rate(priv->auxclk1, 12000000))) {
		pr_err("%s: cannot set auxclk1 clock rate\n", __func__);
		goto err2;
	}

	pr_debug("%s: auxclk1 now at %lu\n", __func__, clk_get_rate(priv->auxclk1));

	clk_enable(priv->auxclk1);
	omap_mux_init_signal("fref_clk1_out", OMAP_PIN_OUTPUT);
	
	clk_put(priv->aux_src_clk);
	priv->aux_src_clk = NULL;
	clk_put(priv->aux_parent_clk);
	priv->aux_parent_clk = NULL;
	return 0;
err2:
	clk_put(priv->aux_parent_clk);
err1:
	clk_put(priv->aux_src_clk);
err:
	clk_put(priv->auxclk1);

	return ret;
}

static ssize_t set_timeout_reset(struct device* dev,
		struct device_attribute *attr, const char* buf, size_t count)
{
	struct jm20329_device *jm20329 = dev_get_drvdata(dev);
	long t=0;
	int len;

	if (!jm20329)
		return count;

	len = strict_strtol(buf, 10, &t);
	if (t<1000) {
		printk("timeout has to be >1000ms, setting to 1000ms\n");
		t = 1000;
	}
	mutex_lock(&jm20329->dev_mutex);
	jm20329->timeout_reset = t;
	jm20329->timeout = t;
	mutex_unlock(&jm20329->dev_mutex);
	return count;
}

static ssize_t show_timeout_reset(struct device *dev,
		struct device_attribute *attr, char* buf)
{
	struct jm20329_device *jm20329 = dev_get_drvdata(dev);
	long t;

	if (!jm20329)
		return 0;
	mutex_lock(&jm20329->dev_mutex);
	t = jm20329->timeout_reset;	
	mutex_unlock(&jm20329->dev_mutex);
	return snprintf(buf, PAGE_SIZE,  "%ld\n", t);
}

static ssize_t set_timeout(struct device* dev,
		struct device_attribute *attr, const char* buf, size_t count)
{
	struct jm20329_device *jm20329 = dev_get_drvdata(dev);
	long t=0;
	int len;

	if (!jm20329)
		return count;

	len = strict_strtol(buf, 10, &t);
	mutex_lock(&jm20329->dev_mutex);
	jm20329->timeout = t;
	mutex_unlock(&jm20329->dev_mutex);
	return count;
}

static ssize_t show_timeout(struct device *dev,
		struct device_attribute *attr, char* buf)
{
	struct jm20329_device *jm20329 = dev_get_drvdata(dev);
	long t;

	if (!jm20329)
		return 0;

	mutex_lock(&jm20329->dev_mutex);
	t = jm20329->timeout;
	mutex_unlock(&jm20329->dev_mutex);
	return snprintf(buf, PAGE_SIZE,  "%ld\n", t);
}

static ssize_t set_sata_vcc(struct device *dev,
		struct device_attribute *attr, const char* buf, size_t count)
{
	struct jm20329_device *jm20329 = dev_get_drvdata(dev);
	struct jm20329_data *priv;

	if (!jm20329)
		return count;

	priv = jm20329->data;

	printk("%s: %s", __func__, buf);
	if (!strncmp(buf, "on", 2))
		{
			regulator_enable(priv->sata_vcc);
			printk("%s: Enabled SATA_VCC\n", __func__);
			return count;
		}
	if (!strncmp(buf, "off", 3)) {
			regulator_disable(priv->sata_vcc);
			printk("%s: Disabled SATA_VCC\n", __func__);
			return count;
		}
	return count;
}

static ssize_t show_sata_vcc(struct device* dev,
		struct device_attribute * attr, char* buf)
{
	struct jm20329_device *jm20329 = dev_get_drvdata(dev);
	struct jm20329_data *priv;
	int enabled;

	if (!jm20329)
		return 0;

	priv = jm20329->data;

	enabled = regulator_is_enabled(priv->sata_vcc);
	return snprintf(buf, PAGE_SIZE, "%s\n", enabled>0?"on":((enabled<0)?"err":"on"));
}

#ifdef JM20329_DEBUG
static ssize_t set_clock(struct device* dev,
		struct device_attribute * attr, const char* buf, size_t count)
{
	struct jm20329_device *jm20329 = dev_get_drvdata(dev);
	struct jm20329_data *priv;

	if (!jm20329)
		return count;

	priv = jm20329->data;
	printk("%s: %s", __func__, buf);
	if (!strncmp(buf, "on", 2))
		{
			clk_enable(priv->auxclk1);
			printk("%s: Enabled auxclk1\n", __func__);
			return count;
		}
	if (!strncmp(buf, "off", 3)) {
			clk_disable(priv->auxclk1);
			printk("%s: Disabled auxclk1\n", __func__);
			return count;
		}
	return count;
}

static ssize_t set_hub(struct device* dev,
		struct device_attribute * attr, const char* buf, size_t count)
{
	struct jm20329_device *jm20329 = dev_get_drvdata(dev);
	struct jm20329_data *priv;

	if (!jm20329)
		return count;

	priv = jm20329->data;
	printk("%s: %s", __func__, buf);
	if (!strncmp(buf, "on", 2))
		{
			regulator_enable(priv->hub_vcc);
			printk("%s: Enabled hub_vcc\n", __func__);
			return count;
		}
	if (!strncmp(buf, "off", 3)) {
			regulator_disable(priv->hub_vcc);
			printk("%s: Disabled hub_vcc\n", __func__);
			return count;
		}
	return count;
}

#endif /* JM20329_DEBUG */

static ssize_t show_sata_ready(struct device* dev,
		struct device_attribute * attr, char* buf)
{
	struct jm20329_device *jm20329 = dev_get_drvdata(dev);
	struct jm20329_data *priv;
	int on = -1;

	if (!jm20329)
		return 0;

	priv = jm20329->data;
	if (gpio_is_valid(priv->gpio_sata_ready))
		on = gpio_get_value(priv->gpio_sata_ready);
	return snprintf(buf, PAGE_SIZE, "%i\n", on);
}


static DEVICE_ATTR(timeout_counter, S_IWUSR | S_IRUGO, show_timeout, set_timeout);
static DEVICE_ATTR(timeout, S_IWUSR | S_IRUGO, show_timeout_reset, set_timeout_reset);
static DEVICE_ATTR(sata_ready, S_IRUGO, show_sata_ready, NULL);
static DEVICE_ATTR(sata_vcc, S_IWUSR | S_IRUGO, show_sata_vcc, set_sata_vcc);
#ifdef JM20329_DEBUG
static DEVICE_ATTR(clock, S_IWUSR , NULL, set_clock);
static DEVICE_ATTR(hub_vcc, S_IWUSR, NULL, set_hub);
#endif //JM20329_DEBUG

static void release_sata_clk(struct jm20329_device *jm20329)
{
	struct jm20329_data *priv = jm20329->data;

	clk_put(priv->auxclk1);
	if (priv->aux_src_clk)
		clk_put(priv->aux_src_clk);
	if (priv->aux_parent_clk)
		clk_put(priv->aux_parent_clk);
}

static int jm20329_platform_disconnect(struct jm20329_device *jm20329)
{
	struct jm20329_data *priv = jm20329->data;
	int retries = 0;

	if (gpio_is_valid(priv->gpio_hdd_power)) {
		while ((gpio_get_value(priv->gpio_hdd_power)) && (retries++ < SATA_SUSPEND_RETRIES)) {
			// Wait for the hdd to be powered off by the controller
			msleep(100);
		}
	}

	/* force turn off of hdd */
	gpio_direction_output(priv->gpio_hdd_power, 0);

	if (priv->auxclk1)
		clk_disable(priv->auxclk1);

	if (priv->sata_vcc)
		regulator_disable(priv->sata_vcc);

	msleep(100);	// We need to wait for the controller to really shut off

	if (priv->hub_vcc)
		regulator_disable(priv->hub_vcc);

	msleep(100);

	return 0;
}

extern void archos_hub_enable(bool en);
static int jm20329_platform_wakeup(struct jm20329_device *jm20329)
{
	int retries = 0;
	struct jm20329_data *priv = jm20329->data;

	/* Force hhd on */
	gpio_direction_output(priv->gpio_hdd_power, 1);

	gpio_direction_input(priv->gpio_sata_ready);

	if (priv->auxclk1)
		clk_enable(priv->auxclk1);
	
	msleep(10);	// Wait for a stable clock before enabling vcc

	if (priv->sata_vcc)
		regulator_enable(priv->sata_vcc);

	msleep(100);

	if (priv->hub_vcc)
		regulator_enable(priv->hub_vcc);

	msleep(100);

	if (gpio_is_valid(priv->gpio_sata_ready)) {
		while ((!gpio_get_value(priv->gpio_sata_ready)) && (retries++ < SATA_WAKEUP_RETRIES))
		{
			printk("JM20329 reset error - trying again\n");
			regulator_disable(priv->sata_vcc);
			msleep(150);
			regulator_enable(priv->sata_vcc);
			msleep(100);
		}
	}

	/* hand over hdd to sata controller */
	gpio_direction_input(priv->gpio_hdd_power);

	return 0;
}

static int jm20329_platform_release(struct jm20329_device *jm20329)
{
	struct jm20329_data *priv = &jm20329_data;

	if (priv->sata_vcc) {
		if (regulator_is_enabled(priv->sata_vcc))
			regulator_disable(priv->sata_vcc);
		regulator_put(priv->sata_vcc);
	}
#ifdef USE_HDD_5V_REGULATOR
	if (priv->hdd_5v) {
		if (regulator_is_enabled(priv->hdd_5v))
			regulator_disable(priv->hdd_5v);
		regulator_put(priv->hdd_5v);
	}
#endif
	if (priv->hub_vcc) {
		if (regulator_is_enabled(priv->hub_vcc))
			regulator_disable(priv->hub_vcc);
		regulator_put(priv->hub_vcc);
	}
	if (priv->auxclk1)
		clk_put(priv->auxclk1);
	if (priv->aux_src_clk)
		clk_put(priv->aux_src_clk);
	if (priv->aux_parent_clk)
		clk_put(priv->aux_parent_clk);

	return 0;
}

static int __init jm20329_platform_init(struct jm20329_device *jm20329)
{
	struct jm20329_data *priv = &jm20329_data;
	const struct archos_sata_config *sata_config;
	const struct archos_sata_conf *conf;
	int ret;

	jm20329->data = priv;
	priv->gpio_hdd_power = -1;
	priv->gpio_sata_power = -1;
	priv->gpio_sata_ready = -1;
	
	sata_config = omap_get_config(ARCHOS_TAG_SATA,
			struct archos_sata_config);
	conf = hwrev_ptr(sata_config, system_rev);
	if (IS_ERR(conf)) {
		pr_err("%s: no device configuration for system_rev %i\n",
				__func__, system_rev);	
		return -ENODEV;
	}
	
	if ((ret = init_sata_clk(jm20329)) < 0)
		goto initfail1;

	if (gpio_is_valid(conf->sata_power)) {
		fixed_reg_sata_vcc.gpio = conf->sata_power;
		priv->gpio_sata_power = conf->sata_power;
		priv->mux_sata_power = conf->sata_power_mux;
	}

	if (gpio_is_valid(conf->hdd_power)) {
		if (!conf->usb_suspend) {
			fixed_reg_hdd_5v.gpio = conf->hdd_power;
			priv->gpio_hdd_power = conf->hdd_power;
			priv->mux_hdd_power = conf->hdd_power_mux;
		} else {
			int ret = gpio_request(conf->hdd_power, "hdd_5v");
			if (!IS_ERR_VALUE(ret)) {
				gpio_direction_input(conf->hdd_power);
				priv->gpio_hdd_power = conf->hdd_power;
				priv->mux_hdd_power = conf->hdd_power_mux;
				remux_regulator_gpio(conf->hdd_power);
			}
		}
	}

	if (gpio_is_valid(conf->sata_ready)) {
		int ret = gpio_request(conf->sata_ready, "sata_ready");
		if (!IS_ERR_VALUE(ret)) {
			gpio_direction_input(conf->sata_ready);
			priv->gpio_sata_ready = conf->sata_ready;
			priv->mux_sata_ready = conf->sata_ready_mux;
			remux_regulator_gpio(priv->gpio_sata_ready);
		}
	}

	platform_add_devices(sata_devices, ARRAY_SIZE(sata_devices));

	priv->sata_vcc = regulator_get(&jm20329_pdev.dev, "sata_vcc");
	if (IS_ERR(priv->sata_vcc)) {
		pr_err("%s: cannot get sata_vcc supply\n", __func__);
		goto initfail2;
	}

#if 1
	regulator_enable(priv->sata_vcc);
#endif
	if (!conf->usb_suspend) {
#ifdef USE_HDD_5V_REGULATOR
		priv->hdd_5v = regulator_get(&jm20329_pdev.dev, "hdd_5v");
		if (IS_ERR(priv->hdd_5v)) {
			pr_err("%s: cannot get hdd_5v supply\n", __func__);
			goto initfail3;
		}
		regulator_enable(priv->hdd_5v);
#else
		gpio_direction_input(priv->gpio_hdd_power);
#endif
	}

	priv->hub_vcc = regulator_get(&jm20329_pdev.dev, "jm20329_hub");
	if (IS_ERR(priv->hub_vcc)) {
		pr_err("%s: cannot get hub_vcc supply\n", __func__);
		goto initfail4;
	}
	regulator_enable(priv->hub_vcc);

	ret = device_create_file(&jm20329_pdev.dev, &dev_attr_timeout);	
	if (ret) {
		printk("%s: Could not create sysfs entry\n", __func__);
		return ret;
	}
	ret = device_create_file(&jm20329_pdev.dev, &dev_attr_timeout_counter);	
	if (ret) {
		printk("%s: Could not create sysfs entry\n", __func__);
		return ret;
	}
	ret = device_create_file(&jm20329_pdev.dev, &dev_attr_sata_ready);
	if (ret) {
		printk("%s: Could not create sysfs entry\n", __func__);
		return ret;
	}

	ret = device_create_file(&jm20329_pdev.dev, &dev_attr_sata_vcc);	
	if (ret) {
		printk("%s: Could not create sysfs entry\n", __func__);
		return ret;
	}

#ifdef JM20329_DEBUG
	ret = device_create_file(&jm20329_pdev.dev, &dev_attr_clock);	
	if (ret) {
		printk("%s: Could not create sysfs entry\n", __func__);
		return ret;
	}
	ret = device_create_file(&jm20329_pdev.dev, &dev_attr_hub_vcc);
	if (ret) {
		printk("%s: Could not create sysfs entry\n", __func__);
		return ret;
	}
#endif //JM20329_DEBUG

	return 0;
initfail4:
#ifdef USE_HDD_5V_REGULATOR
	regulator_put(priv->hdd_5v);
#endif
initfail3:
	regulator_put(priv->sata_vcc);
initfail2:
	release_sata_clk(jm20329);
initfail1:
	return ret;
}

static int __init jm20329_init(void)
{
#if defined(CONFIG_USB_STORAGE_JM20329) || \
		defined(CONFIG_USB_STORAGE_JM20329_MODULE)
	if (cpu_is_omap34xx())
		jm20329.interface_string = "1-2.1:1.0";
	
	return platform_device_register(&jm20329_pdev);
#else
	struct jm20329_device dummy;
	int ret = platform_device_register(&jm20329_pdev);
	if (ret)
		return ret;
	ret = jm20329_platform_init(&dummy);
	if (ret)
		platform_device_del(&jm20329_pdev);
	return ret;
#endif
}

device_initcall(jm20329_init);
