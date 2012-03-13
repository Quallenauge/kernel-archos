#include <linux/types.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <plat/mux.h>
#include <mach/gpio.h>
#include <plat/board.h>
#include <plat/archos-gpio.h>
#include <mach/board-archos.h>

#ifdef CONFIG_POWER_SUPPLY
/* power supply abstraction for built-in battery */

struct archos_battery
{
	struct power_supply main_battery;
	struct power_supply ac_supply;
	struct power_supply usb_supply;
	int current_mV;
	int current_capacity;
	int charge_state;
	int charge_level;
	int on_dcin;
	int on_usb;
	enum charger_type_t charger_type;
	int charge_enable;
	int charge_low;
	int charge_high;
	int gpio_dc_detect;
	int dc_detect_invert;
	int old_dcin;
	struct delayed_work work;
	struct wake_lock wl;
};


static struct archos_battery *bat;

static int read_gpio_dc_detect(struct archos_battery *bat)
{
	if ( bat->gpio_dc_detect ) {
		int dc_detect = gpio_get_value_cansleep(bat->gpio_dc_detect);
		return bat->dc_detect_invert ? !dc_detect : dc_detect;
	} else
		return -1;
}

static int get_on_dcin(struct archos_battery *bat)
{
	if ( bat->gpio_dc_detect ) {
		return read_gpio_dc_detect(bat);
	} else
		return bat->on_dcin;
}

int archos_get_dcin_plug(void)
{
	if ( (bat != NULL) && (bat->gpio_dc_detect) )
		return ( bat->old_dcin ? 0 : read_gpio_dc_detect(bat) );

	return 0;
}
EXPORT_SYMBOL(archos_get_dcin_plug);

int archos_get_highcharge_level(void)
{
	if (bat != NULL)
		return ( bat->charge_level == 3 ? 1 : 0  );

	return 0;
}
EXPORT_SYMBOL(archos_get_highcharge_level);

static int ac_supply_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct archos_battery *bat = container_of(psy, struct archos_battery, ac_supply);
	
	if (psp == POWER_SUPPLY_PROP_ONLINE) {
		val->intval = get_on_dcin(bat);
		return 0;
	}
	return -EINVAL;
}

static int usb_supply_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct archos_battery *bat = container_of(psy, struct archos_battery, usb_supply);
	
	if (psp == POWER_SUPPLY_PROP_ONLINE) {
		val->intval = bat->on_usb;
		return 0;
	}
	return -EINVAL;
}

static int main_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct archos_battery *bat = container_of(psy, struct archos_battery, main_battery);
	
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!get_on_dcin(bat) && !bat->on_usb) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}
		if (bat->charge_state)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
		
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1; /* always present */
		break;
	
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bat->current_capacity;
		break;
		
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bat->current_mV;
		break;
		
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = bat->current_mV;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 230; /* we have no sensor for that */
		break;
		
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
		
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 4200;
		break;
		
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 3300;
		break;
		
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (!gpio_is_valid(bat->charge_enable)) {
			if (bat->charge_state) val->intval = 1000;
			else val->intval = 0;
			break;
		}
		
		switch(bat->charge_level) {
		case 1:
			val->intval = 100;
			break;
		case 2:
			val->intval = 500;
			break;
		case 3:
			val->intval = 1000;
			break;
		case 0:
		default:
			val->intval = 0;
			break;
		}
		break;
		
	default:
		return -EINVAL;
	}
	
	return 0;
}

static void archos_battery_external_power_changed(struct power_supply *psy)
{
}

static char *main_battery_supplied_to[] = {
	"battery",
};

static enum power_supply_property main_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
};

static struct power_supply main_battery_template __initdata = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = main_battery_properties,
	.num_properties = ARRAY_SIZE(main_battery_properties),
	.get_property = main_battery_get_property,
	.external_power_changed = archos_battery_external_power_changed,
};

static enum power_supply_property ac_supply_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply ac_supply_template __initdata = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = ac_supply_properties,
	.num_properties = ARRAY_SIZE(ac_supply_properties),
	.get_property = ac_supply_get_property,
	.external_power_changed = archos_battery_external_power_changed,
};

static struct power_supply usb_supply_template __initdata = {
	.name = "usb",
	.supplied_to = main_battery_supplied_to,
	.num_supplicants = ARRAY_SIZE(main_battery_supplied_to),
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = ac_supply_properties,
	.num_properties = ARRAY_SIZE(ac_supply_properties),
	.get_property = usb_supply_get_property,
	.external_power_changed = archos_battery_external_power_changed,
};

static ssize_t battery_show_voltage(struct device* dev, 
		    struct device_attribute *attr, char* buf)
{
	struct archos_battery *bat = dev_get_drvdata(dev);
	
	return snprintf(buf, PAGE_SIZE, "%i\n", bat->current_mV);
}
static ssize_t battery_store_voltage(struct device* dev,
		    struct device_attribute *attr, const char* buf, size_t len)
{
	struct archos_battery *bat = dev_get_drvdata(dev);
	
	int new_mV = simple_strtol(buf, NULL, 10);

	if (new_mV != bat->current_mV) {
		bat->current_mV = new_mV;
	}
	return len;
}
static DEVICE_ATTR(voltage_avg, S_IRUGO|S_IWUSR, battery_show_voltage, battery_store_voltage);

static ssize_t battery_show_capacity(struct device* dev, 
		    struct device_attribute *attr, char* buf)
{
	struct archos_battery *bat = dev_get_drvdata(dev);
	
	return snprintf(buf, PAGE_SIZE, "%i\n", bat->current_capacity);
}

static ssize_t battery_store_capacity(struct device* dev,
		    struct device_attribute *attr, const char* buf, size_t len)
{
	struct archos_battery *bat = dev_get_drvdata(dev);
	struct power_supply *psy = &bat->main_battery;
	
	int new_capacity = simple_strtol(buf, NULL, 10);

	if (new_capacity != bat->current_capacity) {
		bat->current_capacity = new_capacity;
		power_supply_changed(psy);
	}
	return len;
}
static DEVICE_ATTR(capacity, S_IRUGO|S_IWUSR, battery_show_capacity, battery_store_capacity);

static ssize_t ac_show_online(struct device* dev, 
		    struct device_attribute *attr, char* buf)
{
	struct archos_battery *bat = dev_get_drvdata(dev);
	
	return snprintf(buf, PAGE_SIZE, "%i\n", get_on_dcin(bat));
}
static ssize_t ac_store_online(struct device* dev,
		    struct device_attribute *attr, const char* buf, size_t len)
{
	struct archos_battery *bat = dev_get_drvdata(dev);
	int online = simple_strtol(buf, NULL, 10);
	switch (online) {
		case 0: // off
		case 1: // on
			break;
		default:
			return len;
	}
	if (bat->on_dcin != online) {
		bat->on_dcin = online;
		bat->charge_state = online;
		power_supply_changed(&bat->main_battery);
	}
	return len;
}
static DEVICE_ATTR(ac_online, S_IRUGO|S_IWUSR, ac_show_online, ac_store_online);

static ssize_t usb_show_online(struct device* dev, 
		    struct device_attribute *attr, char* buf)
{
	struct archos_battery *bat = dev_get_drvdata(dev);
	
	return snprintf(buf, PAGE_SIZE, "%i\n", bat->on_usb);
}
static ssize_t usb_store_online(struct device* dev,
		    struct device_attribute *attr, const char* buf, size_t len)
{
	struct archos_battery *bat = dev_get_drvdata(dev);
	int online = simple_strtol(buf, NULL, 10);
	switch (online) {
		case 0: // off
		case 1: // on
			bat->on_usb = online;
			break;
		default:
			return len;
	}
	return len;
}
static DEVICE_ATTR(usb_online, S_IRUGO|S_IWUSR, usb_show_online, usb_store_online);

static ssize_t usb_show_charger_type(struct device* dev, 
		    struct device_attribute *attr, char* buf)
{
	struct archos_battery *bat = dev_get_drvdata(dev);

	switch (bat->charger_type) {
	case CHARGER_LX2208:
		return snprintf(buf, PAGE_SIZE, "CHARGER_LX2208\n");
	case CHARGER_DCIN:
		return snprintf(buf, PAGE_SIZE, "CHARGER_DCIN\n");
	case CHARGER_RT9502:
		return snprintf(buf, PAGE_SIZE, "CHARGER_RT9502\n");
	case CHARGER_TWL6030USB:
		return snprintf(buf, PAGE_SIZE, "CHARGER_TWL6030USB\n");
	case CHARGER_TWL6030USB_DC:
		return snprintf(buf, PAGE_SIZE, "CHARGER_TWL6030USB_DC\n");
	case CHARGER_ISL9220:
		return snprintf(buf, PAGE_SIZE, "CHARGER_ISL9220\n");

	default:
		// return charger type num.
		break;
	}

	return snprintf(buf, PAGE_SIZE, "%i\n", bat->charger_type);
}
static DEVICE_ATTR(charger_type, S_IRUGO|S_IWUSR, usb_show_charger_type, NULL);

static ssize_t bat_charge_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct archos_battery *bat = dev_get_drvdata(dev);
	
	return snprintf(buf, PAGE_SIZE, "%i\n", bat->charge_level);
}

static int bat_charge_set_LX2208( int level )
{
	switch (level) {
	case 0: /* charger off */
		gpio_set_value_cansleep(bat->charge_enable, 1);
		return 0;
		
	case 1: /* charger low (100mA) */
		gpio_set_value_cansleep(bat->charge_enable, 1);
		gpio_set_value_cansleep(bat->charge_low, 0);
		gpio_set_value_cansleep(bat->charge_high, 0);
		gpio_set_value_cansleep(bat->charge_enable, 0);
		return 1;
		
	case 2: /* charge normal (500mA) */
		gpio_set_value_cansleep(bat->charge_enable, 1);
		gpio_set_value_cansleep(bat->charge_low, 1);
		gpio_set_value_cansleep(bat->charge_high, 0);
		gpio_set_value_cansleep(bat->charge_enable, 0);
		return 1;
		
	case 3: /* charger high (1000mA) */
		bat->charge_state = 1;
		gpio_set_value_cansleep(bat->charge_enable, 1);
		gpio_set_value_cansleep(bat->charge_low, 0);
		gpio_set_value_cansleep(bat->charge_high, 1);
		gpio_set_value_cansleep(bat->charge_enable, 0);
		return 1;
	}
	return -1;
}

static int bat_charge_set_RT9502( int level )
{
	switch (level) {
	case 0: /* charger off */
		gpio_set_value_cansleep(bat->charge_enable, 1);
		return 0;
		
	case 1: /* charger low (100mA) */
		gpio_set_value_cansleep(bat->charge_enable, 1);
		gpio_set_value_cansleep(bat->charge_low, 0);
		gpio_set_value_cansleep(bat->charge_enable, 0);
		return 1;
		
	case 2: /* charge normal (500mA) */
	case 3: /* treat high as 500mA, too */
		gpio_set_value_cansleep(bat->charge_enable, 1);
		gpio_set_value_cansleep(bat->charge_low, 1);
		gpio_set_value_cansleep(bat->charge_enable, 0);
		return 1;
	}
	return -1;
}

static int bat_charge_set_ISL9220( int level )
{
	switch (level) {
	case 0: /* charger off */
		gpio_set_value_cansleep(bat->charge_enable, 1);
		return 0;
		
	case 1: /* charger low (100mA) */
		gpio_set_value_cansleep(bat->charge_enable, 1);
		gpio_set_value_cansleep(bat->charge_low, 0);
		gpio_set_value_cansleep(bat->charge_high, 0);
		gpio_set_value_cansleep(bat->charge_enable, 0);
		return 1;
		
	case 2: /* charge normal (500mA) */
		gpio_set_value_cansleep(bat->charge_enable, 1);
		gpio_set_value_cansleep(bat->charge_low, 1);
		gpio_set_value_cansleep(bat->charge_high, 0);
		gpio_set_value_cansleep(bat->charge_enable, 0);
		return 1;
		
	case 3: /* charger high (1500mA) */
		gpio_set_value_cansleep(bat->charge_enable, 1);
		gpio_set_value_cansleep(bat->charge_low, 1);
		gpio_set_value_cansleep(bat->charge_high, 1);
		gpio_set_value_cansleep(bat->charge_enable, 0);
		return 1;
	}
	return -1;
}

static ssize_t bat_charge_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct archos_battery *bat = dev_get_drvdata(dev);
	
	int level = simple_strtol(buf, NULL, 10);
	int charge_state;
	
	switch (bat->charger_type) {
		case CHARGER_LX2208:
			charge_state = bat_charge_set_LX2208( level );
			break;
		case CHARGER_RT9502:
			charge_state = bat_charge_set_RT9502( level );
			break;
		case CHARGER_ISL9220:
			charge_state = bat_charge_set_ISL9220( level );
			break;
		default:
			return len;
	}
	if ( charge_state == 0 || charge_state == 1 )
		bat->charge_state = charge_state;
	else
		printk( KERN_DEBUG "archos_battery: could not set charge level %d\n", level);
	
	bat->charge_level = level;
	power_supply_changed(&bat->main_battery);

	return len;
}
static DEVICE_ATTR(charge_level, S_IRUGO|S_IWUSR, bat_charge_show, bat_charge_store);

static void dcin_read_dc_detect_delayed( struct work_struct *work)
{
	struct archos_battery *bat = container_of(work, struct archos_battery, work.work);
	printk(KERN_DEBUG "dcin_read_dc_detect_delayed\n");
	
	bat->old_dcin = read_gpio_dc_detect(bat);

	/* special handling of RT9502 charger - we need to enable USB charging here to make,
	  if it was off before. RT9502 does not charge with charge_enable=1.
	  On early A35DE boards charge_enable=1 even prevented detection... - not handled here. */
	if ( bat->charger_type == CHARGER_RT9502 && !bat->charge_level) {
		if ( bat->old_dcin ) {
			/* we can charge, but charge was disabled */
			gpio_set_value_cansleep(bat->charge_enable, 0);
		} else {
			/* AC charger disconnect, disable charging again */
			gpio_set_value_cansleep(bat->charge_enable, 1);
		}
	}
	power_supply_changed(&bat->main_battery);	// send uevent
}

static irqreturn_t dcin_irq_handler(int irqno, void *data)
{
	struct platform_device *pdev = data;

	dev_dbg(&pdev->dev, "dcin_irq_handler\n");

	if (delayed_work_pending(&bat->work))
		cancel_delayed_work_sync(&bat->work);

	wake_lock_timeout(&bat->wl, msecs_to_jiffies(500));
	schedule_delayed_work(&bat->work, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}

static int __init archos_battery_probe(struct platform_device *pdev)
{
	int ret;
	const struct archos_charge_config *chg_cfg;
	const struct archos_charge_conf *chg_conf;

	bat = kzalloc(sizeof(struct archos_battery), GFP_KERNEL);
	if (bat == NULL)
		return -ENOMEM;
	
	bat->main_battery = main_battery_template;
	bat->ac_supply = ac_supply_template;
	bat->usb_supply = usb_supply_template;
	bat->charge_enable = UNUSED_GPIO;
	bat->charge_low = UNUSED_GPIO;
	bat->charge_high = UNUSED_GPIO;

	/* FIXME: default values for now */
	bat->current_mV = 3850;
	bat->current_capacity = 50;
	bat->charger_type = -1;

	/* charger configuration */
	chg_cfg = omap_get_config( ARCHOS_TAG_CHARGE, struct archos_charge_config);
	chg_conf = hwrev_ptr(chg_cfg, system_rev);
	if (!IS_ERR(chg_conf)) {
		bat->charger_type = chg_conf->charger_type;
		if (gpio_is_valid(chg_conf->charge_enable) &&
				gpio_is_valid(chg_conf->charge_low)) {

			bat->charge_enable = chg_conf->charge_enable;
			bat->charge_low = chg_conf->charge_low;

			archos_gpio_init_output(bat->charge_enable, "charge enable");
			archos_gpio_init_output(bat->charge_low, "charge low");

			if (gpio_is_valid(chg_conf->charge_high)) {
				bat->charge_high = chg_conf->charge_high;
				archos_gpio_init_output(bat->charge_high, "charge high");
			}

			ret = device_create_file(&pdev->dev, &dev_attr_charge_level);
			if (ret < 0)
				dev_dbg(&pdev->dev, "cannot create charge_level attribute\n");
		}

		if (chg_conf->gpio_dc_detect) {
			int ret;

			bat->gpio_dc_detect = chg_conf->gpio_dc_detect;
			gpio_request(bat->gpio_dc_detect, "dc detect");
			gpio_direction_input(bat->gpio_dc_detect);

			bat->dc_detect_invert = chg_conf->dc_detect_invert;

			bat->old_dcin = read_gpio_dc_detect(bat);

			ret = request_irq(gpio_to_irq(bat->gpio_dc_detect), dcin_irq_handler, 
					IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "dc detect", pdev);
			if (ret < 0)
				dev_warn(&pdev->dev, "failed to request irq %i for dc detect\n", 
						gpio_to_irq(bat->gpio_dc_detect));

			wake_lock_init(&bat->wl, WAKE_LOCK_SUSPEND, "dc_detect");
			INIT_DELAYED_WORK(&bat->work, dcin_read_dc_detect_delayed);
		}
	}

	platform_set_drvdata(pdev, bat);
	
	// Do not register when twl6030 is in use to avoid conflicts in android hc
	if (!((bat->charger_type == CHARGER_TWL6030USB) || 
		 (bat->charger_type == CHARGER_TWL6030USB_DC))) {
		ret = power_supply_register(&pdev->dev, &bat->main_battery);
		if (ret < 0) {
			printk(KERN_DEBUG "archos_battery_probe: "
				"cannot register main battery: %i\n", ret);
		}
		ret = power_supply_register(&pdev->dev, &bat->ac_supply);
		if (ret < 0) {
			printk(KERN_DEBUG "archos_battery_probe: "
				"cannot register ac supply: %i\n", ret);
		}
		ret = power_supply_register(&pdev->dev, &bat->usb_supply);
		if (ret < 0) {
			printk(KERN_DEBUG "archos_battery_probe: "
				"cannot register usb supply: %i\n", ret);
		}
	}

	ret = device_create_file(&pdev->dev, &dev_attr_voltage_avg);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot create voltage_avg attribute\n");
	ret = device_create_file(&pdev->dev, &dev_attr_capacity);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot create capacity attribute\n");
	ret = device_create_file(&pdev->dev, &dev_attr_ac_online);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot create ac_online attribute\n");
	ret = device_create_file(&pdev->dev, &dev_attr_usb_online);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot create usb_online attribute\n");
	ret = device_create_file(&pdev->dev, &dev_attr_charger_type);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot create charger type attribute\n");

	return 0;
}

static struct platform_driver archos_battery_driver = {
	.probe = archos_battery_probe,
	.driver = {
		.name = "battery",
	},
};

static struct platform_device archos_battery_device = {
	.name = "battery",
	.id = -1,
};

static int __init archos_battery_driver_init(void) 
{
	int ret = 0;

#ifdef CONFIG_POWER_SUPPLY
	ret = platform_device_register(&archos_battery_device);
	if (ret < 0) {
		printk(KERN_INFO "omap_battery_init_devicefs: failed to register battery device\n");
		return ret;
	}
	ret = platform_driver_register(&archos_battery_driver);
	if (ret < 0) {
		printk(KERN_INFO "omap_battery_init_devicefs: failed to register battery driver\n");
		return ret;
	}

#endif
	return ret;
}

device_initcall(archos_battery_driver_init);
#endif

int __init archos_battery_twl4030_bci_init(struct twl4030_bci_platform_data *pdata)
{
	const struct archos_charge_config *chg_cfg;
	const struct archos_charge_conf *chg_conf;

	chg_cfg = omap_get_config( ARCHOS_TAG_CHARGE, struct archos_charge_config);
	chg_conf = hwrev_ptr(chg_cfg, system_rev);
	if (IS_ERR(chg_conf))
		return -ENODEV;
	pdata->usb_is_dc = chg_conf->charger_type == CHARGER_TWL6030USB_DC;
	return 0;
}
