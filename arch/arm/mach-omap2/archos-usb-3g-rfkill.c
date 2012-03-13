#include <linux/init.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>

#include "mux.h"

#define DEVICE_NAME	"archos_usb_3g"

#define USB_OFF_WHEN_SUSPENDED

static struct archos_3g_rfkill_device {
	struct platform_device *pdev;
	struct rfkill *rfkill;
	struct regulator *regulator;
	struct regulator *hub;
	bool state;
} archos_3g_rfkill;

static inline void archos_usb_3g_rfkill_enable(struct archos_3g_rfkill_device *rfkill_dev, bool en)
{
	if (rfkill_dev->state == en)
		return;

	if (en) {
		regulator_enable(rfkill_dev->regulator);
		if (rfkill_dev->hub)
			regulator_enable(rfkill_dev->hub);
	} else {
		regulator_disable(rfkill_dev->regulator);
		if (rfkill_dev->hub)
			regulator_disable(rfkill_dev->hub);
	}
	rfkill_dev->state = en;
}

static int archos_usb_3g_rfkill_set_block(void *data, bool blocked)
{
	struct archos_3g_rfkill_device *rfkill_dev = data;
	bool enabled = !blocked;
	int ret = 0;

	dev_dbg(&rfkill_dev->pdev->dev, "%s, enabled: %d\n", __func__, enabled);

	archos_usb_3g_rfkill_enable(rfkill_dev, enabled);

	return ret;
}

static void archos_usb_3g_rfkill_query(struct rfkill *rfkill, void *data)
{
	struct archos_3g_rfkill_device *rfkill_dev = data;

	dev_dbg(&rfkill_dev->pdev->dev, "%s, enabled: %d\n", __func__, rfkill_dev->state);

	rfkill_set_sw_state(rfkill, !rfkill_dev->state);
}

static const struct rfkill_ops archos_usb_3g_rfkill_ops = {
	.set_block = archos_usb_3g_rfkill_set_block,
	.query = archos_usb_3g_rfkill_query,
};

#ifdef CONFIG_PM
static int archos_usb_3g_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct archos_3g_rfkill_device *rfkill_dev = &archos_3g_rfkill;

#ifdef USB_OFF_WHEN_SUSPENDED
	if (rfkill_dev->state && rfkill_dev->regulator != NULL) {
		regulator_disable(rfkill_dev->regulator);
		if (rfkill_dev->hub)
			regulator_disable(rfkill_dev->hub);
	}
#endif

	return 0;
}

/* implemented in usb-ehci.c */
extern int is_ehci_port0_connected(void);
static int archos_usb_3g_resume(struct platform_device *pdev)
{
	struct archos_3g_rfkill_device *rfkill_dev = &archos_3g_rfkill;
//	bool dongle_connected = is_ehci_port0_connected();

#ifdef USB_OFF_WHEN_SUSPENDED
	if (rfkill_dev->state && rfkill_dev->regulator != NULL) {
		regulator_enable(rfkill_dev->regulator);
		if (rfkill_dev->hub)
			regulator_enable(rfkill_dev->hub);
	}
#endif

	return 0;
}
#endif

static struct platform_driver archos_usb_3g_driver = {
#ifdef CONFIG_PM
	.suspend	= archos_usb_3g_suspend,
	.resume		= archos_usb_3g_resume,
#endif
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
	}
};

int __init archos_usb_3g_rfkill_init(void)
{
	struct archos_3g_rfkill_device *rfkill_dev = &archos_3g_rfkill;
	struct platform_device *pdev;
	int ret;
	
	ret = platform_driver_register(&archos_usb_3g_driver);
	if (ret)
		return ret;		
	pdev = platform_device_alloc(DEVICE_NAME, -1);
	if (!pdev) {
		ret = -ENOMEM;
		goto initfail1;
	}
	ret = platform_device_add(pdev);
	if (ret)
		goto initfail2;

	rfkill_dev->pdev = pdev;

	rfkill_dev->rfkill = rfkill_alloc("archos_usb_wwan",
					&pdev->dev,
					RFKILL_TYPE_WWAN,
					&archos_usb_3g_rfkill_ops, (void *)rfkill_dev);

	if (!rfkill_dev->rfkill) {
		dev_err(&pdev->dev, "%s - Out of memory\n", __func__);
		goto initfail3;
	}

	rfkill_dev->regulator = regulator_get(&pdev->dev, "3g_5v");
	if (IS_ERR(rfkill_dev->regulator)) {
		dev_err(&pdev->dev, "Unable to get \"3g_5v\" regulator\n");
		rfkill_dev->regulator = NULL;
		goto initfail4;
	}
	
	rfkill_dev->hub = regulator_get(&pdev->dev, "3g_hub");
	if (IS_ERR(rfkill_dev->hub)) {
		dev_err(&pdev->dev, "Unable to get \"3g_hub\" regulator\n");
		rfkill_dev->hub = NULL;
	}

	if (rfkill_register(rfkill_dev->rfkill) < 0) {
		dev_err(&pdev->dev, "Failed to register rfkill\n");
		goto initfail6;
	}

	archos_usb_3g_rfkill_enable(rfkill_dev, true);
	
	return 0;

initfail6:
	if (rfkill_dev->hub)
		regulator_put(rfkill_dev->hub);
initfail5:
	regulator_put(rfkill_dev->regulator);
initfail4:
	rfkill_destroy(rfkill_dev->rfkill);
initfail3:
	platform_device_del(pdev);
initfail2:
	platform_device_put(pdev);
initfail1:
	platform_driver_unregister(&archos_usb_3g_driver);
	return ret;
}

device_initcall(archos_usb_3g_rfkill_init);
