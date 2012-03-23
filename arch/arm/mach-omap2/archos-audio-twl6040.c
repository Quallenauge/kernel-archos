#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/i2c/twl.h>

#include <mach/gpio.h>
#include <plat/archos-audio-twl6040.h>
#include <plat/archos-gpio.h>
#include <asm/mach-types.h>
#include <plat/board.h>
#include <mach/board-archos.h>
#include <linux/delay.h>

#include "clock.h"
#include "mux.h"

static struct archos_audio_twl6040_conf audio_gpio;

static struct platform_device audio_soc_device = {
	.name		= "archos-twl6040-asoc",
	.id		= -1,
	.dev            = {
		.platform_data = NULL,
	},
};

static void _auxclk_en(int en)
{
	struct clk *auxclk0_ck;

	auxclk0_ck = clk_get(NULL, "auxclk0_ck");
	if (!IS_ERR(auxclk0_ck)) {
		if (en) {
			if (clk_enable(auxclk0_ck) != 0) {
				printk(KERN_ERR "failed to enable %s\n", "auxclk0_ck");
			}
		} else {
			clk_disable(auxclk0_ck);
		}

		clk_put(auxclk0_ck);
	}
}

static int _get_auxclk_rate(void)
{
	struct clk *auxclk0_ck;
	int rate;

	auxclk0_ck = clk_get(NULL, "auxclk0_ck");
	if (IS_ERR(auxclk0_ck)) {
		printk(KERN_ERR "failed to get %s\n", "auxclk0_ck");
	}
	rate = clk_get_rate(auxclk0_ck);
	clk_put(auxclk0_ck);

	return rate;
}

static struct audio_twl6040_device_config audio_twl6040_device_io = {
	.set_speaker_state = NULL,
	.set_codec_master_clk_state = &_auxclk_en,
	.get_master_clock_rate = &_get_auxclk_rate,
	.suspend = NULL,
	.resume = NULL,
};

struct audio_twl6040_device_config *archos_audio_twl6040_get_io(void) {
		return &audio_twl6040_device_io;
}

int __init archos_audio_twl6040_init(struct twl4030_codec_data * codec_data)
{
	const struct archos_audio_twl6040_config *audio_cfg;
	int ret;

	pr_debug("Archos audio init (TWL6040)\n");

	/* audio  */
	audio_cfg = omap_get_config( ARCHOS_TAG_AUDIO_TWL6040, struct archos_audio_twl6040_config );
	if (audio_cfg == NULL) {
		printk(KERN_DEBUG "%s: no board configuration found\n", __FUNCTION__);
		return -ENODEV;
	}
	if ( system_rev >= audio_cfg->nrev ) {
		printk(KERN_DEBUG "%s: system_rev (%i) >= nrev (%i)\n",
			__FUNCTION__, system_rev, audio_cfg->nrev);
		return -ENODEV;
	}

	audio_gpio = audio_cfg->rev[system_rev];

	/* Power ON GPIO mux */
	omap_mux_init_gpio(audio_gpio.power_on, PIN_OUTPUT);
	codec_data->audpwron_gpio = audio_gpio.power_on;

	/* Master clock mux */
	omap_mux_init_signal("fref_clk0_out.fref_clk0_out", OMAP_PIN_OUTPUT);

	/* Audio IRQ mux */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", OMAP_PIN_INPUT_PULLUP);

	/* McPDM mux */
	omap_mux_init_signal("abe_clks.abe_clks", OMAP_PIN_INPUT);
	omap_mux_init_signal("abe_pdm_ul_data.abe_pdm_ul_data", OMAP_PIN_INPUT);
	omap_mux_init_signal("abe_pdm_dl_data.abe_pdm_dl_data", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("abe_pdm_frame.abe_pdm_frame", OMAP_PIN_INPUT);
	omap_mux_init_signal("abe_pdm_lb_clk.abe_pdm_lb_clk", OMAP_PIN_INPUT);

	ret = platform_device_register(&audio_soc_device);
	if (ret) {
		printk(KERN_ERR "Unable to register audio soc platform device\n");
		return -ENODEV;
	}

	return 0;
}

EXPORT_SYMBOL(audio_twl6040_device_io);
