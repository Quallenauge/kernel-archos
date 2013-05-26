#ifndef _ARCH_ARCHOS_AUDIO_TWL6040_H_
#define _ARCH_ARCHOS_AUDIO_TWL6040_H_

#include <linux/mfd/twl6040.h>

struct audio_twl6040_device_config {
	void (*set_codec_master_clk_state)(int state);
	int (*get_master_clock_rate)(void);
	void (*set_speaker_state)(int state);
	void (*suspend)(void);
	void (*resume)(void);
};

struct audio_twl6040_device_config * archos_audio_twl6040_get_io(void);

extern int __init archos_audio_twl6040_init(struct twl6040_platform_data *twl6040_data);

#endif
