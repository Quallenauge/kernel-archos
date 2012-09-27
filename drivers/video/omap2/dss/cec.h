
#ifndef _HDMI_CEC_H_
#define _HDMI_CEC_H_




#ifdef __KERNEL__

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/fs.h>




/***************************/
/* HW specific definitions */
/***************************/

/* HDMI WP base address */
#define HDMI_WP			0x58006000

#define HDMI_WP_WP_CLK		(0x70)
#define HDMI_WP_IRQENABLE_SET	(0x2C)
#define HDMI_WP_IRQSTATUS   (0x28)

/* HDMI CORE SYSTEM base address */
#define HDMI_IP_CORE_SYSTEM    (0x400)
#define HDMI_CORE_SYS_UMASK4   (0x1E0)
#define HDMI_CORE_SYS_INTR4    (0x1D0)
#define HDMI_IP_CORE_CEC       (0xD00)


/**************************************
* HDMI CEC registers *
***************************************/
#define HDMI_DEV_ID                             (0x0)
#define HDMI_CEC_SPEC                           (0x4)
#define HDMI_CEC_TX_INIT                        (0x20)
#define HDMI_CEC_TX_DEST                        (0x24)
#define HDMI_CEC_TRANSMIT_DATA                  (0x7c)
#define HDMI_CEC_SETUP                          (0x38)
#define HDMI_CEC_TX_COMMAND                     (0x3c)
#define HDMI_CEC_DBG_3                          (0x1c)
#define HDMI_CEC_INT_STATUS_1                   (0x9c)
#define HDMI_CEC_INT_STATUS_0                   (0x98)
#define HDMI_CEC_INT_ENABLE_0                   (0x90)
#define HDMI_CEC_CA_7_0                         (0x88)
#define HDMI_CEC_CA_15_8                        (0x8c)
#define HDMI_CEC_TX_OPERAND                     (0x40)
#define HDMI_CEC_RX_CMD_HEADER                  (0xb8)
#define HDMI_CEC_RX_COUNT                       (0xB4)
#define HDMI_CEC_RX_CONTROL                     (0xB0)
#define HDMI_CEC_RX_COMMAND                     (0xbc)
#define HDMI_CEC_RX_OPERAND                     (0xc0)

#define WR_REG_32(base, offset, val)	__raw_writel(val, base + offset)
#define RD_REG_32(base, offset)		__raw_readl(base + offset)


#undef FLD_MASK
#define FLD_MASK(start, end)	(((1 << (start - end + 1)) - 1) << (end))
#undef FLD_VAL
#define FLD_VAL(val, start, end) (((val) << end) & FLD_MASK(start, end))
#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))

#define WR_FIELD_32(base, offset, start, end, val) \
	WR_REG_32(base, offset, FLD_MOD(RD_REG_32(base, offset), val, \
		  start, end))

#define RD_FIELD_32(base, offset, start, end) \
	((RD_REG_32(base, offset) & FLD_MASK(start, end)) >> (end))
#endif /* __KERNEL__ */

typedef enum {
    CEC_TV = 0,
    CEC_RECORDING = 1,
    CEC_TUNER = 2,
    CEC_PLAYBACK = 3,
    CEC_AUDIO = 4,
    CEC_FREE = 5,
    CEC_UNREGISTERED = 6,
    CEC_DEVICE_NB,
} cec_device_type;

typedef enum cec_vendor_id
{
  CEC_VENDOR_SAMSUNG = 0x0000F0,
  CEC_VENDOR_LG = 0x00E091,
  CEC_VENDOR_PANASONIC = 0x008045,
  CEC_VENDOR_PIONEER = 0x00E036,
  CEC_VENDOR_ONKYO = 0x0009B0,
  CEC_VENDOR_YAMAHA = 0x00A0DE,
  CEC_VENDOR_PHILIPS = 0x00903E,
  CEC_VENDOR_SONY = 0x080046,
  CEC_VENDOR_TOSHIBA = 0x000039,
  CEC_VENDOR_UNKNOWN = 0
} cec_vendor_id;

struct cec_device_id {
  int num_id;
  int ids[4];
};

const struct cec_device_id cec_id_list[CEC_DEVICE_NB] = {
  { //TV
    .num_id = 1,
    .ids = {0},
  },
  { //RECORDING
    .num_id = 3,
    .ids = {1,2,9},
  },
  { //TUNER
    .num_id = 4,
    .ids = {3,6,7,10},
  },
  { //PLAYBACK
    .num_id = 3,
    .ids = {4,8,11},
  },
  { //AUDIO
    .num_id = 1,
    .ids = {5},
  },
  { //FREE
    .num_id = 1,
    .ids = {14},
  },
  { //UNREGISTERED
    .num_id = 1,
    .ids = {15},
  },
};

struct cec_tx_data {
	char   dest_device_id;
	char   initiator_device_id;
	char   send_ping;
	char   retry_count;
	char   tx_cmd;
	char   tx_count;
	char   tx_operand[15];
};
struct cec_rx_data {
	char   init_device_id;
	char   dest_device_id;
	char   rx_cmd;
	char   rx_count;
	char   rx_operand[15];
};

enum cec_command {
	cec_cmd_u_feature_abort = 0x00,
	cec_cmd_u_image_view_on = 0x04,
	cec_cmd_u_record_on = 0x09,
	cec_cmd_u_record_status = 0x0A,
	cec_cmd_u_record_off = 0x0B,
	cec_cmd_u_text_view_on = 0x0D,
	cec_cmd_u_record_tv_screen = 0x0F,
	cec_cmd_b_set_menu_language = 0x32,
	cec_cmd_u_clear_analog_timer = 0x33,
	cec_cmd_u_set_analog_timer = 0x34,
	cec_cmd_u_timer_status = 0x35,
	cec_cmd_b_u_standby = 0x36,
	cec_cmd_u_play = 0x41,
	cec_cmd_u_deck_control = 0x42,
	cec_cmd_u_timer_cleared_status = 0x43,
	cec_cmd_u_user_control_pressed = 0x44,
	cec_cmd_u_user_control_released = 0x45,
	cec_cmd_u_give_osd_name = 0x46,
	cec_cmd_u_set_osd_name = 0x47,
	cec_cmd_b_active_source = 0x82,
	cec_cmd_u_give_physical_address = 0x83,
	cec_cmd_b_report_physical_address = 0x84,
	cec_cmd_b_request_active_source = 0x85,
	cec_cmd_u_give_device_power_status = 0x8F,
	cec_cmd_u_report_power_status = 0x90,
	cec_cmd_u_cec_version = 0x9E,
	cec_cmd_u_get_cec_version = 0x9F,
	cec_cmd_u_abort = 0xFF,
	cec_cmd_b_routing_change = 0x80,
	cec_cmd_b_routing_info = 0x81,
	cec_cmd_b_set_stream_path = 0x86,
	cec_cmd_u_inactive_source = 0x9D,
	cec_cmd_u_tuner_step_increment = 0x05,
	cec_cmd_u_tuner_step_decrement = 0x06,
	cec_cmd_u_tuner_device_status = 0x07,
	cec_cmd_u_give_tuner_device_status = 0x08,
	cec_cmd_u_give_deck_status = 0x1A,
	cec_cmd_u_deck_status = 0x1B,
	cec_cmd_u_clear_external_timer = 0xA1,
	cec_cmd_u_set_external_timer = 0xA2,
	cec_cmd_u_set_osd_string = 0x64,
	cec_cmd_u_set_timer_program_title = 0x67,
	cec_cmd_u_system_audio_mode_request = 0x70,
	cec_cmd_u_give_audio_status = 0x71,
	cec_cmd_b_u_set_system_audio_mode = 0x72,
	cec_cmd_u_report_audio_status = 0x7A,
	cec_cmd_u_give_system_audio_mode_status = 0x7D,
	cec_cmd_u_system_audio_mode_status = 0x7E,
	cec_cmd_b_device_vendor_id = 0x87,
	cec_cmd_u_vendor_command = 0x89,
	cec_cmd_u_B_vendor_remote_button_down = 0x8A,
	cec_cmd_u_B_vendor_remote_button_up = 0x8B,
	cec_cmd_u_give_device_vendor_id = 0x8C,
	cec_cmd_u_menu_request = 0x8D,
	cec_cmd_u_menu_status = 0x8E,
	cec_cmd_u_get_menu_language = 0x91,
	cec_cmd_u_select_analogue_service = 0x92,
	cec_cmd_u_select_digital_service = 0x93,
	cec_cmd_u_set_digital_timer = 0x97,
	cec_cmd_u_clear_digital_timer = 0x99,
	cec_cmd_u_set_audio_rate = 0x9A,
	cec_cmd_u_vendor_command_with_id = 0xA0
};
enum cec_ui_command {
	cec_ui_command_select = 0,
	cec_ui_command_up =	0x01,
	cec_ui_command_down = 0x02,
	cec_ui_command_left = 0x03,
	cec_ui_command_right = 0x04,
	cec_ui_command_right_up = 0x05,
	cec_ui_command_right_down = 0x06,
	cec_ui_command_left_up = 0x07,
	cec_ui_command_left_down = 0x08,
	cec_ui_command_root_menu = 0x09,
	cec_ui_command_setup_menu = 0x0a,
	cec_ui_command_contents_menu = 0x0b,
	cec_ui_command_fav_menu = 0x0c,
	cec_ui_command_exit = 0x0d,
	cec_ui_command_reserved1 = 0x0e,
	cec_ui_command_reserved2 = 0x0f,
	cec_ui_command_media_top_menu = 0x10,
	cec_ui_command_media_context_sensitive_menu = 0x11,
	cec_ui_command_reserved3 = 0x12,
	cec_ui_command_reserved4 = 0x13,
	cec_ui_command_num_entry_mode = 0x1d,
	cec_ui_command_num_11 = 0x1e,
	cec_ui_command_num_12 = 0x1f,
	cec_ui_command_num_0_10 = 0x20,
	cec_ui_command_num_1 = 0x21,
	cec_ui_command_num_2 = 0x22,
	cec_ui_command_num_3 = 0x23,
	cec_ui_command_num_4 = 0x24,
	cec_ui_command_num_5 = 0x25,
	cec_ui_command_num_6 = 0x26,
	cec_ui_command_num_7 = 0x27,
	cec_ui_command_num_8 = 0x28,
	cec_ui_command_num_9 = 0x29,
	cec_ui_command_dot = 0x2a,
	cec_ui_command_enter = 0x2b,
	cec_ui_command_clear = 0x2c,
	cec_ui_command_next_fav = 0x2f,
	cec_ui_command_channel_up = 0x30,
	cec_ui_command_channel_down = 0x31,
	cec_ui_command_previous_channel = 0x32,
	cec_ui_command_sound_select = 0x33,
	cec_ui_command_input_select = 0x34,
	cec_ui_command_display_information = 0x35,
	cec_ui_command_help = 0x36,
	cec_ui_command_page_up = 0x37,
	cec_ui_command_page_down = 0x38,
	cec_ui_command_power = 0x40,
	cec_ui_command_volume_up = 0x41,
	cec_ui_command_volume_down = 0x42,
	cec_ui_command_mute = 0x43,
	cec_ui_command_play = 0x44,
	cec_ui_command_stop = 0x45,
	cec_ui_command_pause = 0x46,
	cec_ui_command_record = 0x47,
	cec_ui_command_rewind = 0x48,
	cec_ui_command_fast_forward = 0x49,
	cec_ui_command_eject = 0x4a,
	cec_ui_command_forward = 0x4b,
	cec_ui_command_backward = 0x4c,
	cec_ui_command_stop_record = 0x4d,
	cec_ui_command_pause_record = 0x4e,
	cec_ui_command_angle = 0x50,
	cec_ui_command_sub_picture = 0x51,
	cec_ui_command_video_on_demand = 0x52,
	cec_ui_command_epg = 0x53,
	cec_ui_command_timer_programming = 0x54,
	cec_ui_command_initial_config = 0x55,
	cec_ui_command_play_function = 0x60,
	cec_ui_command_pause_play_function = 0x61,
	cec_ui_command_record_function = 0x62,
	cec_ui_command_pause_record_function = 0x63,
	cec_ui_command_stop_function = 0x64,
	cec_ui_command_mute_function = 0x65,
	cec_ui_command_restore_vol_function = 0x66,
	cec_ui_command_tune_function = 0x67,
	cec_ui_command_select_media_function = 0x68,
	cec_ui_command_select_av_input_function = 0x69,
	cec_ui_command_select_audio_input_function = 0x6a,
	cec_ui_command_power_toggle_function = 0x6b,
	cec_ui_command_power_off_function = 0x6c,
	cec_ui_command_power_on_function = 0x6d,
	cec_ui_command_f1_blue = 0x71,
	cec_ui_command_f2_red = 0x72,
	cec_ui_command_f3_green = 0x73,
	cec_ui_command_f4_yellow = 0x74,
	cec_ui_command_f5 = 0x75,
	cec_ui_command_data = 0x76
};

//All mapped button in CEC.
//Missing Shortcuts to Video/Audio
//Missing play/pause/seek
//Align with .kl layout file in android framework
typedef enum {
	BUTTON_OK	=	0x123,
	DPAD_LEFT	=	0x124,
	DPAD_RIGHT	=	0x125,
	DPAD_UP		=	0x126,
	DPAD_DOWN	=	0x127,
	BUTTON_EXIT	=	0x128,
	BUTTON_0	=	0x129,
	BUTTON_1	=	0x12A,
	BUTTON_2	=	0x12B,
	BUTTON_3	=	0x12C,
	BUTTON_4	=	0x12D,
	BUTTON_5	=	0x12E,
	BUTTON_6	=	0x12F,
	BUTTON_7	=	0x130,
	BUTTON_8	=	0x131,
	BUTTON_9	=	0x132,
	RED_BUTTON	=	0x133,
	GREEN_BUTTON	=	0x134,
	YELLOW_BUTTON	=	0x135,
	BLUE_BUTTON	=	0x136,
	PLAY_BUTTON	=	0x137,
	STOP_BUTTON	=	0x138,
	WIND_BUTTON	=	0x139,
	REWIND_BUTTON	=	0x13A,
} cec_buttons;

struct cec_dev {
	int device_id;
	int clear_existing_device;
	int phy_addr;
	cec_device_type type;
	cec_buttons last_ui_button;
};

#define CEC_IOCTL_MAGIC 'c'
#define CEC_REGISTER_DEVICE	  _IOWR(CEC_IOCTL_MAGIC, 0, \
				struct cec_dev)
#define CEC_TRANSMIT_CMD	  _IOWR(CEC_IOCTL_MAGIC, 1, \
				struct cec_tx_data)
#define CEC_RECV_CMD  _IOWR(CEC_IOCTL_MAGIC, 2, \
				struct cec_rx_data)
#define CEC_GET_PHY_ADDR _IOR(CEC_IOCTL_MAGIC, 3, int)

#endif /* _HDCP_H_ */
