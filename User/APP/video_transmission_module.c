#include "video_transmission_module.h"

/* Neutral VTM state remains part of the 46-byte board frame. */
VTM_Data_t vtm_data;

void vtm_data_init(void)
{
    // 初始化结构体为默认值，保留原初始化逻辑
    vtm_data.head1 = REMOTE_HEAD1;
    vtm_data.head2 = REMOTE_HEAD2;
    vtm_data.ch0 = CHANNEL_MID;
    vtm_data.ch1 = CHANNEL_MID;
    vtm_data.ch2 = CHANNEL_MID;
    vtm_data.ch3 = CHANNEL_MID;
    vtm_data.gear_switch = GEAR_N;
    vtm_data.pause_key = KEY_RELEASE;
    vtm_data.custom_l_key = KEY_RELEASE;
    vtm_data.custom_r_key = KEY_RELEASE;
    vtm_data.wheel = CHANNEL_MID;
    vtm_data.trigger_key = KEY_RELEASE;
    vtm_data.mouse_x = 0;
    vtm_data.mouse_y = 0;
    vtm_data.mouse_z = 0;
    vtm_data.mouse_l_key = KEY_RELEASE;
    vtm_data.mouse_r_key = KEY_RELEASE;
    vtm_data.mouse_m_key = KEY_RELEASE;
    vtm_data.keyboard_key = 0;
    vtm_data.crc = 0;
}
