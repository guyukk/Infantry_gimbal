/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      远程控制器
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "bsp_usart.h"
#include "struct_typedef.h"

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

/* ----------------------- RC RX validation -------------------------------- */
/* 遥控器原始通道量程 364..1684，中值 1024；沿用既有 ±700 失效门限作为
 * 接收校验边界：raw 324..1724 接受，之外判为坏帧。 */
#define RC_CH_VALUE_MIN_RAW     ((uint16_t)324)
#define RC_CH_VALUE_MAX_RAW     ((uint16_t)1724)
#define RC_CH_VALUE_VALID(s)    (((s) >= RC_CH_VALUE_MIN_RAW) && ((s) <= RC_CH_VALUE_MAX_RAW))
#define RC_SWITCH_VALID(s)      ((s) >= RC_SW_UP && (s) <= RC_SW_MID)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
#pragma pack(push, 1)
typedef struct
{
        struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        struct
        {
                uint16_t v;
        } key;
	uint32_t last_fdb;
	/* 发布序号：只有合法完整帧才递增，ISR 单写者，任务侧可做序号快照 */
	volatile uint32_t generation;

} RC_ctrl_t;
#pragma pack(pop)

/* 遥控器接收统计：坏长度/坏通道/坏开关/丢弃块 */
typedef struct
{
    uint32_t chunks;          /* 回调块数 */
    uint32_t frames_ok;       /* 通过校验并发布的帧数 */
    uint32_t bad_channel;     /* 通道越界帧 */
    uint32_t bad_switch;      /* 开关枚举非法帧 */
    uint32_t rejected_chunks; /* 长度非 18/36 的整块丢弃 */
    uint32_t discarded_bytes; /* 被丢弃的字节数 */
    uint32_t overflow;        /* DMA 事件长度异常 */
    uint32_t restarts;        /* 成功重启接收次数 */
} rc_rx_stats_t;

/* ----------------------- Internal Data ----------------------------------- */

extern fp32 GIMBAL_INS_yaw;
extern RC_ctrl_t rc_ctrl;
extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
extern void sbus_to_rc(uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl, remoter_t *remoter);

/* UART5 原始块 -> 校验 -> 发布（只在 ISR 调用） */
extern void remote_control_rx_chunk(const uint8_t *data, uint16_t len);
extern void remote_control_rx_restart(UART_HandleTypeDef *huart);
extern void remote_control_rx_overflow(void);
/* 读取接收统计（任务上下文，短临界区） */
extern void rc_rx_get_stats(rc_rx_stats_t *out);
/* 一致性读取遥控器快照（任务上下文） */
extern uint8_t rc_ctrl_snapshot(RC_ctrl_t *out, uint32_t *generation);


#endif
