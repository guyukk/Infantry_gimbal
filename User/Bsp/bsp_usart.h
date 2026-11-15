/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-07-09 21:22:21
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-07-12 17:41:29
 * @FilePath: \smartcare:\X15\stm32\my_example\RM\H7\DM-balanceV1\User\Bsp\bsp_usart.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef BSP_USART_H
#define BSP_USART_H

#include "main.h"
#include "struct_typedef.h"
#include "protocol.h"
#include "robot_param.h"

#define BUFF_SIZE	25
#define CHASSIS_DATA_LENGTH 28
#define USART_RX_BUF_LENGHT 512

/* -------------------------------------------------------------------------
 * 板间 28 字节帧接收参数（P01）
 * 帧格式不变：0='s'，1..26 为 6 个 float + 2 个 uint8，27='e'。
 * 抖动只影响接收实现，不改线上字节序与字段位置。
 * ---------------------------------------------------------------------- */
#define CHASSIS_FRAME_HEAD      's'
#define CHASSIS_FRAME_TAIL      'e'
#define USART1_RX_DMA_SIZE      64u     /* DMA 环形缓冲，>28 以容纳粘包/余量 */
#define USART1_TX_FRAME_MAX     64u     /* 46 字节发送帧的内部副本上限 */
#define CHASSIS_ANGLE_LIMIT_RAD 3.2f    /* 底盘姿态角合理上限，rad */
#define CHASSIS_SPEED_LIMIT_MPS 10.0f   /* vx/vy 合理上限，m/s */
#define CHASSIS_WZ_LIMIT_RADPS  50.0f   /* wz 合理上限，rad/s */
#define CHASSIS_MODE_MAX        2u      /* NO_MOVE/FOLLOW/SPIN */

// 定义一个联合类型，用于将浮点数转换为字节
typedef union {
    float fp32;
    uint8_t bytes[4];
} fp32_to_bytes;

// 定义一个联合类型，用于将16位整数转换为字节
typedef union {
    int16_t int16;
    uint8_t bytes[2];
} int_to_bytes;

//云台收到的底盘数据,由串口发送到云台
typedef struct chassis_data
{
    //底盘速度信息
    fp32 chassis_vx;
    fp32 chassis_vy;
    fp32 chassis_wz;
    
    //底盘陀螺仪信息
    fp32 chassis_yaw;
    fp32 chassis_pitch;
    fp32 chassis_roll;

    //底盘超级电容电压
    fp32 cap_voltage;

    //底盘模式
    uint8_t chassis_mode;

    //机器人ID
    uint8_t robot_id;
}chassis_data_t;

//底盘收到的云台数据


#ifdef SBUS_remotor
	#define SBUS_RX_BUF_NUM 25u
	#define RC_FRAME_LENGTH 25u

	#define SBUS_HEAD 0X0F
	#define SBUS_END 0X00
	#define REMOTE_RC_OFFSET 1024
	#define REMOTE_UPPER_RC_OFFSET 1024
	#define REMOTE_TOGGLE_DUAL_VAL 1024
	#define REMOTE_TOGGLE_THRE_VAL_A 600
	#define REMOTE_TOGGLE_THRE_VAL_B 1400
	#define DEAD_AREA	20
#else
	#define SBUS_RX_BUF_NUM 36u
	#define RC_FRAME_LENGTH 18u

	#define SBUS_HEAD 0X0F
	#define SBUS_END 0X00
	#define REMOTE_RC_OFFSET 1024
	#define REMOTE_TOGGLE_DUAL_VAL 1024
	#define REMOTE_TOGGLE_THRE_VAL_A 600
	#define REMOTE_TOGGLE_THRE_VAL_B 1400
	#define DEAD_AREA	120
#endif

extern chassis_data_t chassis_data;

/* 板间接收统计：发布计数与各类丢弃原因（P01 验收项） */
typedef struct
{
    uint32_t bytes_seen;        /* 进入解析器的字节数 */
    uint32_t frames_ok;         /* 完整且通过校验的帧数 */
    uint32_t frames_bad_tail;   /* 尾字节不是 'e' */
    uint32_t frames_bad_field;  /* 头尾正确但字段越界/非有限值 */
    uint32_t bytes_discarded;   /* 无头/重同步丢弃的字节数 */
    uint32_t overflow;          /* DMA 事件长度异常 */
    uint32_t restarts;          /* 成功重启接收的次数 */
    uint32_t dma_events;        /* IDLE/HT/TC 事件总数 */
} chassis_rx_stats_t;

/* 46 字节帧 DMA 发送统计：BUSY 丢弃不再表现为静默覆盖 */
typedef struct
{
    uint32_t sent;          /* 成功启动 DMA 的帧数 */
    uint32_t busy_dropped;  /* 上一帧仍在发送，整帧丢弃 */
    uint32_t rejected;      /* 长度/指针非法 */
} chassis_tx_stats_t;

extern fp32 chassis_INS_yaw, chassis_INS_pitch, chassis_spin_speed;
extern uint8_t remote_buff[SBUS_RX_BUF_NUM];
extern uint8_t gimbal_receive_data[CHASSIS_DATA_LENGTH];
extern uint8_t usart7_buf[21];											//设置缓冲区

/* 启动 USART1/UART5 接收；取代 main.c 内的直接 DMA 调用（单一所有者） */
void bsp_usart_rx_start(void);
/* 离线测试/诊断注入：按 UART1 接收顺序喂字节，走同一解析器；正式接收仍走 DMA 回调 */
void chassis_rx_feed_stream(const uint8_t *data, uint16_t len);
/* 诊断/离线测试：清解析器半帧状态与消费位置，不触碰外设 */
void chassis_rx_reset(void);
/* 读取板间接收统计（任务上下文，短临界区） */
void chassis_rx_get_stats(chassis_rx_stats_t *out);
/* 读取 46 字节帧发送统计 */
void usart1_tx_get_stats(chassis_tx_stats_t *out);
/* 一致性读取：ISR 发布，任务侧用序号快照重试 */
uint8_t chassis_data_read(chassis_data_t *out, uint32_t *generation);


//遥控器
typedef struct
{
    uint16_t online;
		uint32_t sbus_recever_time;

    struct
    {
        int16_t ch[10];
    } rc;

    struct
    {
        /* STICK VALUE */
        int16_t left_vert;
        int16_t left_hori;
        int16_t right_vert;
        int16_t right_hori;
    } joy;
		
		struct
		{
			//l1 l2 r2 r1
			uint8_t swa;//2-Stop
			uint8_t swb;//3-Stop
			uint8_t swc;//3-Stop
			uint8_t swd;//2-Stop
		} toggle;

    struct
    {
        /* VAR VALUE */
        float a;
        float b;
    } var;

    struct
    {
        /* KEY VALUE */
        uint8_t l;
				uint8_t r;
    } key;
} remoter_t;


extern remoter_t remoter;
extern void USART1_Transmit_DMA(uint8_t *pData, uint16_t Size);
extern void USART10_Transmit(uint8_t *pData, uint16_t Size);
extern void USART10_Transmit_IT(uint8_t *pData, uint16_t Size);
void USART1_Transmit(uint8_t *pData, uint16_t Size);
void USART1_Transmit_IT(uint8_t *pData, uint16_t Size);

#endif
