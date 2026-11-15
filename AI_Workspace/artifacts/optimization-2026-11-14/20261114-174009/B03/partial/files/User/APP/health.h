#ifndef HEALTH_H
#define HEALTH_H

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 本车健康/故障源（P04）
 * ----------------------------------------------------------------------------
 * 设计约定：
 * 1) 每源一个单写者上下文：RC=UART5 ISR、BOARD_LINK=USART1 ISR、
 *    YAW/TRIGGER/PITCH=对应 FDCAN ISR、FRIC1/FRIC2=FDCAN1 ISR、IMU=INS 任务。
 * 2) 写者用序号保护「seen/last_valid_ms/generation」三元组，读者用序号快照；
 *    时间比较全部使用无符号差值，支持 tick 回绕。
 * 3) 100ms 是新增保护门限，不是设备实测周期；所有消费者统一从本头文件取值。
 * 4) last_valid_ms==0 不能当成"永远无数据"：合法首帧可能发生在 tick=0，
 *    因此用 seen 区分 NEVER_SEEN 与 STALE。
 * ============================================================================
 */

/* 通用失联门限：RC、板间链路、电机反馈统一 100ms */
#define HEALTH_TIMEOUT_MS 100u
/* 自瞄超时单独定义，保持 2000ms */
#define AUTO_AIM_TIMEOUT_MS 2000u

typedef enum
{
    HEALTH_SRC_RC = 0,          /* 遥控器 18 字节帧 */
    HEALTH_SRC_BOARD_LINK,      /* 底盘 28 字节帧 */
    HEALTH_SRC_YAW_FDB,         /* 0x205 云台 Yaw 反馈 */
    HEALTH_SRC_PITCH_FDB,       /* 0x53 DM Pitch 反馈 */
    HEALTH_SRC_TRIGGER_FDB,     /* 0x207 拨弹反馈 */
    HEALTH_SRC_FRIC1_FDB,       /* 0x201 摩擦轮 1 反馈 */
    HEALTH_SRC_FRIC2_FDB,       /* 0x202 摩擦轮 2 反馈 */
    HEALTH_SRC_IMU,             /* BMI088/Mahony 采样链 */
    HEALTH_SRC_COUNT
} health_source_e;

typedef enum
{
    HEALTH_REASON_NEVER_SEEN = 0,
    HEALTH_REASON_STALE,
    HEALTH_REASON_OK
} health_reason_e;

typedef struct
{
    uint8_t fresh;          /* 1 = 见过且未超时 */
    uint8_t reason;         /* health_reason_e */
    uint32_t last_valid_ms; /* 最近一次有效帧/样本的 tick */
    uint32_t generation;    /* 有效更新次数 */
} health_status_t;

/* 初始化（任务启动前调用） */
void health_init(void);

/* 有效数据到达：记录时间并递增 generation（可来自 ISR） */
void health_mark_valid(health_source_e source, uint32_t now_ms);

/* 当前状态快照（任务上下文） */
void health_get(health_source_e source, uint32_t now_ms, health_status_t *out);

/* 便捷判定：seen && (now-last) <= 门限 */
uint8_t health_is_fresh(health_source_e source, uint32_t now_ms);

/* 整表快照（调试/观测，任务上下文） */
void health_get_all(health_status_t *out, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* HEALTH_H */
