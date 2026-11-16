#ifndef _AUTO_AIM_H
#define _AUTO_AIM_H

#include "main.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "bsp_usart.h"
#include "message_task.h"
#include "shoot.h"
// [SYNC_FROM_H] Header synced from H:\DM-balanceV1\User\APP (adds control tick prototype)

// timing (ms)
#define AIM_INIT_TIME     500
#define AUTO_AIM_TIMEOUT  2000
#define AUTO_AIM_TIME     1
#define PRESS_TIME        500

/* uproto tick ownership: prefer comm_app task to tick protocol. */
/* uproto tick职责：优先由comm_app任务驱动协议栈。 */
#ifndef AUTO_AIM_UPROTO_TICK_ENABLE
#define AUTO_AIM_UPROTO_TICK_ENABLE 0
#endif

// simple bounds for received yaw/pitch
//#define MAX_VAL   0.0025f
#define MAX_VAL   0.001f
#define MIN_VAL  -MAX_VAL

#define MAX_YAW   MAX_VAL
#define MIN_YAW   MIN_VAL

#define MAX_PITCH MAX_VAL
#define MIN_PITCH MIN_VAL

typedef union {
    int16_t  int16;
    uint8_t  bytes[2];
} int16_bytes_t;

typedef union {
    fp32     fp32;
    uint8_t  bytes[4];
} fp32_bytes_t;

typedef enum {
    AIM_OFF = 0x00,
    AIM_ON  = 0x01
} aim_switch;

typedef struct {
    fp32 yaw;
    fp32 pitch;
    fp32 distance;
    fp32 shoot_delay;
} received_data;

typedef struct {
    received_data    receive;

    uint8_t          online;
    uint8_t          auto_aim_flag;   // 0: off, 1: on
    uint16_t         shoot_delay;     // ms

    uint16_t         yaw_delay;
    uint16_t         pitch_delay;

    uint32_t         last_fdb;        // Valid DELTA receive time only; tick zero is valid.
    const RC_ctrl_t *aim_rc;
} auto_aim_t;

extern auto_aim_t aim;
#define AUTO_AIM_HEALTH_API 1
typedef struct {
    uint32_t last_rx_ms, generation;
    uint64_t host_ts_us;
    uint16_t status;
    uint8_t seen;
} auto_aim_rx_status_t;
void auto_aim_init(auto_aim_t *state);
void auto_aim_set(auto_aim_t *state);
void auto_aim_feedback_update(auto_aim_t *state);
void auto_aim_cancel(void);
uint8_t auto_aim_is_ready(void);
fp32 auto_aim_take_increment(uint8_t pitch);
void auto_aim_rx_status(auto_aim_rx_status_t *out);

extern void auto_aim_task(void const *pvParameters);
extern void send_to_minipc(void);

// Bridge: apply host delta in micro-degree (udeg)
void auto_aim_apply_delta_udeg(int32_t dyaw_udeg,
                               int32_t dpitch_udeg,
                               uint16_t status,
                               uint64_t ts_us);

// MCU-side control tick: trajectory shaping + increment generation
void auto_aim_control_tick(auto_aim_t *aim_loop);

#endif
