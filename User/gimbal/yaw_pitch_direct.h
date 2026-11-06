#ifndef YAW_PITCH_DIRECT_H
#define YAW_PITCH_DIRECT_H
#include "struct_typedef.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "can_bsp.h"
#include "pid.h"
#include "remote_control.h"

#include "INS_task.h"

typedef struct
{
    uint8_t attempts;
    uint8_t last_tx_status;
    uint8_t confirmed;
    uint8_t exhausted;
    uint32_t last_attempt_tick;
    uint32_t confirmed_feedback_tick;
    uint32_t mit_send_count;
    uint32_t last_mit_tick;
    uint8_t last_mit_tx_status;
    uint8_t mit_target_valid;
    float mit_target_pos;
} pitch_dm_enable_status_t;

extern volatile pitch_dm_enable_status_t pitch_dm_enable_status;

extern void gimbal_init(gimbal_control_t *init);		
extern void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change);
extern void gimbal_set_mode(gimbal_control_t *set_mode);
extern void gimbal_feedback_update(gimbal_control_t *feedback_update);
extern void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change);
extern void gimbal_set_control(gimbal_control_t *set_control);
extern void gimbal_control_loop(gimbal_control_t *control_loop);
extern void gimbal_send_cmd(gimbal_control_t *control_send);
/**
  * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     yaw 中值 指针
  * @param[out]     pitch 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     pitch 最大相对角度 指针
  * @param[out]     pitch 最小相对角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */

/**
  * @brief          gimbal cali data, set motor offset encode, max and min relative angle
  * @param[in]      yaw_offse:yaw middle place encode
  * @param[in]      pitch_offset:pitch place encode
  * @param[in]      max_yaw:yaw max relative angle
  * @param[in]      min_yaw:yaw min relative angle
  * @param[in]      max_yaw:pitch max relative angle
  * @param[in]      min_yaw:pitch min relative angle
  * @retval         none
  */
/**
  * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
  * @param[in]      yaw_offse:yaw 中值
  * @param[in]      pitch_offset:pitch 中值
  * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
  * @param[in]      min_yaw:yaw 最小相对角度
  * @param[in]      max_yaw:pitch 最大相对角度
  * @param[in]      min_yaw:pitch 最小相对角度
  * @retval         返回空
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);


#endif
