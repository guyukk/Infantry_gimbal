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

extern void pitch_dm_params_init(void);
extern void pitch_dm_apply_calibration(gimbal_control_t *control, float min_raw, float max_raw, float current_raw);
extern void gimbal_init(gimbal_control_t *init);		
extern void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change);
extern void gimbal_set_mode(gimbal_control_t *set_mode);
extern void gimbal_feedback_update(gimbal_control_t *feedback_update);
extern void gimbal_set_control(gimbal_control_t *set_control);
extern void gimbal_control_loop(gimbal_control_t *control_loop);
extern void gimbal_send_cmd(gimbal_control_t *control_send);
extern void set_cali_yaw_hook(uint16_t yaw_offset, fp32 max_yaw, fp32 min_yaw);


#endif
