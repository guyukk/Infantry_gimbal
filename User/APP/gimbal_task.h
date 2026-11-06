/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "can_bsp.h"
#include "pid.h"
#include "remote_control.h"
#include "robot_param.h"

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{
    const dji_measure_t *gimbal_motor_measure;
	
	  dm_measure_t *gimbal_DM_measure;    // DM J4310 pitch
	  fp32 emit_pos_set;                  // Pitch position target (rad), also used by MIT
	
	
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;

    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle ; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
	  fp32 radian_of_ecd;

} gimbal_motor_t;	


typedef struct
{
//    fp32 max_yaw = 3.0;
//    fp32 min_yaw = -3.0;
//    fp32 max_pitch = 0.261750996;
//    fp32 min_pitch = 0.82528168;
//    uint16_t max_yaw_ecd = 0x0DE0;
//    uint16_t min_yaw_ecd = 0x1CE5;
//    uint16_t max_pitch_ecd = 0x19B7;
//    uint16_t min_pitch_ecd = 0x1C36;
	  fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;



typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
}gimbal_control_t;


extern gimbal_control_t gimbal_control;

extern int16_t yaw_can_set_current, pitch_can_set_current, shoot_can_set_current;
//让云台保持在固定的值
extern fp32 yaw_target ;
extern fp32 yaw_curret ;
extern uint8_t auto_aim_flag ;
extern fp32 aim_flag;
extern pid_type_def AIM;


extern const gimbal_motor_t *get_yaw_motor_point(void);
extern const gimbal_motor_t *get_pitch_motor_point(void);
extern void gimbal_task(void const *pvParameters);
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
extern void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
extern void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
extern void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);
extern void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
extern void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
extern void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
extern void gimbal_PID_clear(gimbal_PID_t *pid_clear);
extern fp32 gimbal_PID_Calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);


#endif
