/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_3508_H
#define SHOOT_3508_H
#include "struct_typedef.h"
#include "shoot.h"

#include "can_bsp.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"


//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern __attribute__((used))void shoot_init(void);
extern __attribute__((used))void shoot_set_mode(void);
extern __attribute__((used))void shoot_feedback_update(void);
extern __attribute__((used))void trigger_motor_turn_back(void);
extern __attribute__((used))void shoot_bullet_control(void);


extern void fric1_on(shoot_control_t *shoot);
extern void fric2_on(shoot_control_t *shoot);
extern void fric_off(void);


#endif
