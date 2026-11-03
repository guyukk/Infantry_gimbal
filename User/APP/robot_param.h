/**
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  * @file       robot_param.h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     3-26-2025       yibu             1. start and done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  */
#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H
#include "struct_typedef.h"

/*是否在线*/
#define OFFLINE 0X00
#define ONLINE 	0X01

/* 超级电容开关 */
#define Cap_off 0X00
#define Cap_on  0X01

/*机器人类型*/
#define Infantry_robot  0x02

/* 底盘类型 */
#define Steering_wheel  0X02

/* 云台类型 */
#define yaw_pitch_direct   			0X00

/* 发射机构类型 */
#define friction_3508   0X00

/* 开发板放置位置 */
#define gimbal_board  0x01

/* 电机类型 闲置*/

/* 代码类型 调试或者发布*/
#define debug   0
#define release 1

/* ================= 机器人整体配置 ================= */

#define ROBOT_TYPE       	Infantry_robot
#define ROBOT_MODE        debug
#define ROBOT_CHASSIS     Steering_wheel
#define ROBOT_GIMBAL      yaw_pitch_direct
#define ROBOT_FRICTION    friction_3508
#define ROBOT_BOARD				gimbal_board
#define ROBOT_CAP         Cap_off

//把底盘、云台、发射机构等通用参数藏到各个兵种的私有头文件里去
#include "Infantry_robot.h"


/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/* ================= 外部声明 ================= */
//小陀螺转速，在底盘任务中会进行修改
		
#define SPIN_FACTOR  		10
#define SPIN_SPEED 		0.7f

#endif

