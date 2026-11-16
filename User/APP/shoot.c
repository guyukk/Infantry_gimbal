#include "health.h"
//#include "bsp_laser.h"
#include "arm_math.h"
#include "bsp_PWM.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"
#include "main.h"
#include "pid.h"
#include "referee.h"
#include "shoot.h"
#include "shoot_3508.h"
#include <stdbool.h>


#define shoot_fric1_on(shoot)		fric1_on((shoot))		//摩擦轮1宏定义
#define shoot_fric2_on(shoot)		fric2_on((shoot))		//摩擦轮2宏定义
#define shoot_fric_off()				fric_off()					//关闭两个摩擦轮

//#define shoot_laser_on()    laser_on()      //激光开启宏定义
//#define shoot_laser_off()   laser_off()     //激光关闭宏定义

//#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)		//微动开关IO




shoot_control_t shoot_control;			//射击数据


fp32 continue_trigger_speed = CONTINUE_TRIGGER_SPEED;

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
void shoot_safety_reset(void)
{
    shoot_control.shoot_mode=SHOOT_STOP;
    PID_clear(&shoot_control.trigger_motor_angle_pid);
    PID_clear(&shoot_control.trigger_motor_speed_pid);
    PID_clear(&shoot_control.fric1.speed_pid);
    PID_clear(&shoot_control.fric2.speed_pid);
    shoot_control.given_current=0;
    shoot_control.speed_set=shoot_control.trigger_speed_set=0.0f;
    shoot_control.fric1.speed_set=shoot_control.fric2.speed_set=0.0f;
    shoot_control.fric1.give_current=shoot_control.fric2.give_current=0;
    shoot_control.angle_set=shoot_control.angle;
    shoot_control.block_time=shoot_control.reverse_time=0U;
    shoot_control.press_l=shoot_control.last_press_l=shoot_control.press_r=shoot_control.last_press_r=0U;
    shoot_control.press_l_time=shoot_control.press_r_time=shoot_control.rc_s_time=0U;
    shoot_control.move_flag=0U;
    shoot_history_reset();
}

int16_t shoot_control_loop(void)
{
    static uint32_t last_epoch;
    if (!health_gate_ready(HEALTH_GATE_SHOOT,HEALTH_SHOOT_MASK,HAL_GetTick()))
    {
        shoot_safety_reset();
        CAN_cmd_friction(0,0,0,0);
        return 0;
    }
    if (last_epoch != health_gate_epoch(HEALTH_GATE_SHOOT))
    {
        shoot_feedback_update();
        shoot_safety_reset();
        last_epoch=health_gate_epoch(HEALTH_GATE_SHOOT);
    }
	shoot_set_mode();        //设置状态机	
	shoot_feedback_update(); //更新数据
	
	if( shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_R || (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_mid(shoot_control.shoot_rc->rc.s[GIMBAL_MODE_CHANNEL])) ){
		Bombbay_on();
	}
	else if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_G || (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_mid(shoot_control.shoot_rc->rc.s[GIMBAL_MODE_CHANNEL])) ){
		Bombbay_off();
	}

	//射击停止状态设置拨弹盘目标速度为0
	if (shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_control.speed_set = 0.0f;
	}
	//摩擦轮准备状态设置拨弹盘目标速度为0
	else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
	{
		shoot_control.speed_set = 0.0f;
	}
	//可以发射状态设置拨弹盘目标速度为0
	else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)
	{
		shoot_control.speed_set = 0.0f;
	}
	//单发状态执行单发函数
	else if (shoot_control.shoot_mode == SHOOT_BULLET)
	{
		shoot_bullet_control();
	}
	//连发状态设置拨弹盘转速为定值
	else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
	{
		shoot_control.speed_set = continue_trigger_speed;
		trigger_motor_turn_back();
	}
	
	PID_Calc(&shoot_control.trigger_motor_speed_pid, shoot_control.speed, shoot_control.speed_set);
	
		shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_speed_pid.out);
	
	//遥控器中档禁止发弹
	if(gimbal_cmd_to_shoot_stop())
	{
		shoot_control.shoot_mode = SHOOT_STOP;
	}
	
	if(shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_control.given_current = 0;
		fric_off();
	}
	else
	{
		shoot_control.shoot_time = HAL_GetTick();
	}
	
    /* One calculation per wheel and cycle. A stopped or overheated pair must
       not integrate and then have its current overwritten downstream. */
    if (shoot_control.shoot_mode > SHOOT_STOP &&
        shoot_control.fric1.shoot_motor_measure != NULL &&
        shoot_control.fric2.shoot_motor_measure != NULL &&
        shoot_control.fric1.shoot_motor_measure->temperate < FRIC_OVER_TEMP_C &&
        shoot_control.fric2.shoot_motor_measure->temperate < FRIC_OVER_TEMP_C)
    {
        shoot_fric1_on(&shoot_control);
        shoot_fric2_on(&shoot_control);
    }
    else fric_off();
    CAN_cmd_friction(shoot_control.fric1.give_current,shoot_control.fric2.give_current,0,0);

	//拨弹盘的控制电流值
	return shoot_control.given_current;
}

//射击热量检测
void shoot_calories_detect()
{		
	shoot_control.last_calories = shoot_control.calories;
	shoot_control.calories += shoot_control.damn_calories;
	shoot_control.shoot_delay = HAL_GetTick() - shoot_control.shoot_time;
}
