#include "host_control.h"
#include "health.h"
/**
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  * @file       shoot.c/h
  * @brief      3508射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     9-14-2018       yibu              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  */

#include "cmsis_os.h"
#include "main.h"
#include "robot_param.h"
#include "shoot.h"
#include "shoot_3508.h"


//#include "bsp_laser.h"
#include "arm_math.h"
#include "bsp_PWM.h"
#include "can_bsp.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"
#include "pid.h"
#include "referee.h"
#include <stdbool.h>




/* Each wheel owns its PID history. Stop/overtemperature clears both outputs. */
__attribute__((used))void fric1_on(shoot_control_t *shoot)
{
    if (shoot->fric1.shoot_motor_measure == NULL) {shoot->fric1.give_current=0;return;}
    shoot->fric1.give_current=(int16_t)PID_Calc(&shoot->fric1.speed_pid,shoot->fric1.speed,shoot->fric1.speed_set);
}
__attribute__((used))void fric2_on(shoot_control_t *shoot)
{
    if (shoot->fric2.shoot_motor_measure == NULL) {shoot->fric2.give_current=0;return;}
    shoot->fric2.give_current=(int16_t)PID_Calc(&shoot->fric2.speed_pid,shoot->fric2.speed,shoot->fric2.speed_set);
}
__attribute__((used))void fric_off(void)
{
    PID_clear(&shoot_control.fric1.speed_pid);
    PID_clear(&shoot_control.fric2.speed_pid);
    shoot_control.fric1.give_current=0;
    shoot_control.fric2.give_current=0;
}

/* 函数原型声明 */
void shoot_set_mode(void);
void shoot_feedback_update(void);


/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
__attribute__((used))void shoot_init(void)
{
	shoot_control.shoot_mode = SHOOT_STOP;
	shoot_control.shoot_limit = SHOOT_LIMIT;
	shoot_control.shoot_cool = SHOOT_COOL;
	shoot_control.damn_calories = DAMN_CALORIES;
	shoot_control.shoot_time = HAL_GetTick();
	shoot_control.shoot_delay = 0;
	static const fp32 trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
	static const fp32 fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};
	//遥控器指针
	shoot_control.shoot_rc = get_remote_control_point();
	//电机指针
	shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
	shoot_control.fric1.shoot_motor_measure = get_fric1_motor_measure_point();
	shoot_control.fric2.shoot_motor_measure = get_fric2_motor_measure_point();
	//初始化PID
	PID_init(&shoot_control.trigger_motor_speed_pid, PID_POSITION, trigger_speed_pid, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_control.fric1.speed_pid, PID_POSITION, fric_speed_pid, FRIC_MAX_OUT, FRIC_MAX_IOUT);
	PID_init(&shoot_control.fric2.speed_pid, PID_POSITION, fric_speed_pid, FRIC_MAX_OUT, FRIC_MAX_IOUT);
	//更新数据
	shoot_set_mode();
	shoot_feedback_update();
	shoot_control.ecd_count = 0;
	shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
	shoot_control.given_current = 0;
	shoot_control.move_flag = 0;
	shoot_control.angle_set = shoot_control.angle;
	shoot_control.speed = 0.0f;
	shoot_control.speed_set = 0.0f;
	shoot_control.key_time = 0;
}

uint16_t press_long_time = PRESS_LONG_TIME;
fp32 fric_up = FRIC_UP;

/**
  * @brief          射击状态机：上拨切换就绪/关闭；就绪后离开上档进入连发；鼠标释放可触发定时拨弹
  * @param[in]      void
  * @retval         void
  */
static uint8_t host_owns_continuous;
static int8_t shoot_last_s=RC_SW_UP;
static bool shoot_pressed_E, shoot_last_pressed_E;
static fp32 speed_fliter_1, speed_fliter_2, speed_fliter_3;

void shoot_history_reset(void)
{
    speed_fliter_1=speed_fliter_2=speed_fliter_3=0.0f;
    host_owns_continuous=0U;
    host_fire_cancel();
    shoot_last_s=shoot_control.shoot_rc ? shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL] : RC_SW_UP;
    shoot_pressed_E=shoot_last_pressed_E=shoot_control.shoot_rc ? !!(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_E) : 0U;
}
__attribute__((used))void shoot_set_mode(void)
{
	//左拨杆上升沿触发射击，为保证刚上电时，即使左拨杆在上档也不会触发射击，所以初始化上一次左拨杆的位置为上档

	//获取当前左拨杆的位置
	uint8_t current_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];

	/* 上拨状态切换 */
	if (switch_is_up(current_s))
	{
		// 上升沿检测
		if (!switch_is_up(shoot_last_s))
		{
			// 切换 STOP <-> READY_FRIC
			shoot_control.shoot_mode = (shoot_control.shoot_mode == SHOOT_STOP) ? SHOOT_READY_FRIC : SHOOT_STOP;
		}
	}
	/* 下拨触发射击 */
	else if (!switch_is_up(current_s))
	{
		if (switch_is_up(shoot_last_s))
		{
			// 在准备完成状态允许触发
			if (shoot_control.shoot_mode == SHOOT_READY_BULLET) 
			{
				shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
			}
		}
	}
	
	{
		//可以使用E键开启和关闭摩擦轮

		shoot_pressed_E = shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_E;
		if (shoot_pressed_E && !shoot_last_pressed_E)
		{
			shoot_control.shoot_mode = (shoot_control.shoot_mode >= SHOOT_READY_FRIC) ? SHOOT_STOP : SHOOT_READY_FRIC;
		}
		shoot_last_pressed_E = shoot_pressed_E;
	}
	
//	//射击热量及射击热量限制获取
//	shoot_control.heat_limit = ;
//	shoot_control.heat = ;
	
	/* 摩擦轮准备完成检测（95%阈值）*/
	if (shoot_control.shoot_mode == SHOOT_READY_FRIC  && \
		 (shoot_control.fric1.speed <= -FRIC_UP * 0.95f) && \
		 (shoot_control.fric2.speed >= FRIC_UP * 0.95f))
	{
		shoot_control.shoot_mode = SHOOT_READY_BULLET;
	}
	
	//确保不是在 停止 或 摩擦轮准备 时执行发射
	if(shoot_control.shoot_mode >= SHOOT_READY_BULLET)
	{
		//单击左键且在松手时单发
		if (shoot_control.press_l == 0 && shoot_control.last_press_l && shoot_control.shoot_mode != SHOOT_CONTINUE_BULLET)
		{
			shoot_control.shoot_mode = SHOOT_BULLET;
		}
		//长按左键时进入连发
		if ((shoot_control.press_l_time >= press_long_time) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
		{
			shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
		}
		//长按左键且松手时退出连发
		if(shoot_control.press_l == 0 && shoot_control.last_press_l && shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
		{
			shoot_control.shoot_mode = SHOOT_READY_BULLET;
			shoot_control.speed_set = 0.0f;
		}
	}
	
	//打开摩擦轮
	if(shoot_control.shoot_mode == SHOOT_READY_FRIC){
		//直接高速射击
		shoot_control.fric1.speed_set = fric_up;
		shoot_control.fric2.speed_set = -fric_up;
	}
	//关闭摩擦轮
	else if(shoot_control.shoot_mode == SHOOT_STOP){
		shoot_control.fric1.speed_set = 0;
		shoot_control.fric2.speed_set = 0;
	}
	
	//如果云台状态是 无力状态，就关闭射击
	if (gimbal_cmd_to_shoot_stop())
	{
		shoot_control.shoot_mode = SHOOT_STOP;
	}
	
    /* Local stop/manual fire wins. Host requests never modify RC input. */
    if (shoot_control.shoot_mode == SHOOT_STOP || gimbal_cmd_to_shoot_stop())
    { host_fire_cancel();host_owns_continuous=0U; }
    else if (shoot_control.press_l || shoot_control.last_press_l || switch_is_down(current_s))
    { host_fire_cancel();host_owns_continuous=0U; }
    else
    {
        host_fire_request_t request;
        uint8_t active=host_fire_take(&request);
        if (active && shoot_control.shoot_mode >= SHOOT_READY_BULLET)
        {
            if (request.continuous) {shoot_control.shoot_mode=SHOOT_CONTINUE_BULLET;host_owns_continuous=1U;}
            else if (request.single) {shoot_control.shoot_mode=SHOOT_BULLET;host_owns_continuous=0U;}
        }
        else if (host_owns_continuous && shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        { shoot_control.shoot_mode=SHOOT_READY_BULLET;shoot_control.speed_set=0.0f;host_owns_continuous=0U; }
    }
	shoot_last_s = current_s;
}

/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
__attribute__((used))void shoot_feedback_update(void)
{




	//拨弹轮电机速度滤波一下
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

	//更新热量数据，减去热量冷却
	static uint32_t last_time = 0;
	uint32_t time = HAL_GetTick();
	shoot_control.calories -= shoot_control.shoot_cool * (time - last_time) / 1000.0f;
	last_time = time;

	//二阶低通滤波
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];


		shoot_control.speed = shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;

	//电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
	if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
	{
		shoot_control.ecd_count--;
	}
	else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
	{
		shoot_control.ecd_count++;
	}

	if (shoot_control.ecd_count == FULL_COUNT)
	{
		shoot_control.ecd_count = -(FULL_COUNT - 1);
	}
	else if (shoot_control.ecd_count == -FULL_COUNT)
	{
		shoot_control.ecd_count = FULL_COUNT - 1;
	}
	//计算输出轴角度
	shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
	
	//计算摩擦轮电机的速度
	//2006电机是12位，3508是14位，所以需要除以4来转换
	shoot_control.fric1.speed = shoot_control.fric1.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED / 4.0f;
	shoot_control.fric2.speed = shoot_control.fric2.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED / 4.0f;
	shoot_control.fric1.accel = (shoot_control.fric1.shoot_motor_measure->last_ecd - shoot_control.fric1.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE / 4.0f * GIMBAL_CONTROL_TIME / 1000.0f;
	shoot_control.fric2.accel = (shoot_control.fric2.shoot_motor_measure->last_ecd - shoot_control.fric2.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE / 4.0f * GIMBAL_CONTROL_TIME / 1000.0f;

	//鼠标按键
	shoot_control.last_press_l = shoot_control.press_l;
	shoot_control.last_press_r = shoot_control.press_r;
	shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
	shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
	
	//长按计时
	if (shoot_control.press_l)
	{
		if (shoot_control.press_l_time < PRESS_LONG_TIME)
		{
			shoot_control.press_l_time++;
		}
	}
	else
	{
		shoot_control.press_l_time = 0;
	}
	
	{
		static uint8_t aim_mode = 1;
		if(shoot_control.press_r && aim_mode == 1)
		{
			aim_mode = 0;
			gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle;
			gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.kp = 7.2f;
		}
		else if((!shoot_control.press_r) && aim_mode == 0)
		{
			aim_mode = 1;
			gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle;
			gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.kp = 56.0f;
		}
	}
	
	//射击开关下档时间计时
	if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
	{

		if (shoot_control.rc_s_time < RC_S_LONG_TIME)
		{
			shoot_control.rc_s_time++;
		}
	}
	else
	{
		shoot_control.rc_s_time = 0;
	}

	//打开摩擦轮
	if(shoot_control.shoot_mode == SHOOT_READY_FRIC)
	{
		shoot_control.fric1.speed_set = -FRIC_UP;
		shoot_control.fric2.speed_set = FRIC_UP;
	}
	//关闭摩擦轮
	else if(shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_control.fric1.speed_set = FRIC_DOWN;
		shoot_control.fric2.speed_set = FRIC_DOWN;
	}
}

__attribute__((used))void trigger_motor_turn_back(void)
{
	if( shoot_control.block_time < BLOCK_TIME)
	{
		shoot_control.speed_set = CONTINUE_TRIGGER_SPEED;
	}
	else
	{
		shoot_control.speed_set = -CONTINUE_TRIGGER_SPEED;
	}

	if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
	{
		shoot_control.block_time++;
		shoot_control.reverse_time = 0;
	}
	else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
	{
		shoot_control.reverse_time++;
	}
	else
	{
		shoot_control.block_time = 0;
	}
}

uint32_t shoot_bullet_start_time;
uint16_t shoot_duaration = SHOOT_SINGLE_DURATION_MS;
fp32 trigger_speed = TRIGGER_SPEED;

//fp32 plus_angle = 0.95f;

/**
  * @brief          拨弹定时控制；不验证角度完成或实际弹丸数量
  * @param[in]      void
  * @retval         void
  */
__attribute__((used))void shoot_bullet_control(void)
{
	// Single feed is time based; it does not verify a completed angle or ball count.
	if (shoot_control.move_flag == 0)
	{
		shoot_control.move_flag = 1;
//		shoot_control.angle_set = rad_format(shoot_control.angle - plus_angle);
		shoot_bullet_start_time = HAL_GetTick();
	}
	//到达角度或超时判断
	else if((HAL_GetTick() - shoot_bullet_start_time) > shoot_duaration)
	{
		shoot_control.move_flag = 0;
		shoot_control.last_calories = shoot_control.calories; 
		shoot_control.calories += shoot_control.damn_calories;
		shoot_control.shoot_mode = SHOOT_READY_BULLET;
	}
	else
	{
		shoot_control.speed_set = trigger_speed;
	}
}


