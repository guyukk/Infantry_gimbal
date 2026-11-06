#include "yaw_pitch_direct.h"
#include "gimbal_task.h"
#include "main.h"
#include "robot_param.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "can_bsp.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"
#include "auto_aim.h"
#include "calibrate_task.h"


//yaw和pitch的电流极性，便于调试，调试完成后应使用宏定义
int8_t yaw_cuurent_set_polarity = YAW_CURRENT_SET_POLARITY;
int8_t pitch_cuurent_set_polarity = PITCH_CURRENT_SET_POLARITY;

//发送的电机电流
int16_t yaw_can_set_current = 0, pitch_can_set_current = 0, shoot_can_set_current = 0;
uint8_t enable = 1;
volatile pitch_dm_enable_status_t pitch_dm_enable_status;

static uint8_t pitch_dm_enable_ready(const dm_measure_t *measure)
{
    uint32_t now;
    uint32_t retry_interval;
    uint32_t feedback_tick;

    if (measure == NULL)
    {
        return 0;
    }

    now = HAL_GetTick();
    feedback_tick = measure->fdb.last_fdb_time;
    if (feedback_tick != 0U &&
        (int32_t)(feedback_tick - pitch_dm_enable_status.last_attempt_tick) >= 0 &&
        measure->fdb.state == 1)
    {
        pitch_dm_enable_status.confirmed = 1U;
        pitch_dm_enable_status.confirmed_feedback_tick = feedback_tick;
    }

    if (pitch_dm_enable_status.confirmed)
    {
        return 1U;
    }

    if (pitch_dm_enable_status.attempts >= PITCH_DM_ENABLE_MAX_ATTEMPTS)
    {
        pitch_dm_enable_status.exhausted = 1U;
        return 0U;
    }

    retry_interval = (pitch_dm_enable_status.attempts == 1U) ?
                     PITCH_DM_ENABLE_FIRST_RETRY_MS : PITCH_DM_ENABLE_RETRY_MS;
    if ((uint32_t)(now - pitch_dm_enable_status.last_attempt_tick) >= retry_interval)
    {
        pitch_dm_enable_status.last_tx_status = Motor_enable(&GIMBAL_CAN, PITCH_DM_CAN_ID);
        pitch_dm_enable_status.last_attempt_tick = now;
        pitch_dm_enable_status.attempts++;
    }

    return 0U;
}

static uint8_t pitch_dm_feedback_allows_mit(const dm_measure_t *motor)
{
    if (motor->fdb.state == 1U)
    {
        return 1U;
    }

    // This motor uses CAN ID 0x53. Its feedback byte 0 is 0x53 while the
    // indicator is solid green: ID bits overlap the feedback status nibble.
    // Accept only this observed ID/status pair; all other states stay gated.
    return (PITCH_DM_CAN_ID == 0x53U &&
            motor->fdb.id == 3U && motor->fdb.state == 5U);
}

static uint8_t pitch_dm_send_mit(gimbal_control_t *control_send)
{
    dm_measure_t *motor = control_send->gimbal_pitch_motor.gimbal_DM_measure;
    float desired;
    float delta;
    uint32_t now = HAL_GetTick();

    if (motor == NULL)
    {
        return (uint8_t)HAL_ERROR;
    }

    motor->set.VEL = 0.0f;
    motor->set.TOR = 0.0f;
    if (motor->fdb.last_fdb_time != 0U &&
        (uint32_t)(now - motor->fdb.last_fdb_time) <= PITCH_MIT_FEEDBACK_TIMEOUT_MS &&
        pitch_dm_feedback_allows_mit(motor) &&
        control_send->gimbal_pitch_motor.emit_pos_set == control_send->gimbal_pitch_motor.emit_pos_set)
    {
        if (!pitch_dm_enable_status.mit_target_valid)
        {
            // First active target starts at the measured output position.
            pitch_dm_enable_status.mit_target_pos = motor->fdb.pos;
            pitch_dm_enable_status.mit_target_valid = 1U;
        }

        desired = control_send->gimbal_pitch_motor.emit_pos_set;
        if (desired > motor->param.P_MAX) desired = motor->param.P_MAX;
        if (desired < motor->param.P_MIN) desired = motor->param.P_MIN;
        delta = desired - pitch_dm_enable_status.mit_target_pos;
        if (delta > PITCH_MIT_MAX_POS_STEP_RAD) delta = PITCH_MIT_MAX_POS_STEP_RAD;
        if (delta < -PITCH_MIT_MAX_POS_STEP_RAD) delta = -PITCH_MIT_MAX_POS_STEP_RAD;
        pitch_dm_enable_status.mit_target_pos += delta;

        motor->set.POS = pitch_dm_enable_status.mit_target_pos;
        motor->set.KP = PITCH_MIT_KP;
        motor->set.KD = PITCH_MIT_KD;
    }
    else
    {
        // Zero stiffness and feedforward while feedback is missing or stale.
        pitch_dm_enable_status.mit_target_valid = 0U;
        motor->set.POS = 0.0f;
        motor->set.KP = 0.0f;
        motor->set.KD = 0.0f;
    }

    pitch_dm_enable_status.last_mit_tx_status =
        CAN_cmd_MIT(&GIMBAL_CAN, PITCH_DM_CAN_ID, motor);
    pitch_dm_enable_status.mit_send_count++;
    pitch_dm_enable_status.last_mit_tick = now;
    return pitch_dm_enable_status.last_mit_tx_status;
}

/**
  * @brief          发送控制电流
  * @param[in]      
  * @retval         none
  */
__attribute__((used)) void gimbal_send_cmd(gimbal_control_t *control_send)
{
	if(debug == 0)
	{
		(void)pitch_dm_enable_ready(control_send->gimbal_pitch_motor.gimbal_DM_measure);
		yaw_can_set_current = yaw_cuurent_set_polarity * control_send->gimbal_yaw_motor.given_current;
		pitch_can_set_current = pitch_cuurent_set_polarity * control_send->gimbal_pitch_motor.given_current;
		
		if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE) && toe_is_error(TRIGGER_MOTOR_TOE)))
		{
			if(toe_is_error(DBUS_TOE)){ CAN_cmd_gimbal(0, 0, 0, 0); }
			else
			{
				// yaw + trigger use DJI CAN; pitch uses DM MIT.
				CAN_cmd_gimbal(yaw_can_set_current, 0, shoot_can_set_current, 0);
				(void)pitch_dm_send_mit(control_send);
			}
		}
	}
	else
	{
		CAN_cmd_gimbal(0, 0, 0, 0);
	}
}

/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
__attribute__((used))void gimbal_init(gimbal_control_t *init)
{
	static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
	static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, 0.0f, YAW_SPEED_PID_KD};
	//电机数据指针获取
	init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
	init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
	// DM J4310 pitch 电机反馈指针
	init->gimbal_pitch_motor.gimbal_DM_measure = get_dm_motor_measure_point(PITCH_DM_MOTOR_INDEX);
	init->gimbal_pitch_motor.gimbal_DM_measure->param.P_MIN = -PITCH_MIT_P_MAX;
	init->gimbal_pitch_motor.gimbal_DM_measure->param.P_MAX = PITCH_MIT_P_MAX;
	init->gimbal_pitch_motor.gimbal_DM_measure->param.V_MIN = -PITCH_MIT_V_MAX;
	init->gimbal_pitch_motor.gimbal_DM_measure->param.V_MAX = PITCH_MIT_V_MAX;
	init->gimbal_pitch_motor.gimbal_DM_measure->param.KP_MIN = 0.0f;
	init->gimbal_pitch_motor.gimbal_DM_measure->param.KP_MAX = 500.0f;
	init->gimbal_pitch_motor.gimbal_DM_measure->param.KD_MIN = 0.0f;
	init->gimbal_pitch_motor.gimbal_DM_measure->param.KD_MAX = 5.0f;
	init->gimbal_pitch_motor.gimbal_DM_measure->param.T_MIN = -PITCH_MIT_T_MAX;
	init->gimbal_pitch_motor.gimbal_DM_measure->param.T_MAX = PITCH_MIT_T_MAX;
	osDelay(100);
	// Preserve the initial enable attempt; later retries require fresh enabled feedback.
	pitch_dm_enable_status.attempts = 1U;
	pitch_dm_enable_status.confirmed = 0U;
	pitch_dm_enable_status.exhausted = 0U;
	pitch_dm_enable_status.confirmed_feedback_tick = 0U;
	pitch_dm_enable_status.mit_send_count = 0U;
	pitch_dm_enable_status.last_mit_tick = 0U;
	pitch_dm_enable_status.last_mit_tx_status = 0xFFU;
	pitch_dm_enable_status.mit_target_valid = 0U;
	pitch_dm_enable_status.mit_target_pos = 0.0f;
	pitch_dm_enable_status.last_attempt_tick = HAL_GetTick();
	pitch_dm_enable_status.last_tx_status = Motor_enable(&GIMBAL_CAN, PITCH_DM_CAN_ID);
	//陀螺仪数据指针获取
	init->gimbal_INT_angle_point = get_INS_angle_point();
	init->gimbal_INT_gyro_point = get_gyro_data_point();
	//遥控器数据指针获取
	init->gimbal_rc_ctrl = get_remote_control_point();
	//初始化电机模式
	init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
	init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

	//yaw电机pid初始化
	gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
	gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
	PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
	//pitch电机pid初始化
	gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
	gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
	PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
	//清除所有PID
	gimbal_total_pid_clear(init);
	gimbal_feedback_update(init);
	
		init->gimbal_pitch_motor.max_relative_angle = -2.0f;			// 0.50f
		init->gimbal_pitch_motor.min_relative_angle = -2.5f;     //-0.45f
	// Apply a valid DM Pitch safe-position record after defaults are established.
	pitch_dm_cali_load(&init->gimbal_pitch_motor.min_relative_angle,
	                   &init->gimbal_pitch_motor.max_relative_angle);
	pitch_dm_cali_status.min_pos = init->gimbal_pitch_motor.min_relative_angle;
	pitch_dm_cali_status.max_pos = init->gimbal_pitch_motor.max_relative_angle;
	init->gimbal_pitch_motor.emit_pos_set =
		(init->gimbal_pitch_motor.max_relative_angle + init->gimbal_pitch_motor.min_relative_angle) / 2.0f;

	init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
	init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
	init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

	init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
	init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.emit_pos_set;
	init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
}

/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
__attribute__((used))void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}

/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
__attribute__((used))void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    //云台数据更新
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

    // DM J4310 pitch 反馈: pos 为输出轴绝对位置 (rad)
    feedback_update->gimbal_pitch_motor.radian_of_ecd = feedback_update->gimbal_pitch_motor.gimbal_DM_measure->fdb.pos;
    feedback_update->gimbal_pitch_motor.motor_gyro = feedback_update->gimbal_pitch_motor.gimbal_DM_measure->fdb.vel;
	#if PITCH_TURN
    feedback_update->gimbal_pitch_motor.relative_angle = -(feedback_update->gimbal_pitch_motor.radian_of_ecd - PITCH_DM_ZERO_OFFSET_RAD);
	#else
    feedback_update->gimbal_pitch_motor.relative_angle = feedback_update->gimbal_pitch_motor.radian_of_ecd - PITCH_DM_ZERO_OFFSET_RAD;
	#endif

    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    feedback_update->gimbal_yaw_motor.absolute_angle = rad_format(INS.YawTotalAngle);

#if YAW_TURN
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);

#else
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);
#endif
#if PITCH_DM_YAW_GYRO_PROJECTION_ENABLE
    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET)) - \
																	 arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
#else
    feedback_update->gimbal_yaw_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
#endif
    
		feedback_update->gimbal_yaw_motor.radian_of_ecd = feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd	*	MOTOR_ECD_TO_RAD;	

		//发送云台数据（陀螺仪的yaw值fp32、yaw轴编码器的值fp32）  
//    send_data_to_chassis(feedback_update);                                                  
}

/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
__attribute__((used))void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // Keep the last Pitch position command as the starting target.
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.emit_pos_set;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;

}

/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
__attribute__((used))void gimbal_set_control(gimbal_control_t *set_control)
{
	if (set_control == NULL)
	{
		return;
	}

	static fp32 add_yaw_angle = 0.0f;
	static fp32 add_pitch_angle = 0.0f;
	
	//控制值设置
	gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
	
	//yaw电机模式控制
	if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
	{
		//raw模式下，直接发送控制值
		set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
	}
	else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		//gyro模式下，陀螺仪角度控制
		gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
	}
	else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
		//enconde模式下，电机编码角度控制
		gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
	}

	//pitch电机模式控制
	if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
	{
		//raw模式下，直接发送控制值
		set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
	}
	else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		//gyro模式下，陀螺仪角度控制
		gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
	}
	else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
		//enconde模式下，电机编码角度控制
		gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
	}
}

/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
__attribute__((used))void gimbal_control_loop(gimbal_control_t *control_loop)
{
	if (control_loop == NULL)
	{
		return;
	}

	if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
	{
		gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
	}
	else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
	}
	else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
		gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
	}

	// DM J4310 MIT mode: map the existing Pitch angle target to p_des.
	if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
	{
		control_loop->gimbal_pitch_motor.emit_pos_set = (control_loop->gimbal_pitch_motor.max_relative_angle + control_loop->gimbal_pitch_motor.min_relative_angle) / 2.0f;
	}
	else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		control_loop->gimbal_pitch_motor.emit_pos_set = control_loop->gimbal_pitch_motor.absolute_angle_set;
	}
	else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
		control_loop->gimbal_pitch_motor.emit_pos_set = control_loop->gimbal_pitch_motor.relative_angle_set;
	}
}

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
__attribute__((used))void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}

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
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
	if (gimbal_control.gimbal_cali.step == 0)
	{
		//初始化
		gimbal_control.gimbal_cali.step             = GIMBAL_CALI_START_STEP;
		//保存进入时候的数据，作为起始数据，来判断最大，最小值
		gimbal_control.gimbal_cali.max_pitch        = gimbal_control.gimbal_pitch_motor.absolute_angle;
		gimbal_control.gimbal_cali.max_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
		gimbal_control.gimbal_cali.max_yaw          = gimbal_control.gimbal_yaw_motor.absolute_angle;
		gimbal_control.gimbal_cali.max_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
		gimbal_control.gimbal_cali.min_pitch        = gimbal_control.gimbal_pitch_motor.absolute_angle;
		gimbal_control.gimbal_cali.min_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
		gimbal_control.gimbal_cali.min_yaw          = gimbal_control.gimbal_yaw_motor.absolute_angle;
		gimbal_control.gimbal_cali.min_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
		return 0;
	}
	else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
	{
		//校准
		calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
		(*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
		(*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
		(*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
		(*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
		gimbal_control.gimbal_yaw_motor.offset_ecd              = *yaw_offset;
		gimbal_control.gimbal_yaw_motor.max_relative_angle      = *max_yaw;
		gimbal_control.gimbal_yaw_motor.min_relative_angle      = *min_yaw;
		gimbal_control.gimbal_pitch_motor.offset_ecd            = *pitch_offset;
		gimbal_control.gimbal_pitch_motor.max_relative_angle    = *max_pitch;
		gimbal_control.gimbal_pitch_motor.min_relative_angle    = *min_pitch;
		gimbal_control.gimbal_cali.step = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
  * @brief          云台校准计算，将校准记录的中值,最大 最小值
  * @param[out]     yaw 中值 指针
  * @param[out]     pitch 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     pitch 最大相对角度 指针
  * @param[out]     pitch 最小相对角度 指针
  * @retval         none
  */
void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ECD_RANGE;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ECD_RANGE;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);
    
    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    if (temp_max_ecd > temp_min_ecd)
    {
        temp_min_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}


