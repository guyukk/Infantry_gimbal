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


// Yaw DJI current polarity.
int8_t yaw_cuurent_set_polarity = YAW_CURRENT_SET_POLARITY;

int16_t yaw_can_set_current = 0, shoot_can_set_current = 0;
uint8_t enable = 1;
volatile pitch_dm_enable_status_t pitch_dm_enable_status;
static uint8_t yaw_init_hold_valid;

static uint8_t pitch_dm_feedback_allows_mit(const dm_measure_t *motor);

static float pitch_dm_raw_to_relative(float raw)
{
#if PITCH_TURN
    return PITCH_DM_ZERO_OFFSET_RAD - raw;
#else
    return raw - PITCH_DM_ZERO_OFFSET_RAD;
#endif
}

static float pitch_dm_relative_to_raw(float relative)
{
#if PITCH_TURN
    return PITCH_DM_ZERO_OFFSET_RAD - relative;
#else
    return relative + PITCH_DM_ZERO_OFFSET_RAD;
#endif
}

void pitch_dm_params_init(void)
{
    dm_measure_t *motor = get_dm_motor_measure_point(PITCH_DM_MOTOR_INDEX);
    motor->param.P_MIN = -PITCH_MIT_P_MAX;
    motor->param.P_MAX = PITCH_MIT_P_MAX;
    motor->param.V_MIN = -PITCH_MIT_V_MAX;
    motor->param.V_MAX = PITCH_MIT_V_MAX;
    motor->param.KP_MIN = 0.0f;
    motor->param.KP_MAX = 500.0f;
    motor->param.KD_MIN = 0.0f;
    motor->param.KD_MAX = 5.0f;
    motor->param.T_MIN = -PITCH_MIT_T_MAX;
    motor->param.T_MAX = PITCH_MIT_T_MAX;
}

void pitch_dm_apply_calibration(gimbal_control_t *control, float min_raw, float max_raw, float current_raw)
{
    if (control == NULL)
    {
        return;
    }
#if PITCH_TURN
    control->gimbal_pitch_motor.min_relative_angle = pitch_dm_raw_to_relative(max_raw);
    control->gimbal_pitch_motor.max_relative_angle = pitch_dm_raw_to_relative(min_raw);
#else
    control->gimbal_pitch_motor.min_relative_angle = pitch_dm_raw_to_relative(min_raw);
    control->gimbal_pitch_motor.max_relative_angle = pitch_dm_raw_to_relative(max_raw);
#endif
    control->gimbal_pitch_motor.relative_angle_set = pitch_dm_raw_to_relative(current_raw);
    control->gimbal_pitch_motor.emit_pos_set = control->gimbal_pitch_motor.relative_angle_set;
    pitch_dm_enable_status.mit_target_valid = 0U;
}

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
    if (pitch_dm_enable_status.confirmed &&
        (feedback_tick == 0U ||
         (uint32_t)(now - feedback_tick) > PITCH_MIT_FEEDBACK_TIMEOUT_MS ||
         measure->fdb.state != 1U))
    {
        // A confirmed motor can reset during a power dip. Require a new
        // enable exchange, and seed the next position target from feedback.
        pitch_dm_enable_status.confirmed = 0U;
        pitch_dm_enable_status.exhausted = 0U;
        pitch_dm_enable_status.attempts = 0U;
        pitch_dm_enable_status.confirmed_feedback_tick = 0U;
        pitch_dm_enable_status.mit_target_valid = 0U;
        pitch_dm_enable_status.last_attempt_tick = now;
    }
    if (feedback_tick != 0U &&
        (int32_t)(feedback_tick - pitch_dm_enable_status.last_attempt_tick) > 0 &&
        (uint32_t)(now - feedback_tick) <= PITCH_MIT_FEEDBACK_TIMEOUT_MS &&
        measure->fdb.state == 1U)
    {
        pitch_dm_enable_status.confirmed = 1U;
        pitch_dm_enable_status.confirmed_feedback_tick = feedback_tick;
    }

    if (pitch_dm_enable_status.confirmed)
    {
        pitch_dm_enable_status.exhausted = 0U;
        return 1U;
    }

    if (pitch_dm_enable_status.attempts >= PITCH_DM_ENABLE_MAX_ATTEMPTS)
    {
        pitch_dm_enable_status.exhausted = 1U;
        return 0U;
    }

    retry_interval = (pitch_dm_enable_status.attempts == 1U &&
                      pitch_dm_enable_status.last_tx_status == HAL_OK) ?
                     PITCH_DM_ENABLE_FIRST_RETRY_MS : PITCH_DM_ENABLE_RETRY_MS;
    if ((uint32_t)(now - pitch_dm_enable_status.last_attempt_tick) >= retry_interval)
    {
        pitch_dm_enable_status.last_tx_status = Motor_enable(&GIMBAL_CAN, PITCH_DM_CAN_ID);
        pitch_dm_enable_status.last_attempt_tick = now;
        if (pitch_dm_enable_status.last_tx_status == HAL_OK &&
            pitch_dm_enable_status.attempts < PITCH_DM_ENABLE_MAX_ATTEMPTS)
        {
            pitch_dm_enable_status.attempts++;
        }
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
    // Accept this observed pair for MIT output, but it cannot confirm enable.
    return (PITCH_DM_CAN_ID == 0x53U &&
            motor->fdb.id == 3U && motor->fdb.state == 5U);
}

static uint8_t pitch_dm_rc_ready(const RC_ctrl_t *rc, uint32_t now)
{
    uint8_t i;
    if (rc == NULL || rc->last_fdb == 0U ||
        (uint32_t)(now - rc->last_fdb) > PITCH_DM_RC_TIMEOUT_MS ||
        rc->rc.s[GIMBAL_MODE_CHANNEL] == 0U ||
        toe_is_error(DBUS_TOE))
    {
        return 0U;
    }
    for (i = 0U; i < 4U; i++)
    {
        if (rc->rc.ch[i] > 700 || rc->rc.ch[i] < -700)
        {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t pitch_dm_send_mit(gimbal_control_t *control_send)
{
    dm_measure_t *motor = control_send->gimbal_pitch_motor.gimbal_DM_measure;
    float desired;
    float delta;
    uint32_t now = HAL_GetTick();
    uint8_t can_hold;
    uint8_t lower_recovery = 0U;
    uint8_t just_seeded = 0U;

    if (motor == NULL)
    {
        return (uint8_t)HAL_ERROR;
    }

    motor->set.VEL = 0.0f;
    motor->set.TOR = 0.0f;
    can_hold = (motor->fdb.last_fdb_time != 0U &&
                (uint32_t)(now - motor->fdb.last_fdb_time) <= PITCH_MIT_FEEDBACK_TIMEOUT_MS &&
                pitch_dm_feedback_allows_mit(motor) &&
                (pitch_dm_enable_status.confirmed ||
                 (PITCH_DM_CAN_ID == 0x53U && motor->fdb.id == 3U &&
                  motor->fdb.state == 5U &&
                  pitch_dm_enable_status.attempts >= 2U)) &&
                pitch_dm_cali_status.stored_valid &&
                pitch_dm_rc_ready(control_send->gimbal_rc_ctrl, now) &&
                motor->fdb.pos == motor->fdb.pos);
    if (can_hold &&
        motor->fdb.pos < pitch_dm_cali_status.min_pos - PITCH_DM_LOWER_RECOVERY_RAD)
    {
        can_hold = 0U;
    }
    if (can_hold && !pitch_dm_enable_status.mit_target_valid)
    {
        if (motor->fdb.pos > pitch_dm_cali_status.max_pos)
        {
            can_hold = 0U;
        }
        else
        {
            /* Rebase the first target to this boot's measured position. */
            pitch_dm_enable_status.mit_target_pos = motor->fdb.pos;
            pitch_dm_enable_status.mit_target_valid = 1U;
            control_send->gimbal_pitch_motor.relative_angle_set = pitch_dm_raw_to_relative(motor->fdb.pos);
            control_send->gimbal_pitch_motor.emit_pos_set = control_send->gimbal_pitch_motor.relative_angle_set;
            just_seeded = 1U;
        }
    }
    if (can_hold &&
        gimbal_behaviour != GIMBAL_ZERO_FORCE &&
        gimbal_behaviour != GIMBAL_CALI)
    {
        lower_recovery = motor->fdb.pos < pitch_dm_cali_status.min_pos;
        if (gimbal_behaviour == GIMBAL_MOTIONLESS ||
            (gimbal_behaviour == GIMBAL_INIT && !INS.ins_flag))
        {
            desired = pitch_dm_enable_status.mit_target_pos;
        }
        else
        {
            desired = pitch_dm_relative_to_raw(control_send->gimbal_pitch_motor.emit_pos_set);
        }
        if (lower_recovery)
        {
            // The first frame holds the measured position. Subsequent frames
            // can only move the target inward, toward the stored lower limit.
            if (just_seeded || pitch_dm_enable_status.mit_target_pos >=
                               pitch_dm_cali_status.min_pos)
                desired = pitch_dm_enable_status.mit_target_pos;
            else
                desired = pitch_dm_cali_status.min_pos;
        }
        if (desired > pitch_dm_cali_status.max_pos) desired = pitch_dm_cali_status.max_pos;
        if (!just_seeded && desired < pitch_dm_cali_status.min_pos)
            desired = pitch_dm_cali_status.min_pos;
        if (desired > motor->param.P_MAX) desired = motor->param.P_MAX;
        if (desired < motor->param.P_MIN) desired = motor->param.P_MIN;
        delta = desired - pitch_dm_enable_status.mit_target_pos;
        if (lower_recovery)
        {
            if (delta > PITCH_DM_RECOVERY_POS_STEP_RAD) delta = PITCH_DM_RECOVERY_POS_STEP_RAD;
            if (delta < 0.0f) delta = 0.0f;
        }
        else
        {
            if (delta > PITCH_MIT_MAX_POS_STEP_RAD) delta = PITCH_MIT_MAX_POS_STEP_RAD;
            if (delta < -PITCH_MIT_MAX_POS_STEP_RAD) delta = -PITCH_MIT_MAX_POS_STEP_RAD;
        }
        pitch_dm_enable_status.mit_target_pos += delta;
        if (lower_recovery)
        {
            control_send->gimbal_pitch_motor.relative_angle_set =
                pitch_dm_raw_to_relative(pitch_dm_enable_status.mit_target_pos);
            control_send->gimbal_pitch_motor.emit_pos_set =
                control_send->gimbal_pitch_motor.relative_angle_set;
        }

        motor->set.POS = pitch_dm_enable_status.mit_target_pos;
        motor->set.KP = PITCH_MIT_KP;
        motor->set.KD = PITCH_MIT_KD;
    }
    else
    {
        // No calibration, feedback, remote control or enabled state: zero stiffness.
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
		
		if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE) && toe_is_error(TRIGGER_MOTOR_TOE)))
		{
			if(toe_is_error(DBUS_TOE)){ CAN_cmd_gimbal(0, 0, 0, 0); }
			else
			{
				// yaw + trigger use DJI CAN; pitch uses DM MIT.
				CAN_cmd_gimbal(yaw_can_set_current, 0, shoot_can_set_current, 0);
			}
			(void)pitch_dm_send_mit(control_send);
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
	static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, 0.0f, YAW_SPEED_PID_KD};
	float pitch_min_raw = -2.5f;
	float pitch_max_raw = -2.0f;
	//电机数据指针获取
	init->yaw_dji_measure = get_yaw_gimbal_motor_measure_point();
	// DM J4310 pitch 电机反馈指针
	init->gimbal_pitch_motor.gimbal_DM_measure = get_dm_motor_measure_point(PITCH_DM_MOTOR_INDEX);
	pitch_dm_cali_status.init_done = 0U;
	/* A missing/invalid record leaves Pitch enabled for sampling but torque-free. */
	pitch_dm_cali_status.stored_valid = 0U;
	pitch_dm_cali_status.min_pos = pitch_min_raw;
	pitch_dm_cali_status.max_pos = pitch_max_raw;
	pitch_dm_cali_load(&pitch_min_raw, &pitch_max_raw);
	osDelay(100);
	// Preserve the initial enable attempt; later retries require fresh enabled feedback.
	pitch_dm_enable_status.attempts = 0U;
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
	if (pitch_dm_enable_status.last_tx_status == HAL_OK) pitch_dm_enable_status.attempts = 1U;
	//陀螺仪数据指针获取
	init->gimbal_INT_angle_point = get_INS_angle_point();
	init->gimbal_INT_gyro_point = get_gyro_data_point();
	//遥控器数据指针获取
	init->gimbal_rc_ctrl = get_remote_control_point();
	//初始化电机模式
	init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
	init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
	yaw_init_hold_valid = 0U;

	//yaw电机pid初始化
	gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
	gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
	PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
	//清除所有PID
	gimbal_total_pid_clear(init);
	gimbal_feedback_update(init);
	pitch_dm_apply_calibration(init, pitch_min_raw, pitch_max_raw,
	                           init->gimbal_pitch_motor.gimbal_DM_measure->fdb.pos);

	init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
	init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
	init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

	init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
	init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
	pitch_dm_cali_status.init_done = 1U;
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
	#if PITCH_TURN
    feedback_update->gimbal_pitch_motor.relative_angle = -(feedback_update->gimbal_pitch_motor.radian_of_ecd - PITCH_DM_ZERO_OFFSET_RAD);
	#else
    feedback_update->gimbal_pitch_motor.relative_angle = feedback_update->gimbal_pitch_motor.radian_of_ecd - PITCH_DM_ZERO_OFFSET_RAD;
	#endif

    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    feedback_update->gimbal_yaw_motor.absolute_angle = rad_format(INS.YawTotalAngle);

#if YAW_TURN
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->yaw_dji_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);

#else
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->yaw_dji_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);
#endif
#if PITCH_DM_YAW_GYRO_PROJECTION_ENABLE
    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET)) - \
																	 arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
#else
    feedback_update->gimbal_yaw_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
#endif
    
		feedback_update->gimbal_yaw_motor.radian_of_ecd = feedback_update->yaw_dji_measure->ecd * MOTOR_ECD_TO_RAD;

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
        pitch_dm_enable_status.mit_target_valid = 0U;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // Start the DM position controller from the measured output position.
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        pitch_dm_enable_status.mit_target_valid = 0U;
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
	if (gimbal_behaviour != GIMBAL_INIT)
	{
		yaw_init_hold_valid = 0U;
	}
	
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
		if (gimbal_behaviour == GIMBAL_INIT)
		{
			// Capture the first fresh Yaw position after entering INIT. The
			// stored calibration limits are not a startup homing target.
			const dji_measure_t *yaw_fdb = set_control->yaw_dji_measure;
			uint32_t now = HAL_GetTick();
			if (yaw_fdb == NULL || yaw_fdb->last_fdb_time == 0U ||
				(uint32_t)(now - yaw_fdb->last_fdb_time) > YAW_INIT_FEEDBACK_TIMEOUT_MS)
			{
				yaw_init_hold_valid = 0U;
			}
			else if (!yaw_init_hold_valid)
			{
				set_control->gimbal_yaw_motor.relative_angle_set = set_control->gimbal_yaw_motor.relative_angle;
				gimbal_PID_clear(&set_control->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);
				PID_clear(&set_control->gimbal_yaw_motor.gimbal_motor_gyro_pid);
				yaw_init_hold_valid = 1U;
			}
			aim.receive.yaw = 0.0f;
		}
		else
		{
			// Normal encoded-angle control retains the calibrated limits.
			gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
		}
	}

	//pitch电机模式控制
	if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
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
		if (gimbal_behaviour == GIMBAL_INIT && !yaw_init_hold_valid)
		{
			control_loop->gimbal_yaw_motor.current_set = 0.0f;
			control_loop->gimbal_yaw_motor.given_current = 0;
		}
		else
		{
			gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
		}
	}

	// DM J4310 MIT mode: map the existing Pitch angle target to p_des.
	if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
	{
		control_loop->gimbal_pitch_motor.emit_pos_set = pitch_dm_enable_status.mit_target_valid ?
			pitch_dm_raw_to_relative(pitch_dm_enable_status.mit_target_pos) :
			control_loop->gimbal_pitch_motor.relative_angle;
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

/* Apply only the Yaw fields of the legacy HD/GM flash record. */
void set_cali_yaw_hook(uint16_t yaw_offset, fp32 max_yaw, fp32 min_yaw)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;
}
