
#include <stdint.h>
#include <stddef.h>
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3
#define RC_DEADBAND 10
#define YAW_RC_SEN -0.000005f
#define PITCH_RC_SEN -0.000006f
#define YAW_MOUSE_SEN 0.00005f
#define PITCH_MOUSE_SEN 0.00015f
#define YAW_INIT_FEEDBACK_TIMEOUT_MS 100U
typedef float fp32;
#define fabs(x) ((x) < 0.0f ? -(x) : (x))
enum { GIMBAL_MOTOR_RAW = 0, GIMBAL_MOTOR_GYRO = 1, GIMBAL_MOTOR_ENCONDE = 2 };
enum { GIMBAL_ZERO_FORCE = 0, GIMBAL_INIT = 1, GIMBAL_RELATIVE_ANGLE = 4 };
#define PITCH_TURN 0
#define PITCH_DM_ZERO_OFFSET_RAD 0.0f
typedef struct {
    struct { int16_t ch[5]; } rc;
    struct { int16_t x, y; } mouse;
} RC_ctrl_t;
typedef struct {
    int last_gimbal_motor_mode, gimbal_motor_mode, given_current;
    int gimbal_motor_relative_angle_pid, gimbal_motor_gyro_pid;
    fp32 raw_cmd_current, current_set;
    fp32 absolute_angle_set, absolute_angle;
    fp32 relative_angle_set, relative_angle;
    fp32 motor_gyro_set, motor_gyro;
    fp32 min_relative_angle, max_relative_angle, emit_pos_set;
} gimbal_motor_t;
typedef struct { uint32_t last_fdb_time; } dji_measure_t;
typedef struct {
    gimbal_motor_t gimbal_yaw_motor, gimbal_pitch_motor;
    const RC_ctrl_t *gimbal_rc_ctrl;
    const dji_measure_t *yaw_dji_measure;
} gimbal_control_t;
gimbal_control_t gimbal_control;
dji_measure_t yaw_feedback;
int gimbal_behaviour = GIMBAL_ZERO_FORCE;
uint32_t fake_tick;
uint32_t yaw_relative_calls, yaw_pid_clears;
static uint8_t yaw_init_hold_valid;
volatile struct { uint8_t mit_target_valid; fp32 mit_target_pos; } pitch_dm_enable_status;
RC_ctrl_t rc_ctrl;
struct { struct { fp32 yaw, pitch; } receive; } aim;
struct { uint8_t ins_flag; } INS;
#define rad_format(x) (x)
#define rc_deadband_limit(input, output, deadband) \
    do { (output) = ((input) > (deadband) || (input) < -(deadband)) ? (input) : 0; } while (0)
void gimbal_motor_raw_angle_control(gimbal_motor_t *motor) { (void)motor; }
void gimbal_motor_absolute_angle_control(gimbal_motor_t *motor) { (void)motor; }
void gimbal_motor_relative_angle_control(gimbal_motor_t *motor)
{ ++yaw_relative_calls; motor->given_current = 37; motor->current_set = 37.0f; }
void gimbal_PID_clear(int *pid) { (void)pid; ++yaw_pid_clears; }
void PID_clear(int *pid) { (void)pid; ++yaw_pid_clears; }
uint32_t HAL_GetTick(void) { return fake_tick; }
void gimbal_behaviour_control_set(fp32 *yaw, fp32 *pitch, gimbal_control_t *control)
{ (void)control; *yaw = 0.0f; *pitch = 0.0f; }
void gimbal_absolute_angle_limit(gimbal_motor_t *motor, fp32 add)
{ motor->absolute_angle_set += add; }
static float pitch_dm_raw_to_relative(float raw)
{

    return PITCH_DM_ZERO_OFFSET_RAD - raw;

    return raw - PITCH_DM_ZERO_OFFSET_RAD;

}
void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
                    
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
                                                                              
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        pitch_dm_enable_status.mit_target_valid = 0U;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;

}
void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
	static fp32 bias_angle;
	if (gimbal_motor == NULL)
	{
		return;
	}

	        
	if(gimbal_motor == &gimbal_control.gimbal_yaw_motor)
	{
		bias_angle = aim.receive.yaw;
		aim.receive.yaw = 0.0f;
	}
	else if(gimbal_motor == &gimbal_control.gimbal_pitch_motor)
	{
		bias_angle = aim.receive.pitch;
		aim.receive.pitch = 0.0f;
	}

	gimbal_motor->relative_angle_set = gimbal_motor->relative_angle_set + add + bias_angle;
	gimbal_motor->relative_angle_set = rad_format(gimbal_motor->relative_angle_set);
	             
	if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
	{
		gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
	}
	else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
	{
		gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
	}
}
void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;
	
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);
		
		*yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;   
}
void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
	*yaw = 0.0f;
	*pitch = 0.0f;                                                             
}
void gimbal_control_loop(gimbal_control_t *control_loop)
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
void gimbal_set_control(gimbal_control_t *set_control)
{
	if (set_control == NULL)
	{
		return;
	}

	static fp32 add_yaw_angle = 0.0f;
	static fp32 add_pitch_angle = 0.0f;
	
	       
	gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
	if (gimbal_behaviour != GIMBAL_INIT)
	{
		yaw_init_hold_valid = 0U;
	}
	
	           
	if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
	{
		                
		set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
	}
	else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		                 
		gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
	}
	else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
		if (gimbal_behaviour == GIMBAL_INIT)
		{
			                                                                
			                                                             
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
			                                                              
			gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
		}
	}

	             
	if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
	{
		                 
		gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
	}
	else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
		                     
		gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
	}
}
volatile struct { uint32_t status, checks, failure; } test_result;
__asm void stop_test(void) {
    BKPT 0
    BX lr
}
void check(int ok, uint32_t code) {
    ++test_result.checks;
    if (!ok) { test_result.failure = code; stop_test(); }
}
int near(fp32 actual, fp32 expected) {
    return actual > expected - 0.0001f && actual < expected + 0.0001f;
}
void test_entry(void) {
    gimbal_motor_t *pitch = &gimbal_control.gimbal_pitch_motor;
    gimbal_motor_t *yaw = &gimbal_control.gimbal_yaw_motor;
    fp32 add_yaw = 0.0f, add_pitch = 0.0f;
    gimbal_control.gimbal_rc_ctrl = &rc_ctrl;
    pitch->last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    pitch->gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    pitch->relative_angle = -2.45f; /* current DM feedback */
    pitch->emit_pos_set = -2.25f; /* command from RAW mode */
    pitch->min_relative_angle = -2.7f;
    pitch->max_relative_angle = -2.3f;
    yaw->last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    yaw->gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    yaw->relative_angle = 0.2f;
    gimbal_mode_change_control_transit(&gimbal_control);
    check(near(pitch->relative_angle_set, -2.45f), 1);
    check(near(yaw->relative_angle_set, 0.2f), 2);
    gimbal_relative_angle_limit(pitch, -0.00396f);
    check(near(pitch->relative_angle_set, -2.45396f), 3);
    gimbal_relative_angle_limit(pitch, +0.00396f);
    check(near(pitch->relative_angle_set, -2.45f), 4);
    for (int i = 0; i < 100; ++i) gimbal_relative_angle_limit(pitch, -0.00396f);
    check(near(pitch->relative_angle_set, -2.7f), 5);
    for (int i = 0; i < 200; ++i) gimbal_relative_angle_limit(pitch, +0.00396f);
    check(near(pitch->relative_angle_set, -2.3f), 6);
    pitch->relative_angle_set = -2.45f;
    rc_ctrl.rc.ch[PITCH_CHANNEL] = 660;
    gimbal_relative_angle_control(&add_yaw, &add_pitch, &gimbal_control);
    check(near(add_pitch, 660.0f * PITCH_RC_SEN), 7);
    gimbal_relative_angle_limit(pitch, add_pitch);
    gimbal_control_loop(&gimbal_control);
    check(near(pitch->emit_pos_set, -2.45f + 660.0f * PITCH_RC_SEN), 8);
    rc_ctrl.rc.ch[PITCH_CHANNEL] = -660;
    gimbal_relative_angle_control(&add_yaw, &add_pitch, &gimbal_control);
    check(near(add_pitch, -660.0f * PITCH_RC_SEN), 9);
    gimbal_relative_angle_limit(pitch, add_pitch);
    gimbal_control_loop(&gimbal_control);
    check(near(pitch->emit_pos_set, -2.45f), 10);
    yaw->relative_angle = 2.0f;
    pitch->absolute_angle = 0.5f;
    INS.ins_flag = 1;
    add_yaw = 42.0f; add_pitch = 42.0f;
    gimbal_init_control(&add_yaw, &add_pitch, &gimbal_control);
    check(near(add_yaw, 0.0f) && near(add_pitch, 0.0f), 11);
    INS.ins_flag = 0;
    add_yaw = 42.0f; add_pitch = 42.0f;
    gimbal_init_control(&add_yaw, &add_pitch, &gimbal_control);
    check(near(add_yaw, 0.0f) && near(add_pitch, 0.0f), 12);
    gimbal_control.yaw_dji_measure = &yaw_feedback;
    gimbal_behaviour = GIMBAL_INIT;
    yaw->gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    pitch->gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    yaw->relative_angle = 1.0f;
    yaw->relative_angle_set = 0.0f;
    yaw->min_relative_angle = -0.1f;
    yaw->max_relative_angle = 0.1f;
    fake_tick = 1000;
    yaw_feedback.last_fdb_time = 0;
    gimbal_set_control(&gimbal_control);
    gimbal_control_loop(&gimbal_control);
    check(yaw_init_hold_valid == 0 && yaw->given_current == 0, 13);
    yaw_feedback.last_fdb_time = 1000;
    gimbal_set_control(&gimbal_control);
    gimbal_control_loop(&gimbal_control);
    check(yaw_init_hold_valid == 1 && near(yaw->relative_angle_set, 1.0f) &&
          yaw->given_current == 37 && yaw_pid_clears == 2, 14);
    yaw->relative_angle = 1.1f;
    gimbal_set_control(&gimbal_control);
    check(near(yaw->relative_angle_set, 1.0f), 15);
    fake_tick = 1200;
    gimbal_set_control(&gimbal_control);
    gimbal_control_loop(&gimbal_control);
    check(yaw_init_hold_valid == 0 && yaw->given_current == 0, 16);
    fake_tick = 1201;
    yaw_feedback.last_fdb_time = 1201;
    yaw->relative_angle = 1.3f;
    gimbal_set_control(&gimbal_control);
    check(yaw_init_hold_valid == 1 && near(yaw->relative_angle_set, 1.3f), 17);
    gimbal_behaviour = GIMBAL_ZERO_FORCE;
    yaw->gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_set_control(&gimbal_control);
    check(yaw_init_hold_valid == 0, 18);
    test_result.status = 0x600D;
    stop_test();
}
