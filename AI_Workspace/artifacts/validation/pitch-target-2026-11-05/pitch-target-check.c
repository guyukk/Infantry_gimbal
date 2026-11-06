
#include <stdint.h>
#include <stddef.h>
typedef float fp32;
enum { GIMBAL_MOTOR_RAW = 0, GIMBAL_MOTOR_GYRO = 1, GIMBAL_MOTOR_ENCONDE = 2 };
typedef struct {
    int last_gimbal_motor_mode, gimbal_motor_mode, given_current;
    fp32 raw_cmd_current, current_set;
    fp32 absolute_angle_set, absolute_angle;
    fp32 relative_angle_set, relative_angle;
    fp32 motor_gyro_set, motor_gyro;
    fp32 min_relative_angle, max_relative_angle, emit_pos_set;
} gimbal_motor_t;
typedef struct {
    gimbal_motor_t gimbal_yaw_motor, gimbal_pitch_motor;
} gimbal_control_t;
gimbal_control_t gimbal_control;
struct { struct { fp32 yaw, pitch; } receive; } aim;
#define rad_format(x) (x)
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
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
                                                                                  
                                                                             
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.emit_pos_set;
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
    pitch->last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    pitch->gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    pitch->relative_angle = 0.0f; /* current unconfigured DM feedback */
    pitch->emit_pos_set = -2.25f; /* command from RAW mode */
    pitch->min_relative_angle = -2.5f;
    pitch->max_relative_angle = -2.0f;
    yaw->last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    yaw->gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    yaw->relative_angle = 0.2f;
    gimbal_mode_change_control_transit(&gimbal_control);
    check(near(pitch->relative_angle_set, -2.25f), 1);
    check(near(yaw->relative_angle_set, 0.2f), 2);
    gimbal_relative_angle_limit(pitch, -0.00396f);
    check(near(pitch->relative_angle_set, -2.25396f), 3);
    gimbal_relative_angle_limit(pitch, +0.00396f);
    check(near(pitch->relative_angle_set, -2.25f), 4);
    for (int i = 0; i < 100; ++i) gimbal_relative_angle_limit(pitch, -0.00396f);
    check(near(pitch->relative_angle_set, -2.5f), 5);
    for (int i = 0; i < 200; ++i) gimbal_relative_angle_limit(pitch, +0.00396f);
    check(near(pitch->relative_angle_set, -2.0f), 6);
    test_result.status = 0x600D;
    stop_test();
}
