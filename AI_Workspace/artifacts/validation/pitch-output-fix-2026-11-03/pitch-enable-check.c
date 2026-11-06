
#include <stdint.h>
#include <stddef.h>
#define debug 0
#define PITCH_DM_ENABLE_FIRST_RETRY_MS 2000U
#define PITCH_DM_ENABLE_RETRY_MS 500U
#define PITCH_DM_ENABLE_MAX_ATTEMPTS 8U
#define PITCH_DM_CAN_ID 0x53U
#define GIMBAL_CAN fake_can
#define YAW_CURRENT_SET_POLARITY 1
#define PITCH_CURRENT_SET_POLARITY 1
#define YAW_GIMBAL_MOTOR_TOE 0
#define PITCH_GIMBAL_MOTOR_TOE 1
#define TRIGGER_MOTOR_TOE 2
#define DBUS_TOE 3
typedef struct { struct { uint32_t last_fdb_time; int state; } fdb; } dm_measure_t;
typedef struct {
    struct { int given_current; } gimbal_yaw_motor;
    struct {
        dm_measure_t *gimbal_DM_measure;
        int given_current;
        float emit_pos_set, emit_vel_limit, emit_i_limit;
    } gimbal_pitch_motor;
} gimbal_control_t;
typedef struct {
    uint8_t attempts, last_tx_status, confirmed, exhausted;
    uint32_t last_attempt_tick, confirmed_feedback_tick;
    uint32_t emit_send_count, last_emit_tick;
    uint8_t last_emit_tx_status;
} pitch_dm_enable_status_t;
volatile pitch_dm_enable_status_t pitch_dm_enable_status;
uint32_t fake_now, send_count;
uint8_t fake_tx_status;
uint16_t sent_id;
int fake_can;
int16_t yaw_can_set_current, pitch_can_set_current, shoot_can_set_current;
int8_t yaw_cuurent_set_polarity = YAW_CURRENT_SET_POLARITY;
int8_t pitch_cuurent_set_polarity = PITCH_CURRENT_SET_POLARITY;
uint32_t toe_flags, emit_count;
uint8_t fake_emit_status;
uint16_t emit_id;
float emit_pos;
uint32_t HAL_GetTick(void) { return fake_now; }
int toe_is_error(uint8_t toe) { return (toe_flags >> toe) & 1U; }
void CAN_cmd_gimbal(int a, int b, int c, int d) { (void)a; (void)b; (void)c; (void)d; }
uint8_t CAN_cmd_EMIT(void *can, uint16_t id, float pos, float vel, float current) {
    (void)vel; (void)current;
    if (can != &fake_can) return 255;
    ++emit_count;
    emit_id = id;
    emit_pos = pos;
    return fake_emit_status;
}
uint8_t Motor_enable(void *can, uint16_t id) {
    if (can != &fake_can) return 255;
    sent_id = id;
    ++send_count;
    return fake_tx_status;
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
void gimbal_send_cmd(gimbal_control_t *control_send)
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
				                                              
				CAN_cmd_gimbal(yaw_can_set_current, 0, shoot_can_set_current, 0);
				                                                                          
				                                                                                
				pitch_dm_enable_status.last_emit_tx_status = CAN_cmd_EMIT(&GIMBAL_CAN, PITCH_DM_CAN_ID,
									 control_send->gimbal_pitch_motor.emit_pos_set,
									 control_send->gimbal_pitch_motor.emit_vel_limit,
									 control_send->gimbal_pitch_motor.emit_i_limit);
				pitch_dm_enable_status.emit_send_count++;
				pitch_dm_enable_status.last_emit_tick = HAL_GetTick();
			}
		}
	}
	else
	{
		CAN_cmd_gimbal(0, 0, 0, 0);
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
void reset_state(void) {
    pitch_dm_enable_status.attempts = 1;
    pitch_dm_enable_status.last_tx_status = 0;
    pitch_dm_enable_status.confirmed = 0;
    pitch_dm_enable_status.exhausted = 0;
    pitch_dm_enable_status.last_attempt_tick = 100;
    pitch_dm_enable_status.confirmed_feedback_tick = 0;
    send_count = 0;
    fake_tx_status = 0;
    sent_id = 0;
    emit_count = 0;
    toe_flags = 0;
    fake_emit_status = 0;
    pitch_dm_enable_status.emit_send_count = 0;
    pitch_dm_enable_status.last_emit_tick = 0;
    pitch_dm_enable_status.last_emit_tx_status = 255;
}
void test_entry(void) {
    dm_measure_t measure = {{0, 0}};
    reset_state();
    fake_now = 2099;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 0, 1);
    fake_now = 2100;
    fake_tx_status = 2;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 1, 2);
    check(pitch_dm_enable_status.attempts == 2 &&
          pitch_dm_enable_status.last_tx_status == 2 && sent_id == 0x53, 3);
    measure.fdb.last_fdb_time = 2099;
    measure.fdb.state = 1;
    fake_now = 2101;
    check(pitch_dm_enable_ready(&measure) == 0 &&
          pitch_dm_enable_status.confirmed == 0, 4);
    measure.fdb.last_fdb_time = 2101;
    check(pitch_dm_enable_ready(&measure) == 1 &&
          pitch_dm_enable_status.confirmed_feedback_tick == 2101, 5);
    fake_now = 3000;
    check(pitch_dm_enable_ready(&measure) == 1 && send_count == 1, 6);
    reset_state();
    measure.fdb.last_fdb_time = 2101;
    measure.fdb.state = 0;
    fake_now = 599;
    pitch_dm_enable_status.attempts = 7;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 0, 7);
    fake_now = 600;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 1 &&
          pitch_dm_enable_status.attempts == 8, 8);
    fake_now = 1100;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 1 &&
          pitch_dm_enable_status.exhausted == 1, 9);
    check(pitch_dm_enable_ready(NULL) == 0 && send_count == 1, 10);
    reset_state();
    measure.fdb.last_fdb_time = 0;
    measure.fdb.state = 0;
    gimbal_control_t control = {0};
    control.gimbal_pitch_motor.gimbal_DM_measure = &measure;
    control.gimbal_pitch_motor.emit_pos_set = -2.25f;
    control.gimbal_pitch_motor.emit_vel_limit = 30.0f;
    control.gimbal_pitch_motor.emit_i_limit = 0.8f;
    fake_now = 2100;
    fake_emit_status = 2;
    gimbal_send_cmd(&control);
    check(emit_count == 1 && pitch_dm_enable_status.confirmed == 0 &&
          pitch_dm_enable_status.emit_send_count == 1, 11);
    check(emit_id == 0x53 && emit_pos == -2.25f &&
          pitch_dm_enable_status.last_emit_tx_status == 2 &&
          pitch_dm_enable_status.last_emit_tick == 2100, 12);
    toe_flags = 1U << DBUS_TOE;
    fake_now = 2101;
    gimbal_send_cmd(&control);
    check(emit_count == 1 && pitch_dm_enable_status.emit_send_count == 1, 13);
    toe_flags = 0;
    measure.fdb.last_fdb_time = 2102;
    measure.fdb.state = 1;
    fake_now = 2102;
    gimbal_send_cmd(&control);
    check(pitch_dm_enable_status.confirmed == 1 && emit_count == 2, 14);
    test_result.status = 0x600D;
    stop_test();
}
