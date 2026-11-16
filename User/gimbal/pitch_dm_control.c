#include "health.h"
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


#include "gimbal_control_internal.h"

/* Gimbal task owns enable/MIT history; CAN ISR owns feedback. */
volatile pitch_dm_enable_status_t pitch_dm_enable_status;
static uint8_t pitch_dm_feedback_allows_mit(const dm_measure_t *motor);

float pitch_dm_raw_to_relative(float raw)
{
#if PITCH_TURN
    return PITCH_DM_ZERO_OFFSET_RAD - raw;
#else
    return raw - PITCH_DM_ZERO_OFFSET_RAD;
#endif
}

float pitch_dm_relative_to_raw(float relative)
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

uint8_t pitch_dm_enable_ready(const dm_measure_t *measure)
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
        (!health_fresh(HEALTH_PITCH,now) ||
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
    if (health_fresh(HEALTH_PITCH,now) &&
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
    if (rc == NULL || !health_fresh(HEALTH_RC,now) ||
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

uint8_t pitch_dm_send_mit(gimbal_control_t *control_send)
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
    can_hold = (gimbal_axis_ready(control_send,1U) &&
                health_fresh(HEALTH_PITCH,now) &&
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
