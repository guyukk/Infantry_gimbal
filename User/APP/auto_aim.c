#include "health.h"
#include "arm_math.h"
#include "auto_aim.h"
#include "bsp_usart.h"
#include "bsp_vofa+.h"
#include "cmsis_os.h"
#include "CRC8_CRC16.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "main.h"
#include "referee.h"
#include "shoot.h"
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "struct_typedef.h"
#include "tim.h"
#include "uproto.h"
#include "usart.h"
#include "usbd_cdc_if.h"


extern uproto_context_t proto_ctx;
// [SYNC_FROM_H] Synced auto-aim control logic from H:\DM-balanceV1\User\APP

// global auto-aim state
auto_aim_t aim;

fp32 yaw = 0.0f;
fp32 CompensationAngle = 0.0f;

// ---------------- Auto-aim internal control state (MCU-side trajectory shaping) ----------------
// [SYNC_FROM_H] New trajectory shaping state replaces direct motor writes
typedef struct
{
    float err_rad;        // latest vision error (rad)
    float omega_cmd_rad;  // commanded angular velocity (rad/s)
} auto_aim_axis_ctrl_t;

typedef struct
{
    auto_aim_axis_ctrl_t yaw_axis;
    auto_aim_axis_ctrl_t pitch_axis;
    uint32_t             last_tick_ms;
} auto_aim_ctrl_t;

typedef struct {
    fp32 buf[5];     /* 环形缓冲区 */
    uint8_t idx;        /* 写指针 0~4 */
    fp32 last_out;   /* 上次有效输出 */
    uint8_t zero_hold_cnt;
} MedianFilter5;
		MedianFilter5 yawfilter = {0}; 
		MedianFilter5 pitchfilter = {0};

#define SWAP_F(a, b) do{ float _t=(a); (a)=(b); (b)=_t; }while(0)
#define MF5_ZERO_HOLD_MAX_CNT (2U)

static void mf5_reset(MedianFilter5 *f)
{
    if (f == NULL)
    {
        return;
    }

    for (uint8_t i = 0U; i < 5U; i++)
    {
        f->buf[i] = 0.0f;
    }
    f->idx = 0U;
    f->last_out = 0.0f;
    f->zero_hold_cnt = 0U;
}

float mf5_update(MedianFilter5 *f, float new_val)
{
    if (f == NULL)
    {
        return new_val;
    }

    /* Zero is valid data, including after a nonzero measurement. */
    /* 2. 写环形缓冲区 */
    f->buf[f->idx] = new_val;
    f->idx = (f->idx + 1) % 5;

    /* 3. 复制到局部变量，排序网络（5 元素 9 次比较） */
    float a = f->buf[0];
    float b = f->buf[1];
    float c = f->buf[2];
    float d = f->buf[3];
    float e = f->buf[4];

    if (a > b) SWAP_F(a, b);
    if (d > e) SWAP_F(d, e);
    if (a > c) SWAP_F(a, c);
    if (b > c) SWAP_F(b, c);
    if (a > d) SWAP_F(a, d);
    if (c > d) SWAP_F(c, d);
    if (b > e) SWAP_F(b, e);
    if (b > c) SWAP_F(b, c);
    if (d > e) SWAP_F(d, e);

    /* 此时 c 为中值 */
    f->last_out = c;
    return c;
}

static auto_aim_ctrl_t s_auto_aim_ctrl = {0};

//自瞄增益
// basic tuning (can be adjusted during testing)
// Kv: position error -> target angular velocity (rad/s per rad)

// @note: 参数挪到兵种定义头文件中

// #define AA_KV_YAW          (4.0f)
// #define AA_KV_PITCH        (-4.0f)
// Kd: derivative gain on error，用于在高 Kv 下增加阻尼
// #define AA_KD_YAW          (0.05f)
// #define AA_KD_PITCH        (0.05f)
// 非线性区间：误差小于该角度时自动降低 Kv、加大阻尼
// #define AA_ERR_SOFT_ZONE_RAD   (5.0f * PI / 180.0f)       // ~5 deg
// D 项一阶低通滤波系数（0~1，越小越平滑）
// #define AA_D_LPF_ALPHA     (0.4f)

// 最大转动速度
// #define AA_OMEGA_MAX_YAW   (180.0f * PI / 180.0f)         // 18 deg/s
// #define AA_OMEGA_MAX_PITCH (180.0f * PI / 180.0f)

// #define AA_A_MAX_YAW       (720.0f * PI / 180.0f)         // 720 deg/s^2
// #define AA_A_MAX_PITCH     (720.0f * PI / 180.0f)

// #define AA_EPS_TH_RAD      (0.5f * PI / 180.0f)           // ~0.5 deg
// #define AA_EPS_OMEGA_RAD   (3.0f * PI / 180.0f)           // ~3 deg/s

// internal derivative state for PD-like shaping
static float s_last_err_yaw = 0.0f;
static float s_last_err_pitch = 0.0f;
static float s_err_d_yaw = 0.0f;
static float s_err_d_pitch = 0.0f;

typedef struct {
    float yaw_error, pitch_error;
    auto_aim_rx_status_t status;
} auto_aim_input_t;
static auto_aim_input_t s_auto_aim_rx;
static uint32_t s_required_generation;
static uint8_t s_last_r;
static uint32_t s_health_faults[4];
static const uint8_t aim_health_sources[4]={HEALTH_RC,HEALTH_IMU,HEALTH_YAW,HEALTH_PITCH};
static void auto_aim_clear_history(auto_aim_t *state);
static uint8_t auto_aim_health_changed(uint32_t now)
{
    uint8_t i,changed=0U;
    health_status_t source;
    for (i=0U;i<4U;i++)
    {
        health_snapshot(aim_health_sources[i],now,&source);
        if (s_health_faults[i] != source.fault_generation) changed=1U;
        s_health_faults[i]=source.fault_generation;
    }
    if (changed) {aim.auto_aim_flag=AIM_OFF;auto_aim_clear_history(&aim);}
    return changed;
}

static uint8_t auto_aim_local_ready(uint32_t now)
{
    if (auto_aim_health_changed(now)) return 0U;
    if (gimbal_behaviour != GIMBAL_ABSOLUTE_ANGLE && gimbal_behaviour != GIMBAL_RELATIVE_ANGLE &&
        gimbal_behaviour != GIMBAL_SPIN) return 0U;
    return health_fresh(HEALTH_IMU,now) &&
        health_gate_ready(HEALTH_GATE_YAW,HEALTH_YAW_MASK|health_gate_mask(HEALTH_GATE_YAW),now) &&
        health_gate_ready(HEALTH_GATE_PITCH,HEALTH_PITCH_MASK|health_gate_mask(HEALTH_GATE_PITCH),now);
}
static uint8_t auto_aim_command_ready(uint32_t now)
{
    return aim.auto_aim_flag == AIM_ON && s_auto_aim_rx.status.seen &&
        s_auto_aim_rx.status.generation != s_required_generation &&
        (uint32_t)(now-s_auto_aim_rx.status.last_rx_ms) <= AUTO_AIM_TIMEOUT && auto_aim_local_ready(now);
}
static void auto_aim_clear_history(auto_aim_t *state)
{
    memset(&s_auto_aim_ctrl,0,sizeof(s_auto_aim_ctrl));
    s_last_err_yaw=s_last_err_pitch=s_err_d_yaw=s_err_d_pitch=0.0f;
    state->receive.yaw=state->receive.pitch=0.0f;
    state->online=0U;
    mf5_reset(&yawfilter);mf5_reset(&pitchfilter);
    s_required_generation=s_auto_aim_rx.status.generation;
}
void auto_aim_cancel(void)
{
    uint32_t key=__get_PRIMASK();__disable_irq();
    aim.auto_aim_flag=AIM_OFF;auto_aim_clear_history(&aim);__set_PRIMASK(key);
}
uint8_t auto_aim_is_ready(void)
{
    uint32_t key=__get_PRIMASK();uint8_t ready;
    __disable_irq();ready=auto_aim_command_ready(HAL_GetTick());__set_PRIMASK(key);return ready;
}
void auto_aim_rx_status(auto_aim_rx_status_t *out)
{
    uint32_t key;
    if (out == NULL) return;
    key=__get_PRIMASK();__disable_irq();*out=s_auto_aim_rx.status;__set_PRIMASK(key);
}
fp32 auto_aim_take_increment(uint8_t pitch)
{
    uint32_t key=__get_PRIMASK(),now=HAL_GetTick();
    fp32 value=0.0f;
    __disable_irq();
    if (!auto_aim_command_ready(now))
    {
        if (!auto_aim_local_ready(now) || (s_auto_aim_rx.status.seen &&
            (uint32_t)(now-s_auto_aim_rx.status.last_rx_ms)>AUTO_AIM_TIMEOUT)) aim.auto_aim_flag=AIM_OFF;
        auto_aim_clear_history(&aim);
    }
    else if (pitch) {value=aim.receive.pitch;aim.receive.pitch=0.0f;}
    else {value=aim.receive.yaw;aim.receive.yaw=0.0f;}
    __set_PRIMASK(key);return value;
}

static inline float aa_clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


static void auto_aim_control_tick_internal(auto_aim_t *aim_loop);


// Legacy example function (kept for compatibility, currently unused in main loop)
void auto_aim_loop(auto_aim_t* aim_loop)
{
    (void)aim_loop;
}

void auto_aim_init(auto_aim_t *aim_init)
{
    uint32_t key;
    if (aim_init == NULL) return;
    key=__get_PRIMASK();__disable_irq();
    memset(aim_init,0,sizeof(*aim_init));
    aim_init->aim_rc=get_remote_control_point();
    aim_init->last_fdb=s_auto_aim_rx.status.last_rx_ms;
    {
        uint8_t i;health_status_t source;
        for (i=0U;i<4U;i++) {health_snapshot(aim_health_sources[i],HAL_GetTick(),&source);s_health_faults[i]=source.fault_generation;}
    }
    s_last_r=0U;auto_aim_clear_history(aim_init);
    __set_PRIMASK(key);
}

void auto_aim_set(auto_aim_t *aim_set)
{
    static RC_ctrl_t local_rc;
    uint32_t key,now=HAL_GetTick();
    uint8_t pressed;
    if (aim_set == NULL) return;
    key=__get_PRIMASK();__disable_irq();
    if (!rc_ctrl_snapshot(&local_rc,NULL)) {aim_set->auto_aim_flag=AIM_OFF;auto_aim_clear_history(aim_set);__set_PRIMASK(key);return;}
    aim_set->aim_rc=&local_rc;
    pressed=!!(local_rc.key.v & KEY_PRESSED_OFFSET_R);
    if (!auto_aim_local_ready(now))
    {
        aim_set->auto_aim_flag=AIM_OFF;s_last_r=pressed;auto_aim_clear_history(aim_set);
        __set_PRIMASK(key);return;
    }
    if (pressed && !s_last_r)
    {
        aim_set->auto_aim_flag=aim_set->auto_aim_flag == AIM_OFF ? AIM_ON : AIM_OFF;
        auto_aim_clear_history(aim_set); /* A new enable needs a new DELTA. */
    }
    s_last_r=pressed;
    if (aim_set->auto_aim_flag == AIM_OFF || (s_auto_aim_rx.status.seen &&
        (uint32_t)(now-s_auto_aim_rx.status.last_rx_ms)>AUTO_AIM_TIMEOUT))
    {
        aim_set->auto_aim_flag=AIM_OFF;auto_aim_clear_history(aim_set);
    }
    else aim_set->online=auto_aim_command_ready(now);
    __set_PRIMASK(key);
}

void auto_aim_feedback_update(auto_aim_t *aim_update)
{
    uint32_t key;
    if (aim_update == NULL) return;
    key=__get_PRIMASK();__disable_irq();
    aim_update->online=auto_aim_command_ready(HAL_GetTick());
    /* Protocol tick is owned exclusively by COMM_APP. Never synthesize receive time. */
    __set_PRIMASK(key);
}

void auto_aim_task(void const *pvParameters)
{
    (void)pvParameters;
		fp32 debug_vofa[10] = {0.0f, 0.0f};
    osDelay(AIM_INIT_TIME);
    auto_aim_init(&aim);
		
    while (1)
    {
        // [SYNC_FROM_H] Task loop now drives internal control tick each cycle
        auto_aim_set(&aim);
        auto_aim_feedback_update(&aim);
        auto_aim_control_tick(&aim);
				debug_vofa[0] = aim.receive.pitch;
				debug_vofa[1] = gimbal_control.gimbal_pitch_motor.relative_angle_set;
				debug_vofa[2] = gimbal_control.gimbal_pitch_motor.relative_angle;
				debug_vofa[3] = gimbal_control.gimbal_pitch_motor.emit_pos_set;
				debug_vofa[4] = aim.receive.yaw;
				debug_vofa[5] = gimbal_control.gimbal_yaw_motor.absolute_angle_set;
				debug_vofa[6] = gimbal_control.gimbal_yaw_motor.absolute_angle;
				debug_vofa[7] =aim.yaw_delay   ;
				debug_vofa[8] =aim.pitch_delay ;
				debug_vofa[9] =aim.shoot_delay ;
				Vofa_JustFloat_AUTO(debug_vofa);
			
			
        osDelay(AUTO_AIM_TIME);
    }
}



void auto_aim_apply_delta_udeg(int32_t dyaw_udeg,
                               int32_t dpitch_udeg,
                               uint16_t status,
                               uint64_t ts_us)
{
    uint32_t key;
    float yaw_error=((float)dyaw_udeg)*PI/180000000.0f;
    float pitch_error=((float)dpitch_udeg)*PI/180000000.0f;
    if (!isfinite(yaw_error) || !isfinite(pitch_error)) return;
    key=__get_PRIMASK();__disable_irq();
    s_auto_aim_rx.yaw_error=yaw_error;s_auto_aim_rx.pitch_error=pitch_error;
    s_auto_aim_rx.status.last_rx_ms=HAL_GetTick();
    s_auto_aim_rx.status.generation++;
    s_auto_aim_rx.status.seen=1U;
    s_auto_aim_rx.status.status=status;s_auto_aim_rx.status.host_ts_us=ts_us;
    aim.last_fdb=s_auto_aim_rx.status.last_rx_ms;
    __set_PRIMASK(key);
}

void auto_aim_control_tick(auto_aim_t *aim_loop)
{
    uint32_t key=__get_PRIMASK();
    __disable_irq();auto_aim_control_tick_internal(aim_loop);__set_PRIMASK(key);
}

// Core MCU-side control: error -> limited velocity -> per-cycle angle increment (rad)
// [SYNC_FROM_H] New PD-like shaping with slew limits for smooth motion
static void auto_aim_control_tick_internal(auto_aim_t *aim_loop)
{
    if (aim_loop == NULL)
    {
        return;
    }

    uint32_t now_ms = HAL_GetTick();
    if (!auto_aim_command_ready(now_ms))
    {
        if (!auto_aim_local_ready(now_ms) || (s_auto_aim_rx.status.seen &&
            (uint32_t)(now_ms-s_auto_aim_rx.status.last_rx_ms)>AUTO_AIM_TIMEOUT)) aim_loop->auto_aim_flag=AIM_OFF;
        auto_aim_clear_history(aim_loop);return;
    }
    aim_loop->online=1U;
    s_auto_aim_ctrl.yaw_axis.err_rad=s_auto_aim_rx.yaw_error;
    s_auto_aim_ctrl.pitch_axis.err_rad=s_auto_aim_rx.pitch_error;
    float dt_s;
    if (s_auto_aim_ctrl.last_tick_ms == 0U)
    {
        dt_s = (float)AUTO_AIM_TIME / 1000.0f;
    }
    else
    {
        uint32_t diff_ms = now_ms - s_auto_aim_ctrl.last_tick_ms;
        dt_s = (float)diff_ms / 1000.0f;
    }
    s_auto_aim_ctrl.last_tick_ms = now_ms;
    dt_s = aa_clamp(dt_s, 0.0005f, 0.02f); // 0.5–20 ms safety clamp

    // Yaw axis: error -> omega_ref -> omega_cmd with accel limit -> angle increment
    {
        float err = s_auto_aim_ctrl.yaw_axis.err_rad;
        float e_abs = fabsf(err);

        // 非线性 Kv：远处用大增益，接近目标自动降低增益以减小冲击与振荡
        float kv = AA_KV_YAW;
        if (e_abs < AA_ERR_SOFT_ZONE_RAD)
        {
            float scale = e_abs / (AA_ERR_SOFT_ZONE_RAD + 1e-6f); // 0~1
            // Kv 在 [0.5, 1.0] * AA_KV_YAW 之间线性变化
            kv *= (0.5f + 0.5f * scale);
        }

        // 误差导数（简单 D），并做一阶低通
        float derr = (err - s_last_err_yaw) / dt_s;
        s_last_err_yaw = err;
        s_err_d_yaw = AA_D_LPF_ALPHA * derr + (1.0f - AA_D_LPF_ALPHA) * s_err_d_yaw;

        float omega_ref = kv * err + AA_KD_YAW * s_err_d_yaw;
        omega_ref = aa_clamp(omega_ref, -AA_OMEGA_MAX_YAW, AA_OMEGA_MAX_YAW);

        // 误差越小，加速度上限越小 → 目标附近更平滑
        float a_max = AA_A_MAX_YAW;
        const float soft_a_err = 10.0f * PI / 180.0f; // ~10deg
        if (e_abs < soft_a_err)
        {
            float scale = e_abs / (soft_a_err + 1e-3f); // 0~1
            a_max *= (0.3f + 0.7f * scale);             // 保留至少 30% 的加速度能力
        }
        float domega_max = a_max * dt_s;
        float domega = omega_ref - s_auto_aim_ctrl.yaw_axis.omega_cmd_rad;
        domega = aa_clamp(domega, -domega_max, domega_max);
        s_auto_aim_ctrl.yaw_axis.omega_cmd_rad += domega;

        if ((fabsf(err) < AA_EPS_TH_RAD) &&
            (fabsf(s_auto_aim_ctrl.yaw_axis.omega_cmd_rad) < AA_EPS_OMEGA_RAD))
        {
            s_auto_aim_ctrl.yaw_axis.omega_cmd_rad = 0.0f;
        }

        float dtheta = s_auto_aim_ctrl.yaw_axis.omega_cmd_rad * dt_s;
        aim.receive.yaw = dtheta;
    }

    // Pitch axis
    {
        float err = s_auto_aim_ctrl.pitch_axis.err_rad;
        float e_abs = fabsf(err);

        float kv = AA_KV_PITCH;
        if (e_abs < AA_ERR_SOFT_ZONE_RAD)
        {
            float scale = e_abs / (AA_ERR_SOFT_ZONE_RAD + 1e-6f);
            kv *= (0.5f + 0.5f * scale);
        }

        float derr = (err - s_last_err_pitch) / dt_s;
        s_last_err_pitch = err;
        s_err_d_pitch = AA_D_LPF_ALPHA * derr + (1.0f - AA_D_LPF_ALPHA) * s_err_d_pitch;

        float omega_ref = kv * err + AA_KD_PITCH * s_err_d_pitch;
        omega_ref = aa_clamp(omega_ref, -AA_OMEGA_MAX_PITCH, AA_OMEGA_MAX_PITCH);

        float a_max = AA_A_MAX_PITCH;
        const float soft_a_err = 10.0f * PI / 180.0f;
        if (e_abs < soft_a_err)
        {
            float scale = e_abs / (soft_a_err + 1e-3f);
            a_max *= (0.3f + 0.7f * scale);
        }
        float domega_max = a_max * dt_s;
        float domega = omega_ref - s_auto_aim_ctrl.pitch_axis.omega_cmd_rad;
        domega = aa_clamp(domega, -domega_max, domega_max);
        s_auto_aim_ctrl.pitch_axis.omega_cmd_rad += domega;

        if ((fabsf(err) < AA_EPS_TH_RAD) &&
            (fabsf(s_auto_aim_ctrl.pitch_axis.omega_cmd_rad) < AA_EPS_OMEGA_RAD))
        {
            s_auto_aim_ctrl.pitch_axis.omega_cmd_rad = 0.0f;
        }

        float dtheta = s_auto_aim_ctrl.pitch_axis.omega_cmd_rad * dt_s;
        aim.receive.pitch = dtheta;
    }
		aim.receive.yaw   = mf5_update(&yawfilter, aim.receive.yaw);
		aim.receive.pitch = mf5_update(&pitchfilter, aim.receive.pitch);
}
