/* Current H723 calibration; see calibration-manual.md for supported entries and ABI. */
#include "calibrate_task.h"
#include "string.h"
#include "cmsis_os.h"
#include "adc.h"

#include "bsp_flash.h"
#include "bsp_usart.h"
#include "bsp_rc.h"
#include "bsp_buzzer.h"

#include "remote_control.h"
#include "INS_task.h"
#include "gimbal_task.h"
#include "can_bsp.h"
#include "robot_param.h"
#include "yaw_pitch_direct.h"
#include <math.h>
#include <stddef.h>
#include "health.h"


#include "pitch_calibration.h"
static uint8_t pitch_dm_flash_ready;
// Separate 4 KB W25Q sector. The legacy HD/GM record at sector 0 is unchanged.
#define PITCH_DM_CALI_ADDR 0x1000UL
#define PITCH_DM_CALI_MAGIC 0x50444D31UL
#define PITCH_DM_CALI_VERSION 1UL
#define PITCH_DM_CALI_RETRY_MS 1000U
typedef struct
{
    uint32_t magic;
    uint32_t version;
    float min_pos;
    float max_pos;
    uint32_t check;
} pitch_dm_cali_record_t;

typedef char pitch_record_size_check[(sizeof(pitch_dm_cali_record_t)==20) ? 1:-1];
typedef char pitch_record_fields_check[(offsetof(pitch_dm_cali_record_t,min_pos)==8 && offsetof(pitch_dm_cali_record_t,max_pos)==12 && offsetof(pitch_dm_cali_record_t,check)==16) ? 1:-1];

static uint8_t pitch_cali_feedback(dm_measure_t *out)
{
    dm_measure_t *source=get_dm_motor_measure_point(PITCH_DM_MOTOR_INDEX);
    uint8_t ready;
    if(source==NULL) return 0U;
    taskENTER_CRITICAL();
    *out=*source;
    ready=health_fresh(HEALTH_PITCH,HAL_GetTick());
    taskEXIT_CRITICAL();
    return ready && isfinite(out->fdb.pos) && isfinite(out->fdb.vel) &&
        (out->fdb.state==1U || (PITCH_DM_CAN_ID==0x53U && out->fdb.id==3U && out->fdb.state==5U));
}
volatile pitch_dm_cali_status_t pitch_dm_cali_status;

static uint32_t pitch_dm_cali_check(const pitch_dm_cali_record_t *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint8_t bit;
    for (i = 0; i < sizeof(*record) - sizeof(record->check); i++)
    {
        crc ^= bytes[i];
        for (bit = 0; bit < 8U; bit++)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
    }
    return ~crc;
}

static uint8_t pitch_dm_cali_valid_range(float min_pos, float max_pos)
{
    float span = max_pos - min_pos;
    return isfinite(min_pos) && isfinite(max_pos) &&
           min_pos > -PITCH_MIT_P_MAX && max_pos < PITCH_MIT_P_MAX &&
           span >= 0.1f && span <= 1.5f;
}

static uint8_t pitch_dm_cali_read_record(pitch_dm_cali_record_t *record)
{
    if (!pitch_dm_flash_ready)
    {
        pitch_dm_flash_ready = (flash_init() == 0);
    }
    if (!pitch_dm_flash_ready) return 0U;
    if (flash_read(PITCH_DM_CALI_ADDR, (uint32_t *)record, sizeof(*record) / 4U) != 0)
    {
        pitch_dm_flash_ready = 0U;
        return 0U;
    }
    return record->magic == PITCH_DM_CALI_MAGIC &&
           record->version == PITCH_DM_CALI_VERSION &&
           record->check == pitch_dm_cali_check(record) &&
           pitch_dm_cali_valid_range(record->min_pos, record->max_pos);
}

void pitch_dm_cali_load(float *min_pos, float *max_pos)
{
    pitch_dm_cali_record_t record;
    if (min_pos == NULL || max_pos == NULL || !pitch_dm_cali_read_record(&record)) return;
    *min_pos = record.min_pos;
    *max_pos = record.max_pos;
    pitch_dm_cali_status.min_pos = record.min_pos;
    pitch_dm_cali_status.max_pos = record.max_pos;
    pitch_dm_cali_status.stored_valid = 1U;
}

void pitch_dm_cali_recover(void)
{
    static uint32_t last_retry_tick;
    pitch_dm_cali_record_t record;
    dm_measure_t sample;
    dm_measure_t *motor=&sample;
    uint32_t now = HAL_GetTick();

    if (!pitch_dm_cali_status.init_done || (pitch_dm_cali_status.stored_valid && !pitch_dm_cali_status.saved_pending) ||
        pitch_dm_cali_status.request != 0U || pitch_dm_cali_status.captured != 0U ||
        (uint32_t)(now - last_retry_tick) < PITCH_DM_CALI_RETRY_MS) return;
    last_retry_tick = now;
    if (!pitch_dm_cali_read_record(&record)) return;

    if (!pitch_cali_feedback(motor) ||
        motor->fdb.pos < record.min_pos - PITCH_DM_LOWER_RECOVERY_RAD ||
        motor->fdb.pos > record.max_pos) return;

    // Publish limits and the current target together; the gimbal task must not
    // see stored_valid before its position limits have been updated.
    taskENTER_CRITICAL();
    pitch_dm_apply_calibration(&gimbal_control, record.min_pos, record.max_pos,
                               motor->fdb.pos);
    pitch_dm_cali_status.min_pos = record.min_pos;
    pitch_dm_cali_status.max_pos = record.max_pos;
    pitch_dm_cali_status.stored_valid = 1U;
    if (pitch_dm_cali_status.saved_pending) pitch_dm_cali_status.result=2;
    pitch_dm_cali_status.saved_pending = 0U;
    taskEXIT_CRITICAL();
}

void pitch_dm_cali_poll(void)
{
    uint8_t request = pitch_dm_cali_status.request;
    dm_measure_t sample;
    dm_measure_t *motor=&sample;
    float min_pos, max_pos;
    pitch_dm_cali_record_t record;
    pitch_dm_cali_record_t readback;
    if (request == 0U) return;
    pitch_dm_cali_status.request = 0U;
    if (request == 4U)
    {
        pitch_dm_cali_status.captured = 0U;
        pitch_dm_cali_status.result = 0;
        return;
    }
    if (!pitch_cali_feedback(motor))
    {
        pitch_dm_cali_status.result = -1; // stale or unusable feedback
        return;
    }
    if (request == 1U || request == 2U)
    {
        if (!isfinite(motor->fdb.pos) || !isfinite(motor->fdb.vel) ||
            fabsf(motor->fdb.vel) > 0.2f)
        {
            pitch_dm_cali_status.result = -2; // moving or invalid position
            return;
        }
        if (request == 1U)
        {
            pitch_dm_cali_status.first_pos = motor->fdb.pos;
            pitch_dm_cali_status.captured |= 1U;
        }
        else
        {
            pitch_dm_cali_status.second_pos = motor->fdb.pos;
            pitch_dm_cali_status.captured |= 2U;
        }
        pitch_dm_cali_status.result = 1;
        return;
    }
    if (request != 3U || pitch_dm_cali_status.captured != 3U)
    {
        pitch_dm_cali_status.result = -3; // two samples required
        return;
    }
    min_pos = fminf(pitch_dm_cali_status.first_pos, pitch_dm_cali_status.second_pos);
    max_pos = fmaxf(pitch_dm_cali_status.first_pos, pitch_dm_cali_status.second_pos);
    if (!isfinite(motor->fdb.pos) || !isfinite(motor->fdb.vel) || fabsf(motor->fdb.vel)>0.2f ||
        !pitch_dm_cali_valid_range(min_pos, max_pos) ||
        motor->fdb.pos < min_pos || motor->fdb.pos > max_pos)
    {
        pitch_dm_cali_status.result = -4; // unsafe range or current position outside it
        return;
    }
    if (!pitch_dm_flash_ready) pitch_dm_flash_ready = (flash_init() == 0);
    if (!pitch_dm_flash_ready) { pitch_dm_cali_status.result = -5; return; }
    record.magic = PITCH_DM_CALI_MAGIC;
    record.version = PITCH_DM_CALI_VERSION;
    record.min_pos = min_pos;
    record.max_pos = max_pos;
    record.check = pitch_dm_cali_check(&record);
    if (flash_write_single_address(PITCH_DM_CALI_ADDR, (uint32_t *)&record,
                                   sizeof(record) / 4U) != 0)
    {
        pitch_dm_cali_status.result = -5; // flash write failed
        return;
    }
    if (flash_read(PITCH_DM_CALI_ADDR, (uint32_t *)&readback, sizeof(readback) / 4U) != 0 ||
        memcmp(&readback, &record, sizeof(record)) != 0)
    {
        pitch_dm_cali_status.result = -5;
        return;
    }
    /* Flash may block long enough for feedback to expire or the mechanism to move. */
    if (!pitch_cali_feedback(motor) || fabsf(motor->fdb.vel)>0.2f ||
        motor->fdb.pos<min_pos || motor->fdb.pos>max_pos)
    {
        pitch_dm_cali_status.saved_pending=1U;
        pitch_dm_cali_status.captured=0U;
        pitch_dm_cali_status.result=-6; /* persisted, not yet applied; old runtime limits stay */
        return;
    }
    taskENTER_CRITICAL();
    pitch_dm_apply_calibration(&gimbal_control, min_pos, max_pos, motor->fdb.pos);
    pitch_dm_cali_status.min_pos = min_pos;
    pitch_dm_cali_status.max_pos = max_pos;
    pitch_dm_cali_status.stored_valid = 1U;
    pitch_dm_cali_status.saved_pending = 0U;
    taskEXIT_CRITICAL();
    pitch_dm_cali_status.captured = 0U;
    pitch_dm_cali_status.result = 2;
}
void pitch_dm_calibration_set_flash_ready(uint8_t ready) {pitch_dm_flash_ready=ready;}
