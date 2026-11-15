
#include <stdint.h>
#include <stddef.h>
#define PITCH_DM_CALI_MAGIC 0x50444D31UL
#define PITCH_DM_CALI_VERSION 1UL
#define PITCH_DM_CALI_ADDR 0x1000UL
#define PITCH_DM_CALI_RETRY_MS 1000U
#define PITCH_MIT_P_MAX 12.5f
#define PITCH_DM_LOWER_RECOVERY_RAD 0.02f
#define PITCH_MIT_FEEDBACK_TIMEOUT_MS 100U
#define PITCH_DM_MOTOR_INDEX 2
#define PITCH_DM_CAN_ID 0x53
#define HEALTH_PITCH 4
#define isfinite(x) ((x) == (x))
typedef struct { uint32_t magic, version; float min_pos, max_pos; uint32_t check; }
    pitch_dm_cali_record_t;
typedef struct {
    uint8_t request, captured, stored_valid, saved_pending, init_done;
    int8_t result;
    float first_pos, second_pos, min_pos, max_pos;
} pitch_dm_cali_status_t;
typedef struct { struct { float pos,vel; uint32_t last_fdb_time; uint8_t state,id; } fdb; } dm_measure_t;
typedef struct { float min_pos, max_pos, target; } gimbal_control_t;
static uint8_t pitch_dm_flash_ready;
static uint8_t calibration_flash_ready;
volatile pitch_dm_cali_status_t pitch_dm_cali_status;
gimbal_control_t gimbal_control;
dm_measure_t motor;
pitch_dm_cali_record_t stored_record;
uint32_t fake_tick, init_calls, read_calls, apply_calls, critical_depth;
uint8_t init_ok, read_ok, valid_at_apply;
uint32_t HAL_GetTick(void) { return fake_tick; }
int8_t flash_init(void) { ++init_calls; return init_ok ? 0 : -1; }
int8_t flash_read(uint32_t address, uint32_t *buf, uint32_t words) {
    const uint32_t *src = (const uint32_t *)&stored_record;
    uint32_t i;
    ++read_calls;
    if (!read_ok || address != PITCH_DM_CALI_ADDR || words != 5U) return -1;
    for (i = 0; i < words; ++i) buf[i] = src[i];
    return 0;
}
dm_measure_t *get_dm_motor_measure_point(uint8_t index) {
    return index == PITCH_DM_MOTOR_INDEX ? &motor : NULL;
}
void pitch_dm_apply_calibration(gimbal_control_t *control,
                                float min_pos, float max_pos, float current_pos) {
    ++apply_calls;
    valid_at_apply = pitch_dm_cali_status.stored_valid;
    control->min_pos = min_pos;
    control->max_pos = max_pos;
    control->target = current_pos;
}
#define taskENTER_CRITICAL() (++critical_depth)
#define taskEXIT_CRITICAL() (--critical_depth)
/* Focused cold-start chain: health availability is controlled from the fixture's
 * motor timestamps. Full real health/Flash integration is test_b05_persistence.c. */
uint8_t health_fresh(uint8_t id,uint32_t now){(void)id;return motor.fdb.last_fdb_time && now-motor.fdb.last_fdb_time<=100;}
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
static void pitch_dm_cali_recover(void)
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
volatile struct { uint32_t status, checks, failure; } test_result;
__asm void stop_test(void) {
    BKPT 0
    BX lr
}
void check(int ok, uint32_t code) {
    ++test_result.checks;
    if (!ok && !test_result.failure) test_result.failure = code;
}
void test_entry(void) {
    stored_record.magic = PITCH_DM_CALI_MAGIC;
    stored_record.version = PITCH_DM_CALI_VERSION;
    stored_record.min_pos = -2.79f;
    stored_record.max_pos = -2.3f;
    stored_record.check = pitch_dm_cali_check(&stored_record);
    motor.fdb.state=1; motor.fdb.id=3; motor.fdb.vel=0;
    motor.fdb.pos = -2.5f;
    motor.fdb.last_fdb_time = 1000;
    fake_tick = 1000;
    pitch_dm_cali_recover();
    check(init_calls == 0 && !pitch_dm_cali_status.stored_valid, 1);
    pitch_dm_cali_status.init_done = 1;
    pitch_dm_cali_recover();
    check(init_calls == 1 && !pitch_dm_cali_status.stored_valid, 2);
    init_ok = 1;
    fake_tick = 2000;
    motor.fdb.last_fdb_time = 2000;
    pitch_dm_cali_recover();
    check(init_calls == 2 && read_calls == 1 && !pitch_dm_cali_status.stored_valid, 3);
    read_ok = 1;
    fake_tick = 3000;
    motor.fdb.last_fdb_time = 3000;
    stored_record.check ^= 1U;
    pitch_dm_cali_recover();
    check(!pitch_dm_cali_status.stored_valid && apply_calls == 0, 4);
    stored_record.check = pitch_dm_cali_check(&stored_record);
    fake_tick = 4000;
    motor.fdb.last_fdb_time = 3899;
    pitch_dm_cali_recover();
    check(!pitch_dm_cali_status.stored_valid && apply_calls == 0, 5);
    fake_tick = 5000;
    motor.fdb.last_fdb_time = 5000;
    motor.fdb.pos = -2.1f;
    pitch_dm_cali_recover();
    check(!pitch_dm_cali_status.stored_valid && apply_calls == 0, 6);
    fake_tick = 6000;
    motor.fdb.last_fdb_time = 6000;
    motor.fdb.pos = -2.811f;
    pitch_dm_cali_recover();
    check(!pitch_dm_cali_status.stored_valid && apply_calls == 0, 7);
    fake_tick = 7000;
    motor.fdb.last_fdb_time = 7000;
    motor.fdb.pos = -2.801f;
    pitch_dm_cali_recover();
    check(pitch_dm_cali_status.stored_valid && apply_calls == 1 &&
          !valid_at_apply && critical_depth == 0 &&
          calibration_flash_ready == 0, 8);
    check(gimbal_control.min_pos == -2.79f &&
          gimbal_control.max_pos == -2.3f &&
          gimbal_control.target == -2.801f, 9);
    fake_tick = 8000;
    pitch_dm_cali_recover();
    check(apply_calls == 1, 10);
    test_result.status = 0x600D;
    stop_test();
}
