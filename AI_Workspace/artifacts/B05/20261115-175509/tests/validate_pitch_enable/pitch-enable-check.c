
#include <stdint.h>
#include <stddef.h>
#define PITCH_DM_ENABLE_FIRST_RETRY_MS 2000U
#define PITCH_DM_ENABLE_RETRY_MS 500U
#define PITCH_DM_ENABLE_MAX_ATTEMPTS 8U
#define PITCH_MIT_FEEDBACK_TIMEOUT_MS 100U
#define PITCH_DM_CAN_ID 0x53U
#define GIMBAL_CAN fake_can
#define HAL_OK 0U
typedef struct { struct { uint8_t id, state; float pos; uint32_t last_fdb_time; } fdb; } dm_measure_t;
typedef struct {
    uint8_t attempts, last_tx_status, confirmed, exhausted;
    uint32_t last_attempt_tick, confirmed_feedback_tick;
    uint32_t mit_send_count, last_mit_tick;
    uint8_t last_mit_tx_status, mit_target_valid;
    float mit_target_pos;
} pitch_dm_enable_status_t;
volatile pitch_dm_enable_status_t pitch_dm_enable_status;
uint32_t fake_now, send_count;
uint8_t fake_tx_status;
uint16_t sent_id;
int fake_can;
uint32_t HAL_GetTick(void) { return fake_now; }
enum { HEALTH_PITCH=4 };
static const dm_measure_t *health_measure;
uint8_t health_fresh(uint8_t source,uint32_t now) {
    (void)source;
    return health_measure && health_measure->fdb.last_fdb_time!=0U &&
        (uint32_t)(now-health_measure->fdb.last_fdb_time)<=PITCH_MIT_FEEDBACK_TIMEOUT_MS;
}
uint8_t Motor_enable(void *can, uint16_t id) {
    if (can != &fake_can) return 255;
    sent_id = id;
    send_count++;
    return fake_tx_status;
}
static uint8_t pitch_dm_feedback_allows_mit(const dm_measure_t *motor)
{
    if (motor->fdb.state == 1U)
    {
        return 1U;
    }

                                                                          
                                                                             
                                                                               
    return (PITCH_DM_CAN_ID == 0x53U &&
            motor->fdb.id == 3U && motor->fdb.state == 5U);
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
        (!health_fresh(HEALTH_PITCH,now) ||
         (uint32_t)(now - feedback_tick) > PITCH_MIT_FEEDBACK_TIMEOUT_MS ||
         measure->fdb.state != 1U))
    {
                                                                         
                                                                             
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
static uint8_t test_pitch_enable(const dm_measure_t *measure) {
    health_measure=measure;return pitch_dm_enable_ready(measure);
}
#define pitch_dm_enable_ready(measure) test_pitch_enable(measure)
volatile struct { uint32_t status, checks, failure; } test_result;
__asm void stop_test(void) {
    BKPT 0
    BX lr
}
void check(int ok, uint32_t code) {
    ++test_result.checks;
    if (!ok && !test_result.failure) test_result.failure = code;
}
void reset_state(void) {
    pitch_dm_enable_status.attempts = 1;
    pitch_dm_enable_status.last_tx_status = 0;
    pitch_dm_enable_status.confirmed = 0;
    pitch_dm_enable_status.exhausted = 0;
    pitch_dm_enable_status.last_attempt_tick = 100;
    pitch_dm_enable_status.confirmed_feedback_tick = 0;
    send_count = 0;
    sent_id = 0;
    fake_tx_status = 0;
}
void test_entry(void) {
    dm_measure_t measure = {0};
    reset_state();
    measure.fdb.id = 3;
    measure.fdb.state = 5;
    measure.fdb.last_fdb_time = 101;
    fake_now = 101;
    check(pitch_dm_enable_ready(&measure) == 0 &&
          pitch_dm_enable_status.confirmed == 0, 1);
    fake_now = 2099;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 0, 1);
    fake_now = 2100;
    fake_tx_status = 2;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 1, 2);
    check(pitch_dm_enable_status.attempts == 1 &&
          pitch_dm_enable_status.last_tx_status == 2 && sent_id == 0x53, 3);
    fake_now = 2599;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 1, 4);
    fake_now = 2600;
    fake_tx_status = 0;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 2 &&
          pitch_dm_enable_status.attempts == 2, 5);
    measure.fdb.last_fdb_time = 2599;
    measure.fdb.state = 1;
    fake_now = 2601;
    check(pitch_dm_enable_ready(&measure) == 0 &&
          pitch_dm_enable_status.confirmed == 0, 6);
    measure.fdb.last_fdb_time = 2601;
    check(pitch_dm_enable_ready(&measure) == 1 &&
          pitch_dm_enable_status.confirmed_feedback_tick == 2601, 7);
    fake_now = 2700;
    check(pitch_dm_enable_ready(&measure) == 1 && send_count == 2, 8);
    fake_now = 2702;
    pitch_dm_enable_status.mit_target_valid = 1;
    check(pitch_dm_enable_ready(&measure) == 0 &&
          pitch_dm_enable_status.confirmed == 0 &&
          pitch_dm_enable_status.mit_target_valid == 0 &&
          pitch_dm_enable_status.attempts == 0, 17);
    fake_now = 3202;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 3 &&
          pitch_dm_enable_status.attempts == 1, 18);
    measure.fdb.last_fdb_time = 3203;
    fake_now = 3203;
    check(pitch_dm_enable_ready(&measure) == 1 &&
          pitch_dm_enable_status.confirmed == 1, 19);
    measure.fdb.state = 8;
    measure.fdb.last_fdb_time = 3204;
    fake_now = 3204;
    check(pitch_dm_enable_ready(&measure) == 0 &&
          pitch_dm_enable_status.confirmed == 0, 20);
    reset_state();
    fake_now = 1100;
    measure.fdb.id = 3;
    measure.fdb.state = 5;
    measure.fdb.last_fdb_time = 100;
    check(pitch_dm_enable_ready(&measure) == 0, 9);
    measure.fdb.last_fdb_time = 2100;
    fake_now = 2100;
    check(pitch_dm_enable_ready(&measure) == 0 &&
          pitch_dm_enable_status.confirmed == 0 && send_count == 1, 10);
    reset_state();
    fake_now = 1101;
    measure.fdb.id = 4;
    measure.fdb.last_fdb_time = 1101;
    check(pitch_dm_enable_ready(&measure) == 0, 11);
    measure.fdb.id = 3;
    measure.fdb.state = 8;
    check(pitch_dm_enable_ready(&measure) == 0, 12);
    reset_state();
    pitch_dm_enable_status.attempts = PITCH_DM_ENABLE_MAX_ATTEMPTS;
    fake_now = 1099;
    check(pitch_dm_enable_ready(&measure) == 0 &&
          pitch_dm_enable_status.exhausted == 1 && send_count == 0, 13);
    fake_now = 3100;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 0, 14);
    measure.fdb.id = 3;
    measure.fdb.state = 1;
    measure.fdb.last_fdb_time = 3101;
    fake_now = 3101;
    check(pitch_dm_enable_ready(&measure) == 1 &&
          pitch_dm_enable_status.confirmed == 1 &&
          pitch_dm_enable_status.exhausted == 0, 15);
    check(pitch_dm_enable_ready(NULL) == 0, 16);
    test_result.status = 0x600D;
    stop_test();
}
