
#include <stdint.h>
#include <stddef.h>
#define PITCH_DM_ENABLE_FIRST_RETRY_MS 2000U
#define PITCH_DM_ENABLE_RETRY_MS 500U
#define PITCH_DM_ENABLE_MAX_ATTEMPTS 8U
#define PITCH_DM_CAN_ID 0x53U
#define GIMBAL_CAN fake_can
typedef struct { struct { uint32_t last_fdb_time; int state; } fdb; } dm_measure_t;
typedef struct {
    uint8_t attempts, last_tx_status, confirmed, exhausted;
    uint32_t last_attempt_tick, confirmed_feedback_tick;
} pitch_dm_enable_status_t;
volatile pitch_dm_enable_status_t pitch_dm_enable_status;
uint32_t fake_now, send_count;
uint8_t fake_tx_status;
uint16_t sent_id;
int fake_can;
uint32_t HAL_GetTick(void) { return fake_now; }
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
    test_result.status = 0x600D;
    stop_test();
}
