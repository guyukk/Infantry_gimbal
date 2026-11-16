#ifndef IMU_HEATER_CANDIDATE_H
#define IMU_HEATER_CANDIDATE_H
#include <stdint.h>
/* Isolated P06 candidate. No HAL calls; not included by the formal target.
 * demand=0 means no heating. Physical PWM polarity is an explicit mapping. */
typedef struct { float previous_error, older_error; } imu_heater_candidate_t;
static inline void imu_heater_candidate_reset(imu_heater_candidate_t *s)
{ s->previous_error=s->older_error=0.0f; }
static inline uint16_t imu_heater_candidate_demand(imu_heater_candidate_t *s,
    float target_c, float measured_c, uint8_t sensor_valid)
{
    float error, demand;
    /* Ordered comparisons reject NaN and both infinities without fast-math. */
    if (!sensor_valid || !(target_c<=3.402823466e38f && target_c>=-3.402823466e38f) ||
        !(measured_c<=3.402823466e38f && measured_c>=-3.402823466e38f) || measured_c>=target_c)
    { imu_heater_candidate_reset(s);return 0U; }
    error=target_c-measured_c;
    if (!(error<=3.402823466e38f)) {imu_heater_candidate_reset(s);return 0U;}
    /* Preserve the baseline three-error formula and gains, correcting direction. */
    demand=100.0f*error+50.0f*(error+s->previous_error+s->older_error)+10.0f*(error-s->previous_error);
    s->older_error=s->previous_error;s->previous_error=error;
    if (demand>=500.0f)return 500U;
    return demand>0.0f ? (uint16_t)demand : 0U;
}
static inline uint8_t imu_heater_candidate_pwm(uint16_t demand,uint8_t polarity_confirmed,
    uint16_t off_pwm,uint16_t full_pwm,uint16_t *pwm)
{
    int32_t delta;
    if (!polarity_confirmed || !pwm || off_pwm>500U || full_pwm>500U || off_pwm==full_pwm || demand>500U)return 0U;
    delta=(int32_t)full_pwm-off_pwm;
    *pwm=(uint16_t)((int32_t)off_pwm+delta*demand/500);
    return 1U;
}
#endif
