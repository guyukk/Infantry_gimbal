#ifndef GIMBAL_CONTROL_INTERNAL_H
#define GIMBAL_CONTROL_INTERNAL_H
#include "yaw_pitch_direct.h"
/* Internal control-task APIs; no ISR may mutate controller history. */
uint8_t gimbal_axis_ready(gimbal_control_t *control,uint8_t pitch);
float pitch_dm_raw_to_relative(float raw);
float pitch_dm_relative_to_raw(float relative);
uint8_t pitch_dm_enable_ready(const dm_measure_t *measure);
uint8_t pitch_dm_send_mit(gimbal_control_t *control);
#endif
