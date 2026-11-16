#ifndef PITCH_CALIBRATION_H
#define PITCH_CALIBRATION_H
#include "calibrate_task.h"
/* Calibration task owns requests/Flash; control consumes published limits.
 * Sampling/application critical sections remain bounded; Flash stays outside. */
void pitch_dm_cali_poll(void);
void pitch_dm_cali_recover(void);
void pitch_dm_calibration_set_flash_ready(uint8_t ready);
#endif
