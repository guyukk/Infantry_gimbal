#ifndef HOST_CONTROL_H
#define HOST_CONTROL_H
#include <stdint.h>
#ifndef COMM_HOST_CMD_TIMEOUT_MS
#define COMM_HOST_CMD_TIMEOUT_MS 120U
#endif
#ifndef COMM_FIRE_SINGLE_PULSE_MS
#define COMM_FIRE_SINGLE_PULSE_MS 20U
#endif
typedef struct { uint8_t continuous, single; } host_fire_request_t;
typedef struct { uint32_t accepted, rejected, duplicates, expired, chassis_unsupported; } host_control_stats_t;
uint8_t host_fire_submit(int32_t on,int32_t mode,int32_t burst,uint16_t status,uint64_t token);
uint8_t host_fire_take(host_fire_request_t *out);
void host_fire_cancel(void);
void host_control_tick(void);
void host_chassis_reject(void);
void host_control_get_stats(host_control_stats_t *out);
#endif

