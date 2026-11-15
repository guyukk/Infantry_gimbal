#include "host_control.h"
#include "main.h"
#include "health.h"
#include "auto_aim.h"
#include "shoot.h"
#include "gimbal_behaviour.h"
#include <string.h>
typedef struct {
    uint32_t last_rx_ms, single_rx_ms;
    uint64_t last_token;
    uint8_t valid, pending_single, continuous, last_on, token_seen;
} host_fire_state_t;
static host_fire_state_t host_fire;
static host_control_stats_t host_stats;
static uint32_t host_health_faults[HEALTH_SOURCE_COUNT];
static uint8_t host_health_changed(uint8_t capture)
{
    uint8_t i,changed=0U;
    health_status_t status;
    for (i=0U;i<HEALTH_SOURCE_COUNT;i++)
        if ((HEALTH_SHOOT_MASK|HEALTH_BIT(HEALTH_IMU)) & HEALTH_BIT(i))
        {
            health_snapshot(i,HAL_GetTick(),&status);
            if (host_health_faults[i] != status.fault_generation) changed=1U;
            if (capture) host_health_faults[i]=status.fault_generation;
        }
    return changed;
}

static uint8_t host_fire_allowed(void)
{
    return shoot_control.shoot_mode >= SHOOT_READY_BULLET && !gimbal_cmd_to_shoot_stop() &&
        auto_aim_is_ready() &&
        health_gate_ready(HEALTH_GATE_SHOOT,HEALTH_SHOOT_MASK,HAL_GetTick());
}
void host_fire_cancel(void)
{
    uint32_t key=__get_PRIMASK(); __disable_irq();
    host_fire.valid=host_fire.pending_single=host_fire.continuous=0U;
    /* Retain token/edge history across stop, timeout and mode changes. */
    __set_PRIMASK(key);
}
uint8_t host_fire_submit(int32_t on,int32_t mode,int32_t burst,uint16_t status,uint64_t token)
{
    uint32_t key=__get_PRIMASK(), now=HAL_GetTick();
    uint8_t continuous;
    (void)status; /* Opaque until host status bits have an agreed contract. */
    __disable_irq();
    if ((on != 0 && on != 1) || mode < 0 || mode > 2 || burst < 0 ||
        (on && !host_fire_allowed()))
    {
        host_stats.rejected++; host_fire_cancel(); __set_PRIMASK(key); return 0U;
    }
    if (!on)
    {
        host_fire_cancel(); host_fire.last_on=0U; host_stats.accepted++;
        __set_PRIMASK(key); return 1U;
    }
    continuous=(mode == 2 || burst > 1);
    host_fire.valid=1U;host_fire.last_rx_ms=now;host_fire.continuous=continuous;
    (void)host_health_changed(1U);
    if (continuous) host_fire.pending_single=0U;
    else
    {
        uint8_t new_single=token ? (!host_fire.token_seen || token != host_fire.last_token) : !host_fire.last_on;
        if (new_single)
        {
            host_fire.pending_single=1U;host_fire.single_rx_ms=now;
            if (token) {host_fire.last_token=token;host_fire.token_seen=1U;}
        }
        else host_stats.duplicates++;
    }
    host_fire.last_on=1U;host_stats.accepted++;
    __set_PRIMASK(key);return 1U;
}
void host_control_tick(void)
{
    uint32_t key=__get_PRIMASK();__disable_irq();
    if (host_fire.valid && ((uint32_t)(HAL_GetTick()-host_fire.last_rx_ms) > COMM_HOST_CMD_TIMEOUT_MS || host_health_changed(0U) ||
        !host_fire_allowed()))
    {
        host_stats.expired++;host_fire_cancel();
    }
    __set_PRIMASK(key);
}
uint8_t host_fire_take(host_fire_request_t *out)
{
    uint32_t key;
    if (out == NULL) return 0U;
    key=__get_PRIMASK();__disable_irq();
    memset(out,0,sizeof(*out));host_control_tick();
    if (!host_fire.valid) {__set_PRIMASK(key);return 0U;}
    out->continuous=host_fire.continuous;
    if (host_fire.pending_single)
    {
        if ((uint32_t)(HAL_GetTick()-host_fire.single_rx_ms) <= COMM_FIRE_SINGLE_PULSE_MS) out->single=1U;
        host_fire.pending_single=0U;
    }
    __set_PRIMASK(key);
    return out->continuous || out->single;
}
void host_chassis_reject(void)
{
    uint32_t key=__get_PRIMASK();__disable_irq();host_stats.chassis_unsupported++;__set_PRIMASK(key);
}
void host_control_get_stats(host_control_stats_t *out)
{
    uint32_t key;
    if (out == NULL) return;
    key=__get_PRIMASK();__disable_irq();*out=host_stats;__set_PRIMASK(key);
}

