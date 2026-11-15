#include "health.h"
#include "main.h"
#include <string.h>

/* Short PRIMASK sections serialize ISR publishers with task/final-output readers.
 * No scheduler calls, waiting, heap allocation, or task-driven receive heartbeat. */
static health_status_t sources[HEALTH_SOURCE_COUNT];
static health_gate_status_t gates[HEALTH_GATE_COUNT];
static health_bus_stats_t buses[2];

static uint32_t source_age(uint8_t id, uint32_t now)
{
    /* now may have been sampled before a preempting publisher. Refresh it only
       when the live clock confirms the newly published timestamp is in the past. */
    if ((int32_t)(now-sources[id].last_valid_ms) < 0)
    {
        uint32_t current=HAL_GetTick();
        if ((int32_t)(current-sources[id].last_valid_ms) >= 0) now=current;
    }
    return (uint32_t)(now-sources[id].last_valid_ms);
}
static uint8_t source_fresh(uint8_t id, uint32_t now)
{
    return sources[id].seen && sources[id].valid &&
           source_age(id,now) <= HEALTH_TIMEOUT_MS;
}
static void gate_trip_locked(uint8_t gate)
{
    if (!gates[gate].blocked) gates[gate].epoch++;
    gates[gate].blocked = 1U;
    gates[gate].fault_rc_generation = sources[HEALTH_RC].generation;
}
void health_reset(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    memset(sources, 0, sizeof(sources));
    memset(gates, 0, sizeof(gates));
    memset(buses, 0, sizeof(buses));
    __set_PRIMASK(key);
}
void health_touch(uint8_t id, uint32_t now)
{
    uint32_t key;
    uint8_t i;
    if (id >= HEALTH_SOURCE_COUNT) return;
    key = __get_PRIMASK(); __disable_irq();
    /* A resumed source must not hide a timeout that fell between task polls. */
    if (sources[id].seen && (uint32_t)(now-sources[id].last_valid_ms)>HEALTH_TIMEOUT_MS)
    {
        sources[id].fault_generation++;
        for (i=0U;i<HEALTH_GATE_COUNT;i++)
            if (!gates[i].blocked && (id==HEALTH_RC || (gates[i].mask & HEALTH_BIT(id))))
                gate_trip_locked(i);
    }
    sources[id].last_valid_ms = now;
    sources[id].generation++;
    sources[id].seen = sources[id].valid = 1U;
    __set_PRIMASK(key);
}
void health_reject(uint8_t id)
{
    uint32_t key;
    uint8_t i;
    if (id >= HEALTH_SOURCE_COUNT) return;
    key = __get_PRIMASK(); __disable_irq();
    sources[id].valid = 0U;
    sources[id].rejected++;
    sources[id].fault_generation++;
    for (i = 0U; i < HEALTH_GATE_COUNT; i++)
    {
        if (id == HEALTH_RC || (gates[i].mask & HEALTH_BIT(id)))
            gate_trip_locked(i);
    }
    __set_PRIMASK(key);
}
uint8_t health_snapshot(uint8_t id, uint32_t now, health_status_t *out)
{
    uint32_t key;
    if (id >= HEALTH_SOURCE_COUNT || out == 0) return 0U;
    key = __get_PRIMASK(); __disable_irq();
    *out = sources[id];
    out->age_ms = source_age(id,now);
    out->fresh = source_fresh(id, now);
    out->reason = !out->seen ? HEALTH_NEVER_SEEN : !out->valid ? HEALTH_INVALID :
                  out->fresh ? HEALTH_OK : HEALTH_STALE;
    __set_PRIMASK(key);
    return 1U;
}
uint8_t health_fresh(uint8_t id, uint32_t now)
{
    health_status_t status;
    return health_snapshot(id, now, &status) && status.fresh;
}
void health_gate_trip(uint8_t gate)
{
    uint32_t key;
    if (gate >= HEALTH_GATE_COUNT) return;
    key = __get_PRIMASK(); __disable_irq();
    if (!gates[gate].blocked) gate_trip_locked(gate);
    __set_PRIMASK(key);
}
uint8_t health_gate_ready(uint8_t gate, uint32_t mask, uint32_t now)
{
    uint32_t key;
    uint8_t i, ready = 1U;
    if (gate >= HEALTH_GATE_COUNT || mask == 0U ||
        (mask >> HEALTH_SOURCE_COUNT) != 0U) return 0U;
    key = __get_PRIMASK(); __disable_irq();
    gates[gate].mask = mask | HEALTH_BIT(HEALTH_RC);
    for (i = 0U; i < HEALTH_SOURCE_COUNT; i++)
        if ((gates[gate].mask & HEALTH_BIT(i)) && !source_fresh(i, now)) ready = 0U;
    if (!ready)
    {
        /* Keep the first fault generation while a fault persists. */
        if (!gates[gate].blocked) gate_trip_locked(gate);
    }
    else if (gates[gate].blocked)
    {
        if (sources[HEALTH_RC].generation == gates[gate].fault_rc_generation)
            ready = 0U;
        else gates[gate].blocked = 0U;
    }
    __set_PRIMASK(key);
    return ready;
}
uint32_t health_gate_mask(uint8_t gate)
{
    return gate < HEALTH_GATE_COUNT ? gates[gate].mask : 0U;
}
uint32_t health_gate_epoch(uint8_t gate)
{
    return gate < HEALTH_GATE_COUNT ? gates[gate].epoch : 0U;
}
uint8_t health_gate_snapshot(uint8_t gate, health_gate_status_t *out)
{
    uint32_t key;
    if (gate >= HEALTH_GATE_COUNT || out == 0) return 0U;
    key = __get_PRIMASK(); __disable_irq(); *out = gates[gate]; __set_PRIMASK(key);
    return 1U;
}
void health_bus_rx(uint8_t bus, uint8_t accepted)
{
    uint32_t key;
    if (bus >= 2U) return;
    key = __get_PRIMASK(); __disable_irq();
    if (accepted) buses[bus].rx_accepted++; else buses[bus].rx_rejected++;
    __set_PRIMASK(key);
}
void health_bus_tx(uint8_t bus, uint8_t queued)
{
    uint32_t key;
    if (bus >= 2U) return;
    key = __get_PRIMASK(); __disable_irq();
    if (queued) buses[bus].tx_queued++; else buses[bus].tx_failed++;
    __set_PRIMASK(key);
}
uint8_t health_bus_snapshot(uint8_t bus, health_bus_stats_t *out)
{
    uint32_t key;
    if (bus >= 2U || out == 0) return 0U;
    key = __get_PRIMASK(); __disable_irq(); *out = buses[bus]; __set_PRIMASK(key);
    return 1U;
}

