/*
 * ============================================================================
 * 本车健康/故障源实现（P04）
 * ----------------------------------------------------------------------------
 * 每个源固定单写者（ISR 或单个任务）。写者用 seq 奇偶保护三元组，读者用
 * 序号快照重试；不阻塞、不打印、不访问 Flash。
 * ============================================================================
 */
#include "health.h"
#include <string.h>

typedef struct
{
    volatile uint32_t seq;           /* 奇数=写入中，偶数=稳定 */
    volatile uint32_t last_valid_ms;
    volatile uint32_t generation;
    volatile uint8_t seen;
    uint8_t reserved[3];
} health_slot_t;

static health_slot_t health_slots[HEALTH_SRC_COUNT];

static uint8_t health_source_valid(health_source_e source)
{
    return (uint8_t)((uint32_t)source < (uint32_t)HEALTH_SRC_COUNT);
}

void health_init(void)
{
    memset(health_slots, 0, sizeof(health_slots));
}

/*
 * ============================================================================
 * 步骤1：有效数据发布
 * ============================================================================
 * 数据源：各接收 ISR / IMU 任务
 * 操作：序号置奇数 -> 更新时间与计数 -> 序号置偶数
 */
void health_mark_valid(health_source_e source, uint32_t now_ms)
{
    health_slot_t *slot;

    if (!health_source_valid(source))
    {
        return;
    }
    slot = &health_slots[source];
    slot->seq++;
    __DMB();
    slot->last_valid_ms = now_ms;
    slot->generation++;
    slot->seen = 1U;
    __DMB();
    slot->seq++;
    __DMB();
}

/*
 * ============================================================================
 * 步骤2：状态快照
 * ============================================================================
 * 操作：
 * 1) 序号快照读取三元组，奇数或变化时重试
 * 2) 用无符号差值判新鲜，兼容 tick 回绕
 * 3) seen=0 -> NEVER_SEEN；seen 且超时 -> STALE；否则 OK
 */
void health_get(health_source_e source, uint32_t now_ms, health_status_t *out)
{
    health_slot_t *slot;
    uint32_t before;
    uint32_t after;
    uint32_t attempt;
    uint32_t last = 0U;
    uint32_t generation = 0U;
    uint8_t seen = 0U;

    if (out == NULL)
    {
        return;
    }
    out->fresh = 0U;
    out->reason = HEALTH_REASON_NEVER_SEEN;
    out->last_valid_ms = 0U;
    out->generation = 0U;
    if (!health_source_valid(source))
    {
        return;
    }
    slot = &health_slots[source];
    for (attempt = 0U; attempt < 4U; attempt++)
    {
        before = slot->seq;
        if ((before & 1U) != 0U)
        {
            continue;
        }
        __DMB();
        last = slot->last_valid_ms;
        generation = slot->generation;
        seen = slot->seen;
        __DMB();
        after = slot->seq;
        if (before == after)
        {
            break;
        }
    }
    out->last_valid_ms = last;
    out->generation = generation;
    if (!seen || generation == 0U)
    {
        out->fresh = 0U;
        out->reason = HEALTH_REASON_NEVER_SEEN;
        return;
    }
    if ((uint32_t)(now_ms - last) > HEALTH_TIMEOUT_MS)
    {
        out->fresh = 0U;
        out->reason = HEALTH_REASON_STALE;
        return;
    }
    out->fresh = 1U;
    out->reason = HEALTH_REASON_OK;
}

uint8_t health_is_fresh(health_source_e source, uint32_t now_ms)
{
    health_status_t status;
    health_get(source, now_ms, &status);
    return status.fresh;
}

void health_get_all(health_status_t *out, uint32_t now_ms)
{
    uint32_t index;

    if (out == NULL)
    {
        return;
    }
    for (index = 0U; index < (uint32_t)HEALTH_SRC_COUNT; index++)
    {
        health_get((health_source_e)index, now_ms, &out[index]);
    }
}
