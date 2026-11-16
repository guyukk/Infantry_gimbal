#ifndef CONTROL_LOOP_DIAG_H
#define CONTROL_LOOP_DIAG_H
#include <stdint.h>
typedef struct {
    uint32_t iterations, previous_start_ms, current_start_ms;
    uint32_t last_period_ms, max_period_ms, late_starts;
    uint32_t last_execution_ms, max_execution_ms, execution_overruns;
    uint8_t seen;
} control_loop_diag_t;
/* Counters wrap naturally. No logging, allocation or I/O. A zero duration only
 * means <1ms at HAL tick resolution, never proof of a met real-time deadline.
 * Read while halted, or copy under a short critical section for coherent groups. */
static inline void control_loop_diag_begin(control_loop_diag_t *d,uint32_t now,uint32_t period_ms)
{
    d->current_start_ms=now;
    if(d->seen){d->last_period_ms=(uint32_t)(now-d->previous_start_ms);
        if(d->last_period_ms>d->max_period_ms)d->max_period_ms=d->last_period_ms;
        if(d->last_period_ms>period_ms)d->late_starts++;}
    d->previous_start_ms=now;d->seen=1;
}
static inline void control_loop_diag_end(control_loop_diag_t *d,uint32_t now,uint32_t period_ms)
{
    d->last_execution_ms=(uint32_t)(now-d->current_start_ms);
    if(d->last_execution_ms>d->max_execution_ms)d->max_execution_ms=d->last_execution_ms;
    if(d->last_execution_ms>=period_ms)d->execution_overruns++;
    d->iterations++;
}
#endif
