#include <stdint.h>
#include <string.h>
#include "control_loop_diag.h"
volatile struct {uint32_t status,checks,failure;} test_result;
static void check(uint8_t ok,uint32_t n){test_result.checks++;if(!ok){test_result.failure=n;__asm("bkpt 0");}}
void test_entry(void)
{
    control_loop_diag_t d;memset(&d,0,sizeof(d));
    control_loop_diag_begin(&d,0,1);control_loop_diag_end(&d,0,1);
    check(d.seen && d.iterations==1 && !d.late_starts && !d.execution_overruns,1);
    control_loop_diag_begin(&d,1,1);control_loop_diag_end(&d,2,1);
    check(d.last_period_ms==1 && d.execution_overruns==1 && d.last_execution_ms==1,2);
    control_loop_diag_begin(&d,4,1);control_loop_diag_end(&d,6,1);
    check(d.last_period_ms==3 && d.max_period_ms==3 && d.late_starts==1,3);
    check(d.max_execution_ms==2 && d.execution_overruns==2 && d.iterations==3,4);
    memset(&d,0,sizeof(d));control_loop_diag_begin(&d,0xfffffffeU,1);control_loop_diag_end(&d,0xffffffffU,1);
    control_loop_diag_begin(&d,0,1);control_loop_diag_end(&d,0,1);
    check(d.last_period_ms==2 && d.late_starts==1 && d.max_execution_ms==1 && d.iterations==2,5);
    test_result.status=0x600D;__asm("bkpt 0");
}
