#include <string.h>
#include "health.c"
volatile struct { uint32_t status,checks,failure; } test_result;
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
static void check(uint8_t ok,uint32_t n) { test_result.checks++;if(!ok){test_result.failure=n;__asm("bkpt 0");} }
static void fill(uint32_t mask,uint32_t now,uint8_t except) {
    uint8_t i;for(i=0;i<HEALTH_SOURCE_COUNT;i++)if(i!=except && (mask&HEALTH_BIT(i)))health_touch(i,now);
}
void test_entry(void) {
    uint8_t i;
    health_status_t st;
    health_gate_status_t gate;
    health_reset();
    health_snapshot(HEALTH_RC,0,&st);check(!st.seen && !st.fresh && st.reason==HEALTH_NEVER_SEEN,1);
    health_touch(HEALTH_RC,0);health_snapshot(HEALTH_RC,0,&st);
    check(st.seen && st.fresh && st.generation==1 && st.last_valid_ms==0,2);
    check(health_fresh(HEALTH_RC,100),3);check(!health_fresh(HEALTH_RC,101),4);
    health_touch(HEALTH_RC,0xFFFFFFF0U);check(health_fresh(HEALTH_RC,84U),5);check(!health_fresh(HEALTH_RC,85U),6);
    for(i=0;i<HEALTH_SOURCE_COUNT;i++) {
        uint32_t mask=HEALTH_BIT(HEALTH_RC)|HEALTH_BIT(i);
        health_reset();fill(mask,0,255);
        check(health_gate_ready(0,mask,0),100+i);
        fill(mask,100,i); /* Other sources keep updating before the deadline. */
        fill(mask,101,i);
        check(!health_gate_ready(0,mask,101),200+i);
        health_touch(i,101);
        if(i!=HEALTH_RC)check(!health_gate_ready(0,mask,101),300+i);
        health_touch(HEALTH_RC,101);check(health_gate_ready(0,mask,101),400+i);
        health_reject(i);health_snapshot(i,101,&st);
        check(!st.fresh && st.valid==0 && st.rejected==1 && st.generation>=1,500+i);
        check(!health_gate_ready(0,mask,101),600+i);
        health_touch(i,101);
        if(i!=HEALTH_RC)check(!health_gate_ready(0,mask,101),700+i);
        health_touch(HEALTH_RC,101);check(health_gate_ready(0,mask,101),800+i);
        health_gate_snapshot(0,&gate);check(gate.epoch>=2,900+i);
    }
    health_reset();health_touch(HEALTH_RC,1);
    check(health_gate_ready(1,HEALTH_BIT(HEALTH_RC),1),1000);
    health_reject(HEALTH_RC);health_touch(HEALTH_RC,1);
    check(health_gate_ready(1,HEALTH_BIT(HEALTH_RC),1),1001);
    check(health_gate_epoch(1)==1,1002); /* Fault between two task polls remains observable. */
    health_gate_trip(1);check(!health_gate_ready(1,HEALTH_BIT(HEALTH_RC),1),1003);
    check(!health_gate_ready(255,1,1) && !health_snapshot(255,1,&st),1004);
    health_reset();fill(HEALTH_YAW_MASK,0,255);check(health_gate_ready(1,HEALTH_YAW_MASK,0),1010);
    health_touch(HEALTH_RC,101);check(!health_gate_ready(1,HEALTH_YAW_MASK,101),1011);
    health_touch(HEALTH_RC,102);health_gate_trip(1);health_touch(HEALTH_YAW,102);
    check(health_gate_ready(1,HEALTH_YAW_MASK,102),1012);
    health_reset();fill(HEALTH_YAW_MASK,0,255);check(health_gate_ready(1,HEALTH_YAW_MASK,0),1020);
    health_touch(HEALTH_RC,100);health_touch(HEALTH_YAW,101);
    check(!health_gate_ready(1,HEALTH_YAW_MASK,101),1021); /* A late frame cannot erase an unpolled timeout. */
    health_touch(HEALTH_RC,101);check(health_gate_ready(1,HEALTH_YAW_MASK,101),1022);
    health_reset();tick=10;health_touch(HEALTH_RC,tick);tick=11;health_touch(HEALTH_RC,tick);
    check(health_fresh(HEALTH_RC,10),1030); /* IRQ publishes after caller samples now. */
    tick=112;check(!health_fresh(HEALTH_RC,10),1031);
    test_result.status=0x600D;__asm("bkpt 0");
}

