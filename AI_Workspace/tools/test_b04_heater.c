#include <stdint.h>
#include <string.h>
#include "candidates/imu_heater_candidate.h"
volatile struct {uint32_t status,checks,failure;} test_result;
static void check(uint8_t ok,uint32_t n){test_result.checks++;if(!ok){test_result.failure=n;__asm("bkpt 0");}}
void test_entry(void)
{
    imu_heater_candidate_t s={0,0};uint16_t pwm=123,d;uint32_t bits=0x7fc00000U;float nan;
    memcpy(&nan,&bits,4);
    check(imu_heater_candidate_demand(&s,40,35,1)==500,1);
    check(imu_heater_candidate_demand(&s,40,40,1)==0 && s.previous_error==0 && s.older_error==0,2);
    check(imu_heater_candidate_demand(&s,40,45,1)==0,3);
    check(imu_heater_candidate_demand(&s,40,39,1)>0,4);
    check(imu_heater_candidate_demand(&s,40,45,1)==0 && s.previous_error==0,5);
    check(imu_heater_candidate_demand(&s,40,nan,1)==0,6);
    check(imu_heater_candidate_demand(&s,nan,35,1)==0,7);
    check(imu_heater_candidate_demand(&s,40,35,0)==0,8);
    bits=0x7f800000U;memcpy(&nan,&bits,4);check(imu_heater_candidate_demand(&s,40,nan,1)==0,9);
    bits=0xff800000U;memcpy(&nan,&bits,4);check(imu_heater_candidate_demand(&s,40,nan,1)==0,10);
    check(!imu_heater_candidate_pwm(500,0,0,500,&pwm) && pwm==123,11);
    check(imu_heater_candidate_pwm(0,1,500,0,&pwm) && pwm==500,12);
    check(imu_heater_candidate_pwm(500,1,500,0,&pwm) && pwm==0,13);
    check(imu_heater_candidate_pwm(250,1,0,500,&pwm) && pwm==250,14);
    check(!imu_heater_candidate_pwm(501,1,0,500,&pwm),15);
    check(!imu_heater_candidate_pwm(0,1,501,0,&pwm),16);
    check(!imu_heater_candidate_pwm(0,1,0,0,&pwm),17);
    for(bits=0;bits<1000;bits++){d=imu_heater_candidate_demand(&s,40,39.999f-bits*0.01f,1);check(d<=500,18);}
    imu_heater_candidate_reset(&s);check(s.previous_error==0 && s.older_error==0,19);
    test_result.status=0x600D;__asm("bkpt 0");
}
