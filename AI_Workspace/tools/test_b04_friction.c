/* Real shoot/PID/CAN code; transport, local mode and time use the B03 fixture. */
#define test_entry b03_fixture_entry
#include "test_b03_gimbal_guard.c"
#undef test_entry

static void init_pair(void)
{
    const float gains[3]={100.0f,10.0f,0.0f};
    memset(&shoot_control,0,sizeof(shoot_control));
    shoot_control.fric1.shoot_motor_measure=&DJI_MOTOR_MEASURE[0];
    shoot_control.fric2.shoot_motor_measure=&DJI_MOTOR_MEASURE[1];
#ifdef SHOOT_FRICTION_PID_SPLIT
    PID_init(&shoot_control.fric1.speed_pid,PID_POSITION,gains,10000,2000);
    PID_init(&shoot_control.fric2.speed_pid,PID_POSITION,gains,10000,2000);
#else
    PID_init(&shoot_control.fric_motor_pid,PID_POSITION,gains,10000,2000);
#endif
}
static void expect(uint8_t ok,uint32_t bit)
{ test_result.checks++;if(!ok)test_result.failure|=bit; }
void test_entry(void)
{
    unsigned int n;
    init_pair();
    shoot_control.fric1.speed_set=10;
    fric1_on(&shoot_control);fric2_on(&shoot_control);
    expect(shoot_control.fric2.give_current==0,1);
    fric1_on(&shoot_control);fric2_on(&shoot_control);
    expect(shoot_control.fric2.give_current==0,2);
    init_pair();shoot_control.fric1.speed=0;
    shoot_control.fric2.speed=5;shoot_control.fric2.give_current=1234;
    fric_off();
    expect(shoot_control.fric1.give_current==0 && shoot_control.fric2.give_current==0,4);
#ifdef SHOOT_FRICTION_PID_SPLIT
    /* Opposite direction, saturation and integral limits are independent. */
    init_pair();shoot_control.fric1.speed_set=-200;shoot_control.fric2.speed_set=200;
    for(n=0;n<30;n++){fric1_on(&shoot_control);fric2_on(&shoot_control);}
    check(shoot_control.fric1.give_current==-10000 && shoot_control.fric2.give_current==10000,10);
    /* Existing PID_POSITION only integrates positive errors >0.05. Preserve it. */
    check(shoot_control.fric1.speed_pid.Iout==0 && shoot_control.fric2.speed_pid.Iout==2000,11);
    fric_off();
    check(shoot_control.fric1.speed_pid.Iout==0 && shoot_control.fric2.speed_pid.Iout==0,12);
    hfdcan1.Instance=FDCAN1;hfdcan2.Instance=FDCAN2;rx_result=tx_result=HAL_OK;
    healthy();shoot_init();shoot_history_reset();
    shoot_control.shoot_mode=SHOOT_READY_BULLET;
    shoot_control.fric1.speed_set=-4.45f;shoot_control.fric2.speed_set=4.45f;
    DJI_MOTOR_MEASURE[0].temperate=DJI_MOTOR_MEASURE[1].temperate=30;
    (void)shoot_control_loop();check(!zero(sent_fric),20);
    DJI_MOTOR_MEASURE[1].temperate=80;(void)shoot_control_loop();
    check(zero(sent_fric) && shoot_control.fric1.speed_pid.Iout==0 && shoot_control.fric2.speed_pid.Iout==0,21);
    DJI_MOTOR_MEASURE[1].temperate=30;(void)shoot_control_loop();check(!zero(sent_fric),22);
    DJI_MOTOR_MEASURE[0].temperate=80;(void)shoot_control_loop();check(zero(sent_fric),23);
    DJI_MOTOR_MEASURE[0].temperate=30;
    health_reject(HEALTH_FRIC2);(void)shoot_control_loop();
    check(zero(sent_fric) && shoot_control.shoot_mode==SHOOT_STOP && shoot_control.fric1.speed_pid.Iout==0 && shoot_control.fric2.speed_pid.Iout==0,24);
    fill(255);health_touch(HEALTH_RC,tick);(void)shoot_control_loop();
    check(shoot_control.shoot_mode==SHOOT_STOP && zero(sent_fric),25);
    shoot_control.shoot_mode=SHOOT_READY_BULLET;local_stop=1;(void)shoot_control_loop();
    check(zero(sent_fric) && shoot_control.fric1.give_current==0 && shoot_control.fric2.give_current==0,26);
#else
    (void)n;
#endif
    test_result.status=test_result.failure ? 0xBAD : 0x600D;
    __asm("bkpt 0");
}
