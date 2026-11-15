#include <string.h>
#include <math.h>
#include "auto_aim.c"
#include "health.c"
#include "remote_control.c"
volatile struct{uint32_t status,checks,failure;}test_result;
volatile const uint32_t test_robot_mode=ROBOT_MODE;
uproto_context_t proto_ctx;
gimbal_control_t gimbal_control;
volatile gimbal_behaviour_e gimbal_behaviour;
UART_HandleTypeDef huart1,huart5;
DMA_HandleTypeDef hdma_usart5_rx,hdma_usart1_tx,hdma_usart1_rx;
uint8_t remote_buff[SBUS_RX_BUF_NUM];
remoter_t remoter;
static uint32_t tick;
uint32_t HAL_GetTick(void){return tick;}
osStatus osDelay(uint32_t n){(void)n;return osOK;}
uint8_t get_robot_id(void){return 0;}
void Vofa_JustFloat(float*d,uint8_t n){(void)d;(void)n;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*h,uint8_t*d,uint16_t n){(void)h;(void)d;(void)n;return HAL_OK;}
static void check(uint8_t ok,uint32_t n){test_result.checks++;if(!ok){test_result.failure=n;__asm("bkpt 0");}}
static void fresh(void){uint8_t i;for(i=0;i<HEALTH_SOURCE_COUNT;i++)health_touch(i,tick);}
static void setup(uint32_t now){
 health_reset();tick=now;fresh();memset(&rc_ctrl,0,sizeof(rc_ctrl));rc_ctrl.rc.s[0]=RC_SW_DOWN;rc_ctrl.rc.s[1]=RC_SW_UP;
 gimbal_behaviour=GIMBAL_RELATIVE_ANGLE;auto_aim_init(&aim);aim.auto_aim_flag=AIM_ON;aim.online=1;
}
void test_entry(void){
 uint32_t i,stamp;
 (void)i;
 setup(0);auto_aim_apply_delta_udeg(1000000,1000000,0,1);stamp=aim.last_fdb;
 tick=2001;fresh();auto_aim_feedback_update(&aim);auto_aim_control_tick(&aim);
 check(!aim.online && aim.receive.yaw==0 && aim.receive.pitch==0 && aim.last_fdb==stamp,1);
#ifdef AUTO_AIM_HEALTH_API
 setup(0);auto_aim_apply_delta_udeg(1000000,-1000000,0xFFFF,0xFFFFFFFFFFFFFFFFULL);
 tick=2000;fresh();auto_aim_control_tick(&aim);check(aim.online,2);
 tick=2001;fresh();auto_aim_control_tick(&aim);check(!aim.online && !s_auto_aim_ctrl.yaw_axis.err_rad && !s_last_err_yaw && !yawfilter.last_out,3);
 setup(10);auto_aim_apply_delta_udeg(1000000,1000000,0,1);
 for(i=0;i<8;i++){tick++;fresh();auto_aim_control_tick(&aim);}
 check(aim.receive.yaw!=0,10);
 rc_ctrl.key.v=KEY_PRESSED_OFFSET_R;auto_aim_set(&aim);
 check(aim.auto_aim_flag==AIM_OFF && !s_auto_aim_ctrl.yaw_axis.err_rad && !aim.receive.yaw,11);
 rc_ctrl.key.v=0;auto_aim_set(&aim);rc_ctrl.key.v=KEY_PRESSED_OFFSET_R;auto_aim_set(&aim);auto_aim_control_tick(&aim);
 check(aim.auto_aim_flag==AIM_ON && !aim.online && !aim.receive.yaw,12);
 auto_aim_apply_delta_udeg(0,0,0,2);auto_aim_control_tick(&aim);
 check(aim.online && s_auto_aim_ctrl.yaw_axis.err_rad==0 && aim.receive.yaw==0,13);
 stamp=aim.last_fdb;
 for(i=0;i<50;i++){tick++;fresh();auto_aim_feedback_update(&aim);auto_aim_control_tick(&aim);}
 check(aim.last_fdb==stamp,14);
 setup(100);auto_aim_apply_delta_udeg(1000000,1000000,0,3);
 for(i=0;i<8;i++){tick++;fresh();auto_aim_control_tick(&aim);}
 check(auto_aim_take_increment(0)!=0 && aim.receive.yaw==0,20);
 auto_aim_apply_delta_udeg(1000000,1000000,0,4);tick++;fresh();auto_aim_control_tick(&aim);
 check(auto_aim_take_increment(0)!=0,21);
 auto_aim_apply_delta_udeg(0,0,0,5);
 for(i=0;i<200;i++){tick++;fresh();auto_aim_control_tick(&aim);}
 check(aim.online && aim.receive.yaw==0 && aim.receive.pitch==0,22);
 health_reject(HEALTH_RC);check(auto_aim_take_increment(0)==0 && aim.auto_aim_flag==AIM_OFF && !s_auto_aim_ctrl.yaw_axis.err_rad,23);
 setup(200);auto_aim_apply_delta_udeg(1000000,1000000,0,6);
 gimbal_behaviour=GIMBAL_MOTIONLESS;auto_aim_control_tick(&aim);
 check(aim.receive.yaw==0 && !aim.online,24);
 setup(0xFFFFFFF0U);auto_aim_apply_delta_udeg(1000000,1000000,0,7);
 tick=1984;fresh();auto_aim_control_tick(&aim);check(aim.online,30);
 tick=1985;fresh();auto_aim_control_tick(&aim);check(!aim.online,31);
#endif
 test_result.status=0x600D;__asm("bkpt 0");
}

