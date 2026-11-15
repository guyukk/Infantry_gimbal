#include <math.h>
#include <string.h>
#include "calibrate_task.c"
#include "health.c"
volatile struct {uint32_t status,checks,failure;} test_result;
static uint8_t storage[8192],original_tail[4096-36];
static uint32_t tick=100,yaw_applied,pitch_applied,erase_count,write_count;
static int fail_init,fail_read,fail_write,corrupt,age_after_write;
static dm_measure_t motor;
gimbal_control_t gimbal_control;
int8_t flash_init(void){return fail_init?-1:0;}
int8_t flash_read(uint32_t a,uint32_t*b,uint32_t n){if(fail_read)return -1;memcpy(b,storage+a,n*4);if(corrupt && a==0 && write_count)((uint8_t*)b)[0]^=1;return 0;}
int8_t flash_write_single_address(uint32_t a,uint32_t*b,uint32_t n){write_count++;if(age_after_write)tick+=101;if(fail_write)return -1;memset(storage+a/4096*4096,255,4096);memcpy(storage+a,b,n*4);return 0;}
void flash_erase_address(uint32_t a,uint16_t n){erase_count++;memset(storage+a,255,n*4096);}
uint32_t HAL_GetTick(void){return tick;}
void vPortEnterCritical(void){}
void vPortExitCritical(void){}
void set_cali_yaw_hook(uint16_t a,fp32 b,fp32 c){(void)a;(void)b;(void)c;yaw_applied++;}
void pitch_dm_apply_calibration(gimbal_control_t*c,float a,float b,float p){(void)c;(void)a;(void)b;(void)p;pitch_applied++;}
dm_measure_t *get_dm_motor_measure_point(uint8_t i){(void)i;return &motor;}
static void ck(int y,uint32_t n){test_result.checks++;if(!y&&!test_result.failure)test_result.failure=n;}
static void init(void){
 head_cali_t h={0};gimbal_cali_t gm={0};pitch_dm_cali_record_t p={PITCH_DM_CALI_MAGIC,1,-2.79f,-2.1f,0};
 memset(storage,0xa5,sizeof(storage));h.latitude=22;h.temperature=40;gm.yaw_offset=100;gm.yaw_min_angle=-3;gm.yaw_max_angle=3;
 memcpy(storage,&h,8);memcpy(storage+8,"HD\0\x55",4);memcpy(storage+12,&gm,20);memcpy(storage+32,"GM\0\x55",4);
 p.check=pitch_dm_cali_check(&p);memcpy(storage+4096,&p,sizeof(p));memset(cali_sensor,0,sizeof(cali_sensor));
 yaw_applied=pitch_applied=erase_count=write_count=0;fail_init=fail_read=fail_write=corrupt=age_after_write=0;
 memset((void*)&pitch_dm_cali_status,0,sizeof(pitch_dm_cali_status));memset(&motor,0,sizeof(motor));motor.fdb.id=3;motor.fdb.state=1;motor.fdb.last_fdb_time=tick;motor.fdb.pos=-2.5f;health_reset();health_touch(HEALTH_PITCH,tick);
}
void test_entry(void){
 float a=99,b=99,lat;init();((gimbal_cali_t*)(storage+12))->yaw_max_angle=NAN;cali_param_init();ck(!yaw_applied,301);
 pitch_dm_cali_load(&a,&b);ck(pitch_dm_cali_status.stored_valid&&a==-2.79f&&b==-2.1f,302);
 init();((gimbal_cali_t*)(storage+12))->yaw_offset=8192;cali_param_init();ck(!yaw_applied,303);
 init();((gimbal_cali_t*)(storage+12))->yaw_min_angle=3;cali_param_init();ck(!yaw_applied,304);
 init();((gimbal_cali_t*)(storage+12))->yaw_max_angle=4;cali_param_init();ck(!yaw_applied,305);
 init();((head_cali_t*)storage)->latitude=INFINITY;cali_param_init();get_flash_latitude(&lat);ck(lat==22&&yaw_applied==1,306);
 init();((gimbal_cali_t*)(storage+12))->pitch_max_angle=NAN;cali_param_init();ck(yaw_applied==1,307);
 init();cali_param_init();memcpy(original_tail,storage+36,sizeof(original_tail));cali_data_write();
 ck(erase_count==0 && write_count==1,308);ck(!memcmp(storage+36,original_tail,sizeof(original_tail)),309);
 ck(storage[4096]==0x31 && storage[4097]==0x4d,310);
 init();cali_param_init();fail_write=1;cali_data_write();ck(write_count==1 && !erase_count,311);
 init();fail_init=1;cali_param_init();ck(!yaw_applied,312);
 init();fail_read=1;cali_param_init();ck(!yaw_applied,313);
 init();cali_param_init();pitch_dm_cali_status.captured=3;pitch_dm_cali_status.first_pos=-2.79f;pitch_dm_cali_status.second_pos=-2.1f;
 motor.fdb.pos=NAN;pitch_dm_cali_status.request=3;pitch_dm_cali_poll();ck(pitch_dm_cali_status.result<0 && !write_count && !pitch_applied,314);
 init();cali_param_init();pitch_dm_cali_status.captured=3;pitch_dm_cali_status.first_pos=-2.79f;pitch_dm_cali_status.second_pos=-2.1f;
 fail_write=1;pitch_dm_cali_status.stored_valid=1;pitch_dm_cali_status.min_pos=-2.7f;pitch_dm_cali_status.max_pos=-2.2f;pitch_dm_cali_status.request=3;pitch_dm_cali_poll();
 ck(pitch_dm_cali_status.result==-5 && !pitch_applied && pitch_dm_cali_status.min_pos==-2.7f,315);

 init();cali_param_init();corrupt=1;cali_data_write();ck(cali_storage_status.last_result==-3,316);
 init();cali_param_init();fail_read=1;cali_data_write();ck(!write_count && cali_storage_status.last_result==-1,317);
 init();cali_param_init();fail_write=1;cali_data_write();ck(cali_storage_status.last_result==-2,318);
 init();cali_param_init();cali_data_write();ck(cali_storage_status.last_result==1,319);
 init();((gimbal_cali_t*)(storage+12))->yaw_offset=8191;((gimbal_cali_t*)(storage+12))->yaw_min_angle=-PI;((gimbal_cali_t*)(storage+12))->yaw_max_angle=PI;cali_param_init();ck(yaw_applied==1,320);
 init();storage[32]='X';cali_param_init();ck(!yaw_applied,321);
 init();cali_param_init();pitch_dm_cali_status.request=1;pitch_dm_cali_poll();ck(pitch_dm_cali_status.result==1 && pitch_dm_cali_status.captured==1,322);
 motor.fdb.pos=-2.2f;pitch_dm_cali_status.request=2;pitch_dm_cali_poll();ck(pitch_dm_cali_status.result==1 && pitch_dm_cali_status.captured==3,323);
 pitch_dm_cali_status.request=4;pitch_dm_cali_poll();ck(!pitch_dm_cali_status.captured && !write_count,324);
 init();cali_param_init();pitch_dm_cali_status.captured=3;pitch_dm_cali_status.first_pos=-2.79f;pitch_dm_cali_status.second_pos=-2.1f;pitch_dm_cali_status.request=3;
 age_after_write=1;pitch_dm_cali_poll();ck(pitch_dm_cali_status.result==-6 && pitch_dm_cali_status.saved_pending && !pitch_applied,325);
 pitch_dm_cali_status.init_done=1;tick=2000;health_touch(HEALTH_PITCH,tick);motor.fdb.last_fdb_time=tick;pitch_dm_cali_recover();
 ck(pitch_dm_cali_status.stored_valid && !pitch_dm_cali_status.saved_pending && pitch_applied==1,326);
 init();cali_param_init();pitch_dm_cali_status.captured=3;pitch_dm_cali_status.first_pos=-2.79f;pitch_dm_cali_status.second_pos=-2.1f;pitch_dm_cali_status.request=3;
 pitch_dm_cali_poll();ck(pitch_dm_cali_status.result==2 && pitch_applied==1 && !pitch_dm_cali_status.captured,327);
 init();tick=0;motor.fdb.last_fdb_time=0;health_reset();health_touch(HEALTH_PITCH,0);pitch_dm_cali_status.request=1;pitch_dm_cali_poll();ck(pitch_dm_cali_status.result==1,328);
 init();tick=101;health_reset();health_touch(HEALTH_PITCH,0);pitch_dm_cali_status.request=1;pitch_dm_cali_poll();ck(pitch_dm_cali_status.result==-1,329);
 test_result.status=test_result.failure?0xbad:0x600d;__asm("bkpt 0");
}


fp32 get_temprate(void){return 25;}
void INS_set_cali_gyro(fp32 s[3],fp32 o[3]){(void)s;(void)o;}
void INS_cali_gyro(fp32 s[3],fp32 o[3],uint16_t *t){(void)s;(void)o;(void)t;}
void RC_restart(uint16_t n){(void)n;}
void RC_unable(void){}
void buzzer_off(void){}
void buzzer_on(uint16_t a,uint16_t b){(void)a;(void)b;}
const RC_ctrl_t *get_remote_control_point(void){static RC_ctrl_t r;return &r;}
osStatus osDelay(uint32_t t){(void)t;return osOK;}
TickType_t xTaskGetTickCount(void){return tick;}
