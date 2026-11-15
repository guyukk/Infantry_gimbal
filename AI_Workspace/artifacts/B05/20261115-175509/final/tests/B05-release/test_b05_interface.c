#include <math.h>
#include <string.h>

#include "gimbal_task.h"
#include "bsp_usart.h"
#include "referee.h"
#include "INS_task.h"
extern robot_status_t robot_status;
extern game_status_t game_status;
extern projectile_allowance_t projectile_allowance;
#include "User/Communication/channel/gimbal/gimbal_channel.h"
#include "User/Communication/channel/time_sync/time_sync_channel.h"
#include "health.h"
#define COMM_REFEREE_REQUIRE_QUERY 1
#define COMM_REFEREE_PUB_PERIOD_MS 50
#define COMM_REFEREE_STALE_TIMEOUT_MS 1000
static uint64_t g_referee_last_pub_us;
static uint8_t g_referee_query_seen,g_referee_force_pub;
static gimbal_channel_t g_gimbal;
static uint64_t sync_now_us(void *u){(void)u;return 123;}
/* B05_COMM_CALLBACKS */

#include "health.c"
volatile struct {uint32_t status,checks,failure;} test_result;
static uint32_t tick;
static gimbal_motor_t yaw_motor,pitch_motor;
static fp32 angles[3];
chassis_data_t chassis_data;
robot_status_t robot_status;
game_status_t game_status;
projectile_allowance_t projectile_allowance;
static gimbal_referee_t published;
uint32_t HAL_GetTick(void){return tick;}
const gimbal_motor_t *get_yaw_motor_point(void){return &yaw_motor;}
const gimbal_motor_t *get_pitch_motor_point(void){return &pitch_motor;}
fp32 *get_INS_angle_point(void){return angles;}
uint64_t time_sync_channel_now_us(const time_sync_channel_t *t){(void)t;return 123;}
uint8_t referee_data_available(uint32_t t){(void)t;return 1;} /* Deliberately tempting unrelated globals. */
bool gimbal_channel_publish_referee(gimbal_channel_t *g,const gimbal_referee_t *v){(void)g;published=*v;return true;}
uint8_t chassis_data_read(chassis_data_t *o,uint32_t *gen){*o=chassis_data;if(gen)*gen=1;return 1;}
static void ck(int y,uint32_t n){test_result.checks++;if(!y&&!test_result.failure)test_result.failure=n;}
static void fill(void){health_touch(HEALTH_IMU,tick);health_touch(HEALTH_YAW,tick);health_touch(HEALTH_PITCH,tick);}
void test_entry(void){
 gimbal_state_t state;health_reset();
 ck(!gimbal_get_state(&state,NULL),501);
 fill();yaw_motor.radian_of_ecd=1;pitch_motor.radian_of_ecd=-2;angles[0]=0.5f;ck(gimbal_get_state(&state,NULL),502);
 ck(state.enc_yaw>57000000&&state.enc_pitch< -114000000,503);
 angles[1]=NAN;ck(!gimbal_get_state(&state,NULL),504);angles[1]=0;
 angles[2]=1e20f;ck(!gimbal_get_state(&state,NULL),505);angles[2]=0;
 tick=101;ck(!gimbal_get_state(&state,NULL),506);fill();ck(gimbal_get_state(&state,NULL),507);
 robot_status.robot_id=103;robot_status.power_management_shooter_output=1;projectile_allowance.projectile_allowance_17mm=100;
 g_referee_query_seen=1;g_referee_force_pub=1;comm_publish_referee(100000);
 ck(!(published.status&GIMBAL_REFEREE_STATUS_VALID) && published.fire_allowed==0,508);
 chassis_data.robot_id=3;health_touch(HEALTH_BOARD,tick);g_referee_force_pub=1;comm_publish_referee(200000);
 ck(published.robot_id==3 && published.enemy_team==2 && !(published.status&GIMBAL_REFEREE_STATUS_VALID),509);
 tick+=101;g_referee_force_pub=1;comm_publish_referee(300000);
 ck(published.robot_id==0 && published.fire_allowed==0 && (published.status&GIMBAL_REFEREE_STATUS_TIMEOUT),510);
 ck(!gimbal_get_state(NULL,NULL),511);
 test_result.status=test_result.failure?0xbad:0x600d;__asm("bkpt 0");
}

