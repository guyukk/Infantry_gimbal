#define test_entry output_gate_fixture_entry
#include "test_b03_gimbal_guard.c"
#undef test_entry
#include "User/Communication/channel/gimbal/gimbal_channel.h"
/* B03_COMM_HOOKS */
static void ready(void){
 healthy();local_stop=0;test_aim_ready=1;
 rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]=RC_SW_UP;rc_ctrl.key.v=0;
 shoot_control.shoot_rc=&rc_ctrl;shoot_control.shoot_mode=SHOOT_READY_BULLET;
 shoot_control.press_l=shoot_control.last_press_l=0;
 shoot_history_reset();
}
static void command(int32_t on,int32_t mode,int32_t burst,uint64_t token){
 gimbal_fire_cmd_t cmd;memset(&cmd,0,sizeof(cmd));cmd.fire_on=on;cmd.fire_mode=mode;cmd.burst_count=burst;cmd.ts_us=token;
 gimbal_on_fire(&cmd,NULL);
}
void test_entry(void){
 host_fire_request_t request;
 host_control_stats_t stats;
 RC_ctrl_t before;
 gimbal_chassis_cmd_t chassis;
 ready();before=rc_ctrl;memset(&chassis,0,sizeof(chassis));chassis.vx_mm_s=2000;chassis.mode=2;
 gimbal_on_chassis(&chassis,NULL);comm_apply_host_controls(1);
 host_control_get_stats(&stats);check(stats.chassis_unsupported==1 && memcmp(&rc_ctrl,&before,sizeof(before))==0,1);
 command(1,2,5,1);shoot_set_mode();
 check(shoot_control.shoot_mode==SHOOT_CONTINUE_BULLET && memcmp(&before,&rc_ctrl,sizeof(before))==0,2);
 tick=122;fill(255);comm_apply_host_controls(122000);shoot_set_mode();
 check(shoot_control.shoot_mode==SHOOT_READY_BULLET && memcmp(&before,&rc_ctrl,sizeof(before))==0,3);
 ready();command(1,1,1,42);command(1,1,1,42);
 check(host_fire_take(&request) && request.single && !request.continuous,10);
 check(!host_fire_take(&request),11);
 host_fire_cancel();command(1,1,1,42);check(!host_fire_take(&request),12);
 command(1,1,1,43);check(host_fire_take(&request)&&request.single,13);
 command(0,0,0,0);command(1,1,1,0);check(host_fire_take(&request)&&request.single,14);
 command(1,1,1,0);check(!host_fire_take(&request),15);
 command(0,0,0,0);command(1,1,1,0);check(host_fire_take(&request)&&request.single,16);
 ready();command(1,2,5,50);local_stop=1;shoot_set_mode();
 check(shoot_control.shoot_mode==SHOOT_STOP && !host_fire_take(&request),20);
 local_stop=0;shoot_control.shoot_mode=SHOOT_READY_BULLET;shoot_set_mode();
 check(shoot_control.shoot_mode==SHOOT_READY_BULLET && !host_fire_take(&request),21);
 ready();command(1,2,5,51);rc_ctrl.key.v=KEY_PRESSED_OFFSET_E;before=rc_ctrl;shoot_set_mode();
 check(shoot_control.shoot_mode==SHOOT_STOP && !host_fire_take(&request) && memcmp(&before,&rc_ctrl,sizeof(before))==0,22);
 ready();command(1,2,5,52);shoot_set_mode();shoot_control.press_l=1;rc_ctrl.mouse.press_l=1;shoot_set_mode();
 check(!host_fire_take(&request)&&rc_ctrl.mouse.press_l==1,23);
 ready();command(1,2,5,53);test_aim_ready=0;comm_apply_host_controls(0);check(!host_fire_take(&request),24);
 test_aim_ready=1;check(!host_fire_take(&request),25);
 ready();command(1,2,5,54);health_reject(HEALTH_RC);check(!host_fire_take(&request),26);
 fill(255);check(!host_fire_take(&request),27);
 ready();tick=0xFFFFFFF0U;fill(255);command(1,2,5,60);
 tick=104;fill(255);check(host_fire_take(&request)&&request.continuous,30);
 tick=105;fill(255);check(!host_fire_take(&request),31);
 ready();command(1,1,1,61);tick+=20;fill(255);check(host_fire_take(&request)&&request.single,32);
 ready();command(1,1,1,62);tick+=21;fill(255);check(!host_fire_take(&request),33);
 ready();command(2,5,-1,63);check(!host_fire_take(&request),34);
 test_result.status=0x600D;__asm("bkpt 0");
}

