#include <string.h>
#include <math.h>
#include "can_bsp.c"
#include "health.c"
#include "remote_control.c"
#include "detect_task.c"
#include "gimbal_task.c"
#include "yaw_pitch_direct.c"
#include "shoot.c"
#include "shoot_3508.c"
#undef LimitMax
#include "pid.c"
#include "user_lib.c"
volatile struct {uint32_t status,checks,failure;} test_result;
FDCAN_HandleTypeDef hfdcan1,hfdcan2;
UART_HandleTypeDef huart1,huart5,huart10;
DMA_HandleTypeDef hdma_usart5_rx,hdma_usart1_tx,hdma_usart1_rx;
uint8_t remote_buff[SBUS_RX_BUF_NUM];
INS_t INS;
volatile pitch_dm_cali_status_t pitch_dm_cali_status;
volatile gimbal_behaviour_e gimbal_behaviour;
auto_aim_t aim;
remoter_t remoter;
static fp32 imu_values[3];
fp32 *get_INS_angle_point(void){return imu_values;}
fp32 *get_gyro_data_point(void){return imu_values;}
void gimbal_behaviour_control_set(fp32*y,fp32*p,gimbal_control_t*c){(void)c;*y=*p=0;}
void gimbal_behaviour_mode_set(gimbal_control_t*c){(void)c;}
void pitch_dm_cali_load(float*a,float*b){*a=-3;*b=0;}
osStatus osDelay(uint32_t t){(void)t;return osOK;}
void vTaskDelay(TickType_t t){(void)t;}
TickType_t xTaskGetTickCount(void){return HAL_GetTick();}
void *pvPortMalloc(size_t n){(void)n;return NULL;}
void Error_Handler(void){__asm("bkpt 0");}
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*h,uint32_t a,uint32_t b){(void)h;(void)a;(void)b;return HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef*h,FDCAN_FilterTypeDef*f){(void)h;(void)f;return HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef*h,uint32_t a,uint32_t b,uint32_t c,uint32_t d){(void)h;(void)a;(void)b;(void)c;(void)d;return HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*h){(void)h;return HAL_OK;}
static uint32_t tick;
static uint8_t local_stop;
static FDCAN_RxHeaderTypeDef rx;
static uint8_t bytes[8],sent_gimbal[8],sent_fric[8],sent_mit[8];
static HAL_StatusTypeDef rx_result,tx_result;
uint32_t HAL_GetTick(void){return tick;}
void Bombbay_on(void){}
void Bombbay_off(void){}
bool_t gimbal_cmd_to_shoot_stop(void){return local_stop;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*h,uint8_t*d,uint16_t n){(void)h;(void)d;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef*h,uint32_t fifo,FDCAN_RxHeaderTypeDef*out,uint8_t*d){(void)h;(void)fifo;*out=rx;memcpy(d,bytes,8);return rx_result;}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef*h,FDCAN_TxHeaderTypeDef*head,uint8_t*d){
 if(h==&hfdcan1 && head->Identifier==0x200)memcpy(sent_fric,d,8);
 if(h==&hfdcan2 && head->Identifier==0x1FF)memcpy(sent_gimbal,d,8);
 if(h==&hfdcan2 && head->Identifier==0x53)memcpy(sent_mit,d,8);
 return tx_result;
}
static void check(uint8_t ok,uint32_t n){test_result.checks++;if(!ok){test_result.failure=n;__asm("bkpt 0");}}
static int zero(const uint8_t*d){uint8_t i;for(i=0;i<8;i++)if(d[i])return 0;return 1;}
static uint8_t mit_free(void){return (((sent_mit[3]&15U)<<8)|sent_mit[4])==0U && ((sent_mit[5]<<4)|(sent_mit[6]>>4))==0U;}
static void fill(uint8_t except){uint8_t i;for(i=0;i<HEALTH_SOURCE_COUNT;i++)if(i!=except && (HEALTH_SHOOT_MASK&HEALTH_BIT(i)))health_touch(i,tick);}
static void healthy(void){
 health_reset();tick=1;fill(255);
 memset(&gimbal_control,0,sizeof(gimbal_control));
 gimbal_control.gimbal_rc_ctrl=&rc_ctrl;
 gimbal_control.gimbal_pitch_motor.gimbal_DM_measure=&DM_MOTOR_MEASURE[2];
 gimbal_control.gimbal_yaw_motor.gimbal_motor_mode=GIMBAL_MOTOR_ENCONDE;
 gimbal_control.gimbal_pitch_motor.gimbal_motor_mode=GIMBAL_MOTOR_ENCONDE;
 gimbal_control.gimbal_pitch_motor.relative_angle=-2.0f;
 gimbal_control.gimbal_pitch_motor.emit_pos_set=-2.0f;
 gimbal_control.gimbal_pitch_motor.relative_angle_set=-2.0f;
 rc_ctrl.rc.s[0]=RC_SW_UP;rc_ctrl.rc.s[1]=RC_SW_DOWN;rc_ctrl.last_fdb=tick;
 DM_MOTOR_MEASURE[2].fdb.pos=-2.0f;DM_MOTOR_MEASURE[2].fdb.id=3;DM_MOTOR_MEASURE[2].fdb.state=1;DM_MOTOR_MEASURE[2].fdb.last_fdb_time=tick;
 pitch_dm_params_init();pitch_dm_cali_status.stored_valid=1;pitch_dm_cali_status.min_pos=-3;pitch_dm_cali_status.max_pos=0;
 memset((void*)&pitch_dm_enable_status,0,sizeof(pitch_dm_enable_status));pitch_dm_enable_status.confirmed=1;
 gimbal_behaviour=GIMBAL_RELATIVE_ANGLE;
 (void)gimbal_axis_ready(&gimbal_control,0);(void)gimbal_axis_ready(&gimbal_control,1);
 gimbal_control.gimbal_yaw_motor.given_current=1234;shoot_can_set_current=321;
}
static void send(void){gimbal_send_cmd(&gimbal_control);CAN_cmd_friction(10,20,0,0);}
void test_entry(void){
 uint8_t i;health_status_t st;health_bus_stats_t bs;
 hfdcan1.Instance=FDCAN1;hfdcan2.Instance=FDCAN2;rx_result=tx_result=HAL_OK;
 memset(&rx,0,sizeof(rx));rx.DataLength=FDCAN_DLC_BYTES_8;
 healthy();send();check(sent_gimbal[0]==4 && sent_gimbal[1]==210 && !zero(sent_fric) && !mit_free(),1);
 for(i=0;i<HEALTH_SOURCE_COUNT;i++)if(HEALTH_SHOOT_MASK&HEALTH_BIT(i)){
  healthy();send();
  tick=102;fill(i);send();
  check(zero(sent_fric) && sent_gimbal[4]==0 && sent_gimbal[5]==0,100+i);
  if(i==HEALTH_RC || i==HEALTH_YAW)check(sent_gimbal[0]==0 && sent_gimbal[1]==0,120+i);
  else check(sent_gimbal[0]==4 && sent_gimbal[1]==210,140+i);
  if(i==HEALTH_RC || i==HEALTH_PITCH)check(mit_free() && !pitch_dm_enable_status.mit_target_valid,160+i);
  health_touch(i,tick);
  if(i!=HEALTH_RC){send();check(zero(sent_fric),180+i);}
  health_touch(HEALTH_RC,tick);send();check(!zero(sent_fric),200+i);
 }
 healthy();health_reject(HEALTH_RC);
 shoot_control.shoot_rc=&rc_ctrl;shoot_control.given_current=123;shoot_control.fric1.give_current=456;
 shoot_control.trigger_motor_speed_pid.Iout=88;shoot_control.fric_motor_pid.Iout=99;
 check(shoot_control_loop()==0 && shoot_control.given_current==0 && shoot_control.fric1.give_current==0 &&
  shoot_control.trigger_motor_speed_pid.Iout==0 && shoot_control.fric_motor_pid.Iout==0 && zero(sent_fric),240);
 healthy();gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.Iout=99;
 health_reject(HEALTH_YAW);send();check(gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.Iout==0,241);
 /* Final wrappers guard independent callers, not only the high-level task. */
 health_reset();CAN_cmd_gimbal(999,999,999,999);CAN_cmd_friction(9,8,7,6);
 check(zero(sent_gimbal)&&zero(sent_fric),242);
 tick=0;rx.Identifier=CAN_3508_M1_ID;memset(bytes,0,8);
 HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
 health_snapshot(HEALTH_FRIC1,0,&st);check(st.seen && st.generation==1 && st.last_valid_ms==0,250);
 HAL_FDCAN_RxFifo1Callback(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
 rx.IdType=FDCAN_EXTENDED_ID;HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);rx.IdType=FDCAN_STANDARD_ID;
 rx.RxFrameType=FDCAN_REMOTE_FRAME;HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);rx.RxFrameType=FDCAN_DATA_FRAME;
 rx.DataLength=FDCAN_DLC_BYTES_7;HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);rx.DataLength=FDCAN_DLC_BYTES_8;
 rx_result=HAL_ERROR;HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);rx_result=HAL_OK;
 health_snapshot(HEALTH_FRIC1,0,&st);check(st.generation==1,251);
 rx.Identifier=CAN_YAW_MOTOR_ID;HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
 health_snapshot(HEALTH_YAW,0,&st);check(!st.seen,252);
 HAL_FDCAN_RxFifo1Callback(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
 health_snapshot(HEALTH_YAW,0,&st);check(st.seen&&st.last_valid_ms==0,253);
 rx.Identifier=DM_M3_ID;bytes[0]=0x13;HAL_FDCAN_RxFifo1Callback(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
 health_snapshot(HEALTH_PITCH,0,&st);check(st.seen&&st.fresh,254);
 bytes[0]=0x83;HAL_FDCAN_RxFifo1Callback(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
 health_snapshot(HEALTH_PITCH,0,&st);check(!st.fresh&&st.generation==1,255);
 healthy();tx_result=HAL_BUSY;CAN_cmd_friction(1,2,0,0);health_bus_snapshot(0,&bs);check(bs.tx_failed==1&&bs.tx_queued==0,260);
 test_result.status=0x600D;__asm("bkpt 0");
}

