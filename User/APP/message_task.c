#include "arm_math.h"
#include "auto_aim.h"
#include "bsp_usart.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "main.h"
#include "message_task.h"
#include "pid.h"
#include "referee.h"
#include "remote_control.h"
#include "robot_param.h"
#include "shoot.h"
#include "user_lib.h"
#include "video_transmission_module.h"


extern VTM_Data_t vtm_data;



//目前发送给底盘的数据：yaw电机由编码值转换的弧度值、imu的yaw值、pitch值、roll值、云台行为模式、射击模式、图传控制链路数据
//0xFE为帧头，0xFD为帧尾
__attribute__((used))void send_data_to_chassis(void)
{
	static fp32 motor_yaw_now;							//yaw电机由编码值转换的弧度值
  static fp32* INS_now_ptr; 							//存储INS指针, 用于发送imu的yaw值和pitch值
	static uint8_t gimbal_behaviour_now;		//云台行为模式
	static uint8_t shoot_mode_now;					//射击模式
	static uint8_t* vtm_data_ptr = (uint8_t*)&vtm_data;		//图传数据
	
  motor_yaw_now = rad_format(gimbal_control.gimbal_yaw_motor.radian_of_ecd);
  INS_now_ptr = get_INS_angle_point();	//0:yaw, 1:pitch, 2:roll 单位:rad
	gimbal_behaviour_now = (uint8_t)gimbal_behaviour;
	extern shoot_control_t shoot_control; 
	shoot_mode_now = shoot_control.shoot_mode;
		
	//length = 2(帧头和帧尾) + 4 + 4*3 + 1 + 1 + 26 = 46
	const static uint8_t length = 46;
	static uint8_t data_chassis[length];		//发送给底盘的数据
	
	/*******************数据段*******************/
  data_chassis[0] = 0XFE;		//帧头
	
	// 提取每个字节
	uint32_t *ptr1 = (uint32_t*)&motor_yaw_now;
	data_chassis[1] = (*ptr1 >>  0) & 0xFF;			// 最低字节
	data_chassis[2] = (*ptr1 >>  8) & 0xFF;			// 第二个字节
	data_chassis[3] = (*ptr1 >> 16) & 0xFF;			// 第三个字节
	data_chassis[4] = (*ptr1 >> 24) & 0xFF;			// 最高字节
	
	uint32_t *ptr2 = (uint32_t*)INS_now_ptr;
	data_chassis[5] = (ptr2[0] >>  0) & 0xFF;		// 最低字节
	data_chassis[6] = (ptr2[0] >>  8) & 0xFF;		// 第二个字节
	data_chassis[7] = (ptr2[0] >> 16) & 0xFF;		// 第三个字节
	data_chassis[8] = (ptr2[0] >> 24) & 0xFF;		// 最高字节
  
	data_chassis[9]  = (ptr2[1] >>  0) & 0xFF;	// 最低字节
	data_chassis[10] = (ptr2[1] >>  8) & 0xFF;	// 第二个字节
	data_chassis[11] = (ptr2[1] >> 16) & 0xFF;	// 第三个字节
	data_chassis[12] = (ptr2[1] >> 24) & 0xFF;	// 最高字节
	
	data_chassis[13] = (ptr2[2] >> 	0) & 0xFF;	// 最低字节
	data_chassis[14] = (ptr2[2] >> 	8) & 0xFF;	// 第二个字节
	data_chassis[15] = (ptr2[2] >> 16) & 0xFF;	// 第三个字节
	data_chassis[16] = (ptr2[2] >> 24) & 0xFF;	// 最高字节
	
	data_chassis[17] = gimbal_behaviour_now;		// 单个字节
	
	data_chassis[18] = shoot_mode_now;					// 单个字节
	
	// Keep the 46-byte frame; unused VTM bytes must not read adjacent globals.
	for(uint8_t i = 0; i < sizeof(vtm_data) - 2U; i ++)
	{
		data_chassis[19 + i] = vtm_data_ptr[2 + i];
	}
	for(uint8_t i = sizeof(vtm_data) - 2U; i < 26U; i ++)
	{
		data_chassis[19 + i] = 0;
	}
	

  data_chassis[length - 1] = 0XFD;  //帧尾
	/*******************************************/
	
	USART1_Transmit_DMA(data_chassis, length);	//DMA发送数据, 长度见上方 length
}


void message_task(void const *pvParameters)
{

    // 初始化延迟
    vTaskDelay(MESSAGE_TASK_INIT_TIME);
    while(1) {
        send_data_to_chassis();
				vTaskDelay(MESSAGE_TASK_TIME);  // 每次循环的延迟 
			
      
    }
}

