
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
typedef float fp32;
typedef union { float fp32; uint8_t bytes[4]; } fp32_to_bytes;
#pragma pack(1)                      
                       
typedef struct
{
                  
    uint8_t head1;                               
    uint8_t head2;                               
    
                          
    uint16_t ch0 : 11;                    
    uint16_t ch1 : 11;                    
    uint16_t ch2 : 11;                    
    uint16_t ch3 : 11;                    
    
                    
    uint8_t  gear_switch : 2;                      
    
                       
    uint8_t  pause_key : 1;                
    uint8_t  custom_l_key : 1;                
    uint8_t  custom_r_key : 1;                
    
                 
    uint16_t wheel : 11;                   
    
                 
    uint8_t  trigger_key : 1;              
    
                             
    int16_t  mouse_x;                         
    int16_t  mouse_y;                         
    int16_t  mouse_z;                          
    
                      
    uint8_t  mouse_l_key : 2;                
    uint8_t  mouse_r_key : 2;                
    uint8_t  mouse_m_key : 2;                
    
                                  
    uint16_t keyboard_key;                     
    
                              
    uint16_t crc;                              
} VTM_Data_t;
#pragma pack()
typedef char vtm_size_check[(sizeof(VTM_Data_t) == 21) ? 1 : -1];
typedef char vtm_crc_check[(offsetof(VTM_Data_t, crc) == 19) ? 1 : -1];
struct { uint8_t before[8]; VTM_Data_t value; uint8_t after[8]; } vtm_guard;
struct { struct { float radian_of_ecd; } gimbal_yaw_motor; } gimbal_control;
typedef struct { uint8_t shoot_mode; } shoot_control_t;
shoot_control_t shoot_control;
uint8_t gimbal_behaviour;
typedef struct {
  float chassis_vx, chassis_vy, chassis_wz;
  float chassis_yaw, chassis_pitch, chassis_roll, cap_voltage;
  uint8_t chassis_mode, robot_id;
} chassis_data_t;
#define CHASSIS_DATA_LENGTH 28
#define CHASSIS_FRAME_HEAD 's'
#define CHASSIS_FRAME_TAIL 'e'
#define CHASSIS_ANGLE_LIMIT_RAD 3.2f
#define CHASSIS_SPEED_LIMIT_MPS 10.0f
#define CHASSIS_WZ_LIMIT_RADPS 50.0f
#define CHASSIS_MODE_MAX 2u
float ins_angles[3];
float *get_INS_angle_point(void) { return ins_angles; }
uint8_t frame46[46];
uint16_t frame46_size;
void USART1_Transmit_DMA(uint8_t *bytes, uint16_t size) {
  frame46_size = size;
  if (size <= sizeof(frame46)) memcpy(frame46, bytes, size);
}
float loop_float_constrain(float Input, float minValue, float maxValue){
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}
void vtm_data_init(void){

    

    vtm_guard.value.head1 = 0xA9;

    vtm_guard.value.head2 = 0x53;

    vtm_guard.value.ch0 = 1024;

    vtm_guard.value.ch1 = 1024;

    vtm_guard.value.ch2 = 1024;

    vtm_guard.value.ch3 = 1024;

    vtm_guard.value.gear_switch = 1;

    vtm_guard.value.pause_key = 0;

    vtm_guard.value.custom_l_key = 0;

    vtm_guard.value.custom_r_key = 0;

    vtm_guard.value.wheel = 1024;

    vtm_guard.value.trigger_key = 0;

    vtm_guard.value.mouse_x = 0;

    vtm_guard.value.mouse_y = 0;

    vtm_guard.value.mouse_z = 0;

    vtm_guard.value.mouse_l_key = 0;

    vtm_guard.value.mouse_r_key = 0;

    vtm_guard.value.mouse_m_key = 0;

    vtm_guard.value.keyboard_key = 0;

    vtm_guard.value.crc = 0;

}
void send_data_to_chassis(void){
	static fp32 motor_yaw_now;							
  static fp32* INS_now_ptr; 							
	static uint8_t gimbal_behaviour_now;		
	static uint8_t shoot_mode_now;					
	static uint8_t* vtm_data_ptr = (uint8_t*)&vtm_guard.value;		
	
  motor_yaw_now = loop_float_constrain((gimbal_control . gimbal_yaw_motor . radian_of_ecd), -3.14159265358979f, 3.14159265358979f);
  INS_now_ptr = get_INS_angle_point();	
	gimbal_behaviour_now = (uint8_t)gimbal_behaviour;
	extern shoot_control_t shoot_control; 
	shoot_mode_now = shoot_control.shoot_mode;
		
	
	const static uint8_t length = 46;
	static uint8_t data_chassis[length];		
	
	 
  data_chassis[0] = 0XFE;		
	
	
	uint32_t *ptr1 = (uint32_t*)&motor_yaw_now;
	data_chassis[1] = (*ptr1 >>  0) & 0xFF;			
	data_chassis[2] = (*ptr1 >>  8) & 0xFF;			
	data_chassis[3] = (*ptr1 >> 16) & 0xFF;			
	data_chassis[4] = (*ptr1 >> 24) & 0xFF;			
	
	uint32_t *ptr2 = (uint32_t*)INS_now_ptr;
	data_chassis[5] = (ptr2[0] >>  0) & 0xFF;		
	data_chassis[6] = (ptr2[0] >>  8) & 0xFF;		
	data_chassis[7] = (ptr2[0] >> 16) & 0xFF;		
	data_chassis[8] = (ptr2[0] >> 24) & 0xFF;		
  
	data_chassis[9]  = (ptr2[1] >>  0) & 0xFF;	
	data_chassis[10] = (ptr2[1] >>  8) & 0xFF;	
	data_chassis[11] = (ptr2[1] >> 16) & 0xFF;	
	data_chassis[12] = (ptr2[1] >> 24) & 0xFF;	
	
	data_chassis[13] = (ptr2[2] >> 	0) & 0xFF;	
	data_chassis[14] = (ptr2[2] >> 	8) & 0xFF;	
	data_chassis[15] = (ptr2[2] >> 16) & 0xFF;	
	data_chassis[16] = (ptr2[2] >> 24) & 0xFF;	
	
	data_chassis[17] = gimbal_behaviour_now;		
	
	data_chassis[18] = shoot_mode_now;					
	
	
	for(uint8_t i = 0; i < sizeof(vtm_guard.value) - 2U; i ++)
	{
		data_chassis[19 + i] = vtm_data_ptr[2 + i];
	}
	for(uint8_t i = sizeof(vtm_guard.value) - 2U; i < 26U; i ++)
	{
		data_chassis[19 + i] = 0;
	}
	

  data_chassis[length - 1] = 0XFD;  
	 
	
	USART1_Transmit_DMA(data_chassis, length);	
}
static uint8_t chassis_frame_parse(const uint8_t *frame, chassis_data_t *out){
	fp32_to_bytes field;

	if (frame[0] != 's')
	{
		return 1U;
	}
	if (frame[28 - 1] != 'e')
	{
		return 2U;
	}

	field.bytes[0] = frame[1];
	field.bytes[1] = frame[2];
	field.bytes[2] = frame[3];
	field.bytes[3] = frame[4];
	out->chassis_yaw = field.fp32;

	field.bytes[0] = frame[5];
	field.bytes[1] = frame[6];
	field.bytes[2] = frame[7];
	field.bytes[3] = frame[8];
	out->chassis_pitch = field.fp32;

	field.bytes[0] = frame[9];
	field.bytes[1] = frame[10];
	field.bytes[2] = frame[11];
	field.bytes[3] = frame[12];
	out->chassis_roll = field.fp32;

	out->chassis_mode = frame[13];
	out->robot_id = frame[14];

	field.bytes[0] = frame[15];
	field.bytes[1] = frame[16];
	field.bytes[2] = frame[17];
	field.bytes[3] = frame[18];
	out->chassis_vx = field.fp32;

	field.bytes[0] = frame[19];
	field.bytes[1] = frame[20];
	field.bytes[2] = frame[21];
	field.bytes[3] = frame[22];
	out->chassis_vy = field.fp32;

	field.bytes[0] = frame[23];
	field.bytes[1] = frame[24];
	field.bytes[2] = frame[25];
	field.bytes[3] = frame[26];
	out->chassis_wz = field.fp32;

	 
	if (!((sizeof(out->chassis_yaw) == sizeof(float)) ? __ARM_isfinitef(out->chassis_yaw) : __ARM_isfinite(out->chassis_yaw)) || !((sizeof(out->chassis_pitch) == sizeof(float)) ? __ARM_isfinitef(out->chassis_pitch) : __ARM_isfinite(out->chassis_pitch)) ||
	    !((sizeof(out->chassis_roll) == sizeof(float)) ? __ARM_isfinitef(out->chassis_roll) : __ARM_isfinite(out->chassis_roll)) || !((sizeof(out->chassis_vx) == sizeof(float)) ? __ARM_isfinitef(out->chassis_vx) : __ARM_isfinite(out->chassis_vx)) ||
	    !((sizeof(out->chassis_vy) == sizeof(float)) ? __ARM_isfinitef(out->chassis_vy) : __ARM_isfinite(out->chassis_vy)) || !((sizeof(out->chassis_wz) == sizeof(float)) ? __ARM_isfinitef(out->chassis_wz) : __ARM_isfinite(out->chassis_wz)))
	{
		return 3U;
	}
	if (fabsf(out->chassis_yaw) > 3.2f ||
	    fabsf(out->chassis_pitch) > 3.2f ||
	    fabsf(out->chassis_roll) > 3.2f)
	{
		return 3U;
	}
	if (fabsf(out->chassis_vx) > 10.0f ||
	    fabsf(out->chassis_vy) > 10.0f ||
	    fabsf(out->chassis_wz) > 50.0f)
	{
		return 3U;
	}
	if (out->chassis_mode > 2u)
	{
		return 3U;
	}
	 
	return 0U;
}

volatile struct { uint32_t status, checks, failure; } test_result;
__asm void stop_test(void) {
  BKPT 0
  BX lr
}
void check(int ok, uint32_t code) {
  ++test_result.checks;
  if (!ok) { test_result.failure = code; stop_test(); }
}
void test_entry(void) {
  uint8_t incoming[28] = {'s'};
  uint8_t *vtm = (uint8_t *)&vtm_guard.value;
  float expected;
  memset(&vtm_guard, 0xCD, sizeof(vtm_guard));
  vtm_data_init();
  check(vtm_guard.value.head1 == 0xA9 && vtm_guard.value.head2 == 0x53, 1);
  check(vtm_guard.value.ch0 == 1024 && vtm_guard.value.ch3 == 1024, 2);
  check(vtm_guard.value.gear_switch == 1 && vtm_guard.value.wheel == 1024, 3);
  for (uint32_t i = 2; i < sizeof(vtm_guard.value); ++i) vtm[i] = 0x40U + i;
  gimbal_control.gimbal_yaw_motor.radian_of_ecd = 1.25f;
  ins_angles[0] = -2.0f; ins_angles[1] = 0.375f; ins_angles[2] = -0.5f;
  gimbal_behaviour = 5; shoot_control.shoot_mode = 4;
  send_data_to_chassis();
  check(frame46_size == 46 && frame46[0] == 0xFE && frame46[45] == 0xFD, 4);
  check(!memcmp(frame46 + 1, &gimbal_control.gimbal_yaw_motor.radian_of_ecd, 4), 5);
  check(!memcmp(frame46 + 5, ins_angles, 12), 6);
  check(frame46[17] == 5 && frame46[18] == 4, 7);
  check(!memcmp(frame46 + 19, vtm + 2, 19), 8);
  for (uint32_t i = 38; i <= 44; ++i) check(frame46[i] == 0, 9);
  for (uint32_t i = 0; i < 8; ++i) {
    check(vtm_guard.before[i] == 0xCD && vtm_guard.after[i] == 0xCD, 10);
  }
  expected = 0.125f; memcpy(incoming + 1, &expected, 4);
  expected = -1.0f; memcpy(incoming + 5, &expected, 4);
  expected = 0.5f; memcpy(incoming + 9, &expected, 4);
  incoming[13] = 2; incoming[14] = 103;
  expected = 1.0f; memcpy(incoming + 15, &expected, 4);
  expected = -0.25f; memcpy(incoming + 19, &expected, 4);
  expected = 0.75f; memcpy(incoming + 23, &expected, 4);
  incoming[27] = 'e';
  chassis_data_t parsed;
  memset(&parsed, 0, sizeof(parsed));
  check(chassis_frame_parse(incoming, &parsed) == 0, 11);
  check(parsed.chassis_yaw == 0.125f && parsed.chassis_pitch == -1.0f &&
        parsed.chassis_roll == 0.5f, 12);
  check(parsed.chassis_mode == 2 && parsed.robot_id == 103, 13);
  check(parsed.chassis_vx == 1.0f && parsed.chassis_vy == -0.25f &&
        parsed.chassis_wz == 0.75f, 14);
  test_result.status = 0x600D;
  stop_test();
}
