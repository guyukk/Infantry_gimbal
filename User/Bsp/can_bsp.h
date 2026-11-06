#ifndef CAN_BSP_H
#define CAN_BSP_H

#include "struct_typedef.h"
#include "main.h"
#include "fdcan.h"

#define CHASSIS_CAN hfdcan1  //4p
#define GIMBAL_CAN hfdcan2   //can1 2p

//和底盘用一条can，但是摩擦轮控制在云台板上
#define FRICTION_CAN hfdcan1
extern uint8_t              chassis_can_send_data[8];
extern uint8_t              gimbal_can_send_data[8];
extern uint8_t							MOTOR_Data[8];

/*
 * DM电机发送数据帧是看slave(不同模式对应id偏移)
 * master_id是电机反馈帧的id，电机发给单片机的(不受电机模式影响)
 * can_id是电机接收的id，是单片机发给电机的（实际id = can_id + slave）

 * 举个例子，DM_mode == 0x00，电机在MIT模式下，can_id 设置为0x08，master_id为DM_M1_ID
 * 那么在发送给电机的时候选择mit模式，id为0x08（代码会自动计算发送的id， 0x00 + 0x08）
 * 在can中断回调函数就需要接收id为DM_M1_ID的数据

 * 防止和其他品牌（类似于翎控）这种有固定id偏移的电机冲突，此处的地址做了一个偏移
*/
/*
 * CAN send and receive ID
 * 注意: 枚举中存在重复值是有意为之 —— 不同 CAN 总线上不同名称的电机可能映射到相同 ID，
 * 因为它们分布在不同的物理 CAN 总线上（如 CAN_6020_M1_ID 和 CAN_YAW_MOTOR_ID 同为 0x205）。
 * DM_mode 是当前 DM MIT 指令的 ID 偏移量，不与 DJI 电机 ID 混用。
 */
typedef enum
{
	// DM 电机各模式的 ID 偏移量
	DM_mode = 0x00,

	//底盘板上的两条can线使用
	CAN_3508_ALL_ID = 0x200,
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	CAN_3508_M3_ID = 0x203,
	CAN_3508_M4_ID = 0x204,

	CAN_6020_ALL_ID = 0x1FF,
	CAN_6020_M1_ID = 0x205,
	CAN_6020_M2_ID = 0x206,
	CAN_6020_M3_ID = 0x207,
	CAN_6020_M4_ID = 0x208,
	
	//云台的pitch,yaw,拨蛋电机
	CAN_GIMBAL_ALL_ID = 0x1FF,
	CAN_YAW_MOTOR_ID = 0x205,
	CAN_PIT_MOTOR_ID = 0x206,
	CAN_TRIGGER_MOTOR_ID = 0x207,

	//摩擦轮电机,和底盘电机id一致
	CAN_FRICTION_ID = 0X200,
	CAN_FRIC1_MOTOR_ID = 0x201,
	CAN_FRIC2_MOTOR_ID = 0x202,
	
	CAN_FRIC3_MOTOR_ID = 0x203,
	CAN_FRIC4_MOTOR_ID = 0x204,
	
	//DM电机的反馈id
	DM_M1_ID = 0x51,
	DM_M2_ID = 0x52,
	DM_M3_ID = 0x53,
	DM_M4_ID = 0x54,
	DM_M5_ID = 0x55,
	DM_M6_ID = 0x56,
	DM_M7_ID = 0x57,
	DM_M8_ID = 0x58,
	//不够就自己再写，最多到0x100
		
} can_msg_id_e;

//dji电机结构体
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
	uint32_t last_fdb_time;

} dji_measure_t;

//dm电机结构体
//注意：大喵电机在上位机修改模式后，ID会被重置，需要重新设置(截至在2025年10月10日，2325电机上发生该现象)
typedef struct
{
	__packed struct
	{
		//原始数据
		int id;
		int state;
		int p_int;
		int v_int;
		int t_int;
		int kp_int;
		int kd_int;
		//计算后的数据
		float pos;
		float vel;
		float tor; //电机反馈的力矩
		float Kp;
		float Kd;
		float t_mos; //mos温度
		float t_motor; //电机温度
		uint32_t last_fdb_time; //电机反馈时间
	}fdb;
	
	__packed struct
	{
		fp32 KP;
		fp32 KD;
		fp32 POS;
		fp32 VEL;
		fp32 TOR;
	}set;

	__packed struct
	{
		fp32	P_MIN;
		fp32	P_MAX;
		fp32	V_MIN;
		fp32	V_MAX;
		fp32	KP_MIN;
		fp32	KP_MAX;
		fp32	KD_MIN;
		fp32	KD_MAX;
		fp32	T_MIN;
		fp32	T_MAX;
	}param;
	
} dm_measure_t;

extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_cmd_friction(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern const dji_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const dji_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const dji_measure_t *get_trigger_motor_measure_point(void);
extern const dji_measure_t *get_fric1_motor_measure_point(void);
extern const dji_measure_t *get_fric2_motor_measure_point(void);
extern const dji_measure_t *get_fric1__motor_measure_point(void);
extern const dji_measure_t *get_fric2__motor_measure_point(void);

extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_chassis_6020(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern const dji_measure_t *get_chassis_motor_measure_point(uint8_t i);

extern uint8_t CAN_cmd_MIT(FDCAN_HandleTypeDef *hcan, uint16_t id, const dm_measure_t *motor);

extern void RS_MOTOR_PRE(FDCAN_HandleTypeDef *hcan,uint16_t id);
extern void RS_Motor_MIT(FDCAN_HandleTypeDef *hcan,uint16_t id);

extern uint8_t Motor_enable(FDCAN_HandleTypeDef *hcan, uint16_t id);
extern void Motor_disable(FDCAN_HandleTypeDef *hcan, uint16_t id);
extern void Motor_save_zero(FDCAN_HandleTypeDef *hcan, uint16_t id);
extern void Motor_clear_err(FDCAN_HandleTypeDef *hcan, uint16_t id);
extern dm_measure_t *get_dm_motor_measure_point(uint8_t i);

extern void FDCAN1_Config(void);
extern void FDCAN2_Config(void);
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x, float x_min, float x_max, int bits);


#endif
