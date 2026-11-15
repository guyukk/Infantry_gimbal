#include "can_bsp.h"
#include "fdcan.h"
#include "health.h"
#include "string.h"

FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

uint8_t chassis_can_send_data[8];
uint8_t gimbal_can_send_data[8];
uint8_t shoot_can_send_data[8];

uint8_t MOTOR_Data[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 电机数据

// 达妙电机命令
uint8_t MOTOR_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};   // 电机使能命令
uint8_t MOTOR_Failure[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};  // 电机失能命令
uint8_t MOTOR_Save_zero[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; // 电机保存零点命令
uint8_t MOTOR_Clear_err[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB}; // 电机清除错误命令

uint8_t MOTOR_MIT_MODE[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC};	//电机切换到MIT模式
uint8_t MOTOR_POS_MODE[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0xFC};	//电机切换到位置模式
uint8_t MOTOR_VEL_MODE[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0xFC};	//电机切换到速度模式

uint8_t RS_MOTOR_PRE_MODE[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFD}; // 灵足电机私有模式
uint8_t RS_MOTOR_MIT_MODE[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0xFD}; // 灵足电机 MIT 模式
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

//dm电机速度滤波缓冲（抑制近似正弦噪声）
static float dm_vel_lpf[8] = {0.0f};

//dji motor data read（宏改为内联函数）
static inline void get_motor_measure(dji_measure_t *ptr, const uint8_t data[8])
{
	ptr->last_ecd = ptr->ecd;
	ptr->ecd = (uint16_t)((data[0] << 8) | data[1]);
	ptr->speed_rpm = (uint16_t)((data[2] << 8) | data[3]);
	ptr->given_current = (uint16_t)((data[4] << 8) | data[5]);
	ptr->temperate = data[6];
}

/**
 * @brief  DMFdbData: 获取 DM 电机反馈数据（内联），含速度低通滤波
 * @note   vel 噪声近似正弦，使用一阶低通平滑
 */
static inline void DMFdbData(dm_measure_t *DM_measure, const uint8_t rx_data[8], uint8_t cnt)
{
	DM_measure->fdb.id = (rx_data[0]) & 0x0F;
	DM_measure->fdb.state = (rx_data[0]) >> 4;
	DM_measure->fdb.p_int = ((rx_data[1] << 8) | rx_data[2]);
	DM_measure->fdb.v_int = ((rx_data[3] << 4) | (rx_data[4] >> 4));
	DM_measure->fdb.t_int = (((rx_data[4] & 0xF) << 8) | rx_data[5]);
	DM_measure->fdb.pos = uint_to_float(DM_measure->fdb.p_int, DM_measure->param.P_MIN, DM_measure->param.P_MAX, 16);

	const float vel_raw = uint_to_float(DM_measure->fdb.v_int, DM_measure->param.V_MIN, DM_measure->param.V_MAX, 12);
	uint8_t idx = cnt;
	const float alpha = 0.15f; // 一阶低通滤波系数
	dm_vel_lpf[idx] = dm_vel_lpf[idx] + alpha * (vel_raw - dm_vel_lpf[idx]);
	DM_measure->fdb.vel = dm_vel_lpf[idx];

	DM_measure->fdb.tor = uint_to_float(DM_measure->fdb.t_int, DM_measure->param.T_MIN, DM_measure->param.T_MAX, 12);
	DM_measure->fdb.t_mos = (float)(rx_data[6]);
	DM_measure->fdb.t_motor = (float)(rx_data[7]);
}

/*
 * DJI 电机反馈数据数组
 * 索引映射: [0]~[3] 底盘 3508 / 摩擦轮 1~4; [4] yaw 6020; [5] 未用; [6] trigger 2006; [7] 保留
 */
dji_measure_t DJI_MOTOR_MEASURE[8];

/*
 * DM 电机反馈数据数组, 索引 [0]~[7] 对应 DM_M1~DM_M8
 */
dm_measure_t DM_MOTOR_MEASURE[8];

// 前向声明（定义在本文件末尾）
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t can1_cnt = 0; // CAN1 中断回调中临时电机索引
uint8_t can2_cnt = 0; // CAN2 中断回调中临时电机索引

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const dji_measure_t *get_yaw_gimbal_motor_measure_point(void){
	return &DJI_MOTOR_MEASURE[4];
}

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const dji_measure_t *get_trigger_motor_measure_point(void){
	return &DJI_MOTOR_MEASURE[6];
}

/**
  * @brief          返回摩擦轮1电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const dji_measure_t *get_fric1_motor_measure_point(void){
	return &DJI_MOTOR_MEASURE[0];
}

/**
  * @brief          返回摩擦轮2电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const dji_measure_t *get_fric2_motor_measure_point(void){
	return &DJI_MOTOR_MEASURE[1];
}

/**
  * @brief          返回摩擦轮3电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const dji_measure_t *get_fric1__motor_measure_point(void){
	return &DJI_MOTOR_MEASURE[2];
}

/**
  * @brief          返回摩擦轮4电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const dji_measure_t *get_fric2__motor_measure_point(void){
	return &DJI_MOTOR_MEASURE[3];
}

/**
  * @brief          发送云台电机控制电流（广播帧 ID=CAN_GIMBAL_ALL_ID=0x1FF）
  * @param[in]      yaw:    6020电机控制电流, 对应帧内偏移 0x205, 范围 [-25000,25000]
  * @param[in]      pitch:  6020电机控制电流, 对应帧内偏移 0x206, 范围 [-25000,25000]
  * @param[in]      shoot:  2006电机控制电流, 对应帧内偏移 0x207, 范围 [-10000,10000]
  * @param[in]      rev:    保留，对应帧内偏移 0x208
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
		canx_send_data(&GIMBAL_CAN, CAN_GIMBAL_ALL_ID, gimbal_can_send_data, 8);
}

/**
  * @brief          发送底盘轮向电机控制电流（广播帧 ID=CAN_3508_ALL_ID=0x200）
  * @param[in]      motor1: 3508电机控制电流, 对应帧内偏移 0x201, 范围 [-16384,16384]
  * @param[in]      motor2: 3508电机控制电流, 对应帧内偏移 0x202, 范围 [-16384,16384]
  * @param[in]      motor3: 3508电机控制电流, 对应帧内偏移 0x203, 范围 [-16384,16384]
  * @param[in]      motor4: 3508电机控制电流, 对应帧内偏移 0x204, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
		canx_send_data(&CHASSIS_CAN, CAN_3508_ALL_ID, chassis_can_send_data, 8);
}

/**
  * @brief          发送底盘舵向电机控制电流（广播帧 ID=CAN_6020_ALL_ID=0x1FF）
  * @param[in]      motor1: 6020电机控制电流, 对应帧内偏移 0x205, 范围 [-25000,25000]
  * @param[in]      motor2: 6020电机控制电流, 对应帧内偏移 0x206, 范围 [-25000,25000]
  * @param[in]      motor3: 6020电机控制电流, 对应帧内偏移 0x207, 范围 [-25000,25000]
  * @param[in]      motor4: 6020电机控制电流, 对应帧内偏移 0x208, 范围 [-25000,25000]
  * @retval         none
  */
void CAN_cmd_chassis_6020(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
		canx_send_data(&CHASSIS_CAN, CAN_6020_ALL_ID, chassis_can_send_data, 8);
}

/**
  * @brief          发送摩擦轮电机控制电流（广播帧 ID=CAN_3508_ALL_ID=0x200）
  * @param[in]      motor1: 3508电机控制电流, 对应帧内偏移 0x201, 范围 [-16384,16384]
  * @param[in]      motor2: 3508电机控制电流, 对应帧内偏移 0x202, 范围 [-16384,16384]
  * @param[in]      motor3: 3508电机控制电流, 对应帧内偏移 0x203, 范围 [-16384,16384]
  * @param[in]      motor4: 3508电机控制电流, 对应帧内偏移 0x204, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_friction(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    shoot_can_send_data[0] = motor1 >> 8;
    shoot_can_send_data[1] = motor1;
    shoot_can_send_data[2] = motor2 >> 8;
    shoot_can_send_data[3] = motor2;
    shoot_can_send_data[4] = motor3 >> 8;
    shoot_can_send_data[5] = motor3;
    shoot_can_send_data[6] = motor4 >> 8;
    shoot_can_send_data[7] = motor4;
		canx_send_data(&FRICTION_CAN, CAN_3508_ALL_ID, shoot_can_send_data, 8);
}

static uint16_t dm_mit_encode(float value, float lower, float upper, uint8_t bits)
{
	uint32_t maximum = (1U << bits) - 1U;
	if (value < lower) value = lower;
	if (value > upper) value = upper;
	return (uint16_t)((value - lower) * (float)maximum / (upper - lower));
}

/** MIT frame: [p:16][v:12][Kp:12][Kd:12][t_ff:12], standard ID = motor CAN ID. */
uint8_t CAN_cmd_MIT(FDCAN_HandleTypeDef *hcan, uint16_t id, const dm_measure_t *motor)
{
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	if (hcan == NULL || motor == NULL ||
		!(motor->param.P_MIN < motor->param.P_MAX) ||
		!(motor->param.V_MIN < motor->param.V_MAX) ||
		!(motor->param.KP_MIN < motor->param.KP_MAX) ||
		!(motor->param.KD_MIN < motor->param.KD_MAX) ||
		!(motor->param.T_MIN < motor->param.T_MAX) ||
		motor->set.POS != motor->set.POS || motor->set.VEL != motor->set.VEL ||
		motor->set.KP != motor->set.KP || motor->set.KD != motor->set.KD ||
		motor->set.TOR != motor->set.TOR)
	{
		return (uint8_t)HAL_ERROR;
	}

	pos_tmp = dm_mit_encode(motor->set.POS, motor->param.P_MIN, motor->param.P_MAX, 16);
	vel_tmp = dm_mit_encode(motor->set.VEL, motor->param.V_MIN, motor->param.V_MAX, 12);
	kp_tmp = dm_mit_encode(motor->set.KP, motor->param.KP_MIN, motor->param.KP_MAX, 12);
	kd_tmp = dm_mit_encode(motor->set.KD, motor->param.KD_MIN, motor->param.KD_MAX, 12);
	tor_tmp = dm_mit_encode(motor->set.TOR, motor->param.T_MIN, motor->param.T_MAX, 12);

	MOTOR_Data[0] = (uint8_t)(pos_tmp >> 8);
	MOTOR_Data[1] = (uint8_t)pos_tmp;
	MOTOR_Data[2] = (uint8_t)(vel_tmp >> 4);
	MOTOR_Data[3] = (uint8_t)(((vel_tmp & 0xFU) << 4) | (kp_tmp >> 8));
	MOTOR_Data[4] = (uint8_t)kp_tmp;
	MOTOR_Data[5] = (uint8_t)(kd_tmp >> 4);
	MOTOR_Data[6] = (uint8_t)(((kd_tmp & 0xFU) << 4) | (tor_tmp >> 8));
	MOTOR_Data[7] = (uint8_t)tor_tmp;

	return canx_send_data(hcan, id + DM_mode, MOTOR_Data, 8);
}

/**
 * @brief  达妙电机使能
 * @param  hcan   CAN 句柄
 * @param  id     电机 ID
 */
uint8_t Motor_enable(FDCAN_HandleTypeDef *hcan, uint16_t id)
{
	return canx_send_data(hcan, id, MOTOR_Enable, 8);
}

/**
 * @brief  达妙电机失能
 * @param  hcan   CAN 句柄
 * @param  id     电机 ID
 */
void Motor_disable(FDCAN_HandleTypeDef *hcan, uint16_t id)
{
	canx_send_data(hcan, id, MOTOR_Failure, 8);
}

/**
 * @brief  保存电机当前位置为零点
 * @param  hcan   CAN 句柄
 * @param  id     电机 ID
 */
void Motor_save_zero(FDCAN_HandleTypeDef *hcan, uint16_t id)
{
	canx_send_data(hcan, id, MOTOR_Save_zero, 8);
}

/**
* @brief       清除电机错误
* @param[in]   hcan: CAN 句柄
* @param[in]   id:   电机 ID
* @retval      void
*/
void Motor_clear_err(FDCAN_HandleTypeDef *hcan, uint16_t id)
{
	canx_send_data(hcan, id, MOTOR_Clear_err, 8);
}

/**
 * @brief  设置灵足电机为私有模式
 * @param  hcan   CAN 句柄
 * @param  id     电机 ID
 */
void RS_MOTOR_PRE(FDCAN_HandleTypeDef *hcan,uint16_t id){
	canx_send_data(hcan, id, RS_MOTOR_PRE_MODE, 8);
}

/**
 * @brief  切换灵足电机为 MIT 模式
 * @param  hcan   CAN 句柄
 * @param  id     电机 ID
 */
void RS_Motor_MIT(FDCAN_HandleTypeDef *hcan,uint16_t id){
	canx_send_data(hcan, id, RS_MOTOR_MIT_MODE, 8);
}

/**
 * @brief          返回 DJI 电机结构指针
  * @param[in]       i: 电机编号, 范围 [0,7]
  * @retval          电机数据指针     	
 */
const dji_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
	return &DJI_MOTOR_MEASURE[(i & 0x07)];
}

/**
 * @brief          返回 DM 电机结构指针
  * @param[in]       i: 电机编号, 范围 [0,7]
  * @retval          电机数据指针     	
 */
dm_measure_t *get_dm_motor_measure_point(uint8_t i)
{
	return &DM_MOTOR_MEASURE[i];
}


/* ========== CAN 过滤器配置、中断回调及辅助函数 ========== */

/**
 * @brief  FDCAN1 配置：Mask=0 接收全部标准帧 → RX FIFO0
 * @note   滤波器设为全通过（FilterID=0, Mask=0），由软件在回调中按 ID 分发；
 *         global filter 拒绝扩展帧和远程帧。
 */
void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* 接收标准帧（扩展ID不接收） */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000; // Mask=0 → 接收全部标准帧
  sFilterConfig.FilterID2 = 0x00000000;
  if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
  /* global filter */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

	/* 开启RX FIFO0的新数据中断 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 
  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  FDCAN2 配置：Mask=0 接收全部标准帧 → RX FIFO1
 * @note   与 FDCAN1 不同，FDCAN2 路由到 FIFO1 以区分两条 CAN 总线；
 *         同样由软件在回调中按 ID 分发。
 */
void FDCAN2_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x00000000; // Mask=0 → 接收全部标准帧
  sFilterConfig.FilterID2 = 0x00000000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate Rx FIFO 1 new message notification */
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

// 记录最后一次 canx_send_data 的请求长度（调试用，非仅失败场景）
uint32_t g_can_fail_len = 0xFFFF;

/**
 * @brief  通用 CAN 发送函数（经典 CAN / CANFD 自适应）
 * @param  hcan   CAN 句柄
 * @param  id     CAN 报文 ID（标准帧）
 * @param  data   数据缓冲区指针
 * @param  len    数据字节数（支持 0~8, 12, 16, 20, 24, 48, 64）
 * @retval HAL status from HAL_FDCAN_AddMessageToTxFifoQ (0 means queued)
 * @note   DLC 编码: len<=8 时直接移位; 12/16/20/24/48/64 使用 FDCAN_DLC_BYTES 宏;
 *         Queued does not prove that the motor accepted or acknowledged the frame.
 */
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	g_can_fail_len = len;
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.Identifier = id;                 // CAN ID
	TxHeader.IdType =  FDCAN_STANDARD_ID ;        
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;  
	
	if(len<=8)	
	{
		TxHeader.DataLength = len<<16;    // 发送长度：8byte
	}
	else if(len==12)	
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_12;
	}
	else if(len==16)	
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_16;
	}
	else if(len==20)
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_20;
	}	
	else if(len==24)	
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_24;	
	}
	else if(len==32)
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_32;
	}
	else if(len==48)
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_48;
	}
	else if(len==64)
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_64;
	}									
	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	// 发送CAN指令
	return (uint8_t)HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data);
}

/**
 * @brief  FDCAN1 RX FIFO0 回调, 按 CAN ID 分发解析 DM / DJI 电机反馈帧
 * @note   依赖 ID 枚举值的连续性（DM_M1~DM_M8 和 CAN_3508_M1~CAN_TRIGGER_MOTOR
 *         各自连续排列），通过 (Identifier - 首ID) 计算数组索引，枚举值顺序不可随意调整。
 * @param  hcan        CAN 句柄
 * @param  RxFifo0ITs  中断标志
 */
/*
 * ============================================================================
 * 步骤1：CAN 接收合法性判定（P04）
 * ============================================================================
 * 判定项：GetRxMessage 返回值、标准帧、数据帧、DLC=8。
 * 任一项不满足都只计数，不更新电机测量、不喂健康计时器。
 */
static uint32_t can_rx_errors;
static uint32_t can_rx_ignored;

static uint8_t can_dlc_is_8(const FDCAN_RxHeaderTypeDef *header)
{
	return (uint8_t)((((header->DataLength) >> 16) & 0x0FU) == 8U);
}

static uint8_t can_frame_is_motor(const FDCAN_RxHeaderTypeDef *header)
{
	return (uint8_t)(header->IdType == FDCAN_STANDARD_ID &&
	                 header->RxFrameType == FDCAN_DATA_FRAME &&
	                 can_dlc_is_8(header));
}

void can_bsp_get_rx_stats(uint32_t *errors, uint32_t *ignored)
{
	if (errors != NULL) *errors = can_rx_errors;
	if (ignored != NULL) *ignored = can_rx_ignored;
}

/*
 * ============================================================================
 * 步骤2：FDCAN1（FRICTION_CAN）按总线归属分发
 * ============================================================================
 * 只接受摩擦轮 0x201/0x202（0x203/0x204 保留更新但不参与健康）。
 * 其它总线上的同 ID 帧不会出现在这里；这里出现未知 ID 只计数。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET &&
	    hcan->Instance == FDCAN1)
	{
		uint32_t fdb_time;
		if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData) != HAL_OK)
		{
			can_rx_errors++;
			return;
		}
		if (!can_frame_is_motor(&RxHeader1))
		{
			can_rx_ignored++;
			return;
		}
		fdb_time = HAL_GetTick();
		switch (RxHeader1.Identifier)
		{
			case CAN_3508_M1_ID:
				get_motor_measure(&DJI_MOTOR_MEASURE[0], g_Can1RxData);
				DJI_MOTOR_MEASURE[0].last_fdb_time = fdb_time;
				health_mark_valid(HEALTH_SRC_FRIC1_FDB, fdb_time);
				break;
			case CAN_3508_M2_ID:
				get_motor_measure(&DJI_MOTOR_MEASURE[1], g_Can1RxData);
				DJI_MOTOR_MEASURE[1].last_fdb_time = fdb_time;
				health_mark_valid(HEALTH_SRC_FRIC2_FDB, fdb_time);
				break;
			case CAN_FRIC3_MOTOR_ID:
				get_motor_measure(&DJI_MOTOR_MEASURE[2], g_Can1RxData);
				DJI_MOTOR_MEASURE[2].last_fdb_time = fdb_time;
				break;
			case CAN_FRIC4_MOTOR_ID:
				get_motor_measure(&DJI_MOTOR_MEASURE[3], g_Can1RxData);
				DJI_MOTOR_MEASURE[3].last_fdb_time = fdb_time;
				break;
			default:
				can_rx_ignored++;
				break;
		}
	}
}

/*
 * ============================================================================
 * 步骤3：FDCAN2（GIMBAL_CAN）按总线归属分发
 * ============================================================================
 * 只接受 Yaw 0x205、拨弹 0x207、DM Pitch 0x53；同一 ID 出现在 FDCAN1
 * 不会更新这里的电机，也不会喂健康计时器。
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo1ITs)
{
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET &&
	    hcan->Instance == FDCAN2)
	{
		uint32_t fdb_time;
		if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData) != HAL_OK)
		{
			can_rx_errors++;
			return;
		}
		if (!can_frame_is_motor(&RxHeader2))
		{
			can_rx_ignored++;
			return;
		}
		fdb_time = HAL_GetTick();
		switch (RxHeader2.Identifier)
		{
			case CAN_YAW_MOTOR_ID:
				get_motor_measure(&DJI_MOTOR_MEASURE[4], g_Can2RxData);
				DJI_MOTOR_MEASURE[4].last_fdb_time = fdb_time;
				health_mark_valid(HEALTH_SRC_YAW_FDB, fdb_time);
				break;
			case CAN_TRIGGER_MOTOR_ID:
				get_motor_measure(&DJI_MOTOR_MEASURE[6], g_Can2RxData);
				DJI_MOTOR_MEASURE[6].last_fdb_time = fdb_time;
				health_mark_valid(HEALTH_SRC_TRIGGER_FDB, fdb_time);
				break;
			case PITCH_DM_FEEDBACK_ID:
				DMFdbData(&DM_MOTOR_MEASURE[2], g_Can2RxData, 2U);
				DM_MOTOR_MEASURE[2].fdb.last_fdb_time = fdb_time;
				health_mark_valid(HEALTH_SRC_PITCH_FDB, fdb_time);
				break;
			default:
				can_rx_ignored++;
				break;
		}
	}
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
