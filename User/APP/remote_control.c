#include "health.h"
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      远程控制器
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  **/
#include "main.h"
#include "remote_control.h"
#include "robot_param.h"
#include "string.h"

//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_usart5_rx;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

//遥控器控制变量
RC_ctrl_t rc_ctrl;

/* 发布序号：奇数=写入中，偶数=稳定；ISR 单写者，任务侧用 rc_ctrl_snapshot 重试 */
static volatile uint32_t rc_publish_seq;
static uint32_t rc_frame_count;
static rc_rx_stats_t rc_rx_stats;

//取正函数


/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
	remote_control_rx_restart(&huart5);
}

/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    RC_ctrl_t sample;
    uint8_t i;
    if (!health_fresh(HEALTH_RC,HAL_GetTick())) return 1U;
    if (!rc_ctrl_snapshot(&sample,NULL)) return 1U;
    for (i=0U;i<4U;i++)
        if (sample.rc.ch[i] < -700 || sample.rc.ch[i] > 700) return 1U;
    return sample.rc.s[0] < 1U || sample.rc.s[0] > 3U || sample.rc.s[1] < 1U || sample.rc.s[1] > 3U;
}

/*
 * ============================================================================
 * 步骤1：18 字节 DBUS 帧解析
 * ============================================================================
 * 数据源：一个完整 18 字节块（UART5，100000 8E2，18 字节 DBUS 分支）
 * 操作：保持原有位域/偏移与中值换算不变，只把发布动作留给调用者
 */
static void rc_frame_parse(const uint8_t *sbus_buf, RC_ctrl_t *out)
{
	out->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
	out->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
	out->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
	                 (sbus_buf[4] << 10)) & 0x07ff;
	out->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
	out->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                        //!< Right switch (gimbal mode)
	out->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                   //!< Left switch (shoot mode)
	out->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                     //!< Mouse X axis
	out->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                     //!< Mouse Y axis
	out->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                   //!< Mouse Z axis
	out->mouse.press_l = sbus_buf[12];                                   //!< Mouse Left Is Press ?
	out->mouse.press_r = sbus_buf[13];                                   //!< Mouse Right Is Press ?
	out->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                     //!< KeyBoard value
	out->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                  //NULL

	out->rc.ch[0] -= RC_CH_VALUE_OFFSET;
	out->rc.ch[1] -= RC_CH_VALUE_OFFSET;
	out->rc.ch[2] -= RC_CH_VALUE_OFFSET;
	out->rc.ch[3] -= RC_CH_VALUE_OFFSET;
	out->rc.ch[4] -= RC_CH_VALUE_OFFSET;
	out->last_fdb = HAL_GetTick();
}

/**
	* @brief          遥控器协议解析（兼容入口：解析但不做合法性判断）
	* @param[in]      sbus_buf: 原生数据指针（18 字节）
	* @param[out]     rc_ctrl: 遥控器数据指针
	* @param[in]      remoter: 兼容参数（未使用）
	* @retval         none
	*/
void sbus_to_rc(uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl, remoter_t *remoter)
{
	(void)remoter;
	if (sbus_buf == NULL || rc_ctrl == NULL)
	{
		return;
	}
	rc_frame_parse(sbus_buf, rc_ctrl);
	rc_ctrl->generation++;
}

/*
 * ============================================================================
 * 步骤2：帧合法性校验
 * ============================================================================
 * 操作：
 * 1) 四通道原始值必须落在 ±700 失效门限内（raw 324..1724）
 * 2) 两个开关必须是 1/2/3 的合法枚举，0 视为无效
 * 返回：0=有效，2=通道越界，3=开关非法
 */
static uint8_t rc_frame_validate(const uint8_t *sbus_buf)
{
	uint16_t raw[4];
	uint8_t sw0;
	uint8_t sw1;
	uint8_t index;

	raw[0] = (uint16_t)((sbus_buf[0] | ((uint16_t)sbus_buf[1] << 8)) & 0x07FFu);
	raw[1] = (uint16_t)((((uint16_t)sbus_buf[1] >> 3) | ((uint16_t)sbus_buf[2] << 5)) & 0x07FFu);
	raw[2] = (uint16_t)((((uint16_t)sbus_buf[2] >> 6) | ((uint16_t)sbus_buf[3] << 2) |
	                     ((uint16_t)sbus_buf[4] << 10)) & 0x07FFu);
	raw[3] = (uint16_t)((((uint16_t)sbus_buf[4] >> 1) | ((uint16_t)sbus_buf[5] << 7)) & 0x07FFu);
	sw0 = (uint8_t)((sbus_buf[5] >> 4) & 0x03u);
	sw1 = (uint8_t)(((sbus_buf[5] >> 4) & 0x0Cu) >> 2);

	for (index = 0U; index < 4U; index++)
	{
		if (!RC_CH_VALUE_VALID(raw[index]))
		{
			return 2U;
		}
	}
	if (!RC_SWITCH_VALID(sw0) || !RC_SWITCH_VALID(sw1))
	{
		return 3U;
	}
	return 0U;
}

/*
 * ============================================================================
 * 步骤3：一致性发布
 * ============================================================================
 * 目标：ISR 单写者发布整帧；任务侧用序号快照避免读后清零丢新值
 */
static void rc_publish(const RC_ctrl_t *parsed)
{
	__DMB();
	rc_publish_seq++;
	__DMB();
	rc_ctrl = *parsed;
	rc_frame_count++;
	rc_ctrl.generation = rc_frame_count;
	health_touch(HEALTH_RC, rc_ctrl.last_fdb);
	__DMB();
	rc_publish_seq++;
	__DMB();
}

uint8_t rc_ctrl_snapshot(RC_ctrl_t *out, uint32_t *generation)
{
	uint32_t before;
	uint32_t after;
	uint32_t attempt;
	if (out == NULL)
	{
		return 0U;
	}
	for (attempt = 0U; attempt < 8U; attempt++)
	{
		before = rc_publish_seq;
		if ((before & 1U) != 0U)
		{
			continue;
		}
		__DMB();
		*out = rc_ctrl;
		__DMB();
		after = rc_publish_seq;
		if (before == after)
		{
			if (generation != NULL)
			{
				*generation = before >> 1;
			}
			return 1U;
		}
	}
	return 0U;
}

/*
 * ============================================================================
 * 步骤4：DMA 块交接与整块策略
 * ============================================================================
 * 数据源：UART5 normal DMA + ReceiveToIdle 的每轮数据
 * 策略：
 * 1) 18 字节块 = 1 帧；36 字节块 = 2 帧
 * 2) 其它长度（半帧/噪声）整块丢弃并计数，不发布、不刷新时间戳
 * 3) 帧内校验失败只丢该帧，另一帧仍可独立通过（36 字节块）
 */
static void rc_try_frame(const uint8_t *frame)
{
	RC_ctrl_t parsed;
	uint8_t verdict = rc_frame_validate(frame);

	if (verdict == 2U)
	{
		health_reject(HEALTH_RC);
		rc_rx_stats.bad_channel++;
		rc_rx_stats.discarded_bytes += RC_FRAME_LENGTH;
		return;
	}
	if (verdict == 3U)
	{
		health_reject(HEALTH_RC);
		rc_rx_stats.bad_switch++;
		rc_rx_stats.discarded_bytes += RC_FRAME_LENGTH;
		return;
	}
	sbus_to_rc((uint8_t *)frame, &parsed, &remoter);
	rc_publish(&parsed);
	rc_rx_stats.frames_ok++;
}

void remote_control_rx_chunk(const uint8_t *data, uint16_t len)
{
	rc_rx_stats.chunks++;
	if (data == NULL || len == 0U)
	{
		health_reject(HEALTH_RC);
		rc_rx_stats.rejected_chunks++;
		return;
	}
	if (len == RC_FRAME_LENGTH)
	{
		rc_try_frame(data);
	}
	else if (len == (uint16_t)(2U * RC_FRAME_LENGTH))
	{
		rc_try_frame(data);
		rc_try_frame(data + RC_FRAME_LENGTH);
	}
	else
	{
		health_reject(HEALTH_RC);
		rc_rx_stats.rejected_chunks++;
		rc_rx_stats.discarded_bytes += len;
	}
}

void remote_control_rx_restart(UART_HandleTypeDef *huart)
{
	if (huart == NULL)
	{
		return;
	}
	if (HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_buff, SBUS_RX_BUF_NUM) == HAL_OK)
	{
		rc_rx_stats.restarts++;
	}
}

void remote_control_rx_overflow(void)
{
	health_reject(HEALTH_RC);
	rc_rx_stats.overflow++;
	rc_rx_stats.rejected_chunks++;
}

void rc_rx_get_stats(rc_rx_stats_t *out)
{
	uint32_t primask;
	if (out == NULL)
	{
		return;
	}
	primask = __get_PRIMASK();
	__disable_irq();
	*out = rc_rx_stats;
	__set_PRIMASK(primask);
}
