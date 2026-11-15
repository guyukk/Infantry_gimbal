#include "health.h"
#include "bsp_tfmini.h"
#include "bsp_usart.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include <math.h>
#include <string.h>
#include "usart.h"
#include "video_transmission_module.h"

/*
 * ============================================================================
 * 串口接收/发送实现（P01/P03/P08）
 * ----------------------------------------------------------------------------
 * 1) USART1：底盘 -> 云台 28 字节帧，RX DMA 环形 + 流式解包，只有完整且
 *    头尾/字段校验通过的帧才发布，ISR 内不打印、不阻塞。
 * 2) UART5：遥控器 18 字节 DBUS 帧，normal DMA + ReceiveToIdle，整块校验后发布。
 * 3) USART1 TX：46 字节帧 DMA 发送使用内部副本 + BUSY 丢弃计数，避免发送
 *    过程中被下一帧覆盖。
 * ============================================================================
 */

/* 串口1接收缓冲：DMA 写入的原始环形缓冲，不是业务帧缓冲 */
static uint8_t chassis_rx_dma[USART1_RX_DMA_SIZE];
/* 最近一次通过校验的原始帧，仅作证据使用，不参与业务解析 */
uint8_t gimbal_receive_data[CHASSIS_DATA_LENGTH];

uint8_t remote_buff[SBUS_RX_BUF_NUM];
remoter_t remoter;

/* 串口7 */
uint8_t usart7_buf[21];		//设置缓冲区

/* 串口1发送缓冲：DMA 异步读取，必须独立于调用者栈/静态帧缓冲 */
static uint8_t usart1_tx_frame[USART1_TX_FRAME_MAX];
static chassis_tx_stats_t usart1_tx_stats;

/* 解析器状态 */
typedef struct
{
	uint8_t  frame[CHASSIS_DATA_LENGTH];
	uint8_t  length;
	chassis_rx_stats_t stats;
} chassis_rx_parser_t;

static chassis_rx_parser_t chassis_rx;
static uint32_t chassis_rx_last_pos;
/* 发布序号：奇数=写入中，偶数=稳定；每帧 +2，帧计数 = 序号/2 */
static volatile uint32_t chassis_rx_seq;

/* 云台收到的底盘数据（只有完整有效帧才会改写） */
chassis_data_t chassis_data;

/* 串口1发送函数（阻塞式，保留给调试/初始化路径） */
void USART1_Transmit_DMA(uint8_t *pData, uint16_t Size)
{
	if (pData == NULL || Size == 0U || Size > USART1_TX_FRAME_MAX)
	{
		usart1_tx_stats.rejected++;
		return;
	}
	/* 发送进行中不允许覆盖 DMA 正在读取的缓冲，整帧丢弃并计数 */
	if (huart1.gState != HAL_UART_STATE_READY)
	{
		usart1_tx_stats.busy_dropped++;
		return;
	}
	memcpy(usart1_tx_frame, pData, Size);
	if (HAL_UART_Transmit_DMA(&huart1, usart1_tx_frame, Size) == HAL_OK)
	{
		usart1_tx_stats.sent++;
	}
	else
	{
		usart1_tx_stats.busy_dropped++;
	}
}

/* 串口1发送函数 */
void USART1_Transmit_IT(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit_IT(&huart1, pData, Size);
}

/* 串口1发送函数 */
void USART1_Transmit(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit(&huart1, pData, Size, 50);
}

/* 串口10发送函数 */
void USART10_Transmit(uint8_t *pData, uint16_t Size)
{
	HAL_UART_Transmit(&huart10, pData, Size, 50);
}

/* 串口10中断发送函数 */
void USART10_Transmit_IT(uint8_t *pData, uint16_t Size)
{
	HAL_UART_Transmit_IT(&huart10, pData, Size);
}

void usart1_tx_get_stats(chassis_tx_stats_t *out)
{
	uint32_t primask;
	if (out == NULL)
	{
		return;
	}
	primask = __get_PRIMASK();
	__disable_irq();
	*out = usart1_tx_stats;
	__set_PRIMASK(primask);
}

/*
 * ============================================================================
 * 步骤1：28 字节帧字段解析与校验
 * ============================================================================
 * 数据源：chassis_rx.frame（已收满 28 字节）
 * 操作：
 * 1) 校验帧头 's' / 帧尾 'e'
 * 2) 按现有小端布局取 6 个 float 与 chassis_mode/robot_id
 * 3) 有限值 + 合理范围 + 枚举范围检查
 * 返回：0=有效，1=帧头错，2=帧尾错，3=字段非法
 */
static uint8_t chassis_frame_parse(const uint8_t *frame, chassis_data_t *out)
{
	fp32_to_bytes field;

	if (frame[0] != CHASSIS_FRAME_HEAD)
	{
		return 1U;
	}
	if (frame[CHASSIS_DATA_LENGTH - 1] != CHASSIS_FRAME_TAIL)
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

	/* NaN/Inf 一律拒绝，避免污染底盘姿态与速度状态 */
	if (!isfinite(out->chassis_yaw) || !isfinite(out->chassis_pitch) ||
	    !isfinite(out->chassis_roll) || !isfinite(out->chassis_vx) ||
	    !isfinite(out->chassis_vy) || !isfinite(out->chassis_wz))
	{
		return 3U;
	}
	if (fabsf(out->chassis_yaw) > CHASSIS_ANGLE_LIMIT_RAD ||
	    fabsf(out->chassis_pitch) > CHASSIS_ANGLE_LIMIT_RAD ||
	    fabsf(out->chassis_roll) > CHASSIS_ANGLE_LIMIT_RAD)
	{
		return 3U;
	}
	if (fabsf(out->chassis_vx) > CHASSIS_SPEED_LIMIT_MPS ||
	    fabsf(out->chassis_vy) > CHASSIS_SPEED_LIMIT_MPS ||
	    fabsf(out->chassis_wz) > CHASSIS_WZ_LIMIT_RADPS)
	{
		return 3U;
	}
	if (out->chassis_mode > CHASSIS_MODE_MAX)
	{
		return 3U;
	}
	/* robot_id 是身份参考，0 表示裁判数据未就绪，不据此拒帧 */
	return 0U;
}

/*
 * ============================================================================
 * 步骤2：有效帧原子发布
 * ============================================================================
 * 目标：ISR 单写者，任务读者用序号快照重试
 * 操作：
 * 1) 序号置奇数表示写入中
 * 2) 更新原始帧证据与业务结构
 * 3) 序号置偶数表示稳定，帧计数 = 序号/2
 */
static void chassis_rx_publish(const uint8_t *frame, const chassis_data_t *parsed)
{
	__DMB();
	chassis_rx_seq++;
	__DMB();
	memcpy(gimbal_receive_data, frame, CHASSIS_DATA_LENGTH);
	chassis_data = *parsed;
	health_touch(HEALTH_BOARD, HAL_GetTick());
	__DMB();
	chassis_rx_seq++;
	__DMB();
}

/*
 * ============================================================================
 * 步骤3：流式解包与逐字节重同步
 * ============================================================================
 * 操作：
 * 1) 未锁定帧头时逐字节丢弃并计数，不跳过潜在帧头
 * 2) 收满 28 字节后校验：通过则发布；失败则丢弃首字节并保留后缀重新扫描
 * 3) 窗口长度天然有界（<=28），噪声不会无限累积
 */
static void chassis_rx_feed_byte(uint8_t value)
{
	chassis_rx.stats.bytes_seen++;

	if (chassis_rx.length == 0U)
	{
		if (value != CHASSIS_FRAME_HEAD)
		{
			chassis_rx.stats.bytes_discarded++;
			return;
		}
		chassis_rx.frame[0] = value;
		chassis_rx.length = 1U;
		return;
	}

	chassis_rx.frame[chassis_rx.length++] = value;
	if (chassis_rx.length < CHASSIS_DATA_LENGTH)
	{
		return;
	}

	chassis_rx.length = 0U;
	{
		chassis_data_t parsed;
		uint8_t verdict = chassis_frame_parse(chassis_rx.frame, &parsed);
		if (verdict == 0U)
		{
			chassis_rx_publish(chassis_rx.frame, &parsed);
			chassis_rx.stats.frames_ok++;
			return;
		}
		if (verdict == 2U)
		{
			chassis_rx.stats.frames_bad_tail++;
		}
		else if (verdict == 3U)
		{
			chassis_rx.stats.frames_bad_field++;
		}

		/* 重同步：丢弃首字节，保留其余字节中可能存在的帧头 */
		{
			uint8_t index = 1U;
			while (index < CHASSIS_DATA_LENGTH &&
			       chassis_rx.frame[index] != CHASSIS_FRAME_HEAD)
			{
				index++;
			}
			if (index >= CHASSIS_DATA_LENGTH)
			{
				chassis_rx.stats.bytes_discarded += CHASSIS_DATA_LENGTH;
			}
			else
			{
				uint8_t remain = (uint8_t)(CHASSIS_DATA_LENGTH - index);
				memmove(chassis_rx.frame, &chassis_rx.frame[index], remain);
				chassis_rx.length = remain;
				chassis_rx.stats.bytes_discarded += index;
			}
		}
	}
}

static void chassis_rx_consume_range(uint32_t from, uint32_t to)
{
	uint32_t index;
	for (index = from; index < to && index < USART1_RX_DMA_SIZE; index++)
	{
		chassis_rx_feed_byte(chassis_rx_dma[index]);
	}
	chassis_rx_last_pos = to;
}

/*
 * ============================================================================
 * 步骤4：DMA 事件消费
 * ============================================================================
 * 数据源：DMA 当前 NDTR；HAL 的 Size 仅作事件范围校验。
 * 延迟的 HT/TC 回调可能晚于 IDLE，不能把旧 Size 当成新的回绕。
 * 操作：
 * 1) 位置单调递增时消费 [last_pos, pos)
 * 2) 位置回绕（环形 DMA）时先消费到缓冲末尾，再消费 [0, pos)
 * 3) pos==缓冲长度表示整轮写完，下一字节从 0 开始
 * 4) 长度异常时丢弃解析器状态并重启接收，防止旧数据冒充新帧
 */
static void chassis_rx_restart(void)
{
	memset(chassis_rx_dma, 0, sizeof(chassis_rx_dma));
	chassis_rx.length = 0U;
	if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, chassis_rx_dma, USART1_RX_DMA_SIZE) == HAL_OK)
	{
		chassis_rx_last_pos = 0U;
		chassis_rx.stats.restarts++;
	}
}

static void chassis_rx_on_event(uint16_t size)
{
	/* USART1 RX is DMA1_Stream2 in Core/Src/usart.c (not a BDMA channel). */
	uint32_t remaining = ((DMA_Stream_TypeDef *)huart1.hdmarx->Instance)->NDTR;
	uint32_t pos;

	chassis_rx.stats.dma_events++;
	if (size > USART1_RX_DMA_SIZE || remaining > USART1_RX_DMA_SIZE)
	{
		chassis_rx.stats.overflow++;
		chassis_rx_last_pos = 0U;
		chassis_rx_restart();
		return;
	}
	if (size == 0U) return;
	pos = USART1_RX_DMA_SIZE - remaining;
	if (pos == 0U) pos = USART1_RX_DMA_SIZE;
	if (pos > chassis_rx_last_pos)
	{
		chassis_rx_consume_range(chassis_rx_last_pos, pos);
	}
	else if (pos < chassis_rx_last_pos)
	{
		chassis_rx_consume_range(chassis_rx_last_pos, USART1_RX_DMA_SIZE);
		chassis_rx_consume_range(0U, pos);
	}
	/* Keep the exclusive end position: TC followed by IDLE at size must not replay. */
	chassis_rx_last_pos = pos;
}

/* UART5：normal DMA，每个事件是一段完整或部分帧，交给遥控器解析器校验 */
static void remote_control_on_event(UART_HandleTypeDef *huart, uint16_t size)
{
	if (size > SBUS_RX_BUF_NUM)
	{
		remote_control_rx_overflow();
		size = 0U;
	}
	remote_control_rx_chunk(remote_buff, size);
	/* HT 事件时 RxState 仍为 BUSY，HAL 会返回 HAL_BUSY 而不影响正在进行的接收 */
	(void)HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_buff, SBUS_RX_BUF_NUM);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART10)
	{
		/* TFmini UART RX callback */
		tfmini_uart_rx_cplt_callback(huart);
	}
}

//串口接收时间回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == USART1)
	{
		chassis_rx_on_event(Size);
	}

	if(huart->Instance == UART5)
	{
		remote_control_on_event(huart, Size);
	}
}

//串口出错回调函数
void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == USART1)
	{
		/* 同一缓冲重启；解析器与消费位置一起重置，防止重复消费 */
		chassis_rx.length = 0U;
		chassis_rx_restart();
	}

	if(huart->Instance == UART5)
	{
		remote_control_rx_restart(huart);
	}

	if(huart->Instance == USART10)
	{
		tfmini_uart_error_callback(huart);
	}
}

/*
 * ============================================================================
 * 步骤5：接收启动与统计接口
 * ============================================================================
 */
void bsp_usart_rx_start(void)
{
	memset(&chassis_rx, 0, sizeof(chassis_rx));
	memset(chassis_rx_dma, 0, sizeof(chassis_rx_dma));
	chassis_rx_last_pos = 0U;
	(void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, chassis_rx_dma, USART1_RX_DMA_SIZE);
	remote_control_rx_restart(&huart5);
}

void chassis_rx_feed_stream(const uint8_t *data, uint16_t len)
{
	uint16_t index;
	if (data == NULL)
	{
		return;
	}
	for (index = 0U; index < len; index++)
	{
		chassis_rx_feed_byte(data[index]);
	}
}

/* 诊断/离线测试用：丢弃解析器半帧状态与 DMA 消费位置，不触碰外设 */
void chassis_rx_reset(void)
{
	chassis_rx.length = 0U;
	chassis_rx_last_pos = 0U;
}

void chassis_rx_get_stats(chassis_rx_stats_t *out)
{
	uint32_t primask;
	if (out == NULL)
	{
		return;
	}
	primask = __get_PRIMASK();
	__disable_irq();
	*out = chassis_rx.stats;
	__set_PRIMASK(primask);
}

uint8_t chassis_data_read(chassis_data_t *out, uint32_t *generation)
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
		before = chassis_rx_seq;
		if ((before & 1U) != 0U)
		{
			continue;
		}
		__DMB();
		*out = chassis_data;
		__DMB();
		after = chassis_rx_seq;
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
