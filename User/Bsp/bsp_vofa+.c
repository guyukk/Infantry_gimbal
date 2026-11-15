#include "bsp_vofa+.h"
#include "bsp_usart.h"

/* VOFA 调试输出统计：USB/协议字节流不受影响 */
static volatile uint32_t vofa_dropped;
static volatile uint32_t vofa_sent;

/*
 * ============================================================================
 * VOFA JustFloat 发送（USART10，调试专用）
 * ============================================================================
 * 操作：
 * 1) 上一帧仍在发送时整帧丢弃并计数，避免覆盖 HAL 正在读取的静态缓冲
 * 2) 长度超限时丢弃，不越界写 buf
 */
void Vofa_JustFloat(float *_data, uint8_t _num)
{
    static uint8_t buf[256];
    static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    uint16_t length;

    if (_data == NULL || _num == 0U)
    {
        vofa_dropped++;
        return;
    }
    length = (uint16_t)(4U * _num + 4U);
    if (length > sizeof(buf))
    {
        vofa_dropped++;
        return;
    }
    if (huart10.gState != HAL_UART_STATE_READY)
    {
        vofa_dropped++;
        return;
    }
    memcpy(buf, _data, 4U * _num);              // 1. 数据
    memcpy(buf + 4U * _num, tail, 4);           // 2. 尾
    if (HAL_UART_Transmit_IT(&huart10, buf, length) == HAL_OK)
    {
        vofa_sent++;
    }
    else
    {
        vofa_dropped++;
    }
}

void vofa_get_stats(uint32_t *sent, uint32_t *dropped)
{
    if (sent != NULL)
    {
        *sent = vofa_sent;
    }
    if (dropped != NULL)
    {
        *dropped = vofa_dropped;
    }
}
