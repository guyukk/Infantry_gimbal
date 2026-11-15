/*
 * ============================================================================
 * USB CDC 端口适配层（P08）
 * ----------------------------------------------------------------------------
 * 1) RX：CDC 中断只把原始字节写入无锁有界环（ISR 单生产者），协议解析全部
 *    放到 comm_app 任务（usb_cdc_port_rx_pump）执行，避免 ISR/任务的收发
 *    状态并发。
 * 2) TX：uproto 编码结果先复制到本层静态缓冲再启动 USB 发送；上一帧未发完
 *    时返回 0（调用者按 TX_FULL 处理），不覆盖正在被 USB 外设读取的缓冲。
 * 3) 所有丢弃/失败都有计数，不静默宣称成功。
 * ============================================================================
 */
#include "usb_cdc_port.h"
#include "stm32h7xx_hal.h"
#include "usbd_core.h"
#include "usbd_cdc_if.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceHS;

/* 接收交接环：head 只由 ISR 写，tail 只由任务写，均 16 位对齐访问 */
static volatile uint16_t usb_rx_head;
static volatile uint16_t usb_rx_tail;
static uint8_t usb_rx_ring[USB_CDC_RX_RING_SIZE];

/* 发送暂存：USB 外设异步读取，长度不超过单包 MTU */
static uint8_t usb_tx_stage[USB_CDC_TX_STAGE_SIZE];
static usb_cdc_port_stats_t usb_stats;

/* 绑定后的协议上下文：只在 comm_app 任务读取 */
static uproto_context_t *s_ctx = NULL;

void usb_cdc_port_init(void)
{
    usb_rx_head = 0u;
    usb_rx_tail = 0u;
}

/*
 * ============================================================================
 * 步骤1：ISR 侧入环
 * ============================================================================
 * 数据源：CDC_Receive_HS 的 USB OUT 包
 * 操作：
 * 1) 逐字节写入环形缓冲，head 只在最后更新
 * 2) 环满时丢弃新字节并计数；不阻塞、不解析
 */
void usb_cdc_port_on_rx(const uint8_t *data, uint32_t len)
{
    uint16_t head;
    uint16_t next;
    uint32_t index;

    if (data == NULL || len == 0u)
    {
        return;
    }
    usb_stats.isr_bytes += len;
    head = usb_rx_head;
    for (index = 0u; index < len; index++)
    {
        next = (uint16_t)((head + 1u) % USB_CDC_RX_RING_SIZE);
        if (next == usb_rx_tail)
        {
            usb_stats.isr_dropped += (len - index);
            break;
        }
        usb_rx_ring[head] = data[index];
        head = next;
    }
    usb_rx_head = head;
}

/*
 * ============================================================================
 * 步骤2：任务侧出环并解析
 * ============================================================================
 * 操作：
 * 1) 按块取出环内字节，一次性交给 uproto_on_rx_bytes
 * 2) tail 只在交给协议后更新；单圈最多处理环容量
 */
void usb_cdc_port_rx_pump(void)
{
    uint16_t tail;
    uint16_t head;
    uint8_t chunk[64];
    uint32_t count;

    if (s_ctx == NULL)
    {
        return;
    }
    for (;;)
    {
        tail = usb_rx_tail;
        head = usb_rx_head;
        if (tail == head)
        {
            return;
        }
        count = 0u;
        while (tail != head && count < sizeof(chunk))
        {
            chunk[count++] = usb_rx_ring[tail];
            tail = (uint16_t)((tail + 1u) % USB_CDC_RX_RING_SIZE);
        }
        uproto_on_rx_bytes(s_ctx, chunk, count);
        usb_stats.pumped_bytes += count;
        usb_rx_tail = tail;
    }
}

/*
 * ============================================================================
 * 步骤3：发送（有界、非阻塞）
 * ============================================================================
 * 操作：
 * 1) 未枚举/参数非法直接返回 0 并计数
 * 2) 复制到本层暂存后再启动 CDC 发送，调用者栈缓冲即可安全返回
 * 3) 上一帧未发完（TxState != 0）时不覆盖，返回 0 并计数
 */
uint32_t usb_cdc_port_write(void *user, const uint8_t *data, uint32_t len)
{
    USBD_CDC_HandleTypeDef *cdc;
    uint8_t result;
    (void)user;
    if (!data || len == 0u)
    {
        usb_stats.tx_rejected++;
        return 0u;
    }
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED || hUsbDeviceHS.pClassData == NULL)
    {
        usb_stats.tx_rejected++;
        return 0u;
    }
    if (len > USB_CDC_TX_STAGE_SIZE)
    {
        /* A protocol frame is accepted whole or not at all. */
        usb_stats.tx_rejected++;
        return 0u;
    }
    cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    if (cdc->TxState != 0u)
    {
        /* Check before copying: the peripheral still owns usb_tx_stage. */
        usb_stats.tx_busy_dropped++;
        return 0u;
    }
    memcpy(usb_tx_stage, data, len);
    result = CDC_Transmit_HS(usb_tx_stage, (uint16_t)len);
    if (result != USBD_OK)
    {
        if (result == USBD_BUSY) usb_stats.tx_busy_dropped++;
        else usb_stats.tx_rejected++;
        return 0u;
    }
    usb_stats.tx_frames++;
    return len;
}

static void usb_cdc_port_flush(void *user)
{
    (void)user; /* USB CDC 无显式 flush */
}

static uint16_t usb_cdc_port_get_mtu(void *user)
{
    (void)user;
    return (uint16_t)USB_CDC_TX_STAGE_SIZE;
}

void usb_cdc_port_get_ops(uproto_port_ops_t *ops)
{
    if (!ops) return;
    ops->write = usb_cdc_port_write;
    ops->flush = usb_cdc_port_flush;
    ops->get_mtu = usb_cdc_port_get_mtu;
    ops->user = NULL;
}

void usb_cdc_port_bind_uproto(uproto_context_t *ctx)
{
    s_ctx = ctx;
}

void usb_cdc_port_get_stats(usb_cdc_port_stats_t *out)
{
    uint32_t primask;
    if (out == NULL)
    {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *out = usb_stats;
    __set_PRIMASK(primask);
}
