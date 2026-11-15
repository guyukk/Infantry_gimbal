#ifndef USB_CDC_PORT_H
#define USB_CDC_PORT_H

#include <stdint.h>
#include "uproto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RX 交接环容量：2 的幂，ISR 单生产者 / 任务单消费者，无锁 */
#ifndef USB_CDC_RX_RING_SIZE
#define USB_CDC_RX_RING_SIZE 1024u
#endif

/* 单次 USB 发送暂存上限：HS bulk IN 单包 512 */
#ifndef USB_CDC_TX_STAGE_SIZE
#define USB_CDC_TX_STAGE_SIZE 512u
#endif

typedef struct
{
    uint32_t isr_bytes;      /* CDC 回调收到的字节数 */
    uint32_t isr_dropped;    /* 交接环满时丢弃的字节数 */
    uint32_t pumped_bytes;   /* 任务侧交给 uproto 的字节数 */
    uint32_t tx_frames;      /* 成功启动 USB 发送的帧数 */
    uint32_t tx_busy_dropped;/* 上一帧未发完而丢弃的帧数 */
    uint32_t tx_rejected;    /* 未枚举/参数非法 */
} usb_cdc_port_stats_t;

/* Initialize any internal state if needed */
void usb_cdc_port_init(void);

/* Build default uproto port ops backed by USB CDC (HS) */
void usb_cdc_port_get_ops(uproto_port_ops_t *ops);

/* Direct write helper (non-blocking). Returns bytes accepted. */
uint32_t usb_cdc_port_write(void *user, const uint8_t *data, uint32_t len);

/* Bind uproto ctx so USB CDC RX bytes can be handed to the protocol */
void usb_cdc_port_bind_uproto(uproto_context_t *ctx);

/* ISR 侧：只把原始字节放入有界交接环，不做协议解析 */
void usb_cdc_port_on_rx(const uint8_t *data, uint32_t len);

/* 任务侧：把交接环中的字节交给 uproto 解析（只在 comm_app 任务调用） */
void usb_cdc_port_rx_pump(void);

/* 读取统计（任务上下文，短临界区） */
void usb_cdc_port_get_stats(usb_cdc_port_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_PORT_H */
