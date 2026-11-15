
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define USB_CDC_RX_RING_SIZE 64u
#define USB_CDC_TX_STAGE_SIZE 512u
typedef struct uproto_context uproto_context_t;
struct uproto_context { int unused; };
typedef struct { uint32_t dev_state; void *pClassData; } USBD_HandleTypeDef;
typedef struct { volatile uint32_t TxState; } USBD_CDC_HandleTypeDef;
static USBD_CDC_HandleTypeDef cdc_class;
typedef struct { int unused; } uproto_port_ops_t;
#define USBD_STATE_CONFIGURED 2u
#define USBD_OK 0u
#define USBD_FAIL 1u
#define USBD_BUSY 2u
typedef struct { uint32_t isr_bytes, isr_dropped, pumped_bytes, tx_frames,
                 tx_busy_dropped, tx_rejected; } usb_cdc_port_stats_t;
USBD_HandleTypeDef hUsbDeviceHS;
static uproto_context_t ctx_object;
static uproto_context_t *s_ctx = &ctx_object;
static volatile uint16_t usb_rx_head;
static volatile uint16_t usb_rx_tail;
static uint8_t usb_rx_ring[USB_CDC_RX_RING_SIZE];
static uint8_t usb_tx_stage[USB_CDC_TX_STAGE_SIZE];
static usb_cdc_port_stats_t usb_stats;
/* 协议替身：记录任务侧实际收到的字节流 */
uint8_t rx_seen[512];
uint32_t rx_seen_len;
void uproto_on_rx_bytes(uproto_context_t *ctx, const uint8_t *data, uint32_t len) {
  (void)ctx;
  if (rx_seen_len + len <= sizeof(rx_seen)) {
    memcpy(rx_seen + rx_seen_len, data, len);
    rx_seen_len += len;
  }
}
/* USB 替身：只在“完成”时才把异步读取的指针内容取走，用于证明缓冲寿命 */
#define usb_busy cdc_class.TxState
uint8_t *usb_pending_ptr;
uint16_t usb_pending_len;
uint8_t usb_captured[512];
uint8_t CDC_Transmit_HS(uint8_t *buf, uint16_t len) {
  if (usb_busy) return USBD_BUSY;
  usb_pending_ptr = buf;
  usb_pending_len = len;
  usb_busy = 1u;
  return USBD_OK;
}
volatile struct { uint32_t status, checks, failure; } test_result;
void usb_cdc_port_on_rx(const uint8_t *data, uint32_t len) {
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
void usb_cdc_port_rx_pump(void) {
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
uint32_t usb_cdc_port_write(void *user, const uint8_t *data, uint32_t len) {
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
                                                               
        usb_stats.tx_rejected++;
        return 0u;
    }
    cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    if (cdc->TxState != 0u)
    {
                                                                           
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

void check(int condition, uint32_t number) {
  ++test_result.checks;
  if (!condition && test_result.failure == 0u) test_result.failure = number;
}
static void reset_usb(void) {
  usb_rx_head = 0; usb_rx_tail = 0; usb_busy = 0;
  usb_pending_ptr = 0; usb_pending_len = 0; rx_seen_len = 0;
  memset(&usb_stats, 0, sizeof(usb_stats));
  hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
  hUsbDeviceHS.pClassData = &cdc_class;
}
void test_entry(void) {
  uint8_t data[256];
  uint8_t stack_buf[64];
  uint32_t index;

  reset_usb();
  for (index = 0; index < sizeof(data); ++index) data[index] = (uint8_t)index;
  usb_cdc_port_on_rx(data, 40);
  check(usb_stats.isr_bytes == 40u, 1);
  usb_cdc_port_rx_pump();
  check(rx_seen_len == 40u && memcmp(rx_seen, data, 40) == 0, 2);
  check(usb_stats.pumped_bytes == 40u && usb_rx_tail == usb_rx_head, 3);

  /* 交错入环/出环 */
  usb_cdc_port_on_rx(data + 40, 10);
  usb_cdc_port_rx_pump();
  usb_cdc_port_on_rx(data + 50, 10);
  usb_cdc_port_rx_pump();
  check(rx_seen_len == 60u && memcmp(rx_seen + 40, data + 40, 20) == 0, 4);

  /* 环满：有界丢弃，计数正确，不破坏已入环数据 */
  reset_usb();
  for (index = 0; index < 200u; ++index) usb_cdc_port_on_rx(&data[index % 200], 1);
  check(usb_stats.isr_dropped > 0u, 5);
  usb_cdc_port_rx_pump();
  check(usb_stats.pumped_bytes + usb_stats.isr_dropped == 200u, 6);
  check(rx_seen_len == usb_stats.pumped_bytes, 7);

  /* TX：调用者栈缓冲在返回后可以被覆盖，USB 读到的仍是原内容 */
  reset_usb();
  memset(stack_buf, 0, sizeof(stack_buf));
  for (index = 0; index < sizeof(stack_buf); ++index) stack_buf[index] = (uint8_t)(0xA0u + index);
  check(usb_cdc_port_write(NULL, stack_buf, 40) == 40u, 8);
  check(usb_stats.tx_frames == 1u && usb_pending_len == 40u, 9);
  memset(stack_buf, 0x5A, sizeof(stack_buf));   /* 模拟调用者复用栈/缓冲 */
  memcpy(usb_captured, usb_pending_ptr, usb_pending_len);
  check(usb_captured[0] == 0xA0u && usb_captured[39] == 0xC7u, 10);

  /* BUSY：不覆盖在途缓冲，返回 0 并计数 */
  check(usb_cdc_port_write(NULL, stack_buf, 8) == 0u, 11);
  check(usb_stats.tx_busy_dropped == 1u, 12);
  check(usb_pending_len == 40u, 13);
  memcpy(usb_captured, usb_pending_ptr, usb_pending_len);
  check(usb_captured[0] == 0xA0u && usb_captured[7] == 0xA7u &&
        usb_captured[39] == 0xC7u, 19);
  usb_busy = 0; /* 假 USB 完成 */
  check(usb_cdc_port_write(NULL, stack_buf, 8) == 8u, 14);
  check(usb_stats.tx_frames == 2u, 15);

  /* 未枚举 / 未绑定类数据：返回 0，不发起发送 */
  reset_usb();
  hUsbDeviceHS.dev_state = 0;
  check(usb_cdc_port_write(NULL, stack_buf, 8) == 0u && usb_stats.tx_rejected == 1u, 16);
  reset_usb();
  hUsbDeviceHS.pClassData = 0;
  check(usb_cdc_port_write(NULL, stack_buf, 8) == 0u && usb_stats.tx_rejected == 1u, 17);

  /* 超过暂存上限：整帧拒绝，不能把截断前缀发到协议字节流。 */
  reset_usb();
  {
    static uint8_t huge[600];
    check(usb_cdc_port_write(NULL, huge, sizeof(huge)) == 0u &&
          usb_stats.tx_rejected == 1u && usb_pending_ptr == 0, 18);
  }

  
  reset_usb();
  check(usb_cdc_port_write(NULL, NULL, 4u) == 0u && usb_stats.tx_rejected == 1u, 20);
  reset_usb();
  check(usb_cdc_port_write(NULL, stack_buf, 0u) == 0u && usb_stats.tx_rejected == 1u, 21);
  reset_usb();
  {
    static uint8_t exact[512];
    memset(exact, 0x3C, sizeof(exact));
    check(usb_cdc_port_write(NULL, exact, sizeof(exact)) == 512u &&
          usb_pending_len == 512u && usb_stats.tx_frames == 1u, 22);
    memset(exact, 0x99, sizeof(exact));
    check(usb_pending_ptr[0] == 0x3Cu && usb_pending_ptr[511] == 0x3Cu, 23);
  }
test_result.status = 0x600D;
  __asm("bkpt 0");
}
