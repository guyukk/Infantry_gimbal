
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define HAL_UART_STATE_READY 0u
#define HAL_UART_STATE_BUSY_TX 1u
#define HAL_OK 0u
#define HAL_ERROR 1u
typedef struct { uint32_t gState; } UART_HandleTypeDef;
UART_HandleTypeDef huart10;
static volatile uint32_t vofa_dropped, vofa_sent;
/* The large trailing canary safely exposes the complete old uint8_t range. */
static struct { uint8_t before[16], payload[256], after[1024]; } guard;
static float input[255];
static uint8_t *pending;
static uint16_t pending_length;
static uint32_t sends, fail_send;
uint32_t HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *buf, uint16_t length) {
  (void)uart;
  ++sends;
  if (fail_send) return HAL_ERROR;
  pending = buf;
  pending_length = length;
  huart10.gState = HAL_UART_STATE_BUSY_TX;
  return HAL_OK;
}
volatile struct { uint32_t status, checks, failure; } test_result;
void Vofa_JustFloat(float *_data, uint8_t _num) {
    
    static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    uint16_t length;

    if (_data == NULL || _num == 0U)
    {
        vofa_dropped++;
        return;
    }
    length = (uint16_t)(4U * _num + 4U);
    if (length > sizeof(guard.payload))
    {
        vofa_dropped++;
        return;
    }
    if (huart10.gState != HAL_UART_STATE_READY)
    {
        vofa_dropped++;
        return;
    }
    memcpy(guard.payload, _data, 4U * _num);                       
    memcpy(guard.payload + 4U * _num, tail, 4);                   
    if (HAL_UART_Transmit_IT(&huart10, guard.payload, length) == HAL_OK)
    {
        vofa_sent++;
    }
    else
    {
        vofa_dropped++;
    }
}
static void check(int ok, uint32_t id) {
  ++test_result.checks;
  if (!ok && !test_result.failure) test_result.failure = id;
}
static int all_bytes(const uint8_t *p, uint32_t size, uint8_t value) {
  uint32_t i;
  for (i = 0; i < size; ++i) if (p[i] != value) return 0;
  return 1;
}
static void reset(void) {
  memset(&guard, 0xA5, sizeof(guard));
  huart10.gState = HAL_UART_STATE_READY;
  sends = fail_send = vofa_dropped = vofa_sent = 0;
  pending = 0; pending_length = 0;
}
void test_entry(void) {
  uint32_t n, i;
  static uint8_t saved[256];
  const uint8_t tail[4] = {0, 0, 0x80, 0x7F};
  for (i = 0; i < 255; ++i) input[i] = (float)i + 0.25f;
  reset(); Vofa_JustFloat(0, 10);
  check(!sends && vofa_dropped == 1, 1);
  reset(); Vofa_JustFloat(input, 0);
  check(!sends && vofa_dropped == 1, 2);
  for (n = 1; n <= 255; ++n) {
    reset();
    Vofa_JustFloat(input, (uint8_t)n);
    check(all_bytes(guard.before, sizeof(guard.before), 0xA5), 1000+n);
    check(all_bytes(guard.after, sizeof(guard.after), 0xA5), 2000+n);
    if (n <= 63) {
      check(sends == 1 && vofa_sent == 1 && !vofa_dropped, 3000+n);
      check(pending_length == 4*n+4, 4000+n);
      check(memcmp(pending, input, 4*n) == 0, 5000+n);
      check(memcmp(pending+4*n, tail, 4) == 0, 6000+n);
    } else {
      check(!sends && !vofa_sent && vofa_dropped == 1, 7000+n);
      check(all_bytes(guard.payload, sizeof(guard.payload), 0xA5), 8000+n);
    }
  }
  reset(); Vofa_JustFloat(input, 10);
  memcpy(saved, pending, pending_length);
  Vofa_JustFloat(input+10, 10);
  check(sends == 1 && vofa_dropped == 1 && vofa_sent == 1, 9001);
  check(memcmp(saved, pending, pending_length) == 0, 9002);
  huart10.gState = HAL_UART_STATE_READY;
  Vofa_JustFloat(input+10, 10);
  check(sends == 2 && memcmp(pending, input+10, 40) == 0, 9003);
  reset(); fail_send = 1; Vofa_JustFloat(input, 10);
  check(sends == 1 && !vofa_sent && vofa_dropped == 1, 9004);
  test_result.status = 0x600D;
  __asm("bkpt 0");
}
