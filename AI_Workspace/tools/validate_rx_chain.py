"""B02 receive-chain checks: real ARMCC-compiled parsing callbacks under Unicorn.

Three harnesses, each built from source function bodies extracted through the
project's ARMCC preprocessing (same route as validate_runtime.py):

  B02-01 chassis_rx : USART1 28-byte frame stream parser (P01)
  B02-02 rc_rx      : UART5 18-byte DBUS frame validation/publish (P03)
  B02-03 usb_handoff: CDC RX bounded hand-off ring and TX buffer lifetime (P08)

These are function/callback integration tests with HAL/USB stand-ins; they do
not prove peripheral or hardware behaviour.
"""
import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

from audit import read_source
from health_test_support import with_health
from project_paths import ARM_BIN, ARTIFACTS, CACHE, DEPENDENCIES, ROOT, project_path
from validate_invariants import functions

sys.path.insert(0, str(DEPENDENCIES))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import UC_ARM_REG_SP

DEFAULT_CACHE = CACHE / "validation/b02-rx-chain"
DEFAULT_RESULTS = ARTIFACTS / "optimization-2026-11-14/20261114-174009/B02/tests/rx-chain"


def define(text, name):
    match = re.search(r"^\s*#define\s+" + re.escape(name) + r"\s+(.+?)\s*(?:/\*.*)?$",
                      text, re.MULTILINE)
    if not match:
        raise RuntimeError("missing define: " + name)
    return match.group(1).strip()


def build(name, source, cache, results):
    source=with_health(source)
    cache.mkdir(parents=True, exist_ok=True)
    results.mkdir(parents=True, exist_ok=True)
    code = cache / (name + ".c")
    obj = cache / (name + ".o")
    image = cache / (name + ".axf")
    code.write_text(source, encoding="utf-8")
    for command in (
        [str(ARM_BIN / "armcc.exe"), "--cpu", "Cortex-M7", "--thumb", "--c99",
         "-O0", "-c", str(code), "-o", str(obj)],
        [str(ARM_BIN / "armlink.exe"), "--ro_base", "0x10000", "--rw_base", "0x20000000",
         "--entry", "test_entry", str(obj), "-o", str(image)],
    ):
        done = subprocess.run(command, capture_output=True, timeout=60)
        if done.returncode:
            raise RuntimeError(name + "\n" + done.stdout.decode("gb18030", errors="replace")
                               + done.stderr.decode("gb18030", errors="replace"))
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.mem_map(0, 0x100000)
    uc.mem_map(0x20000000, 0x100000)
    with image.open("rb") as stream:
        elf = ELFFile(stream)
        for segment in elf.iter_segments():
            if segment["p_type"] == "PT_LOAD":
                uc.mem_write(segment["p_vaddr"], segment.data())
        for section in elf.iter_sections():
            if section["sh_flags"] & 2 and section["sh_type"] == "SHT_PROGBITS":
                uc.mem_write(section["sh_addr"], section.data())
        symbols = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
        entry = elf["e_entry"]
    uc.reg_write(UC_ARM_REG_SP, 0x200FF000)

    def stop_on_breakpoint(machine, address, size, _):
        if machine.mem_read(address, 2) == b"\x00\xbe":
            machine.emu_stop()

    uc.hook_add(UC_HOOK_CODE, stop_on_breakpoint)
    uc.emu_start(entry | 1, 0, timeout=20000000, count=4000000)
    raw = bytes(uc.mem_read(symbols["test_result"], 12))
    status, checks, failure = (int.from_bytes(raw[i:i + 4], "little") for i in range(0, 12, 4))
    outcome = {"name": name, "status": hex(status), "checks": checks, "failure": failure,
               "passed": status == 0x600D and failure == 0}
    (results / (name + "-results.json")).write_text(json.dumps(outcome, indent=2), encoding="utf-8")
    return outcome


# ---------------------------------------------------------------------------
# B02-01 chassis 28-byte stream parser (P01)
# ---------------------------------------------------------------------------
def chassis_source():
    header = read_source(ROOT / "User/Bsp/bsp_usart.h")
    names = ["CHASSIS_DATA_LENGTH", "USART1_RX_DMA_SIZE", "CHASSIS_FRAME_HEAD",
             "CHASSIS_FRAME_TAIL", "CHASSIS_ANGLE_LIMIT_RAD", "CHASSIS_SPEED_LIMIT_MPS",
             "CHASSIS_WZ_LIMIT_RADPS", "CHASSIS_MODE_MAX", "SBUS_RX_BUF_NUM"]
    values = {name: define(header, name) for name in names}
    code = functions(read_source(ROOT / "User/Bsp/bsp_usart.c"))
    order = ["chassis_frame_parse", "chassis_rx_publish", "chassis_rx_feed_byte",
             "chassis_rx_consume_range", "chassis_rx_restart", "chassis_rx_on_event",
             "chassis_data_read", "chassis_rx_feed_stream", "chassis_rx_reset", "HAL_UART_ErrorCallback"]
    signatures = {
        "chassis_frame_parse": "static uint8_t chassis_frame_parse(const uint8_t *frame, chassis_data_t *out)",
        "chassis_rx_publish": "static void chassis_rx_publish(const uint8_t *frame, const chassis_data_t *parsed)",
        "chassis_rx_feed_byte": "static void chassis_rx_feed_byte(uint8_t value)",
        "chassis_rx_consume_range": "static void chassis_rx_consume_range(uint32_t from, uint32_t to)",
        "chassis_rx_restart": "static void chassis_rx_restart(void)",
        "chassis_rx_on_event": "static void chassis_rx_on_event(uint16_t size)",
        "chassis_data_read": "uint8_t chassis_data_read(chassis_data_t *out, uint32_t *generation)",
        "chassis_rx_feed_stream": "void chassis_rx_feed_stream(const uint8_t *data, uint16_t len)",
        "chassis_rx_reset": "void chassis_rx_reset(void)",
        "HAL_UART_ErrorCallback": "void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)",
    }
    joins = [signatures[name] + " " + code[name] + "\n" for name in order]
    return r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
typedef float fp32;
typedef union { float fp32; uint8_t bytes[4]; } fp32_to_bytes;
typedef struct
{
    fp32 chassis_vx, chassis_vy, chassis_wz;
    fp32 chassis_yaw, chassis_pitch, chassis_roll, cap_voltage;
    uint8_t chassis_mode, robot_id;
} chassis_data_t;
typedef struct
{
    uint32_t bytes_seen, frames_ok, frames_bad_tail, frames_bad_field;
    uint32_t bytes_discarded, overflow, restarts, dma_events;
} chassis_rx_stats_t;
typedef struct { uint32_t NDTR; } TestDMAStream;
typedef TestDMAStream DMA_Stream_TypeDef;
typedef struct { TestDMAStream *Instance; } TestDMAHandle;
typedef struct { void *Instance; uint32_t gState; TestDMAHandle *hdmarx; } UART_HandleTypeDef;
#define HAL_OK 0
#define USART1 ((void *)0x40011000)
#define UART5  ((void *)0x40005000)
#define USART10 ((void *)0x4000F000)
#define __DMB() ((void)0)
''' + "\n".join("#define %s %s" % (k, v) for k, v in values.items()) + r'''
uint8_t *last_dma_buf;
uint16_t last_dma_size;
uint32_t dma_start_count;
uint32_t HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *b, uint16_t l) {
  (void)h;
  last_dma_buf = b;
  last_dma_size = l;
  dma_start_count++;
  return HAL_OK;
}
uint32_t HAL_GetTick(void) { return 0u; }
void remote_control_rx_chunk(const uint8_t *d, uint16_t l) { (void)d; (void)l; }
void remote_control_rx_restart(UART_HandleTypeDef *h) { (void)h; }
void remote_control_rx_overflow(void) { }
void tfmini_uart_error_callback(UART_HandleTypeDef *h) { (void)h; }
UART_HandleTypeDef huart1;
TestDMAStream dma_stream;
TestDMAHandle dma_handle = { &dma_stream };
uint8_t remote_buff[SBUS_RX_BUF_NUM];
struct { uint8_t before[8]; uint8_t value[USART1_RX_DMA_SIZE]; uint8_t after[8]; } chassis_guard;
struct { uint8_t before[8]; uint8_t value[CHASSIS_DATA_LENGTH]; uint8_t after[8]; } raw_guard;
struct { uint8_t before[8]; chassis_data_t value; uint8_t after[8]; } data_guard;
#define chassis_rx_dma chassis_guard.value
#define gimbal_receive_data raw_guard.value
#define chassis_data data_guard.value
typedef struct { uint8_t frame[CHASSIS_DATA_LENGTH]; uint8_t length; chassis_rx_stats_t stats; } chassis_rx_parser_t;
static chassis_rx_parser_t chassis_rx;
static uint32_t chassis_rx_last_pos;
static volatile uint32_t chassis_rx_seq;
volatile struct { uint32_t status, checks, failure; } test_result;
''' + "".join(joins) + r'''
void check(int condition, uint32_t number) {
  ++test_result.checks;
  if (!condition && test_result.failure == 0u) test_result.failure = number;
}
static void make_frame(uint8_t *f, float yaw, float pitch, float roll, uint8_t mode,
                       uint8_t id, float vx, float vy, float wz) {
  f[0] = CHASSIS_FRAME_HEAD;
  memcpy(f + 1, &yaw, 4);
  memcpy(f + 5, &pitch, 4);
  memcpy(f + 9, &roll, 4);
  f[13] = mode; f[14] = id;
  memcpy(f + 15, &vx, 4);
  memcpy(f + 19, &vy, 4);
  memcpy(f + 23, &wz, 4);
  f[CHASSIS_DATA_LENGTH - 1] = CHASSIS_FRAME_TAIL;
}
static void stream(const uint8_t *data, uint32_t len) {
  chassis_rx_feed_stream(data, (uint16_t)len);
}
static void reset_all(void) {
  huart1.hdmarx = &dma_handle;
  dma_stream.NDTR = USART1_RX_DMA_SIZE;
  memset(&chassis_rx, 0, sizeof(chassis_rx));
  chassis_rx_last_pos = 0;
  chassis_rx_seq = 0;
  memset(&chassis_guard, 0xCD, sizeof(chassis_guard));
  memset(&raw_guard, 0xCD, sizeof(raw_guard));
  memset(&data_guard, 0xCD, sizeof(data_guard));
}
static int guards_ok(void) {
  uint32_t i;
  for (i = 0; i < sizeof(chassis_guard.before); ++i)
    if (chassis_guard.before[i] != 0xCD || chassis_guard.after[i] != 0xCD) return 0;
  for (i = 0; i < sizeof(raw_guard.before); ++i)
    if (raw_guard.before[i] != 0xCD || raw_guard.after[i] != 0xCD) return 0;
  for (i = 0; i < sizeof(data_guard.before); ++i)
    if (data_guard.before[i] != 0xCD || data_guard.after[i] != 0xCD) return 0;
  return 1;
}
static int fields_ok(const chassis_data_t *c) {
  return c->chassis_yaw == 0.25f && c->chassis_pitch == -0.5f &&
         c->chassis_roll == 1.5f && c->chassis_mode == 1u && c->robot_id == 103u &&
         c->chassis_vx == 1.25f && c->chassis_vy == -0.75f && c->chassis_wz == 2.5f;
}
void test_entry(void) {
  uint8_t frame[CHASSIS_DATA_LENGTH];
  uint8_t frame2[CHASSIS_DATA_LENGTH];
  uint8_t buffer[128];
  uint8_t garbage[64];
  chassis_data_t snapshot;
  uint32_t generation;
  uint32_t short_len;

  reset_all();
  make_frame(frame, 0.25f, -0.5f, 1.5f, 1u, 103u, 1.25f, -0.75f, 2.5f);

  /* 完整帧：字段逐项正确，发布计数=1，原始帧留证 */
  stream(frame, CHASSIS_DATA_LENGTH);
  check(chassis_rx.stats.frames_ok == 1u, 1);
  check(fields_ok(&chassis_data), 2);
  check(memcmp(raw_guard.value, frame, CHASSIS_DATA_LENGTH) == 0, 3);
  check(chassis_rx_seq == 2u && chassis_rx.length == 0u, 4);
  check(chassis_data_read(&snapshot, &generation) == 1u && generation == 1u &&
        fields_ok(&snapshot), 5);

  /* 0..27 字节短包都不发布 */
  for (short_len = 0; short_len < CHASSIS_DATA_LENGTH; ++short_len) {
    reset_all();
    stream(frame, short_len);
    check(chassis_rx.stats.frames_ok == 0u, 100u + short_len);
    check(chassis_rx.length == short_len, 200u + short_len);
  }

  /* 帧起点 0..27：前置任意字节数后整帧仍被接收 */
  {
    uint32_t prefix;
    for (prefix = 0; prefix < CHASSIS_DATA_LENGTH; ++prefix) {
      reset_all();
      memset(garbage, 0xAA, sizeof(garbage));
      stream(garbage, prefix);
      stream(frame, CHASSIS_DATA_LENGTH);
      check(chassis_rx.stats.frames_ok == 1u, 300u + prefix);
      check(fields_ok(&chassis_data), 400u + prefix);
    }
  }

  /* 无帧头：全部丢弃且不发布 */
  reset_all();
  memset(garbage, 0x00, sizeof(garbage));
  stream(garbage, sizeof(garbage));
  check(chassis_rx.stats.frames_ok == 0u && chassis_rx.length == 0u, 6);
  check(chassis_rx.stats.bytes_discarded == sizeof(garbage), 7);

  /* 帧头出现在 float 载荷：伪起点被拒后必须找回真正的帧头 */
  reset_all();
  memset(buffer, 0, sizeof(buffer));
  buffer[0] = CHASSIS_FRAME_HEAD;            /* 噪声里的 's' */
  buffer[5] = CHASSIS_FRAME_HEAD;            /* 真帧头插在噪声之后 */
  memcpy(buffer + 6, frame + 1, CHASSIS_DATA_LENGTH - 1);
  stream(buffer, 5);
  check(chassis_rx.stats.frames_ok == 0u, 8);
  stream(buffer + 5, CHASSIS_DATA_LENGTH);
  check(chassis_rx.stats.frames_ok == 1u, 9);
  check(fields_ok(&chassis_data), 10);

  /* 错尾：整帧丢弃，后续合法帧恢复 */
  reset_all();
  memcpy(frame2, frame, CHASSIS_DATA_LENGTH);
  frame2[CHASSIS_DATA_LENGTH - 1] = 'x';
  stream(frame2, CHASSIS_DATA_LENGTH);
  check(chassis_rx.stats.frames_ok == 0u && chassis_rx.stats.frames_bad_tail == 1u, 11);
  stream(frame, CHASSIS_DATA_LENGTH);
  check(chassis_rx.stats.frames_ok == 1u && fields_ok(&chassis_data), 12);

  /* 逐字节拆包 */
  reset_all();
  {
    uint32_t index;
    for (index = 0; index < CHASSIS_DATA_LENGTH; ++index) {
      chassis_rx_feed_byte(frame[index]);
      if (index + 1u < CHASSIS_DATA_LENGTH)
        check(chassis_rx.stats.frames_ok == 0u, 500u + index);
    }
  }
  check(chassis_rx.stats.frames_ok == 1u && fields_ok(&chassis_data), 13);

  /* 同一事件两帧/三帧粘包（64 字节缓冲分两次事件） */
  reset_all();
  memcpy(buffer, frame, CHASSIS_DATA_LENGTH);
  memcpy(buffer + 28, frame, CHASSIS_DATA_LENGTH);
  memcpy(chassis_rx_dma, buffer, 56);
  dma_stream.NDTR = 8;
  chassis_rx_on_event(56);
  check(chassis_rx.stats.frames_ok == 2u && chassis_rx.stats.dma_events == 1u, 14);
  check(chassis_rx_last_pos == 56u, 15);
  /* A pending HT event carries an older Size, while DMA still points at 56. */
  chassis_rx_on_event(32);
  check(chassis_rx.stats.frames_ok == 2u && chassis_rx.length == 0u, 39);

  /* 环形环绕：64 字节整轮 = 2 帧 + 8 字节，再补 20 字节完成第 3 帧 */
  reset_all();
  memcpy(chassis_rx_dma, frame, 28);
  memcpy(chassis_rx_dma + 28, frame, 28);
  memcpy(buffer, frame, 28);
  memcpy(chassis_rx_dma + 56, buffer, 8);
  dma_stream.NDTR = 0;
  chassis_rx_on_event(64);
  check(chassis_rx.stats.frames_ok == 2u && chassis_rx_last_pos == 64u, 16);
  /* TC + IDLE at the same exclusive end must leave the 8-byte partial frame intact. */
  chassis_rx_on_event(64);
  check(chassis_rx.stats.frames_ok == 2u && chassis_rx.length == 8u, 38);
  memcpy(chassis_rx_dma, buffer + 8, 20);
  dma_stream.NDTR = 44;
  chassis_rx_on_event(20);
  check(chassis_rx.stats.frames_ok == 3u && chassis_rx_last_pos == 20u, 17);
  check(fields_ok(&chassis_data), 18);

  /* 噪声后恢复 */
  reset_all();
  {
    uint32_t index;
    for (index = 0; index < 200u; ++index) chassis_rx_feed_byte((uint8_t)(index * 37u));
  }
  check(chassis_rx.stats.frames_ok == 0u, 19);
  stream(frame, CHASSIS_DATA_LENGTH);
  check(chassis_rx.stats.frames_ok == 1u && fields_ok(&chassis_data), 20);

  /* NaN/Inf 与越界枚举拒绝 */
  reset_all();
  make_frame(frame2, NAN, -0.5f, 1.5f, 1u, 103u, 0.0f, 0.0f, 0.0f);
  stream(frame2, CHASSIS_DATA_LENGTH);
  check(chassis_rx.stats.frames_ok == 0u && chassis_rx.stats.frames_bad_field == 1u, 21);
  reset_all();
  make_frame(frame2, 0.25f, -0.5f, 1.5f, 3u, 103u, 0.0f, 0.0f, 0.0f);
  stream(frame2, CHASSIS_DATA_LENGTH);
  check(chassis_rx.stats.frames_ok == 0u && chassis_rx.stats.frames_bad_field == 1u, 22);
  reset_all();
  make_frame(frame2, 0.25f, -0.5f, 1.5f, 1u, 103u, 0.0f, 0.0f, 100.0f);
  stream(frame2, CHASSIS_DATA_LENGTH);
  check(chassis_rx.stats.frames_ok == 0u, 23);

  /* 错误后恢复：解析器与消费位置重置，旧帧不得冒充新帧 */
  reset_all();
  stream(frame, 10);
  chassis_rx.length = 0;
  chassis_rx_last_pos = 0;
  dma_start_count = 0;
  chassis_rx_restart();
  check(last_dma_buf == chassis_rx_dma && last_dma_size == USART1_RX_DMA_SIZE, 24);
  check(chassis_rx_last_pos == 0u && dma_start_count == 1u, 25);
  stream(frame, CHASSIS_DATA_LENGTH);
  check(chassis_rx.stats.frames_ok == 1u && fields_ok(&chassis_data), 26);

  /* 非法事件长度：有界丢弃并重启 */
  reset_all();
  chassis_rx_on_event(USART1_RX_DMA_SIZE + 1u);
  check(chassis_rx.stats.overflow == 1u && chassis_rx_last_pos == 0u, 27);

  /* 512/28 缓冲错配回归：启动与错误重启必须使用同一个 DMA 缓冲与长度 */
  reset_all();
  dma_start_count = 0;
  chassis_rx_restart();
  check(last_dma_buf == chassis_rx_dma && last_dma_size == USART1_RX_DMA_SIZE, 30);
  huart1.Instance = USART1;
  reset_all();
  stream(frame, 10);
  HAL_UART_ErrorCallback(&huart1);
  check(last_dma_buf == chassis_rx_dma && last_dma_size == USART1_RX_DMA_SIZE, 31);
  check(chassis_rx.length == 0u && chassis_rx_last_pos == 0u, 32);
  stream(frame, CHASSIS_DATA_LENGTH);
  check(chassis_rx.stats.frames_ok == 1u && fields_ok(&chassis_data), 33);

  /* start_index 残留回归：已发布帧 A 后喂不含帧头的字节，业务状态不得被改写 */
  reset_all();
  stream(frame, CHASSIS_DATA_LENGTH);
  check(fields_ok(&chassis_data), 34);
  {
    uint8_t noise[27];
    memset(noise, 0x5A, sizeof(noise));   /* 'Z'，不含 's' 与帧尾 */
    stream(noise, sizeof(noise));
    check(chassis_rx.stats.frames_ok == 1u, 35);   /* 仍只有帧 A */
  }
  check(fields_ok(&chassis_data), 36);
  check(chassis_rx.length == 0u && chassis_rx.stats.bytes_discarded == 27u, 37);

  check(guards_ok(), 28);
  check(chassis_rx.stats.bytes_seen > 0u, 29);
  test_result.status = 0x600D;
  __asm("bkpt 0");
}
'''


# ---------------------------------------------------------------------------
# B02-02 UART5 DBUS frame validation (P03)
# ---------------------------------------------------------------------------
def rc_source():
    header = read_source(ROOT / "User/APP/remote_control.h")
    bus_header = read_source(ROOT / "User/Bsp/bsp_usart.h")
    values = {name: define(header, name)
              for name in ["RC_CH_VALUE_MIN_RAW", "RC_CH_VALUE_MAX_RAW"]}
    values["RC_FRAME_LENGTH"] = define(bus_header, "RC_FRAME_LENGTH")
    code = functions(read_source(ROOT / "User/APP/remote_control.c"))
    order = ["rc_frame_parse", "sbus_to_rc", "rc_frame_validate", "rc_publish",
             "rc_try_frame", "remote_control_rx_chunk", "remote_control_rx_overflow",
             "rc_ctrl_snapshot"]

    signatures = {
        "rc_frame_parse": "static void rc_frame_parse(const uint8_t *sbus_buf, RC_ctrl_t *out)",
        "sbus_to_rc": "void sbus_to_rc(uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl, remoter_t *remoter)",
        "rc_frame_validate": "static uint8_t rc_frame_validate(const uint8_t *sbus_buf)",
        "rc_publish": "static void rc_publish(const RC_ctrl_t *parsed)",
        "rc_try_frame": "static void rc_try_frame(const uint8_t *frame)",
        "remote_control_rx_chunk": "void remote_control_rx_chunk(const uint8_t *data, uint16_t len)",
        "remote_control_rx_overflow": "void remote_control_rx_overflow(void)",
        "rc_ctrl_snapshot": "uint8_t rc_ctrl_snapshot(RC_ctrl_t *out, uint32_t *generation)",
    }
    joins = [signatures[name] + " " + code[name] + "\n" for name in order]
    return r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef __packed struct
{
    __packed struct { int16_t ch[5]; char s[2]; } rc;
    __packed struct { int16_t x, y, z; uint8_t press_l, press_r; } mouse;
    __packed struct { uint16_t v; } key;
    uint32_t last_fdb;
    volatile uint32_t generation;
} RC_ctrl_t;
typedef struct { uint32_t chunks, frames_ok, bad_channel, bad_switch, rejected_chunks,
                 discarded_bytes, overflow, restarts; } rc_rx_stats_t;
typedef struct { uint8_t ch[5]; uint8_t s; } remoter_t;
typedef struct { void *Instance; } UART_HandleTypeDef;
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_SW_UP 1u
#define RC_SW_MID 3u
#define RC_SW_DOWN 2u
#define __DMB() ((void)0)
''' + "\n".join("#define %s %s" % (k, v) for k, v in values.items()) + r'''
#define RC_CH_VALUE_VALID(s) (((s) >= RC_CH_VALUE_MIN_RAW) && ((s) <= RC_CH_VALUE_MAX_RAW))
#define RC_SWITCH_VALID(s) ((s) >= RC_SW_UP && (s) <= RC_SW_MID)
uint32_t fake_tick = 0x1234u;
uint32_t HAL_GetTick(void) { return fake_tick; }
RC_ctrl_t rc_ctrl;
remoter_t remoter;
static volatile uint32_t rc_publish_seq;
static uint32_t rc_frame_count;
static rc_rx_stats_t rc_rx_stats;
volatile struct { uint32_t status, checks, failure; } test_result;
''' + "".join(joins) + r'''
void check(int condition, uint32_t number) {
  ++test_result.checks;
  if (!condition && test_result.failure == 0u) test_result.failure = number;
}
static uint16_t low11(int16_t value) { return (uint16_t)(value + 1024); }
static void make_rc(uint8_t *f, int16_t ch0, int16_t ch1, int16_t ch2, int16_t ch3,
                    uint8_t sw0, uint8_t sw1) {
  uint64_t bits = (uint64_t)low11(ch0)
                | ((uint64_t)low11(ch1) << 11)
                | ((uint64_t)low11(ch2) << 22)
                | ((uint64_t)low11(ch3) << 33)
                | ((uint64_t)(sw0 & 0x03u) << 44)
                | ((uint64_t)(sw1 & 0x03u) << 46);
  memset(f, 0, RC_FRAME_LENGTH);
  f[0] = (uint8_t)(bits & 0xFF);
  f[1] = (uint8_t)((bits >> 8) & 0xFF);
  f[2] = (uint8_t)((bits >> 16) & 0xFF);
  f[3] = (uint8_t)((bits >> 24) & 0xFF);
  f[4] = (uint8_t)((bits >> 32) & 0xFF);
  f[5] = (uint8_t)((bits >> 40) & 0xFF);
  f[6] = 0x11; f[7] = 0x22;
  f[8] = 0x33; f[9] = 0x44;
  f[10] = 0x55; f[11] = 0x66;
  f[12] = 1; f[13] = 0;
  f[14] = 0x78; f[15] = 0x56;
  f[16] = 0x00; f[17] = 0x00;
}
static void reset_all(void) {
  memset(&rc_ctrl, 0, sizeof(rc_ctrl));
  memset(&rc_rx_stats, 0, sizeof(rc_rx_stats));
  rc_publish_seq = 0;
  rc_frame_count = 0;
}
void test_entry(void) {
  uint8_t frame[RC_FRAME_LENGTH];
  uint8_t block[2 * RC_FRAME_LENGTH];
  uint16_t length;
  RC_ctrl_t snapshot;
  uint32_t generation;

  /* 正常帧：字段、时间戳、generation */
  reset_all();
  fake_tick = 0xFFFFF000u;
  make_rc(frame, 100, -200, 300, -400, RC_SW_UP, RC_SW_DOWN);
  remote_control_rx_chunk(frame, RC_FRAME_LENGTH);
  check(rc_rx_stats.frames_ok == 1u, 1);
  check(rc_ctrl.rc.ch[0] == 100 && rc_ctrl.rc.ch[1] == -200 &&
        rc_ctrl.rc.ch[2] == 300 && rc_ctrl.rc.ch[3] == -400, 2);
  check(rc_ctrl.rc.s[0] == RC_SW_UP && rc_ctrl.rc.s[1] == RC_SW_DOWN, 3);
  check(rc_ctrl.last_fdb == 0xFFFFF000u, 4);
  check(rc_ctrl.generation == 1u, 5);
  check(rc_ctrl.mouse.x == 0x2211 && rc_ctrl.mouse.y == 0x4433 &&
        rc_ctrl.mouse.z == 0x6655 && rc_ctrl.mouse.press_l == 1 &&
        rc_ctrl.mouse.press_r == 0 && rc_ctrl.key.v == 0x5678, 6);
  check(rc_ctrl_snapshot(&snapshot, &generation) == 1u && generation == 1u &&
        snapshot.rc.ch[0] == 100, 7);

  /* 长度 0..17 / 19..37（含 36 以外）都整块拒绝，不刷新时间戳与 generation */
  for (length = 0; length <= 37; length++) {
    if (length == RC_FRAME_LENGTH || length == 2 * RC_FRAME_LENGTH) continue;
    reset_all();
    make_rc(frame, 100, 100, 100, 100, RC_SW_UP, RC_SW_UP);
    remote_control_rx_chunk(frame, length);
    check(rc_rx_stats.frames_ok == 0u, 100u + length);
    check(rc_rx_stats.rejected_chunks == 1u, 200u + length);
    check(rc_ctrl.last_fdb == 0u && rc_ctrl.generation == 0u, 300u + length);
  }

  /* 36 字节块 = 两帧 */
  reset_all();
  make_rc(block, 10, 20, 30, 40, RC_SW_UP, RC_SW_MID);
  make_rc(block + RC_FRAME_LENGTH, -10, -20, -30, -40, RC_SW_DOWN, RC_SW_UP);
  remote_control_rx_chunk(block, 2 * RC_FRAME_LENGTH);
  check(rc_rx_stats.frames_ok == 2u && rc_ctrl.generation == 2u, 8);
  check(rc_ctrl.rc.ch[0] == -10 && rc_ctrl.rc.ch[3] == -40, 9);

  /* 36 字节块中一帧开关非法：只发布另一帧 */
  reset_all();
  make_rc(block, 10, 20, 30, 40, RC_SW_UP, RC_SW_MID);
  make_rc(block + RC_FRAME_LENGTH, 10, 20, 30, 40, 0, RC_SW_MID);
  remote_control_rx_chunk(block, 2 * RC_FRAME_LENGTH);
  check(rc_rx_stats.frames_ok == 1u && rc_rx_stats.bad_switch == 1u, 10);

  /* 开关枚举 0 非法 */
  reset_all();
  make_rc(frame, 100, 100, 100, 100, 0, RC_SW_MID);
  remote_control_rx_chunk(frame, RC_FRAME_LENGTH);
  check(rc_rx_stats.frames_ok == 0u && rc_rx_stats.bad_switch == 1u, 11);
  check(rc_ctrl.last_fdb == 0u, 12);

  /* 通道 ±700 边界接受，±701 拒绝 */
  reset_all();
  make_rc(frame, 700, -700, 699, -699, RC_SW_UP, RC_SW_UP);
  remote_control_rx_chunk(frame, RC_FRAME_LENGTH);
  check(rc_rx_stats.frames_ok == 1u && rc_ctrl.rc.ch[0] == 700 &&
        rc_ctrl.rc.ch[1] == -700, 13);
  reset_all();
  make_rc(frame, 701, -700, 0, 0, RC_SW_UP, RC_SW_UP);
  remote_control_rx_chunk(frame, RC_FRAME_LENGTH);
  check(rc_rx_stats.frames_ok == 0u && rc_rx_stats.bad_channel == 1u, 14);
  reset_all();
  make_rc(frame, 0, 0, -701, 0, RC_SW_UP, RC_SW_UP);
  remote_control_rx_chunk(frame, RC_FRAME_LENGTH);
  check(rc_rx_stats.frames_ok == 0u && rc_rx_stats.bad_channel == 1u, 15);

  /* 坏帧后好帧恢复；坏帧不改变已有值 */
  reset_all();
  make_rc(frame, 50, 60, 70, 80, RC_SW_UP, RC_SW_UP);
  remote_control_rx_chunk(frame, RC_FRAME_LENGTH);
  make_rc(frame, 701, 0, 0, 0, RC_SW_UP, RC_SW_UP);
  remote_control_rx_chunk(frame, RC_FRAME_LENGTH);
  check(rc_rx_stats.frames_ok == 1u && rc_ctrl.rc.ch[0] == 50, 16);
  make_rc(frame, 11, 22, 33, 44, RC_SW_MID, RC_SW_MID);
  remote_control_rx_chunk(frame, RC_FRAME_LENGTH);
  check(rc_rx_stats.frames_ok == 2u && rc_ctrl.rc.ch[0] == 11 &&
        rc_ctrl.generation == 2u, 17);

  /* 计时回绕 */
  reset_all();
  fake_tick = 0xFFFFFFFFu;
  make_rc(frame, 1, 2, 3, 4, RC_SW_UP, RC_SW_UP);
  remote_control_rx_chunk(frame, RC_FRAME_LENGTH);
  check(rc_ctrl.last_fdb == 0xFFFFFFFFu, 18);
  fake_tick = 5u;
  make_rc(frame, 5, 6, 7, 8, RC_SW_UP, RC_SW_UP);
  remote_control_rx_chunk(frame, RC_FRAME_LENGTH);
  check(rc_ctrl.last_fdb == 5u && rc_ctrl.generation == 2u, 19);

  /* 溢出路径 */
  reset_all();
  remote_control_rx_overflow();
  check(rc_rx_stats.overflow == 1u && rc_rx_stats.rejected_chunks == 1u, 20);

  test_result.status = 0x600D;
  __asm("bkpt 0");
}
'''


# ---------------------------------------------------------------------------
# B02-03 USB CDC hand-off ring and TX buffer lifetime (P08)
# ---------------------------------------------------------------------------
def usb_source():
    code = functions(read_source(ROOT / "User/Communication/example/device/usb_cdc_port.c"))
    order = ["usb_cdc_port_on_rx", "usb_cdc_port_rx_pump", "usb_cdc_port_write"]

    signatures = {
        "usb_cdc_port_on_rx": "void usb_cdc_port_on_rx(const uint8_t *data, uint32_t len)",
        "usb_cdc_port_rx_pump": "void usb_cdc_port_rx_pump(void)",
        "usb_cdc_port_write": "uint32_t usb_cdc_port_write(void *user, const uint8_t *data, uint32_t len)",
    }
    joins = [signatures[name] + " " + code[name] + "\n" for name in order]
    return r'''
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
''' + "".join(joins) + r'''
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

  test_result.status = 0x600D;
  __asm("bkpt 0");
}
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    parser.add_argument("--results", type=Path, default=DEFAULT_RESULTS)
    args = parser.parse_args()
    cache = project_path(args.cache)
    results = project_path(args.results)
    cache.mkdir(parents=True, exist_ok=True)
    results.mkdir(parents=True, exist_ok=True)
    outcomes = []
    failures = []
    for name, source in (("b02-chassis-rx", chassis_source()),
                         ("b02-rc-rx", rc_source()),
                         ("b02-usb-handoff", usb_source())):
        outcome = build(name, source, cache, results)
        outcomes.append(outcome)
        print(json.dumps(outcome))
        if not outcome["passed"]:
            failures.append(outcome)
    (results / "summary.json").write_text(json.dumps(outcomes, indent=2), encoding="utf-8")
    if failures:
        raise RuntimeError("receive-chain checks failed: " +
                           json.dumps([f["name"] + ": failure " + str(f["failure"]) for f in failures]))
    print("all receive-chain checks passed")


if __name__ == "__main__":
    main()
