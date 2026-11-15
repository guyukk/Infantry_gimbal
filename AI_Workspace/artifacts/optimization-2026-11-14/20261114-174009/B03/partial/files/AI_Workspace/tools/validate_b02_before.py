"""B02 before-change evidence: run the pre-change parsers against the same expectations.

The sources come from the immutable B02-before snapshot zip, are extracted to a
cache directory and compiled with ARMCC/Unicorn exactly like the after tests.
Each harness is expected to FAIL on the acceptance expectations; the tool records
the first failing expectation as the before-fix reproduction.
"""
import argparse
import json
import re
import subprocess
import sys
import zipfile
from pathlib import Path

from audit import read_source
from project_paths import ARM_BIN, ARTIFACTS, CACHE, DEPENDENCIES, project_path
from validate_invariants import functions

sys.path.insert(0, str(DEPENDENCIES))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import UC_ARM_REG_SP

SNAPSHOT = "AI_Workspace/snapshots/optimization-2026-11-14-20261114-174009-B02-before/gimbal-sources.zip"
DEFAULT_CACHE = CACHE / "validation/b02-before"
DEFAULT_RESULTS = ARTIFACTS / "optimization-2026-11-14/20261114-174009/B02/tests/before"


def extract(zip_path, root, names):
    with zipfile.ZipFile(zip_path) as archive:
        for name in names:
            target = root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(archive.read(name))


def build(name, source, cache, results):
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
               "before_passed_acceptance": status == 0x600D and failure == 0}
    (results / (name + "-results.json")).write_text(json.dumps(outcome, indent=2), encoding="utf-8")
    return outcome


def before_chassis(before_root):
    code = functions(read_source(before_root / "User/Bsp/bsp_usart.c"))
    return r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef float fp32;
typedef union { float fp32; uint8_t bytes[4]; } fp32_to_bytes;
typedef struct
{
    fp32 chassis_vx, chassis_vy, chassis_wz;
    fp32 chassis_yaw, chassis_pitch, chassis_roll, cap_voltage;
    uint8_t chassis_mode, robot_id;
} chassis_data_t;
#define CHASSIS_DATA_LENGTH 28
uint8_t gimbal_receive_data[CHASSIS_DATA_LENGTH];
chassis_data_t chassis_data;
volatile struct { uint32_t status, checks, failure; } test_result;
''' + "void chassis_to_gimbal(uint8_t *data_chassis) " + code["chassis_to_gimbal"] + "\n" \
  + "static void gimbal_memcpy_data_from_chassis(void) " + code["gimbal_memcpy_data_from_chassis"] + "\n" + r'''
void check(int condition, uint32_t number) {
  ++test_result.checks;
  if (!condition && test_result.failure == 0u) test_result.failure = number;
}
static void make_frame(uint8_t *f, float yaw) {
  fp32_to_bytes value = {.fp32 = yaw};
  memset(f, 0, CHASSIS_DATA_LENGTH);
  f[0] = 's';
  memcpy(f + 1, value.bytes, 4);
  f[27] = 'e';
}
void test_entry(void) {
  uint8_t frame[CHASSIS_DATA_LENGTH];
  uint8_t noise[CHASSIS_DATA_LENGTH];

  make_frame(frame, 0.25f);
  memcpy(gimbal_receive_data, frame, CHASSIS_DATA_LENGTH);
  gimbal_memcpy_data_from_chassis();
  check(chassis_data.chassis_yaw == 0.25f, 1);   /* 基线：合法帧可解析 */

  /* 期望：没有 's' 的新数据不得改写业务状态（旧实现复用 start_index 与 temp） */
  memset(noise, 0xAA, sizeof(noise));
  memcpy(gimbal_receive_data, noise, CHASSIS_DATA_LENGTH);
  gimbal_memcpy_data_from_chassis();
  check(chassis_data.chassis_yaw == 0.25f, 2);

  /* 期望：错尾拒绝（旧实现完全不校验尾字节） */
  make_frame(frame, 0.5f);
  frame[CHASSIS_DATA_LENGTH - 1] = 'x';
  memcpy(gimbal_receive_data, frame, CHASSIS_DATA_LENGTH);
  gimbal_memcpy_data_from_chassis();
  check(chassis_data.chassis_yaw == 0.25f, 3);

  /* 期望：半帧/短事件不发布（旧实现忽略 Size，只有 10 字节也会解析整块） */
  make_frame(frame, 0.75f);
  memcpy(gimbal_receive_data, frame, 10);
  memset(gimbal_receive_data + 10, 0x00, CHASSIS_DATA_LENGTH - 10);
  gimbal_memcpy_data_from_chassis();
  check(chassis_data.chassis_yaw == 0.25f && chassis_data.chassis_pitch == 0.0f, 4);

  test_result.status = 0x600D;
  __asm("bkpt 0");
}
'''


def before_rc(before_root):
    code = functions(read_source(before_root / "User/APP/remote_control.c"))
    return r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef float fp32;
typedef __packed struct
{
    __packed struct { int16_t ch[5]; char s[2]; } rc;
    __packed struct { int16_t x, y, z; uint8_t press_l, press_r; } mouse;
    __packed struct { uint16_t v; } key;
    uint32_t last_fdb;
} RC_ctrl_t;
typedef struct { uint8_t ch[5]; uint8_t s; } remoter_t;
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
uint32_t fake_tick = 1000u;
uint32_t HAL_GetTick(void) { return fake_tick; }
RC_ctrl_t rc_ctrl;
remoter_t remoter;
volatile struct { uint32_t status, checks, failure; } test_result;
''' + "void sbus_to_rc(uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl, remoter_t *remoter) " \
  + code["sbus_to_rc"] + "\n" + r'''
void check(int condition, uint32_t number) {
  ++test_result.checks;
  if (!condition && test_result.failure == 0u) test_result.failure = number;
}
static void make_rc(uint8_t *f, int16_t ch0, int16_t ch1, int16_t ch2, int16_t ch3,
                    uint8_t sw0, uint8_t sw1) {
  uint64_t bits = (uint64_t)(uint16_t)(ch0 + 1024)
                | ((uint64_t)(uint16_t)(ch1 + 1024) << 11)
                | ((uint64_t)(uint16_t)(ch2 + 1024) << 22)
                | ((uint64_t)(uint16_t)(ch3 + 1024) << 33)
                | ((uint64_t)(sw0 & 0x03u) << 44)
                | ((uint64_t)(sw1 & 0x03u) << 46);
  memset(f, 0, 18);
  for (int i = 0; i < 6; ++i) f[i] = (uint8_t)((bits >> (8 * i)) & 0xFF);
}
void test_entry(void) {
  uint8_t frame[18];

  make_rc(frame, 100, 200, 300, 400, 1, 3);
  sbus_to_rc(frame, &rc_ctrl, &remoter);
  check(rc_ctrl.rc.ch[0] == 100 && rc_ctrl.rc.s[0] == 1, 1);  /* 基线 */

  /* 期望：开关 0 非法，不发布、不刷新时间戳（旧实现直接发布） */
  fake_tick = 2000u;
  make_rc(frame, 100, 200, 300, 400, 0, 0);
  sbus_to_rc(frame, &rc_ctrl, &remoter);
  check(rc_ctrl.last_fdb == 1000u, 2);

  /* 期望：通道 701 越界，不发布（旧实现直接发布） */
  fake_tick = 3000u;
  make_rc(frame, 701, 0, 0, 0, 1, 3);
  sbus_to_rc(frame, &rc_ctrl, &remoter);
  check(rc_ctrl.last_fdb == 1000u, 3);

  test_result.status = 0x600D;
  __asm("bkpt 0");
}
'''


def before_usb(before_root):
    code = functions(read_source(before_root / "User/Communication/example/device/usb_cdc_port.c"))
    return r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef struct { uint32_t dev_state; void *pClassData; } USBD_HandleTypeDef;
#define USBD_STATE_CONFIGURED 2u
#define USBD_OK 0u
#define USBD_BUSY 2u
USBD_HandleTypeDef hUsbDeviceHS;
uint8_t usb_busy;
uint8_t *usb_pending_ptr;
uint16_t usb_pending_len;
uint8_t CDC_Transmit_HS(uint8_t *buf, uint16_t len) {
  if (usb_busy) return USBD_BUSY;
  usb_pending_ptr = buf;
  usb_pending_len = len;
  usb_busy = 1u;
  return USBD_OK;
}
volatile struct { uint32_t status, checks, failure; } test_result;
''' + "uint32_t usb_cdc_port_write(void *user, const uint8_t *data, uint32_t len) " \
  + code["usb_cdc_port_write"] + "\n" + r'''
void check(int condition, uint32_t number) {
  ++test_result.checks;
  if (!condition && test_result.failure == 0u) test_result.failure = number;
}
void test_entry(void) {
  uint8_t stack_buf[40];
  for (uint32_t i = 0; i < sizeof(stack_buf); ++i) stack_buf[i] = (uint8_t)(0xA0u + i);
  hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
  hUsbDeviceHS.pClassData = (void *)1;
  check(usb_cdc_port_write(NULL, stack_buf, sizeof(stack_buf)) == sizeof(stack_buf), 1);
  memset(stack_buf, 0x5A, sizeof(stack_buf));   /* 调用者返回后复用缓冲 */
  check(usb_pending_ptr != NULL && usb_pending_len == sizeof(stack_buf), 2);
  /* 期望：USB 异步读到的仍是原内容；旧实现直接发送调用者指针，内容已被覆盖 */
  check(usb_pending_ptr != NULL && usb_pending_ptr[0] == 0xA0u &&
        usb_pending_ptr[39] == 0xC7u, 3);
  test_result.status = 0x600D;
  __asm("bkpt 0");
}
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", type=Path, default=Path(SNAPSHOT))
    parser.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    parser.add_argument("--results", type=Path, default=DEFAULT_RESULTS)
    args = parser.parse_args()
    cache = project_path(args.cache)
    results = project_path(args.results)
    before_root = cache / "sources"
    names = ["User/Bsp/bsp_usart.c", "User/APP/remote_control.c",
             "User/Communication/example/device/usb_cdc_port.c"]
    snapshot = project_path(args.snapshot)
    if not snapshot.is_file():
        raise SystemExit("missing before snapshot: " + str(snapshot))
    extract(snapshot, before_root, names)
    outcomes = []
    for name, source in (("b02-before-chassis", before_chassis(before_root)),
                         ("b02-before-rc", before_rc(before_root)),
                         ("b02-before-usb", before_usb(before_root))):
        outcome = build(name, source, cache, results)
        outcomes.append(outcome)
        print(json.dumps(outcome))
    (results / "summary.json").write_text(json.dumps(outcomes, indent=2), encoding="utf-8")
    expected = {
        "b02-before-chassis": 2,
        "b02-before-rc": 2,
        "b02-before-usb": 3,
    }
    problems = []
    for outcome in outcomes:
        wanted = expected[outcome["name"]]
        if outcome["before_passed_acceptance"]:
            problems.append(outcome["name"] + ": unexpectedly satisfied acceptance")
        elif outcome["failure"] != wanted:
            problems.append("%s: reproduced at check %s, expected %s"
                            % (outcome["name"], outcome["failure"], wanted))
    if problems:
        raise RuntimeError("; ".join(problems))
    print("before-change failures reproduced at the expected acceptance checks")


if __name__ == "__main__":
    main()
