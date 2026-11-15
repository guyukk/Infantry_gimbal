"""Exercise gimbal-owned frame functions with ARMCC and Unicorn, using only this project.

The peer frame is supplied as test bytes. No external source tree is read.
This checks serialization and parsing, not peripherals, task timing, or motors.
"""
import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

from audit import read_source, without_comments
from snapshot import PROJECTS
from validate_invariants import functions, preprocess
from project_paths import ARTIFACTS, CACHE, DEPENDENCIES, ARM_BIN, project_path

sys.path.insert(0, str(DEPENDENCIES))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP

ROOT, PROJECT = PROJECTS["gimbal"]
DEFAULT_CACHE = CACHE / "validation/gimbal-runtime"
DEFAULT_RESULTS = ARTIFACTS / "validation/gimbal-runtime"


def source():
    sender = functions(preprocess(ROOT, PROJECT, "User/APP/message_task.c", "gimbal"))["send_data_to_chassis"]
    receiver = functions(preprocess(ROOT, PROJECT, "User/Bsp/bsp_usart.c", "gimbal"))["chassis_frame_parse"]
    neutral = functions(preprocess(ROOT, PROJECT, "User/APP/video_transmission_module.c", "gimbal"))["vtm_data_init"]
    constrain = functions(read_source(ROOT / "User/Lib/user_lib.c"))["loop_float_constrain"]
    header = without_comments(read_source(ROOT / "User/APP/video_transmission_module.h"))
    packed = header[header.index("#pragma pack(1)"):header.index("#pragma pack()", header.index("#pragma pack(1)")) + len("#pragma pack()")]
    sender = re.sub(r"\bvtm_data\b", "vtm_guard.value", sender)
    neutral = re.sub(r"\bvtm_data\b", "vtm_guard.value", neutral)
    return r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
typedef float fp32;
typedef union { float fp32; uint8_t bytes[4]; } fp32_to_bytes;
''' + packed + r'''
typedef char vtm_size_check[(sizeof(VTM_Data_t) == 21) ? 1 : -1];
typedef char vtm_crc_check[(offsetof(VTM_Data_t, crc) == 19) ? 1 : -1];
struct { uint8_t before[8]; VTM_Data_t value; uint8_t after[8]; } vtm_guard;
struct { struct { float radian_of_ecd; } gimbal_yaw_motor; } gimbal_control;
typedef struct { uint8_t shoot_mode; } shoot_control_t;
shoot_control_t shoot_control;
uint8_t gimbal_behaviour;
typedef struct {
  float chassis_vx, chassis_vy, chassis_wz;
  float chassis_yaw, chassis_pitch, chassis_roll, cap_voltage;
  uint8_t chassis_mode, robot_id;
} chassis_data_t;
#define CHASSIS_DATA_LENGTH 28
#define CHASSIS_FRAME_HEAD 's'
#define CHASSIS_FRAME_TAIL 'e'
#define CHASSIS_ANGLE_LIMIT_RAD 3.2f
#define CHASSIS_SPEED_LIMIT_MPS 10.0f
#define CHASSIS_WZ_LIMIT_RADPS 50.0f
#define CHASSIS_MODE_MAX 2u
float ins_angles[3];
float *get_INS_angle_point(void) { return ins_angles; }
uint8_t frame46[46];
uint16_t frame46_size;
void USART1_Transmit_DMA(uint8_t *bytes, uint16_t size) {
  frame46_size = size;
  if (size <= sizeof(frame46)) memcpy(frame46, bytes, size);
}
float loop_float_constrain(float Input, float minValue, float maxValue)''' + constrain + "\n" + (
        "void vtm_data_init(void)" + neutral + "\n"
        "void send_data_to_chassis(void)" + sender + "\n"
        "static uint8_t chassis_frame_parse(const uint8_t *frame, chassis_data_t *out)" + receiver + "\n"
    ) + r'''
volatile struct { uint32_t status, checks, failure; } test_result;
__asm void stop_test(void) {
  BKPT 0
  BX lr
}
void check(int ok, uint32_t code) {
  ++test_result.checks;
  if (!ok) { test_result.failure = code; stop_test(); }
}
void test_entry(void) {
  uint8_t incoming[28] = {'s'};
  uint8_t *vtm = (uint8_t *)&vtm_guard.value;
  float expected;
  memset(&vtm_guard, 0xCD, sizeof(vtm_guard));
  vtm_data_init();
  check(vtm_guard.value.head1 == 0xA9 && vtm_guard.value.head2 == 0x53, 1);
  check(vtm_guard.value.ch0 == 1024 && vtm_guard.value.ch3 == 1024, 2);
  check(vtm_guard.value.gear_switch == 1 && vtm_guard.value.wheel == 1024, 3);
  for (uint32_t i = 2; i < sizeof(vtm_guard.value); ++i) vtm[i] = 0x40U + i;
  gimbal_control.gimbal_yaw_motor.radian_of_ecd = 1.25f;
  ins_angles[0] = -2.0f; ins_angles[1] = 0.375f; ins_angles[2] = -0.5f;
  gimbal_behaviour = 5; shoot_control.shoot_mode = 4;
  send_data_to_chassis();
  check(frame46_size == 46 && frame46[0] == 0xFE && frame46[45] == 0xFD, 4);
  check(!memcmp(frame46 + 1, &gimbal_control.gimbal_yaw_motor.radian_of_ecd, 4), 5);
  check(!memcmp(frame46 + 5, ins_angles, 12), 6);
  check(frame46[17] == 5 && frame46[18] == 4, 7);
  check(!memcmp(frame46 + 19, vtm + 2, 19), 8);
  for (uint32_t i = 38; i <= 44; ++i) check(frame46[i] == 0, 9);
  for (uint32_t i = 0; i < 8; ++i) {
    check(vtm_guard.before[i] == 0xCD && vtm_guard.after[i] == 0xCD, 10);
  }
  expected = 0.125f; memcpy(incoming + 1, &expected, 4);
  expected = -1.0f; memcpy(incoming + 5, &expected, 4);
  expected = 0.5f; memcpy(incoming + 9, &expected, 4);
  incoming[13] = 2; incoming[14] = 103;
  expected = 1.0f; memcpy(incoming + 15, &expected, 4);
  expected = -0.25f; memcpy(incoming + 19, &expected, 4);
  expected = 0.75f; memcpy(incoming + 23, &expected, 4);
  incoming[27] = 'e';
  chassis_data_t parsed;
  memset(&parsed, 0, sizeof(parsed));
  check(chassis_frame_parse(incoming, &parsed) == 0, 11);
  check(parsed.chassis_yaw == 0.125f && parsed.chassis_pitch == -1.0f &&
        parsed.chassis_roll == 0.5f, 12);
  check(parsed.chassis_mode == 2 && parsed.robot_id == 103, 13);
  check(parsed.chassis_vx == 1.0f && parsed.chassis_vy == -0.25f &&
        parsed.chassis_wz == 0.75f, 14);
  test_result.status = 0x600D;
  stop_test();
}
'''


def run(cache, results):
    cache.mkdir(parents=True, exist_ok=True)
    results.mkdir(parents=True, exist_ok=True)
    code = cache / "gimbal-runtime-check.c"
    obj = cache / "gimbal-runtime-check.o"
    image = cache / "gimbal-runtime-check.axf"
    code.write_text(source(), encoding="utf-8")
    commands = [
        [str(ARM_BIN / "armcc.exe"), "--cpu", "Cortex-M7", "--thumb", "--c99", "-O0",
         "-c", str(code), "-o", str(obj)],
        [str(ARM_BIN / "armlink.exe"), "--ro_base", "0x10000", "--rw_base", "0x20000000",
         "--entry", "test_entry", str(obj), "-o", str(image)],
    ]
    for command in commands:
        done = subprocess.run(command, capture_output=True, timeout=45)
        if done.returncode:
            raise RuntimeError(done.stdout.decode("gb18030", errors="replace") +
                               done.stderr.decode("gb18030", errors="replace"))
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
    uc.emu_start(entry | 1, 0, timeout=10000000, count=2000000)
    raw = bytes(uc.mem_read(symbols["test_result"], 12))
    values = [int.from_bytes(raw[i:i+4], "little") for i in range(0, 12, 4)]
    outcome = {"status": hex(values[0]), "checks": values[1], "failure": values[2],
               "scope": "gimbal source and synthetic peer frame",
               "passed": values[0] == 0x600D and values[2] == 0}
    (results / "runtime-results.json").write_text(json.dumps(outcome, indent=2), encoding="utf-8")
    shutil.copy2(code, results / code.name)
    shutil.copy2(image, results / image.name)
    print(json.dumps(outcome, indent=2))
    if not outcome["passed"]:
        raise RuntimeError("Gimbal frame check failed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    parser.add_argument("--results", type=Path, default=DEFAULT_RESULTS)
    args = parser.parse_args()
    run(project_path(args.cache), project_path(args.results))
