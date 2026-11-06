"""Run the active Pitch enable retry function under ARMCC and Unicorn."""

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

from audit import read_source
from project_paths import ARM_BIN, ARTIFACTS, CACHE, DEPENDENCIES, ROOT, project_path
from validate_invariants import functions

sys.path.insert(0, str(DEPENDENCIES))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import UC_ARM_REG_SP


def harness_source():
    body = functions(read_source(ROOT / "User/gimbal/yaw_pitch_direct.c"))["pitch_dm_enable_ready"]
    send_body = functions(read_source(ROOT / "User/gimbal/yaw_pitch_direct.c"))["gimbal_send_cmd"]
    config = read_source(ROOT / "User/ROBOT/Infantry_robot.h")
    names = ("PITCH_DM_ENABLE_FIRST_RETRY_MS", "PITCH_DM_ENABLE_RETRY_MS",
             "PITCH_DM_ENABLE_MAX_ATTEMPTS")
    definitions = []
    for name in names:
        match = re.search(r"^\s*#define\s+" + name + r"\s+(\d+U?)\b", config, re.MULTILINE)
        if not match:
            raise RuntimeError("Missing numeric configuration: " + name)
        definitions.append("#define " + name + " " + match.group(1))
    return r'''
#include <stdint.h>
#include <stddef.h>
#define debug 0
''' + "\n".join(definitions) + r'''
#define PITCH_DM_CAN_ID 0x53U
#define GIMBAL_CAN fake_can
#define YAW_CURRENT_SET_POLARITY 1
#define PITCH_CURRENT_SET_POLARITY 1
#define YAW_GIMBAL_MOTOR_TOE 0
#define PITCH_GIMBAL_MOTOR_TOE 1
#define TRIGGER_MOTOR_TOE 2
#define DBUS_TOE 3
typedef struct { struct { uint32_t last_fdb_time; int state; } fdb; } dm_measure_t;
typedef struct {
    struct { int given_current; } gimbal_yaw_motor;
    struct {
        dm_measure_t *gimbal_DM_measure;
        int given_current;
        float emit_pos_set, emit_vel_limit, emit_i_limit;
    } gimbal_pitch_motor;
} gimbal_control_t;
typedef struct {
    uint8_t attempts, last_tx_status, confirmed, exhausted;
    uint32_t last_attempt_tick, confirmed_feedback_tick;
    uint32_t emit_send_count, last_emit_tick;
    uint8_t last_emit_tx_status;
} pitch_dm_enable_status_t;
volatile pitch_dm_enable_status_t pitch_dm_enable_status;
uint32_t fake_now, send_count;
uint8_t fake_tx_status;
uint16_t sent_id;
int fake_can;
int16_t yaw_can_set_current, pitch_can_set_current, shoot_can_set_current;
int8_t yaw_cuurent_set_polarity = YAW_CURRENT_SET_POLARITY;
int8_t pitch_cuurent_set_polarity = PITCH_CURRENT_SET_POLARITY;
uint32_t toe_flags, emit_count;
uint8_t fake_emit_status;
uint16_t emit_id;
float emit_pos;
uint32_t HAL_GetTick(void) { return fake_now; }
int toe_is_error(uint8_t toe) { return (toe_flags >> toe) & 1U; }
void CAN_cmd_gimbal(int a, int b, int c, int d) { (void)a; (void)b; (void)c; (void)d; }
uint8_t CAN_cmd_EMIT(void *can, uint16_t id, float pos, float vel, float current) {
    (void)vel; (void)current;
    if (can != &fake_can) return 255;
    ++emit_count;
    emit_id = id;
    emit_pos = pos;
    return fake_emit_status;
}
uint8_t Motor_enable(void *can, uint16_t id) {
    if (can != &fake_can) return 255;
    sent_id = id;
    ++send_count;
    return fake_tx_status;
}
static uint8_t pitch_dm_enable_ready(const dm_measure_t *measure)
''' + body + r'''
void gimbal_send_cmd(gimbal_control_t *control_send)
''' + send_body + r'''
volatile struct { uint32_t status, checks, failure; } test_result;
__asm void stop_test(void) {
    BKPT 0
    BX lr
}
void check(int ok, uint32_t code) {
    ++test_result.checks;
    if (!ok) { test_result.failure = code; stop_test(); }
}
void reset_state(void) {
    pitch_dm_enable_status.attempts = 1;
    pitch_dm_enable_status.last_tx_status = 0;
    pitch_dm_enable_status.confirmed = 0;
    pitch_dm_enable_status.exhausted = 0;
    pitch_dm_enable_status.last_attempt_tick = 100;
    pitch_dm_enable_status.confirmed_feedback_tick = 0;
    send_count = 0;
    fake_tx_status = 0;
    sent_id = 0;
    emit_count = 0;
    toe_flags = 0;
    fake_emit_status = 0;
    pitch_dm_enable_status.emit_send_count = 0;
    pitch_dm_enable_status.last_emit_tick = 0;
    pitch_dm_enable_status.last_emit_tx_status = 255;
}
void test_entry(void) {
    dm_measure_t measure = {{0, 0}};
    uint32_t first_retry_at = 100U + PITCH_DM_ENABLE_FIRST_RETRY_MS;
    reset_state();
    fake_now = first_retry_at - 1U;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 0, 1);
    fake_now = first_retry_at;
    fake_tx_status = 2;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 1, 2);
    check(pitch_dm_enable_status.attempts == 2 &&
          pitch_dm_enable_status.last_tx_status == 2 && sent_id == 0x53, 3);
    measure.fdb.last_fdb_time = first_retry_at - 1U;
    measure.fdb.state = 1;
    fake_now = first_retry_at + 1U;
    check(pitch_dm_enable_ready(&measure) == 0 &&
          pitch_dm_enable_status.confirmed == 0, 4);
    measure.fdb.last_fdb_time = first_retry_at + 1U;
    check(pitch_dm_enable_ready(&measure) == 1 &&
          pitch_dm_enable_status.confirmed_feedback_tick == first_retry_at + 1U, 5);
    fake_now = first_retry_at + 900U;
    check(pitch_dm_enable_ready(&measure) == 1 && send_count == 1, 6);
    reset_state();
    measure.fdb.last_fdb_time = first_retry_at + 1U;
    measure.fdb.state = 0;
    fake_now = 599;
    pitch_dm_enable_status.attempts = 7;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 0, 7);
    fake_now = 600;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 1 &&
          pitch_dm_enable_status.attempts == 8, 8);
    fake_now = 1100;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 1 &&
          pitch_dm_enable_status.exhausted == 1, 9);
    check(pitch_dm_enable_ready(NULL) == 0 && send_count == 1, 10);
    reset_state();
    measure.fdb.last_fdb_time = 0;
    measure.fdb.state = 0;
    gimbal_control_t control = {0};
    control.gimbal_pitch_motor.gimbal_DM_measure = &measure;
    control.gimbal_pitch_motor.emit_pos_set = -2.25f;
    control.gimbal_pitch_motor.emit_vel_limit = 30.0f;
    control.gimbal_pitch_motor.emit_i_limit = 0.8f;
    fake_now = first_retry_at;
    fake_emit_status = 2;
    gimbal_send_cmd(&control);
    check(emit_count == 1 && pitch_dm_enable_status.confirmed == 0 &&
          pitch_dm_enable_status.emit_send_count == 1, 11);
    check(emit_id == 0x53 && emit_pos == -2.25f &&
          pitch_dm_enable_status.last_emit_tx_status == 2 &&
          pitch_dm_enable_status.last_emit_tick == first_retry_at, 12);
    toe_flags = 1U << DBUS_TOE;
    fake_now = first_retry_at + 1U;
    gimbal_send_cmd(&control);
    check(emit_count == 1 && pitch_dm_enable_status.emit_send_count == 1, 13);
    toe_flags = 0;
    measure.fdb.last_fdb_time = first_retry_at + 2U;
    measure.fdb.state = 1;
    fake_now = first_retry_at + 2U;
    gimbal_send_cmd(&control);
    check(pitch_dm_enable_status.confirmed == 1 && emit_count == 2, 14);
    test_result.status = 0x600D;
    stop_test();
}
'''


def run(cache, results):
    cache.mkdir(parents=True, exist_ok=True)
    results.mkdir(parents=True, exist_ok=True)
    code = cache / "pitch-enable-check.c"
    obj = cache / "pitch-enable-check.o"
    binary = cache / "pitch-enable-check.axf"
    code.write_text(harness_source(), encoding="utf-8")
    commands = [
        [str(ARM_BIN / "armcc.exe"), "--cpu", "Cortex-M7", "--thumb", "--c99", "-O0",
         "-c", str(code), "-o", str(obj)],
        [str(ARM_BIN / "armlink.exe"), "--ro_base", "0x10000", "--rw_base", "0x20000000",
         "--entry", "test_entry", str(obj), "-o", str(binary)],
    ]
    for command in commands:
        done = subprocess.run(command, capture_output=True, timeout=45)
        if done.returncode:
            raise RuntimeError(done.stdout.decode("gb18030", errors="replace") +
                               done.stderr.decode("gb18030", errors="replace"))
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.mem_map(0, 0x100000)
    uc.mem_map(0x20000000, 0x100000)
    with binary.open("rb") as stream:
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
    values = [int.from_bytes(raw[i:i + 4], "little") for i in range(0, 12, 4)]
    outcome = {"status": hex(values[0]), "checks": values[1], "failure": values[2],
               "passed": values[0] == 0x600D and values[2] == 0,
               "scope": "actual enable retry and command-send bodies with synthetic feedback, remote error and HAL status"}
    (results / "results.json").write_text(json.dumps(outcome, indent=2), encoding="utf-8")
    shutil.copy2(code, results / code.name)
    print(json.dumps(outcome, indent=2))
    if not outcome["passed"]:
        raise RuntimeError("Pitch enable state check failed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache", type=Path, default=CACHE / "validation/pitch-enable")
    parser.add_argument("--results", type=Path, default=ARTIFACTS / "validation/pitch-enable")
    args = parser.parse_args()
    run(project_path(args.cache), project_path(args.results))
