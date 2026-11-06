"""Exercise the active DM Pitch enable retry gate under ARMCC/Unicorn."""

import argparse
import json
import re
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
    gimbal = functions(read_source(ROOT / "User/gimbal/yaw_pitch_direct.c"))
    config = read_source(ROOT / "User/ROBOT/Infantry_robot.h")
    names = ("PITCH_DM_ENABLE_FIRST_RETRY_MS", "PITCH_DM_ENABLE_RETRY_MS",
             "PITCH_DM_ENABLE_MAX_ATTEMPTS", "PITCH_MIT_FEEDBACK_TIMEOUT_MS")
    definitions = []
    for name in names:
        match = re.search(r"^\s*#define\s+" + name + r"\s+(\d+U?)\b", config, re.MULTILINE)
        if not match:
            raise RuntimeError("Missing numeric configuration: " + name)
        definitions.append("#define " + name + " " + match.group(1))
    return r'''
#include <stdint.h>
#include <stddef.h>
''' + "\n".join(definitions) + r'''
#define PITCH_DM_CAN_ID 0x53U
#define GIMBAL_CAN fake_can
typedef struct { struct { uint8_t id, state; float pos; uint32_t last_fdb_time; } fdb; } dm_measure_t;
typedef struct {
    uint8_t attempts, last_tx_status, confirmed, exhausted;
    uint32_t last_attempt_tick, confirmed_feedback_tick;
    uint32_t mit_send_count, last_mit_tick;
    uint8_t last_mit_tx_status, mit_target_valid;
    float mit_target_pos;
} pitch_dm_enable_status_t;
volatile pitch_dm_enable_status_t pitch_dm_enable_status;
uint32_t fake_now, send_count;
uint8_t fake_tx_status;
uint16_t sent_id;
int fake_can;
uint32_t HAL_GetTick(void) { return fake_now; }
uint8_t Motor_enable(void *can, uint16_t id) {
    if (can != &fake_can) return 255;
    sent_id = id;
    send_count++;
    return fake_tx_status;
}
static uint8_t pitch_dm_feedback_allows_mit(const dm_measure_t *motor)
''' + gimbal["pitch_dm_feedback_allows_mit"] + r'''
static uint8_t pitch_dm_enable_ready(const dm_measure_t *measure)
''' + gimbal["pitch_dm_enable_ready"] + r'''
volatile struct { uint32_t status, checks, failure; } test_result;
__asm void stop_test(void) {
    BKPT 0
    BX lr
}
void check(int ok, uint32_t code) {
    ++test_result.checks;
    if (!ok && !test_result.failure) test_result.failure = code;
}
void reset_state(void) {
    pitch_dm_enable_status.attempts = 1;
    pitch_dm_enable_status.last_tx_status = 0;
    pitch_dm_enable_status.confirmed = 0;
    pitch_dm_enable_status.exhausted = 0;
    pitch_dm_enable_status.last_attempt_tick = 100;
    pitch_dm_enable_status.confirmed_feedback_tick = 0;
    send_count = 0;
    sent_id = 0;
    fake_tx_status = 0;
}
void test_entry(void) {
    dm_measure_t measure = {0};
    reset_state();
    fake_now = 1099;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 0, 1);
    fake_now = 1100;
    fake_tx_status = 2;
    check(pitch_dm_enable_ready(&measure) == 0 && send_count == 1, 2);
    check(pitch_dm_enable_status.attempts == 2 &&
          pitch_dm_enable_status.last_tx_status == 2 && sent_id == 0x53, 3);
    measure.fdb.last_fdb_time = 1099;
    measure.fdb.state = 1;
    fake_now = 1101;
    check(pitch_dm_enable_ready(&measure) == 0 &&
          pitch_dm_enable_status.confirmed == 0, 4);
    measure.fdb.last_fdb_time = 1101;
    check(pitch_dm_enable_ready(&measure) == 1 &&
          pitch_dm_enable_status.confirmed_feedback_tick == 1101, 5);
    fake_now = 2000;
    check(pitch_dm_enable_ready(&measure) == 1 && send_count == 1, 6);
    reset_state();
    fake_now = 1100;
    measure.fdb.id = 3;
    measure.fdb.state = 5;
    measure.fdb.last_fdb_time = 100;
    check(pitch_dm_enable_ready(&measure) == 0, 7);
    measure.fdb.last_fdb_time = 1101;
    fake_now = 1101;
    check(pitch_dm_enable_ready(&measure) == 1, 8);
    reset_state();
    fake_now = 1101;
    measure.fdb.id = 4;
    measure.fdb.last_fdb_time = 1101;
    check(pitch_dm_enable_ready(&measure) == 0, 9);
    measure.fdb.id = 3;
    measure.fdb.state = 8;
    check(pitch_dm_enable_ready(&measure) == 0, 10);
    reset_state();
    pitch_dm_enable_status.attempts = PITCH_DM_ENABLE_MAX_ATTEMPTS;
    check(pitch_dm_enable_ready(&measure) == 0 &&
          pitch_dm_enable_status.exhausted == 1 && send_count == 0, 11);
    check(pitch_dm_enable_ready(NULL) == 0, 12);
    test_result.status = 0x600D;
    stop_test();
}
'''


def run(cache, results):
    cache.mkdir(parents=True, exist_ok=True)
    results.mkdir(parents=True, exist_ok=True)
    source = cache / "pitch-enable-check.c"
    obj = cache / "pitch-enable-check.o"
    binary = cache / "pitch-enable-check.axf"
    source.write_text(harness_source(), encoding="utf-8")
    for command in (
        [str(ARM_BIN / "armcc.exe"), "--cpu", "Cortex-M7", "--thumb", "--c99",
         "-O0", "-c", str(source), "-o", str(obj)],
        [str(ARM_BIN / "armlink.exe"), "--ro_base", "0x10000", "--rw_base", "0x20000000",
         "--entry", "test_entry", str(obj), "-o", str(binary)],
    ):
        completed = subprocess.run(command, capture_output=True, timeout=45)
        if completed.returncode:
            raise RuntimeError(completed.stdout.decode("gb18030", errors="replace") +
                               completed.stderr.decode("gb18030", errors="replace"))
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
               "scope": "active enable retry, fresh feedback, ID/status compatibility, retry exhaustion"}
    (results / "results.json").write_text(json.dumps(outcome, indent=2), encoding="utf-8")
    (results / source.name).write_bytes(source.read_bytes())
    print(json.dumps(outcome, indent=2))
    if not outcome["passed"]:
        raise RuntimeError("Pitch enable state check failed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache", type=Path, default=CACHE / "validation/pitch-enable")
    parser.add_argument("--results", type=Path, default=ARTIFACTS / "validation/pitch-enable")
    args = parser.parse_args()
    run(project_path(args.cache), project_path(args.results))
