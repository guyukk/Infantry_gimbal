"""Exercise the active Pitch calibration recovery path under ARMCC/Unicorn."""

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
    cali = functions(read_source(ROOT / "User/APP/calibrate_task.c") + "\n" + read_source(ROOT / "User/APP/pitch_calibration.c"))
    config = read_source(ROOT / "User/ROBOT/Infantry_robot.h")
    match = re.search(r"^\s*#define\s+PITCH_DM_LOWER_RECOVERY_RAD\s+(\S+)", config, re.MULTILINE)
    if not match:
        raise RuntimeError("Missing PITCH_DM_LOWER_RECOVERY_RAD")
    names = ("pitch_dm_cali_check", "pitch_dm_cali_valid_range",
             "pitch_dm_cali_read_record", "pitch_cali_feedback", "pitch_dm_cali_recover")
    bodies = {name: cali[name] for name in names}
    return r'''
#include <stdint.h>
#include <stddef.h>
#define PITCH_DM_CALI_MAGIC 0x50444D31UL
#define PITCH_DM_CALI_VERSION 1UL
#define PITCH_DM_CALI_ADDR 0x1000UL
#define PITCH_DM_CALI_RETRY_MS 1000U
#define PITCH_MIT_P_MAX 12.5f
''' + "#define PITCH_DM_LOWER_RECOVERY_RAD " + match.group(1) + r'''
#define PITCH_MIT_FEEDBACK_TIMEOUT_MS 100U
#define PITCH_DM_MOTOR_INDEX 2
#define PITCH_DM_CAN_ID 0x53
#define HEALTH_PITCH 4
#define isfinite(x) ((x) == (x))
typedef struct { uint32_t magic, version; float min_pos, max_pos; uint32_t check; }
    pitch_dm_cali_record_t;
typedef struct {
    uint8_t request, captured, stored_valid, saved_pending, init_done;
    int8_t result;
    float first_pos, second_pos, min_pos, max_pos;
} pitch_dm_cali_status_t;
typedef struct { struct { float pos,vel; uint32_t last_fdb_time; uint8_t state,id; } fdb; } dm_measure_t;
typedef struct { float min_pos, max_pos, target; } gimbal_control_t;
static uint8_t pitch_dm_flash_ready;
static uint8_t calibration_flash_ready;
volatile pitch_dm_cali_status_t pitch_dm_cali_status;
gimbal_control_t gimbal_control;
dm_measure_t motor;
pitch_dm_cali_record_t stored_record;
uint32_t fake_tick, init_calls, read_calls, apply_calls, critical_depth;
uint8_t init_ok, read_ok, valid_at_apply;
uint32_t HAL_GetTick(void) { return fake_tick; }
int8_t flash_init(void) { ++init_calls; return init_ok ? 0 : -1; }
int8_t flash_read(uint32_t address, uint32_t *buf, uint32_t words) {
    const uint32_t *src = (const uint32_t *)&stored_record;
    uint32_t i;
    ++read_calls;
    if (!read_ok || address != PITCH_DM_CALI_ADDR || words != 5U) return -1;
    for (i = 0; i < words; ++i) buf[i] = src[i];
    return 0;
}
dm_measure_t *get_dm_motor_measure_point(uint8_t index) {
    return index == PITCH_DM_MOTOR_INDEX ? &motor : NULL;
}
void pitch_dm_apply_calibration(gimbal_control_t *control,
                                float min_pos, float max_pos, float current_pos) {
    ++apply_calls;
    valid_at_apply = pitch_dm_cali_status.stored_valid;
    control->min_pos = min_pos;
    control->max_pos = max_pos;
    control->target = current_pos;
}
#define taskENTER_CRITICAL() (++critical_depth)
#define taskEXIT_CRITICAL() (--critical_depth)
/* Focused cold-start chain: health availability is controlled from the fixture's
 * motor timestamps. Full real health/Flash integration is test_b05_persistence.c. */
uint8_t health_fresh(uint8_t id,uint32_t now){(void)id;return motor.fdb.last_fdb_time && now-motor.fdb.last_fdb_time<=100;}
static uint8_t pitch_cali_feedback(dm_measure_t *out)
''' + bodies["pitch_cali_feedback"] + r'''
static uint32_t pitch_dm_cali_check(const pitch_dm_cali_record_t *record)
''' + bodies["pitch_dm_cali_check"] + r'''
static uint8_t pitch_dm_cali_valid_range(float min_pos, float max_pos)
''' + bodies["pitch_dm_cali_valid_range"] + r'''
static uint8_t pitch_dm_cali_read_record(pitch_dm_cali_record_t *record)
''' + bodies["pitch_dm_cali_read_record"] + r'''
static void pitch_dm_cali_recover(void)
''' + bodies["pitch_dm_cali_recover"] + r'''
volatile struct { uint32_t status, checks, failure; } test_result;
__asm void stop_test(void) {
    BKPT 0
    BX lr
}
void check(int ok, uint32_t code) {
    ++test_result.checks;
    if (!ok && !test_result.failure) test_result.failure = code;
}
void test_entry(void) {
    stored_record.magic = PITCH_DM_CALI_MAGIC;
    stored_record.version = PITCH_DM_CALI_VERSION;
    stored_record.min_pos = -2.79f;
    stored_record.max_pos = -2.3f;
    stored_record.check = pitch_dm_cali_check(&stored_record);
    motor.fdb.state=1; motor.fdb.id=3; motor.fdb.vel=0;
    motor.fdb.pos = -2.5f;
    motor.fdb.last_fdb_time = 1000;
    fake_tick = 1000;
    pitch_dm_cali_recover();
    check(init_calls == 0 && !pitch_dm_cali_status.stored_valid, 1);
    pitch_dm_cali_status.init_done = 1;
    pitch_dm_cali_recover();
    check(init_calls == 1 && !pitch_dm_cali_status.stored_valid, 2);
    init_ok = 1;
    fake_tick = 2000;
    motor.fdb.last_fdb_time = 2000;
    pitch_dm_cali_recover();
    check(init_calls == 2 && read_calls == 1 && !pitch_dm_cali_status.stored_valid, 3);
    read_ok = 1;
    fake_tick = 3000;
    motor.fdb.last_fdb_time = 3000;
    stored_record.check ^= 1U;
    pitch_dm_cali_recover();
    check(!pitch_dm_cali_status.stored_valid && apply_calls == 0, 4);
    stored_record.check = pitch_dm_cali_check(&stored_record);
    fake_tick = 4000;
    motor.fdb.last_fdb_time = 3899;
    pitch_dm_cali_recover();
    check(!pitch_dm_cali_status.stored_valid && apply_calls == 0, 5);
    fake_tick = 5000;
    motor.fdb.last_fdb_time = 5000;
    motor.fdb.pos = -2.1f;
    pitch_dm_cali_recover();
    check(!pitch_dm_cali_status.stored_valid && apply_calls == 0, 6);
    fake_tick = 6000;
    motor.fdb.last_fdb_time = 6000;
    motor.fdb.pos = -2.811f;
    pitch_dm_cali_recover();
    check(!pitch_dm_cali_status.stored_valid && apply_calls == 0, 7);
    fake_tick = 7000;
    motor.fdb.last_fdb_time = 7000;
    motor.fdb.pos = -2.801f;
    pitch_dm_cali_recover();
    check(pitch_dm_cali_status.stored_valid && apply_calls == 1 &&
          !valid_at_apply && critical_depth == 0 &&
          calibration_flash_ready == 0, 8);
    check(gimbal_control.min_pos == -2.79f &&
          gimbal_control.max_pos == -2.3f &&
          gimbal_control.target == -2.801f, 9);
    fake_tick = 8000;
    pitch_dm_cali_recover();
    check(apply_calls == 1, 10);
    test_result.status = 0x600D;
    stop_test();
}
'''


def run(cache, results):
    cache.mkdir(parents=True, exist_ok=True)
    results.mkdir(parents=True, exist_ok=True)
    source = cache / "pitch-cold-start-check.c"
    obj = cache / "pitch-cold-start-check.o"
    binary = cache / "pitch-cold-start-check.axf"
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
               "scope": "cold Flash failure, retry, CRC, feedback age, calibrated range, atomic publish"}
    (results / "results.json").write_text(json.dumps(outcome, indent=2), encoding="utf-8")
    (results / source.name).write_bytes(source.read_bytes())
    print(json.dumps(outcome, indent=2))
    if not outcome["passed"]:
        raise RuntimeError("Pitch cold-start recovery check failed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache", type=Path, default=CACHE / "validation/pitch-cold-start")
    parser.add_argument("--results", type=Path, default=ARTIFACTS / "validation/pitch-cold-start")
    args = parser.parse_args()
    run(project_path(args.cache), project_path(args.results))
