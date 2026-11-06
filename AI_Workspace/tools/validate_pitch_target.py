"""Execute the active Pitch mode transition and target limiter under ARMCC."""

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
    transition = functions(read_source(ROOT / "User/gimbal/yaw_pitch_direct.c"))[
        "gimbal_mode_change_control_transit"]
    limiter = functions(read_source(ROOT / "User/APP/gimbal_task.c"))[
        "gimbal_relative_angle_limit"]
    relative_rc = functions(read_source(ROOT / "User/APP/gimbal_behaviour.c"))[
        "gimbal_relative_angle_control"]
    control_loop = functions(read_source(ROOT / "User/gimbal/yaw_pitch_direct.c"))[
        "gimbal_control_loop"]
    config = read_source(ROOT / "User/ROBOT/Infantry_robot.h")
    config_names = ("YAW_CHANNEL", "PITCH_CHANNEL", "RC_DEADBAND", "YAW_RC_SEN",
                    "PITCH_RC_SEN", "YAW_MOUSE_SEN", "PITCH_MOUSE_SEN")
    definitions = []
    for name in config_names:
        match = re.search(r"^\s*#define\s+" + name + r"\s+(\S+)", config, re.MULTILINE)
        if not match:
            raise RuntimeError("Missing configuration: " + name)
        definitions.append("#define " + name + " " + match.group(1))
    return r'''
#include <stdint.h>
#include <stddef.h>
''' + "\n".join(definitions) + r'''
typedef float fp32;
enum { GIMBAL_MOTOR_RAW = 0, GIMBAL_MOTOR_GYRO = 1, GIMBAL_MOTOR_ENCONDE = 2 };
typedef struct {
    struct { int16_t ch[5]; } rc;
    struct { int16_t x, y; } mouse;
} RC_ctrl_t;
typedef struct {
    int last_gimbal_motor_mode, gimbal_motor_mode, given_current;
    fp32 raw_cmd_current, current_set;
    fp32 absolute_angle_set, absolute_angle;
    fp32 relative_angle_set, relative_angle;
    fp32 motor_gyro_set, motor_gyro;
    fp32 min_relative_angle, max_relative_angle, emit_pos_set;
} gimbal_motor_t;
typedef struct {
    gimbal_motor_t gimbal_yaw_motor, gimbal_pitch_motor;
    const RC_ctrl_t *gimbal_rc_ctrl;
} gimbal_control_t;
gimbal_control_t gimbal_control;
RC_ctrl_t rc_ctrl;
struct { struct { fp32 yaw, pitch; } receive; } aim;
#define rad_format(x) (x)
#define rc_deadband_limit(input, output, deadband) \
    do { (output) = ((input) > (deadband) || (input) < -(deadband)) ? (input) : 0; } while (0)
void gimbal_motor_raw_angle_control(gimbal_motor_t *motor) { (void)motor; }
void gimbal_motor_absolute_angle_control(gimbal_motor_t *motor) { (void)motor; }
void gimbal_motor_relative_angle_control(gimbal_motor_t *motor) { (void)motor; }
void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
''' + transition + r'''
void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
''' + limiter + r'''
void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
''' + relative_rc + r'''
void gimbal_control_loop(gimbal_control_t *control_loop)
''' + control_loop + r'''
volatile struct { uint32_t status, checks, failure; } test_result;
__asm void stop_test(void) {
    BKPT 0
    BX lr
}
void check(int ok, uint32_t code) {
    ++test_result.checks;
    if (!ok) { test_result.failure = code; stop_test(); }
}
int near(fp32 actual, fp32 expected) {
    return actual > expected - 0.0001f && actual < expected + 0.0001f;
}
void test_entry(void) {
    gimbal_motor_t *pitch = &gimbal_control.gimbal_pitch_motor;
    gimbal_motor_t *yaw = &gimbal_control.gimbal_yaw_motor;
    fp32 add_yaw = 0.0f, add_pitch = 0.0f;
    gimbal_control.gimbal_rc_ctrl = &rc_ctrl;
    pitch->last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    pitch->gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    pitch->relative_angle = 0.0f; /* current unconfigured DM feedback */
    pitch->emit_pos_set = -2.25f; /* command from RAW mode */
    pitch->min_relative_angle = -2.5f;
    pitch->max_relative_angle = -2.0f;
    yaw->last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    yaw->gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    yaw->relative_angle = 0.2f;
    gimbal_mode_change_control_transit(&gimbal_control);
    check(near(pitch->relative_angle_set, -2.25f), 1);
    check(near(yaw->relative_angle_set, 0.2f), 2);
    gimbal_relative_angle_limit(pitch, -0.00396f);
    check(near(pitch->relative_angle_set, -2.25396f), 3);
    gimbal_relative_angle_limit(pitch, +0.00396f);
    check(near(pitch->relative_angle_set, -2.25f), 4);
    for (int i = 0; i < 100; ++i) gimbal_relative_angle_limit(pitch, -0.00396f);
    check(near(pitch->relative_angle_set, -2.5f), 5);
    for (int i = 0; i < 200; ++i) gimbal_relative_angle_limit(pitch, +0.00396f);
    check(near(pitch->relative_angle_set, -2.0f), 6);
    pitch->relative_angle_set = -2.25f;
    rc_ctrl.rc.ch[PITCH_CHANNEL] = 660;
    gimbal_relative_angle_control(&add_yaw, &add_pitch, &gimbal_control);
    check(near(add_pitch, 660.0f * PITCH_RC_SEN), 7);
    gimbal_relative_angle_limit(pitch, add_pitch);
    gimbal_control_loop(&gimbal_control);
    check(near(pitch->emit_pos_set, -2.25f + 660.0f * PITCH_RC_SEN), 8);
    rc_ctrl.rc.ch[PITCH_CHANNEL] = -660;
    gimbal_relative_angle_control(&add_yaw, &add_pitch, &gimbal_control);
    check(near(add_pitch, -660.0f * PITCH_RC_SEN), 9);
    gimbal_relative_angle_limit(pitch, add_pitch);
    gimbal_control_loop(&gimbal_control);
    check(near(pitch->emit_pos_set, -2.25f), 10);
    test_result.status = 0x600D;
    stop_test();
}
'''


def run(cache, results):
    cache.mkdir(parents=True, exist_ok=True)
    results.mkdir(parents=True, exist_ok=True)
    source = cache / "pitch-target-check.c"
    obj = cache / "pitch-target-check.o"
    binary = cache / "pitch-target-check.axf"
    source.write_text(harness_source(), encoding="utf-8")
    commands = [
        [str(ARM_BIN / "armcc.exe"), "--cpu", "Cortex-M7", "--thumb", "--c99", "-O0",
         "-c", str(source), "-o", str(obj)],
        [str(ARM_BIN / "armlink.exe"), "--ro_base", "0x10000", "--rw_base", "0x20000000",
         "--entry", "test_entry", str(obj), "-o", str(binary)],
    ]
    for command in commands:
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
        symbols = {symbol.name: symbol["st_value"] for symbol in
                   elf.get_section_by_name(".symtab").iter_symbols()}
        entry = elf["e_entry"]
    uc.reg_write(UC_ARM_REG_SP, 0x200FF000)

    def stop_on_breakpoint(machine, address, size, _):
        if machine.mem_read(address, 2) == b"\x00\xbe":
            machine.emu_stop()

    uc.hook_add(UC_HOOK_CODE, stop_on_breakpoint)
    uc.emu_start(entry | 1, 0, timeout=10000000, count=2000000)
    raw = bytes(uc.mem_read(symbols["test_result"], 12))
    status, checks, failure = (int.from_bytes(raw[i:i + 4], "little")
                               for i in range(0, 12, 4))
    outcome = {"status": hex(status), "checks": checks, "failure": failure,
               "passed": status == 0x600D and failure == 0,
               "scope": "active Pitch transition, RC channel 3 increment, limiter and position target mapping"}
    (results / "results.json").write_text(json.dumps(outcome, indent=2), encoding="utf-8")
    (results / source.name).write_bytes(source.read_bytes())
    print(json.dumps(outcome, indent=2))
    if not outcome["passed"]:
        raise RuntimeError("Pitch target continuity check failed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache", type=Path, default=CACHE / "validation/pitch-target")
    parser.add_argument("--results", type=Path, default=ARTIFACTS / "validation/pitch-target")
    args = parser.parse_args()
    run(project_path(args.cache), project_path(args.results))
