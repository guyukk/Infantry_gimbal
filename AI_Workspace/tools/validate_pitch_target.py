"""Focused Pitch transition/limit tests. B03 axis health/PID recovery and atomic
aim consumption are fixtures here; real integrations use validate_b03_health.py.
"""

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
    init_control = functions(read_source(ROOT / "User/APP/gimbal_behaviour.c"))[
        "gimbal_init_control"]
    control_loop = functions(read_source(ROOT / "User/gimbal/yaw_pitch_direct.c"))[
        "gimbal_control_loop"]
    set_control = functions(read_source(ROOT / "User/gimbal/yaw_pitch_direct.c"))[
        "gimbal_set_control"]
    raw_to_relative = functions(read_source(ROOT / "User/gimbal/yaw_pitch_direct.c"))[
        "pitch_dm_raw_to_relative"]
    config = read_source(ROOT / "User/ROBOT/Infantry_robot.h")
    config_names = ("YAW_CHANNEL", "PITCH_CHANNEL", "RC_DEADBAND", "YAW_RC_SEN",
                    "PITCH_RC_SEN", "YAW_MOUSE_SEN", "PITCH_MOUSE_SEN",
                    "INIT_PITCH_SET", "GIMBAL_INIT_PITCH_SPEED",
                    "YAW_INIT_FEEDBACK_TIMEOUT_MS")
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
#define fabs(x) ((x) < 0.0f ? -(x) : (x))
enum { GIMBAL_MOTOR_RAW = 0, GIMBAL_MOTOR_GYRO = 1, GIMBAL_MOTOR_ENCONDE = 2 };
enum { GIMBAL_ZERO_FORCE = 0, GIMBAL_INIT = 1, GIMBAL_RELATIVE_ANGLE = 4 };
#define PITCH_TURN 0
#define PITCH_DM_ZERO_OFFSET_RAD 0.0f
typedef struct {
    struct { int16_t ch[5]; } rc;
    struct { int16_t x, y; } mouse;
} RC_ctrl_t;
typedef struct {
    int last_gimbal_motor_mode, gimbal_motor_mode, given_current;
    int gimbal_motor_relative_angle_pid, gimbal_motor_gyro_pid;
    fp32 raw_cmd_current, current_set;
    fp32 absolute_angle_set, absolute_angle;
    fp32 relative_angle_set, relative_angle;
    fp32 motor_gyro_set, motor_gyro;
    fp32 min_relative_angle, max_relative_angle, emit_pos_set;
} gimbal_motor_t;
typedef struct { uint32_t last_fdb_time; } dji_measure_t;
typedef struct {
    gimbal_motor_t gimbal_yaw_motor, gimbal_pitch_motor;
    const RC_ctrl_t *gimbal_rc_ctrl;
    const dji_measure_t *yaw_dji_measure;
} gimbal_control_t;
gimbal_control_t gimbal_control;
dji_measure_t yaw_feedback;
int gimbal_behaviour = GIMBAL_ZERO_FORCE;
uint32_t fake_tick;
uint32_t yaw_relative_calls, yaw_pid_clears;
static uint8_t yaw_init_hold_valid;
volatile struct { uint8_t mit_target_valid; fp32 mit_target_pos; } pitch_dm_enable_status;
RC_ctrl_t rc_ctrl;
struct { struct { fp32 yaw, pitch; } receive; } aim;
struct { uint8_t ins_flag; } INS;
#define rad_format(x) (x)
#define rc_deadband_limit(input, output, deadband) \
    do { (output) = ((input) > (deadband) || (input) < -(deadband)) ? (input) : 0; } while (0)
void gimbal_motor_raw_angle_control(gimbal_motor_t *motor) { (void)motor; }
void gimbal_motor_absolute_angle_control(gimbal_motor_t *motor) { (void)motor; }
void gimbal_motor_relative_angle_control(gimbal_motor_t *motor)
{ ++yaw_relative_calls; motor->given_current = 37; motor->current_set = 37.0f; }
void gimbal_PID_clear(int *pid) { (void)pid; ++yaw_pid_clears; }
void PID_clear(int *pid) { (void)pid; ++yaw_pid_clears; }
uint32_t HAL_GetTick(void) { return fake_tick; }
enum { HEALTH_YAW=3 };
uint8_t health_fresh(uint8_t source,uint32_t now) {
    (void)source;return yaw_feedback.last_fdb_time!=0U &&
        (uint32_t)(now-yaw_feedback.last_fdb_time)<=YAW_INIT_FEEDBACK_TIMEOUT_MS;
}
uint8_t gimbal_axis_ready(gimbal_control_t *control,uint8_t pitch) { (void)control;(void)pitch;return 1U; }
fp32 auto_aim_take_increment(uint8_t pitch) {
    fp32 value=pitch?aim.receive.pitch:aim.receive.yaw;
    if(pitch) aim.receive.pitch=0; else aim.receive.yaw=0;
    return value;
}
void gimbal_behaviour_control_set(fp32 *yaw, fp32 *pitch, gimbal_control_t *control)
{ (void)control; *yaw = 0.0f; *pitch = 0.0f; }
void gimbal_absolute_angle_limit(gimbal_motor_t *motor, fp32 add)
{ motor->absolute_angle_set += add; }
static float pitch_dm_raw_to_relative(float raw)
''' + raw_to_relative + r'''
void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
''' + transition + r'''
void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
''' + limiter + r'''
void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
''' + relative_rc + r'''
void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
''' + init_control + r'''
void gimbal_control_loop(gimbal_control_t *control_loop)
''' + control_loop + r'''
void gimbal_set_control(gimbal_control_t *set_control)
''' + set_control + r'''
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
    pitch->relative_angle = -2.45f; /* current DM feedback */
    pitch->emit_pos_set = -2.25f; /* command from RAW mode */
    pitch->min_relative_angle = -2.7f;
    pitch->max_relative_angle = -2.3f;
    yaw->last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    yaw->gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    yaw->relative_angle = 0.2f;
    gimbal_mode_change_control_transit(&gimbal_control);
    check(near(pitch->relative_angle_set, -2.45f), 1);
    check(near(yaw->relative_angle_set, 0.2f), 2);
    gimbal_relative_angle_limit(pitch, -0.00396f);
    check(near(pitch->relative_angle_set, -2.45396f), 3);
    gimbal_relative_angle_limit(pitch, +0.00396f);
    check(near(pitch->relative_angle_set, -2.45f), 4);
    for (int i = 0; i < 100; ++i) gimbal_relative_angle_limit(pitch, -0.00396f);
    check(near(pitch->relative_angle_set, -2.7f), 5);
    for (int i = 0; i < 200; ++i) gimbal_relative_angle_limit(pitch, +0.00396f);
    check(near(pitch->relative_angle_set, -2.3f), 6);
    pitch->relative_angle_set = -2.45f;
    rc_ctrl.rc.ch[PITCH_CHANNEL] = 660;
    gimbal_relative_angle_control(&add_yaw, &add_pitch, &gimbal_control);
    check(near(add_pitch, 660.0f * PITCH_RC_SEN), 7);
    gimbal_relative_angle_limit(pitch, add_pitch);
    gimbal_control_loop(&gimbal_control);
    check(near(pitch->emit_pos_set, -2.45f + 660.0f * PITCH_RC_SEN), 8);
    rc_ctrl.rc.ch[PITCH_CHANNEL] = -660;
    gimbal_relative_angle_control(&add_yaw, &add_pitch, &gimbal_control);
    check(near(add_pitch, -660.0f * PITCH_RC_SEN), 9);
    gimbal_relative_angle_limit(pitch, add_pitch);
    gimbal_control_loop(&gimbal_control);
    check(near(pitch->emit_pos_set, -2.45f), 10);
    yaw->relative_angle = 2.0f;
    pitch->absolute_angle = 0.5f;
    INS.ins_flag = 1;
    add_yaw = 42.0f; add_pitch = 42.0f;
    gimbal_init_control(&add_yaw, &add_pitch, &gimbal_control);
    check(near(add_yaw, 0.0f) && near(add_pitch, -0.002f), 11);
    INS.ins_flag = 0;
    add_yaw = 42.0f; add_pitch = 42.0f;
    gimbal_init_control(&add_yaw, &add_pitch, &gimbal_control);
    check(near(add_yaw, 0.0f) && near(add_pitch, 0.0f), 12);
    gimbal_control.yaw_dji_measure = &yaw_feedback;
    gimbal_behaviour = GIMBAL_INIT;
    yaw->gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    pitch->gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    yaw->relative_angle = 1.0f;
    yaw->relative_angle_set = 0.0f;
    yaw->min_relative_angle = -0.1f;
    yaw->max_relative_angle = 0.1f;
    fake_tick = 1000;
    yaw_feedback.last_fdb_time = 0;
    gimbal_set_control(&gimbal_control);
    gimbal_control_loop(&gimbal_control);
    check(yaw_init_hold_valid == 0 && yaw->given_current == 0, 13);
    yaw_feedback.last_fdb_time = 1000;
    gimbal_set_control(&gimbal_control);
    gimbal_control_loop(&gimbal_control);
    check(yaw_init_hold_valid == 1 && near(yaw->relative_angle_set, 1.0f) &&
          yaw->given_current == 37 && yaw_pid_clears == 2, 14);
    yaw->relative_angle = 1.1f;
    gimbal_set_control(&gimbal_control);
    check(near(yaw->relative_angle_set, 1.0f), 15);
    fake_tick = 1200;
    gimbal_set_control(&gimbal_control);
    gimbal_control_loop(&gimbal_control);
    check(yaw_init_hold_valid == 0 && yaw->given_current == 0, 16);
    fake_tick = 1201;
    yaw_feedback.last_fdb_time = 1201;
    yaw->relative_angle = 1.3f;
    gimbal_set_control(&gimbal_control);
    check(yaw_init_hold_valid == 1 && near(yaw->relative_angle_set, 1.3f), 17);
    gimbal_behaviour = GIMBAL_ZERO_FORCE;
    yaw->gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_set_control(&gimbal_control);
    check(yaw_init_hold_valid == 0, 18);
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
               "scope": "Pitch target mapping; Yaw INIT waits for fresh feedback, holds current angle outside stored limits, and zeros current on stale feedback"}
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
