"""Focused Pitch MIT packing/position checks. B03 health/PID recovery is a fixture
here and is exercised with real sources by validate_b03_health.py instead.
Legacy timestamp fixtures use zero as not-fed; actual tick-zero seen is tested there.
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
    can = functions(read_source(ROOT / "User/Bsp/can_bsp.c"))
    gimbal = functions(read_source(ROOT / "User/gimbal/yaw_pitch_direct.c") + "\n" + read_source(ROOT / "User/gimbal/pitch_dm_control.c"))
    config = read_source(ROOT / "User/ROBOT/Infantry_robot.h")
    names = ("PITCH_MIT_MAX_POS_STEP_RAD", "PITCH_DM_LOWER_RECOVERY_RAD",
             "PITCH_DM_RECOVERY_POS_STEP_RAD")
    definitions = []
    for name in names:
        match = re.search(r"^\s*#define\s+" + name + r"\s+(\S+)", config, re.MULTILINE)
        if not match:
            raise RuntimeError("Missing configuration: " + name)
        definitions.append("#define " + name + " " + match.group(1))
    return r'''
#include <stdint.h>
#include <stddef.h>
typedef struct { void *Instance; } FDCAN_HandleTypeDef;
typedef struct {
    struct { int id, state; float pos; uint32_t last_fdb_time; } fdb;
    struct { float KP, KD, POS, VEL, TOR; } set;
    struct { float P_MIN, P_MAX, V_MIN, V_MAX, KP_MIN, KP_MAX,
                   KD_MIN, KD_MAX, T_MIN, T_MAX; } param;
} dm_measure_t;
typedef struct { struct { int16_t ch[5]; uint8_t s[2]; } rc; uint32_t last_fdb; } RC_ctrl_t;
typedef struct { dm_measure_t *gimbal_DM_measure; float emit_pos_set, relative_angle_set; } pitch_motor_t;
typedef struct { pitch_motor_t gimbal_pitch_motor; RC_ctrl_t *gimbal_rc_ctrl; } gimbal_control_t;
typedef struct {
    uint8_t attempts, last_tx_status, confirmed, exhausted;
    uint32_t last_attempt_tick, confirmed_feedback_tick;
    uint32_t mit_send_count, last_mit_tick;
    uint8_t last_mit_tx_status, mit_target_valid;
    float mit_target_pos;
} pitch_dm_enable_status_t;
typedef struct { uint8_t request, captured, stored_valid; int8_t result;
                 float first_pos, second_pos, min_pos, max_pos; } pitch_dm_cali_status_t;
enum { HAL_ERROR = 1, DM_mode = 0, GIMBAL_ZERO_FORCE = 0,
       GIMBAL_INIT = 1, GIMBAL_CALI = 2, GIMBAL_RELATIVE_ANGLE = 4,
       GIMBAL_MOTIONLESS = 6, DBUS_TOE = 0 };
#define PITCH_DM_CAN_ID 0x53
#define PITCH_MIT_FEEDBACK_TIMEOUT_MS 100U
#define PITCH_DM_RC_TIMEOUT_MS 100U
''' + "\n".join(definitions) + r'''
#define PITCH_MIT_KP 5.0f
#define PITCH_MIT_KD 0.1f
#define PITCH_TURN 0
#define PITCH_DM_ZERO_OFFSET_RAD 0.0f
#define GIMBAL_MODE_CHANNEL 0
#define toe_is_error(x) 0
FDCAN_HandleTypeDef hfdcan2;
#define GIMBAL_CAN hfdcan2
uint8_t MOTOR_Data[8];
volatile pitch_dm_enable_status_t pitch_dm_enable_status;
volatile pitch_dm_cali_status_t pitch_dm_cali_status;
int gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
struct { uint8_t ins_flag; } INS;
uint32_t fake_tick, last_can_id, send_count;
dm_measure_t motor;
gimbal_control_t control;
RC_ctrl_t rc_ctrl;
struct { uint32_t status, checks, failure; float observed_target; } test_result;
uint32_t HAL_GetTick(void) { return fake_tick; }
enum { DM_M3_ID=0x53, HEALTH_RC=0, HEALTH_PITCH=4, HEALTH_GATE_PITCH=2, HEALTH_PITCH_MASK=17 };
/* This test isolates serialization/position logic, not the final actuator gate. */
uint32_t health_gate_mask(uint8_t gate) { (void)gate; return 0U; }
uint8_t health_gate_ready(uint8_t gate,uint32_t mask,uint32_t now)
{ (void)gate;(void)mask;(void)now;return 1U; }
uint8_t gimbal_axis_ready(gimbal_control_t *c,uint8_t pitch) { (void)c;(void)pitch;return 1U; }
uint8_t health_fresh(uint8_t source,uint32_t now) {
    uint32_t stamp=source==HEALTH_RC ? rc_ctrl.last_fdb : motor.fdb.last_fdb_time;
    return stamp!=0U && (uint32_t)(now-stamp)<=100U;
}
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{ (void)hcan; (void)data; if (len != 8) return HAL_ERROR; last_can_id = id; send_count++; return 0; }
static uint16_t dm_mit_encode(float value, float lower, float upper, uint8_t bits)
''' + can["dm_mit_encode"] + r'''
uint8_t CAN_cmd_MIT(FDCAN_HandleTypeDef *hcan, uint16_t id, const dm_measure_t *motor)
''' + can["CAN_cmd_MIT"] + r'''
static uint8_t pitch_dm_feedback_allows_mit(const dm_measure_t *motor)
''' + gimbal["pitch_dm_feedback_allows_mit"] + r'''
static float pitch_dm_raw_to_relative(float raw)
''' + gimbal["pitch_dm_raw_to_relative"] + r'''
static float pitch_dm_relative_to_raw(float relative)
''' + gimbal["pitch_dm_relative_to_raw"] + r'''
static uint8_t pitch_dm_rc_ready(const RC_ctrl_t *rc, uint32_t now)
''' + gimbal["pitch_dm_rc_ready"] + r'''
static uint8_t pitch_dm_send_mit(gimbal_control_t *control_send)
''' + gimbal["pitch_dm_send_mit"] + r'''
void check(int condition, uint32_t number)
{ test_result.checks++; if (!condition && !test_result.failure) test_result.failure = number; }
int near(float a, float b)
{ float d = a - b; return d > -0.0001f && d < 0.0001f; }
void stop_test(void) { __asm("bkpt 0"); }
void test_entry(void)
{
    check(CAN_cmd_MIT(&GIMBAL_CAN, 0x53, &motor) == HAL_ERROR, 1);
    check(send_count == 0, 2);
    motor.param.P_MIN = -12.5f; motor.param.P_MAX = 12.5f;
    motor.param.V_MIN = -30.0f; motor.param.V_MAX = 30.0f;
    motor.param.KP_MIN = 0.0f; motor.param.KP_MAX = 500.0f;
    motor.param.KD_MIN = 0.0f; motor.param.KD_MAX = 5.0f;
    motor.param.T_MIN = -10.0f; motor.param.T_MAX = 10.0f;
    motor.set.KP = 5.0f; motor.set.KD = 0.1f;
    check(CAN_cmd_MIT(&GIMBAL_CAN, 0x53, &motor) == 0, 3);
    check(last_can_id == 0x53 && send_count == 1, 4);
    check(MOTOR_Data[0] == 0x7f && MOTOR_Data[1] == 0xff &&
          MOTOR_Data[2] == 0x7f && MOTOR_Data[3] == 0xf0 &&
          MOTOR_Data[4] == 0x28 && MOTOR_Data[5] == 0x05 &&
          MOTOR_Data[6] == 0x17 && MOTOR_Data[7] == 0xff, 5);
    motor.set.POS = 100.0f; motor.set.VEL = 100.0f;
    motor.set.KP = 1000.0f; motor.set.KD = 10.0f; motor.set.TOR = 100.0f;
    check(CAN_cmd_MIT(&GIMBAL_CAN, 0x53, &motor) == 0, 6);
    check(MOTOR_Data[0] == 0xff && MOTOR_Data[1] == 0xff &&
          MOTOR_Data[2] == 0xff && MOTOR_Data[3] == 0xff &&
          MOTOR_Data[4] == 0xff && MOTOR_Data[5] == 0xff &&
          MOTOR_Data[6] == 0xff && MOTOR_Data[7] == 0xff, 7);
    control.gimbal_pitch_motor.gimbal_DM_measure = &motor;
    control.gimbal_rc_ctrl = &rc_ctrl;
    control.gimbal_pitch_motor.emit_pos_set = -2.25f;
    fake_tick = 1000;
    check(pitch_dm_send_mit(&control) == 0, 8);
    check(pitch_dm_enable_status.mit_target_valid == 0 &&
          near(motor.set.KP, 0.0f) && near(motor.set.KD, 0.0f) &&
          near(motor.set.TOR, 0.0f), 9);
    motor.fdb.state = 1; motor.fdb.pos = -2.6f; motor.fdb.last_fdb_time = 999;
    pitch_dm_enable_status.confirmed = 1;
    pitch_dm_cali_status.stored_valid = 1;
    pitch_dm_cali_status.min_pos = -2.7f;
    pitch_dm_cali_status.max_pos = -2.3f;
    rc_ctrl.last_fdb = 999;
    rc_ctrl.rc.s[0] = 2;
    check(pitch_dm_send_mit(&control) == 0, 10);
    test_result.observed_target = pitch_dm_enable_status.mit_target_pos;
    check(pitch_dm_enable_status.mit_target_valid == 1, 11);
    check(near(pitch_dm_enable_status.mit_target_pos, -2.6f), 22);
    check(near(motor.set.KP, 5.0f) && near(motor.set.KD, 0.1f), 23);
    control.gimbal_pitch_motor.emit_pos_set = -2.25f;
    check(pitch_dm_send_mit(&control) == 0 &&
          near(pitch_dm_enable_status.mit_target_pos, -2.598f), 12);
    motor.fdb.id = 3; motor.fdb.state = 5;
    pitch_dm_enable_status.confirmed = 0;
    pitch_dm_enable_status.attempts = 1;
    check(pitch_dm_send_mit(&control) == 0 &&
          pitch_dm_enable_status.mit_target_valid == 0 &&
          near(motor.set.KP, 0.0f), 24);
    pitch_dm_enable_status.attempts = 2;
    check(pitch_dm_send_mit(&control) == 0 &&
          pitch_dm_enable_status.mit_target_valid == 1 &&
          near(motor.set.KP, 5.0f), 14);
    motor.fdb.id = 4;
    check(pitch_dm_send_mit(&control) == 0 &&
          pitch_dm_enable_status.mit_target_valid == 0 &&
          near(motor.set.KP, 0.0f), 15);
    motor.fdb.id = 3; motor.fdb.state = 8;
    check(pitch_dm_send_mit(&control) == 0 &&
          pitch_dm_enable_status.mit_target_valid == 0 &&
          near(motor.set.KP, 0.0f), 16);
    motor.fdb.state = 5;
    check(pitch_dm_send_mit(&control) == 0 &&
          pitch_dm_enable_status.mit_target_valid == 1 &&
          near(motor.set.KP, 5.0f), 17);
    pitch_dm_cali_status.stored_valid = 0;
    check(pitch_dm_send_mit(&control) == 0 &&
          pitch_dm_enable_status.mit_target_valid == 0 && near(motor.set.KP, 0.0f), 18);
    pitch_dm_cali_status.stored_valid = 1;
    rc_ctrl.last_fdb = 800;
    check(pitch_dm_send_mit(&control) == 0 && near(motor.set.KP, 0.0f), 19);
    rc_ctrl.last_fdb = 999;
    motor.fdb.pos = -2.8f;
    check(pitch_dm_send_mit(&control) == 0 && near(motor.set.KP, 0.0f), 20);
    motor.fdb.pos = -2.6f;
    gimbal_behaviour = GIMBAL_INIT;
    INS.ins_flag = 0;
    check(pitch_dm_send_mit(&control) == 0 &&
          near(pitch_dm_enable_status.mit_target_pos, -2.6f), 21);
    gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    fake_tick = 1200;
    check(pitch_dm_send_mit(&control) == 0 &&
          pitch_dm_enable_status.mit_target_valid == 0 &&
          near(motor.set.KP, 0.0f) && near(motor.set.TOR, 0.0f), 13);
    pitch_dm_cali_status.min_pos = -2.79f;
    pitch_dm_cali_status.max_pos = -2.3f;
    fake_tick = 1300; rc_ctrl.last_fdb = 1300;
    motor.fdb.pos = -2.801f; motor.fdb.last_fdb_time = 1300;
    gimbal_behaviour = GIMBAL_MOTIONLESS;
    check(pitch_dm_send_mit(&control) == 0 &&
          pitch_dm_enable_status.mit_target_valid == 1 &&
          near(pitch_dm_enable_status.mit_target_pos, -2.801f) &&
          near(motor.set.KP, 5.0f), 25);
    float last = pitch_dm_enable_status.mit_target_pos;
    for (int i = 0; i < 25; ++i) {
        fake_tick++; rc_ctrl.last_fdb = fake_tick;
        motor.fdb.last_fdb_time = fake_tick;
        pitch_dm_send_mit(&control);
        check(pitch_dm_enable_status.mit_target_pos >= last &&
              pitch_dm_enable_status.mit_target_pos <= -2.79f + 0.0001f, 26);
        last = pitch_dm_enable_status.mit_target_pos;
    }
    check(near(pitch_dm_enable_status.mit_target_pos, -2.79f), 27);
    motor.fdb.pos = -2.789f;
    fake_tick++; rc_ctrl.last_fdb = fake_tick; motor.fdb.last_fdb_time = fake_tick;
    check(pitch_dm_send_mit(&control) == 0 &&
          near(motor.set.POS, -2.79f), 28);
    pitch_dm_enable_status.mit_target_valid = 0;
    motor.fdb.pos = -2.811f;
    fake_tick++; rc_ctrl.last_fdb = fake_tick; motor.fdb.last_fdb_time = fake_tick;
    check(pitch_dm_send_mit(&control) == 0 &&
          pitch_dm_enable_status.mit_target_valid == 0 &&
          near(motor.set.KP, 0.0f), 29);
    motor.fdb.pos = -2.801f;
    fake_tick++; rc_ctrl.last_fdb = fake_tick; motor.fdb.last_fdb_time = fake_tick;
    check(pitch_dm_send_mit(&control) == 0 &&
          near(pitch_dm_enable_status.mit_target_pos, -2.801f), 30);
    motor.fdb.pos = -2.82f;
    fake_tick++; rc_ctrl.last_fdb = fake_tick; motor.fdb.last_fdb_time = fake_tick;
    check(pitch_dm_send_mit(&control) == 0 &&
          pitch_dm_enable_status.mit_target_valid == 0 &&
          near(motor.set.KP, 0.0f), 31);
    test_result.status = 0x600d;
    stop_test();
}
'''


def run(cache, results):
    cache.mkdir(parents=True, exist_ok=True)
    results.mkdir(parents=True, exist_ok=True)
    source = cache / "pitch-mit-check.c"
    obj = cache / "pitch-mit-check.o"
    binary = cache / "pitch-mit-check.axf"
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
        symbols = {symbol.name: symbol["st_value"] for symbol in
                   elf.get_section_by_name(".symtab").iter_symbols()}
        entry = elf["e_entry"]
    uc.reg_write(UC_ARM_REG_SP, 0x200FF000)

    def stop_on_breakpoint(machine, address, size, _):
        if machine.mem_read(address, 2) == b"\x00\xbe":
            machine.emu_stop()

    uc.hook_add(UC_HOOK_CODE, stop_on_breakpoint)
    uc.emu_start(entry | 1, 0, timeout=10000000, count=2000000)
    raw = bytes(uc.mem_read(symbols["test_result"], 16))
    status, checks, failure = (int.from_bytes(raw[i:i + 4], "little")
                               for i in range(0, 12, 4))
    import struct
    outcome = {"status": hex(status), "checks": checks, "failure": failure,
               "first_target": struct.unpack("<f", raw[12:16])[0],
               "passed": status == 0x600d and failure == 0,
               "scope": "MIT packing and enable gate; measured -2.801 recovery from -2.79 lower limit; inward-only slew, recovery window, stale feedback and normal handoff"}
    (results / "results.json").write_text(json.dumps(outcome, indent=2), encoding="utf-8")
    print(json.dumps(outcome, indent=2))
    if not outcome["passed"]:
        raise RuntimeError("Pitch MIT protocol check failed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache", type=Path, default=CACHE / "validation/pitch-mit")
    parser.add_argument("--results", type=Path, default=ARTIFACTS / "validation/pitch-mit")
    args = parser.parse_args()
    run(project_path(args.cache), project_path(args.results))
