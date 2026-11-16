> 2026-11-16：B04/B06软件阶段完成，B07总核与交付见[B07最终报告](B07-总核与最终交付报告.md)。P06温控仅交隔离候选，极性仍待设备确认；小陀螺平移未修复，整车设备验收未完成。

> B05更新（2026-11-15）：校准值域、当前ARMCC布局和合法帧兼容已软件验证；校准操作以[最终手册](calibration-manual.md)为准，接口有效性见[说明](ui-and-interfaces.md)。下文是原始调查记录，设备未知项仍未验收；原“待软件检查”项由[B05报告](B05-实施与软件签收报告.md)覆盖，不代表B04/B06/B07已开始。

# 板间协议交叉一致性核查（C↔G，2026-11-14）

- 板别：C（F407 底盘）+ G（H723 云台），两端各存一份相同内容
- 日期：2026-11-14
- run-id：20261114-174009
- 目的：把 28 字节（C→G）与 46 字节（G→C）帧做成“发送端写法 vs 接收端读法”的机械对照，确认两端对同一帧的理解逐字节一致，并给 verifier 一份必须冻结的字段清单。
- 判定口径：**字节级一致** = 偏移、长度、字节序、表达式语义都对得上；**语义不一致** = 字节对得上但解释/变换/数据源不同；**未使用** = 一端写了但另一端没有任何消费者。
- 快照：本核查基于 2026-11-14 18:00 左右的工作树（B02 进行中）。发送端 `C:application/message_task.c`、`G:User/APP/message_task.c` 均未改动；接收端已被 B02 重写（C `bsp/boards/bsp_usart.c`、G `User/Bsp/bsp_usart.c`）。为可复现，固定以下 SHA256：

```text
C application/message_task.c   8EA32991743E817DF350EF11DD31E4A293F499C0A1C15046CCB93240C1D4DDBA
C bsp/boards/bsp_usart.c       FF9CABEC1176C7EAF0BCDEABEFCD51B1C55773BAB98F5C9DF3B5E5393F22A685
C application/frame_stream.c   61335716ABFA3C74A2EE9CF730EC86254549638D08667655FEA970184344B104
G User/APP/message_task.c      551E25143F5DF6CA67D95A385A58CF06D90EA526D1B73623F557AACB1355BD5A
G User/Bsp/bsp_usart.c         756F686408F9B64330E4D419E19D3AB49EE2251C3A564D0876CA69AA9862EBAC
```

- 重要提示：两份 `interboard-protocol.md` 的“当前接收实现/缺陷”章节描述的是 B00 基线；B02 已经重写接收层，因此那两章现在应视为 **superseded**（见第 8 节）。本文件只对“帧契约”负责，不对 B02 测试结果负责。

---

## 1. 帧常量对照

| 常量 | C 侧 | G 侧 | 是否一致 |
| --- | --- | --- | --- |
| C→G 帧头/尾 | `'s'`(0x73) / `'e'`(0x65)（`C application/message_task.c:63`、`:100`） | `CHASSIS_FRAME_HEAD 's'` / `CHASSIS_FRAME_TAIL 'e'`（`G User/Bsp/bsp_usart.h:26-27`） | 一致 |
| C→G 长度 | 28（`C application/message_task.c:48`） | `CHASSIS_DATA_LENGTH 28`（`G User/Bsp/bsp_usart.h:18`） | 一致 |
| G→C 帧头/尾 | 0xFE / 0xFD（`C bsp/boards/bsp_usart.c:26-27`） | 0xFE / 0xFD（`G User/APP/message_task.c:45`、`:85`） | 一致 |
| G→C 长度 | `GIMBAL_DATA_LENGTH 46`（`C bsp/boards/bsp_usart.h`） | 46（`G User/APP/message_task.c:41`） | 一致 |
| 字节序 | 小端 IEEE754（`bytes[0]`=LSB） | 同 | 一致 |
| C→G 角度限值 | n/a（发送端不限） | ±3.2 rad（`G User/Bsp/bsp_usart.h:30`） | 覆盖 C 的 ±π |
| C→G 速度限值 | n/a | vx/vy ≤10 m/s、wz ≤50 rad/s（`:31-32`） | 覆盖当前恒 0 |
| C→G 模式上限 | 0..2（枚举） | `CHASSIS_MODE_MAX 2u`（`:33`） | 一致 |
| G→C 角度限值 | ±(π+0.001)（`C bsp/boards/bsp_usart.c:30`） | n/a | 覆盖 G 的 `rad_format` 输出 |
| G→C 枚举上限 | behaviour ≤6、shoot ≤4（`C bsp/boards/bsp_usart.c:28-29`） | 枚举定义 0..6 / 0..4 | 一致 |

---

## 2. 28 字节（C→G）发送 vs 接收

| 偏移 | 类型/长度 | C 发送端写法（表达式，行号） | G 接收端读法（表达式，行号） | 判定 |
| ---: | --- | --- | --- | --- |
| 0 | u8 | `'s'`（`message_task.c:63`） | `frame[0]==CHASSIS_FRAME_HEAD`（`bsp_usart.c:130-133`） | 一致 |
| 1..4 | f32 LE | `chassis_move.chassis_yaw` ← `wrap(INS_angle[0])`（`:51`、`:66-69`） | `chassis_yaw`（`bsp_usart.c:139-143`） | 偏移/类型一致；**G 侧无消费者** |
| 5..8 | f32 LE | `chassis_move.chassis_pitch` ← `wrap(INS_angle[1])`（`:52`、`:71-74`） | `chassis_pitch`（`:145-149`） | 同上 |
| 9..12 | f32 LE | `chassis_move.chassis_roll` ← `wrap(INS_angle[2])`（`:53`、`:76-79`） | `chassis_roll`（`:151-155`） | 同上 |
| 13 | u8 | `(uint8_t)chassis_behaviour_mode`，枚举 0..2（`:55`、`:81`） | `chassis_mode`（`:157`），校验 ≤2（`:197-200`） | 一致 |
| 14 | u8 | `robot_status.robot_id`（`:57`、`:83`） | `robot_id`（`:158`），**不做范围校验**（注释明确 0 合法） | 偏移一致；G 侧无消费者 |
| 15..18 | f32 LE | `chassis_move.vx`（`:58`、`:85-88`）——全工程从未赋值，恒 0 | `chassis_vx`（`:160-164`），校验 ≤10 m/s（`:191-196`） | 偏移一致；**占位 0 被当速度值解析** |
| 19..22 | f32 LE | `chassis_move.vy`（`:59`、`:90-93`）恒 0 | `chassis_vy`（`:166-170`） | 同上 |
| 23..26 | f32 LE | `chassis_move.wz`（`:60`、`:95-98`）恒 0 | `chassis_wz`（`:172-176`），校验 ≤50 rad/s | 同上 |
| 27 | u8 | `'e'`（`:100`） | `frame[27]==CHASSIS_FRAME_TAIL`（`bsp_usart.c:134-137`） | 一致 |

补充：[源码事实] `chassis_move.vx/vy/wz` 在 C 全仓仍然没有任何赋值点（`grep -E "chassis_move\.(vx|vy|wz)\s*=[^=]"` 无结果），所以 15..26 在线上恒为 12 个 0x00。G 侧新解析器把它们当普通 float 校验并发布到 `chassis_data`，但 `chassis_data_read()` 目前**没有外部调用者**（`G User/Bsp/bsp_usart.c:436`，全仓无调用）。

---

## 3. 46 字节（G→C）发送 vs 接收

| 偏移 | 类型/长度 | G 发送端写法（表达式，行号） | C 接收端读法（表达式，行号） | 判定 |
| ---: | --- | --- | --- | --- |
| 0 | u8 | 0xFE（`G message_task.c:45`） | `USART1_FRAME_HEAD` 校验（`C bsp_usart.c:134`） | 一致 |
| 1..4 | f32 LE | `rad_format(gimbal_yaw_motor.radian_of_ecd)`（`:34`、`:48-52`），即 `rad_format(ecd*MOTOR_ECD_TO_RAD)` | `gimbal_to_chassis` 取 1..4 → `motor_yaw`，先 clamp ±π，再按 `GIMBAL_YAW_RADIAN_REVERSE=1` 取反（`C bsp_usart.c:296-308`、`:301-302`、`:306-308`） | 偏移/类型一致；**语义含 clamp+取反变换**（见 4.2） |
| 5..8 | f32 LE | `INS_angle[0]`（`:35`、`:54-58`） | `GIMBAL_INS_yaw`（`:310-314`） | 一致；C 校验 ±(π+0.001) |
| 9..12 | f32 LE | `INS_angle[1]`（`:60-63`） | `GIMBAL_INS_pitch`（`:316-320`） | 一致 |
| 13..16 | f32 LE | `INS_angle[2]`（`:65-68`） | `GIMBAL_INS_roll`（`:322-326`） | 一致 |
| 17 | u8 | `(uint8_t)gimbal_behaviour`，0..6（`:36`、`:70`） | `chassis_move.gimbal_behaviour`（`:328-329`），校验 ≤6（`:145`） | 一致 |
| 18 | u8 | `shoot_control.shoot_mode`，0..4（`:37-38`、`:72`） | `chassis_move.gimbal_shoot_mode`（`:331-332`），校验 ≤4（`:146`） | 一致 |
| 19..37 | 19B 原样 | `vtm_data` 字节 2..20：`data_chassis[19+i]=vtm_data_ptr[2+i]`，`i<sizeof-2`（`:75-78`） | 写回 `vtm_data` 字节 2..20：`vtm_data_ptr[2+i]=gimbal_data[19+i]`，带 `19+i<46` 边界（`:334-339`） | 偏移/长度一致；**数据源语义不一致**（见 4.1） |
| 38..44 | 7B | 显式清零（`:79-82`） | **不读取、不校验**（`gimbal_to_chassis` 只用 1..37） | 发送端清零、接收端未校验 |
| 45 | u8 | 0xFD（`:85`） | `USART1_FRAME_TAIL` 校验（`C bsp_usart.c:134`） | 一致 |

---

## 4. 专项核对（lead 指定四项）

### 4.1 46B 的 1..4 是否只有 G 一处写入

- [源码事实] 是。C 端唯一构造板间帧的函数是 `send_to_gimbal()`，长度固定 28，不含 1..4 这个 46 字节概念（`C application/message_task.c:38-104`）。C 的 46 字节路径只有接收（`gimbal_to_chassis`），不写。
- [源码事实] G 端唯一写入是 `send_data_to_chassis()` 的 `data_chassis[1..4]`（`G User/APP/message_task.c:34`、`:48-52`）；G 的 28 字节路径只有接收。
- 结论：1..4 的写入者唯一；不存在第二处写入。

### 4.2 46B 的 38..44 预留是否两端都清零

- 发送端：G 在 `:79-82` 循环写 0，覆盖 38..44 共 7 字节。
- 接收端：C 的 `gimbal_to_chassis()` 只读 `gimbal_data[1..37]`（`:296-339`），**从不读 38..44**，也不校验它们为 0。
- 结论：**只有发送端清零，接收端不校验**。契约上应写成“发送端必须为 0，接收端当前忽略”；如果未来把预留字节另作他用，C 端不需要改，但 golden 帧必须锁 0。

### 4.3 46B 的 19..37 VTM 是否两端都按 offset 2 开始拷

- G 发：`vtm_data_ptr[2+i]`，`i<sizeof(vtm_data)-2`（`:75-78`）；`sizeof==21` 有断言（`G video_transmission_module.h:67`）。
- C 收：`vtm_data_ptr[2+i] = gimbal_data[19+i]`，额外加 `(19+i)<GIMBAL_DATA_LENGTH` 边界（`C bsp_usart.c:334-339`）。
- 结论：两端都从 VTM 的 **byte 2** 开始、长度 19，一致。
- 但 [源码事实] G 的 `vtm_data` 只在 `vtm_data_init()` 里赋值一次（`G video_transmission_module.c:6-52`），全仓没有第二个写点；G 也没有 21 字节 VTM 解包函数。因此 19..37 在线上是**中性默认值**（ch0..ch3=1024、gear=N、其余 0），不是实时图传数据。C 侧 `RC_TYPE==1` 分支（`C application/remote_control.c:257-284`）是死分支，即使读到的也是中性值。

### 4.4 28B 的 15..22 是否恒 0

- C 发：`chassis_move.vx/vy`（`:58-59`、`:85-93`）。这两个字段全仓无赋值点，BSS 初值 0，故线上恒 0x00。
- G 收：`:160-170` 解析并做 ≤10 m/s 校验，通过后发布到 `chassis_data`；无外部读者。
- 结论：**当前恒 0 成立**，但这是“从未赋值”的结果，不是显式的 `=0`；一旦 C 端将来写入真实速度，G 的 10 m/s 限值会开始生效（与 C 的 2.6 m/s 上限不冲突）。

---

## 5. 逐字段判定汇总

| 帧 | 字段数 | 字节级一致 | 语义不一致 | 未使用（一端有写/读但无消费者） |
| --- | ---: | ---: | ---: | ---: |
| 28B C→G | 11 段 | 11 | 0 | 10（除偏移 13 的 mode 外，G 侧 `chassis_data` 全部无读者） |
| 46B G→C | 11 段 | 11 | 1（19..37 的 VTM 数据源） | 3（5..16 的 INS 角、17/18 枚举在 C 侧只被 `usart1_status` 记录，当前无外部读者） |
| 合计 | 22 段 | **22** | **1** | 13 |

说明：“字节级一致”指偏移/类型/字节序/数值传递方式一致；它不保证语义正确，语义问题单列在第 4、6 节。

---

## 6. 真正的不一致/必须写入契约的差异（4 条）

| # | 类型 | 内容 | 证据 | 对 verifier 的影响 |
| --- | --- | --- | --- | --- |
| 1 | 语义不一致（数据源） | 46B 19..37 的契约是“VTM 图传数据”，实际 G 只发 `vtm_data_init()` 的中性默认值，从未更新 | `G User/APP/video_transmission_module.c:6-52`（唯一写点）；G 无 unpack 函数 | golden 帧若按“实时 VTM”设计会永远对不上；只能冻结“中性默认值”或先造注入点 |
| 2 | 语义变换（未写入契约） | 46B 1..4 G 发未取反的 `rad_format(radian_of_ecd)`；C 收后 clamp ±π 并取反（`GIMBAL_YAW_RADIAN_REVERSE=1`），对外发布的是 `-wire` | `G message_task.c:34`；`C bsp_usart.c:301-308`；`C application/robot_param.h:30` | 跨端比对时必须约定“线上值”和“C 发布值”差一个负号；否则会误判方向错误 |
| 3 | 校验缺口（预留字节） | 46B 38..44 只由 G 清零，C 不校验 | `G message_task.c:79-82`；C 只读 1..37 | golden 应锁 0，但不要把它当“C 会拒绝非零”的保护 |
| 4 | 占位被当值（速度） | 28B 15..26 是恒 0 占位，G 端按速度值解析并限幅 | `C message_task.c:58-60`；`G bsp_usart.c:191-196` | D10 的“零值兼容”必须写成契约：0 表示 unavailable，不是“已测得静止” |

另：两份 `interboard-protocol.md` 的“当前接收实现/缺陷”章节与 B02 后的代码不一致（C/G 接收层均已重写为流式解析）。这属于**文档被代码取代**，应标 superseded，不是帧契约矛盾。

---

## 7. 必须冻结的 golden 帧字段清单（给 verifier）

### 7.1 C→G（28B）

| 偏移 | 冻结内容 | 允许变化 |
| --- | --- | --- |
| 0 / 27 | 0x73 / 0x65 | 不允许 |
| 1..12 | 三个 f32（小端），分别对应 C 的 chassis yaw/pitch/roll；用固定输入值 | 只允许由注入输入决定 |
| 13 | 0/1/2（NO_MOVE/FOLLOW/SPIN） | 只允许这三个值 |
| 14 | robot_id（0 表示裁判未就绪） | 0..255，G 不校验 |
| 15..26 | 12 × 0x00（vx/vy/wz 恒 0） | 冻结为 0；D10 明确不得填目标速度 |
| 27 | 0x65 | 不允许 |

### 7.2 G→C（46B）

| 偏移 | 冻结内容 | 允许变化 |
| --- | --- | --- |
| 0 / 45 | 0xFE / 0xFD | 不允许 |
| 1..4 | f32，G 的 `rad_format(radian_of_ecd)`；需注明 C 发布值 = `-clamp(wire,±π)` | 由 Yaw 编码器输入决定 |
| 5..16 | 三个 f32 INS 角，范围 ±π（C 侧校验 ±(π+0.001)） | 由 IMU 输入决定 |
| 17 | 0..6（gimbal_behaviour_e） | 只允许这 7 个值 |
| 18 | 0..4（shoot_mode_e） | 只允许这 5 个值 |
| 19..37 | VTM 19 字节；当前为 `vtm_data_init()` 中性值（按实际构建实测位域后冻结） | 在 VTM 数据源接入前保持不变 |
| 38..44 | 7 × 0x00 | 发送端必须为 0；接收端当前不校验 |
| 45 | 0xFD | 不允许 |

### 7.3 冻结前必须做的两件事

1. 用实际 ARMCC 构建测出 `VTM_Data_t` 的 19 个线上字节（当前只有 `sizeof==21` 断言，位域偏移仅注释）。
2. 明确 46B 1..4 的“线上值 vs C 发布值”符号约定，写进 golden 说明，避免测试把正确方向判成错误。

---

## 8. 两份 protocol 文档的 superseded 标注

| 文档 | 被取代的章节 | 取代者 | 处理 |
| --- | --- | --- | --- |
| C `interboard-protocol.md` | 第 9 节“当前接收实现的缺陷”、第 6 节“C 侧当前收发代码位置”中的接收部分 | B02 后的 `C bsp/boards/bsp_usart.c` + 新 `frame_stream.*`、`rx_status.*` | 保留原文作为 B00 基线证据，不修改；在 B02 报告中标注 superseded |
| G `interboard-protocol.md` | 第 3 节“G 接收 28 字节：当前实现”、第 6 节“G 接收链的其它缺陷” | B02 后的 `G User/Bsp/bsp_usart.c`（流式解析 + 序号发布） | 同上 |

本文件（consistency）只对“帧契约 + 当前工作树实现”负责；B02 完成后的最终数值以 B02 的 golden 测试为准。

---

## 9. 给 lead 的一句话结论

- 字节级：**0 条真正不一致**（22 段全部对齐）。
- 语义/校验级：**4 条必须写入契约**（VTM 数据源、Yaw 符号变换、预留只清不校、速度占位被当值）。
- 文档状态：两份 protocol.md 的接收实现章节已被 B02 取代，需标 superseded，不改原文。
