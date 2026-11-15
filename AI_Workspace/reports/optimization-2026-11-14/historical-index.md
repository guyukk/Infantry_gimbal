# G 仓历史报告索引与 superseded 标注（2026-11-14）

- 板别：G = 云台板（H723）
- 日期：2026-11-14
- run-id：20261114-174009
- 依据：交接文档 D20（当前报告与历史证据分开；不抹去旧报告的日期、旧警告数、旧路径；修当前索引并标 superseded）
- 原则：**本文件只新建索引，不修改任何历史报告正文**。旧数字/旧结论按原样引用，仅在本文件里作为历史事实出现。
- 路径约定：相对 `G\AI_Workspace\`。

---

## 1. 历史报告逐条

| 路径 | 日期 | 当时声称的结论 | 本轮状态 | 取代者/关联 |
| --- | --- | --- | --- | --- |
| `reports/独立项目收敛-2026-11-02.md` | 2026-11-02 | CubeMX 6.17 对 CMSIS-RTOS v1 `.ioc` 生成内部错误并丢失 FreeRTOS 入口，ST 确认不支持；正式文件未写回，待兼容生成器 | **仍有效**（本轮 P16 结论相同：不重生成） | `optimization-2026-11-14/hardware-unknowns.md`、P16 |
| `reports/云台工程简化与结构整理.md` | 2026-11-02 | 当前架构与删减边界：Keil Target/任务/IRQ/DMA/回调/MAP 为判定依据；USART1 收 28/发 46、19 字节 VTM+7 字节清零预留；UART5 遥控；USB uproto 承担自瞄与主机通信 | **仍有效**（架构基线）；其“当前清单”角色由本轮配置清单细化 | `optimization-2026-11-14/current-config.md` |
| `reports/云台初始化与DM-Pitch清理-2026-11-07.md` | 2026-11-07 | DM 两点校准只记录有机械余量的位置；保存前要求当前反馈在区间内、写后回读；启动读失败/标记/版本/CRC/范围不符→零刚度；`PITCH_TURN` 与零点偏移用同一对正反换算 | **仍有效**（当前 `pitch_dm_cali_*` 即此实现） | `optimization-2026-11-14/calibration-manual.md`、`calibration-gaps.md` |
| `reports/云台运行链与僵尸代码清理-2026-11-06.md` | 2026-11-06 | 判定边界：唯一 Target、110 条目、`ROBOT_MODE=debug`；删减证据由 Target/任务/IRQ/MAP/编译共同组成；冻结控制参数、帧、错误输出、Flash 布局 | **仅历史**（其条目数 110 与当前 109 不同）；删除项继续由 P13 审计 | `optimization-2026-11-14/issues.csv`、`hardware-unknowns.md` |
| `reports/Pitch-DM安全位置校准-2026-11-06.md` | 2026-11-06 | 旧 HD/GM 记录保留 W25Q 第一扇区；DM Pitch 记录独立第二扇区（标记/版本/两端点/CRC32）；旧 GM 自动校准禁用；Yaw 只加载历史值、无新流程 | **仍有效**（记录 v1 未变）；“无 Yaw 新流程”仍是本轮 D13 的边界 | `calibration-manual.md`（第 2/3 节） |
| `reports/Pitch电机上电使能重试-2026-11-03.md` | 2026-11-03 | `Motor_enable` 返回只表示入队；离线 10 项重试检查通过；首次/后续重试与次数上限 | **仍有效**（当前 2000/500/8，`yaw_pitch_direct.c:81-144`） | `current-config.md`、`calibration-manual.md` |
| `reports/Pitch反馈状态5与MIT零力矩-2026-11-06.md` | 2026-11-06 | MIT 门限额外接受 `0x53/state=5/id=3`；只用它放行 MIT，不确认使能；其他异常仍零刚度 | **仍有效**，并被 11-07 报告补充“不能区分红灯/绿灯” | `Pitch状态5使能回归与Yaw免回零-2026-11-07.md` |
| `reports/Pitch冷启动自动恢复-2026-11-07.md` | 2026-11-07 | 冷启动 Pitch 可能无力矩；两条一直保持失败的路径；文中已注明“使能部分已被状态5使能回归修正”，Flash 恢复部分仍使用 | **部分 superseded**（使能结论被取代；恢复逻辑仍有效） | `Pitch状态5使能回归与Yaw免回零-2026-11-07.md` |
| `reports/Pitch绿灯无输出修正-2026-11-03.md` | 2026-11-03 | 反馈门控会永久挡住 EMIT；改为 `pitch_dm_enable_ready()` 不再决定能否发送 EMIT | **与当前源码不一致（superseded）**：当前 `pitch_dm_send_mit()` 重新用 `confirmed` 或 `state5 && attempts>=2` 作 MIT 门控（`yaw_pitch_direct.c:197-206`） | 由 11-06/11-07 的 MIT/冷启动改动取代；原文保留 |
| `reports/Pitch目标角切换修正-2026-11-05.md` | 2026-11-05 | 初始目标设为边界中值 -2.25；ENCONDE 用切换前 `emit_pos_set` 起步；首次使能 1000 ms/后续 500 ms/8 次 | **部分 superseded**：-2.25 归中已被 11-07“上电自动到限位中点（已撤销）”撤销；当前初始目标来自默认区间加载/当前反馈 | `Pitch上电自动到限位中点-2026-11-07.md`（撤销）、`current-config.md` |
| `reports/Pitch切换MIT协议-2026-11-06.md` | 2026-11-06 | Pitch MIT 八字节发 `0x53`，五量按量程编码，限幅/NaN 拒发并返回 HAL 入队状态；使能帧不变 | **仍有效**（`CAN_cmd_MIT`，`can_bsp.c:232-264`） | `current-config.md`、`interboard-protocol.md`（CAN 部分） |
| `reports/Pitch上电自动到限位中点-2026-11-07.md` | 2026-11-07 | 标题即“已撤销”：保留归中过程同步、反馈丢失保持、两点更新重算中点等记录 | **自我撤销（superseded）**：当前启动不再自动到中点 | `Pitch状态5使能回归与Yaw免回零-2026-11-07.md`、`Yaw初始化限幅导致回零-2026-11-07.md` |
| `reports/Pitch下限外单向回收-2026-11-07.md` | 2026-11-07 | 下限外回收只向内；Flash 延迟恢复允许反馈落在同一窗口；CRC/版本/范围/新鲜度保留 | **仍有效**（0.02 窗口、0.0005 步长，`yaw_pitch_direct.c:242-275`） | `calibration-manual.md`（第 6.3 节） |
| `reports/Pitch状态5使能回归与Yaw免回零-2026-11-07.md` | 2026-11-07 | 本电机 CAN ID 与 Master ID 都是 0x53，反馈首字节含 ID 位，state=5 不能区分红灯/绿灯；Yaw INIT 保持当前角、不回零 | **仍有效**（当前 `yaw_pitch_direct.c:529-556`、`:146-158`） | `current-config.md`、`calibration-manual.md` |
| `reports/Pitch左摇杆上下排查-2026-11-03.md` | 2026-11-03 | `detect_task.c` 被编译但未创建、无 `detect_hook` 调用，`DBUS_TOE` 不能当运行中掉线结论；此前归因撤回 | **仍有效且更强**：本轮确认 `detect_hook` 无任何调用者、`error_list` 为 BSS 零，`toe_is_error` 恒 false | `optimization-2026-11-14/current-config.md`、`controls-and-keys.md` |
| `reports/Yaw初始化限幅导致回零-2026-11-07.md` | 2026-11-07 | 反馈未到/超 100 ms 时 INIT 中 Yaw 零电流并撤销保持；恢复后从当前角重新捕获并清 PID | **仍有效**（`yaw_pitch_direct.c:529-556`、`:591-597`） | `calibration-manual.md`（第 3.2 节） |
| `reports/Yaw抖动与Pitch零位耦合修正-2026-11-06.md` | 2026-11-06 | MIT 后 `fdb.pos` 可表示绝对位置，`Pitch≈-2.7952` 时投影公式近似反号；增加 `PITCH_DM_YAW_GYRO_PROJECTION_ENABLE=0`，Yaw 速度直取 gyroZ | **仍有效**（宏仍为 0，`Infantry_robot.h:141`；`yaw_pitch_direct.c:437-442`） | `calibration-manual.md`（第 6.2 节） |

---

## 2. history 与候选清单

| 路径 | 日期 | 当时声称的结论 | 本轮状态 | 备注 |
| --- | --- | --- | --- | --- |
| `reports/history/2026-10-08-batch-01.md` | 2026-10-08 | 两板简化执行记录；底盘删 33/125→111、0 错 0 警；云台删 77/139→110、0 错 40 警；VTM 19 字节修复；42 项离线检查 | **仅历史**；条目数/警告数与当前 109/30 不同 | 与 C 仓 `simplification/batch-01.md` 同源 |
| `reports/history/2026-10-08-preparation.md` | 2026-10-08 | 简化前置准备：只建基线，未删码 | **superseded（阶段）** | 由 batch-01 执行 |
| `reports/history/2026-10-08-candidates.csv` | 2026-10-08 | 当时的候选清单 | **部分关闭/部分仍开**；P13 复审 | 新问题登记见 `optimization-2026-11-14/issues.csv` |
| `reports/history/2026-11-02-shared-structure-02.md` | 2026-11-02 | 结构整理续批与验收：逐条警告诊断、Keil 返回码 1=有警告但成功；底盘/云台初始化链与任务清单 | **仅历史**；当前任务清单以 `current-config.md` 为准 | — |
| `artifacts/cleanup-2026-11-06/candidates.csv` | 2026-11-06 | 清理决策清单（removed batch 01/02、unused warnings 等） | **仅历史**；本轮 P13 重新审计 | 不在 `reports/` 下，但属历史候选证据 |

---

## 3. 警告数与“当前构建”的关系

- 历史报告中的 0 错 40 警（batch-01，2026-10-08）、2026-11-07 的 0 错 30 警都是**当时事实**，不得改写、不得合并成“一直是 30”。
- 本轮没有重新构建；任何“当前 0 警告/30 警告”的说法必须由 B01/B02 的新鲜日志支撑。历史数字只作对照，见 `hardware-unknowns.md` 与 P12。

---

## 4. 本轮报告（不是历史）

`reports/optimization-2026-11-14/` 下的 `current-config.md`、`interboard-protocol.md`、`interboard-protocol-consistency.md`、`controls-and-keys.md`、`calibration-manual.md`、`calibration-gaps.md`、`hardware-unknowns.md`、`historical-index.md` 为本轮生成，不回写历史。

---

## 5. 索引核对说明

1. “当时结论”列只做摘要，细节以原文为准；引用旧结论时必须同时引用“本轮状态”。
2. 两份报告已被明确取代/部分取代：`Pitch绿灯无输出修正-2026-11-03`（门控结论与现状相反）、`Pitch上电自动到限位中点-2026-11-07`（自我撤销）、`Pitch目标角切换修正-2026-11-05`（-2.25 归中被撤）。这三条最容易被误当现行行为。
3. 旧路径与旧警告数只在本文件出现，不扩散到新报告正文以外的地方。
