> 2026-11-16：B04/B06软件阶段完成，B07总核与交付见[B07最终报告](B07-总核与最终交付报告.md)。P06温控仅交隔离候选，极性仍待设备确认；小陀螺平移未修复，整车设备验收未完成。

> 2026-11-15 B05软件交付完成、待用户确认；当前结论见[B05-实施与软件签收报告.md](B05-实施与软件签收报告.md)。B04/B06/B07未开始；下方旧报告保持历史语境。

> 2026-11-15 B03 软件阶段已完成；实机未验收。当前结论见 [B03-两步实施与签收报告.md](B03-两步实施与签收报告.md)。以下保留为历史记录。

> 2026-11-15 B02 收尾签收：软件阶段通过；以 [B02-收尾签收报告](B02-收尾签收报告.md) 为新增复验及最终构建依据。下文保留终止时历史记录；B03–B07 及实机验收未完成。

# 整车工程优化执行回报

执行者：本会话执行 agent（Lead + 4 个并行子 agent：doc-analyst / chassis-dev / gimbal-dev / verifier）
执行起止时间（Asia/Shanghai）：2026-11-14 17:38 — 2026-11-14 19:16（**用户要求提前终止**）
依据：整车工程优化交接与执行规范-2026-11-14.md，版本 1.0
run-id：20261114-174009
最终状态：**software_partial**
可部署状态：not_approved（无设备验收、无另行授权；本轮也未烧录）

> 终止说明：B00 / B01 / B02 三批已按规范完成并留证；B03 在两个工程各完成一部分（健康模块已落地到工作树，未接入完、未验证）后按用户指令终止。当前两个仓库已被**恢复到 B02 完成状态**（可编译、有证据），中断的 B03 工作逐字保存在 `B03/partial/`（原文件副本 + 恢复报告），未丢失、未提交。

## 1. 执行前后版本与工程身份

| 板 | 根目录 | Target | 初始HEAD | 最终状态 | 源码清单 |
| --- | --- | --- | --- | --- | --- |
| C（F407 底盘） | D:\文档\Printk\bubing\chassis\Infantry_chassis | standard_tpye_c | 83c85da3150d1092988f653ca53e40dd78006a28（分支 soft_turn） | 未提交：11 个已跟踪文件修改 + 6 个新增源文件 + 6 个新增工具；无 delete | `snapshots/optimization-2026-11-14-20261114-174009-B02-after/chassis-sha256.json` |
| G（H723 云台） | D:\文档\Printk\bubing\gimbal\Infantry_gimbal | CtrlBoard-H7_IMU | 17880289d097c03b2c26ac080311d2d7183d5d2a（分支 main） | 未提交：13 个已跟踪文件修改 + 4 个新增工具；无 delete | `snapshots/optimization-2026-11-14-20261114-174009-B02-after/gimbal-sha256.json` |

用户既有未提交修改及保护方式：C 的 `AGENTS.md`（把云台路径从旧 `cloud_platform` 改为 `gimbal\Infantry_gimbal`）自始至终**未还原、未提交**，且已进入 B00 原始证据包。未创建任何 git 提交、未切分支、未 reset/stash。

与交接基准不同的事实及证据：

| 编号 | 交接文档陈述 | 当前事实 | 证据 |
| --- | --- | --- | --- |
| F1 | G 现存日志 2026-11-07 为 0 错误/30 警告 | 当前 HEAD 重编译为 0 错误/**31 警告**（B01 时 33，B02 修掉 2 条） | `B01/build/gimbal.log`、`B02/build/gimbal.log` |
| F2 | G 的 28 字节接收缺陷 | 比文档更严重：512/28 缓冲错配已被运行时复现（错误恢复重挂 512B 而解析窗 28B）、start_index 残留导致越界、未初始化 temp 被解析 | verifier `B01/tests/verifier-head/rx-boundaries/v03-rx-results.json` |
| F3 | “detect 链空转” | 已用运行时+静态双证据确认：`toe_is_error()` 恒 false，`detect_hook()` 全树无调用者 | verifier V04/V05 |
| F4 | C 的 vx/vy/wz “缺有效实际速度赋值” | 实际是**从未赋值**（恒 0） | `interboard-protocol-consistency.md` |
| F5 | P06 温控“err=set-ref 反向” | 该缺陷只在 G 成立；C 用常规 `PID_calc(temp,target)` 接口，方向正常 | `hardware-unknowns.md` |
| F6 | 编译优化字段 | C 目标 -O1，但 45 个 HAL/RTOS/USB 文件被文件级 `Optim=0`；G 有组级 `Optim=0`+`wLevel=4`，且用 `uGnu=1/v6Lang=3` | `current-config.md`（两端） |
| F7 | VTM 布局 | 只有 `sizeof==21` 断言，无 offsetof 证据；verifier 用 ARMCC write-probe 实测 20 个字段，两端完全一致（crc 占 19..20，mouse_x/y/z=10/12/14） | `v02-frames-results.json` 的 `vtm_field_map` |

## 2. 决策与问题逐项关闭

| 问题ID | 状态 | 修改/保留理由 | 源码位置 | 批次 | 测试ID | 未完成原因 |
| --- | --- | --- | --- | --- | --- | --- |
| P01 | fixed | 28B 流式解析、64B 单环形缓冲（启动/错误重启共用）、头尾+字段+枚举校验、仅合法帧发布 | G:User/Bsp/bsp_usart.c、User/APP/remote_control.c、Core/Src/main.c | B02 | V03-P01 改前 3/14 → 改后 176/176 | 独立复跑未完成（终止） |
| P02 | fixed | HAL 成为 DMA 唯一所有者（`Src/usart.c:183` 已 CIRCULAR）、NDTR 消费指针、46B 帧流式解析、字段/枚举校验、错误恢复同缓冲 | C:bsp/boards/bsp_usart.c、application/frame_stream.*、application/rx_status.* | B02 | usart-frames 改前 10/23 → 改后 31/31 | 同上 |
| P03 | fixed | 确认实际是 18B DBUS；UART5 只接受 18/36 整块并校验通道/开关；C 侧 USART3 仅完整合法帧刷新 generation/last_rx | G:User/APP/remote_control.c；C:application/remote_control.c | B02 | 改前 4/11 → 改后 131/131（G） | C 侧“非法帧不写 rc_ctrl”留到 B03，未做 |
| P08 | fixed | USB 只由 defaultTask 初始化；ISR 只写 1024B 无锁 SPSC 环；TX 静态 512B 副本、单帧在途；VOFA BUSY 丢弃+计数；删未调用的 `Vofa_FireWater` | G:usbd_cdc_if.c、usb_cdc_port.*、comm_app.c、bsp_vofa+.* | B02 | 改前失败→18/18 | 无 |
| P11 | fixed | projects.json 指向正确云台工程；两端 build/snapshot 增加预检（预检通过前不建目录）；支持按 run-id/batch 落证据 | C/G:AI_Workspace/tools/* | B01 | 反向用例 N1/N2/N3/N4 全通过 | 无 |
| P04 | not_done | 健康模块两板均已写（C `application/health.*`、G `User/APP/health.*`），但未接完、未验证 | 见 `B03/partial/files/` | B03 | 无 | 用户终止 |
| P05 | not_done | 未开始 | — | B03 | 无 | 用户终止 |
| P06 | pending_hardware | 未开始；极性无证据，按规范只能交 pending 候选 | — | B04 | 无 | 用户终止 |
| P07 | not_done | 未开始；改前缺陷已由 verifier 固定（两轮共用 PID，wheel2 err=0 仍输出 1000→2000） | — | B04 | V08 改前 0/3 | 用户终止 |
| P09 | pending | 校准手册与缺口清单已交文档；代码侧值域检查/布局断言未做 | `calibration-manual.md`、`calibration-gaps.md` | B05 | 无 | 用户终止 |
| P10 | pending | 文档侧已核实并记录（速度字段恒 0、G 的 chassis_data 无读者、裁判 NOT_READY 恒成立、R 键双消费者）；代码/UI 未改 | 同上 | B05 | 无 | 用户终止 |
| P12 | pending | 31 条警告已逐类归档；明确可修项（隐式声明、末行换行）已在 B02 顺带清掉 2 条 | `execution-log.md` B01 表 | B05 | 无 | 用户终止 |
| P13 | not_done | 只登记候选（含 G 的 `cali_gyro_hook`、C 的 `bsp_rc.c` 注释块、`static sbus_rx_buf` 每编译单元一份等），未删 | — | B06 | 无 | 用户终止 |
| P14 | not_done | 参数集中/GBK→UTF-8 转换/命名整理未做；文档旧路径已在历史索引中标注 | `historical-index.md` | B06 | 无 | 用户终止 |
| P15 | not_done | 未开始 | — | B06 | 无 | 用户终止 |
| P16 | documented | 硬件未知表 + 范围外设计清单已交；未做 CubeMX 隔离实验 | `hardware-unknowns.md`（两端） | — | 无 | 用户终止 |

决策项 D01–D20：

| 决策 | 状态 | 说明 |
| --- | --- | --- |
| D01 | 维持 | 两仓库独立；C 工具显式定位 G，G 工具不读 C 源码 |
| D02 | 维持并验证 | 28/46 帧、VTM 19+7、CAN ID、枚举线上值全部未变；V02 双向互解 14/14 |
| D03/D04 | not_done | 超时与故障输出矩阵属 B03，未完成 |
| D05 | 部分 | 健康模块已落地未接入（`B03/partial`）；未启动旧 detect_task |
| D06–D09 | not_done | 自瞄超时、独立摩擦轮 PID、主机底盘拒绝、RC 注入解耦 |
| D10 | documented | 速度字段恒 0 已写入协议文档；代码未改 |
| D11/D12 | documented | 裁判 NOT_READY/UI 未知项已核实记录；代码未改 |
| D13 | documented | 校准手册与缺口已交；未迁移 Flash、未改机械零位 |
| D14/D15/D16 | not_done | 温控、Yaw KI 收敛、debug 语义清理 |
| D17 | fixed | USB 单所有者已实现（B02） |
| D18 | 维持 | UART7/SHOOT_DETECT/servo/LED/USB 保留，未删 |
| D19 | 维持 | 未重跑 CubeMX 生成器 |
| D20 | fixed | `historical-index.md` 建索引并标 superseded，未改任何历史正文 |

## 3. 实际行为变化

正常输入路径保持项：合法 28/46 字节帧的字段偏移、字节序、枚举线上值、VTM 19 字节布局、接收后写入的全局变量与原来一致；verifier 用两端真实收发函数做端到端互解（V02 14/14 通过）。CAN ID/槽位、PID 增益/限幅、任务周期与优先级、Flash 地址与记录格式、MIT 映射、Pitch 启动/使能/状态 5 兼容全部未动。

故障/恢复路径变化（B02 范围）：

| 变化 | 改前 | 改后 |
| --- | --- | --- |
| 无帧头/错尾数据 | 照常发布并改写状态 | 丢弃，逐字节重同步，计数 |
| 半帧（短事件） | 被当完整帧读取（越界） | 不发布、不刷时间戳 |
| NaN/Inf/枚举越界 | 照常进入业务 | 字段校验拒绝并计数 |
| 错误中断后 | 重挂缓冲与解析缓冲不一致（512/28），旧窗再发一次 | 同一缓冲，重置解析器并对齐写指针 |
| USB 异步发送 | 调用者栈缓冲可能在途被覆盖 | 静态副本 + 单帧在途 + BUSY 计数 |

参数变化表：**无**。本轮没有改任何控制参数、零偏、限幅或超时（100ms 超时属 B03，未落地）。

协议及 Flash 布局对比：**无变化**（V02 逐字节核对）。

诊断与 Watch 变量旧→新映射（B02 新增）：

| 旧 | 新 |
| --- | --- |
| 无 | `usart1_status.gimbal_yaw_rad` / `ins_yaw_rad` / `ins_pitch_rad` / `ins_roll_rad` / `gimbal_behaviour` / `gimbal_shoot_mode` / `vtm` |
| 无 | `gimbal_status_t.frames_ok/frames_rejected/bad_header_bytes/bad_tail/stream_resets/last_rx_ms/generation/seen` |
| 无 | `rc_status_t.frames_ok/frames_rejected/short_frames/bad_channels/bad_switch/last_rx_ms/generation/seen`（C） |
| `gimbal_memcpy_data_from_chassis`（G） | `chassis_frame_parse()` + `chassis_rx_feed_stream()` |
| 无 | `bsp_usart_rx_start()`（G，USART1 RX 唯一启动入口） |

## 4. 构建结果

| 板/配置 | 完整命令 | errors/warnings | Code/RO/RW/ZI | AXF SHA256 | 日志/MAP |
| --- | --- | --- | --- | --- | --- |
| C B01 基线 | `pwsh -File AI_Workspace/tools/build.ps1 -Board both -RunId 20261114-174009 -Batch B01` | 0 / 0 | 81216 / 4644 / 976 / 59544 | CE0438F5…6366 | `C:artifacts/optimization-2026-11-14/20261114-174009/B01/build/chassis.log` |
| G B01 基线 | 同上（同一命令的 gimbal 分支） | 0 / 33 | 112860 / 2132 / 1132 / 56628 | C6DE7AAA…7B8C | 同上 `gimbal.log` |
| C B02 终态 | `build.ps1 -Board chassis -Batch B02-final-rebuild` | 0 / 0 | 82504 / 4644 / 980 / 59700 | CB22F545…38F0 | `.../B02-final-rebuild/build/chassis.log` |
| G B02 终态 | `build.ps1 -Batch B02-final-rebuild` | 0 / 31 | 114684 / 2132 / 1124 / 55356 | A408BE06…D2D7 | `.../B02-final-rebuild/build/gimbal.log` |

注意：AXF 内含构建时间戳，**同一份源码重复构建的 SHA256 不同**（本轮同一 B02 源出现过 3 个不同哈希）。哈希只用于标识"本次产物"，不能用于证明源码一致；源码一致性请用 `*-sha256.json` 清单。

G 警告变化：33 → 31（清掉 `usb_cdc_port` 末行换行与 `comm_app` 隐式声明），**无新增类别**。剩余 31 条构成：24× `__packed__ does not apply`（calibrate_task.h:175）、2× 末行无换行、1× 返回类型不匹配、3× 声明未引用、1× 动态初始化在不可达代码、2×（B02 已清）、其余为同类。这些属 P12，未处理。

## 5. 验证结果

独立验证者（verifier）在 `git archive HEAD` 冻结副本上建立了"修改前"基线，不受执行者并行改动影响：

| 测试ID | 测试对象 | 修改前复现 | 修改后结果 | 原始输出 | 替身/证据边界 |
| --- | --- | --- | --- | --- | --- |
| V02 | 28/46 帧序列化与双向互解、VTM 实测布局 | 14/14 通过（契约本来就对） | 契约冻结，未变 | `B01/tests/verifier-head/interboard-frames/` | ARMCC 5.06 编译真实收发函数 + Unicorn；无外设/RTOS |
| V03-P01 | G 28B 接收 | 3/14 通过，11 条失败 | 执行者自测 176/176（独立复跑未做） | `.../rx-boundaries/`、`G:B02/tests/rx-chain/` | 同上，DMA 写入由测试执行 |
| V03-P02 | C 46B 接收 | 5/12 通过 | 执行者自测 31/31（独立复跑未做） | `C:B02/tests/usart-frames-{before,after}/` | 同上 |
| V03-P03 | 遥控帧 | 4/11 通过 | 执行者自测 131/131（G）/待补（C） | `G:B02/tests/before/`、`rx-chain/` | 同上 |
| V04–V08 | 健康/超时/仲裁/两轮 PID | 4/17 通过，13 条失败（含"RC 失效仍下发 yaw=1234"、"两轮共用 PID"、"debug 无超时"） | not_run（B03 未做） | `.../B01/tests/verifier-head/`、`validation-matrix.csv` | 真实函数 + 替身 I/O |
| V13 | soft-turn 回归 | — | 260543 checks 通过（C，B02 后） | `C:B02/tests/soft-turn/` | CAN/UART/RTOS 替身 |
| V01 | 全量构建 | — | 见第 4 节 | 构建日志 | 完整固件 |

明确证据等级：V01 为完整固件构建；V02/V03/V04–V08 为 ARMCC 编译真实工程函数后在 Unicorn 中执行（板级 I/O、USART/DMA 寄存器、RTOS 为替身）；`validate_invariants` 为 ARMCC 预处理函数对比；`soft-turn` 为真实控制源码 + I/O 替身。**不存在实机或实板证据**。

设备测试（Hxx）全部 not_run：安全支撑、接线、RC 失联、CAN 断开、板间断开、Flash 保存/掉电、温控极性/升温、四轮方向/零偏、Yaw 保持、Pitch 端点/下限回收、射击空载输出、任务周期/栈/堆、USB 重连。

## 6. 删除及重构清单

| 文件/符号 | 动作 | 调用/Target/MAP证据 | 主机/Flash/协议消费者 | 恢复来源 |
| --- | --- | --- | --- | --- |
| G `Vofa_FireWater` | 删除 | 未调用，且会写 USART1（协议链路） | 无 | B02-before 快照 |
| G `__io_putchar` 重定向 | 删除 | 会把 printf 混入 CDC 流 | 无 | 同上 |
| C `usart1_init` 内 DBM/M0AR/M1AR/NDTR 手写块与 RXNE 使能 | 删除 | 与 HAL_UART_Receive_DMA 双所有者、RXNE 抢 DR | 无 | 同上 |
| C/G `AI_Workspace/tools/*` | 新增 | 证据链工具 | 无 | 不适用 |

未做任何"僵尸代码"批量清理（P13/B06 未开始），也未删除 HAL/CMSIS 供应商代码。

## 7. 当前文档与工具入口

文档（C 与 G 各自 `AI_Workspace/reports/optimization-2026-11-14/`，同名文件内容按各自仓库事实）：

| 文件 | 用途 |
| --- | --- |
| current-config.md | B00 配置清单：Target/Define/IncludePath/文件级覆盖/DMA/NVIC/回调/任务栈 |
| interboard-protocol.md + interboard-protocol-consistency.md | 28/46 帧契约；一致性核查结论（字节级 0 条不一致，语义/校验 4 条） |
| controls-and-keys.md | 按键/摇杆矩阵，含 R 键双消费者、X/C/Shift 与 servo 复用 |
| calibration-manual.md + calibration-gaps.md | 现车校准手册 + 代码侧缺口 |
| hardware-unknowns.md | 硬件未知项（源码事实/需设备/需用户决定）+ 范围外设计 |
| historical-index.md | 历史报告索引与 superseded 标注 |
| execution-log.md / decision-log.md / issues.csv / validation-matrix.csv | 过程、判断、问题、验证矩阵 |
| final-report.md | 本文件 |

命令（任意 cwd 可用）：

```powershell
# 双板构建（C 编排层，含预检）
& 'C:\Program Files\PowerShell\7\pwsh.exe' -NoLogo -NoProfile -File <C>\AI_Workspace\tools\build.ps1 -Board both -RunId <id> -Batch <batch>
# 单板构建
& 'C:\Program Files\PowerShell\7\pwsh.exe' -NoLogo -NoProfile -File <C>\AI_Workspace\tools\build.ps1 -Board chassis -RunId <id> -Batch <batch>
& 'C:\Program Files\PowerShell\7\pwsh.exe' -NoLogo -NoProfile -File <G>\AI_Workspace\tools\build.ps1 -RunId <id> -Batch <batch>
# 快照 / 校验
& 'C:\Program Files\Python39\python.exe' -B <repo>\AI_Workspace\tools\snapshot.py capture <snapshots\名字>
& 'C:\Program Files\Python39\python.exe' -B <repo>\AI_Workspace\tools\snapshot.py verify <snapshots\名字> --result <输出.json>
# 原始证据保全（不依赖 projects.json）
& 'C:\Program Files\Python39\python.exe' -B <repo>\AI_Workspace\tools\capture_evidence.py --repo <repo> --out <dir> --label C --run-id <id>
# 按原编码改文件（GBK 必用）
& 'C:\Program Files\Python39\python.exe' -B <repo>\AI_Workspace\tools\safe_edit.py detect <相对路径>
# 已有离线验证
& 'C:\Program Files\Python39\python.exe' -B <C>\AI_Workspace\tools\validate_soft_turn.py --cache ... --results ...
& 'C:\Program Files\Python39\python.exe' -B <C>\AI_Workspace\tools\validate_usart_frames.py --cache ... --results ...
& 'C:\Program Files\Python39\python.exe' -B <G>\AI_Workspace\tools\validate_rx_chain.py --cache ... --results ...
# 独立验证
& 'C:\Program Files\Python39\python.exe' -B <C>\AI_Workspace\tools\check_interboard_frames.py --chassis-root ... --gimbal-root ... --cache ... --results ...
& 'C:\Program Files\Python39\python.exe' -B <C>\AI_Workspace\tools\check_rx_boundaries.py ... / check_rx_b02.py / check_health_timeouts.py / check_parse_drift.py
```

## 8. 恢复与可追溯性

| 批次 | C 包 | G 包 | 完整性 | diff 与产物 |
| --- | --- | --- | --- | --- |
| B00 | `snapshots/optimization-2026-11-14-20261114-174009-B00-before/`（312 文件，zip sha `e3812e2f…7977`） | 同名（1268 文件，zip sha `ee4856fa…517d`） | zip 自检 + 逐条哈希 OK | `B00/p11-tool-path-evidence.md`、capture.json、git-diff.patch、firmware/ |
| B01 | `-B01-after/`（双板 312+1268） | `-B01-after/` | verify integrity=OK, changed=[] | `B01/changes-{C,G}-tools.diff`、build/、tests/tool-preflight-negative.md |
| B02 | `-B02-before/`、`-B02-after/` | `-B02-before/`、`-B02-after/`、`-B02-after-intermediate/` | 本板条目校验 OK；跨板差异已说明 | `B02/changes.diff`、`B02/build/`、`B02/tests/`、`B02/comparisons/` |
| B03（中断） | `B03/partial/`（restore-report.json + files/ 逐字副本） | `-B03-before/` 快照 + `B03/partial/` | 未做 | 无构建/测试 |

终止时的恢复操作（已执行，可复核）：把两个仓库中**自 B02 完成以来**被改动的源文件（C 8 个、G 3 个）与新增文件（两端 `health.c/.h`）逐字保存到 `B03/partial/files/` 后，从 B02 快照恢复这些文件、删除本轮新增的 `health.c/.h`，再全量重建确认 0 error。**未做整仓库覆盖、未 reset、未丢弃任何用户改动**。恢复后 git status 中无 delete 条目。

## 9. 剩余风险与需要决策

| ID | 当前证据 | 未解决原因 | 推荐决定 | 是否阻止部署 |
| --- | --- | --- | --- | --- |
| R1 | P04/P05/P06/P07 未实现 | 用户终止 | 恢复执行后从 B03 继续，`B03/partial/files/` 里有已写一半的 health 模块可复用 | 是（保护缺失） |
| R2 | IMU 加热极性无任何电路/设备证据（G TIM3/PB1、C TIM10/PF6） | 需设备 | 保持 pending_hardware，不得激活反向控制 | 是 |
| R3 | HSE 元数据与 HSE_VALUE 冲突（C: 25M vs 12M；G: 12M vs 24M） | 需实测晶振/时钟 | 记录为待核对，不改 PLL/HSE | 否，但影响"时序已实测"类结论 |
| R4 | CAN 物理拓扑/终端电阻未知 | 需设备 | 在线判定已加总线/帧型/DLC 校验，但物理层仍需验证 | 否 |
| R5 | G 的 24 条 `__packed__` 告警 | 需真实布局证据 | 用 sizeof/offsetof 证据固定后再消除，不靠关诊断 | 否 |
| R6 | C 无 Flash 写调用者、G `cali_data_write` 固定写 108B 会清 36..107 | 代码 | B05 处理；在校准手册中已标"不能安全重标" | 是（无法重标定） |
| R7 | R 键同沿双消费者（弹舱 + 自瞄） | 行为现状 | 保留，仅文档标注（已裁决非缺陷） | 否 |
| R8 | B02 的独立复跑未完成（verifier 只做了改前基线） | 终止 | 恢复执行第一步：复跑 V02/V03 + 正常路径逐字段比对 | 是（B02 定级为 fixed 但缺独立改后证据） |

## 10. 实体设备验收计划

全部 not_run，前提与判据见两端 `hardware-unknowns.md`。授权边界：烧录/上电/驱动电机必须用户另行明确授权与现场条件。分组：加热极性与升温（低/高 PWM 占空比、到温退出、断传感器零输出）、机械边界（Pitch 两点采样与下限外回收、Yaw 保持）、RC/CAN/板间失联（拔线后八电机与射击零输出、恢复需新有效帧）、掉电校准（Flash 写后回读）、周期/栈/堆（实测 dt 与 missed deadline）、USB 重连。

## 11. 给决策者的结论

已完成的软件项：P11（工具定位与预检，含反向用例）、P01/P02/P03/P08（接收正确性与 USB 所有权，改前失败→改后通过，自测通过）、P16/D20 文档侧交付、现状文档五件套（配置/协议/按键/校准/硬件未知）。

未完成的必做项：P04、P05、P06、P07、P09 代码侧、P10 代码侧、P12、P13、P14、P15；B03–B07 全部阶段门未过。

待设备项：温控极性、HSE 实际值、CAN 拓扑/终端、Flash 实机读写、RC/CAN/板间失联、机械边界、周期/栈/堆。

范围外后续设计：协议 v2+CRC、速度里程计、完整裁判转发、主机底盘运动通道、Flash 双槽、Yaw 新校准、Pitch 机械零位与投影、RTOS/工具链迁移（见 `hardware-unknowns.md`）。

建议下一步：①先让 verifier 复跑 B02 的改后用例与正常路径逐字段比对，把 B02 从"执行者自测"升到"独立验证"；②用 `B03/partial/files/` 里的 health 模块继续 B03（C 与 G 各一套，接口已定：`seen/last_valid_ms/generation/fresh/reason` + 集中 100ms 超时）；③在拿到设备条件前不要宣称整车可发布。
