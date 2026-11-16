# 执行日志：整车工程优化（F407 底盘 + H723 云台）

依据：整车工程优化交接与执行规范-2026-11-14.md，版本 1.0
run-id：20261114-174009
执行者：本会话执行 agent（Lead + 并行执行/验证 agent）
时区：Asia/Shanghai
性质：append-only。历史条目只追加，不回改；需要更正时另起条目并标注"更正"。

## 环境事实（开工时核实）

| 项 | 值 |
| --- | --- |
| C 根 | D:\文档\Printk\bubing\chassis\Infantry_chassis |
| C HEAD / 分支 | 83c85da3150d1092988f653ca53e40dd78006a28 / soft_turn |
| C git status | M AGENTS.md（用户改云台路径）、?? 交接文档、?? 本轮新增工具 |
| G 根 | D:\文档\Printk\bubing\gimbal\Infantry_gimbal |
| G HEAD / 分支 | 17880289d097c03b2c26ac080311d2d7183d5d2a / main |
| G git status | 干净（本轮仅新增工具文件，未提交） |
| Keil | D:\keil5\UV4\UV4.exe 存在；ARMCC 5.06 update 7 |
| Python | C:\Program Files\Python39\python.exe（3.9.13） |
| 验证依赖 | unicorn 2.1.4 + pyelftools 0.32，vendored 在两端 cache/dependencies/packages |
| 主机编译器 | 无 gcc/clang/cl；离线执行只有 ARMCC + Unicorn 一条路 |

## B00：状态核对与原始证据保全（2026-11-14 17:40–17:45）

做了什么：

1. 复核两端 Git 状态，与交接文档 1 节一致：C 只有 AGENTS.md 既有未提交修改（内容为把云台路径从旧 cloud_platform 改为 gimbal\Infantry_gimbal），G 干净。用户改动原样保留，未还原、未提交。
2. 新建 `AI_Workspace/tools/capture_evidence.py`（独立于 projects.json 的原始保全工具），对两端各生成一份完整包：
   - C：`snapshots/optimization-2026-11-14-20261114-174009-B00-before/`，312 文件 / 23.8 MB，sources.zip SHA256 `e3812e2f3b0536f95c27464abb7f0eaa64c376e96997b1be42570375226e7977`
   - G：`snapshots/optimization-2026-11-14-20261114-174009-B00-before/`，1268 文件 / 120.0 MB，sources.zip SHA256 `ee4856faa77c005099f1463445b9c46733e790a7d87b1c7ae699f8125e08517d`
   - 每包含：源码/工程/脚本配置原始字节 + 逐文件 SHA256 + git HEAD/status/diff/stash + 现存 AXF/HEX/MAP/构建日志副本 + 历史区域索引。
3. 验包：两端 zip `testzip()` 无损坏，逐条目哈希与 source-sha256.json 全部一致（missing=0 extra=0 bad=0）。C 现存固件 `standard_tpye_c.axf`（2026-10-30 22:01）、G 现存固件 `CtrlBoard-H7_IMU.axf`（2026-11-07 22:10）已单独复制并记哈希。
4. 记录与交接文档的差异：G 当前 HEAD（2026-11-12）晚于文档引用的 2026-11-07 构建，重编译实测 33 warnings（文档记 30），逐条分类见 B01。

说明：G 的 B00 包内含本轮新增的 `capture_evidence.py`（打包时工具已落地），C 的 B00 包同。该文件为工具，不是固件源码，已在 git-status.txt 中标注为 untracked。

## B01：工具定位修复 + 双端编译基线（2026-11-14 17:43–17:52）

### 修改前复现（P11）

| 复现项 | 命令 | 结果 |
| --- | --- | --- |
| 旧 gimbal root 解析 | 读 projects.json | `D:\cloud_platform\H7_new\H7_new\DM-balanceV1`，`exists=False` |
| snapshot.py 双板捕获 | `snapshot.py capture <cache>/p11-repro-snapshot` | FileNotFoundError，exit 1 |
| build.ps1 -Board gimbal | `build.ps1 -Board gimbal -LogDirectory <cache>/p11-repro-build` | ReadAllText 抛错，exit 1，**且留下空结果目录** |

详细记录：`artifacts/optimization-2026-11-14/20261114-174009/B00/p11-tool-path-evidence.md`

### 修改内容（仅工具，未改固件源码）

| 文件 | 改动 |
| --- | --- |
| C `AI_Workspace/tools/projects.json` | gimbal root → `../../gimbal/Infantry_gimbal` |
| C `AI_Workspace/tools/build.ps1` | 先全量预检（root/project/Target/全部启用 Keil 条目）再建目录；新增 -RunId/-Batch，日志落 artifacts/optimization-2026-11-14/<run>/<batch>/build/ |
| C `AI_Workspace/tools/snapshot.py` | 新增 preflight()，通过后才 mkdir |
| G `AI_Workspace/tools/build.ps1` | 同上（单板版） |
| G `AI_Workspace/tools/snapshot.py` | 同上；补 CONFIG 导入 |
| 两端 `AI_Workspace/tools/capture_evidence.py` | 新增（B00 原始保全） |
| 两端 `AI_Workspace/tools/safe_edit.py` | 新增（按原编码改文件，防 GBK 汉字损坏） |
| C `AI_Workspace/cache/dependencies/packages` | 从 G 的 vendored 依赖复制 unicorn/pyelftools（本地缓存，未入库） |

原始字节保全：修改前的 projects.json、build.ps1、snapshot.py 均在 B00 sources.zip 内可逐字节复原。

### 构建基线（修复后第一次全量编译）

| 板 | Target | errors | warnings | Code/RO/RW/ZI | AXF SHA256 |
| --- | --- | ---: | ---: | --- | --- |
| C | standard_tpye_c | 0 | 0 | 81216 / 4644 / 976 / 59544 | CE0438F5120F72E55B682B80B9F79318E2D84D9F6D8A8E92F5B4C8726AF06366 |
| G | CtrlBoard-H7_IMU | 0 | 33 | 112860 / 2132 / 1132 / 56628 | C6DE7AAADBA6D91BB36467A7D14D8B28941FB73701CE129382E94387A76C7B8C |

日志目录：`artifacts/optimization-2026-11-14/20261114-174009/B01/build/{chassis,gimbal}.log` + results.json。

G 的 33 条警告分类（B01 时点真值，覆盖文档中的历史 30；B02 后降到 31）：

| 类别 | 条数 | 代表位置 | 归属批次 |
| --- | ---: | --- | --- |
| `#3108-D __packed__ does not apply` | 24 | User/APP/calibrate_task.h(175) | B05 / P12 |
| `#1-D last line ends without a newline` | 2 | User/APP/INS_task.c(402) | B05 / P12 |
| `#120-D return type mismatch` | 1 | User/APP/INS_task.c(401) | B05 / P12 |
| `#177-D declared but never referenced` | 3 | calibrate_task.c(836) cali_gyro_hook；556/557 GYRO_FLAG/CHASSIS_FLAG | B05/B06 |
| `#185-D dynamic init in unreachable code` | 1 | Communication/channel/camera/camera_channel.c(112) | B05 / P12 |
| `#223-D declared implicitly` | 2 | stm32h7xx_it.c(488) comm_camera_trigger_pulse；comm_app.c(300) usb_cdc_port_bind_uproto | B02/B05 |

### B01 退出条件核对

- 两个正确 Target 完整构建：是（上表）。
- 工具从任意 cwd 定位一致：是（Lead 在 default-workspace 下调用 C 的 validate_runtime.py 与 build.ps1，均定位到 C 根）。
- 新鲜 AXF 与完整日志：是（build.ps1 用 mtime + 0 Error(s) 双重判定）。
- 快照完整：`snapshots/optimization-2026-11-14-20261114-174009-B01-after/` 已生成并 verify 通过（changed=[]，integrity=OK）。

## B02：接收正确性（C 侧完成，G 侧进行中）

开始：2026-11-14 17:53 · 证据：`artifacts/optimization-2026-11-14/20261114-174009/B02/`

### C 侧（P02 + P03 的 C 条款）

改动文件：

| 文件 | 动作 |
| --- | --- |
| application/frame_stream.c/.h | 新增：定长帧流式重组（逐字节重同步、按字节保留可能的新帧头、帧尾不符时回退） |
| application/rx_status.c/.h | 新增：接收状态发布/快照（frames_ok/rejected/bad_header/bad_tail/resets/generation/seen，任务侧短临界区整体拷贝） |
| bsp/boards/bsp_usart.c/.h | 接收路径重写：循环 DMA + NDTR 消费指针 + 帧流解析 + 字段/枚举校验；错误恢复用同一缓冲并对齐写指针；删掉手写 DBM/M0AR/M1AR/NDTR 与 RXNE 中断 |
| application/remote_control.c | USART3 遥控：只有完整合法帧才刷新 generation/last_rx |
| MDK-ARM/standard_tpye_c.uvprojx | applications 组新增 2 个文件（条目 113→115） |
| AI_Workspace/tools/{test_usart_frames.c,validate_usart_frames.py} | 新增针对本批的验证入口 |

DMA 所有权定案：`HAL_UART_Receive_DMA` 唯一配置入口，`Src/usart.c:183` 已是 `DMA_CIRCULAR`；`USART_BUF_LENGHT` 90→96（原半缓冲 45 < 46 字节帧，放不下整帧）。已核对：这不是"改了 HAL 配置"，而是让 HAL 成为唯一所有者。

构建：0 errors / 0 warnings；Code 81216→82504，RO 4644，RW 976→980，ZI 59544→59700；AXF SHA256 `6FE796A265AA3A57A7E3272B075E6A6BD01EC83FDD1780285EE62DC88B9A4278`。

测试（真实 `bsp_usart.c`/`remote_control.c` 经 ARMCC 编译、Unicorn 执行，仅板级 I/O 替身，同一测试源跑前后两版）：

| 用例组 | 修改前 | 修改后 |
| --- | --- | --- |
| usart-frames（23 必测项：前缀/错尾/噪声内假头/粘包/环绕/逐字节/NaN/越界/枚举/遥控边界/计数与时间戳/错误恢复） | 10 通过、13 失败 | 23→扩到 31 项全部通过 |

回归：`validate_soft_turn.py` 260543 checks 通过；`validate_invariants.py --board chassis` 342 函数中 7 处差异全部为本批声明修改。

快照：`snapshots/optimization-2026-11-14-20261114-174009-B02-before/`、`-B02-after/`；after 的 C 侧归档校验通过（跨板 verify 会因 G 同批改动报差异，已在 B02/notes.md 说明）。

待决与后续：见下方"B02 决策点"。

### G 侧（P01 + P03 + P08）

改动文件（11 个）：

| 文件 | 动作 |
| --- | --- |
| User/Bsp/bsp_usart.c/.h | 重写 28 字节流式解包；USART1 RX 改为 64B 单一静态环形缓冲，启动与错误重启共用（修掉 512/28 错配）；ISR 按 HAL 位置增量消费；`bsp_usart_rx_start()` 成为唯一启动入口 |
| User/APP/remote_control.c/.h | UART5 36B normal DMA，只接受 18/36 整块并校验通道/开关，半帧不发布不刷时间戳 |
| Core/Src/main.c | RX 启动收敛到唯一入口 |
| USB_DEVICE/App/usbd_cdc_if.c、usb_cdc_port.c/.h、comm_app.c | USB ISR 只写 1024B 无锁 SPSC 环，任务侧 pump 解析；TX 用静态 512B 副本、单帧在途；USB 只由 defaultTask 初始化 |
| User/Bsp/bsp_vofa+.c/.h | BUSY 丢弃 + 计数；删除会写 USART1 的未调用 `Vofa_FireWater` |
| AI_Workspace/tools/* | validate_runtime.py 适配新解析入口；新增 validate_rx_chain.py、validate_b02_before.py |

构建：0 errors / 31 warnings（基线 33，减少 2 条：usb_cdc_port 末行换行、comm_app 隐式声明；无新增类别）。Code=114684 RO=2132 RW=1124 ZI=55356。AXF SHA256 `6FAA7334441CA8CE018C0B1EF45325822EA051B93AA67C8AA653C663F44AB6FD`。

测试（同一测试源跑改前/改后，真实工程函数经 ARMCC + Unicorn 执行）：

| 用例组 | 修改前 | 修改后 |
| --- | --- | --- |
| B02-01 28 字节解析（512/28 错配、start_index 残留、逐字节、粘包、环绕、NaN、错尾） | 第 2 项失败（无帧头也改写状态） | 176/176 |
| B02-02 遥控（开关 0、通道越界、半帧、计时回绕） | 第 2 项失败（开关 0 照常发布并刷时间戳） | 131/131 |
| B02-03 USB 发送/收包交接 | 第 3 项失败（异步读到被覆盖的调用者缓冲） | 18/18 |

回归：runtime 27、pitch_mit 55、pitch_target 18、pitch_enable 21、pitch_cold_start 10、locality 全部通过。

快照：`snapshots/...-B02-before/`（17:55:34，早于首次编辑）与 `...-B02-after/`；before 的 verify 显示 changed 恰为本次 11 文件，完整性 OK。

### B02 决策点

| 编号 | 事项 | 判定 |
| --- | --- | --- |
| B02-D1 | 无 CRC，全 0 字段帧仍判合法（C 启动早期真实输出） | 接受。文档已承认无 CRC，字段范围无法区分"合法的全 0"；不改判定规则 |
| B02-D2 | G 的 UART5 采用 IDLE 整块策略，帧中停顿丢整块（有计数），不做滑动重同步 | 接受。文档允许"明确的重组或明确丢弃策略"；已要求验证"丢弃有计数、旧数据不冒充新帧" |
| B02-D3 | C 侧 `validate_runtime.py` 依赖 G 的解析函数名 | 由 C 侧按 G 新符号适配（`chassis_frame_parse`/`chassis_rx_feed_stream`），B03 不再改名；禁止删断言 |
| B02-D4 | 跨板 verify 会报对方板差异 | 每批 after 包只对**本板**条目判定完整性，跨板差异在 notes 说明，不视为失败 |

## 终止与状态固化（2026-11-14 19:12–19:18）

用户要求提前终止任务并交付交接文件。处理：

1. 定格：B00 / B01 / B02 三批为"已完成"，B03 为"中断未验证"。
2. 保全中断工作：把两个仓库自 B02 完成以来改动的源文件（C 8 个、G 3 个）与新增的 `health.c/.h` 逐字复制到
   `artifacts/optimization-2026-11-14/20261114-174009/B03/partial/files/`，并生成 `restore-report.json`（记录每个文件的来源与恢复源快照）。
3. 恢复到"B02 完成状态"：C 从 `snapshots/...-B02-after/`、G 从 `snapshots/...-B03-before/`（18:13:19，早于 G 的首次 B03 编辑）逐文件恢复，删除本轮新增的 `health.c/.h`。
   未做整仓库覆盖、未 git reset、未丢弃用户改动；恢复后两端 git status 无 delete 条目。
4. 复核构建（`-Batch B02-final-rebuild`）：C 0 errors / 0 warnings，Code 82504，AXF `CB22F545…38F0`；G 0 errors / 31 warnings，Code 114684，AXF `A408BE06…D2D7`。两端均 `Success: true`。
5. 已交付：本报告的 `final-report.md`、五件套文档（配置/协议/按键/校准/硬件未知）、历史索引、校准缺口、执行日志、决策日志、问题表、验证矩阵。

一次已修正的操作失误（如实记录）：恢复脚本在判定"快照外新增文件"时把 `AI_Workspace/` 与 `.gitignore` 误判为新增并删除（因为 snapshot.py 本身跳过 AI_Workspace）。删除前脚本已把待删文件复制到 `B03/partial/files/`，随即从该副本完整还原 25 个（C）/ 20 个（G）文件，`git status` 复核无 delete、工具与配置齐全、构建通过。后续同类操作应显式排除 `AI_Workspace/`。

## 后续接手要点（给下一个执行者）

- 从 B03 继续：C 的 `application/health.*` 与 G 的 `User/APP/health.*` 已在 `B03/partial/files/` 里写好一半，可直接复用；接口与超时宏约定见该批 notes 与 docs。
- B02 的"改后独立验证"还没做：verifier 只完成了改前基线（68 例 30 pass / 38 fail）。接手第一步应是复跑 V02/V03 + 正常路径逐字段比对。
- 设备项一律未做（Hxx 全 not_run），不得据此宣称可发布。


## 2026-11-15 B02 收尾签收

用户授权本次执行 B02 收尾。先保存双板 before 与旧报告；复跑并修复测试夹具。补修 C USART1 HT/TC 连续接收、G TC/IDLE 重放与延迟 HT、VOFA 长度回绕、USB 在途覆盖和超限截断。最终交叉接收 35/35、golden 14/14、合法帧对照 10/10；C 串口 31/31、软转向 260547；G 接收 178/178、RC 131/131、USB 19/19、TX 23/23、VOFA 1152/1152。两端正式全量构建零错误，C 0 / G 31 既有警告。详见 B02-收尾签收报告.md 及两端 artifacts/B02-acceptance/20261114-192801/。

修复了 issues.csv 中 C+P0/C+P1/C+P2 导致的错列，B02 状态提升为 verified_software；保留其他批次状态。旧 validation-matrix 的结果保留，另加 B02收尾列。纠正“B03无测试”的笼统表述：归档有失败904尝试；并非 B03 验收通过。后续不应直接回灌 partial。B03 尚未开始，本次没有设备操作。


## 2026-11-15 B03 两步执行与软件签收

持续100ms健康+最终输出门先落地并封存阶段一；随后完成自瞄2000ms清历史、主机独立仲裁120ms/单发20ms及非法RC发布收紧。最终 C 0错误0告警，G debug/release 0错误31既有告警；真实源离线回归全通过，详见 [B03-两步实施与签收报告.md](B03-两步实施与签收报告.md) 和 A/final/acceptance-evidence.json。P04/P05为verified_software，硬件not_run；B04未开始。


## 2026-11-15 B05实施与自验完成

P09/P10/P12软件阶段完成：校准值域与兼容布局、Flash边界/回读、UI/主机有效性和31条告警修复；正式双板及G release各0错误0告警，相关回归通过。校准等草稿已按最终实现重写。没有设备操作，不推进B04/B06/B07；详见[B05-实施与软件签收报告.md](B05-实施与软件签收报告.md)及A/final/acceptance-evidence.json。


B05封存补记：Keil退出后将已有health/host条目同步进uvoptx的IDE文件列表，正式uvprojx和固件未变。原after候选保留，最终包改为B05-20261115-175509-after-verified；最终文档另封acceptance-documents-final.zip，逐项重验。


## 2026-11-16 B04/B06/B07继续实施

用户已明确授权继续三个批次，覆盖此前B05后等待确认的约束。B04：独立摩擦轮PID及停机清理落地，温控仅交经测试隔离候选、P06仍pending_hardware。B06：依赖闭合清理、参数/编码/说明整理、Pitch控制与校准模块拆分、毫秒级观察入口，冻结控制参数/协议/存储布局。B07：总核、全量正式/隔离release构建与完整回归、恢复包、后续设计和设备清单。细节及最终签收证据见三份批次报告。

不升级协议/RTOS/HAL，不激活未知极性温控，不修改小陀螺算法，不烧录或操作实体设备。PID积分非对称性新增为后续评估项；软件阶段交付不等于整车发布。
