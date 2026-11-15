# B02 执行记录（P01 / P03 / P08）

日期：2026-11-14（Asia/Shanghai）
run-id：20261114-174009
基线：HEAD 17880289d097c03b2c26ac080311d2d7183d5d2a；B01 构建 0 errors / 33 warnings，Code=112860 RO=2132 RW=1132 ZI=56628，AXF SHA256=C6DE7AAADBA6D91BB36467A7D14D8B28941FB73701CE129382E94387A76C7B8C。

## 1. 改动文件

| 文件 | 改动 | 问题项 |
| --- | --- | --- |
| `User/Bsp/bsp_usart.c` | 重写 USART1 接收：64B RX DMA 环形、流式 28B 解包、逐字节重同步、序号发布；46B TX 内部副本+BUSY 丢弃；新增统计/注入 API | P01/P08 |
| `User/Bsp/bsp_usart.h` | 接收/发送统计结构、帧头尾/限值常量、`bsp_usart_rx_start`、`chassis_rx_feed_stream/reset`、`chassis_data_read` | P01 |
| `User/APP/remote_control.c` | UART5 整块策略（18/36）、通道/开关校验后才发布、序号快照、统计 | P03 |
| `User/APP/remote_control.h` | 校验门限、`rc_rx_stats_t`、generation、快照/统计 API | P03 |
| `USB_DEVICE/App/usbd_cdc_if.c` | RX 只入环；TX 增加 pClassData/长度守卫；删除 `__io_putchar` 单字节 USB 打印 | P08 |
| `User/Communication/example/device/usb_cdc_port.c/.h` | ISR→任务有界 SPSC 环、任务侧 pump、TX 静态副本+BUSY 计数、统计 | P08 |
| `User/Communication/example/device/comm_app.c` | 删除重复 `MX_USB_DEVICE_Init`、等待枚举、循环内 pump、补 include 修复隐式声明 | P08/P12 |
| `User/Bsp/bsp_vofa+.c/.h` | 调试输出 BUSY 丢弃+计数；删除会写 USART1 的未调用 `Vofa_FireWater` | P08 |
| `Core/Src/main.c` | USART1/UART5 接收启动收敛到 `bsp_usart_rx_start()`（单一所有者） | P01/P03 |
| `AI_Workspace/tools/validate_runtime.py` | 接收侧改为真实 `chassis_frame_parse`，断言数不变（27 checks） | 工具适配 |
| `AI_Workspace/tools/validate_rx_chain.py`（新） | B02-01/02/03 的 ARMCC+Unicorn 用例 | 验证 |
| `AI_Workspace/tools/validate_b02_before.py`（新） | 从 before 快照抽旧函数，复现修改前失败 | 验证 |

未改：28/46 帧格式、CAN ID/槽位、控制参数、任务周期/优先级、Flash 布局。

## 2. 快照与完整性

- B02-before：`AI_Workspace/snapshots/optimization-2026-11-14-20261114-174009-B02-before`，捕获时间 2026-11-14 17:55:34，早于第一次源码编辑（17:56 之后）。
  完整性校验 `B02/snapshot-before-verify.json`：`snapshot_integrity=OK`，changed 列表恰为本次 B02 的 11 个源码文件，证明该包是干净的改前状态。**不是**用改后源码补拍的包。
  结论：Lead 提到的“B02-before 未落地”不成立；B01-after 与 B02-before 内容等价（同一 HEAD、无源码差异），两个包均可作为 before。
- B02-after：`AI_Workspace/snapshots/optimization-2026-11-14-20261114-174009-B02-after`（最终源，含 feed_stream API）。
- 中间包 `...-B02-after-intermediate` 为加了 feed_stream API 之前的状态，仅本地保留，不用于交付。

## 3. 构建

| 尝试 | 结果 | 说明 |
| --- | --- | --- |
| attempt-1 | 6 errors | `usb_cdc_port.h` 被误截断（缺 `#endif`），日志与目录保留为 `build-attempt-1-header-error/` |
| attempt-2 | 0E/31W | 屏障前中间状态，`build-attempt-2-pre-barrier/` |
| attempt-3 | 0E/31W | feed_stream 前中间状态，`build-attempt-3-pre-feedstream/` |
| 最终 | **0 errors / 31 warnings** | `build/`，Code=114684 RO=2132 RW=1124 ZI=55356；AXF SHA256=`6FAA7334441CA8CE018C0B1EF45325822EA051B93AA67C8AA653C663F44AB6FD` |

警告逐类对比见 `comparisons/rx-contract.md` §5：减少 2 条（usb_cdc_port 末行换行、comm_app 隐式声明），保留 31 条均为基线既有，无新增。

## 4. 针对性验证（真实函数体 + ARMCC + Unicorn）

| 用例组 | 内容 | 修改前 | 修改后 | 证据 |
| --- | --- | --- | --- | --- |
| B02-01 chassis_rx | 176 checks：完整帧字段、0..27 短包、起点 0..27、无头、载荷内帧头重同步、错尾、逐字节、两帧/三帧粘包、环形环绕、噪声、NaN/Inf、枚举越界、错误恢复、512/28 错配、start_index 残留、哨兵 | 旧 `gimbal_memcpy_data_from_chassis` 在 4 项验收中第 2 项失败（无帧头也改写状态）；错尾被接受；半帧被解析 | 176/176 通过 | `tests/rx-chain/b02-chassis-rx*`、`tests/before/b02-before-chassis*` |
| B02-02 rc_rx | 131 checks：18 字节全字段、长度 0..37、36 字节双帧、单帧坏开关、开关 0、通道 ±700/±701、坏帧后恢复、计时回绕、溢出；坏帧不刷新 `last_fdb`/`generation` | 旧 `sbus_to_rc` 在开关 0 场景第 2 项失败（照常发布并刷新时间戳） | 131/131 通过 | `tests/rx-chain/b02-rc-rx*`、`tests/before/b02-before-rc*` |
| B02-03 usb_handoff | 18 checks：ISR 入环/任务出环、交错、环满有界丢弃、TX 缓冲寿命、BUSY 不覆盖、未枚举/空 pClassData、MTU 截断 | 旧 `usb_cdc_port_write` 第 3 项失败（USB 异步读到被覆盖的调用者缓冲） | 18/18 通过 | `tests/rx-chain/b02-usb-handoff*`、`tests/before/b02-before-usb*` |
| 回归 | `validate_runtime` 27 checks、`validate_pitch_mit` 55、`validate_pitch_target` 18、`validate_pitch_enable` 21、`validate_pitch_cold_start` 10、`check_locality` | — | 全部通过 | `tests/regression/` |

替身边界：HAL DMA/UART、USB CDC、`HAL_GetTick` 为替身；用例调用工程真实解析/发布函数与真实回调（`HAL_UART_ErrorCallback`），不复制函数自证。设备行为未测。

## 5. 对 verifier / 其他执行者的接口说明

- 28B 帧权威解析：`static uint8_t chassis_frame_parse(const uint8_t *frame, chassis_data_t *out)`（返回 0 有效 / 1 头 / 2 尾 / 3 字段）。
- 离线注入：`void chassis_rx_feed_stream(const uint8_t *data, uint16_t len)` + `chassis_rx_reset()`；正式接收走 `bsp_usart_rx_start()` 的 DMA 回调。
- DMA 回调语义严格按 HAL：`Size` 是本轮 DMA 已写入位置（circular 会回绕），不是本次事件长度；离线用例请用 `feed_stream`，不要用回调伪造长度。
- `chassis_data` 全树无控制消费者（与 Lead/verifier 结论一致）；本轮只保证解析与发布正确，P10 再谈身份字段。

## 6. 残留风险与边界

1. 无 CRC/序号：头尾+范围校验不能识别所有“外观合法”的损坏。全 0 字段帧（含 mode=0/robot_id=0）会被当作合法帧；这是启动早期 C 侧 INS 未收敛时的真实输出，本轮不据此拒帧。
2. RC 无帧头：采用“IDLE 分块 + 整块 18/36”的明确策略；若发送端在一帧中间停顿时，该块被整块丢弃（计数可见），下一块恢复。不做逐字节滑动重同步，避免发布错位通道。
3. circular DMA 下 `Size==last_pos`（HT 与 IDLE 同位置）按“无新数据”处理，不重复消费。
4. VOFA 为 1ms 调用、USART10 115200：按 BUSY 丢弃后实际约 1/4 帧输出；仅调试观测，不参与控制。
5. USB RX 环满丢新字节（1024B），不阻塞 ISR；`usb_cdc_port_get_stats()` 可观测。
6. 未做：USB 重连/枚举时序设备验证、真实串口错误注入、DMA 与 D-cache 一致性（本轮未改 MPU/cache）。
