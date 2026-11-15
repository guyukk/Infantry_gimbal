# B02 接收/发送链路契约与所有权（P01 / P03 / P08）

日期：2026-11-14（Asia/Shanghai）
批次：B02（run-id 20261114-174009）
范围：G 侧源码；不含设备验收。

## 1. 板间 28 字节帧（C→G，线上格式未变）

| 偏移 | 长度 | 内容 | 接收校验 |
| --- | --- | --- | --- |
| 0 | 1 | `'s'` | 必须 |
| 1..4 | 4 | chassis yaw，float LE | 有限值，\|x\|≤3.2 rad |
| 5..8 | 4 | chassis pitch，float LE | 同上 |
| 9..12 | 4 | chassis roll，float LE | 同上 |
| 13 | 1 | chassis_mode | ≤2（NO_MOVE/FOLLOW/SPIN） |
| 14 | 1 | robot_id | 不限制（0=裁判未就绪，身份有效性归 P10） |
| 15..18 | 4 | vx，m/s | 有限值，\|x\|≤10 |
| 19..22 | 4 | vy，m/s | 有限值，\|x\|≤10 |
| 23..26 | 4 | wz，rad/s | 有限值，\|x\|≤50 |
| 27 | 1 | `'e'` | 必须 |

长度仍为 28；无 CRC/序号（本轮不升级协议）。校验失败不发布、不改写业务状态。
发布序号 `chassis_rx_seq` 每有效帧 +2，`chassis_data_read()` 用序号快照重试，帧计数 = 序号/2。

## 2. 遥控 18 字节 DBUS 帧（P03）

预处理确认实际启用分支：未定义 `SBUS_remotor` → `RC_FRAME_LENGTH=18`、`SBUS_RX_BUF_NUM=36`。
位域/偏移与旧实现逐位一致；新增校验后才发布：

| 项 | 接受范围 | 依据 |
| --- | --- | --- |
| ch0..ch3 原始 11 位 | 324..1724（中值 1024，±700） | 沿用既有 `RC_CHANNAL_ERROR_VALUE=700` |
| 开关 s[0]/s[1] | 枚举 1/2/3，0 拒绝 | 既有 `RC_data_is_error` 语义 |
| 帧块长度 | 18（1 帧）或 36（2 帧） | 半帧/噪声整块丢弃，不发布、不刷新时间戳 |

时间戳 `last_fdb` 与 `generation` 只由合法帧更新，且整帧在短临界区内发布。
`rc_ctrl_snapshot()` 提供序号快照读取（新代码用；旧消费者按 16 位字段原子读取，行为不变）。

## 3. 所有权与缓冲寿命

| 对象 | 生产者 | 消费者 | 缓冲 | BUSY/溢出处理 |
| --- | --- | --- | --- | --- |
| USART1 RX DMA（64B） | DMA 硬件 | `chassis_rx_on_event`（USART1 ISR） | 单一静态缓冲，启动/错误重启共用 | 事件长度异常→清零解析器+重启+计数 |
| 28 字节解析窗 | ISR | ISR | 28B 窗口，有界 | 重同步逐字节推进，不跳过潜在帧头 |
| `gimbal_receive_data` | ISR | 证据/诊断 | 28B，最近有效原始帧 | 仅有效帧覆盖 |
| USART1 TX 46 字节 | `message_task` | DMA | `usart1_tx_frame[64]` 内部副本 | `gState!=READY` 整帧丢弃+计数 |
| UART5 RX（36B） | DMA 硬件 | `remote_control_rx_chunk`（UART5 ISR） | 单一静态缓冲 | 非 18/36 块丢弃+计数 |
| USB RX | `CDC_Receive_HS`（USB ISR） | `usb_cdc_port_rx_pump`（comm_app 任务） | 1024B 无锁 SPSC 环 | 环满丢新字节+`isr_dropped` |
| USB TX | `usb_cdc_port_write`（任务） | USB 外设（ISR 异步读） | `usb_tx_stage[512]` 静态副本 | `TxState!=0` 返回 0+`tx_busy_dropped` |
| VOFA 调试 | auto_aim 任务 | USART10 IT | 256B 静态 | `gState!=READY` 整帧丢弃+计数 |
| USB printf | 已移除 | — | — | 不再有单字节 `__io_putchar` 混入协议流 |

## 4. 与基线差异（行为）

| 场景 | 基线 | B02 后 |
| --- | --- | --- |
| 无帧头/短包 | 解析未初始化 `temp` 或残留 `start_index`，会发布垃圾 | 不发布，字节计数 |
| 错尾 | 接受 | 拒绝，`frames_bad_tail` |
| NaN/Inf/越界枚举 | 接受 | 拒绝，`frames_bad_field` |
| 逐字节/粘包/环绕 | 逐字节全丢；粘包只取首个 | 流式重组，同事件多帧，有界消费 |
| 错误恢复 | 重启 512B 缓冲但解析 28B | 启动/重启同一 64B 缓冲，重置解析器与消费位置 |
| 46B 帧 BUSY | 覆盖在途 DMA 缓冲 | 整帧丢弃并计数 |
| 遥控短帧 | `Size<=18` 即发布 | 仅 18/36 整块且通道/开关合法才发布 |
| USB 初始化 | defaultTask 与 comm_app 各一次 | 仅 defaultTask；comm_app 只等待枚举 |
| USB RX 解析 | USB ISR 内直接跑 uproto | ISR 只入有界环，任务解析 |
| USB TX 缓冲 | 调用者缓冲直接交给 USB（含栈） | 静态副本+单帧在途 |

## 5. 警告对比（ARMCC 5.06u7，Target CtrlBoard-H7_IMU）

基线 B01：0 errors / 33 warnings；B02 最终：0 errors / 31 warnings。

移除的 2 条：
- `..\User\Communication\example\device\usb_cdc_port.c(62): #1-D last line of file ends without a newline`（文件重写）
- `..\User\Communication\example\device\comm_app.c(300): #223-D usb_cdc_port_bind_uproto declared implicitly`（新增 include）

保留的 31 条全部为基线既有项（packed 告警 24 条、INS_task 2 条、calibrate_task 3 条、camera 1 条、it.c 1 条），无新增。
附：基线 packed 告警的实际分布为 `calibrate_task.h` 2、`remote_control.h` 1、`user_lib.h` 2、`referee.h` 19；不是全部来自 `calibrate_task.h`。
