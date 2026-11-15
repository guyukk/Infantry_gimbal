> 2026-11-15 B05核对：28/46B、VTM不变且golden/合法帧历史比较通过；速度为不可用兼容占位，身份不等于裁判许可，无自瞄字段显示UNK。当前接收/超时见[B03报告](B03-两步实施与签收报告.md)与[接口说明](ui-and-interfaces.md)。以下原缺陷描述保留历史语境。

# H723 云台板板间协议现状（28 字节收 / 46 字节发，2026-11-14）

- 板别：G = 云台板（STM32H723VGTx）
- 日期：2026-11-14
- run-id：20261114-174009
- 依据：交接文档 v1.0 第 1.6 节、P01/P03、D02
- 方法：按当前源码逐字段重建。C 侧为对照（详细见 C 仓同名文档），本文件重点是 G 的收/发实现与缺陷。
- 边界：帧里没有 CRC、序号、长度字段；本文件不提出协议升级。

---

## 1. 物理层与角色

| 项目 | G 侧 | 来源 |
| --- | --- | --- |
| 串口 | USART1 | `Core/Src/usart.c:124-165` |
| 参数 | 115200 / 8B / NONE / 1 stop / 过采样 16 / FIFO 关闭 | `Core/Src/usart.c:134-160` |
| 引脚 | PA9 TX、PA10 RX（AF7，LOW 速） | `Core/Src/usart.c:355-364` |
| 接收 | `HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gimbal_receive_data, 28)` | `Core/Src/main.c:174-175` |
| 发送 | `USART1_Transmit_DMA()` → `HAL_UART_Transmit_DMA(&huart1, ...)` | `User/Bsp/bsp_usart.c:23-26` |
| 发送缓冲 | `usart1_buf[2][512]` 与 `gimbal_receive_data[28]` 并存（见缺陷 3） | `User/Bsp/bsp_usart.c:10-11` |
| 接收缓冲 | `gimbal_receive_data[CHASSIS_DATA_LENGTH=28]` | `User/Bsp/bsp_usart.h:18` |
| 发送周期 | `message_task` 初始 500 tick，之后每 3 tick | `User/APP/message_task.c:96-99`、`message_task.h:8-9` |

字节序：G 发送用 `uint32_t*` 取 float 内存后按 `>>0/8/16/24` 拆字节（`User/APP/message_task.c:48-52`），接收用 `uint8_t bytes[4]` 拼回 float（`User/Bsp/bsp_usart.c:68-111`），统一小端 IEEE754。

---

## 2. 本板收发的契约（与 C 仓一致，此处以 G 视角列）

```text
收 C→G（28B）: 's' | yaw | pitch | roll | chassis_behaviour | robot_id | vx | vy | wz | 'e'
发 G→C（46B）: 0xFE | yaw_ecd | INS_yaw | INS_pitch | INS_roll | gimbal_behaviour |
               shoot_mode | VTM[19] | reserved[7] | 0xFD
```

枚举线上数值：

| 字段 | 枚举 | 数值 | 来源 |
| --- | --- | --- | --- |
| C→G 偏移 13 | `chassis_behaviour_e` | NO_MOVE=0、FOLLOW_GIMBAL_YAW=1、SPIN=2 | C `application/chassis_behaviour.h:31-37` |
| G→C 偏移 17 | `gimbal_behaviour_e` | ZERO_FORCE=0、INIT=1、CALI=2、ABSOLUTE_ANGLE=3、RELATIVE_ANGLE=4、SPIN=5、MOTIONLESS=6 | `User/APP/gimbal_behaviour.h:55-64` |
| G→C 偏移 18 | `shoot_mode_e` | STOP=0、READY_FRIC=1、READY_BULLET=2、BULLET=3、CONTINUE_BULLET=4 | `User/APP/shoot.h:13-20` |

---

## 3. G 接收 28 字节：当前实现（`User/Bsp/bsp_usart.c`）

三层：

1. `HAL_UARTEx_RxEventCallback(huart, Size)`（`:149-170`）：
   - USART1 分支只调 `gimbal_memcpy_data_from_chassis()`，**完全忽略 `Size`**（`:151-154`）。
   - UART5 分支才是遥控：`if (Size <= RC_FRAME_LENGTH)` 就解析（`:156-161`）。
2. `gimbal_memcpy_data_from_chassis()`（`:115-137`）：
   - 在 `gimbal_receive_data[0..27]` 中找 `'s'`，用 **static** `start_index` 保存位置（`:118`、`:121-128`）。
   - 找到后 `memcpy(temp, &gimbal_receive_data[start_index], 28)`（`:133`）：当 `start_index>0` 时越界读 28 字节——`temp` 是 28 字节栈数组，源缓冲也只有 28 字节。
   - 找不到 `'s'` 时，`temp` **未初始化**仍被传给 `chassis_to_gimbal(temp)`（`:119`、`:131-136`）。
   - `start_index` 是 static，跨事件残留。
3. `chassis_to_gimbal(data)`（`:58-113`）：按固定偏移解 8 个字段，**不校验 `data[0]=='s'`、不校验 `data[27]=='e'`**。

结果：任意 28 字节噪声（或短包留下的旧数据）都会被当成合法底盘状态写入 `chassis_data`；`chassis_data` 目前**没有任何读取者**（全仓只有写入点：`bsp_usart.c:72-111`），所以当前“写了也没用”，但一旦 B03/B05 接入消费者，这套解析会把错误值直接送进业务。

G 端 `chassis_data_t` 定义：`User/Bsp/bsp_usart.h:34-54`（含 `cap_voltage` 字段，帧里没有对应字节——该字段永远是 0）。

---

## 4. G 发送 46 字节：当前实现（`User/APP/message_task.c:26-89`）

| 偏移 | 内容 | 赋值 | 行号 |
| ---: | --- | --- | --- |
| 0 | 0xFE | 常量 | `:45` |
| 1..4 | `rad_format(gimbal_control.gimbal_yaw_motor.radian_of_ecd)` | float 拆字节 | `:34`、`:48-52` |
| 5..8 | `INS_angle[0]`（yaw） | 取 `get_INS_angle_point()` | `:35`、`:54-58` |
| 9..12 | `INS_angle[1]`（pitch） | 同上 | `:60-63` |
| 13..16 | `INS_angle[2]`（roll） | 同上 | `:65-68` |
| 17 | `(uint8_t)gimbal_behaviour` | 枚举 | `:36`、`:70` |
| 18 | `shoot_control.shoot_mode` | 枚举 | `:37-38`、`:72` |
| 19..37 | `vtm_data` 字节 2..20 | 逐字节拷贝 | `:75-78` |
| 38..44 | 0 | 显式清零 | `:79-82` |
| 45 | 0xFD | 常量 | `:85` |

说明：`radian_of_ecd` 由 `ecd * MOTOR_ECD_TO_RAD` 得到（`User/gimbal/yaw_pitch_direct.c:444`），是原始编码换算值；`offset_ecd` 校准只影响 `relative_angle`（`:434-436`），不影响本字段。因此 C 端收到的 1..4 不是“校准后相对角”，不能被当作零位基准。

---

## 5. VTM_Data_t 布局与编译期检查

- G 结构：`User/APP/video_transmission_module.h:20-64`，`#pragma pack(1)`，字段顺序与 C 仓一致。
- 编译期断言：`typedef char vtm_board_layout_size_check[(sizeof(VTM_Data_t) == 21U) ? 1 : -1];`（`:67`）。
- 位域偏移只在注释里（ch0@16 … crc@152），**没有 offsetof 证据**；`sizeof==21` 不能证明位序。详见 C 仓协议文档第 5 节（同一结论）。
- G 的 `vtm_data` 由 `Core/Src/main.c:169` 调 `vtm_data_init()` 初始化；发送时整段原样搬到帧里，没有逐字段序列化。

---

## 6. G 接收链的其它缺陷（源码事实）

| # | 缺陷 | 证据 | 后果 |
| --- | --- | --- | --- |
| 1 | USART1 的 `Size` 被忽略 | `User/Bsp/bsp_usart.c:151-154` | 任何 RX 事件（含半帧）都触发整帧解析 |
| 2 | `memcpy` 从 `start_index` 起固定 28 字节 | `:133` | `start_index>0` 时越界读；缓冲前后相邻全局可能被读入 |
| 3 | 错误恢复用的缓冲与解析缓冲不是一个 | 错误回调把接收重挂到 `usart1_buf[0]`、长度 512（`:177-178`），而解析读 `gimbal_receive_data` | 错误后新数据进了别的缓冲，解析器继续读旧缓冲，可能把旧帧当新帧 |
| 4 | 找不到帧头时把未初始化的 `temp` 传下去 | `:119`、`:131-136` | 随机栈内容进入 `chassis_data` |
| 5 | 没有接收计数/时间戳/坏帧计数 | 全文件 | 健康状态无法从协议层获得（与 P04 相关） |
| 6 | `chassis_data` 无消费者 | 全仓只有 `bsp_usart.c` 写入 | 当前解析结果完全未被使用；D11 说的“收到 robot_id 存入 chassis_data”属实但无下游 |

验收含义：P01 的用例（0..27 短包、起点 0..27、无头、头出现在 float 载荷、错尾、逐字节拆包、两帧粘连、DMA 环绕、噪声、NaN/Inf、错误后恢复）在当前实现下都会“解析”，修复后才能“只发布合法帧”。

---

## 7. Golden 帧示例（由源码推得，非实测）

输入与算法同 C 仓协议文档第 10 节；这里给 G 端视角的字节，供 B02 双向比对。

### 7.1 G→C，46 字节

```text
FE 00 00 80 3E 9A 99 99 3E CD CC CC BD 0A D7 23 3C 04 02
00 04 20 00 01 18 00 08 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 FD
```

（连续 46 字节；输入：`radian_of_ecd=0.25`、`INS_yaw=0.3`、`INS_pitch=-0.1`、`INS_roll=0.01`、`gimbal_behaviour=4`、`shoot_mode=2`、VTM 为 `vtm_data_init()` 默认值。）

| 字节 | 值 | 来源 |
| --- | --- | --- |
| 0 / 45 | `FE` / `FD` | `message_task.c:45` / `:85` |
| 1..4 | `00 00 80 3E` | 0.25f（`:48-52`） |
| 5..8 | `9A 99 99 3E` | 0.3f（`:54-58`） |
| 9..12 | `CD CC CC BD` | -0.1f（`:60-63`） |
| 13..16 | `0A D7 23 3C` | 0.01f（`:65-68`） |
| 17 / 18 | `04` / `02` | 枚举（`:70`、`:72`） |
| 19..37 | `00 04 20 00 01 18 00 08 00×11` | VTM 19 字节（**按注释 bit 偏移推导**，见第 5 节风险） |
| 38..44 | 7 × `00` | 预留清零（`:79-82`） |

### 7.2 C→G，28 字节（G 端应接受的目标帧）

```text
73 CD CC CC 3D CD CC 4C BD 0A D7 A3 3C 01 03 00 00 00 00 00 00 00 00 00 00 00 00 65
```

含义：yaw=0.1、pitch=-0.05、roll=0.02、behaviour=1、robot_id=3、vx=vy=wz=0。G 解包后应得到 `chassis_data.chassis_mode=1`、`robot_id=3`（当前无消费者，只能通过 Watch 观察）。

---

## 8. 长度常量与缓冲区对照（补充）

| 常量/缓冲 | 值 | 定义位置 | 使用者 |
| --- | ---: | --- | --- |
| `CHASSIS_DATA_LENGTH` | 28 | `User/Bsp/bsp_usart.h:18` | `main.c:175` 的 `ReceiveToIdle`、`bsp_usart.c` 的 `temp[28]`、`memcpy` 长度 |
| `SBUS_RX_BUF_NUM` | 36（`#else` 分支） | `User/Bsp/bsp_usart.h:72` | UART5 遥控 |
| `RC_FRAME_LENGTH` | 18（`#else` 分支） | `:73` | UART5 的 `Size <=` 判据 |
| `USART_RX_BUF_LENGHT` | 512 | `:19` | `usart1_buf[2][]`、错误回调重挂长度 |
| `gimbal_receive_data` | 28 | `User/Bsp/bsp_usart.c:11` | 28 字节接收缓冲 |
| `usart1_buf` | 2×512 | `:10` | 错误恢复时被重挂，实际不被解析 |
| `usart7_buf` | 21 | `:18` | UART7，无消费者（见硬件未知文档） |
| VTM `sizeof` | 21 | `User/APP/video_transmission_module.h:67`（断言） | 发送 19 字节（去掉 2 字节头） |

结论：G 的 28 字节路径涉及 3 个长度概念（28 / 36 / 512），当前“解析读 28、错误恢复写 512”的错配就来自这张表；B02 统一时必须同时改 `main.c` 的初始化长度、`bsp_usart.c` 的解析缓冲与错误回调三处，漏一处就会出现“恢复后读到旧数据”。

---

## 9. 与交接文档 v1.0 的差异

| # | 原陈述 | 当前事实 | 证据 | 影响 |
| --- | --- | --- | --- | --- |
| GP-D1 | P01：“28 字节数组中找到非零起点后仍复制 28 字节；start_index 静态残留；找不到头时局部 temp 未有效初始化却被解析；忽略 Size；未校验尾” | 全部属实，逐条对应 `bsp_usart.c:118`、`:133`、`:119`+`:136`、`:151-154`、`:58-113` | 同左 | P01 可直接用这四条做“修改前失败”用例 |
| GP-D2 | P01：“错误恢复改接收缓冲而解析仍读原缓冲” | 属实：错误回调重挂 `usart1_buf[0]`（512 字节），解析读 `gimbal_receive_data`（28 字节） | `bsp_usart.c:177-178` vs `:11`、`:115-137` | 修复要统一到同一缓冲与同一消费位置 |
| GP-D3 | 1.6：“G→C 19..37 为 VTM 去掉两字节头后的 19 字节原布局” | 属实；`sizeof(vtm_data)-2` 两处一致 | `message_task.c:75-78`；C `bsp_usart.c:279` | 无 |
| GP-D4 | 1.6：“VTM_Data_t 当前 sizeof=21，有编译期检查” | 属实，但只有 sizeof；位域偏移仅注释 | `video_transmission_module.h:67` | golden 冻结前必须实测位域 |
| GP-D5 | P03：“Size <= RC_FRAME_LENGTH 会接受短帧并读取完整帧字段，随后刷新 last_fdb” | 属实（UART5 分支）；`sbus_to_rc` 末尾 `last_fdb=HAL_GetTick()` | `bsp_usart.c:158-161`；`User/APP/remote_control.c:148` | P03 的根因与修法明确 |
| GP-D6 | P03 提示：“需以预处理确认是 18 字节 DBUS 还是 25 字节 SBUS 分支” | 当前 `SBUS_remotor` 未在工程宏中定义（Cads Define 只有 `USE_HAL_DRIVER,STM32H723xx`），走 `#else` 的 18 字节分支 | `User/Bsp/bsp_usart.h:59-82`；uvprojx:342 | 修接收时可按 18 字节设计，但要在报告里写清预处理依据 |
| GP-D7 | P10：“G 收到 robot_id 存入 chassis_data，发布裁判却读本地 robot_status 等” | 属实，且更彻底：`chassis_data` 全字段**无任何读取者**；`robot_status` 也全为 0（无裁判数据源） | `bsp_usart.c:72-111`；`comm_app.c:564`；`referee.c:87` 无调用者 | D11 的“身份参考”当前连参考都没接入；B05 不能假定 chassis_data 已被使用 |
| GP-D8 | D02：“保留 28/46 字节帧……先修接收” | 一致；补充：G 发送侧没有需要改的字节语义，改动应集中在接收与缓冲所有权 | 同左 | 无 |
| GP-D9 | 1.6：“G→C 1..4 为 Yaw 电机编码转换并 rad_format 的值；不能擅改为相对校准角” | 属实 | `yaw_pitch_direct.c:444`；`message_task.c:34` | 无 |
| GP-D10 | P01 验收：“合法帧逐字节与基准一致” | 当前没有 golden 帧文件；本文件第 7 节给出源码推导的基准，但 VTM 段需实测确认 | 本文件 | B02 需要先把第 7 节实测化再冻结 |

差异共 10 条；对 B02 直接相关的是 GP-D1、GP-D2、GP-D4、GP-D6。

---

## 10. B02 需要冻结的点（G 端）

1. 46 字节发送：头/尾、偏移 17/18 枚举、38..44 全 0 与第 7.1 节一致。
2. 28 字节接收：只接受 `[0]=='s' && [27]=='e'` 的完整 28 字节；短包、错位、错尾、噪声一律不更新 `chassis_data`。
3. 错误恢复：重启接收与解析器必须使用同一缓冲、同一消费位置；恢复后第一帧不能是旧数据。
4. 缓冲哨兵：`gimbal_receive_data[28]` 与 `usart1_buf[2][512]` 前后不得越界写。
5. `chassis_data` 一旦接入消费者（B03/B05），必须确认它只由合法帧更新。

---

## 11. 接收层状态机（仅设计约束，B02 参考）

本节不写实现代码，只固定“必须满足的边界”，避免 B02 修好一个用例又打破另一个。

1. **缓冲所有权唯一**：`HAL_UARTEx_ReceiveToIdle_DMA` 的缓冲、DMA 写入位置、解析消费位置必须是同一组变量；错误回调只能重置这一组，不能再引入 `usart1_buf` 作为第二缓冲。
2. **增量消费**：用 `Size` 与上一轮消费点计算本次新增字节，允许环绕；不能把 `Size` 直接当帧长，也不能每次从下标 0 重扫固定 28 字节。
3. **流式重同步**：逐字节推进找 `'s'`，凑满 28 字节后校验 `[27]=='e'`；失败时只前进 1 字节继续找，不能跳过潜在帧头。
4. **字段合法化**：发布前检查 6 个 float 的有限性（NaN/Inf）、偏移 13 的枚举 0..2、偏移 14 的 robot_id 范围；非法帧丢弃并计数。
5. **原子发布**：解析结果先放局部/影子结构，校验全通过后一次性复制到 `chassis_data`；禁止边解边写全局。
6. **可观测**：至少提供 `rx_frames_ok`、`rx_bad_len`、`rx_bad_tail`、`rx_bad_value`、`rx_overflow`、`rx_restart` 六个计数与 `last_valid_rx_tick`。
7. **错误恢复**：重启接收后清空消费点与解析器状态；恢复后的第一帧必须是本次新收到的完整帧，不得复用旧缓冲。

与第 3、6 节对照：当前实现 7 条全部不满足（`User/Bsp/bsp_usart.c:115-137`、`:149-170`、`:173-190`）。B02 的“修改前失败”用例应逐条对应上表编号，便于报告逐项关闭。
