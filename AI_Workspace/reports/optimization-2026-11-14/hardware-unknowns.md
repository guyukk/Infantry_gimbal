> B05更新（2026-11-15）：校准值域、当前ARMCC布局和合法帧兼容已软件验证；校准操作以[最终手册](calibration-manual.md)为准，接口有效性见[说明](ui-and-interfaces.md)。下文是原始调查记录，设备未知项仍未验收；原“待软件检查”项由[B05报告](B05-实施与软件签收报告.md)覆盖，不代表B04/B06/B07已开始。

# H723 云台板硬件未知项与范围外设计（2026-11-14）

- 板别：G = 云台板（STM32H723VGTx）
- 日期：2026-11-14
- run-id：20261114-174009
- 依据：交接文档 v1.0 第 1.4 节、P16、第 6 章
- 方法：只读源码。每项按“源码能证明什么 / 不能证明什么 / 设备观察方法 / 是否阻塞”写。
- 分类约定：**[源码事实]** / **[需设备]** / **[需用户决定]**。

---

## 1. 硬件未知表（G 侧）

### 1.1 UART7（921600，PE7 RX / PE8 TX，21 字节 DMA）

- **[源码事实]** UART7 初始化 921600/8N1（`Core/Src/usart.c:90-100`），RX 用 DMA1_Stream6 CIRCULAR、LOW 优先级（`:312-321`），NVIC 5（`:330`）。`main` 里 `HAL_UART_Receive_DMA(&huart7, usart7_buf, 21)`（`Core/Src/main.c:170`）。缓冲区 `usart7_buf[21]`（`User/Bsp/bsp_usart.c:18`）。
- **[源码事实]** `usart7_buf` 除定义与 extern 外**没有任何读取者**；`HAL_UART_RxCpltCallback` 只处理 USART10（`bsp_usart.c:139-146`）；UART7 的 IRQ 只调 `HAL_UART_IRQHandler`（`Core/Src/stm32h7xx_it.c:406-415`）。DMA 是循环模式，每 21 字节会触发一次 RxCplt，但没人消费。
- **推断（标注为推断）**：21 字节与 `REMOTE_FRAME_LEN=21`（图传帧长）一致，历史上可能用于图传遥控；但 G 的 `video_transmission_module` 只有 `vtm_data_init`，**没有 unpack 函数**（`User/APP/video_transmission_module.h:73`），所以现在既没有解析也没有消费者。
- **[需设备]** PE7/PE8 上实际接的是什么（图传？上位机？串口屏？），波特率是否真是 921600，线序是否交叉。
- **观察方法**：逻辑分析仪/示波器量 PE7 是否有 921600 波形；把 `usart7_buf` 加到 Watch，看是否有数据变化；用 USB-TTL 反接发送已知串验证。
- **阻塞**：不阻塞软件交付；P16/D18 要求保留，不得删除。

### 1.2 SHOOT_DETECT（PA2，EXTI2 下降沿）

- **[源码事实]** PA2 配成 EXTI 下降沿、上拉（`Core/Src/gpio.c:93-94`），NVIC 优先级 5（`:114-115`）。`EXTI2_IRQHandler` 调 `HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2)`（`Core/Src/stm32h7xx_it.c:196-207`）。但 `HAL_GPIO_EXTI_Callback` 只判断 `CAM_TRIG_Pin`（`:480-489`），**没有 PA2 分支**。
- **结论**：SHOOT_DETECT 中断会进 HAL 回调，但回调里什么都不做；`detect_task` 也未运行（见 1.4），所以它当前不产生任何状态。
- **[需设备]** PA2 接的是否真是“发射检测”光电/微动，触发极性，是否需要接入健康或计数。
- **观察方法**：手动遮挡/触发传感器，示波器看 PA2 是否有下降沿；在 `HAL_GPIO_EXTI_Callback` 里临时加计数（仅隔离测试构建）观察。
- **阻塞**：不阻塞；P16 明确“SHOOT_DETECT 物理使用未知，保留”。

### 1.3 LED

- **[源码事实]** G 的 `Core/Inc/main.h:59-79` 里没有 LED 引脚宏；`User/` 下没有任何 LED 驱动或 `aRGB` 类代码；`gpio.c` 只配置电源使能、片选、CAM/SHOOT、OSPI 片选/时钟。
- **结论**：G 没有软件可控的 LED。板上的电源/状态灯若存在，只能是硬件直连。
- **[需设备]** 板上是否有电源灯/状态灯、是否由 MCU 控制。
- **观察方法**：上电观察灯；对照原理图确认网络。
- **阻塞**：不阻塞。

### 1.4 main 里额外初始化、但没有业务消费者的外设

| 外设 | 初始化 | 消费者现状 | 证据 |
| --- | --- | --- | --- |
| FDCAN3 | `MX_FDCAN3_Init()` | 无 `FDCAN3_Config()`、无过滤器、无 FIFO 回调；IRQ handler 存在 | `Core/Src/fdcan.c:128-166`；`Core/Src/main.c:137`；`stm32h7xx_it.c:434`；`can_bsp.c:496/561` 只处理 1/2 |
| TIM1 CH1（PA8） | `MX_TIM1_Init()` | 无写 CCR1 的代码 | `Core/Src/tim.c:49-105`；全仓 grep 无 `htim1` 使用 |
| ADC1（DMA）/ ADC3 | `MX_ADC1_Init()`、`MX_ADC3_Init()`、`HAL_ADC_Start_DMA` | `adc_val[2]`、`voltage` 无读取者；ADC3 启动但无读取 | `Core/Src/main.c:143/148/177-180`；全仓 grep `adc_val`/`voltage` 只有定义与启动 |
| CRC（硬件） | `MX_CRC_Init()` | 裁判校验用软件 `CRC8_CRC16.c`，硬件 CRC 无调用 | `Core/Src/crc.c`；`User/Controller/CRC8_CRC16.c` |
| RNG | `MX_RNG_Init()` | 无 `HAL_RNG_*` 业务调用 | `Core/Src/rng.c` |
| TIM12 CH2（PB15） | `MX_TIM12_Init()` | 蜂鸣器 `buzzer_on/off`（`bsp_buzzer.c:15-23`）；由 calibrate_task 调用 | 已使用 |
| TIM2 CH1（PA15） | `MX_TIM2_Init()` | 弹舱 `Bombbay_on/off` | 已使用 |
| TIM24 | `MX_TIM24_Init()` | `tim24_timebase_*` 时间戳 | 已使用 |
| OCTOSPI2 | `MX_OCTOSPI2_Init()` | W25Q 读写（Pitch 记录） | 已使用 |
| USART10 | `MX_USART10_UART_Init()` | RX=TFmini；TX=VOFA/printf | 已使用 |
| UART7 | `MX_UART7_Init()` | 见 1.1，无消费者 | — |

- **[源码事实]** `detect_task.c` 被编译但 `detect_task` 未创建、`detect_hook()` 无调用者（`Core/Src/freertos.c:133-159`；`User/APP/detect_task.c:72`、`:207`）。
- **[需设备]** 上述“未用外设”在板上是否真的空置（例如 ADC1 是否接电流/电压采样、FDCAN3 是否接第三个电机、TIM1 是否有风扇）。
- **观察方法**：对照原理图；ADC 通道用万用表量分压；FDCAN3 用 CAN 分析仪看是否有帧；TIM1 PA8 用示波器看是否有输出（当前 CCR=0）。
- **阻塞**：不阻塞；P16 要求保留。

### 1.5 IMU 加热 PWM 极性（TIM3 CH4 → PB1）

- **[源码事实]** 加热输出是 TIM3_CH4/PB1（`Core/Src/tim.c:412-414`），1 kHz PWM（PSC=23、ARR=9999，`:176-181`）。`IMU_TempCtrl()` 直接写 `htim3.Instance->CCR4`，范围 0..500（`User/APP/INS_task.c:59-71`）。目标温度实际是 `BMI088.TempWhenCali`，`DES_TEMP=40.0f` 未使用（`:25`、`:169`）。
- **[源码事实]** `err = set - ref = 实测温度 - 目标温度`；`err<=0` 时不写 CCR4（保持上次值，残留 PWM）。方向是否为“正 PWM 加热”取决于外部 MOS/加热电阻接法，源码无极性宏、无电路描述。
- **[需设备]** PB1 增大占空比时温度上升还是下降；加热器供电（5V/12V/24V）、MOS 类型、是否与 IMU 热耦合。
- **观察方法**：示波器量 PB1 脉宽；占空比 0/50/100% 三点各保持 30 s，用红外测温或 BMI088 温度读数看斜率。判定：温度随占空比单调升→正极性。
- **阻塞**：**阻塞“整车可发布”声明**（P06：无极性证据不得宣称可发布），不阻塞其它软件批。与 C 的 TIM10/PF6 是两套独立硬件。

### 1.6 HSE 晶振实际值（G：HSE_VALUE=24 MHz vs Keil CLOCK=12 MHz）

- **[源码事实]** `HSE_VALUE=24000000`（`Core/Inc/stm32h7xx_hal_conf.h:110`），PLL1 M=2/N=40/P=1 → 480 MHz（`Core/Src/main.c:231-235`）。Keil Cpu 串写 `CLOCK(12000000)`（`CtrlBoard-H7_IMU.uvprojx:22`）。两者矛盾。
- **[需设备]** 板上晶振丝印/实测；当前固件是否真以 480 MHz 运行。
- **观察方法**：看晶振丝印；频率计量 OSC/MCO；Watch 读 `HAL_RCC_GetHCLKFreq()`；用 USART1 115200 的误码率间接验证。
- **阻塞**：不阻塞软件，但阻塞“时序/波特率已实测”的声明。本轮禁止改 PLL/HSE_VALUE。

### 1.7 CAN 物理拓扑与终端电阻

- **[源码事实]** FDCAN1（PD0/PD1）用于摩擦轮：反馈 `0x201/0x202`（`CAN_FRIC1/2`），发送 `0x200`（`CAN_cmd_friction`，`can_bsp.c:210-221`）。FDCAN2（PB5/PB6）用于 Yaw `0x205`、拨弹 `0x207`、DM Pitch `0x53`（`can_bsp.h:52-55`、`:66-73`；`CAN_cmd_gimbal` `:147-158`；`CAN_cmd_MIT` `:232-264`）。FDCAN3（PD12/PD13）初始化但未启动（`fdcan.c:128-166`）。
- **[源码事实]** FDCAN1 的路由是 RX FIFO0，FDCAN2 是 RX FIFO1（`can_bsp.c:353-423`）；两者过滤器全通过。源码不能证明终端电阻、节点顺序、是否共地、DM 电机与 DJI 电机是否在同一条总线。
- **[需设备]** 两条总线的 120 Ω 终端数量、线束长度、每个电机的物理位置与 ID 拨码。
- **观察方法**：断电万用表量 CANH-CANL（约 60 Ω=双终端）；上电示波器看差分幅度/反射；用 CAN 分析仪只收不发，确认 ID 与周期。
- **阻塞**：不阻塞软件，但**阻塞电机上电调试与 P04 健康判定**（坏线束会制造假掉线）。

### 1.8 外部 Flash（W25Q，OCTOSPI2）实际器件与读写

- **[源码事实]** OCTOSPI2 参数：MemoryType=MICRON、DeviceSize=23（2^23=8 MB）、ClockPrescaler=3、SampleShifting=HALFCYCLE（`Core/Src/octospi.c:42-52`）。驱动 `OSPI_W25Qxx_Init()` 读 ID 并与 `W25Qxx_FLASH_ID` 比较，不匹配返回 `W25Qxx_ERROR_INIT`（`User/Bsp/w25q64.c:13-31`）；`flash_init()` 只把该返回值转成 0/-1（`User/Bsp/bsp_flash.c:8-11`）。因此“换一颗不同 ID 的 Flash”会在初始化就失败，而不是静默写入；但**期望 ID 宏是否匹配实车器件**仍需设备确认。
- **[源码事实]** 使用位置：HD/GM 记录（地址 0，写 108 字节、擦 1 扇区）、Pitch 记录（0x1000，20 字节）。读失败或 `flash_init()` 失败时，Pitch 记录无效 → 零刚度；HD/GM 清 valid。
- **[需设备]** 实际器件型号与容量（W25Q64=8 MB 与 DeviceSize=23 匹配；若换 16 MB 需改 DeviceSize）、读写是否稳定、掉电后记录是否保持。
- **观察方法**：上电后 Watch `pitch_dm_cali_status.stored_valid`；用调试器读 0/0x1000 的内容；反复写-断电-读 10 次统计成功率。
- **阻塞**：不阻塞软件；阻塞“校准记录可靠持久化”的设备验收。

### 1.9 电源使能与 CAM_TRIG

- **[源码事实]** `PWR_EN_5V`（PC15）、`Power_OUT1`（PC13）、`Power_OUT2`（PC14）在 `MX_GPIO_Init` 里先置低，`main` 里再 `Power_OUT1_ON/2_ON`（`Core/Src/main.c:163-164`；`Core/Inc/gpio.h:36-40`）。CAM_TRIG（PA0）EXTI 上升沿 → `comm_camera_trigger_pulse()`（`it.c:480-489`）。
- **[需设备]** 这三路分别给什么供电（5V 外设/图传/激光/相机）、上电时序是否必须、CAM_TRIG 的物理源与极性。
- **观察方法**：万用表量 PC13/PC14/PC15 电压与对应负载；示波器看 PA0 触发沿。
- **阻塞**：不阻塞；但若 Power_OUT 供的是图传，断电时序会影响 USB/图传联调。

---

## 2. 阻塞判定汇总

| 项目 | 阻塞软件交付？ | 阻塞部署/整车发布？ | 需要谁决定 |
| --- | --- | --- | --- |
| UART7 用途 | 否 | 否 | [需用户决定] 是否保留/启用 |
| SHOOT_DETECT | 否 | 否 | [需用户决定] 是否接入健康 |
| LED | 否 | 否 | 无需 |
| 额外外设（FDCAN3/TIM1/ADC/CRC/RNG） | 否 | 否 | [需用户决定] 是否清理 |
| IMU 加热极性（TIM3/PB1） | 否（可交 pending 候选） | **是** | [需设备] + [需用户决定] 加温测试授权 |
| HSE 实际值 | 否 | 是（时序未证实） | [需设备] |
| CAN 终端/拓扑 | 否 | 是（电机上电验收前提） | [需设备] |
| 外部 Flash 器件/读写 | 否 | 否（但影响校准持久化验收） | [需设备] |
| 电源使能/CAM_TRIG | 否 | 否 | [需用户决定] 用途 |

**最可能阻塞交付的三项（G 侧）**：1.5 IMU 加热极性；1.6 HSE 实际值；1.7 CAN 终端/拓扑。

---

## 3. 范围外设计清单（本轮不做）

### 3.1 协议 v2 + CRC/序号（板间 28/46 字节）

- **为什么不做**：会破坏两端与 golden 帧；D02 明确先修接收。当前 28 字节接收连头尾都不校验，先修 v1。
- **接口大致变化（≤5 行）**：帧头加版本；帧尾换 CRC16（复用 `CRC8_CRC16.c`）；加 16 位序号；长度显式化；两端改流式收帧并重冻结 golden。

### 3.2 速度里程计

- **为什么不做**：G 没有轮速来源；C 的 vx/vy/wz 恒 0；D10 明确本轮不造里程计。
- **接口大致变化（≤5 行）**：C 端用四轮反馈做正解；28 字节帧字段语义从“占位 0”变“实测”；G 端消费者需处理“不可用”状态；需打滑/低通。

### 3.3 完整裁判转发

- **为什么不做**：G 没有裁判数据源（`referee_handle_data` 无调用者，`robot_status` 恒 0）；D11 只允许 robot_id 作身份参考。
- **接口大致变化（≤5 行）**：C 端把裁判帧经板间链路转发；需新增帧类型/分包；G 端加超时与 NOT_READY；`fire_allowed` 仍不得据无源数据置 1。

### 3.4 主机底盘运动通道

- **为什么不做**：D08 拒绝；当前实现是“改写 rc_ctrl”（D09 要拆）。需要独立命令通道与仲裁，属新功能。
- **接口大致变化（≤5 行）**：新增 SID 或在现有 CHASSIS 上改为“命令→本地仲裁”；RC 优先、超时清零；拒绝回 NACK；不再写 `rc_ctrl`。

### 3.5 Flash 双槽

- **为什么不做**：D13/范围外；单槽掉电风险已文档化；本轮保留 v1 记录。
- **接口大致变化（≤5 行）**：两个 4 KB 扇区交替；记录加 seq/状态；启动选最新有效；写失败回退；需布局断言与寿命预算。

### 3.6 Yaw 新校准流程

- **为什么不做**：旧 GM 自动校准已禁用；新流程涉及机械端点，需设备与用户决策；D13 不新增 Yaw 自动端点校准。
- **接口大致变化（≤5 行）**：独立 Yaw 记录（不改 GM 20 字节）；反馈有效+静止才采样；保存前校验跨度/方向；写后回读；不恢复撞限位。

### 3.7 Pitch 机械零位与投影启用

- **为什么不做**：`PITCH_DM_ZERO_OFFSET_RAD=0`、投影=0 是当前安全边界；零位需机械基准。
- **接口大致变化（≤5 行）**：两点采样后增加“水平点”采样；写零位偏移；投影开关置 1 前验证 Yaw 陀螺方向；新增验收。

### 3.8 RTOS / 工具链迁移

- **为什么不做**：G 用 `ARM_CM4F` 移植跑 M7，迁移会改 FPU 上下文/启动/优化/警告集；AC5→AC6 同理。
- **接口大致变化（≤5 行）**：换 `ARM_CM7` 移植；开 FPU 上下文；统一 AC6 选项；重做启动与优先级；完整回归+MAP 对比。

---

## 4. 三分法总结

**[源码事实]**

1. UART7 收 21 字节但无人消费；SHOOT_DETECT 有 EXTI 无回调分支；LED 无软件控制。
2. 额外外设清单与消费者（1.4 表）。
3. 加热输出引脚、PWM 参数、目标温度来源、`err<=0` 残留。
4. `HSE_VALUE=24 MHz` 与 Keil `CLOCK(12000000)` 矛盾。
5. FDCAN1/2 的 ID 与 FIFO 路由；FDCAN3 未启动。
6. 外部 Flash 参数、地址与记录用途；驱动不校验器件 ID。
7. 电源使能引脚与 CAM_TRIG 路径。

**[需设备]**

1. IMU 加热极性（TIM3/PB1）。
2. HSE 实际频率。
3. CAN 终端与拓扑。
4. W25Q 实际器件与读写稳定性。
5. 电源使能负载、CAM_TRIG 源、UART7/SHOOT_DETECT 接线。

**[需用户决定]**

1. UART7、SHOOT_DETECT 是否保留/接入。
2. 额外外设是否清理（P16 默认保留）。
3. 是否授权加温极性测试。
4. 协议 v2/双槽/里程计/裁判转发/主机底盘通道是否进入下一阶段。
5. R 键“自瞄+弹舱”双用是否维持（见 controls 文档）。

---

## 5. 观察变量速查（B05 设备阶段直接用）

| 未知项 | 先看哪个变量/寄存器 | 判据 | 需要的工具 |
| --- | --- | --- | --- |
| 加热极性 | `htim3.Instance->CCR4`、`BMI088.Temperature`、`BMI088.TempWhenCali` | 占空比升高时温度单调升=正极性 | 示波器 + 测温 |
| HSE | `HAL_RCC_GetHCLKFreq()`、`HAL_RCC_GetSysClockFreq()` | 期望 240 MHz HCLK / 480 MHz SYSCLK | Watch/调试器 |
| CAN 终端 | CANH-CANL 电阻 | 约 60 Ω=双终端；120 Ω=单；∞=无 | 万用表（断电） |
| 外部 Flash | `pitch_dm_cali_status.stored_valid`、`pitch_dm_cali_status.min_pos/max_pos` | 重启后仍为 1 且值不变 | Watch + 断电重启 |
| UART7 | `usart7_buf[0..20]` | 是否有数据变化 | Watch/逻辑分析仪 |
| SHOOT_DETECT | `EXTI->PR` 对应位或临时计数 | 触发时是否有下降沿 | 示波器 |
| 电源使能 | PC13/PC14/PC15 电平 | `Power_OUT*_ON` 后是否为高 | 万用表 |
| DM 反馈 | `DM_MOTOR_MEASURE[2].fdb.state/id/pos`、`last_fdb_time` | 上电后 state=1（或窄条件 3/5），pos 有限 | Watch |

用法：先看变量确认“固件认为的状态”，再用仪器确认“物理状态”，两者不一致时以物理为准并记录差异。

---

## 6. 本轮边界声明

- 未改任何固件源码、未烧录、未操作设备。
- 未对未知项做猜测性修复；未启用/关闭 D-cache、未改 PLL/HSE、未改加热方向。
- 上述阻塞判定只影响“交付声明/整车发布”，不影响无设备软件批继续。
