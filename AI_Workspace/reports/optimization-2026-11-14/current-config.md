> 2026-11-18：小陀螺平移软件修复及正式底盘构建已完成，见[本批次报告](小陀螺平移修复与验收-2026-11-18.md)。采用同帧角度对和混合旋转预算；实车未验收，P18未关闭。下方带日期内容保留历史状态。

> 2026-11-16：B04/B06软件阶段完成，B07总核与交付见[B07最终报告](B07-总核与最终交付报告.md)。P06温控仅交隔离候选，极性仍待设备确认；小陀螺平移未修复，整车设备验收未完成。

## 2026-11-16 当前配置增补

| 项目 | 底盘C | 云台G |
| --- | --- | --- |
| 工程/Target | MDK-ARM/standard_tpye_c.uvprojx / standard_tpye_c | MDK-ARM/CtrlBoard-H7_IMU.uvprojx / CtrlBoard-H7_IMU |
| MCU/DFP | STM32F407IGHx / F4 DFP3.1.1 | STM32H723VGTx / H7 DFP4.1.3 |
| 编译器/正式源输入 | ARMCC5.06u7 build960 / 116 | ARMCC5.06u7 build960 / 113 |
| RTOS tick/堆 | 1000Hz / 24576字节 | 1000Hz / 30720字节 |
| 控制任务 | chassis High/栈参数1024；1ms DelayUntil | gimbal High/栈参数1024；1ms Delay |
| 其他任务 | message/INS/LED/referee/servo/UI | default/cali/auto_aim/COMM_APP/message/INS |
| 输入/电机 | USART3 RC，USART1板间；CAN1驱动201..204、CAN2转向205..208 | UART5 RC，USART1板间；FDCAN2 Yaw205/拨弹207/DM53，FDCAN1摩擦201/202 |
| 当前默认 | F407舵轮；软转向和现车零偏保留 | Infantry/yaw_pitch_direct/friction_3508/gimbal_board；ROBOT_MODE默认0 |
| 存储/协议 | sector9旧布局，28B发送/46B接收 | NOR sector0旧HD/GM、0x1000 Pitch v1，46B发送/28B接收 |

完整参数及消费者见[B06索引](B06-维护边界与参数索引.md)。112/111等旧条目数属于历史阶段；最新构建和固件身份以B07报告为准。HSE_VALUE为C12MHz/G24MHz的软件配置，不代替实物晶振测量。

---

# B05当前配置与原始提取索引

更新2026-11-15：C有效源116，G111；Target/MCU/DFP/宏值、时钟、外设、任务创建配置与控制参数保持B03。两端health.c已编入，G另编入host_control.c；正式debug=0，release=1仅隔离验证。

当前行为以[B03报告](B03-两步实施与签收报告.md)、[B05报告](B05-实施与软件签收报告.md)、[校准手册](calibration-manual.md)、[接口说明](ui-and-interfaces.md)为准。本轮加值域/布局断言、Flash错误处理及状态有效性；B04/B06/B07未开始。

下方保留原B00外围配置提取；旧条目数、日期和“未编译/未接入”是历史状态，涉及变更以上述实现文档覆盖，不能当作当前结论。

---

> 2026-11-15 B03 软件阶段已完成；实机未验收。当前结论见 [B03-两步实施与签收报告.md](B03-两步实施与签收报告.md)。以下保留为历史记录。

# H723 云台板当前配置清单（B00，2026-11-14）

- 板别：G = 云台板（STM32H723VGTx）
- 日期：2026-11-14
- run-id：20261114-174009
- 依据：交接文档 v1.0 第 1 章、第 4 章 B00
- 方法：只读源码与 Keil 工程文件，逐项取自 `.uvprojx` / `.c` / `.h`；未编译、未烧录、未改源码。
- 仓库状态：git HEAD `1788028`（“修复了冷启动pitch不使能的问题并校准了pitch电机范围”）。**本文件读取时（2026-11-14 17:45–17:55）** `Core/`、`User/`、`USB_DEVICE/` 无未提交修改，下文描述该提交的 B00 基线。读取之后 B02 实施者已开始改动 `User/APP/remote_control.h`、`User/Bsp/bsp_usart.h` 并出现 `*.new` 文件；这些不在本文范围内，比较时请以 `git diff 1788028` 为准。
- 凡源码无法判定的写 `unknown`。

---

## 1. 工程身份

| 项目 | 值 | 来源 |
| --- | --- | --- |
| Keil 工程 | `MDK-ARM/CtrlBoard-H7_IMU.uvprojx` | 文件存在 |
| Target 名 | `CtrlBoard-H7_IMU` | `MDK-ARM/CtrlBoard-H7_IMU.uvprojx:10` |
| Toolset | `ARM-ADS`，`0x4` | uvprojx:11-12 |
| 编译器 | `pCCUsed = 5060960::V5.06 update 7 (build 960)::.\ARMCC` | uvprojx:14 |
| `uAC6` | `0` | uvprojx:15 |
| Device | `STM32H723VGTx` | uvprojx:18 |
| DFP | `Keil.STM32H7xx_DFP.4.1.3` | uvprojx:20 |
| Cpu 串 | `IRAM(0x20000000,0x00020000) IRAM2(0x24000000,0x00050000) IROM(0x08000000,0x00100000) XRAM(0x30000000,0x00008000) XRAM2(0x38000000,0x00004000) CPUTYPE("Cortex-M7") FPU3(DFPU) CLOCK(12000000) ELITTLE` | uvprojx:22 |
| Flash 下载算法 | `STM32H72x-73x_1024.FLM`（1 MB @0x08000000） | uvprojx:25 |
| 输出目录 | `CtrlBoard-H7_IMU\` | uvprojx:52 |
| 输出名 | `CtrlBoard-H7_IMU` | uvprojx:53 |
| CreateExecutable / CreateLib | 1 / 0 | uvprojx:54-55 |
| CreateHexFile | 1 | uvprojx:56 |
| DebugInformation / BrowseInformation | 1 / 1 | uvprojx:57-58 |
| 链接 useFile | 0 | uvprojx:371 |
| ScatterFile | 空 | uvprojx:375 |
| IncludeLibs / LinkerInputFile | 空 | uvprojx:376 / :379 |
| Optim | 1 | uvprojx:318 |
| oTime / SplitLS | 0 / 0 | uvprojx:319-320 |
| wLevel | 2 | uvprojx:327 |
| uC99 / uGnu | 1 / 1（**使用 GNU 扩展**，与 C 仓不同） | uvprojx:330-331 |
| v6Lang / v6LangP | 3 / 3 | uvprojx:333 |
| `.ioc` | 仓库根 `CtrlBoard-H7_IMU.ioc` | 文件存在 |

Keil `CLOCK(12000000)` 与 HAL `HSE_VALUE=24000000`（`Core/Inc/stm32h7xx_hal_conf.h:110`）不一致，属待核对元数据，不据此改晶振。

---

## 2. 目标级 Define 与 IncludePath

Define（uvprojx:342，逐条）：

```text
USE_HAL_DRIVER
STM32H723xx
```

IncludePath（uvprojx:344，25 条，逐条）：

```text
../Core/Inc
../Drivers/STM32H7xx_HAL_Driver/Inc
../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
../Drivers/CMSIS/Device/ST/STM32H7xx/Include
../Drivers/CMSIS/Include
../Drivers/CMSIS/DSP/Include
../Middlewares/Third_Party/FreeRTOS/Source/include
../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F
../User/Algorithm/mahony
../User/APP
../User/Bsp
../User/Devices/BMI088
../User/Controller
../User/Lib
../User/Algorithm/PID
../User/ROBOT
../User/shoot
../User/gimbal
../USB_DEVICE/App
../USB_DEVICE/Target
../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
../User/Communication/channel
../User/Communication/core
../User/Communication/example/device
```

汇编器 IncludePath：`../Core/Inc;../Drivers/CMSIS/Include`（uvprojx:362）。

事实性不一致：FreeRTOS 移植目录是 `portable/RVDS/ARM_CM4F`，而 MCU 是 Cortex-M7 + 双精度 FPU（uvprojx:22）。交接文档已声明“本轮不迁移 RTOS 移植层”，本清单只记录该组合存在。

---

## 3. 组级 / 文件级覆盖

| 组 | 条目 | GroupOption | 文件级 FileOption |
| --- | ---: | --- | --- |
| Application/User/Core | 15 | 无 | 9 个文件 Optim=0（freertos/adc/crc/fdcan/octospi/rng/tim/usart/hal_timebase_tim） |
| APP | 11 | 无 | 无 |
| ROBOT | 2（.h） | 无 | 无 |
| gimbal | 1 | 无 | 无 |
| shoot | 1 | 无 | 无 |
| common | 8 | 无 | 无 |
| bsp | 11 | 无 | 无 |
| Controller | 1 | 无 | 无 |
| Algorithm | 3 | 无 | 无 |
| Application/MDK-ARM | 1（.s） | 无 | 无 |
| Drivers/STM32H7xx_HAL_Driver | 33 | 无 | 13 个文件 Optim=0（pcd/pcd_ex/ll_usb/adc/adc_ex/crc/crc_ex/fdcan/ospi/rng/rng_ex/uart/uart_ex） |
| Device | 2 | 无 | 无 |
| Drivers/CMSIS | 1 | 无 | 无 |
| Middlewares/FreeRTOS | 10 | 有：`Optim=0`、`wLevel=4`、`IncludeInBuild=2` | 10 个文件 Optim=0 |
| DSP | 1（.lib） | 无 | 无 |
| Application/User/USB_DEVICE/App | 3 | 有：`Optim=0`、`wLevel=4`、`IncludeInBuild=2` | 3 个文件 Optim=0 |
| Application/User/USB_DEVICE/Target | 1 | 有：`Optim=0`、`wLevel=4`、`IncludeInBuild=2` | 1 个文件 Optim=0 |
| Middlewares/USB_Device_Library | 4 | 有：`Optim=0`、`wLevel=4`、`IncludeInBuild=2` | 4 个文件 Optim=0 |
| `::CMSIS` | 0 | 无 | 无 |

行号：FreeRTOS GroupOption `uvprojx:2024-2059`（Optim=0 在 `:2044`，wLevel=4 在 `:2053`）；USB App `:2668-2703`（`:2688`、`:2697`）；USB Target `:2910-2945`；USB 库 `:3040-3075`。文件级 Optim=0 的 `Optim` 行例如 `:422`、`:478`、`:534`、`:595`、`:651`、`:707`、`:768`、`:824`、`:890`（Core），`:1201`…`:1963`（HAL），`:2118`…`:2622`（FreeRTOS）。

结论：目标级 `-O1`；FreeRTOS/USB/部分 HAL 与部分 Core 文件为 `-O0`。没有任何 `IncludeInBuild=0`、没有按文件的 Define/IncludePath 覆盖。

---

## 4. 库与启动

| 项目 | 值 | 来源 |
| --- | --- | --- |
| 启动文件 | `startup_stm32h723xx.s` | uvprojx:1158 / :1160 |
| 预编译库 | `..\Drivers\CMSIS\DSP\Lib\ARM\arm_cortexM7lfdp_math.lib`（FileType=4） | uvprojx:2660 / :2662 |
| 其它 .lib | 无 | uvprojx 全表 |
| 算法库 | Mahony 源码（`User/Algorithm/mahony/mahony_filter.c`） | uvprojx Algorithm 组；源文件存在 |

与 C 仓不同：G 没有 `AHRS.lib`，姿态解算走 Mahony 源码；DSP 库为 M7 双精度 `lfdp` 版本。

---

## 5. 时钟与内存

| 项目 | 值 | 来源 |
| --- | --- | --- |
| HSE_VALUE | 24 MHz | `Core/Inc/stm32h7xx_hal_conf.h:110` |
| HSI / CSI / LSE | 64 MHz / 4 MHz / 32768 Hz | `:131`、`:122`、`:139` |
| 主 PLL（PLL1） | 源 HSE，M=2，N=40，P=1，Q=2，R=2，RGE=RANGE_3，VCO=WIDE，FRACN=0 | `Core/Src/main.c:226-238` |
| SYSCLK 源 / 分频 | PLLCLK / 1 | `:249-250` |
| AHB | /2 → 240 MHz | `:251` |
| APB1/2/3/4 | 均 HCLK /2 | `:252-255` |
| FLASH_LATENCY | 3 | `:257` |
| 推导 SYSCLK | 480 MHz | 由 24/2×40/1 推导 |
| PLL2 | M=24，N=200，P/Q/R=2，RGE=RANGE_0，VCO=MEDIUM | `:274-281` |
| ADC / FDCAN 时钟源 | PLL2（`RCC_ADCCLKSOURCE_PLL2`、`RCC_FDCANCLKSOURCE_PLL2`） | `:282-283` |
| ADC/FDCAN 时钟（推导） | 24/24×200/2 = 100 MHz | 由上述常量推导 |
| 供电 | `PWR_LDO_SUPPLY`，VOS0 | `:215-221` |
| 时基 | TIM6，1 MHz 计数、1 ms 周期，优先级 `TICK_INT_PRIORITY=15` | `Core/Src/stm32h7xx_hal_timebase_tim.c:80-101`；`Core/Inc/stm32h7xx_hal_conf.h:169` |
| 追踪时基 | TIM24，PSC=239 → 1 MHz，ARR=0xFFFFFFFF | `Core/Src/tim.c:264-269`；`User/Bsp/bsp_tim24.c:6-35` |
| DWT | `DWT_Init(480)`（按 480 MHz 换算） | `Core/Src/main.c:153` |

内存：Flash 1 MB @0x08000000；IRAM 128 KB @0x20000000；IRAM2 320 KB @0x24000000；XRAM 32 KB @0x30000000；XRAM2 16 KB @0x38000000（uvprojx:22）。工程不使用自定义 ScatterFile，实际段分配、DMA 缓冲所在区、D-cache/MPU 设置：`unknown`（无 MAP 证据；交接文档要求单独报告缓存一致性疑点，本轮不开启/关闭 D-cache）。

---

## 6. 启动序列（`Core/Src/main.c`）

`main()`（`:105-202`）：

1. `HAL_Init()` — `:114`
2. `SystemClock_Config()` — `:121`
3. `PeriphCommonClock_Config()` — `:124`
4. `MX_GPIO_Init()` — `:131`
5. `MX_DMA_Init()` — `:132`
6. `MX_SPI2_Init()` — `:133`
7. `MX_FDCAN1_Init()` — `:134`
8. `MX_FDCAN2_Init()` — `:135`
9. `MX_TIM3_Init()` — `:136`
10. `MX_FDCAN3_Init()` — `:137`
11. `MX_USART1_UART_Init()` — `:138`
12. `MX_UART5_Init()` — `:139`
13. `MX_USART10_UART_Init()` — `:140`
14. `MX_CRC_Init()` — `:141`
15. `MX_RNG_Init()` — `:142`
16. `MX_ADC1_Init()` — `:143`
17. `MX_UART7_Init()` — `:144`
18. `MX_TIM1_Init()` — `:145`
19. `MX_TIM2_Init()` — `:146`
20. `MX_TIM12_Init()` — `:147`
21. `MX_ADC3_Init()` — `:148`
22. `MX_OCTOSPI2_Init()` — `:149`
23. `MX_TIM24_Init()` — `:150`
24. 用户初始化（`:152-181`）：`tim24_timebase_init()`、`DWT_Init(480)`、`pitch_dm_params_init()`、`cali_param_init()`、`BMI088_init(&hspi2,0)` 死等、`Power_OUT1_ON/Power_OUT2_ON`、`FDCAN1_Config()`、`FDCAN2_Config()`、`vtm_data_init()`、`HAL_UART_Receive_DMA(&huart7, usart7_buf, 21)`、`HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_buff, 36)`、`HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gimbal_receive_data, 28)`、ADC 校准与启动、`proto_init_from_main()`
25. `MX_FREERTOS_Init()` — `:186`；`osKernelStart()` — `:189`

USB 初始化**不在 main**：`StartDefaultTask` 里 `MX_USB_DEVICE_Init()`（`Core/Src/freertos.c:177`），且 `comm_app_task` 里**又调用一次**（`User/Communication/example/device/comm_app.c:288`）。两处都在源码中，是 P08 的直接证据。

`USART10` 的接收初始化不在 main：`comm_app_task` 里 `tfmini_uart_init(&huart10)`（`comm_app.c:291`），注释也说明“避免 main 里初始化导致缓冲越界”（`Core/Src/main.c:171`）。

---

## 7. GPIO 引脚复用表

直接 GPIO（`Core/Inc/main.h` + `Core/Src/gpio.c`）：

| 引脚 | 宏 | 方向/模式 | 来源 |
| --- | --- | --- | --- |
| PC15 | `PWR_EN_5V_Pin` | 输出 PP，初始 SET | `main.h:60-61`；`gpio.c:59` |
| PC0 | `ACC_CS_Pin` | 输出 PP，初始 SET | `main.h:62-63`；`gpio.c:59` |
| PC3 | `GYRO_CS_Pin` | 输出 PP，初始 SET | `main.h:64-65`；`gpio.c:59` |
| PA0 | `CAM_TRIG_Pin` | EXTI 上升沿 | `main.h:66-68`；`gpio.c:87-88`、`:111-112` |
| PA2 | `SHOOT_DETECT_Pin` | EXTI 下降沿，上拉 | `main.h:69-71`；`gpio.c:93-94`、`:114-115` |
| PE9 | `CS_Pin`（W25Q 片选） | 输出 PP，初始 SET | `main.h:72-73`；`gpio.c:62` |
| PE10 | `ACC_INT_Pin` | 输入 IT | `main.h:74-75`；`gpio.c:106-107` |
| PE12 | `GYRO_INT_Pin` | 输入 IT | `main.h:76-77`；`gpio.c:106-107` |
| PE13 | `CLK_Pin` | 输出 PP，初始 SET | `main.h:78-79`；`gpio.c:62` |
| PC13 / PC14 | `Power_OUT1/2` | 输出 PP，初始 RESET；main 里置 ON | `Core/Inc/gpio.h:36-40`；`gpio.c:56`、`:65-67`；`main.c:163-164` |

外设复用：

| 外设 | 引脚 | 备注 | 来源 |
| --- | --- | --- | --- |
| USART1 | PA9 TX、PA10 RX | AF7，LOW 速 | `Core/Src/usart.c:355-364` |
| UART5 | PC12 TX、PD2 RX | AF8，LOW 速 | `usart.c:236-252` |
| UART7 | PE7 RX、PE8 TX | AF7，LOW 速 | `usart.c:299-308` |
| USART10 | PE2 RX（AF4，上拉）、PE3 TX（AF11） | LOW 速 | `usart.c:429-445` |
| FDCAN1 | PD0 RX、PD1 TX | 复用 | `fdcan.c:195-198` |
| FDCAN2 | PB5 RX、PB6 TX | 复用 | `fdcan.c:225-228` |
| FDCAN3 | PD12 RX、PD13 TX | 复用 | `fdcan.c:255-258` |
| SPI2 | PC1 MOSI、PC2 MISO、PB13 SCK | AF，NSS 软件 | `spi.c:100-111` |
| TIM3 CH4 | PB1 | IMU 加热 | `tim.c:412-414` |
| TIM2 CH1 | PA15（JTDI） | 弹舱 | `tim.c:391-393` |
| TIM12 CH2 | PB15 | 蜂鸣器 | `tim.c:433-435`；`bsp_buzzer.c:17-18` |
| TIM1 CH1 | PA8 | 已配置，消费者 `unknown` | `tim.c:370-372` |
| OCTOSPI2 | W25Q（4 线） | 外扩 Flash | `octospi.c:42-52` |
| USB HS | OTG_HS | CDC | `USB_DEVICE/App/usb_device.c:64-84` |

`CAM_TRIG`/`SHOOT_DETECT` 的物理用途：`CAM_TRIG` 有 EXTI→`comm_camera_trigger_pulse()`（`Core/Src/stm32h7xx_it.c:480-489`）；`SHOOT_DETECT` 的 EXTI2 只调 `HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2)`（`it.c:196-207`），但 `HAL_GPIO_EXTI_Callback` 只判断 `CAM_TRIG_Pin`，**没有 PA2 分支**，所以 SHOOT_DETECT 目前不产生任何行为。两者物理用途仍 `unknown`。

---

## 8. DMA 与 NVIC

| 外设 | Stream / Request | 模式 | 优先级 | 来源 |
| --- | --- | --- | --- | --- |
| USART1 RX | DMA1_Stream2 / USART1_RX | CIRCULAR | LOW | `usart.c:368-377` |
| USART1 TX | DMA1_Stream4 / USART1_TX | NORMAL | LOW | `usart.c:386-395` |
| UART5 RX | DMA1_Stream5 / UART5_RX | NORMAL | VERY_HIGH | `usart.c:256-265` |
| UART7 RX | DMA1_Stream6 / UART7_RX | CIRCULAR | LOW | `usart.c:312-321` |
| SPI2 RX | DMA1_Stream0 / SPI2_RX | NORMAL | VERY_HIGH | `spi.c:120-129` |
| SPI2 TX | DMA1_Stream1 / SPI2_TX | NORMAL | HIGH | `spi.c:138-147` |
| ADC1 | DMA1_Stream3 / ADC1，半字对齐 | CIRCULAR | LOW | `adc.c:182-191` |

NVIC：

| 中断 | 优先级 | 来源 |
| --- | ---: | --- |
| EXTI0（CAM_TRIG） | 5 | `gpio.c:111` |
| EXTI2（SHOOT_DETECT） | 5 | `gpio.c:114` |
| DMA1_Stream0 / 1 | 6 / 6 | `dma.c:47-51` |
| DMA1_Stream2 / 3 / 4 / 5 / 6 | 5 | `dma.c:53-66` |
| USART1 | 5 | `usart.c:404` |
| UART5 | 5 | `usart.c:274` |
| UART7 | 5 | `usart.c:330` |
| USART10 | 5 | `usart.c:448` |
| SPI2 | 5 | `spi.c:156` |
| FDCAN1_IT0 | 6 | `fdcan.c:206` |
| FDCAN2_IT0 | 5 | `fdcan.c:236` |
| FDCAN3_IT0 | 5 | `fdcan.c:266` |
| TIM6_DAC（HAL 时基） | 15 | `stm32h7xx_hal_conf.h:169` |
| TIM24 | 由 `HAL_TIM_Base_Start_IT` 使用，优先级未在源码单独设置（默认 0） | `bsp_tim24.c:10`；`stm32h7xx_it.c:448` |

FreeRTOS 边界：`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`、最低 15（`Core/Inc/FreeRTOSConfig.h:104`、`:110`）。FDCAN1 为 6、DMA1_Stream0/1 为 6、TIM6 为 15，均在允许范围；其余 5 恰好在边界上。TIM24 若真为默认 0，则其 ISR 不能调用 FromISR API；当前 TIM24 ISR 只做 `HAL_TIM_IRQHandler`→`tim24_timebase_on_overflow()` 自增计数，未调用 RTOS API，因此暂不违规，但属隐患。

`stm32h7xx_it.c` 中的 IRQ 数量：EXTI0、EXTI2、DMA1_Stream0..6、FDCAN1/2/3_IT0、SPI2、USART1、UART5、UART7、USART10、TIM6_DAC、TIM24、OTG_HS（`it.c:182-453`）。FDCAN3 有 IRQ 处理但**没有 `FDCAN3_Config()`**，也没有 FIFO 回调处理 FDCAN3 的数据（见第 11 节）。

---

## 9. 通信外设实际参数

### 9.1 UART

| 实例 | 波特率 | 字长 | 校验 | 停止位 | 过采样 | 来源 |
| --- | ---: | --- | --- | ---: | --- | --- |
| USART1 | 115200 | 8B | NONE | 1 | 16 | `usart.c:134-144` |
| UART5 | 100000 | **9B** | EVEN | **2** | 16 | `usart.c:47-57` |
| UART7 | 921600 | 8B | NONE | 1 | 16 | `usart.c:90-100` |
| USART10 | 115200 | 8B | NONE | 1 | 16 | `usart.c:178-188` |

四个口都 `HAL_UARTEx_DisableFifoMode`（FIFO 关闭）、Tx/Rx FIFO 阈值 1/8。UART5 的 9B+偶校验+2 stop 是当前遥控输入（交接文档 1.4 一致）。

### 9.2 FDCAN（classic）

| 实例 | NominalPrescaler | SJW | Seg1 | Seg2 | Data 段 | StdFilters | FIFO | 来源 |
| --- | ---: | ---: | ---: | ---: | --- | ---: | --- | --- |
| FDCAN1 | 4 | 10 | 19 | 5 | P=3,SJW=10,Seg1=29,Seg2=10 | 1 | FIFO0×8 | `fdcan.c:42-69` |
| FDCAN2 | 5 | 1 | 14 | 5 | P=5,SJW=1,Seg1=14,Seg2=5 | 5 | FIFO0×8 + FIFO1×8 | `fdcan.c:90-117` |
| FDCAN3 | 5 | 1 | 14 | 5 | 同上 | 5 std + 5 ext | FIFO0×8 + FIFO1×8 | `fdcan.c:138-165` |

位率（推导）：FDCAN1 = 100 MHz/4/(1+19+5) = 1 Mbit/s；FDCAN2/3 = 100 MHz/5/(1+14+5) = 1 Mbit/s。FDCAN1 的 `ProtocolException=ENABLE`，FDCAN2/3 为 DISABLE（`fdcan.c:47`、`:95`、`:143`）。MessageRAMOffset：0 / 854 / 1709（`fdcan.c:56`、`:104`、`:152`）。

过滤器与路由（`User/Bsp/can_bsp.c`）：

- FDCAN1：Mask=0 全通过 → RX FIFO0，global filter 拒绝扩展/远程帧，`FDCAN_IT_RX_FIFO0_NEW_MESSAGE`（`:353-385`）。
- FDCAN2：全通过 → **RX FIFO1**，`FDCAN_IT_RX_FIFO1_NEW_MESSAGE`（`:392-423`）。
- FDCAN3：没有 Config 函数、没有启动、没有过滤器配置。`FDCAN3_IT0_IRQHandler` 存在（`it.c:434`），但 `HAL_FDCAN_RxFifo0Callback/Fifo1Callback` 只处理 FDCAN1/FDCAN2（`can_bsp.c:500`、`:565`）。

CAN ID 表（`can_bsp.h:34-76`）：`CAN_GIMBAL_ALL_ID=0x1FF`、`CAN_YAW_MOTOR_ID=0x205`、`CAN_TRIGGER_MOTOR_ID=0x207`（旧 DJI Pitch 0x206 已退出）；`CAN_3508_ALL_ID=0x200`、`CAN_3508_M1..M4=0x201..0x204`；`CAN_FRICTION_ID=0x200`、`CAN_FRIC1/2=0x201/0x202`；DM 反馈 `0x51..0x58`（Pitch 用 `0x53`，索引 2）。

### 9.3 SPI2 / ADC / OCTOSPI

| 项目 | 值 | 来源 |
| --- | --- | --- |
| SPI2 | MASTER、2LINES、8 bit、CPOL=HIGH、CPHA=2EDGE、NSS SOFT、预分频 32、MSB、CRC 关、NSSPulse=ENABLE | `spi.c:42-63` |
| ADC1 | 16 bit、非扫描、连续转换、1 通道 `ADC_CHANNEL_4`（PC4？注释未标引脚）、采样 1.5 cycle | `adc.c:48-80` |
| ADC3 | 12 bit、非扫描、单次、`ADC_CHANNEL_TEMPSENSOR`、640.5 cycle | `adc.c:110-137` |
| ADC1 启动 | `HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_val, 2)` | `main.c:179` |
| OCTOSPI2 | FIFO 阈值 8、MemoryType=MICRON、DeviceSize=23（2^23=8 MB）、ChipSelectHighTime=1、ClockPrescaler=3、SampleShifting=HALFCYCLE | `octospi.c:42-52` |
| W25Q 驱动使用 | `flash_init()`→`OSPI_W25Qxx_Init()`；扇区 4 KB、页 256 B、基址 0 | `bsp_flash.c:3-11` |

### 9.4 TIM（PWM）

| 实例 | PSC | ARR | 频率（推导） | 通道/引脚 | 消费者 | 来源 |
| --- | ---: | ---: | --- | --- | --- | --- |
| TIM1 | 0 | 65535 | 240M/65536 ≈ 3662 Hz | CH1 PA8 | `unknown` | `tim.c:49-55`、`:370-372` |
| TIM2 | 167 | 19999 | 240M/168/20000 = 71.4 Hz | CH1 PA15 | 弹舱 `Bombbay_on/off`（500/1300） | `tim.c:123-128`、`:391-393`；`bsp_PWM.c:23-32` |
| TIM3 | 23 | 9999 | 240M/24/10000 = 1 kHz | CH4 PB1 | IMU 加热，`CCR4=0..500` | `tim.c:176-181`、`:412-414`；`INS_task.c:59-71` |
| TIM12 | 167 | 65535 | 240M/168/65536 ≈ 21.8 Hz（蜂鸣器由 PSC 动态改） | CH2 PB15 | 蜂鸣器 | `tim.c:220-225`、`:433-435`；`bsp_buzzer.c:17-18` |
| TIM24 | 239 | 0xFFFFFFFF | 1 MHz 自由运行计数 | — | 时间戳 | `tim.c:264-269`；`bsp_tim24.c` |

PWM 启动：TIM3 CH4 在 `INS_task` 里 `HAL_TIM_PWM_Start`（`INS_task.c:76`）；TIM2/TIM12 的启动由 CubeMX `HAL_TIM_MspPostInit` 完成（`tim.c` 对应 MspPostInit 段）；TIM1 CH1 也启动但无写 CCR 的消费者。

### 9.5 USB

| 项目 | 值 | 来源 |
| --- | --- | --- |
| 实例 | USB OTG_HS，`DEVICE_HS`，CDC | `USB_DEVICE/App/usb_device.c:64-84` |
| 接收回调 | `CDC_Receive_HS` → `uproto_on_rx_bytes(&proto_ctx, Buf, *Len)` 后重挂接收 | `usbd_cdc_if.c:275-283` |
| 发送 | `CDC_Transmit_HS` | `usbd_cdc_if.c:295` |
| printf 重定向 | `fputc` → `HAL_UART_Transmit_IT(&huart10, ...)`（**不是 USB**） | `Core/Src/main.c:93-97` |

---

## 10. HAL 回调所有者

| 回调 | 定义位置 | 作用 |
| --- | --- | --- |
| `HAL_UARTEx_RxEventCallback` | `User/Bsp/bsp_usart.c:149-170` | USART1→解底盘 28 字节；UART5→遥控；两边都重挂接收 |
| `HAL_UART_ErrorCallback` | `bsp_usart.c:173-190` | USART1 重启到 `usart1_buf[0]`/512；UART5 重启；USART10 交 TFmini |
| `HAL_UART_RxCpltCallback` | `bsp_usart.c:139-146` | 仅 USART10→`tfmini_uart_rx_cplt_callback` |
| `HAL_FDCAN_RxFifo0Callback` | `User/Bsp/can_bsp.c:496-553` | FDCAN1：DM 0x51..0x58、DJI 0x201..0x207 |
| `HAL_FDCAN_RxFifo1Callback` | `can_bsp.c:561-617` | FDCAN2：同上 |
| `HAL_GPIO_EXTI_Callback` | `Core/Src/stm32h7xx_it.c:480-489` | 仅 CAM_TRIG→`comm_camera_trigger_pulse()` |
| `HAL_TIM_PeriodElapsedCallback` | `Core/Src/main.c:332-346` | TIM6→`HAL_IncTick`；TIM24→`tim24_timebase_on_overflow` |
| `CDC_Receive_HS` | `USB_DEVICE/App/usbd_cdc_if.c:275` | 交 uproto 解析 |
| `HAL_ADC_ConvCpltCallback` | 未见实现 | `unknown` |

---

## 11. RTOS 配置与任务

`Core/Inc/FreeRTOSConfig.h`：`configENABLE_FPU=1`（`:55`）、`configENABLE_MPU=0`（`:56`）、`configUSE_PREEMPTION=1`（`:58`）、`configSUPPORT_STATIC_ALLOCATION=0`（`:59`）、`configSUPPORT_DYNAMIC_ALLOCATION=1`（`:60`）、`configTICK_RATE_HZ=1000`（`:64`）、`configMAX_PRIORITIES=7`（`:65`）、`configMINIMAL_STACK_SIZE=128`（`:66`）、`configTOTAL_HEAP_SIZE=30720`（`:67`）、`INCLUDE_vTaskDelayUntil=0`（`:90`）、`configTIMER_TASK_STACK_DEPTH=256`（`:136`）。

任务（`Core/Src/freertos.c:133-159`）：

| 任务 | 入口 | 优先级 | 栈参数 | 周期 | 来源 |
| --- | --- | --- | ---: | --- | --- |
| defaultTask | `StartDefaultTask` | Normal | 128 | `osDelay(1)`；开头 `MX_USB_DEVICE_Init()` | `freertos.c:133-134`、`:174-185` |
| gimbalTask | `gimbal_task` | High | 1024 | 初始 200 tick；循环 `vTaskDelay(1)` | `freertos.c:141-142`；`User/APP/gimbal_task.c:111`、`:131` |
| cali | `calibrate_task` | Normal | 512 | 循环 `osDelay(1)` | `freertos.c:144-145`；`User/APP/calibrate_task.c:491`；`calibrate_task.h:139` |
| auto_aim | `auto_aim_task` | Normal | 256 | 初始 500 ms；循环 `osDelay(1)` | `freertos.c:148-149`；`auto_aim.c:268`、`:290`；`auto_aim.h:13-15` |
| COMM_APP | `comm_app_task` | BelowNormal | 640 | `osDelay(1)` | `freertos.c:151-152`；`comm_app_config.h:20-28` |
| message | `message_task` | Normal | 128 | 初始 500 tick；循环 3 tick | `freertos.c:155-156`；`message_task.c:96`、`:99`；`message_task.h:8-9` |
| imuTask | `INS_task` | Realtime | 1024 | 循环 `osDelay(1)`（无初始延时） | `freertos.c:158-159`；`INS_task.c:73-174` |

没有 `detect_task` 任务（`User/APP/detect_task.c` 被编译进工程，但 `MX_FREERTOS_Init` 未创建它）。`osThreadDef` 的 stacksize 直接作为 `xTaskCreate` 的栈深度（同 C 仓的 cmsis_os.c 实现），单位是 word。

---

## 12. 统计与编码

| 指标 | 值 |
| --- | ---: |
| 组数 | 19 |
| Keil 条目 | 109 |
| .c / .s / .lib / .h | 105 / 1 / 1 / 2（`Infantry_robot.h`、`robot_param.h`） |
| 有效编译单元（.c+.s） | 106 |
| FilePath 缺失 | 0 |
| 重复 FilePath | 0 |
| 仓库内未登记的 `.c` | 仅 `User/Communication/core/CRC8_CRC16.c`（与 `User/Controller/CRC8_CRC16.c` 内容重复，工程只编译 Controller 版本） |

编码：`User/APP/video_transmission_module.h` 不是合法 UTF-8（GBK）；`User/Communication/channel/gimbal/gimbal_channel.c` 是合法 UTF-8，但注释已是乱码（`浜戝彴閫氶亾...`），属已发生的二次乱码，不能靠转码恢复。其余源文件为 UTF-8/ASCII。

补充：`User/Bsp/bsp_PWM.c` 顶部声明了 `extern TIM_HandleTypeDef htim4;`（`:22`）但函数实际用 `htim2`；`htim4` 在本工程不存在（`tim.c` 只有 htim1/2/3/12/24），是悬空 extern。

---

## 13. 无法从源码判定（unknown）

1. FreeRTOS 用 `ARM_CM4F` 移植跑 Cortex-M7：功能上是否被验证过、FPU 上下文保存是否完整：`unknown`（本轮不迁移）。
2. D-cache/MPU 与实际 DMA 缓冲一致性：`unknown`（未开/关 D-cache 的证据，需 MAP 与实测）。
3. `TIM1 CH1(PA8)`、`SHOOT_DETECT(PA2)`、`UART7`（921600 21 字节 DMA）、`USART10 TX`（VOFA 调试）的物理用途：`unknown`。
4. `FDCAN3` 是否接了电机、为何初始化但不启动：`unknown`。
5. 裁判系统数据来源：`referee_handle_data()` **没有任何调用者**，`robot_status` 等结构恒为 0；G 上也没有裁判串口接收路径。`unknown`（当前板上不存在可用的裁判数据源）。
6. `gimbal_mailbox_get()` 没有调用者：`unknown`（旁路未接通）。
7. `detect_task` 未创建、`detect_hook()` 无调用者：健康监测链整体未运行（见 P04 文档）。
8. W25Q 实际容量/型号与 `DeviceSize=23` 是否匹配：`unknown`（无设备读取）。

---

## 14. 与交接文档 v1.0 的差异（G 侧配置）

| # | 原陈述 | 当前事实 | 证据 | 影响 |
| --- | --- | --- | --- | --- |
| G-D1 | 1.1：“优化字段 Cads/Optim=1，oTime=0，SplitLS=0” | 目标级如此，但 FreeRTOS 组、USB App/Target/Lib 组以及 Core 9 个文件、HAL 13 个文件为 `Optim=0`；上述组 `wLevel=4` | uvprojx:318-320、`:2044/:2053`、`:2688/:2697`、`:2910-2945`、`:3040-3075`；文件级 `:422..:1963` | B01 记录编译单元级优化档，否则差异比较失真 |
| G-D2 | 1.1：“C 与 G 同左（编译器）” | 属实；但 G 的 `uGnu=1`、`v6Lang=3`，C 为 `uGnu=0`、`v6Lang=1` | G uvprojx:331/333；C uvprojx:330/332 | 两端编译选项不同，跨板“同源码同结果”的假设不成立 |
| G-D3 | 1.1：“G 使用 Mahony 源码及 arm_cortexM7lfdp_math.lib” | 属实 | uvprojx:2660-2662；`User/Algorithm/mahony/mahony_filter.c` | 无 |
| G-D4 | 1.4 表：“G FDCAN3 初始化参数同 FDCAN2 的 nominal timing；main 初始化，但未见与 1/2 相同的启动配置调用” | 属实；补充：FDCAN3 的 NVIC 已使能、ISR 存在，但无 Config/无过滤器/无 FIFO 回调，等于收不到任何帧 | `fdcan.c:128-166`；`it.c:434`；`can_bsp.c:496/561` 只处理 1/2 | P04 的“验证 CAN 帧所属总线”在 FDCAN3 上无从谈起 |
| G-D5 | 1.4：“G USB HS CDC：defaultTask 和 COMM_APP 均有初始化调用，B02 统一为单一所有者” | 属实，两处均在：`freertos.c:177` 与 `comm_app.c:288` | 同左 | P08 |
| G-D6 | 1.4：“G USART10 RX 归 TFmini；TX 有 VOFA 调试输出” | 属实；补充 `fputc` 也重定向到 USART10（`main.c:93-97`），VOFA 与 printf 共用同一 TX，存在竞争 | `comm_app.c:291`、`:233-250`；`bsp_vofa+.c`；`main.c:93-97` | P08 的“打印不能混入协议字节流”在 USART10 上要按现状审计 |
| G-D7 | 1.3 表：“G 没有创建 detect_task” | 属实，且更强：`detect_hook()` 也没有任何调用者，`error_list[]` 是 BSS 零初始化，`toe_is_error()` 恒为 false | `detect_task.c:72`、`:192-195`、`:207`；全仓 grep 无 `detect_hook(` 调用；`freertos.c:133-159` 无 detect 任务 | P04 的“不要直接启动旧 detect_task，先使消费者接入”必须按“现在所有 toe_is_error 都是空判”的前提做 |
| G-D8 | 1.1：“G 的 git status --short 为空” | 当前有 `AI_Workspace/tools/build.ps1`、`snapshot.py` 修改与新增 `capture_evidence.py`（本轮工具修复） | `git -C G status --short` | B00 恢复包要包含工具改动 |
| G-D9 | 1.2 表：“G 主 PLL M=2,N=40,P=1,Q=2,R=2；AHB=SYSCLK/2；APB1/2/3/4 均 HCLK/2” | 与源码一致 | `main.c:231-255` | 无 |
| G-D10 | 1.2 表：“Keil Cpu CLOCK 元数据 12 MHz” | 属实，与 HSE_VALUE=24 MHz 冲突 | uvprojx:22；`stm32h7xx_hal_conf.h:110` | 待核对，不要据此改晶振 |
| G-D11 | 1.3 表：“G configTOTAL_HEAP_SIZE=30720、configSUPPORT_STATIC_ALLOCATION=0、configENABLE_FPU=1” | 与源码一致 | `FreeRTOSConfig.h:55/59/67` | 无 |
| G-D12 | 1.1：“G 保留原设置，不擅自解释为另一个编译器的 -O 值” | 同意；补充组级 `wLevel=4` 只出现在 FreeRTOS/USB 组，含义未在源码注释说明，`unknown`（不猜测） | uvprojx:2053 等 | 报告里只写字段值，不解释语义 |

差异共 12 条；影响 B02/B03 的是 G-D1、G-D2、G-D4、G-D7。
