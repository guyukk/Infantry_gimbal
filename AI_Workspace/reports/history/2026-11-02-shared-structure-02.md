# 2026-11-02 结构整理续批与验收记录

本批衔接 [第一批代码简化](simplification/batch-01.md) 和 [2026-10-08 结构整理记录](工程结构优化执行记录.md)。判定依据是当前两个 `.uvprojx`、启动代码、任务创建、回调与编译结果。旧路径和旧说明只保留为历史来源。未烧录或操作实体设备。

## 1. 当前边界与阶段前基线

- 底盘根目录为 `chassis/F4/F4/standard_tpye_c/standard_tpye_c`，Git `main` 在本批开始时干净。云台根目录已迁至 `gimbal/Infantry_gimbal`，该目录没有 Git；共享工具原配置指向已不存在的 `cloud_platform/H7_new/H7_new/DM-balanceV1`。更正配置前先核对现存工程文件；更正后验证旧 `before-structure-01` ZIP 的完整性，并比较当前源码：云台只有 `AGENTS.md` 与旧包不同；底盘另有后续软转向及工程整理变化。结果在 `artifacts/structure-02/preexisting-drift.json`。
- 新基线为 `snapshots/before-structure-02/`：两端源码与工程配置 ZIP、逐文件 SHA-256、Keil 文件清单、AXF、HEX、MAP、完整构建日志、逐条警告、对象与链接移除清单。快照排除 `AI_Workspace` 和 Keil 输出。底盘 293 个文件、111 个 Keil 条目；云台 1392 个文件、110 个条目；工程条目均存在。
- 底盘 Target `standard_tpye_c`，STM32F407IGHx，`USE_HAL_DRIVER,STM32F407xx`；云台 Target `CtrlBoard-H7_IMU`，STM32H723VGTx，`USE_HAL_DRIVER,STM32H723xx`。两端均使用 ARM Compiler 5.06 update 7、DFP 3.1.1。当前 `.ioc` 分别登记 F4 V1.28.3、H7 V1.11.1；本批没有将生成器候选写回。

| 工程 | 阶段前全量构建 | Code | RO | RW | ZI |
| --- | --- | ---: | ---: | ---: | ---: |
| 底盘 | 0 错误、0 警告 | 79128 | 4620 | 980 | 59500 |
| 云台 | 0 错误、40 条已有警告 | 111712 | 4196 | 1152 | 58672 |

大小单位为字节。逐条诊断见 `before-structure-02/gimbal-warnings.csv`；Keil 返回码 1 是本次云台有警告但成功生成镜像的状态。

## 2. 当前保留架构与删减边界

- 底盘 `Src/main.c` 依次初始化时钟、GPIO、DMA、ADC、CAN、I2C、SPI、定时器、串口等，再初始化 USB、创建 RTOS 任务。`Src/freertos.c` 创建 message、INS、LED、chassis、referee、servo、UI 七个任务。CAN、串口及 SPI/DMA/EXTI 路径仍与四个驱动轮、四个舵向电机、惯导、裁判/UI、板间状态及 PM01 电压显示相连。
- 云台 `Core/Src/main.c` 初始化 FDCAN、UART、SPI、ADC、定时器、OCTOSPI 等，`Core/Src/freertos.c` 创建 defaultTask、gimbalTask、cali、auto_aim、COMM_APP、message、imuTask 七个任务。defaultTask 初始化 USB；gimbalTask 同循环控制 Yaw/Pitch 与射击。Pitch 的 DM EMIT/使能、BMI088/Mahony、校准读写、板间 USART1 和 `User/Communication/example/device` 的 USB/自瞄/测距/时间同步链均保留。
- 底盘→云台 28 字节、云台→底盘 46 字节，19 字节 VTM 有效载荷加 7 字节保留区；本批没有变更通信帧、控制参数、周期、故障输出或校准扇区布局。具体旧功能删除与共享类型分离证据见第一批记录。
- 云台 `User/Communication/core/CRC8_CRC16.c` 虽不在 Keil 110 个条目内，仍被 `User/Communication/example/host/CMakeLists.txt` 编入主机示例；它与 `User/Controller/CRC8_CRC16.c` 哈希相同。此文件不是可直接删除的无消费者源码，后续若统一实现，应同时更新并运行主机示例构建。
- 底盘 LED、servo、USB 初始化仍在运行链。本批追到云台 `comm_app_task` 调用 `tfmini_uart_init(&huart10)`，USART10 的接收/错误回调继续交给 TFmini，故它是当前必须保留的测距接口；`CAM_TRIG` EXTI 回调调用 `comm_camera_trigger_pulse`，也是活跃入口。UART7 只有 `RC_TYPE==1` 的图传/备选遥控回调，而当前 `RC_TYPE=0`；初始化和 IRQ 仍存在。UART7、底盘 USB、LED/servo 是否取消尚缺设备用途确认，先保留。云台检测任务未创建、串口收帧边界问题和既有 40 条警告仍按独立缺陷处理。

| 判定 | 对象与源码证据 | 删除前置条件 |
| --- | --- | --- |
| 必须保留 | 云台 USART10/TFmini：`comm_app_task` 初始化，UART 接收及错误回调调用 TFmini；CAM_TRIG EXTI 调用相机触发；底盘 PM01 有 CAN 回调和 UI 电压消费者 | 当前业务仍在使用，不列入纯删减 |
| 可收敛候选 | 两套字节相同的 CRC8/CRC16 C 文件：Keil 使用 `User/Controller`，主机 CMake 使用 `User/Communication/core` | 统一主机和固件引用、运行主机测试及两端通信验证后才移除重复副本 |
| 用途待确认 | 云台 UART7：当前 `RC_TYPE=0`，其图传回调仅在 `RC_TYPE==1`；但启动、DMA 和 IRQ 仍启用 | 确认现车无 UART7 图传/备用遥控和诊断用途，再同步 `.ioc`、MSP、IRQ、Keil 与 HAL |
| 用途待确认 | 底盘 USB：`main` 初始化，CDC 接收只重启收包；底盘 LED 与 servo 有实际任务和输出 | 分别核对现车接口、接线及使用方式；若删任务还需核对初始化、调度和故障输出 |
| 用途待确认 | 云台 SHOOT_DETECT：EXTI2 调用 HAL，但业务 GPIO 回调只处理 CAM_TRIG | 确认该输入没有外部检测用途，再连同引脚、NVIC 和生成配置一起处理 |

另记一项现有接口风险：`auto_aim_task` 每轮调用 `Vofa_JustFloat_AUTO`，其实现经 `huart10` 发出调试帧；同一 USART10 也承担 TFmini 接收。应单独查明现车连线和串口共用是否有意设计，再决定是否限制调试发送，不能在本次结构整理中悄悄改变输出。

## 3. 本批改动

1. 共享工具 `tools/projects.json` 的云台相对路径改为当前 `gimbal/Infantry_gimbal`。底盘根 `README.md`、两端维护入口和工具说明同步当前路径；云台新增根 `README.md`，说明运行任务和共享恢复入口。
2. 云台 `Core/Src/freertos.c` 仅删除重复的 `robot_param.h` include 和多余的内层 `USER CODE BEGIN RTOS_THREADS` 标记及空注释。外层用户区、七个 `osThreadDef`/`osThreadCreate`、初始化顺序及调用均保留。源码清单比较显示阶段前后底盘只变更 `AGENTS.md`，云台只有这个 C 文件变更；新增的云台 README 另存于阶段后包 `docs/`。
3. `tools/cubemx_generate.ps1` 现在要求 Java 返回 0、CLI 出现完成标记且日志没有 `[ERROR]`，并将三个判断写入 `result.json`。单独的生成器退出码 0 不再作为可应用依据。
4. 云台主机示例的旧 `User/Communication/example/host/build` 含 56 个可再生成的 CMake 文件和旧机器路径；逐文件 SHA-256 核对后，整体移至共享 `cache/gimbal-host-build-before-structure-02/`。迁移清单在 `artifacts/structure-02/host-build-migration.json`，源码 `host/CMakeLists.txt` 保留原位。

## 4. CubeMX 隔离验收

隔离副本位于 `cache/cubemx/`。旧底盘候选将 `.ioc` 七个实际任务登记并移除旧 `test_task`/IWDG，但生成的 `freertos.c` 额外出现弱 `message_task` 和 USB 初始化位置变化，不能证明与正式工程等价。

本批再次调用 CubeMX 6.17.0 对云台候选生成，Java/CLI 返回 0，日志却含配置读取超时及 `DEVICE_SERIAL*_CDC_HS` 参数错误。日志还明确提示无法读取 `STMicroelectronics.X-CUBE-FREERTOS.1.6.0` 的 IP mode 文件、缺失 FreeRTOS mode；生成后的嵌套 `.ioc` 从 27 个 IP 减至 26 个，丢失 `FREERTOS`，`main.c` 不再启动内核，`Core/Src/freertos.c` 不存在，业务 `User/` 层也未生成。修正工具判定后复跑，进程仍返回 0，但工具查出 12 条生成器内部错误并正确返回失败。原云台 `.ioc` 记录 CubeMX 6.10.0、CMSIS-RTOS v1；[ST 官方技术答复](https://community.st.com/stm32cubemx-mcus-29/unable-to-generate-freertos-code-after-migration-164098)确认 CubeMX 6.17 的 FreeRTOS 已不支持 CMSIS-RTOS v1。这解释了当前版本组合的核心兼容性问题，但日志中的 USB 参数错误仍需单独核查。该结果未满足任务唯一创建、用户代码保存、再次生成及可编译条件，因此未应用候选 `.ioc` 或生成源码。完整 stdout、stderr、命令、复跑结果和 SHA-256 清单已存入 `after-structure-02/generator/`。后续可在隔离环境使用支持 CMSIS-RTOS v1 的 CubeMX 版本核验，或另立受控任务迁移到 v2；再核对输出目录、全量编译、再次生成及硬件配置差异。不得用 CLI 退出码替代这些验收。

## 5. 阶段后验收与恢复

`snapshots/after-structure-02/` 保存阶段后源码/配置 ZIP、哈希、两端新固件、MAP、完整构建日志、逐条警告和生成器原始证据。快照 ZIP 与逐文件哈希完整，当前源码无差异；从云台目录调用同一验证脚本也得到一致结果，见 `artifacts/structure-02/after-integrity.json` 与 `from-gimbal-cwd.json`。两端全量构建分别仍为 0/0 与 0/40；两端 Code/RO/RW/ZI 逐项与阶段前一致。云台 40 条警告按文件、行号、编号、文本逐条比较，无差异。

从阶段前快照到当前源码，底盘 334 个、云台 483 个保留函数体经 ARMCC 预处理后全部一致，结果在 `artifacts/validation/structure-02/function-comparison.json`。板间帧与校准加载的 ARMCC/Unicorn 离线验证 42 项通过，结果在同目录 `runtime-results.json`；汇总在 `artifacts/structure-02/acceptance.json`。AXF 哈希因重新构建而不同；容量、函数体及逐条警告对照才是本批运行等价的离线证据。上述检查不模拟中断、RTOS 调度、外设或电机。

按文件恢复：从 `before-structure-02/gimbal-sources.zip` 选取 `Core/Src/freertos.c` 可撤销本批源码整理；云台新增 README 的阶段后副本在 `after-structure-02/docs/gimbal-README.md`。工具与底盘文档由 Git 差异恢复。选择文件恢复后核对对应 `*-sha256.json`，再全量编译两端；不要整包覆盖期间新增工作。大型 ZIP、固件和日志只在本地 `snapshots/`，小型报告与工具纳入 Git。

仍需确认的删减范围：底盘 USB 是否为现车诊断接口、LED/servo 实际接线及职责；云台 UART7 备选图传输入和 `SHOOT_DETECT` 输入是否仍接现车设备。确认前保留其初始化、IRQ、驱动和 HAL。USART10/TFmini 与 CAM_TRIG 已有实际消费者，列为必须保留。实体设备验收仍需另行安排。
