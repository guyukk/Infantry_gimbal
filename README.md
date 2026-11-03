# H723 舵轮步兵云台

本目录独立保存当前云台板的源码、STM32CubeMX 配置和 Keil 工程。固件不从其他项目目录取文件；板间通信按本目录的协议定义实现。Keil 工程入口为 [CtrlBoard-H7_IMU.uvprojx](MDK-ARM/CtrlBoard-H7_IMU.uvprojx)，Target 为 `CtrlBoard-H7_IMU`，使用 ARM Compiler 5.06 update 7。生成器配置为 [CtrlBoard-H7_IMU.ioc](CtrlBoard-H7_IMU.ioc)。

## 当前固件

`Core/Src/main.c` 初始化时钟、GPIO、DMA、SPI、FDCAN、UART、定时器、ADC、OCTOSPI，然后创建 RTOS 任务。`Core/Src/freertos.c` 创建 `defaultTask`、`gimbalTask`、`cali`、`auto_aim`、`COMM_APP`、`message`、`imuTask`。USB 在 `defaultTask` 中初始化。

`gimbalTask` 控制 Yaw、Pitch 和射击机构；`imuTask` 使用 BMI088/Mahony；`cali` 保留外部 Flash 校准；`message` 通过 USART1 收发 28/46 字节板间帧。`User/Communication/example/device` 虽含 `example` 目录名，实际承担 USB、自瞄、TFmini 和时间同步。当前遥控输入走 UART5；UART7 硬件仍在初始化，其实际接线用途待设备核对。参数选择见 [robot_param.h](User/APP/robot_param.h)。

## 构建与恢复

在本目录运行 `./AI_Workspace/tools/build.ps1` 全量编译。阶段快照、完整日志、逐条警告、MAP、离线验证和按文件恢复方法见 [AI_Workspace 索引](AI_Workspace/README.md)。当前目录没有 Git 仓库，恢复以本地阶段包为准。

最近一次独立项目整理的编译和对照结果见 [阶段报告](AI_Workspace/reports/独立项目收敛-2026-11-02.md)。编译与离线模拟不等于设备验收；尚未烧录或操作实体设备。
