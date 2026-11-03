# 项目约定

- 本目录是独立的 STM32H723 步兵云台工程；源码、配置、构建与验证脚本不得依赖其他项目目录。
- 当前唯一设备配置在 `User/APP/robot_param.h`：`Infantry_robot`、`Steering_wheel`、`yaw_pitch_direct`、`friction_3508`、`gimbal_board`。通信对端只按本工程定义的帧格式处理。
- 以 `MDK-ARM/CtrlBoard-H7_IMU.uvprojx` 的有效 Target、启动入口、任务、IRQ/DMA/回调、调用链和 MAP 判定运行职责；旧说明和注释仅作参考。
- 保持现有控制参数、单位、周期、初始化顺序、故障输出、28/46 字节通信帧及校准存储布局。删除前记录证据和恢复文件；跨接口修改后进行完整编译与针对性离线验证。
- 阶段记录、工具和恢复包统一在本目录 `AI_Workspace/`；从 `AI_Workspace/README.md` 查索引。大型快照仅本地保存且不得覆盖。
- 用户已授权使用本机 `D:\keil5\UV4\UV4.exe` 编译验证。烧录或操作实体设备须有另行指示。
- Windows 命令使用 `C:\Program Files\PowerShell\7\pwsh.exe`，日常命令以 `-NoLogo -NoProfile` 启动。
