# 本项目 AI 工作区

本工作区只服务于本目录的 H723 云台工程。全部当前工具只读取本目录工程源码与本机已安装的 Keil/CubeMX 工具。旧阶段资料按当时内容归档，不能作为当前配置依据。

| 目录 | 内容 | 保存方式 |
| --- | --- | --- |
| `reports/` | 当前调查、分批结果及带日期的历史记录 | 小型文本可纳入版本控制 |
| `tools/` | 构建、快照、审计、帧验证和隔离生成脚本 | 源码可纳入版本控制 |
| `snapshots/` | 独立阶段的源码 ZIP、配置、固件、MAP、日志、哈希 | 本地大文件，不覆盖 |
| `artifacts/` | 迁移清单、差异、验证结果 | 小型证据可纳入版本控制 |
| `cache/` | 依赖、临时编译输出及生成器隔离副本 | 可再生成，不入版本控制 |

## 当前索引

- [Pitch 下限外单向回收](reports/Pitch下限外单向回收-2026-11-07.md)：保留两点校准工作下限，对少量低于下限的上电位置只允许向范围内缓慢恢复。
- [Pitch 上电自动到限位中点（已撤销）](reports/Pitch上电自动到限位中点-2026-11-07.md)：保留阶段记录与恢复包；当前固件恢复为上电保持当前位置。
- [Yaw 初始化限幅导致回零](reports/Yaw初始化限幅导致回零-2026-11-07.md)：Flash 保存的是零偏和范围；初始化时通用限幅把目标夹到固定边界，现改为用新鲜反馈锁定当前角。
- [Pitch 状态 5 使能回归与 Yaw 免回零](reports/Pitch状态5使能回归与Yaw免回零-2026-11-07.md)：冷上电补发使能已获用户实机确认；Yaw 的第一阶段修改仍有回零现象，见上方后续报告。
- [DM Pitch 冷启动自动恢复](reports/Pitch冷启动自动恢复-2026-11-07.md)：前一阶段记录；Flash 校准记录恢复仍在使用，使能判定已由上方报告修正。
- [云台初始化与 DM Pitch 清理](reports/云台初始化与DM-Pitch清理-2026-11-07.md)：本轮启动链、旧 DJI Pitch 清理、Flash 校准门控、编译与离线验证；恢复包见 `snapshots/before-gimbal-init-optimization-2026-11-06/` 与 `snapshots/after-gimbal-init-03-final-2026-11-07/`。
- [云台运行链与僵尸代码清理](reports/云台运行链与僵尸代码清理-2026-11-06.md)：有效 Target、七任务、RC/IMU/CAN/USB/板间帧数据链及五批删减证据。
- [删减候选与保留判定](artifacts/cleanup-2026-11-06/candidates.csv)：已删、待确认及现存功能缺口。
- `snapshots/before-full-runtime-cleanup-2026-11-06/`、`snapshots/after-full-runtime-cleanup-01-2026-11-06/` 至 `after-full-runtime-cleanup-05-2026-11-06/`：阶段源码 ZIP、哈希、构建/MAP/警告与最终固件。
- [DM Pitch 安全位置校准](reports/Pitch-DM安全位置校准-2026-11-06.md)：两点手动采样、独立 Flash 记录、Keil Watch 操作和设备验收边界。
- `snapshots/before-pitch-dm-calibration-2026-11-06/`、`snapshots/after-pitch-dm-calibration-2026-11-06-final/`：本次改动前后恢复包；后者包含固件、MAP、日志与哈希。
- [Pitch 反馈状态 5 与 MIT 零力矩](reports/Pitch反馈状态5与MIT零力矩-2026-11-06.md)：记录设备 Watch 根因、`0x53` 状态字节兼容修正、编译与离线验证。
- `snapshots/before-pitch-status53-fix-2026-11-06/`：此次修正前的源码恢复包。
- `snapshots/after-pitch-status53-fix-2026-11-06/`：修正后完整源码、固件、MAP、HEX、日志与哈希。
- [Yaw 抖动与 Pitch 零位耦合修正](reports/Yaw抖动与Pitch零位耦合修正-2026-11-06.md)：临时停用未校零的 Pitch 角在 Yaw 陀螺仪投影中的作用，记录原因、编译与设备复验点。
- `snapshots/before-yaw-pitch-coupling-fix-2026-11-06/`：此次 Yaw 反馈修正前的源码恢复包。
- `snapshots/after-yaw-pitch-coupling-fix-2026-11-06/`：修正后源码、固件、MAP、HEX、日志与哈希。
- [Pitch 切换 MIT 协议](reports/Pitch切换MIT协议-2026-11-06.md)：按电机实测 PMAX/VMAX/TMAX 改为 MIT 命令，记录离线验证与上板观察点。
- `snapshots/before-pitch-mit-2026-11-06/`：本次协议修改前的源码恢复包。
- `snapshots/after-pitch-mit-2026-11-06-final/`：本次协议修改后的源码、固件、MAP、HEX、构建日志与哈希。
- [Pitch 目标角切换修正](reports/Pitch目标角切换修正-2026-11-05.md)：根据云台板 Watch 截图修正活动模式起点，并记录离线验证。
- `snapshots/before-pitch-target-continuity-2026-11-05/`：此次 Pitch 目标角修改前的源码恢复包。
- `snapshots/after-pitch-target-continuity-2026-11-05/`：修正后的源码、固件、MAP、日志与哈希。
- [Pitch 左摇杆上下排查](reports/Pitch左摇杆上下排查-2026-11-03.md)：当前代码链、1 秒使能补发、云台板 Watch 判读。
- `snapshots/before-pitch-rc-investigation-2026-11-03/`：本次时序微调前的源码恢复包。
- `snapshots/after-pitch-rc-investigation-2026-11-03/`：1 秒补发版本的源码、固件、MAP、日志与哈希。
- [Pitch 绿灯无输出修正](reports/Pitch绿灯无输出修正-2026-11-03.md)：撤销反馈对控制帧的门控，保留延迟使能及新增发送状态观察。
- `snapshots/before-pitch-output-fix-2026-11-03/`：绿灯无输出阶段的源码恢复包。
- `snapshots/after-pitch-output-fix-2026-11-03/`：修正后的源码、固件、MAP、日志与哈希。
- [Pitch 电机上电使能重试](reports/Pitch电机上电使能重试-2026-11-03.md)：代码修改、验证结果及上板观察项。
- `snapshots/before-pitch-enable-2026-11-03/`：本次修改前的源码与构建恢复包。
- `snapshots/after-pitch-enable-2026-11-03/`：本次修改后的源码、配置、固件、MAP、构建日志与哈希。
- [独立项目收敛记录](reports/独立项目收敛-2026-11-02.md)
- [云台架构与边界调查](reports/云台工程简化与结构整理.md)
- [工具与恢复命令](tools/README.md)
- `snapshots/gimbal-local-baseline-2026-11-02/`：本地独立构建的旧基线。
- `snapshots/before-independent-01/`：本次删减前源码、工程配置与文件哈希。
- `snapshots/after-independent-01/`：本次删减后源码、工程配置、固件、MAP、日志与哈希。
- `reports/history/`、`artifacts/history/`：历史时期材料，可能包含旧来源路径，只用于追溯。

恢复单个文件：从指定阶段的 `gimbal-sources.zip` 提取目标路径，以 `gimbal-sha256.json` 核对，再运行 `tools/build.ps1`。大型恢复包只保存在本机；本目录已纳入 Git，阶段快照可用于核对和恢复本轮未提交改动。
