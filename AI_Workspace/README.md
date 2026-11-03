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

- [独立项目收敛记录](reports/独立项目收敛-2026-11-02.md)
- [云台架构与边界调查](reports/云台工程简化与结构整理.md)
- [工具与恢复命令](tools/README.md)
- `snapshots/gimbal-local-baseline-2026-11-02/`：本地独立构建的旧基线。
- `snapshots/before-independent-01/`：本次删减前源码、工程配置与文件哈希。
- `snapshots/after-independent-01/`：本次删减后源码、工程配置、固件、MAP、日志与哈希。
- `reports/history/`、`artifacts/history/`：历史时期材料，可能包含旧来源路径，只用于追溯。

恢复单个文件：从指定阶段的 `gimbal-sources.zip` 提取目标路径，以 `gimbal-sha256.json` 核对，再运行 `tools/build.ps1`。大型恢复包只保存在本机；当前目录没有 Git 仓库。
