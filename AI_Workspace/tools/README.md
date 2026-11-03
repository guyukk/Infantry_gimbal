# 云台工程工具

脚本从自身位置向上查找 `MDK-ARM/CtrlBoard-H7_IMU.uvprojx`，读取 `projects.json` 中本工程标识和本机 Keil/CubeMX 可执行文件路径。传给 Python 脚本的相对路径按本项目根目录解析，与启动工作目录无关；它们不读取其他项目的源码。使用 PowerShell 7：

```powershell
./AI_Workspace/tools/build.ps1
& 'C:\Program Files\Python39\python.exe' -B ./AI_Workspace/tools/snapshot.py capture ./AI_Workspace/snapshots/before-next-batch
./AI_Workspace/tools/build.ps1 -LogDirectory AI_Workspace/snapshots/before-next-batch/build
& 'C:\Program Files\Python39\python.exe' -B ./AI_Workspace/tools/audit.py ./AI_Workspace/snapshots/before-next-batch
& 'C:\Program Files\Python39\python.exe' -B ./AI_Workspace/tools/snapshot.py verify ./AI_Workspace/snapshots/before-next-batch --result ./AI_Workspace/artifacts/verify-next.json
```

阶段包不能重名；`snapshot.py capture` 拒绝覆盖。`validate_invariants.py` 将当前保留函数体与给定阶段 ZIP 的 ARMCC 预处理结果比较。`validate_runtime.py` 用本工程实际收发函数、VTM 初始化函数和合成对端帧做 ARMCC/Unicorn 离线检查：

```powershell
& 'C:\Program Files\Python39\python.exe' -B ./AI_Workspace/tools/validate_invariants.py --baseline ./AI_Workspace/snapshots/before-independent-01 --cache ./AI_Workspace/cache/validation/invariants-independent --results ./AI_Workspace/artifacts/validation/invariants-independent
& 'C:\Program Files\Python39\python.exe' -B ./AI_Workspace/tools/validate_runtime.py --cache ./AI_Workspace/cache/validation/runtime-independent --results ./AI_Workspace/artifacts/validation/runtime-independent
& 'C:\Program Files\Python39\python.exe' -B ./AI_Workspace/tools/check_locality.py --result ./AI_Workspace/artifacts/validation/locality.json
& 'C:\Program Files\Python39\python.exe' -B ./AI_Workspace/tools/compare_stages.py ./AI_Workspace/snapshots/before-independent-01 ./AI_Workspace/snapshots/after-independent-01 --result ./AI_Workspace/artifacts/validation/independent-stage-comparison.json
```

旧版 42 项双板模拟结果是历史记录；当前运行工具只验证本项目负责的帧字段、长度、默认值和解析，不能证明对端实现或硬件行为。`cubemx_generate.ps1` 只允许在 `cache/cubemx/` 隔离副本运行。现有 `.ioc` 使用 CMSIS-RTOS v1，安装的 CubeMX 6.17 隔离生成未通过检查；生成结果未应用至正式工程。
