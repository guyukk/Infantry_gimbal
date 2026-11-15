# 现车校准与存储手册

B05实现版，2026-11-15，证据批次20261115-175509；取代原草稿。旧稿在本板AI_Workspace/artifacts/B05/20261115-175509/before-documents.zip。构建与离线验证见[B05报告](B05-实施与软件签收报告.md)。本次未烧录、未读写设备Flash、未实际校准。

## 1. 工程与可用入口

| 板 | 根目录 | 正式工程/Target | 能力 |
| --- | --- | --- | --- |
| C | D:\文档\Printk\bubing\chassis\Infantry_chassis | MDK-ARM/standard_tpye_c.uvprojx / standard_tpye_c | 舵向零偏宏；启动只读HEAD/GYR |
| G | D:\文档\Printk\bubing\gimbal\Infantry_gimbal | MDK-ARM/CtrlBoard-H7_IMU.uvprojx / CtrlBoard-H7_IMU | 历史Yaw加载；Watch驱动Pitch两点 |

源码路径按C/G前缀区分。控制增益、正常限幅、零位、任务周期、Flash位置与格式保持；没有旧摇杆自动撞限位或陀螺交互标定入口。机械参考、实际轮位、角度正向、温控极性均待设备确认。

## 2. C四轮舵向零偏

入口C/application/robot_param.h的CHASSIS_6020_INIT_ANGLE_0..3；启动chassis_wheel_angle_offset_init写wheel_angle_offset.initial/now，单位rad，本轮不改数值。

| 索引 | 软件轮位别名 | 3508 CAN1 / 6020 CAN2反馈ID | 现有零偏rad | 实际安装 |
| --- | --- | --- | --- | --- |
| 0 | WHEEL_RB，软件右后 | 0x201 / 0x205 | 2.691592450 | 待接线/设备确认 |
| 1 | WHEEL_RF，软件右前 | 0x202 / 0x206 | 0.028407477 | 同上 |
| 2 | WHEEL_LF，软件左前 | 0x203 / 0x207 | 0.389999986 | 同上 |
| 3 | WHEEL_LB，软件左后 | 0x204 / 0x208 | -2.73318529 | 同上 |

软件索引图（车头朝上，不是已核验接线图）：

~~~
             车头 / +x
        LF i=2       RF i=1
        LB i=3       RB i=0
~~~

逆解旁“1号电机左前”等旧注释不能代替上表宏映射。实际车头/轮位/驱动正向必须现场标记。拟重新标定时另行开展设备操作：

1. 保存原宏、固件和轮位/总线ID记录，抬轮、卸载危险机构并确认输出被抑制；按机械参考把滚动方向对齐车体前向，不能自动堵转撞止点。
2. 读取对应6020原始ecd（0..8191），静止多次采样。跨零用圆周均值或先解包，不能直接平均8190与2。
3. 按当前消费者换算：q=wrap(ecd / GM6020_Angle_Ratio)，当前系数1303.63813886f，约8191/(2π)，不能套用常见8192公式。
4. 逆解target=wrap(atan2(vy_i,vx_i)-offset_i+offset)，全局offset当前0；传入逆解的车体纯前进vx_i>0、vy_i=0时atan2=0（不一定等于按W，因为前级有坐标旋转），所以候选offset_i=wrap(-q)，不是直接q。换机械参考时重新从同一目标方程推导。软转向可翻轮180°并反转驱动速度，需要设备验证低速滚动方向。
5. 在隔离副本更新候选，保存ecd→q→offset计算记录，运行软转向/协议/构建回归；确认后再安排烧录、逐轮低速、直行、转向和失联恢复。当前数值保持原值。

本轮不提供未经设备确认的新零偏。

## 3. C内部Flash：只读旧记录

基址0x080A0000（sector9），地址单位byte、read的len单位32位word；总120B，无CRC。

| 记录 | 数据偏移/字节数 | 名称3B+标志1B | 当前消费者 |
| --- | --- | --- | --- |
| HEAD | 0 / 8 | 8..11，标志第11字节0x55 | 原始温度；有效纬度 |
| GM | 12 / 20 | 32..35 | 保留占位 |
| GYR | 36 / 24 | 60..63 | INS_set_cali_gyro的offset[3] |
| ACC | 64 / 24 | 88..91 | 未应用 |
| MAG | 92 / 24 | 116..119 | 未应用 |

HEAD字段offset：self_id=0，firmware_version=1，temperature=3，latitude=4；GYR offset[3]在0..11，scale[3]在12..23。GM不能删除，否则GYR起点36改变。当前ARMCC探针和编译期断言已固定这些尺寸/偏移。

- 纬度要求标志0x55、有限、[-90,90]度；否则回退22.0f；NULL输出安全返回。
- GYR要求标志0x55；三个实际使用的offset有限，绝对值<=32768×BMI088_GYRO_2000_SEN（约34.91rad/s，来自当前±2000°/s配置）。失败不调用INS_set_cali_gyro。
- scale当前被INS_set_cali_gyro忽略，字节保留，不给未消费值编造约束或启用新比例标定。
- **温度仍返回原始int8，即便HEAD无效。** 缺少硬件工作范围与安全回退定义，本轮不发明默认温度、不修改温控。纬度/GYR通过不证明温控目标可信。
- 没有交互写入口，也没有启动校准任务。INS_cali_gyro存在但没有活动入口；旧摇杆说明不适用于当前车。

设备只读自检应另行经操作指示后读取120B、保存bin/SHA256并逐字段核验；不擦sector9、不手改有效标志。本轮仅使用合成字节，不是设备读回。可靠重建HEAD/GYR工具仍缺失。

C BMI088安装矩阵为{0,1,0;-1,0,0;0,0,1}，AHRS.lib/积分常量与温控PID未改；温控目标仍来自上述原始温度，不能据本手册宣称已实测安全。

## 4. G外部Flash旧HD/GM

地址是W25Q64的byte偏移，不是H723内部0x080xxxxx地址。HD在0、GM在12，前36B与上表对应；名称HD\0、GM\0，标志0x55。读写所有者原本HEAD=8B；USB先包含出现12B属于头文件ABI不一致，本轮固定到所有者8B，未迁移成12B记录。

HEAD纬度有限且[-90,90]。不合法时既有HEAD钩子可生成默认元数据（SELF_ID、固件版本、纬度22、由MCU温度推导并上限45的字段）；它不是现车G加热目标，也不能据此定义C安全温度范围。

Yaw旧GM要求标志/名称正确、yaw_offset<8192、min/max有限且-π<=min<max<=π；只应用Yaw零偏和相对范围。旧Pitch字段保留但不参与DM控制，其NaN不连带拒绝合法Yaw。非法Yaw不应用，也不妨碍0x1000处独立合法Pitch记录。没有新Yaw采样入口，旧自动撞限位保持禁用。

HD/GM更新：先读完整4KB sector0到静态缓冲，替换前36B，一次擦除/写入，再分块回读比较。保留36..4095原字节；不再清36..107或额外先擦一次；sector1不受影响。备份读取失败不擦写；写/回读失败时调用方不把RAM cali_done当持久化成功。

Watch观测cali_storage_status：

| 字段/值 | 含义 |
| --- | --- |
| invalid_head / invalid_yaw | 拒绝的旧记录计数 |
| read_failures / write_failures / verify_failures | 读取、写入、回读失败计数 |
| last_result=0 | 尚无相关读写结果，不证明初始化成功 |
| last_result=1 | 本次sector0写入且回读一致 |
| last_result=-1/-2/-3 | 不可用或读失败 / 写失败 / 回读失败 |
| calibration_flash_ready=0 | 初始化或旧记录读取不可用，可在调试器观测 |

活动Flash接口检查NULL、字数溢出和8MB范围，零长度不访问/不擦除。flash_write_single_address会擦覆盖扇区，不自动保留范围外字节；sector0由上述RMW保护，Pitch扇区仍专用。旧未使用void擦除API仅保留有界检查，活动写入使用带返回值的接口。

HD/GM无CRC，标志/值域/回读不能检测所有合法数值位翻转；单扇区更新不是掉电原子操作，不保证掉电保留旧记录。本轮不迁移双槽/版本。

## 5. G Pitch两点安全范围

### 5.1 记录和角度

外部地址0x1000，sector1，v1共20B：

| offset | 字段 |
| --- | --- |
| 0 / 4 | magic=0x50444D31 / version=1 |
| 8 / 12 | min_pos / max_pos，float，输出轴raw rad |
| 16 | 前16B CRC32，反射0xEDB88320，初值/末异或0xFFFFFFFF |

端点有限且严格在(-12.5,12.5)内，跨度0.1..1.5rad。当前PITCH_TURN=0、PITCH_DM_ZERO_OFFSET_RAD=0：relative=raw-zero_offset，raw=relative+zero_offset。若以后反向必须配套正反变换，本轮未改。两点只定安全区间，不等于机械水平零位；Yaw投影开关仍0。

无有效记录时stored_valid=0，[-2.5,-2.0]只是初始化观察值，不是机械验收限位。输出零刚度/阻尼/力矩，机构必须有支撑；保存失败保留旧运行限位，不证明旧Flash仍完好。

### 5.2 操作前置

另行批准设备操作后，拆弹、抬轮、准备急停/机构支撑并备份记录。DM走G FDCAN2，发送0x53，反馈低半字节id3；state1或当前0x53/id3/state5窄兼容允许采样，不能扩大成任意state5正常。

**MOTIONLESS中档可能继续MIT位置保持，不是可靠无力入口。** 手动采样前必须抑制最终输出并确认KP/KD/TOR=0。例如保持Pitch反馈在线，让RC失联触发B03保护并等待健康失效，再检查最终命令；采样接口不要求RC在线。机械效果、电机自身超时仍须先实测，不能仅凭软件变量手搬受力机构。

已有合法范围时仅在既有已确认安全范围内操作，不能绕限位；首次无记录由操作者在机械行程内留足余量选两点，不碰止点。CPU须继续运行，Watch写request由任务消费，停核时不会完成。

### 5.3 request与状态

1. 确认HEALTH_PITCH新鲜有效<=100ms（tick0也合法），pos/vel有限、状态允许。
2. 到第一安全端点并静止（|vel|<=0.2rad/s），request=1；等待request清0、result1、captured bit0置位，记录first_pos。
3. 到另一安全端点静止，request=2；等待bit1置位，记录second_pos。两点都为负角也可合法，异号不是安全证明。
4. 回到两点之间并静止，request=3；要求两点齐全、跨度和值域合法、当前在区间且速度<=0.2。写入/逐字节回读一致后再次检查反馈，再应用范围与当前目标。
5. request=4只取消未提交采样、清captured/result，不擦已保存记录，不撤销已经提交pending，不改机械零位。

stored_valid表示有被接受的运行范围，min_pos/max_pos描述当前运行范围；saved_pending=1表示新记录已存储尚未应用。result不是机械验收证明。

| result | 含义 |
| --- | --- |
| 0 / 1 / 2 | 空闲或取消 / 已采点（查看captured） / 已保存并应用（pending恢复完成也更新2） |
| -1 | 反馈失效、状态不可用或pos/vel非有限 |
| -2 | 捕获点时移动过快 |
| -3 | 命令非法或两点不齐 |
| -4 | 区间不安全、当前不在区间内或提交时移动过快 |
| -5 | Flash初始化/写/回读失败，旧运行限位保持 |
| -6 | 已保存，写后反馈过期/移动/不在区间；saved_pending=1，旧运行范围保持 |

-6不能写成保存失败或校准完成。保持输出抑制、恢复新鲜安全反馈；1000ms恢复链重读CRC记录，满足安全范围条件后应用并清pending/result2。无旧范围时stored_valid仍0；有旧范围时它仍指旧范围，不能误认为新范围生效。pending未解除前不要恢复运动。

启动/运行恢复保留已存下限外0.02rad以内的向内回收；超窗口或高于上限不放行。MIT首目标从当前反馈开始，正常步长上限0.002rad，下限外恢复0.0005rad且仅向内，未拓宽限制。

### 5.4 保存、回读与重启验收

记录板号/固件SHA、两点和当前角、vel/反馈年龄、captured/result/stored_valid/pending及写前/写后20B、CRC。result2且pending0才完成软件应用；再安排重启回读、首目标与机械区间验证。回读只能证明当时字节一致，不能证明掉电持久性/方向/机械余量。擦写间掉电可能丢v1，无记录回零刚度；双槽v2未实施。

## 6. G IMU与剩余能力

BMI088_init(&hspi2,0)使用固定参数：Gx/Gy/Gz偏置-0.000681414269、-0.00134240754、-0.00143384014rad/s，Ax/Ay/Az为0.299675316、0.0720675737、0；gNORM/变换仍按驱动配置。固定分支TempWhenCali=40，G加热目标来自它而非HEAD。Mahony前1000次有效循环采Z偏置，约3000次有效循环后INS就绪；循环计数不等于实测精确毫秒，无效SPI批次会延迟就绪。

未注册旧cali_gyro_hook在B05警告修复中移除；仍没有陀螺交互/持久化标定入口。G温控方向/残留PWM、有效极性和双摩擦轮共享PID属于B04，本轮未修；C记录重建工具、Yaw安全重采、双槽方案也未凭空实现。

## 7. 交付记录模板

设备操作后需提交：板号/电机ID/Target/AXF SHA/日期操作者；机械基准和正向证据；C120B、G sector0及Pitch20B原始bin/SHA；每轮ecd→零偏计算或Pitch两点/当前pos/vel/年龄；request/result/pending；写入与应用分开记录；重启回读、首目标、限位和断线恢复结果；通过/失败/未测项。

本次设备项全部not_run。没有修改机械零位，也没有烧录或设备Flash写入。软件构建、合成记录、真实代码离线执行和字节布局检查不能代替设备签收。
