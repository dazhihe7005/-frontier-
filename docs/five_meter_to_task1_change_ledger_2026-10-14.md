# 五米验证代码变更台账与任务一迁移清单

更新日期：2026-10-17

## 2026-10-17 最新真机复核与开机入口切换

最新完整真机样本为
`/home/nuc/flight_logs/five_meter/20261016_202737/`，有效飞行 ULog 为
`px4_20261016T123046Z_id251.ulg`。断电遗留的 `ros_all_0.bag.active` 只在
`/home/nuc/flight_logs/analysis/20261016_202737/` 中复制并重建索引，原始证据未改动。
任务正常进入 OFFBOARD、到达 `GOAL_REACHED_AND_SETTLED`，随后先制动再退出到 POSCTL；
没有非空 `command_fault`。

实飞任务段水平跟踪误差 RMS/最大值为 `0.093/0.192 m`，垂直误差 RMS/最大值为
`0.030/0.059 m`。按飞行高度相关的红色障碍点云复核，SUPER/实飞轨迹最小水平净距为
`1.274/1.172 m`，均高于 `0.8 m` 硬阈值；OFFBOARD 实测三维速度峰值
`0.692 m/s`，低于 `0.8 m/s` 硬上限。横滚/俯仰峰值约 `3.28/2.72°`。Yaw 动态跟踪
误差 RMS/最大值为 `3.80/5.83°`，相对上一完整架次 id249 的 `4.64/10.02°` 有改善，
但仍不是零误差。完整红色点云三维图、水平安全高度带图、Yaw 图和 JSON 摘要保存在
`/home/nuc/flight_logs/analysis/20261016_202737/report/`。

本架次前方没有形成必须绕行的障碍，主要验证了两侧环境中的轨迹跟踪、速度和净距，
不能单独外推为正式任务一长距离、多目标、返航和所有点云退化分支均已完成真机验收。
不过本架次所用的感知门、`config/super_task1.yaml`、SUPER、命令桥及
`0.6/0.7/0.8 m/s`、半径 `0.8 m`/总高 `1.0 m` 圆柱合同与正式任务一完全共用；正式任务一
特有调度与首目标握手也已由上一轮回归和 SITL 覆盖，因此不再复制第二份实现。

NUC 的开机入口已切换：`mine-uav-five-meter.service` 已停止并禁用；仓库中的
`mine-uav-task1.service` 通过 systemd link 注册并设为 `enabled`，下次开机运行
`scripts/start_task1_real.sh`。切换时没有启动正式真机栈；服务仍不会解锁、起飞或自动
开始任务，必须由飞手起飞稳定后产生任意方向的 CH7 稳定电平变化。等待或活动期间再次
改变 CH7 会取消任务。两个等待硬件的启动包装器新增
INT/TERM 处理，避免尚未进入 roslaunch 时停止服务仍拖到 45 秒超时。

最终几何以用户重新测量后的值为准，并覆盖本节早先讨论过的总高 `0.6 m` 方案：圆柱
水平半径 `0.8 m`，以飞机参考中心计向下 `0.5 m`、向上 `0.5 m`，总高 `1.0 m`。
规划后原始障碍逐点复核、实时当前位置保护、任务一高层候选可通行判断均使用该合同；
ROG/A* 前端采用水平 `0.9 m`、上下各 `0.5 m`。CIRI 仍以 `0.3 m` 基础球构造连续走廊，
但任何优化结果都必须再通过精确 `0.8/0.5/0.5 m` 圆柱硬复核，不能绕过最终合同。
表面完整性要求的垂直跨度仍为 `0.6 m`，它是点云证据门，不是圆柱半高。返航近邻
筛查半径保持 `0.8 m`。

当前 Release 构建成功；在正式 Fast-LIO、SUPER 和 `frontier_runtime_ws` 覆盖环境中运行
任务一整包回归为 `276 tests, 0 errors, 0 failures, 0 skipped`，SUPER planner 为
`138/138`、ROG-Map 为 `12/12`。正式 launch 展开确认探索器和实时桥均加载水平
`0.8 m`、向上/向下各 `0.5 m`，速度仍为 `0.6/0.7/0.8 m/s`。

## 2026-10-16 迁移实施结果

用户批准的三部分迁移已经进入正式任务一代码，固定五米目标和五米完成判据没有复制：

- 调度器安全闭环：`mission_scheduler` 直接订阅锁存的
  `/mine_uav/task1/command_fault`；任务一活动期间任一非空故障都会同时撤销
  `goaf_enable` 和 `auto_enable` 并回到 Hold，原因保留为
  `task1_command_fault_<根因>`。CH7 下降沿现在也会取消等待中或活动中的任务一。
- 启动门和两阶段授权：CH7 上升沿只提出启动请求；调度器要求 MAVROS 已连接、PX4
  已解锁、`task1_planning` 里程计和单位旋转对齐有效，并要求三维速度不超过
  `0.25 m/s`、垂直速度不超过 `0.10 m/s` 连续 `1.0 s`。随后仅打开规划门；直到
  决策器在本任务 epoch 先发布零复位、再发布 SUPER 实际接受的首个 goal ID，才打开
  `auto_enable`。等待稳定悬停限时 `15 s`，首目标接受限时 `30 s`。
- 记录链：新增独立 `start_task1_real.sh`、`mine-uav-task1.service` 和只负责安装/下次
  启用的脚本。每次运行建立独立目录，检查至少 `100 GiB` 可用空间，动态且无歧义地
  解析 `/dev/serial/by-id`，保存代码状态、实际配置、ROS 参数/节点/话题和强制拉取的
  PX4 参数；rosbag 使用 LZ4、五分钟分卷和锁存消息重复写入，ULog 在解锁后重新上锁
  时自动下载并保存 SHA-256。日志节点均非 required，不会因记录失败杀死飞行控制链。
- 两个真机入口共用 `/run/lock/mine-uav-flight-stack.lock`，因此五米测试和正式任务一
  不能误启动在同一架飞机上。`task1_real.launch` 要求显式传入 PX4 by-id 设备；生产
  包装器负责提供，不再使用固定 tty 猜测。

当前速度合同未随迁移改变：SUPER 名义规划 `0.6 m/s`、预测闭环治理 `0.7 m/s`、
规划前馈和实测三维速度硬上限 `0.8 m/s`、OFFBOARD/调度器进入速度门 `0.25 m/s`。

自动验证已覆盖：稳定悬停等待、首目标前禁止自动、旧非零 goal ID 不复用、同一进程
连续多任务、两种不同 `command_fault`、CH7 取消、完成、人工切出 OFFBOARD、规划
里程计丢失、Task-2 独立 watchdog，以及共享桥接器的预测速度治理、实测超速、模式接管、
坐标帧和目标生命周期。整包结果为 `273 tests, 0 errors, 0 failures`；编译、launch 展开
和脚本/XML 静态检查也均通过。

迁移后的完整 PX4/Gazebo/Fast-LIO/SUPER Task-1 SITL 进一步确认实际加载
`0.6/0.7/0.8 m/s`：CH7 后先等待稳定悬停，`goaf_enable=true` 时仍保持
`auto_enable=false`；SUPER 接受本轮 goal ID 1 后才变为 true。随后仿真点云表面垂直
跨度只有 `0.472 m`、低于覆盖门 `0.6 m`，桥接器锁存
`OBSTACLE_SURFACE_INCOMPLETE`，调度器立即以
`task1_command_fault_OBSTACLE_SURFACE_INCOMPLETE` 回到 Hold。这验证了启动和故障闭环，
但本轮不是完整探索/返航成功样本。

该结果表示代码迁移和隔离回归完成，不代表已经完成正式任务一长距离、多目标、返航真机验收；仍应按
第8节从 SITL、拆桨到低高度短距离逐级验收。systemd 文件只写入仓库，本轮没有安装、
启用或启动真实飞行服务。

## 2026-10-16 迁移前当前态复核（历史，覆盖下文旧速度结论）

以下六项“尚未迁移”描述是本轮实施前的审计输入，已由上节结果取代，保留用于追溯。

本次按实际 launch 展开结果重新核对五米真机入口与 `task1_real.launch`。下文保留的
`0.2/0.3/0.4 m/s`、任务一桥接器约 `1.1 m/s` 等内容是 2026-10-14 当时的历史状态，
不再代表当前配置。当前两个真机入口实际都加载：SUPER 名义速度 `0.6 m/s`、预测闭环
治理 `0.7 m/s`、规划与实测硬上限 `0.8 m/s`、OFFBOARD 进入门 `0.25 m/s`。五米入口
显式传入这些桥接参数，任务一入口使用同一个 `super_px4_command_bridge.launch` 的相同
默认值；两者还共同加载 `config/super_task1.yaml`。因此旧 P0-4 的“参数尚未移植”问题
已经消失，但 `0.6 m/s` 尚未完成正式任务一长距离、多目标和返航真机验收。

当前无需二次修改、已经自动作用于两个任务的共享层如下：

- `fastlio_px4_vision_bridge` 及其配置：安装角、杆臂、健康检查和 PX4 外部视觉；
- `task1_perception_gate`：`task1_planning` 坐标扶正、任务点云门和对齐保护；
- 同一个 SUPER `fsm_node`、同一个 `config/super_task1.yaml` 以及同一套 SUPER/ROG-Map
  二进制：左右侧绕优先、再上绕、最后下绕，A* 水平 `0.9 m` 前端膨胀，最终水平
  半径 `0.8 m`/总高 `1.0 m` 圆柱硬检查、优化后逐点复核和禁止提前回中；
- `super_px4_command_bridge`：命令 epoch、旧轨迹隔离、速度治理、实测超速、进入检查、
  实时水平半径 `0.8 m`/上下各 `0.5 m` 净距制动、障碍表面覆盖门和故障话题；
- 写入同一真实 PX4 的姿态 PID。飞控参数不区分五米任务和正式任务一，但更换飞控或
  恢复参数后仍须重新核对。

截至本次复核，仍未迁移到正式任务一的内容集中在任务编排和真机运行入口：

1. `mission_scheduler` 不订阅 `/mine_uav/task1/command_fault`。桥接器能够制动退出，但
   OFFBOARD 前故障可能不会让正式任务一上层立即回到 Hold。
2. `mission_scheduler::applyTask()` 仍同时发布 `goaf_enable=true` 和
   `auto_enable=true`；它没有等待本任务首个 `goal_id` 被 SUPER 接受。五米协调器已经
   实现“暖图/发目标/准确 ID 接受/再开放自动权限”的两阶段时序。
3. 正式调度器只把原始 `/Odometry` 当作时间戳心跳，不检查
   `/mine_uav/task1/odometry` 的 `task1_planning` 帧、对齐就绪、已解锁、水平/垂直速度
   或连续稳定悬停时间；五米协调器具备这些启动门。
4. 五米任务中 CH7 下降沿会立即取消；正式调度器在任务运行时会消费并忽略该边沿，
   目前只能依赖 CH5/模式退出等方式接管。这一差异应在正式任务一安全交互设计中明确。
5. `task1_real.launch` 没有五米入口的自动 rosbag、解锁后 ULog 下载、每架次元数据目录、
   100 GiB 空间门和动态 PX4 串口识别；仓库中也没有对应的 `start_task1_real.sh` 或
   systemd unit。
6. 现有 `mission_scheduler_edges` 只验证基本边沿、完成、定位丢失和模式接管，仍缺少
   首目标接受前禁用自动、桥接故障闭环、连续第二任务 epoch、稳定悬停拒绝以及旧目标
   不复用的正式任务一联合回归。

五米专用的固定前方 `5 m` 目标、`60 s` 超时、到点稳定判定和对应配置/节点不应复制到
正式探索；正式任务一继续由 `super_exploration_decider` 负责连续目标、地图闭合和返航。
结论是：共享模块修改已经生效，不需要二次移植；正式任务一目前仍不能仅因五米任务成功
就判定整套任务通过，必须补齐上述调度、启动记录和任务一专用回归。

## 1. 台账范围与基线

本台账记录 `/home/nuc/frontier-upload` 当前工作区相对以下 Git 基线的全部改动，供五米
验证通过后迁移到正式任务一时逐项核对：

```text
baseline commit: db897c4e3a1607f5852b484afd0a01523cb08c6b
baseline date:   2026-10-12 21:56:28 +0800
baseline title:  Integrate NUC path planning and avoidance updates
```

`/home/nuc/frontier_runtime_ws/src/mine_uav_control` 是指向
`/home/nuc/frontier-upload` 的符号链接，两者是同一份源码。

重要说明：当前工作区仍包含未提交修改。本台账是“当前状态说明”，不是 Git 提交，也不应
代替提交前的最终 `git diff` 审核。以后继续改动代码时，应同步更新本文件的日期、参数表、
文件清单和验证结果。

## 2. 促成这些修改的实飞事实

2026-10-14 最近架次中的两次任务不能归为同一种故障：

- 第一次 OFFBOARD 任务中，真实三维速度峰值约 `0.463 m/s`，水平峰值约
  `0.458 m/s`；速度超过 `0.4 m/s` 约 `0.704 s`，桥接器以
  `MEASURED_SPEED_LIMIT` 退出。SUPER 速度前馈并没有明显超限，主要问题是正常运行值
  离 `0.4 m/s` 硬线太近，没有给 PX4 位置误差追赶、惯性和估计延迟留下余量。
- 第二次开始时，预发送位置从当前约 `(-0.40, -1.34, -0.61)` 跳向上一任务旧目标
  `(-0.05, 4.75, -0.56)`，距离约 `6.10 m`。PX4 内部速度设定值达到约
  `5.443 m/s`，OFFBOARD 内实测约 `2.163 m/s`，惯性阶段 ULog 峰值约
  `3.320 m/s`。桥接器报了 `SPEED_GOVERNOR_INFEASIBLE`，但五米任务协调器没有直接
  消费桥接故障，所以任务状态仍继续等待，最终表现为明显超速却没有及时退出任务。

因此修复目标不是关闭超速退出，而是同时满足：

1. 正常飞行的规划值和预测闭环速度远离 `0.4 m/s` 硬线；
2. 上一任务的任何命令都不能进入下一任务的 OFFBOARD 预发送；
3. 真正超速或命令不可治理时，控制桥和上层任务都必须进入安全终止状态。

## 3. 修改后的数据与控制链

```text
MID360 / Fast-LIO 原始 camera_init 数据
        │
        ├─ fastlio_px4_vision_bridge
        │    ├─ 物理安装角、杆臂、健康度、延迟预测
        │    ├─ MAVROS/PX4 外部视觉
        │    ├─ fastlio_to_px4_alignment
        │    └─ sensor_from_body / sensor_position_in_body
        │
        └─ task1_perception_gate
             ├─ 点云和里程计统一扶正到 task1_planning
             ├─ 任务关闭时仍输出里程计，点云保持门控
             └─ planning_to_px4_alignment
                    │
                    ├─ five_meter_avoidance_test（当前验证入口）
                    │       └─ 单目标、稳定悬停、故障终止
                    │
                    └─ super_exploration_decider（以后正式任务一）
                            └─ 连续探索目标
                                      │
                                      ▼
                              SUPER /planning/pos_cmd
                                      │
                              super_px4_command_bridge
                              ├─ 新任务命令隔离
                              ├─ OFFBOARD 进入检查
                              ├─ PX4 闭环速度预测治理
                              ├─ 实测速度硬保护
                              └─ command_fault
                                      │
                                      ▼
                              offboard_bridge → MAVROS → PX4
```

坐标契约现在是：规划器、任务决策器、任务目标、规划点云和规划里程计统一使用
`task1_planning`；原始 `camera_init` 不得绕过 `task1_perception_gate` 直接进入 SUPER。

## 4. 已完成的功能修改

### 4.1 雷达安装角、杆臂与水平规划坐标

- 新增通用三维四元数和向量工具，支持安装角组合、旋转、逆旋转和角距离检查。
- 真机临时安装参数为：roll `0°`、pitch `25°`、yaw `0°`；此前 `-90.9°` yaw
  临时绕法已经移除。
- 雷达相对机体杆臂设为 `(0.133803, -0.011364, 0.173245) m`，姿态变化时补偿传感器
  绕机体原点产生的假平移。
- 初始捕获时可同时清零机体 roll、pitch、yaw，使水平飞行不再在规划坐标中产生假高度。
- 点云、里程计、协方差、机体姿态和发往 PX4 的命令统一使用三维旋转，不再只处理 yaw。
- `task1_perception_gate` 把原始点云和里程计扶正到 `task1_planning`，并发布只剩平移的
  `planning_to_px4_alignment`。
- 对齐或传感器安装关系在任务打开后发生超限变化时锁存故障，禁止混用两个坐标系。
- SITL 明确把真机安装补偿和杆臂覆盖为零，避免把真机几何错误地带入仿真模型。
- RViz、SUPER FSM、ROG-Map 和探索决策器的固定坐标统一改为 `task1_planning`。

### 4.2 Fast-LIO 到 PX4 的输入健康与延迟处理

- 检查输入时间戳、位置跳变、线速度、角速度和位置协方差。
- 支持 `/fast_lio/localization_healthy` 健康话题及超时；真机要求健康，SITL 可显式关闭。
- 对低频 Fast-LIO 位姿做有界的位置和姿态短期预测，再以 30 Hz 输出给 MAVROS。
- 预测时间增加协方差，避免把预测量继续当作原始测量精度。
- 输入不健康时发布不健康状态，并提高外部视觉位置方差，不静默输出伪正常数据。
- 发布雷达安装旋转和杆臂元数据，供规划门和指令桥使用同一套几何关系。

### 4.3 速度治理与 OFFBOARD 生命周期

- 新增 PX4 闭环速度预测：

  ```text
  v_raw  = v_plan + diag(MPC_XY_P, MPC_XY_P, MPC_Z_P) × position_error
  v_safe = project_to_3D_speed_ball(v_raw)
  v_ff   = v_safe - position_gain × position_error
  ```

- 当前五米真机/SITL专用值：

  | 项目 | 当前值 | 作用 |
  |---|---:|---|
  | SUPER `max_vel` | `0.20 m/s` | 正常任务名义规划速度 |
  | `max_predicted_speed` | `0.30 m/s` | 规划速度与 PX4 位置追赶合成后的治理线 |
  | `max_speed` | `0.40 m/s` | 规划前馈硬故障线 |
  | `max_measured_speed` | `0.40 m/s` | MAVROS 实测三维速度硬故障线 |
  | `overspeed_duration` | `0.10 s` | 实测超速持续确认时间 |
  | `max_offboard_entry_speed` | `0.20 m/s` | 进入 OFFBOARD 前最大实测速度 |
  | `max_governor_tracking_error` | `0.65 m` | 允许速度治理的最大位置跟踪误差 |
  | `tracking_position_gain_xy` | `0.9` | 必须与 PX4 `MPC_XY_P` 一致 |
  | `tracking_position_gain_z` | `1.0` | 必须与 PX4 `MPC_Z_P` 一致 |

- 普通预测超限进入 `SPEED_GOVERNING`，通过调整速度前馈继续任务；不能治理的大跟踪误差、
  非有限值或所需反向前馈超过硬限时触发 `SPEED_GOVERNOR_INFEASIBLE`。
- 实测速度超过 `0.40 m/s` 持续 `0.10 s` 仍触发 `MEASURED_SPEED_LIMIT`。这一最终保护
  不能为了避免退出而删除或继续提高。
- 每次 `goaf_enable` 上升沿以及 `auto_enable` 上升沿都建立新的命令 epoch，清空上一任务
  命令、源时间戳、连续性状态、预发送状态和重捕获状态。
- 自动权限打开后必须再收到一帧新命令，暖图或目标接受前的命令不能直接启动 OFFBOARD。
- OFFBOARD 预发送固定保持当前实测位置，不再使用上一任务最后位置。
- 请求 OFFBOARD 前检查本地位姿/速度新鲜、飞机速度不超过 `0.20 m/s`、初始跟踪误差不
  超过 `0.65 m`。
- 模式请求节流改为只限制重复请求同一模式；刚请求 OFFBOARD 后发生故障时，POSCTL
  退出请求不需要再等待原来的一秒节流。
- 正常完成或故障退出时，先以最后一帧已经实际下发的目标连续制动/保持，再请求
  `POSCTL`；清空上游命令时间戳不会把退出目标突然改成当前测量点。
- 新增锁存故障话题 `/mine_uav/task1/command_fault`，一个任务周期只发布第一个非空根因；
  任务权限关闭后发布空字符串复位。

### 4.4 五米任务协调器

- 启动前要求对齐已经就绪且为水平 `task1_planning`。
- 规划选择权限与自动飞行权限分开：暖图时只发布 `goaf_enable=true`，保持
  `auto_enable=false`；收到本次准确 `goal_id` 的接受消息后才开启自动飞行。
- 订阅 `/mine_uav/task1/command_fault`。在暖图或活动阶段收到非空故障时：
  取消当前 SUPER 目标、发布 `ABORTED:COMMAND_BRIDGE_FAULT_<原因>`、关闭
  `auto_enable` 和 `goaf_enable`。
- 目标距离可由 launch 参数覆盖，但当前下一架次方案保持默认 5 m，不使用 1 m、3 m
  覆盖值。
- 下一架次飞行高度改为离地约 `1.0 m`；规划避障包络改为水平半径 `1.0 m`、垂直半径
  `0.6 m`。连续走廊使用 `0.6 m` 基础球和 `0.4 m` 水平附加余量，ROG-Map 使用
  `inflation_step=13`、`inflation_vertical_step=6`（分辨率 `0.1 m`）。水平前端搜索余量
  取 `1.3 m` 是为了覆盖障碍栅格和斜向线段量化，最终连续走廊的水平安全半径仍为
  `1.0 m`，没有缩小水平避障距离。
- 目标和里程计改用已经扶正的 `task1_planning`，不再对临时目标单独做重复角度换算。
- 保留稳定悬停检查、RC 低到高边沿、CH7 取消、目标超时和到达后稳定判定。

### 4.5 飞行记录和 ULog

- 五米真机启动脚本为每次软件运行建立独立时间戳目录，并保存 Git 状态、参数、节点、
  话题和实际 `goal_distance`。
- rosbag 继续录制全部关键 ROS 数据，只排除可由原始 Livox 数据重建的两份高带宽注册
  点云；启用 LZ4、五分钟分卷、锁存话题重复写入和 256 MB 缓冲。
- 新增 PX4 ULog 自动下载器：只在一次真实解锁后重新上锁时工作；等待 PX4 写盘后通过
  MAVROS raw log transfer 下载；缺块重试；检查 ULog magic 和文件大小；使用 `.part`
  临时文件并原子改名；同时写 SHA-256。
- ULog 下载失败只发布 `/mine_uav/logging/px4_ulog_status`，不会停止飞行控制链。
- 真机脚本允许 `MINE_UAV_GOAL_DISTANCE=0.5..5.0` 安全覆盖，并把值写入元数据。

### 4.6 PX4 实机参数记录

`params/real_tuning/px4_pitch_rate_step1_2026-10-14.params` 记录了已经在上锁状态写入真机并
回读确认的第一步俯仰角速度阻尼调整：

```text
MC_PITCHRATE_P = 0.165
MC_PITCHRATE_D = 0.0033
```

该文件不会被 launch 自动加载。以后更换飞控、恢复参数或刷写固件时，必须重新核对真机
参数；不得因为仓库中存在此文件就假设飞控已经使用这些值。

## 5. 文件级变更清单

### 5.1 构建与核心源码

| 文件 | 状态 | 记录的修改 |
|---|---|---|
| `CMakeLists.txt` | 修改 | 注册规划坐标、五米故障、安装角和 ULog 下载器测试；安装 ULog 脚本 |
| `include/mine_uav_control/fastlio_mount_alignment.hpp` | 新增 | 三维安装角、四元数、坐标扶正和旋转预测工具 |
| `include/mine_uav_control/tracking_speed_governor.hpp` | 新增 | PX4 合成速度三维投影治理器 |
| `include/mine_uav_control/five_meter_avoidance_logic.hpp` | 修改 | 增加安装角相关目标工具；当前主流程已改为直接使用水平规划坐标 |
| `src/fastlio_px4_vision_bridge.cpp` | 修改 | 健康检查、短期预测、三维安装角、杆臂、协方差和安装元数据 |
| `src/task1_perception_gate.cpp` | 修改 | 点云/里程计扶正、规划对齐发布、对齐变化故障和任务点云门控 |
| `src/super_px4_command_bridge.cpp` | 修改 | 三维命令转换、杆臂、速度治理、实测超速、命令 epoch、进入检查、故障话题和立即回退 |
| `src/five_meter_avoidance_test.cpp` | 修改 | 水平坐标、两阶段权限、目标接受门、桥接故障终止和可调距离 |
| `src/super_exploration_decider.cpp` | 修改 | 默认输入改为任务门后的水平点云/里程计 |

SUPER 工作区同步增加了非等向避障支持：ROG-Map 的水平/垂直栅格膨胀步数分开；
CIRI 连续走廊在 `0.6 m` 基础球上只对水平方向增加 `0.4 m`；A* 与种子线捷径
检查共用同一个水平 `1.0 m`/垂直 `0.6 m` 邻域，避免 A* 绕行路径被球形捷径
再次切回水平安全距离内。相关修改文件为：

- `rog_map/include/rog_map/rog_map_core/config.hpp`
- `rog_map/include/rog_map_ros/rog_map_ros1.hpp`
- `rog_map/include/rog_map/rog_map_core/inflation_geometry.h`
- `rog_map/src/rog_map/prob_map.cpp`
- `rog_map/src/rog_map/rog_map.cpp`
- `super_planner/include/super_core/config.hpp`
- `super_planner/include/super_core/ciri.h`
- `super_planner/include/super_core/horizontal_clearance_contract.h`
- `super_planner/include/path_search/astar.h`
- `super_planner/src/super_core/ciri.cpp`
- `super_planner/src/super_core/corridor_generator.cpp`
- `super_planner/src/super_core/astar.cpp`
- `super_planner/src/super_core/super_planner.cpp`
- `rog_map/test/inflation_geometry_test.cpp`
- `super_planner/test/horizontal_clearance_contract_test.cpp`

### 5.2 配置和可视化

| 文件 | 状态 | 记录的修改 |
|---|---|---|
| `config/fastlio_px4_vision_bridge.yaml` | 修改 | 输入健康、预测噪声、25° 安装角、杆臂和 Fast-LIO 健康参数 |
| `config/five_meter_avoidance_test.yaml` | 修改 | 水平里程计/对齐话题，0.2 m/s 对应的 60 s 超时 |
| `config/super_px4_command_bridge.yaml` | 修改 | 水平帧、三维对齐、速度治理和 OFFBOARD 进入参数 |
| `config/super_task1.yaml` | 修改 | `task1_planning`、名义速度 `0.2 m/s` 和水平 ROG 可视化帧 |
| `config/super_exploration_decider.yaml` | 修改 | 输入和世界坐标切到 `task1_planning` |
| `config/super_task1_goaf_smooth.yaml` | 修改 | SITL/平滑配置的规划和可视化帧同步 |
| `config/super_task1_maze_clearance_probe.yaml` | 修改 | 探针配置的规划帧同步 |
| `config/super_task1_strict_clearance_probe.yaml` | 修改 | 探针配置的规划帧同步 |
| `rviz/task1_mid360.rviz` | 修改 | Fixed Frame 和视角目标切到 `task1_planning` |

### 5.3 Launch、脚本和参数快照

| 文件 | 状态 | 记录的修改 |
|---|---|---|
| `launch/five_meter_avoidance_real.launch` | 修改 | 可调距离、记录链、ULog 下载和 `0.3/0.4 m/s` 五米限速覆盖 |
| `launch/five_meter_avoidance_sitl.launch` | 修改 | 可调距离、关闭真机安装补偿、使用同一五米限速 |
| `launch/super_px4_command_bridge.launch` | 修改 | 暴露实测/预测/进入速度及超速持续时间参数 |
| `launch/task1_px4_sitl.launch` | 修改 | SITL 显式清零安装角/杆臂并关闭真机健康依赖 |
| `launch/task2_shaft_500m_opticalflow_px4_sitl.launch` | 修改 | 同上，防止任务二 SITL 误用真机雷达参数 |
| `scripts/start_five_meter_avoidance_real.sh` | 修改 | 记录 `goal_distance` 并支持安全范围内的环境变量覆盖 |
| `scripts/px4_ulog_downloader.py` | 新增 | 上锁后安全下载、校验并原子保存 PX4 ULog |
| `params/real_tuning/px4_pitch_rate_step1_2026-10-14.params` | 新增 | 真机已实施的第一步 pitch-rate 参数记录，不自动加载 |

### 5.4 自动测试

| 文件 | 状态 | 覆盖内容 |
|---|---|---|
| `test/test_fastlio_mount_alignment.cpp` | 新增 | 安装角顺序、水平移动、初始姿态、杆臂和姿态预测 |
| `test/task1_planning_frame.test` | 新增 | 启动规划门的 ROS 测试 |
| `test/test_task1_planning_frame.py` | 新增 | 25° 原始运动转换为水平点云/里程计并验证对齐 |
| `test/test_motion_continuity.cpp` | 修改 | 增加闭环速度治理的未介入、介入和三维投影测试 |
| `test/test_five_meter_avoidance_logic.cpp` | 修改 | 安装角下水平前向目标计算 |
| `test/super_bridge_manual_override.test` | 修改 | 给桥接测试配置速度参数并延长总时限 |
| `test/test_super_bridge_manual_override.py` | 修改 | 本地速度、实测超速、预测治理、旧目标拒绝、故障话题和立即 POSCTL |
| `test/five_meter_command_fault_abort.test` | 新增 | 五米任务故障闭环 ROS 测试 |
| `test/test_five_meter_command_fault_abort.py` | 新增 | 同一次运行连续两任务，两个不同桥接故障均终止并撤权 |
| `test/test_px4_ulog_downloader.py` | 新增 | 缺块合并、命名、原子保存和校验和 |

### 5.5 文档

| 文件 | 状态 | 内容 |
|---|---|---|
| `docs/five_meter_avoidance_test_2026-10-11.md` | 修改 | 五米链路、安装角、记录、速度治理和历史验证结果 |
| `docs/temporary_radar_yaw_compensation_2026-10-13.md` | 新增 | 早期雷达方向诊断历史；其中临时 yaw 方案已被后续正向安装取代 |
| `docs/five_meter_pitch_oscillation_2026-10-14.md` | 新增 | 实飞俯仰振荡分析和 PX4 小步调参记录 |
| `docs/next_flight_experiment_2026-10-15.md` | 新增 | 地面/SITL 通过后直接离地约 1.0 m 的 5 m 带障碍方案 |
| `docs/task1_architecture.md` | 修改 | 增加任务一切换前必须完成本台账 P0 项的醒目标记 |
| `docs/five_meter_to_task1_change_ledger_2026-10-14.md` | 新增 | 本台账和任务一迁移清单 |

## 6. 五米代码与正式任务一的覆盖关系

| 能力 | 五米验证 | 当前 `task1_real.launch` | 切换前结论 |
|---|---|---|---|
| Fast-LIO 健康、安装角、杆臂 | 已使用 | 已包含同一个视觉桥 | 已共享，但必须核对真机参数 |
| 水平 `task1_planning` 点云/里程计 | 已使用 | 已启动同一个 perception gate | 已共享 |
| SUPER/决策器水平帧配置 | 已使用 | 已使用同一配置 | 已共享 |
| 桥接器闭环速度治理和实测超速 | 已使用 | 已包含同一个桥接器 | 代码共享，但限速参数不同 |
| 新任务命令 epoch、当前位姿预发送、进入检查 | 已使用 | 同一桥接器自动继承 | 已共享 |
| SUPER 接受目标前保持 `auto_enable=false` | 已实现 | `mission_scheduler` 仍同时打开两个权限 | **未迁移，P0** |
| 上层直接消费 `command_fault` | 已实现 | `mission_scheduler` 未订阅 | **未迁移，P0** |
| 启动前稳定悬停 | 已实现 | 调度器只检查里程计新鲜度 | **未迁移，P0** |
| `0.2 / 0.3 / 0.4 m/s` 五米速度层级 | launch 明确覆盖 | 任务一桥接 launch 仍使用约 `1.1 m/s` 默认值 | **未迁移，P0** |
| 自动 rosbag 和 ULog | 五米真机入口已实现 | `task1_real.launch` 没有对应记录包装 | **未迁移，P0** |
| 连续第二次任务回归 | 已覆盖五米协调器和桥接器 | 没有任务一调度器等价测试 | **未迁移，P0** |
| 可覆盖的固定目标距离 | 五米专用；当前保持默认 5 m | 正式探索不适用 | 不迁移 |
| 五米到达/60 s 超时 | 五米专用 | 正式任务使用地图闭合和返航 | 不迁移 |

当前正式任务一不能仅通过把 `five_meter_avoidance_real.launch` 换成
`task1_real.launch` 就开始实飞。上述 P0 项没有完成以前，应继续把五米入口作为唯一真机
自动飞行验证入口。

## 7. 切换任务一前必须实施的修改

### P0-1：任务一故障必须闭环到调度器

修改 `mission_scheduler`：

1. 订阅锁存话题 `/mine_uav/task1/command_fault`；
2. 当活动任务为任务一且消息非空时，立即 `applyTask(Hold, reason, false)`；
3. 对外状态明确记录 `task1_command_fault_<原因>`；
4. 同一次故障只能终止一次，不得因为锁存消息重复切换；
5. 空字符串只清除观察状态，不能自动重新启动任务；
6. 即使故障发生在 PX4 首次进入 OFFBOARD 之前，也必须关闭任务。不能继续依赖
   `px4_offboard_exited`，因为此时可能根本没有模式退出事件。

必须增加调度器 ROS 回归：第一次任务注入 `SPEED_GOVERNOR_INFEASIBLE`，第二次不重启
节点再注入 `MEASURED_SPEED_LIMIT`，两次都要求 `active_task=hold`、`auto_enable=false`、
`goaf_enable=false`。

### P0-2：任务选择与自动飞行权限分两阶段

当前 `mission_scheduler::applyTask()` 会同时发布：

```text
goaf_enable = true
auto_enable = true
```

正式任务一必须改成：

```text
CH7 边沿
  → goaf_enable=true, auto_enable=false
  → perception gate/decider 建图并生成本任务第一个 goal_id
  → decider/SUPER 确认本次 goal_id 已接受
  → auto_enable=true
  → bridge 清空接受前缓存，等待接受后的新 /planning/pos_cmd
  → 当前位姿预发送和 OFFBOARD 进入检查
```

推荐给探索决策器增加一个带生命周期含义的 `task_ready` 或
`accepted_initial_goal_id`，由调度器验证它属于当前任务 epoch。不要仅凭任意历史
`accepted_goal_id` 打开自动权限。

### P0-3：任务一启动前增加稳定悬停门

任务一调度器当前只检查原始 `/Odometry` 是否新鲜。迁移时应：

- 改用 `/mine_uav/task1/odometry`；
- 检查坐标必须为 `task1_planning`；
- 使用位姿差分或可信速度，要求三维速度不超过 `0.20 m/s`、垂直速度不超过
  `0.10 m/s`，连续稳定至少 `1.0 s`；
- 要求 MAVROS 已连接、飞机已解锁、规划对齐就绪；
- 15 s 内不能稳定则本次启动失败并回到 Hold，不打开自动权限。

桥接器的 `max_offboard_entry_speed=0.20` 是最后一道拒绝门，不能代替上层在任务开始前给
飞手明确的 `WAIT_HOVER` 状态。

### P0-4：任务一首次实飞沿用保守速度层级

`task1_real.launch` 当前直接包含 `super_px4_command_bridge.launch`，没有覆盖参数，因此
桥接器约使用 `1.1 m/s` 默认限制。正式任务一首次切换前至少应显式传入：

```xml
<arg name="max_speed" value="0.4"/>
<arg name="max_measured_speed" value="0.4"/>
<arg name="max_predicted_speed" value="0.3"/>
<arg name="max_offboard_entry_speed" value="0.2"/>
```

`config/super_task1.yaml` 的 `max_vel=0.2` 已是保守值。只有连续任务一 SITL、短距离真机和
日志复盘都通过后，才讨论逐级提高速度；每次提高都必须同时重新计算预测余量，不能只改
SUPER 或只改桥接硬线。

### P0-5：把五米记录链移植到任务一真机入口

正式任务一启动入口必须具备：

- 每次运行独立目录；
- ROS 全链路 bag、五分钟分卷、LZ4、锁存话题重复写入；
- PX4 ULog 上锁后自动下载和 SHA-256；
- 保存 Git commit/dirty 状态、最终 ROS 参数、节点、话题、设备名和 PX4 参数快照；
- 磁盘不足时拒绝在地面启动，但记录器运行失败不得直接杀死在飞控制链；
- PX4 设备按 `/dev/serial/by-id` 明确识别，不能猜测任意 tty。

建议建立独立的 `start_task1_real.sh` 和 systemd unit，不要让正式任务一继续复用名称和目录
都写着 `five_meter` 的脚本。

### P0-6：增加正式任务一专用回归

至少新增以下自动测试：

1. 任务一首次目标接受前 `auto_enable` 一直为 false；
2. 上一任务终点在第二次任务中不会成为预发送目标；
3. 第二次任务在不重启任何节点时能够产生新的任务 epoch 和新目标；
4. OFFBOARD 前速度大于 `0.20 m/s` 时任务被拒绝并回到 Hold；
5. `command_fault` 在 OFFBOARD 前、OFFBOARD 中和制动阶段都能终止任务；
6. 正常预测超限只进入 `SPEED_GOVERNING`，不会误终止任务；
7. 实测超过 `0.40 m/s` 持续 `0.10 s` 时桥接器和调度器都终止；
8. CH5 人工接管后任何自动节点都不得重新请求 OFFBOARD，直到新的人工任务边沿；
9. 对齐变化、Fast-LIO 不健康、里程计超时均关闭权限；
10. 连续两次完整任务结束后，故障话题、目标 ID、返回状态和权限都正确复位。

## 8. 任务一迁移验收顺序

任务一改动完成后按以下顺序验收，任何一级失败都不进入下一级：

1. 编译、GTest、Python 单测和全部相关 rostest；
2. 注入旧目标、实测超速、预测不可治理、对齐变化、定位丢失和人工接管；
3. 任务一 SITL 连续运行两次，中间不重启 ROS/PX4/SUPER；
4. SITL 检查 `task1_planning` 水平坐标、PX4 内部速度设定值和任务故障闭环；
5. 真机卸桨检查坐标方向、安装角、杆臂、模式请求和故障撤权；
6. 真机低高度短距离任务一，只允许一个短前向目标；
7. 真机连续第二次短距离任务一，专门检查旧目标和旧轨迹；
8. 扩大距离但保持无障碍；
9. 最后才进行完整探索和返航。

每一级的硬性通过条件至少包括：

- 目标接受前不进入 OFFBOARD；
- 预发送目标等于当前位姿，不指向旧终点；
- 预测三维速度通常不超过 `0.30 m/s`，实测建议不超过 `0.36 m/s`；
- 实测速度不得超过 `0.40 m/s` 持续 `0.10 s`；
- 任意桥接故障后任务权限均为 false；
- 第二次任务与第一次满足相同标准；
- ROS bag、PX4 ULog、参数和状态记录完整。

## 9. 当前验证结果

截至本台账更新时：

- `catkin_make --pkg mine_uav_control -DCATKIN_ENABLE_TESTING=ON` 通过；
- `super_bridge_manual_override.test`：`7/7` 通过，其中新增任务结束连续保持再切
  `POSCTL` 的回归；
- `five_meter_command_fault_abort.test`：`1/1` 通过，该单项在同一次节点运行中连续启动两个
  任务，分别注入 `SPEED_GOVERNOR_INFEASIBLE` 和 `MEASURED_SPEED_LIMIT`，两次均进入
  `ABORTED` 并撤销权限；
- 工作区累计自动测试 `263` 项，`0` error、`0` failure；`git diff --check` 通过；
- 最终“水平 `1.0 m`/垂直 `0.6 m`”配置已完成一次 `1.0 m` 起飞高度、带中央
  障碍的完整 PX4/Gazebo/Fast-LIO/SUPER 仿真：任务到达
  `GOAL_REACHED_AND_SETTLED`，任务阶段实测三维速度峰值 `0.237 m/s`，PX4 三维速度
  设定值峰值 `0.242 m/s`，下视测距约 `1.04–1.26 m`，未触发超速退出或
  `OUTPUT_DISCONTINUITY_REJECTED`；
- 本次 SITL 没有实体遥控器，PX4 因无有效手动输入拒绝 `POSCTL`；桥接器持续保持
  终点并重试回退，任务状态已正常完成。该仿真限制不用于判断真机 Position 接管，没有为它
  增加生产控制逻辑或保留模拟遥控器注入；
- 尚未进行修复后的真实飞行。

下一次五米实飞严格按 `docs/next_flight_experiment_2026-10-15.md` 执行。任务一的 P0 项
没有完成并通过上述迁移回归前，不得把五米验证通过直接解释为正式任务一已经可飞。

## 10. 后续更新本台账的规则

每次涉及以下任一项时必须同步更新本文件：

- 话题名、坐标系或安装角/杆臂；
- SUPER、桥接器或 PX4 速度/增益参数；
- 任务权限时序和目标生命周期；
- 故障原因、制动或模式回退行为；
- 真机启动、记录和 ULog 下载方式；
- 测试数量、测试结果或真机验收结论。

更新时应记录日期、修改文件、旧值、新值、原因、验证方式和对应日志目录。不要删除失败
记录；过时方案应标为历史状态并指向替代方案，以便后续复盘。
