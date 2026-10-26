# 任务一代码职责与数据流

> 2026-10-26 状态：正式真机 Task-1 定位链已从 Fast-LIO2 切换为 Omni-LIVO（MID360
> + Seeker 四目 + IMU）。旧 Fast-LIO 仅保留在隔离的 SITL/历史五米验证入口，不再由
> `task1_real.launch` 启动。新链路尚需同步板接入后的拆桨与分级真机验收；本页不能作为
> 当前真机放飞许可。
>
> 五米测试中已验证的调度器故障闭环、稳定悬停启动门、首目标接受后
> 才开放自动权限以及独立记录链，已经迁移到正式任务一并通过隔离自动回归。当前工作树的
> 速度合同为名义规划/预测治理与规划、实测硬上限 `0.9/1.1/1.1 m/s`。水平几何合同为
> `0.8 m` 硬选点净距、`1.1 m` 优选净距和 `0.65 m` 独立近障保护；突发障碍先固定
> 悬停监测，连续存在 `0.5 s` 后请求新验证路线；硬近障只允许明确增加
> 障碍净距的新编号 SUPER 轨迹接管；定位或点云失效时仍执行退出保护。这只是代码迁移完成，
> 仍须按 `docs/five_meter_to_task1_change_ledger_2026-10-14.md` 第8节完成正式任务一 SITL、
> 拆桨和分级真机验收；本页不能作为当前真机放飞许可。
> SUPER 移植边界、停顿因果和安全参数分层见
> [`super_porting_adapter_audit_2026-10-26.md`](super_porting_adapter_audit_2026-10-26.md)。

## 结论

当前工程的问题不是“所有代码都不可维护”，而是原来的 `task1_sitl_adapter.py`
同时承担了定位适配、模拟传感器、模拟遥控器、自动起飞和健康状态发布，职责过多；
另外，任务决策器自己的地图清空动作没有清空 SUPER 内部的 ROG-Map。现在已经按数据
边界整理：定位链路和任务规划感知链路分开，SITL 的飞手模拟和定位适配分开，SUPER
参数归工程管理。

## 运行时职责

| 模块 | 只负责 | 不负责 |
|---|---|---|
| Omni-LIVO / MID360 / Seeker | 融合点云、四目图像和IMU，输出里程计、注册点云与健康状态 | 任务选择、飞行模式 |
| `lio_px4_vision_bridge` | `/omnilivo/odometry` 对齐、校验并发送给PX4 EKF2 | 规划点云、目标选择 |
| `mission_scheduler` | RC任务/自动允许状态机 | 点云、路径规划、PX4解锁 |
| `task1_perception_gate` | 任务一 CH9/任务四 CH7 授权后把本轮点云和里程计转给任务规划器 | 修改原始定位、控制飞机 |
| `super_exploration_decider` | 可达地图、前向目标、覆盖/闭合判断、返航目标 | 轨迹优化、MAVLink |
| SUPER `fsm_node` | 局部地图、安全走廊、轨迹优化和 `/planning/pos_cmd` | 任务选择、PX4模式 |
| `super_px4_command_bridge` | 坐标变换、指令限幅、健康检查、OFFBOARD进出 | 解锁、地图、探索决策 |
| PX4/MAVROS | 飞控状态估计和执行 | 生成探索目标 |

真机图中 `mission_scheduler` 与 `super_exploration_decider` 都是
`required="true"` 的关键进程。任一进程异常退出，`roslaunch` 必须关闭整个
`task1_real.launch` 进程图；生产包装器等待该退出、执行同步板与子进程清理，并把
`roslaunch` 对 required-node 故障错误返回的零状态转换为非零后交回 systemd，不能留下
没有调度器或没有探索决策器的残缺飞行栈继续运行。正常的
`Ctrl+C`/systemd 停止仍走相同的有序清理路径。

## 两条必须保持独立的链路

```text
Omni /omnilivo/odometry ──> lio_px4_vision_bridge ──> MAVROS ──> PX4 EKF2
      └─> task1_perception_gate（持续输出调平里程计）─> mission_scheduler

Omni点云/里程计 ──> task1_perception_gate（点云受任务门控，里程计持续调平）
                  ├─> super_exploration_decider ──> /goal
                  └─> SUPER ROG-Map/FSM ──> /planning/pos_cmd
                                      └─> command_bridge ──> MAVROS/PX4
```

Omni-LIVO 不直接发布 MAVROS pose；只有桥接器在时间戳、协方差、平移可观测性、连续性
和 MAVROS 连接均健康时发布 `/mavros/vision_pose/pose_cov`。规划门关闭时，PX4仍能继续获得定位，SUPER不会吸收起飞平台、起飞爬升和任务开始前
的机体回波。任务重新开启时，规划门打开新一轮数据流，决策器在第一组同步样本上
重新建立 home。

桥接器把“连续但物理可达的高速转动”和“姿态不连续跳变”分开处理：超过
`2 rad/s`、但不超过物理边界的连续转动暂停姿态外推并提高协方差，不锁死视觉；单帧
姿态跳变、非有限增量和连续不可实现角速度仍立即或确认后锁存关闭。锁存只能在自动权限
已撤销且 LIO 连续稳定若干帧后恢复，并复用原坐标对齐；飞机已解锁时不允许重新捕获原点。

四相机图像仍按 `1 ms` 严格时间戳容差配组，不会把不同曝光时刻强行拼成一组。运行配置
保留最近 `3` 帧的小型有界窗口，对固定一帧错相直接丢弃落后帧并重新对齐；日志分别累计
容量、时间戳错相和相对 LIO 过期三类丢帧。安装标定中的
`hardware_time_sync_verified: false` 会传入运行配置并持续告警；软件修复不能替代硬件触发、
同一时钟和长时间真机吞吐验收。

## SITL 专用模块

SITL 不能混入真机启动文件：

- `sitl_localization_adapter.py`：只将 `/mavros/local_position/odom` 转成项目的
  `/Odometry`，并发布路径、健康状态和单位对齐变换。
- `sitl_task_operator.py`：只模拟预发送悬停点、OFFBOARD、解锁、爬升、悬停和CH7。
- `gazebo_mid360_fastlio_adapter.py`：只把Gazebo雷达模型转为注册点云，并做仿真机体
  自回波过滤。

真机 `task1_real.launch` 不启动上述三个SITL适配器。

## 本轮回归结果

使用 `40m x 40m x 30m` 采空区场景和平台起飞：

- 任务开始前规划门保持关闭；CH7任务切换后打开。
- home 在 `z=1.51m` 捕获，首个SUPER目标也保持 `z=1.51m`。
- 日志中未出现此前的 `z=2.25m` 首条异常目标，也未出现 `HEIGHT_GEOFENCE`。
- 最终出现 `map closure confirmed`，桥接器执行 `TASK1_COMPLETE` 并请求
  `AUTO.LOITER`。

这证明“任务前点云污染SUPER地图”这一根因已被隔离。仿真仍有大量
`BackOpt/ExpOpt failed` 日志，主要是当前Gazebo点云和SUPER局部走廊边界造成的重规划
压力；它属于下一阶段的平滑性/仿真模型问题，不能通过放宽PX4安全围栏解决。

## 后续重构边界

下一步若继续拆分C++，只按以下稳定接口拆，不改变算法行为：

1. `ExplorationMap`：体素、射线、可达自由空间、frontier候选。
2. `CoverageEvaluator`：前向/侧墙证据、闭合条件、稳定计数。
3. `ExplorationPolicy`：前向优先、frontier fallback、目标与返航目标。
4. ROS节点壳：订阅同步、参数、发布话题和服务。

在这四个模块的单元测试具备之前，不直接大规模重写现有 `super_exploration_decider.cpp`，
以免把已经验证的返航逻辑和安全边界一起改坏。
