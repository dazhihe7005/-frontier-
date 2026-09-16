# 任务一代码职责与数据流

## 结论

当前工程的问题不是“所有代码都不可维护”，而是原来的 `task1_sitl_adapter.py`
同时承担了定位适配、模拟传感器、模拟遥控器、自动起飞和健康状态发布，职责过多；
另外，任务决策器自己的地图清空动作没有清空 SUPER 内部的 ROG-Map。现在已经按数据
边界整理：定位链路和任务规划感知链路分开，SITL 的飞手模拟和定位适配分开，SUPER
参数归工程管理。

## 运行时职责

| 模块 | 只负责 | 不负责 |
|---|---|---|
| Fast-LIO2 / MID360 | 原始点云与里程计 | 任务选择、飞行模式 |
| `fastlio_px4_vision_bridge` | `/Odometry` 对齐并发送给PX4 EKF2 | 规划点云、目标选择 |
| `mission_scheduler` | RC任务/自动允许状态机 | 点云、路径规划、PX4解锁 |
| `task1_perception_gate` | CH7后把本轮点云和里程计转给任务规划器 | 修改原始定位、控制飞机 |
| `super_exploration_decider` | 可达地图、前向目标、覆盖/闭合判断、返航目标 | 轨迹优化、MAVLink |
| SUPER `fsm_node` | 局部地图、安全走廊、轨迹优化和 `/planning/pos_cmd` | 任务选择、PX4模式 |
| `super_px4_command_bridge` | 坐标变换、指令限幅、健康检查、OFFBOARD进出 | 解锁、地图、探索决策 |
| PX4/MAVROS | 飞控状态估计和执行 | 生成探索目标 |

## 两条必须保持独立的链路

```text
原始 /Odometry ──> fastlio_px4_vision_bridge ──> MAVROS ──> PX4 EKF2
      └──────────> mission_scheduler

原始点云/里程计 ──> task1_perception_gate(CH7 && vision healthy)
                  ├─> super_exploration_decider ──> /goal
                  └─> SUPER ROG-Map/FSM ──> /planning/pos_cmd
                                      └─> command_bridge ──> MAVROS/PX4
```

规划门关闭时，PX4仍能继续获得定位，SUPER不会吸收起飞平台、起飞爬升和任务开始前
的机体回波。任务重新开启时，规划门打开新一轮数据流，决策器在第一组同步样本上
重新建立 home。

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
