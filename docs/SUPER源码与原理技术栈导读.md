# SUPER 源码、原理与技术栈深度导读

> 本文基于本机当前源码：`/home/nuc/super_ws/src/SUPER`。当前本地仓库提交为 `2ad3419c`（2025-06-04）。本文将“论文中的 SUPER 系统”和“当前 checkout 中可直接读到的代码”分开说明；二者不是每个细节都一一对应。

## 1. 先给结论：SUPER 到底是什么

SUPER 不是一个单独的避障函数，也不是一个完整的自主探索器。更准确地说，它是一套面向高速 MAV 的局部导航栈：

```text
点云 + 位姿
    │
    ├── ROG-Map：局部、滑动、多分辨率占据/自由空间表示
    │
    ├── A*：在离散地图中找一条几何引导路径
    │
    ├── CIRI：沿引导路径生成配置空间安全走廊（重叠凸多面体）
    │
    ├── MINCO + L-BFGS：在走廊内优化时间和多项式轨迹
    │
    ├── BackupTrajOpt：为规划失败或未知障碍准备已知自由空间内的备份轨迹
    │
    ├── YawTrajOpt + 四旋翼平坦性映射：得到 yaw、姿态、角速度、推力
    │
    └── FSM/ROS 接口：维护轨迹、重规划、发布 PositionCommand
```

论文的关键思想是每个重规划周期同时生成两条轨迹：

- 探索轨迹 `T_e`：允许经过已知自由空间和未知空间，目标是尽量高速到达目标。
- 备份轨迹 `T_b`：从探索轨迹上的切换点开始，完全位于已知自由空间，目标是在新规划失败时仍然能够刹停或退回安全区域。
- 实际执行的提交轨迹 `T_c` 是 `T_e` 的前半段加上 `T_b` 的后半段。

这也是 SUPER 和普通“把未知空间当作自由空间”的局部规划器的根本区别：未知空间可以用于提速，但系统始终准备一条可退回的安全轨迹。[1]

对你的矿井项目，最重要的边界是：SUPER 可以承担“任务一中的局部安全导航与轨迹生成”，但它本身不是采空区任务决策器。采空区的下一个观察点、前沿点、覆盖策略、建模完成条件、返航条件，应该由你们自己的自主决策器/任务调度器产生，再把一个局部目标交给 SUPER。

## 2. 论文系统与当前源码的对应关系

论文描述的完整系统包含 FAST-LIO2 状态估计、点云感知/地图、SUPER 规划器、控制器和 PX4 飞控。论文实验系统中，规划器输出的是给控制模块使用的轨迹，控制模块再输出角速度和推力加速度给 PX4 的低层控制。[1]

当前源码仓库主要公开的是规划模块，并附带 ROS1/ROS2 封装、仿真和自定义消息。README 明确把 ROS1 Noetic 作为 Tier 1 平台，同时说明探索 demo、独立 ROG-Map/CIRI 示例、硬件和控制模块教程仍属于 TODO。[2]

因此，当前源码的正确理解方式是：

```text
当前仓库：感知输入 → 局部地图 → 局部规划 → PositionCommand/PolynomialTrajectory
你的系统：Fast-LIO2/点云 → SUPER → 任务统一输出层 → MAVROS/PX4
```

当前源码中，`FsmRos1` 发布的是 `quadrotor_msgs::PositionCommand`，不是 MAVROS 的 `mavros_msgs::PositionTarget`。所以不能把 SUPER 的 ROS 消息直接当作 PX4 setpoint；中间必须有你们自己的适配器。

## 3. 代码目录地图

| 模块 | 主要目录/文件 | 作用 |
|---|---|---|
| ROG-Map | [`rog_map/include/rog_map/rog_map.h`](/home/nuc/super_ws/src/SUPER/rog_map/include/rog_map/rog_map.h)、[`prob_map.h`](/home/nuc/super_ws/src/SUPER/rog_map/include/rog_map/prob_map.h)、[`inf_map.h`](/home/nuc/super_ws/src/SUPER/rog_map/include/rog_map/inf_map.h) | 地图查询、概率占据、膨胀层、滑动局部地图 |
| 地图实现 | [`rog_map/src/rog_map/prob_map.cpp`](/home/nuc/super_ws/src/SUPER/rog_map/src/rog_map/prob_map.cpp)、[`sliding_map.cpp`](/home/nuc/super_ws/src/SUPER/rog_map/src/rog_map/sliding_map.cpp) | 点云更新、射线投射、概率更新、内存滑动 |
| ROS 地图包装 | [`rog_map_ros1.hpp`](/home/nuc/super_ws/src/SUPER/rog_map/include/rog_map_ros/rog_map_ros1.hpp) | 订阅 odom/point cloud，定时更新地图，发布 RViz 数据 |
| SUPER 核心 | [`super_planner.h`](/home/nuc/super_ws/src/SUPER/super_planner/include/super_core/super_planner.h)、[`super_planner.cpp`](/home/nuc/super_ws/src/SUPER/super_planner/src/super_core/super_planner.cpp) | 组织 A*、走廊、探索轨迹、备份轨迹和提交轨迹 |
| FSM | [`fsm.h`](/home/nuc/super_ws/src/SUPER/super_planner/include/fsm/fsm.h)、[`fsm.cpp`](/home/nuc/super_ws/src/SUPER/super_planner/src/super_core/fsm.cpp) | INIT/WAIT_GOAL/GENERATE/FOLLOW/EMER_STOP 状态机 |
| ROS1 FSM | [`fsm_ros1.hpp`](/home/nuc/super_ws/src/SUPER/super_planner/include/ros_interface/ros1/fsm_ros1.hpp) | ROS 消息、定时器、目标订阅、轨迹发布 |
| 路径搜索 | [`astar.h`](/home/nuc/super_ws/src/SUPER/super_planner/include/path_search/astar.h)、[`astar.cpp`](/home/nuc/super_ws/src/SUPER/super_planner/src/super_core/astar.cpp) | 在概率层或膨胀层上做 A* |
| 安全走廊 | [`corridor_generator.h`](/home/nuc/super_ws/src/SUPER/super_planner/include/super_core/corridor_generator.h)、[`ciri.h`](/home/nuc/super_ws/src/SUPER/super_planner/include/super_core/ciri.h) | 从点云和线种子生成凸多面体走廊 |
| 轨迹优化 | [`exp_traj_optimizer_s4.cpp`](/home/nuc/super_ws/src/SUPER/super_planner/src/traj_opt/exp_traj_optimizer_s4.cpp)、[`backup_traj_optimizer_s4.cpp`](/home/nuc/super_ws/src/SUPER/super_planner/src/traj_opt/backup_traj_optimizer_s4.cpp) | 4 阶导数代价、时间、动力学约束、走廊约束 |
| 多项式表示 | [`minco.h`](/home/nuc/super_ws/src/SUPER/super_planner/include/traj_opt/minco.h)、[`trajectory.h`](/home/nuc/super_ws/src/SUPER/super_planner/include/data_structure/base/trajectory.h) | 多段多项式、时间参数化、状态求值 |
| 消息 | [`PositionCommand.msg`](/home/nuc/super_ws/src/SUPER/mars_uav_sim/mars_quadrotor_msgs/msg/PositionCommand.msg)、[`PolynomialTrajectory.msg`](/home/nuc/super_ws/src/SUPER/mars_uav_sim/mars_quadrotor_msgs/msg/PolynomialTrajectory.msg) | 输出位置/速度/加速度/jerk/yaw 或整段多项式 |

## 4. ROG-Map：从点云到可规划空间

### 4.1 类继承关系

```text
SlidingMap
   └── CounterMap
        ├── InfMap       （膨胀后的低分辨率地图）
        ├── FreeCntMap   （可选，前沿提取）
        └── ESDFMap      （可选，距离场）

SlidingMap
   └── ProbMap          （高分辨率概率占据层）
        └── ROGMap
             └── ROGMapROS
```

`ROGMap` 继承 `ProbMap`，再由 `ROGMapROS` 加上 ROS 订阅、定时器和可视化。SUPER Planner 持有的是 `ROGMapROS::Ptr`，因此规划器可以直接调用地图查询接口。

### 4.2 三态地图模型

概率层的每个栅格保存一个类似 log-odds 的值 `L`：

```text
L ← clamp(L + l_hit  × hit_num,  l_min, l_max)
L ← clamp(L + l_miss × miss_num, l_min, l_max)
```

代码中使用三个主要状态：

- `KNOWN_FREE`：`L < l_free`
- `UNKNOWN`：`l_free <= L < l_occ`
- `OCCUPIED`：`L >= l_occ`

这不是简单的“点所在格子设为障碍”。如果启用 raycasting，点到传感器之间的栅格会收到 miss/free 更新，点所在栅格收到 hit/occupied 更新；因此地图能够区分已知自由和未知。

### 4.3 射线投射与延迟更新

`ProbMap::raycastProcess()` 的主流程是：

1. 对输入点云做强度过滤和抽样。
2. 逐条射线从当前位姿向点云点进行离散遍历。
3. 把需要改变的全局栅格索引放进 `update_cache_id_g`。
4. 用 `operation_cnt` 和 `hit_cnt` 累计本帧对每个格子的 miss/hit 次数。
5. `probabilisticMapFromCache()` 批量更新概率值。

这样做的意义是把大量点云操作变成连续数组和计数器操作，减少重复写入与锁竞争。`batch_update_size` 控制多少帧后批量落盘；实时系统通常从 1 开始调。

### 4.4 膨胀层不是重新做一遍体素膨胀

`InfMap` 用 `occ_inflate_cnt` 记录一个低分辨率栅格周围有多少个已占据邻居。当概率层某个格子的状态发生跳变时，`triggerJumpingEdge()` 增减周围球形邻域的计数：

```text
UNKNOWN/FREE → OCCUPIED：周围 inflation counter +1
OCCUPIED → UNKNOWN/FREE：周围 inflation counter -1
```

只要 `occ_inflate_cnt > 0`，膨胀层就认为该位置不可飞。规划器主要用 `getInfGridType()` 和 `isOccupiedInflate()` 做安全判断。这样既考虑了无人机半径 `robot_r`，又避免每次点云更新都全图膨胀。

### 4.5 局部滑动地图

`SlidingMap` 的内存大小固定，地图中心随 odom 移动。滑动时：

- 根据 odom 计算新的全局栅格中心。
- 只清理刚刚移出局部窗口的栅格。
- 使用局部索引和 hash 将新区域映射到已有内存。
- 若一次移动距离超过地图尺寸，则全部清空后重新初始化。

这就是“固定内存 + 局部窗口 + 滑动索引”的核心。它适合矿井中沿巷道前进的局部避障，但它不是 400 多米竖井的全局建模地图；竖井任务应另行维护任务级距离/高度状态和全局轨迹。

### 4.6 当前源码的关键输入约束

这是你接 Fast-LIO2 时必须优先验证的地方：仓库 README 明确写出，ROG-Map 不使用输入点云的 `frame_id` 或 `/tf` 自动做坐标转换，默认要求输入点云已经在 world frame。[2]

当前 `ROGMapROS` 的行为也印证了这一点：

- odom 回调直接把 `nav_msgs/Odometry.pose` 保存为机器人世界位姿。
- cloud 回调只 `pcl::fromROSMsg()` 读取点云坐标，并把当前机器人位姿附给这帧点云。
- 地图更新函数直接把点云的 x/y/z 当作地图坐标使用。

所以你的接口契约应该明确写成：

```text
cloud_registered：PointCloud2，点坐标已经是 world/地图坐标
lidar_slam/odom：机器人在同一个 world/地图坐标系中的位姿
两者时间戳、坐标原点、ENU 轴方向必须一致
```

如果 Fast-LIO2 输出的是 LiDAR/body frame 点云，必须在进入 SUPER 前完成外参和位姿变换；不要指望 ROG-Map 自动完成。

### 4.7 ROS callback 模式与 API 模式只能二选一

配置中：

```yaml
rog_map:
  ros_callback:
    enable: true
```

启用后，`ROGMapROS` 自己订阅 odom、point cloud，并用 1 ms 定时器把缓存送入 `updateProbMap()`。同时 `ROGMap::updateMap()` 会拒绝外部 API 插入并打印警告。因此你需要选择一种架构：

- 推荐前期联调：让 ROGMapROS 直接订阅标准化后的点云和 odom。
- 推荐后期任务系统：关闭 `ros_callback.enable`，由你的感知适配器统一调用 `updateMap(cloud, pose)`，更容易控制时间同步和任务切换。

不要两种方式同时使用，否则会出现重复更新、时间顺序不清和调试困难。

## 5. SUPER Planner 的一轮重规划

`SuperPlanner::PlanFromRest()` 是第一次从静止/当前状态建立轨迹；`ReplanOnce()` 是后续滚动重规划。核心顺序如下。

### 5.1 选择起点和目标

FSM 收到 `PoseStamped` 目标后，先把目标投影到膨胀地图中最近的非占据栅格。`PlanFromRest()` 又会把当前机器人位置投影到附近可用的非占据点，避免规划从障碍内部开始。

如果目标离当前点超过规划视野，代码会通过 A* 的局部搜索范围使用一个局部目标；因此 SUPER 的 goal 更像“当前局部规划目标”，不等于任务级全局目标。

### 5.2 热启动与旧轨迹碰撞检查

后续 `ReplanOnce()` 不会每次从零开始。它会：

1. 从当前提交轨迹获取重规划时刻的状态。
2. 对旧探索轨迹向前采样。
3. 查询这些采样点在膨胀地图中的状态。
4. 如果旧轨迹还安全且没有新目标，就尽量保留更长的前缀。
5. 如果旧轨迹将进入障碍/越界，就缩短保留前缀并立即重新搜索。

这就是 receding horizon 和 hot initialization 的代码实现。`replan_forward_dt` 不只是一个普通周期参数，它实际上定义了规划计算必须在下一次安全接管前完成多少时间预算。

### 5.3 A* 只负责几何引导，不直接输出可执行轨迹

`Astar::pointToPointPathSearch()` 在局部栅格中搜索离散点路径。它可以选择：

- 概率层 `ON_PROB_MAP`
- 膨胀层 `ON_INF_MAP`
- 未知视为障碍 `UNKNOWN_AS_OCCUPIED`
- 未知视为自由 `UNKNOWN_AS_FREE`
- 是否使用膨胀邻域

探索轨迹需要速度，所以它不能只靠 A* 的折线路径。A* 的结果只是后续走廊生成和 MINCO 优化的 guide path。

### 5.4 沿 guide path 生成安全走廊

`CorridorGenerator::SearchPolytopeOnPath()` 反复寻找从当前种子点能看见的最远路径点，形成线段：

```text
s0 ───────── s1 ───── s2 ───── s3 ...
```

每条线段送入 `GeneratePolytopeFromLine()`：

1. 按线段建立一个局部搜索盒。
2. 从地图中提取盒内障碍点。
3. 将障碍点和线种子送给 CIRI。
4. CIRI 求一个包含线种子的凸多面体，同时把障碍点排除在外。
5. 相邻多面体必须有重叠区域，轨迹才可以连续穿过走廊。

从几何上看，每个走廊是半空间的交：

```text
P_i = { p | A_i p <= b_i }
```

它不是把轨迹离散采样后逐点避障，而是把连续轨迹的位置约束到凸多面体中，再利用凸包/多项式约束提高效率。

### 5.5 CIRI：配置空间中的凸分解

CIRI 的名字是 Configuration-space Iterative Regional Inflation。它源于 FIRI 的最大内接椭球/迭代区域膨胀思想，但直接在输入点云和机器人配置空间中工作。SUPER README 将 CIRI、ROG-Map、GCOPTER、FIRI 和 FASTER 列为系统的重要基础。[2]

代码中 `CIRI::comvexDecomposition()` 接收：

- 初始边界盒 `bd`
- 障碍点矩阵 `pc`
- 线段起点 `a`
- 线段终点 `b`

内部先寻找一个合适的椭球，再根据椭球和障碍点生成/调整多面体平面，最终由 `getPolytope()` 取出结果。`robot_r` 会进入 CIRI，使走廊在配置空间中考虑无人机尺寸，而不是只把无人机当成质点。

要特别区分：

- ROG-Map 的 inflation 是地图查询层面的障碍膨胀。
- CIRI 是线段周围的凸安全区域生成。

两者都与“无人机半径”有关，但在不同层次解决问题。A* 常在膨胀层上搜索，CIRI 再围绕实际线种子建立连续走廊。

## 6. MINCO、轨迹优化和动力学约束

### 6.1 为什么是 7 次多项式

当前配置调用 `MINCO_S4NU`。令 `s=4`，每段轨迹使用 `2s=8` 个系数，因此多项式阶数为 7：

```text
p_i(t) = C_i(Q, T, s0, sf) · [1, t, t², ..., t⁷]^T
```

多段之间连续到三阶导数，通常对应位置、速度、加速度、jerk 的连续性；优化的主要高阶导数代价是 snap/四阶导数能量。

### 6.2 优化变量

探索轨迹优化的变量主要是：

- 中间连接点 `Q`
- 每段持续时间 `T`

MINCO 根据起点状态、终点状态、`Q` 和 `T` 快速求多项式系数，并提供对时间和系数的梯度。这样不必把每个多项式系数都作为独立变量，变量数量更少、结构更稀疏。

### 6.3 代价函数

代码中的 `ExpTrajOpt::costFunctional()` 和 `BackupTrajOpt::costFunctional()` 综合了：

```text
总成本 = 高阶导数能量
       + 总时间代价
       + 位置越界软惩罚
       + 速度/加速度/jerk 约束惩罚
       + 角速度、推力约束惩罚
       + 走廊内部点吸引项
```

走廊吸引项把中间点向相邻走廊重叠区域中心吸引，避免轨迹贴着障碍边界飞行，同时保留一定优化自由度。代码使用 L-BFGS 迭代求解；这也是 SUPER 能在较短时间内同时优化空间形状和时间分配的原因。[1][3]

### 6.4 “约束”在实现中并不全是硬约束

走廊位置约束、速度、加速度、角速度和推力等条件，主要通过 barrier/penalty 方式纳入无约束优化。优化结束后代码还会检查惩罚量和轨迹峰值；如果超限则返回失败。

因此不要把“调用 optimize 返回 true”理解成数学意义上的绝对安全证明。工程上的安全来自多层组合：膨胀地图、凸走廊、惩罚检查、备份轨迹、旧提交轨迹保持、飞控控制器和传感器冗余。

## 7. 备份轨迹：SUPER 安全性的核心

### 7.1 论文逻辑

设当前执行位置是 `p_c`，探索轨迹未来某个切换点是 `p_s`，备份轨迹终点是 `p_l`：

```text
T_e: p_c ─────────────── p_s ──（未知/已知空间）── p_g
T_b: p_s ─────────────────────────────── p_l
T_c: p_c ─────────────── p_s ─────────── p_l
```

`p_c → p_s` 和整个 `T_b` 必须在已知自由空间。规划失败时，系统继续执行上一次已提交的 `T_c`；如果执行过程进入备份区，就沿 `T_b` 减速到安全终点。[1]

### 7.2 当前代码如何找备份轨迹

`generateBackupTrajectory()` 对探索轨迹从当前时间开始采样：

1. 计算当前点到每个未来点的停止距离。
2. 用 `ROGMap::isLineFree()` 检查当前点到这些点的可见性。
3. 找到第一个不可见点，向前退到一个可见/可生成走廊的种子点。
4. 对当前机器人位置到种子点生成一个 CIRI polytope。
5. 可选地用 FOV 和 sensing horizon 裁剪 polytope。
6. 调用 `BackupTrajOpt` 优化一条以较低速度、零速度/安全终态为目标的轨迹。
7. 共同优化或检查备份起始时间 `ts`。

如果整条探索轨迹都在已知自由空间内，代码返回 `FINISH/NO_NEED`，不生成 backup。这正是“安全时不牺牲速度”的设计。

### 7.3 `trajectory_flag` 的含义

在 ROS1 封装中：

```cpp
pos_cmd.trajectory_flag = on_backup_traj ? 2 : 1;
```

因此 `PositionCommand` 的 `trajectory_flag` 可以用来观察当前命令是否已经切换到备份轨迹。你的 PX4 适配器不应该丢掉这个字段，建议把它映射到自己的诊断状态和故障处理逻辑。

## 8. FSM 与 ROS1 运行时

### 8.1 状态机

当前状态包括：

```text
INIT
  ↓ 收到有效目标且有新鲜 odom
WAIT_GOAL
  ↓ new_goal
GENERATE_TRAJ
  ↓ PlanFromRest 成功
FOLLOW_TRAJ
  ├── 定时发布当前轨迹命令
  ├── 定时 ReplanOnce
  └── 轨迹结束后回 WAIT_GOAL 或重新生成

EMER_STOP → WAIT_GOAL
```

`callMainFsmOnce()` 会检查 odom 是否存在以及是否超过约 0.1 s；没有新鲜 odom 时不应继续正常规划。`callReplanOnce()` 只在 `FOLLOW_TRAJ` 状态工作。

### 8.2 ROS1 定时器与话题

在 `click_smooth_ros1.yaml` 中，当前示例参数是：

```yaml
fsm:
  replan_rate: 15.0
  cmd_topic: "/planning/pos_cmd"
  mpc_cmd_topic: "/planning_cmd/poly_traj"
```

代码中的定时器是：

- FSM 执行：100 Hz
- PositionCommand 发布：100 Hz
- 重规划：配置值，示例为 15 Hz
- ROGMap 更新：ROS callback 模式下使用 1 ms timer 拉取缓存

论文描述的实验重规划频率是 10 Hz，而当前示例配置是 15 Hz；调试时以当前 YAML 和实际运行日志为准，不要只照搬论文数字。[1]

### 8.3 PositionCommand 的实际内容

每个命令包含：

- position
- velocity
- acceleration
- jerk
- attitude（roll/pitch/yaw）
- angular_velocity
- thrust
- yaw/yaw_dot
- `trajectory_flag`

`FsmRos1::getOnePositionCommand()` 先从多项式轨迹求 `p/v/a/j`，再调用 `convertFlatOutputToAttAndOmg()` 将平坦输出转换为姿态、角速度和推力。

这意味着当前 ROS1 封装已经做了一部分“轨迹到控制参考量”的工作，但它并没有完成 PX4/MAVROS 坐标系、消息类型、OFFBOARD 状态管理和飞控安全握手。那些属于你们的统一输出/适配层。

## 9. 技术栈清单

| 层次 | 技术 |
|---|---|
| 语言 | C++17 |
| 中间件 | ROS1 Noetic，`roscpp`，ROS message/action 风格接口 |
| 数学 | Eigen |
| 点云 | PCL、`pcl_ros`、`pcl_conversions` |
| 配置 | YAML + 自定义 `YamlLoader` |
| 地图 | 固定内存局部滑动栅格、概率 log-odds、counter map、多分辨率 inflation |
| 射线 | 离散 3D raycaster/DDA 风格遍历 |
| 路径 | 3D A*，可选对角邻接、启发函数 |
| 几何 | 凸多面体、线性半空间、内接椭球、CIRI/FIRI 思路、SDLP |
| 轨迹 | 多段 7 次多项式、MINCO、时间分配、连续到三阶导数 |
| 数值优化 | L-BFGS、解析/半解析梯度、软 barrier/penalty |
| 飞行学 | 四旋翼 differential flatness，位置导数到姿态/角速度/推力 |
| 调试 | RViz、Marker/PointCloud2、CSV、cereal 二进制 replan log、fmt、backward-cpp |
| 飞控相关 | 仓库面向 PX4 体系，但当前规划源码通过自定义消息输出；项目中需要 MAVROS/PX4 adapter |

构建方面，ROG-Map 和 SUPER Planner 都使用 C++17、Release、`-O3`。当前仓库通过 `../../../devel/include` 依赖生成的 `quadrotor_msgs`，所以编译顺序和 workspace 环境很重要。

## 10. 当前源码中值得提前记住的坑

### 10.1 `fsm_node_ros1.cpp` 默认配置名可能过期

入口文件默认尝试加载 `super_planner/config/click.yaml`，但当前 `super_planner/config/` 中实际能看到的是 `click_smooth_ros1.yaml`、`click_smooth_ros2.yaml`、`static_dense.yaml` 和 `static_high_speed.yaml`。运行时应显式传 `config_path`，不要依赖默认值。

### 10.2 点云 frame_id 不会自动救你

输入点云必须已经是 world frame。要在你的矿井系统中建立“坐标契约”：Fast-LIO2 输出的位姿、点云、SUPER 的 `world`、MAVROS 的 ENU/NED 转换分别在哪一层完成，并写成接口文档和启动参数。

### 10.3 当前 checkout 与论文的地图表述可能不是一套实现

论文材料中描述了基于点云的时空滑动地图、每个占据格子的最近命中时间以及 lazy update；但当前 checkout 中直接可见的 `ProbMap` 是概率更新 + raycasting + 局部滑动 + inflation 结构，代码搜索不到论文描述的 `thit` 时间戳字段。因此：

- 论文用于理解设计目标和安全逻辑。
- 源码用于判断当前程序的真实输入输出和行为。
- 若要复现实验指标，必须固定仓库 commit、配置和传感器版本。

### 10.4 SUPER 不负责探索任务决策

当前 FSM 只有点击目标输入；`mission_planner` 主要是 waypoint 示例。它不会自动完成“选择下一个采空区前沿点、覆盖区域、判断建模完成、返航”。你们需要在 SUPER 上层增加 exploration manager/decision maker。

### 10.5 地图参数不适合直接拿去做 400 m 竖井

示例配置的局部地图尺寸、虚拟地面/天花板和规划 horizon 都是局部避障参数。例如示例 `map_size` 为 `[50,50,6]`、`planning_horizon` 为 7 m、虚拟高度约为 `-0.1` 到 `3.5` m。竖井任务必须单独设计高度/距离控制和测距触发逻辑，不能把竖井 400 m 当成一次 SUPER 局部规划。

## 11. 对你当前矿井项目的合理集成方式

推荐把系统拆成四个职责清晰的层：

```text
任务调度器 MissionScheduler
    ├── Task 1：采空区探索决策器 → SUPER local goal
    └── Task 2：竖井任务控制器 → 竖直位置/速度参考

统一任务输出层 CommandMux / Px4Adapter
    ├── 选择当前任务唯一输出
    ├── 做 world/ENU/NED 和消息转换
    ├── 维护 50~100 Hz PX4 setpoint
    ├── 做数据新鲜度、模式、急停和降级检查
    └── 发布 MAVROS/PX4 需要的消息
```

具体到任务一：

1. Fast-LIO2 适配器提供同一 world frame 的 odom 和 registered cloud。
2. SUPER/ROGMap 建立局部地图。
3. 采空区决策器选择下一个 viewpoint/frontier/局部目标。
4. SUPER FSM 产生 `quadrotor_msgs/PositionCommand`。
5. `super_to_px4_adapter` 将 position/velocity/acceleration/yaw 转为 PX4 接口。
6. 统一输出层发布给 MAVROS；SUPER 不直接抢占 PX4 通道。

具体到任务二，建议不要复用 SUPER 的 backup 语义来实现竖井返航。竖井任务应维护明确的状态：

```text
WAIT_START → DESCEND → HOLD/MEASURE → RETURN → ARRIVE → COMPLETE/ABORT
```

其中“测到地面/底部”是任务二的事件，不是 SUPER 的地图碰撞事件。Task 2 只输出同一种统一的 `DesiredState`，再由 `CommandMux` 转 PX4。

建议定义一个内部统一命令结构，而不是让调度器直接拼 MAVROS 消息：

```text
DesiredState {
  stamp
  source_task
  frame_id = world
  position[3]
  velocity[3]
  acceleration[3]
  yaw
  yaw_rate
  valid_until
  safety_level
  trajectory_flag
}
```

SUPER 的 `PositionCommand` 和竖井控制器都填充这个结构；只有 `Px4Adapter` 负责转成 MAVROS/PX4 消息。这样切换任务时不会出现两个节点同时发布 setpoint 的竞争。

## 12. 推荐的源码阅读顺序

按下面顺序读，最容易建立完整心智模型：

1. [`README.md`](/home/nuc/super_ws/src/SUPER/README.md)：先知道系统边界、ROS1 入口和已知限制。
2. [`PositionCommand.msg`](/home/nuc/super_ws/src/SUPER/mars_uav_sim/mars_quadrotor_msgs/msg/PositionCommand.msg)：明确最终命令字段。
3. [`fsm_ros1.hpp`](/home/nuc/super_ws/src/SUPER/super_planner/include/ros_interface/ros1/fsm_ros1.hpp)：看 ROS 定时器、输入、输出。
4. [`fsm.cpp`](/home/nuc/super_ws/src/SUPER/super_planner/src/super_core/fsm.cpp)：看状态变化。
5. [`super_planner.cpp` 的 `PlanFromRest/ReplanOnce`](/home/nuc/super_ws/src/SUPER/super_planner/src/super_core/super_planner.cpp)：看一轮规划如何串起来。
6. [`rog_map_ros1.hpp`](/home/nuc/super_ws/src/SUPER/rog_map/include/rog_map_ros/rog_map_ros1.hpp)：看输入点云和 odom 如何进入地图。
7. [`prob_map.cpp`](/home/nuc/super_ws/src/SUPER/rog_map/src/rog_map/prob_map.cpp)：看概率、射线和地图查询。
8. [`astar.cpp`](/home/nuc/super_ws/src/SUPER/super_planner/src/super_core/astar.cpp)：看离散路径如何产生。
9. [`corridor_generator.cpp`](/home/nuc/super_ws/src/SUPER/super_planner/src/super_core/corridor_generator.cpp) 与 [`ciri.cpp`](/home/nuc/super_ws/src/SUPER/super_planner/src/super_core/ciri.cpp)：看离散路径如何变成凸走廊。
10. [`exp_traj_optimizer_s4.cpp`](/home/nuc/super_ws/src/SUPER/super_planner/src/traj_opt/exp_traj_optimizer_s4.cpp)：看探索轨迹的变量、代价和优化。
11. [`backup_traj_optimizer_s4.cpp`](/home/nuc/super_ws/src/SUPER/super_planner/src/traj_opt/backup_traj_optimizer_s4.cpp)：理解 SUPER 的安全机制。
12. [`trajectory.h`](/home/nuc/super_ws/src/SUPER/super_planner/include/data_structure/base/trajectory.h)、[`cmd_traj.h`](/home/nuc/super_ws/src/SUPER/super_planner/include/data_structure/cmd_traj.h)：理解时间轴、拼接和 backup 切换。

阅读每个 cpp 时，优先追踪四类数据：`robot_state_`、`gi_`、`guide_path`、`cmd_traj_info_`。这比一开始逐行看所有可视化代码有效得多。

## 13. 适合当前阶段的实验路线

不要一开始就接 PX4 做飞行实验。建议按下面的可观测链路推进：

### 实验 A：只确认构建和消息

```bash
cd /home/nuc/super_ws
source /opt/ros/noetic/setup.bash
catkin_make -DBUILD_TYPE=Release
source devel/setup.bash
rosmsg show quadrotor_msgs/PositionCommand
```

### 实验 B：用静态点云验证 ROG-Map

打开可视化，确认以下内容在 RViz 中空间位置正确：

- occupied map
- inflated occupied map
- local map boundary
- robot odom
- 输入点云

最先排查的是 world frame、坐标原点、z 轴正方向和点云时间戳，而不是优化参数。

### 实验 C：给一个固定局部目标

先关闭任务决策器，只通过 `/goal` 或测试节点给一个固定目标，观察：

```text
目标 → PlanFromRest → A* → CIRI → ExpTrajOpt → BackupTrajOpt → PositionCommand
```

用 `rostopic echo /planning/pos_cmd` 检查位置、速度、加速度、yaw 和 `trajectory_flag`，用 `rostopic hz` 检查发布频率。

### 实验 D：人为制造重规划失败

在旧轨迹前方加入障碍点云，观察：

- `ReplanOnce` 是否失败或生成新轨迹。
- 是否进入 `trajectory_flag=2` 的 backup 段。
- 旧提交轨迹是否仍然保持发布。
- 是否能进入 `EMER_STOP`。

这是理解 SUPER 的关键实验，比单纯点击一个空旷目标更有价值。

### 实验 E：最后才接 PX4

先让 `Px4Adapter` 只发布/打印转换结果，不改变飞控模式；确认 world/ENU/NED、z 正负、yaw、时间戳和 setpoint 频率之后，再做 OFFBOARD 和低风险悬停测试。PX4 官方文档要求外部位姿和 Offboard setpoint 满足明确的消息、频率和坐标系约束。[4][5]

## 14. 你真正需要掌握的“核心问题”

如果目标是深刻理解，而不是只会运行，最终应该能回答下面这些问题：

1. 一个点云点在进入 ROG-Map 前究竟处于哪个坐标系？
2. 一个栅格从 UNKNOWN 变成 OCCUPIED 时，哪些 inflation counter 被改变？
3. A* 为什么只能产生 guide path，不能直接给 PX4？
4. CIRI 产生的多面体如何保证线种子被包含、障碍点被排除？
5. MINCO 的变量为什么是中间点和时间，而不是所有多项式系数？
6. 规划失败时，为什么继续执行旧提交轨迹仍可能安全？
7. backup 的切换点为什么不能太早，也不能太晚？
8. `PositionCommand` 和 PX4 的 `PositionTarget` 在语义上差在哪里？
9. 任务一和任务二切换时，谁拥有唯一的 PX4 输出权？
10. 如果 odom 停止、点云延迟、MAVROS 断开或任务节点崩溃，系统具体进入什么降级状态？

对你的项目，最后三个问题比“能否把 SUPER 编译起来”更重要。SUPER 只负责规划链路中的局部导航部分；任务调度、统一命令、PX4 适配、通信健康检查和失效安全必须由项目架构补齐。

## 15. 参考资料

[1] Ren et al., *Safety-assured high-speed navigation for MAVs*, Science Robotics, 2025. [DOI 页面](https://doi.org/10.1126/scirobotics.ado6187)

[2] HKU MaRS Lab, [SUPER 官方仓库与 README](https://github.com/hku-mars/SUPER)

[3] ZJU FAST Lab, [GCOPTER 官方仓库](https://github.com/ZJU-FAST-Lab/GCOPTER)

[4] PX4, [External Position Estimation](https://docs.px4.io/main/en/ros/external_position_estimation)

[5] PX4, [Offboard Mode](https://docs.px4.io/main/en/flight_modes/offboard)

[6] HKU MaRS Lab, [ROG-Map 官方仓库](https://github.com/hku-mars/ROG-Map)
