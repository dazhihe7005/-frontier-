# SUPER 目标生命周期协议设计

日期：2026-09-16

## 1. 问题与已确认因果链

任务一的自主决策器把 `have_active_goal_` 作为本地状态。当目标到达、前向目标交接、目标被前墙阻断、目标超时或任务被禁用时，它会清除该状态，但当前接口只允许向 SUPER 发布新的 `geometry_msgs/PoseStamped`，没有取消语义。

SUPER 的 ROS1 接口只订阅目标位姿。其 FSM 会把目标保存在 `gi_.goal_p` 中；轨迹结束后，如果飞行器距离这个旧目标仍大于硬编码的 0.1 m，FSM 会从 `FOLLOW_TRAJ` 返回 `GENERATE_TRAJ`，继续对旧目标执行 `PlanFromRest`。PX4 桥接器虽然在 `WAIT_MAP_CLOSURE` 等状态下丢弃这些指令并发布当前位置悬停，但这只保护了飞控，没有修复 SUPER 的内部目标状态。

两份 bag 已独立复现该链路。其中官方 SUPER 基线记录在 21.7 s 的连续 `WAIT_MAP_CLOSURE` 区间内没有新目标，却仍出现 316 次 `PlanFromRest` 和 1904 条 `/planning/pos_cmd`。因此本设计只修复“目标生命周期协议缺失”，不同时改变优化器、速度、安全走廊、frontier 或返航条件。

## 2. 设计目标

- 让决策器能够明确、原子地表达“设置目标”和“取消目标”。
- 取消后，SUPER 不得继续规划或发布旧目标轨迹。
- 取消与紧随其后的新目标必须保持顺序，不能依赖两个 ROS topic 的跨连接到达顺序。
- SUPER 重启或晚订阅时能够恢复决策器当前发布的生命周期状态。
- 保留官方 `/goal` 点选接口作为独立演示模式，但任务一运行时只能启用一种目标输入模式。
- PX4 桥接器继续承担无有效任务指令时的悬停保护，但不参与修复 SUPER 内部状态。

## 3. 接口所有权

目标命令是 SUPER ROS 接口的一部分，因此消息定义放在 `super_planner` 包内，而不让官方规划核心反向依赖项目包。

新增 `super_planner/GoalCommand.msg`：

```text
uint8 SET_GOAL=1
uint8 CANCEL_GOAL=2

std_msgs/Header header
uint8 command
uint64 goal_id
geometry_msgs/Pose goal
string reason
```

语义如下：

- `SET_GOAL`：`goal_id` 必须非零，`goal` 是 `header.frame_id` 下的新目标。
- `CANCEL_GOAL`：`goal_id=0` 表示无条件清除 SUPER 当前目标；非零值表示取消对应目标。
- 决策器是任务模式下唯一发布者，并单调递增 `goal_id`。
- 命令发布器使用 latch；SUPER 晚订阅或重启后会收到最新的“当前目标”或“当前无目标”状态。
- 所有设置和取消事件使用同一 topic，依赖单发布者 TCPROS 顺序，避免跨 topic 竞争。

默认任务话题为 `/mine_uav/super/goal_command`。

## 4. 决策器状态转换

`SuperExplorationDecider` 新增两个唯一出口：

- `setActiveGoal(pose, reason)`：增加 `goal_id`，发布 `SET_GOAL`，再更新本地活动目标状态。
- `cancelActiveGoal(reason, unconditional)`：发布 `CANCEL_GOAL`，再清除本地活动目标状态。

所有直接写 `have_active_goal_ = false` 的生命周期分支必须改为调用取消出口，包括：

- 到达目标；
- 前向目标交接；
- 已确认前墙阻断当前前向目标；
- 目标超时；
- 任务调度器禁用任务一；
- 已有目标期间 Fast-LIO2 同步数据转为不新鲜，决策器进入 `WAIT_DATA`；
- 返航到家并结束任务；
- 清理或重启任务状态。

同一个决策周期内发生“取消旧目标后立刻设置新目标”时，两条命令在同一发布连接上按顺序发送。`WAIT_MAP_CLOSURE`、`WAIT_FRONTIER`、`WAIT_MODEL_COVERAGE` 和 `DISABLED` 状态必须以最近一次命令为 `CANCEL_GOAL`。

节点启动并创建 latched 发布器后，必须先发送一次 `goal_id=0` 的无条件取消。这样决策器重启但 SUPER 未重启时，不会继续执行上一进程遗留的目标；随后如果任务有效，正常决策周期再发布新目标。

## 5. SUPER FSM 行为

SUPER 核心新增一个最小、与 ROS 无关的 `cancelGoal()` 状态转换：

- `gi_.new_goal = false`；
- `plan_from_rest_ = false`；
- `finish_plan = true`；
- 从 `GENERATE_TRAJ`、`FOLLOW_TRAJ` 或 `EMER_STOP` 转到 `WAIT_GOAL`；
- 清除可视化路径；
- 不修改地图，不重启规划器，不伪造当前位置目标。

SUPER ROS1 节点使用 `ros::AsyncSpinner(0)`，目标、主 FSM、重规划和指令定时器可以并发执行。因此 FSM 还必须维护一个内部单调递增的 `goal_epoch`：每次设置或取消目标都推进 epoch；一次规划开始时保存 epoch，规划结束准备提交状态或轨迹前重新检查。若 epoch 已变化，该结果属于被取消或替换的旧目标，必须丢弃，不能把 FSM 从 `WAIT_GOAL` 改回 `FOLLOW_TRAJ`，也不能发布旧轨迹。

目标信息、epoch、活动标志和状态转换由一个短临界区保护。`PlanFromRest` 与 `ReplanOnce` 等耗时计算在临界区外执行，只在读取输入快照和提交结果时加锁，避免为修复取消竞争而阻塞 100 Hz 指令输出。指令回调发布前也必须校验活动 epoch；允许取消回调之前已经完成的一条在途消息，不允许取消提交完成后再生成旧目标消息。

ROS1 包装层订阅统一命令：

- 收到合法 `SET_GOAL` 时调用现有 `setGoalPosiAndYaw()`；
- 收到匹配当前目标的取消或无条件取消时调用 `cancelGoal()`；
- 收到未知命令、零编号设置目标、非有限位姿或针对非当前编号的取消时记录告警并拒绝；
- 取消回调完成后，由于 FSM 已处于 `WAIT_GOAL`，100 Hz 指令定时器不得再发布旧 `PositionCommand`。

ROS1 包装层在接受取消时同时把本地 `traj_finish_` 复位为 `false`，只用于避免下一目标第一次取指令时读取上条轨迹的完成标志；禁止借此发布任何取消前的轨迹点。

配置增加 `goal_command_en` 和 `goal_command_topic`。任务一配置启用统一命令并关闭 `click_goal_en`；官方点选示例保持原行为。初始化时继续要求恰好启用一种目标输入模式。

## 6. 安全性与错误处理

- 不用“当前位置伪目标”模拟取消，因为它仍会触发规划并掩盖生命周期缺失。
- 不通过调大 0.1 m 到点阈值、延长超时或过滤更多规划指令修复此问题。
- PX4 桥接器的状态悬停逻辑不删除；它是下游失效保护，而不是协议实现。
- 不假设 ROS 回调串行执行。取消提交后，任何携带旧 `goal_epoch` 的规划结果、状态转换和位置指令都必须被拒绝。
- 不把整个规划调用包在全局互斥锁内；否则一次耗时重规划会阻塞 100 Hz 指令发布，形成新的控制时序问题。
- `goal_id` 只用于防止误取消当前目标，不代替 ROS 时间戳，也不改变 SUPER 内部 `trajectory_id`。

## 7. 测试顺序与验收标准

实现严格采用测试先行。

### 7.1 RED：状态机失败测试

先增加 SUPER FSM 单元测试，构造 `FOLLOW_TRAJ` 且保留旧目标的状态。测试要求调用尚不存在的 `cancelGoal()` 后进入 `WAIT_GOAL`、清除 `new_goal`、禁止继续规划。实现前测试应因接口不存在或断言失败而处于 RED。

再增加 epoch 并发回归测试：开始一次旧目标规划，规划完成前执行取消，随后模拟旧规划结果提交。测试必须证明旧 epoch 无法恢复 `FOLLOW_TRAJ` 或获得发布许可；新目标的新 epoch 可以正常提交。该测试不依赖 ROS 线程调度，使用确定性的开始令牌、取消和提交顺序复现竞争。

### 7.2 GREEN：最小状态转换

仅实现取消状态转换，使状态机测试通过。不得在这一阶段修改规划器参数或决策逻辑。

### 7.3 协议与决策器测试

- `SET_GOAL` 使用非零且递增的编号。
- 目标到达、交接、阻断、超时和禁用分支均发布取消。
- `CANCEL(old) → SET(new)` 的发布顺序固定。
- 非当前编号取消不能终止新目标。
- 无条件取消能够清除重启前残留目标。
- 取消期间正在计算的旧目标规划结果不能在取消后提交。

### 7.4 构建与静态检查

- `super_planner` 和 `mine_uav_control` Release 构建成功。
- 新增测试和现有测试全部通过。
- 官方点选示例在 `click_goal_en=true`、`goal_command_en=false` 下仍可构建运行。

### 7.5 相同场景 A/B

使用相同的 40×40×30 m 采空区、30 m 仿真雷达、同一启动顺序和参数：

- 进入 `WAIT_MAP_CLOSURE` 后一个控制周期内停止旧目标 `/planning/pos_cmd`；
- 整个无目标等待区间中，对旧目标的 `PlanFromRest` 次数为 0；
- 取消后设置新目标时，只能规划新目标；
- PX4 setpoint 保持为当前位置悬停，不出现取消瞬间的位置跳变；
- 不以本修复承诺解决独立的线搜索失败、位置约束失败或返航判据问题。

## 8. 影响文件边界

运行工作区预计修改：

- `SUPER/super_planner/msg/GoalCommand.msg`
- `SUPER/super_planner/ros/ros1.CMakeLists.txt`
- `SUPER/super_planner/ros/ros1.package.xml`
- 当前选定 ROS1 后的 `super_planner/CMakeLists.txt`、`package.xml`
- `SUPER/super_planner/include/fsm/fsm.h`
- `SUPER/super_planner/include/fsm/goal_lifecycle_guard.h`
- `SUPER/super_planner/src/super_core/fsm.cpp`
- `SUPER/super_planner/include/ros_interface/ros1/fsm_ros1.hpp`
- SUPER FSM 单元测试
- `mine_uav_control` 的决策器头文件、实现、配置、构建依赖与相关测试
- 任务一 README 和根因报告

项目仓库保存 `mine_uav_control` 源码、配置、测试、本文档和一份可对官方 SUPER 基线应用的独立补丁。不会提交 bag、构建产物或临时诊断打印。

## 9. 后续根因调查纪律

本协议通过全部验收后，才开启下一条独立调查。候选问题按证据优先级为：非退化时间输入下的 L-BFGS 线搜索失败、位置约束失败、目标附近轨迹连续性和地图闭合后未返航。每一项都必须重新执行“稳定复现 → 源码传播路径 → 失败测试或只读诊断 → 单变量修复 → 同场景 A/B”，不得把本协议修复的改善外推为整套任务已经可安全真飞。
