# 任务一目标交接轨迹跳变：因果链与 A/B 验证（2026-09-17）

状态：固定 40×40×30 m SITL 场景的两次修复回归（run13/run14）均已完成探索、返航和 `AUTO.LOITER`；连续目标交接的 SUPER 速度指令突降消失。此结论只针对该场景、该版本和录包采样，不是实机安全认证。动态高度合同仍需验收。

## 从现象追到原因

基线 run11 的 `/planning/pos_cmd` 有 14 次相邻有效采样（间隔不超过 60 ms）速度差大于 0.2 m/s，最大 1.002 m/s；最大相邻位置目标变化 0.283 m。14 次全部在新 `SET_GOAL` 之后 17～45 ms，分别发生于前进接力和返航面包屑目标交接。PX4 实际速度最大相邻变化 0.123 m/s，说明飞控在理想仿真中部分滤掉了跳变，但不能替代规划指令连续性。

完整代码链：`SuperExplorationDecider::publishGoal` 对每一个新目标先调用 `cancelActiveGoal("replace_goal")`；SUPER ROS1 回调收到 `CANCEL_GOAL` 后 `Fsm::cancelGoal` 令旧 epoch 失效、状态转 `WAIT_GOAL`。随后 `SET_GOAL` 激活新 epoch，主状态机经 `WAIT_GOAL → GENERATE_TRAJ` 调用 `SuperPlanner::PlanFromRest`。该函数给 `generateExpTraj` 传空的上一条轨迹；后者把初始 `StatePVAJ` 速度、加速度置零，从当前定位点开始新轨迹。于是正在以约 1 m/s 运行的旧指令被突然换成速度零、位置略回跳的新指令。这些分别由录包时间相关性、目标消息顺序和上述源码路径直接支持，而非仅凭轨迹外观推测。

SUPER 原有 `ReplanOnce` 能以已提交轨迹未来时刻的 P/V/A 状态作为新轨迹初态；问题不在优化器必然无法平滑，而在任务层每次交接都强制取消，导致没有机会使用这一路径。不能通过 PX4 桥接器逐点限幅来掩盖上游不连续，否则会破坏轨迹与安全走廊的一致性。

## 只修改对应机制

- 前进目标已证明为已知空闲且可达时，以及沿已观测面包屑返航时，决策器直接发布新 `SET_GOAL`，不先发 `CANCEL_GOAL`。第一次启动、前墙预占、模型闭合转返航、失联、任务关闭及任务完成仍执行原有显式取消或独立起始语义。
- SUPER 在接到直接替换且旧轨迹仍是当前已提交轨迹、处于 `FOLLOW_TRAJ`、尚未进入备份轨迹且剩余时间大于 0.4 s 时，给新 goal epoch 转移旧轨迹的指令执行资格，保持 `FOLLOW_TRAJ`，用 `ReplanOnce` 从旧轨迹的未来状态连续规划。旧 goal 的异步规划结果仍因 epoch 失效被拒绝。条件不满足时仍回到 `PlanFromRest`，不强行延续过期或备份轨迹。
- `CANCEL_GOAL` 的停止旧目标/禁止旧指令语义没有更改；用户/遥控器接管的桥接器锁存保护也没有更改。没有在输出端添加平滑滤波器。

本地源码分别位于 `frontier-upload/src/super_exploration_decider.cpp`、`super_ws/src/SUPER/super_planner/src/super_core/fsm.cpp` 和 `super_planner.cpp`。项目仓库与实际 Catkin 工作区是两份目录，改动已同步到实际工作区进行构建和仿真。

SUPER 侧增量修改另存为 `patches/super_goal_continuous_retarget.patch`，已在包含目标生命周期协议的 SUPER 基线提交 `71eddfd` 上通过 `git apply --check`；它不是可直接打到港大官方 `2ad3419` 的独立补丁。当前 NUC 的 SUPER 工作区保留着先前任务相关改动，不能把这份增量补丁误称为完整的 SUPER 源码快照。

## 同一场景对比

使用项目脚本 `scripts/analyze_task1_trajectory_continuity.py` 对两个 ROS bag 采用相同计算方法；跳变诊断阈值 0.2 m/s 不是飞行安全阈值。

| 指标 | 基线 run11 | 修复 run13 | 独立复测 run14 |
| --- | ---: | ---: | ---: |
| `SET_GOAL` / `CANCEL_GOAL` | 18 / 20 | 18 / 2 | 18 / 3 |
| 未先取消的直接目标接力 | 0 | 16 | 16 |
| SUPER 相邻速度差最大值 | 1.002 m/s | 0.0187 m/s | 0.0151 m/s |
| SUPER 相邻位置目标差最大值 | 0.283 m | 0.0194 m | 0.0157 m |
| SUPER 速度差 >0.2 m/s 次数 | 14 | 0 | 0 |
| PX4 实际相邻速度差最大值 | 0.123 m/s | 0.0496 m/s | 0.134 m/s |
| 探索/返航/任务完成 | 完成 | 完成 | 完成 |
| 桥接器故障状态 | 0 | 0 | 0 |

run13 全程 SUPER 命令日志的最大速度约 1.007 m/s、加速度 1.285 m/s²、jerk 4.998 m/s³；PX4 位姿范围 x≈-0.26～43.50 m、y≈-1.04～0.19 m、z≈0.99～1.66 m。两次修复回归都只有一次超过 0.1 s 的 SUPER 指令空档，分别约 6.03 s、7.45 s，发生于确认前墙后显式取消、等待地图闭合再开始返航；它不是连续目标交接期间的空档。桥接器在空档报告 `HOLD_COMMAND_TIMEOUT`/`WAIT_SUPER_COMMAND`，无 `GEOFENCE`/`FAULT`。这段保持等待仍需单独评估 PX4 模式与安全策略，不能把“目标交接无跳变”扩展为“全任务从不停顿”。run14 PX4 实际速度最大相邻变化高于 run11；本修复确证改善的是规划指令连续性，不能声称每次仿真的飞控所有运动指标都单调改善。

验证命令：

```bash
/usr/bin/python3 scripts/analyze_task1_trajectory_continuity.py /home/nuc/task1_logs/failure_cases/goal_lifecycle_fixed/task1_40m_auto_frame_run11.bag
/usr/bin/python3 scripts/analyze_task1_trajectory_continuity.py /home/nuc/task1_logs/task1_continuous_handover_run13.bag
/usr/bin/python3 scripts/analyze_task1_trajectory_continuity.py /home/nuc/task1_logs/task1_continuous_handover_run14.bag
```

另已通过 SUPER `fsm_goal_cancellation_test` 共 19 项（含接力与退化测试）、决策器 `test_super_goal_publication` 3 项。bag 保存在 NUC 本地，不提交 GitHub。仍需复测不同场景、规划失败时的安全退化、真实 MID360/Fast-LIO2 和实机；尤其规划高度上界与 PX4 1.8 m 桥接围栏尚未统一。
