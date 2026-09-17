# 任务一：等待悬停与规划高度约束的因果链修复（2026-09-17）

## 原因与改动

1. **等待时下沉。** 旧 `super_px4_command_bridge.cpp::makeHoldTarget()` 每周期把最新 PX4 实际位姿当作悬停目标；run16 的 6.11 s 指令空档中，实际 z 从 1.283 降至 0.768 m，输出目标也几乎同步下移。现在进入等待时只捕获一次本地位置目标并持续发送；新有效轨迹接管、离开 OFFBOARD 或任务退出时清除锁存。未修改手动接管的受控退出语义。
2. **未来轨迹超高。** SUPER 的走廊生成原来按种子线附近 `corridor_bound_dis` 扩张，并由地图局部边界限制；`getSeedBBox` 中虚拟顶板裁剪原本注释关闭，PX4 的 1.8 m 高度上界并未成为规划约束。任务一配置新增规划坐标系 `flight_min_z=-2.0`、`flight_max_z=1.8`，走廊盒在考虑 0.40 m 机体半径后限制中心可行域。生成点、线和起点补充多面体都检查包络，界外种子直接规划失败，而不是把越界点伪装成可达点。
3. **优化器软约束剩余误差。** 仅限制走廊后，run19 完整多项式仍有最多 0.0022 m 的未来高度越界，说明走廊优化不是严格数学不等式。现在 SUPER 在四个新轨迹提交路径上，用 Bernstein 控制点凸包及 de Casteljau 细分认证整条 z 多项式；无法证明全段位于包络内时拒绝候选，保留旧已提交轨迹或进入等待。它不对 PX4 输出逐点夹紧，也不把坏轨迹修饰成好轨迹。桥接器原有围栏仍是最后一道保护。

## 同场景证据

场景为 `task1_40m_platform_sitl.launch` 的 40×40×30 m 采空区；本次 SITL 对齐 `alignment_z=0`，PX4 局部 z 上限 1.8 m。使用 `scripts/analyze_task1_vertical_trajectory.py --alignment-z 0` 对采样指令、完整发布多项式和等待期实际位姿复算。

| 条件 | 结果 | 已发布多项式最高 z | 前墙等待期间实际 Δz / Δx |
|---|---|---:|---:|
| run16，原动态 hold 与原走廊 | COMPLETE | 1.885 m | -0.515 / +0.925 m（6.11 s） |
| run17，仅固定 hold | COMPLETE | 1.956 m | -0.071 / +0.039 m（6.66 s） |
| run18，固定 hold + 限制走廊 | COMPLETE | 1.7996 m | -0.064 / +0.011 m（6.47 s） |
| run19，同配置独立重复 | COMPLETE | 1.8022 m | -0.050 / +0.054 m（6.57 s） |
| run20，走廊 + 提交前硬校验 | COMPLETE，最终 AUTO.LOITER，无桥接故障 | 1.7818 m；超界条数 0 | -0.068 / +0.025 m（6.97 s） |

run19 是重要反例：走廊约束单独使用不构成整条轨迹的硬保证。run20 中没有产生需要硬校验拒绝的候选，因此只能说代码路径已接入且针对越界/段内峰值的 4 项单测通过；不能以 run20 的“零拒绝”声称在各种障碍条件下都不会停顿。等待期还会有约 5–7 cm 实际高度跟踪误差，目标本身已保持不变；这不等于实机定位或高度控制精度。run20 的 rosbag 因录制退出时索引未自动封口，已执行 `rosbag reindex`，保留为 `.bag.active`；没有删除原始证据。

## 代码、复现与边界

- 主仓库的桥接器源码、集成测试和 `config/super_task1.yaml` 已更新；SUPER 的增量改动保存在 `patches/super_task1_flight_z_corridor.patch` 与 `patches/super_task1_flight_z_contract.patch`。先应用既有 SUPER 目标生命周期/连续接力补丁，再按顺序应用这两个补丁；不要把补丁直接套到与当前依赖不符的港大官方原始提交上。
- 验证：`rostest mine_uav_control super_bridge_manual_override.test` 通过；`flight_z_contract_test` 4/4；两次走廊修复闭环与一次硬校验闭环均通过。仿真结束后专用 11312 master、Gazebo/PX4 均已关闭。
- 高度界限目前是 SUPER `camera_init` **规划坐标**。只有本 SITL 的 `alignment_z=0` 时，它与 PX4 本地围栏数值一致。实机若对齐偏移为 `alignment_z`，必须在启动前使用 `z_planner = z_px4 - alignment_z` 转换上下界，且定位融合、对齐稳定和有桨真机均未由本轮验证。当前真实任务一不能据此宣布飞行安全；动态对齐高度合同仍需单独实现。
- 本轮仅检查固定场景和规划位置高度；其他采空区几何、速度/加速度连续性、障碍距离和真实雷达数据需要继续独立验收。

本地证据文件：`/home/nuc/task1_logs/task1_fixed_hold_run17.bag`、`task1_bounded_corridor_run18.bag`、`task1_bounded_corridor_run19.bag`、`task1_hard_z_contract_run20.bag.active`。bag 不上传 GitHub。
