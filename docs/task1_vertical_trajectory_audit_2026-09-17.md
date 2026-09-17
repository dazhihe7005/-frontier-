# 任务一仿真高度来源与规划高度审计（2026-09-17）

## 高度从哪里来

当前 `task1_40m_platform_sitl.launch` 不运行真实 MID360/Fast-LIO2 位姿解算。`sitl_localization_adapter.py` 将 PX4 经 MAVROS 发布的 `/mavros/local_position/odom` 复制为 `/Odometry`，坐标名改为 `camera_init`，同时发布零平移/零转角的对齐变换。决策器和 SUPER 用这个 `/Odometry` 的 z；SUPER 产生的 z 经 `super_px4_command_bridge.cpp` 加对齐 z 后作为 PX4 本地位置目标。因此当前仿真里看到的高度是 PX4 SITL 本地估计坐标相对其原点的高度，不是地面净空，也不是直接取 MID360 点云的 z。

本轮仿真运行参数读取到 `EKF2_HGT_REF=1`（GPS 高度参考）、`EKF2_GPS_CTRL=7`、`EKF2_BARO_CTRL=1`、`EKF2_EV_CTRL=15`、`EKF2_RNG_CTRL=1`。`EV_CTRL` 允许外部视觉输入，不代表有视觉观测实际参与融合：此 SITL 没有 `/mavros/vision_pose/pose_cov` 的发布节点。可准确称为“PX4 EKF 的本地高度输出；本仿真以模拟 GPS 高度为参考并启用气压计辅助”，不能称为 Fast-LIO2 高度融合测试。Gazebo 雷达适配器反而用 `/Odometry` 将射线注册到全局点云，雷达只提供障碍/建图输入。

## 检测方法

`scripts/analyze_task1_vertical_trajectory.py` 对 bag 同时检查：

- `/planning/pos_cmd` 的相邻高度目标差、用速度积分校正后的不连续残差，以及指令空档；
- `/planning_cmd/poly_traj` 每段七阶 z 多项式在端点和导数零点处的极值、段边界跳变；
- PX4 `/mavros/local_position/pose` 与 `/mavros/setpoint_raw/local` 在等待期的实际/目标高度。

多项式消息里的 `header.frame_id=world` 是 SUPER 写死的标签；数值仍按当前仿真任务的 `camera_init` 合同解读。围栏比对必须提供实测的对齐偏移；本仿真偏移为 0，所以以 `--alignment-z 0 --max-height 1.8` 分析。早期 run04 和 run13/run14 未录制多项式，只能对其采样指令作结论；本轮 run16 完整录制多项式。

## 证据与结论

| 记录 | 采样目标 z 范围 | 最大相邻 z 差 | >0.05 m 的相邻高度残差 | 说明 |
|---|---:|---:|---:|---|
| 历史故障 run04 | 1.232–2.912 m | 0.720 m / 0.029 s | 6 | 确有高度跳变与越界，不能说此类问题从未出现 |
| 修复后 run13 | 1.050–1.640 m | 0.0030 m | 0 | 完成闭环，前墙附近有 6.03 s 空档 |
| 修复后 run14 | 0.850–1.514 m | 0.0049 m | 0 | 完成闭环，空档 7.45 s |
| 本轮 run16 | 0.750–1.563 m | 0.0025 m | 0 | 完成闭环，空档 6.11 s，完整多项式已审计 |

run16 共收到 1594 次多项式消息（含重复/刷新）；段内高度边界最大不连续约 `8.5e-14 m`，可视为数值零。整条已发布未来轨迹的最高 z 达 `1.885 m`，有 **2 条不同的轨迹**的未来段超过桥接器 1.8 m 上限：分别在仿真 `50.353 s`、`57.972 s` 发布；对应峰值预定在 `57.134 s`、`64.354 s` 才到达，但分别在 `50.420 s`、`58.016 s` 已被下一条轨迹替代。故这不是飞机实际飞到 1.885 m，也没有在本轮执行中触发高度围栏；仍证明规划层与桥接层高度合同尚不一致，下一轮应分析为什么目标高度仅 1.356–1.548 m 时，规划器仍能生成超上界的未来轨迹。不能以桥接器逐点夹紧掩盖规划层问题。

前墙确认后的**等待可以发生**，但等待时高度下降不可当作正常悬停。run16 在约 6.11 s 无新 SUPER 指令期间，PX4 实际 z 从 1.283 降至 0.768 m（-0.515 m），同时 x 前移 0.925 m；桥接器的 hold 位置目标 z 也从约 1.282 跟随下降到 0.768 m。run13/run14 分别下降 0.495/0.587 m。`makeHoldTarget()` 每个周期取 `latest_local_pose_`，所以这个“hold”实际上随飞机当前位置移动，不能提供固定高度的悬停参考；最初使飞机开始漂移的扰动尚未单独隔离。等待结束后规划器从已降低的当前位置继续规划，这与“规划输出瞬间向下跳 0.5 m”是两件事。

run16 最终 `/mine_uav/task1/command_status=TASK1_COMPLETE`。为保证诊断运行可重现，仅修复仿真操作员的启动交接竞态：任务桥第一次报告 `/mine_uav/task1/command_ready=true` 前持续发布起飞悬停目标，此后永久让出发布权。上一条 run15 因操作员过早停止 setpoint、首条 SUPER 轨迹未出前 PX4 离开 OFFBOARD 而锁存，不能作为任务闭环样本。此修复不改真机飞行控制/生产桥接器。

## 尚未解决的事项

1. 规划器内部高度约束与 PX4 本地高度围栏一致化，包括动态坐标对齐；先定位 1.885 m 未来轨迹的生成层和可行域，再修改。
2. 将前墙等待中的固定悬停目标与首个等待时刻绑定，而不是每周期追随实际位置；动手前须补充单测/仿真验证手动接管、任务退出和下一目标交接的语义。
3. 真实 Fast-LIO2 位姿融合、MID360 实测高度和有桨真机均未由本次 SITL 证明。

本地证据：`/home/nuc/task1_logs/task1_vertical_poly_run16.bag`，旧样本在 `/home/nuc/task1_logs/` 和 `failure_cases/goal_lifecycle_fixed/`。bag 体积较大，不上传 GitHub。
