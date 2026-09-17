# 任务一高度约束合同：当前风险与下一步实现

状态（2026-09-17）：**已完成历史 bag 的独立审计；尚未修改 SUPER 规划器，不能宣称高度约束已闭环。** 此文只针对任务一，任务二竖井纵向运动必须另行设计。

## 唯一高度基准

桥接器的 `min_height`、`max_height` 是 PX4 本地 ENU 坐标的参考点高度，不是地面测距高度、Gazebo 世界高度，也不是相对于任务 home 的高度。SUPER `/planning/pos_cmd` 在 `camera_init`；对齐桥给出的 `alignment_z` 满足：

```text
z_px4_local = z_camera_init + alignment_z
z_camera_init_allowed = [min_height - alignment_z, max_height - alignment_z]
```

真机的 `alignment_z` 在 Fast-LIO2 与 PX4 对齐时确定，不能假定为零。SITL `sitl_localization_adapter.py` 明确发布单位变换，因此旧 SITL bag 可用 `--alignment-z 0` 复算；以后录 bag 应直接包含 `/mine_uav/task1/fastlio_to_px4_alignment`，避免手工假设。

## 已确认的因果链

1. 决策器只把观察目标限制在 home 附近；SUPER 可以为避障和返航重新规划完整轨迹，因此“目标高度合理”不代表每个轨迹样本合理。
2. `super_task1.yaml` 的 ROG-Map 虚拟顶板为 4.0 m。ROG-Map 对它做网格量化及膨胀，`getGridType(Vec3f)` 与 `isOccupiedInflate` 的边界判定也不同；它不等于 PX4 `z≤1.8 m`。
3. 官方 `CorridorGenerator::getSeedBBox` 中收紧上下界的两行目前被注释，生成的走廊盒不能作为 PX4 高度围栏证明；备份走廊和空走廊还须分别覆盖。仅恢复这两行也没有解决动态 `alignment_z` 和连续轨迹验证。
4. 桥接器在转换坐标后才检查/小幅夹紧越界高度。run04 入口返航时目标附近的假障碍导致 SUPER 把内部目标上抬，桥接器后来拒绝高度超限指令；这不是飞控突然抬高。仿真几何过滤已消除这个场景的假回波，但规划层不一致仍存在。

用 `scripts/analyze_task1_height_contract.py` 对历史 bag 按同一 PX4 本地 `[-2.0,1.8] m` 限制复算：

| bag | SUPER 有效指令数 | 高于 1.8 m | 最高指令高度 | 结论 |
| --- | ---: | ---: | ---: | --- |
| run04（失败反例） | 15,028 | 3,745 | 2.912 m | 不通过 |
| run11（固定场景已通过） | 12,217 | 0 | 1.600 m | 抽样通过 |

```bash
/usr/bin/python3 scripts/analyze_task1_height_contract.py \
  /home/nuc/task1_logs/failure_cases/goal_lifecycle_fixed/task1_40m_breadcrumb_run04.bag --alignment-z 0
/usr/bin/python3 scripts/analyze_task1_height_contract.py \
  /home/nuc/task1_logs/failure_cases/goal_lifecycle_fixed/task1_40m_auto_frame_run11.bag --alignment-z 0
```

此脚本检查发布样本，不检查采样间的连续多项式轨迹；run11 抽样通过也不能证明规划器本身已受约束。

## 实现与验收门槛

1. 由一个任务一高度约束源使用最新有效对齐，生成带 `camera_init` 帧及对齐版本的上下界；未获得对齐、帧不符、对齐突变时，不允许开始新规划/进入 OFFBOARD，并取消已提交轨迹。PX4 侧桥接围栏保持独立生效。桥接层的对齐突变锁存/退出已单独完成，SUPER 内部的旧轨迹取消与动态走廊约束仍待实现。
2. 在 SUPER 的主、备份和紧急轨迹所用安全走廊中加入同一相机系高度半空间；对起点、终点、走廊交集和可行性做显式验证。不能只把最终 `/planning/pos_cmd` 高度截断，因为这会破坏速度、加速度与避障轨迹的连续性。
3. 在轨迹提交前对连续多项式的高度极值做检查，并在对齐变化时废止旧轨迹；采样监控只是最后的运行时诊断。由于这是项目特定功能，先做可单独回退的补丁与单元测试，不无条件修改上游 SUPER 默认行为。
4. 先在无障碍、入口平台、近顶板障碍、不同起飞原点/`alignment_z`、返航及对齐失效案例中做 A/B；要求规划高度、桥接状态、任务完成与地图覆盖同时通过，不可仅通过加大 PX4 围栏或关闭备份轨迹获得“通过”。

当前阶段只完成了第 0 步（测量与合同定义），第 1–4 步待实现并仿真回归；任务一仍不准直接用于有桨真机自主飞行。
