# 白象山 1:1 地图：全图探索验收

目标是探索从起点连通且无人机能保持 1 m 安全半径的所有巷道、岔路和尽头；不把单次规划失败、看见部分点云或一条主巷道跑通当作“完成”。

## 可重复的离线参考

从原始 Gazebo 碰撞 OBJ 生成只读参考：

```bash
/home/nuc/.local/opt/blender-portable/blender-5.2.0-linux-x64/blender \
  --background --python tools/build_baixiangshan_reference.py -- \
  --resolution 0.5 \
  --output runtime/map_reference/baixianshan_reachable_0p5m.json.gz
```

`coverage_audit:=true` 时，`diagnostic_map_coverage.py` 只读 Gazebo 真值位姿和原始 MID360 射线，每 15 个仿真秒写入 `runtime/map_reference/latest_coverage.json`。这些数据只用于验收，绝不反馈给 FAST-LIO2、Voxblox、GBPlanner、PX4 或安全制动。`visible_cells` 是射线可见性代理量；`visited_cells` 是实际飞经的网格。`largest_unseen_components` 用于定位未探索的连通巷道。

同一审计开关还以约 10 Hz 记录真值机体中心轨迹到 `runtime/map_reference/truth_trajectory_*.csv`。飞行后用实际 Gazebo 碰撞网格独立核验雷达盲区中的顶板、地面、岩壁和管线距离：

```bash
/home/nuc/.local/opt/blender-portable/blender-5.2.0-linux-x64/blender \
  --background --python tools/audit_truth_mesh_clearance.py -- \
  --trajectory runtime/map_reference/truth_trajectory_YYYYMMDD_HHMMSS.csv \
  --output runtime/map_reference/mesh_clearance_audit.json
```

报告保存碰撞网格 SHA-256、探索阶段采样最小距离、逐类障碍最小距离、采样间隔和小于 1 m 的计数；`piecewise_linear_clearance_lower_bound_m` 只约束**采样点间直线插值**，不能证明未采样的真实连续轨迹。审计节点不发布任何控制话题。要宣称 1 m 硬安全已被全过程验证，必须同时审查完整架次、采样间隔、几何模型一致性和定位误差，不能仅凭 MID360 点云最小值。

2026-09-26 的组合试验在斜地面侧支处，单束仿真下视约报 1.32–1.57 m，但网格真值最近地面仅 1.07256 m；单束垂直量程不能代表球形包络的最近三维距离。该次虽无采样点低于 1 m，只有约 7 cm 净空余量且陷入顶板/地面脱险循环，必须判为**未通过完整安全验收**。该仿真下视传感器也不是已确认安装在真机上的硬件，真机安全论证不能依赖它。

当前参考是 0.5 m 的二维栅格，采用最高地面和最近岩壁/管线的几何距离。它不能证明连续三维通路可飞，也可能漏掉上下重叠巷道。参考中未与起点连通的栅格，须区分真实不可达、坡度过大与栅格离散误差，不能未经核查直接从“全图”分母剔除。

## 完成条件

1. MID360S 原始点云经 FAST-LIO2 和 Voxblox 进入 GBPlanner2；PX4 外部视觉 XY 与轨迹输出持续有效，禁止真值进入控制链路。
2. 全过程 1 m 三维安全半径不被突破；地面盲区由当前仿真下视测距守护。失去定位、测距或路径安全证据时停住而不是继续飞。
3. 所有经几何复核的可达分支和尽头都被观察，未见区域的每个连通块都有“真实不可达/传感器盲区/尚未探索”的可审计归因；未归因的可达区域为零。
4. 从冷启动飞到探索结束并安全返航，保存覆盖率时间序列、轨迹、速度、定位误差、最小净空、停滞及恢复次数，至少重复一次。只有完整运行证据通过才能声明达标。

仿真中 `time_budget_limit=3600` 秒只是避免 900 秒提前返航；真实两电池的容量、压降、续航及安全返航阈值尚未通过实测验证，不能据此推断真机能单架次完成。
