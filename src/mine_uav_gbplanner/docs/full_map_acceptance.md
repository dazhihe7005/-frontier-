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

同一审计开关还把 GBPlanner 发出的原始轨迹，与近同时刻的规划里程计、Gazebo 真值位姿配对，写入 `planned_trajectory_*.csv`。`x/y/z` 是用于**离线诊断**的近似真值坐标投影；它受锚点时间差、定位和偏航误差影响，不能回流到规划或控制。要分析管线附近的规划点，可运行：

```bash
/home/nuc/.local/opt/blender-portable/blender-5.2.0-linux-x64/blender \
  --background --python tools/audit_planned_mesh_clearance.py -- \
  --planned runtime/map_reference/planned_trajectory_YYYYMMDD_HHMMSS.csv \
  --x-min 105 --x-max 125 --y-min 2 --y-max 9 \
  --output runtime/map_reference/planned_mesh_clearance.json
```

该报告仅能区分“候选计划在真实网格上可能贴近障碍”与“实际飞行另有误差”这类根因候选；不能把近似投影直接当作在线碰撞证书。判断 PX4 实际净空仍以独立真值轨迹网格核验为准。

同一审计开关还以约 10 Hz 记录真值机体中心轨迹到 `runtime/map_reference/truth_trajectory_*.csv`。飞行后用实际 Gazebo 碰撞网格独立核验雷达盲区中的顶板、地面、岩壁和管线距离：

```bash
/home/nuc/.local/opt/blender-portable/blender-5.2.0-linux-x64/blender \
  --background --python tools/audit_truth_mesh_clearance.py -- \
  --trajectory runtime/map_reference/truth_trajectory_YYYYMMDD_HHMMSS.csv \
  --output runtime/map_reference/mesh_clearance_audit.json
```

报告保存碰撞网格 SHA-256、探索阶段采样最小距离、逐类障碍最小距离、采样间隔和小于 1 m 的计数；`piecewise_linear_clearance_lower_bound_m` 只约束**采样点间直线插值**，不能证明未采样的真实连续轨迹。审计节点不发布任何控制话题。要宣称 1 m 硬安全已被全过程验证，必须同时审查完整架次、采样间隔、几何模型一致性和定位误差，不能仅凭 MID360 点云最小值。

2026-09-26 的组合试验在斜地面侧支处，单束仿真下视约报 1.32–1.57 m，但网格真值最近地面仅 1.07256 m；单束垂直量程不能代表球形包络的最近三维距离。该次虽无采样点低于 1 m，只有约 7 cm 净空余量且陷入顶板/地面脱险循环，必须判为**未通过完整安全验收**。该仿真下视传感器也不是已确认安装在真机上的硬件，真机安全论证不能依赖它。

后续 121 束下视扇形的 A/B 越过了旧侧支，但在 x≈112 m 的管线/设施旁最小网格距离又降至 1.08412 m 并停滞；可见格仅 44.87%。这再次说明“采样没有低于 1 m”不等于有足够裕量、安全控制已完善或全图已完成。

下一次相同配置的冷启动已**实际突破 1 m 半径**：管线附近碰撞网格样本最小 0.99570 m；规划轨迹近似真值投影还显示部分被接受的路径点落在 1 m 内。当前不能再称这套配置“在已测区段守住硬安全”。规划垂直碰撞盒半高 0.6 m 小于要求的球形半径，上方管线又位于 MID360S 当前安装视场的盲区；增大整个矩形盒的 A/B 在 x≈21 m 就过早折返，因此需要更合适的球形碰撞检查和可验证的上方感知。验收脚本的链路通过不覆盖这项网格失败。

当前参考是 0.5 m 的二维栅格，采用最高地面和最近岩壁/管线的几何距离。它不能证明连续三维通路可飞，也可能漏掉上下重叠巷道。参考中未与起点连通的栅格，须区分真实不可达、坡度过大与栅格离散误差，不能未经核查直接从“全图”分母剔除。

## 完成条件

1. MID360S 原始点云经 FAST-LIO2 和 Voxblox 进入 GBPlanner2；PX4 外部视觉 XY 与轨迹输出持续有效，禁止真值进入控制链路。
2. 全过程 1 m 三维安全半径不被突破；地面盲区由当前仿真下视测距守护。失去定位、测距或路径安全证据时停住而不是继续飞。
3. 所有经几何复核的可达分支和尽头都被观察，未见区域的每个连通块都有“真实不可达/传感器盲区/尚未探索”的可审计归因；未归因的可达区域为零。
4. 从冷启动飞到探索结束并安全返航，保存覆盖率时间序列、轨迹、速度、定位误差、最小净空、停滞及恢复次数，至少重复一次。只有完整运行证据通过才能声明达标。

`validate_full_chain.py` 的 `passed` 现在只代表链路持续工作及**已观测传感器回波**未低于 1 m，不是碰撞网格 1 m 硬安全或全图覆盖认证。2026-09-26 的管线复现中，该工具的传感器检查通过，独立网格却记录到 0.99570 m。网格安全必须在完整架次结束后单独审计；真机还必须解决 MID360S 及仿真额外下视硬件的上下盲区问题。

每次开启 `coverage_audit:=true` 后，覆盖报告和全链路报告都保存带时间戳的独立文件，避免 `latest_coverage.json` 被下一次试验覆盖。完成整架次并生成碰撞网格报告后，可运行失败即不通过的自动预检：

```bash
python3 tools/check_full_map_acceptance.py \
  --chain runtime/map_reference/full_chain_本架次.json \
  --coverage runtime/map_reference/coverage_本架次.json \
  --mesh runtime/map_reference/mesh_clearance_本架次.json \
  --truth runtime/map_reference/truth_trajectory_本架次.csv
```

预检要求同一 ROS 启动架次的完整链路观察、参考格全部可见、碰撞网格逐样本及采样点间直线下界守住 1 m、额外留出 0.15 m 样本裕量，以及最终返回起点附近并解除武装。0.15 m 只是初步验收缓冲，不代表已经覆盖定位或执行误差。`automatic_precheck_passed` 即使为真，也**不是**连续三维安全或所有可达空间探索完毕的证明；最终仍需审查 2-D 参考的漏项、真实传感器盲区和连续轨迹。当前试验显然不能通过该预检。

仿真中 `time_budget_limit=3600` 秒只是避免 900 秒提前返航；真实两电池的容量、压降、续航及安全返航阈值尚未通过实测验证，不能据此推断真机能单架次完成。

当前 `go_home_if_fully_explored=false` 是刻意保留的：已有多次在未到地图尽头时局部/全局 frontier 被清空的证据，若直接启用“无 frontier 即返航”，会把错误的空候选当作完成。`auto_homing_enable=true` 目前仅提供预算不足时或显式服务请求的返航路径，不提供可靠的“全图已探明”信号。安全缺口、frontier 漏选、原生完成判据和实际返航均尚未通过端到端验证；不能用 3600 s 超时返航代替全图完成。
