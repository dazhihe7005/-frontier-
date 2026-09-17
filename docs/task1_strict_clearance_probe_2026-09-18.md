# 任务一严格水平净距探针：2026-09-18

状态：隔离配置已在迷宫与空旷 40 m 两种 SITL 场景完成探索、返航与几何净距验收；**不等于真实雷达/真机验收**。下述前期失败探针仍保留作因果链证据。任务二可在此仿真里程碑后启动。

## 验收合同

- 场景：`goaf_serpentine_maze.world`，40×40×30 m 采空区、三道左右交替开口的全高隔墙，入口平台起飞。
- 水平墙边：按 0.4 m 机体等效半径，SUPER 位置指令与 PX4 实际位姿到三道隔墙碰撞盒的机体外余量都应 ≥1 m；要深入局部 x≥45 m、确认模型完成并回 home。
- 额外检查：规划速度/高度、失目标时悬停、没有出航时朝入口后方误飞。碰撞盒距离仍不能代替桨叶包络/真实雷达误差验收。
- 试验使用 `config/super_task1_strict_clearance_probe.yaml`；默认真机配置 `super_task1.yaml` 不变。

## 已证实的因果链

1. 旧成功闭环 bag 的第三墙机体外最小余量只有 0.136 m。原 SUPER 连续走廊只用 0.4 m 物理半径，A* 与连续轨迹安全边界不一致。
2. 给 CIRI 障碍平面增加额外水平 1.2 m 后，首轮入口失败种子线 z≈0.85 m 被 z≈-0.05 m 的地面/平台障碍点产生的斜平面切掉。增加 ±0.30 m 局部走廊高度带，仅对能与物理机体垂直相交的点作水平加宽；对应探针前进到 x=15.45 m，但未过第一墙。
3. 一轮目标在 18.024 s 由决策器发布，18.029 s 被 SUPER 因地图未就绪拒绝；此前决策器仍等 30 s 并标记不可达。现加按目标 ID 的拒绝回执和短退避重选。
4. SUPER `SearchPolytopeOnPath` 原先可在整条前端路径全被膨胀图判占据时越界读取 `path[path.size()]`，日志出现约 `6.95e-310` 的垃圾种子坐标。现显式拒绝全占据路径，3 项边界测试通过。
5. ROG 原 `inflation_step=7` 是三维球形，按 0.2 m 分辨率也将入口地面向上膨胀约 1.4 m。A* 对第一目标反复 `TIME_OUT`，决策器若也用 1.8 m 三维球半径则 `reachable=0`。新增仅在严格探针启用的水平/垂直分离：ROG 水平膨胀 7 格、垂直 2 格；决策器水平 1.8 m、垂直 0.4 m。默认未开启时保持旧球形逻辑。两组几何单测分别 2/2 通过。
6. 分离后一次探针飞到 x≈13.4 m，却因前墙回退选择器明确偏爱侧/后候选，把 `(3.25,-10.25)` 及 `(-12.75,-12.75)` 作为出航目标，实际 x≈-14.55 m。已立即停止。现候选合同禁止超过 2 m 的出航倒退；前墙确认后先产生最多 4 m、逐栅格核对已知自由与机体净距的侧向过渡目标。对应测试 2/2 通过。
7. 下一轮侧向目标依次约 `(14.9,2.7)`、`(15.0,5.7)`、`(15.0,9.1)`，实际横移到 y≈9 m，未再出入口。但从开口前进时停住：68.173 s 的 A* 前端路径离第一墙碰撞盒最低中心距 1.6155 m；CIRI 加宽后要求约 1.6 m，几乎无裕度，附近实际障碍点到某段种子线的水平距离约 1.589 m，CIRI 拒绝是合理的。该轮联合验收因最大局部 x≈15.46 m、未完成返回而失败。随后把严格探针水平膨胀从 7 格增至 9 格，保留垂直 2 格，作为下一轮可复算的因果对照。

## 9 格水平膨胀后的两场景验收

使用 `task1_maze_sitl.launch`（默认现已指向隔离严格配置）、`task1_40m_strict_sitl.launch`。两场景均为 Gazebo 合成点云与 PX4 SITL，无真实 MID360/Fast-LIO2 误差、遥控人工接管或实机融合验证。

- 迷宫 bag：`/home/nuc/task1_logs/success_cases/strict_clearance_2026-09-18/task1_lateral_detour_inflation9_20260918.bag`。任务最大局部 x=45.729 m，已通过三道隔墙，在真实尽头后自主返航，约 404.82 s `COMPLETE: home reached`。最后记录局部位置约 `(0.054,-0.096,1.386)` m。按照 0.4 m 机体等效半径，三道隔墙、左右侧墙和尽头墙共六面碰撞盒的机体外最小余量均 ≥1 m；其中最小规划/实际余量均出现在尽头墙，约 1.650/1.671 m；三道隔墙的实际最小值约 2.033/1.746/1.760 m。PX4 实际高度最高 1.560 m，任务指令区间最低 0.733 m；采样位置指令最高 1.499 m，邻接高度步最大约 0.037 m。前墙等待期间约 6.03 s 无新位置指令，PX4 位移 x≈0.044 m、高度下降≈0.046 m，属允许的等待，但不是持续发布新路径。未录制 `/planning_cmd/poly_traj`，故本 bag 不能证明两采样间多项式的全局高度极值。
- 空旷 40 m bag：`/home/nuc/task1_logs/success_cases/strict_clearance_2026-09-18/task1_open40_strict_regression_20260918.bag`。任务最大局部 x=42.605 m；雷达已观察到尽头墙，地图闭合判据满足后自主返航，约 193 s `COMPLETE`，最后局部位置约 `(0.085,-0.033,1.486)` m。空旷场景不要求一定飞到 x=45 m；距尽头墙最近时机体外实际余量约 4.795 m。左右墙及尽头墙全部 ≥1 m；采样位置指令 z 约 0.917～1.550 m，PX4 实际最高约 1.647 m。尽头墙确认等待约 7.03 s，无新位置指令，实际 x≈0.028 m、高度上升≈0.031 m。途中最大横向偏移约 3.55 m，目标大体沿入口轴线，但 SUPER 的局部路径仍有侧向弯曲；该场景空间足够且未违反墙距，不代表轨迹形状已针对效率最优。

复算指令（先 `source /opt/ros/noetic/setup.bash`）：`python3 scripts/analyze_maze_clearance.py BAG worlds/goaf_serpentine_maze.world --vehicle-radius 0.4 --min-planned-margin 1 --min-actual-margin 1 --min-local-x 45 --require-complete --include-vertical-walls`；空旷场景换 `worlds/goaf_40x40x30_platform.world` 并把最低深入改为 40 m。`python3 scripts/analyze_task1_vertical_trajectory.py BAG --alignment-z 0 --min-height -2 --max-height 1.8`。净距解析器单测 6/6 通过。平台/地板/顶板的竖向距离未由此二维墙距脚本直接证明，真实环境更不能照搬已知 SDF 几何。

## 证据与复现边界

两份成功 bag 已移入本机 `/home/nuc/task1_logs/success_cases/strict_clearance_2026-09-18/`；代表性的 7 格侧向探针失败 bag 已移入 `/home/nuc/task1_logs/failure_cases/maze_2026-09-18/`。其余早期失败 bag 仍在 `/tmp`，可能被系统清理。bag 未上传 GitHub。净距用 `scripts/analyze_maze_clearance.py`；前端路径用 `scripts/analyze_maze_frontend_path.py`。一轮某种子线的日志及 1.6155 m 几何结果需按相同仿真时刻一起看，不能仅凭规划报错推断 PX4 跟踪问题。

SUPER/ROG 在主仓库 `patches/super_runtime_strict_clearance_snapshot_20260918.patch`、`patches/rog_map_anisotropic_inflation_snapshot_20260918.patch` 留存运行工作树快照；与旧补丁有重叠，不能直接叠加。主仓库直接保存决策器、launch、探针 YAML 与单测。真实机体尺寸、MID360 点云误差、Fast-LIO2/PX4 坐标对齐、SITL 高度范围和真实飞控接管都仍需独立验收。
