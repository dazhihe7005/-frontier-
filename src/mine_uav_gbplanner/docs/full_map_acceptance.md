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

二维参考会压平上下重叠巷道，也会漏掉需要改变高度才连通的空间。现已另建**不反馈控制链路**的三维参考：

```bash
/home/nuc/.local/opt/blender-portable/blender-5.2.0-linux-x64/blender \
  --background --python tools/build_baixiangshan_reference_3d.py -- \
  --resolution 0.5 \
  --output runtime/map_reference/baixianshan_reachable_3d_0p5m.json.gz
```

它对所有原始碰撞网格检查中心 1 m 净空并留 0.05 m 几何余量；26 邻接连边用曲面距离的 1-Lipschitz 界递归检查整段，不只检查端点。当前网格有 12,514 个有效三维中心、其中 8,460 个与起点连通；投影到 XY 是 3,367 格，比旧二维连通参考多 288 格、少 10 格。两者差异说明不能用二维参考单独宣布全图完成。三维参考依然是 0.5 m 离散模型，不证明更窄的真实通路不存在。

`coverage_audit:=true` 时，`diagnostic_map_coverage.py` 只读 Gazebo 真值位姿和原始 MID360 射线，每 15 个仿真秒写入 `runtime/map_reference/latest_coverage.json`。这些数据只用于验收，绝不反馈给 FAST-LIO2、Voxblox、GBPlanner、PX4 或安全制动。`visible_cells` 是射线可见性代理量；`visited_cells` 是实际飞经的网格。`largest_unseen_components` 用于定位未探索的连通巷道。

同一报告新增 `three_d` 字段，按三维可达中心记录可见体素、实际飞经体素、未见连通块及三维体积；报告还带碰撞网格哈希。启动脚本在每次开启审计时会从当前碰撞网格重建二维和三维参考，避免旧参考与当前场景版本不一致。自动预检现在同时要求二维与三维参考可见格全部完成，且三维参考哈希与飞行网格审计一致。2026-09-26 的默认失败即停短测在约 45 s 时仅见二维 137/3089、三维 328/8460；这是审计链路生效的证据，**不是探索成功**。

同一审计开关还把 GBPlanner 发出的原始轨迹，与近同时刻的规划里程计、Gazebo 真值位姿配对，写入 `planned_trajectory_*.csv`。`x/y/z` 是用于**离线诊断**的近似真值坐标投影；它受锚点时间差、定位和偏航误差影响，不能回流到规划或控制。要分析管线附近的规划点，可运行：

```bash
/home/nuc/.local/opt/blender-portable/blender-5.2.0-linux-x64/blender \
  --background --python tools/audit_planned_mesh_clearance.py -- \
  --planned runtime/map_reference/planned_trajectory_YYYYMMDD_HHMMSS.csv \
  --x-min 105 --x-max 125 --y-min 2 --y-max 9 \
  --output runtime/map_reference/planned_mesh_clearance.json
```

该报告仅能区分“候选计划在真实网格上可能贴近障碍”与“实际飞行另有误差”这类根因候选；不能把近似投影直接当作在线碰撞证书。判断 PX4 实际净空仍以独立真值轨迹网格核验为准。

同时还记录 `raw_planner_path_*.csv`：这是 GBPlanner 服务返回、PCI 重新连接当前位置及插值之前的原始路径。可用同一个网格审计脚本分别核对 `raw_planner_path_*.csv` 与 `planned_trajectory_*.csv`，区分“地图/规划已漏障碍”与“PCI 改写路径后穿墙”。两份文件的投影都只供离线诊断，绝不作为控制输入。

第一次球形碰撞短试验中，原始路径的 58 个投影点已有 36 个距真实碰撞网格不足 1 m，最小约 0.049 m（约 33.6 仿真秒，首点与当前里程计差 0.016 m，锚点时间差 0.088 s）；因此 PCI 重接/插值**不是唯一根因**。同一试验的机体仍靠实时防撞停在起点附近，87 s 仅前进约 3.3 m，可见格约 328/3089；真实轨迹最小网格距离 1.17159 m。这只是短时保护效果，不是完整安全或探索成功。

源码还发现：GBPlanner2 `getBestPath()` 对原始图路径再次检测时，遇到非空闲段只打印消息，**仍把路径继续发给 PCI**。已将 `planner_strict_final_path_check:=true` 设为默认，在图路径提取和最终插值后遇非空闲段即拒绝。它只能保证当前体素图的检查结果被执行，不能证明地图与实体完全一致。

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

二维参考仍采用 0.5 m 平面栅格、最高地面和最近岩壁/管线；三维参考补充不同高度的可达中心，但仍不能证明连续三维通路完整。参考中未与起点连通的格，须区分真实不可达、传感器盲区与栅格离散误差，不能未经核查直接从“全图”分母剔除。

## 完成条件

1. MID360S 原始点云经 FAST-LIO2 和 Voxblox 进入 GBPlanner2；PX4 外部视觉 XY 与轨迹输出持续有效，禁止真值进入控制链路。
2. 全过程 1 m 三维安全半径不被突破；地面盲区由当前仿真下视测距守护。失去定位、测距或路径安全证据时停住而不是继续飞。
3. 二维及三维几何参考中所有可达分支和尽头都被观察，未见区域的每个连通块都有“真实不可达/传感器盲区/尚未探索”的可审计归因；未归因的可达区域为零。
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

已将 `planner_collision_sphere_radius:=1.0` 设为默认失败即停模式：隔离 GBPlanner2 地图查询将 1 m 球体和路径胶囊内相交的占据/未知体素视为不可通行，替代原先垂直半高仅 0.6 m 的碰撞盒。为避免漏掉与球体相交的 0.2 m 体素，此试验使用半体素对角线的保守扩展；因此可能在狭窄巷道过度保守。它不修复 MID360S 的上方视场盲区，也不能把未知体素直接清成安全空间。默认失败即停并不等于可行性或完整安全验收通过。

2026-09-26 后续诊断定位到球形模式自身的一个严重实现错误：`getCenterPointFromGridIndex` 需要体素边长，却收到边长倒数。0.2 m 分辨率下计算的体素中心被放大 25 倍，原模式因而可以把近旁已知岩壁误判为不相交。错误路径的规划坐标 `(0.815,-2.895,1.014)` 附近，同架次 TSDF 在 0.121 m 处已有距离值 0.127 m 的体素；Gazebo 原始激光在该岩壁块的理论命中 23/23 都有近似匹配回波。因此“只因激光看不见这面岩壁”的解释被排除。已将体素中心计算改为传入实际边长，原错误配置不得用于安全论证。

修正后，按旧 `<=0.2 m` TSDF 表面阈值和半体素对角线叠加，起点 1 m 球外约 1.16 m 的岩壁仍会被当成碰撞，规划全部为空。实验阈值改成 `<=0.05 m`（仅球形模式）后，根节点到前方候选边仍被判 `kUnknown` 或 `kOccupied`，图构建每轮约 1200 次连边均未成功。只读 TSDF 快照中，起点到前方约 2 m 的胶囊内有顶板附近 `(0.3,-0.1,2.1)` 以及侧壁附近 `(1.3,0.7,0.9)` 等未观测体素。以前者为例，起点雷达中心约 `(0.07,-0.08,1.26)`，目标相对仰角约 75°，超过当前安装姿态约 +27° 的最大上视角；这块近距顶板不可能由该姿态的 MID360S 在起点直接观测。侧壁空洞则还需进一步区分遮挡与地图融合时序。这是“已知障碍被误判为空闲”修好后暴露出的近距离地图空洞。当前失败关闭比带病继续飞安全，但**还没有达到完整地图探索**。下一步需验证这些空洞的原始射线覆盖与地图融合时序，并明确真实传感器能否覆盖顶板/地面盲区；不能为追求覆盖率直接把未知空间当空闲。
