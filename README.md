# GBPlanner2 隔离仿真工作区

本工作区用于在白象山实景巷道中验证 GBPlanner2、FAST-LIO2 和 PX4
SITL 的完整闭环。它不 source 或修改现有的 frontier/SUPER 工作区。

## 隔离边界

- ROS master：`http://127.0.0.1:11331`
- PX4 instance：`2`，MAV_SYS_ID：`3`
- PX4 simulator TCP：`4562`
- Gazebo MAVLink UDP：`14562`
- MAVROS：本地 `14542`，PX4 远端 `14582`
- PX4 参数和日志：每次启动使用 `runtime/px4_runs/<时间戳>_<PID>`，避免
  上一次异常退出写入的校准状态污染下一次冷启动
- ROS 日志：`runtime/logs/ros`
- 车辆模型副本：`isolated_assets/models`
- PX4 1018 空机架副本：`isolated_assets/px4_airframes`；启动时只链接到本工作区
  的私有 `runtime/px4_root`，不再引用 frontier/SUPER 中的空机架文件
- 白象山 1:1 实测模型和网格从 `/home/nuc/frontier-upload` 只读引用；隔离
  world 仅叠加起落架足迹大小的平整起飞台，不缩放或改写巷道
- FAST-LIO2 从 `/home/nuc/fastlio2_ws/devel` 作为只读 underlay 使用

## 运行

```bash
cd /home/nuc/gbplanner2_isolated_ws
./run_full_chain.bash gui:=true rviz:=true
```

无界面运行：

```bash
./run_full_chain.bash gui:=false rviz:=false
```

路口专项回归（地图仍为原始 1:1 尺度，只旋转场景让实地图中的交叉口位于
标准起点前方）：

```bash
./run_full_chain.bash \
  world:=/home/nuc/gbplanner2_isolated_ws/isolated_assets/worlds/baixianshan_crossroad_validation.world \
  gui:=true rviz:=true
```

全链路运行后，可在另一终端启动自动验收：

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/fastlio2_ws/devel/setup.bash
source /home/nuc/gbplanner2_isolated_ws/devel/setup.bash --extend
export ROS_MASTER_URI=http://127.0.0.1:11331
rosrun mine_uav_gbplanner validate_full_chain.py _duration:=180
```

## 数据链路

```text
Gazebo MID360S PointCloud
  -> 距离裁剪 + 机体自反射过滤
  -> PointCloud2
  -> 真实 FAST-LIO2 + PX4 IMU
  -> /Odometry
     -> PX4 external vision XY
     -> GBPlanner2 planner odometry
     -> FAST-LIO 位姿驱动的 Voxblox 射线原点
  -> GBPlanner2 MultiDOF trajectory
  -> 安全校验、插值和坐标对齐
  -> /mavros/setpoint_raw/local (50 Hz)
  -> PX4 内部位置/姿态/角速度闭环
```

控制链路不订阅 `/gazebo/model_states` 或 `/gazebo/link_states`。Gazebo
真值只允许在独立诊断命令中用于离线误差检查。

## 安全行为

- 实时防撞使用固定 1 m 三维球形安全半径，不设置硬性 `+X` 方向过滤。规划层水平
  半包络为 1.1 m（含 0.1 m 体素余量）。执行器不裁掉 GBPlanner 轨迹末端，
  避免 PCI 因等不到名义终点而互锁；独立 MID360S 实时保护层负责处理弯后
  才显露的凹凸墙面，持续受阻时从实际位置触发重规划/安全回退。
- 当前运动方向只参与软评分，权重为 0.30；它不会过滤反向候选，因此仍允许
  回退和探索可达分支，但信息增益近似相同时不会无故 180° 掉头。
- 对 1.5 m 以下的短路径降低信息增益评分，并在候选路径短于 2 m 时，对
  最近 12 次选中过的观察位置
  施加 0.75 m 范围内的软性重复惩罚；这不放行未知/占用体素，也不限定世界坐标
  主方向。起点附近的 2.2 m 宽规划碰撞盒常因侧向少量未知体素只能生成
  约 0.4–1.0 m 的短路径，故冷启动离开起点的时间仍有随机性。
- 总升降量软惩罚保留为可选第三方补丁
  `patches/gbplanner-vertical-travel-penalty.patch`，当前权重为 0（禁用）：
  2.5 权重的交叉口试验出现小于 1 m 的三维净空，尚不能作为默认策略。
- 已修复 GBPlanner2 上游方向参考轨迹把所有采样点错误放在同一终点的问题；
  补丁保存在 `patches/gbplanner-direction-reference.patch`。
- 起飞平台使冷启动接触稳定，扫描与位姿又已严格同步，因此不再在悬停后
  清空约 17 秒的成熟 Voxblox 地图；触发后只保留 2 秒稳定窗口，避免从
  稀疏地图开始产生大量短航段。
- 起飞交接要求位置、姿态和速度连续稳定 2 秒。外部视觉速度门槛为
  3.5 m/s，与 PX4 最高 3.0 m/s 起飞包络匹配。当前开阔段规划上限为
  0.70 m/s；狭窄环境上限仍为 0.35 m/s，实时安全层还会按净空和跟踪
  误差降速。2.0 m/s 试验因换段误差导致碰撞，已禁用。
- 不对首次或后续候选施加最短路径硬过滤；短路径也会被执行并触发下一轮
  决策，避免执行器拒绝后与 PCI 相互等待。1 m 是碰撞安全包络，不是
  最小航段长度或主方向硬约束。
- 轨迹执行器拒绝错误坐标系、非有限值、时间倒序、速度/偏航角速度
  超限、首点跳变以及飞行范围越界的轨迹。
- 执行器同时发送位置和速度前馈，持续比较 PX4 实际位置与当前轨迹点；
  跟踪误差超过 0.20 m 后按比例降低轨迹时钟，达到 0.50 m 时暂停推进。
  速度前馈的水平/垂直变化率还分别限制为 0.50/0.40 m/s²，
  避免 6 kg 机体在前进、回退和换段时收到瞬时反向速度指令；偏航指令
  也经过连续限速。FAST-LIO2 异常时重建零速度悬停指令，不会复用旧轨迹的
  速度前馈。
- MID360 增加独立预测防撞：依据当前运动方向计算进入 1 m 包络前的剩余
  距离，普通航段从 1.60 m 制动余量开始降速、到 0.75 m 余量时停止；接近
  frontier 末端时使用 1.85/0.95 m 的更保守阈值。三维绝对净空从 1.55 m
  开始降速，到 1.25 m 停止；顶板/地面净空低于 1.30 m 时另执行 0.35 m
  垂直脱离。只有垂直距离不小于水平距离的回波才作为顶板/地面候选；更大的
  全方向顶板保护区曾在 1.34 m 正常顶板净空处反复停车，
  因而只扩大前进方向的制动区。
  平行侧墙不会误触发，点云超时则立即停止。新航段首点相对 PX4 实际位置
  超过 0.60 m 会被拒绝，FAST-LIO2
  异常时持续发送最后安全悬停点，不再直接中断 OFFBOARD 设定点流。
  水平薄层投影同时保留为保守诊断，但不把存在垂直高差、实际三维距离仍
  大于 1 m 的点误判成球形半径侵入。
- MID360S 的安装视场无法覆盖机体正下方的近地盲锥。隔离仿真中已将模型
  自带的向下射线移到起落架下方，并换算为机体中心到地面/下方障碍的距离；
  它参与近地制动和验收；即使无人机水平飞行，地面突起逼近机体中心
  1.35 m 时也触发向上脱离，并覆盖此前可能锁存的顶板下降目标。
  它不参与 FAST-LIO2/PX4 水平定位。该传感器属于
  **仿真补充硬件**；若真机没有对应向下测距，不能把这项仿真安全结果直接
  外推到真机。曾尝试把单个地面回波直接送入 Voxblox，但使起点连续产生
  单点无效轨迹，故已撤销该入图实验。
- 实时防撞连续 2 秒无法推进轨迹时，显式通知恢复节点从 FAST-LIO2
  实际位姿重新规划；已执行轨迹足够长时才回退，否则原地重规划，不再出现
  “安全层已停车、PCI 却一直等待到达末点”的死锁。
- FAST-LIO2 输入跳变或速度异常时，外部视觉输出立即停止。
- MID360S 当前仍是 Gazebo 10 Hz 整帧 ray 近似：PointCloud2 只有 XYZI，
  FAST-LIO2 仿真入口将逐点相对时间置零，因此没有真实逐点扫描运动畸变
  与去畸变验证。急转弯下的定位和净空结果可能偏乐观；本轮先不改动这条
  高影响链路，不能据此宣称真机快转定位误差一定很小。
- 外部视觉严格按 FAST-LIO2 的 10 Hz 新帧逐帧发送，不再把同一帧以 30 Hz
  重复灌入 PX4；处理延迟由隔离 1018 空机架的 `EKF2_EV_DELAY` 明确补偿。
- 单次低信息增益或一次全局 frontier 未命中不会自动判定“探索完成”并执行
  一条几十米的整程返航轨迹；规划器继续重试未知空间。显式返航以及时间/
  电量预算触发的返航能力仍保留。
- 局部树和全局 frontier 都持续返回空结果时，恢复节点会在 5 次空决策后
  暂停 PCI，沿最近一段已执行、已碰撞检查的轨迹以 0.35 m/s 回退，再从较早
  的根节点恢复未知探索；恢复必须由 FAST-LIO2 里程计连续确认到达，不能再
  按预估时长假定成功；若实时防撞连续阻断新回退路线 1 秒，立即放弃
  该路线，从当前实际位姿重新规划，不再原地等几十秒；若垂直脱险等安全
  暂停使回退最终超时，也会从当前位姿重规划。回退同样服从实时方向净空
  和向下测距制动，不能因路径曾经走过就越过新的障碍；不使用 Gazebo 真值。

## 依赖复现

第三方源码及固定提交记录在 `dependencies.repos`。在空工作区根目录可运行：

```bash
vcs import < dependencies.repos
git -C src/exploration/gbplanner_ros apply ../../../patches/gbplanner-direction-reference.patch
git -C src/exploration/gbplanner_ros apply ../../../patches/gbplanner-short-path-gain.patch
git -C src/exploration/gbplanner_ros apply ../../../patches/gbplanner-vertical-travel-penalty.patch
git -C src/misc/eigen_checks apply ../../../patches/eigen-checks-disable-tests.patch
```

随后按当前工作区配置使用 `catkin build`。仓库不包含第三方 Git 工作树、
编译产物、运行日志或 PX4 持久状态。

最新提速验证见 [VALIDATION_2026-09-26.md](VALIDATION_2026-09-26.md)；原始
0.5 m/s 验收见 [VALIDATION_2026-09-25.md](VALIDATION_2026-09-25.md)，早期
问题定位过程保留在 [VALIDATION_2026-09-24.md](VALIDATION_2026-09-24.md)。
同日后续复核发现 0.85 m/s 试验净空仅 1.0369 m，0.80 m/s 试验均速
反降，均已回退；最终 0.70 m/s 配置在交叉口测试的最小净空为
1.0095 m，不能视为拥有足够安全裕量或已完成真机快转验收。
