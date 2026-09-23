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

- 规划参考点采用固定 1 m 三轴安全包络，不设置硬性 `+X` 方向过滤。
- 当前运动方向只参与软评分，权重为 0.30；它不会过滤反向候选，因此仍允许
  回退和探索可达分支，但信息增益近似相同时不会无故 180° 掉头。
- 已修复 GBPlanner2 上游方向参考轨迹把所有采样点错误放在同一终点的问题；
  补丁保存在 `patches/gbplanner-direction-reference.patch`。
- 起飞平台使冷启动接触稳定，扫描与位姿又已严格同步，因此不再在悬停后
  清空约 17 秒的成熟 Voxblox 地图；触发后只保留 2 秒稳定窗口，避免从
  稀疏地图开始产生大量短航段。
- 起飞交接要求位置、姿态和速度连续稳定 2 秒。外部视觉速度门槛为
  3.5 m/s，与 PX4 最高 3.0 m/s 起飞包络匹配；正常规划仍限 0.7 m/s。
- 不对首次或后续候选施加最短路径硬过滤；短路径也会被执行并触发下一轮
  决策，避免执行器拒绝后与 PCI 相互等待。1 m 是碰撞安全包络，不是
  最小航段长度或主方向硬约束。
- 轨迹执行器拒绝错误坐标系、非有限值、时间倒序、速度/偏航角速度
  超限、首点跳变以及飞行范围越界的轨迹。
- FAST-LIO2 输入跳变或速度异常时，外部视觉输出立即停止。
- 外部视觉严格按 FAST-LIO2 的 10 Hz 新帧逐帧发送，不再把同一帧以 30 Hz
  重复灌入 PX4；处理延迟由隔离 1018 空机架的 `EKF2_EV_DELAY` 明确补偿。
- 单次低信息增益或一次全局 frontier 未命中不会自动判定“探索完成”并执行
  一条几十米的整程返航轨迹；规划器继续重试未知空间。显式返航以及时间/
  电量预算触发的返航能力仍保留。
- 局部树和全局 frontier 都持续返回空结果时，恢复节点会在 15 次空决策后
  暂停 PCI，沿最近一段已执行、已碰撞检查的轨迹以 0.45 m/s 回退，再从较早
  的根节点恢复未知探索；不使用 Gazebo 真值，也不缩小 1 m 防撞包络。

## 依赖复现

第三方源码及固定提交记录在 `dependencies.repos`。在空工作区根目录可运行：

```bash
vcs import < dependencies.repos
git -C src/exploration/gbplanner_ros apply ../../../patches/gbplanner-direction-reference.patch
git -C src/misc/eigen_checks apply ../../../patches/eigen-checks-disable-tests.patch
```

随后按当前工作区配置使用 `catkin build`。仓库不包含第三方 Git 工作树、
编译产物、运行日志或 PX4 持久状态。

验证记录见 [VALIDATION_2026-09-24.md](VALIDATION_2026-09-24.md)。
