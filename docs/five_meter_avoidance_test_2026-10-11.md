# 临时 5 米 SUPER 避障实飞前验证

## 目的与边界

本功能只用于正式任务一首次 Offboard 实飞前的短距离检查。它不修改、
删除或复用 `super_exploration_decider` 内部探索逻辑，也不改
`task1_real.launch`。专用 launch 中不启动任务调度器和任务一决策器，改由
`five_meter_avoidance_test` 发布一个目标，其余链路保持为：

```text
MID360 → Fast-LIO2 → task1_perception_gate → SUPER
       → super_px4_command_bridge → offboard_bridge → MAVROS → PX4
```

临时节点不会解锁飞控。实机仍由飞手手动解锁、起飞和悬停。

## 实机操作

1. 确认 CH7 在低位，CH5 能正常切回人工模式。
2. 使用 `five_meter_avoidance_real.launch`，不要同时启动
   `task1_real.launch`、`mission_scheduler` 或 `super_exploration_decider`。
3. 人工起飞并在障碍前稳定悬停，机头对准期望的 5 m 方向。
4. 将 CH7 从低位拨到高位。节点先保持任务权限关闭并进入 `WAIT_HOVER`：
   MAVROS/PX4 必须已连接和解锁，Fast-LIO2 位姿差分总速度不超过
   0.20 m/s、垂直速度不超过 0.10 m/s，连续稳定 1.0 s。通过后才捕获
   位姿和航向、开放任务点云门；再等待至少 5 帧新点云且暖图 1 s，发布
   同高度、正前方 5 m 的唯一目标。15 s 内不能稳定悬停则中止本次尝试。
5. 到目标 0.35 m 内且位姿差分速度低于 0.20 m/s 持续 1 s 后，节点取消
   SUPER 目标并让现有桥接器退出 Offboard 到 `POSCTL`。
6. 任务过程中把 CH7 拨回低位会立即取消；CH5 仍是独立人工接管。遥控、
   里程计、点云失联，SUPER 拒绝目标或 25 s 超时也会取消并退出。

建议先做无障碍 5 m，再在前方约 2.5 m 放置宽、轻质、雷达可识别的障碍，
两侧各留至少 3 m 空间。首次实飞仍应从宽阔区域和低速开始。

状态查看：

```bash
rostopic echo /mine_uav/five_meter_test/status
rostopic echo /mine_uav/task1/command_status
```

## 仿真场景与运行

`worlds/five_meter_avoidance.world` 是 14 m × 10 m × 4.5 m 的封闭房间；
无人机从 `(0, 0)` 起飞，目标约为 `(5, 0)`，不可飞越的中心障碍位于
`(2.5, 0)`，尺寸为 `0.6 m × 1.4 m × 4.5 m`，强制 SUPER 左右绕行。

先编译：

```bash
cd /home/nuc/frontier_runtime_ws
source /home/nuc/super_ws/devel/setup.bash
source /home/nuc/fastlio2_ws/devel/setup.bash --extend
catkin_make -DCMAKE_BUILD_TYPE=Release -j4
```

再运行隔离 SITL：

```bash
/home/nuc/frontier-upload/scripts/start_five_meter_avoidance_sitl.sh \
  gui:=false rviz:=false
```

脚本固定使用 ROS master `11319`、Gazebo master `11349` 和本地 PX4 SITL，
不会接触真实飞控。

## 2026-10-11 验证结果

- 新增逻辑单元测试原始 3/3 通过；增加稳定悬停门后专项测试为 4/4 通过。
- 两次完整 PX4/Gazebo/Fast-LIO2/SUPER 回归均在约 33.5 s 仿真时间发布
  `COMPLETE:GOAL_REACHED_AND_SETTLED`。
- 目标约为 `(4.98, 0.02, 1.40)`；SUPER 接受目标并进入 `FOLLOW_TRAJ`。
- 第一次运行中途观测 Fast-LIO2 位姿 `(3.67, -1.43, 1.34)`，同期 Gazebo
  真值约 `(4.02, -0.98, 1.51)`；中心障碍在 `y=0`，可见明显侧向绕行。
- 两次 SUPER 累计轨迹长度分别约 7.25 m、7.29 m，显著长于 5 m 直线，
  与绕障路径一致。
- 完成后任务点云门关闭、目标取消、桥接器退出 Offboard；SITL 配置使用
  `AUTO.LAND` 并最终落地，实机专用 launch 使用 `POSCTL`。

仿真中个别在线重规划出现高度候选拒绝和一次 CIRI 走廊不可行，但 SUPER
保留/恢复了有效轨迹，两次均完成。这说明当前简易场景可验证完整链路，但不
等于已经完成真机安全验收。

增加稳定悬停门后的完整回归中，CH7 上升沿在仿真时间 16.894 s 被接受，
节点到 19.094 s 才确认稳定并开放任务权限，20.094 s 发布目标，38.994 s
完成。接管时 PX4 高度设定值的单次变化由改动前约 8.6 cm 降至约 1.67 cm；
SUPER 首次轨迹交接偏移约 2.3 cm，并由通用桥在 3.5 s 内平滑消除。该结果
证明门控在隔离 SITL 生效，但真机仍需卸桨/系留检查实际 Fast-LIO 噪声和
PX4 视觉高度融合状态。

考虑首次 PID 调参和 Fast-LIO 差分噪声后，专用配置最终放宽为总速度
0.20 m/s、垂直速度 0.10 m/s、稳定 1.0 s、最长等待 15 s。复测中触发于
16.614 s，17.664 s 即通过门控，18.664 s 发布目标，33.965 s 正常完成；
接管阶段实际最大垂直速度约 0.044 m/s，高度设定值单次变化约 3.22 cm。
该折中不要求非常理想的 PID，同时仍拒绝持续明显升降时自动前飞。

## 开机自启动与飞行记录

`mine-uav-five-meter.service`可在NUC开机后启动本页所述真机软件链。它会先从
`/dev/serial/by-id/`自动发现PX4串口，并等待`enp89s0`的`192.168.1.10/24`地址，
再启动ROS；等待期间不会创建任何飞行控制节点。设备名中含PX4、Auterion、Pixhawk、
Holybro或Cube时优先选择；否则只有唯一一个USB串口候选时才使用。存在多个无法区分的
候选设备时会保持等待，不会任意选择。服务只在开机时启动一次，运行故障后不会自行重启。

开机启动不等于自动飞行：测试节点不会解锁或起飞；启动时收到的第一档CH7只建立
基线，即使CH7原本在高位也不会触发。飞手必须先让CH7稳定到低位，人工解锁、起飞、
稳定悬停，再产生新的低到高变化，并通过本页所列连接、解锁、里程计和悬停门控。

每次启动会建立`/home/nuc/flight_logs/five_meter/YYYYMMDD_HHMMSS/`，使用LZ4和
5分钟分卷录制全部ROS话题，同时保存启动元数据、配置、ROS参数、节点和话题清单。
磁盘可用空间不足100 GiB时拒绝启动；不会自动删除旧飞行证据。突然断电时，最后一个
分卷可能保留为`.active`，此前已经封口的分卷仍可直接分析。PX4内部状态仍以飞控SD卡
ULog为准，ROS bag不能替代ULog。

安装并启用下一次开机启动（不会立即启动实机链路）：

```bash
/home/nuc/frontier-upload/scripts/install_five_meter_autostart.sh
```

常用检查和人工控制命令：

```bash
systemctl status mine-uav-five-meter.service
journalctl -u mine-uav-five-meter.service -b
sudo systemctl stop mine-uav-five-meter.service
sudo systemctl start mine-uav-five-meter.service
sudo systemctl disable mine-uav-five-meter.service
```
