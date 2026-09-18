# mine_uav_control 地面站

这是一个基于 ROS Noetic、MAVROS 和 PyQt5 的 PX4 NUC 轨迹发送器。它只负责生成并
发送轨迹 setpoint；QGC 负责 PX4 的连接、模式、解锁、降落和安全监控。串口上运行
的是 MAVLink，不是向 CH340 直接发送 ASCII 字符。

## 启动真实 PX4 链路

当前真机链路使用 CH340 转接 PX4 TELEM2。先确认稳定设备名存在：

```bash
ls -l /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

默认使用 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0:500000`，与当前
PX4 的 `MAV_1_CONFIG=102`、`MAV_1_MODE=2` 和 `SER_TEL2_BAUD=500000` 匹配：

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
roslaunch mine_uav_control real_uav_ground_station.launch
```

指定端口和波特率：

```bash
roslaunch mine_uav_control real_uav_ground_station.launch \
  device:=/dev/ttyUSB0 baud:=500000
```

## 与 QGC 的连接方式

MAVROS 和 QGC 不能同时独占同一个串口设备。TELEM2 链路下应让 MAVROS 独占
CH340 串口，QGC 通过 NUC 转发的 UDP MAVLink 链路连接，或暂时关闭 MAVROS
后再让 QGC 直接打开 USB。

启动前确认 CH340 串口没有被其他串口程序占用。

### USB数传直接连接QGC

当前实测USB数传电脑端是Silicon Labs CP2102，稳定设备名为：

```text
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

该数传电脑端串口波特率是`57600`。使用pymavlink在`57600`下已收到PX4心跳
（system id 1、component id 1）；QGC重新以该串口状态启动后识别PX4 v5.1.4并加载参数。
无线数传指示灯常亮只说明两端无线链路建立，不能证明电脑串口波特率正确。

对于把CP2102识别为普通串口、没有自动选对波特率的电脑，应在QGC的
`Application Settings -> Comm Links`中添加Serial链路，选择上述设备或对应COM口，
设置`57600`、无流控并连接。不要选择项目TELEM2直连所使用的`500000`；那是另一条
CH340/MAVROS链路的波特率。QGC、MAVROS或串口终端同一时间只能有一个进程占用该串口。

## 任务一真实闭环

### 代码职责边界

任务一现在将原始定位链路和任务规划感知链路分开。`task1_perception_gate` 只在任务一
由CH7开启且视觉定位健康时，把 `/cloud_registered`、`/Odometry` 转发到
`/mine_uav/task1/cloud_registered`、`/mine_uav/task1/odometry`；原始 `/Odometry` 仍持续
供 `fastlio_px4_vision_bridge` 和调度器使用。这样任务开始前的起飞平台/机体回波不会
污染SUPER的ROG-Map，但不会中断PX4定位。

SITL 的原先单体适配器已经拆为：`sitl_localization_adapter.py`（只做定位适配）、
`sitl_task_operator.py`（只模拟飞手起飞/CH7）和 `gazebo_mid360_fastlio_adapter.py`
（只做仿真雷达点云）。详细职责和数据流见
[`docs/task1_architecture.md`](docs/task1_architecture.md)。

任务一的SUPER参数现在由项目自己的 `config/super_task1.yaml` 管理，真机和SITL不再依赖
直接修改SUPER上游的 `click_smooth_ros1.yaml`。

### SITL局部轨迹优化说明

当前运行工作区已恢复官方SUPER `2ad3419` 核心逻辑，只保留项目所需的 `camera_init` 消息帧
接口适配；任务一参数由项目自己的 YAML 注入。历史补丁
`patches/super_preserve_reachable_goal.patch` 与
`patches/super_remove_duplicate_backup_optimize.patch` 会改变官方目标修正和备份轨迹优化语义，
默认不应重新应用。公平对照还发现SUPER在轨迹
结束但实际位置距目标超过硬编码的0.1 m时，会对同一目标再次从静止规划，这更符合“接近目标后
长时间反复规划”的现象。详细提交对照、A/B证据和后续隔离顺序见
[`docs/super_version_audit.md`](docs/super_version_audit.md)。

继续追踪官方核心后，已通过最小 GTest 证实 ROGMap 最近邻查询存在输入/输出别名缺陷：官方
多个调用点把同一坐标同时作为输入与输出，而查询函数会先把输出写成 `NaN`，从而污染输入并
绕过 `max_dis`。完整源码证据、复现结果、影响边界和验证计划见
[`docs/rog_map_nearest_cell_alias_defect_report.md`](docs/rog_map_nearest_cell_alias_defect_report.md)。
该缺陷目前是“已复现、尚未修复”，不能据此宣称全部轨迹问题已经解决。

继续追踪后已经用同场景运行时诊断确认另一项官方核心问题：安全走廊内部点与终点可能匹配到
同一个导引采样时间，官方 `10 ms` 下限再乘 `0.8` 后生成约 `8 ms` 的退化轨迹段，并直接触发
动力学约束失败。该项现已通过 `段长/max_vel` 几何时间下界修复：7个新增单元测试通过，Release
编译成功；同场景再次捕获5个原始0.008 s输入，修正后对应约束失败由5次降为0次。可复现补丁为
[`patches/super_geometric_time_lower_bound.patch`](patches/super_geometric_time_lower_bound.patch)。
同时，高层 `WAIT_MAP_CLOSURE`、PX4悬停桥与SUPER保留旧目标之间仍缺少明确的目标取消协议。
完整证据、A/B结果和剩余边界见
[`docs/super_optimizer_and_goal_protocol_root_cause_report.md`](docs/super_optimizer_and_goal_protocol_root_cause_report.md)。

这次修复只关闭了“重复时间戳→8 ms退化段→对应动力学失败”链路，不代表其他L-BFGS线搜索、
位置约束或目标生命周期问题已经解决。在完成剩余隔离回归前，不能仅凭日志中的
`ReplanOnce succeed`判定真机可飞，也不能通过扩大高度围栏或继续叠加入口过滤来掩盖局部轨迹问题。

### MID360 与 NUC 的固定地址

MID360 雷达地址固定为 `192.168.1.157`，NUC 专用雷达网口 `enp89s0` 已建立
NetworkManager 永久连接 `mid360-static`：

```text
NUC enp89s0: 192.168.1.10/24
MID360:      192.168.1.157/24
网关/DNS:    不设置
```

该连接已设置为开机自动启用，并设置 `ipv4.never-default=yes`，不会替代 NUC 原有的
上网默认路由。正常重启后不需要再次执行 `ifconfig`、`ip addr add` 或手工配置地址。
雷达关闭时网口可能显示无载波，这是正常现象。只在需要手动重新激活连接时执行：

```bash
nmcli connection up mid360-static
```

### 仅启动定位链路：MID360 → Fast-LIO2 → PX4

该入口只启动 MID360 驱动、Fast-LIO2、MAVROS 和外部视觉位姿桥，不启动 SUPER、
不发送飞行目标、不切换模式，也不会解锁飞机：

```bash
source /home/nuc/super_ws/src/mine_uav_control/scripts/setup_fastlio2_super_env.sh
export ROS_MASTER_URI=http://localhost:11312
roslaunch mine_uav_control px4_fastlio_localization.launch rviz:=false
```

如需同时查看点云，把最后一项改为 `rviz:=true`。保持上述终端运行，在另一个终端检查：

```bash
source /home/nuc/super_ws/src/mine_uav_control/scripts/setup_fastlio2_super_env.sh
export ROS_MASTER_URI=http://localhost:11312
rostopic hz /livox/imu
rostopic hz /Odometry
rostopic hz /mavros/vision_pose/pose_cov
rostopic echo -n 1 /mine_uav/task1/vision_healthy
rostopic echo -n 1 /mine_uav/task1/vision_status
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/estimator_status
```

数据路径是 `MID360 → /livox/lidar,/livox/imu → Fast-LIO2 /Odometry →
/mavros/vision_pose/pose_cov → MAVROS → PX4 EKF2`。因此 `/Odometry` 是 Fast-LIO2
在 NUC 上产生的位姿，不是雷达直接产生的消息。此前现场已验证其典型频率分别约为
IMU 200 Hz、Odometry 10 Hz、发送给 PX4 的 vision pose 30 Hz。

### 启动完整任务一

完整任务一已经包含上述全部定位节点，不能与 `px4_fastlio_localization.launch` 同时运行。
若前者正在运行，先在它的终端按 `Ctrl+C`，再执行：

```bash
source /home/nuc/super_ws/src/mine_uav_control/scripts/setup_fastlio2_super_env.sh
export ROS_MASTER_URI=http://localhost:11312
roslaunch mine_uav_control task1_real.launch rviz:=true
```

当前任务调度忽略 CH6；CH7/CH11 的首次稳定电平只建立基线。确认定位健康后，
由飞手手动解锁、起飞并稳定悬停，再拨动 CH7 到另一稳定位置并保持约 0.5 秒。
任务一随后以当前位姿建立 home，开始向 SUPER 发布探索目标，指令桥预发送悬停目标后
自动请求 OFFBOARD。节点不会自动解锁或自动起飞。**任务运行期间再次拨动 CH7/CH11
会被忽略，CH7 拨回低位不是中止键。**人工接管应使用已拆桨验证的 CH5 飞行模式切换；
真实定位丢失时不能假设 `POSCTL` 一定能够悬停。

任务一运行状态可在第二个终端查看：

```bash
source /home/nuc/super_ws/src/mine_uav_control/scripts/setup_fastlio2_super_env.sh
export ROS_MASTER_URI=http://localhost:11312
rostopic echo /mine_uav/mission/status
rostopic echo /mine_uav/exploration/status
rostopic echo /mine_uav/task1/command_status
rostopic echo /goal
```

首次带飞前仍应拆桨验证 CH7、定位中断保护和人工接管；当前任务一SUPER名义轨迹速度
上限为 `1.0 m/s`，指令异常保护阈值为 `1.1 m/s`，PX4 本地高度硬上限为
`1.8 m`（不是相对任务 home）。PX4实际速度仍可能存在短时跟踪误差，不能把规划上限当成实测速度保证。

### 实验室临时测试启动逻辑（手动起飞后 CH7 启动）

`task1_real.launch` 已启用 `require_mission_enable_edge=true`，因此任务一不会因节点启动、
旧的锁存消息或第一帧里程计而提前建立 home。现场按以下顺序操作：

1. 确认 CH7、CH11 的 MAVROS 通道映射；启动时保持任一稳定位置，供调度器建立基线。
2. 启动任务一链路，确认 Fast-LIO2 点云/位姿、MAVROS 连接和 PX4 状态正常。
3. 拆桨完成手动解锁、起飞并悬停，在 POSCTL/手动模式下稳定飞机。
4. 将 CH7 拨到另一位置。调度器对该变化去抖约 0.5 s；任务一收到
   `goaf_enable: false -> true` 后，在下一帧同步的点云+位姿处捕获当前位姿为 home，
   然后才开始发布探索目标给 SUPER。
5. 任务运行中再次拨动 CH7 或 CH11 会被忽略。任务完成或飞手切出 OFFBOARD 后，
   再次拨动 CH7 可启动新一轮任务一。该逻辑不会自动起飞。

当前任务调度以 CH7 的任一稳定电平变化启动任务一；CH6 不参与任务选择。
必须避免在实验室同时运行两个
调度器、两个 SUPER 或两个 PX4 setpoint 发布节点。SUPER 当前具备已知障碍膨胀、A*/安全
走廊、轨迹碰撞复查和备份轨迹等避障环节；高层决策器还会把墙后及当前不可达自由空间之外
的 frontier 过滤掉。它适合进行拆桨和低速实验验证，但真实飞行前仍必须单独验证点云坐标系、
外参、速度/制动距离、失联、定位中断和人工接管等安全条件。

任务一现在使用一条唯一的控制链：

```text
MID360 → Fast-LIO2 → 自主决策器 → SUPER
                     ↓            ↓
                  任务调度器   坐标/安全指令桥 → MAVROS → PX4
Fast-LIO2 ─────────→ 外部视觉位姿桥 ─────────────→ PX4 EKF2
```

`fastlio_px4_vision_bridge` 将 `/Odometry` 对齐到起飞时的 PX4 本地原点，
以 30 Hz 发布 `/mavros/vision_pose/pose_cov`。对齐只允许在未解锁状态建立，
Fast-LIO2 跳变、超时或 MAVROS 断开都会停止外部视觉输出。

`super_px4_command_bridge` 使用同一对齐关系，将 SUPER 的
`/planning/pos_cmd` 转成 MAVROS 本地位置目标。输出受独立自动允许开关、任务一选择、
视觉定位健康、MAVROS 连接和软件抑制门控。满足条件并已由操作者解锁后，节点先预发送
悬停点，再自动请求 OFFBOARD；节点本身永不解锁 PX4。SUPER 短暂重规划时会发送
当前位置悬停目标；任务切走、定位失效、通信断开、指令越界或任务完成时会退出受控
OFFBOARD。真机默认退出到 `POSCTL`，必须保证遥控器和 PX4 失效保护配置可用。
若飞手通过 CH5 或 PX4 失效保护让飞控从受控 OFFBOARD 切到其他模式，桥接器会锁住自动重新进入；调度器也会撤销任务。稳定电平重新建立基线后，必须再次产生新的 CH7 电平变化才可能重新授权。此项已有模拟 MAVROS 模式切换的 ROS 回归及 40 m 仿真闭环回归，但仍需拆桨实测遥控器 CH5 和真实 PX4 模式切换，不能把仿真当作人工接管实机验收。见 [`docs/task1_manual_override_guard.md`](docs/task1_manual_override_guard.md)。
任务执行中的 Fast-LIO2→PX4 对齐突变也会锁存故障并退出受控 OFFBOARD；这只是桥接层保护，不代表 SUPER 内部的规划高度约束已经完成。

统一启动文件（首次实机必须拆桨）：

```bash
source /home/nuc/super_ws/src/mine_uav_control/scripts/setup_fastlio2_super_env.sh
export ROS_MASTER_URI=http://localhost:11312
roslaunch mine_uav_control task1_real.launch
```

真机入口包含 `task1_real_time_guard.py`：若共用曾运行 SITL 的 ROS master，
`/use_sim_time=true` 且无 `/clock` 时它会报错并停止整个启动。先停止 SITL，
改用真实时间 master；不要把静止的仿真时间误当真实定位故障。2026-09-18
隔离检测证据见[真机雷达与决策输入诊断](docs/task1_live_mid360_decider_2026-09-18.md)。

关键状态检查：

```bash
rostopic echo /mavros/state
rostopic hz /livox/lidar
rostopic hz /Odometry
rostopic echo /mine_uav/task1/vision_status
rostopic echo /mine_uav/mission/status
rostopic echo /mine_uav/task1/command_status
```

软件服务现在是可选的总抑制开关，默认允许；真正的自动任务授权来自独立遥控通道
（默认物理 CH7）。需要在调试时重新打开软件门可使用：

```bash
rosservice call /super_px4_command_bridge/enable "data: true"
```

关闭指令门：

```bash
rosservice call /super_px4_command_bridge/enable "data: false"
```

当前雷达发现应答报告的设备类型是 `35`，即 `Mid360s`，不是类型 `9` 的
`MID360`；实际地址是 `192.168.1.157`，NUC 雷达网口是
`192.168.1.10/24`。正式启动必须使用 `msg_MID360s.launch` 和
`MID360s_config.json`，否则 SDK 虽能收到发现应答，也会因设备类型不匹配而忽略。
`MID360s_config.json` 使用 SDK2 1.4.3 的数组式
`host_net_info`。驱动不再在初始化时静态强制 Sampling；只有真正收到该设备的
点云或 IMU 包时才进入 Sampling，因此能从漏掉的异步配置回调中恢复，又不会在
雷达离线时伪报正常。对应补丁为
`patches/livox_ros_driver2_mid360s_reconnect.patch`。

Fast-LIO2 不应跨越雷达断电继续使用旧 EKF 和地图。点云或 IMU 时间戳回跳、或
数据间隔超过 1 秒时，`laserMapping` 会主动退出；`mapping_mid360.launch` 在 1 秒后
自动拉起全新进程。对应补丁为 `patches/fast_lio2_sensor_restart.patch`。因此正式
系统不要求“先插雷达还是先插飞控”，两个设备可以独立恢复，但恢复期间安全门会
保持关闭。

在未修改的 `livox_ros_driver2` 目录中应用时使用：

```bash
git apply --unidiff-zero /path/to/frontier-upload/patches/livox_ros_driver2_mid360s_reconnect.patch
git apply --unidiff-zero /path/to/frontier-upload/patches/fast_lio2_sensor_restart.patch
```

当前SUPER工作区以港大官方提交 `2ad3419` 为核心基线，应用指令坐标帧适配和已回归验证的
几何时间下界补丁：

```bash
cd /path/to/SUPER
git apply /path/to/frontier-upload/patches/super_ros1_fastlio_task_gate.patch
git apply /path/to/frontier-upload/patches/super_geometric_time_lower_bound.patch
```

任务点云、里程计话题、三维目标高度及任务动力学参数全部由项目自己的
`config/super_task1.yaml` 注入，不修改SUPER官方示例配置。以下三个历史补丁只保留作审计，
默认不得应用：`super_optimizer_diagnostics.patch`、`super_preserve_reachable_goal.patch`、
`super_remove_duplicate_backup_optimize.patch`。其中诊断补丁还混有 `out_t → eval_t` 的算法变化，
后续必须拆分后才能单变量验证。

注意：在正式飞行前仍必须通过移动机体验证 PX4 确实融合外部视觉、标定
雷达 IMU 坐标与飞行器 FRD 机体系安装关系，并确认 `EKF2_EV_CTRL` 的高度源选择。
“话题有数据”不等于 EKF 已可靠融合，也不等于已经可以带桨飞行。

## 操作顺序

1. 确认 MAVROS 状态为“已连接”。
2. 设置高度、半径、速度、爬升速度和圆弧角度。
3. 点击“开始发送轨迹”，等待界面状态进入 `ready`。
4. 使用 QGC 确认飞行区域安全，切换 OFFBOARD 并解锁。
5. 需要终止轨迹时点击“停止轨迹并保持”；需要降落时使用 QGC。

程序不会自动解锁或自动起飞。首次接入真机应拆桨，并先验证 PX4 的姿态、位置、
电池、遥控器和失联保护配置。

## Fast-LIO2 → 自主探索决策器 → SUPER

新增的 `super_exploration_decider` 位于 Fast-LIO2 和 SUPER 之间，职责是：

- 订阅 Fast-LIO2 的 `/cloud_registered` 和 `/Odometry`；
- 用点云射线建立轻量局部 free/occupied voxel map；
- 从当前位置出发，在膨胀后的已知自由体素上做六连通洪泛搜索；
- 只从该可达连通分量提取 frontier candidate，目标必须同时为
  `known-free`、满足机体安全间距且位于安全围栏内；
- 按信息增益、距离和已覆盖目标去重选择下一个观察点；
- 到达目标后继续选择下一个目标；
- 地图边界闭合并稳定、收到返航命令或电量低于阈值时返回 home。

### 任务一的机头方向优先状态机

当前决策器已增加两阶段方向策略：

1. `FORWARD_PRIORITY`：从当前 Fast-LIO2 位姿四元数计算机体 yaw，只在机头前方配置角度内的 frontier 中选点，并对机头方向增加评分权重。
2. `FRONTIER_FALLBACK`：当前方障碍连续多帧确认，或连续多帧既没有左右墙体观测又没有前方可行 frontier 时启用；候选目标仍受任务轴横向硬走廊限制，不会因一侧点云缺失而无限向另一侧漂移。

左右墙体和前方障碍使用当前 `/cloud_registered` 的点云证据，并通过连续帧确认抑制单帧误检。地面点会通过垂直方向过滤，避免把地面误判为前方墙体。相关参数在 `config/super_exploration_decider.yaml` 中，包括前方角度、墙体量程、障碍确认帧数和机头方向权重。

该状态机只决定发布给 SUPER 的观察目标，不直接发布 PX4 setpoint，也不改变 Fast-LIO2 → PX4 的定位链路。高层可达性过滤防止把墙后或断开的自由区域选作frontier目标；SUPER继续使用ROG-Map膨胀障碍、A*、安全飞行走廊、在线碰撞复查和备份轨迹作为第二层保护。地图完成/返航判据仍是实验功能，尚未按未知尺寸采空区最终方案验收，不能仅凭当前状态量宣称实机建模完整。原三面墙覆盖率只作诊断。

启动：

```bash
source /home/nuc/super_ws/src/mine_uav_control/scripts/setup_fastlio2_super_env.sh
roslaunch mine_uav_control super_exploration_decider.launch
```

它向 SUPER 发布 `geometry_msgs/PoseStamped`，默认目标话题为 `/goal`；它不直接向 PX4 发布控制命令。SUPER 的 `fsm_node` 需要同时运行，并且其配置中的 `fsm.click_goal_topic` 要保持为 `/goal`。

注意：当前 SUPER 的 `FsmRos1::setGoalPosiAndYaw()` 会在 `fsm.click_height > -5` 时强制覆盖目标 z。要让观察点使用点云计算出的三维 z 高度，应在 SUPER 使用的 YAML 中设置 `fsm.click_height: -10.0`。

当前真机链路统一使用 Fast-LIO2 的 `camera_init`：`/Odometry`、`/cloud_registered`、决策器 `/goal`、SUPER 的 ROG-Map 和 `/planning/pos_cmd` 都必须使用该坐标系。决策器不做 TF 变换，`strict_cloud_frame: true` 会拒绝其他坐标系的点云；SUPER 到 PX4 的转换由独立对齐桥完成，这不等于雷达安装外参已自动标定。

观察点高度通过 `min_observation_height_above_home` 和 `max_observation_height_above_home` 限制在任务起点之上。当前默认范围为 `0.5–2.5 m`，与本次 SUPER 局部地图的有效高度范围匹配；正式飞行前必须根据采空区净高、雷达安装高度和安全裕量重新标定。返航目标的水平位置使用原始 home，垂直位置由 `return_home_height_offset` 控制。真机默认偏移为 `0 m`；复杂场景PX4 SITL验证中覆盖为 `1.8 m`，返航阶段保持安全巡航高度，降落应由独立状态处理。

`max_exploration_radius_from_home` 是相对任务起点的水平安全围栏，避免目标随着滚动点云不断向外漂移。真机默认值为 `35 m`，未知尺寸场景不能按预估洞长设置，而应根据续航、动态返航代价和安全制度给出允许上限。当前 `max_task_lateral_offset=4 m` 仍是早期狭长巷道限制，不适合直接验收未知宽度的大采空区，后续覆盖策略阶段必须将它与完成判据解耦。

必须使用上面的统一环境脚本同时加载两个 Catkin 工作空间。直接依次 source 两个工作空间时，后一个会覆盖前一个的搜索路径，常见现象是 `rostopic` 无法加载 `livox_ros_driver2/CustomMsg`。统一脚本会同时保留 Fast-LIO2 与 SUPER 的 ROS 包、Python 消息和动态库路径。

观察运行状态：

```bash
rostopic echo /mine_uav/exploration/status
rostopic echo /mine_uav/exploration/model_coverage
rostopic echo /mine_uav/exploration/model_complete
rostopic echo /mine_uav/exploration/finished
rostopic echo /mine_uav/exploration/returning
rostopic hz /cloud_registered
rostopic hz /Odometry
```

手动触发返航：

```bash
rostopic pub -1 /mine_uav/exploration/return_home std_msgs/Bool "data: true"
```

重要限制：这是第一版 frontier/viewpoint 决策器，使用本节点的轻量 voxel map；SUPER 仍使用自己的 ROG-Map。两者共享同一份 Fast-LIO2 输入，但不是同一份内存地图。当前已确认的是“目标可达性过滤 + 下层避墙”，未知尺寸采空区的完整覆盖和最终返航判据尚未验收。此前累计地图终墙锁存实验出现过入口/顶板组合误判，相关实验补丁已从工作版本剥离。后续应将全局覆盖地图独立出来，再单独设计和回归最终完成条件。

## Fast-LIO2 → 任务调度器 → 采空区/竖井任务

`mission_scheduler` 通过 MAVROS 的 `/mavros/rc/in` 读取 CH7 与 CH11，并发布唯一的任务选择结果：

- 上电默认手动；首次稳定 RC 读数只建立基线，不启动任务；
- CH7 稳定电平变化（两个方向均可）：启动任务一 `goaf_exploration`；
- CH11 稳定电平变化（两个方向均可）：在任务二可用时启动 `shaft_exploration`；
- 任务运行中忽略两个通道的新变化。完成或飞手退出 OFFBOARD 后回到 `hold`，新的变化可再次启动任务；
- 遥控器或 MAVROS 断链会撤销当前任务；任务一还要求 Fast-LIO2 位姿新鲜。任务二由自身的相对深度与测距逻辑检查传感器，生产开关仍为禁用。

启动：

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
roslaunch mine_uav_control mission_scheduler.launch
```

默认 `goaf_trigger_channel: 6`、`shaft_trigger_channel: 10` 是 ROS 数组下标，对应物理 CH7、CH11；应通过 `/mavros/rc/in` 和 QGroundControl 核实通道顺序及开关电平。CH6 不参与任务调度。

主要输出：

```text
/mine_uav/mission/active_task   std_msgs/UInt8   0=hold, 1=采空区, 2=竖井
/mine_uav/mission/goaf_enable   std_msgs/Bool    任务一是否允许运行
/mine_uav/mission/shaft_enable  std_msgs/Bool    任务二是否允许运行
/mine_uav/mission/return_home   std_msgs/Bool    是否请求当前任务返航
/mine_uav/mission/status        std_msgs/String  状态与健康信息
```

该节点只做任务调度和安全门控，不直接向 PX4 发布 setpoint。两个任务都应遵守：只有收到对应 `*_enable=true` 时才发布自己的任务输出；最终由唯一的 PX4 command router 选择当前任务输出，避免两个任务同时控制飞行器。任务一的 `super_exploration_decider` 已订阅 `goaf_enable`；任务二的 `shaft_mission_node` 已订阅 `shaft_enable`，但仅有仿真专用 PX4 指令路由器，真机任务二仍被禁用。定位丢失时不会请求无法保证安全的自主返航。

### SITL 联调

#### 任务一 PX4/Gazebo 完整动态闭环

当前 40×40×30 m 入口平台场景的 run10/run11 连续通过飞行与五面地图离线验收：自动深入、
返航并切 `AUTO.LOITER`；run11 的 home 误差 0.155 m、最大横偏 0.891 m、桥接故障 0，
左右/尽头三墙的 2 m 三维网格全覆盖，地板 98.3%、顶板 100%，累计地图未达到容量上限。
这只说明已知 Gazebo 场景和当前参数组合通过，**不批准真机自主放飞**。历史失败、
因果链、仿真几何过滤的边界与残余风险见
[`docs/task1_40m_sitl_ab_report_2026-09-16.md`](docs/task1_40m_sitl_ab_report_2026-09-16.md)。

`task1_px4_sitl.launch` 已将采空区世界、带 MID360 风格三维雷达的 Iris、PX4 SITL、
Gazebo、MAVROS、任务调度器、自主决策器、SUPER 和 PX4 指令桥合并为一条动态闭环。
Gazebo Ray 传感器输出机体系 `/mine_uav/sitl/mid360/points`，注册适配器转换为
`camera_init` 下的 `/cloud_registered`；40 m 测试场景额外以 Gazebo 位姿及静态世界
几何筛除模拟假回波，默认其他场景和真机不启用；决策器及SUPER沿用真机接口，不再使用
预先枚举的解析墙面点云。该模块模拟三维测距、视场、遮挡、量程和高斯噪声，但不是
Livox UDP数据包、MID360非重复扫描时序或真实Fast-LIO2 EKF。

必须使用独立 ROS Master，避免向默认 `11311` 上的真实 MAVROS 发送任何仿真控制：

```bash
# 终端 1
roscore -p 11312

# 终端 2
export ROS_MASTER_URI=http://127.0.0.1:11312
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
source /home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  /home/nuc/PX4-Autopilot /home/nuc/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=/home/nuc/PX4-Autopilot:/home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic:$ROS_PACKAGE_PATH
roslaunch mine_uav_control task1_px4_sitl.launch gui:=true rviz:=true
```

无界面验收使用 `gui:=false rviz:=false`。仿真适配器只在 `/use_sim_time=true` 时允许
自动解锁，并在日志中明确警告不可连接真机。它模拟 CH6 选择任务一、CH7 从低位切到
自动允许。SITL 没有真实遥控输入，`POSCTL` 无法可靠接管，因此任务完成后专门退出到
`AUTO.LOITER`；真机启动文件仍保持 `POSCTL`。

可观察的话题：

```bash
rostopic echo /mavros/state
rostopic echo /mine_uav/exploration/status
rostopic echo /mine_uav/exploration/model_coverage
rostopic echo /mine_uav/exploration/model_complete
rostopic echo /mine_uav/task1/command_status
rostopic echo /goal
rostopic hz /mine_uav/sitl/mid360/points
rostopic hz /cloud_registered
rostopic hz /mine_uav/sitl/global_cloud
rostopic hz /Odometry
rostopic hz /mavros/setpoint_raw/local
```

本次自动验收结果：任务选择后 PX4 解锁并进入 OFFBOARD。普通 frontier 在远距离看到
尽头后消失时，决策器主动发布 `(19.5, 0, 1.11) m` 的尽头接近观察点；飞机推进到
纵向 `17.89 m` 后，判定尽头深度 `22.50 m`、左右墙覆盖率 `1.00/1.00`、最大连续
缺口 `0/0`、尽头墙横向跨度 `9.00 m`，并连续确认 `4/4` 周期。随后发布约 `1 m` 高的
home返航点，最终位置约 `(-0.35, 0.17, 1.05) m`，`model_complete=true`、
`finished=true`、指令桥状态为 `TASK1_COMPLETE`，PX4切到 `AUTO.LOITER`，之后不再
发布任务setpoint。模拟点云和里程计均稳定为 `10 Hz`。

当前默认使用“前向边界闭合 + 中心走廊无可执行前沿 + 地图增长稳定 + 多周期确认”触发
返航，左右墙和尽头墙覆盖率仅作诊断，旧的无 frontier 单条件判据默认关闭。本判据验证
的是决策器占据体素中的几何闭合与稳定性，不能
替代雷达建模模块对点云密度、配准误差、孔洞和最终模型质量的验收，也不能替代真机外部
视觉融合、安装外参和碰撞裕量测试。

任务完成后，上游按设计停止 `/mine_uav/setpoint_cmd`。`offboard_bridge` 会读取
`/mavros/state`：如果 PX4 仍处于 `OFFBOARD`，陈旧或缺失指令仍会触发安全告警并停止
转发；如果 PX4 已切换到 `AUTO.LOITER`、`POSCTL` 等非 OFFBOARD 模式，则静默停止，
不会把正常的任务收尾重复报告为 `Setpoint command is ... old`。

#### 复杂地形采空区场景

新增 `worlds/goaf_complex.world`，包含连续折线侧墙、支护柱、落石、低矮地面障碍和更长
的主巷道。使用原有雷达、决策器、SUPER和PX4闭环，只替换世界文件：

```bash
roslaunch mine_uav_control task1_px4_sitl.launch \
  world:=/home/nuc/super_ws/src/mine_uav_control/worlds/goaf_complex.world \
  gui:=true rviz:=true
```

复杂场景的尽头约为 `x=30 m`，启动时可增加 `max_exploration_radius:=34.0` 覆盖默认安全半径：

```bash
roslaunch mine_uav_control task1_px4_sitl.launch \
  world:=/home/nuc/super_ws/src/mine_uav_control/worlds/goaf_complex.world \
  max_exploration_radius:=34.0 gui:=true rviz:=true
```

第63轮的基础复杂场景回归曾完整通过：PX4进入OFFBOARD，决策器完成三面墙判定并返航，
最终发布 `model_complete=true`、`finished=true`，指令桥进入 `TASK1_COMPLETE`，
PX4切换到 `AUTO.LOITER`。后续为复现用户报告的SUPER原地不动问题，又定位并修正了
Gazebo Iris 自体回波：当前长距模型实测仍有约0.803 m的旋翼/机体边界回波，仿真适配器的
`self_filter_xy_radius` 已调整为1.00 m；该过滤只对仿真适配器生效，真实Fast-LIO2
不使用它。

第64轮使用两侧障碍版本重新测试时，PX4进入OFFBOARD，飞机前进至约25.9 m，三面墙
判据达到 `end_seen=true`、左右覆盖率 `1.00/1.00`、缺口 `0/0`、确认
`4/4`，决策器进入 `RETURNING`。本轮对话结束前返航尚未完全回到起点，因此该轮
只记为“探索完成、返航进行中”，不宣称返航闭环已验收。SITL启动文件另将出生高度设为
1.15 m，表示真实机载系统应在任务一前独立完成起飞阶段。

第65轮继续调整复杂场景：支护柱移到侧墙附近，返航高度改为保持安全巡航高度
return_home_height_offset=1.8，不在任务返航阶段下降穿过已建模地面。最终测试中飞机
前进约24.9 m，三面墙判定达到 confirm=4/4，返航到入口附近
(-0.27, 0.09, 1.78) m；决策器状态为 COMPLETE，指令桥为 TASK1_COMPLETE，
PX4切换到 AUTO.LOITER。这证明当前仿真任务一闭环已通过；真实系统仍需单独设计起飞、
降落和返航失联保护状态机。

第66轮把“以2 m/s探索”落实为流畅性目标，而不是要求飞行器始终精确保持2 m/s。SUPER
的轨迹约束设置为 `max_vel=1.0 m/s`、`max_acc=1.5 m/s²`、`max_jerk=20 m/s³`；
决策器使用8 m前视目标，并在距当前目标3 m时提前交接下一个目标。目标选择先尝试全部
机头中心线距离，再考虑侧向候选；前方墙体需要至少20个点且具备1.0 m横向跨度和0.8 m
垂直跨度，避免单点、地面切片或支护构件导致误返航。SUPER仍负责局部避障，因此上述
逻辑是任务层主方向约束，不是强制直线穿越障碍。

复杂场景最终rosbag回归中，首个目标为 `(7.49, 0.00, 0.77) m`，尽头安全接近目标为
`(26.50, 0.00, 0.77) m`，三面墙确认后发布入口返航目标。有效航段实际速度中位数
`1.94 m/s`、90分位 `2.04 m/s`，低于 `0.2 m/s` 的最长连续时间为 `0.30 s`；
横向范围为 `-0.43–0.36 m`，全程无指令桥限幅故障。飞机最终回到入口并退出OFFBOARD、
进入AUTO.LOITER。实际速度瞬时最大值约 `2.48 m/s`，说明2 m/s是规划巡航上限而不是
真实速度刚性钳位；真机应通过PX4速度环参数和逐级试飞继续检查跟踪超调。

本轮仍观察到SUPER的备份轨迹/指数轨迹优化偶发失败并触发快速重规划，未造成停飞或
闭环失败，但不应视为已消除。后续需结合轨迹日志调整膨胀半径、局部地图、优化器和
PX4跟踪参数，并增加“单位时间位移、低速持续时间、重规划失败率”作为正式验收指标。

#### rosbag自动验收

`analyze_task1_sitl_bag.py`把任务一SITL结果转换为可重复的PASS/FAIL判定。运行仿真时可在
另一个终端记录轻量验收话题（不录大体积点云）：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11312
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
mkdir -p /home/nuc/task1_logs
rosbag record -O /home/nuc/task1_logs/task1_acceptance.bag \
  /clock /Odometry /goal /mavros/state \
  /mavros/local_position/pose /mavros/local_position/velocity_local \
  /planning/pos_cmd /mine_uav/task1/fastlio_to_px4_alignment \
  /mine_uav/exploration/finished \
  /mine_uav/exploration/model_coverage /mine_uav/exploration/status \
  /mine_uav/task1/command_status /rosout_agg
```

任务完成并进入`AUTO.LOITER`后停止录包，再执行：

```bash
rosrun mine_uav_control analyze_task1_sitl_bag.py \
  /home/nuc/task1_logs/task1_acceptance.bag \
  --json-output /home/nuc/task1_logs/task1_acceptance.json
```

另可用 `scripts/analyze_task1_height_contract.py` 按对齐后的 PX4 本地高度围栏检查 SUPER 发布样本。历史 SITL bag 未记录对齐话题时需显式传 `--alignment-z 0`；真机 bag 不得擅自假定偏移为零。诊断结论、run04/run11 对照与尚未实施的规划层修复见 [`docs/task1_height_envelope_contract.md`](docs/task1_height_envelope_contract.md)。

默认验收包括：进入OFFBOARD、任务COMPLETE、地图闭合ready、退出到AUTO.LOITER、至少三个
任务目标、路径长度不小于45 m、水平返航误差不大于1 m、最大横向偏移不大于1.5 m、
有效航段速度中位数不小于1.5 m/s、低于0.2 m/s的连续时间不超过1 s、实际速度不超过
3 m/s、SUPER规划速度不超过2.05 m/s、规划加速度不超过1.60 m/s²、里程计不低于5 Hz、
OFFBOARD阶段不超过90 s且无指令桥故障。任一硬指标失败时程序返回非零退出码。

`/rosout_agg`存在时程序还会统计SUPER优化失败日志。考虑到当前已知的快速重规划警告，
默认只报告数量；调参形成稳定基线后可用`--max-super-failures N`把它升级为硬验收门槛。
录包先于Gazebo启动也没有问题：验收器会利用`/clock`把rosbag接收时间重新映射到仿真
时间，避免暂停、启动和仿真倍率造成任务时长及低速时长误判。

当前复杂场景基线保留`backup_traj/penna_acc=1.0e5`。单变量试验中，`5.0e5`虽把备用
优化告警从79条降至5条，但任务阶段增至34 s并产生1.136 s连续低速，验收FAIL；
`3.0e5`则进入持续的走廊/静止重规划失败。不要仅为减少日志数量而提高该权重或放宽
`penna_margin`，后续应联合检查备用轨迹时间分配、初值和重规划触发条件。

专用RViz配置 `rviz/task1_mid360.rviz` 使用 `camera_init` 作为Fixed Frame，并默认显示
`/cloud_registered`、累计点云和PX4轨迹。此前使用SUPER的 `top_down.rviz` 时Fixed
Frame为 `world`，而仿真没有发布 `world -> camera_init` TF，因此话题虽为10 Hz也可能
完全不可见。传感器级隔离测试中，Gazebo原始点云稳定为10 Hz、每帧11520个射线采样；
去掉最小量程和最大量程无效点后，注册点云约为每帧6540个有效点，frame为
`camera_init`。修改启动文件后必须完整停止并重新启动旧仿真，运行中的Gazebo不会热加载
新机体模型。

#### 任务一算法闭环仿真（建议先运行）

该模式使用 SUPER 自带的 `perfect_drone_sim` 产生 360° 模拟点云和理想里程计，闭环运行“模拟传感器 → 自主决策器 → SUPER → 模拟无人机”。它可以直接观察 frontier、目标、局部地图、规划轨迹和无人机运动，但不包含 PX4、MAVROS 和 Fast-LIO2 状态估计。

为了不和真雷达使用的 ROS Master 冲突，建议使用独立端口：

```bash
source /home/nuc/super_ws/src/mine_uav_control/scripts/setup_fastlio2_super_env.sh
export ROS_MASTER_URI=http://127.0.0.1:11312
roslaunch mine_uav_control goaf_algorithm_sim.launch
```

RViz 默认配置会显示 `/cloud_registered`、SUPER 占据地图/轨迹、无人机模型和运动路径。要显示决策器候选点，在 RViz 中点击 `Add → MarkerArray`，将 Topic 设为 `/mine_uav/exploration/frontiers`。无图形界面测试时使用：

```bash
roslaunch mine_uav_control goaf_algorithm_sim.launch rviz:=false
```

#### PX4 SITL 调度器联调

可以先使用：

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
export ROS_PACKAGE_PATH=/home/nuc/PX4-Autopilot:/home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ROS_PACKAGE_PATH
source /home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  /home/nuc/PX4-Autopilot /home/nuc/PX4-Autopilot/build/px4_sitl_default
roslaunch mine_uav_control mission_scheduler_sitl.launch
```

该启动文件使用 PX4 SITL/MAVROS 的 `/mavros/local_position/odom` 作为调度器的临时位姿健康输入，并不代表真实 Fast-LIO2 已接入。调度器的 RC 输入仍然是 `/mavros/rc/in`。SITL 中可以先验证 PX4/MAVROS 连接、RC 通道、任务一/任务二切换和位姿超时保护；任务一 SUPER 的完整规划还需要另行提供 SITL 点云或回放 Fast-LIO2 点云，因为标准 PX4 SITL 不会自动产生 Fast-LIO2 的 `/cloud_registered`。

#### 150 m 采空区大场景仿真

用于观察“平台外悬停 → 狭窄入口 → 100 m 宽、30 m 高、150 m 深采空区 → 前墙 → 返航”的完整任务一闭环：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11312
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
source /home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  /home/nuc/PX4-Autopilot /home/nuc/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=/home/nuc/PX4-Autopilot:/home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ROS_PACKAGE_PATH
roslaunch mine_uav_control task1_large_platform_sitl.launch gui:=true rviz:=true
```

该专用场景使用理想化 Gazebo 三维射线模型模拟 MID360，不是 Livox 原始数据、回波强度或真实噪声模型。仿真专用雷达范围约 70 m；实际 MID360 约 60 m 的有效探测距离意味着前墙通常要在无人机接近后才能识别。第三次运行已验证：无人机到达约 154.5 m，末端墙深度 157.5 m，墙面跨度 85 m，左右覆盖率 0.97/1.00，连续确认 4/4，随后返航并进入 `AUTO.LOITER`。

#### 40 m × 40 m × 30 m 空旷采空区仿真

当前展示场景采用入口外中部高度平台、40 m 深、40 m 宽、30 m 高的空旷采空区，内部不设置立柱或横向障碍：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11312
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
source /home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  /home/nuc/PX4-Autopilot /home/nuc/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=/home/nuc/PX4-Autopilot:/home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ROS_PACKAGE_PATH
roslaunch mine_uav_control task1_40m_platform_sitl.launch gui:=true rviz:=true
```

该场景把任务级目标限制在入口航向轴左右 1 m 内，正常前向 look-ahead 只搜索中心线左右 0.5 m；SUPER仍负责局部轨迹平滑、碰撞检测和必要的避障偏移。该限制用于空旷涵洞主航段，不能替代真机刹停距离、点云外参及失效接管测试。

2026-09-17 连续目标交接修复后，同一 40 m 场景两次独立 SITL 回归都完成探索和返航；SUPER 相邻指令速度跳变 >0.2 m/s 的次数从旧 run11 的 14 次降为 0。成因、改动范围、复算脚本与尚存的前墙等待空档见 [目标交接因果链报告](docs/task1_goal_handover_continuity_report_2026-09-17.md)。这只证明该仿真配置下的目标交接指令更连续，不是实机安全结论。

本轮进一步确认该仿真高度来自 PX4 本地估计而非真实 Fast-LIO2：最新闭环的执行指令无大幅瞬时高度跳变，但 SUPER 的两条未执行到峰值的未来轨迹超过 1.8 m 上限；前墙等待期也出现随实际高度下移的“移动式悬停”。证据、复算命令和风险边界见[高度来源与轨迹审计](docs/task1_vertical_trajectory_audit_2026-09-17.md)。

后续按因果链修复：桥接器在等待开始时锁存固定悬停目标；SUPER 的任务一走廊使用规划坐标系高度包络，并在提交前校验整条多项式，超界候选整体拒绝。40 m SITL run20 完成探索与返航，已发布多项式最高 1.782 m，等待下沉约 0.068 m；复现实验和实机坐标偏移限制见[高度合同修复报告](docs/task1_vertical_flight_contract_fix_2026-09-17.md)。SUPER 改动保存在 `patches/super_task1_flight_z_corridor.patch` 和 `patches/super_task1_flight_z_contract.patch`，应按顺序应用；真实 Fast-LIO2/有桨飞行仍未验收。

迷宫 SITL 已能在原 0.4 m 机体半径配置下穿过三道隔墙并返航，但第三墙实际机体外最小余量仅约 0.136 m，**未达到 1 m 安全验收**。直接放大 SUPER 半径或让 CIRI 查询三维膨胀地图的隔离探针均在第一墙前停滞。追查并修正了 CIRI 在竖直相切时的 NaN 与切平面朝向错误，确定性单测 3/3 通过，但后续 SITL 仍因严格净距下的走廊/侧向目标不可行而未穿第一墙。详见[迷宫审计](docs/task1_maze_sitl_audit_2026-09-17.md)和[CIRI 根因报告](docs/super_ciri_tangent_plane_root_cause_2026-09-17.md)。保留原任务配置；任务一不能据此用于有桨真机，任务二仍未开始。

2026-09-18 继续严格净距试验：新增隔离的水平净距/垂直高度带探针、SUPER 首目标拒绝回执与全占据路径越界保护；ROG 试验性水平/垂直分离膨胀和决策器分离半径均保留默认兼容行为。入口曾恢复前进，但新的探针在第一墙处错误选择入口后方前沿并朝外飞，已停止；**任务一严格净距闭环仍不通过，任务二继续后置，严禁把试验参数用于有桨实机**。见 [2026-09-18 对话记录](docs/mine_uav_project_conversation.md)。SUPER 与 ROG 的运行源码快照分别保存在 `patches/super_runtime_strict_clearance_snapshot_20260918.patch`、`patches/rog_map_anisotropic_inflation_snapshot_20260918.patch`；这两份是对运行工作树的归档快照，含与历史补丁重叠的改动，不能和那些补丁直接叠加应用。
SUPER 侧增量保存在 `patches/super_goal_continuous_retarget.patch`，需先有目标生命周期协议基线 `71eddfd`，不能直接应用到港大官方 `2ad3419`。

2026-09-18 新里程碑：**隔离的严格净距配置**在三道隔墙迷宫和无隔墙 40 m 场景都完成自主探索与返航；迷宫六面竖直墙体的规划/实际机体外最小余量约 1.650/1.671 m，空旷场景也通过。该配置现为 `task1_maze_sitl.launch` 的默认参数，空旷场景可用 `task1_40m_strict_sitl.launch`；两者都是仿真入口，不是实机入口。详细数据、bag 保存位置与复算方法见[严格净距报告](docs/task1_strict_clearance_probe_2026-09-18.md)。真实雷达、定位源融合、坐标对齐、机体尺寸和人工接管尚未重验，严禁把该 SITL 结果当作有桨真机放飞许可。任务二在此仿真里程碑后开始。

当前安全状态：SITL先建立1.5 m稳定悬停，再模拟CH7启动并在空中捕获home。run07～run10 已消除该场景先前入口返航高度越界，桥接器 1.8 m 围栏始终保留；但规划器内部虚拟高度上界与桥接围栏仍未完全统一，且 40 m 仿真使用理想化雷达和世界真值几何过滤，任务一不得直接用于有桨真机自主飞行。

任务原点在任务一第一次被允许时从当前 `/Odometry` 捕获，而不是无条件使用节点启动时的第一帧。因此实机可以先手动起飞并稳定悬停，再打开自动允许；返航高度是相对该任务原点的高度。任务一当前不做地形跟随：下方地面突然下降时不会自动跟着下降，地面突然升高则依靠点云障碍/规划保护，真正的地形跟随需要单独加入高度控制策略。

末端墙判定要求“通过任务中线的连续横向墙体”，不再使用最左点到最右点的简单跨度，避免入口平台边缘和两侧墙被拼成假墙。但狭窄走廊若存在中线前墙且满足真实配置的最小跨度，仍可能被判为末端；实机还应加入入口缓冲距离、走廊/采空区开阔度判据和连续多帧确认。

2026-09-18 任务二开始：新增隔离的竖井状态机和 ROS 节点，要求独立相对深度与下视测距同时新鲜，先下探、井底连续确认后回到记录的入口深度。6 项单测及 1 项 ROS 节点集成测试通过，包含理想 420 m 运动学闭环。通用任务节点只发布 `/mine_uav/shaft/velocity_intent_enu`；真实任务的 `shaft_task_available` 仍为 `false`。下视测距本身无法提供 400 多米返航所需的入口深度参考；实机必须另行解决定位与失效恢复。接口与下一关见[任务二竖井逻辑记录](docs/task2_shaft_logic_2026-09-18.md)。

任务二后续完成首个 **22 m PX4/Gazebo 接口闭环**：专用启动文件 `task2_shaft_22m_px4_sitl.launch` 仅在仿真中启用任务二，并用 Gazebo 世界真值模拟独立深度与下视测距；PX4 经 OFFBOARD 下探、井底触发、返航，完成后切 AUTO.LOITER。bag 显示离底最小机体外余量 1.373 m、XY 最大偏移 0.142 m。第一次完整闭环因理想测距基准错约 0.9 m 只剩 0.504 m 余量，失败与修正的 A/B 证据均在[任务二报告](docs/task2_shaft_logic_2026-09-18.md)。**这不等于真实 400 m 竖井或无可靠 Z 定位已验证**；生产调度器仍禁用任务二，未接真实测距、独立深度源或人工接管验证。

调度器现按 CH7/CH11 的稳定电平变化分别启动任务一/任务二；新版 ROS 边沿测试和两任务各自的 SITL 已通过。任务二专用 SITL 还可用 `range_drop_after_depth:=6.0` 在下探 6 m 后停止模拟测距，检查传感器超时、任务撤销和 PX4 模式交接。延长观测的故障试验中，从 FAULT 到 AUTO.LOITER 约 0.539 s，后续 24.667 s 内最大额外下沉约 0.152 m；这是标准 SITL 定位有效时的结果，不能外推到无可靠 Z 的实井。详见[任务二报告](docs/task2_shaft_logic_2026-09-18.md)。
