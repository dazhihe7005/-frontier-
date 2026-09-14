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

## 任务一真实闭环

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

统一启动文件（首次实机必须拆桨）：

```bash
source /home/nuc/super_ws/src/mine_uav_control/scripts/setup_fastlio2_super_env.sh
roslaunch mine_uav_control task1_real.launch
```

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
- 从已知自由空间与未知空间的边界提取 frontier candidate；
- 按信息增益、距离和已覆盖目标去重选择下一个观察点；
- 到达目标后继续选择下一个目标；
- 连续没有新 frontier、收到返航命令或电量低于阈值时返回 home。

### 任务一的机头方向优先状态机

当前决策器已增加两阶段方向策略：

1. `FORWARD_PRIORITY`：从当前 Fast-LIO2 位姿四元数计算机体 yaw，只在机头前方配置角度内的 frontier 中选点，并对机头方向增加评分权重。
2. `FRONTIER_FALLBACK`：当前方障碍连续多帧确认，或连续多帧既没有左右墙体观测又没有前方可行 frontier 时启用；优先选择非前方 frontier，避免继续向已确认的前方障碍飞行。

左右墙体和前方障碍使用当前 `/cloud_registered` 的点云证据，并通过连续帧确认抑制单帧误检。地面点会通过垂直方向过滤，避免把地面误判为前方墙体。相关参数在 `config/super_exploration_decider.yaml` 中，包括前方角度、墙体量程、障碍确认帧数和机头方向权重。

该状态机只决定发布给 SUPER 的观察目标，不直接发布 PX4 setpoint，也不改变 Fast-LIO2 → PX4 的定位链路。当前已增加以进入时机头方向为纵轴的三面墙完成判据：把左右占据墙面按纵向分箱，检查覆盖比例和最大连续缺口；尽头墙必须同时具备中部回波和足够横向跨度，并要求飞机进入设定的尽头接近距离。普通 frontier 提前消失时，会主动生成尽头墙安全停距观察点。连续多周期满足条件后才发布模型完成并返航；无 frontier 超时不再单独代表建模完成。

启动：

```bash
source /home/nuc/super_ws/src/mine_uav_control/scripts/setup_fastlio2_super_env.sh
roslaunch mine_uav_control super_exploration_decider.launch
```

它向 SUPER 发布 `geometry_msgs/PoseStamped`，默认目标话题为 `/goal`；它不直接向 PX4 发布控制命令。SUPER 的 `fsm_node` 需要同时运行，并且其配置中的 `fsm.click_goal_topic` 要保持为 `/goal`。

注意：当前 SUPER 的 `FsmRos1::setGoalPosiAndYaw()` 会在 `fsm.click_height > -5` 时强制覆盖目标 z。要让观察点使用点云计算出的三维 z 高度，应在 SUPER 使用的 YAML 中设置 `fsm.click_height: -10.0`。

当前真机链路统一使用 Fast-LIO2 的 `camera_init`：`/Odometry`、`/cloud_registered`、决策器 `/goal`、SUPER 的 ROG-Map 和 `/planning/pos_cmd` 都必须使用该坐标系。该节点不做 TF 变换，`strict_cloud_frame: true` 会拒绝其他坐标系的点云。

观察点高度通过 `min_observation_height_above_home` 和 `max_observation_height_above_home` 限制在任务起点之上。当前默认范围为 `0.5–2.5 m`，与本次 SUPER 局部地图的有效高度范围匹配；正式飞行前必须根据采空区净高、雷达安装高度和安全裕量重新标定。返航目标的水平位置使用原始 home，垂直位置由 `return_home_height_offset` 控制。真机默认偏移为 `0 m`；PX4 SITL 验证中覆盖为 `1 m`，避免返航完成时触地。

`max_exploration_radius_from_home` 是相对任务起点的水平安全围栏，避免目标随着滚动点云不断向外漂移。真机默认值为 `35 m`，应按实际采空区长度和通信/续航能力调整；算法演示因 SUPER 示例地图较窄而覆盖为 `6 m`。

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

重要限制：这是第一版 frontier/viewpoint 决策器，使用本节点的轻量 voxel map；SUPER 仍使用自己的 ROG-Map。两者共享同一份 Fast-LIO2 输入，但不是同一份内存地图。后续可以把决策器改成直接读取 ROG-Map 的 frontier API，或将全局建模地图独立出来。

## Fast-LIO2 → 任务调度器 → 采空区/竖井任务

`mission_scheduler` 通过 MAVROS 的 `/mavros/rc/in` 读取遥控器二段开关，并发布唯一的任务选择结果：

- RC 低位：任务一 `goaf_exploration`，采空区自主探测与建模；
- RC 高位：任务二 `shaft_exploration`，竖井探测；
- 遥控器丢失、开关处于中间无效区或 Fast-LIO2 位姿超时：进入 `hold`，并在已进入任务后发布返航请求。

启动：

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
roslaunch mine_uav_control mission_scheduler.launch
```

默认 `rc_switch_channel: 5` 是 ROS 数组下标，对应物理 CH6，不代表一定是你的实际二段开关；应通过 `/mavros/rc/in` 和 QGroundControl 确认后修改配置。

主要输出：

```text
/mine_uav/mission/active_task   std_msgs/UInt8   0=hold, 1=采空区, 2=竖井
/mine_uav/mission/goaf_enable   std_msgs/Bool    任务一是否允许运行
/mine_uav/mission/shaft_enable  std_msgs/Bool    任务二是否允许运行
/mine_uav/mission/return_home   std_msgs/Bool    是否请求当前任务返航
/mine_uav/mission/status        std_msgs/String  状态与健康信息
```

该节点只做任务调度和安全门控，不直接向 PX4 发布 setpoint。两个任务都应遵守：只有收到对应 `*_enable=true` 时才发布自己的任务输出；最终由唯一的 PX4 command router 选择当前任务输出，避免两个任务同时控制飞行器。当前任务一的 `super_exploration_decider` 已订阅 `goaf_enable`；任务二实现后接入 `shaft_enable`。

### SITL 联调

#### 任务一 PX4/Gazebo 完整动态闭环

`task1_px4_sitl.launch` 已将采空区世界、带 MID360 风格三维雷达的 Iris、PX4 SITL、
Gazebo、MAVROS、任务调度器、自主决策器、SUPER 和 PX4 指令桥合并为一条动态闭环。
Gazebo Ray 传感器输出机体系 `/mine_uav/sitl/mid360/points`，注册适配器根据PX4实时姿态
转换为 `camera_init` 下的 `/cloud_registered`；决策器及SUPER沿用真机接口，不再使用
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

当前已使用“左右墙纵向连续覆盖 + 尽头墙横向覆盖 + 接近尽头 + 多周期确认”触发返航，
旧的无 frontier 超时判据默认关闭。本判据验证的是决策器占据体素中的几何连续性，不能
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

本轮完整动态回归已通过：PX4在任务选择后进入OFFBOARD，飞机沿主方向前进到约26 m，
点云注册与累计地图持续更新；决策器确认端墙、左右墙连续覆盖后发布返航目标，飞机返回
入口附近，最终发布 `model_complete=true`、`finished=true`，指令桥状态为
`TASK1_COMPLETE`，PX4切换到 `AUTO.LOITER`。本轮累计地图约12万体素，注册点云保持约
10 Hz，未出现高度围栏故障。该结果证明复杂场景下任务一接口和闭环可运行，但仍不等价于
真实MID360 + Fast-LIO2的建图精度验收。

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
