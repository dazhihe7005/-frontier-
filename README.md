# mine_uav_control 地面站

这是一个基于 ROS Noetic、MAVROS 和 PyQt5 的 PX4 NUC 轨迹发送器。它只负责生成并
发送轨迹 setpoint；QGC 负责 PX4 的连接、模式、解锁、降落和安全监控。串口上运行
的是 MAVLink，不是向 CH340 直接发送 ASCII 字符。

## 启动真实 PX4 链路

先确认飞控串口设备，例如：

```bash
ls -l /dev/ttyUSB*
```

单 USB 链路默认使用 `/dev/ttyACM0:115200`：

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
roslaunch mine_uav_control real_uav_ground_station.launch
```

指定端口和波特率：

```bash
roslaunch mine_uav_control real_uav_ground_station.launch \
  device:=/dev/ttyUSB0 baud:=115200
```

## 与 QGC 的连接方式

MAVROS 和 QGC 不能同时独占同一个串口设备。单 USB 链路下应让 MAVROS 独占
`/dev/ttyACM0`，QGC 通过 NUC 转发的 UDP MAVLink 链路连接，或暂时关闭 MAVROS
后再让 QGC 直接打开 USB。

启动前确认 `/dev/ttyACM0` 没有被其他串口程序占用。

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

该状态机只决定发布给 SUPER 的观察目标，不直接发布 PX4 setpoint，也不改变 Fast-LIO2 → PX4 的定位链路。它目前仍不是完整的“尽头识别、三面墙完整建模、原路返航”判定器；完成判据仍需后续接入全局覆盖/建模质量信息。

启动：

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
roslaunch mine_uav_control super_exploration_decider.launch
```

它向 SUPER 发布 `geometry_msgs/PoseStamped`，默认目标话题为 `/goal`；它不直接向 PX4 发布控制命令。SUPER 的 `fsm_node` 需要同时运行，并且其配置中的 `fsm.click_goal_topic` 要保持为 `/goal`。

注意：当前 SUPER 的 `FsmRos1::setGoalPosiAndYaw()` 会在 `fsm.click_height > -5` 时强制覆盖目标 z。要让观察点使用点云计算出的三维 z 高度，应在 SUPER 使用的 YAML 中设置 `fsm.click_height: -10.0`。

Fast-LIO2 点云必须已经是与 SUPER 一致的 world/地图坐标系。该节点不做 TF 变换；验证完成后建议把配置中的 `strict_cloud_frame` 改成 `true`。

观察运行状态：

```bash
rostopic echo /mine_uav/exploration/status
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
