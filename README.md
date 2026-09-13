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

## 项目文档

- [对话记录](docs/mine_uav_project_conversation.md)
- [SUPER 源码与原理技术栈导读](docs/SUPER源码与原理技术栈导读.md)
