# 下一次五米任务实验方案（2026-10-15）

本轮完整代码变更、文件清单以及以后迁移到正式任务一前必须完成的 P0 项，见
`docs/five_meter_to_task1_change_ledger_2026-10-14.md`。

## 实验目的

本次实验验证 2026-10-14 两次异常对应的修复。地面检查和带障碍 SITL 通过后，
真机直接在离地约 `1.0 m` 高度执行完整 5 m 带障碍飞行，不安排 1 m、3 m 过渡任务：

1. 正常跟踪时，飞机不得因为接近 `0.8 m/s` 的硬保护线而退出任务；
2. 第二次启动不得复用上一次目标，也不得在进入 `OFFBOARD` 时产生约 6 m 的目标跳变；
3. 一旦出现真实超速、不可治理的跟踪误差或其它桥接器故障，五米任务必须立即进入
   `ABORTED`，撤销 `auto_enable` 和 `goaf_enable`，不能继续等到任务超时；
4. CH5 人工接管和 CH7 取消功能保持有效。

本次使用的速度层级为：SUPER 名义速度 `0.60 m/s`，预测 PX4 合成速度上限
`0.70 m/s`，规划速度前馈硬上限和实测三维速度硬上限均为 `0.80 m/s`。
进入 OFFBOARD 前的悬停稳定门限为 `0.25 m/s`，它不是任务规划速度。

避障包络使用水平半径 `0.8 m`、垂直半径 `0.5 m`。连续安全走廊采用 `0.5 m`
基础球加 `0.3 m` 仅水平方向余量；A* 地图使用 `0.9 m` 前端搜索膨胀，以覆盖一个
`0.1 m` 栅格单元的离散误差；优化后仍按原始障碍点复核最终水平 `0.8 m`、垂直
`0.5 m` 的硬约束。

## 已加入的防护

- 每次任务权限从关到开时都会清空上一次的轨迹、时间戳、预发送目标和重捕获状态；
- SUPER 接受本次新目标以前，只开放感知/规划选择，不开放自动飞行权限；
- `OFFBOARD` 预发送只保持飞机当前实测位置，不再发送上一任务的最后目标；
- 请求进入 `OFFBOARD` 前，要求本地位姿和速度新鲜、实测速度不超过
  `0.25 m/s`、新命令与当前位置的误差不超过 `0.65 m`；否则本次任务直接报错；
- 桥接器通过锁存话题 `/mine_uav/task1/command_fault` 上报首个故障，任务协调器收到后
  立即取消 SUPER 目标并进入 `ABORTED`；
- 安全退出到 `POSCTL` 不再受刚发出的 `OFFBOARD` 请求的一秒节流影响；
- 正常预测速度限制设为 `0.70 m/s`，与 `0.80 m/s` 实测硬保护之间保留
  `0.10 m/s` 动态余量。

## 放飞前门槛

以下任一项不通过，不进行真机带桨实验。

```bash
cd /home/nuc/frontier_runtime_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg mine_uav_control -DCATKIN_ENABLE_TESTING=ON

export CMAKE_PREFIX_PATH=/home/nuc/frontier_runtime_ws/devel:/home/nuc/super_ws/devel:/opt/ros/noetic
export ROS_PACKAGE_PATH=/home/nuc/frontier_runtime_ws/src:/opt/ros/noetic/share
export PYTHONPATH=/home/nuc/frontier_runtime_ws/devel/lib/python3/dist-packages:/home/nuc/super_ws/devel/lib/python3/dist-packages:$PYTHONPATH
export LD_LIBRARY_PATH=/home/nuc/frontier_runtime_ws/devel/lib:/home/nuc/super_ws/devel/lib:$LD_LIBRARY_PATH
rostest mine_uav_control super_bridge_manual_override.test
rostest mine_uav_control five_meter_command_fault_abort.test
```

必须确认：

- 旧目标注入测试产生 `INITIAL_COMMAND_POSITION_JUMP` 或
  `PRESTREAM_TRACKING_ERROR`，没有请求 `OFFBOARD`；
- 实测超速注入产生 `MEASURED_SPEED_LIMIT`，并及时请求 `POSCTL`；
- 任务协调器连续启动两个任务时，收到上述故障后都进入 `ABORTED`，两个任务权限均
  变为 `false`；
- 编译和两组 ROS 测试全部通过。

水平 `1.0 m`、垂直 `0.6 m` 的最终包络已在同一中央障碍场景以
`1.0 m` 起飞高度完成验证：

```bash
/home/nuc/frontier-upload/scripts/start_five_meter_avoidance_sitl.sh \
  gui:=false rviz:=false
```

最终包络绕过中央障碍并完成 `GOAL_REACHED_AND_SETTLED`；任务阶段实测三维速度
峰值 `0.237 m/s`，PX4 三维速度设定值峰值 `0.242 m/s`，未触发超速退出。任务中
下视测距约 `1.04–1.26 m`，起点和目标高度均约 `1.0 m`，中间为规划器绕障产生的
有界上浮。无实体遥控器的 SITL 不满足 PX4 的 Position 手动输入条件，因此任务结束后
桥接器持续请求 `POSCTL` 但 PX4 拒绝切换；这不作为真机接管结论，也不据此改变生产逻辑。
不重启软件的第二任务生命周期由上面的 ROS 回归覆盖，并在下面的真机第二次 CH7
任务中再次验证。

## 真机实验顺序

本次直接布置 5 m 带障碍场景：障碍放在航线前方约 2.5 m，使用雷达可识别的宽、轻质
障碍物，两侧各保留至少 3 m 绕行空间。飞机在离地 `1.0 m` 左右稳定悬停；高度明显
超过 `1.0 m` 时不启动任务。飞手全程握住 CH5 人工接管开关，CH7 初始必须在低位。

保持默认 5 m 目标，不设置 `MINE_UAV_GOAL_DISTANCE`。需要人工启动记录链时使用：

```bash
sudo systemctl stop mine-uav-five-meter.service
/home/nuc/frontier-upload/scripts/start_five_meter_avoidance_real.sh
```

人工起飞到离地约 `1.0 m` 并稳定悬停，再将 CH7 从低拨高，完成第一次 5 m 带障碍任务。
任务目标会保持拨动 CH7 时捕获的当前高度，不会主动爬升到 `1.5 m`。第一次完成且飞机
重新稳定后，把 CH7 放回低位再拨高，在不重启软件的情况下执行第二次 5 m 带障碍任务，
专门验证上一任务目标不会被复用。第一次出现任何异常时，不执行第二次。

## 每次任务的通过标准

- `SUPER accepted ... goal` 出现以前，`auto_enable` 必须保持 `false`，PX4 不得进入
  `OFFBOARD`；
- 进入 `OFFBOARD` 前实测三维速度不超过 `0.25 m/s`；
- 预发送目标与当时实测位置一致，不得出现指向上一次终点的跳变；建议日志中该误差
  不超过 `0.15 m`，绝对拒绝线为 `0.65 m`；
- PX4 预测三维速度请求不得超过治理值 `0.70 m/s`；实测速度达到 `0.75 m/s`
  应作为检查点，且不得超过 `0.80 m/s` 持续 `0.10 s`；
- 正常架次不得出现 `MEASURED_SPEED_LIMIT`、`SPEED_GOVERNOR_INFEASIBLE`、
  `PRESTREAM_TRACKING_ERROR` 或 `OUTPUT_DISCONTINUITY_REJECTED`；
- 正常完成后状态为 `COMPLETE:GOAL_REACHED_AND_SETTLED`；
- 如果故意或意外触发桥接器故障，任务状态必须变为
  `ABORTED:COMMAND_BRIDGE_FAULT_<原因>`，并撤销两个任务权限；
- 第二次任务必须与第一次满足完全相同的标准。

## 立即中止条件

出现下列任一情况，飞手立即使用 CH5 接管，不再重复试飞：

- 目标或位置设定值突然跳向上一次终点；
- 实测三维速度达到或越过 `0.80 m/s`；
- 明显快速运动时任务状态仍保持 `ACTIVE`，或桥接器已经报错但任务没有变为
  `ABORTED`；
- 位置/速度设定值、Fast-LIO 位姿、PX4 本地位姿任一出现跳变或坐标方向不一致；
- CH7 取消、CH5 人工接管、`POSCTL` 回退任一不生效。

中止后不要现场提高速度门限，也不要立即重复任务。保留完整 ROS bag 与 PX4 ULog，
先按同一时间轴检查规划速度、目标位置误差、PX4 内部速度设定值、实测速度、模式和
`/mine_uav/task1/command_fault`。

## 记录与复盘

启动脚本会为每次软件运行创建新的
`/home/nuc/flight_logs/five_meter/YYYYMMDD_HHMMSS/`，并在元数据中记录本次
`goal_distance`。复盘至少读取：

- `/mine_uav/five_meter_test/status`
- `/mine_uav/task1/command_status`
- `/mine_uav/task1/command_fault`
- `/mine_uav/mission/auto_enable`
- `/mine_uav/mission/goaf_enable`
- `/mavros/state`
- `/mavros/local_position/pose`
- `/mavros/local_position/velocity_local`
- 桥接层输出的位置/速度目标与 SUPER 原始轨迹命令
- PX4 ULog 中的 `vehicle_local_position`、`vehicle_local_position_setpoint` 和模式事件

本次通过条件是：地面检查和带障碍 SITL 通过，并且真机在离地约 `1.0 m` 高度完成两次
5 m 带障碍任务并满足上述标准。第一次未通过时立即结束实验，不执行第二次。
