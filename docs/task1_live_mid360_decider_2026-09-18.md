# MID360s → Fast-LIO2 → 任务一决策输入现场诊断（2026-09-18）

结论：**真实雷达数据已经到达决策器**；**定位绝对精度尚未验收**。本轮在隔离的 `ROS_MASTER_URI=http://localhost:11320` 上运行，未连接 MAVROS、PX4、SUPER 或任何电机指令节点。仅为验证感知门控，视觉桥的 `require_mavros_connection` 临时置为 `false`；真机生产配置仍为 `true`，故本轮不证明 PX4 已融合视觉位姿。

## 可复核的链路证据

- MID360s `192.168.1.157` 可从 NUC `192.168.1.10` 访问；Livox ROS 驱动识别 device type 35。
- `/livox/lidar` 约 10 Hz、`/livox/imu` 约 200 Hz；Fast-LIO2 `/Odometry`、`/cloud_registered` 约 10 Hz，配准点云和位姿 `frame_id=camera_init`。
- 使用真机同名的 `task1_perception_gate` 和 `super_exploration_decider` 节点，决策器订阅 `/mine_uav/task1/cloud_registered` 与 `/mine_uav/task1/odometry`。在隔离 master 上模拟一次任务一允许后，门控状态 `OPEN`，两路输出均约 10 Hz，决策器状态进入 `EXPLORING: active frontier goal published; awaiting odometry progress`，并有模型覆盖诊断（`occupied=1544`、`actionable=2` 的一次样本）。关闭允许后，门控回 `CLOSED_WAIT_TASK1`，决策器回 `DISABLED`。没有目标消费者，也没有飞控指令。
- 一次约 24.901 s 的诊断收集了 250 帧里程计和 250 帧配准点云；位姿首末差 0.0066 m、相对首帧最大 0.0103 m、最大相邻位姿步长 0.0059 m；里程计相邻接收间隔中位 0.1001 s、最大 0.1023 s；点云点数中位 4485、最小 4424；两个话题时间戳在该窗口内严格递增。**仅当雷达在这段时间确实未移动时**，位移数字才可解释为短时静态漂移，而不是绝对精度。

## 已确认的时间配置故障与修正

原有 11312 master 留存 `/use_sim_time=true`，但 `/clock` 没有发布者。原始驱动能产生消息，依赖 ROS 仿真时间的频率与新鲜度判断却会失真；隔离的真实时间 master 上数据立即稳定。现给 `task1_real.launch` 加入必需的 `task1_real_time_guard.py`：若真机入口看到 `/use_sim_time=true`，明确报错并停止该 launch，而不是让任务在冻结时间下继续。守卫在 11312 的错误配置上以退出码 2 拒绝启动，在 11320 真实时间下持续运行；`roslaunch --nodes` 解析通过。**该守卫不替代 PX4、传感器或飞行安全检查。**

## 尚不能宣称定位精度合格

静态位姿小幅变化不等于运动中精度、尺度、航向、Z 轴或长期漂移正确；Fast-LIO2 的内部协方差也不是独立真值。下一个有意义的验收需：固定并记录雷达安装外参与机体几何，使用量尺/全站仪/动捕等独立参照做已知位移与往返轨迹，比较平移、航向和高度误差；再连接 PX4，核对外部视觉融合状态与机体移动方向，并做定位断流/人工接管测试。当前不应仅凭本轮诊断有桨自主飞行。

本轮临时设置 `pcd_save_en=false`，避免 Fast-LIO 默认 `interval=-1` 将所有扫描持续积累成一个 PCD；这仅是诊断参数，未更改 Fast-LIO2 负责人的默认配置。所有隔离测试节点已在结束后停止，原有 11312 master 保持原样。
