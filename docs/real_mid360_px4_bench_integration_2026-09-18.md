# 2026-09-18 拆桨实机 MID360s–NUC–PX4 联调记录

范围：只做未解锁、无桨台架验证。未运行真实任务一/二，未进入 OFFBOARD，未进行有桨飞行。雷达曾由操作者单独移动，不能将此试验当作“雷达已刚性安装到机体”的外参验收。

## 本轮实测

| 环节 | 观测 | 结论 |
| --- | --- | --- |
| MID360s 到 NUC | 雷达 `192.168.1.157` 可达；`/livox/lidar` 约 10 Hz，`/livox/imu` 约 200 Hz | 数据链通 |
| Fast-LIO2 | `/Odometry` 约 10 Hz、`frame_id=camera_init` | 算法有持续位姿 |
| NUC 到 PX4 | CH340 `/dev/ttyUSB0`，500000 波特；MAVROS `connected=true`，视觉桥 `STREAMING`；`/mavros/vision_pose/pose_cov` 约 30 Hz 且被 MAVROS 订阅 | 串口及 ROS/MAVROS 视觉输入通 |
| PX4 水平定位 | 初始 `x/y position control=Fail`、`const_pos_mode=true`；移动雷达后 PX4 水平位置跟随视觉位置，诊断转为 `x/y position control=Ok`，运动时曾见 `const_pos_mode=false` | 有视觉信息进入估计器的强证据；不是完整融合日志验收 |
| 横向动作 | 左右晃雷达期间，PX4 的估计位移投影到其机体前/左轴，范围约 0.037/0.113 m；视觉对应约 0.042/0.111 m。已知向雷达左侧移动的采样末段，PX4 机体左向分量从约 0 增至 +0.060 m | 水平“左右”在当前估计框架内没有明显轴互换；因雷达单独移动、绝对航向也来自视觉，不能反推真实机体安装角或独立机体位移 |
| 高度 | 同时刻视觉桥输出 Z 约 -0.055 m，PX4 `/mavros/local_position/pose` Z 约 +1.028 m；差约 +1.083 m。台架过程中差值变化，不宜当固定常数 | **任务指令 Z 仍未对齐** |
| 安全保持指令 | 临时启动指令桥与 `offboard_bridge`，并把测试节点 `automatic_mode_switch=false`；向桥提供原地目标，观察到 `/mine_uav/setpoint_cmd` 160 帧、`/mavros/setpoint_raw/local` 169 帧，二者最后目标同为约 (0.042,-0.071,0.954) m；PX4 前后均 STABILIZED、未解锁 | 确认 NUC 指令桥→MAVROS 输入输出链；**没有证明 PX4 在 OFFBOARD 执行目标**。之前 `/mavros/setpoint_raw/target_local` 反馈位置为 NaN，不能当成执行证据 |

遥控器初次复连时没有通道帧、`rc receiver=Fail`；后来诊断转为 Ok、`manual_input=true`，飞控模式变为 STABILIZED。用户要求暂不处理遥控器，因此未做 CH5/CH7 验收或任务触发。测试结束后已停掉本轮 ROS 定位栈和临时指令桥，未留下任务 setpoint 进程。ROS master 11312 保留。

## 高度问题的确定因果链及未确定部分

1. 视觉桥配置 `zero_initial_z=true`：把 Fast-LIO2 启动时的 Z 当作视觉零点；这只定义 NUC 输出坐标。
2. PX4 参数现场读取为 `EKF2_EV_CTRL=9`（视觉水平位置和航向，未选视觉 Z）、`EKF2_HGT_REF=2`（配置测距为高度参考）、`EKF2_RNG_CTRL=2`、`EKF2_BARO_CTRL=1`。这说明不能从当前参数推出“PX4 本地 Z 与视觉 Z 同源同零点”；具体时刻实际融合了哪一路高度，还需 PX4 估计器日志确认。
3. `fastlio_px4_vision_bridge.cpp` 发布的 `fastlio_to_px4_alignment` 仅按首次 Fast-LIO2 位姿平移/偏航置零，不读取 PX4 本地高度；`super_px4_command_bridge.cpp::convert()` 则直接以该变换将 SUPER Z 送作 PX4 目标 Z。因此本轮 Z 差会变成真实任务目标高度误差。代码中的 PX4 本地高度围栏 `[-0.5,1.8]` 也不能自动解释为“距本次任务 home 的相对高度”。
4. PX4 本地 Z 为什么在台架上变化、当前测距是否持续有效，尚未取得可证实的传感器融合日志；不能盲目填 +1.08 m 常数或擅改 EKF2 高度参数。

PX4 官方文档分别说明 [视觉融合位掩码与高度参考](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf) 和 [外部视觉坐标/传感器相对机体外参](https://docs.px4.io/main/en/ros/external_position_estimation)。MAVROS ROS 坐标惯例为机体 FLU（前、左、上），PX4 机体惯例为 FRD（前、右、下）；本项目经 MAVROS 转换，但雷达相对真实机体的旋转和平移不会自动标定。

## 目前不能宣称通过的项目

- 雷达需要刚性安装后，整机按已知前、左、上、偏航方向移动，复核视觉与 PX4 本地姿态、速度、方向、时间延迟；单独晃雷达只能验证数据传输/估计响应。
- 明确任务一的高度基准，解决视觉 Z → PX4 本地 Z 的动态对齐，以及地形/测距高度参考变化时的安全策略；之后回归现有 SUPER 高度包络与 PX4 1.8 m 围栏。
- 用 PX4 估计器日志确认视觉水平与航向的实际融合、创新门限与高度实际来源；诊断 `Ok` 和 ROS 话题不替代日志。
- 拆桨验证真实 SUPER 轨迹、OFFBOARD 切入/退出、CH5 人工接管及定位/串口中断。**本轮只是未解锁保持目标的通路探针**。

Fast-LIO2 配置目前启用 `pcd_save_en=true, interval=-1`，本轮停机时在本地 `/home/nuc/fastlio2_ws/src/FAST_LIO/PCD/scans.pcd` 生成约 638 MB 点云文件；未删除、未上传。长时运行需留意磁盘空间。

## 用户确认后的 PX4 高度源调整与静态复测

用户明确选择 Fast-LIO2 的视觉 Z 参与 PX4 EKF，并关闭机载激光测距的 EKF 高度融合。用与 QGC 参数页相同的 MAVLink 参数通道（MAVROS `/mavros/param/set`）操作；写入前现场确认 PX4 `connected=true, armed=false`，并完整拉取 1102 个参数。原值备份与新值如下：

| PX4 参数 | 原值 | 新值 | 作用 |
| --- | ---: | ---: | --- |
| `EKF2_EV_CTRL` | 9 | 11 | 保留视觉水平位置、航向，新增视觉垂直位置位 |
| `EKF2_HGT_REF` | 2 | 3 | 高度参考由测距改为视觉 |
| `EKF2_RNG_CTRL` | 2 | 0 | 关闭测距数据的 EKF 融合；不等于关闭测距硬件 |
| `EKF2_BARO_CTRL` | 1 | 1 | 保留气压计融合，不修改 |

三项 `ParamSet` 均 ACK 成功且逐项 `ParamGet` 回读一致；PX4 在未解锁状态接受 MAVLink 重启命令（246，`param1=1`，结果 0）。随后重新建立 MAVROS 连接，强制从飞控拉取 1101 个参数，重读仍为 `11/3/0/1`。官方 [PX4 外部定位配置](https://docs.px4.io/main/en/ros/external_position_estimation) 指明此类设置需重启，以上验证属于参数已写入、重启命令已接受且复连后持久回读，不以 QGC 界面显示替代读回。

用 `roslaunch mine_uav_control px4_fastlio_localization.launch rviz:=false save_pcd:=false` 再启雷达/FAST-LIO2/MAVROS/视觉桥；`/pcd_save/pcd_save_en=false` 已从实际 ROS 参数服务器读回，以免覆盖此前 638 MB PCD。重启后静止 25 s 的约 750 帧视觉与 PX4 本地位姿按采样时刻配对，Z 差 `-0.002～+0.004 m`，水平差也在厘米级。第二段静止 25 s 视觉 Z 仅变化约 0.005 m，PX4 Z 变化约 0.007 m，最大瞬时 Z 差约 0.005 m。**原先约 1 m 的高度零点偏差在该台架条件下消失**，但这两段没有包含足够上下位移，尚不能宣称动态高度融合、外参、OFFBOARD 或真实飞行通过。机载测距仍可作为任务二的独立“见底”信号使用，关闭的只是当前 PX4 EKF 融合。

需要注意，任务二的原需求是“PX4 Z 不可靠时下井”；任务一把视觉 Z 加入 EKF 并不能自动解决任务二的独立深度与失效返航问题。此参数组是当前任务一的台架配置，任务二生产开关继续关闭。回滚原 PX4 配置时，应在未解锁状态恢复 `EKF2_EV_CTRL=9`、`EKF2_HGT_REF=2`、`EKF2_RNG_CTRL=2` 并重启；回滚后原来的视觉/PX4 高度零点差预计会重新出现。
