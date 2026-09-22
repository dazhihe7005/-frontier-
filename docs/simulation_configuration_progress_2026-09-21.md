# 无人机统一仿真配置：需求与进度存档

存档时间：2026-09-21，最近更新：2026-09-23（Asia/Shanghai）

项目目录：`/home/nuc/frontier-upload`

运行镜像工作区：`/home/nuc/task2_isolated_ws/src/mine_uav_control`

## 1. 用户最终需求

### 1.1 真实无人机统一参数

以下参数应作为 Task1、Task2 以及以后新增无人机仿真的统一基础配置，不能只在某一个场景中生效：

- 四旋翼总质量：6 kg；
- 机架此前口径为 800 mm；用户已明确尺寸以完整 STEP 总装为准。STEP
  中四个 U7 中心相邻间距约 439.002 mm、对角轴距约 620.843 mm，800 mm
  仅保留为机架等级/整体尺寸旧口径，不再驱动力矩臂；
- 定位传感器：Livox MID360S；
- 无 GPS，水平定位必须来自 MID360 模拟点云经过真实 FAST-LIO2 算法得到的里程计；
- 电池：两块 DJI Matrice 4D/4TD `BPX230-6768-22.14` Li-ion 6S 电池并联；单块 6768 mAh、22.14 V、149.9 Wh、约 640 g、154×96×59 mm、4C，并联后仍为 6S，总容量 13536 mAh、标称总能量 299.8 Wh、总质量约 1.28 kg；
- 电调：LANRC 60A 四合一、BLHeli_S、DSHOT600、2–6S；
- 桨叶：P15×5 英寸碳纤维桨；
- 电机：T-Motor U7，四台；
- 电机 KV：420；
- 槽极：12N14P；
- 相电阻：33 mΩ；
- 外形：Φ60.7×39.5 mm；
- 轴径：6 mm；
- 电机线：750 mm、16#；
- 单电机质量：255 g（不含线）、296 g（含线）；仿真采用 296 g；
- 空载电流：0.9 A；
- 支持电池：3–8S；
- 最大电流：40 A；
- 最大持续功率：350 W；
- 用户提供的单电机最大拉力：4.64 kgf。

### 1.2 PX4 与控制参数

- 真机 PX4 接入后，只读导出完整飞控参数，包括位置、速度、姿态、角速度 PID、限幅、滤波、估计器和其他参数；
- 保存未经修改的真机原始参数快照；
- 生成 SITL 专用迁移配置；
- 可迁移的飞行控制参数同步到 SITL；
- 串口、传感器校准、硬件 ID、板载总线、驱动等硬件专属参数不能直接灌入 SITL；
- 不向真机写参数，除非用户以后单独明确授权；
- 任务节点只输出轨迹/速度意图，闭环 PID 由 PX4 完成，不能把任务逻辑误称为电机 PID。

### 1.3 Task2 竖井环境

- 圆形竖井；
- 平均直径：5 m；
- 深度：500 m；
- 井壁应为矿坑岩壁：灰色、不规则、具有真实几何起伏；
- 起伏必须参与视觉、激光射线和碰撞，不能只贴纹理；
- MID360 模拟点云 → FAST-LIO2 → `/Odometry` → PX4 外部视觉 XY，应形成实际运行链路；
- 不允许默认使用 Gazebo 位姿真值冒充 FAST-LIO2。

### 1.4 Task1 采空区环境

- Task1 场景也要使用灰色、粗糙、不规则起伏的岩壁；
- 采空区整体大多为近似半椭圆形；
- 用户提供的长、宽、高只代表平均尺寸，不应生成完全规则的长方体或标准椭球；
- 场景应具有确定性的不规则变化，便于回归测试，同时保持激光和碰撞几何一致。

### 1.5 3D 模型导入

已读取用户提供的完整 STEP 总装 `/home/nuc/【二代】总装.STEP`，并以 STEP
尺寸而不是早期 800 mm 口径建立定制机体。Gazebo 使用从总装提取的外部
视觉子集，并保留独立的简化碰撞体、动态桨盘和传感器链接；这样既保持
外形与实物一致，也避免把高面数 CAD 直接用于实时碰撞。

## 2. 已完成工作

### 2.1 统一 6 kg 无 GPS 机体模型

新增统一基础模型：

- `models/mine_uav_800_no_gps/model.sdf`
- `models/mine_uav_800_no_gps/model.config`
- `models/mine_uav_800_no_gps/README.md`

当前质量分配：

- 中央机体、电池、电调及其余部件合并质量：4.536 kg；
- 四个 U7 电机/转子链接：4 × 0.296 kg；
- IMU：0.015 kg；
- MID360S：0.265 kg；
- 合计：6.000 kg。

几何配置：

- STEP 实测对角电机轴距：约 0.620843 m；
- STEP 实测相邻电机间距：约 0.439002 m；
- FLU 电机中心坐标：`±0.219501 m`；
- 15 英寸桨直径：0.381 m，半径：0.1905 m；
- 以圆形桨盘保守计算，机体中心到桨尖的最大径向包络约 0.500921 m；
- 位于平均半径 2.5 m 的竖井中心时，理论径向净空约 1.999079 m。

以下 MID360 包装模型已改为引用统一基础机体，因而 Task1 和 Task2 不再使用原始 1.58 kg Iris：

- `models/iris_mid360/iris_mid360.sdf`
- `models/iris_mid360_long_range/iris_mid360_long_range.sdf`
- `models/iris_mid360_shaft_8m/iris_mid360_shaft_8m.sdf`

模型中没有 `model://gps` 传感器。

完整 STEP AP203 总装已解析。视觉模型使用其中机架、中心板、电池/支架
和起落结构的 101750 三角形外部子集；雷达视场辅助实体、内部细节、U7、
桨叶和 MID360S 本体不重复写入机体网格，后三者分别由动态转子链接和
MID360S 包装链接表示。视觉网格不用于碰撞。STEP 原生轴映射到 FLU 为
`[X,-Z,Y]`，机体网格和简化碰撞/电机力臂使用相同尺寸基准。

### 2.2 当前电机气动近似

在缺少 U7 420KV + P15×5 的实测推力/RPM/电流完整表时，Gazebo 使用以下临时近似：

- 6S 标称电压：22.14 V；
- 空载转速上限：`420 × 22.14 = 9298.8 rpm`；
- 最大角速度：973.768 rad/s；
- 单电机最大推力锚点：4.64 kgf；
- `motorConstant = 4.79874427e-05 N/(rad/s)^2`；
- `momentConstant = 0.00789903182 m`；
- 四电机理论总最大拉力：18.56 kgf；
- 6 kg 机体理论最大推重比：约 3.09；
- 理想悬停转速：约 5287 rpm。

注意：Gazebo Classic 电机插件使用气动系数，不会真实求解 BLHeli_S、DSHOT600、电池压降、电机温升、电调电流或 350 W/40 A 保护。40 A × 22.14 V 与 350 W 持续功率也不是同一工作点，因此在得到实测台架曲线前，不能把当前功率模型视为最终实物标定。

### 2.3 6S 和无 GPS PX4 SITL 配置

已修改项目自定义 PX4 空机架：

- `px4_airframes/1018_gazebo-classic_iris_xy_vision`

已设置：

- `BAT1_N_CELLS = 6`；
- `BAT1_CAPACITY = 13536`（两块 6768 mAh 并联）；
- `BAT1_V_CHARGED = 4.25`（总满充电压 25.5 V）；
- `EKF2_GPS_CTRL = 0`；
- `EKF2_EV_CTRL = 1`，只融合外部视觉水平位置；
- 垂直参考暂时保留气压计。

之前在 SITL 中实际读取并确认：`BAT1_N_CELLS=6`、`EKF2_GPS_CTRL=0`、`EKF2_EV_CTRL=1`。

### 2.4 PX4 SITL 控制参数

在上一轮稳定飞行中从 PX4 SITL 实际读取到的参数为：

| 参数 | 值 |
|---|---:|
| MPC_XY_P | 0.95 |
| MPC_Z_P | 1.0 |
| MPC_XY_VEL_P_ACC | 1.8 |
| MPC_XY_VEL_I_ACC | 0.4 |
| MPC_XY_VEL_D_ACC | 0.2 |
| MPC_Z_VEL_P_ACC | 4.0 |
| MPC_Z_VEL_I_ACC | 2.0 |
| MPC_Z_VEL_D_ACC | 0.0 |
| MC_ROLL_P | 6.5 |
| MC_PITCH_P | 6.5 |
| MC_YAW_P | 2.8 |
| MC_ROLLRATE_P/I/D | 0.15 / 0.20 / 0.003 |
| MC_PITCHRATE_P/I/D | 0.15 / 0.20 / 0.003 |
| MC_YAWRATE_P/I/D | 0.20 / 0.10 / 0.0 |
| MPC_THR_HOVER | 0.5 |
| MPC_THR_MIN / MAX | 0.12 / 1.0 |
| THR_MDL_FAC | 0.0 |

这些是上一轮 SITL 基线。2026-09-21 已另外从真机只读导出 1102 个
参数，并筛选出 82 个控制器、限幅和滤波候选；其中 76 个与当前 SITL
参数元数据同名兼容，已写入专用 SITL 空机架。6 个当前 SITL 不支持的
新版本参数未加载。硬件校准、设备 ID、串口/MAVLink、PWM/DSHOT、RC、
驱动/总线、真机控制分配几何和临时测试电池值均未迁移。

冷启动后实际读回确认：

- `BAT1_N_CELLS=6`、`BAT1_CAPACITY=13536`、`BAT1_V_CHARGED=4.25`；
- `MC_ROLL_P=4.0`、`MC_PITCH_P=4.0`、`MC_YAW_P=2.8`；
- `MPC_XY_P=0.9`、`MPC_XY_VEL_I_ACC=0.33`、`MPC_Z_VEL_P_ACC=4.1`；
- `IMU_GYRO_CUTOFF=40`、`THR_MDL_FAC=0`；
- `EKF2_GPS_CTRL=0`、`EKF2_EV_CTRL=1`、`EKF2_HGT_REF=0`。

本次冷启动只验证加载与连接，未解锁；这组真实控制参数在当前近似气动
模型上的飞行稳定性仍需 FAST-LIO2 闭环验收。

### 2.5 Task2 不规则岩壁网格

新增：

- `models/shaft_rock_500m/model.sdf`
- `models/shaft_rock_500m/model.config`
- `models/shaft_rock_500m/meshes/shaft_rock_inner.obj`
- `scripts/generate_rock_shaft_mesh.py`

网格特点：

- 平均内半径 2.50 m；
- 从入口 `z=0.25 m` 延伸到中心底部 `z=-499.75 m`，参考深度为 500 m；
- 圆周 32 段、竖向 1 m 一层；
- 使用多尺度确定性起伏，半径约在 2.36–2.64 m 范围内变化；
- 井底也有小尺度起伏；
- 约 3.2 万个三角形，OBJ 文件约 1.1 MB；
- 视觉和碰撞引用同一个网格；
- 灰色、低高光材质；
- `worlds/shaft_500m_logic_sitl.world` 已改为引用该岩壁模型。

### 2.6 FAST-LIO2 默认链路改造

新增或修改：

- `config/fastlio_gazebo_mid360.yaml`
- `src/pointcloud1_to_pointcloud2.cpp`
- `scripts/fastlio_shaft_depth.py`
- `launch/task2_shaft_500m_opticalflow_px4_sitl.launch`
- `scripts/sitl_shaft_sensor_adapter.py`

目标默认链路：

```text
Gazebo MID360 ray sensor (PointCloud, 10 Hz)
  -> pointcloud1_to_pointcloud2
  -> PointCloud2
  -> FAST-LIO2 laserMapping + /mavros/imu/data_raw
  -> /Odometry
  -> fastlio_px4_vision_bridge
  -> /mavros/vision_pose/pose_cov
  -> PX4 EKF2 external-vision XY
```

同时，任务相对深度默认改为 FAST-LIO2 `/Odometry` 的相对 Z，来源标识为 `fastlio2_vertical_displacement`。Gazebo 真值定位只保留为显式 A/B 参数 `use_vision_truth:=true`，默认值为 `false`。

已确认：

- Gazebo 原始 MID360 话题约 10 Hz；
- 模拟 PX4 IMU 约 50 Hz；
- 原始激光插件实际输出 `sensor_msgs/PointCloud`；
- 新转换话题输出 `sensor_msgs/PointCloud2`，约 10 Hz；
- FAST-LIO2 使用其 `lidar_type=4` 标准 PointCloud2 仿真入口；
- 统一 FLU 机体系为 `+X` 向前、`+Y` 向左、`+Z` 向上；MID360S
  中心位姿为 `[0.1315, 0, 0.223] m`，绕 `+Y` 正向俯 25°，即前向轴
  朝 `-Z`；相对位于 `[0,0,0.02] m` 的模拟 IMU，FAST-LIO2 使用
  `extrinsic_T=[0.1315,0,0.203] m` 及对应 25° 旋转矩阵。

### 2.7 Task1 确定性不规则采空区

已新增 `models/goaf_rock_24x8x4/`，其平均尺寸为 24×8×4 m，截面是带
确定性扰动的半椭圆而不是规则长方体或标准椭球。入口位于 `x=-2 m`，
末端位于 `x=22 m`；视觉、MID360 射线和碰撞引用同一 OBJ。世界
`worlds/goaf_mine.world` 已改为引用该模型，网格可由
`scripts/generate_irregular_goaf_mesh.py` 重复生成。

### 2.8 白象山实测巷道地图与自主探索

已把用户提供的 `白象山巷道_展示精修版_管线修订.blend` 转换为 Gazebo
模型 `models/baixianshan_tunnel/`。原始 Blender 文件保持不变且未复制进
Git；导出的岩壁、地面、顶板和基础设施共约 50 万至 150 万三角形，场景
原始范围约 89.4×293.8×19.4 m。四类 OBJ 分别同时承担视觉、MID360
射线和碰撞，纹理由 `.blend` 内嵌资源导出。

新增：

- `worlds/baixianshan_tunnel_sitl.world`；
- `launch/task1_baixiangshan_px4_sitl.launch`；
- `scripts/export_blend_gazebo_map.py` 及场景检查/净空采样脚本；
- `test/test_baixiangshan_map.py`。

该入口继续使用定制 1018 无 GPS 空机架、6 kg STEP 机体、MID360S、
FAST-LIO2 和 PX4 外部视觉，而不是 Gazebo 位姿真值。地图固定任务航向为
Gazebo `yaw=pi`，横向任务边界为 8 m。探索决策器增加一次死路航向反转
和出航面包屑回退：确认无可通过的侧向绕行或有效前沿后，沿已记录路径
返回任务原点，再清除旧路径并向相反方向继续探索。

## 3. 已完成验证

- `test/test_task2_vehicle_model.py`：9 项通过；
- `test/test_task2_range_censoring.py`：5 项通过；
- `test/test_gazebo_geometry_filter.py`：3 项通过；
- `test/test_baixiangshan_map.py`：3 项通过；
- 新增模型 XML 和 launch XML 已通过语法解析；
- 不规则岩壁网格可以被 Gazebo 世界加载；
- 新增 PointCloud → PointCloud2 C++ 转换节点已成功编译；
- 运行工作区使用 `CATKIN_ENABLE_TESTING=OFF` 编译成功。关闭测试是因为隔离运行镜像没有复制全部旧测试源文件，并非产品源代码编译失败；
- 旧的 Gazebo 真值定位链路下，6 kg 机型曾稳定下降：10 秒采样中从 -80.78 m 到 -85.77 m，平均垂直速度 -0.5011 m/s，最大水平偏移 0.0654 m；
- 白象山场景中已动态确认 PX4 `connected/armed/OFFBOARD`、外部视觉
  `STREAMING`、自主前飞、死路航向约 180° 反转以及约 8 m 的首段回退；
- 最终面包屑持续回到任务原点的代码已编译并完成配置回归，但因局部点云
  净空与规划器接受结果存在运行间波动，尚未取得完整的动态闭环验收证据。

## 4. 暂停时尚未完成或尚未验证的事项

### 4.1 真机 PX4 参数已只读导出，安全迁移已生成

已确认：

- 串口：`/dev/ttyUSB0`；
- USB 芯片：QinHeng/CH340，VID:PID `1a86:7523`；
- 稳定设备链接：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`；
- TELEM 波特率：500000；
- MAVROS 已收到 PX4 心跳，导出时飞控未解锁且处于 `ALTCTL`；
- 只读收到 1102 个参数并保存为
  `params/real_snapshots/px4_real_2026-09-21_qgc.params`；
- 没有调用参数写入服务，没有修改真机参数；
- 真机导出的电池值属于临时测试电池，不用于仿真；仿真以用户提供的
  两块 BPX230 6S、单块 6768 mAh 数据为准；
- 已完成自动分类和当前 PX4 SITL 元数据兼容检查：82 个控制/限幅/滤波
  候选中迁移 76 个，6 个当前 SITL 不支持的参数未加载；排除报告和迁移
  说明保存在 `params/sitl/`。仍禁止把 SITL 参数反向写入真机。

### 4.2 FAST-LIO2 闭环尚未完成全部场景最终飞行验收

最初 FAST-LIO2 直接订阅 Gazebo 点云时发现消息类型不匹配，随后已加入
C++ 转换节点并验证转换话题为 PointCloud2。白象山 Task1 已得到外部视觉
流、解锁和 OFFBOARD 的动态证据；Task2 500 m 竖井以及白象山全程探索
仍需分别完成最终验收：

- 记录 `/Odometry` 的长时间频率、漂移和协方差；
- 使用 FAST-LIO2 而非真值定位完成竖井起飞和至少 10 秒稳定下降；
- 在白象山地图完成死路面包屑回到任务原点并继续反向探索；
- FAST-LIO2 轨迹与 Gazebo 真值仅用于离线误差评估，不能反馈到控制链路。

### 4.3 Task1 岩壁已生成，仍需完整飞行验收

`worlds/goaf_mine.world` 已引用确定性不规则半椭圆岩壁，白象山实测模型也
已形成独立 Task1 入口。剩余工作是分别采集长时间 FAST-LIO2 闭环、规划
成功率、碰撞净空和任务完成条件的动态证据。

### 4.4 项目级“所有未来仿真”统一入口尚需审计完成

三个主要 MID360 模型包装器以及 Task1/Task2 主入口已经统一引用新基础
机体。仍有旧的真值验证入口和通用调度入口保留 `iris`/`iris_vision` 名称；
这些名称可能只是 PX4 实例/空机架选择，也可能是遗留仿真入口，提交后仍
需逐一分类，防止以后绕过 canonical SDF。基础模型当前仍借用 PX4 Iris 的
桨叶视觉网格，但质量、惯量、力臂、气动参数和碰撞体均由定制模型定义。

### 4.5 电池续航模型仍缺少数据

已知电池为 DJI `BPX230-6768-22.14` Li-ion 6S，单块 6768 mAh、149.9 Wh、640 g、4C；仿真已设置并联总容量 13536 mAh、满充单节电压 4.25 V。仍缺内阻和荷电状态—电压曲线，因此仍不能可信模拟：

- 电压随荷电状态变化；
- 并联电池容量；
- 大电流压降；
- 续航时间；
- 低电压保护。

## 5. 本轮提交状态

- 2026-09-23 正在把本存档所述修改整理为一个 Git 提交并推送远端；
- 未执行 `git reset`、`git checkout` 或其他破坏性命令；
- 项目原始任务算法逻辑没有被替换；
- 修改集中在仿真模型、世界、SITL 空机架、传感器适配、可视化、分析及测试；
- 同步副本已写入 `/home/nuc/task2_isolated_ws/src/mine_uav_control`；
- 当前 Gazebo/PX4 SITL/FAST-LIO2、RViz 及真机 MAVROS 连接均已停止；
- 真机 PX4 未被写入任何参数。

当前 Git 变更涉及：

- `CMakeLists.txt`
- `package.xml`
- `launch/task1_px4_sitl.launch`
- `launch/task1_baixiangshan_px4_sitl.launch`
- `launch/task2_shaft_22m_px4_sitl.launch`
- `launch/task2_shaft_500m_opticalflow_px4_sitl.launch`
- 三个 MID360 SDF 包装模型
- `models/mine_uav_800_no_gps/`
- `models/shaft_rock_500m/`
- `models/goaf_rock_24x8x4/`
- `models/baixianshan_tunnel/`（不含原始 `.blend`）；
- `px4_airframes/1018_gazebo-classic_iris_xy_vision`
- `config/fastlio_gazebo_mid360.yaml`
- `src/pointcloud1_to_pointcloud2.cpp`
- `scripts/fastlio_shaft_depth.py`
- `scripts/generate_rock_shaft_mesh.py`
- `scripts/sitl_shaft_sensor_adapter.py`
- 白象山转换/检查脚本、Task1/Task2 世界、RViz、分析脚本和测试。

## 6. 建议恢复顺序

1. 已确认 `/dev/ttyUSB0` 对应 PX4 TELEM 端口为 500000 波特并收到 MAVROS 心跳；
2. 已只读拉取 1102 个参数并保存 QGroundControl 格式原始快照；
3. 已完成安全筛选，并在 `params/sitl/README.md` 记录迁移与排除策略；
4. 已把 76 个兼容的真机 PID/限幅/滤波参数加载到项目自定义 SITL 空机架并完成冷启动读回；
5. 已生成 Task1 不规则灰色半椭圆采空区，并导入白象山实测巷道；
6. 已在白象山动态观察外部视觉闭环、自主前飞、死路反转和首段回退；
7. 完成白象山面包屑回到任务原点并继续反向探索的连续动态验收；
8. 重启 Task2，验证 PointCloud2 → FAST-LIO2 → PX4 外部视觉下降闭环；
9. 完成遗留 `iris`/`iris_vision` 入口分类审计；
10. 补充长时间稳定性、定位漂移和碰撞净空统计。

## 7. 仍需用户后续提供的信息

- DJI BPX230 电池内阻和荷电状态—电压曲线；
- U7 420KV + P15×5 + 6S 的实测推力/RPM/电流/功率表；
- 更准确的整机总重、重心和三轴转动惯量实测值（当前总质量按 6 kg，重心
  近似机体中心，惯量按 CAD 外形和部件质量近似）；
- MID360S 与机体 IMU 的实测六自由度外参，用于替换当前安装尺寸推导值。
