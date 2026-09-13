# 矿井探测无人机项目对话记录

> 用途：记录本项目的每轮对话、关键判断、已达成共识和待办事项。
> 后续每轮对话都在本文档末尾追加，不覆盖历史内容。

## 项目背景（用户首轮描述）

项目是矿井探测无人机，当前期望的系统架构为：

- NUC 上层接收 Fast-LIO2 输出的位姿，并作为 PX4 的定位来源。
- 下方并存两个可切换任务：
  - 任务一：自主决策器加 SUPER，完成采空区自主探测和建模。
  - 任务二：当 PX4 没有可靠的 Z 方向定位来源时，进行四百多米竖井探测，保持 Z 轴匀速；测距模块测到地面或底部后开始返航。
- 最下层提供通用接口，把上述任务输出转换为 PX4 坐标系下的控制输入。
- 用户只负责 NUC 机载计算机部分，Fast-LIO2 本身不在负责范围内。

用户询问：该架构是否合理，并要求生成 Markdown 文档，后续每轮对话把重点记录在其中。

## 第 1 轮：架构合理性初步分析

### 初步判断

总体分层方向合理，但需要补充一个“任务管理/控制仲裁与安全监督层”。两个任务可以并存运行，但任何时刻只能有一个任务拥有 PX4 控制权，不能让两个任务同时直接发布控制指令。

建议的数据流为：

```text
Fast-LIO2（外部输入）
        ↓
定位适配与健康监测
        ↓
PX4 状态估计

任务一 ─┐
任务二 ─┼→ 任务管理器/控制仲裁器 → 坐标与控制接口适配 → PX4
安全监督 ┘
```

### 需要明确的关键点

1. Fast-LIO2 位姿输入和任务控制输出是两条不同的数据流，不能都抽象成“位姿转换”。
2. 任务模块最好输出统一的轨迹参考量，例如位置、速度、加速度、偏航或偏航角速度，并带有坐标系、时间戳、有效期和控制模式。
3. 任务二在没有可靠 Z 位置时，应优先设计成 Z 方向速度控制；但这仍要求 PX4 至少有可用的垂直速度估计。若位置和速度估计都不可用，则不能直接假定普通 Offboard 位置/速度控制仍然安全。
4. 竖井返航不能只依赖 PX4 RTL。没有可靠位置来源时，应由任务二保存入口参考状态，并显式执行上升返航；同时定义水平漂移、超时、电量、通信中断和传感器失效等保护条件。
5. 测距模块触发“到底/地面”应采用有效性检查、连续采样确认、滞回和超时机制，不能由单个异常测距值直接触发返航。
6. 任务切换应通过状态机完成：停止当前任务输出 → 控制权交接 → 检查新任务前置条件 → 平滑接管；没有可用任务时进入保持或预设安全动作。

### 当前工作范围共识

- 用户负责 NUC 机载计算机侧的系统集成、任务调度、控制接口和 PX4 通信。
- Fast-LIO2 作为外部定位输入处理，暂不展开其内部实现；后续只需约定输入消息、坐标系、时间戳、频率、协方差和健康状态等接口边界。
- 本 Markdown 的主要用途是保存项目对话内容，而不是只保存一份静态架构说明。

### 参考依据

- PX4 外部视觉/里程计输入说明：<https://docs.px4.io/main/en/ros/external_position_estimation>
- PX4 Offboard 模式及失联保护说明：<https://docs.px4.io/main/en/flight_modes/offboard>

## 待继续讨论

- 任务管理器、控制仲裁器和安全监督器是否合并为一个节点，还是拆成多个 ROS 节点。
- 任务一与 SUPER 的具体输入输出接口。
- 任务二的垂直速度控制方式，以及 PX4 在无可靠 Z 位置时实际可用的估计量。
- 入口点/返航参考的定义与保存方式。
- 测距模块类型、量程、安装方向、刷新率和失效判定。
- 采用 ROS1 Noetic + MAVROS 还是其他 PX4 通信接口，以及最终统一消息定义。

## 第 2 轮：收缩当前实现范围并确定执行顺序

### 用户补充与范围修正

- 现有圆弧飞行代码只是测试代码，不属于矿井探测无人机的正式任务，不纳入后续主架构和实现计划。
- 测距雷达尚未接入，因此当前不能实现真实的“测到地面/底部后返航”闭环，只能先预留测距输入接口，并用模拟事件或定时事件做状态机联调。
- 当前只做两个任务到 PX4 输出层的部分。
- Fast-LIO2 内部不做开发；只保留其作为定位输入的接口边界。

### 本阶段最重要的设计修正

两个任务不应该都被强行定义为只输出 `PoseStamped`。公共接口应定义为“控制参考/轨迹参考”，至少支持：

- 位置参考；
- 速度参考；
- 偏航或偏航角速度；
- 控制模式（位置、速度、保持、停止等）；
- 坐标系、时间戳、有效期、来源和有效标志。

原因是任务一通常可以输出位置/速度轨迹，而任务二的核心动作是保持竖直方向匀速，更自然的是输出 Z 方向速度参考。最终发给 PX4 的消息可以是 `mavros_msgs/PositionTarget`，但任务层不应直接依赖 MAVROS 消息。

### 推荐执行步骤

#### 第 0 步：冻结当前范围

要做：

- 把 `arc_trajectory_node`、圆弧轨迹编辑器等标记为测试遗留代码，不作为任务一或任务二的实现基础。
- 暂不接入雷达驱动，也不在任务二里写死真实雷达逻辑。
- 保留现有 ROS1 Noetic + MAVROS 链路作为通信基础。

完成标准：

- 文档、启动文件和后续代码中不再把圆弧测试代码称为正式任务。
- 明确当前可交付物是“两个任务的统一控制输出链路”。

#### 第 1 步：先定统一消息和坐标约定

要做：

- 约定任务层统一使用一种 ROS 侧坐标系，例如 `mission_local`，不要让任务一和任务二各自定义坐标系。
- 约定公共控制消息包含 `mode`、位置、速度、偏航、时间戳、有效期、来源和有效标志。
- 明确 PX4/MAVROS 边界所使用的坐标系和字段掩码，所有转换只放在一个公共适配层。
- 明确 Z 轴正方向、偏航正方向和机体外参与时间戳规则，并用单元测试验证轴向。

建议不要只用一条 `PoseStamped` 消息承载所有任务输出。可以先在 `mine_uav_control` 中增加一个轻量的自定义消息包，或者在原型阶段使用约定好的 `PositionTarget`，但最终要保留任务模式和来源信息。

完成标准：

- 给出一页接口定义，任何任务只看接口就能输出控制参考。
- 用已知向量验证坐标变换，例如水平两个轴、向上/向下和零偏航，确认没有重复转换或符号错误。

#### 第 2 步：实现唯一的 PX4 输出节点

建议节点名：`mission_output_router` 或 `px4_command_router`。

要做：

- 订阅任务一和任务二的公共控制参考。
- 根据当前激活任务选择唯一一路输入。
- 执行坐标变换、字段掩码设置、速度/加速度限幅和数值有效性检查。
- 以固定频率向一个唯一出口发布 PX4 setpoint。
- 任务输出超时、无效或没有激活任务时，进入预先定义的保持/安全状态，而不是继续发送旧目标。
- 在输出中记录当前来源、任务状态、命令年龄和拒绝原因。

现有 `offboard_bridge` 可以作为最底层发送器的基础：它已经从 `/mine_uav/setpoint_cmd` 转发到 `/mavros/setpoint_raw/local`，并有固定频率和命令超时逻辑。但新的任务代码不能绕过路由器直接发布 `/mavros/setpoint_raw/local`；现有地面站和圆弧节点只能作为测试工具使用。

完成标准：

- ROS 图中只有一个节点拥有 `/mavros/setpoint_raw/local` 的正式发布权。
- 关闭任务节点或停止其心跳后，路由器能识别超时并进入安全输出。

#### 第 3 步：实现任务管理和控制权切换

要做：

- 定义任务状态：`IDLE`、`TASK1_ACTIVE`、`TASK2_ACTIVE`、`HOLD`、`ABORT` 等。
- 定义切换流程：请求切换 → 当前任务停止/释放 → 路由器短暂保持 → 检查新任务可用 → 新任务接管。
- 要求每个任务带有心跳和来源标识；未被选中的任务只能计算或待机，不能抢占 PX4 输出。
- 切换时对位置、速度和偏航做平滑接管，避免 setpoint 突变。

完成标准：

- 任务一和任务二同时启动时只有被选中的任务有效输出。
- 切换期间不会出现两路消息交替覆盖或沿用上一个任务的陈旧 setpoint。

#### 第 4 步：先做任务一适配器，不改 SUPER 内部

要做：

- 为 SUPER/自主决策器定义一个输入适配器，把它的规划结果转换成公共控制参考。
- 先允许接入模拟路径、单点目标或固定保持点，验证接口和时序。
- 对规划结果做空结果、超时、跳变、越界和规划器退出处理。
- 建模、地图保存和可视化放在控制链路之外，不能阻塞 setpoint 输出。

完成标准：

- 用模拟的任务一路径即可驱动公共路由器。
- SUPER 暂停或没有新规划结果时，系统能保持或安全退出。

#### 第 5 步：实现任务二状态机，但先使用模拟测距事件

建议状态：

```text
IDLE
  → ENTRY_HOLD
  → DESCEND
  → BOTTOM_CONFIRM
  → ASCEND
  → RETURN_HOLD
  → COMPLETE
```

要做：

- `ENTRY_HOLD` 保存竖井入口的水平参考和任务起始状态。
- `DESCEND` 保持 X/Y 不主动改变，输出预设的竖直匀速参考，并设置最大下降时间/深度限制。
- `BOTTOM_CONFIRM` 先接受模拟的 `bottom_detected` 事件，后续再替换为真实雷达判定；不要把模拟事件伪装成真实测距数据。
- 触发到底后先停止下降并稳定，再进入 `ASCEND`。
- `ASCEND` 执行与入口方向一致的上升/返航逻辑；当前没有可靠 Z 反馈时，暂时只能用测试用的时间或内部积分验证状态流转，不能把它当作最终安全返航方案。
- 对任务启动、任务中止、超时、PX4 断连、输出超时和无效输入定义明确的状态转移。

完成标准：

- 不接雷达也能通过模拟事件完整跑通“入口保持—下降—到底事件—上升—完成”的控制参考输出。
- 模拟事件撤销、重复或延迟时不会重复触发返航。

#### 第 6 步：先做仿真/离线输出测试，再接真实 PX4

测试顺序：

1. 只启动公共路由器，验证无任务时的安全输出。
2. 只接任务一模拟输出，检查位置、速度和偏航字段。
3. 只接任务二模拟输出，检查下降/上升方向和匀速参考。
4. 同时启动两个任务，验证仲裁结果。
5. 测试任务切换、任务超时、命令跳变、PX4 断连和路由器重启。
6. 再接 PX4 SITL 或拆桨真机，先只验证保持和小范围单轴运动。

不要一开始就做四百米竖井实飞。当前雷达未接入时，能验证的是消息链路、坐标方向、状态机和安全超时，不能验证真实到底判断和最终返航精度。

#### 第 7 步：最后接入 Fast-LIO2 和真实雷达接口

要做：

- Fast-LIO2 只需提供约定的位姿/里程计消息，NUC 侧做时间戳、坐标系、频率、协方差和健康状态检查；不修改 Fast-LIO2 算法。
- 将模拟 `bottom_detected` 替换为真实测距输入适配器。
- 用连续有效采样、量程检查、滞回、超时和传感器失效处理形成到底判定。
- 重新验证 PX4 估计状态、任务输出和返航策略之间的联动。

### 本轮结论

当前最合理的开发顺序不是先写两个完整任务，而是：

```text
统一消息/坐标
  → 唯一 PX4 输出节点
  → 任务仲裁与切换
  → 任务一适配器
  → 任务二状态机（模拟测距事件）
  → 仿真和拆桨测试
  → Fast-LIO2/真实雷达接口联调
```

本阶段的最小可交付物是：任务一和任务二都能通过统一接口产生控制参考，由同一个公共节点转换并输出到 PX4；两个任务互不抢占；雷达缺失时可以用模拟事件验证任务二状态机。

### 与当前工作区代码的对应关系

- `arc_trajectory.*` 和 `ground_station.py` 中的圆弧功能保留为测试遗留，不作为正式任务实现。
- `offboard_bridge` 可以继续作为最底层 PX4 setpoint 发送器，但正式运行时必须保证它是唯一的 PX4 setpoint 出口。
- 当前 `ground_station.py` 会直接发布 `/mavros/setpoint_raw/local`，正式任务链路启用后不能让它与任务输出同时运行；需要改为测试专用，或让它只发布到测试命名空间。
- 后续实现可以按以下逻辑节点拆分：统一消息、`mission_manager`、`px4_command_router`、任务一适配器、任务二竖井状态机。节点是否最终合并，可在接口稳定后再决定。

## 第 3 轮：明确任务调度器职责，优先打通 NUC—PX4 通信

### 用户补充

- 系统必须有任务调度器，用于切换任务一和任务二。
- SUPER 接收的是 Fast-LIO2 输出的点云，用于采空区探测/建模；SUPER 不负责产生定位输入。
- 当前暂不优先实现完整任务逻辑，先把无人机与 NUC 的 ROS1/PX4 通信打通。
- 目标是确认后续任务输出的位姿/控制参考可以正常到达 PX4。

### 本轮工作目标

1. 检查无人机通过 USB/串口接入 NUC 后是否出现可用设备。
2. 检查 ROS1、MAVROS 和当前 `mine_uav_control` 启动文件是否具备通信条件。
3. 启动 MAVROS 后确认 PX4 心跳、连接状态、飞控状态和本地位姿回传。
4. 确认外部位姿输入链路的接口形式，后续再接 Fast-LIO2 的真实输出。
5. 暂不让任务一、任务二或 SUPER 直接控制 PX4；后续由任务调度器选择活动任务。

### 架构职责修正

```text
Fast-LIO2 点云 ─→ SUPER ─→ 任务一输出
Fast-LIO2 位姿 ─→ 外部定位适配 ─→ PX4 状态估计

任务一输出 ─┐
            ├→ 任务调度器/控制仲裁 → PX4 输出适配 → MAVROS → PX4
任务二输出 ─┘
```

SUPER 的点云输入和 Fast-LIO2 的位姿输出属于两条不同的数据流；后续接口设计必须分开。

### NUC 当前环境检查结果

- ROS 版本：ROS1 Noetic。
- MAVROS：1.20.1。
- `mavros_msgs`：已安装并可被 `rospack` 找到。
- `mine_uav_control`：工作区已被 `rospack` 找到，`super_ws/devel/setup.bash` 存在。
- 当前 `px4_serial.launch` 使用 `/dev/ttyACM0:115200`、MAVLink 2，并且只启动 MAVROS；它适合做通信测试。
- 当前不应使用 `real_uav_ground_station.launch` 做纯通信测试，因为它还会启动地面站程序，且地面站会直接发布 `/mavros/setpoint_raw/local`。

### 当前硬件检查结果

刚才检查时没有发现稳定存在的 `/dev/ttyACM0` 或 `/dev/ttyUSB*`。但内核日志显示：

- PX4 设备曾被识别为 `Auterion / PX4 FMU v6C.x`；
- 曾创建 `/dev/ttyACM0`；
- 随后出现 USB disconnect，并反复重新枚举。

因此当前结论是：NUC 的 ROS1/MAVROS 软件条件基本具备，但 USB 飞控设备处于不稳定连接状态，尚未完成 MAVROS 与 PX4 的真实通信验证。下一步应先让 `/dev/ttyACM0` 稳定存在，再启动 MAVROS。

补充：系统的 `dialout` 组配置中已经包含用户 `nuc`，但当前登录会话的有效组列表尚未显示 `dialout`。如果设备稳定后出现串口权限错误，需要重新登录（或重启当前会话）使组权限生效，再测试 MAVROS。

## 第 4 轮：Fast-LIO2 移出当前范围，重新检查飞控连接

### 用户补充

- Fast-LIO2 的位姿输出由其他人负责，后续会直接移植到 NUC；当前完全不考虑 Fast-LIO2 的实现和联调。
- 用户已重新连接飞控，要求立即检查 NUC 与 PX4 的连接状态。

### 本轮执行范围

- 只检查 USB/串口设备、权限和 MAVROS/PX4 通信。
- 不启动旧圆弧测试、旧地面站、任务一、任务二或 SUPER。
- 当前通信验证目标是 PX4 心跳和状态回传；外部位姿输入留到后续接口接入阶段。

### 本轮复查结果

- 重新连接后，PX4 再次被内核识别为 `Auterion PX4 FMU v6C.x`，并短暂创建 `/dev/ttyACM0`。
- 设备随后再次断开；内核日志出现 `Device not responding to setup address`、`error -71` 和 USB 端口 power cycle。
- 复查结束时 `/dev/ttyACM0` 已不存在，因此没有启动 MAVROS，也没有收到 `/mavros/state` 或本地位姿；本轮尚未进行 ROS1—PX4 心跳验证。
- 当前更像是 USB 数据线、USB 端口/接触、供电或飞控 USB 重启问题，尚不能归因于 ROS 包。

### 下一步硬件处理

1. 保持 PX4 独立供电并确认飞控正常启动。
2. 使用确认支持数据传输的 USB 线，避免只充电线和 USB Hub；优先直连 NUC 的另一个 USB 口。
3. 重新插入后先观察 `/dev/ttyACM0` 能否持续存在，再启动 MAVROS。
4. 如果设备稳定但 MAVROS 报 `Permission denied`，重新登录当前用户会话以加载 `dialout` 组权限。

## 第 5 轮：再次确认飞控当前是否在线

### 用户确认

用户询问：飞控是否曾经接入成功，但随后又失联，并要求再次实时检查。

### 检查目标

- 确认当前是否存在 `/dev/ttyACM0`。
- 确认 USB 设备是否仍被识别为 PX4 FMU。
- 查看最近内核日志中是否继续出现断开、重枚举或 USB 错误。
- 如果设备稳定，再继续启动 MAVROS；如果仍不稳定，先处理硬件链路。

### 再次实时检查结果

- 连续约 8 秒检查中，`/dev/ttyACM0` 始终不存在。
- 当前没有发现 PX4/Auterion 的 USB 枚举信息。
- 最近两分钟内核日志没有新的 PX4 连接记录，只看到 USB Hub 设备。

结论：前一次确实发生过“PX4 接入并创建 `ttyACM0`，随后断开”；这一次复查时飞控尚未重新成功接入，不能启动 MAVROS 验证心跳。

## 第 6 轮：确认是否检查了正确 USB 口，以及 PX4 参数的影响

### 用户疑问

用户询问：当前问题是否可能由 PX4 设置造成，之前检查的是否是正确的 USB 口，是否可能实际出现在其他串口设备上。

### 本轮检查目标

- 扫描所有 `/dev/ttyACM*`、`/dev/ttyUSB*`，不限定 `ttyACM0`。
- 扫描当前所有 USB 枚举设备。
- 对照内核日志中的物理 USB 端口路径。
- 区分“USB 设备没有枚举”和“设备已枚举但 MAVLink/PX4 参数通信失败”。

### 全部端口扫描结果

- 已扫描所有 `/dev/ttyACM*` 和 `/dev/ttyUSB*`，当前没有任何飞控串口设备；没有发现 `ttyACM1` 或其他隐藏串口。
- 已扫描当前 USB 总线，当前只有 Hub、存储、键鼠和蓝牙等设备，没有 PX4/Auterion 设备。
- 历史内核日志显示飞控曾出现在两个不同的物理路径：`usb 3-6` 和 `usb 3-3`；这说明之前并非只检查一个物理 USB 口。
- 两个物理口上飞控都被分配为 `/dev/ttyACM0`。串口编号是系统动态分配的，若存在其他 ACM 设备，扫描中也会出现 `ttyACM1` 等名称；当前没有。
- `usb 4-1` 是 GenesysLogic USB Hub，不是 PX4。

### 关于 PX4 设置的判断

当前问题发生在 USB 枚举层：设备节点都没有稳定存在，内核还出现 USB disconnect 和 `error -71`。如果只是 PX4 的 MAVLink 端口、波特率或协议参数不匹配，通常设备仍会存在，只是 MAVROS 无法完成心跳握手；因此目前不能优先归因于 PX4 参数。

## 第 7 轮：用户重新连接后再次检查

### 用户操作

用户重新连接飞控，要求再次进行设备和通信检查。

### 本轮执行

- 扫描全部 `ttyACM`、`ttyUSB` 串口。
- 扫描全部 USB 枚举设备。
- 连续观察飞控设备节点是否保持在线。
- 如果设备稳定，继续启动纯 MAVROS 链路；不启动旧地面站。

### 本次重新连接检查结果

- 09:58:14，PX4 在物理路径 `usb 3-6` 开始枚举。
- 09:58:15，系统识别到 `Auterion PX4 FMU v6C.x` 并创建 `/dev/ttyACM0`。
- 同一秒内设备立即断开。
- 09:58:41 再次尝试枚举，但没有完成设备描述符/串口创建。
- 09:59:11 复查时仍没有任何 `ttyACM` 或 `ttyUSB` 飞控设备。

本轮仍未启动 MAVROS，因为串口设备没有稳定到足以进行心跳通信测试。问题仍发生在 USB 枚举/连接层，而不是任务调度器、SUPER 或 Fast-LIO2 层。

## 第 8 轮：观察 `/dev/ttyACM0` 是否稳定

### 用户问题

用户询问如何观察 `/dev/ttyACM0` 是否稳定存在。

### 建议方法

在一个终端运行设备节点监视：

```bash
watch -n 0.5 'date; ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "没有发现串口设备"'
```

在另一个终端运行内核日志监视：

```bash
journalctl -kf | grep -Ei --line-buffered 'usb|ttyACM|cdc_acm|px4|auterion|disconnect|error -71|power cycle'
```

观察 30 秒到 1 分钟。稳定状态应是 `/dev/ttyACM0` 持续存在，没有反复出现 `USB disconnect`、`error -71` 或 `power cycle`。观察期间先不要启动 MAVROS，避免把通信程序和 USB 问题混在一起。

## 第 9 轮：检查当前 `/dev/ttyACM0` 状态

### 用户请求

用户要求立即检查飞控串口当前是否已经在线。

### 本轮执行

- 立即扫描所有 `ttyACM` 和 `ttyUSB` 设备。
- 检查 PX4 USB 枚举信息。
- 连续观察设备节点数秒，确认不是瞬时出现。

### 第 9 轮最终检查结果

- 主机环境可以访问 `/dev/ttyACM0`，权限为 `root:dialout`。
- 通过现有 `px4_serial.launch` 成功启动了纯 MAVROS 链路，实际使用 `/dev/ttyACM0:115200`、MAVLink 2。
- MAVROS 日志显示已收到 `HEARTBEAT`，并识别为 `PX4 Autopilot`。
- `/mavros/state` 正常发布，实测 `connected: True`、`armed: False`、模式为 `AUTO.LOITER`。
- `/mavros/local_position/pose` 正常发布，实测频率约 30 Hz，说明 PX4 → NUC 的 ROS 状态/位姿回传正常。
- `vision_pose_estimate` 插件已加载，且 `/mavros/vision_pose/pose` 有 MAVROS 订阅者，说明外部位姿入口已经存在。
- 本轮没有注入模拟外部位姿，因此尚未验证具体 Fast-LIO2 位姿值被 PX4 估计器融合；该部分留待 Fast-LIO2 移植后联调。

本轮结论：NUC—PX4 的 ROS1/MAVROS 基础通信已经打通，位姿回传链路已验证，外部位姿输入接口已确认；USB 连接此前存在周期性重枚举风险，但本次 MAVROS 已成功连接并持续运行到检查结束。

补充：已清理之前一次诊断命令遗留的 MAVROS 进程，当前 ROS 图中保留本轮验证用的 `/mavros` 和 `/rosout`。

## 第 10 轮：USB 链路最后一次复查与后续 TELEM2 方案

### 用户计划

- 再复查一次当前 USB/MAVROS 通信。
- 如果仍然反复连接失败，则确认 PX4 USB 口链路不稳定。
- 后续改用 NUC 的 USB 转 TTL，连接飞控 TELEM2；Fast-LIO2 仍不纳入当前工作范围。

### 本轮检查目标

- 检查 `/dev/ttyACM0` 是否存在。
- 检查 MAVROS 是否仍在运行并保持 PX4 连接。
- 检查 `/mavros/state` 和 `/mavros/local_position/pose` 是否持续发布。
- 查看最近内核日志是否有 USB 断开或重枚举。

### 第 10 轮检查结果

- 最后一次启动 MAVROS 时，`/dev/ttyACM0` 打开成功，并再次收到 PX4 heartbeat。
- MAVROS 曾正常运行并持续请求飞控数据，但随后出现 `serial0: receive: End of file`，进程退出。
- 同一时间段内核日志持续记录 `usb 3-6` 的 disconnect、重新枚举、`error -71` 和 power cycle。
- 因此 USB 链路最终确认存在反复断连问题；本次没有继续注入任何控制指令或外部位姿。
- 已停止本轮测试用的 ROS master，避免留下无用的测试进程。

### 后续 TELEM2 通信方案

改用 NUC USB 转 TTL 接飞控 TELEM2 是合理的下一步。后续需要：

1. 确认 USB 转 TTL 的电平为 3.3 V，并核对该飞控具体型号的 TELEM2 引脚定义。
2. 连接时使用共地，TX/RX 交叉；除非确认需要，不把 USB 转 TTL 的 VCC 接到飞控供电端。
3. 在 PX4 参数中把 TELEM2 配置为 MAVLink 端口，并使端口协议/波特率与 USB 转 TTL 一致；TELEM2 链路比 USB 方案更依赖这些参数。
4. 在 NUC 上使用 USB 转 TTL 出现的稳定设备名，优先使用 `/dev/serial/by-id/...`，不要长期写死 `/dev/ttyUSB0`。
5. 将 `px4_serial.launch` 的 `device` 和 `baud` 改为 TELEM2 对应串口后，重新验证 heartbeat、`/mavros/state` 和 `/mavros/local_position/pose`。
6. 通信稳定后，再做任务调度器、任务一/任务二输出和 PX4 接口；Fast-LIO2 仍等待其他人移植后再接入。

## 第 11 轮：改用 USB 转 TTL 接 TELEM2 是否需要重新配置

### 用户问题

用户询问：从当前 PX4 USB 直连方案切换为 NUC USB 转 TTL 连接飞控 TELEM2 后，是否需要重新配置。

### 初步结论

需要重新配置底层串口链路，但 ROS 上层的任务调度器、任务一/任务二输出话题以及 MAVROS 话题名称原则上可以保持不变。

需要重新确认的内容包括：

- PX4 TELEM2 是否启用 MAVLink；
- TELEM2 的 MAVLink 模式、系统/组件配置和波特率；
- USB 转 TTL 的设备名和波特率；
- TX/RX、GND 和电平连接；
- `px4_serial.launch` 中的 `device` 和 `baud`。

USB 直连时使用的是 `/dev/ttyACM0`，改用 USB 转 TTL 后通常会变成 `/dev/ttyUSB0` 或 `/dev/ttyACM0`，实际必须以 NUC 枚举结果为准；优先使用 `/dev/serial/by-id/...`。

### TELEM2 方案的具体配置边界

需要区分两类配置：

- PX4 端：TELEM2 必须运行 MAVLink，并设置正确的 MAVLink 模式、串口波特率和流控。
- NUC 端：MAVROS 的 `device` 改为 USB 转 TTL 实际枚举出的串口，`baud` 必须与 PX4 的 `SER_TEL2_BAUD` 一致。

当前 ROS 话题和上层任务接口不需要因为 USB 改为 TELEM2 而改变。改变的是 MAVROS 的底层 `fcu_url`，例如：

```bash
roslaunch mine_uav_control px4_serial.launch \
  device:=/dev/serial/by-id/usb-<实际适配器名称> \
  baud:=<与 SER_TEL2_BAUD 相同的值> \
  fcu_protocol:=v2.0
```

需要特别注意：

- USB 转 TTL 必须是 UART TTL 电平，不能使用 RS-232 电平转换器。
- TX/RX 交叉，GND 共地；不要默认连接适配器 VCC。
- 只接 TX/RX/GND 时，要确认 PX4 流控没有被配置成必须使用 RTS/CTS。
- `MAV_1_CONFIG`、`MAV_1_MODE` 等参数的具体值要在当前飞控固件的 QGroundControl 参数页确认；当前 NUC 日志显示飞控固件版本与本地 PX4 源码版本不一致，不能直接照搬本地源码的默认值。
- 修改 TELEM2 端口分配后通常需要重启飞控，再设置/确认 `SER_TEL2_BAUD`。

官方 PX4 文档说明，TELEM2 通常用于机载/伴随计算机，常见配置使用 `MAV_1_CONFIG` 指向 TELEM2、`MAV_1_MODE` 使用 Onboard，并通过 `SER_TEL2_BAUD` 设置串口波特率：<https://docs.px4.io/main/en/peripherals/serial_configuration>。

## 第 12 轮：暂缓 TELEM2，确定下一项开发任务

### 用户决定

- 暂时不更换 TELEM2 链路，因为当前 USB 方案已经证明 ROS1/MAVROS 基础通信可以工作，后续切换预计只涉及底层串口和 PX4 端口配置。
- 希望继续开展其他任务，并询问下一步优先做什么。

### 建议的下一项工作

优先实现“任务调度器 + 统一控制输出接口”，而不是马上接入真实 SUPER 或真实竖井传感器。原因是：

- 不依赖 Fast-LIO2 的真实移植；
- 不依赖尚未接入的雷达；
- 能尽早验证任务一/任务二不会抢占控制权；
- 能复用已经验证过的 ROS1/MAVROS/PX4 通信链路。

建议顺序：

1. 定义统一任务控制参考消息。
2. 实现 `mission_manager`，支持任务一/任务二选择、状态和心跳。
3. 实现唯一的 `px4_command_router`，只允许被选任务输出到 PX4。
4. 写任务一模拟节点，模拟 SUPER/自主规划器输出。
5. 写任务二模拟节点，模拟入口保持、下降、到底事件、上升和完成。
6. 用 ROS topic/service 模拟任务切换和 `bottom_detected`，先完成离线或 SITL 联调。
7. 通过 `offboard_bridge` 接入 PX4，但保持未解锁，不进行真实飞行动作。

SUPER 的真实点云输入适配和任务二的真实雷达输入，等调度器和公共输出接口稳定后再接入。
## 第13轮：深入学习 SUPER 源码与原理技术栈

用户希望深刻了解 SUPER 的源码、原理和技术栈。本轮范围确定为：以本机 `/home/nuc/super_ws/src/SUPER` 源码为主，重点分析 ROG-Map、SUPER Planner、FSM、ROS1 接口、轨迹优化、备份轨迹和输出消息；暂不深入圆弧测试代码、雷达硬件接入、Fast-LIO2 内部实现和仿真渲染。将生成一份独立的中文 Markdown 深度导读，同时继续维护本对话记录。

### 本轮完成

- 完成对 SUPER 当前 checkout 的源码链路梳理：ROG-Map → A* → CIRI → MINCO/L-BFGS → BackupTraj → FSM/ROS1 输出。
- 明确 SUPER 的 `PositionCommand` 不是 PX4/MAVROS 原生 setpoint，需要统一任务输出层和 PX4 adapter。
- 记录了当前源码的关键约束：点云需已在 world frame；ROG-Map ROS callback 与外部 `updateMap()` 二选一；默认 `click.yaml` 与当前配置文件名可能不一致；SUPER 本身不是采空区自主决策器。
- 生成详细导读：[SUPER源码与原理技术栈导读.md](/home/nuc/SUPER源码与原理技术栈导读.md)。

## 第14轮：在 Fast-LIO2 与 SUPER 之间增加采空区自主决策器

用户提出在 Fast-LIO2 点云与 SUPER 之间增加自主决策器，负责下一个观察点/前沿点、覆盖策略、建模完成条件和返航条件，并要求适配 Fast-LIO2 数据。

本轮确定的架构：决策器负责任务级“去哪儿”和“何时结束”，SUPER 负责局部安全路径与轨迹生成；决策器通过 `geometry_msgs/PoseStamped` 向 SUPER 的 `/goal` 发布局部目标，不直接向 PX4 发布控制量。Fast-LIO2 输入做成可配置 ROS1 话题，默认适配常见的 `/cloud_registered` + `/Odometry`，使用近似时间同步；点云必须已经是与 SUPER 一致的 world/地图坐标系。

计划实现 `mine_uav_control` 中的 `super_exploration_decider`：基于点云射线更新轻量级局部 free/occupied voxel map，从已知自由与未知相邻处生成 frontier candidate，按信息增益/距离/覆盖去重选择 viewpoint；到达目标后继续选点；无新 frontier 超时、外部返航命令或低电量时发布返航目标；同时发布状态和 RViz Marker 供观察。

### 本轮完成

- 新增 `super_exploration_decider` 节点，使用 ROS1 `message_filters::ApproximateTime` 同步 `/cloud_registered`（`sensor_msgs/PointCloud2`）和 `/Odometry`（`nav_msgs/Odometry`）。
- 新增轻量局部 voxel 地图：点云端点标记 occupied，传感器到端点的射线标记 free，free/unknown 边界生成 frontier 候选。
- 新增信息增益 + 距离评分、目标覆盖去重、目标到达/超时、无 frontier 完成判定、记录 home、外部返航和低电量返航。
- 新增输出：`/goal` 给 SUPER，`/mine_uav/exploration/status`、`finished`、`returning`、RViz Marker；新增 `enable` 和 `reset` 服务。
- 新增配置：[super_exploration_decider.yaml](/home/nuc/super_ws/src/mine_uav_control/config/super_exploration_decider.yaml)、启动文件和 README 说明。
- 已通过 `catkin_make --pkg mine_uav_control`；ROS 图自检确认输入、目标、状态话题和服务均正常注册。
- 明确限制：这是第一版局部 frontier 决策器，与 SUPER 的 ROG-Map 共享输入但不共享内存地图；后续可接入任务调度器，并将全局建模/覆盖地图独立出来。

## 第15轮：确认新增源码是否可以上传到 GitHub

用户希望将本轮新增源码上传到自己的 GitHub 保存。

### 检查结果

- `/home/nuc/super_ws/src/SUPER` 是一个独立 Git 仓库，当前远程为上游仓库 `hku-mars/SUPER`，不能直接作为用户个人项目仓库推送目标。
- 新增的 `mine_uav_control` 位于 SUPER 仓库之外，目前没有发现已配置的用户个人 GitHub 远程地址。
- 本地未安装 `gh` 命令，因此不能通过 GitHub CLI 自动创建或选择个人仓库。
- SUPER 仓库当前存在用户已有的未提交改动 `super_planner/CMakeLists.txt`，后续提交时必须避开该无关改动。

### 当前结论

可以上传，但需要用户提供自己的 GitHub 仓库地址，例如 `https://github.com/<用户名>/<仓库名>.git`；如果仓库尚未创建，需要先在 GitHub 创建一个空仓库。拿到明确远程地址后，只提交本轮 `mine_uav_control` 新增/修改的源码、配置和说明文件，不提交 SUPER 上游仓库及其无关改动。

## 第16轮：整理并尝试推送到用户 GitHub 仓库

用户提供了 GitHub 仓库：`https://github.com/dazhihe7005/-frontier-.git`。

### 本轮完成

- 已确认远程仓库可访问，目标分支为 `main`，远程原先只有一个初始 README 提交。
- 在独立临时工作副本中整理了 `mine_uav_control` ROS1 包，包含源码、头文件、配置、launch、测试、脚本、README 和 `.gitignore`。
- 将本对话记录与 SUPER 技术导读保存到仓库的 `docs/` 目录。
- 删除了 Python 缓存等不应上传的文件。
- 创建本地提交：`14d2aa5 feat: add frontier exploration decider`。
- 已将完整提交副本保存到 `/home/nuc/frontier-upload`。

### 推送结果

推送尚未成功。GitHub 返回 `could not read Username for 'https://github.com'`，说明当前 NUC 没有可用的 GitHub HTTPS 凭据；本机也没有发现 SSH key。代码和本地提交均未丢失，待在 NUC 上配置 GitHub PAT/凭据或 SSH key 后，再执行：

```bash
git -C /home/nuc/frontier-upload push origin main
```

不要在对话中发送 GitHub 密码或 PAT。

## 第17轮：配置 GitHub 推送认证

用户询问如何配置 GitHub 认证。

### 推荐方案：SSH key

在 NUC 上生成 Ed25519 SSH key，将 `.pub` 公钥添加到 GitHub 的 Settings → SSH and GPG keys，然后把本地仓库远程地址改为：

```bash
git -C /home/nuc/frontier-upload remote set-url origin git@github.com:dazhihe7005/-frontier-.git
git -C /home/nuc/frontier-upload push origin main
```

私钥只保存在 NUC，不应发送到对话中。也可以使用 GitHub PAT 通过 HTTPS 认证，但 PAT 应按密码保护，不能明文发布。

## 第18轮：生成 SSH key 时的保存路径

用户执行 `ssh-keygen -t ed25519` 后看到默认保存路径提示：`/home/nuc/.ssh/id_ed25519`。

说明：此处直接按回车，接受默认路径即可。随后出现 passphrase 提示时，可以设置一个密钥密码；如果希望后续推送不重复输入，也可以直接回车留空。若提示文件已存在，不应直接覆盖，应先确认是否要复用已有密钥。

## 第19轮：SSH key 已生成

用户已成功生成 Ed25519 SSH key：

- 私钥：`/home/nuc/.ssh/id_ed25519`
- 公钥：`/home/nuc/.ssh/id_ed25519.pub`
- 指纹：`SHA256:4fKQFit6+pBtqrnqoq12OowdskcZB149XEJ1Iu3OZYs`

下一步是复制公钥到 GitHub 的 SSH keys 页面，测试 `ssh -T git@github.com`，再将 `/home/nuc/frontier-upload` 的远程地址切换为 SSH 并推送 `main` 分支。
