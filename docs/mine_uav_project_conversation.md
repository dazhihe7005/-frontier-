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

## 第20轮：GitHub SSH 认证成功

用户执行 `ssh -T git@github.com` 成功，GitHub 返回已认证用户 `dazhihe7005`，说明 SSH key 已正确添加。

用户随后复制命令时多输入了单引号，导致 Shell 出现 `>` 续行提示；通过 `Ctrl+C` 取消即可。下一步直接使用无多余引号的 SSH remote 推送本地提交。

## 第21轮：源码成功推送到 GitHub

- SSH remote 已配置为 `git@github.com:dazhihe7005/-frontier-.git`。
- 本地 `main` 分支已成功推送到远程 `main` 分支。
- 推送范围包含 `mine_uav_control` 源码、配置、launch、测试、README、对话记录和 SUPER 技术导读。
- 最新提交为：`d6579d6 docs: record successful GitHub authentication`。
- GitHub 仓库地址：[dazhihe7005/-frontier-](https://github.com/dazhihe7005/-frontier-)

## 第22轮：实现遥控器二段开关任务调度器

用户希望在 Fast-LIO2 位姿输入之后、采空区探测和竖井探测任务之前加入任务调度器，并使用无人机遥控器上的二段开关选择任务。

### 架构判断

该设计合理。调度器负责任务选择、RC 去抖、Fast-LIO2 位姿健康检查和失联保护；它不直接向 PX4 发布 setpoint。两个任务分别订阅自己的 enable 信号，最终由唯一的 PX4 command router 输出控制指令，避免任务一和任务二并发控制飞行器。

### 本轮实现

- 新增 `mission_scheduler` 节点。
- 订阅 `/mavros/rc/in`、`/Odometry` 和 `/mavros/state`。
- 默认使用 ROS 通道下标 `5`，即物理 CH6；实际通道号可在 YAML 中修改。
- RC 低位选择任务一 `goaf_exploration`，高位选择任务二 `shaft_exploration`。
- 采用 0.5 秒稳定去抖；中间无效值、RC 丢失或 Fast-LIO2 位姿超时进入 `hold`。
- 任务已经运行后发生 RC/位姿失联时发布 `/mine_uav/mission/return_home=true`。
- 发布 `/mine_uav/mission/active_task`、`goaf_enable`、`shaft_enable`、`return_home`、`status` 和 `switch_event`。
- `super_exploration_decider` 已接入 `/mine_uav/mission/goaf_enable`；切换到任务二时暂停任务一，切回任务一时恢复任务一。

### 验证结果

- `catkin_make --pkg mine_uav_control` 编译通过。
- 模拟低位 RC + Fast-LIO2 位姿：`active_task=1`、`goaf_enable=True`。
- 模拟高位 RC + Fast-LIO2 位姿：`active_task=2`、`goaf_enable=False`。
- 停止模拟 Fast-LIO2 位姿：进入 `active_task=0`，`return_home=True`。
- ROS 图检查确认 SUPER 订阅 `/mine_uav/mission/goaf_enable`。

### 当前边界

- 任务二节点尚未实现，因此目前只发布其 enable 信号。
- 调度器还没有接管或路由 PX4 setpoint；后续需要实现唯一的 `px4_command_router`，并让任务一/任务二分别发布到独立命令话题。
- 真机使用前必须通过 `rostopic echo /mavros/rc/in` 移动开关确认实际物理通道及 PWM 阈值。

## 第23轮：确认点云接口、PX4 位姿链路并完成 SITL 验证

用户确认之前的规划器是否接收 Fast-LIO2 点云、是否会干扰给 PX4 的位姿，以及是否可以先用 SITL。

### 接口结论

- `super_exploration_decider` 接收 `/cloud_registered` (`sensor_msgs/PointCloud2`) 和 `/Odometry` (`nav_msgs/Odometry`)。
- 点云通过 `pcl::fromROSMsg` 转为 `pcl::PointXYZ`，原生使用标准 `x/y/z` 字段；其他字段不会参与 frontier 计算。
- 当前节点只发布 SUPER 的 `/goal`，不发布 PX4 外部定位话题或 PX4 setpoint，因此与 Fast-LIO2 → PX4 位姿链路没有直接发布冲突。
- 调度器只订阅位姿做健康检查，也不发布 PX4 位姿。
- 当前点云适配前提是坐标已经处于 `world` 地图坐标系；节点不自动做 TF 变换。`strict_cloud_frame` 可在确认坐标系后设为 `true`。

### SITL 启动与验证

- 新增 `mission_scheduler_sitl.launch`，使用 PX4 SITL/MAVROS 的 `/mavros/local_position/odom` 临时替代 `/Odometry`，不改变真实 Fast-LIO2 接口。
- 启动时需要加载 PX4/Gazebo 的 `ROS_PACKAGE_PATH` 和 `setup_gazebo.bash`。
- 已成功启动 PX4 SITL、Gazebo、MAVROS 和调度器，MAVROS 状态为 `connected=True`。
- 调度器实际订阅 `/mavros/local_position/odom`、`/mavros/rc/in` 和 `/mavros/state`。
- SITL 中临时发布 RC 低位得到任务一、高位得到任务二，切换结果正确。
- 标准 PX4 SITL 不产生 Fast-LIO2 的 `/cloud_registered`，所以当前只完成调度器/MAVROS/SITL 链路验证；SUPER 完整规划需要后续接入点云回放或仿真点云源。

## 第24轮：同步 GitHub 当前最新内容

用户要求将当前最新源码和 README 再次更新到 GitHub。

### 同步结果

- 已核对本地 `mine_uav_control` 与 GitHub 工作副本，源码、调度器、SUPER enable 门控、配置、launch 和 README 内容一致。
- README 已包含 Fast-LIO2 → 任务调度器 → 采空区/竖井任务架构、任务话题、SITL 启动方法和当前限制。
- 本轮仅新增 GitHub 同步记录，不重复修改已经验证通过的源码。
- 待推送提交用于保存本次最新同步状态。

## 第26轮：确认 PX4 坐标获取与 SUPER 坐标对齐

用户询问是否可以从已连接的 PX4 获取坐标系，并将 SUPER 输出坐标对齐到 PX4，同时进一步询问任务一能否完成采空区自主飞行、探测和完整建模。

### 本轮先回答坐标对齐问题

- 当前代码中的 SUPER `/goal` 使用 `world` 坐标系，规划器假设 Fast-LIO2 点云、Fast-LIO2 位姿和 SUPER ROG-Map 处于同一地图坐标系。
- 当前调度器和 frontier 决策器不会读取或修改 PX4 外部定位，也不向 PX4 发布位姿。
- PX4 对齐应放在“统一 PX4 输出/command router”中：先读取 MAVROS 的 `/mavros/local_position/odom`、TF 和 `/mavros/state`，在初始化时计算 Fast-LIO2 world 到 PX4 local ENU 的刚体变换，再将 SUPER 目标变换后输出给 PX4。
- MAVROS 对外通常使用 ENU，PX4 内部使用 NED；不应手工重复做 ENU/NED 转换，统一通过 MAVROS 接口和明确的 `frame_id` 管理。
- 当前实际检查时 ROS master 未启动，系统也未看到 `/dev/ttyACM*` 或 `/dev/ttyUSB*`，因此尚未从真实 PX4 读取坐标话题；这只是链路未启动/设备未枚举，不代表对齐方案不可行。

## 第29轮：采空区三面墙完成判据与真实 PX4 坐标读取准备

用户补充采空区结构：一面作为入口，另外三面为墙体；只要获得连续三面墙体的稳定建模即可判定完成。用户同时确认 PX4 已连接，希望开始读取 PX4 和 SUPER 坐标系并做对齐。

### 建模判据方向

“入口 + 左墙 + 右墙 + 前墙”的结构化判据比当前通用 frontier 超时更符合实际采空区。后续应从点云/地图中检测三面墙体，并分别维护墙体连续性、观测覆盖率、厚度/距离稳定性和时间稳定性；三面墙满足阈值且入口返航通道可验证后，才触发完整建模返航。

### 本轮硬件检查

- 内核日志显示 PX4 在 12:08:42 枚举为 `/dev/ttyACM0`。
- 12:08:46 设备断开，12:08:48 再次枚举，说明 USB 连接存在反复掉线。
- 随后实时检查时 `/dev/ttyACM0` 已不在，ROS master 也未运行，因此本轮尚未读取真实 PX4 坐标话题。
- 待串口稳定后，先启动 MAVROS 并读取 `/mavros/local_position/odom`、TF 和 `/mavros/state`，再实现 Fast-LIO2/SUPER world 到 PX4 local ENU 的对齐变换。

## 第30轮：确认 `/dev/ttyACM0` 当前存在与任务一主方向策略

用户反馈 `watch -n 0.2 'test -e /dev/ttyACM0 && ls -l /dev/ttyACM0 || echo absent'` 显示：

```text
crw-rw----+ 1 root dialout 166, 0 /dev/ttyACM0
```

### 串口判断

- 该输出证明 `/dev/ttyACM0` 在当前采样时刻存在，是字符设备，属主为 `root`，属组为 `dialout`。
- 如果 `watch` 持续运行期间一直显示该行、从未出现 `absent`，可认为它在观察窗口内持续存在；单次输出不能证明长期稳定。
- 还需要通过 `dmesg -w` 或 `udevadm monitor --kernel --udev --subsystem-match=tty` 观察是否发生断开/重新枚举。
- 后续优先使用 `/dev/serial/by-id/` 下的稳定设备链接；如果没有该链接，再使用 `/dev/ttyACM0`。

### 任务一策略补充

用户明确任务一应以无人机当前机头/进入方向作为主方向，优先沿主方向探索；只有连续探测不到左右墙体或主方向信息不足时，才转入侧向 frontier/其他方向探索。当前代码还未实现这个主方向优先状态机，现有版本仍是通用 frontier 选择器。

## 第27轮：任务一当前能力边界

用户要求继续回答任务一目前能做到的程度。

### 当前可以做到

- 在接收到 Fast-LIO2 位姿和标准 `PointCloud2` 点云后，更新轻量 free/occupied 体素地图。
- 从已知自由空间与未知空间的边界提取 frontier candidate。
- 按信息增益、距离、目标间距和机体碰撞半径选择下一观察目标。
- 将目标发布给 SUPER，由 SUPER 负责局部避障、路径和轨迹生成。
- 到达目标后继续选择下一个目标；无新 frontier、外部返航或低电量时生成 home 目标。
- 通过任务调度器的 `goaf_enable` 控制任务一是否运行。

### 当前不能保证

- 还不能保证首次进入采空区后自动沿主通道一直飞到尽头、识别尽头、掉头并返航；当前算法是一般 frontier 选择器，不是“走廊尽头探测器”。
- 还不能保证采空区整体建模完成。当前地图是半径受限的轻量局部地图，会裁剪远处体素，没有全局持久化地图、完整覆盖率指标、闭环一致性检查和建模质量判据。
- 真实雷达尚未接入，点云质量、遮挡、量程和坐标系尚未实测。
- 当前 `/goal` 只进入 SUPER，统一 PX4 command router 和真实轨迹闭环尚未完成，因此目前不能宣称已经可以安全自主飞行。

### 任务一准确定位

当前任务一完成的是“Fast-LIO2 点云/位姿 → frontier 观察点决策 → SUPER 目标”的第一版任务级骨架；要实现采空区整体建模，还需要增加全局地图/覆盖管理、尽头检测、通道探索策略、建模完整性判定、返航策略和 PX4 输出闭环。

## 第25轮：梳理任务一/任务二 NUC 核心工作完成度

用户希望逐个了解：任务一和任务二的 NUC 核心工作已完成哪些、还需完善哪些、已验证哪些、SITL 能看到什么以及能否生成采空区地图。约定本轮先只回答第一个问题：当前已完成的核心工作。

### 当前完成内容

- 已建立 ROS1/MAVROS/PX4 的 NUC 工程基础，已有串口链路和 MAVROS 通信配置。
- 已完成 Fast-LIO2 输入接口约定：采空区决策器接收 `/cloud_registered` (`sensor_msgs/PointCloud2`) 和 `/Odometry` (`nav_msgs/Odometry`)。
- 已完成任务一第一版 `super_exploration_decider`：轻量体素地图、free/occupied 更新、frontier 候选、信息增益/距离评分、目标发布到 SUPER `/goal`、完成/返航条件。
- 已完成任务一的调度门控：任务一订阅 `/mine_uav/mission/goaf_enable`，切换到任务二时暂停，切回任务一时恢复。
- 已完成任务调度器 `mission_scheduler`：通过 MAVROS `/mavros/rc/in` 的二段开关在任务一和任务二之间选择，并检查位姿新鲜度、RC 丢失和返航请求。
- 已完成任务二的调度接口占位：发布 `shaft_enable` 和 `active_task=2`，但竖井探测的下降、匀速、测距触底和返航控制器尚未实现。
- 已完成 SITL 启动文件和 MAVROS 位姿重映射，用于先验证调度器链路。
- 代码已通过 `catkin_make --pkg mine_uav_control`，调度器已在模拟消息和 PX4 SITL 中验证任务切换。

### 重要边界

当前“任务一”完成的是任务级 frontier 决策器的第一版和 SUPER 门控，不等于已经完成真实采空区自主探测闭环；当前“任务二”只完成任务选择接口，不等于已经完成竖井探测任务本体。统一 PX4 command router 和两个任务到 PX4 setpoint 的完整闭环也尚未完成。

## 第28轮：当前 frontier 方向、机头和建模完整性行为

用户进一步询问：进入采空区后当前算法是否会沿某个方向飞行、遇到左右墙壁或前方障碍是否会掉头返航，以及能否判断采空区是否全部建模。

### 当前实际行为

- 当前决策器不会预先规划“沿采空区主方向一直飞到尽头”；它每次从已知 free 与 unknown 的边界中选 frontier。
- 如果前方是墙或障碍，SUPER 会负责局部避障和轨迹优化，但任务层没有专门的“尽头检测 + 180 度掉头 + 返航”状态机。
- 当前没有 frontier 时，会在满足已开始探索、已到达过目标且持续超过 `no_frontier_timeout` 后触发建模完成并生成 home 目标；这只是局部启发式条件，不等价于确认采空区整体完成。
- frontier 候选的姿态四元数会指向未知空间方向；但 SUPER 当前 `yaw_mode: 1` 是跟随速度方向，最终机头由 SUPER 的 yaw 轨迹平滑生成，不保证原地旋转 180 度后再反向飞行。

### 当前能够知道什么

轻量体素地图中，已观测并标记的 free/occupied 是已知区域，地图中不存在的体素被当作 unknown，因此可以知道“附近还有未知边界”。

### 当前不能证明什么

当前不能证明“采空区已经全部建模”，原因是地图是局部裁剪的，没有任务区域边界、全局持久地图、可达未知区域统计、覆盖率、重访一致性和雷达建模质量判据。雷达开发者负责点云质量，但我们负责把这些观测组织成覆盖策略和完成判据。

### 要实现用户描述的采空区逻辑

后续需要增加专门的任务状态机：入口确认 → 主方向探索 → 尽头/障碍确认 → 侧向 frontier 覆盖 → 已覆盖区域复核 → 完整性判定 → 沿安全路径返航。只有完成全局覆盖判据后，才能把返航称为“建模完成返航”，而不是“当前局部没有 frontier 就返航”。

## 第31轮：完成机头方向优先状态机并分析 USB 日志

用户要求先完成任务一的机头方向优先状态及策略，再分析 `usb usb4-port1: Cannot enable. Maybe the USB cable is bad?`。

### 本轮代码变更

- `super_exploration_decider` 增加 `FORWARD_PRIORITY` 和 `FRONTIER_FALLBACK` 两个方向阶段。
- 在 `FORWARD_PRIORITY` 阶段，根据 Fast-LIO2 位姿四元数计算当前机体 yaw，只在机头前方角度范围内的 frontier 中选目标，并增加机头方向评分权重；发布目标时将目标 yaw 对齐当前机头方向。
- 当前方点云障碍连续多帧确认后，进入 `FRONTIER_FALLBACK`；确认的前方障碍存在时，优先选择非前方候选，若没有安全的非前方候选则等待更多地图数据，不直接穿越障碍。
- 当连续多帧既没有左右墙体观测又没有前方可行 frontier 时，进入 `FRONTIER_FALLBACK`，再使用一般 frontier 选择策略。
- 左右墙体、前方障碍的证据来自当前 Fast-LIO2 `/cloud_registered`，通过确认帧数抑制单帧误检；增加了地面点垂直方向过滤，避免将地面误判为前方墙体。
- 参数已加入 `config/super_exploration_decider.yaml`：前方角度、左右墙体扇区和量程、前方障碍量程、确认帧数及方向权重。
- 该逻辑只影响发给 SUPER 的 `geometry_msgs/PoseStamped` 观察目标，不直接发布 PX4 setpoint，也不改动 Fast-LIO2 到 PX4 的定位链路。
- 已将源码同步至 `/home/nuc/super_ws/src/mine_uav_control`，并通过 `catkin_make --pkg mine_uav_control` 编译。

### USB 日志分析

- `usb usb4-port1: Cannot enable` 表示 Linux 正在尝试启用 NUC 的 USB 总线 4、端口 1，但端口初始化失败；它本身不能证明 PX4 Type-C 口损坏。
- 当前 `lsusb -t` 显示 PX4（`Auterion / PX4 FMU v6C.x`，`cdc_acm`）位于 USB 总线 3、端口 2，而报错反复指向 USB 总线 4、端口 1，因此这批错误很可能来自 NUC 的另一个 USB 端口/控制器，不能直接归因于 PX4。
- 更可能的原因依次包括：USB 数据线质量或仅充电线、接头接触/松动、NUC 端口供电或端口本身问题、USB 控制器/扩展坞问题；只有在拔插 PX4 时明确看到 PX4 所在端口也出现同样错误，才重点怀疑 PX4 Type-C 接口。
- 推荐定位顺序：不用 HUB 换一根确认支持数据传输的短线；换 NUC 的另一个 USB 口；用同一线缆/端口测试其他设备；再用同一 PX4 和线缆接另一台电脑。用 `dmesg -w` 配合拔插，确认报错端口是否与 PX4 枚举出的端口一致。
- `/dev/ttyACM0` 节点存在只能说明 CDC ACM 设备已枚举；是否通信稳定仍需观察 `USB disconnect`、MAVROS `/mavros/state.connected` 和 `/mavros/local_position/odom`。

## 第32轮：生成当前工作空间代码与原理 Word 文档

用户要求生成当前工作空间全部代码及其原理、算法名称、运行原理等内容的 Word 文档，并明确本轮不更新 GitHub，后续轮次恢复正常同步规则。

### 本轮完成

- 以当前 `/home/nuc/super_ws/src/mine_uav_control` 为源码范围，生成本地 Word 文档：`/home/nuc/mine_uav_control_workspace_code_guide.docx`。
- 文档包括：总体架构、技术栈、ROS 话题接口、mission_scheduler 原理、super_exploration_decider 的体素/射线地图、Frontier 探索、机头方向优先有限状态机、SUPER/PX4/MAVROS/Fast-LIO2 数据链路、坐标系边界、运行时序、SITL 验证、USB 分析、后续开发优先级。
- 附录收录 `mine_uav_control` 当前全部文本源码：C++、头文件、Python、YAML、launch、package.xml、CMakeLists、README 和测试文件；排除了 `__pycache__/*.pyc` 二进制缓存。
- 文档已通过 ZIP/XML 结构完整性检查生成；本轮未提交、未推送 GitHub。

## 第33轮：安装本地 WeChat `.deb` 文件

用户执行 `sudo apt install WeChatLinux_x86_64.deb` 时得到 `Unable to locate package`。

原因：`apt install` 默认把参数当作软件包名搜索；本地 `.deb` 文件必须使用带路径的形式，例如当前目录使用 `./WeChatLinux_x86_64.deb`，或使用绝对路径。还需要确认文件名大小写和当前目录确实存在该文件。

正确方式：

```bash
ls -l ./WeChatLinux_x86_64.deb
sudo apt install ./WeChatLinux_x86_64.deb
```

如果文件不在当前目录，应先 `cd` 到实际目录，或直接执行 `sudo apt install /实际路径/WeChatLinux_x86_64.deb`。如果旧版本 apt 无法处理本地文件，可先使用 `sudo dpkg -i /实际路径/WeChatLinux_x86_64.deb`，再使用 `sudo apt -f install` 修复依赖。

## 第34轮：本地 WeChat `.deb` 路径仍未被识别

用户在家目录执行 `sudo apt install ./WeChatLinux_x86_64.deb`，收到 `Unsupported file ./WeChatLinux_x86_64.deb given on commandline`。

判断：当前目录很可能没有这个文件，或者实际文件名/大小写不同；也可能是系统 apt 版本对本地 deb 路径支持不完整。应先用 `pwd`、`ls -l` 或 `find ~/ -type f -iname 'WeChatLinux*.deb'` 定位真实文件，再使用完整路径；最兼容的安装方式是 `sudo dpkg -i /实际路径/文件.deb`，随后执行 `sudo apt-get -f install` 修复依赖。本轮不上传 GitHub。

## 第35轮：实测 Fast-LIO2、任务一决策器和 SUPER 通信链路

用户表示 Fast-LIO2 雷达已经接入 NUC，希望实际打通 Fast-LIO2 与现有 SUPER。

### 已确认的 ROS 图连接

- ROS Master 可访问。
- `/livox_lidar_publisher2`、`/laserMapping`、`/super_exploration_decider` 已运行。
- `/laserMapping` 发布 `/Odometry`、`/cloud_registered`；`/super_exploration_decider` 已同时订阅这两个话题，TCPROS 连接已建立。
- SUPER 的 `/fsm_node` 已成功启动，并订阅 `/goal`、`/Odometry`、`/cloud_registered`。
- SUPER 配置实际读取到：`fsm.click_goal_topic=/goal`、`rog_map/ros_callback/cloud_topic=/cloud_registered`、`rog_map/ros_callback/odom_topic=/Odometry`。
- `/goal` 的消息类型为 `geometry_msgs/PoseStamped`，任务一决策器发布端与 SUPER 订阅端类型一致。

### 当前阻塞点

- `/livox/lidar`、`/livox/imu`、`/Odometry`、`/cloud_registered` 实测均没有新消息。
- 任务一状态持续为 `WAIT_DATA: Fast-LIO2 synchronized data is stale or absent`。
- `enp89s0` 是配置的 MID360 网口，地址为 `192.168.1.10`，但当前显示 `NO-CARRIER`、`state DOWN`、对应路由为 `linkdown`。
- Livox 驱动虽然运行并向 `/laserMapping` 建立了 ROS 发布连接，但没有收到雷达 UDP 数据；因此 Fast-LIO2 没有输入，后面的两个输出自然为空。
- SUPER 能收到任务一的锁存旧 `/goal`，但因没有实时位姿和点云而报 `No odom`、`No point cloud input`，这属于输入链路未恢复，不是 `/goal` 话题接口不匹配。

### 下一步

检查 MID360 电源、网线、网口指示灯和 NUC 的实际网口；确认 `enp89s0` 从 `NO-CARRIER` 变为有 carrier 后，再测试 `ping -I enp89s0 192.168.1.157`、`rostopic hz /livox/lidar`、`rostopic hz /Odometry` 和 `rostopic hz /cloud_registered`。数据恢复后再处理 SUPER 的 `fix_map_origin`、虚拟地面高度和实际坐标系校准。本轮不上传 GitHub。

### 本次实际启动 SUPER 的结果

- 直接运行已编译的 `/home/nuc/super_ws/devel/lib/super_planner/fsm_node` 后，配置成功加载，且日志明确显示 `click_goal_topic=/goal`、`cloud_topic=/cloud_registered`、`odom_topic=/Odometry`。
- `rosnode info /fsm_node` 确认 SUPER 已订阅 `/goal`、`/Odometry`、`/cloud_registered`；任务一决策器也已经发布 `/goal`，两端消息类型匹配。
- 因没有实时位姿和点云，SUPER 日志出现 `No odom`、`No point cloud input` 和 `PlanFromRest failed`；这证明阻塞在输入数据，不是 SUPER 接口不通。测试结束后已停止临时 SUPER 进程。
- 当前发现同时存在两个 `livox_ros_driver2_node` 进程（PID 82026 和 85919），启动时间分别为 14:57 和 15:45，但 ROS 图上只有一个同名节点。应保留一个 Livox 驱动实例，避免 UDP 端口或雷达控制冲突；不要在问题解决前重复启动驱动。

## 第36轮：雷达上线后完成 Fast-LIO2 → 决策器 → SUPER 实机数据联调

用户打开 MID360 后要求重新测试，并修复上一轮发现的问题。

### 雷达与 Fast-LIO2 恢复结果

- MID360 网络链路恢复，NUC 使用 `192.168.1.10/24`，雷达 `192.168.1.157` 可达。
- 清理了一个未注册到当前 ROS Master 的旧 Livox 驱动进程，只保留实际发布数据的 `/livox_lidar_publisher2`，避免重复驱动争用雷达 UDP/控制链路。
- 原 Fast-LIO2 曾发散到数千至数万米，已停止旧实例并在雷达数据稳定后重新启动；IMU 初始化完成后位姿恢复到起点附近。
- 最终实测 `/livox/lidar` 约 `10 Hz`、`/livox/imu` 约 `200 Hz`、`/Odometry` 和 `/cloud_registered` 均约 `10 Hz`。当前位姿约为 `(0.02, 0.03, -0.07)`，未再出现发散。
- Fast-LIO2 注册点云字段为标准 `x/y/z/intensity/normal_x/normal_y/normal_z/curvature`，点云和里程计均使用 `camera_init`。

### 接口和源码修复

- 决策器 `world_frame` 改为 `camera_init`，并启用 `strict_cloud_frame: true`；不再接受未经变换的其他坐标系点云。
- SUPER 配置改为：`click_height: -10.0`，保留决策器给出的三维 z；`fix_map_origin: [0,0,0]`；可视化坐标系为 `camera_init`。
- 台架联调将 SUPER `virtual_ground_height` 临时设为 `-2.0 m`，实际离散后的有效虚拟地面约为 `-1.5 m`，避免起点附近 z 被误判为低于地面。正式飞行前必须按雷达安装高度和真实地面重新标定。
- 发现 SUPER ROS1 源码仍把 ROG-Map、轨迹命令、TF 和可视化消息的 frame 硬编码为 `world`。已改为统一读取 YAML 的 `rog_map/visualization/frame_id`，而不是写死 `camera_init`，以便以后通过配置切换真机和 SITL。
- 新增统一环境脚本 `scripts/setup_fastlio2_super_env.sh`，同时保留 Fast-LIO2 与 SUPER 的 `ROS_PACKAGE_PATH`、`PYTHONPATH`、动态库和 CMake 搜索路径。实测 `CustomMsg import: OK`，解决后 source 的工作空间覆盖前一个工作空间、导致 `rostopic` 无法解析 `livox_ros_driver2/CustomMsg` 的问题。
- 从历史目标日志发现决策器曾发布 z 为 `6–12 m` 的观察点，超过 SUPER 当前有效上限约 `2.9 m`。已新增相对任务起点的观察点高度范围，默认 `0.5–2.5 m`；返航 home 高度不受该观察点限制。

### 编译和端到端验证

- `super_planner` 和 `mine_uav_control` 均重新编译成功。
- 决策器成功实时记录 home、分析点云并发布 `/goal`；短超时连续抽测 11 个目标，z 均为 `0.75–1.75 m`，没有再越过高度边界。
- SUPER 成功订阅 `/goal`、`/Odometry`、`/cloud_registered`，生成 ROG 占据地图和轨迹。抽样地图包含 `3249` 个占据点，规划阶段 `/planning/pos_cmd` 以 `100 Hz` 输出。
- 修复后实测 `/goal`、`/fsm_node/rog_map/occ`、`/planning/pos_cmd` 的 `frame_id` 全部为 `camera_init`，Fast-LIO2 → 决策器 → SUPER 数据链路已打通。
- 台架上的无人机不会执行 SUPER 轨迹，因此运行一段时间后会出现目标超时、重复重规划、优化失败或 yaw rate 告警；这不能替代闭环 SITL/拆桨真机跟踪验证。为避免持续空转刷日志，本轮验证结束后停止了临时 SUPER，保留雷达、Fast-LIO2 和正式参数的决策器在线。
- 决策器没有 SUPER 的轨迹执行反馈，因此将原先可能误导的状态文字“SUPER is following”改成“目标已发布，等待里程计进展”；后续闭环阶段应再接入 SUPER 规划成功/失败反馈。
- 本轮只更新本地源码、README 和本对话记录，没有推送 GitHub。

## 第37轮：项目完成度盘点并加入任务一闭环算法仿真

用户询问整个项目已完成部分、完成程度、待调参数、SUPER 参数、剩余任务、可完成场景，以及如何在仿真中观察现象。

### 当前完成度结论

- 项目已经形成可运行的任务一原型，但还不是可下井自主飞行的完整系统。NUC 软件功能原型整体约完成一半，真机自主任务闭环完成度更低。
- 真雷达、Livox 驱动和 Fast-LIO2 已实测输出稳定；Fast-LIO2 点云和位姿已打通决策器与 SUPER。
- 任务调度器已实现 RC 二段开关选择、稳定时间、RC/里程计超时和 HOLD/返航请求；实际遥控器通道和真机故障保护仍需标定。
- 任务一已实现轻量体素/射线地图、frontier 候选、机头优先、前障碍 fallback、观察点高度限制、home 水平安全围栏、简单无 frontier 完成判据和返航目标。
- SUPER 已完成 `camera_init` 坐标系适配、ROG-Map 接入、目标接入和轨迹输出；真机 PX4 闭环尚未完成。
- 任务二目前只有调度器的 `shaft_enable` 输出，没有竖井下降、测距触底、匀速控制和返航控制器。
- Fast-LIO2 位姿送入 PX4 外部视觉估计、SUPER `PositionCommand` 转 MAVROS 指令、任务一/任务二唯一命令仲裁器尚未实现。现有 `offboard_bridge` 只转发已经是 `mavros_msgs/PositionTarget` 的 `/mine_uav/setpoint_cmd`。
- 当前“建模完成”仍是局部 frontier 超时启发式，不等同于连续三面墙或全局模型完整性验证。

### 本轮新增安全约束

- 增加 `max_exploration_radius_from_home`：候选观察点不得超过相对 home 的水平安全半径，真机默认 `35 m`，需要按采空区长度、通信和续航重新设置。
- 该约束来自仿真中发现的真实接口问题：滚动探索目标可持续向外漂移，而 SUPER 示例地图是固定有限范围。

### 新增任务一算法仿真

- 新增 `launch/goaf_algorithm_sim.launch`，使用 SUPER 的 `perfect_drone_sim` 在独立 ROS Master 上闭环运行：模拟 360° 点云和理想里程计 → `super_exploration_decider` → SUPER → 模拟无人机。
- 该模式不是 PX4 SITL，也绕过 Fast-LIO2 状态估计；用途是先验证任务一决策、局部规划和返回逻辑。
- 首轮发现目标超出 SUPER 示例固定地图；加入 `6 m` 仿真搜索半径和 home 围栏后，闭环验证通过。
- 实测模拟点云约 `10 Hz`、里程计约 `100 Hz`。模拟无人机执行多个观察点后返回 home，最终状态为 `COMPLETE`，终点约 `(0.025, 0.025, 1.525)`，起点为 `(0,0,1.5)`。
- 图形运行命令：

```bash
source /home/nuc/super_ws/src/mine_uav_control/scripts/setup_fastlio2_super_env.sh
export ROS_MASTER_URI=http://127.0.0.1:11312
roslaunch mine_uav_control goaf_algorithm_sim.launch
```

- 第二终端必须设置相同 `ROS_MASTER_URI` 后再观察状态和话题。无图形验证可追加 `rviz:=false`。
- 真雷达继续使用默认 `11311`，算法仿真使用 `11312`，两套 ROS 图互不干扰。验证结束后已停止仿真，重新启动了包含 `35 m` home 围栏的真机决策器。

### 完整 PX4 SITL 仍缺的部分

- 当前工作空间没有 MID360/Livox Gazebo 仿真插件。真正的全链路需要模拟雷达原始包和 IMU、Fast-LIO2、PX4 SITL/MAVROS、外部视觉注入、SUPER 指令转换与命令仲裁器。
- 可立即运行的 `mission_scheduler_sitl.launch` 只验证 PX4/MAVROS 和任务切换；本轮新增的 `goaf_algorithm_sim.launch` 验证任务一算法闭环。后续应把这两条链路合并。
- 本轮没有推送 GitHub。

## 第38轮：RViz 点云确认与 Gazebo 全链路仿真边界

用户询问 RViz 是否能看到当前点云，以及能否把采空区环境、雷达、Fast-LIO2、SUPER、NUC 控制和 PX4 SITL 全部放入 Gazebo 仿真。

### RViz 点云结论

- 本轮检查时默认 ROS Master 没有运行，因此“当前这一刻”RViz 不会收到真实雷达点云；这只是软件栈未启动，不能据此判断雷达故障。
- 真雷达链路启动后，在 RViz 中将 Fixed Frame 设为 `camera_init`，添加 `PointCloud2` 并选择 `/cloud_registered`，即可显示 Fast-LIO2 注册后的点云地图。原始 `/livox/lidar` 是 Livox 自定义消息，不能直接作为 RViz 的标准 `PointCloud2` 显示。
- 现有 `goaf_algorithm_sim.launch` 的 RViz Fixed Frame 为 `world`，可显示 `/cloud_registered`、`/global_pc`、里程计、SUPER 轨迹和地图；决策器候选点使用 `MarkerArray` 话题 `/mine_uav/exploration/frontiers`。
- 本轮再次无界面短测算法仿真：`/cloud_registered` 约 `10 Hz`，单帧约 `76765` 点，frame 为 `world`；`/lidar_slam/odom` 约 `100 Hz`，同时存在 frontier、SUPER 目标和轨迹话题。测试后已停止临时仿真。

### Gazebo 全链路可行性

- 技术上可行，但当前还没有一个启动文件能把全部模块一次启动。现有算法仿真使用 SUPER 的 `perfect_drone_sim`，不是 Gazebo/PX4；现有 `mission_scheduler_sitl.launch` 只有 Gazebo/PX4/MAVROS/调度器，没有雷达点云、Fast-LIO2 和 SUPER 控制闭环。
- 本机已安装 Gazebo Classic 的三维 block-laser、GPU laser、深度相机和 IMU ROS 插件；PX4 v1.14.4 也自带 `iris_rplidar`、`iris_depth_camera` 和 `iris_triple_depth_camera` 模型。
- 当前 Fast-LIO2 源码包含 `lidar_type: 4` 的 MARSIM 接口，可直接读取 `sensor_msgs/PointCloud2` 和仿真 IMU，因此无需强制模拟 Livox UDP 协议或 `CustomMsg` 才能做算法级闭环。
- 真正完整的目标链路应为：Gazebo 采空区世界和虚拟三维雷达/IMU → Fast-LIO2 → `/Odometry`、`/cloud_registered` → 自主决策器 → SUPER → `PositionCommand` 转换和任务命令仲裁 → MAVROS → PX4 SITL → Gazebo 飞行动力学。
- 物理 MID360 不能作为虚拟无人机的闭环传感器，因为真雷达不随 Gazebo 飞行器运动；仿真必须使用虚拟雷达。真雷达 rosbag 只能用于数据接口回放和开环测试。
- 当前全链路还缺：采空区 Gazebo world、适合三维建图的虚拟雷达模型及 IMU 话题、Gazebo/ROS/PX4/Fast-LIO 坐标与时间同步、Fast-LIO 外部视觉送 PX4、SUPER `quadrotor_msgs/PositionCommand` 到 MAVROS 的转换、任务一/任务二唯一命令仲裁器，以及一键启动文件。
- 推荐分两步实施：先完成 Gazebo + PX4 + 虚拟注册点云 + 决策器 + SUPER + 控制转换，验证飞行控制闭环；再接入虚拟原始雷达和 IMU，通过 Fast-LIO2 估计并将位姿反馈给 PX4。这样能将规划/控制问题与定位问题分开定位。
- 本轮仅检查、验证并更新本对话记录，没有推送 GitHub。

## 第39轮：CH340 转 TELEM2 串口链路检查与对话记录同步

用户将 CH340 USB 转 TTL 接到 NUC，并将 TTL 端连接 PX4 TELEM2，要求检查链路稳定性，同时把此前对话记录更新到 GitHub。

### 本轮串口检查结果

- 内核日志表明 CH340 曾于 17:31:35 被正确识别：USB VID:PID 为 `1a86:7523`，位于 `usb 3-3`，驱动 `ch341` 已加载，并成功创建过 `/dev/ttyUSB0`。
- 本轮实际检查时，`lsusb` 中已经没有 CH340，`/dev/ttyUSB0`、`/dev/ttyACM0` 和 `/dev/serial/by-id` 均不存在。
- 连续 15 秒、每 0.2 秒检查一次 `/dev/ttyUSB0`，始终为 `absent`。因此目前只能判定电脑到 CH340 的 USB 枚举链路不在线，无法继续验证串口字节流、MAVLink 心跳和 MAVROS 连接稳定性。
- 后续内核日志又捕捉到 CH340 于 17:38:40 从 `usb 3-3` 断开，17:38:49 改从 `usb 3-2` 重新接入并再次创建 `/dev/ttyUSB0`。本轮当时在受限检查环境中看不到 `/dev/ttyUSB0`，曾据此判断设备再次消失；第40轮通过主机权限复查后确认这是设备节点隔离造成的假阴性，17:38:49 以后 CH340 实际保持在线。
- 即使 TELEM2 的 TX/RX 没接或 PX4 参数未配置，CH340 只要插在 NUC USB 口上也应稳定枚举。因此当前问题优先位于 CH340、USB 插头/延长线或 NUC USB 口一侧，而不是 PX4 的 `MAV_1_CONFIG`、`SER_TEL2_BAUD` 等参数。
- 此前反复出现 `usb4-port1: Cannot enable` 的设备实际是 VID:PID `05e3:0626` 的 GenesysLogic USB3.1 Hub；本次 CH340 在 `usb 3-3`，两者不是同一个 USB 设备。旧 Hub/Type-C 链路仍不稳定，但不能用该日志直接判定 CH340 或 PX4 TELEM2 故障。
- 当前 `nuc` 用户的补充组列表中未见 `dialout`。待 `/dev/ttyUSB0` 恢复后，需要检查设备 ACL；如果当前用户没有读写权限，再执行 `sudo usermod -aG dialout nuc` 并注销重新登录。

### 恢复枚举后的验证顺序

1. 先只把 CH340 插入 NUC，确认 `/dev/ttyUSB0` 连续存在；这一阶段不依赖 PX4 和 TELEM2。
2. 接线使用共地、交叉收发：CH340 TX 接 TELEM2 RX，CH340 RX 接 TELEM2 TX，GND 接 GND；确认使用 3.3 V TTL 电平，不要用 RS-232 电平，也不要把 CH340 的 5 V/VCC 接到 TELEM2 供电针脚。
3. PX4 中将对应 MAVLink 实例配置到 TELEM2，保证 `SER_TEL2_BAUD` 与 MAVROS 波特率一致。项目现有 `px4_serial.launch` 默认仍是 `/dev/ttyACM0:115200`，使用 CH340 时必须通过参数改为实际 `/dev/ttyUSB0` 和 TELEM2 的实际波特率；稳定后建议改用 `/dev/serial/by-id` 或自定义 udev 固定名称。
4. 串口设备稳定后启动 MAVROS，检查 `/mavros/state.connected`、MAVLink 丢包率和持续心跳，再进行拔插/晃动和较长时间稳定性测试。

### GitHub 同步范围

- 按用户要求，将截至本轮的完整对话记录同步到仓库 `git@github.com:dazhihe7005/-frontier-.git` 的 `docs/mine_uav_project_conversation.md`。
- 串口链路尚未通过，因此记录中明确保留当前阻塞结论，不宣称 MAVLink 已连通。
- GitHub 默认 DNS 和 SSH 22 端口在当前网络中不可用，最终通过 GitHub SSH 443 入口完成推送；远端 `main` 已包含本轮记录。

## 第40轮：CH340/TELEM2 在 500000 波特率下通信验证通过

用户确认 CH340 已正确连接且指示灯常亮，并补充 TELEM2 波特率为 `500000`，要求重新判断是否需要配置 TELEM2、是否为 NUC 故障。

### 纠正设备可见性误判

- 普通工具检查运行在受限设备环境中，虽然能看到内核 sysfs 的 `ttyUSB0`，却看不到主机真实 `/dev/ttyUSB0`，导致上一轮最终检查出现假阴性。
- 使用主机权限重新检查后确认 CH340 正常存在：VID:PID `1a86:7523`，设备为 `/dev/ttyUSB0`，稳定链接为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`。
- 设备权限是 `root:dialout`、`0660`，同时 ACL 已给 `nuc` 用户读写权限，因此当前不需要为串口访问修改用户组。
- 连续 30 秒 USB 监测中设备一直存在，没有新增断开事件。

### 波特率与 MAVLink 验证

- 之前项目启动配置仍是旧的 `/dev/ttyACM0:115200`，初次测试使用 921600；这些波特率与 TELEM2 的 500000 不匹配，因此能看到电信号却无法解析 MAVLink 心跳。
- 改用 `/dev/ttyUSB0:500000` 后，MAVROS 立即报告 `Got HEARTBEAT, connected. FCU: PX4 Autopilot`，并识别 IMU、RC_CHANNELS 和 PX4 飞控版本。
- 通过 MAVLink 只读查询确认 PX4 已正确配置：`MAV_1_CONFIG=102`（TELEM2）、`MAV_1_MODE=2`（Onboard）、`MAV_1_RATE=0`、`SER_TEL2_BAUD=500000`。因此当前不需要再修改 TELEM2 参数。
- 实测 `/mavros/state.connected=true`，飞控处于 `ALTCTL`、未解锁；IMU 约 `44.2 Hz`，RC 输入约 `17.7 Hz`。
- 45 秒持续检查中，47 次 MAVROS 状态采样全部为 connected，`/dev/ttyUSB0` 全程存在；累计接收增长到 `62016` 包，丢包 `0`、缓冲区溢出 `0`、解析错误 `0`。
- 结论：当前 NUC → CH340 → TELEM2 → PX4 的 MAVLink 链路稳定。此前直连 PX4 USB 的 `error -71` 更可能来自 PX4 USB 接口、线缆或旧 Hub 链路，不能据此判定整台 NUC 的 USB 控制器故障；同一 NUC 当前可稳定运行 CH340 是直接反证。

### 项目配置修正

- `px4_serial.launch` 和 `real_uav_ground_station.launch` 默认设备改为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，默认波特率改为 `500000`。
- README 已更新为当前 CH340/TELEM2 接法、参数和启动命令。
- 不带覆盖参数重新启动验证通过：MAVROS 实际 FCU URL 为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0:500000`，连接正常且错误计数均为 0。

## 第41轮：任务一 PX4 闭环补齐、指令安全桥和雷达重连诊断

用户要求暂时不处理任务二，逐步确保“采空区任务一”形成完整闭环。

### 开始本轮时的准确结论

- 已分别验证两段链路：MID360 → Fast-LIO2 → 自主决策器 → SUPER，以及 NUC → CH340 → TELEM2 → PX4/MAVROS。
- 两段链路当时还没有真正连成一个控制闭环，缺少 Fast-LIO2 位姿送 PX4 EKF2，以及 SUPER `PositionCommand` 到 MAVROS/PX4 设定值的坐标转换和安全门控。
- 因此不能仅凭“雷达和串口都通”宣称任务一已经可飞。

### Fast-LIO2 外部视觉位姿桥

- 新增 `fastlio_px4_vision_bridge`，订阅 `/Odometry` 和 `/mavros/state`，以 30 Hz 发布 `/mavros/vision_pose/pose_cov`。
- 默认在 PX4 未解锁时捕获 Fast-LIO2 起点和初始 yaw，使 PX4 本地原点与 Fast-LIO2 当前起点对齐；同时发布锁存的 `/mine_uav/task1/fastlio_to_px4_alignment`，供控制指令使用同一变换。
- 增加帧名、有限值、输入跳变、数据超时、MAVROS 连接和“禁止解锁后重建对齐”等保护；健康状态发布到 `/mine_uav/task1/vision_healthy` 和 `/mine_uav/task1/vision_status`。
- 实机台架测试中，位姿桥成功捕获约 `[-0.269, 0.144, -0.082]`、初始 yaw 约 `-25.40°`，输出稳定为 `30 Hz`；PX4 保持 `ALTCTL`、未解锁，MAVLink 无丢包和解析错误。
- PX4 当前 `EKF2_EV_CTRL=9`，即启用外部视觉水平位置和 yaw，未启用外部视觉垂直位置/速度。估计器仍出现 `const_pos_mode=true`，所以仍需通过实际小幅移动机体和日志确认 EV 已可靠融合；当前不能据此带桨飞行。

### SUPER 到 PX4 指令桥

- 新增 `super_px4_command_bridge`，订阅 SUPER `/planning/pos_cmd`，把位置、速度、加速度、yaw 和 yaw rate 从 `camera_init` 按位姿桥的同一对齐关系转换到 PX4 本地 ENU，并输出 `/mine_uav/setpoint_cmd`；现有 `offboard_bridge` 再以 50 Hz 转发到 `/mavros/setpoint_raw/local`。
- MAVROS 的 `PositionTarget` 在 ROS 一侧使用 ENU；消息的 `coordinate_frame=FRAME_LOCAL_NED` 由 MAVROS 在发送 MAVLink 时完成 ENU/NED 转换，桥接节点不再重复交换轴或反转 z。
- 增加四重安全门：任务调度器选择任务一、视觉定位健康、MAVROS 在线、操作者显式调用 `/super_px4_command_bridge/enable`。默认关闭，节点永不自动解锁，也不自动切换 OFFBOARD。
- 增加帧名、非有限数、轨迹状态、速度、加速度、水平半径和高度围栏检查。任务切走、视觉失效、MAVROS 断开、PX4 本地位姿超时或越界指令会停止输出并锁回人工关闭。
- 修复了首次版本的连续性问题：SUPER 在两段轨迹之间会短暂停止发布，不能把它直接视为永久故障。新版在规划间隙按 PX4 当前本地位姿持续输出悬停目标；新轨迹到达后可自动恢复跟踪，而关键健康门失效仍会锁断。

### 隔离回归测试结果

- 在独立 ROS Master 上注入“绕 z 轴旋转 90°、平移 `(10,20,1)`”的对齐关系，SUPER 输入位置 `(1,2,0.5)` 精确转换为 `(8,21,1.5)`；速度 `(1,0,0)` 转为 `(0,1,0)`，加速度 `(0,1,0)` 转为 `(-1,0,0)`，yaw `0.25` 转为约 `1.8208 rad`。
- 停止 SUPER 轨迹后，桥接器以 50 Hz 输出 PX4 当前测试位姿 `(3,4,1.2)` 和 yaw `0.5 rad` 的位置悬停目标，状态为 `HOLD_COMMAND_TIMEOUT`。
- 将任务一选择改为 false 后，设定值立即停止，人工使能锁回关闭。以上测试完全在隔离 ROS 环境中进行，没有连接真实 PX4。

### 一键启动和首次实机参数

- 新增 `task1_real.launch`，可统一启动 Livox、Fast-LIO2、MAVROS、位姿桥、任务调度器、自主决策器、SUPER、指令桥和设定值转发器。
- 修正统一环境脚本：Fast-LIO2 使用 `devel` 空间，SUPER 以保留前一 overlay 的方式加载；`roslaunch --nodes` 已成功解析出 9 个预期节点。
- SUPER 首次实机限速降为 `1.0 m/s`、最大加速度 `1.5 m/s²`、最大 jerk `20 m/s³`、最大 yaw rate `1.0 rad/s`，机体安全半径统一为 `0.35 m`。
- 调度器正式配置改为必须检查 MAVROS 连接。当前 `/mavros/rc/in` 的物理 CH6 值约 `1499`，落在无效中间区，因此调度器实际输出 `hold`；必须让用户拨动开关并观察通道变化后，才能确定 CH6 是否真是目标二段开关。

### 当前真实链路状态与 MID360 阻塞

- PX4 链路当前仍为 `connected=true`、`ALTCTL`、`armed=false`。真实指令桥和调度器已启动，但指令桥状态为 `WAIT_TASK1_SELECTION`、`command_ready=false`，实测没有向 `/mavros/setpoint_raw/local` 发布任何设定值。
- 本轮重启 Livox 驱动时发现源配置误写为雷达 `192.168.1.5`；实际 MID360 是 `192.168.1.157`，已把 Livox 源配置改为 `.157`。
- 进一步用端口监听确认 `.157` 正在正确向 NUC `.10` 发送点云约 `2084 包/s` 到 UDP `56301`、IMU 约 `200 包/s` 到 `56401`、状态到 `56201`。网线和雷达数据发送本身正常。
- 当前 Livox 驱动只绑定发现端口 `56000`，没有绑定数据端口，内核 `UdpNoPorts` 约增加 `2290 包/s`。原因是新驱动启动时雷达仍保持上一连接状态、没有重新完成设备发现握手。
- 当前需要保持 Livox 驱动运行，将 MID360 单独断电再上电一次，让 SDK 重新收到设备发现广播。雷达恢复后再启动 SUPER，并在“未解锁”状态验证 `/planning/pos_cmd → /mine_uav/setpoint_cmd → /mavros/setpoint_raw/local` 的真实全链路。

### 仍未完成的飞行放行项

- 通过移动机体确认 PX4 EKF2 真正融合外部视觉，且坐标移动方向、尺度、yaw 和复位行为正确；解决或解释 `const_pos_mode`。
- 标定 MID360/IMU 到无人机 PX4 FRD 机体系的安装旋转和平移。目前 Fast-LIO2 内部的雷达到 IMU 外参不等于整机安装外参。
- 根据高度源方案决定是否启用 EV z/速度；完成拆桨 OFFBOARD 跟踪、失联/定位故障保护和系留低空测试后，才能进行带桨自主探索。
- 当前任务一的“建模完成”仍是 frontier 超时启发式，连续三面墙覆盖率判据尚未实现；这不影响控制链打通，但影响最终采空区完整建模验收。

### 本轮收尾状态

- 本轮新增源码、配置、README 和对话记录已推送 GitHub `main`，提交为 `97b5edc feat: connect task-one SUPER loop to PX4`。
- 收尾复查仍只有 Livox 发现端口 `56000` 被驱动绑定，`/livox/lidar` 无新消息；PX4 同时保持 `connected=true`、`ALTCTL`、`armed=false`，任务一 `command_ready=false`，没有向 PX4 发送设定值。
- 下一步需要用户保持当前 Livox 驱动运行，将 MID360 单独断电再上电一次，然后立即复查 `56201/56301/56401` 端口、点云、IMU、Fast-LIO2 位姿和注册点云。

## 第42轮：再次确认 MID360 当前状态

用户询问雷达是否仍未调通。实时复查确认：`192.168.1.157` 仍处于 `REACHABLE`，说明雷达网络在线；但 Livox 驱动仍只绑定 UDP 发现端口 `56000`，未绑定 `56201/56301/56401`，且 `/livox/lidar` 连续检查无新消息。因此目前是“雷达硬件与 NUC 网络已通，Livox ROS 驱动尚未完成重连握手”，还不能称为 ROS 雷达链路已调通。下一步仍是保持驱动运行并将 MID360 单独断电再上电。

## 第43轮：MID360 重新上电后修复 SDK2 接收状态并恢复 Fast-LIO2

用户说明雷达此前已经打通过，并已重新给雷达上电，询问这次具体出了什么问题。

### 原因定位

- 此前确实完成过全链路实测；本次不是首次配置失败，而是 Livox 驱动被重启后，旧 Fast-LIO2/雷达会话状态没有一起重建。
- 雷达重新上电后，`.157` 可达且持续向 NUC 的 `56301/56401/56201` 发送数据；旧版 SDK2 配置只监听发现端口 `56000`，没有预先接管数据端口。
- 把配置改为显式单雷达 IP 后，SDK2 能识别序列号 `ARMCP7Q0030457`、收到发现响应和命令 ACK，也能绑定 `56101/56201/56301/56401`；但该固件/SDK 组合卡在“更新配置回调完成后才将设备标记为 Sampling”的状态，导致已有数据仍不发布到 ROS。

### 修复内容

- 将 `MID360_config.json` 的 `host_net_info` 改为 SDK2 新数组格式，并写入 `lidar_ip: [192.168.1.157]` 和 `host_ip: 192.168.1.10`，同时把 `lidar_configs.ip` 从错误的 `.5` 改为 `.157`。
- 在 `livox_ros_driver2/src/lds_lidar.cpp` 中，对配置文件明确指定的雷达 handle 在初始化时设为 `kConnectStateSampling`；该处理只作用于显式列出的设备，使已在正确端口持续采样的 MID360 不再因为遗漏配置回调而阻塞 ROS 发布。
- `livox_ros_driver2` 重新编译成功。为便于恢复环境，仓库增加 `patches/livox_ros_driver2_mid360_static_ip.patch`。

### 恢复验证

- `/livox/lidar` 恢复到约 `10.00 Hz`，`/livox/imu` 恢复到约 `200 Hz`。
- `/Odometry` 恢复到约 `10.00 Hz`，`/cloud_registered` 恢复到约 `10.01 Hz`。
- 因旧 Fast-LIO2 在长时间断流后恢复时已发散到约十万米，位姿桥正确触发 `INPUT_JUMP` 并停止向 PX4 发送错误外部视觉。
- 停止发散的 `/laserMapping`，在稳定雷达数据上重新启动 Fast-LIO2；初始化后位姿恢复到起点附近，5 次位置抽样约在 `(0.017~-0.020, -0.015~-0.010, -0.061~-0.059) m`，没有继续发散。
- 在 PX4 `armed=false` 状态调用位姿桥重置服务，重新捕获对齐。最终 `/mine_uav/task1/vision_status=STREAMING`、`vision_healthy=true`，`/mavros/vision_pose/pose_cov` 稳定为 `30 Hz`。
- PX4 仍为 `connected=true`、`ALTCTL`、`armed=false`；RC CH6 当前约 `1499`，调度器保持 `hold`；`command_ready=false`，`/mavros/setpoint_raw/local` 没有输出。恢复过程没有控制无人机。

## 第44轮：手动飞行与自动任务选择的遥控器分层方案

用户提出：原来的二段开关只能选择任务一或任务二，但系统还需要手动飞行，询问是否应把三种状态放进一个三段开关，或另外使用一个通道切换自动/手动。

### 结论

- 不建议把“手动、任务一、任务二”合并到同一个三段开关。这样会把 PX4 飞行控制权切换和 NUC 任务切换耦合在一起，容易在任务一与任务二之间直接误切，也不利于故障时快速回到人工控制。
- 推荐采用两个彼此独立的逻辑：PX4 原生飞行模式负责“人工控制还是 OFFBOARD 自动控制”，NUC 任务调度器只负责“任务一还是任务二”。
- 当前任务二尚未实现，因此即使任务选择开关落在任务二，也必须保持 `HOLD/disabled`，不能输出任务二飞行指令。

### 当前实机只读检查

- PX4 在线、未解锁，当前为 `ALTCTL`。
- `RC_MAP_FLTMODE=5`：物理 CH5 已是 PX4 原生飞行模式通道；当前三档分别配置为 `STABILIZED / ALTCTL / POSCTL`，还没有 OFFBOARD 档。
- 现有 NUC 调度器使用物理 CH6 选择任务一/任务二；当前 CH6 约为 `1499`，仍位于中间无效区。
- `RC_MAP_OFFB_SW=0`：目前没有分配独立的 OFFBOARD 开关；因此可以从空闲通道中选择一个二段开关专门映射 OFFBOARD，而不占用 CH6。
- `RC_MAP_KILL_SW=15`：物理 CH15 已映射为电机紧急停止，不能拿来做自动/手动选择。
- `COM_RC_OVERRIDE=3`、`COM_RC_STICK_OV=30%`：当前 PX4 v1.17 配置允许在自动和 OFFBOARD 模式下通过较大摇杆动作切回位置控制（位置不可用时退回高度控制），但仍需在拆桨测试中实际验证。
- `COM_OF_LOSS_T=1.0 s`、`COM_OBL_RC_ACT=0`：OFFBOARD 设定值丢失 1 秒后，PX4 在遥控可用时切到 Position；`NAV_RCL_ACT=3`、`COM_RCL_EXCEPT=0` 表示 RC 丢失保护没有在 OFFBOARD 中被豁免。参数含义和实际动作仍需拆桨及系留测试确认。

### 推荐通道布局

- CH5：保留 PX4 手动/辅助飞行模式选择，作为人工飞行和回退通道。
- CH6：只做 NUC 任务选择，低档任务一，高档预留任务二；任务切换本身不自动解锁、不自动进入 OFFBOARD。
- 另选一个未占用二段通道（候选 CH7 或 CH8，必须通过拨动遥控器实测确认）：低档为人工控制/禁止自动，高档为允许并请求 OFFBOARD。
- CH15：保留紧急停止，不改动。

### 后续软件状态机

1. `MANUAL_DISABLED`：自动开关低，NUC 不允许任务轨迹控制；PX4 由 CH5 对应的人工模式控制。
2. `AUTO_PRESTREAM`：自动开关高且定位、通信、规划器均健康，NUC 先以高于 2 Hz 持续发送“当前位置悬停”设定值，满足 PX4 进入 OFFBOARD 前必须已有设定值流的要求。
3. `AUTO_ACTIVE`：PX4 已确认进入 OFFBOARD 后，才把所选任务的轨迹送入 PX4；绝不由 NUC 自动解锁。
4. `FAULT/HOLD`：规划器短暂停顿时发送当前位置悬停；定位、RC 或通信等关键健康条件失败时停止任务轨迹并锁断，由 PX4 原生失效保护接管。

### 下一步

- 用户需要指定遥控器上准备作为“自动/手动”开关的物理开关，并依次拨到低、高档；通过 `/mavros/rc/in` 找出对应的空闲物理通道。
- 通道确认后，再修改任务调度器和指令桥，使其接入独立 `auto_enable`，并在不装桨状态验证“预发送悬停 → OFFBOARD → 任务一 → 人工接管”的完整状态转换。

## 第45轮：定位雷达反复重连故障的根因并补齐自动恢复生命周期

用户质疑每次重新插入后都出现雷达或定位问题，询问是否因为飞控没有断电、是否必须先插雷达再插飞控，并要求解决根因而不是继续依赖人工断电和临时补丁。

### 根因结论

- MID360 经以太网向 NUC 发送点云/IMU，PX4 经 CH340/TELEM2 向 NUC 发送 MAVLink；两条链路彼此独立。PX4 是否断电不会决定 Livox 驱动是否发布点云，因此不存在必须“先雷达、后飞控”的系统要求。
- 全部断开时实测雷达网口 `enp89s0=DOWN`，点云、IMU 和 `/Odometry` 均停止，位姿桥进入 `ODOMETRY_TIMEOUT`；MAVROS 同时为 `connected=false`。CH340 的 `/dev/ttyUSB0` 仍存在，说明 USB 转串口本体仍插在 NUC，飞控侧断电与串口设备枚举是两件事。
- 上一次雷达恢复后，Livox 驱动已经发布数据，但长期存活的 Fast-LIO2 出现大量 `Too few input point cloud`、`No Effective Points`。源码检查确认 Fast-LIO2 在点云或 IMU 时间回跳时只清输入缓冲区，不会重置 EKF、地图和 IMU 状态；因此雷达断流/重启后的新数据可能接到旧估计器状态上，随后出现大位姿跳变并被位姿桥以 `INPUT_JUMP` 拦截。
- 初次 Livox 不发布问题来自 SDK 异步设备/配置回调未把设备状态推进到 Sampling。先前“初始化时静态强制 Sampling”虽然恢复了数据，但状态依据不够严格，本轮已替换为真实数据包驱动的状态转换。

### 根因修复

- `livox_ros_driver2` 不再在初始化时无条件把静态 IP 设备设为 Sampling；只有真正收到该 MID360 的有效点云或 IMU 包后，才设置 `kConnectStateSampling`。因此设备重连不再依赖可能漏掉的异步配置回调，同时雷达不存在时不会伪装成正在采样。
- MID360 配置保持雷达 `192.168.1.157`、主机 `192.168.1.10`，并按 Livox-SDK2 1.4.3 使用数组式 `host_net_info` 与 `multicast_ip`。
- Fast-LIO2 新增点云/IMU 时间连续性检查，默认任一数据流间隔超过 `1.0 s` 或时间戳回跳就主动正常退出；`mapping_mid360.launch` 配置 `respawn=true`、延迟 `1.0 s`，从而自动创建全新的 EKF 和地图，不再跨雷达断电延续旧状态。
- 位姿桥新增断流恢复处理：若 `/Odometry` 超时后恢复，只有 PX4 未解锁时才自动清除旧跳变锁存并重新捕获对齐；若 PX4 已解锁，则拒绝重建坐标系并保持视觉输出关闭。

### 验证结果

- `fastlio2_ws` 和 `super_ws` 均重新编译成功。
- 隔离测试向 Fast-LIO2 输入时间为 `100 s`、`102 s` 的两帧 IMU，进程识别 `2.000 s` 中断后正常退出。
- `roslaunch` 自动恢复测试中，旧 PID `126692` 正常结束，1 秒后以新 PID `126804` 拉起，证明新 EKF/地图进程自动重建有效。
- 位姿桥隔离测试模拟断流后 Fast-LIO 原点由 `0 m` 变为 `100 m`；桥在 PX4 未解锁时自动重建为 `-100 m` 对齐，没有触发 `INPUT_JUMP`。
- 可复用变更保存为 `patches/livox_ros_driver2_mid360_reconnect.patch` 和 `patches/fast_lio2_sensor_restart.patch`，两个补丁均通过反向应用检查。

### 下一次实物复测方法

- 为便于定位，下一次可以先只接通 MID360，确认 `/livox/lidar≈10 Hz`、`/livox/imu≈200 Hz`、`/Odometry≈10 Hz` 且位姿稳定，再给 PX4 上电并确认 MAVROS；这只是分段验收方法，不是永久启动顺序要求。
- 后续还要做一次真实的“运行中断开 MID360 → 等待数秒 → 重新上电”测试，验证驱动恢复、Fast-LIO PID 自动变化、位姿桥在未解锁时重新对齐，以及全程不向 PX4 输出任务控制。
- 对 TELEM2 航插/杜邦连接，最安全做法仍是断电接线后再上电；但 MAVLink 软件链路本身应能重连。PX4 Type-C 的旧 `error -71` 属于另一条物理 USB 问题，与 MID360 重连无关。

## 第46轮：确认实物为 Mid360s 并打通雷达至 Fast-LIO2 全链路

用户确认 PX4 与 NUC 链路已经通过，要求本轮只调试已重新接入的雷达与 NUC。

### 分层诊断结果

- NUC 有线网口 `enp89s0` 为 `UP/LOWER_UP`，地址为 `192.168.1.10/24`；雷达 `192.168.1.157` 连续 3 次 ping 均成功，丢包率为 0%，平均时延约 1.61 ms。
- 使用原 `msg_MID360.launch` 时，驱动只打开发现端口 `56000`，没有打开 `56101/56201/56301/56401`，`/livox/lidar` 与 `/livox/imu` 均无数据。
- 对 SDK 网络系统调用进行跟踪后确认：NUC 每秒向 `255.255.255.255:56000` 发出 24 字节发现请求；雷达 `192.168.1.157` 每次均返回 48 字节有效应答，序列号为 `ARMCP7Q0030457`。因此供电、网线、IP和雷达网口均正常。
- 发现应答中的设备类型为十进制 `35`。本地 Livox-SDK2 定义 `35 = kLivoxLidarTypeMid360s`，而原启动文件使用的 `MID360` 类型为 `9`。SDK收到应答后因型号与配置不匹配而静默忽略，这就是“能 ping 通但没有点云”的直接根因。

### 修正与实机结果

- 改用 `msg_MID360s.launch` 和 `MID360s_config.json`，其中NUC地址为 `192.168.1.10`、雷达地址为 `192.168.1.157`。
- 驱动随即建立全部数据通道，实测 `/livox/lidar ≈ 10.00 Hz`、`/livox/imu ≈ 200.0 Hz`。
- 启动 Fast-LIO2 后完成 IMU 初始化；`/cloud_registered ≈ 10.00 Hz`、`/Odometry ≈ 10.00 Hz`，里程计位置处于起点附近且数值正常。
- ROS连接关系显示 `/laserMapping` 正在订阅 `/livox/lidar` 和 `/livox/imu`，同时 `/super_exploration_decider` 已订阅 `/cloud_registered` 与 `/Odometry`，`/mission_scheduler` 已订阅 `/Odometry`。因此“雷达 → Livox Driver → Fast-LIO2 → 决策器/调度器”的数据链路已实机打通。

### 持久修正

- `task1_real.launch` 的默认雷达启动项改为 `msg_MID360s.launch`，避免后续重新启动时再次选错型号。
- 雷达补丁改名为 `livox_ros_driver2_mid360s_reconnect.patch`，配置目标改为 `MID360s_config.json`；保留真实数据包驱动 Sampling 状态和 Fast-LIO2 断流重启保护。
- 当前Livox Driver和Fast-LIO2保持运行。下一项硬件验收应是运行中单独给雷达断电再上电，验证自动恢复；该测试与PX4无关。

## 第48轮：手动打开RViz查看实时点云

用户关闭了自动打开的RViz，询问如何自行重新打开。

### 打开命令

新建终端后执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/fastlio2_ws/devel/setup.bash
rviz -d /home/nuc/fastlio2_ws/src/FAST_LIO/rviz_cfg/loam_livox.rviz
```

该配置默认使用 `camera_init` 作为 Fixed Frame，并显示 `/cloud_registered`、`/Odometry` 等Fast-LIO2输出。关闭RViz不会停止雷达驱动或Fast-LIO2。

若画面为空，在RViz左侧确认 `Global Options -> Fixed Frame` 为 `camera_init`，并确认PointCloud2显示项的Topic为 `/cloud_registered`、Enabled已勾选；按 `F` 可让视角聚焦当前点云。


## 第47轮：确认雷达数据内容及其是否到达自主决策器

用户询问如何查看雷达传来的数据，以及如何确认数据已经送达自主决策器。

### 数据链路与现场证据

- 雷达驱动输出 `/livox/lidar`，消息类型为 `livox_ros_driver2/CustomMsg`；它包含单点的三维坐标、反射强度、线号和相对时间等原始测量。现场抽取的一帧 `point_num=19968`、`lidar_id=192`。
- 雷达IMU输出 `/livox/imu`，消息类型为 `sensor_msgs/Imu`。
- Fast-LIO2融合原始点云和IMU后输出 `/cloud_registered`（`sensor_msgs/PointCloud2`）和 `/Odometry`（`nav_msgs/Odometry`）。现场配准点云一帧宽度为 `3520`，坐标系为 `camera_init`。
- `/super_exploration_decider` 明确订阅 `/cloud_registered` 和 `/Odometry`；ROS连接详情显示两个话题均由 `/laserMapping` 通过实际TCPROS入站连接送入决策器。`rostopic info`也显示发布者为 `/laserMapping`、订阅者为 `/super_exploration_decider`，因此数据已真实到达，不只是存在同名话题。

### 当前为什么没有输出探索目标

- 决策器状态为 `DISABLED`，调度器信号 `/mine_uav/mission/goaf_enable=False`。
- 这表示“雷达→Fast-LIO2→决策器”的接收链路正常，但任务一尚未被调度器允许执行；因此当前不会生成新的 `/goal` 或前沿点。
- `finished=False`、`returning=False`，当前不是建模完成或返航状态。

### 常用自检命令

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/fastlio2_ws/devel/setup.bash
source /home/nuc/super_ws/devel/setup.bash

rostopic hz /livox/lidar
rostopic echo -n 1 /livox/lidar/point_num
rostopic hz /cloud_registered
rostopic echo -n 1 /cloud_registered/width
rostopic info /cloud_registered
rosnode info /super_exploration_decider
rostopic echo /mine_uav/exploration/status
rostopic echo /mine_uav/mission/goaf_enable
```

- 判断标准：话题有稳定频率、每帧点数大于零、`rostopic info`能看到决策器作为Subscriber，三项同时成立才能确认数据已送达决策器。
