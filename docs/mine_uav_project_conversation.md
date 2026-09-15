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

## 第49轮：点云可视化通过后的下一步

用户确认能够继续，询问接下来进行什么工作。

### 推荐顺序

1. 先完成Mid360s运行中断电重连验收：保持NUC、ROS和网线不变，只关闭雷达电源5～10秒后重新上电，检查Livox Driver数据通道、Fast-LIO2进程重建、点云和里程计恢复。
2. 重连通过后进行任务一静态闭环：允许采空区决策器运行，观察前沿点、下一观察点和 `/goal`，确认SUPER接收目标并产生轨迹。
3. 在指令桥输出门保持关闭的情况下检查SUPER轨迹转换结果，确保不会控制PX4。
4. 最后才在拆桨、未解锁条件下验证坐标对齐、设定值预发送和OFFBOARD切换。

当前立即执行第1步。操作要求：只断Mid360s电源，不关闭NUC、不停止ROS进程、不拔网线；等待5～10秒后重新上电并通知检查。该步骤用于验证本轮加入的自动恢复机制，而不是规定永久上电顺序。

## 第50轮：Mid360s运行中断电重连实机验收

用户按要求在系统运行期间重新给Mid360s上电，要求检查自动恢复结果。

### 验收结果

- 雷达 `192.168.1.157` 连续3次ping成功，丢包率0%，平均时延约1.53 ms；NUC上的 `56000/56101/56201/56301/56401` 端口均保持建立。
- Livox Driver PID保持为 `129353`，说明驱动没有依赖人工重启，而是在同一进程内重新连接雷达。
- Fast-LIO2 PID从断电前的 `129676` 变为 `134012`；roslaunch日志在 `21:25:41` 明确记录 `laserMapping` 进程自动重启。
- 恢复后 `/livox/lidar≈10.00 Hz`、`/livox/imu≈200.05 Hz`、`/cloud_registered≈10.00 Hz`、`/Odometry≈9.99 Hz`，四段数据流均稳定。
- 新里程计位置约为 `(0.0228, 0.0159, -0.0611) m`，仍在起点附近，没有沿用旧估计器后出现发散。
- 自主决策器已自动重新建立来自新 `/laserMapping` 进程的 `/cloud_registered` 和 `/Odometry` TCPROS入站连接；决策器仍为 `DISABLED`，断电恢复期间没有启动自主任务。

### 结论与下一步

- “雷达断电→重新上电→Livox Driver恢复→Fast-LIO2新估计器启动→决策器重新订阅”的自动恢复链路实机验收通过。
- PX4视觉桥状态话题本次没有返回样本，下一步在任务一静态闭环中检查并启动位姿桥；随后在PX4指令门关闭的条件下启用决策器，观察前沿点、目标点和SUPER轨迹。

## 第51轮：任务一真实点云静态规划闭环测试

用户要求继续下一步。本轮在不控制PX4的条件下，使用Mid360s真实点云和Fast-LIO2里程计验证“决策器→SUPER”的静态闭环。

### 安全前提

- 调用 `/super_px4_command_bridge/enable=false`，返回“task-one PX4 command output disabled”。
- 测试前后 `/mavros/setpoint_raw/local` 均无任何消息；MAVROS当前无飞控心跳，调度器保持 `hold`。
- 测试结束后再次关闭决策器和指令桥，并停止临时 `fsm_node`。

### 已通过的链路

- SUPER `fsm_node` 正确加载 `click_smooth_ros1.yaml`，实际订阅 `/cloud_registered`、`/Odometry` 和 `/goal`；点云与里程计输入约10 Hz。
- 决策器重置启用后进入 `EXPLORING`，生成大量frontier候选，当前选择的目标点为 `(5.25, 3.75, 1.25) m`，坐标系为 `camera_init`，朝向基本保持当前机头方向。
- SUPER收到 `/goal` 后首次记录 `GenerateExpTrajectory SUCCESS`，状态由 `INIT→WAIT_GOAL→GENERATE_TRAJ→FOLLOW_TRAJ`，并输出 `/planning/pos_cmd`。这证明“真实雷达→Fast-LIO2→自主决策器→目标点→SUPER→轨迹指令”的接口链路已经贯通。

### 暴露的问题与边界

- 无人机在台架上没有移动，但SUPER的轨迹时钟继续前进，实际里程计无法跟踪期望轨迹，随后出现大量重复重规划、`Yaw rate too large`、`Omg or thr or Pos violation`、备份轨迹优化失败等警告。
- 这些警告主要来自“输出轨迹但机体完全不跟随”的开环静态条件，不能仅靠原地测试判断真实闭环稳定性；同时也说明当前尚不能带桨实飞，必须继续做运动闭环和参数验证。
- 当前任务一完成程度是数据/接口闭环通过，不是动态控制闭环通过。

### 下一步

- 优先在PX4 SITL中让模拟机体真实跟随SUPER指令，验证目标方向、轨迹连续性、重规划频率、避障和到达目标后的下一frontier切换。
- 动态闭环通过后，再连接实物PX4检查视觉位姿桥；最后才进行拆桨OFFBOARD台架测试。

## 第52轮：静态规划通过后的下一步

用户询问下一步具体应做什么。

### 下一步结论

- 下一步是搭建并运行“任务一PX4 SITL动态闭环”，暂不进行实机带桨测试。
- 需要让Gazebo中的模拟无人机根据SUPER输出真实移动，并把随运动变化的模拟里程计和点云重新送回决策器与SUPER，形成闭环。

### 执行顺序

1. 启动PX4 SITL、Gazebo和MAVROS UDP链路。
2. 将Gazebo模拟里程计适配成系统使用的 `/Odometry`，并准备可随无人机运动更新的模拟点云输入。
3. 启动任务调度器、采空区决策器、SUPER和PX4指令桥；在SITL中完成设定值预发送和OFFBOARD切换。
4. 观察无人机是否朝机头优先方向的frontier飞行、是否跟随连续轨迹、遇障碍是否重新选择目标、到达目标后是否选择下一frontier。
5. 统计重规划失败、偏航角速度超限和轨迹优化失败；只有在模拟机体正常跟随后仍持续出现的警告，才作为SUPER参数或实现问题修复。

### 通过标准

- 模拟机体能进入OFFBOARD并保持稳定；位置误差不持续扩大。
- SUPER轨迹连续且速度/加速度不超过当前限制。
- 目标点、轨迹、里程计和点云形成持续反馈，而不是台架测试中的开环状态。
- 任务切换、定位失效或轨迹超时时能够回到HOLD，且不会继续发送失控目标。

## 第53轮：完成任务一PX4/Gazebo动态闭环并建立强制同步约束

用户要求继续之前的工作，并增加固定前提：如果本次5小时token额度即将耗尽，必须先整理此前尚未更新的项目内容并上传到GitHub，不能让未同步工作因会话额度结束而丢失。

### 固定工作约束

- 后续工作应持续维护本对话记录；出现额度临近、长时间测试或其他可能中断会话的情况时，优先停止扩展开发，整理工作区源码、README和对话记录并提交、推送到 `dazhihe7005/-frontier-`。
- 本轮不等待额度临界点，任务一动态闭环验收完成后立即同步当前全部未上传内容。

### 新增完整SITL链路

- 新增 `scripts/task1_sitl_adapter.py`，把PX4 SITL的 `/mavros/local_position/odom` 转换成与Fast-LIO2一致的 `/Odometry`，并以 `camera_init` 为统一坐标系。
- 适配器按无人机实时位置生成 `/cloud_registered`，模拟入口开放、左右墙、尽头墙和顶板；同时发布全局模拟点云和飞行路径，供RViz观察。
- 适配器模拟物理CH6选择任务一、CH7先低后高允许自动任务。它只在 `/use_sim_time=true` 时允许自动解锁PX4，并明确禁止连接真实飞控。
- 新增 `worlds/goaf_mine.world` 和 `launch/task1_px4_sitl.launch`，一次启动Gazebo采空区、PX4 SITL、MAVROS、调度器、模拟Fast-LIO2接口、决策器、SUPER、坐标安全指令桥和最终MAVROS转发。

### 方向证据和返航安全修复

- 将点云方向判断的垂直容差收紧为 `1.5 m`，避免把顶板当成侧墙或前墙。
- 新增独立 `front_obstacle_sector_deg=24°`，不再复用较宽的 `forward_sector_deg=70°`；实测消除了左右墙过早触发“前方障碍”的主要问题。
- 新增 `return_home_height_offset`。SITL使用 `1 m` 安全返航高度，返航完成距离也改为相对该抬高后的目标计算，避免返回地面原点时触地。
- 指令桥订阅 `/mine_uav/exploration/finished`；任务完成后锁止新任务指令并退出受管OFFBOARD。故障和退出日志也改为只在状态首次变化时输出，避免重复刷屏。
- 真机默认退出模式仍为 `POSCTL`，用于遥控接管；无遥控输入的SITL中，PX4虽然会接受POSCTL请求但不会实际切换，因此SITL专门覆盖为 `AUTO.LOITER`。

### 动态闭环验收结果

- PX4成功解锁、进入OFFBOARD并跟踪SUPER轨迹；决策器依次发布机头优先观察点，在前障碍确认后切换frontier fallback，再发布返航目标。
- 本轮第二次完整运行的观察点包括约 `(3.25,-0.25,1.25)`、`(6.75,-0.75,1.75)`、`(10.75,-2.25,0.75)`、`(14.75,1.25,1.75)` 和 `(16.75,-0.75,0.75) m`，随后返回约 `(0,0,0.99) m`。
- 最终飞机稳定在约 `(0.13,0.05,1.01) m`；`/mine_uav/exploration/finished=true`，指令桥状态为 `TASK1_COMPLETE`，`command_ready=false`，PX4实际模式为 `AUTO.LOITER`。
- 完成后 `/mavros/setpoint_raw/local` 不再有新消息；模拟 `/cloud_registered` 和 `/Odometry` 均稳定为 `10 Hz`。
- `catkin_make --pkg mine_uav_control`、Python语法检查、launch/world XML检查全部通过；测试使用独立ROS Master `11312`，结束后PX4 SITL、Gazebo和该ROS Master均已停止，没有接触默认 `11311` 的真机链路。

### 当前能力边界

- 本轮证明“任务调度→点云/位姿→自主决策→SUPER→PX4动态执行→安全高度返航→退出OFFBOARD”的仿真控制闭环成立。
- 仿真适配器模拟的是Fast-LIO2输出，不是Livox原始UDP和Fast-LIO2 EKF本体，因此不能替代真机外部视觉融合及雷达安装外参验收。
- 当前建模完成条件仍是“至少完成观察点后，持续无新frontier”的启发式判据；用户要求的左右墙加尽头墙三面连续覆盖判定尚未实现，是任务一下一项核心算法工作。
- SUPER在部分转弯和重规划过程中仍会出现偶发优化/角速度告警，应通过仿真日志统计和参数整定继续降低，暂不能据此宣称可以直接带桨下井飞行。

## 第54轮：实现采空区三面墙完整建模判据并完成PX4动态仿真

用户询问何时可以进行采空区仿真并要求直接实现。本轮在已有PX4/Gazebo动态闭环基础上，实现并验证“左右墙体与尽头墙体连续建模完成后才返航”的任务语义。

### 三面墙完成算法

- 任务开始时锁定无人机进入采空区的初始机头方向，建立任务纵向轴和横向轴；SUPER在飞行中改变机头姿态不会改变“采空区深处、左墙、右墙、尽头墙”的定义。
- 从决策器的占据体素地图中排除地面和顶板，只统计配置高度范围内的墙面体素。
- 左右墙沿纵向按 `1 m` 分箱，分别计算从入口到尽头的覆盖率和最大连续缺口；当前阈值为左右覆盖率均不低于 `80%`，连续缺口不超过2个分箱。
- 尽头墙候选必须位于最小探测深度之外，同时包含横向中心回波并达到最小横向跨度，避免仅由左右两条侧墙在同一纵向位置造成假尽头。
- 飞机还必须进入距尽头墙 `6 m` 的接近范围，并连续4个决策周期满足全部条件，才发布 `/mine_uav/exploration/model_complete=true` 并返航。
- 默认启用 `require_three_wall_completion=true` 后，无frontier超时不再单独触发建模完成；外部返航请求和低电量仍保留更高优先级。

### 尽头接近策略

- 首次仿真发现：雷达从约 `13.5 m` 位置已经看见 `22.5 m` 的尽头墙和完整左右墙，普通frontier因此消失，但飞机尚未满足接近尽头条件。
- 新增 `three_wall_end_approach` 目标：当三面墙拓扑已经识别、侧墙连续但飞机离尽头仍较远时，主动生成距尽头墙 `3 m` 的中心观察点。
- 同时将机头优先方向从“实时机头yaw”改为“进入采空区时锁定的yaw”，避免SUPER转向后任务主方向跟着旋转，确保目标总体持续向采空区深处推进。

### 可观测接口

- 新增 `/mine_uav/exploration/model_coverage`，实时报告尽头是否识别、尽头深度、飞机纵向进度、左右墙覆盖率、尽头横向跨度、最大缺口和确认周期。
- 新增 `/mine_uav/exploration/model_complete`，只表示三面墙几何覆盖判据已通过；原有 `/mine_uav/exploration/finished` 仍表示飞机已完成返航并到达home。
- 所有阈值均加入 `config/super_exploration_decider.yaml`，可按真实采空区宽度、长度、雷达有效距离和建模分辨率标定。

### 仿真验收结果

- PX4解锁后进入OFFBOARD，目标沿初始机头主方向依次向内推进；普通frontier结束后，决策器发布约 `(19.50, 0.00, 1.11) m` 的尽头接近观察点。
- 最终覆盖状态为：尽头深度 `22.50 m`、飞机最大纵向进度 `17.89 m`、左右墙覆盖率 `1.00/1.00`、最大连续缺口 `0/0`、尽头墙横向跨度 `9.00 m`、确认周期 `4/4`。
- 三面墙确认后返航目标约为 `(0,0,0.96) m`；返航完成后 `model_complete=true`、`finished=true`、指令桥 `TASK1_COMPLETE`，PX4从OFFBOARD切换至 `AUTO.LOITER`，后续无MAVROS setpoint输出。
- 第一次语义仿真正确拒绝了“墙已看见但飞机仅到13.5 m”的提前返航；第二次加入尽头接近策略后闭环通过，证明完成条件不是旧的无frontier超时。

### SUPER收尾门控

- 发现安全指令桥停止输出后，SUPER仍会围绕最后返航目标后台重规划并刷日志。
- 在SUPER ROS1接口中增加 `/mine_uav/exploration/finished` 订阅：完成后停止主规划、重规划和指令定时器；任务reset重新发布false时可恢复。
- 最终回归日志在仿真60秒明确出现 `SUPER planning paused: exploration mission complete`，之后无新的SUPER重规划记录。
- SUPER工作区的全部ROS1/Fast-LIO2适配修改已整理为 `patches/super_ros1_fastlio_task_gate.patch`，并通过反向应用检查，保证可以保存到项目仓库。

### 能力边界

- 当前判据确认的是决策器占据体素中的三面墙几何连续性，不等于雷达开发者的最终模型已达到点云密度、配准误差、孔洞率或工程精度要求。
- 本轮点云由SITL适配器直接模拟Fast-LIO2输出，未仿真Livox原始UDP、IMU噪声、Fast-LIO2漂移和真实粉尘/弱纹理环境；这些需要后续噪声场景和实机测试。
- 部分轨迹转弯或返航初期仍存在SUPER优化告警，但最终轨迹执行、返航、模式退出和完成后停止规划均成功；下一步应针对告警做参数统计与整定，并增加缺墙、断墙和不规则墙面的负例仿真。

## 第55轮：区分OFFBOARD断流故障与任务完成后的正常停流

用户在采空区SITL完成后看到 `Setpoint command is ... old; stopping publication of old target` 每秒持续出现。定位确认该日志来自最末层 `offboard_bridge`，不是雷达、Fast-LIO2、SUPER或PX4自身故障。

### 原因

- 三面墙建模完成并返航后，指令桥按设计停止发布 `/mine_uav/setpoint_cmd`，并把PX4从OFFBOARD切换到SITL使用的 `AUTO.LOITER`。
- 旧版 `offboard_bridge` 不订阅PX4模式，只知道最后一条指令持续变旧，因此即使PX4已经不依赖外部setpoint，仍每秒重复输出陈旧指令告警。
- 日志中的“stopping publication”本身说明安全超时正在生效，没有继续向PX4转发旧目标；问题是任务完成后的重复告警语义不准确。

### 修复与验证

- `offboard_bridge` 新增 `/mavros/state` 订阅和可配置的 `offboard_mode`（默认 `OFFBOARD`）。
- PX4处于OFFBOARD或尚未取得MAVROS状态时，缺失/陈旧指令仍会告警并停止转发，保留真实飞行中的断流保护。
- PX4已经切换到 `AUTO.LOITER`、`POSCTL` 等非OFFBOARD模式时，陈旧上游指令属于正常收尾，节点静默停止转发。
- `mine_uav_control`重新编译通过。隔离ROS Master最小测试中，模拟OFFBOARD后旧指令持续触发告警；切换为AUTO.LOITER后立即无新增告警，证明没有掩盖OFFBOARD飞行中的真实断流。

## 第56轮：用户首次手动启动完整采空区SITL并确认结果

用户按说明重新启动采空区仿真。检查独立ROS Master `11312` 后，PX4 SITL、Gazebo、MAVROS、任务调度器、SUPER、决策器和两级PX4指令桥等关键节点均正常在线。

本次运行已经自动完成整个任务一闭环：

- PX4连接、解锁并在任务阶段进入OFFBOARD，任务结束后切换为 `AUTO.LOITER`。
- 三面墙覆盖状态为：尽头深度 `22.50 m`、最大纵向进度 `17.89 m`、左右墙覆盖率 `1.00/1.00`、最大缺口 `0/0`、尽头跨度 `9.00 m`、确认周期 `4/4`。
- `/mine_uav/exploration/model_complete=true`，`/mine_uav/exploration/finished=true`，指令桥状态为 `TASK1_COMPLETE`。
- 最终PX4局部位置约为 `(-0.30, 0.09, 0.97) m`，已回到入口附近并保持悬停。

用户若要观察完整运动过程，需要保留端口11312的roscore，停止当前launch后重新运行 `task1_px4_sitl.launch gui:=true rviz:=true`；在RViz中重点显示当前点云、累计点云、frontier标记和PX4轨迹。

## 第57轮：加入随无人机运动的Gazebo MID360风格三维雷达

用户反馈RViz没有看到点云，并要求把雷达模块加入仿真。检查确认旧 `/cloud_registered` 实际稳定为10 Hz且frame为 `camera_init`，但旧RViz配置的Fixed Frame为 `world`，不存在对应TF，所以点云可能完全不可见；同时旧闭环点云是解析环境生成，并非Gazebo机载传感器。

### 新增传感器与数据链

- 新增 `models/iris_mid360/iris_mid360.sdf`：在PX4 Iris顶部通过固定关节安装MID360风格Gazebo Ray传感器，水平360线、垂直32线、10 Hz、30 m量程、2 cm高斯噪声。
- 原始雷达输出 `/mine_uav/sitl/mid360/points`，类型为Gazebo block laser使用的 `sensor_msgs/PointCloud`，frame为 `mid360_link`。
- 新增 `gazebo_mid360_fastlio_adapter.py`，根据 `/Odometry` 的PX4位置和姿态以及0.14 m安装偏移，把机体系原始点云注册到 `camera_init`，输出Fast-LIO2兼容的 `/cloud_registered`（`PointCloud2`）。
- 适配器按0.2 m体素累计 `/mine_uav/sitl/global_cloud`，过滤0.3 m以内近场点和29.5 m以外无回波/最大量程点。
- 旧 `task1_sitl_adapter.py` 在完整SITL中关闭解析点云，只保留Fast-LIO2风格里程计、轨迹、RC开关模拟和SITL自动解锁功能，避免同一话题存在两个有效点云源。

### RViz与启动文件

- `task1_px4_sitl.launch` 默认加载新的Iris MID360 SDF、注册适配节点和专用 `rviz/task1_mid360.rviz`。
- 专用RViz的Fixed Frame为 `camera_init`，默认显示注册当前帧点云、累计点云、PX4路径和frontier，修复旧 `world` 固定帧造成的不可见问题。
- 该雷达层模拟真实几何遮挡、视场、量程和噪声，但不模拟Livox UDP包、MID360非重复扫描时序，也没有运行真实Fast-LIO2 EKF。

### 隔离验证

- 模型通过SDF解析检查，`mine_uav_control`编译通过。
- 在独立ROS Master 11314和Gazebo Master 11346中成功生成带雷达Iris；原始点云稳定10 Hz，每帧11520个射线点。
- 修正Gazebo block laser原始消息为 `sensor_msgs/PointCloud` 后，注册点云稳定约10 Hz，frame=`camera_init`；量程过滤后单帧约6540个有效点，累计地图正常增长。
- 用户当前11312仍运行旧Gazebo实例；机体传感器不能热加载，需要停止旧launch并重新启动后才能进行新的完整PX4闭环回归。

## 第58轮：按5h限额要求优先同步GitHub

用户提醒：当5h token限额接近耗尽时，必须至少提前2%整理并上传尚未同步的项目内容。本轮立即暂停继续仿真操作，检查到GitHub本地仓库比远端领先1个提交，包含MID360 Gazebo模型、注册适配器、RViz配置、启动文件、README和前序对话记录。

已通过SSH 443重试推送并确认 `origin/main` 与本地一致，最新提交为 `aa6bf94 feat: add Gazebo MID360 lidar to task1 SITL`。本轮后续工作在仓库安全同步完成的基础上继续。

## 第59轮：关闭旧采空区仿真进程

用户说明没有其他终端，希望关闭仿真进程。检查确认当前已不存在 `px4`、`gzserver`、`gzclient`、RViz 或 `task1_px4_sitl.launch` 进程，旧仿真已经退出，没有执行范围外的强制杀进程操作。后续可直接使用新启动文件加载MID360模型和专用RViz配置。

## 第60轮：重新启动带MID360雷达的采空区仿真步骤

用户询问关闭旧仿真后的下一步。重新启动时应先在终端一运行独立ROS Master `roscore -p 11312`，再在终端二加载ROS、SUPER、PX4 Gazebo环境并执行 `roslaunch mine_uav_control task1_px4_sitl.launch gui:=true rviz:=true`。新启动文件会加载 `iris_mid360.sdf`、Gazebo雷达原始话题、点云注册适配器和 `task1_mid360.rviz`；终端三可检查原始雷达点云、注册后的 `/cloud_registered`、PX4状态和三面墙完成状态。

## 第61轮：明确MID360风格雷达与Fast-LIO2的仿真边界

用户询问当前仿真雷达是否为MID360S以及是否使用Fast-LIO2。明确如下：

- 当前是Gazebo Ray传感器的MID360风格近似模型，不是实体Livox MID-360/MID360S、Livox SDK或真实UDP数据流；配置了360度水平扫描、32条垂直采样、10 Hz、30 m量程和高斯噪声。
- 当前没有运行Fast-LIO2。`gazebo_mid360_fastlio_adapter.py`只是把Gazebo `sensor_msgs/PointCloud`原始点云依据 `/Odometry` 和安装偏移刚性变换到 `camera_init`，重新发布为 `PointCloud2`；这不是Fast-LIO2的IMU预积分、特征处理、扫描配准或状态估计。
- 当前仿真验证的是“雷达几何数据→坐标注册→决策器→SUPER→PX4”接口和任务逻辑。要验证真实Fast-LIO2，后续应接入Livox驱动、IMU和Fast-LIO2节点，再让其原生输出 `/Odometry`、`/cloud_registered`，并关闭该仿真注册适配器以避免重复发布。

## 第62轮：新增复杂地形采空区仿真场景

用户希望在复杂地形中进行仿真测试。新增 `worlds/goaf_complex.world`，保留入口方向和连续三面墙拓扑，同时加入侧墙折线变化、支护柱、落石、低矮地面障碍和更长的30 m主巷道，用于测试真实雷达遮挡、frontier变化、绕障、机头方向优先和三面墙完成判据。

复杂场景不覆盖原 `goaf_mine.world`，通过 `world:=.../goaf_complex.world` 选择。场景SDF解析通过，规则和数据链代码不变；由于复杂场景需要重新加载Gazebo机体和PX4，必须先退出旧仿真再启动，完整PX4动态回归待用户重新启动后继续验证。

## 第63轮：完成复杂地形采空区任务一动态闭环验证

用户希望在复杂地形中运行仿真测试。本轮先停止上一轮故障锁存的旧仿真，重新编译带诊断信息的 `super_px4_command_bridge`，再在独立 ROS Master `11312` 上启动 `goaf_complex.world`。

### 仿真结果

- Gazebo成功加载30 m级复杂采空区、折线侧墙、支护柱、落石和低矮障碍；带机载MID360风格Ray传感器的Iris成功生成。
- 原始 `/mine_uav/sitl/mid360/points` 与注册后的 `/cloud_registered` 持续约10 Hz；适配器根据 `/Odometry` 和安装偏移完成 `camera_init` 坐标注册，累计地图最终约12万体素。
- 任务调度器选择任务一后，PX4成功解锁并进入 `OFFBOARD`；决策器沿机头主方向生成端墙接近目标，目标推进到约26 m后生成返航目标。
- 三面墙覆盖判据达到 `end_seen=true`、左右覆盖率 `1.00/1.00`、缺口 `0/0`、连续确认 `4/4`；随后发布 `model_complete=true` 和 `finished=true`。
- 飞机返回入口附近，指令桥状态为 `TASK1_COMPLETE`，PX4切换为 `AUTO.LOITER`，仿真过程未出现高度围栏故障。

### 对上一轮故障的结论

上一轮的 `HEIGHT_GEOFENCE` 是任务启动/规划时序下的偶发保护触发，不能归因于雷达断链；本轮重启后桥接状态正常。复杂场景中仍可看到SUPER在局部障碍附近尝试重规划，但最终闭环完成，因此当前验证覆盖“仿真雷达几何数据→位姿注册→决策器→SUPER→PX4→返航”的接口和任务逻辑，不代表真实MID360、Fast-LIO2和雷达建模精度已经验收。

本轮动态仿真启动命令：

```bash
roslaunch mine_uav_control task1_px4_sitl.launch \
  world:=/home/nuc/super_ws/src/mine_uav_control/worlds/goaf_complex.world \
  max_exploration_radius:=34.0 gui:=true rviz:=true
```

## 第64轮：修复复杂场景自体回波并复测任务一

用户报告SUPER日志持续出现 `GeneratePolytopeFromLine failed`，无人机在起飞区原地不动。
本轮通过当前SITL的原始和注册点云进行定位：

- Gazebo Ray雷达原始点云每帧约11520点；起点附近有约3960个0.8 m内的近点，最近距离约
  0.20 m，属于Gazebo Iris机体/起落架的自体回波，不是真实采空区障碍。
- 之前适配器的过滤半径0.55 m不足，注册点云仍出现距离飞机约0.576 m的机体回波，CIRI
  将其膨胀为不可行障碍，导致SUPER反复重规划。
- `gazebo_mid360_fastlio_adapter.py` 的仿真专用 `self_filter_xy_radius` 默认值及SITL
  启动参数改为0.80 m；真实Fast-LIO2数据不经过该过滤。
- 复杂世界中的低矮货架和落石调整到侧墙附近，保留复杂地形和遮挡效果，为主方向探测及
  返航留出中心通道；这只修改Gazebo测试世界，不改变真实任务算法。
- SITL启动文件的出生高度改为1.15 m，并在注释中说明真实无人机需要单独起飞阶段后才能
  开启任务一。注意PX4本地坐标会重新建立原点，出生高度不等于本地z坐标，因此它不能
  替代真实起飞状态机。

### 复测结果

- 过滤参数已在启动输出中确认：`self_filter_enable=true`、半径0.80 m；雷达注册点云
  约10 Hz，累计地图持续增长。
- PX4成功连接、解锁并进入 `OFFBOARD`；飞机从入口前进到约25.9 m。
- 三面墙判定达到 `end_seen=true`、左右覆盖率 `1.00/1.00`、缺口 `0/0`、确认
  `4/4`，决策器发布 `RETURNING` 并进入返航。
- 本轮对话被用户在返航等待阶段中断，返航尚未验证到起点，故不能把本轮记作完整
  `TASK1_COMPLETE`；第63轮已有一次完整闭环记录。

本轮暂未修改真实PX4串口、TELEM2、Fast-LIO2或SUPER源码，只修改仿真适配器参数、复杂
Gazebo世界和SITL启动参数。

## 第65轮：完成安全返航高度下的复杂场景任务一闭环

用户要求继续之前的工作。本轮先检查到上一轮仿真仍在返航阶段，飞机在约x=17.8 m处
没有收到新的有效SUPER返航轨迹，指令桥处于 HOLD_COMMAND_TIMEOUT。分析确认：

- 返航目标已正确发布到 camera_init 的入口附近，但默认返航高度约0.97 m使SUPER
  返航轨迹下降到贴近地面的区域，复杂障碍和地面占据栅格导致反复重规划。
- 任务一SITL启动文件的 return_home_height_offset 改为1.8，返航阶段保持安全巡航
  高度；真实系统后续应把返航和降落拆成两个状态。
- 第一组支护柱再次移到靠近侧墙的位置，保留复杂场景遮挡但避免主通道和返航通道被
  支护柱引导进狭窄侧边区域。

### 最终验证

- PX4连接、解锁和 OFFBOARD 正常；仿真雷达注册点云保持约10 Hz，累计地图持续更新。
- 飞机前向探测进度约24.9 m，三面墙判据达到 end_seen=true、左右覆盖率
  1.00/1.00、缺口 0/0、连续确认 4/4。
- 决策器进入 RETURNING，飞机最终回到入口附近约
  (-0.27, 0.09, 1.78) m。
- 最终状态为决策器 COMPLETE、/mine_uav/exploration/finished=true、指令桥
  TASK1_COMPLETE，PX4切换为 AUTO.LOITER。

至此，当前Gazebo复杂采空区下“仿真雷达→点云注册→自主决策器→SUPER→PX4→三面墙
完成判定→安全高度返航”的任务一闭环已验证。该结论不等同于实体MID360、真实Fast-LIO2
和真实飞行安全验收；仿真自体回波过滤仍仅适用于Gazebo适配器。

## 第66轮：把2 m/s解释为流畅巡航目标并完成连续探索回归

用户进一步说明，期望不是把速度刚性锁死为2 m/s，而是无人机尽可能流畅地沿某个方向
探索，在开阔直线段接近2 m/s，遇到障碍、转弯和尽头时允许主动减速，不能在一个位置
长时间停留。实现和验证时因此不再把“瞬时速度始终等于2”作为目标。

### 本轮调整

- SUPER轨迹约束最终使用 `max_vel=2.0 m/s`、`max_acc=1.5 m/s²`、
  `max_jerk=20 m/s³`。曾测试的 `2/3/40` 组合导致跟踪过激和加速度保护触发，已放弃。
- 决策器增加8 m连续前视目标，并在距离目标3 m时提前交接下一目标，减少到点悬停后再
  规划的停顿。
- 前向目标按进入采空区时锁定的机头方向生成。选择顺序改为先遍历所有中心线前视距离，
  只有中心线均不可用时才考虑侧向偏移，避免因最远处单个占据体素提前横移。
- 前方封闭墙不再由单个点触发：要求至少20个回波点、横向跨度至少1.0 m、垂直跨度
  至少0.8 m，并保留多帧确认。
- SITL巡航目标高度保持在home以上约0.8 m的窄范围内，减少无意义升降和轨迹不可行。
- PX4命令桥的2.2 m/s与2.0 m/s²是异常指令保护阈值，不是飞行速度控制器；实际巡航
  速度仍由SUPER连续轨迹和PX4跟踪共同决定。

### 最终复杂场景动态结果

完整日志保存于本机 `/home/nuc/task1_logs/task1_continuous_run06.bag`，未加入Git仓库。
任务目标依次为 `(7.49, 0.00, 0.77) m`、尽头安全接近点
`(26.50, 0.00, 0.77) m`、入口返航点。PX4在仿真时间10.30 s进入OFFBOARD，42.30 s
退出到AUTO.LOITER；状态完整经过探索、尽头接近、三面墙建模完成、返航和COMPLETE。

- 有效航段实际速度平均1.77 m/s、中位数1.94 m/s、90分位2.04 m/s；
- 速度低于0.2 m/s的占比1.1%，最长连续低速仅0.30 s；
- 83.5%的有效航段速度高于1.5 m/s；
- SUPER规划速度最大2.001 m/s、规划加速度最大1.510 m/s²；
- 实际速度有短时2.48 m/s跟踪超调，因此不能把规划上限误解成实际速度硬限制；
- 横向位置范围为-0.43至0.36 m，路径长度54.10 m，无桥接限速、限加速度或围栏故障；
- 最终三面墙状态为 `end_seen=true`、左右覆盖率1.00/1.00、缺口0/0、确认4/4。

结论是当前复杂Gazebo场景已经实现“主方向连续前进、尽头确认、连续返航”，流畅性显著
优于逐frontier到点停顿模式。日志中仍存在SUPER备份轨迹/指数轨迹优化失败后快速重规划
的警告，虽然本轮未造成停飞，后续仍需量化重规划失败率并联合调整地图膨胀、优化器和
PX4跟踪参数。真机必须从较低速度和更大安全裕量开始逐级验证，不能直接照搬SITL速度。

## 第67轮：复杂采空区完整仿真操作步骤

用户要求给出当前任务一仿真的具体启动步骤。推荐使用独立ROS Master端口11312，避免
仿真MAVROS与默认11311端口上可能存在的真机节点串线。操作分为三个终端：终端一运行
`roscore -p 11312`；终端二设置相同的`ROS_MASTER_URI`，加载ROS、SUPER工作空间和
PX4 Gazebo Classic环境脚本，然后启动`task1_px4_sitl.launch`并选择
`goaf_complex.world`、34 m探索半径、Gazebo GUI和RViz；终端三加载相同环境后查看
PX4状态、决策器状态、三面墙覆盖率、目标、点云频率和桥接状态。

必须source具体文件
`/home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash`，不能source
`gazebo-classic/`目录，也不能把`setup_gazebo.bash`当作已安装的全局命令直接执行。
启动文件会自动运行PX4 SITL、Gazebo MID360风格传感器、点云注册适配器、任务调度器、
自主决策器、SUPER、MAVROS和PX4指令桥。仿真RC适配器会自动允许任务一、解锁并请求
OFFBOARD，不需要手工发送目标或模式命令。

成功现象依次为点云出现、`WAIT_DATA`、`EXPLORING`、PX4进入`OFFBOARD`、前向连续飞行、
`APPROACHING_END_WALL`、`MODEL_COMPLETE`、`RETURNING`、`COMPLETE`，最后PX4进入
`AUTO.LOITER`。结束时先在主launch终端按Ctrl+C，等待PX4和Gazebo退出，再停止roscore。

## 第68轮：复用11312上的现有ROS Master

用户执行`roscore -p 11312`时收到“another roscore/master is already running”。只读检查
确认11312端口已有健康的`roscore`（PID 15490）和`rosmaster`（PID 15519）监听，因此
不应重复启动第二个Master。Master中仅有`/gazebo_gui`和一个`/rostopic_*`旧注册，实际
进程均已不存在且节点连接被拒绝；已使用`rosnode cleanup`只清理这两个失效注册，没有
停止现有Master。用户可关闭本次报错终端，直接从仿真启动步骤的终端二继续，并统一设置
`ROS_MASTER_URI=http://127.0.0.1:11312`。`.ros/log`超过1 GB只是磁盘占用警告，与本次
Master冲突无关；本轮没有删除历史日志。

## 第69轮：任务一当前完成度评估

用户询问任务一当前完成进度。按研发层级评估，而不把SITL通过等同于可直接下井：NUC
任务一软件功能约完成85%，复杂Gazebo/SITL动态闭环约完成90%，真机数据与通信接口约
完成70%，真实自主飞行和矿井安全验收约完成35%至45%；综合工程完成度估计约70%。这些
百分比是按尚缺工作量和风险给出的工程估计，不是自动化测试覆盖率。

已经完成并有证据的部分包括：实体MID360/Livox Driver/Fast-LIO2输出的
`/cloud_registered`与`/Odometry`曾实机到达自主决策器和SUPER；CH340/TELEM2以
500000波特率完成MAVROS/PX4通信验证；任务调度、自动允许门控、机头主方向优先、8 m
连续前视与3 m提前目标交接、frontier备选、三面墙几何连续覆盖判据、尽头安全接近、
返航状态和SUPER到PX4指令桥均已实现。复杂SITL已完成“点云→决策→SUPER→PX4→尽头
确认→返航→AUTO.LOITER”闭环，最终日志中巡航速度中位数1.94 m/s、最长低速停顿
0.30 s、无桥接围栏或限幅故障。

尚未完成或不能宣称通过的部分包括：真实Fast-LIO2地图系与PX4本地NED系的整机外参和
长期漂移验收；PX4 EKF2实际融合外部视觉后的连续性和复位处理；真实桨动力下SUPER轨迹
跟踪；真实遥控器自动允许/任务选择通道标定和随时人工接管；独立起飞、降落、低电量、
定位失效、雷达失效、MAVLink断链及规划持续失败的完整安全状态机；真实采空区的碰撞
裕量、粉尘/弱纹理/重复结构和通信条件测试；雷达建模模块对最终模型密度、孔洞、配准
误差的质量验收。当前三面墙判据只能证明决策器地图中存在连续几何证据，不能证明雷达
开发者输出的最终三维模型质量完全合格。SUPER仍偶发轨迹优化失败后快速重规划，也需要
继续量化和调参。

推荐下一阶段按风险顺序推进：先拆桨完成真实全链统一启动与rosbag记录；标定整机外参并
比较Fast-LIO2和PX4位姿；验证EKF2外部视觉、坐标变换、时间戳和重定位；再在防护条件下
以0.3至0.5 m/s完成短距离前进、停止、返航和遥控接管；随后逐级提升至1.0、1.5和目标
巡航速度；最后进入可控矿井样场验证三面墙与最终模型质量。近期最高优先级不是继续增加
巡航速度，而是真机坐标/EKF闭环和故障接管。

## 第70轮：转入仿真开发并增加自动验收器

用户说明当前无法实机测试，要求先继续仿真部分。本轮未修改任何真机PX4、TELEM2、
Fast-LIO2或遥控器配置，优先把现有任务一SITL闭环变成可重复量化的测试基线。

新增`scripts/analyze_task1_sitl_bag.py`，读取轻量rosbag中的PX4模式、实际位姿/速度、
SUPER轨迹指令、任务目标、探索状态、三面墙覆盖状态、桥接状态和可选`/rosout_agg`。
验收项包括：进入与退出OFFBOARD、任务完成、三面墙ready、返航误差、横向偏移、路径
长度、巡航速度中位数、最长低速停顿、实际速度、规划速度/加速度、里程计频率、任务
时长和桥接故障。程序输出人类可读摘要和可选JSON，失败时返回非零退出码，便于后续做
自动回归。

使用第66轮完整日志`task1_continuous_run06.bag`验证，新验收器得到PASS：OFFBOARD阶段
32.0 s、路径54.095 m、水平返航误差0.269 m、最大横向偏移0.417 m、实际速度中位数
1.935 m/s、最长低速0.300 s、SUPER规划速度最大2.001 m/s、规划加速度最大
1.510 m/s²、里程计10.0 Hz、桥接故障0。该旧包没有录`/rosout_agg`，因此显示的SUPER
失败日志数为0只表示“包中没有该话题”，不能据此宣称优化警告已经消失；新的录包命令
已在README中加入`/rosout_agg`。

仿真后续顺序调整为：先用该验收器形成每次修改的对比基线；再采集并降低SUPER优化失败
与无效重规划频率；之后增加点云/里程计断流、任务取消和规划持续失败等故障注入回归。

## 第71轮：任务一SITL基线、SUPER诊断与A/B回归

继续只做仿真，不改真机配置。首先重新执行复杂采空区完整闭环并录入`/clock`和
`/rosout_agg`。验收器增加了rosbag接收时间到Gazebo仿真时间的插值映射，解决录包进程
先于仿真启动时，墙钟时间和仿真时间混用导致的时长误判。

原始SUPER流程的`task1_acceptance_run07.bag`通过验收：任务阶段32.000 s、路径
54.778 m、返航误差0.240 m、最大横向偏移0.606 m、速度中位数1.983 m/s、最长低速
停顿0 s、规划速度最大2.001 m/s、规划加速度最大1.511 m/s²、里程计10 Hz、桥接故障
0。日志中仍有215条SUPER失败相关日志行，其中备用优化123条、备用轨迹返回72条、
走廊生成9条、主轨迹优化5条、主轨迹返回6条；这些是日志行数，不等于215次独立故障。

源码检查发现`generateBackupTrajectory`内存在一次输出未直接提交的第二次优化调用。曾以
A/B方式暂时移除该调用，但`run08`在55 s前仍未完成任务，主轨迹优化和返回失败分别升至
154和159条，自动验收正确返回FAIL。由此确认该调用会刷新优化器内部热启动状态，并非
可安全删除的冗余计算，已恢复原流程并补充源码注释。

SUPER优化失败日志现会输出LBFGS返回值以及位置、速度、加速度、角速度和推力残差；同时
修正备用轨迹后段采样误用旧时间变量的问题。最终`task1_acceptance_run09.bag`重新通过：
任务阶段32.001 s、路径55.105 m、返航误差0.064 m、最大横向偏移0.619 m、速度中位数
1.965 m/s、最长低速停顿0 s、实际最大速度2.666 m/s、规划速度最大2.001 m/s、规划
加速度最大1.508 m/s²、里程计10 Hz、桥接故障0，并在42.004 s完成后切回
`AUTO.LOITER`。

run09共有139条SUPER失败相关日志，其中备用优化79条、备用返回43条、走廊3条、主轨迹
优化6条、主轨迹返回8条。79条备用优化告警全部有加速度残差，残差中位数0.113、P90
0.187、最大3.225；位置、角速度和推力残差均为0。主轨迹只有6条但个别转折时残差很大。
因此下一轮应针对备用轨迹加速度约束、优化收敛和重规划频率做受控参数试验，不应直接
把`penna_margin`放宽或把飞行速度硬限制为2 m/s。Gazebo点云适配器还修复了roslaunch
退出时传感器回调晚于publisher关闭造成的异常；最终关停无`closed topic`回调堆栈。

新增`patches/super_optimizer_diagnostics.patch`保存本轮SUPER外部源码修改，并在README
写明与原有SUPER任务门控补丁的应用顺序。下一步仿真重点是以run09为基线做单变量调参，
降低备用优化告警而不牺牲32秒闭环时间、连续速度、避障裕量和返航精度；之后再做点云及
里程计断流、任务取消和规划持续失败的故障注入。

## 第72轮：备用轨迹加速度惩罚单变量试验

按用户要求继续只做仿真。以run09为对照，临时把SUPER
`backup_traj/penna_acc`从`1.0e5`提高到`5.0e5`。run10仍完成三墙探索、返航和
`AUTO.LOITER`切换，备用优化告警由79条降到5条，但任务阶段由32.001 s增至34.000 s，
最长低速由0增至1.136 s，返航误差由0.064 m增至0.357 m，自动验收因连续低速超过
1 s返回FAIL。说明高权重能压低违规次数，却损害连续飞行表现，不能直接采用。

随后测试中间值`3.0e5`。有一次启动因同名参数替换误改到主轨迹项，以及一次录包晚于
Gazebo启动，均在日志中识别后立即作废，不用于结论。按配置段精确修正并严格按
“roscore、确认rosbag开始、再启动Gazebo”的顺序重跑后，run12在早期进入持续的
`PlanFromRest/GeneratePolytopeFromLine`失败；26.7 s时仍未完成，验收返回FAIL，记录
1280条主轨迹返回失败日志，未形成有效飞行路径，因此提前终止。

所有临时参数已恢复，当前仍使用已通过run09的`backup_traj/penna_acc=1.0e5`。这组A/B
结果说明告警数不是唯一优化目标，也不能简单靠提高惩罚权重或放宽`penna_margin`解决。
下一步应分析备用轨迹时间分配、停止点初值和重规划触发条件，并始终同时约束闭环完成、
低速持续时间、路径、返航精度与动力学上限。

## 第73轮：USB数传无法连接QGC诊断

用户将飞控和两端数传上电，数传指示已建立无线连接，但当前NUC和其他电脑上的QGC无法
连接，只有一台曾配置过的设备可以连接。首先只读检查宿主机USB设备和串口占用：电脑端
数传被识别为Silicon Labs CP2102，设备为`/dev/ttyUSB1`，稳定路径是
`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`；
当前用户属于`dialout`组，权限正常。QGC已经独占该串口，ModemManager没有占用，所以
问题不是“QGC看不到设备”或Linux串口权限。

本机`QGroundControl.ini`没有保存手工Comm Link，依赖自动串口连接。临时关闭QGC后，
用pymavlink依次监听常用波特率，在`57600`下立即收到有效PX4 MAVLink心跳：system id 1、
component id 1、autopilot 12、vehicle type 2。由此确认飞控MAVLink、飞行端数传、无线
链路和电脑端CP2102均正常，电脑端正确波特率为57600，不是CH340接TELEM2链路使用的
500000。

重新启动QGC后，日志显示识别PX4 v5.1.4并加载`1_1`参数缓存，缓存时间更新为本轮时间，
QGC当前已连接且保持运行。现有证据把根因定位到不同电脑上的QGC没有为通用CP2102稳定
使用正确的57600串口设置，自动连接可能受先前串口状态影响；无线灯常亮本身不能证明电脑端波特率正确。永久设置
方法是在每台QGC的`Application Settings -> Comm Links`中新增Serial链路，选择CP2102
对应端口，波特率设57600、无流控后保存连接。同一串口不能同时被QGC、MAVROS或串口工具
占用。本轮没有修改PX4参数、数传参数或任务算法。

## 第74轮：Codex整机访问权限说明

用户询问如何给予Codex这台设备的完全访问权限。当前会话由客户端以受管的
`workspace-write`模式启动，可写`/home/nuc`和临时目录，但读取真实USB设备、启动本地
网络服务、访问系统目录或执行部分宿主机命令仍可能需要单次批准。项目配置中的
`trust_level = "trusted"`只表示信任项目，不等于关闭沙箱或授予root权限；聊天中声明
“以后全部同意”也不能覆盖客户端/系统层的批准机制。

本机Codex CLI 0.154.0提供两种相关启动方式：`-s danger-full-access -a never`关闭Codex
文件系统沙箱并不再询问批准；`--dangerously-bypass-approvals-and-sandbox`同时跳过批准和
沙箱，CLI明确标记为极度危险、仅适合设备本身已有外层隔离的环境。两种方式都只获得
当前Linux用户`nuc`本来拥有的权限，并不会自动获得root；不建议把Codex以root运行或为
所有命令配置`NOPASSWD: ALL`。当前会话不能自行升级，必须由用户在客户端选择Full Access
后新建会话，或从终端用相应参数重新启动。更推荐保留当前模式，对USB、Gazebo、Git推送
等所需类别使用可持久化的窄范围批准规则。

## 第75轮：现场启动任务一复杂采空区仿真

用户询问任务一当前完成程度并要求直接启动仿真查看效果。由于真实飞控仍通过数传连接且
真实PX4和SITL都使用system id 1，为防止QGC把两台载具合并显示，先关闭QGC；没有修改或
控制真实飞控。确认无残留ROS、Gazebo、RViz和PX4进程后，在独立ROS Master 11312上启动
`task1_px4_sitl.launch`，使用`goaf_complex.world`、34 m探索半径，并同时打开Gazebo和
RViz。

本次完整链路包括PX4 SITL、Gazebo Iris、模拟MID360三维扫描、Fast-LIO2接口格式适配器、
任务调度器、自主决策器、SUPER、坐标/指令桥和MAVROS。仿真9 s时调度器从hold切到
`goaf_exploration`，SUPER收到前向观察目标；约10 s进入OFFBOARD并持续飞行。随后完成
尽头探测、连续三墙覆盖判定和自动返航，最终状态为`COMPLETE`，三墙状态为`ready`：
`end_seen=true`、尽头深度29.50 m、左右覆盖率1.00/1.00、左右缺口0/0、尽头墙跨度
11.00 m、连续确认4/4。PX4已退出OFFBOARD并进入`AUTO.LOITER`，最终位于入口附近
`(-0.64, 0.14, 1.85) m`保持。Gazebo和RViz在本轮结束时继续运行，供用户查看最终累计
点云、轨迹和场景。

任务一当前达到“仿真完整闭环通过”：可按机头主方向探索、利用点云判断墙体覆盖、调用
SUPER避障规划、向PX4输出轨迹、完成后返航并安全退出自动控制。尚未达到“真机可飞”：
真实Fast-LIO2到PX4 EKF融合与外参、真实MID360建模质量、遥控开关、人工接管和断流/低电
等故障保护仍需实机或硬件在环验收；SUPER备用轨迹优化告警也仍需继续优化。

## 第76轮：150 m采空区高度、雷达范围与入口误判核对

用户询问任务一启动条件、悬停启动、人工切回手动时的高度/姿态安全、任务高度来源、
60 m雷达范围、深处点云显示异常，以及从狭窄走廊进入采空区时是否会把走廊误判为三面墙。

本轮用户要求重新启动任务一仿真，并保存前面失败的仿真现象供论文使用。已创建
`/home/nuc/task1_logs/failure_cases/large_platform/`，保存入口平台误判包
`run01_false_end_wall.bag`、深处同步超时包`run02_sync_timeout.bag.active`及案例说明。
成功对照包`task1_large_platform_run03.bag`仍保留在上级目录，没有重复复制大文件。

重新启动前发现上一轮完成后的旧roslaunch没有及时退出，继续运行到约10000 s仿真时间后
`fsm_node`以退出码-9终止；该长时间资源累积现象也写入案例说明。旧进程随后已停止，
新的任务一仿真将在清理完成后重新运行。本轮资料目录不删除或覆盖原始数据。

随后重新启动run04，运行至约128 m目标附近时用户要求停止。停止前日志显示0.867--133.582 s
持续有`Gazebo MID360 registered`输出，说明逐帧`/cloud_registered`仍在生成；累计体素约
120000后基本饱和，RViz因此表现为累计点云不再明显增加，而SUPER障碍物轮廓仍更新。日志中
另有约18次`odometry is -0.00x s old`间歇丢帧，但没有持续雷达断流。run04已归档为
`/home/nuc/task1_logs/task1_large_platform_run04_user_interrupted.bag`，对应ROS日志和
分析说明保存在`failure_cases/large_platform/`。

本轮诊断结论：问题不是雷达完全无数据；主要原因是累计全局点云达到120000体素上限、
逐帧点云RViz显示衰减时间仅0.4 s，以及仿真时钟造成少量负时间戳延迟。后续应分别增加或
改为滑动窗口显示、修正仿真时间戳负延迟容忍度，并用`rostopic hz /cloud_registered`
和`rostopic echo /cloud_registered/header`区分数据链路与RViz显示问题。

第三次大场景仿真已经完成闭环：PX4保持连接并进入OFFBOARD，无人机从入口外高平台沿
任务机头方向进入，实际速度约1.7 m/s，随后在约154.5 m进度处识别前墙，覆盖状态为
`three_wall=ready end_seen=true end_depth=157.50 end_span=85.00 left=0.97 right=1.00 gaps=2/0 confirm=4/4`，完成后返航并进入`AUTO.LOITER`。因此本轮验证的是仿真闭环和判据，不代表真实雷达、EKF或人工接管已经实机验收。

高度分两层理解：任务一决策器使用Fast-LIO2适配后的`/Odometry`，以任务一第一次被允许
时捕获的home位姿为零点，观察目标高度是相对home的z；最终PX4实际稳定高度由其EKF本地
位置和位置控制器执行。第三次运行还修正为只有任务启用后才捕获home，避免节点提前启动时
把地面第一帧当成返航原点。当前任务一是固定相对任务高度，不是地形跟随：下方地面下降
时无人机通常保持原高度、离地间隙变大；地面上升时点云/SUPER可作为障碍约束，但尚未
实现专门的地形跟随。如果EKF本身发生高度重置，PX4可能出现估计跳变，这必须通过EKF
参数和实机飞行测试单独验证，不能由任务一代码保证。

当前大场景采用约70 m的理想化三维射线雷达模型；真实MID360约60 m有效范围时，前墙不能
从入口直接看到，必须飞到距前墙约60 m以内才可识别。深处“无点云”需区分：`/cloud_registered`
是逐帧注册点云，理论上仍应持续发布；累计全局显示可能达到仿真适配器的120000体素上限，
RViz也可能因Fixed Frame、显示队列或负载看起来不再增长。第二次运行曾因70 m高密度射线
造成处理延迟超过决策器1 s新鲜度门限，出现`WAIT_DATA`和悬停；第三次将仿真射线降为
240×16、令注册云与里程计共用时间戳、仿真数据门限放宽到3 s后通过。真实Fast-LIO2
参数和真实MID360数据链路没有被这些仿真参数修改。

入口误判方面，原算法曾只用末端切片最左/最右占据点跨度，第一次大场景因此在约42 m把
入口平台和两侧墙误判为101 m末端墙。已改为检查包含任务中线的连续横向占据分量，第三次
运行没有再在入口误判，并到达前墙。这个修复不能等价于走廊语义识别：若狭窄走廊前端有
中线墙且达到真实配置的最小跨度4 m，仍有误判可能。实机还需增加入口缓冲距离、走廊长度/
开阔度判据以及与三面墙连续覆盖的多帧联合确认。

## 第78轮：停止无效仿真并重新审查坐标对齐与完成定义

用户指出多次仿真没有返航、左侧覆盖缺失时无人机却向右侧走，并明确完成条件不应是
“扫到三面墙”，而应是点云/模型闭合；要求停止当前仿真，不再徒劳测试。

已停止当前run07仿真、PX4 SITL、Gazebo、MAVROS、SUPER和ROS Master，录包为
`/home/nuc/task1_logs/task1_large_platform_run07_gap15.bag`。

坐标链路审查结论：`fastlio_px4_vision_bridge`会在首次有效Fast-LIO2 `/Odometry`
上捕获原点和初始yaw，将`camera_init`坐标变换到PX4视觉里程计的`odom`局部坐标；
`super_px4_command_bridge`再使用同一对齐结果把SUPER目标转换为PX4本地NED setpoint。
但`super_exploration_decider`本身不查询TF，要求输入点云和`/Odometry`已经在配置的
`camera_init`同一坐标系；真实设备的雷达外参、Fast-LIO2 frame_id、PX4 EKF视觉融合
仍需实机标定验证。因此不能说“所有定位设备自动完成坐标对齐”，只能说已有软件对齐
接口和变换公式。

左侧缺失却向右走的原因是：任务主方向选择器只把前向候选作为优先；当左侧墙或左侧
frontier因点云稀疏、坐标偏差或局部未知区被过滤后，右侧仍有可行候选，SUPER会在当前
规划可行域内向右侧补充探索。现有代码没有“左侧缺失时禁止右偏”“横向偏移超限即返航”
和“走廊中心保持”的独立硬约束。多次不返航则是因为完成状态仍要求左右墙覆盖率、最大
缺口和末端墙全部满足；这些条件未满足时，系统会继续规划或超时重规划，而不是按点云
闭合条件结束。

根据用户新的需求，下一步应先重构任务一完成判据为“模型闭合/未知空间收敛 + 返航安全
门”，保留墙体信息作为辅助观测而非完成硬条件；同时增加主方向航迹走廊、横向偏移上限、
左侧缺失时的中心保持/补扫策略。在这两个定义完成前不再重复大场景仿真。


## 第79轮：按地图闭合重构任务一完成与横向约束

用户要求按当前发现的问题直接修改：完成条件不再要求三面墙，只要求点云模型闭合；
左侧点云缺失时不得继续无约束向右漂移；多次未返航的问题必须从判据上修正，并避免
重复无效仿真。

本轮将任务一默认完成策略由`require_three_wall_completion`改为
`use_map_closure_completion`。新的闭合安全门必须同时满足：飞机沿任务起始机头轴达到
最小深入距离、当前点云连续确认前向闭合边界、任务中心走廊内可执行前沿数降到阈值、
占据体素地图在稳定时间窗内无显著增长，并连续多个决策周期成立。满足后发布
`model_complete`并通过SUPER返回任务起点。原左右墙覆盖率、尽头墙跨度和缺口仍在
`/mine_uav/exploration/model_coverage`发布，但只作诊断，不再阻塞返航。前向边界已经
确认时决策器不再切到左右frontier fallback，而是保持并等待地图收敛。

所有frontier候选（包括fallback）新增任务坐标系横向硬限制
`max_task_lateral_offset`，默认正负4 m；因此“一侧点云缺失”不再允许高层观察目标
无限偏向另一侧。该约束基于任务启用瞬间的home和机头yaw，不是Gazebo世界X/Y轴。
本轮未改动Fast-LIO2→PX4视觉里程计对齐桥和SUPER→PX4指令桥；决策器仍严格要求
`/Odometry`与`/cloud_registered`同属`camera_init`。已有软件坐标转换不等于真实
MID360安装外参或PX4 EKF已经自动标定，后两者仍需真机验证。

同时修正长距离仿真点云显示链路：累计地图达到上限后不再每帧错误地多插入一个体素；
大场景累计上限由120000提高到500000；允许里程计时间戳最多超前0.05 s，避免仿真时钟
轻微负延迟导致丢帧。验收脚本的硬完成项由`three_wall=ready`改为
`map_closure=ready`。

完成一次有针对性的基础场景测试：PX4正常解锁并进入OFFBOARD，飞机沿任务轴推进；
在进度约18.1 m处得到`front_closed=true`、`actionable=0`，横向位置约-1.1 m，
没有越过4 m任务走廊。测试同时发现0.5 m占据体素受Gazebo位姿抖动影响，静止附近仍以
约35个/s增长，原20体素增长阈值过严，导致状态停在`WAIT_MAP_CLOSURE`。据实测将默认
阈值调整为500，大场景调整为1000；该调整后未再重复启动仿真，避免继续无效测试，真实
MID360阈值需用静态悬停数据重新标定。

`catkin_make --pkg mine_uav_control`、Python语法检查、两个SITL launch XML检查以及
Git diff空白错误检查均通过；测试结束后ROS、Gazebo、PX4和SUPER进程均已停止。当前可
确认状态机和接口已编译通过，前向闭合与横向约束在仿真中出现预期状态；采用新噪声阈值
后的“闭合→返航→AUTO.LOITER”完整回归尚未再次执行，因此不把本轮结果宣称为完整闭环
通过。

## 第80轮：150 m大场景闭合判据实测与当前流程答疑

用户要求先按原场景执行仿真，再回答最小深入距离、入口平台误判条件和任务一完整流程。
本轮启动入口外高平台、内部约150 m深、100 m宽、30 m高的PX4/Gazebo/RViz全链路仿真，
并录制`/home/nuc/task1_logs/task1_large_platform_run08_map_closure.bag`。

仿真正常进入OFFBOARD。约24.8 m进度时`front_closed=false`且仍有10个可执行前沿，入口
平台没有被误判为完成；约71.6 m时世界y约-4.1 m，但按任务初始航向约-2.9°换算后的
横向误差接近0，证明飞机沿任务机头轴飞行。末端累计地图报告`end_seen=true`、
`end_depth=158.5 m`、前墙连续跨度约82 m，任务进度约157.5 m，中心走廊无可执行前沿
持续约40 s，但`front_closed`没有稳定成立，任务未返航并保持OFFBOARD。验收结果为FAIL，
不是完整闭环通过。

根因是当前`front_closed`仍来自瞬时单帧点云：6 m范围、24°总角宽、至少20点、横向跨度
至少1 m、垂直跨度至少0.8 m并连续3帧。飞机太接近墙时，窄锥体在墙上的物理横向跨度会
降到1 m以下，瞬时标志重新变false；累计地图已经闭合和瞬时门无法与数秒稳定条件同时
成立。该失败现象和分析已保存到`failure_cases/large_platform/`，仿真、录包和所有ROS
进程均已停止。

当前通用配置最小深入距离是8 m，大场景launch覆盖为30 m。旧“三面墙”只保留诊断；
大场景平台前缘距任务起点约8 m，低于诊断末端最小深度15 m，也低于地图闭合最小进度
30 m，因此本次没有误判。若任务原点捕获错误、平台结构在任务前方达到这些距离门槛，且
同时满足前向表面点数/跨度、无前沿、地图稳定和连续确认，仍可能误判。


## 第81轮：墙外目标根因与一次飞入建模的返航定义

用户询问为什么frontier会把障碍物外面的点作为目标，以及“一次飞入并带回完整SLAM模型”
应采用什么返航条件。源码核对发现，末端墙外目标未必来自标准frontier，而可能来自
`publishForwardLookaheadGoal()`：中心线目标被特殊允许为“只要没有标记occupied即可”，
不要求`isKnownFree`。当前地图存在点云孔洞、体素离散或配准误差时，墙后的unknown体素
因此可能被发布为目标。标准frontier也只检查目标附近球形区域没有occupied，没有验证目标
属于无人机当前可达自由空间连通分量，也没有对当前位姿到目标做全线碰撞检查。SUPER负责
轨迹避障，通常会拒绝穿墙轨迹，但高层仍会持续提供不可达墙外目标，形成大量重规划失败。

正确修复应包括：中心线look-ahead也必须是known-free；frontier只从当前位姿可达的free
连通分量提取；发布前对整条连接线做膨胀占据检查；已锁存末端边界后禁止任何目标越过
`end_depth - standoff`；SUPER报告规划失败时将目标暂时加入黑名单。高层目标有效性和
SUPER局部避障是两层保护，不能只依赖SUPER替错误目标兜底。

对于“一次沿中轴飞入”的任务，不应以三面墙或全局frontier清零作为唯一完成条件。推荐
返航安全门为：累计地图可靠识别并锁存前端连续边界；飞机到达该边界前的安全停距；只在
当前可达自由空间和任务关注区域内统计的有效frontier/未知体素降到阈值；新增占据体素和
地图覆盖率在滑动时间窗内收敛；点云、位姿质量持续健康；返航路径可用且剩余电量覆盖
返航并有安全余量。任何低电、断流、规划连续失败或任务超时应触发提前安全返航，而不是
宣称建模完整。

在本项目理想化100 m宽、MID360有效半径约60 m、无人机沿中轴飞行的场景中，一次航线理论
上可看到两侧墙、地面、顶板和前墙；但有支柱遮挡、凹区或非视距空间时，一次直线飞行无法
从算法上保证完整建模。此时必须由建模模块提供ROI覆盖率/未知体素率等质量指标，决策器只
能依据该指标决定继续补扫还是返航。

## 第82轮：冻结补丁式修改并确定未知尺寸采空区最终方案

用户指出实际任务是在飞行前完全不知道采空区尺寸的情况下，让无人机携带雷达进入一次并
带回尽可能完整的建模，要求停止继续改代码，先确定最终可行方案。检查发现当前未提交版本
已在5个核心文件中累计约510行新增内容，将可达frontier、终墙锁存、地图收敛和目标生成
耦合进同一决策器；本轮冻结算法代码，不再继续增加针对单一仿真现象的判据。

最终方案确定为“全局覆盖探索器 + SUPER局部规划器”的分层结构。全局探索器维护动态增长
的稀疏占据地图和可达自由空间，聚类当前可达frontier，按信息增益、路程、返航代价、定位
质量和雷达观测角度选择下一观察点；SUPER只负责到该观察点的局部无碰轨迹。任务采用
ENTRY、EXPLORE、BOUNDARY_SWEEP、VERIFY、RETURN、ABORT状态机。机头方向深入只是初期
效率偏置，不是固定直线规则；沿墙飞行只在侧向未覆盖或边界补扫时启用，不能作为唯一探索
算法。未知尺寸由动态地图和frontier自然扩展，配置中的半径只作为电量/法规/通信安全上限，
不能参与“建模完成”判断。

正常完成条件定义为：任务可达区域内不存在达到最小体积和信息增益的frontier；地图新增
覆盖在滑动窗口内收敛；所有已发现边界和遮挡后区域已完成观察或被标记不可达；入口至当前位置
存在经过膨胀验证的返航通路；定位、点云和地图质量有效；剩余电量高于动态估算的返航消耗与
安全余量。低电、定位失效、点云中断、连续规划失败和通信/任务超时属于ABORT安全返航，不能
发布“建模完成”。对任意未知、可能存在隐藏连通空间的环境，单架无人机单次飞行无法数学保证
绝对完整；工程上的完整定义是“在任务安全约束下，所有可达且可观测frontier均已消除并满足
地图质量阈值”。后续实现前需先重构模块和接口，再分别做单元场景、尺寸变化和复杂遮挡回归。

## 第83轮：收敛为空旷涵洞模型并专项验证frontier与避障

用户进一步明确采空区通常是内部没有大型横向障碍的空旷涵洞，当前不需要设计复杂内部障碍
绕行或沿墙主策略，首先必须保证自动避障正常，并且frontier绝不允许发布墙后或其他不可达
目标。本轮据此把工作范围收敛为两道安全门：高层候选必须是known-free、满足机体膨胀间距，
且属于从当前位置在自由体素中洪泛得到的连通分量；前向look-ahead与普通frontier共用该
可达集合。下层SUPER继续通过ROG-Map膨胀、A*、安全飞行走廊、在线碰撞复查和备份轨迹避障。

对未提交工作进行拆分，移除了未通过验证的累计终墙锁存、终墙接近和自适应地图收敛补丁，
源码改动从约510行降到约140行，只保留可达洪泛、候选过滤、候选聚类和复用候选集合。当前
终墙/返航完成条件恢复到原实验版本，明确不作为未知尺寸采空区最终完成判据。

完成一次PX4/Gazebo专项SITL。正常决策器在世界终墙约22 m时发布的目标依次约为x=7.48、
7.06、7.38和8.22 m，没有发布墙后目标。随后故意向SUPER注入x=25 m墙后目标，SUPER先沿
已知自由空间接近，在墙前出现安全走廊/优化失败并执行备份撤退；连续采样的飞机最大x约
19.2 m，未越过终墙。结果证明下层不会直接穿墙，但错误目标会造成大量重规划，因此高层
可达过滤是必要的第一道保护。测试后已关闭全部仿真进程，精简版本通过
`catkin_make --pkg mine_uav_control -j2`编译。

## 第84轮：验证无人机到达尽头后的自主返航

用户澄清本轮不是手动发布返航命令，而是验证无人机自主深入、识别尽头、判断任务完成并
自行返航。第一次启动因PX4 SITL在启动阶段以255退出，未计入测试；单独诊断PX4启动正常，
随后重新启动完整链路。

第二次完整PX4/Gazebo/SUPER/MID360模拟链路成功运行。任务调度器自动切换到任务一，决策器
捕获home约为`(-0.01,-0.01,-0.13)`，沿前方依次发布约`x=7.48、12.61、15.40、18.43 m`
的look-ahead目标。接近终墙后前方障碍连续确认并切换到frontier fallback；地图状态最终为
`map_closure=ready`、`front_closed=true`、`actionable=0`、`closure_confirm=4/4`、
`end_seen=true`、`end_depth=22.50 m`、`progress=18.82 m`。

随后决策器自行发布home目标`(-0.01,-0.01,1.67)`，SUPER暂停探索，PX4接受模式切换到
`AUTO.LOITER`。最终状态为`COMPLETE`、`finished=True`、`returning=False`，MAVROS里程计
约为`(-0.35,-0.01,1.52)`，已回到home附近。因此当前标准场景已经验证“自动发现尽头→完成
确认→生成返航目标→回到home→AUTO.LOITER”的闭环。该结果只证明当前仿真场景的自动返航
链路有效，不代表未知尺寸、超出续航或存在遮挡的真实采空区已经具备完整建模保证.

## 第85轮：实验室手动悬停后用物理CH7启动任务一

用户提出临时实验逻辑：飞手先手动起飞并悬停，物理CH7发生低→高变化后建立home，再开始任务一的路径规划和自动避障；同时确认当前代码是否具备避障能力。

本轮将任务一实验入口设置为“启动边沿门控”。新增参数 `require_mission_enable_edge`，真实任务一启动文件开启该参数。任务一节点启动时保持禁用，不会因为旧的锁存消息或启动时第一帧里程计而提前捕获home。调度器仍按原配置使用物理CH6选择任务：低位选择任务一；物理CH7对应ROS数组下标6，低位为自动未允许、高位为自动允许/启动。CH7低→高经过约0.5秒去抖后，调度器发布 `/mine_uav/mission/goaf_enable=true`；任务一收到该次使能边沿后清空上一轮任务状态，等待下一帧同步的Fast-LIO2点云与位姿，并把此时的无人机位姿、航向记录为home，然后才向SUPER发送探索目标。CH7拨回低位会停止任务一自动目标输出，飞手可以接管；重新测试必须先低位再高位。该逻辑不会自动起飞。

同步回调仍要求点云和里程计时间匹配；home来自任务启动瞬间的同步位姿，而不是固定原点。任务一的高层目标只从当前可达、已知自由且满足膨胀安全距离的空间中产生，SUPER继续负责局部A*/安全走廊、轨迹碰撞检查和备份轨迹，因此两层共同构成避障链路。

代码已同步到 `/home/nuc/super_ws`并通过 `catkin_make --pkg mine_uav_control -j2`编译通过。当前结论是：在SITL和代码逻辑层面已有避障能力，且此前已验证终墙前不会穿墙；但真实实验仍需拆桨、低速、可立即人工接管，并另外验证真实点云坐标系/外参、制动距离、定位和通信中断保护。当前修改尚未在本轮推送GitHub，待本轮结束按项目约定提交并推送。
本轮已完成本地提交 `0a44420 feat: gate task one start on CH7 enable edge`；推送因当前环境暂时无法解析 `github.com` 失败，网络恢复后需执行 `git push origin main`。
本轮确认：飞行中CH7拨低会撤销自动允许，桥接器先发布当前位置保持目标，再请求PX4退出OFFBOARD并切换到POSCTL；这不是自动定点悬停，飞手必须保持摇杆和模式可控。任务一速度来自SUPER的 traj_opt.boundary.max_vel=2.0 m/s，桥接器 max_speed=2.2 m/s 是校验上限；实际速度还受加速度1.5 m/s²、轨迹、障碍和PX4跟踪能力限制，因此不能保证始终达到2.0 m/s。
本轮将任务一速度限制统一改为1.0 m/s：SUPER的 traj_opt.boundary.max_vel=1.0，PX4指令桥 max_speed=1.0；同时将指令桥 max_height 和任务一真实/SITL启动入口的高度上限统一改为1.8 m。任务一的巡航高度仍由相对home的观察高度参数决定，最终输出还会经过指令桥高度硬限制。
本轮临时测试调整：新增 force_goaf_task 参数并在 task1_real.launch 和 task1_px4_sitl.launch 设为 true，忽略CH6任务选择通道；CH7仍作为自动允许/启动开关，任务二保持不可用。测试结束后将该参数恢复为 false，即重新启用CH6任务选择逻辑。mission_scheduler 编译已通过。
本轮确认Fast-LIO2 /Odometry可通过 fastlio_px4_vision_bridge 发布到 /mavros/vision_pose/pose_cov，默认30 Hz，并在未解锁时建立 camera_init 到 PX4 odom 的初始对齐；输入超时、跳变、无效四元数或MAVROS断开时会将 vision_healthy 置为false。该链路具备PX4接入接口，但不能仅凭ROS话题存在保证QGC不报警：PX4还必须启用EKF2外部视觉融合并正确设置EV高度源、安装外参/延迟等参数。现场应同时检查 /Odometry、/mavros/vision_pose/pose_cov 的频率、/mine_uav/task1/vision_healthy，以及QGC的EKF/定位状态。
本轮现场只读检查：当前没有运行roscore、MAVROS或fastlio_px4_vision_bridge，因此无法读取 /Odometry 或验证其到PX4的实际转发；/dev/ttyACM0、/dev/ttyUSB0、/dev/ttyUSB1 当前均不存在。内核日志曾显示CH341/CH340在USB 3-2短暂枚举为 ttyUSB1，随后设备断开并重连失败/再次短暂连接，说明当前串口设备并未稳定存在。
用户重新连接设备后要求排除沙箱假阴性。本轮使用主机权限复查：CH340稳定枚举为 /dev/ttyUSB0，并存在 /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0；按项目配置以500000波特率重启MAVROS后，日志确认 FCU URL 正确且 `CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot`。ROS Master使用11312。当前 `/mavros/state` 为 connected=True、armed=False、mode=AUTO.LOITER。但 `/Odometry` 没有发布者，`/mavros/vision_pose/pose_cov` 虽被MAVROS订阅却没有新消息，`/mine_uav/task1/vision_healthy`不存在；现场没有运行真实Livox/Fast-LIO2或fastlio_px4_vision_bridge。因此结论是PX4串口链路已通，雷达Odometry尚未发到PX4。
随后用户配置好有线网卡后，本轮启动并验证完整传感器到PX4位姿链：enp89s0=192.168.1.10/24，MID360配置IP=192.168.1.157；重启MID360驱动、Fast-LIO2和fastlio_px4_vision_bridge后，/livox/imu约200 Hz，/Odometry约10 Hz，/mavros/vision_pose/pose_cov约30 Hz，/mine_uav/task1/vision_healthy=True，vision_status=STREAMING。话题连接关系确认：/laserMapping发布/Odometry并被位姿桥订阅；位姿桥发布/mavros/vision_pose/pose_cov并被/mavros订阅。PX4/MAVROS保持connected=True、armed=False、AUTO.LOITER；本轮未启动任务一、未解锁。
本轮继续检查QGC的外部定位报警：PX4实际参数为 EKF2_EV_CTRL=9、EKF2_HGT_REF=2、EKF2_EV_DELAY=0、EKF2_EV_POS_X/Y/Z=0、EKF2_GPS_CTRL=0、EKF2_BARO_CTRL=1、EKF2_RNG_CTRL=2。根据本机PX4源码定义，EV_CTRL=9只启用水平位置和航向，不启用垂直位置；HGT_REF=2选择测距作为高度主源，而当前MAVROS的rangefinder插件被屏蔽且没有测距话题。因此ROS外部视觉位姿虽然正常发布，PX4的高度/完整位置融合条件仍不成立，QGC报警主要由这组EKF2参数造成。本轮未修改飞控参数。
本轮用户询问修改方法。建议拆桨、未解锁时在QGC参数页或MAVROS执行：将 EKF2_EV_CTRL 从9改为11（水平位置+垂直位置+航向；当前只发布位姿，不启用外部视觉速度），EKF2_HGT_REF 从2改为3（Vision），确认 EKF2_EV_DELAY=0，按MID360相对飞控机体FRD坐标填写 EKF2_EV_POS_X/Y/Z；若没有测距传感器，将 EKF2_RNG_CTRL设为0。保存后重启PX4。注意EV_POS外参不能盲填，0只适用于雷达与飞控参考点基本重合。
用户补充确认机载有激光测距模块，因此修正建议：不能因为MAVROS的rangefinder/distance_sensor插件被屏蔽就直接把 EKF2_RNG_CTRL设为0；若激光直接接PX4并已配置驱动，应保留 EKF2_RNG_CTRL=2。激光测距与Fast-LIO2 Odometry是两条不同输入：前者用于距离/高度约束，后者用于外部视觉位置姿态。任务一若希望Fast-LIO2同时提供高度，应设 EKF2_EV_CTRL=11、EKF2_HGT_REF=3，激光仍可作为辅助；若使用激光作为高度主源则HGT_REF=2，但EV_CTRL=9不会融合外部视觉垂直位置。
本轮澄清：当前 EKF2_EV_CTRL=9 已包含水平位置融合位（bit0）和航向融合位（bit3），因此该参数本身不会关闭外部视觉水平定位；EKF2_HGT_REF=2只影响高度主源。若QGC仍提示没有可靠水平/外部定位源，应继续检查PX4是否实际融合、数据时间戳、坐标系、协方差和EKF创新状态，不能仅凭参数值判定。
最终诊断：QGC显示“没有可靠的水平定位源”不是因为EKF2_EV_CTRL=9关闭了水平融合，而是测试过程中外部视觉链路没有持续稳定送入PX4。现场先后观察到ROS Master地址分裂（11311与11312并存）、重复Fast-LIO进程、MAVROS进程退出、/Odometry与/mavros/vision_pose/pose_cov一度无消息；在统一到11312并重启MID360、Fast-LIO2、位姿桥和MAVROS后，/Odometry约10 Hz、vision_pose约30 Hz、vision_healthy=True且MAVROS订阅正常。用户随后已拔掉设备，因此无法再读取当时PX4的EKF融合标志；现有证据支持“运行链路/启动管理不稳定”为主因，HGT_REF=2只涉及高度，不是水平报警根因。
本轮用户确认PX4已经参与融合，并要求永久保存MID360/NUC网络地址及整理全部启动命令。已用NetworkManager为NUC雷达网口enp89s0创建永久连接`mid360-static`：NUC为192.168.1.10/24、无网关和DNS、never-default、开机自动连接；MID360继续使用192.168.1.157。用户随后关闭雷达并明确不需要继续检查，因此本轮未把“雷达离线”误判为配置失败。新增`px4_fastlio_localization.launch`作为安全的定位专用入口，只启动MID360驱动、Fast-LIO2、MAVROS和Fast-LIO到PX4视觉位姿桥，不启动SUPER、不发布飞行目标、不切模式且不解锁。README已区分两套互斥命令：定位链路单独验证，以及完整`task1_real.launch`；统一使用ROS Master端口11312，任务一仍为手动起飞悬停后CH7低到高启动、CH6临时忽略。
本轮进一步明确现场完整启动方式：正常执行任务一时只运行`task1_real.launch`，该入口已经同时包含MID360驱动、Fast-LIO2 `/Odometry`、外部视觉位姿到PX4、MAVROS、CH7任务调度、自主决策器、SUPER及PX4指令桥，不得再同时运行`px4_fastlio_localization.launch`。操作者先让CH7保持低位并运行统一启动命令，确认定位健康后手动解锁、起飞、悬停，再将CH7拨高启动任务；CH7拨低退出自动任务并切回人工接管。
本轮用户要求只保留完整任务一的逐行可执行终端指令，不需要起飞前检查或额外诊断命令。完整入口仍为：加载项目环境、统一ROS_MASTER_URI到11312、运行`roslaunch mine_uav_control task1_real.launch rviz:=true`；三行在同一终端依次执行即可。
