# 任务一人工接管后禁止自动抢回 OFFBOARD

状态（2026-09-17）：桥接器修复已通过单元测试、ROS 模式切换与对齐突变集成测试，以及 40 m 无界面仿真回归；**尚未用真实遥控器/PX4 拆桨验收**。

## 因果链

原桥接器只检查任务允许、定位、指令和 PX4 当前模式。任务已经受控进入 OFFBOARD 后，飞手通过 CH5 切到 POSCTL/MANUAL，`mavros/state` 改为非 OFFBOARD；自动允许仍为高位且任务未结束时，输出定时器把这当作“尚未进入 OFFBOARD”，继续预发送并再次调用 `/mavros/set_mode` 请求 OFFBOARD。这会抵消飞手的人工接管。

现在只把**已观察到的受控 OFFBOARD → 其他模式**且桥接器并未请求退出的转变认定为外部接管/飞控失效保护：锁存 `OFFBOARD_EXITED_EXTERNALLY` 故障，停止重新请求 OFFBOARD，并令旧 SUPER 命令失效。初始 POSCTL 预发送、桥接器主动完成任务退出、未受控的模式变化都不会误锁。现行临时 CH7 高电平授权方式下，飞手须先把 CH7 拨低清除软件锁存，再重新授权；这不改变 PX4 自身遥控器/失效保护的优先级。

同一 ROS 回归还暴露并覆盖一个对齐风险：`fastlio_to_px4_alignment` 若在任务执行中平移突变超过 0.05 m 或航向突变超过 0.05 rad，旧 SUPER 轨迹将被按新坐标关系解释。桥接器现在先将命令置为无效并锁存 `ALIGNMENT_CHANGED_DURING_TASK`，再更新变换；若已在受控 OFFBOARD，则持续发送当前 PX4 位姿悬停目标并请求退出。低于阈值的小变化仍允许通过，因而此阈值只是跳变检测，不是传感器连续漂移或绝对定位误差保证。

## 验证证据

- `test_offboard_mode_guard`：3 组纯逻辑边界测试通过。
- `super_bridge_manual_override.test`：模拟 `/mavros/state` 从 POSCTL→OFFBOARD→POSCTL；人工切回 POSCTL 后的 1.5 s 内 OFFBOARD 请求计数保持不变，CH7 授权低→高后才再次请求。
- 同一集成测试：0.02 m 对齐变化不触发退出；0.10 m 变化触发锁存并请求 POSCTL，随后不再重新请求 OFFBOARD。
- 原 40×40×30 m 任务一无界面 PX4/Gazebo 回归：已进入 OFFBOARD、前向探索、返航、任务 `COMPLETE`，最终 PX4 `AUTO.LOITER`；此回归说明主动完成任务退出未被当作人工接管锁住。

## 仍须验收

拆桨连接真实 PX4/遥控器，验证 CH5 手动/定位模式切换、RC 丢失、PX4 自身 failsafe、CH7 重授权、真实对齐重置，以及失去定位时的模式行为。当前补丁不解决 SUPER 规划高度与 PX4 1.8 m 围栏不一致，详见 `task1_height_envelope_contract.md`。
