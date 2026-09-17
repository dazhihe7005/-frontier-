# SUPER CIRI 竖直相切平面 NaN 与错误朝向（2026-09-17）

## 复现与完整因果链

任务一迷宫 SITL 的隔离探针让 CIRI 查询三维膨胀占据点，暴露出 `SearchPolytopeOnPath` 持续失败。一次约 18.024 s 的仪器化日志抓到：障碍点 `(6.7,-0.1,1.1)`、线段终点 `(6.7,-0.1,1.5)`、椭球中心 `(6.3,-0.1,1.5)`、物理半径 `0.4 m`，走 `findTangentPlaneOfSphere` 的 `branch=end`；切平面结果为 NaN。原函数先计算恒为零的 `pass_point - pass_point`，因此错误进入重合点扰动分支；更关键的是竖直相切经旋转后，理论 `|P_xy|²-r²=0` 可因舍入略小于零，直接 `sqrt` 生成 NaN。后续 `hPoly` 含 NaN 返回 `FAILED`；官方日志名 `maxVolInsEllipsoid failed` 实际对应 `hPoly.array().isNaN()` 检查，不能据此宣称 MVIE 求解器故障。

先为上述现场坐标新增确定性 C++ 测试：原源码失败；只修正恒零差值后仍失败，证明单一拼写修复不够。再加入仅在浮点级误差内的相切判定、共线种子的非零侧向扰动、球内终点显式拒绝，现场 NaN 测试才通过。随后增强几何断言又发现第二条独立错误：原切平面对障碍球心取负值，而 SUPER 的 `Polytope::PointIsInside` 定义为所有平面值 `≤0` 才在内部。仅生成有限值仍可能把障碍放进安全走廊。最终修复强制球心位于走廊外侧，归一化法向量，并验证终点与原始种子点均在允许侧，否则安全返回失败。

保存的源码增量见 `patches/super_ciri_tangent_plane_contract.patch`；确定性测试源见 `test/super_ciri_tangent_plane_test.cpp`，对应的 CMake 注册增量见 `patches/super_ciri_tangent_plane_test_registration.patch`。原测试红灯 1/2，差值单独修复后仍红灯；完整修复后 3/3 通过，`fsm_node` 构建通过。两个补丁均可在当前 SUPER 运行工作树用 `git apply --check --reverse` 验证与工作树一致。尚未把此修复上升为真机安全认证。

## 同场景 A/B 与剩余问题

在同样三维膨胀地图探针下，修复 NaN 后的 `task1_maze_tangent_fix_inflated_probe_20260917.bag` 中没有复现 NaN，但最大局部 x 约 13.4 m；进一步强制球心排除后的 `task1_maze_tangent_orientation_probe_20260917.bag` 最大局部 x 为 13.27 m，约 33.21 s 发送侧向前沿目标 `(22.25,7.75)`，30 s 超时后重复，直到 109 s 仍未穿过局部 x=18 m 第一墙。后一轮 `/rosout_agg` 记录 7253 条 `SearchPolytopeOnPath for new path failed`；可执行验收明确报 `insufficient_forward_progress` 与 `mission_not_complete`。这证明切平面缺陷被定位修复，**但它不是整个迷宫停滞的唯一原因**。

下一条因果链在严格净距下的前端路径/高层目标协议：绕第一道全高墙必须先横移至开口安全侧，再前进。现决策器直接给墙后的 frontier，A* 可选择贴膨胀边界的近路，CIRI 得到几乎相切、缺少严格正裕度的种子线。继续工作应记录前端路径到原始障碍的逐段净距，设计统一的水平/竖向分离净距合同与已知自由侧向过渡点，再做迷宫和空旷两场景 A/B，检查**规划与实际**墙边余量、深入、返航、高度及停滞。不能把 3/3 几何单测当作任务一闭环通过。

已进一步从此前保留的 190 MB 迷宫诊断 bag 读取 `/fsm_node/visualization/frontend_path`，用 `scripts/analyze_maze_frontend_path.py` 对 SDF 碰撞盒做精确线段—矩形距离计算（几何测试 4/4）：47.204 s 的 30 段前端路径中，第 27 段 `(16.1,5.1)→(16.3,5.3)` 到第一道墙的最小水平中心距**恰好 1.400 m**。同一附近的膨胀占据点到 CIRI 种子线约 0.400 m。由此可复算：A* 与 CIRI 使用同一膨胀边界时，最短路会贴在连续走廊的可行边界上，严格正裕度为零；仅继续增大全局膨胀步数仍可能在新边界上重复这一问题。此前成功基线在第二墙附近已观察约 0.201 m 水平跟踪误差，因此为了实测机体外至少 1 m，SITL 规划中心距至少应把 `0.4+1.0+跟踪裕度` 分开设定；此裕度尚待量化验收，不能把 1.4 m 的前端中心距直接算作可飞安全。

本轮所有三维膨胀地图、半径和 CIRI 诊断打印的临时代码已撤回，仿真进程已停止；保留上述经测试的切平面修复。原始 bag 只在 NUC `/home/nuc/task1_logs/failure_cases/maze_2026-09-17/`，不上传 GitHub。
