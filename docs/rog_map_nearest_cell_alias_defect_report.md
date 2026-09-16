# SUPER / ROGMap 最近邻查询输入输出别名缺陷报告

- 报告日期：2026-09-16
- 影响组件：港大 SUPER 内置 ROG-Map
- 本地官方基线：`2ad3419c127a617c6d7df6925e81a14175a9c096`
- 当前状态：官方内置 ROGMap 缺陷已复现且运行时可达；本次任务一 SITL 未观察到它改变目标或触发优化失败
- 风险等级：中高（属于潜在安全缺陷，但不是当前轨迹异常的已证实根因）

## 1. 摘要

ROGMap 的 `findNearestCellThat()` 和 `findNearestInfCellThat()` 接口允许调用者分别传入
只读起点 `start_pos` 和输出点 `nearest_pt`。SUPER 官方源码中有多处将同一个
`Eigen::Vector3d` 对象同时作为这两个参数传入。

两个函数在搜索前都会执行：

```cpp
nearest_pt.setConstant(NAN);
```

当输入和输出引用同一个对象时，这句代码也会把 `start_pos` 变成 `NaN`。后续距离判断
`(q_pos - start_pos).norm() > max_dis` 的左侧因此为 `NaN`。按照浮点比较规则，
`NaN > max_dis` 为 `false`，导致 `max_dis` 约束被绕过。

最小 GTest 已证明：对完全相同的查询点和 `max_dis=0`，输入输出分离时函数返回 `false`，
输入输出复用同一变量时却返回 `true`。普通概率图和膨胀图两条查询路径均可复现。

这是一个已经证实的底层接口缺陷。不过，后续保持决策条件不变的运行时诊断表明：在本次
40×40×30 m 任务一仿真里，目标点本身处于可用栅格，查询总是在第一个候选处返回原目标；因此
虽然输入引用确实被写成了 `NaN`，但目标没有被移动，也没有越过 3 m 限制。同期 20 次轨迹优化
失败不能归因于该缺陷。本报告必须区分“官方源码存在缺陷”和“它是不是当前现象根因”这两个命题。

## 2. 受影响的官方代码

### 2.1 普通概率图查询

官方基线 `rog_map/src/rog_map/rog_map.cpp:80-102`：

```cpp
bool ROGMap::findNearestCellThat(const bool & is, const GridType& target_type,
    const Vec3f & start_pos, Vec3f& nearest_pt, const double & max_dis) const {
    Vec3i start_id;
    posToGlobalIndex(start_pos, start_id);
    nearest_pt.setConstant(NAN);

    for(const auto & nei_id: cfg_.spherical_neighbor) {
        const Vec3i q_id = start_id + nei_id;
        Vec3f q_pos;
        globalIndexToPos(q_id, q_pos);
        if((q_pos - start_pos).norm() > max_dis) {
            return false;
        }
        // ...
    }
}
```

### 2.2 膨胀图查询

官方基线 `rog_map/src/rog_map/rog_map.cpp:105-128` 使用相同写法：

```cpp
posToGlobalIndex(start_pos, start_id);
nearest_pt.setConstant(NAN);
// ...
if((q_pos - start_pos).norm() > max_dis) {
    return false;
}
```

稳定源码链接：

- [ROGMap 两个最近邻函数](https://github.com/hku-mars/SUPER/blob/2ad3419c127a617c6d7df6925e81a14175a9c096/rog_map/src/rog_map/rog_map.cpp#L80-L128)
- [港大 SUPER 当前官方仓库](https://github.com/hku-mars/SUPER)

2026-09-16 联网刷新官方远端后，`origin/master` 仍为上述提交，相关实现仍然存在。运行工作区在
加入诊断前的该文件 SHA-256 也与干净官方基线完全一致。

范围限定：独立的 `hku-mars/ROG-Map` 当前主分支没有这两个接口，所以这里所说的是
“SUPER 仓库内置的 ROGMap 版本”，不能扩展成整个 ROG-Map 独立项目都存在同一问题。

## 3. 官方调用点

### 3.1 FSM 重规划目标

`super_planner/src/super_core/fsm.cpp:63`：

```cpp
planner_ptr_->getMap()->getNearestInfCellNot(
    GridType::OCCUPIED, gi_.goal_p, gi_.goal_p, 3.0);
```

这里 `gi_.goal_p` 同时作为输入和输出，并且返回值未检查。若查询失败，后续
`ReplanOnce()` 仍会直接使用 `gi_.goal_p`。

[官方 FSM 调用点](https://github.com/hku-mars/SUPER/blob/2ad3419c127a617c6d7df6925e81a14175a9c096/super_planner/src/super_core/fsm.cpp#L58-L67)

### 3.2 A* 局部起点和终点

`super_planner/src/super_core/astar.cpp:261-262` 与 `284`：

```cpp
map_ptr_->getNearestInfCellNot(
    OCCUPIED, local_start_pt, local_start_pt, 3.0);

map_ptr_->getNearestInfCellNot(
    OCCUPIED, local_end_pt, local_end_pt, 2.0);
```

这两处检查了返回值，但同样可能在别名调用时绕过 3 m 或 2 m 的限制。

[官方 A* 调用点](https://github.com/hku-mars/SUPER/blob/2ad3419c127a617c6d7df6925e81a14175a9c096/super_planner/src/super_core/astar.cpp#L245-L295)

### 3.3 PlanFromRest 起点修正

`super_planner/src/super_core/super_planner.cpp:920`：

```cpp
map_ptr_->getNearestCellNot(
    GridType::OCCUPIED, shifted_robot_p, shifted_robot_p, 3.0);
```

该点随后被直接用于构造安全走廊的种子线段。若搜索点超出预期距离，可能改变 CIRI 的起始
几何条件；若失败后出现非有限值，则规划应当被明确拒绝，不能继续进入几何或轨迹优化。

[官方 PlanFromRest 调用点](https://github.com/hku-mars/SUPER/blob/2ad3419c127a617c6d7df6925e81a14175a9c096/super_planner/src/super_core/super_planner.cpp#L917-L930)

## 4. 缺陷机理

调用形式如下：

```cpp
Vec3f point = ...;
getNearestInfCellNot(OCCUPIED, point, point, 3.0);
```

执行顺序为：

1. `start_pos` 和 `nearest_pt` 分别绑定为同一个 `point` 对象的常量引用和可写引用。
2. `posToGlobalIndex(start_pos, start_id)` 在污染前完成，因此 `start_id` 仍然有效。
3. `nearest_pt.setConstant(NAN)` 把 `point` 三个分量全部改为 `NaN`。
4. 因为两个引用指向同一对象，`start_pos` 也随即表现为 `NaN`。
5. 每个候选点的距离变成 `(q_pos - NaN).norm()`，结果仍为 `NaN`。
6. `NaN > max_dis` 始终为 `false`，搜索不会在超过 `max_dis` 时退出。
7. 函数可能返回距离限制之外的第一个匹配栅格；若一直未找到，调用变量可能保持为 `NaN`。

预计算的 `spherical_neighbor` 默认覆盖最大约 5 m，因此 `max_dis=2 m` 或 `3 m` 被绕过时，
查询最多可能继续到该预计算邻域的边界，而不是按调用者要求停止。

## 5. 最小复现测试

测试文件：

```text
/home/nuc/super_ws/src/SUPER/rog_map/test/nearest_cell_alias_test.cpp
```

测试对同一个非栅格中心点执行两组查询：

```cpp
const Vec3f query(0.03, 0.04, 0.02);

Vec3f separate_output;
bool separate_result = map.getNearestInfCellNot(
    OCCUPIED, query, separate_output, 0.0);

Vec3f aliased_query = query;
bool aliased_result = map.getNearestInfCellNot(
    OCCUPIED, aliased_query, aliased_query, 0.0);
```

由于查询点不在栅格中心，`max_dis=0` 时正确结果必须为 `false`。实际结果：

```text
separate_inf_result  = false
aliased_inf_result   = true
separate_prob_result = false
aliased_prob_result  = true
```

GTest 结果：

```text
[==========] Running 1 test from 1 test suite.
[  FAILED  ] NearestCellSearch.AliasedInputOutputHonorsMaximumDistance
[  PASSED  ] 0 tests.
[  FAILED  ] 1 test.
```

XML 证据保存在本机：

```text
/home/nuc/super_ws/build/test_results/rog_map/gtest-rog_map_nearest_cell_alias_test.xml
```

复现命令：

```bash
cd /home/nuc/super_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  run_tests_rog_map_gtest_rog_map_nearest_cell_alias_test
```

注意：当前 Catkin 测试目标即使 GTest 用例失败，也可能使最外层 `catkin_make` 返回 0；判断结果时
必须读取 GTest 输出或 XML 中的 `failures` 字段，不能只看 shell 返回码。

## 6. 运行时因果验证

### 6.1 方法

在不改变任何判断条件、返回值、目标值和规划参数的前提下，临时加入只读诊断：

- 在 ROGMap 中保存调用前输入，检测输入输出是否别名，并计算返回点到原始输入的真实距离；
- 在 FSM 中记录查询前后目标、返回值、目标位移和坐标有限性；
- 前20次、每100次以及任何失败、非有限、超过 `max_dis` 或大位移事件都强制输出；
- 使用同一套40×40×30 m 场景、30 m仿真雷达、官方 SUPER 核心和任务一参数运行到约165 s；
- 取证后撤销全部临时日志代码，再重新编译官方执行路径。

原始控制台日志保存在本机：

```text
/home/nuc/task1_logs/failure_cases/official_super_alias_diagnostic/console.log
```

### 6.2 结果

| 指标 | 结果 |
|---|---:|
| SUPER 退出时报告的总重规划次数 | 2149 |
| 别名诊断序号 | 超过2000 |
| 输出的代表性/异常诊断样本 | 40组 ROGMap + 40组 FSM |
| 样本中输入被输出初始化同步改成非有限值 | 40/40 |
| 真实返回距离超过 `max_dis=3 m` | 0 |
| 查询失败或输出非有限 | 0 |
| 样本中目标位移 | 全部为0 m |
| 高层新目标 | 4次，推进至约7.95 m |
| `Traj finish` | 73次 |
| `Omg or thr or Pos violation` | 20次 |
| 本轮 `NaN or Inf` 日志 | 0次 |

20次优化失败分成四段，仿真时间约为 `37.162–37.231 s`、`38.851–38.892 s`、
`92.556 s`、`125.956–126.355 s`。诊断器对所有失败、非有限、超距和大位移事件都会无条件打印，
但这些失败附近没有对应的异常别名查询记录；后续目标仍从约1.95 m推进到4.45、5.95、7.95 m。

### 6.3 为什么本轮“代码缺陷存在，但没有影响结果”

函数在写 `nearest_pt=NaN` 之前已经根据原始输入算好了 `start_id`。当高层给出的目标栅格本身不是
膨胀障碍时，搜索邻域的第一个候选就是目标所在栅格，并立即返回同一个栅格中心。此时距离上限虽因
别名被破坏，但搜索根本没有走远，所以输出仍与输入一致。

只有当起点/目标栅格不满足查询条件，需要继续搜索，或者整个邻域都没有匹配栅格时，这个缺陷才可能
表现为超距返回或失败后污染调用变量。因此它是需要修复的潜在安全问题，但本次运行给出了一个明确
反例：轨迹优化失败可以在别名查询输出完全稳定时发生。

### 6.4 因果结论

- 已证明：缺陷来自港大官方 SUPER 当前内置源码，而不是本项目后来添加的补丁。
- 已证明：任务一运行会执行该别名调用，输入引用会在函数内部被污染。
- 未观察到：本轮目标被移动、超过3 m搜索限制、查询失败或非有限目标。
- 可以排除：把本轮20次优化失败直接归因于该别名缺陷。
- 尚未排除：在目标落入障碍、地图边界或无可用邻居的其他场景里，该缺陷造成真实规划故障。

当前任务一轨迹问题应继续从轨迹完成后重复重规划、优化器输入状态、动力学约束和上层目标交接时序
向下追踪，而不能先修这个缺陷后就宣称当前问题已经解决。

## 7. 推荐修复边界

本缺陷仍应作为独立安全修复，但根据运行时证据，它不再作为当前轨迹异常的首要根因。修复时仍要
保持单变量，暂时不同时修改速度、加速度、障碍膨胀、0.1 m 到点门槛、CIRI 或任务一返航条件。

推荐在两个底层查询函数中：

1. 进入函数后立即复制原始查询点，例如 `const Vec3f query_pos = start_pos`。
2. 全部索引和距离计算只使用 `query_pos`。
3. 使用局部候选结果，查询成功后才写入 `nearest_pt`。
4. 非别名调用失败时继续维持官方原有的 `nearest_pt=NaN` 语义。
5. 别名调用失败时保留原始输入点，避免调用者变量被破坏。
6. 在 FSM 中补充返回值检查，禁止失败结果继续进入 `ReplanOnce()`；该调用点保护应在底层修复
   验证后作为独立提交处理，避免一次改变两个变量。

## 8. 验证计划

### 阶段一：单元测试

- 当前失败测试必须转为通过。
- 增加别名成功、别名失败、非别名失败、普通图、膨胀图五类测试。
- 明确检查失败后的输出语义和所有坐标是否为有限数。

### 阶段二：构建回归

- 完整 Release 编译 SUPER 工作空间。
- 不允许新增编译错误或链接错误。
- 保持唯一项目接口适配 `frame_id="camera_init"` 不变。

### 阶段三：缺陷修复后的同场景 SITL A/B

使用与官方基线完全相同的 40×40×30 m 场景、30 m 雷达范围、初始位置、参数和任务启动时序，
比较：

- 最近邻查询失败/超距次数；
- 每次重规划前后的原始目标和修正目标；
- `PlanFromRest` 与 `ReplanOnce` 次数；
- `NaN/Inf` 次数；
- 轨迹终点漂移距离；
- 飞行高度、跟踪误差和最小障碍距离；
- 是否仍在固定目标附近反复生成轨迹。

该 A/B 用于证明修复没有副作用，不能再预设它会消除当前20次轨迹优化失败。当前轨迹异常应另建
因果链，对每次优化失败记录输入轨迹状态、约束违规量、安全走廊和跟踪状态。

## 9. 安全结论

该缺陷位于起点/目标安全修正链路，不是单纯日志问题。修复和回归完成前：

- 不应把当前任务一版本用于有桨自主飞行；
- 不应通过增大高度围栏、减小安全距离或过滤障碍点来掩盖轨迹异常；
- 不应把一次 `ReplanOnce succeed` 当作完整安全验证；
- 真机前必须完成固定桨/无桨、低速、限高和人工随时接管测试。

## 10. 结论

本报告用三层证据得出两个不同结论：

1. 源码审计与最小失败测试证明，港大官方 SUPER 当前内置 ROGMap 的最近邻接口确实存在输入输出
   别名缺陷，能够绕过最大搜索距离；这是事实，不是本项目补丁造成的。
2. 保持行为不变的同场景运行时诊断证明，本轮任务一里该缺陷没有移动目标、没有产生超距结果，也
   没有在20次轨迹优化失败时输出异常值；因此它不是当前轨迹问题的已证实根因。

后续应把它作为独立安全缺陷修复并回归，但当前调试主线应转向优化器失败的真实输入与重复重规划
机制。这样既不忽略官方缺陷，也不会为了符合先入结论而错误归因。
