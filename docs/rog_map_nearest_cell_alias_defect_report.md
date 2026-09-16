# SUPER / ROGMap 最近邻查询输入输出别名缺陷报告

- 报告日期：2026-09-16
- 影响组件：港大 SUPER 内置 ROG-Map
- 本地官方基线：`2ad3419c127a617c6d7df6925e81a14175a9c096`
- 当前状态：缺陷已通过单元测试复现，尚未修改生产实现
- 风险等级：高（可能绕过查询距离限制，并可能向规划链路传播非有限坐标）

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

这是一个已经证实的底层接口缺陷。它能够解释“最近安全点被搜索到预期距离以外”以及
“失败后调用变量可能变成 `NaN`”的机制，但尚不能仅凭当前证据断言它是此前全部轨迹漂移和
优化器 `NaN/Inf` 的唯一原因。完整因果关系仍需修复后进行同场景 A/B 验证。

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

2026-09-16 检查官方 `master` 时，上述实现仍然存在。

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

## 6. 与任务一仿真现象的关系

### 6.1 已证实

- 别名调用会使普通图和膨胀图查询结果与非别名调用不一致。
- 别名调用能够绕过 `max_dis`。
- SUPER 官方源码中确实存在多个别名调用点。
- FSM 的一个别名调用点没有检查函数返回值。

### 6.2 已观察但尚未证明由本缺陷单独造成

恢复官方核心基线后的 40×40×30 m SITL 中观察到：

- 高层目标推进到约 7.32 m 后，SUPER 对旧目标反复 `PlanFromRest`。
- 零速度轨迹终点在多个局部位置之间漂移。
- 实际高度和目标误差逐步增大。
- 随后轨迹优化器密集报告 `NaN or Inf` 和线搜索失败。

这些现象与“重规划时目标或起点被最近邻函数异常移动/污染”在机制上相符，但现有 bag 没有记录
函数调用前后的内部变量，所以不能把相关性写成已经证明的直接因果关系。

### 6.3 仍需验证

- 发生优化器 `NaN/Inf` 的具体时刻，是否有某次别名查询失败并把目标或起点变成 `NaN`。
- 修复别名缺陷后，重复 `PlanFromRest`、局部终点漂移和优化器失败是否显著减少。
- 官方硬编码 0.1 m 到点门槛是否仍然独立导致旧目标重规划。

## 7. 推荐修复边界

本缺陷应先作为单变量修复，暂时不同时修改速度、加速度、障碍膨胀、0.1 m 到点门槛、CIRI
或任务一返航条件。

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

### 阶段三：同场景 SITL A/B

使用与官方基线完全相同的 40×40×30 m 场景、30 m 雷达范围、初始位置、参数和任务启动时序，
比较：

- 最近邻查询失败/超距次数；
- 每次重规划前后的原始目标和修正目标；
- `PlanFromRest` 与 `ReplanOnce` 次数；
- `NaN/Inf` 次数；
- 轨迹终点漂移距离；
- 飞行高度、跟踪误差和最小障碍距离；
- 是否仍在固定目标附近反复生成轨迹。

只有 A/B 显示缺陷消失且没有降低避障安全性后，才考虑继续处理 0.1 m 到点门槛和高层目标交接协议。

## 9. 安全结论

该缺陷位于起点/目标安全修正链路，不是单纯日志问题。修复和回归完成前：

- 不应把当前任务一版本用于有桨自主飞行；
- 不应通过增大高度围栏、减小安全距离或过滤障碍点来掩盖轨迹异常；
- 不应把一次 `ReplanOnce succeed` 当作完整安全验证；
- 真机前必须完成固定桨/无桨、低速、限高和人工随时接管测试。

## 10. 结论

本报告已经用源码语义和可重复失败测试证明：ROGMap 最近邻查询在输入输出别名时会破坏输入，
并绕过调用者提供的最大搜索距离。它是当前 SUPER 轨迹问题中一个需要优先修复的底层确定性缺陷。

下一步应严格保持单变量：先修复这两个查询函数并让回归测试通过，再运行同场景 A/B。只有这样
才能判断它对目标漂移、重复重规划和优化器 `NaN/Inf` 的实际贡献，避免再次用参数补丁掩盖根因。
