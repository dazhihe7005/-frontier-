# 轨迹同步优化前可复现快照（2026-10-26）

本快照冻结 2026-10-26 最新实飞分析完成后、轨迹时钟与安全语义优化开始前的代码状态。
它的用途是：后续优化若造成平滑性、安全性或任务生命周期退化，可以准确恢复到本版本，
而不是依赖某台 NUC 上未提交的工作树。

## Git 边界

### 工程适配仓库

- GitHub：`git@github.com:dazhihe7005/-frontier-.git`
- 分支：`main`
- 优化前基线父提交：`dc70ad272d9744a4624455cae98bc8e0937e2fa2`
- 冻结标签：`pre-trajectory-sync-optimization-20261026`
- 标签指向的提交包含当前工程源码、配置、launch、测试、文档和本文件所述 SUPER 补丁。

### SUPER/ROG-Map 核心

- 上游：`https://github.com/hku-mars/SUPER.git`
- 上游基线：`2ad3419c127a617c6d7df6925e81a14175a9c096`
- 本机冻结提交：`49136f1`（分支 `mine-uav/pre-optimization-20261026`）
- 可移植补丁：`patches/super_pre_optimization_snapshot_20261026.patch`
- 补丁 SHA-256：`26066fdd2a69f01da9909fdfde0d8caa139dd6e6551e50f1362544bd183bb596`
- 补丁范围：66 个文件，5195 行新增、509 行删除，包括所有此前未跟踪的消息、头文件和测试。

SUPER 的远程仍是官方上游，未尝试向 `hku-mars/SUPER` 推送项目定制修改。完整定制内容通过
上述补丁进入用户自己的 GitHub 仓库。

## 真机实际源码解析

在 `scripts/setup_fastlio2_super_env.sh` 环境中：

```text
mine_uav_control = /home/nuc/frontier_runtime_ws/src/mine_uav_control
                 -> /home/nuc/frontier-upload
super_planner    = /home/nuc/super_ws/src/SUPER/super_planner
rog_map          = /home/nuc/super_ws/src/SUPER/rog_map
```

`/home/nuc/super_ws/src/mine_uav_control` 是旧副本，不属于本快照的正式运行入口。

## 从 GitHub 重建

```bash
git clone git@github.com:dazhihe7005/-frontier-.git frontier-upload
cd frontier-upload
git checkout pre-trajectory-sync-optimization-20261026

cd ..
git clone https://github.com/hku-mars/SUPER.git SUPER
cd SUPER
git checkout 2ad3419c127a617c6d7df6925e81a14175a9c096
git am ../frontier-upload/patches/super_pre_optimization_snapshot_20261026.patch
```

应用后 SUPER 树应对应本机冻结提交 `49136f1` 的文件内容。可再次计算补丁 SHA-256，确认下载
和复制过程中没有改变。

## 冻结时验证

### 工程适配仓库

```bash
cd /home/nuc/frontier_runtime_ws
catkin_make --pkg mine_uav_control -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_mine_uav_control
catkin_test_results --all --verbose build/test_results/mine_uav_control
```

- Release 构建通过；
- 共执行 493 项测试，490 项通过、3 项失败、0 项错误；
- 三项失败均为已有静态文本断言与当前等价实现不一致，并非本次冻结时新增的编译或运行崩溃：
  1. Omni-LIVO 迁移测试要求 `task1_real.launch` 直接 include 固定文件名；
  2. 启动包装测试要求出现固定英文提示文本；
  3. 自动启动测试要求不带 `--wait` 的固定 `roslaunch` 字符串，而当前脚本使用
     `roslaunch --wait`。

为保持快照真实性，没有在冻结提交中顺手改测试或生产脚本。后续应独立提交，将这些断言改成
检查行为和参数语义。

### SUPER/ROG-Map

```bash
cd /home/nuc/super_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES='rog_map;super_planner' -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_rog_map run_tests_super_planner \
  -DCATKIN_WHITELIST_PACKAGES='rog_map;super_planner'
```

- Release 构建通过；
- 当前 CMake 注册的 3 个 ROG-Map 测试目标和 14 个 SUPER 测试目标全部通过，共 96 个
  GoogleTest case；
- CMake 报告的 VTK 可选工具缺失和 PCL 的 pcap/png/libusb 功能警告仍存在，但没有阻止上述
  两个包构建或测试。

补丁还在一个从官方基线创建的临时干净仓库中通过 `git am` 验证。应用前后树对象均为
`c1c18b685429e099dde4758f026c54f2c90c7f5e`，证明补丁包含本机冻结提交的完整文件内容。

## 有意不上传的内容

- ROS bag、PX4 ULog、相机图像和其它实飞原始数据：体积大，继续保存在
  `/home/nuc/flight_logs/`；
- catkin 的 `build/`、`devel/`、`install/`、`log/` 与 `.catkin_tools/`：均可由源码重建；
- Python 缓存、IDE 配置和本机临时文件；
- 任何私钥、令牌或凭据。本次冻结前已按文件名和常见凭据格式扫描，未发现需提交的凭据。

最新问题证据位于 `/home/nuc/flight_logs/task1/20261026_211226/`。第二段 rosbag 因记录进程
异常结束保留为 `.bag.active`，分析时只在临时副本上重建索引，原始证据未修改。

## 本快照已知问题

1. 桥接器因短时障碍保持固定飞机时，SUPER 时间轨迹仍可能继续向前。
2. 恢复后五次重捕获会追赶已经领先的移动参考；速度治理只限制速度前馈，不同步重定时位置。
3. `max_governor_tracking_error=0.65 m` 混合了横向偏离和沿程时间落后，可能把同一路径上的
   时间不同步误判为 `SPEED_GOVERNOR_INFEASIBLE`。
4. 表面完整性软门仍可能在明显大于硬碰撞半径的回波距离触发悬停。
5. 工程适配与 SUPER 核心仍是两个源码边界；发布时必须同时使用本标签和对应 SUPER 补丁。

这些问题是下一轮优化的输入，不应在本快照提交中顺手修改。优化应单独提交，以便与本标签
进行代码、rosbag 回放和真机分级 A/B。
