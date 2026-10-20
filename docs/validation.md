# 整理后的验证记录

验证日期：2026-10-20。环境：ROS 2 Humble、GCC、Release，`OMP_NUM_THREADS=4`。

## 统一入口与 Nav2 接入

- 新入口 `mapping_test.launch.py mode:=sim|real` 构建和安装通过。
- 原有 26 项测试加 8 项 Nav2 测试通过。Nav2 测试使用真实 ROS action 客户端和
  假服务器，覆盖目标位姿、成功、拒绝、执行中止、服务等待超时、执行超时取消、
  stop 取消、延迟接受及任务替换后的旧回调隔离。
- 仿真冒烟验证：`publish_rate=2.0`、`pixel_stride=8`，生成 3 个地图版本，
  heightmap、占据 JSON/HTML、ESDF 文件和评价结果均写入命令行指定目录。
- 实机模式使用外部合成相机/TF 和假 Nav2 服务验证：确认入口不启动场景/评价器，
  `navigate_action` 覆盖生效，状态经过 `waiting_for_station_commit` →
  `moving_to_observation` → `station_observed`，同时生成地图文件。
  分别验证了 reliable 和 best-effort 点云发布端；该验证不代表真实硬件或导航性能验收。
- 验证了 YAML 节点专属参数与命令行覆盖的优先级，避免实机模式被仿真 YAML 的
  `navigation_enabled=false` 覆盖；Python/YAML 语法和文档中的任务 JSON 命令通过检查。

## 构建与回归

- 构建算法库、ROS 建图库、两个 C++ 节点和测试可执行文件。
- 22 个 GridMap 测试、4 个算法测试全部通过。
- 安装到独立前缀，再以外部 CMake 工程 `find_package(plan_env)`、链接 `plan_env::plan_env`，编译与运行通过。
- Python/YAML 语法、当前文档相对链接、Git 空白检查通过。
- 无界面 Gemini 实验使用 `publish_rate=2.0`、`pixel_stride=8`，验证共享 YAML、多个站位提交、可视化文件和评价指标生成；完成后退出实验。此实验是流程冒烟验证，不代表地图精度验收。

当前环境没有 `bimax_msgs`，因此编译与运行覆盖的是无 bimax 高度图发布器的分支；实际 `bimax_msgs/HeightMap` 发布及真实相机接入尚未验证。独立节点的 OccupancyGrid 高度可视化已运行。

## ESDF 微基准

对照整理前工作区快照与整理后实现。96×80×40（307,200）体素，分辨率 0.02 m，`std::mt19937(42)` 生成约 5% 障碍，signed ESDF；GCC `-O3 -std=c++14 -fopenmp`。每次先预热 5 次，再测 50 次更新平均耗时，前后交替执行三轮。

| 轮次 | 整理前（ms/次） | 整理后（ms/次） |
|---|---:|---:|
| 1 | 10.458 | 8.374 |
| 2 | 10.913 | 8.513 |
| 3 | 9.498 | 8.338 |

三轮中位数由 10.458 ms 降为 8.374 ms，约减少 20%。这是共享运行环境下的单算法测量，不能等同于完整建图或实机延迟改善。

静态内存变化：`OccupancyVoxel` 从 8 字节缩减为 4 字节；移除高层局部占用密集体积（旧实现每高层体素额外 8 字节）和可选局部 TSDF（每体素额外 8 字节）。原始射线证据仍保存到站位提交，快照仍复制高低两层体积。
