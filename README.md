# plan_env

面向任务多边形的双分辨率建图与观测规划。高分辨率层在站位稳定、融合达到配额后提交；低分辨率层随有效点云实时更新。占用、可选 TSDF、ESDF 和不可变快照组成查询接口。

## 快速开始

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select plan_env --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch plan_env gemini336_mapping_experiment.launch.py use_rviz:=false
```

在含有已构建依赖的 ROS 2 工作空间执行。`bimax_msgs` 可选；找不到它时，仍可运行独立实验，但不提供 `bimax_msgs/HeightMap` 发布。CMake 使用工具链所选编译器，未指定构建类型时使用 Release。

真实相机接入可复制 [最小配置](config/grid_map.yaml)，先调整点云话题、坐标系、两层分辨率、地图高度、扩张边距和每站位帧数。参数只在初始化时读取，改变配置后重启节点。无需逐项设置算法参数。

```bash
ros2 run plan_env observation_planner_node --ros-args --params-file /path/to/grid_map.yaml
```

节点等待 `/area_designation` 的任务 JSON，任务格式与观测/擦拭接口见 [观测规划](docs/observation_planning.md)。库调用顺序：`initMap(node)` → `startTaskMapping(polygon)` → 等待 `isMappingReadyForPlanning()` → 查询或冻结快照 → `stopTaskMapping()`。

## 代码导航

| 位置 | 职责 |
|---|---|
| `include/plan_env/grid_map.hpp` | 对外建图接口和私有运行状态 |
| `include/plan_env/map_snapshot.hpp`、`grid_geometry.hpp` | 无 ROS 依赖的数据契约 |
| `src/mapping/grid_map_pipeline.cpp` | 最新帧队列、双层更新、站位提交、发布主流程 |
| `src/mapping/grid_map_config.cpp` | 参数读取、验证及 ROS 输入输出初始化 |
| `src/mapping/` 其余文件 | 任务生命周期、点云预处理、融合、运动门控、快照与高度图适配 |
| `src/algorithms/` | 占用、TSDF、ESDF、射线、支撑过滤、ICP、观测规划 |
| `src/utils/` | 几何工具与快照查询工具 |
| `src/nodes/` | 独立实验和观测编排入口 |
| `test/` | 建图回归测试、ESDF 暴力参考对照、支撑过滤验证 |

算法构建为 `plan_env_algorithms`，ROS 建图适配构建为 `plan_env`；现有客户端继续链接 `plan_env`。

## 文档

- [架构与算法](docs/architecture.md)：数据流、线程、查询语义、性能取舍。
- [参数参考](docs/parameters.md)：基本配置和进阶选项。
- [Gemini 336 实验](docs/experiment.md)：运行、输出与评价指标。
- [局部 ESDF 更新](docs/local_updates.md)：默认启用的精确增量更新及 2 线程上限。
- [性能对比与复现](docs/performance.md)：旧版内核、新版优化前后耗时和 CPU 时间。
- [迁移说明](docs/migration.md)：已删除的参数及接口变化。
- [观测规划](docs/observation_planning.md)：观测站位选择与擦拭编排。

`docs/archive/` 保存旧设计和排查记录，不再作为使用指南。

## 验证

```bash
colcon test --packages-select plan_env
colcon test-result --verbose
```

验证环境、覆盖范围和微基准见 [验证记录](docs/validation.md)。

ESDF 测试覆盖随机场、全占用/全自由场和并行三轴变换；建图测试覆盖帧配额、跨站位提交、移动期间低层更新、冻结/任务切换和过期 ESDF 标记。
