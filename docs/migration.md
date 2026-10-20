# 本次整理的迁移说明

## 参数与 API

| 旧项 | 当前用法 |
|---|---|
| `grid_map.resolution` | `grid_map.high_resolution`，不再接受别名优先级 |
| `grid_map.cloud_leaf_size` | `grid_map.point_cloud_leaf_size` |
| `grid_map.expansion_ratio` | 删除；地图扩张仅使用 `expansion_margin`，单位米 |
| `grid_map.stationary_frames_min` | 删除；高度图与规划均等待 `stationary_frames` 次成功融合后的提交 |
| `observation.min_unknown_gain` | 删除；从未参与候选评分或停止判断 |
| `observation.finish_on_complete` | 删除；该参数从未控制结束行为，状态机按完成/轮次条件结束 |
| `configure(..., expansion_ratio)` | 删除最后一个参数；`configure` 现在接受 3 项，`configureDualResolution` 接受 4 项 |
| `configureMotion(..., stationary_frames_min)` | 删除最后一个参数，仅保留位移、转角、帧配额 |
| `OccupancyVoxel::hit_count/miss_count` | 删除无人读取的计数，单体素由 8 字节缩减为 4 字节 |
| `MappingStatus::stationary_frames_min`、`TaskRegion::expansion_ratio`、`QualityVoxel`、`ObservationTarget::visible_opening` | 删除无效或多余的数据字段 |

`height_vision_limit` 保留且现在真正参与高度分类；此前读取后被固定的 `0.10` 覆盖。默认行为不变。

几何默认值统一为高层 `0.01 m`、低层 `0.03 m`、地图高度 `0.60 m`、扩张边距 `0.25 m`，直接构造和 ROS 初始化采用相同默认值。原 ROS 默认高度 `0.20 m`、边距 `0.8 m` 需显式配置才能保留；原直接构造高层 `0.05 m` 同理。实验 YAML 保留 `0.02/0.04 m`、高度 `1.8 m`、边距 `0.8 m`。

ROS 参数覆盖不会自动报出所有未知名称；请清理旧 YAML。仓库中的实验配置与 `mm_config` 参数读取已同步更新。

## 代码与文档

源码移动至 `src/mapping`、`src/algorithms`、`src/nodes`、`src/utils`，CMake 已同步。公共头文件路径保持不变，新增加无 ROS 依赖的 `grid_geometry.hpp`、`map_snapshot.hpp`、`ray_integration.hpp`。`grid_map.h` 和规划器仍调用的 `resetAstarBuffer()`、`odomValid()`、`getOdomDepthTimeout()` 保留为兼容适配。

客户端继续链接 `plan_env`；纯算法使用者可链接 `plan_env_algorithms`。重新构建依赖此包的 C++ 客户端，勿复用旧 ABI 产物。

原 `DUAL_RESOLUTION_MAP.md` 与 `README_GEMINI336_EXPERIMENT.md` 已整合进 `docs/architecture.md` 和 `docs/experiment.md`。历史报告、修改意见和设计草案保存在 `docs/archive/`，不代表当前行为。

实验只支持 `use_rviz`、`rviz_config`、`use_observation_planner` 三个 launch 参数。场景、帧率、输出路径等修改 YAML；不再沿用旧文档中的 `run_id:=...`、`scenario:=...` 启动方式。
