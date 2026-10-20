> 历史设计记录：不作为当前代码、参数或运行方式的依据。当前入口见 [README](../../README.md)。

# 点云采集与机器人运动状态

## 现有 `mobile_manipulator_controller/whole_body_planner/plan_env`

旧版 `GridMap` 的点云回调只把最新 `PointCloud2` 放入 mailbox。占据定时器取出它后，在点云时间戳上执行

```text
lookupTransform(map_frame, cloud_frame, cloud.header.stamp)
```

变换的平移保存为相机位置，旋转用于把点云变换到地图坐标系。随后每条相机到点的射线都会立即更新 occupancy 和 ESDF。代码没有订阅里程计或速度，也没有根据连续 TF 位姿判断静止；`has_odom_` 只表示 TF 查找成功。因此旧版会在机器人运动时融合点云，且 mailbox 会丢弃中间帧。

## 重构版状态机

重构版仍以时间戳 TF 作为唯一运动观测源。优先查询 `map_frame <- motion_frame`（默认 `base_footprint`），这样机械臂或相机的运动不会被误判为底盘运动；该 TF 不存在时退回点云 frame 的位姿。

```text
WAITING_FOR_POSE --(第一帧，仅建立基线)--> WAITING_FOR_POSE
WAITING/MOVING --(位姿稳定)--> STATIONARY_COLLECTING
STATIONARY_COLLECTING --(累计 30 帧)--> READY_TO_MOVE
任意静止态 --(位姿变化超过阈值)--> MOVING
MOVING --(位姿稳定)--> STATIONARY_COLLECTING（新位置）
```

只有 `STATIONARY_COLLECTING` 中的帧会进入地图。第一帧只建立运动基线；运动中的帧会被转换和过滤，但不会写入 `cloud_points_`、TSDF 或 occupancy。达到 30 帧后状态变为 `READY_TO_MOVE`，同一停靠点的后续帧被忽略，直到 TF 显示机器人已经移动。

状态可通过 `GridMap::mappingStatus()` 或 `GridMap::readyToMove()` 查询，也会复制到 `EsdfSnapshot::mapping_status`。`stationary_frame_count`、`position_index` 以及两种融合计数可用于让上层运动控制器在 30 帧完成后发出移动命令。

## 两类融合接口

地图更新前会根据状态选择两个独立入口：

- `fuseSamePositionFrame()`：同一停靠点的时间融合。
- `fuseDifferentPositionFrame()`：完成一个停靠点后，在新位置把结果合并到世界地图。

当前两个入口都调用原有射线更新，仅作为稳定的默认行为；策略边界已经分开，后续可以分别替换为同点平均、关键帧合并或其他融合方法，而不需要改运动状态机。

## 参数

可通过 `GridMap::configureMotion(translation_threshold, rotation_threshold_rad, stationary_frames)` 设置，也可以在 ROS 参数中覆盖：

- `grid_map.motion_frame`（默认 `base_footprint`）
- `grid_map.motion_translation_threshold`（默认 `0.03 m`）
- `grid_map.motion_rotation_threshold_rad`（默认 `0.02 rad`）
- `grid_map.stationary_frames`（默认 `30`）
