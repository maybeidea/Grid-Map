# 原始地图观测规划与擦拭编排

## 修改范围

本次在 `plan_env` 中增加 `ObservationPlanner` 和可选节点
`observation_planner_node`。规划器只读取 `GridMap::snapshot()` 的高分辨率原始
地图；扩展低分辨率地图仍用于底盘导航和碰撞检查，不参与擦拭区域完整度判断。
快照新增 `source_roi_polygon`，保存 `startTaskMapping()` 收到的原始多边形，避免
把建图窗口的矩形边界误当成待擦区域。

## 选择规则

每个二维栅格柱在擦拭高度带内折叠成 `FREE`、`OCCUPIED` 或 `UNKNOWN`。规划器先
在原始多边形内找已知自由区域，再检查其边界附近的自由通道。边界自由柱只是开口
候选；规划器还会在扩展地图的擦拭高度带上检查一条短直线伸入通道，并在通道中心
及左右边缘采样。采样柱必须全部为已知自由，未知、占用或地图越界都会拒绝该开口，
避免“区域内部开放但边界外有障碍”时误判可擦。观测站位放在开口外侧并朝向开口。

此检查只验证固定宽度和短深度的二维直线通道，不运行机械臂 IK、不生成完整伸臂轨迹，
也不检查手臂扫掠体积。它用于快速剔除明显不可进入的开口，不代替擦拭执行阶段的安全
控制。

首个观测不再按多边形墙面顺序遍历候选点，而是把每个未知前沿作为信息目标，按
可见未知栅格数、可见可擦栅格数和行驶距离评分，选择一次带来最大未知减少量的
站位。后续站位继续使用同一评分并降低已访问站位的权重，因此在达到阈值前尽量
使用最少的观测次数。射线穿过已知占用柱的候选会被丢弃。

“完整”使用待擦区域附近的未知比例定义，而不是整个扩展窗口的未知比例。附近
区域的未知比例低于 `observation.max_unknown_ratio`，同时必须存在开口和足够大的
已知自由连通区域（`observation.min_wipeable_cells`）才会进入擦拭。若剩余未知
区域没有任何候选射线可以到达，规划器会标记 `theoretically_unobservable`，允许
在已知区域可擦时继续，并把该原因写入擦拭请求。

擦拭完成后通过 `observation.wipe_complete_topic`（默认
`/complex_area_plan/wipe_complete`）发送 `Bool(true)`。已擦栅格会被记录并从下一
轮剩余区域中排除；若剩余区域仍值得处理，规划器重新等待建图门控并选择下一站，
否则保持完成状态。

## 与运动门控的协同

`observation_planner_node` 只在快照的
`mapping_status.motion_state == kReadyToMove` 且 `ready_to_move == true` 时选择并
发布目标。仿真中，非当前位置的目标发布后必须观察到 `kMoving`；实机 Nav2 模式
必须收到导航成功结果。两种模式都要求机器人位于目标附近、当前站位达到
`kReadyToMove`，才会把这一站标记为已观测。这保证了一个位置的局部地图达到
驻留帧配额并提交到全局地图后才移动到下一位置。

完整后节点调用 `freezeOccupancyAndEsdf()`，发布擦拭请求并暂停新的观测；收到擦拭
完成信号后调用 `clearEsdfFreezing()`，再评估剩余区域。目标位姿发布为
`geometry_msgs/msg/PoseStamped`，默认话题为
`/complex_area_plan/observation_target`，状态话题为
`/complex_area_plan/observation_state`。

## 运行

构建 `plan_env` 后启动：

```bash
ros2 launch plan_env mapping_test.launch.py mode:=sim
# 实机：外部相机、TF 和 Nav2 已启动
ros2 launch plan_env mapping_test.launch.py mode:=real
```

统一入口同时启动地图导出节点。实机导航使用与 `cleaning/orchestrator.py` 相同的
异步 `NavigateToPose` 接口，包含服务等待、执行超时、失败处理和旧回调隔离。
停止任务或替换任务会取消旧导航；实机任务完成后继续等待新任务。
参数、输入示例和导出格式见 [独立测试](experiment.md)。

任务输入沿用 `orchestrator.py` 的 `polygon/clean`、`run_complex_area_clean` 或
简化 `start` JSON，默认订阅 `/area_designation`。擦拭执行器应订阅
`/complex_area_plan/wipe_request`，完成后向 `wipe_complete_topic` 发布 `true`。

主要参数包括：

- `observation.wipe_z_min/max`：擦拭高度带；
- `observation.arm_reach`、`opening_width`：开口可进入判定；
- `observation.arm_entry_depth`：开口内的简化伸入检查深度；
- `observation.sensor_range`、`sensor_half_fov`：观测射线模型；
- `observation.max_unknown_ratio`：完整度阈值；
- `observation.min_wipeable_cells`：最小可擦区域；
- `observation.candidate_standoff`：观测站到开口/未知前沿的距离。

当前节点是独立编排入口，使用自己的 `GridMap` 点云订阅。已有上层编排器可以直
接复用 `ObservationPlanner`，或者把同样的三个状态事件（目标发布、地图提交、擦拭
完成）接到现有 action 状态机中。
