# 参数参考

所有地图参数以 `grid_map.` 开头，距离单位米、角度单位弧度。参数在 `initMap()` 读取，不支持在线重配。优先使用 [最小配置](../config/grid_map.yaml)；实验配置覆盖的值见 [实验说明](experiment.md)。

## 首先调整这些参数

| 名称（省略 `grid_map.`） | 默认值 | 用途 |
|---|---:|---|
| `cloud_topic` | `grid_map/cloud` | 输入 PointCloud2 |
| `frame_id` | `map` | 地图固定坐标系 |
| `motion_frame` | `base_footprint` | 运动门控坐标系；避免相机/机械臂运动被误认为底盘移动 |
| `high_resolution` | 0.01 | 原始任务窗口体素边长 |
| `low_resolution` | 0.03 | 扩展窗口体素边长，通常应大于高层 |
| `map_size_z` | 0.60 | 从 Z=0 起的地图高度 |
| `expansion_margin` | 0.25 | 低层在任务包围盒四周扩张的距离 |
| `point_cloud_leaf_size` | 0.005 | PCL 体素滤波叶尺寸；增大可减少射线数量 |
| `stationary_frames` | 30 | 当前站位提交所需的成功融合帧数 |

先保证话题、TF 和任务区域正确，再调整分辨率、点云密度和帧配额。几何参数必须有限且为正，边距可为零。两层分辨率独立，接口不强制大小关系。

## 点云与运动门控

| 名称 | 默认值 | 用途 |
|---|---:|---|
| `cloud_min_range` | 0.2 | 相机坐标系内最小距离 |
| `cloud_reliable` | true | reliable 点云订阅；实机配置为 false，以兼容 best-effort 相机 |
| `local_update_range_x/y/z` | 5.0 / 5.0 / 3.0 | 相机坐标系各轴绝对范围 |
| `cloud_min_map_z` | 0.02 | 地图坐标系障碍端点最低高度 |
| `cloud_low_z_clear_rays` | false | 保留低 Z 回波的 miss 射线，不将端点作为障碍 |
| `motion_translation_threshold` | 0.03 | 相邻位姿和驻留锚点的位移门限 |
| `motion_rotation_threshold_rad` | 0.02 | 同上，转角门限 |
| `support_filter.enabled` | false | 高层内部障碍的地面连通过滤 |
| `support_filter.ground_z` | 0.0 | 支撑过滤的地面高度 |
| `support_filter.ground_band` | 0.02 | 地面种子高度容差 |

## 占用与距离场

| 名称 | 默认值 | 用途 |
|---|---:|---|
| `p_hit` / `p_miss` | 0.70 / 0.35 | hit/miss 的 Log-odds 更新概率 |
| `p_min` / `p_max` | 0.12 / 0.97 | 概率饱和边界 |
| `p_occ` | 0.80 | 占用判断阈值；自由阈值固定为 p=0.5 |
| `esdf_unknown_mode` | `task_roi` | `off`：未知不作为障碍；`task_roi`：地图内高度带未知作为障碍；`all`：全部未知作为障碍 |
| `ground_height` | 0.0 | 未知障碍高度带下界；不移动地图原点 |
| `esdf_roi_z_max` | 0.50 | 未知障碍高度带上界；已知障碍不受此高度限制 |
| `esdf_signed_distance` | true | 在障碍体素中输出负距离 |
| `esdf_max_distance` | 100000.0 | 无障碍源时距离与距离截断上限 |
| `esdf_recompute_change_voxels` | 128 | 高层累计占用状态变化达到阈值后重算；0 为每次提交重算 |
| `tsdf.enabled` | false | 开启高层 TSDF；碰撞查询仍使用占用和 ESDF |
| `tsdf.truncation_distance` | 0.05 | TSDF 表面截断带 |

低层可独立覆盖 `low_p_hit/miss/min/max/occ`、`low_esdf_unknown_mode`、`low_esdf_roi_z_max`、`low_esdf_signed_distance`、`low_esdf_max_distance`，默认继承对应高层参数。`low_tsdf.enabled=false`、`low_tsdf.truncation_distance=0.05` 独立配置。低层 ESDF 始终随有效帧重算。

ROS 概率默认值经 Log-odds 转换；直接使用算法类的 `LogOddsConfig` 是较宽松的独立默认模型。库内直接配置时以算法头文件为准，勿把概率直接写入 `*_log_odds` 字段。

## 跨站位 ICP

| 名称 | 默认值 |
|---|---:|
| `registration.enabled` | true |
| `registration.max_correspondence_distance` | 0.50 |
| `registration.maximum_iterations` | 40 |
| `registration.min_overlap` | 0.20 |
| `registration.max_fitness` | 0.25 |
| `registration.max_translation_correction` | 1.0 |
| `registration.max_rotation_correction_rad` | 0.50 |
| `registration.min_target_points` | 50 |

当局部点和全局目标点数量不足时，使用 TF 先验提交；达到点数条件后才执行 ICP 和验收门限。`transformation_epsilon`、`euclidean_fitness_epsilon` 只通过 C++ `registration::Config` 设置。

## 高度图

| 名称 | 默认值 | 用途 |
|---|---:|---|
| `height_map_enable` | true | 启用 bimax 高度图发布（需要 bimax_msgs） |
| `height_map_topic` | `/complex_area_plan/height` | 高度图话题 |
| `height_map_frame` | `base_footprint` | 输出坐标系 |
| `height_band_z_max` | 0.50 | 高度状态统计上界 |
| `height_completeness_z_max` | 0.20 | 已知比例统计上界 |
| `height_floor_tol` | 0.02 | 地面容差 |
| `height_z_tcp_min` | 0.07 | 工具最低可通行高度 |
| `height_vision_limit` | 0.10 | 工具/基座可通行分类分界 |
| `height_ceil_threshold_open` | 0.50 | 完全开放的顶面高度阈值 |

保持 `floor_tol <= z_tcp_min <= vision_limit <= ceil_threshold_open`。高度图仅在当前站位提交后可用。独立实验节点的 OccupancyGrid 可视化不是 bimax 高度图发布器。

## 观测编排

以下名称以 `observation.` 开头。

| 名称 | 默认值 | 用途 |
|---|---:|---|
| `task_topic` | `/area_designation` | 任务 JSON |
| `target_topic` | `/complex_area_plan/observation_target` | 目标 PoseStamped |
| `state_topic` | `/complex_area_plan/observation_state` | 状态 JSON |
| `wipe_topic` | `/complex_area_plan/wipe_request` | 擦拭请求 JSON |
| `wipe_complete_topic` | `/complex_area_plan/wipe_complete` | 擦拭完成 Bool |
| `max_stations_per_round` / `max_rounds` | 5 / 3 | 每轮站位上限和任务轮次上限 |
| `arrival_tolerance` | 0.15 | 目标到达容差 |
| `tick_period_ms` | 100 | 编排周期，最低 20 ms |
| `navigation_enabled` | false | 直接调用 Nav2；统一入口实机强制开启，仿真关闭 |
| `exit_on_complete` | false | 完成后退出节点；统一入口仿真开启，实机关闭 |
| `wipe_z_min/max` | 0.02 / 0.20 | 擦拭评估高度带 |
| `arm_reach` / `arm_entry_depth` | 0.85 / 0.25 | 可达半径与简化伸入检查深度 |
| `opening_width` | 0.12 | 开口所需宽度 |
| `sensor_range` / `sensor_half_fov` | 1.20 / 0.65 | 可见性射线范围与半视角 |
| `candidate_spacing` / `candidate_standoff` | 0.20 / 0.55 | 候选采样间距和观测退让距离 |
| `min_wipeable_cells` | 8.0 | 最小连通可擦栅格数 |
| `max_unknown_ratio` | 0.18 | 可擦区域附近允许的未知比例 |
| `travel_penalty` | 0.08 | 行驶距离评分惩罚 |
| `max_candidates` | 64 | 候选站位数上限 |

任务格式、状态机与擦拭回执见 [观测规划](observation_planning.md)。

Nav2 参数不带 `observation.` 前缀，与 `cleaning/orchestrator.py` 的命名一致：

| 名称 | 默认值 | 用途 |
|---|---|---|
| `navigate_action` | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` 服务名 |
| `action_server_timeout` | 5.0 | 等待 action server 的秒数 |
| `nav_timeout` | 120.0 | 发送目标后等待接受及执行结果的总秒数 |

两个超时值必须有限且大于零，使用单调时钟，不受 ROS 时间跳变影响。
