# Gemini 336 深度点云仿真实验

该实验把双目深度图、相机位姿和 `plan_env::GridMap` 串成一个可重复的 ROS 2 流程：

```text
gemini336_depth_scene.py
  640x480 32FC1 depth image + PointCloud2 + map->camera TF
                 |
                 v
grid_map_standalone_test
  high/low resolution occupancy + ESDF + height classification
                 |
                 +--> /grid_map/heightmap       (nav_msgs/OccupancyGrid)
                 +--> /grid_map/occupied_voxels (PointCloud2, 3D occupied voxels)
                 +--> /grid_map/esdf_slice      (PointCloud2, intensity=ESDF)
                 +--> /scene/ground_truth       (PointCloud2, U 型结构和后墙真值)
                 +--> /scene/reference_markers  (真值实体、地图边界、观测点坐标)
```

## 运行

在 ROS 2 工作空间中构建 `plan_env` 后执行：

```bash
ros2 launch plan_env gemini336_mapping_experiment.launch.py
```

每次启动默认创建独立目录 `/tmp/plan_env_gemini336/<run_id>/`。地图快照、`versions_3d.csv`、`metrics.csv` 和 `metrics.json` 都属于本次运行。可手动指定目录名，便于复现实验：

```bash
ros2 launch plan_env gemini336_mapping_experiment.launch.py run_id:=baseline_stride2
```

切换两种初始观测条件：

```bash
ros2 launch plan_env gemini336_mapping_experiment.launch.py scenario:=front_opening
ros2 launch plan_env gemini336_mapping_experiment.launch.py scenario:=side_obstacle
```

不启动 RViz 时：

```bash
ros2 launch plan_env gemini336_mapping_experiment.launch.py use_rviz:=false
```

场景是单开口 U 型障碍：内侧开口宽 0.30 m、腔体深 0.50 m，后侧与房间墙面接触；墙厚 0.08 m。RViz 中半透明红色实体和红色点云是真值，绿色线框为高分辨率原始任务地图范围。`front_opening` 或 `side_obstacle` 只决定第一个种子站位；启用 `use_observation_planner=true` 后，后续站位完全由 `observation_planner.cpp` 按未知前沿的信息增益选择，直到找到达到擦拭下限的连通可擦区域。每轮最多 5 个站位，每个任务最多 3 轮观测和擦拭；每轮擦拭回执后会扣除该区域并评估剩余区域。站位上限时达到擦拭下限则正常擦拭，否则退出任务。

每个地图版本还会保存到本次运行目录：`*_heightmap.ppm` 是高度分类图，`*_esdf3d.html` 是浏览器中的交互式占据体素图，`*_esdf3d.json` 保留体素原点、分辨率、尺寸、占据语义和 occupancy state，`*_esdf_distance.f32` 是与体素线性顺序一致的 little-endian float32 signed ESDF 距离。

地图更新分为两层：扩展低分辨率层在每个有效点云帧更新并计算 ESDF，用于实时避障；原始高分辨率层只在当前站位达到 `stationary_frames` 并提交时更新。高分辨率 ESDF 还会比较占据状态变化量，小于 `grid_map.esdf_recompute_change_voxels` 时复用上一次距离场，避免重复计算整张体积。

规划器接口为 `/complex_area_plan/observation_target`（目标位姿）、`/complex_area_plan/observation_state`（状态）、`/complex_area_plan/wipe_request`（本轮可擦栅格）和 `/grid_map/mapping_status`（站位融合门）。新人可用 `ros2 topic echo` 查看目标、`moving_to_observation`、`wiping`、`assessing_remaining_region` 和最终状态。

`gemini336_mapping_evaluator.py` 按版本与合成场景真值比较，写出 `metrics.csv` 和 `metrics.json`：

- `occupied_iou`：已知 occupancy state 中，预测占据体素与真值障碍体素的交并比；未知体素不计入预测占据，但真值障碍中的未知体素仍计入并集，因此该值也会反映障碍未观测完整的问题。
- `free_space_precision`：预测为空闲的体素中，真值确实不在障碍物内部的比例。
- `heightmap_accuracy`：整张高度图按开放、基座可通行、工具可通行、过低、非地板/障碍、未知六类计算准确率；`heightmap_known_accuracy` 是只在已预测单元上的准确率，`heightmap_coverage` 是非未知单元比例。OccupancyGrid 的编码值 `0/25/50/75/100` 会映射成这五种已知类别。
- `esdf_mae_m` / `esdf_rmse_m`：地图 ESDF 与真值障碍盒解析 signed distance 的平均绝对误差/均方根误差，单位米；`esdf_bias_m` 为带符号平均误差。真值障碍表面采样栅格的几何量化误差会进入该指标。

## 参数说明

| 参数 | 默认值 | 含义与调节建议 |
|---|---:|---|
| `use_rviz` | `true` | 是否启动 RViz。远程或无图形桌面运行时设为 `false`。 |
| `rviz_config` | 包内配置 | RViz 显示配置文件路径；只有启用 RViz 时使用。 |
| `scenario` | `front_opening` | 初始观测站位顺序。可选 `front_opening`（开口正面）和 `side_obstacle`（左侧障碍侧面）。 |
| `use_observation_planner` | `true` | 启用 `observation_planner_node`。开启后只有规划器的完成决策才能结束实验；关闭时使用场景内置的固定站位序列。 |
| `publish_rate` | `5.0` Hz | 合成相机帧和点云发布频率。频率越高，CPU 处理和传输负担越大；低速验证可先设 `1` 到 `2` Hz。 |
| `pixel_stride` | `2` px | 深度图采样步长。`1` 使用全部 640×480 光线；`2` 每隔一个像素采样；数值越大速度越快但边界更稀疏。 |
| `frames_per_observation` | `10` 帧 | 相机在每个观测站位连续发布的帧数。需足以满足地图的局部融合帧数；太小会导致站位未完成融合就切换。 |
| `wait_for_fusion` | `true` | 是否在切换观测站位前等待地图融合完成。开启时场景节点等待 `/grid_map/mapping_status` 对当前站位发布 `ready_to_move=true`；关闭时仅按 `frames_per_observation` 计数切换，适合快速调试但可能遗漏最后几帧融合。 |
| `observation.max_stations_per_round` | `5` | 每轮完整观测允许的最多站位数。达到上限时，若连通可擦区域达到擦拭下限则发出擦拭请求，否则以 `observation_aborted_below_wipe_threshold` 退出。 |
| `observation.max_rounds` | `3` | 每个任务允许的最多观测和擦拭轮数。每轮擦拭回执后会从后续判断中扣除已擦区域。 |
| `observation.min_wipeable_cells` | `8` | 连通已知自由栅格达到该数值时视为找到一个完整可擦区域。高分辨率下相同栅格数对应的物理面积更小。 |
| `observation.finish_on_complete` | `true`（本实验 launch） | 最多 3 轮擦拭完成或擦拭后剩余区域低于下限时，发布 `observation_complete` 并结束仿真。 |
| `high_resolution` | `0.02` m | 原始任务区域高分辨率体素边长。越小几何细节越多，内存和 ESDF 计算开销按体素总量快速增加。 |
| `low_resolution` | `0.04` m | 扩展区域低分辨率体素边长。用于更宽范围的实时地图，越大越省算力但细节更少。 |
| `cloud_leaf_size` | `0.02` m | 融合前 PointCloud2 体素滤波尺寸。增大可减少重复射线和 ESDF 负载，但会丢失细小表面；`pixel_stride=1` 仍表示相机端采集全部像素。高分辨率压力测试可先用 `0.04` m。 |
| `grid_map.esdf_recompute_change_voxels` | `128` | 高分辨率站位提交后，只有占据状态变化体素达到该阈值才重新计算高分辨率 ESDF；设为 `0` 强制每次提交重算。低分辨率 ESDF 仍随每帧实时更新。 |
| `output_root` | `/tmp/plan_env_gemini336` | 实验结果根目录。每次运行会在其下建立一个 `run_id` 子目录。 |
| `run_id` | 启动时间戳 | 本次运行目录名。设置固定名称前请确保该目录是新目录，以免文件名覆盖。 |
| `camera.width` / `camera.height` | `640` / `480` | 合成深度相机分辨率，单位像素。改动时应同时使用与相机匹配的内参。 |
| `camera.fx` / `camera.fy` | `387.229248` | 相机水平/垂直焦距，单位像素。决定像素射线张角。 |
| `camera.cx` / `camera.cy` | `321.046387` / `243.449692` | 主点坐标，单位像素，决定光学坐标射线中心。 |
| `camera.frame_id` | `gemini336_depth_optical_frame` | 点云所在相机光学坐标系名称，必须与发布的 TF 子坐标系一致。 |
| `map_frame` | `map` | 场景真值、任务区域和相机 TF 使用的固定世界坐标系。 |
| `depth.min_range` / `depth.max_range` | `0.2` / `5.0` m | 合成深度有效范围；范围外像素作为无回波处理。 |

Launch 中另外固定了地图内部参数：`map_size_z=1.8 m` 为垂直地图范围，`cloud_leaf_size=0.02 m` 为融合前点云体素滤波尺寸，`cloud_low_z_clear_rays=true` 保留地面回波的清空射线但不把地面当障碍，`stationary_frames=2` 为一个站位提交地图所需的成功融合帧数，`stationary_frames_min=1` 为开始使用高度图的最少融合帧，运动阈值为相邻帧平移 `0.03 m`、旋转 `0.02 rad`。`registration.enabled=false` 表示当前基线不启用跨站位 ICP 重定位。

建议新人第一次使用 `use_rviz:=false publish_rate:=1 pixel_stride:=8 use_observation_planner:=true`，确认规划器状态从 `waiting_for_station_commit` 到 `observation_complete` 后，再逐步将采样步长降到 2 或 1、提高帧率，并对比指标和运行时间。

## 相机与坐标

默认内参来自 Gemini 336 的 640x480 配置：

```yaml
fx: 387.229248
fy: 387.229248
cx: 321.046387
cy: 243.449692
```

深度图使用 `32FC1`、单位为米；点云是相机光学坐标系中的 `x=(u-cx)z/fx`、`y=(v-cy)z/fy`、`z=depth`。实验发布带时间戳的 `map -> gemini336_depth_optical_frame`：开口起始位姿为 `(0.00, -0.75, 1.15)` m，侧障碍起始位姿为 `(-0.65, 0.24, 1.15)` m。相机光学坐标到地图坐标采用绕 X 轴 -90 度，再按观测朝向绕 Z 轴旋转。

U 形内开口为 0.30 m，侧臂深度为 0.50 m，墙厚为 0.08 m；U 形后壁与房间后墙相接。原始地图范围为 `x=[-1.0, 1.0] m`、`y=[-0.9, 0.85] m`，RViz 绿色框线直接显示该范围。

## 可调实验变量

- `pixel_stride`：点云下采样步长，默认 2；设为 1 可使用完整 640x480 光线。
- `high_resolution` / `low_resolution`：高、低分辨率占据图，默认 0.02 / 0.04 m。
- `publish_rate`：深度帧率，默认 5 Hz。
- `scenario`：初始观测方向，`front_opening` 或 `side_obstacle`。
- `frames_per_observation`：每个观测位置的驻留帧数，默认 10。
- `depth.min_range/max_range`：有效深度范围。

建议记录每个版本的未知、自由、占据体素数量、heightmap 分类数量和 ESDF 最小距离；改变 `pixel_stride`、深度噪声或位姿扰动后，与无噪声基准比较占据 IoU、heightmap 分类准确率和 ESDF 误差。
