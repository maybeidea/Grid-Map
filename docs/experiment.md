# Gemini 336 深度点云实验

## 启动与配置

```bash
ros2 launch plan_env gemini336_mapping_experiment.launch.py use_rviz:=false
```

所有节点参数在 [gemini336_mapping_experiment.yaml](../config/gemini336_mapping_experiment.yaml)。`/**` 区块统一两套地图实例的配置，节点区块只保留各自参数。安装式构建后修改源码 YAML 需要重新构建/安装。

Launch 仅提供三个开关：`use_rviz=true`、`rviz_config=包内配置路径`、`use_observation_planner=true`。修改场景、帧率、采样步长和输出目录应编辑 YAML。关闭观测规划节点时，同时将场景节点的 `observation_planner_enabled` 改为 `false`，以采用固定站位序列。

初次验证可将 `publish_rate` 调为 `1.0`、`pixel_stride` 调为 `8`。确认有地图提交和观测状态后，再逐步恢复默认 `5 Hz`、步长 `2`。每站位 `frames_per_observation=10`，默认 `wait_for_fusion=true`，切站等待 `/grid_map/mapping_status` 的提交状态。

## 场景与数据流

`gemini336_depth_scene.py` 发布带时间戳的相机 TF、PointCloud2 和 640×480 的 `32FC1` 深度图。相机内参为 `fx=fy=387.229248`、`cx=321.046387`、`cy=243.449692`，深度单位米，有效范围 `0.2–5.0 m`。

单开口 U 型结构内宽 `0.30 m`、深 `0.50 m`、墙厚 `0.08 m`，后侧接房间墙面。`scenario=front_opening` 或 `side_obstacle` 决定初始站位。启用观测规划后，后续站位由信息增益评分选择。每轮最多 5 个站位、最多 3 轮观测/擦拭，擦拭完成回执推动下一轮。

独立地图节点和观测规划节点各持有一个 GridMap；前者用于可视化/导出，后者用于目标选择。实验开销包含两次建图。

| 话题 | 数据 |
|---|---|
| `/camera/depth/points` | 合成点云 |
| `/grid_map/heightmap` | 独立节点的高度分类 OccupancyGrid |
| `/grid_map/occupied_voxels` | 三维已知占用体素 |
| `/grid_map/esdf_slice` | intensity 为 ESDF 距离的切片点云 |
| `/grid_map/mapping_status` | 当前站位、融合计数、是否可移动 |
| `/scene/ground_truth`、`/scene/reference_markers` | 障碍真值与参考标记 |
| `/complex_area_plan/observation_target` | 观测目标位姿 |
| `/complex_area_plan/wipe_request` | 本轮可擦区域 |

## 输出与指标

默认输出目录固定为 `/tmp/plan_env_gemini336`，不会自动创建 run_id 子目录。需要分组比较时，同时修改独立节点的 `visualization_dir` 和评价节点的 `output_dir`。

每个高层版本可导出高度分类 PPM、交互地图 HTML、体素数据 JSON、float32 距离场和 `versions_3d.csv`。

直接用浏览器打开 `map_v<版本>_esdf3d.html`，无需联网。页面沿用 `test/visualize_round_coverage.py` 的深蓝主题、指标卡片和图例配色：

- `ESDF 3D` 按实际高度渐变显示 ESDF 距离 ≤ 0 的体素，叠加任务 ROI 和米制坐标网格；`HeightMap` 切换为六类高度状态俯视图，PPM 使用相同分类配色。
- 左侧显示体素数量、占用已知率、高度图已观测比例和 ESDF 是否复用；右侧显示图层开关、点大小和图例。已知率由原始占用状态计算，与 ESDF 体素数量分别标注。
- 拖动旋转，Shift/右键拖动平移，滚轮缩放；支持 Top / Front / Side、自动旋转、复位和 `Save PNG` 导出当前地图视图。高度图模式拖动平移。
- 三维显示均匀抽样至最多 30,000 个点，指标和 JSON 保留完整数据。JSON 新增 `roi`、行优先的 `heightmap` 和 `esdf_current` 字段。

评价节点生成 `metrics.csv`、`metrics.json`：

- `occupied_iou`：预测占用与障碍真值的交并比，未知障碍影响并集。
- `free_space_precision`：预测自由空间的正确比例。
- `heightmap_accuracy`、`heightmap_known_accuracy`、`heightmap_coverage`：高度分类准确率和覆盖率。
- `esdf_mae_m`、`esdf_rmse_m`、`esdf_bias_m`：相对解析障碍距离的误差，含体素离散误差。

实验固定使用 `0.02/0.04 m` 双层分辨率、`0.02 m` 点云叶尺寸、`1.8 m` 地图高度、`0.8 m` 扩张边距、每站位 2 次成功融合，关闭 ICP、开启地面清空射线。高层默认少于 128 个状态变化体素时复用 ESDF；精确距离对照可将该阈值设为 `0`。
