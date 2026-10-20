# 仿真与实机独立测试

## 统一入口

```bash
# 合成点云、TF、任务和模拟移动/擦拭回执
ros2 launch plan_env mapping_test.launch.py mode:=sim use_rviz:=false

# 外部相机、定位 TF 和已激活的 Nav2
ros2 launch plan_env mapping_test.launch.py mode:=real use_rviz:=false \
  cloud_topic:=/camera/depth/points motion_frame:=base_footprint \
  navigate_action:=/navigate_to_pose output_dir:=/tmp/plan_env_real_run1
```

旧入口 `gemini336_mapping_experiment.launch.py` 转发到同一入口，默认仍为仿真。
两种模式均启动观测规划节点和地图可视化/导出节点；两者各自持有 GridMap，会重复
建图。导出来自独立节点，不能视为规划节点的同一份冻结快照。

| 模式 | 默认配置 | 数据源与导航 | 任务结束 |
|---|---|---|---|
| `sim` | `gemini336_mapping_experiment.yaml` | 启动合成场景和评价器；场景响应观测点模拟移动 | 规划节点结束后关闭实验 |
| `real` | `real_mapping_test.yaml` | 不启动合成场景/评价器；直接调用 Nav2 `NavigateToPose` | 节点保持运行，等待下一任务 |

`params_file:=/absolute/path/config.yaml` 替换默认配置。YAML 支持 `/**` 公共参数
以及节点名（也可加前导 `/`）的 `ros__parameters`，可使用点号参数名或嵌套字典。
命令行覆盖参数优先于 YAML；`mode` 强制决定是否启用 Nav2 和结束后是否退出。

可用 launch 参数：

- `use_rviz`、`rviz_config`、`use_observation_planner`；关闭观测规划时，仿真自动采用固定站位序列。
- `cloud_topic`、`cloud_reliable`、`map_frame`、`motion_frame`、`task_topic`、`navigate_action`：非空时覆盖 YAML。
- `output_dir`：统一设置地图导出及仿真评价目录，默认仿真 `/tmp/plan_env_gemini336`、实机 `/tmp/plan_env_real`。
- `record_bag`：非空时覆盖 YAML；默认仿真关闭、实机开启，点云和 TF 录制到 `output_dir/bag`。
- `use_sim_time`：默认 `false`；合成场景使用系统时钟，不发布 `/clock`。只有外部提供 `/clock` 时才开启。

安装式构建后修改源码 YAML/launch 需要重新构建/安装。每次实验建议指定新的
`output_dir`，避免重启后同名地图版本文件覆盖。

## 实机准备与任务接口

先启动相机、定位和 Nav2。`plan_env` 不负责启动这些外部系统，也不启动擦拭执行器。
固定坐标系默认 `map`，底盘运动坐标系默认 `base_footprint`，需要在点云时间戳处
存在 `map <- 点云 header.frame_id` 和 `map <- base_footprint` 的 TF。
地图从 Z=0 起，实机配置高度为 1.8 m，按现场修改高度、分辨率及任务范围。

实机默认 `grid_map.cloud_reliable=false`，可接收 best-effort 或 reliable 点云；
仿真默认 reliable，减少大点云丢包。相机发布 reliable 时可以显式开启 reliable 订阅。

任务话题默认 `/area_designation`，类型 `std_msgs/msg/String`，订阅 QoS 为
reliable + transient_local。以下多边形只是格式示例，坐标应替换为现场范围：

```bash
ros2 topic pub --once --qos-durability transient_local \
  /area_designation std_msgs/msg/String \
  "{data: '{\"command\":\"start\",\"frame_id\":\"map\",\"points\":[{\"x\":0.0,\"y\":0.0},{\"x\":1.0,\"y\":0.0},{\"x\":1.0,\"y\":1.0},{\"x\":0.0,\"y\":1.0}]}'}"
```

收到范围立即启动任务，没有单独的“设置范围后再启动”步骤。也支持 `polygon/clean`
和 `run_complex_area_clean` JSON。任务坐标系必须与 `grid_map.frame_id` 一致，
节点不对任务多边形做 TF 转换。新任务会重建任务地图并取消上一任务的导航。

停止任务（会取消正在执行或尚未返回接受结果的导航）：

```bash
ros2 topic pub --once --qos-durability transient_local \
  /area_designation std_msgs/msg/String "{data: '{\"command\":\"stop\"}'}"
```

实机观测过程：当前位置静止融合达到配额 → 规划/发布观测点 → 调用 Nav2 →
Nav2 成功且 TF 位置在到达容差内、站位地图提交完成 → 继续评估。
`/complex_area_plan/observation_target` 继续发布 `PoseStamped` 供查看，实机模式已由
规划节点直接发送 action，不能再另外启动一个订阅该话题并重复下发导航的桥接器。

导航沿用 `cleaning/orchestrator.py` 的异步模式：`NavigateToPose.Goal.pose` 使用
地图坐标系和目标朝向，`behavior_tree` 留空以采用 Nav2 默认配置。服务器等待超时
默认 5 s，目标接受/执行总超时默认 120 s；拒绝、取消、失败或超时会停止观测规划，
发布 `navigation_failed:<原因>`，不自动重试。旧任务的延迟回调不能推进新任务。
导航失败后独立导出节点仍可继续采集；若要一并停止采集，发送上述 stop 任务。

查看状态：

```bash
ros2 topic echo /complex_area_plan/observation_state
ros2 topic echo /grid_map/mapping_status
```

达到可擦条件后发布 `/complex_area_plan/wipe_request`，冻结规划节点地图并等待
`/complex_area_plan/wipe_complete` 的 `Bool(true)`。实机不自动发送此回执。
独立测试中确认本轮结束后，可手动模拟回执：

```bash
ros2 topic pub --once /complex_area_plan/wipe_complete std_msgs/msg/Bool '{data: true}'
```

## 仿真调试

场景、帧率、采样步长等参数在 YAML 中配置。初次验证可将 `publish_rate` 调为
`1.0`、`pixel_stride` 调为 `8`，确认地图提交后再恢复默认 `5 Hz`、步长 `2`。
每站位 `frames_per_observation=10`，`wait_for_fusion=true`，切站等待
`/grid_map/mapping_status` 的提交状态。

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

输出目录由 launch 的 `output_dir` 统一设置，不会自动创建 run_id 子目录。

每个高层版本可导出高度分类 PPM、交互占用图 HTML、体素数据 JSON、float32 距离场和 `versions_3d.csv`。PPM 是高度分类图，不是连续高度值；JSON 的 `occupancy_state` 字段为原始三态占据（字符 0=自由、1=占用、2=未知），`occupancy` 则按 ESDF 距离派生，可能包含作为障碍处理的未知区域。评价节点生成 `metrics.csv`、`metrics.json`：

- `occupied_iou`：预测占用与障碍真值的交并比，未知障碍影响并集。
- `free_space_precision`：预测自由空间的正确比例。
- `heightmap_accuracy`、`heightmap_known_accuracy`、`heightmap_coverage`：高度分类准确率和覆盖率。
- `esdf_mae_m`、`esdf_rmse_m`、`esdf_bias_m`：相对解析障碍距离的误差，含体素离散误差。

实验固定使用 `0.02/0.04 m` 双层分辨率、`0.02 m` 点云叶尺寸、`1.8 m` 地图高度、`0.8 m` 扩张边距、每站位 2 次成功融合，关闭 ICP、开启地面清空射线。高层默认少于 128 个状态变化体素时复用 ESDF；精确距离对照可将该阈值设为 `0`。
