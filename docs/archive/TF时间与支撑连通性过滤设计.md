> 历史设计记录：不作为当前代码、参数或运行方式的依据。当前入口见 [README](../../README.md)。

# TF 时间门控与支撑连通性过滤设计

本文针对 `plan_env` 当前的点云建图流程，设计两项前置质量控制：

1. 在查询 TF 和融合点云前检查点云时间戳，拒绝过旧、未来和不属于当前静止阶段的点云。
2. 在射线占据更新前，对当前帧的 hit 候选体素做地面支撑连通性启发式过滤，降低孤立离群点造成的误占据。

本文是实现设计，不改变现有 log-odds、射线遍历、TSDF 和 ESDF 的定义。

## 1. 现有流程与设计约束

当前点云处理顺序为：

```text
PointCloud2
  -> 有限值检查
  -> PCL VoxelGrid 降采样
  -> 按 cloud.header.stamp 查询 map <- cloud_frame TF
  -> 传感器范围过滤
  -> 变换到 map_frame
  -> ROI/高度过滤
  -> 运动门控
  -> 射线占据更新
```

后台 mailbox 只保存一帧待处理点云，新消息可能覆盖尚未处理的旧消息。因此，点云进入回调时满足时间要求，并不代表工作线程真正处理时仍然满足时间要求。

本设计采用以下前置条件：

- `cloud.header.stamp` 是点云采集时间，而不是消息到达时间。
- 点云时间戳保证来自机器人已经静止之后；因此不需要用一帧采集于运动阶段的点云参与当前停靠点建图。
- 点云、TF 和 `node->get_clock()` 使用同一 ROS 时间域。仿真或 bag 回放必须正确启用 `/use_sim_time`。
- 任务地图的 Z 方向和地面模型已知，或者可以从点云/外部地图估计得到。

“点云来自静止之后”只解决运动畸变和停靠点归属问题，不能解决消息排队造成的过旧、传感器时钟超前造成的未来时间，或者 TF 缓存缺失。因此仍需要显式时间门控。

## 2. TF 时间门控

### 2.1 时间定义

处理一帧云时记录以下时间：

| 名称 | 来源 | 含义 |
| --- | --- | --- |
| `cloud_stamp` | `cloud.header.stamp` | 传感器采集时间 |
| `now` | `node_->get_clock()->now()` | 当前 ROS 时间 |
| `stationary_since` | 运动状态机 | 当前停靠点允许采集的最早时间 |
| `tf_stamp` | 返回的 `TransformStamped` | TF 实际变换时间；实现提供时检查 |

时间差定义为：

```text
age         = now - cloud_stamp
future_skew = cloud_stamp - now
```

正常情况下两者不会同时为正；实现上可直接使用带符号的 `age` 判断。

### 2.2 接纳规则

建议增加以下 ROS 参数：

| 参数 | 初始建议值 | 作用 |
| --- | ---: | --- |
| `grid_map.max_cloud_age` | `0.20 s` | 允许的最大点云年龄 |
| `grid_map.max_cloud_future_skew` | `0.03 s` | 允许的最大未来时间偏差 |
| `grid_map.stationary_stamp_tolerance` | `0.02 s` | 静止阶段起点的时间容差 |
| `grid_map.max_tf_time_error` | `0.02 s` | 返回 TF 时间与请求时间的最大差值；无法取得 TF 时间时只依赖 tf2 查询结果 |
| `grid_map.tf_lookup_timeout` | `0.05 s` | TF 查询等待时间，沿用当前默认值 |

点云在 TF 查询前必须满足：

```text
cloud_stamp != 0
age <= max_cloud_age
age >= -max_cloud_future_skew
cloud_stamp + stationary_stamp_tolerance >= stationary_since
```

其中 `age < 0` 表示点云时间在当前 ROS 时间之后。超过未来窗口的点云必须拒绝，不能把它当成“最新数据”处理。零时间戳也应拒绝，不能隐式转换为 `tf2::TimePointZero` 的最新 TF 查询。

### 2.3 `stationary_since` 的定义

运动门控已有相邻位姿和驻留锚点判断，但当前结构没有保存时间信息。设计上应给 `PreparedCloud` 增加 `cloud_stamp`，并在运动状态机中保存当前停靠点的时间边界：

1. 位姿变化超过阈值时进入 `MOVING`，清空当前 local map。
2. 连续位姿重新满足静止条件时，使用第一帧满足条件的 `cloud_stamp` 建立 `stationary_since` 候选。
3. 只有 `cloud_stamp >= stationary_since - stationary_stamp_tolerance` 的帧可以进入当前停靠点的 local map。
4. 由于系统保证点云来自静止之后，正常输入不会被该规则拒绝；规则主要用于检测驱动时间错误、旧 mailbox 数据和任务切换边界错误。

`stationary_since` 应使用传感器时间戳，而不是工作线程接收时间。若没有可靠的静止事件时间，可将第一帧通过运动门控的云时间作为当前停靠点起点，并记录该选择的日志。

### 2.4 处理时机

时间检查需要执行两次：

1. **TF 查询前**：防止为明显过旧或未来的点云查询 TF。
2. **工作线程即将融合前**：防止点云在 mailbox 或 TF 等待期间变旧。

第二次检查失败时，必须在任何 occupancy、TSDF、局部观测和驻留帧计数更新前返回。`recordSuccessfulFusion()` 只能在所有时间和几何检查成功后调用。

推荐的处理顺序为：

```text
读取点云
  -> 校验 stamp/ROS 时间域
  -> 检查 age/future/stationary_since
  -> lookupTransform(..., cloud_stamp)
  -> 检查返回 TF 的时间误差（若可用）
  -> 点云空间过滤与变换
  -> 工作线程再次检查 age/future
  -> 支撑连通性过滤
  -> 射线融合
```

### 2.5 TF 查询策略

查询必须继续使用点云采集时间：

```text
lookupTransform(map_frame, source_frame, cloud_stamp, timeout)
```

不建议在查询失败时无条件退回 `TimePointZero` 或最新 TF。最新 TF 与旧点云组合会造成空间错位，而且错误通常不会立刻触发异常。若需要等待 TF 到达，可以在 `tf_lookup_timeout` 内重试或使用 `tf2_ros::MessageFilter`，但等待仍受 `max_cloud_age` 限制。

如果返回的 `TransformStamped` 包含有效时间戳，计算：

```text
tf_time_error = abs(tf_stamp - cloud_stamp)
```

超过 `max_tf_time_error` 时拒绝该帧并记录告警。对于 tf2 正常插值，返回时间通常应接近请求时间；该检查主要用于防止未来引入“取最近 TF”回退逻辑后静默接受错误时间。

### 2.6 时间异常与日志

以下情况应分别统计，而不是都记为“TF 查询失败”：

- `invalid_stamp`：零时间戳或非法时间。
- `stale_cloud`：超过 `max_cloud_age`。
- `future_cloud`：超过 `max_cloud_future_skew`。
- `before_stationary`：早于当前停靠点时间边界。
- `tf_unavailable`：请求时间不在 TF buffer 或 TF 发布延迟。
- `tf_time_mismatch`：返回 TF 时间偏差过大。
- `clock_jump`：ROS 时间发生不可接受的前后跳变。

日志应至少包含 `cloud_stamp`、`now`、年龄、源坐标系和当前运动状态。建议使用节流日志，并增加计数器用于运行时诊断。

当 ROS 时间向后跳变、仿真时间重置或 bag 切换时，应清空待处理云、重置 `stationary_since` 和上一帧时间基线，等待新的有效时间和位姿建立后再融合。时间向前大幅跳变时，也应暂时拒绝旧缓存中的点云。

## 3. 支撑连通性启发式过滤

### 3.1 目标和适用范围

该过滤器用于去除当前帧中空间上孤立的小块点云，例如深度传感器偶发的离群点或不连续反射点。它不是严格的物理“是否落地”判断，也不能解决整体 TF 偏移、动态物体或系统性传感器误差。

过滤对象应是当前帧中可能产生 `hit` 的 endpoint 体素，不应直接在全局 occupancy map 上做连通性判定。全局地图含有历史观测，容易把旧障碍和定位误差带入当前帧的离群点判断。

### 3.2 输入和输出

输入为已经完成以下步骤的数据：

- 有限值检查；
- VoxelGrid 降采样；
- TF 变换到 `map_frame`；
- 传感器范围、任务 ROI 和高度过滤。

过滤器输出当前点的保留标志，供后续生成 `RayObservation` 使用：

```text
保留点 -> 正常执行 miss + endpoint hit
删除点 -> 不生成该点对应的射线
```

被删除的点不能继续生成 free-space 射线，否则虽然不再写入错误的 occupied endpoint，仍可能沿错误射线清除真实障碍前的体素。

### 3.3 地面支撑图

将每个点转换为地图体素 ID，并对相同体素去重。只使用 endpoint 候选体素构建稀疏集合：

```text
candidate_voxels = { voxel_id(point) | point 是有效 hit 候选 }
```

地面种子由地面模型生成。若地图地面为水平面 `z = ground_z`，可以使用：

```text
abs(voxel_center.z - ground_z) <= ground_band
```

若地面不水平，应使用点到估计地面平面的距离。`cloud_min_map_z` 是点云高度裁剪参数，不能直接当作 `ground_z`。

推荐增加以下参数：

| 参数 | 初始建议 | 作用 |
| --- | ---: | --- |
| `grid_map.support_filter.enabled` | `false` | 是否启用过滤，先灰度验证 |
| `grid_map.support_filter.ground_z` | `0.0 m` | 水平地面高度 |
| `grid_map.support_filter.ground_band` | `1~2` 个地图体素 | 地面种子厚度 |
| `grid_map.support_filter.max_gap_voxels` | `1` | 支撑链允许的最大断裂 |
| `grid_map.support_filter.min_component_voxels` | `3~10` | 小组件过滤门槛 |
| `grid_map.support_filter.mode` | `supported_only` | 是否保留大悬空组件 |
| `grid_map.support_filter.temporal_votes` | `2/3` | 可选的跨帧确认窗口 |

### 3.4 连通性算法

基础版本采用地面种子到候选体素的 BFS/DFS：

```text
seeds = candidate_voxels 中靠近地面的体素
supported = 空集合
queue = seeds

while queue 非空:
    v = queue.pop()
    if v 已访问:
        continue
    标记 v 为 supported
    对 v 的六个面邻居 n:
        if n 在 candidate_voxels 中:
            queue.push(n)
        else if n 与候选体素的最近距离不超过 max_gap_voxels:
            允许一次有限间隙连接
```

对每个候选体素记录其所属组件的大小。组件不满足以下任一条件时删除：

- 能通过六邻接或允许的有限间隙到达地面；
- `mode=retain_large_floating` 且组件大小超过最小门槛；
- 通过跨帧一致性确认。

实现上不建议直接使用无限制的 26 邻接。26 邻接对稀疏点云更宽容，但容易把仅在对角线接触的噪声连成大组件。优先采用六邻接加一个明确的间隙上限，便于通过地图分辨率控制行为。

### 3.5 时间融合和动态对象

单帧支撑连通性对稀疏表面会比较敏感。建议先实现单帧过滤，再按需要增加有限的时间确认：

```text
同一 component 的 endpoint 体素在最近 3 个静止帧中至少出现 2 次
```

时间确认只能在同一停靠点的 local map 内进行，进入 `MOVING` 或切换位置时清空。它可以减少偶发噪声，但不能当作动态物体检测；行人等物体同样可能在连续帧中保持地面连通。

### 3.6 低 Z 清空射线

当前 `cloud_low_z_clear_rays` 允许低于 `cloud_min_map_z` 的点产生清空射线，但不产生 hit。第一阶段建议只对正常 occupied endpoint 应用支撑连通性过滤，低 Z 清空射线保持现有策略，以免改变地面清理语义。

如果现场确认低 Z 离群点会造成明显误清空，再增加独立参数控制是否过滤清空射线。该选项应默认关闭，并在日志中区分“hit endpoint 被过滤”和“clear ray 被过滤”。

## 4. 误差和失败模式

| 场景 | 预期行为 | 处理建议 |
| --- | --- | --- |
| 单个孤立噪声体素 | 删除 | 六邻接过滤有效 |
| 成片但不接地的反射噪声 | 可能保留 | 增加组件大小、跨帧确认或传感器质量检查 |
| 稀疏墙面/桌面 | 可能被断开 | 使用 `max_gap_voxels=1` 或小尺度膨胀 |
| 悬挂真实障碍 | 可能被删除 | 使用 `retain_large_floating` 或关闭 supported-only |
| 地面点云缺失 | 整帧候选可能无种子 | 使用已知地面模型，不依赖当前帧一定看到地面 |
| TF 整体偏移 | 连通性可能仍然成立 | 由时间门控、TF 质量和 ICP 处理，不能依赖本过滤器 |
| 运动过程中点云 | 不应进入当前 local map | 由静止时间戳不变量和运动门控拒绝 |
| 动态行人 | 可能保留 | 需要独立动态障碍层或时间衰减 |

支撑连通性删除的是观测，不是占据状态。当前帧删除不会自动清除历史 global map 中已经存在的 occupied 体素；如果需要动态障碍消失，还必须增加时间衰减、反向观测或独立动态层。

## 5. 建议的实现边界

建议把新增逻辑拆成两个独立模块，避免继续向 `GridMap` 添加算法细节：

- `timestamp_gate`：纯时间和静止阶段判断，输入点云时间、当前 ROS 时间和状态边界，输出拒绝原因。
- `support_filter`：纯体素候选连通性判断，输入点、地图几何和地面模型，输出保留掩码。

`GridMap::processPointCloud()` 负责时间门控和点云预处理，`GridMap::updateMapsFromRays()` 之前调用支撑过滤器。占据模型、ray casting、TSDF 和 ESDF 不需要知道过滤器内部规则。

第一版不建议直接重写全局地图或改变 log-odds 参数。先以 `enabled=false` 默认加入参数和统计，再通过 bag 回放比较过滤前后的：

- 每帧输入点数、保留点数和删除点数；
- 支撑组件数量及最大组件大小；
- 各类时间拒绝计数；
- TF 查询失败和等待时间；
- 最终 occupied 体素数量与 ESDF 变化。

## 6. 验证计划

### 6.1 时间门控单元测试

至少覆盖：

1. `cloud_stamp == 0` 被拒绝。
2. 年龄刚好等于 `max_cloud_age` 时按约定接受，超过一个时间精度单位时拒绝。
3. 未来时间在 `max_cloud_future_skew` 内接受，超过时拒绝。
4. `cloud_stamp` 早于 `stationary_since` 且超出容差时拒绝。
5. 点云在 mailbox 中等待后变旧，在第二次检查被拒绝。
6. TF 查询失败与时间门控拒绝产生不同统计和日志。
7. ROS 时间回退后清空时间基线，不融合旧缓存点云。

### 6.2 支撑过滤单元测试

至少覆盖：

1. 与地面相连的直立组件全部保留。
2. 单个孤立体素被删除。
3. 允许一个体素间隙时组件保留，超过间隙时断开。
4. 小型悬空组件在 `supported_only` 模式被删除。
5. `retain_large_floating` 模式保留足够大的悬空组件。
6. 候选集合为空或没有地面种子时行为明确且不会访问越界。
7. 过滤点不会生成 miss 或 hit 射线。
8. 低 Z 清空射线在默认配置下保持现有行为。

### 6.3 Bag 回放

使用包含以下情况的 bag：

- 点云延迟逐步增加；
- 点云时间戳超前；
- TF 发布延迟或 buffer 断档；
- 静止前和静止后的点云混合；
- 地面、墙面、桌面和悬挂物同时出现；
- 孤立深度噪声和成片误测。

验收标准是：过旧/未来点云不会产生任何占据、TSDF、local observation 或驻留计数更新；启用支撑过滤后孤立噪声减少，真实地面相连障碍的漏检率在可接受范围内；关闭过滤器后输出与现有实现一致。

