> 历史设计记录：不作为当前代码、参数或运行方式的依据。当前入口见 [README](../../README.md)。

# plan_env 调用迁移与任务生命周期

本次调用端使用 `plan_env::GridMap`，不再依赖旧头文件的全局 `GridMap` 或 `using namespace std`。原 `grid_map.h` 包含路径仍可使用。

## 初始化与任务入口

- `getResolution()` 在构造后即返回默认 `0.05 m`；`initMap(node)` 读取 `grid_map.resolution` 后更新。stop 保留配置分辨率。
- planner 服务调用 `startTaskMapping(polygon, &message)` / `stopTaskMapping()`。任务执行中拒绝 start/stop，以免更换正在使用的地图。
- 独立 `whole_body_qp_node` 在 ESDF 启用后收到第一帧 odom 时自动建图；不依赖 planner 进程的地图。
- 可设置 `whole_body_qp.task_polygon_xy: [x0, y0, x1, y1, ...]`，坐标系为 `grid_map.frame_id`。
- 未配置 polygon 时，QP 使用首帧 odom 在地图坐标系中的位置作为中心，按 `grid_map.map_size_x/y`（默认各 3 m）生成固定任务矩形，再由 GridMap 外扩。odom 坐标系不同时等待对应时间戳的 TF。地图在此 QP 会话中不滚动。

## 驻留与快照

相邻位姿变化或相对驻留锚点的累计变化超过阈值后，丢弃旧锚点和当前驻留计数。到新位置稳定后建立新锚点，重新累计成功融合帧。跨站位的首帧必须通过配准；失败、ROI 无有效点或未实际积分都不会消费融合配额，也不会让下一帧跳过配准。已接受的配准修正用于该站位后续帧。

只有当前站位达到 `grid_map.stationary_frames` 配额、完成 ESDF 计算并发布快照后，`isMappingReadyForPlanning()` / `odomValid()` 才返回 true。移动后返回 false，上一站位的同任务快照仍可供距离查询，但不能作为新站位已就绪的依据。

`captureEsdfFreezing()` 只捕获已经就绪的快照；`freezeOccupancyAndEsdf()` 还切换冻结查询并停止接收和融合新点云。`clearEsdfFreezing()` 清除冻结覆盖并恢复点云接收，后续移动会启动新的驻留采集。

任务启动、停止与整帧融合/发布使用同一状态锁。点云回调只持有短时队列锁，避免等待配准和 ESDF 计算。每次 start/stop 增加任务代次并丢弃排队点云、实时快照、冻结快照；工作线程已取出的旧任务点云也会被代次检查拒绝。空快照不能通过就绪或冻结检查。

## 参数行为

| 参数 | 行为 |
| --- | --- |
| `grid_map.esdf_unknown_mode=off` | unknown 不作为 ESDF 障碍源 |
| `grid_map.esdf_unknown_mode=task_roi` | 扩展矩形范围内，仅 `ground_height` 到 `esdf_roi_z_max` 的体素层将 unknown 作为障碍源 |
| `grid_map.esdf_unknown_mode=all` | 全地图高度的 unknown 作为障碍源 |
| `grid_map.esdf_roi_z_max` | 默认 0.50 m；沿用旧版以 floor 索引选取高度层且包含上界所在层的语义 |
| `grid_map.cloud_min_map_z` | 默认 0.02 m；低于此地图高度的点不产生 HIT |
| `grid_map.cloud_low_z_clear_rays=false` | 丢弃低点 |
| `grid_map.cloud_low_z_clear_rays=true` | 低点保留为全程 MISS 射线，不写入 TSDF 表面 |
| `grid_map.cloud_min_range` / `local_update_range_x/y/z` | 相机系近距及轴向范围过滤 |

高度限制只作用于 unknown；已知 occupied 在高度带外仍作为 ESDF 障碍源。本次未实现占据膨胀。观测规划器采用扩展 ROI 的 Python 接线仍是单独的后续工作，擦拭分区继续使用原始 polygon。

## 验证范围

Humble / GCC 下在原项目位置构建新版 plan_env，并运行 17 个 GTest 用例，覆盖连续平移/转动站位、累计漂移、失败配准重试、配准修正保持、地面过滤、unknown 高度策略、冻结与任务切换、异步队列并发。

当前环境缺少 bimax_msgs，尚不能验证 HeightMap 消息分支和完整 planner/controller 联编。package.xml 已声明 bimax_msgs 依赖，确保完整工作区按消息包依赖顺序构建。实际替换时应只保留一个可发现的 plan_env 包，并重编下游，不能混用旧版二进制。

## 2026-10-09 实际替换与日志适配

- 正式维护位置：`whole_body_planner/plan_env`。旧模块文件已移除；`grid_map.h` 保留为包含 `grid_map.hpp` 的兼容入口。
- `new_Grid-Map` 保留迁移前开发副本，根目录 `COLCON_IGNORE` 防止 colcon 发现第二个同名包。后续修改以原项目位置为准。
- 替换前含未提交修改的旧目录备份：`/home/st/plan_env_backups/20261009_113402/original_plan_env.tar.gz`。
- `test` 记录器默认格式 `plan_env_occupancy_v2`，按时间戳配对冻结二维／三维点云，保存分辨率、地图范围、投影与强度语义。三维空快照也正常发布，避免保留旧障碍。
- 新日志覆盖率使用最新完整的全高度占据投影，擦拭 ROI 保持初始任务多边形；地图扩展边界另外显示。缺少切片不推断 free。旧日志按旧口径读取。用法见 `test/README.md`。
- 已通过：17 项 C++ 测试、新旧日志 Python 回归、新旧二维 PNG 和交互式三维 HTML 输出、汇总报表生成、唯一包发现检查。记录器使用真实 ROS PointCloud2 的回调检查覆盖乱序、空云、段落清理和任务切换；该检查只替代了当前缺失且未使用的 ReferenceTrajectory 消息导入，未进行在线 DDS 全链路验证。
- 下游语法检查：mm_config、kino_astar、rrt、sample_mani_RRT 通过（另一窗口完成 ArmSide 调用迁移后复查）。当前未安装 bimax_msgs，完整 planner/controller 联编及 HeightMap 发布分支仍需在完整 ROS 工作区验证。

完成公共头文件迁移、准备消息依赖后，从工作区根目录重编下游，勿复用旧 plan_env 二进制：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to whole_body_controller whole_body_planner \
  --cmake-args -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++
```

如原 build 中缓存了另一编译器，应在新 build/install 目录编译或清理对应包的 CMake 缓存后重新构建。
