# 旧实现对照与双分辨率性能优化

本页记录第一轮优化（变化时全量重算）的基线；后续默认启用精确局部更新，见 [局部更新说明](local_updates.md)。

2026-10-23，Intel Xeon Gold 6430（32 个逻辑 CPU）、GCC 11.4、ROS 2 Humble、Release / `-O3`。这是同一机器上的合成数据测试，不是机器人实机或相机 rosbag 回放。

旧实现来源：`/home/st/st_ws/src/mobile_manipulator_controller/whole_body_planner/plan_env`。
新实现来源：`/home/st/st_ws/bimax/src/mobile_manipulator_controller/whole_body_planner/plan_env`；“优化前”指本次修改前的工作区副本，包含用户已有改动。

## 对比结论

- 旧版默认地图 3×3×0.5 m、分辨率 0.015 m，分配 136 万体素。旧版占据更新上限 5 Hz、ESDF 按当前射线包围盒局部重算，launch 默认 2 个 OpenMP 线程。
- 新版高层按任务包围盒分配，低层按扩展包围盒分配；低层在每个有效输入帧融合，原来每帧全量计算 ESDF、生成两层快照。高层在站位配额完成时提交，可执行 ICP。
- 新版原来不读取 `grid_map.omp_threads`，也不读取旧版的 `occupancy_update_hz` 和 `cloud_leaf_size`。同一个旧 launch 不代表两版实际负载相同；新版滤波参数名是 `point_cloud_leaf_size`，默认 0.005 m，旧 launch 的 `cloud_leaf_size=0.015` 不会覆盖它。
- 新版默认扩边是每侧 0.25 m，本报告的双图流水线主动配置为每侧 0.8 m，沿用前面讨论的场景；没有修改生产地图尺寸、滤波精度、站位帧数或处理频率。

## 相同体素数的 ESDF 内核

每个体积使用 `std::mt19937(42)` 生成约 5% 障碍；关闭未知障碍策略、计算有符号距离。统一限制 2 线程，预热 3 次，每轮 30 次，3 轮均值的中位数。单位 ms/次。

| 体素数 | 旧版内核墙钟 / CPU | 新版优化前墙钟 / CPU | 新版优化后墙钟 / CPU |
|---|---:|---:|---:|
| 125,000 | 3.850 / 7.683 | 5.053 / 10.120 | 3.845 / 7.677 |
| 83,300 | 2.599 / 5.199 | 3.124 / 6.258 | 2.463 / 4.930 |
| 1,360,000 | 41.095 / 82.189 | 53.640 / 98.135 | 41.768 / 81.591 |

旧版由脚本原样提取 `fillESDF` 和 `updateESDF3d`，使用预先分类的障碍掩码和简化适配层，保留三轴变换、临时体积与正负距离合并。它不包括点云、射线融合、任务多边形分类或发布，因此是算法成本参考，不能当作旧版完整单帧耗时。旧版负距离有一个分辨率的偏移，本次保留新版原有负距离定义。

新版强制重算耗时降低约 21%–24%，接近旧版内核。83,300 体素的低层源不变时，仅检查障碍掩码，耗时由 3.112 ms 降至 0.263 ms（约减少 92%），CPU 时间由 6.245 ms 降至 0.417 ms。

## 新版完整同步处理路径前后对比

调用真实 `processQueuedCloud`：PointCloud2 转换、PCL 降采样、TF 查询、运动门控、射线融合、ESDF、不可变快照。绕过 DDS 输入调度；没有 `bimax_msgs`，不含实际 HeightMap 构造和发布。没有实机相机数据，因此这些结果不代表完整机器人进程 CPU 占用。

输入为 10,000 个点，XY 约 2×2 m，平面高度 0.12 m。任务包围盒 0.5×0.5 m，高/低分辨率 0.01/0.03 m，高度 0.5 m，每侧扩边 0.8 m；使用直接 C++ API 默认占用模型。高层配额 30 帧，关闭 ICP，预热 35 帧（包含首次高层提交），测量 50 帧，3 轮取均值的中位数。由于当前几何实现的 float 舍入与 ceil，实测高层为 51×51×50=130,050、低层为 71×71×17=85,697 体素；优化前后完全一致。

| 场景（统一 2 线程） | 优化前墙钟 | 优化后墙钟 | 耗时减少 | 优化前 CPU 时间 | 优化后 CPU 时间 | CPU 时间减少 |
|---|---:|---:|---:|---:|---:|---:|
| 已提交、稳定场景 | 10.269 ms | 6.677 ms | 35.0% | 17.074 ms | 6.683 ms | 60.9% |
| 底盘运动门控、障碍高度交替变化 | 10.348 ms | 7.414 ms | 28.4% | 17.115 ms | 8.850 ms | 48.3% |

变化场景将中心区域端点高度在 0.12/0.30 m 间切换，同时切换底盘 TF 触发运动门控。它包含真实的低层占用源变化，但并非每帧都有分类变化。两种场景的最终低层占用及 ESDF 数值校验和前后完全一致。高层提交/ICP 的峰值延迟不在预热后的这张表内。

CPU 时间是 `std::clock()` 累计的全进程 CPU 时间（含 OpenMP 工作线程）；墙钟是 `steady_clock`。连续满速测试的 `cpu_percent = 100 × CPU时间 / 墙钟时间`，100% 表示一个核。按相同 30 Hz 有效帧率粗估，稳定场景这些计算的 CPU 预算由约 51% 单核降至约 20%；实际还受空闲等待、DDS、其他节点和真实点数影响。

### 不设置 OMP_NUM_THREADS 的检查

另测 20 帧，优化前 OpenMP 默认上限为 32，优化后库默认上限为 2：

| 场景 | 优化前墙钟 / CPU ms | 优化后墙钟 / CPU ms | 连续测量 CPU% 前 → 后 |
|---|---:|---:|---:|
| 稳定场景 | 11.219 / 207.334 | 7.037 / 7.047 | 1848% → 100% |
| 变化、移动场景 | 21.302 / 515.432 | 7.324 / 8.008 | 2420% → 109% |

这体现了该 32 核环境下过多线程和 OpenMP 等待的成本，不应套用到不同硬件。减少线程可能增加单次强制 ESDF 重算的墙钟时间：例如 83,300 体素由默认 32 线程的 1.782 ms 变成 2 线程的 2.542 ms，但 CPU 时间由 55.976 ms 降为 5.042 ms；完整流水线仍因消除重复计算而变快。

## 修改内容和正确性边界

1. ESDF 从连续 Z 方向开始变换，再处理 Y、X，参考旧版遍历顺序；正负变换复用同一个 float 临时体积，去除重复行清零和重复占用分类，均匀场直接填充。
2. 新增每体积 `EsdfConfig::omp_threads`，默认 2；ROS 高低层支持 `grid_map.omp_threads` 和 `grid_map.low_omp_threads`，低层默认继承高层。每工作线程至少分配约 32,768 体素，且尊重更小的 OpenMP 运行时上限，不修改全局线程设置。
3. 低层 `computeIfChanged` 比较完整二值障碍源；已知占用、未知空间策略与高度带均参与判断。源变动任意一个体素就重算，无阈值延迟。重配置、清空及可写距离数组访问使缓存失效；原 `compute` 仍强制重算。
4. 高层未提交时，快照复制前一快照中的高层概率/状态，避免重复 exp 和分类；保留对外 vector 字段、不可变快照和版本语义。低层占用概率仍每帧融合发布。

缓存复用会保留每体素 4 字节临时距离和 1 字节障碍掩码，换取减少分配；结束任务时释放这些新增缓存。正负距离的临时峰值从两份 float 体积变成一份。快照目前仍复制两层数组；ICP 和高层射线重放仍占用同一个映射线程，这些开销没有被隐去。

构建算法库、建图库、两个节点与基准程序；30 项测试通过。覆盖随机场暴力参考、串并行一致、均匀场、源不变/变化、未知高度带、占用阈值变化、缓存失效、线程参数、高低层快照和原有站位融合回归。环境缺少 `bimax_msgs`，未验证实际 HeightMap 消息发布分支；没有进行实机精度或实时性验收。

## 复现

```bash
source /opt/ros/humble/setup.bash
cmake -S /path/to/plan_env -B /tmp/plan_env_build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DPLAN_ENV_BUILD_BENCHMARKS=ON
cmake --build /tmp/plan_env_build -j4
ROS_LOG_DIR=/tmp/plan_env_ros_logs ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=2 \
  ctest --test-dir /tmp/plan_env_build --output-on-failure
OMP_NUM_THREADS=2 /tmp/plan_env_build/mapping_benchmark esdf 30
ROS_LOG_DIR=/tmp/plan_env_ros_logs ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=2 \
  /tmp/plan_env_build/mapping_benchmark pipeline 50
python3 /path/to/plan_env/scripts/benchmark_legacy_esdf.py \
  /path/to/old/plan_env/src/grid_map.cpp /tmp/legacy_esdf_benchmark
```

本机 MPI 头文件发现需要额外传入 `-DMPI_C_HEADER_DIR=/usr/lib/x86_64-linux-gnu/openmpi/include`；这是本机构建环境配置，不修改项目依赖。测试环境禁止网络接口枚举，DDS 会输出传输初始化警告；上述回归通过直接注入 TF/点云执行，不依赖 DDS 通信。

本次汇总原始数据见 [performance_results.csv](performance_results.csv)。隔离构建和优化前副本保留于 `/tmp/plan_env_perf`；比较修改前版本时，基准中关闭 `PLAN_ENV_BENCHMARK_SOURCE_CACHE` 宏，使不支持缓存接口的版本走原 `compute()`。
