# zhao_lio 交接文档

更新日期：2026-10-29

## 1. 这是什么

zhao_lio 是一个用来学习和做实验的 LiDAR-惯性里程计，组合了两套已有实现的做法：

- **前端和地图**取自 Super-LIO：降采样时每个体素保留离中心最近的原始点（`VoxelGridClosest`），地图用紧凑的八叉体素 `OctVoxMap`。
- **滤波**取自 Point-LIO / small_point_lio：30 维 ESKF，角速度和加速度作为状态，IMU 作为观测，每个 LiDAR 点做一次 rank-1 更新。

更新方式做成可切换的策略，同一个可执行文件就能在同一份数据上做 A/B 对比：

| 策略 | 做法 |
|---|---|
| `pointwise` | 每个点串行执行：预测 → KNN → 平面拟合 → 更新。基线，对应 small_point_lio |
| `pointwise_prematch` | 先用 IMU 预测位姿，TBB 并行算出整帧点的平面；再串行逐点更新，只重算残差；残差 > `rematch_thr` 时重新搜索 |

设计文档：`docs/specs/2026-10-29-zhao-lio-design.md`
实现计划：`docs/plans/2026-10-29-zhao-lio.md`

## 2. 当前状态

- 10 个计划任务全部完成，两个包都能编译。
- 单元测试 40 个全部通过，其中包括合成场景（盒子房间，8 s 平滑轨迹）的端到端测试。
- 在真实数据 `tracked-01`（Livox + VectorNav，620 s）上两种策略都跑完了，没有发散。这份数据**没有真值，外参也未知**，所以目前只能做合理性检查，见第 7 节。
- 代码在 GitHub 仓库 `Zanniedy/zhao_lio` 里维护（本地 `~/cs_project/liop/zhao_lio`）。2026-10-30 从 `liop/first_lio` 迁移过来，包名、命名空间、可执行文件和话题都从 first_lio 改成了 zhao_lio。
- 测试平台 liobench 在仓库的 `liobench/` 目录下，用法见 `liobench/docs/TUTORIAL.md`。

## 3. 环境和构建

依赖：Ubuntu 22.04、ROS2 Humble、Eigen 3.4、oneTBB 2021.5。可选依赖 `livox_ros_driver2`，本机编译在 `~/cs_project/nav/cs/install`，不在系统 ROS 环境里。

```bash
cd ~/cs_project/liop/zhao_lio
source /opt/ros/humble/setup.bash
source ~/cs_project/nav/cs/install/setup.bash   # 不 source 也能编译，但只支持 PointCloud2 输入
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select zhao_lio_core && colcon test-result --verbose
```

单独跑某个测试：`./build/zhao_lio_core/test_pipeline`。降采样基准测试：`./build/zhao_lio_core/bench_downsample 20000 0.5`。

**环境坑：** miniconda 在 CMake 的搜索路径上，曾导致回放工具链接到 conda 的 `libyaml-cpp.so.0.9`，运行时报找不到库。`zhao_lio_ros2/CMakeLists.txt` 开头已固定使用系统的 yaml-cpp。以后其他 ROS 包遇到类似的 `.so` 找不到，先查一下是不是链接了 conda 的库。

## 4. 运行

每个终端先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/cs_project/nav/cs/install/setup.bash
source ~/cs_project/liop/zhao_lio/install/setup.bash
```

**离线回放**（推荐做实验用）：直接读包，不经过 ROS 通信，计时不受播放速率影响。结束时打印每个阶段的平均耗时。

```bash
ros2 run zhao_lio_ros2 zhao_lio_replay --ros-args \
  --params-file ~/cs_project/liop/zhao_lio/src/zhao_lio_ros2/config/tracked.yaml \
  -p bag:=$HOME/datasets/tracked-01/tracked-01_ros2 \
  -p update_strategy:=pointwise_prematch \
  -p traj_path:=pm.tum -p timing_csv:=pm.csv
```

**实时节点 + RViz2**：先启动节点，再播包。节点要用开头的静止段初始化。

```bash
ros2 launch zhao_lio_ros2 zhao_lio.launch.py config:=tracked use_sim_time:=true   # 终端 1
ros2 bag play ~/datasets/tracked-01/tracked-01_ros2 --clock --topics /livox/lidar /vectornav/IMU   # 终端 2
```

launch 参数：`config`（mid360 | avia | tracked）、`update_strategy`、`rviz`（默认 true）、`use_sim_time`（默认 false）。其他参数要改 yaml 再重新 `colcon build`；launch 文件目前只支持覆盖 `update_strategy`。

输出话题：`/zhao_lio/odom`、`/zhao_lio/path`、`/zhao_lio/cloud_world`（降采样后的世界系点云），以及 TF `odom → body`。

**轨迹对比**：`scripts/compare.sh ref.tum est.tum ...`，需要先 `pip install evo`。没有真值时 ref 可以用另一条估计轨迹，此时 APE 衡量的是两条轨迹的一致性，不是误差。

## 5. 代码结构

```
zhao_lio/
  docs/specs/  docs/plans/  docs/HANDOFF.md
  scripts/compare.sh
  src/zhao_lio_core/           纯 C++17 库，只依赖 Eigen、TBB，不依赖 ROS 和 PCL，没有全局变量
    include/zhao_lio/
      types.h params.h          数据类型；所有参数和默认值
      sync.h                    LiDAR 帧与 IMU 配对（MeasureSync）
      preprocess.h              距离过滤、按点采样、按时间排序
      voxel_downsample.h        最近点降采样，串行版和 TBB 并行版
      oct_vox_map.hpp           OctVoxMap 移植；hknn_list60.h 是原样复制的搜索顺序表
      plane.h                   5 近邻 QR 平面拟合
      eskf30.h                  30 维 ESKF
      events.h                  点与 IMU 按时间合并成事件序列
      updater/                  UpdateStrategy 接口和两种策略
      zhao_lio.h               初始化状态机 + 每帧主流程
      io.h                      TUM 轨迹、计时 CSV 输出
    third_party/tsl/            从 Super-LIO 复制的 robin_map / robin_hood
    test/  bench/
  src/zhao_lio_ros2/
    include/zhao_lio_ros2/config.h          ROS 参数 → Params
    include/zhao_lio_ros2/msg_conversions.h Livox CustomMsg / PointCloud2 / Imu 转换
    src/zhao_lio_node.cpp      实时节点
    src/replay.cpp              离线回放
    config/ launch/ rviz/
```

**每帧流程**（`ZhaoLio::process`）：

```
预处理 → 降采样（点保持在 LiDAR 系，时间戳不变）→ 按时间排序
→ 和 IMU 合并成事件序列 → 策略逐事件处理：
     IMU 事件：predict_state + predict_cov + update_imu
     点事件：  predict_state → 取平面 → 门限检查 → update_point
→ 预测到帧末 → 整帧世界系点批量插入地图 → 输出
```

地图在帧末才插入，所以同一帧的点不会互相匹配，预关联阶段地图是只读的，可以安全并行。

**初始化状态机：** `WaitImuInit` 累计 100 个 IMU，**假设静止**：用平均加速度对齐重力，世界系 z 轴朝上，平均角速度作为陀螺零偏。然后 `WaitMapInit` 插入 3 帧点云建立地图，之后进入 `Process`。状态出现 NaN 时清空地图，回到 `WaitImuInit`。

**加新策略：** 继承 `UpdateStrategy`，实现 `process()`，在 `params.h` 的 `StrategyType` / `parse_strategy` 和 `updater/common.cpp` 的 `make_strategy` 里注册即可，主流程不用改。IMU 事件统一用 `imu_step()` 处理。

## 6. 和参考实现不同的地方（有意为之）

| 位置 | 内容 |
|---|---|
| `eskf30.cpp` `update_imu` | small_point_lio 处理加速度通道时读的是陀螺的饱和标志 `satu[i]`，这里改为 `satu[i+3]`。单元测试覆盖 |
| `eskf30.cpp` `predict_cov` | 速度对姿态的 F 块乘了 `dt`。small_point_lio `eskf.h:116` 漏乘，Point-LIO 的 IKFoM 是乘的 |
| `eskf30.cpp` `reset_covariance` | 外参不估计时，把 P 中外参的行和列置零，保证任何更新都不会改动外参 |
| `oct_vox_map.hpp` | Super-LIO 的提前停止表 `orders_min_dis2` 写死对应 0.5 m 分辨率，这里按分辨率缩放。在 0.5 m 和 0.2 m 下都和暴力搜索结果一致 |
| `msg_conversions.h` `scan_from_livox` | 点时间 = `header.stamp + offset_time`，和 Point-LIO 一致。`tracked-01` 里的 `timebase` 是设备上电后计时（约 3408 s），和 IMU 的 ROS 时间对不上 |
| 降采样 | 不用 PCL 的 `VoxelGrid`，因为它求质心会把点的时间平均掉，逐点更新需要每个点有准确时间 |

**实现与计划文档的差异：**
- 合成场景 `test/sim_scene.h` 里仰角序列的系数从 0.618034 改成了 0.7548777。原来的值和黄金角方位序列相关，点全落在一条螺旋线上，降采样后只剩 130 个点。
- `eskf30.h/.cpp` 补了 `#include <Eigen/Geometry>`。
- `package.xml` 里的邮箱改成了 `lsy@example.com`，因为 `lsy@localhost` 通不过 colcon 校验。
- `replay.cpp` 打不开数据包时会打印错误并以退出码 1 结束。
- 新增了 `config/tracked.yaml`、`rviz/zhao_lio.rviz`，launch 文件新增 `rviz` 和 `use_sim_time` 参数。

## 7. 实验结果（tracked-01，没有真值）

数据：`~/datasets/tracked-01/tracked-01_ros2`，620 s，6206 帧。Livox CustomMsg 话题 `/livox/lidar`，每帧约 1 万点；VectorNav IMU 话题 `/vectornav/IMU`，约 200 Hz，单位 m/s²。运行结果在 `~/datasets/tracked-01/runs/`。

| 降采样 | 策略 | 降采样后/用于更新的点数 | 每帧耗时 | 更新阶段 | 轨迹长度 | 起点到终点 | 重新搜索比例 |
|---|---|---|---|---|---|---|---|
| 0.5 m | pointwise | 567 / 437 | 0.83 ms | 0.70 ms | 296.5 m | 0.39 m | — |
| 0.5 m | prematch | 567 / 431 | 0.75 ms | 0.36 ms（+预关联 0.25 ms） | 292.9 m | 0.37 m | 7.8% |
| 0.2 m | pointwise | 1366 / 1068 | 1.53 ms | 1.35 ms | 291.9 m | 0.30 m | — |
| 0.2 m | prematch | 1366 / 1045 | 1.13 ms | 0.58 ms（+预关联 0.36 ms） | 286.0 m | 0.34 m | 5.7% |

- 每帧耗时都在 2 ms 以内，雷达是 10 Hz。整段回放约 9 s，内存峰值约 50 MB。
- 点越多，预关联的优势越明显：0.2 m 时总耗时少 27%。
- 两种策略的轨迹差：0.5 m 时平均 0.09 m；0.2 m 时平均 0.9 m、最大 2.3 m。没有真值，判断不了哪条更准。
- 真实数据上的重新搜索比例（6–8%）比合成数据（< 1%）高很多，说明 IMU 预测在这台履带车上偏差较大。
- 降采样不是瓶颈：2 万点时串行 0.37 ms、并行 0.33 ms，所以默认串行（`downsample_parallel: false`）。

## 8. 已知问题和风险

1. **`tracked.yaml` 的外参是单位阵，这是猜的。** VectorNav 和 Livox 是两个独立设备，实际外参肯定不是单位阵，旋转外参的影响最大。这是当前结果里最大的不确定来源。可以先试 `extrinsic_est_en: true`，或者向数据集提供方要标定结果。
2. **初始化假设开头静止。** 数据开头如果在运动，重力方向和零偏都会估错。
3. **没有真值，精度没有评估过。** 第 7 节的"起点到终点"只在轨迹是回环时才有意义，目前不确定 tracked-01 是不是回环。
4. **噪声参数**沿用 small_point_lio mid360.yaml 的默认值（`acc_cov 500`、`omg_cov 1000` 等），没有针对 VectorNav 调过。
5. `rematch_thr`（0.1 m）在真实数据上触发了 6–8% 的重新搜索，这个值是否合适还没研究过。
6. **PointCloud2 输入只在代码层面写好，没用真实数据测过。** 它要求有 float64 的 `timestamp` 字段，数值大于 1e12 时按纳秒处理。
7. **`/zhao_lio/path` 无限增长。** 长时间运行时 RViz 会越来越慢。
8. **地图没有保存功能。**

## 9. 建议的下一步

按优先级：

1. 拿到 tracked-01 的外参和真值，或者换一个带真值的公开 Mid360 数据集，用 `compare.sh` 算 APE。
2. 在同一个包上跑 Point-LIO、small_point_lio、Super-LIO，得到同条件下的对比基线。
3. 研究 prematch 的重新搜索比例：用 `timing_csv` 按帧画出 `rematched / points_used`，和角速度对照，看它是不是在剧烈转动时升高。
4. 实现 `batch_window` 策略（毫秒时间窗批量更新，对标 Batch-LIO / lm）。按设计它只是一个新的 `UpdateStrategy`，需要单独写设计文档和计划。
5. 让 launch 文件支持覆盖更多参数，比如 `downsample_size`。

## 10. 相关资料

- 七套 LIO 实现的结构对比：`liop/LIO算法结构对比.md`。其中 small_point_lio 饱和标志的 bug 见 11.1 节。
- 参考源码：`liop/Super-LIO`、`liop/Point-LIO`、`liop/small_point_lio`、`liop/Batch-LIO`（它的 `docs/RESULTS.md` 里有批量更新的实验数据）。
- 许可证：OctVoxMap 移植自 Super-LIO（GPLv3），因此 zhao_lio 整体按 GPLv3 发布。
