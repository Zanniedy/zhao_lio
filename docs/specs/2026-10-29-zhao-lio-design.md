# zhao_lio 设计

日期：2026-10-29

## 1. 目标

zhao_lio 是一个用于学习和实验的 LiDAR-惯性里程计。它组合两类已有做法：

- Super-LIO 的预处理、降采样和紧凑体素地图（`VoxelGridClosest`、`OctVoxMap`）；
- Point-LIO / small_point_lio 的逐点 EKF 更新（30 维状态，IMU 作为观测）。

在此基础上加入 TBB 并行预关联：先并行算出整帧点的局部平面，再串行逐点更新。

更新策略做成可切换参数，同一个可执行文件即可在同一数据上做 A/B 对比。

### 成功标准

1. `pointwise` 与 `pointwise_prematch` 两种策略都能在 Livox Avia/Mid360 数据集上稳定跑完，不发散。
2. `pointwise_prematch` 的关联+更新阶段耗时明显低于 `pointwise`，以实测为准。
3. 慢速数据上两种策略轨迹差很小，`prematch` 的重新搜索比例接近 0。
4. 可以输出 TUM 格式轨迹，用 `evo` 与 Point-LIO、Super-LIO 对比。

### 第一版不做

- `batch_window`（毫秒时间窗批量更新），之后单独做一版；
- `scan_batch`（整帧去畸变 + 迭代更新）；
- 重定位、先验地图对齐、地图保存以外的工程功能。

## 2. 环境

- Ubuntu 22.04、ROS2 Humble、C++17
- Eigen 3.4、TBB 2021.5；不依赖 PCL
- 许可证：`OctVoxMap` 移植自 Super-LIO（GPLv3），因此 zhao_lio 整体按 GPLv3 发布
- 传感器：Livox Avia / Mid360（`livox_ros_driver2/CustomMsg`），也支持 `sensor_msgs/PointCloud2`
- `livox_ros_driver2` 不在系统 ROS 环境里，本机已编译在 `~/cs_project/nav/cs/install`，构建 zhao_lio 前需要先 source 该工作空间；CMake 里做成可选依赖，找不到时只编译 PointCloud2 输入
- `evo` 未安装，评估前需要 `pip install evo`

## 3. 目录结构

```
liop/zhao_lio/
  docs/specs/                本文件
  src/
    zhao_lio_core/          纯 C++ 库，只依赖 Eigen、TBB，不依赖 ROS、PCL
      include/zhao_lio/
        types.h              Point{xyz, t}、ImuSample、ScanFrame、NavState
        params.h             参数结构体
        preprocess.h         盲区/距离过滤、按点采样、按时间排序
        voxel_downsample.h   VoxelGridClosest 改写，保留原始点和时间戳
        oct_vox_map.hpp      移植 Super-LIO OctVoxMap + HKNN 邻域顺序表
        plane.h              5 近邻平面拟合
        so3.h                Exp / Log / hat / A 矩阵
        eskf30.h             30 维 ESKF
        updater/
          updater.h          UpdateStrategy 接口
          pointwise.h
          pointwise_prematch.h
        zhao_lio.h          初始化状态机 + 主流程
      src/                   对应 .cpp
      test/                  gtest
    zhao_lio_ros2/          ROS2 节点、离线 bag 回放可执行文件、launch、config
  scripts/compare.sh         evo 轨迹对比
```

`zhao_lio_core` 的类不依赖任何全局变量，Point-LIO 那种全局状态不复用。

## 4. 每帧数据流

```
LiDAR 帧 + 覆盖该帧的 IMU 段
  ↓ 预处理（串行距离过滤和按点采样，开销很小；点保持在 LiDAR 坐标系，外参在观测模型里使用）
  ↓ 降采样：每个体素保留离体素中心最近的原始点，时间戳不变
  ↓ 按点时间排序，与 IMU 按时间合并成事件序列
  ↓ 更新策略（第 6 节）
  ↓ 用每个点更新后的状态把点变到世界系
  ↓ 帧末把整帧世界点批量插入 OctVoxMap
  ↓ 输出：帧末 NavState、可选逐点 odom、世界系点云
```

地图在帧末批量插入，同一帧内的点不会彼此匹配。关联期间地图只读，因此预关联可以安全地并行。

### 初始化状态机

```
WaitImuInit  → 累计 imu_init_num 个 IMU（假设静止）：用平均加速度对齐重力使世界系 z 轴朝上，
               a 取平均加速度方向乘重力模长，bg 取平均角速度，ω = 0
WaitMapInit  → IMU 正常传播和更新，用当前位姿把点插入地图，累计 init_map_frames 帧
Process      → 正常处理
```

## 5. 组件

### 5.1 降采样 `VoxelDownsample`

- 体素键：`round(p / leaf)`，打包成 64 位整数（与 Super-LIO 相同）。
- 每个体素保留离中心最近的点，点的时间戳原样保留。之所以不用 PCL `VoxelGrid` 求质心，是因为质心会把时间平均掉，逐点更新需要每个点有准确的时间。
- 两种实现：
  - 串行版：哈希表（`robin_hood::unordered_flat_map`），即 Super-LIO 原版。
  - 并行版：TBB 并行计算每个点的 (key, dist², idx)，`tbb::parallel_sort` 按 key 排序，再按段归约选最小 dist²。
- 输出顺序不影响结果，因为后续会按时间排序。
- 先用基准测试比较两种实现；并行版不快于串行版时，默认用串行版。

### 5.2 地图 `OctVoxMap`

从 `Super-LIO/src/super_lio/include/OctVoxMap/` 移植，包括 `OctVoxMap.hpp`、`HKNN_list60_gem.h` 和 `tsl/`。移植时：

- 改命名空间为 `zhao_lio`；
- 去掉对 Super-LIO 全局参数的依赖，分辨率和容量由构造参数传入；
- 保留 `getTopK`（K=5）、`insert`、LRU 容量淘汰；
- 原版提前停止表 `orders_min_dis2` 写死对应 0.5 m 分辨率，移植时按 `(子体素边长/0.25)²` 缩放，使其他分辨率下搜索仍然正确；
- `getTopK` 必须是 const 并且线程安全（只读），供 TBB 并行调用。

### 5.3 平面拟合 `fit_plane`

沿用 Super-LIO 的 `calc_plane_coeff`：4–5 个近邻，用 QR 求解 `A n = -1`，归一化后检查每个近邻点到平面的距离 ≤ `plane_thr`（默认 0.1 m）。输出 `(n, d)`，满足 `nᵀp + d = 0`。

### 5.4 滤波器 `Eskf30`

参考 small_point_lio 的 `eskf.h` 重写。

误差状态顺序（与 small_point_lio 一致，H 的前 12 列对应 p、R、外参 R、外参 t）：

| 索引 | 量 |
|---|---|
| 0–2 | 位置 p |
| 3–5 | 姿态 R |
| 6–8 | 外参旋转 R_LI |
| 9–11 | 外参平移 t_LI |
| 12–14 | 速度 v |
| 15–17 | 角速度 ω |
| 18–20 | 加速度 a（机体系，不含重力） |
| 21–23 | 重力 g |
| 24–26 | 陀螺零偏 bg |
| 27–29 | 加速度计零偏 ba |

接口：

- `predict_state(t)`：只传播名义状态 p、R、v，每个点调用一次。
- `predict_cov(t, Q)`：传播 30×30 协方差，只在 IMU 时刻调用。F 中速度对姿态的块取 `-R·[a]×·dt`（small_point_lio `eskf.h:116` 漏乘了 dt，Point-LIO 的 IKFoM 是乘 dt 的）。
- `update_imu(ω_m, a_m)`：6 维观测 `z = [ω_m - ω - bg; a_m·g/|a_m| - a - ba]`。饱和轴（`|ω_m| > satu_gyro` 或 `|a_m| > satu_acc`）的对应行不参与更新。陀螺通道读 `satu[i]`，加速度通道读 `satu[i+3]`（small_point_lio 原代码在加速度通道读 `satu[i]`，这里修正）。
- `update_point(n, d, p_lidar, R_meas)`：1×12 的 H，rank-1 更新，只访问 P 的前 12 列。`extrinsic_est_en=false` 时外参列置零，并用固定外参。
- `point_residual(n, d, p_lidar)`：返回 `r = nᵀ(R(R_LI p + t_LI) + p) + d`，供策略做门限判断。

残差门限采用 Point-LIO 的规则：`|p_lidar| > match_s · r²` 才接受。

### 5.5 更新策略接口

```cpp
struct StrategyStats {
  int points_in, points_used, rematched;
  double t_prematch_ms, t_update_ms;
};

class UpdateStrategy {
 public:
  virtual ~UpdateStrategy() = default;
  // events: 按时间排序后的点与 IMU 事件；points_world 输出每个点更新后的世界坐标
  virtual void process(const std::vector<Event>& events, Eskf30& kf, const OctVoxMap& map,
                       std::vector<Eigen::Vector3f>& points_world, StrategyStats& stats) = 0;
};
```

IMU 事件对两种策略相同：`predict_state` → `predict_cov` → `update_imu`。

## 6. 两种策略

### 6.1 `pointwise`

对每个点事件，串行执行：

```
predict_state(t_i)
p_w = T_cur · p_i
近邻 = map.getTopK(p_w)；不足 4 个 → 跳过
(n, d) = fit_plane(近邻)；失败 → 跳过
r = point_residual(...)；不过门限 → 跳过
update_point(...)
points_world[i] = T_cur · p_i      （用更新后的状态）
```

这是基线，对应 small_point_lio 的逐点更新，区别是地图换成 OctVoxMap、帧末批量插入。

### 6.2 `pointwise_prematch`

阶段 1（预测）：复制滤波器的名义状态，按事件序列只做 `predict_state`，遇到 IMU 事件时用 IMU 测量更新 ω、a 的近似值（取测量值减当前零偏），得到每个点时刻的预测位姿 `T_pred_i`。这一步不改动真实滤波器。

阶段 2（并行预关联，TBB）：对每个点 `p_w = T_pred_i · p_i`，执行 KNN + 平面拟合，结果存入 `planes[i]`（可能无效）。

阶段 3（串行更新）：

```
predict_state(t_i)
若 planes[i] 无效 → 跳过
r = point_residual(planes[i], p_i)
若 |r| > rematch_thr：
    重新 KNN + fit_plane（用当前状态）；失败 → 跳过；rematched++
若 r 不过门限 → 跳过
update_point(...)
points_world[i] = T_cur · p_i
```

`rematched / points_used` 作为实验指标记录，它反映预关联在什么运动强度下开始失效。

## 7. 参数（yaml）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `update_strategy` | `pointwise_prematch` | `pointwise` 或 `pointwise_prematch` |
| `num_threads` | 0 | 0 表示 TBB 自动 |
| `blind_min` / `blind_max` | 0.5 / 100 m | 距离过滤 |
| `point_filter_num` | 1 | 按点采样间隔 |
| `downsample_size` | 0.5 m | 降采样体素 |
| `downsample_parallel` | false | 降采样是否用并行版 |
| `map_resolution` | 0.5 m | OctVoxMap 大体素 |
| `map_capacity` | 1000000 | 地图体素上限 |
| `plane_thr` | 0.1 m | 平面拟合检查 |
| `match_s` | 81 | 残差门限 |
| `rematch_thr` | 0.1 m | 预关联重新搜索门限 |
| `laser_point_cov` | 0.01 | 点观测噪声 |
| `imu_meas_omg_cov` / `imu_meas_acc_cov` | 按 Point-LIO 默认 | IMU 观测噪声 |
| 过程噪声 | 按 Point-LIO output 模式默认 | vel、ω、a、bg、ba |
| `satu_gyro` / `satu_acc` | 35 rad/s / 3 g | 饱和阈值 |
| `extrinsic_est_en` | false | 外参在线估计 |
| `extrinsic_R` / `extrinsic_T` | 传感器配置 | LiDAR→IMU 外参 |
| `imu_init_num` | 100 | 初始化 IMU 数 |
| `init_map_frames` | 3 | 地图初始化帧数 |
| `publish_pointwise_odom` | false | 是否逐点发布 odom |

为 Avia 和 Mid360 各提供一份 yaml。

## 8. ROS2 节点与工具

`zhao_lio_ros2`：
- 订阅 `CustomMsg` 或 `PointCloud2`，以及 `sensor_msgs/Imu`；
- 用 IMU/LiDAR 缓冲按时间同步，组成 ScanFrame；
- 发布 `/zhao_lio/odom`、`/zhao_lio/path`、`/zhao_lio/cloud_world` 和 TF；
- launch 文件按传感器选择 yaml，可在命令行覆盖 `update_strategy`。

离线回放（同在 `zhao_lio_ros2` 包内，复用消息转换和参数加载）：
- `zhao_lio_replay`：用 `rosbag2_cpp` 直接读包，按时间顺序喂给 core，不经过 ROS 通信，计时不受播放速率影响；
- 每帧记录预处理、降采样、预关联、更新、建图耗时和 StrategyStats，写 CSV；
- 轨迹写成 TUM 格式；
- `scripts/compare.sh`：用 `evo_ape` / `evo_traj` 对比多条轨迹。

数据：用户自行选择公开的 Mid360 数据集；ROS1 包用 `Batch-LIO/scripts/convert_bag.py` 转换为 ROS2。第一版开发阶段只依赖合成数据的单元测试，不依赖真实数据集。

## 9. 错误处理

- 帧内没有可用点：跳过更新，只做 IMU 传播，打印警告。
- 点时间早于当前滤波器时间：丢弃该点。
- LiDAR 帧缺少覆盖它的 IMU：等待；等待中的帧超过 20 帧时丢弃最旧的帧。
- rank-1 更新分母接近 0、`update_imu` LDLT 失败：跳过该次更新。
- 状态出现 NaN：打印错误，清空地图并重置到 `WaitImuInit`。

## 10. 测试

gtest，放在 `zhao_lio_core/test/`：

1. 降采样：每个体素只保留一个点；保留的是离中心最近的点；时间戳不变；并行版与串行版结果集合相同。
2. 地图：随机点云上 `getTopK` 结果与暴力搜索一致（在其搜索邻域内）；并发调用 `getTopK` 结果稳定。
3. 平面拟合：合成平面加噪声能恢复法向；非平面点被拒绝。
4. ESKF：静止 IMU 输入下 1000 次传播+更新状态不发散；合成平面上单点更新后残差下降；饱和屏蔽时对应通道状态不变，陀螺饱和不影响加速度通道。
5. 策略：在合成场景（盒子房间 + 已知平滑轨迹）上两种策略都能跟踪轨迹，误差小于阈值；`prematch` 的 `rematched / points_used` < 1%。

集成验证：在真实数据包上用 `zhao_lio_replay` 跑两种策略，对比耗时和轨迹。
