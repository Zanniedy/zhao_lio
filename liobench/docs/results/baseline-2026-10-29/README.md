# 基线对比：6 套 LIO 在 NarrowWide Tracked-01 上的表现

运行日期：2026-10-29；真值评估：2026-10-30

> 2026-10-30 起 first_lio 更名为 zhao_lio（代码相同）。本文保留运行时的名字 first_lio；`results/baseline` 里的运行记录已经改成 zhao_lio，可以直接和新版本 `compare`。
平台：liobench（`~/cs_project/liop/liobench`）

## 1. 结论

- 6 套算法、7 个配置在 tracked-01 全长（621 s，6215 帧）上运行。**6 个跑完全程，small_point_lio 在第 302 s 附近发散**。
- **精度**（ATE RMSE，有真值）：
  - first_lio 两种策略最好：pointwise 0.119 m，prematch 0.122 m。genz-lio 0.125 m 紧随其后。
  - 之后依次是 Super-LIO 0.154 m、Point-LIO 0.191 m、Batch-LIO 0.192 m。
  - **最好和最差之间只差 7 cm，而且每个配置只跑了 1 次。** first_lio 和 genz-lio 之间的差距（0.003–0.006 m）在重复运行的波动范围内，不能说谁更好。
- **速度和资源**：first_lio 最低，延迟 P95 约 6 ms，CPU 约 5%，内存约 116 MB。genz-lio 最高，CPU 341%，内存 1.17 GB，延迟 P95 24.5 ms。
- **first_lio 两种策略的取舍**：
  - prematch 的内部耗时少 20%（0.89 对 1.11 ms），P95 少 31%（1.42 对 2.06 ms）。
  - 精度上，prematch 的 ATE 比 pointwise 多 0.003 m，10 m 窗口的 RPE 多 0.026 m。差距很小，但方向和 HANDOFF 里"IMU 预测在这台履带车上偏差较大"的观察一致。
- **和 GenZ-LIO 论文对照**：genz-lio 本次 0.125 m，论文给出 0.16 m，量级一致，说明平台的数据转换和评估流程没有引入明显偏差。Point-LIO 本次 0.19 m，论文给出 1.48 m。差别来自配置不同：本次使用 Point-LIO 默认参数加作者提供的外参，论文没有公开它用的 Point-LIO 配置。

## 2. 实验条件

| 项 | 值 |
|---|---|
| 数据 | NarrowWide Tracked-01：Livox MID-70（CustomMsg，10 Hz，每帧约 9984 点）+ VectorNav VN-100（200 Hz）。621.4 s，6215 帧，履带机器人，路线形成回环 |
| 真值 | 数据集提供的 `gt_tracked-01.txt`，VN-100 IMU 坐标系，6211 个位姿，约 10 Hz。真值轨迹长 265.7 m，起点到终点 0.19 m |
| 数据处理 | `liobench convert` 转成标准格式：点时间改用 header.stamp + offset_time（原包的 timebase 是设备上电时间）；点按时间排序；雷达帧按扫描结束时间写入；IMU 单位为 m/s² |
| 外参 | T_imu_lidar = [-0.017062, -0.003617, -0.019032]，R = I，取自 genz-lio 作者的 NarrowWide 配置。所有算法都关闭外参在线估计 |
| 播放 | `ros2 bag play`，1.0 倍速，每次运行单独起进程，`ROS_LOCALHOST_ONLY=1`，Fast DDS 共享内存段 64 MB |
| 参数 | 各算法使用自带的默认配置，只改话题、雷达类型、IMU 单位、外参和盲区（0.5 m），并关闭可视化输出。genz-lio 使用作者针对 Tracked-01 调好的配置 |
| 精度评估 | evo。按时间戳配对（容差 50 ms），SE(3) Umeyama 对齐（不估尺度）。RPE 取 10 m 窗口 |
| 重复次数 | 1 次 |
| 机器 | Intel i9-13900HX（32 线程），31 GB 内存，Ubuntu 22.04，ROS2 Humble，内核 6.8 |

## 3. 精度

| 算法 | 配置 | ATE RMSE m | RPE 10 m（m） | RPE 10 m（°） | 配对到真值的位姿 |
|---|---|---:|---:|---:|---:|
| first_lio | pointwise | **0.119** | **0.218** | **1.814** | 6204 / 6211 |
| first_lio | prematch（默认） | 0.122 | 0.244 | 1.810 | 6204 / 6211 |
| genz-lio | 作者 Tracked 配置 | 0.125 | 0.314 | 2.064 | 6211 / 6211 |
| Super-LIO | 默认 | 0.154 | 0.266 | 2.018 | 6206 / 6211 |
| Point-LIO | 默认 | 0.191 | 0.240 | 1.855 | 6210 / 6211 |
| Batch-LIO | batch_dt 1 ms + OMP | 0.192 | 0.240 | 1.958 | 6209 / 6211 |
| small_point_lio | 默认 | 发散（10034） | 24.0 | – | 6203 / 6211 |

RPE 随窗口长度变化很大，不同窗口下的排序也不一样。下表是另外算的 1 m 和 100 m 窗口（只取平移部分，单位 m）：

| 算法 | 配置 | RPE 1 m | RPE 10 m | RPE 100 m |
|---|---|---:|---:|---:|
| first_lio | pointwise | **0.067** | **0.218** | 2.003 |
| first_lio | prematch | 0.071 | 0.244 | 2.058 |
| Super-LIO | 默认 | 0.070 | 0.266 | 2.611 |
| genz-lio | 默认 | 0.077 | 0.314 | 2.972 |
| Point-LIO | 默认 | 0.080 | 0.240 | **1.253** |
| Batch-LIO | 默认 | 0.082 | 0.240 | 1.475 |

- 短距离（1 m）上 first_lio 和 Super-LIO 的局部一致性最好。长距离（100 m）上 Point-LIO 和 Batch-LIO 的累积漂移最小。
- 不过 Point-LIO 和 Batch-LIO 的 ATE 反而较大，轨迹长度也多出约 50 m（317 m，真值 265.7 m）。这说明它们的轨迹有高频抖动：抖动让 ATE 和轨迹长度变大，但长距离上会相互抵消。
- 这里每种窗口下的 RPE 都只是一次运行的结果。想对 RPE 下结论，要先看重复运行的波动。

## 4. 速度与资源

| 算法 | 配置 | 延迟均值 ms | 延迟 P95 ms | 延迟最大 ms | 内部耗时均值 / P95 ms | CPU 均值 % | CPU P95 % | 峰值内存 MB |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| first_lio | prematch（默认） | **3.16** | **5.86** | 10.5 | 0.89 / 1.42 | 5 | 8 | 117 |
| first_lio | pointwise | 3.40 | 6.13 | 11.5 | 1.11 / 2.06 | **4** | 8 | **115** |
| Super-LIO | 默认 | 3.61 | 6.08 | 10.5 | – | 13 | 20 | 143 |
| Batch-LIO | batch_dt 1 ms + OMP | 5.36 | 8.65 | 16.7 | – | 140 | 177 | 174 |
| Point-LIO | 默认 | 7.58 | 11.57 | 16.5 | – | 8 | 12 | 170 |
| genz-lio | 作者 Tracked 配置 | 15.41 | 24.51 | 75.7 | – | 341 | 508 | 1171 |
| small_point_lio | 默认 | 发散 | | | | | | |

- **延迟**：从雷达帧到达，到算法发布这一帧的里程计所用的墙钟时间。它比内部耗时多出约 2.3 ms（first_lio：3.16 对 0.89 ms）。多出的部分主要是等待 IMU：算法要收到时间晚于扫描结束时刻的那条 IMU 才能处理这一帧，IMU 是 200 Hz，最多要等 5 ms；其余是 ROS 通信开销。所有算法的这部分开销相同，所以延迟可以横向比较，但它不等于纯计算时间。
- **内部耗时**：算法自己统计的每帧处理时间。目前只有 first_lio 输出这个数据。
- **CPU**：100% 表示占满一个核。Batch-LIO 用 OpenMP、genz-lio 用多线程，所以会超过 100%。实时播放时，算法大部分时间在等数据，所以这一列反映的是"实时运行要占多少资源"，不是峰值算力。
- **重复运行的波动**：每个配置只跑了 1 次。2026-10-29 用同一份 first_lio 代码跑了 3 次（每次 60 s），内部耗时在 -7.6% 到 +3.2% 之间波动。两个 first_lio 配置之间延迟只差 0.2 ms，在这个范围内不能下结论。
- Batch-LIO 比 Point-LIO 快 29%（延迟均值 5.36 对 7.58 ms），精度几乎相同（ATE 0.192 对 0.191 m），代价是 CPU 从 8% 升到 140%。这和 Batch-LIO 文档里"批量更新让 OpenMP 并行有效"的说法一致。

## 5. 稳定性

| 算法 | 配置 | 结果 | 有里程计的帧 | 起点到终点 m | 轨迹长度 m |
|---|---|---|---:|---:|---:|
| 真值 | – | – | – | 0.19 | 265.7 |
| first_lio | prematch | 跑完全程 | 6205 / 6215 | 0.50 | 293.2 |
| first_lio | pointwise | 跑完全程 | 6205 / 6215 | 0.41 | 298.6 |
| Super-LIO | 默认 | 跑完全程 | 6207 / 6215 | 0.47 | 272.5 |
| Batch-LIO | 默认 | 跑完全程 | 6210 / 6215 | 0.86 | 317.4 |
| Point-LIO | 默认 | 跑完全程 | 6211 / 6215 | 0.64 | 317.8 |
| genz-lio | 默认 | 跑完全程 | 6213 / 6215 | 0.30 | 266.0 |
| small_point_lio | 默认 | **发散** | 6204 / 6215 | 41676 | 47048 |

**small_point_lio 发散的过程：**
- 第 288 s，单帧位移首次超过 1 m；
- 第 302 s，速度超过 20 m/s；
- 此后轨迹飞出 4 万多米。

前 30 s 的冒烟测试里它是正常的，所以问题出在中段的某段运动或场景上，比如狭窄通道和开阔区域之间的切换。它使用上游的 mid360 默认参数（0.1 m 降采样），没有针对 MID-70 或 VN-100 调过。在 30 s 的参数扫描里，把降采样改成 0.5 m 会更早漂移，所以默认值仍然是 0.1 m。

## 6. 下一步

1. **Tracked-02 的真值已经有了**（`gt_tracked-02.txt`）。下载 Tracked-02 的 ROS2 包，放到 `~/datasets/narrowwide/raw/Tracked-02`，然后运行：
   ```bash
   liobench convert narrowwide && liobench run experiments/baseline.yaml
   ```
   已完成的 tracked-01 运行会自动跳过。这样就能得到两条序列上的成功率。
2. **精度和耗时都改成重复 3 次。** first_lio 和 genz-lio 之间、first_lio 两种策略之间的精度差距只有毫米级，单次运行判断不了。
3. **prematch 的精度代价**：ATE 只差 0.003 m，但 10 m 窗口的 RPE 差 0.026 m。可以用 `rematch_thr` 做参数扫描，看能不能在保住速度的同时把 RPE 拉回到 pointwise 的水平。
4. **small_point_lio** 如果要作为对比基线，需要单独查第 288–302 s 发生了什么。

## 7. 复现与文件

```bash
cd ~/cs_project/liop/liobench && source /opt/ros/humble/setup.bash
.venv/bin/liobench convert narrowwide --seq tracked-01
.venv/bin/liobench run experiments/baseline.yaml      # 约 75 分钟
.venv/bin/liobench report results/baseline
```

本目录中的文件：

| 文件 | 内容 |
|---|---|
| `summary.md` / `summary.csv` | liobench 自动生成的汇总表，一行对应一个配置和一条序列 |
| `overall.csv` | 每个配置的成功率 |
| `runs.csv` | 每次运行的全部字段，包括实际使用的参数 |
| `experiment.yaml` | 实验配置 |
| `run.log` | 运行过程的输出（英文，是当时的原始记录） |
| `tracked-01.meta.yaml` | 转换后数据的统计信息 |
| `trajectories/<算法>.tum` | 估计轨迹，TUM 格式，IMU 坐标系 |
| `trajectories/<算法>.run.json` | 单次运行的全部指标，包括 ATE 和 RPE |

完整的原始结果在 `results/baseline/` 下（包括每次运行的日志、资源采样和消息时间）。这个目录不会提交到 git，删除后可以重跑生成。
