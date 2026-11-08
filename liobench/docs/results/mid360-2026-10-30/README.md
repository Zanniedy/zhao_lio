# Mid360 对比：6 套 LIO 在 TIERS IndoorOffice1 上的表现

日期：2026-10-30
平台：liobench（`zhao_lio/liobench`）
实验配置：`experiments/mid360.yaml`

## 1. 结论

- 6 套算法、7 个配置，每个配置重复 3 次，一共 21 次运行，**全部跑完，没有发散**。
- **这条序列分不出精度高低。** 所有算法的 ATE 都在 0.025–0.027 m 之间，最大和最小只差 2 mm。3 次重复的标准差小于 1 mm。序列只有 66 s、35 m，在室内走一圈，对这些算法来说都太简单。
- **RPE 这次不可用。** 1 m 窗口的 RPE 平移误差约 1.7 m，比窗口本身还长，说明误差主要来自坐标系不一致：动捕跟踪的是机器人刚体，算法输出的是 Mid360 的 IMU 位姿，两者之间有一个未知的旋转。原因详见第 4 节。ATE 只比较对齐后的位置，受这个旋转的影响很小，所以 ATE 仍然可信。
- **速度和资源能比较出差别：**
  - 延迟最低的是 Super-LIO（P95 7.2 ms），其次是 zhao_lio prematch（8.0 ms）。
  - CPU 最低的是 zhao_lio：pointwise 7%，prematch 11%。
  - 内存最低的是 zhao_lio，约 100 MB。
  - 资源占用最高的是 genz-lio：CPU 381%，内存 248 MB，延迟 P95 31 ms。
- **zhao_lio 两种策略**：prematch 的内部耗时比 pointwise 少 39%（2.51 对 4.09 ms），P95 少 41%。在 Mid360 上的提速比 NarrowWide 上（20%）更明显，因为每帧点数翻了一倍（约 2 万点）。

## 2. 实验条件

| 项 | 值 |
|---|---|
| 数据 | [TIERS Multi-Modal LiDAR Dataset](https://github.com/TIERS/multi_modal_lidar_dataset) 的 IndoorOffice1。宇树 B1 四足机器人，室内办公室。只用 Mid360：点云 `/mid360/livox/lidar`（PointCloud2，每点有 UNIX 纳秒时间戳，10 Hz，4 线，每帧约 2 万点），IMU `/mid360/livox/imu`（内置 IMU，单位 g，约 200 Hz） |
| 时长 | 66 s，660 帧 |
| 真值 | 动捕 `/vrpn_client_node/unitree_b1/pose`，约 120 Hz，跟踪的是机器人刚体。轨迹长 35.3 m，高度变化 0.18 m |
| 真值处理 | 刚体姿态大约每 3–4 帧翻转一次 180°，可能是标记点对称造成的。用 `orientation_outlier_deg: 30` 去掉翻转帧后，7664 个位姿剩 5571 个 |
| 数据处理 | `liobench convert tiers`：PointCloud2 转成 Livox CustomMsg，点按时间排序；IMU 从 g 换算成 m/s² |
| 外参 | T_imu_lidar = [-0.011, -0.02329, 0.04412]，R = I，即 Mid360 的出厂值，和 FAST-LIO、Point-LIO 的 mid360 配置一致。数据集本身没有给外参 |
| 算法参数 | 每个算法用自带的 Mid360 配置（Point-LIO、Batch-LIO 用 `Point-LIO/config/mid360.yaml`，genz-lio 用 `default/mid360.yaml`，其余用各自的 mid360 配置），只改话题、外参、IMU 单位和盲区（0.5 m），并关闭可视化输出 |
| 精度评估 | evo，按时间戳配对（容差 50 ms），SE(3) 对齐（不估尺度），只算位置误差。这和数据集作者的评估方法一致（`evo_ape --align -r trans_part`） |
| 播放 | 1.0 倍速，每次运行单独起进程 |
| 机器 | Intel i9-13900HX（32 线程），31 GB 内存，Ubuntu 22.04，ROS2 Humble |

## 3. 结果

均值 ± 标准差，来自 3 次重复：

| 算法 | 配置 | ATE m | 延迟均值 ms | 延迟 P95 ms | 内部耗时 均值 / P95 ms | CPU % | 内存 MB |
|---|---|---:|---:|---:|---:|---:|---:|
| Super-LIO | 默认 | 0.026 | **6.17 ± 0.05** | **7.22 ± 0.04** | – | 19 | 133 |
| zhao_lio | prematch（默认） | 0.026 | 6.78 ± 0.08 | 8.03 ± 0.13 | **2.51 / 3.54** | 11 | 100 |
| zhao_lio | pointwise | 0.026 | 8.34 ± 0.06 | 10.44 ± 0.08 | 4.09 / 6.05 | **7** | **97** |
| Batch-LIO | batch_dt 1 ms + OMP | **0.025** | 9.23 ± 0.05 | 10.75 ± 0.09 | – | 170 | 157 |
| Point-LIO | 默认 | **0.025** | 20.23 ± 0.01 | 23.37 ± 0.07 | – | 19 | 157 |
| genz-lio | 默认 | 0.027 | 23.01 ± 0.21 | 31.32 ± 1.70 | – | 381 | 248 |
| small_point_lio | 默认 | **0.025** | 28.07 ± 0.16 | 31.31 ± 0.34 | – | 29 | 118 |

各列的说明：
- **ATE**：所有配置 3 次重复的标准差都小于 0.5 mm，表里没有写 ± 值。各算法之间最大差 2 mm，排名没有意义。
- **延迟**：从雷达帧到达，到发布这一帧里程计所用的时间。它比纯计算多出等待 IMU 和 ROS 通信的时间，zhao_lio 上大约多 4.3 ms。3 次重复的标准差在 2% 以内，可以横向比较。
- **内部耗时**：只有 zhao_lio 输出这项数据。
- **CPU**：100% 表示占满一个核。Batch-LIO 和 genz-lio 是多线程的。

### 和 NarrowWide（MID-70）的对比

| | NarrowWide Tracked-01 | TIERS IndoorOffice1 |
|---|---|---|
| 每帧点数 | 约 1 万 | 约 2 万 |
| zhao_lio prematch 内部耗时 | 0.89 ms | 2.51 ms |
| prematch 相对 pointwise 少用的时间 | 20% | 39% |
| 延迟排名前三 | zhao_lio prematch、zhao_lio pointwise、Super-LIO | Super-LIO、zhao_lio prematch、zhao_lio pointwise |
| 算法之间 ATE 的差距 | 0.07 m（0.119–0.192） | 0.002 m |

Point-LIO 和 small_point_lio 在 Mid360 上延迟较高，分别是 20 ms 和 28 ms。Point-LIO 在 NarrowWide 上只有 7.6 ms；small_point_lio 在 NarrowWide 上发散了，没有可比的数据。原因还没有查证。可能的因素有：
- 每帧点数翻了一倍；
- 两者的 Mid360 配置不同：Point-LIO 的 mid360.yaml 把 `lidar_time_inte` 从 0.1 改成了 0.025，small_point_lio 的降采样是 0.1 m。

## 4. 本次结果的局限

1. **RPE 不可用。**
   - 现象：1 m 窗口的平移 RPE 约 1.7 m，旋转 RPE 在 2–6° 之间，而且各算法的排序和 ATE 不一致。
   - 原因：RPE 比较的是"每走 1 m，在自身坐标系里看到的相对运动"，所以真值和估计必须在同一个机体坐标系里。这里动捕刚体和 Mid360 IMU 之间的旋转是未知的，相对运动被投影到了不同的坐标轴上。
   - 另外，动捕姿态还会频繁翻转 180°，过滤后仍然可能留下错误的帧。
   - 补救办法：用轨迹做手眼标定，估出动捕刚体到 IMU 的旋转，再把它写进 `imu_T_gt`。这一步还没有做。
2. **这条序列太容易。** 所有算法的 ATE 都在 2.5 cm 左右，没有区分度。想看出精度差异，需要更长、更难的 Mid360 序列，比如 TIERS 的 IndoorOffice2（96 s），或者运动更剧烈的数据。
3. **动捕跟踪点和 IMU 不在同一位置。** 两者之间有一段杆臂，长度未知。机器人转动时，这段杆臂会给 ATE 带来一个固定的小偏差，所有算法都一样，所以不影响横向比较。

## 5. 下一步

1. 估计动捕刚体到 IMU 的旋转外参，写进 `datasets/tiers.yaml`，让 RPE 可用。
2. 下载 TIERS IndoorOffice2，放到 `~/datasets/tiers/raw/IndoorOffice2_dataset.bag`，然后运行 `liobench convert tiers && liobench run experiments/mid360.yaml`。
3. 找一条更难的 Mid360 序列，比如有剧烈运动、长走廊或退化场景的数据，才能拉开精度差距。

## 6. 复现与文件

```bash
cd ~/cs_project/liop/zhao_lio/liobench && source /opt/ros/humble/setup.bash
.venv/bin/liobench convert tiers --seq indoor_office1      # 约 2 s
.venv/bin/liobench run experiments/mid360.yaml             # 21 次运行，约 26 分钟
.venv/bin/liobench report results/mid360
```

| 文件 | 内容 |
|---|---|
| `summary.md` / `summary.csv` | 自动生成的汇总表 |
| `overall.csv` | 每个配置的成功率 |
| `runs.csv` | 每次运行的全部字段 |
| `experiment.yaml` | 实验配置 |
| `indoor_office1.meta.yaml` | 转换后数据的统计信息，包括去掉了多少个翻转帧 |
| `trajectories/<算法>.tum`、`.run.json` | 第 1 次重复的估计轨迹和指标 |
