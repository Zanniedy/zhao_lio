# liobench

liobench 是 ROS2 LiDAR-惯性里程计的测试平台。给定算法和数据集，它对每种组合都输出同一套指标：

| 指标 | 测量方法 |
|---|---|
| ATE RMSE（m） | evo：先做 SE(3) Umeyama 对齐（不估尺度），再算位置误差 RMSE |
| RPE（m / °） | evo：固定距离窗口（默认 10 m）内的相对位移和相对旋转误差 RMSE |
| 每帧延迟 均值 / P95 | 从雷达帧到达，到算法发布这一帧里程计的墙钟时间。算法自己输出每帧耗时的，另外给出"内部均值 / P95" |
| CPU 使用率 | psutil 每 0.25 s 采样一次算法进程树，取播放期间的平均值，100% 表示占满一个核 |
| 峰值内存 | 采样到的 RSS 和内核记录的 VmHWM 取较大值 |
| 发散率 / 成功率 | 满足任一条件即判为失败：崩溃、没有输出、出现 NaN、位置跳变或速度超过阈值、有输出的帧少于 90%、有真值时 ATE 超过 `max_ate`。按算法统计所有序列和重复次数 |

算法按黑盒方式运行：直接启动，不修改源码。每次运行都单独起进程，所以不同运行之间的 CPU 和内存读数不会混在一起。

教程见 [`docs/TUTORIAL.md`](docs/TUTORIAL.md)。第一次基线对比的结果见 [`docs/results/baseline-2026-10-29/`](docs/results/baseline-2026-10-29/README.md)。

## 安装（只需一次）

liobench 放在 zhao_lio 仓库里。对比用的参考算法源码不在仓库里，默认从仓库的上一级目录（`liop/`）找，也可以用环境变量 `LIOBENCH_REFS` 指定。livox_ros_driver2 的安装目录用 `LIOBENCH_LIVOX` 指定，默认是 `~/cs_project/nav/cs/install`。

```bash
cd ~/cs_project/liop/zhao_lio
# 1. 编译 zhao_lio 本身（仓库根目录就是 colcon 工作空间）
source /opt/ros/humble/setup.bash && source ~/cs_project/nav/cs/install/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# 2. Python 环境：用系统 Python 3.10，这样能 import rclpy
cd liobench
/usr/bin/python3 -m venv --system-site-packages .venv
.venv/bin/pip install -e '.[test]' rosbags
# 3. 编译参考算法（约 6 分钟）
ws/build_refs.sh
```

`ws/build_refs.sh` 会在 `ws/src` 里建立指向参考算法源码的符号链接，再执行 `colcon build`。它会自动把 conda 从 PATH 里去掉，否则 CMake 会链接到 conda 的库。

## 日常使用

```bash
cd ~/cs_project/liop/liobench && source /opt/ros/humble/setup.bash
L=.venv/bin/liobench

$L ls                                        # 算法、序列、转换状态、有没有真值
$L convert narrowwide                        # 把原始数据包和真值转成标准格式（每条序列一次）
$L run experiments/smoke.yaml                # 每个算法跑 30 s，检查都能启动
$L run experiments/baseline.yaml             # 完整对比
$L report results/baseline                   # 表格 + summary.md / summary.csv / overall.csv / runs.csv
$L reeval results/baseline                   # 不重跑算法，重新计算指标（比如补上真值以后）
$L compare results/baseline results/new      # 逐项对比两次实验，用来检查退化
$L dev dev/zhao_lio.yaml --note "改了什么"   # 开发循环：编译 + 存档 + 测试 + 对比 + 验收
$L history dev/zhao_lio.yaml                # 所有开发版本的指标
$L restore dev/zhao_lio.yaml <版本> <目录>   # 取出某个版本的源码快照
```

`run` 的常用参数：
- `--only zhao_lio point_lio`：只跑这几个算法。
- `--duration 60`：每个包只播前 60 秒。
- `--rate 2`：2 倍速播放。
- `--force`：已经跑完的也重跑。
- `--rerun-failed`：重跑状态为"失败"的运行。
- `--dry-run`：只列出要跑哪些，不真正执行。

已经跑完的运行会自动跳过，所以一批实验中断后，重新执行同一条命令就能接着跑。

## 实验配置（`experiments/*.yaml`）

```yaml
name: sweep_zhao_lio
algorithms:
  - point_lio                                 # 默认参数
  - name: zhao_lio
    params:                                   # 写成列表就做扫描：2 × 2 = 4 个配置
      update_strategy: [pointwise, pointwise_prematch]
      downsample_size: [0.2, 0.5]
sequences: [narrowwide/tracked-01, narrowwide/*]   # dataset/* 表示所有已转换的序列
repeats: 3                                    # 重复次数；报告里显示 均值 ± 标准差
rate: 1.0
rpe: {delta: 10, unit: m}                     # unit：m | s | frames
divergence: {max_speed: 20, max_jump: 5, min_coverage: 0.9, max_ate: 10}
```

`params` 的键是算法自己的参数名。用 ROS 参数的算法写点分形式的全名，比如 `mapping.plane_thr`。本身是列表的参数要扫描时，写成嵌套列表：`extrinsic_T: [[0,0,0], [0.1,0,0]]`。

结果目录是 `results/<name>/<算法>/<参数哈希>/<序列>/r<重复序号>/`：

| 文件 | 内容 |
|---|---|
| `run.json` | 全部指标、状态和失败原因 |
| `est.tum` | 估计轨迹 |
| `algo_config.yaml`、`command.sh` | 实际使用的配置和启动命令 |
| `algo.log`、`play.log`、`recorder.log` | 各进程的输出 |
| `lidar.csv`、`odom.csv` | 消息到达时间 |
| `resources.csv` | CPU 和内存采样 |
| `internal_timing.csv` | 算法自己的每帧耗时（zhao_lio） |

报告表格和 `summary.md` 是中文。CSV 的列名用 `run.json` 里的英文键名，方便 pandas 读取。

## 数据集与转换器

每个数据集先转换一次，变成统一的标准格式。所有算法读同一份转换后的数据：

| 话题 | 格式 |
|---|---|
| `/liobench/lidar` | `livox_ros_driver2/CustomMsg`。`header.stamp` 和 `timebase` 都是最早一个点的时间；`offset_time` 是相对它的纳秒数；点按时间排序；`line` 存线号（ring）。消息按扫描结束时间写入包，和真实驱动的发布时刻一致 |
| `/liobench/imu` | `sensor_msgs/Imu`，加速度 m/s²，角速度 rad/s |

真值保存为 IMU 坐标系下的 `gt.tum`。`meta.yaml` 记录扫描频率、线数和统计信息。

6 套算法都能读 CustomMsg，所以每个算法只需要适配这一种输入。原始数据可以是 ROS1 包或 ROS2 包；转换器用 `rosbags` 读取，不需要安装 ROS1。解码器按名字注册：

```bash
$L convert --list
# 雷达：livox_custom、livox_pointcloud2、velodyne、ouster、hesai、robosense、pointcloud2（通用，自己指定字段）
# IMU：imu（acc_unit m/s2|g，gyro_unit rad/s|deg/s）
# 真值：tum、csv（指定列名）、kitti（另需 times 文件）；可选 imu_T_gt 和 time_offset
```

数据集 yaml 的写法和示例见 `datasets/README.md`。要支持新的雷达格式，在 `liobench/convert/lidar.py` 里加一个带 `@LIDAR.register("名字")` 的类，返回一个 `Scan` 即可。

## 接入新算法（`algorithms/<名字>.yaml`）

```yaml
package: my_lio                      # ROS2 包名和可执行文件名
executable: my_lio_node
workspace: ws/install                # 运行前 source 这个目录的 setup.bash
extra_setup: [$LIOBENCH_LIVOX/setup.bash]   # livox_ros_driver2
config:
  mode: ros_params                   # ros_params：用 --params-file 传；yaml_file：把文件路径作为一个 ROS 参数传入
  base: ../my_lio/config/default.yaml
  node_key: "/**"                    # ros_params：yaml 的顶层键
  # path_param: config_path          # yaml_file：接收文件路径的 ROS 参数名
lidar_types: {livox_custom: 1}       # 这个算法里表示 Livox CustomMsg 输入的取值
sensor:                              # 算法参数 <- 数据集 / 转换结果里的传感器信息
  common.lid_topic: "{lidar_topic}"
  common.imu_topic: "{imu_topic}"
  mapping.extrinsic_T: "{extrinsic_t}"
  mapping.extrinsic_R: "{extrinsic_r}"     # 另有 extrinsic_r_colmajor、extrinsic_tr_colmajor
  mapping.acc_norm: "{acc_norm}"
fixed: {mapping.extrinsic_est_en: false}   # 每次都设置；实验里的 params 会覆盖 sensor 和 fixed
odom_topic: /odom                    # nav_msgs/Odometry，IMU 坐标系下的位姿，时间戳在扫描结束附近
helpers: []                          # 同时启动的辅助命令（比如静态 TF）
timing: {param: timing_csv, column: total_ms}   # 可选：算法自己输出的每帧耗时 CSV
```

写好之后运行 `liobench run experiments/smoke.yaml --only <名字>`。失败时看对应目录里的 `algo.log`。

## 测量上的注意事项

- **延迟不等于纯计算时间。** 在 tracked-01 上，zhao_lio 的延迟是 3.2 ms，内部耗时是 0.9 ms。多出的部分主要是等待扫描结束后的第一条 IMU（200 Hz 时最多 5 ms），每个算法都要等这条 IMU 才能处理这一帧；其余是 ROS 通信开销。延迟按播放倍速测得。算法如果比实时慢，会积压帧，表现为延迟持续上升、有输出的帧比例下降。
- 每次运行使用 `ROS_DOMAIN_ID=77`、`ROS_LOCALHOST_ONLY=1`，以及 `dds/` 里的 Fast DDS 配置。默认的 512 KiB 共享内存段会让 best-effort 订阅者（Point-LIO、Super-LIO）丢掉大部分雷达帧。
- 适配器里关闭了各算法的可视化输出（见 `fixed:`），这样算法的耗时不包含给 RViz 发数据的开销。
- 同一时间只跑一批实验。并行跑会争抢 CPU，计时就不准了。

## 单元测试

```bash
.venv/bin/python -m pytest -q
```
