# 数据集配置

每个 `<名字>.yaml` 描述一个数据集，包含下面几部分：

- **`output`**：`liobench convert <名字>` 写出标准格式数据的位置。每条序列一个目录：`<output>/<序列>/{bag, gt.tum, meta.yaml}`。
- **`source`**：怎么解码原始数据。序列里可以写自己的 `source:`，覆盖这里的设置。
  - `lidar.decoder`：解码器名称，可选值见 `liobench convert --list`，包括 `livox_custom`、`velodyne`、`ouster`、`hesai`、`robosense`、`livox_pointcloud2`，以及通用的 `pointcloud2`。用通用的 `pointcloud2` 时要指定这几项：
    - `time_field`：时间字段名
    - `time_unit`：s | ms | us | ns | auto
    - `time_ref`：relative（相对 header）| absolute（绝对时间）
    - `ring_field`、`intensity_field`：线号和强度的字段名

    `lidar.time_offset`（秒）会加到雷达时间上，用来补偿雷达和 IMU 之间的时间差。
  - `imu`：`acc_unit` 取 m/s2 或 g，`gyro_unit` 取 rad/s 或 deg/s，`time_offset` 单位是秒。
  - `gt.format`：真值格式。
    - `tum`
    - `csv`：用 `columns: {t, x, y, z, qx, qy, qz, qw}` 指定列名，`time_unit` 指定时间单位
    - `kitti`：另外用 `times: 文件` 指定时间文件

    `imu_T_gt: {t: [3], r: [9 个数，行优先] 或 q: [x, y, z, w]}` 把真值位姿换算到 IMU 坐标系。`time_offset`（秒）平移真值的时间。
- **`sensor`**：给算法适配器用的值，包括外参、IMU 饱和阈值和最近距离盲区。
- **`sequences`**：每条序列的 `bag`（ROS1 的 `.bag` 文件，或 ROS2 包的目录）和可选的 `gt` 真值文件。

转换后，`meta.yaml` 里的话题名、加速度单位、线数和扫描频率会自动合并进 `sensor`，不用手写。改了 `source` 之后，重新运行 `liobench convert`，它会检测到配置变化并重新转换。如果只是后来加上了真值文件，它只转换真值，不重新转换数据包。

## 示例：Velodyne + IMU 的 ROS1 数据集

```yaml
output: ~/datasets/liobench/mydata
source:
  lidar: {topic: /velodyne_points, decoder: velodyne}
  imu: {topic: /imu/data, acc_unit: m/s2}
  gt: {format: csv, columns: {t: time, x: x, y: y, z: z, qx: qx, qy: qy, qz: qz, qw: qw}, time_unit: ns}
sensor:
  extrinsic_t: [0, 0, 0.1]              # 雷达在 IMU 坐标系下的位置
  extrinsic_r: [1, 0, 0, 0, 1, 0, 0, 0, 1]
  satu_acc: 150.0                       # m/s²
  satu_gyro: 35.0                       # rad/s
  blind: 1.0                            # m
sequences:
  seq01: {bag: ~/data/seq01.bag, gt: ~/data/seq01_gt.csv}
```
