# map_3d_to_2d

Originally adapted from `pcdtomap/src/ROS2_pcd_to_occupancy_grid`, then migrated from the robot workspace. It converts a `sensor_msgs/msg/PointCloud2` map into a 2D `nav_msgs/msg/OccupancyGrid` and exposes a two-slider RViz panel for the active Z range. See `docs/preprocessing.md` for Dynablox input and the complete build.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon --log-base log_map_3d_to_2d build \
  --base-paths src/map_3d_to_2d \
  --build-base build_map_3d_to_2d \
  --install-base install_map_3d_to_2d \
  --packages-select map_3d_to_2d \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install_map_3d_to_2d/setup.bash
```

## Run with the Zhao-LIO map

```bash
ros2 launch map_3d_to_2d mapper.launch.py rviz:=true
```

## 高度配置与投影规则

编辑 `config/params.yaml`，启动时用 `config:=绝对路径` 指定该文件即可，无需重新编译。

```yaml
ground_z: 0.0
z_min: 0.10
z_max: 1.50
resolution: 0.05
occupancy_threshold: 1
max_range: 0.0
```

`ground_z` 是输入点云坐标系中的地面 Z；默认 0 是可调整的参考值，不是自动测量结果。
`z_min/z_max` 单位为米，均为相对地面的高度，而非原先的绝对 Z。
所有满足 `ground_z + z_min <= point.z <= ground_z + z_max` 的有限点，
都按 XY 投影到栅格；默认一个点即标为占用（100），其余为未知（-1），不把未知当作可通行区域。
`max_range: 0.0` 关闭原先的 80m 截断，`occupancy_threshold: 1` 不过滤单点栅格。
如果要把地面也投影为占用，可将 `z_min` 设为 0 或略小于 0；导航障碍层通常应排除地面。

输入应为累积地图 `/zhao_lio/map`，节点每次重投影最新地图，不叠加重复计数。
空高度区间会发布全未知栅格，清除旧障碍；改变高度后即使回放暂停也能重投影缓存地图。
格网原点按分辨率对齐，最大限制为 1600 万格，过大时需调大分辨率。
配置 `frame_id` 只重命名坐标系，不执行 TF 变换；通常保持空字符串沿用输入帧。

## 一键建图回放与 RViz

```bash
source /opt/ros/humble/setup.bash
source install_preprocessing/local_setup.bash
ros2 launch map_3d_to_2d replay_grid.launch.py \
  bag:=/path/to/rosbag2 \
  config:="$PWD/src/map_3d_to_2d/config/params.yaml"
```

启动前关闭同话题的旧回放、mapper 和 RViz，避免多个发布者互相干扰。
此入口依赖已安装的 `zhao_lio_pointcloud_preprocessor` 和 `zhao_lio_ros2`；
它复用现有的外参与建图配置，不重新标定、不修改原始 bag，也不自动保存地图文件。
`rate:=1.0` 为原速，整个链路使用仿真时钟。
RViz 的 `Map Z Range` 显示配置地面、两个相对高度和点数/栅格状态；
面板启动只读取 mapper 状态，不覆盖 YAML，滑块调整成对提交并立即更新。
面板调整仅作用于本次运行，永久保存请修改 YAML。
`LIO map` 可勾选以对照三维点云，二维显示为 `/map2d`，Fixed Frame 为 `lio_odom`。

默认显示使用 `/map2d/markers`，按实际栅格逐格绘制：黑色占用、灰色未知。
它由同一份 OccupancyGrid 生成，不膨胀、不改写地图数据。
本机默认 Map 纹理显示曾出现 `active samplers with a different type` 报错和绿色显示，
因此默认不加载原生 Map 显示，避免依赖该纹理着色器。
需要原生 Map 显示时可手动 Add → Map 并指定 `/map2d`；导航仍订阅标准 `/map2d`，不订阅 markers。

接口：

- Input: `/zhao_lio/map`
- Output: `/map2d`
- `/map_3d_to_2d/set_range`: `geometry_msgs/Vector3`，x/y 为两个相对高度，z 忽略。
- `/map_3d_to_2d/range`: 实际范围状态，x/y 为高度，z 为配置地面高度。
- `/map_3d_to_2d/status`: `std_msgs/String`，点数、命中点数和栅格状态。
- 兼容旧 Float32 控制话题 `/map_3d_to_2d/z_min` 和 `/map_3d_to_2d/z_max`，数值现在为相对高度。

状态和地图使用 transient-local QoS，晚启动的 RViz 也可读取最后结果。输入点云不被修改。

## 回归验证

```bash
source /opt/ros/humble/setup.bash
source install_preprocessing/local_setup.bash
ROS_DOMAIN_ID=177 /usr/bin/python3 -m pytest -q \
  src/map_3d_to_2d/test/test_projection.py
```

If the shell is inside a Conda environment, rebuild this package after changing
the CMake file so the RViz plugin uses ROS Humble's system `yaml-cpp` library.
