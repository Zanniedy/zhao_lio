# zhao_lio
my first lio use it to recongise a girl

zhao_lio 是一个 LiDAR-惯性里程计，基于 ROS2 Humble 和 C++17，组合了两套已有实现：

- **前端和地图**取自 Super-LIO：降采样时每个体素保留离中心最近的原始点（`VoxelGridClosest`），地图用紧凑的八叉体素 `OctVoxMap`。
- **滤波**取自 Point-LIO / small_point_lio：30 维 ESKF，角速度和加速度作为状态，IMU 作为观测，每个 LiDAR 点做一次 rank-1 更新。

更新方式可以切换，同一个可执行文件就能在同一份数据上做对比：

| `update_strategy` | 做法 |
|---|---|
| `pointwise` | 每个点串行执行：预测 → 近邻搜索 → 平面拟合 → 更新 |
| `pointwise_prematch`（默认） | 先用 IMU 预测位姿，TBB 并行算出整帧点的平面；再串行逐点更新，残差过大的点重新搜索 |

## 目前的表现

NarrowWide Tracked-01（Livox MID-70 + VN-100，621 s，有真值），和另外 5 套开源 LIO 用同样的条件对比，每个配置跑 1 次：

| 算法 | ATE RMSE | 延迟 P95 | CPU | 内存 |
|---|---:|---:|---:|---:|
| zhao_lio（pointwise） | 0.119 m | 6.1 ms | 4% | 115 MB |
| zhao_lio（prematch） | 0.122 m | 5.9 ms | 5% | 117 MB |
| genz-lio | 0.125 m | 24.5 ms | 341% | 1171 MB |
| Super-LIO | 0.154 m | 6.1 ms | 13% | 143 MB |
| Point-LIO | 0.191 m | 11.6 ms | 8% | 170 MB |
| Batch-LIO | 0.192 m | 8.7 ms | 140% | 174 MB |

完整结果和实验条件见 [`liobench/docs/results/baseline-2026-10-29/`](liobench/docs/results/baseline-2026-10-29/README.md)。

## 目录

```
src/zhao_lio_core/     纯 C++17 库，只依赖 Eigen 和 TBB，不依赖 ROS 和 PCL
src/zhao_lio_ros2/     实时节点 zhao_lio_node、离线回放 zhao_lio_replay、配置、launch、rviz
docs/                  设计文档、实现计划、交接文档 HANDOFF.md
liobench/              测试平台：转换数据集、和其他算法对比、开发循环（编译 → 测试 → 对比 → 验收）
scripts/compare.sh     用 evo 对比两条轨迹
```

## 编译

依赖：Ubuntu 22.04、ROS2 Humble、Eigen 3.4、oneTBB。可选依赖 `livox_ros_driver2`；没有它也能编译，但只支持 PointCloud2 输入。

```bash
cd ~/cs_project/liop/zhao_lio
source /opt/ros/humble/setup.bash
source ~/cs_project/nav/cs/install/setup.bash        # livox_ros_driver2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select zhao_lio_core && colcon test-result --verbose
```

如果 miniconda 在 PATH 上，CMake 可能会链接到 conda 的库。`zhao_lio_ros2/CMakeLists.txt` 已经固定使用系统的 yaml-cpp。

## 运行

```bash
source install/setup.bash
# 实时节点 + RViz2；config 可选 mid360 | avia | tracked
ros2 launch zhao_lio_ros2 zhao_lio.launch.py config:=tracked use_sim_time:=true
# 离线回放：直接读包，结束时打印各阶段的平均耗时
ros2 run zhao_lio_ros2 zhao_lio_replay --ros-args \
  --params-file src/zhao_lio_ros2/config/tracked.yaml -p bag:=<包路径> -p traj_path:=out.tum
```

输出话题：`/zhao_lio/odom`、`/zhao_lio/path`、`/zhao_lio/cloud_world`，以及 TF `odom → body`。

## 改代码、测效果

```bash
cd liobench && source /opt/ros/humble/setup.bash
.venv/bin/liobench dev dev/zhao_lio.yaml --note "这一版改了什么"
```

这条命令会编译、跑测试、和上一版对比，再按门槛判定这一版通过还是不通过。第一次使用前要先按 [`liobench/README.md`](liobench/README.md) 装好 Python 环境，并编译参考算法。教程见 [`liobench/docs/TUTORIAL.md`](liobench/docs/TUTORIAL.md)。

## 许可证

`OctVoxMap` 移植自 Super-LIO（GPLv3），所以 zhao_lio 整体按 GPLv3 发布，见 `LICENSE`。`src/zhao_lio_core/third_party/tsl/` 是 Tessil 的 robin-map 和 Martin Ankerl 的 robin_hood（都是 MIT 许可证）。
