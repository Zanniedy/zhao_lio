# 输入预处理与 Dynablox 动态点云分支

2026-11-17 从 `robot/zhao_lio_x5` 迁回本仓库。保留此前检验过的输入、外参、
官方检测阈值和二维显示，不修改前端 ESKF、地图更新、回环或位姿图算法。

## 数据链路

```text
/input/lidar -> 基础前处理 -> /zhao_lio_x5/points/optimized -> Zhao-LIO
       |                                                     |
       +-------------------> Dynablox <---- LIO TF -----------+
                                 |
                   /dynablox/static_map -> /map2d
```

Dynablox 是依赖 LIO 位姿的独立分支，不反向接入当前 LIO 输入；原 `/zhao_lio/map`
不会因此回删历史人体点。保留 `/zhao_lio_x5/*` 话题名称以兼容已有配置。
此次不迁移另一工作区的 PCD 快照时间戳写出改动。

新增包：
- `zhao_lio_pointcloud_preprocessor`：基础过滤、原始点字段保留、旧动态过滤及接口测试。
- `dynablox_ros2`：官方聚类、跟踪、ever-free 和 Voxblox 核心快照，ROS2/MID70适配。
- `map_3d_to_2d`：之前使用的高度投影及 RViz Map Z Range 面板。

原 ROS2 包仅新增 X5 配置、输入契约测试和 `publish_tf` 开关；原核心/后端文件不覆盖。
第三方版本、许可证与移植边界见 `src/dynablox_ros2/SOURCE.md`。

## 独立构建

使用新终端，仅 source 系统 ROS，不叠加 robot 工作区的同名包。
依赖包括 Eigen、TBB、PCL、nanoflann、glog、protobuf、Qt5 和 RViz2；现有三方许可证保留。

```bash
cd /home/lsy/cs_project/liop/zhao_lio
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=3 colcon --log-base log_preprocessing build \
  --base-paths src --packages-up-to dynablox_ros2 \
  --build-base build_preprocessing --install-base install_preprocessing \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt
source install_preprocessing/local_setup.bash
```

这组构建目录不覆盖原 `build/`、`install/`。未安装 Livox CustomMsg 驱动时，
本次标准 PointCloud2 输入仍可构建和运行。

## 回放与检查

```bash
bash scripts/replay_preprocessed.sh /path/to/rosbag2
# 仅数据链路，不开窗口：
bash scripts/replay_preprocessed.sh /path/to/rosbag2 rviz:=false
# 关闭二维投影：
bash scripts/replay_preprocessed.sh /path/to/rosbag2 map2d:=false
```

脚本默认本机 domain184、1倍速、不循环，不启用导航或底盘。如显式设置 ROS_DOMAIN_ID，
脚本沿用该值；不要选已有相同话题发布者的域。Ctrl+C 停止，锁文件防重复启动。
bag 路径为必填参数，不硬编码外部实验目录。直接 launch 也支持：

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=184 ros2 launch dynablox_ros2 replay.launch.py \
  bag:=/path/to/rosbag2 rate:=1.0 rviz:=true map2d:=true
```

可用 `config`、`lio_config`、`optimizer_config`、`map2d_config` 覆盖对应 YAML。
该入口固定关闭旧 `dynamic_filter`，避免叠加两套动态删除策略；保留基础过滤：
距离0.3～100m、体素0.10m、base_link高度−0.2～4.0m、强度≥5。
单独启动 `zhao_lio_pointcloud_preprocessor replay_mapping.launch.py` 则仍默认启用旧动态过滤。

输入 x/y/z 为 float32，timestamp 为 float64 绝对秒，IMU加速度为 m/s²。
X5配置 `acc_norm=9.81`、body_frame=imu_link，沿用此前外参，不是新标定结果。
回放会发布配置中的 imu_link→mid70_link 静态外参，以及模型约定的零偏移
imu_link→base_link。外部已有对应TF时，用 `publish_calibrated_tf:=false`、
`publish_base_tf:=false` 分别关闭，避免重复发布；换设备前务必换成正确配置。

RViz绿色=当前保留点，红色=官方动态检测，黄色=未分类。二维黑色=占用，灰色=未知，
初始高度0.63～1.53m、ground_z=0，可在 Map Z Range 调整；不是自动测得地面。
二维输入为非动态 TSDF 零面附近的体素中心（0.20m），不是原始点记录。
0.05m二维格网可能稀疏，未填洞、未推断自由空间，不是直接可导航的地图。

## 验证

```bash
source /opt/ros/humble/setup.bash
source install_preprocessing/local_setup.bash
colcon --log-base log_preprocessing test --build-base build_preprocessing \
  --packages-select zhao_lio_pointcloud_preprocessor dynablox_ros2 zhao_lio_ros2 \
  --event-handlers console_direct+
colcon test-result --test-result-base build_preprocessing --verbose
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=185 /usr/bin/python3 -m pytest -q \
  src/zhao_lio_pointcloud_preprocessor/test/test_ros_node.py
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=185 /usr/bin/python3 -m pytest -q \
  src/dynablox_ros2/test/test_ros_node.py
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=185 /usr/bin/python3 -m pytest -q \
  src/map_3d_to_2d/test/test_projection.py
(cd src/dynablox_ros2 && sha256sum --check SOURCE.sha256)
```

合成与接口测试验证字段保留、异常旁路和调用链，不代表真实人体过滤正确率。
历史上在 robot 工作区得到的运行统计不等于本仓库新构建后的验收结果。

### 本仓库迁入验证（2026-11-17）

- 独立构建6个包成功，未引用 robot 的安装目录；第三方108个归档文件校验通过。
- 36项C++用例通过：基础过滤9、旧动态过滤13、整簇跟踪6、Dynablox6、输入契约2。
- ROS接口测试6项通过：前处理3、Dynablox2、二维投影1。
- domain186无界面实包检查28秒（含启动等待）：216次里程计、214次正常检测、
  49次TSDF和二维地图输出；末次TSDF为4512点，二维322个占用格、0个自由格。
- 核验旧动态过滤关闭，LIO输入为 optimized，检测输入为原始 lidar，二维输入为
  Dynablox static_map。结束后只停止本次检查，不启动或操作底盘。
- 本地详细运行记录在 `log_preprocessing/migration-verification/`（不入Git）。
  这是接口和运行验证，不是GUI截图验收，也不是人体检测精度评测。
- 验证保留了工作区已有的核心/后端改动；它们没有纳入此次迁移提交。
