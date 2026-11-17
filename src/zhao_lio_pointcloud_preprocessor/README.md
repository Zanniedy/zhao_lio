# X5 点云前处理与短时间动态过滤

已迁入 Zhao-LIO 主工作区；统一入口见 `docs/preprocessing.md`。
下文历史回放统计来自 robot 工作区。Dynablox入口关闭本包的旧动态过滤，
只复用基础过滤；本包单独回放仍默认开启旧动态过滤。

输入链路：`/input/lidar` → 距离/高度区间/最低强度过滤 → 体素过滤 → IMU 辅助短窗动态过滤 →
`/zhao_lio_x5/points/optimized` → LIO。原 `/input/imu` 仍直接供 LIO 使用。

## 高度区间与强度（简单阈值）

不做分层、地面拟合或分类，只对每个点直接判断：

- 查询点云 header 时刻的 `base_link <- 点云frame` TF，以变换后 Z 判断高度。
  `z_base < min_z` 或 `z_base > max_z` 删除；当前配置 `min_z: -0.20`、`max_z: 4.0`，
  即只保留 base_link 下方 20 cm 到上方 4 m 的点（边界留 1 微米浮点容差）。
  上限不是 MID70 上方 1.5 m，以最终确认的 base_link 4 m 为准。
  这是相对底盘坐标系的高度，不是世界坐标 Z，也不是自动地面高度。
- `intensity < min_intensity` 删除；默认 `min_intensity: 5.0`，使用原始强度数值，不归一化。
  强度为 NaN/Inf 时也删除，等于阈值保留。
- 筛选在体素选择之前执行，避免低强度点占据体素后丢掉同格有效点。
  两个判断均只删点，不修改保留点的坐标、字段或逐点时间戳。

`config/pointcloud_optimizer.yaml` 中配置 `height_filter_enabled`、`height_frame`、
`min_z`、`max_z`、`min_intensity`。关闭高度过滤设 `height_filter_enabled: false`；关闭强度过滤设
`min_intensity: 0.0`。重启节点生效。缺少高度 TF 或 intensity 字段时，仅跳过缺失的那项过滤，
日志明确警告，不把雷达 Z 冒充 base_link Z。异常字段布局和非法高度区间仍拒绝处理。

`/zhao_lio_x5/point_filter/status` 给出输入/保留点数、`low_height`、`high_height`、`low_intensity` 和两项
`*_applied` 状态。计数按高度、强度顺序统计，一个点只计入首次命中的删除原因。

回放默认补充 `imu_link -> base_link` 零偏移静态 TF，依据当前
`photo_nav_robot.urdf.xacro` 中的 `imu_joint`，**只是模型约定，不是新的实测外参**。
这避免给 LIO 已发布的 `imu_link` 再挂第二个父节点。实际设备应提供正确的底盘 TF；
已有底盘 TF 时用 `publish_base_tf:=false` 关闭回放补充，不能重复发布或把零偏移当作标定结果。

高度/强度版本验证：17 项 C++ 测试、2 项 ROS 集成测试通过。原包约 69 秒回放中，
693 帧均应用了两个阈值，收到 682 次里程计、34 次地图。逐点检查的 621 帧中，
最低 `z_base` 为 -0.0999988 m、最低强度为 5，均满足配置。累计按高度删除 1374208 点、
按强度删除 1408144 点（体素下采样之前计数，重复观测也计入，不是独立地图点数量）。

## 算法和边界

1. 按 IMU 消息时间查询 TF `mid70_link <- IMU frame`，把角速度旋转到雷达坐标系，
   用相邻 IMU 的平均角速度积分，插值到每个点的绝对 `timestamp`。
2. **只在匹配副本**中做逐点旋转补偿；以前一对点云的平移速度近似补偿扫描内平移。
   用固定 IMU 相对旋转、最近邻和坐标中位数迭代估计两帧平移，不等待当前 LIO 位姿。
3. 默认 YAML 开启 `dynamic.cluster_tracking_enabled`：在 TF 变换后的 base_link 高度上，
   将高于 `cluster.ground_z + cluster.ground_clearance` 的点作欧氏连通聚类，
   防止地面把目标与墙连成一体；邻近簇的近地非地面点附回该簇。地面参考高度由配置指定，
   不做复杂分层或自动地面拟合。太小或跨度超过上限的簇保留，不作为运动目标。
4. 使用自运动补偿后的簇中心、尺寸、预测位置进行跨帧互相最佳关联；歧义关联保留。
   比较约 0.30 秒前同一簇，用双向近邻及簇间平移拟合确认位移、速度、形状重合与误差改善。
   连续两次方向一致的运动证据通过后，**删除当前整个点簇**，包括仍与旧位置重合的点，
   而不是只删人体边缘的不匹配点。停止运动后，随历史窗口更新恢复保留。
5. IMU/TF 缺失、时间倒退、点数太少、配准不可信、删除比例过大时，保留当前点云，
   重置或跳过相应历史。等待 IMU 扫描结束时间有超时，队列最多五帧。

关闭 `dynamic.cluster_tracking_enabled` 可退回旧的逐点模式：自由空间判据、局部候选聚集、
连续候选确认。`dynamic.static_distance`、`angular_bin_deg`、`min_visibility_bins`、
`free_space_margin`、`cluster_radius`、`min_cluster_points`、`max_object_speed`、
`confirmation_frames` 只控制旧模式，不用于整簇运动判定。C++ 类默认仍关闭簇模式以兼容旧调用。

保留/删除输出均逐字节复制原点记录，保留原坐标、frame、header、字段、逐点时间和顺序。
不会把已去畸变坐标再次送给 LIO，也不会重复应用已标定外参。

这是**几何点簇运动过滤**，不是语义识别人车或保证删除所有运动物体。
人靠墙或多人相距小于聚类半径时可能粘连；慢速、远距离稀疏点、遮挡及关联歧义可能保留。
采样变化或错误关联仍可能误删；重复结构、大面积动态场景也不适合仅依靠该方法。
默认至少约 0.4 秒历史才能确认运动，确认前的点仍送入 LIO；**不会回删已写入累计地图的旧点**。
因此累计地图中的人影不等于当前点云仍未过滤。静止的人也不是此算法的删除目标。
目前不积分加速度推算绝对平移：没有可靠初速度、重力和偏置时会误把自运动判为动态。
TF 的平移/杆臂效应通过点云平移配准和恒速近似处理，不宣称完成了加速度杆臂补偿。
没有修改原来的地面姿态标定，亦没有重新标定陀螺仪偏置。

## 配置

编辑 `config/pointcloud_optimizer.yaml`，重启节点生效：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `dynamic.enabled` | true | 关闭动态过滤，仍保留高度/强度/距离/体素过滤 |
| `dynamic.cluster_tracking_enabled` | true | YAML 默认整簇跟踪，false 退回逐点模式 |
| `dynamic.min_dt` / `dynamic.max_dt` | 0.04 / 0.30 s | 帧间匹配时间窗口 |
| `dynamic.max_scan_duration` | 0.15 s | 逐点时间允许的扫描长度 |
| `dynamic.max_imu_gap` | 0.03 s | 大于此值重置 IMU/匹配历史 |
| `dynamic.imu_wait_timeout` | 0.20 s | 墙钟等待 IMU 上限，暂停播放也会超时旁路 |
| `dynamic.static_distance` | 0.12 m | 动态候选的最近邻距离下限 |
| `dynamic.registration_inlier_distance` | 0.30 m | 单独用于评估配准质量，不再共用候选阈值 |
| `dynamic.correspondence_distance` | 0.60 m | 平移迭代最大对应距离 |
| `dynamic.min_inlier_ratio` | 0.60 | 配准最小内点比例 |
| `dynamic.max_registration_rmse` | 0.18 m | 内点配准误差上限 |
| `dynamic.max_ego_speed` | 3.0 m/s | 估计自运动速度上限 |
| `dynamic.angular_bin_deg` | 1.0° | 可见性角度网格 |
| `dynamic.min_visibility_bins` | 3 | 邻域最少有观测的角度格数 |
| `dynamic.free_space_margin` | 0.15 m | 候选与原观测表面的最小距离差 |
| `dynamic.cluster_radius` / `dynamic.min_cluster_points` | 0.35 m / 3 | 候选局部点簇要求 |
| `dynamic.max_object_speed` | 6.0 m/s | 两帧候选关联最大距离 / dt |
| `dynamic.confirmation_frames` | 2 | 连续候选确认次数，包含首帧候选 |
| `dynamic.max_removed_ratio` | 0.40 | 超过则整帧不作动态删除；提高上限以容纳近处完整目标 |
| `dynamic.min_points` | 100 | 最小可匹配点数 |
| `cluster.tolerance` / `cluster.min_points` | 0.35 m / 8 | 欧氏聚类邻接距离及最少点数 |
| `cluster.ground_z` / `cluster.ground_clearance` | 0.0 / 0.12 m | base_link 地面参考及聚类离地高度 |
| `cluster.max_extent` | 4.0 m | 任一轴跨度超过此值不作目标 |
| `cluster.association_distance` / `cluster.association_ambiguity` | 0.70 / 0.12 m | 关联基础距离及最佳/次佳分数差 |
| `cluster.motion_window` / `cluster.track_timeout` | 0.30 / 0.40 s | 运动比较窗口及失联清除时间 |
| `cluster.min_displacement` / `cluster.min_speed` | 0.10 m / 0.25 m/s | 整簇运动下限，须同时满足 |
| `cluster.max_speed` | 6.0 m/s | 整簇运动速度上限 |
| `cluster.fit_distance` / `cluster.max_fit_rmse` | 0.30 / 0.12 m | 簇间对应距离及拟合误差上限 |
| `cluster.min_fit_improvement` / `cluster.min_overlap` | 0.025 m / 0.65 | 运动拟合相对静止假设的改善及双向重合率 |
| `cluster.confirmation_frames` | 2 | 连续运动确认次数 |

单独启动默认要求外部 TF；缺失时会旁路，不会假设 IMU 和雷达轴一致。
`replay_mapping.launch.py` 则从**与 LIO 相同的** `mid70_x5_visual.yaml` 读取
`extrinsic_R/T`，发布 `body_frame -> mid70_link` 静态 TF。外部已有相同 TF 时，
应关闭 `publish_calibrated_tf`，不要重复发布不同外参。
回放启动参数是 `publish_calibrated_tf:=false`；单独关闭动态过滤可用 `dynamic_filter:=false`。
要完全恢复最初的距离/体素前处理，还需在配置中关闭高度和强度过滤。
`timestamp` 支持 FLOAT64 绝对秒或绝对纳秒，不能用相对 offset 冒充。

## 回放与观察

```bash
source /opt/ros/humble/setup.bash
source install_preprocessing/local_setup.bash
ros2 launch zhao_lio_pointcloud_preprocessor replay_mapping.launch.py \
  bag:=/path/to/rosbag2
```

默认 RViz 聚焦过滤：绿色保留点、红色删除点、点簇框和 ID/速度/点数/确认次数。
红框表示整簇已删，绿框表示静止假设下保留，黄框表示预热、待确认或形状不可信。
`kept:limit` 表示已判断运动，但触发整帧删除比例保护。MarkerArray 话题为
`/zhao_lio_x5/dynamic_filter/clusters`。红点为疑似动态点，不代表已经有真实标注验证。
灰色原始点、世界系扫描和累计地图默认关闭，避免旧点遮盖过滤效果；需要看建图时，
勾选 `Accumulated map` 即可，后台 LIO 建图和轨迹输出并未关闭。
`/zhao_lio_x5/dynamic_filter/status` 发布每帧原因、保留/删除/候选数量、dt、内点比例、RMSE、
当前簇数 `tracks`、运动簇数 `moving_clusters`、不满足聚类条件的分量数 `rejected_components`：

```bash
ros2 topic echo /zhao_lio_x5/dynamic_filter/status
```

`matched` 表示通过匹配质量检查，允许零删除；`warmup` 表示初始化；
`imu_wait_timeout`、`imu_not_bracketed`、`registration_unreliable`、`removal_limit` 等表示保护性保留。
不要为了强行出现红点而降低全部保护阈值。

本次提高灵敏度时，将配准质量距离与候选距离分开，避免因为 MID70 相邻扫描采样不重合，
多数帧先被配准门槛旁路。自由空间差从 0.40 m 改为 0.15 m，点簇最小点数从 5 改为 3；
仍保留两次候选确认、配准质量与单帧最多删除 15% 的保护。状态增加 `unmatched`、
`clustered`、`confirmed`、`converged`，分别定位未匹配、聚簇、确认和迭代收敛阶段。

4 m 上限与新动态参数验证：20 项 C++ 测试、2 项 ROS 集成测试通过。按当时 YAML 中
`min_z=-0.2`、`max_z=4.0`、`min_intensity=5.0` 回放约 69 秒，691 帧状态中有 664 帧
通过配准、26 帧保护性旁路、1 帧预热；236 帧实际删除累计 1780 个疑似动态点。
高度上限累计删除 347 点，620 帧逐点检查满足配置范围（含 1 微米浮点容差），
最高保留点的 base_link 坐标 Z 为 3.9995689 m，LIO 输出 681 次里程计及 34 次地图。
开头 20 秒也有 865 个删除点：机器人静止并不证明场景无运动，但无动态标注时不能区分
真实运动、采样变化与误删。因此这些计数仅验证过滤已执行，不代表动态识别准确率。

## 构建与测试

依赖 ROS Humble、Eigen3、`libnanoflann-dev`，不依赖当前帧 LIO 输出。
迁移后的统一构建和 Dynablox 入口见仓库 `docs/preprocessing.md`。
以下构建本包及其运行依赖，不覆盖原 install：

```bash
cd /home/lsy/cs_project/liop/zhao_lio
source /opt/ros/humble/setup.bash
colcon --log-base log_preprocessing build --base-paths src \
  --packages-up-to zhao_lio_pointcloud_preprocessor \
  --build-base build_preprocessing --install-base install_preprocessing \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
ctest --test-dir build_preprocessing/zhao_lio_pointcloud_preprocessor --output-on-failure
```

测试覆盖静态场景旋转/平移、逐点时间、移动点簇确认、背景显露、删除比例保护、
IMU 缺失/间断、时间倒退、原点记录保留、纳秒时间和异常布局。
整簇版本增加横向运动时全部 90 个模拟人体点删除（含重叠部分与近地附加点）、
静态采样扰动、自运动下双目标独立跟踪、目标出现/停止、时间重置以及非零高度 TF 测试。

2026-11-17 整簇版本验证：28 项 C++ 测试与 3 项 ROS 集成测试通过；原包约 69 秒回放中，
688 帧动态状态包含 662 帧匹配成功、24 帧配准保护、1 帧预热和1帧 IMU 未包围。
293 帧执行删除，累计删除 12992 个点观测，517 次运动簇观测（不是 517 个人或独立目标）。
收到 679 次里程计、34 次地图；620 帧输出逐点通过 base_link 高度和强度检查。
开头 20 秒也删除了 8213 点；机器人静止不等于环境没有运动，但此结果同样不能排除
采样变化或关联造成的误删。没有真人标注，不能以这些计数宣称真人召回率或“所有人已滤净”。

ROS 集成测试（需要允许 DDS 通信；使用独立 domain，避免影响正在运行的回放）：

```bash
source install_preprocessing/local_setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=185 ROS_LOG_DIR=/tmp/x5_dynamic_test_logs /usr/bin/python3 -m pytest -q \
  -p no:cacheprovider src/zhao_lio_pointcloud_preprocessor/test/test_ros_node.py
```

此测试使用非单位外参验证旋转方向、逐点 IMU 等待、IMU 超时和 TF 缺失旁路。

### 2026-11-17 验证记录

12 项 C++ 测试和 1 项 ROS 集成测试通过。原始 bag 前约 109 秒以 2 倍速回放，
接收到 1090 帧优化点云、1081 次 LIO 里程计和 54 次地图，末次地图 42903 点。
363 帧通过匹配检查、727 帧保护性旁路、1 帧预热；累计 474 个候选点，最终删除 0 点。
这些数字仅说明该片段的链路与保护机制工作，不能据此断言真实场景动态识别准确率。
动态点的实际删除、连续确认和静态点保留由合成测试验证；真实人车效果仍需有标注或现场对照。
