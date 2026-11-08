# zhao_lio_backend

zhao_lio 的位姿图后端，参考 z-localization 后端的设计思路重新实现（没有复制它的 GPL-2.0-only 源码）。
纯 C++17，只依赖 Eigen、TBB 和 zhao_lio_core，不依赖 ROS、PCL、GTSAM。

前端照常在 odom 系里运行，后端维护 `T_map_odom`，输出 `T_map_body = T_map_odom * T_odom_body`。

## 模块

| 文件 | 内容 |
|---|---|
| `se3.h` | SE(3) Exp/Log、伴随、右雅可比逆（右扰动，切向量顺序 [ρ; φ]） |
| `pose_graph` | Eigen 手写位姿图：LM + `SimplicialLDLT`；先验/相对/重力方向因子；平方根信息矩阵可以降秩；边缘协方差、NIS；最老节点 Schur 边缘化成线性先验 |
| `registration` | 点到面 ICP（粗 1.2 m / 细 0.4 m 两层 OctVoxMap，PCA 拟合平面并剔除近似共线的邻居）。信息矩阵按可观方向投影：旋转按 10 m/rad 换算后做特征分解，只保留标准差 < 0.3 m 且不低于最大特征值 0.2% 的方向 |
| `scan_context` | Scan Context 描述子（20 环 × 60 扇区，最大高度），ring key 预筛 + 列平移余弦距离，给出 yaw |
| `backend` | 关键帧、回环、先验图定位、状态机、地图包读写 |

## 模式

- **mapping**：里程计因子 + 重力因子 + 回环因子（距离近邻候选和 SC 候选 → 子图 ICP → 要求 6 维可观 → NIS 检验）。保留全部关键帧，不做滑窗。
- **localization**：加载地图包，每个关键帧和先验图做点到面配准；NIS 通过就加入可观方向的绝对位姿因子。滑窗保留 100 个关键帧，更早的边缘化。

状态机：`INITIALIZING → NORMAL ⇄ OBS_DEGRADED → SUSPECT → LOST`
- 观测与预测冲突（NIS 超限，或者内点比例低于阈值的一半）记一次冲突；连续 3 次冲突进入 SUSPECT。
- SUSPECT：保留前端，在预测位置做 12 个朝向的扫描，再加上 SC 的前 5 个候选，逐个配准重新找 `T_map_odom`。如果最好和次好的候选差距不到 0.05 视为有歧义，拒绝本次结果。连续失败 5 次进入 LOST。
- LOST：请求前端重置（`ZhaoLio::reset()`，`session()` 加 1），衔接新 odom 系时假设机器人没有移动，然后重定位。
- odom 速度超过 `max_speed` 直接进入 LOST 并重置前端。
- SUSPECT 和 LOST 期间不输出位姿，TF 保持在最后一个可信值。

地图包是一个目录，包含 `keyframes.txt`（位姿）、`clouds.bin`（关键帧点云，body 系，float32）和 `map.pcd`（0.1 m 体素，供 CloudCompare 查看）。

## 使用

```bash
# 离线回放：建图，保存地图包
ros2 run zhao_lio_ros2 zhao_lio_replay --ros-args --params-file <前端.yaml> -p bag:=<bag> \
  -p backend.mode:=mapping -p backend.save_dir:=<地图包目录> \
  -p backend.traj_path:=rt.tum -p backend.opt_traj_path:=opt.tum
# 离线回放：先验图定位（start_offset 跳过 bag 开头，用来测试全局重定位）
ros2 run zhao_lio_ros2 zhao_lio_replay --ros-args --params-file <前端.yaml> -p bag:=<bag> \
  -p backend.mode:=localization -p backend.map_path:=<地图包目录> -p start_offset:=60.0
# 实时节点：后端在独立线程里运行
ros2 run zhao_lio_ros2 zhao_lio_node --ros-args --params-file <前端.yaml> \
  --params-file src/zhao_lio_ros2/config/backend.yaml
```

实时节点额外发布 TF `map → odom`、`/zhao_lio/backend/pose`、`/zhao_lio/backend/path`、`/zhao_lio/backend/state`，
并提供 `/zhao_lio/backend/save_map` 服务。

回放的测试开关：`test_perturb_time`、`test_perturb:=[dx, dy, yaw_deg]`、`test_perturb_ramp`（秒，0 表示阶跃）。
打开后，从指定时刻起给送入后端的里程计叠加一个偏移，用来验证状态机。

## 测试结果（2026-11-01，每个配置跑 1 次）

TBB 并行会让每次运行有少量差异，下面的数字在几个百分点内波动。

**NarrowWide tracked-01**（MID-70，621 s，262 m，有真值）

| 建图 | ATE RMSE | ATE max | RPE 200 m |
|---|---:|---:|---:|
| 纯 LIO | 0.109 m | 0.54 m | 1.06 m |
| 后端实时输出 | 0.119 m | 0.53 m | 1.06 m |
| 后端优化后全轨迹 | 0.124 m | 0.43 m | 0.94 m |

接受的回环 29 个。用真值核对过，回环测量和 LIO 一样准或者更准。这条序列上 LIO 本身漂移很小（首尾误差约 0.2 m），
所以回环只降低了最大误差和长距离 RPE，ATE RMSE 没有改善。

| 先验图定位（用 tracked-01 的地图，从 200 s 开始回放） | map 系误差 RMSE | p95 | max |
|---|---:|---:|---:|
| 纯 LIO（只用首次重定位的 TF） | 0.22–0.41 m | 0.40–0.92 m | 0.69–1.06 m |
| 后端 | 0.12 m | 0.27 m | 0.47 m |
| 后端 + 阶跃注入 30 m / 90° | 0.12 m | 0.26 m | 0.48 m |
| 后端 + 渐变注入 3.6 m / 25°（10 s 内） | 0.23 m | 0.31 m | 2.40 m |

map 系误差的计算方法：把建图轨迹用 Umeyama 对齐到真值，得到一个变换（拟合残差 0.12 m），再用这个变换评估定位轨迹。所以误差里也包含了地图本身的误差。
纯 LIO 基线取决于首次重定位的精度，所以给的是两次运行的范围。

**自录 Mid-360**（174 s，170 m，没有真值）：同一个 bag 建图，再从 60 s 开始定位。以建图轨迹作参考，结果偏乐观。

| | 位置 RMSE | 位置 max | 姿态 max |
|---|---:|---:|---:|
| 纯 LIO | 0.68 m | 1.08 m | 2.1° |
| 后端 | 0.012 m | 0.04 m | 0.7° |
| 阶跃注入 20 m / 90° | 0.012 m | 0.04 m | 0.7° |
| 渐变注入（8 s 内 20° 加 2.8 m，绕 odom 原点转动，实际位置偏移超过 10 m） | 1.8 m | 6.8 m | 15.6° |

建图时回环把首尾差从 1.41 m 降到 0.69 m。

## 已知限制

- 渐变漂移在 3 个关键帧内就能检出，但如果漂移持续，里程计因子没有门控，图会在错误的 odom 和地图之间折中。
  上面那个极端的渐变注入，在注入结束后还要约 15 s 才收敛回来。
- 重定位最多要做 17 次配准，最长约 3.4 s。实时节点里它在后端线程中运行，队列超过 50 帧时丢弃最老的帧；离线回放会阻塞。
- 阈值（`min_inlier_ratio` 0.3、`reloc_min_inlier_ratio` 0.35、`reg.sigma_min` 0.1）是用这两套数据调的。换场景或换雷达后需要重新检查。
- 先验图如果是单个 PCD，没有 SC 描述子，全局重定位只能依靠 `backend.initial_pose` 加朝向扫描。
