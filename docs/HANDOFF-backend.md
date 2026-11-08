# zhao_lio 后端交接文档

更新日期：2026-11-01

前端（LIO 本身）的交接见 [`HANDOFF.md`](HANDOFF.md)。本文只讲 2026-10-31 到 11-01 加入的后端。

## 1. 这是什么

`src/zhao_lio_backend/` 是 zhao_lio 的位姿图后端，用来修正 LIO 的漂移。思路来自 z-localization 的后端（`liop/z-localization/docs/design.md` 第一章和第三章）：

- 前端照常在 odom 系里运行，后端不改前端的状态。
- 后端维护 `T_map_odom`，对外输出 `T_map_body = T_map_odom * T_odom_body`。
- 关键帧位姿放进因子图优化。约束有三类：前端给的相对运动（里程计因子），回环或先验图配准（绝对约束），以及重力方向。
- 每个观测先过 NIS 卡方检验，与预测冲突就拒绝，然后由状态机决定是否需要重定位或重启前端。

实现了三项功能：**无图回环**（mapping 模式）、**先验图定位**（localization 模式）、**重定位和状态机**。UWB 没有做。

**许可证：** z-localization 是 GPL-2.0-only，zhao_lio 是 GPLv3，两者的源码不能合并进同一个程序。所以这里只参考了它的设计，代码全部重写，**没有复制 z-localization 的任何源码**。以后改这个包也要保持这一点。

**依赖：** 和 zhao_lio_core 一样，纯 C++17，只依赖 Eigen、TBB 和 zhao_lio_core，不依赖 ROS、PCL、GTSAM。位姿图是用 Eigen 手写的（LM + 稀疏 LDLT），这是当时选定的方案，因为本机没有装 GTSAM。

## 2. 当前状态

- 能编译。后端单元测试 9 个全部通过，zhao_lio_core 原有的测试也全部通过。
- 在两套数据上用离线回放跑完了建图、定位和状态机注入测试，结果见第 6 节。
- 实时节点 `zhao_lio_node` 已经接入后端。用 `ros2 bag play` 做过冒烟测试，状态、TF 和位姿话题都正常；**没有在机器人上跑过**。
- **代码没有提交。** 工作区里还有前端的一些未提交改动（`io.h`、几个 config、`replay.cpp`、`zhao_lio_node.cpp` 等，在后端开发之前就存在），提交时注意分开。
- 后端新增代码约 2000 行（不含测试）。

## 3. 构建和测试

```bash
cd ~/cs_project/liop/zhao_lio
source /opt/ros/humble/setup.bash
source ~/cs_project/nav/cs/install/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
./build/zhao_lio_backend/backend_test_math           # SE(3)、卡方分位数、位姿图（回环、边缘化、降秩因子）
./build/zhao_lio_backend/backend_test_registration   # 配准、走廊退化方向、Scan Context 求 yaw
```

包之间的依赖：`zhao_lio_core` ← `zhao_lio_backend` ← `zhao_lio_ros2`。只改了后端时，可以只编译这两个包：`--packages-select zhao_lio_backend zhao_lio_ros2`。

## 4. 运行

后端的参数都在 `backend.*` 下。`backend.mode` 为空时不启用后端，前端的行为和原来完全一样。参数示例见 `src/zhao_lio_ros2/config/backend.yaml`。

**离线回放**（做实验用，结果是确定的，只有 TBB 并行带来的少量差异）：

```bash
# 建图：退出时保存地图包
ros2 run zhao_lio_ros2 zhao_lio_replay --ros-args --params-file scripts/backend/replay_tracked01.yaml \
  -p bag:=$HOME/datasets/liobench/narrowwide/tracked-01/bag \
  -p backend.mode:=mapping -p backend.save_dir:=/tmp/t01_map \
  -p backend.traj_path:=rt.tum -p backend.opt_traj_path:=opt.tum -p traj_path:=lio.tum

# 先验图定位：start_offset 跳过 bag 开头，前端会在另一个位置启动，用来测试不给初值的全局重定位
ros2 run zhao_lio_ros2 zhao_lio_replay --ros-args --params-file scripts/backend/replay_tracked01.yaml \
  -p bag:=$HOME/datasets/liobench/narrowwide/tracked-01/bag -p start_offset:=200.0 \
  -p backend.mode:=localization -p backend.map_path:=/tmp/t01_map -p backend.traj_path:=loc.tum
```

回放结束时会打印一行 `backend: ...` 汇总：最终状态、可信帧数、回环接受数/尝试数、地图观测接受数和拒绝原因、重定位次数、前端重置次数、每帧耗时。

回放输出的轨迹：
- `backend.traj_path`：实时输出的 map 系轨迹，只写可信帧。
- `backend.opt_traj_path`：所有帧用最终的关键帧位姿重新计算，只在离线评估时有意义。

**状态机测试开关**（只在回放里有）：`test_perturb_time`（秒，从 bag 开头算）、`test_perturb:=[dx, dy, yaw_deg]`、`test_perturb_ramp`（秒，0 表示阶跃）。打开后，从指定时刻起给送入后端的里程计左乘一个偏移，前端本身不受影响。注意偏移是绕 odom 原点转的，机器人离原点远时，一个小 yaw 也会带来很大的位置偏移。

**实时节点：**

```bash
ros2 run zhao_lio_ros2 zhao_lio_node --ros-args --params-file src/zhao_lio_ros2/config/mid360.yaml \
  --params-file src/zhao_lio_ros2/config/backend.yaml -p backend.mode:=localization -p backend.map_path:=<地图包>
```

后端在单独的线程里运行（`include/zhao_lio_ros2/backend_runner.h`），不阻塞 LIO 回调。积压超过 50 帧时丢弃最老的帧。额外的输出：

| 名称 | 类型 | 内容 |
|---|---|---|
| TF `map → odom` | | 每帧发布；SUSPECT/LOST 时保持最后一个可信值 |
| `/zhao_lio/backend/pose` | PoseStamped | map 系 body 位姿，只发可信帧 |
| `/zhao_lio/backend/path` | Path | 关键帧位姿 |
| `/zhao_lio/backend/state` | String，transient local | 例如 `NORMAL trusted`，状态变化时发布 |
| `/zhao_lio/backend/save_map` | Trigger 服务 | 只在 mapping 模式可用 |

后端请求重启前端时，节点在下一帧之前调用 `ZhaoLio::reset()`。

**地图包**是一个目录：`keyframes.txt`（每个关键帧的 map 系位姿和点数）、`clouds.bin`（关键帧点云，body 系，float32 xyz，每帧前面有一个 uint32 点数）、`map.pcd`（0.1 m 体素的全局点云，给 CloudCompare 看，定位时不读）。加载时会根据关键帧点云重新计算 Scan Context。`backend.map_path` 也可以直接给一个 PCD，但这样没有 SC 描述子，全局重定位只能依靠 `backend.initial_pose: [x, y, z, yaw_deg]` 加 12 个朝向扫描。

## 5. 代码结构

```
src/zhao_lio_backend/
  include/zhao_lio_backend/
    se3.h               SE(3) Exp/Log、伴随、右雅可比逆。右扰动 T*Exp(ξ)，ξ = [ρ; φ]
    chi2.h              卡方分位数（Wilson-Hilferty 近似，不依赖 boost）
    pose_graph.h        位姿图
    registration.h      点到面 ICP + 可观方向投影
    scan_context.h      Scan Context 描述子和数据库（实现在 scan_context_impl.h 和 .cpp）
    map_io.h            PCD 读取（ascii/binary）、体素滤波
    backend.h           Backend 主类：BackendParams、状态、统计
  src/  test/  README.md
src/zhao_lio_ros2/include/zhao_lio_ros2/
  backend_config.h      ROS 参数 backend.* → BackendParams
  backend_runner.h      实时节点里的后端线程、发布、服务
scripts/backend/        评估脚本、回放配置、run_eval.sh
```

### 5.1 位姿图（`pose_graph`）

- 节点：关键帧 id → `T_map_body`。
- 因子：
  - 先验 `Log(Z⁻¹T)`；
  - 相对 `Log(Z⁻¹ Ti⁻¹ Tj)`；
  - 重力方向 `Rᵀe_z − up_body`，只约束 roll 和 pitch。

  信息矩阵用平方根形式 `L`（r×6，r ≤ 6），所以可以只约束部分方向。可选 Huber 核（作用在白化残差的范数上，IRLS）。
- 求解：LM，`Eigen::SimplicialLDLT`。每次都是全量求解，没有做增量。514 个关键帧时最慢一帧约 60 ms。
- `marginal(ids)`：在当前估计处线性化，解出指定节点的边缘协方差。
- `innovation_prior / innovation_between`：用 `J P Jᵀ + I`（白化空间）计算候选因子的 NIS，不会把因子加进图。
- `marginalize_oldest()`：删除最老的节点。所有涉及它的因子在当前估计处线性化，再用 Schur 补消元，变成相连节点上的线性先验 `g'δ + ½δ'Hδ`。只在定位模式的滑窗里使用。

### 5.2 配准（`registration`）

- 目标点云放在两层 `zhao_lio::OctVoxMap` 里：粗层 1.2 m、细层 0.4 m，复用前端的 5 近邻搜索。
- 平面用 PCA 拟合。**近似共线的邻居会被拒绝**（中间特征值小于最大特征值的 5%），沿法向的散布也必须明显小于阈值。这条规则是测试中加的：前端那种最小二乘拟合在墙角、边缘会给出方向随意的法向，在走廊这类退化场景里会凭空产生沿走廊方向的信息。
- 信息矩阵 = `H / (σ² · max(1, N/n_eff))`。σ 取内点 RMSE，下限是 `sigma_floor`；`n_eff = 200`，把点之间的相关性折算进去。
- 可观方向投影：旋转按 `rot_scale = 10 m/rad` 换算成长度后做特征分解。保留的方向要满足两个条件：标准差 < `max_sigma`（0.3 m），且特征值不小于最大特征值的 `min_eig_ratio`（0.2%）。每个保留方向的标准差再加上一个下限 `sigma_min`（0.1 m），见第 8 节。

### 5.3 每帧流程（`Backend::push`）

```
session 变了？→ 新 odom 系：T_map_odom 用"机器人没动"来衔接；非 INITIALIZING 时进入 LOST
odom 速度 > max_speed？→ LOST，请求重置前端，丢弃旧 session 的帧
缓存最近 kf_accum_frames 帧点云（odom 系）
新 session 的第一个关键帧要等缓存满了才生成；之后按 1 m / 10° / 2 s 选关键帧
关键帧：合并缓存点云并转到 body 系 → 0.25 m 体素 → 用重力方向调平后计算 SC
         → mapping 或 localization 的处理逻辑（见下）
输出 T_map_odom、T_map_body、状态、是否可信
```

**mapping：** 加节点、重力因子和里程计因子，然后找回环。候选有两种：
- 6 m 内、时间相隔 ≥ 30 s 的最近关键帧；
- SC 距离 < 0.3 的关键帧。

候选关键帧前后各 6 帧合成子图，做 ICP（粗层 + 细层）。验收条件：收敛、rank = 6、内点比例 ≥ 0.3、RMSE ≤ 0.12，NIS 不超过 99.99% 分位数。通过后加入回环因子（带 Huber 核）。全部关键帧都保留，不做滑窗。

**localization：**
- `INITIALIZING / LOST`：只做重定位，不往图里加节点。
- `NORMAL / OBS_DEGRADED`：
  1. 加节点和里程计因子。
  2. 从预测位姿出发，只用细层和先验图配准。
  3. 质量合格且 NIS 不超过 99% 分位数，就加入可观方向的绝对因子。
  4. 优化。
  5. 图里超过 100 个关键帧时边缘化最老的。
- `SUSPECT`：继续加里程计节点，并在每个关键帧尝试重定位。

定位模式**不加重力因子**，原因见第 8 节。

**状态机：**
- NIS 超限，或者内点比例低于阈值的一半，记一次冲突。连续 3 次冲突进入 SUSPECT。
- 重定位的候选：在预测位置按 30° 间隔试 12 个朝向（只有曾经可信过，或者给了初值时才做），加上 SC 的前 5 个候选。每个候选做粗层加细层配准。
- 最好候选的内点比例 ≥ 0.35 且 rank = 6 才接受。如果有另一个位姿明显不同的候选，内点比例与它相差不到 0.05，视为有歧义，拒绝。
- 重定位成功后清空图，以配准结果为锚点重新开始。
- SUSPECT 下连续失败 5 次进入 LOST，并重启前端。

### 5.4 对已有代码的改动

| 文件 | 改动 |
|---|---|
| `zhao_lio_core/include/zhao_lio/zhao_lio.h`, `src/zhao_lio.cpp` | `reset()` 改为公有；新增 `session()`，每次 reset 加 1（包括发散后的自动 reset） |
| `zhao_lio_ros2/src/replay.cpp` | 接入后端；新增 `start_offset`、`test_perturb*` 参数；结束时打印后端汇总、写优化后的轨迹、保存地图包（只在 mapping 模式） |
| `zhao_lio_ros2/src/zhao_lio_node.cpp` | 可选的 `BackendRunner`；处理后端发来的重置请求 |
| `zhao_lio_ros2/CMakeLists.txt`, `package.xml` | 依赖 `zhao_lio_backend`、`std_msgs` |
| 新文件 | `config/backend.yaml`、`include/zhao_lio_ros2/backend_config.h`、`backend_runner.h` |

## 6. 实验结果

所有数字都是每个配置跑 1 次（2026-11-01）。TBB 并行让每次运行有几个百分点的差异，纯 LIO 定位基线的差异更大（见下文）。复现：`scripts/backend/run_eval.sh`，结果写到 `/tmp/zb`。**`/tmp/zb` 里现有的结果重启后就没了**，需要的话请重新跑。

### 6.1 NarrowWide tracked-01（MID-70，621 s，262 m，有真值）

回放配置是 `scripts/backend/replay_tracked01.yaml`，也就是 liobench 基线用的参数，只是把更新策略换成了 `pointwise_prematch`。

**建图加回环**（evo，SE(3) 对齐）：

| | ATE RMSE | ATE max | RPE 200 m |
|---|---:|---:|---:|
| 纯 LIO | 0.109 m | 0.54 m | 1.06 m |
| 后端实时输出 | 0.119 m | 0.53 m | 1.06 m |
| 后端优化后全轨迹 | 0.124 m | 0.43 m | 0.94 m |

接受了 29 个回环。`scripts/backend/loop_check.py` 用真值逐个核对过，回环测量和 LIO 一样准或者更准，没有发现错误的回环。这条序列上 LIO 本身漂移很小（首尾差约 0.2 m），所以回环只降低了最大误差和长距离 RPE，ATE RMSE 反而略差。**在这份数据上，回环不能算有明显收益。**

注意：不要用"只拿开头 30 s 对齐"来评估这条序列。开头 30 s 只移动了 3 m，对齐时旋转估不准，会得出后端更差的错误结论。我一开始就在这里被误导过。

**先验图定位**：用上面的地图包，从 200 s 开始回放，不给初值。误差在 map 系里计算（`loc_eval.py`：把建图轨迹对齐到真值，拟合残差 0.12 m，再用同一个变换评估定位轨迹，所以误差里也包含了地图本身的误差）：

| | RMSE | p95 | max |
|---|---:|---:|---:|
| 纯 LIO（只用首次重定位的 TF） | 0.22–0.41 m | 0.40–0.92 m | 0.69–1.06 m |
| 后端 | 0.12 m | 0.27 m | 0.47 m |
| 后端 + 350 s 阶跃 30 m / 90° | 0.12 m | 0.26 m | 0.48 m |
| 后端 + 350 s 起 10 s 渐变 3.6 m / 25° | 0.23 m | 0.31 m | 2.40 m |

纯 LIO 这一行取决于首次重定位的精度，两次运行之间差别很大，所以给的是范围。这里的地图和定位用的是同一个 bag，比重新录一段数据要容易。NarrowWide 的 tracked-02 和 tracked-01 在同一个场地、同一个真值坐标系，但本机只有真值，没有 tracked-02 的 bag。下载之后就能做真正的跨序列定位测试。

### 6.2 自录 Mid-360（`offline_map_20261014`，174 s，170 m，没有真值）

回放配置是 `scripts/backend/replay_own_mid360.yaml`（`mid360_liobench.yaml`，降采样和地图分辨率改成 0.3 m）。这台雷达的安装横滚角约 54°。

- 建图：接受了 14 个回环，首尾差从 1.41 m（LIO）降到 0.69 m。
- 定位：用同一个 bag 建的图，从 60 s 开始回放，不给初值。参考轨迹是建图的优化轨迹，**这只能检验一致性，数字偏乐观**。

| | 位置 RMSE | 位置 max | 姿态 max |
|---|---:|---:|---:|
| 纯 LIO | 0.68 m | 1.08 m | 2.1° |
| 后端 | 0.012 m | 0.04 m | 0.7° |
| 100 s 阶跃 20 m / 90° | 0.012 m | 0.04 m | 0.7° |
| 100 s 起 8 s 渐变 2.8 m / 20°（绕 odom 原点转，实际偏移 > 10 m） | 1.8 m | 6.8 m | 15.6° |

后端在第 1 个关键帧就完成了全局重定位，之后 161 次地图观测全部接受。

### 6.3 状态机行为

- **阶跃**：两套数据上都在下一帧被速度检查发现（`max_speed` 15 m/s）→ LOST → 重启前端 → 新 session 的第一个关键帧就重定位成功。
- **渐变**：约 3 个关键帧内 NIS 连续超限 → SUSPECT → 重定位。但如果漂移在继续，里程计因子是不过门控的，图会在错误的里程计和地图之间折中。在上表那个极端例子里，注入结束后还要约 15 s 才收敛回来。
- **耗时**：正常帧平均 0.5–1.3 ms。一次重定位最多 17 次配准：tracked-01 上约 0.2–0.3 s，自录数据上最长 3.4 s。

## 7. 和 z-localization 的差异（有意为之）

| 方面 | z-localization | 这里 |
|---|---|---|
| 优化库 | GTSAM ISAM2 增量求解 | Eigen 手写 LM，每次全量求解，规模靠关键帧数（建图）或滑窗（定位）控制 |
| 退化处理 | DCReg：Schur 补加预条件器 | 只做特征分解投影，不处理求解时的数值条件 |
| 观测协方差 | 残差尺度 + 海森矩阵 | 同样的做法，另外加了 n_eff 折算和每个方向的标准差下限 |
| 描述子 | SC 和 G-PROBE | 只有 SC，地图包里也没有按网格离线生成的描述子 |
| UWB | 有，支持在线标定 | 没有做 |
| 前后端接口 | 独立进程，消息带代次，有协调的 reset 事务 | 同一个进程；用 `session()` 计数区分 odom 系；reset 是单向请求 |
| 输出冻结 | SUSPECT/LOST 时冻结在最后可信位姿 | 同样做法（TF 保持，pose 话题不发） |

## 8. 调试中发现的问题（已修复，以后改动时注意）

1. **调平不能用姿态的 roll/pitch。** 自录 Mid-360 横滚约 54°，前端的重力估计（`State30::grav`）和姿态对应的"水平"之间差了约 6°。原来的重力因子和 SC 调平都用姿态，结果每次重定位后都持续冲突（一次运行里重定位了 32 次）。现在 `push()` 接收前端估计的 `up_odom = -grav`，重力因子和 SC 调平都用它；定位模式下不再加重力因子，因为 roll/pitch 由先验图观测。
2. **配准海森矩阵过于自信。** 点残差之间并不独立，正常的观测也会被 99% 的 NIS 门限拒绝，进而频繁进入 SUSPECT。现在每个方向的标准差有 `reg.sigma_min = 0.1 m` 的下限。tracked-01 上取 0.05 时拒绝 31 次，0.1 时拒绝 5 次，0.15 时 0 次，精度基本不变。
3. **共线邻居产生假信息**，见 5.2。
4. **新 session 的第一个关键帧只有 1 帧点云。** 实时节点上，Mid-360 单帧约 5000 点，重定位很容易配错。现在第一个关键帧要等合并满 `kf_accum_frames` 帧才生成。
5. **定位模式不能保存地图包。** 窗口外关键帧的点云已经释放，保存出来的包是不完整的，还可能覆盖先验图。现在只在 mapping 模式保存。

## 9. 已知问题和风险

1. **阈值是用这两套数据调的**：`min_inlier_ratio` 0.3、`reloc_min_inlier_ratio` 0.35、`reg.sigma_min` 0.1、里程计噪声每米 0.02 m / 0.002 rad。换场景或换雷达后需要重新检查。z-localization 原文的内点要求更高，这里放低是因为 MID-70 视场窄，和地图的重叠只有一部分。
2. **没有真正的跨序列定位测试。** 两套数据都是用同一个 bag 建图再定位。
3. **里程计因子不过门控**，持续的渐变漂移只能靠状态机兜底，见 6.3。
4. **重定位会阻塞后端线程**，最长约 3.4 s。实时节点里积压超过 50 帧会丢帧（丢的是后端的输入，不影响前端），这期间 TF 不更新。
5. **歧义检查比较粗**：只比较内点比例。在对称的场地里，可能需要像 z-localization 那样看候选得分的分布。
6. **建图模式下的 LOST**：前端重启之后，新 session 的第一个节点只有一个很弱的先验，必须靠 SC 回环才能接回旧地图。这条路径**没有测试过**。
7. **建图模式每个关键帧都全量优化**，关键帧多了以后会变慢。514 个关键帧时最慢一帧约 60 ms，几千个关键帧时需要改成增量求解，或者只在接受回环之后才优化。
8. 实时节点的 `/zhao_lio/backend/path` 无限增长，和前端的 path 一样。

## 10. 建议的下一步

1. 下载 NarrowWide tracked-02 的 bag，用 tracked-01 建的图在 tracked-02 上做定位，这是目前最有说服力的测试。
2. 在自己的场地再录一段，和建图那段错开路线和起点，测试实际的定位效果。
3. 在机器人上跑实时节点，重点看重定位期间的延迟和丢帧。
4. 如果要用于长时间建图：回环接受后才做全量优化，平时只做局部优化，或者引入增量求解。
5. 给里程计因子加一个一致性检查（比如和最近的地图观测比较），缩短持续漂移的恢复时间。
6. 把后端接进 liobench，目前只能手动评估。liobench 按黑盒方式运行实时节点，订阅 `/zhao_lio/odom`，需要让它改为读取 `/zhao_lio/backend/pose`。

## 11. 相关资料

- z-localization 设计说明：`liop/z-localization/docs/design.md`。源码在 `liop/z-localization/src/backend/`，只能参考思路，不能复制。
- 后端包说明：`src/zhao_lio_backend/README.md`（模块、模式、使用方法）。
- 参数示例：`src/zhao_lio_ros2/config/backend.yaml`。
- 评估脚本：`scripts/backend/`。`run_eval.sh` 复现第 6 节；`loc_eval.py` 计算 map 系误差；`lio_in_map.py` 生成纯 LIO 基线；`loop_check.py` 用真值核对回环。
- Scan Context：Kim & Kim, IROS 2018。
