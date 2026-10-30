# liobench 教程：用测试驱动 zhao_lio 的迭代

这份教程讲日常怎么用 liobench：改代码 → 跑一条命令 → 看这一版比上一版好还是差 → 决定保留还是回退。

## 0. 每个终端先做的事

```bash
cd ~/cs_project/liop/liobench
source /opt/ros/humble/setup.bash
alias L=.venv/bin/liobench        # 下文的 L 都指 liobench
```

`L ls` 列出接入了哪些算法、哪些序列已经转换、哪些序列有真值。

## 1. 平台由哪几块组成

```
数据集原始包 ──L convert──▶ 标准格式包 ──▶ 算法（黑盒）──▶ 记录器 ──▶ 指标 ──▶ 报告 / 历史
 datasets/*.yaml           ~/datasets/liobench/      algorithms/*.yaml       run.json   summary.md
```

| 目录或文件 | 作用 | 什么时候改 |
|---|---|---|
| `datasets/*.yaml` | 数据集在哪里、原始格式怎么解码、外参 | 加新数据集、拿到真值以后 |
| `algorithms/*.yaml` | 每个算法怎么启动、参数怎么映射 | 加新算法，或者算法改了参数名 |
| `experiments/*.yaml` | 跑哪些算法、哪些参数、哪些序列、重复几次 | 做对比实验、做参数扫描 |
| `dev/zhao_lio.yaml` | 开发循环：怎么编译、跑什么、跟谁比、验收门槛 | 调整验收标准 |
| `results/` | 所有结果，可以删，删了重跑即可 | 不用手改 |

## 2. 核心用法：开发循环

改完 zhao_lio 的代码以后，只跑这一条命令：

```bash
L dev dev/zhao_lio.yaml --label rematch-005 --note "rematch_thr 从 0.1 改成 0.05"
```

它按顺序做下面几件事，任何一步失败都会停下来：

1. **编译**：在 仓库根目录 里执行 `colcon build`，日志写在 `results/dev/zhao_lio/<版本>/build.log`。
2. **存档源码**：把 `../src` 打包成 `source.tar.gz`。如果源码和以前某一版完全一样，会提示你。
3. **跑测试**：执行 `experiments/dev_zhao_lio.yaml`，默认是两种更新策略 × 所有已转换的序列。
4. **对比**：和参考版本逐项对比，每个指标显示变化量和百分比。
5. **门槛检查**：每一条标为"通过"、"不通过"或"跳过"，最后给出结论。
6. **记入历史**：结果追加到 `results/dev/zhao_lio/history.csv`。

下面是一次真实输出（2026-10-29，同一份代码第三次运行，每个包只播前 60 s）：

```
=== tutorial_demo v3（源码 8ee4412f74，每个包只播前 60 s）对比 v2 ===

算法       配置                                序列                   成功 参考→新  ATE(m)变化  RPE(m)变化  延迟均值(ms)变化  延迟P95(ms)变化  内部均值(ms)变化  CPU(%)变化  内存(MB)变化
---------  ----------------------------------  ---------------------  ------------  ----------  ----------  ----------------  ---------------  ----------------  ----------  ------------
zhao_lio  update_strategy=pointwise           narrowwide/tracked-01     1/1 → 1/1           -           -       +0.01 (+0%)      +0.12 (+2%)       -0.00 (-0%)    +0 (+5%)      -0 (-0%)
zhao_lio  update_strategy=pointwise_prematch  narrowwide/tracked-01     1/1 → 1/1           -           -       -0.05 (-1%)      -0.11 (-2%)       -0.06 (-8%)    -0 (-1%)      +0 (+0%)

  [通过] update_strategy=pointwise  成功率：1（下限 1.0）
  [跳过] update_strategy=pointwise  ATE(m)：没有数据（比如没有真值）
  [跳过] update_strategy=pointwise  RPE(m)：没有数据（比如没有真值）
  [通过] update_strategy=pointwise  内部均值(ms)：0.9765 -> 0.9726 (-0.4%)
  [通过] update_strategy=pointwise  延迟P95(ms)：6.409（上限 20）
  [通过] update_strategy=pointwise  内存(MB)：94.14（上限 500）
  [通过] update_strategy=pointwise_prematch  成功率：1（下限 1.0）
  ...
  [通过] update_strategy=pointwise_prematch  内部均值(ms)：0.8543 -> 0.7895 (-7.6%)
  ...

结论：通过
```

怎么读这份输出：
- **第一张表**：每个配置相对参考版本的变化，括号里是百分比。负数表示变小，对误差、耗时和内存来说就是变好。
- **门槛行**：每个配置、每个门槛各占一行。ATE 和 RPE 标为"跳过"，因为做这次演示时 tracked-01 还没有真值。2026-10-30 已经补上真值，现在这两项会参与判定。
- **不通过时的样子**：比如内部耗时涨了 28%，超过 20% 的门槛，会显示：
  ```
    [不通过] update_strategy=pointwise  内部均值(ms)：0.77 -> 0.99 (+28.6% > +20%)
  结论：不通过
  ```
  命令的退出码是 1，可以在脚本里用 `&&` 串联后续步骤。

**先认识噪声，再定门槛。** 上面三次运行用的是同一份代码，内部耗时还是变了 -7.6% 到 +3.2%。所以：
- 耗时的相对门槛不要设得比 10% 更严，否则代码没改也可能判为不通过。默认是 20%。
- 想看清 5% 以内的耗时变化，就在 `experiments/dev_zhao_lio.yaml` 里设 `repeats: 3`，并且用全长运行。
- 精度指标（ATE、RPE）由算法本身决定，同一份代码重跑几乎不变，可以设得更严，默认 5%。

**常用参数：**
- `--duration 60`：只播每个包的前 60 秒，用来快速确认改动能跑通。快速检查只和同样时长的版本比，历史表里"播放"列会标出 `60 s`，"最好版本"只从全长运行里选。
- `--skip-build`：你已经手动编译过时用。
- `--label`：版本名。不写的话按时间自动生成，比如 `v20261029-205500`。同一个版本名不能重复使用。

## 3. 查看所有版本

```bash
L history dev/zhao_lio.yaml
```

真实输出（上面那三次演示运行）：

```
版本  时间              配置                                播放  通过  成功/次数  延迟均值(ms)  延迟P95(ms)  内部均值(ms)  内部P95(ms)  CPU(%)  内存(MB)  说明
----  ----------------  ----------------------------------  ----  ----  ---------  ------------  -----------  ------------  -----------  ------  --------  ----------------------------
v1    2026-10-29 20:55  update_strategy=pointwise           60 s    是        1/1         4.973        6.324         0.966        1.515   3.563    94.350  教程演示：当前代码
v1    2026-10-29 20:55  update_strategy=pointwise_prematch  60 s    是        1/1         4.776        5.956         0.828        1.176   4.779    96.540  教程演示：当前代码
v2    2026-10-29 20:57  update_strategy=pointwise           60 s    是        1/1         4.952        6.294         0.977        1.525   3.430    94.520  教程演示：同一份代码再跑一次
v2    2026-10-29 20:57  update_strategy=pointwise_prematch  60 s    是        1/1         4.799        6.098         0.854        1.212   4.745    96.420  教程演示：同一份代码再跑一次
v3    2026-10-29 21:01  update_strategy=pointwise           60 s    是        1/1         4.959        6.409         0.973        1.542   3.598    94.140  教程演示：第三次
v3    2026-10-29 21:01  update_strategy=pointwise_prematch  60 s    是        1/1         4.750        5.984         0.790        1.041   4.695    96.490  教程演示：第三次
```

有真值以后，表里会多出 ATE 和 RPE 两列，末尾还会显示"按 ATE(m) 最好的通过版本（全长运行）：……"。

`history.csv` 可以直接用 Excel 或 pandas 打开画曲线。

## 4. 找回满意的版本

zhao_lio 已经用 git 管理，平时用 `git log`、`git checkout` 找回旧版本就行。`dev` 另外给每一版存了源码快照，这样即使某次测试时代码还没提交，也能找回当时测的是哪一份：

```bash
L restore dev/zhao_lio.yaml rematch-01 /tmp/zhao_lio_rematch-01
diff -r /tmp/zhao_lio_rematch-01 ../src     # 看和现在差了什么
```

`restore` 只解压到一个新的空目录，不会覆盖你正在改的代码。确认要回退时，再自己把文件拷回去。

## 5. 配置验收标准：`dev/zhao_lio.yaml`

```yaml
reference: previous      # 和谁比：previous（上一版）| best（历史最好的通过版）| 某个 label，比如 rematch-01
primary: ate_rmse_m      # "最好"按哪个指标算，越小越好
gates:
  success_rate:     {min: 1.0}                     # 任何序列都不允许发散
  ate_rmse_m:       {max_increase_pct: 5}          # 精度最多变差 5%
  internal_mean_ms: {max: 2.0, max_increase_pct: 20}  # 内部耗时不超过 2 ms，且最多变慢 20%
  latency_p95_ms:   {max: 20}
  peak_rss_mb:      {max: 500}
```

每个指标可以设四种门槛：`max` 和 `min` 是绝对上下限，`max_increase_pct` 和 `max_decrease_pct` 是相对参考版本的变化幅度。配置里用英文键名，输出里显示中文名：

| 配置键名 | 输出中的名字 | 配置键名 | 输出中的名字 |
|---|---|---|---|
| `success_rate` | 成功率 | `internal_mean_ms` | 内部均值(ms) |
| `ate_rmse_m` | ATE(m) | `internal_p95_ms` | 内部P95(ms) |
| `rpe_trans_m` | RPE(m) | `latency_mean_ms` | 延迟均值(ms) |
| `rpe_rot_deg` | RPE(°) | `latency_p95_ms` | 延迟P95(ms) |
| `cpu_mean_pct` | CPU(%) | `peak_rss_mb` | 内存(MB) |

怎么选参考版本：
- 小步调参时用 `previous`，每一步都不允许变差。
- 大改动时用 `best`，或者固定一个 label，避免很多次小幅退化累积起来。

## 6. 配置每一版要跑的测试：`experiments/dev_zhao_lio.yaml`

```yaml
algorithms:
  - name: zhao_lio
    params:
      update_strategy: [pointwise, pointwise_prematch]   # 两个变体分别判定
sequences: [narrowwide/*]       # 已转换的全部序列
repeats: 1                      # 想看耗时的波动就改成 3，报告里会显示 ±std
```

`params` 里写的是 zhao_lio 自己的 ROS 参数名，就是 `config/tracked.yaml` 里的那些键。写成列表就会做参数扫描。例如：

```yaml
params:
  update_strategy: [pointwise_prematch]
  rematch_thr: [0.05, 0.1, 0.2]
  downsample_size: [0.2, 0.5]       # 3 × 2 = 6 个变体
```

**两类改动的区别：**
- 只改参数、不改代码：用 `L run experiments/xxx.yaml` 做扫描就够了，不需要 `dev`。
- 改了代码：用 `L dev`，它会编译、存档、记历史。

## 7. 和其他算法横向对比

```bash
L run experiments/baseline.yaml      # 7 个变体 × 全部序列，tracked-01 一条大约 75 分钟
L report results/baseline            # 重新生成表格，summary.md 可以直接贴进文档
L compare results/baseline results/baseline_v2   # 两次对比实验之间的差别
```

完成的运行会自动跳过，中断后再执行同一条命令就从断点继续。只想重跑某个算法时，加 `--only zhao_lio --force`。

## 8. 真值

tracked-01 和 tracked-02 的真值放在 `~/datasets/narrowwide/gt/gt_tracked-0{1,2}.txt`。tracked-01 已经转换好了（`L ls` 里显示"有真值"），所以它的 ATE、RPE 和对应的门槛现在都会参与判定。tracked-02 还缺数据包，下载以后执行 `L convert narrowwide`，数据包和真值会一起转换。

以后再拿到新的真值文件：
1. 放到 `datasets/<名字>.yaml` 里 `gt:` 写的位置。
2. 运行 `L convert <名字>`。它发现真值文件是新增的，只转换真值，不重新转换数据包。
3. 运行 `L reeval results/<实验>`，不重跑算法，直接给已有结果补上 ATE 和 RPE。开发版本也可以这样补：`L reeval results/dev/zhao_lio/<版本>/runs`。

## 9. 加新数据集

可以参考 `datasets/README.md`。最小步骤：

1. 新建 `datasets/<name>.yaml`，写清楚 `output`、`source`（话题和解码器）、`sensor`（外参）和 `sequences`。
2. 用 `L convert --list` 查看支持哪些解码器。机械旋转雷达用 `velodyne`、`ouster`、`hesai`、`robosense`。不认识的 PointCloud2 用通用的 `pointcloud2`，在配置里写清楚时间字段名和单位。
3. 执行 `L convert <name>`，然后 `L ls` 确认序列已转换。
4. 在实验的 `sequences` 里加上 `<name>/*`。

## 10. 指标是怎么算的

| 指标 | 含义 | 注意 |
|---|---|---|
| ATE(m) | 先做 SE(3) 对齐（不估尺度），再算位置误差的 RMSE | 需要真值 |
| RPE(m) / RPE(°) | 每走 10 m 的相对位移误差和相对旋转误差（RMSE） | 窗口长度由实验里的 `rpe.delta` 设置 |
| 延迟均值 / 延迟P95 | 从雷达帧到达，到算法发布这一帧的里程计，所用的墙钟时间 | 比纯计算多约 2–3 ms：要等扫描结束后的第一条 IMU（200 Hz，最多 5 ms），再加 ROS 通信；所有算法一样 |
| 内部均值 / 内部P95 | 算法自己统计的每帧耗时 | 只有 zhao_lio 提供（`timing_csv`） |
| CPU(%) | 播放期间的平均值，100% 表示占满一个核 | genz-lio、Batch-LIO 是多线程的，会超过 100% |
| 内存(MB) | 算法进程的峰值内存 | |
| 成功率 / 发散率 | 崩溃、没有输出、NaN、跳变、速度过大、覆盖率低于 90%、ATE 超限，满足任意一条就算失败 | 阈值在实验 yaml 的 `divergence` 里设置 |

**看结果时注意：**
- 计时受机器负载影响。跑测试时不要同时编译别的东西。想比较耗时，最好设 `repeats: 3`，看 ± 的范围。
- 序列没有真值时，"轨迹长度"和"两种策略的轨迹差"只能说明是否一致，不能说明谁更准。
- 单次运行的 ATE 差几毫米时不要下结论，先用 `repeats: 3` 看波动。
- 一次运行失败时，看它目录里的 `algo.log`（算法输出）和 `run.json` 里的 `reasons`。
