"""Per-run metrics from the recorder output, resource samples and ground truth."""
from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np


def read_csv(path: Path) -> dict[str, np.ndarray]:
    if not path.exists():
        return {}
    with open(path) as f:
        rows = list(csv.reader(f))
    if len(rows) < 2:
        return {h: np.array([]) for h in (rows[0] if rows else [])}
    header, body = rows[0], rows[1:]
    body = [r for r in body if len(r) == len(header)]  # drop a partially written last line
    cols = list(zip(*body)) if body else [[] for _ in header]
    out = {}
    for h, c in zip(header, cols):
        try:
            out[h] = np.array(c, dtype=float)
        except ValueError:
            out[h] = np.array(c)
    return out


def percentile(x: np.ndarray, q: float) -> float | None:
    return float(np.percentile(x, q)) if len(x) else None


def mean(x: np.ndarray) -> float | None:
    return float(np.mean(x)) if len(x) else None


# ---------------------------------------------------------------- latency


def frame_latency(lidar: dict, odom: dict, stamp_tol: float = 0.01) -> np.ndarray:
    """Wall-clock time from a LiDAR message arriving to the algorithm publishing
    the odometry for that scan.

    All supported algorithms stamp their per-scan odometry near the scan end.
    Scan k owns the stamp window [s_k + w, s_{k+1} + w) with w = scan_period / 2,
    and counts as done at the first odometry message (in receive order) whose
    stamp falls in that window. A scan whose output was skipped or merged into a
    later one gets no latency, so len(result) / n_scans is the output coverage.
    This also works for algorithms that publish at IMU rate.
    Returns one latency per covered scan (seconds).
    """
    ls, lr = lidar.get("stamp", np.array([])), lidar.get("recv_time", np.array([]))
    os_, orc = odom.get("stamp", np.array([])), odom.get("recv_time", np.array([]))
    if len(ls) < 2 or len(os_) == 0:
        return np.array([])
    order_l = np.argsort(ls)
    ls, lr = ls[order_l], lr[order_l]
    period = float(np.median(np.diff(ls)))
    edges = np.append(ls, ls[-1] + period) + period / 2
    k = np.searchsorted(edges, os_, side="right") - 1      # scan index owning each odom stamp
    valid = (k >= 0) & (k < len(ls))
    # IMU-rate outputs can be predicted before the scan arrives; only count outputs after it.
    valid[valid] &= orc[valid] >= lr[k[valid]]
    first = np.full(len(ls), np.inf)
    np.minimum.at(first, k[valid], orc[valid])
    done = np.isfinite(first)
    lat = first[done] - lr[done]
    return lat[lat >= 0]


def internal_timing(path: Path, column: str, scale: float = 1e-3) -> np.ndarray:
    d = read_csv(path)
    if column not in d:
        return np.array([])
    return d[column].astype(float) * scale


# ---------------------------------------------------------------- resources


def resource_summary(res: dict, t0: float | None, t1: float | None) -> dict:
    """res: columns t, cpu_pct, rss_mb, hwm_mb. CPU is in % of one core."""
    if not res or len(res.get("t", [])) == 0:
        return {"cpu_mean_pct": None, "cpu_p95_pct": None, "peak_rss_mb": None}
    t = res["t"]
    sel = np.ones(len(t), bool)
    if t0 is not None:
        sel &= t >= t0
    if t1 is not None:
        sel &= t <= t1
    cpu = res["cpu_pct"][sel]
    peak = float(max(np.max(res["rss_mb"]), np.max(res.get("hwm_mb", res["rss_mb"]))))
    return {"cpu_mean_pct": mean(cpu), "cpu_p95_pct": percentile(cpu, 95), "peak_rss_mb": peak}


# ---------------------------------------------------------------- trajectory


def load_tum(path: Path) -> np.ndarray:
    """N x 8 (t x y z qx qy qz qw); comment lines and blank lines are skipped."""
    rows = []
    with open(path) as f:
        for line in f:
            s = line.strip().replace(",", " ")
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            try:
                rows.append([float(x) for x in parts[:8]])
            except ValueError:
                continue  # header line
    return np.array(rows).reshape(-1, 8)


def write_tum(path: Path, data: np.ndarray) -> None:
    with open(path, "w") as f:
        for r in data:
            f.write(f"{r[0]:.9f} " + " ".join(f"{v:.9f}" for v in r[1:8]) + "\n")


def odom_to_tum(odom: dict) -> np.ndarray:
    if not odom or len(odom.get("stamp", [])) == 0:
        return np.zeros((0, 8))
    cols = ["stamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
    data = np.stack([odom[c] for c in cols], axis=1)
    # Keep the last message for each stamp, sorted by stamp.
    _, last = np.unique(data[::-1, 0], return_index=True)
    return data[::-1][last]


def divergence_checks(traj: np.ndarray, n_lidar: int, n_frames_done: int, crit: dict) -> list[str]:
    """Reasons this run is considered diverged/failed (empty = success)."""
    reasons = []
    if len(traj) == 0:
        return ["没有里程计输出"]
    if not np.all(np.isfinite(traj)):
        reasons.append("里程计中出现 NaN/Inf")
        traj = traj[np.all(np.isfinite(traj), axis=1)]
    if len(traj) >= 2:
        dt = np.diff(traj[:, 0])
        dp = np.linalg.norm(np.diff(traj[:, 1:4], axis=0), axis=1)
        if crit.get("max_jump") is not None and np.max(dp) > crit["max_jump"]:
            reasons.append(f"位置跳变 {np.max(dp):.2f} m > {crit['max_jump']} m")
        good = dt > 1e-3
        if crit.get("max_speed") is not None and np.any(good):
            v = np.max(dp[good] / dt[good])
            if v > crit["max_speed"]:
                reasons.append(f"速度 {v:.1f} m/s > {crit['max_speed']} m/s")
    if n_lidar > 0 and crit.get("min_coverage") is not None:
        cov = n_frames_done / n_lidar
        if cov < crit["min_coverage"]:
            reasons.append(f"有输出的帧比例 {cov:.2f} < {crit['min_coverage']}")
    return reasons


def trajectory_errors(est: np.ndarray, gt: np.ndarray, rpe_cfg: dict, max_diff: float = 0.05) -> dict:
    """ATE (SE3 Umeyama alignment, no scale) and RPE via evo."""
    from evo.core import metrics, sync
    from evo.core.trajectory import PoseTrajectory3D
    from evo.core.units import Unit

    def to_traj(a):
        # evo wants quaternions as w x y z
        return PoseTrajectory3D(a[:, 1:4], a[:, [7, 4, 5, 6]], a[:, 0])

    est = est[np.all(np.isfinite(est), axis=1)]
    t_ref, t_est = sync.associate_trajectories(to_traj(gt), to_traj(est), max_diff=max_diff)
    n = t_ref.num_poses
    if n < 10:
        raise ValueError(f"只有 {n} 个位姿在 {max_diff} s 内匹配到真值")
    t_est.align(t_ref, correct_scale=False)

    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((t_ref, t_est))
    unit = {"m": Unit.meters, "frames": Unit.frames, "s": Unit.seconds}[rpe_cfg.get("unit", "m")]
    out = {"ate_rmse_m": float(ape.get_statistic(metrics.StatisticsType.rmse)), "gt_matched": n,
           "gt_coverage": n / max(len(gt), 1)}
    for rel, key, conv in [(metrics.PoseRelation.translation_part, "rpe_trans_m", 1.0),
                           (metrics.PoseRelation.rotation_angle_deg, "rpe_rot_deg", 1.0)]:
        try:
            rpe = metrics.RPE(rel, delta=float(rpe_cfg["delta"]), delta_unit=unit, all_pairs=False)
            rpe.process_data((t_ref, t_est))
            out[key] = float(rpe.get_statistic(metrics.StatisticsType.rmse)) * conv
        except Exception:  # sequence shorter than the RPE delta
            out[key] = None
    return out


def path_length(traj: np.ndarray) -> float:
    if len(traj) < 2:
        return 0.0
    p = traj[:, 1:4]
    p = p[np.all(np.isfinite(p), axis=1)]
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def finite_or_none(x):
    return None if x is None or (isinstance(x, float) and not math.isfinite(x)) else x
