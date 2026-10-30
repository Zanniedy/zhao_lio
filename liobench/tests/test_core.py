import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from liobench import metrics as M
from liobench import report as R
from liobench.adapter import build_command, render_config, sensor_context
from liobench.config import ROOT, Algorithm, RunSpec, Sequence, expand_experiment, load_algorithm, param_grid


def test_param_grid():
    assert param_grid({}) == [{}]
    g = param_grid({"a": [1, 2], "b": 3})
    assert g == [{"a": 1, "b": 3}, {"a": 2, "b": 3}]
    g = param_grid({"t": [[0, 0, 0], [1, 0, 0]]})  # nested list sweeps over list values
    assert g == [{"t": [0, 0, 0]}, {"t": [1, 0, 0]}]


def test_expand_experiment_counts():
    exp = {"algorithms": ["point_lio", {"name": "zhao_lio", "params": {"update_strategy": ["pointwise", "pointwise_prematch"]}}],
           "sequences": ["narrowwide/tracked-01"], "repeats": 2}
    runs = expand_experiment(exp)
    assert len(runs) == (1 + 2) * 2
    assert len({r.key for r in runs}) == len(runs)


def make_run(algo_name, params=None, tmp=None):
    seq = Sequence("narrowwide", "tracked-01", Path("/nonexistent"), None, {
        "lidar_topic": "/livox/lidar", "lidar_type": "livox_custom", "imu_topic": "/imu", "acc_norm": 9.81,
        "satu_acc": 150.0, "satu_gyro": 35.0, "blind": 0.5, "scan_lines": 4,
        "extrinsic_t": [1, 2, 3], "extrinsic_r": [0, -1, 0, 1, 0, 0, 0, 0, 1]})
    return RunSpec(load_algorithm(algo_name), seq, params or {}, 0, {})


@pytest.mark.parametrize("name", ["point_lio", "batch_lio", "small_point_lio", "super_lio", "genz_lio", "zhao_lio"])
def test_render_all_algorithms(name, tmp_path):
    run = make_run(name)
    path, applied = render_config(run, tmp_path)
    doc = yaml.safe_load(path.read_text())
    assert doc
    cmd = build_command(run, path)
    assert cmd.startswith("exec ")
    assert str(path) in cmd


def test_point_lio_overrides(tmp_path):
    run = make_run("point_lio", {"filter_size_surf": 0.3})
    path, _ = render_config(run, tmp_path)
    p = yaml.safe_load(path.read_text())["/**"]["ros__parameters"]
    assert p["common.imu_topic"] == "/imu"
    assert p["preprocess.lidar_type"] == 1
    assert p["mapping.acc_norm"] == 9.81
    assert p["mapping.extrinsic_T"] == [1.0, 2.0, 3.0]
    assert p["filter_size_surf"] == 0.3
    assert "common" not in p  # flattened, no duplicate nested copy


def test_super_lio_colmajor(tmp_path):
    run = make_run("super_lio")
    path, _ = render_config(run, tmp_path)
    p = yaml.safe_load(path.read_text())["/**"]["ros__parameters"]
    # row-major [0,-1,0, 1,0,0, 0,0,1] -> column-major [0,1,0, -1,0,0, 0,0,1]
    assert p["lio.extrinsic.lidar_imu"] == [1.0, 2.0, 3.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0]


def test_genz_yaml_file(tmp_path):
    run = make_run("genz_lio")
    path, _ = render_config(run, tmp_path)
    doc = yaml.safe_load(path.read_text())
    assert doc["common"]["imu_topic"] == "/imu"
    assert doc["preprocess"]["lidar_type"] == "livox"
    assert "-p config_path:=" in build_command(run, path)


def test_zhao_lio_timing_param(tmp_path):
    run = make_run("zhao_lio", {"update_strategy": "pointwise"})
    path, _ = render_config(run, tmp_path)
    p = yaml.safe_load(path.read_text())["/**"]["ros__parameters"]
    assert p["timing_csv"] == str(tmp_path / "internal_timing.csv")
    assert p["update_strategy"] == "pointwise"


def test_unknown_lidar_type():
    run = make_run("point_lio")
    run.sequence.sensor["lidar_type"] = "velodyne"
    with pytest.raises(ValueError):
        sensor_context(run.sequence.sensor, run.algorithm)


# ---------------------------------------------------------------- metrics


def test_latency_per_scan():
    ls = np.arange(10) * 0.1                   # scan start stamps
    lr = 100 + ls + 0.1                        # arrives at scan end
    os_ = ls + 0.1                             # odom stamped at scan end
    orc = lr + 0.005                           # 5 ms processing
    lat = M.frame_latency({"stamp": ls, "recv_time": lr}, {"stamp": os_, "recv_time": orc})
    assert len(lat) == 10
    assert np.allclose(lat, 0.005)


def test_latency_high_rate_odom():
    ls = np.arange(5) * 0.1
    lr = 100 + ls + 0.1
    # odom at 200 Hz, delivered 3 ms after the scan it completes arrives
    os_ = np.arange(0, 0.5, 0.005) + 0.005
    orc = 100 + np.ceil((os_ - 1e-9) / 0.1) * 0.1 + 0.003
    lat = M.frame_latency({"stamp": ls, "recv_time": lr}, {"stamp": os_, "recv_time": orc})
    assert len(lat) == 5
    assert np.allclose(lat, 0.003, atol=1e-6)


def test_latency_missing_frames():
    ls = np.arange(10) * 0.1
    lr = 100 + ls + 0.1
    lat = M.frame_latency({"stamp": ls, "recv_time": lr}, {"stamp": ls[:5] + 0.1, "recv_time": lr[:5] + 0.01})
    assert len(lat) == 5


def circle(n=600, r=10.0, noise=0.0, seed=0):
    t = np.linspace(0, 60, n)
    a = t / 60 * 2 * np.pi
    rng = np.random.default_rng(seed)
    pos = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(a)], 1) + rng.normal(0, noise, (n, 3))
    yaw = a + np.pi / 2
    q = np.stack([np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2), np.cos(yaw / 2)], 1)
    return np.concatenate([t[:, None], pos, q], 1)


def transform(traj, yaw, t):
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    out = traj.copy()
    out[:, 1:4] = traj[:, 1:4] @ Rz.T + t
    half = yaw / 2
    qz, qw = np.sin(half), np.cos(half)
    x, y, z, w = traj[:, 4], traj[:, 5], traj[:, 6], traj[:, 7]
    out[:, 4] = qw * x - qz * y
    out[:, 5] = qw * y + qz * x
    out[:, 6] = qw * z + qz * w
    out[:, 7] = qw * w - qz * z
    return out


def test_ate_zero_after_alignment():
    gt = circle()
    est = transform(gt, 0.7, np.array([5.0, -3.0, 1.0]))
    err = M.trajectory_errors(est, gt, {"delta": 5, "unit": "m"})
    assert err["ate_rmse_m"] < 1e-6
    assert err["rpe_trans_m"] < 1e-6
    assert err["rpe_rot_deg"] < 1e-4


def test_ate_detects_noise():
    gt = circle()
    est = circle(noise=0.1, seed=1)
    err = M.trajectory_errors(est, gt, {"delta": 5, "unit": "m"})
    assert 0.12 < err["ate_rmse_m"] < 0.22   # sqrt(3) * 0.1 ≈ 0.17


def test_divergence():
    gt = circle()
    crit = {"max_speed": 20, "max_jump": 5, "min_coverage": 0.9}
    assert M.divergence_checks(gt, 600, 600, crit) == []
    bad = gt.copy()
    bad[300:, 1] += 50
    assert any("跳变" in r for r in M.divergence_checks(bad, 600, 600, crit))
    bad = gt.copy()
    bad[10, 2] = np.nan
    assert any("NaN" in r for r in M.divergence_checks(bad, 600, 600, crit))
    assert any("有输出的帧比例" in r for r in M.divergence_checks(gt, 600, 100, crit))
    assert M.divergence_checks(np.zeros((0, 8)), 600, 0, crit) == ["没有里程计输出"]


def test_odom_to_tum_sorted_unique():
    odom = {"stamp": np.array([2.0, 1.0, 2.0]), "x": np.array([1, 2, 3.0]), "y": np.zeros(3), "z": np.zeros(3),
            "qx": np.zeros(3), "qy": np.zeros(3), "qz": np.zeros(3), "qw": np.ones(3)}
    t = M.odom_to_tum(odom)
    assert list(t[:, 0]) == [1.0, 2.0]
    assert t[1, 1] == 3.0  # last message for a duplicated stamp


def test_read_csv_partial_line(tmp_path):
    p = tmp_path / "x.csv"
    p.write_text("a,b\n1,2\n3,4\n5\n")
    d = M.read_csv(p)
    assert list(d["a"]) == [1, 3]


def test_resource_summary():
    res = {"t": np.array([0, 1, 2, 3.0]), "cpu_pct": np.array([10, 100, 200, 10.0]),
           "rss_mb": np.array([5, 50, 60, 55.0]), "hwm_mb": np.array([5, 50, 70, 70.0])}
    s = M.resource_summary(res, 1, 2)
    assert s["cpu_mean_pct"] == 150
    assert s["peak_rss_mb"] == 70


# ---------------------------------------------------------------- report


def fake_run(root, algo, seq, rep, status="ok", ate=1.0, gt=True):
    d = root / algo / seq / f"r{rep}"
    d.mkdir(parents=True)
    r = {"algorithm": algo, "variant": "default", "sequence": seq, "repeat": rep, "status": status,
         "reasons": [] if status == "ok" else ["速度 99 m/s > 20 m/s"], "has_gt": gt,
         "ate_rmse_m": ate if gt else None, "latency_mean_ms": 10.0 + rep, "peak_rss_mb": 100.0}
    (d / "run.json").write_text(json.dumps(r))


def test_report_and_compare(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    fake_run(a, "x", "ds/s1", 0)
    fake_run(a, "x", "ds/s1", 1)
    fake_run(a, "x", "ds/s2", 0, status="diverged")
    fake_run(a, "y", "ds/s1", 0, gt=False)
    text = R.report(a)
    assert "无真值" in text
    assert "67%" in text           # x: 2 of 3 ok
    assert (a / "summary.md").exists() and (a / "summary.csv").exists()
    rows = {(r["algorithm"], r["sequence"]): r for r in R.aggregate(R.load_runs(a))}
    assert rows[("x", "ds/s1")]["latency_mean_ms"] == 10.5
    assert rows[("x", "ds/s1")]["latency_mean_ms_std"] == 0.5
    fake_run(b, "x", "ds/s1", 0, ate=0.5)
    out = R.compare(a, b)
    assert "-0.500" in out and "(-50%)" in out
