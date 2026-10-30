"""Development loop for one algorithm under active development.

    liobench dev dev/zhao_lio.yaml --label rematch-0.05 --note "lower rematch_thr"
    liobench history dev/zhao_lio.yaml
    liobench restore dev/zhao_lio.yaml rematch-0.05 /tmp/zhao_lio_src

Each `dev` call: build -> snapshot sources -> run the experiment -> compare with the
reference version -> check the gates -> append to history.csv. Exit code 1 if the
build fails or a gate fails, so it can be used from scripts or CI.
"""
from __future__ import annotations

import csv
import hashlib
import json
import subprocess
import tarfile
import time
from pathlib import Path

from . import report as R
from .config import ROOT, expand_experiment, expand_path, load_yaml
from .runner import clean_env

METRIC_ZH = {
    "success_rate": "成功率", "ate_rmse_m": "ATE(m)", "rpe_trans_m": "RPE(m)", "rpe_rot_deg": "RPE(°)",
    "latency_mean_ms": "延迟均值(ms)", "latency_p95_ms": "延迟P95(ms)", "internal_mean_ms": "内部均值(ms)",
    "internal_p95_ms": "内部P95(ms)", "cpu_mean_pct": "CPU(%)", "peak_rss_mb": "内存(MB)",
}
HISTORY_METRICS = ["ate_rmse_m", "rpe_trans_m", "rpe_rot_deg", "latency_mean_ms", "latency_p95_ms",
                   "internal_mean_ms", "internal_p95_ms", "cpu_mean_pct", "peak_rss_mb"]


def resolve(p: str) -> Path:
    q = expand_path(p)
    return q if q.is_absolute() else (ROOT / q).resolve()


def load_dev(path: Path) -> dict:
    cfg = load_yaml(path)
    cfg.setdefault("name", path.stem)
    cfg.setdefault("reference", "previous")
    cfg.setdefault("primary", "ate_rmse_m")
    cfg.setdefault("gates", {})
    cfg["_root"] = ROOT / "results" / "dev" / cfg["name"]
    return cfg


def source_files(cfg: dict) -> list[Path]:
    src = resolve(cfg["source_dir"])
    exts = tuple(cfg.get("source_exts", [".h", ".hpp", ".cpp", ".cc", ".c", ".txt", ".yaml", ".py", ".xml"]))
    return sorted(p for p in src.rglob("*") if p.is_file() and (p.suffix in exts or p.name == "CMakeLists.txt"))


def source_hash(cfg: dict) -> str:
    h = hashlib.sha1()
    src = resolve(cfg["source_dir"])
    for p in source_files(cfg):
        h.update(str(p.relative_to(src)).encode())
        h.update(p.read_bytes())
    return h.hexdigest()[:10]


def snapshot(cfg: dict, dst: Path) -> None:
    src = resolve(cfg["source_dir"])
    with tarfile.open(dst, "w:gz") as tar:
        for p in source_files(cfg):
            tar.add(p, arcname=str(p.relative_to(src)))


# ---------------------------------------------------------------- history


def read_history(root: Path) -> list[dict]:
    path = root / "history.csv"
    if not path.exists():
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def append_history(root: Path, rows: list[dict]) -> None:
    path = root / "history.csv"
    old = read_history(root)
    keys = ["version", "time", "variant", "source_hash", "duration", "passed", "note", "sequences", "ok", "runs",
            "success_rate"] + HISTORY_METRICS
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in old + rows:
            w.writerow(r)


def version_summary(results: Path) -> dict[str, dict]:
    """Per variant: metrics averaged over sequences, plus the success rate."""
    runs = R.load_runs(results)
    rows = R.aggregate(runs)
    tot = {r["variant"]: r for r in R.overall(runs)}
    out = {}
    for variant, t in tot.items():
        mine = [r for r in rows if r["variant"] == variant]
        s = {"variant": variant, "sequences": t["sequences"], "ok": t["ok"], "runs": t["runs"],
             "success_rate": round(t["success_rate"], 4)}
        for m in HISTORY_METRICS:
            vals = [r[m] for r in mine if r.get(m) is not None]
            # A mean over sequences is only comparable if every sequence has the metric.
            s[m] = round(sum(vals) / len(vals), 6) if vals and len(vals) == len(mine) else None
        out[variant] = s
    return out


def pick_reference(cfg: dict, history: list[dict], current: str, duration: str = "") -> str | None:
    """previous / best only consider versions run with the same --duration ("" = full length),
    because a 60 s quick check is not comparable with a full run."""
    ref = cfg["reference"]
    history = [h for h in history if h["version"] == current or h.get("duration", "") == duration]
    versions = [h["version"] for h in history if h["version"] != current]
    if not versions:
        return None
    if ref == "previous":
        return versions[-1]
    if ref == "best":
        m = cfg["primary"]
        cands = [h for h in history if h["version"] != current and h["passed"] == "True" and h.get(m)]
        if not cands:
            return versions[-1]
        return min(cands, key=lambda h: float(h[m]))["version"]
    return ref  # explicit version label


# ---------------------------------------------------------------- gates


def check_gates(gates: dict, cur: dict, ref: dict | None) -> list[tuple[str, str, bool | None, str]]:
    """-> [(variant, metric, passed or None if not checkable, explanation)].
    Gate keys per metric: max, min (absolute) and max_increase_pct, max_decrease_pct (vs reference)."""
    out = []
    for variant, c in sorted(cur.items()):
        r = (ref or {}).get(variant)
        for metric, g in gates.items():
            v = c.get(metric)
            if v is None:
                out.append((variant, metric, None, "没有数据（比如没有真值）"))
                continue
            v = float(v)
            if "max" in g and v > g["max"]:
                out.append((variant, metric, False, f"{v:.4g} > 上限 {g['max']}"))
                continue
            if "min" in g and v < g["min"]:
                out.append((variant, metric, False, f"{v:.4g} < 下限 {g['min']}"))
                continue
            rv = r.get(metric) if r else None
            limits = "，".join(f"{'上限' if k == 'max' else '下限'} {g[k]}" for k in ("max", "min") if k in g)
            relative = "max_increase_pct" in g or "max_decrease_pct" in g
            if not relative:
                out.append((variant, metric, True, f"{v:.4g}（{limits}）"))
                continue
            if rv in (None, ""):
                out.append((variant, metric, True, f"{v:.4g}（参考版本没有这项，只检查绝对限值）"))
                continue
            rv = float(rv)
            pct = 100.0 * (v - rv) / rv if rv else 0.0
            if "max_increase_pct" in g and pct > g["max_increase_pct"]:
                out.append((variant, metric, False, f"{rv:.4g} -> {v:.4g} ({pct:+.1f}% > +{g['max_increase_pct']}%)"))
            elif "max_decrease_pct" in g and -pct > g["max_decrease_pct"]:
                out.append((variant, metric, False, f"{rv:.4g} -> {v:.4g} ({pct:+.1f}% < -{g['max_decrease_pct']}%)"))
            else:
                out.append((variant, metric, True, f"{rv:.4g} -> {v:.4g} ({pct:+.1f}%)"))
    return out


# ---------------------------------------------------------------- commands


def run_build(cfg: dict, log: Path) -> bool:
    cmd = cfg.get("build")
    if not cmd:
        return True
    cwd = resolve(cfg.get("build_dir", "."))
    env = clean_env(int(cfg.get("domain_id", 77)))
    shell = f"source /opt/ros/humble/setup.bash && {cmd}"
    print(f"编译：{cmd}（目录 {cwd}）", flush=True)
    t0 = time.monotonic()
    with open(log, "w") as f:
        p = subprocess.run(["bash", "-c", shell], cwd=cwd, env=env, stdout=f, stderr=subprocess.STDOUT)
    print(f"编译{'成功' if p.returncode == 0 else '失败'}，用时 {time.monotonic() - t0:.0f} s，日志：{log}")
    return p.returncode == 0


def dev(cfg_path: Path, label: str | None, note: str, skip_build: bool, duration: float | None) -> int:
    from .cli import execute_runs  # shared with `liobench run`

    cfg = load_dev(cfg_path)
    root = cfg["_root"]
    root.mkdir(parents=True, exist_ok=True)
    shash = source_hash(cfg)
    label = label or time.strftime("v%Y%m%d-%H%M%S")
    vdir = root / label
    if (vdir / "version.json").exists():
        print(f"版本 '{label}' 已存在于 {root}，请换一个 --label")
        return 1
    vdir.mkdir(parents=True, exist_ok=True)
    history = read_history(root)
    same = [h["version"] for h in history if h["source_hash"] == shash]
    if same:
        print(f"提示：源码和版本 {'、'.join(sorted(set(same)))} 完全相同")

    if not skip_build and not run_build(cfg, vdir / "build.log"):
        return 1
    snapshot(cfg, vdir / "source.tar.gz")

    exp_path = resolve(cfg["experiment"])
    exp = load_yaml(exp_path)
    if duration:
        exp["duration"] = duration
    runs = expand_experiment(exp)
    (vdir / "experiment.yaml").write_text(exp_path.read_text())
    execute_runs(runs, vdir / "runs", force=False, rerun_failed=False, domain_id=int(cfg.get("domain_id", 77)))

    cur = version_summary(vdir / "runs")
    dur = f"{duration:g}" if duration else ""
    ref_label = pick_reference(cfg, history + [{"version": label, "duration": dur}], label, dur)
    ref = None
    if ref_label:
        ref_dir = root / ref_label / "runs"
        if ref_dir.exists():
            ref = version_summary(ref_dir)
        else:
            print(f"在 {root} 里找不到参考版本 '{ref_label}'，门槛只检查绝对上下限")
            ref_label = None

    span = f"每个包只播前 {dur} s" if dur else "全长"
    print(f"\n=== {cfg['name']} {label}（源码 {shash}，{span}）对比 {ref_label or '无（没有同样播放时长的历史版本）'} ===\n")
    if ref_label:
        print(R.compare(root / ref_label / "runs", vdir / "runs"))
        print()
    gates = check_gates(cfg["gates"], cur, ref)
    passed = all(ok is not False for _, _, ok, _ in gates)
    for variant, metric, ok, why in gates:
        mark = {True: "通过", False: "不通过", None: "跳过"}[ok]
        print(f"  [{mark}] {variant}  {METRIC_ZH.get(metric, metric)}：{why}")
    print(f"\n结论：{'通过' if passed else '不通过'}")

    rows = [{"version": label, "time": time.strftime("%Y-%m-%d %H:%M"), "source_hash": shash, "duration": dur,
             "passed": passed, "note": note, **s} for s in cur.values()]
    append_history(root, rows)
    (vdir / "version.json").write_text(json.dumps(
        {"version": label, "source_hash": shash, "duration": dur, "note": note, "reference": ref_label, "passed": passed,
         "gates": [dict(variant=v, metric=m, passed=o, detail=w) for v, m, o, w in gates], "summary": cur},
        indent=2))
    print(f"历史记录：{root / 'history.csv'}（查看：liobench history {cfg_path}）")
    return 0 if passed else 1


def history(cfg_path: Path) -> None:
    cfg = load_dev(cfg_path)
    rows = read_history(cfg["_root"])
    if not rows:
        print(f"{cfg['_root']} 里还没有版本")
        return
    metrics = [m for m in HISTORY_METRICS if any(r.get(m) for r in rows)]
    header = ["版本", "时间", "配置", "播放", "通过", "成功/次数"] + [METRIC_ZH.get(m, m) for m in metrics] + ["说明"]
    body = []
    for r in rows:
        cells = [r["version"], r["time"], r["variant"], f"{r['duration']} s" if r.get("duration") else "全长",
                 "是" if r["passed"] == "True" else "否",
                 f"{r['ok']}/{r['runs']}"]
        cells += [f"{float(r[m]):.3f}" if r.get(m) else "-" for m in metrics]
        body.append(cells + [r.get("note", "")])
    print("\n".join(R._render(header, body, md=False, left={0, 1, 2, 3, len(header) - 1})))
    m = cfg["primary"]
    best = [r for r in rows if r["passed"] == "True" and r.get(m) and not r.get("duration")]
    if best:
        b = min(best, key=lambda r: float(r[m]))
        print(f"\n按 {METRIC_ZH.get(m, m)} 最好的通过版本（全长运行）：{b['version']} [{b['variant']}] = {float(b[m]):.4f}")


def restore(cfg_path: Path, label: str, dst: Path) -> None:
    cfg = load_dev(cfg_path)
    tar = cfg["_root"] / label / "source.tar.gz"
    if not tar.exists():
        raise SystemExit(f"版本 '{label}' 没有源码快照（{tar}）")
    if dst.exists() and any(dst.iterdir()):
        raise SystemExit(f"{dst} 不是空目录。请恢复到一个新目录，再从那里对比或拷贝")
    dst.mkdir(parents=True, exist_ok=True)
    with tarfile.open(tar) as t:
        t.extractall(dst, filter="data")
    print(f"已把版本 {label} 的源码恢复到 {dst}")
