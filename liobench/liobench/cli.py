"""liobench command line.

    liobench ls                                  algorithms, sequences, bag/GT availability
    liobench run experiments/baseline.yaml       run (skips runs that already finished)
    liobench report results/baseline             tables + summary.md/csv
    liobench reeval results/baseline             recompute metrics (e.g. after adding GT)
    liobench compare results/v1 results/v2       metric deltas, for regression checks
    liobench convert narrowwide [--seq tracked-01] convert source bags/GT to the canonical format
    liobench convert --list                      available lidar/imu/gt decoders
    liobench dev dev/zhao_lio.yaml --note "..." build + test + compare + gate one version of your LIO
    liobench history dev/zhao_lio.yaml          all versions and their metrics
    liobench restore dev/zhao_lio.yaml LABEL DIR  extract a version's source snapshot
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from . import report as R
from .config import ROOT, RunSpec, deep_merge as R_merge, expand_experiment, load_algorithm, load_dataset, load_sequence, load_yaml
from .runner import CRASH_PREFIX, evaluate, finish, run_one


def cmd_ls(_a):
    print("算法：")
    for p in sorted((ROOT / "algorithms").glob("*.yaml")):
        spec = load_yaml(p)
        print(f"  {p.stem:18s} {spec['package']}/{spec['executable']}  odom={spec['odom_topic']}")
    print("序列：")
    for p in sorted((ROOT / "datasets").glob("*.yaml")):
        ds = load_dataset(p.stem)
        for s in ds.get("sequences", {}):
            seq = load_sequence(f"{p.stem}/{s}")
            if "output" in ds:
                state = "已转换" if seq.converted else "未转换"
            else:
                state = "数据包在" if seq.bag.exists() else "无数据包"
            gt = "有真值" if seq.gt and seq.gt.exists() else "无真值"
            print(f"  {seq.id:28s} {state}  {gt}  {seq.bag}")


STATUS_ZH = {"ok": "成功", "diverged": "发散", "failed": "失败"}


def run_label(run: RunSpec) -> str:
    variant = "默认" if run.variant == "default" else run.variant
    return f"{run.algorithm.name} [{variant}] {run.sequence.id} 第{run.repeat + 1}次"


def cmd_run(a):
    exp_path = Path(a.experiment)
    exp = load_yaml(exp_path)
    name = a.name or exp.get("name") or exp_path.stem
    out = Path(a.output) if a.output else ROOT / "results" / name
    runs = expand_experiment(exp)
    if a.only:
        runs = [r for r in runs if r.algorithm.name in a.only]
    if a.rate:
        for r in runs:
            r.settings["rate"] = a.rate
    if a.duration:
        for r in runs:
            r.settings["duration"] = a.duration
    out.mkdir(parents=True, exist_ok=True)
    (out / "experiment.yaml").write_text(exp_path.read_text())
    print(f"共 {len(runs)} 次运行 -> {out}")
    if a.dry_run:
        for r in runs:
            print("  " + run_label(r))
        return
    execute_runs(runs, out, a.force, a.rerun_failed, a.domain_id)
    print(R.report(out))


def execute_runs(runs: list[RunSpec], out: Path, force: bool, rerun_failed: bool, domain_id: int) -> None:
    """Run each RunSpec into out/<key>, skipping finished runs."""
    out.mkdir(parents=True, exist_ok=True)
    t_start = time.monotonic()
    for i, run in enumerate(runs, 1):
        run_dir = out / run.key
        done = run_dir / "run.json"
        if done.exists() and not force:
            prev = json.loads(done.read_text())
            if prev.get("status") != "failed" or not rerun_failed:
                print(f"[{i}/{len(runs)}] 跳过（已完成：{STATUS_ZH.get(prev.get('status'), prev.get('status'))}）{run_label(run)}")
                continue
        print(f"[{i}/{len(runs)}] {run_label(run)} ...", flush=True)
        t0 = time.monotonic()
        res = run_one(run, run_dir, domain_id=domain_id)
        msg = f"    {STATUS_ZH.get(res['status'], res['status'])}，用时 {time.monotonic() - t0:.0f} s"
        if res.get("ate_rmse_m") is not None:
            msg += f"，ATE {res['ate_rmse_m']:.3f} m"
        if res.get("latency_mean_ms") is not None:
            msg += f"，延迟 {res['latency_mean_ms']:.1f} ms"
        if res["reasons"]:
            msg += " | " + "；".join(res["reasons"])
        print(msg, flush=True)
    print(f"\n全部运行完成，用时 {(time.monotonic() - t_start) / 60:.1f} 分钟\n")


def cmd_report(a):
    print(R.report(Path(a.results)))


def cmd_reeval(a):
    root = Path(a.results)
    for p in sorted(root.rglob("run.json")):
        r = json.loads(p.read_text())
        if "settings" not in r or r.get("status") == "failed":
            continue
        run = RunSpec(load_algorithm(r["algorithm"]), load_sequence(r["sequence"]), r["params"], r["repeat"],
                      r["settings"])
        base_reasons = [x for x in r.get("reasons", []) if x.startswith(CRASH_PREFIX)]
        r.update(evaluate(run, p.parent, None, None, base_reasons))
        finish(r, p.parent)
        print(f"{STATUS_ZH.get(r['status'], r['status'])}  {p.parent.relative_to(root)}")
    print(R.report(root))


def cmd_compare(a):
    print(R.compare(Path(a.base), Path(a.new)))


def cmd_convert(a):
    from .config import converted_dir
    from .convert import convert_sequence, list_converters
    if a.list or not a.dataset:
        kinds = {"lidar": "雷达解码器", "imu": "IMU 解码器", "gt": "真值读取器"}
        for kind, items in list_converters().items():
            print(f"{kinds[kind]}（{kind}）：")
            for name, doc in items:
                print(f"  {name:18s} {doc}")
        return
    ds = load_dataset(a.dataset)
    if "output" not in ds or "source" not in ds:
        sys.exit(f"数据集 '{a.dataset}' 没有 `output` / `source` 配置，无法转换")
    failed = 0
    for name, s in ds["sequences"].items():
        if a.seq and name not in a.seq:
            continue
        s = s or {}
        source = R_merge(ds["source"], s.get("source") or {})
        try:
            convert_sequence(name, s, source, converted_dir(ds, name), force=a.force)
        except (FileNotFoundError, KeyError, ValueError) as e:
            failed += 1
            print(f"{name}：转换失败：{e}")
    if failed:
        sys.exit(1)


def cmd_dev(a):
    from .dev import dev
    sys.exit(dev(Path(a.config), a.label, a.note, a.skip_build, a.duration))


def cmd_history(a):
    from .dev import history
    history(Path(a.config))


def cmd_restore(a):
    from .dev import restore
    restore(Path(a.config), a.label, Path(a.dst))


def main(argv=None):
    p = argparse.ArgumentParser(prog="liobench", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ls").set_defaults(func=cmd_ls)
    r = sub.add_parser("run")
    r.add_argument("experiment")
    r.add_argument("--name", help="results/<name> (default: experiment name)")
    r.add_argument("--output", help="explicit results directory")
    r.add_argument("--only", nargs="+", help="run only these algorithms")
    r.add_argument("--rate", type=float, help="override playback rate")
    r.add_argument("--duration", type=float, help="play only the first N seconds of each bag (smoke tests)")
    r.add_argument("--force", action="store_true", help="rerun finished runs too")
    r.add_argument("--rerun-failed", action="store_true", help="rerun runs whose status is 'failed'")
    r.add_argument("--dry-run", action="store_true")
    r.add_argument("--domain-id", type=int, default=77, help="ROS_DOMAIN_ID used for the runs")
    r.set_defaults(func=cmd_run)
    s = sub.add_parser("report")
    s.add_argument("results")
    s.set_defaults(func=cmd_report)
    e = sub.add_parser("reeval")
    e.add_argument("results")
    e.set_defaults(func=cmd_reeval)
    c = sub.add_parser("compare")
    c.add_argument("base")
    c.add_argument("new")
    c.set_defaults(func=cmd_compare)
    v = sub.add_parser("convert", help="convert a dataset to the canonical format")
    v.add_argument("dataset", nargs="?")
    v.add_argument("--seq", nargs="+", help="only these sequences")
    v.add_argument("--force", action="store_true", help="reconvert even if up to date")
    v.add_argument("--list", action="store_true", help="list available decoders")
    v.set_defaults(func=cmd_convert)
    d = sub.add_parser("dev", help="build, test and compare one version of an algorithm under development")
    d.add_argument("config", help="dev/<name>.yaml")
    d.add_argument("--label", help="version name (default: v<date>-<time>)")
    d.add_argument("--note", default="", help="what changed in this version")
    d.add_argument("--skip-build", action="store_true")
    d.add_argument("--duration", type=float, help="play only the first N seconds (quick check)")
    d.set_defaults(func=cmd_dev)
    h = sub.add_parser("history", help="metrics of all versions")
    h.add_argument("config")
    h.set_defaults(func=cmd_history)
    rs = sub.add_parser("restore", help="extract the source snapshot of a version")
    rs.add_argument("config")
    rs.add_argument("label")
    rs.add_argument("dst")
    rs.set_defaults(func=cmd_restore)
    a = p.parse_args(argv)
    a.func(a)


if __name__ == "__main__":
    sys.exit(main())
