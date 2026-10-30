"""Aggregate run.json files into tables (terminal, CSV, Markdown).

Tables and summary.md are in Chinese; CSV files keep the English run.json keys
so scripts can read them."""
from __future__ import annotations

import csv
import json
import unicodedata
from collections import defaultdict
from pathlib import Path

import numpy as np

# (key in run.json, column header, format)
COLUMNS = [
    ("ate_rmse_m", "ATE(m)", "{:.3f}"),
    ("rpe_trans_m", "RPE(m)", "{:.3f}"),
    ("rpe_rot_deg", "RPE(°)", "{:.3f}"),
    ("latency_mean_ms", "延迟均值(ms)", "{:.2f}"),
    ("latency_p95_ms", "延迟P95(ms)", "{:.2f}"),
    ("internal_mean_ms", "内部均值(ms)", "{:.2f}"),
    ("internal_p95_ms", "内部P95(ms)", "{:.2f}"),
    ("cpu_mean_pct", "CPU(%)", "{:.0f}"),
    ("peak_rss_mb", "内存(MB)", "{:.0f}"),
    ("path_length_m", "轨迹长(m)", "{:.1f}"),
]


def load_runs(root: Path) -> list[dict]:
    runs = []
    for p in sorted(root.rglob("run.json")):
        with open(p) as f:
            r = json.load(f)
        r["_dir"] = str(p.parent)
        runs.append(r)
    return runs


def _num(v):
    return v if isinstance(v, (int, float)) and not isinstance(v, bool) and np.isfinite(v) else None


def aggregate(runs: list[dict]) -> list[dict]:
    """One row per (algorithm, variant, sequence); metrics averaged over successful repeats."""
    groups = defaultdict(list)
    for r in runs:
        groups[(r["algorithm"], r["variant"], r["sequence"])].append(r)
    rows = []
    for (algo, variant, seq), rs in sorted(groups.items()):
        ok = [r for r in rs if r.get("status") == "ok"]
        row = {"algorithm": algo, "variant": variant, "sequence": seq, "runs": len(rs), "ok": len(ok),
               "has_gt": any(r.get("has_gt") for r in rs)}
        for key, _, _ in COLUMNS:
            vals = [_num(r.get(key)) for r in ok]
            vals = [v for v in vals if v is not None]
            row[key] = float(np.mean(vals)) if vals else None
            if len(vals) > 1:
                row[key + "_std"] = float(np.std(vals))
        reasons = sorted({x for r in rs if r.get("status") != "ok" for x in r.get("reasons", [])})
        row["failures"] = "；".join(reasons)
        rows.append(row)
    return rows


def overall(runs: list[dict]) -> list[dict]:
    """Success rate per (algorithm, variant) across all sequences and repeats."""
    groups = defaultdict(list)
    for r in runs:
        groups[(r["algorithm"], r["variant"])].append(r)
    out = []
    for (algo, variant), rs in sorted(groups.items()):
        n_ok = sum(r.get("status") == "ok" for r in rs)
        out.append({"algorithm": algo, "variant": variant, "runs": len(rs), "ok": n_ok,
                    "success_rate": n_ok / len(rs), "divergence_rate": 1 - n_ok / len(rs),
                    "sequences": len({r["sequence"] for r in rs})})
    return out


def vname(variant: str) -> str:
    return "默认" if variant == "default" else variant


def fmt(row: dict, key: str, f: str) -> str:
    v = row.get(key)
    if v is None:
        if key.startswith(("ate", "rpe")) and not row.get("has_gt", True):
            return "无真值"
        return "-"
    s = f.format(v)
    if row.get(key + "_std") is not None:
        s += "±" + f.format(row[key + "_std"])
    return s


def table_lines(rows: list[dict], md: bool = False) -> list[str]:
    cols = [k for k, _, _ in COLUMNS if any(r.get(k) is not None for r in rows)]
    if any(not r.get("has_gt") for r in rows):  # keep ATE/RPE columns so "无真值" is visible
        cols = [k for k, _, _ in COLUMNS[:3]] + [c for c in cols if c not in ("ate_rmse_m", "rpe_trans_m", "rpe_rot_deg")]
    meta = {k: (h, f) for k, h, f in COLUMNS}
    header = ["算法", "配置", "序列", "成功/次数"] + [meta[c][0] for c in cols]
    body = [[r["algorithm"], vname(r["variant"]), r["sequence"], f"{r['ok']}/{r['runs']}"]
            + [fmt(r, c, meta[c][1]) for c in cols] for r in rows]
    return _render(header, body, md)


def overall_lines(rows: list[dict], md: bool = False) -> list[str]:
    header = ["算法", "配置", "序列数", "成功/次数", "成功率", "发散率"]
    body = [[r["algorithm"], vname(r["variant"]), str(r["sequences"]), f"{r['ok']}/{r['runs']}",
             f"{100 * r['success_rate']:.0f}%", f"{100 * r['divergence_rate']:.0f}%"] for r in rows]
    return _render(header, body, md)


def _render(header: list[str], body: list[list[str]], md: bool, left: set[int] | None = None) -> list[str]:
    """Text table; columns in `left` are left-aligned (default: the first three)."""
    left = {0, 1, 2} if left is None else left
    if md:
        lines = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
        lines += ["| " + " | ".join(r) + " |" for r in body]
        return lines
    w = [max(width(str(x)) for x in col) for col in zip(header, *body)]

    def cell(x, n, right):
        pad = " " * (n - width(str(x)))
        return pad + str(x) if right else str(x) + pad

    line = lambda r: "  ".join(cell(x, n, i not in left) for i, (x, n) in enumerate(zip(r, w)))
    return [line(header), "  ".join("-" * n for n in w)] + [line(r) for r in body]


def width(s: str) -> int:
    """Terminal display width: CJK characters take two columns."""
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)


def write_csv(path: Path, rows: list[dict]) -> None:
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys and not k.startswith("_")]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: (json.dumps(v) if isinstance(v, (dict, list)) else v) for k, v in r.items()})


def report(root: Path, quiet: bool = False) -> str:
    runs = load_runs(root)
    if not runs:
        return f"{root} 下没有 run.json"
    rows, tot = aggregate(runs), overall(runs)
    write_csv(root / "runs.csv", runs)
    write_csv(root / "summary.csv", rows)
    write_csv(root / "overall.csv", tot)
    failures = [f"- {r['algorithm']} [{vname(r['variant'])}] {r['sequence']}：{r['failures']}"
                for r in rows if r["failures"]]
    md = (["# liobench 测试报告", "", f"`{root}`", "", "## 各序列结果", ""] + table_lines(rows, md=True)
          + ["", "## 成功率", ""] + overall_lines(tot, md=True)
          + (["", "## 失败原因", ""] + failures if failures else [])
          + ["", NOTES])
    (root / "summary.md").write_text("\n".join(md) + "\n")
    text = "\n".join(table_lines(rows) + [""] + overall_lines(tot) + ([""] + failures if failures else []))
    return text + f"\n\n已写入：{root}/summary.md、summary.csv、overall.csv、runs.csv"


NOTES = """说明：
- ATE：SE(3) 对齐（不估尺度）后的位置误差 RMSE。RPE：固定距离窗口内的相对位移 / 旋转误差 RMSE，窗口见实验配置的 `rpe`。
- 延迟：从雷达帧到达，到算法发布这一帧里程计的墙钟时间，按本次的播放倍速测得。比纯计算多出等待扫描后第一条 IMU 和 ROS 通信的时间。
- 内部：算法自己统计的每帧耗时，只有提供了这项数据的算法才有。
- CPU：播放期间的平均值，100% 表示占满一个核。内存：算法进程的峰值常驻内存。
- 各指标取成功运行的平均值，± 为多次重复之间的标准差。"""


def compare(base: Path, new: Path) -> str:
    """Per (algorithm, variant, sequence): new - base for the main metrics."""
    a = {(r["algorithm"], r["variant"], r["sequence"]): r for r in aggregate(load_runs(base))}
    b = {(r["algorithm"], r["variant"], r["sequence"]): r for r in aggregate(load_runs(new))}
    keys = ["ate_rmse_m", "rpe_trans_m", "latency_mean_ms", "latency_p95_ms", "internal_mean_ms",
            "cpu_mean_pct", "peak_rss_mb"]
    meta = {k: (h, f) for k, h, f in COLUMNS}
    header = ["算法", "配置", "序列", "成功 参考→新"] + [meta[k][0] + "变化" for k in keys]
    body = []
    for k in sorted(set(a) | set(b)):
        ra, rb = a.get(k), b.get(k)
        cells = [k[0], vname(k[1]), k[2], f"{ra['ok'] if ra else '-'}/{ra['runs'] if ra else '-'} → "
                                   f"{rb['ok'] if rb else '-'}/{rb['runs'] if rb else '-'}"]
        for m in keys:
            va, vb = (ra or {}).get(m), (rb or {}).get(m)
            if va is None or vb is None:
                cells.append("-")
            else:
                d = vb - va
                pct = f" ({100 * d / va:+.0f}%)" if va else ""
                cells.append(("+" if d >= 0 else "") + meta[m][1].format(d) + pct)
        body.append(cells)
    return "\n".join(_render(header, body, md=False))
