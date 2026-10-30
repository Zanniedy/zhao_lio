"""Experiment configuration: registries of algorithms/datasets and run expansion."""
from __future__ import annotations

import copy
import hashlib
import itertools
import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parent.parent

# External locations, overridable from the environment:
#   LIOBENCH_REFS   directory holding the reference algorithm sources (Point-LIO, Super-LIO, ...)
#   LIOBENCH_LIVOX  colcon install directory that provides livox_ros_driver2
os.environ.setdefault("LIOBENCH_REFS", str(ROOT.parent.parent))
os.environ.setdefault("LIOBENCH_LIVOX", str(Path.home() / "cs_project/nav/cs/install"))

DEFAULT_DIVERGENCE = {
    "max_speed": 20.0,      # m/s between consecutive odom poses
    "max_jump": 5.0,        # m between consecutive odom poses
    "min_coverage": 0.9,    # matched odom frames / lidar frames
    "max_ate": None,        # m, only when ground truth exists
}
DEFAULT_RPE = {"delta": 10.0, "unit": "m"}


def expand_path(p: str | None) -> Path | None:
    if p is None:
        return None
    return Path(os.path.expandvars(os.path.expanduser(str(p))))


def load_yaml(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f) or {}


def deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def set_dotted(d: dict, key: str, value: Any) -> None:
    parts = key.split(".")
    for p in parts[:-1]:
        d = d.setdefault(p, {})
    d[parts[-1]] = value


def format_value(v: Any, ctx: dict) -> Any:
    """Substitute "{name}" placeholders. A string that is exactly one placeholder
    keeps the type of the context value (lists, floats)."""
    if isinstance(v, str):
        if v.startswith("{") and v.endswith("}") and v[1:-1] in ctx:
            return ctx[v[1:-1]]
        return v.format(**ctx)
    if isinstance(v, list):
        return [format_value(x, ctx) for x in v]
    if isinstance(v, dict):
        return {k: format_value(x, ctx) for k, x in v.items()}
    return v


@dataclass
class Sequence:
    dataset: str
    name: str
    bag: Path
    gt: Path | None
    sensor: dict           # lidar_topic, imu_topic, lidar_type, extrinsic_t/r, ...
    converted: bool = True # False: dataset needs `liobench convert` first

    @property
    def id(self) -> str:
        return f"{self.dataset}/{self.name}"


@dataclass
class Algorithm:
    name: str
    spec: dict             # contents of algorithms/<name>.yaml
    path: Path


@dataclass
class RunSpec:
    algorithm: Algorithm
    sequence: Sequence
    params: dict           # user param overrides (dotted keys)
    repeat: int
    settings: dict         # rate, timeouts, divergence, rpe

    @property
    def variant(self) -> str:
        """Readable label for the param combination."""
        if not self.params:
            return "default"
        return ",".join(f"{k.split('.')[-1]}={v}" for k, v in sorted(self.params.items()))

    @property
    def key(self) -> str:
        h = hashlib.sha1(json.dumps(self.params, sort_keys=True).encode()).hexdigest()[:8]
        return f"{self.algorithm.name}/{h}/{self.sequence.dataset}_{self.sequence.name}/r{self.repeat}"


def load_dataset(name: str, root: Path = ROOT) -> dict:
    path = root / "datasets" / f"{name}.yaml"
    if not path.exists():
        raise FileNotFoundError(f"unknown dataset '{name}' (no {path})")
    return load_yaml(path)


def load_sequence(seq_id: str, root: Path = ROOT) -> Sequence:
    if "/" not in seq_id:
        raise ValueError(f"sequence must be 'dataset/sequence', got '{seq_id}'")
    ds_name, seq_name = seq_id.split("/", 1)
    ds = load_dataset(ds_name, root)
    seqs = ds.get("sequences", {})
    if seq_name not in seqs:
        raise KeyError(f"dataset '{ds_name}' has no sequence '{seq_name}' (have: {', '.join(seqs)})")
    s = seqs[seq_name] or {}
    sensor = deep_merge(ds.get("sensor", {}), s.get("sensor", {}))
    if "output" not in ds:  # legacy: bag used as is
        return Sequence(ds_name, seq_name, expand_path(s["bag"]), expand_path(s.get("gt")), sensor)
    # Converted dataset: runs read the canonical copy made by `liobench convert`.
    d = converted_dir(ds, seq_name)
    meta_path = d / "meta.yaml"
    if meta_path.exists():
        sensor = deep_merge(sensor, load_yaml(meta_path).get("sensor", {}))
    gt = d / "gt.tum"
    return Sequence(ds_name, seq_name, d / "bag", gt if gt.exists() else None, sensor,
                    converted=meta_path.exists())


def converted_dir(ds: dict, seq_name: str) -> Path:
    return expand_path(ds["output"]) / seq_name


def expand_sequences(patterns: list[str], root: Path = ROOT) -> list[Sequence]:
    """'dataset/seq' or 'dataset/*'."""
    out = []
    for p in patterns:
        ds_name, seq_name = p.split("/", 1)
        if seq_name == "*":  # all sequences that are available; name one explicitly to require it
            for s in load_dataset(ds_name, root).get("sequences", {}):
                seq = load_sequence(f"{ds_name}/{s}", root)
                if seq.converted and seq.bag.exists():
                    out.append(seq)
                else:
                    print(f"提示：跳过 {seq.id}（未转换或没有数据包）")
        else:
            out.append(load_sequence(p, root))
    return out


def load_algorithm(name: str, root: Path = ROOT) -> Algorithm:
    path = root / "algorithms" / f"{name}.yaml"
    if not path.exists():
        raise FileNotFoundError(f"unknown algorithm '{name}' (no {path})")
    return Algorithm(name, load_yaml(path), path)


def param_grid(params: dict) -> list[dict]:
    """{'a': [1, 2], 'b': 3} -> [{'a': 1, 'b': 3}, {'a': 2, 'b': 3}].
    Use a nested list to sweep over list-valued params: {'t': [[0,0,0],[1,0,0]]}."""
    if not params:
        return [{}]
    keys = list(params)
    values = [params[k] if isinstance(params[k], list) else [params[k]] for k in keys]
    return [dict(zip(keys, combo)) for combo in itertools.product(*values)]


def expand_experiment(exp: dict, root: Path = ROOT) -> list[RunSpec]:
    settings = {
        "rate": float(exp.get("rate", 1.0)),
        "startup_timeout": float(exp.get("startup_timeout", 30.0)),
        "drain_timeout": float(exp.get("drain_timeout", 10.0)),
        "warmup": float(exp.get("warmup", 2.0)),
        "divergence": {**DEFAULT_DIVERGENCE, **(exp.get("divergence") or {})},
        "rpe": {**DEFAULT_RPE, **(exp.get("rpe") or {})},
        "duration": exp.get("duration"),
    }
    sequences = expand_sequences(exp["sequences"], root)
    repeats = int(exp.get("repeats", 1))
    runs = []
    for entry in exp["algorithms"]:
        if isinstance(entry, str):
            entry = {"name": entry}
        algo = load_algorithm(entry["name"], root)
        for params in param_grid(entry.get("params") or {}):
            for seq in sequences:
                for r in range(repeats):
                    runs.append(RunSpec(algo, seq, params, r, settings))
    return runs
