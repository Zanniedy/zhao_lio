"""Turn (algorithm spec, sequence, param overrides) into a config file and a command.

An algorithm spec (algorithms/<name>.yaml) looks like:

    package: point_lio
    executable: pointlio_mapping
    workspace: ws/install          # setup.bash sourced before running (relative to liobench/)
    config:
      mode: ros_params             # ros_params: --params-file; yaml_file: path passed as a ROS param
      base: $LIOBENCH_REFS/Point-LIO/config/avia.yaml
      node_key: "/**"              # ros_params only
      path_param: config_path      # yaml_file only
    lidar_types: {livox_custom: 1} # dataset lidar_type -> value of {lidar_type}
    sensor:                        # config key -> template filled from the dataset sensor block
      common.lid_topic: "{lidar_topic}"
    fixed: {}                      # config keys always set
    ros_args: {}                   # extra -p key:=value
    odom_topic: /aft_mapped_to_init
    helpers: []                    # extra commands started alongside (e.g. static TF)
    env: {}
    timing: {kind: csv, param: timing_csv, column: total_ms}   # optional internal per-frame timing
"""
from __future__ import annotations

import shlex
from pathlib import Path

import numpy as np
import yaml

from .config import ROOT, Algorithm, RunSpec, expand_path, format_value, load_yaml, set_dotted


def resolve(p: str, base: Path = ROOT) -> Path:
    q = expand_path(p)
    return q if q.is_absolute() else (base / q).resolve()


def sensor_context(seq_sensor: dict, algo: Algorithm) -> dict:
    """Values available as {placeholders} in an algorithm's sensor mapping."""
    ctx = dict(seq_sensor)
    t = [float(x) for x in seq_sensor.get("extrinsic_t", [0, 0, 0])]
    r = [float(x) for x in seq_sensor.get("extrinsic_r", [1, 0, 0, 0, 1, 0, 0, 0, 1])]
    r_col = np.array(r).reshape(3, 3).T.flatten().tolist()
    ctx.update(extrinsic_t=t, extrinsic_r=r, extrinsic_r_colmajor=r_col, extrinsic_tr_colmajor=t + r_col)
    lt = seq_sensor.get("lidar_type")
    types = algo.spec.get("lidar_types", {})
    if lt is not None:
        if lt not in types:
            raise ValueError(f"{algo.name}: no mapping for lidar_type '{lt}' (have: {list(types)})")
        ctx["lidar_type"] = types[lt]
    return ctx


def flatten(d: dict, prefix: str = "") -> dict:
    out = {}
    for k, v in d.items():
        key = f"{prefix}.{k}" if prefix else str(k)
        if isinstance(v, dict):
            out.update(flatten(v, key))
        else:
            out[key] = v
    return out


def render_config(run: RunSpec, run_dir: Path) -> tuple[Path, dict]:
    """Write the algorithm config for this run; return (path, flat dict of applied overrides)."""
    spec = run.algorithm.spec
    cfg = spec["config"]
    ctx = sensor_context(run.sequence.sensor, run.algorithm)
    ctx["run_dir"] = str(run_dir)
    overrides = {}
    for k, v in (spec.get("sensor") or {}).items():
        overrides[k] = format_value(v, ctx)
    for k, v in (spec.get("fixed") or {}).items():
        overrides[k] = format_value(v, ctx)
    timing = spec.get("timing")
    if timing and timing.get("param"):
        overrides[timing["param"]] = str(run_dir / "internal_timing.csv")
    overrides.update(run.params)

    base = load_yaml(resolve(cfg["base"]))
    out = run_dir / "algo_config.yaml"
    if cfg["mode"] == "ros_params":
        node_key = cfg.get("node_key", "/**")
        node = base.get(node_key, {})
        params = flatten(node.get("ros__parameters", {}))
        params.update(overrides)
        doc = {node_key: {"ros__parameters": params}}
    elif cfg["mode"] == "yaml_file":
        doc = base
        for k, v in overrides.items():
            set_dotted(doc, k, v)
    else:
        raise ValueError(f"unknown config mode {cfg['mode']}")
    with open(out, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)
    return out, overrides


def build_command(run: RunSpec, config_path: Path) -> str:
    """Shell command that execs the node binary directly (no `ros2 run` wrapper process),
    so the measured process tree is the algorithm alone."""
    spec = run.algorithm.spec
    cfg = spec["config"]
    pkg, exe = spec["package"], spec["executable"]
    cmd = ["--ros-args"]
    if cfg["mode"] == "ros_params":
        cmd += ["--params-file", str(config_path)]
    else:
        cmd += ["-p", f"{cfg['path_param']}:={config_path}"]
    ctx = sensor_context(run.sequence.sensor, run.algorithm)
    for k, v in (spec.get("ros_args") or {}).items():
        cmd += ["-p", f"{k}:={format_value(v, ctx)}"]
    binary = f'"$(ros2 pkg prefix {shlex.quote(pkg)})/lib/{shlex.quote(pkg)}/{shlex.quote(exe)}"'
    return f"exec {binary} " + " ".join(shlex.quote(c) for c in cmd)


def setup_script(spec: dict) -> str:
    """Shell prefix that sources ROS, the livox driver and the algorithm's workspace."""
    files = ["/opt/ros/humble/setup.bash"]
    files += [str(resolve(p)) for p in spec.get("extra_setup", [])]
    if spec.get("workspace"):
        files.append(str(resolve(spec["workspace"]) / "setup.bash"))
    return " && ".join(f"source {shlex.quote(f)}" for f in files)
