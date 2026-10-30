"""Run one (algorithm, sequence, params, repeat) and write run.json."""
from __future__ import annotations

import json
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import psutil

from . import metrics as M
from .adapter import build_command, render_config, resolve, setup_script
from .config import ROOT, RunSpec

PY = sys.executable
CRASH_PREFIX = "算法在播放过程中崩溃"


def clean_env(domain_id: int, extra: dict | None = None) -> dict:
    """Environment without conda on PATH (it shadows system libraries such as yaml-cpp)."""
    env = {k: v for k, v in os.environ.items() if not k.startswith("CONDA")}
    env["PATH"] = ":".join(p for p in env.get("PATH", "").split(":") if "conda" not in p)
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["PYTHONUNBUFFERED"] = "1"
    # Larger Fast DDS shared-memory segment for every process (algorithm, recorder, bag player):
    # with the 512 KiB default, best-effort subscribers drop back-to-back LiDAR scans.
    env["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(ROOT / "dds" / "fastdds_local.xml")
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    env["PYTHONPATH"] = str(ROOT) + (":" + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    env.pop("RCUTILS_CONSOLE_OUTPUT_FORMAT", None)
    for k, v in (extra or {}).items():
        env[k] = str(v)
    return env


def spawn(shell_cmd: str, env: dict, log: Path, cwd: Path) -> subprocess.Popen:
    f = open(log, "w")
    return subprocess.Popen(["bash", "-c", shell_cmd], env=env, cwd=cwd, stdout=f, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)


def stop(proc: subprocess.Popen | None, timeout: float = 15.0) -> int | None:
    """SIGINT the process group, then SIGKILL. Returns the exit code."""
    if proc is None:
        return None
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        except ProcessLookupError:
            pass
    return proc.returncode


class Sampler(threading.Thread):
    """Samples CPU (% of one core) and RSS of a process and all its descendants."""

    def __init__(self, pid: int, out: Path, period: float = 0.25):
        super().__init__(daemon=True)
        self.root = psutil.Process(pid)
        self.out = out
        self.period = period
        self._halt = threading.Event()
        self.cpu_total = 0.0  # seconds of CPU across the tree, last sample

    def tree(self):
        try:
            return [self.root] + self.root.children(recursive=True)
        except psutil.NoSuchProcess:
            return []

    @staticmethod
    def hwm_mb(p) -> float:
        try:
            with open(f"/proc/{p.pid}/status") as f:
                for line in f:
                    if line.startswith("VmHWM:"):
                        return int(line.split()[1]) / 1024.0
        except OSError:
            pass
        return 0.0

    def run(self):
        prev_t, prev_cpu = None, None
        with open(self.out, "w", buffering=1) as f:
            f.write("t,cpu_pct,rss_mb,hwm_mb,threads\n")
            while not self._halt.is_set():
                t = time.monotonic()
                cpu = rss = hwm = 0.0
                threads = 0
                for p in self.tree():
                    try:
                        ct = p.cpu_times()
                        cpu += ct.user + ct.system
                        rss += p.memory_info().rss / 2**20
                        hwm += self.hwm_mb(p)
                        threads += p.num_threads()
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
                if threads == 0:
                    break
                if prev_t is not None:
                    pct = 100.0 * (cpu - prev_cpu) / max(t - prev_t, 1e-6)
                    f.write(f"{t:.6f},{pct:.2f},{rss:.2f},{hwm:.2f},{threads}\n")
                prev_t, prev_cpu = t, cpu
                self.cpu_total = cpu
                self._halt.wait(self.period)

    def stop(self):
        self._halt.set()
        self.join(5)


def wait_for(cond, timeout: float, poll: float = 0.1) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(poll)
    return cond()


def run_one(run: RunSpec, run_dir: Path, domain_id: int = 42, log=print) -> dict:
    run_dir.mkdir(parents=True, exist_ok=True)
    spec, seq, st = run.algorithm.spec, run.sequence, run.settings
    sensor = seq.sensor
    result = {
        "algorithm": run.algorithm.name, "variant": run.variant, "params": run.params,
        "sequence": seq.id, "repeat": run.repeat, "rate": st["rate"],
        "status": "failed", "reasons": [], "settings": st, "started": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    if not seq.converted:
        result["reasons"] = [f"{seq.id} 还没有转换：先运行 `liobench convert {seq.dataset}`"]
        return finish(result, run_dir)
    if not seq.bag.exists():
        result["reasons"] = [f"找不到数据包：{seq.bag}"]
        return finish(result, run_dir)

    cfg_path, applied = render_config(run, run_dir)
    result["applied_params"] = applied
    setup = setup_script(spec)
    env = clean_env(domain_id, spec.get("env"))
    procs = {}
    sampler = None
    t_play0 = t_play1 = None
    try:
        rec_cmd = (f"{setup} && exec {shlex.quote(PY)} -m liobench.recorder "
                   f"--lidar-topic {shlex.quote(sensor['lidar_topic'])} "
                   f"--lidar-type {shlex.quote(sensor['lidar_msg'])} "
                   f"--imu-topic {shlex.quote(sensor['imu_topic'])} "
                   f"--odom-topic {shlex.quote(spec['odom_topic'])} "
                   f"--odom-type {shlex.quote(spec.get('odom_type', 'nav_msgs/msg/Odometry'))} "
                   f"--out {shlex.quote(str(run_dir))}")
        procs["recorder"] = spawn(rec_cmd, env, run_dir / "recorder.log", run_dir)
        for i, h in enumerate(spec.get("helpers") or []):
            procs[f"helper{i}"] = spawn(f"{setup} && exec {h}", env, run_dir / f"helper{i}.log", run_dir)
        algo_cmd = f"{setup} && {build_command(run, cfg_path)}"
        (run_dir / "command.sh").write_text(algo_cmd + "\n")
        procs["algo"] = spawn(algo_cmd, env, run_dir / "algo.log", run_dir)
        time.sleep(0.5)
        sampler = Sampler(procs["algo"].pid, run_dir / "resources.csv")
        sampler.start()

        ready = run_dir / "ready"
        if not wait_for(lambda: ready.exists() or procs["algo"].poll() is not None, st["startup_timeout"]):
            result["reasons"] = [f"算法在 {st['startup_timeout']} s 内没有订阅 {sensor['lidar_topic']} 和 "
                                 f"{sensor['imu_topic']}，见 algo.log"]
            return finish(result, run_dir)
        if procs["algo"].poll() is not None:
            result["reasons"] = [f"算法启动时退出（退出码 {procs['algo'].returncode}），见 algo.log"]
            return finish(result, run_dir)
        time.sleep(st["warmup"])

        play_cmd = (f"{setup} && exec ros2 bag play {shlex.quote(str(seq.bag))} -r {st['rate']} "
                    f"--disable-keyboard-controls --read-ahead-queue-size 5000 "
                    f"--topics {shlex.quote(sensor['lidar_topic'])} {shlex.quote(sensor['imu_topic'])}")
        t_play0 = time.monotonic()
        procs["play"] = spawn(play_cmd, env, run_dir / "play.log", run_dir)
        deadline = t_play0 + st["duration"] / st["rate"] if st.get("duration") else None
        crashed = False
        while procs["play"].poll() is None:
            if procs["algo"].poll() is not None:
                crashed = True
                break
            if deadline and time.monotonic() > deadline:
                stop(procs["play"])
                break
            time.sleep(0.2)
        t_play1 = time.monotonic()
        if crashed:
            stop(procs["play"])
            result["reasons"].append(f"{CRASH_PREFIX}（退出码 {procs['algo'].returncode}），见 algo.log")
        else:
            # Drain: wait until odometry stops arriving.
            odom_csv = run_dir / "odom.csv"
            last = [odom_csv.stat().st_mtime if odom_csv.exists() else 0]

            def quiet():
                m = odom_csv.stat().st_mtime if odom_csv.exists() else 0
                if m != last[0]:
                    last[0] = m
                    return False
                return time.time() - m > 2.0
            wait_for(quiet, st["drain_timeout"], poll=0.5)
    finally:
        # Each process is stopped independently so one failure cannot leave the others running.
        try:
            result["algo_exit_code"] = stop(procs.get("algo"))
        except Exception as e:  # noqa: BLE001
            result["reasons"].append(f"无法停止算法进程：{e}")
        for name, p in procs.items():
            if name != "algo":
                try:
                    stop(p, timeout=5)
                except Exception:  # noqa: BLE001
                    pass
        if sampler:
            sampler.stop()

    result["wall_s"] = (t_play1 - t_play0) if t_play0 and t_play1 else None
    result.update(evaluate(run, run_dir, t_play0, t_play1, result["reasons"]))
    return finish(result, run_dir)


def evaluate(run: RunSpec, run_dir: Path, t0, t1, reasons: list) -> dict:
    spec, st = run.algorithm.spec, run.settings
    lidar = M.read_csv(run_dir / "lidar.csv")
    odom = M.read_csv(run_dir / "odom.csv")
    traj = M.odom_to_tum(odom)
    M.write_tum(run_dir / "est.tum", traj)
    n_lidar = len(lidar.get("stamp", []))
    lat = M.frame_latency(lidar, odom)
    out = {
        "lidar_frames": n_lidar, "odom_msgs": len(odom.get("stamp", [])), "frames_done": int(len(lat)),
        "latency_mean_ms": M.mean(lat * 1e3), "latency_p95_ms": M.percentile(lat * 1e3, 95),
        "latency_max_ms": float(lat.max() * 1e3) if len(lat) else None,
        "path_length_m": M.path_length(traj),
    }
    timing = spec.get("timing")
    if timing:
        it = M.internal_timing(run_dir / "internal_timing.csv", timing["column"], timing.get("scale", 1e-3))
        out["internal_mean_ms"] = M.mean(it * 1e3)
        out["internal_p95_ms"] = M.percentile(it * 1e3, 95)
    if (t0 is None or t1 is None) and len(lidar.get("recv_time", [])):
        # reeval: playback window = first to last LiDAR arrival (same clock as resources.csv)
        t0, t1 = float(lidar["recv_time"].min()), float(lidar["recv_time"].max())
    out.update(M.resource_summary(M.read_csv(run_dir / "resources.csv"), t0, t1))

    reasons = list(reasons)
    reasons += M.divergence_checks(traj, n_lidar, len(lat), st["divergence"])
    gt = run.sequence.gt
    out["has_gt"] = bool(gt and gt.exists())
    if out["has_gt"] and len(traj):
        try:
            err = M.trajectory_errors(traj, M.load_tum(gt), st["rpe"])
            out.update(err)
            mx = st["divergence"].get("max_ate")
            if mx is not None and err["ate_rmse_m"] > mx:
                reasons.append(f"ATE {err['ate_rmse_m']:.2f} m > {mx} m")
        except Exception as e:  # noqa: BLE001 - report, don't crash the batch
            reasons.append(f"真值评估失败：{e}")
    out["reasons"] = reasons
    out["status"] = "ok" if not reasons else "diverged"
    return out


def finish(result: dict, run_dir: Path) -> dict:
    result["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(run_dir / "run.json", "w") as f:
        json.dump(result, f, indent=2, default=lambda o: None)
    return result
