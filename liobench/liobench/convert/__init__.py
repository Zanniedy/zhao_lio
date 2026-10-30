"""Dataset converter: many source formats in, one canonical format out.

Canonical sequence directory (<dataset output>/<sequence>/):
    bag/        ROS2 bag (sqlite3) with
                  /liobench/lidar  livox_ros_driver2/msg/CustomMsg; header.stamp = timebase = time of
                                   the earliest point, offset_time = ns since then, points sorted by time,
                                   line = laser/ring index. Written at the scan end time, as a driver would.
                  /liobench/imu    sensor_msgs/msg/Imu, acceleration in m/s^2, angular velocity in rad/s
    gt.tum      ground truth in the IMU frame, TUM format (only if the source has one)
    meta.yaml   canonical sensor block for the runner + source info and statistics

Decoders are looked up by name in registries (see `liobench convert --list`):
    LIDAR: livox_custom, pointcloud2, velodyne, ouster, hesai, robosense, livox_pointcloud2
    IMU:   imu
    GT:    tum, csv, kitti
Bags are read with `rosbags`, so ROS1 and ROS2 bags both work without a ROS installation.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import time
from pathlib import Path

import numpy as np
import yaml

from . import imu_gt, lidar  # noqa: F401  (registers decoders)
from .registry import GT, IMU, LIDAR
from .scan import CUSTOM_MSG_DEF, CUSTOM_POINT_DEF, encode_custom_cdr

VERSION = 1
LIDAR_TOPIC = "/liobench/lidar"
IMU_TOPIC = "/liobench/imu"
LIDAR_MSG = "livox_ros_driver2/msg/CustomMsg"

CANONICAL_SENSOR = {
    "lidar_topic": LIDAR_TOPIC,
    "lidar_msg": LIDAR_MSG,
    "lidar_type": "livox_custom",
    "imu_topic": IMU_TOPIC,
    "acc_norm": 9.81,
}


def list_converters() -> dict[str, list[tuple[str, str]]]:
    return {"lidar": LIDAR.describe(), "imu": IMU.describe(), "gt": GT.describe()}


def _options(cfg: dict, drop=("topic", "decoder", "format", "imu_T_gt", "time_offset")) -> dict:
    return {k: v for k, v in (cfg or {}).items() if k not in drop}


def fingerprint(source: dict) -> str:
    return hashlib.sha1(json.dumps({"v": VERSION, **source}, sort_keys=True, default=str).encode()).hexdigest()[:12]


def _typestore():
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    ts = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    types.update(get_types_from_msg(CUSTOM_POINT_DEF, "livox_ros_driver2/msg/CustomPoint"))
    types.update(get_types_from_msg(CUSTOM_MSG_DEF, LIDAR_MSG))
    ts.register(types)
    return ts


def _imu_cdr(ts, stamp_ns: int, gyro, acc, frame_id: str = "imu") -> bytes:
    T = ts.types
    sec, nsec = divmod(int(stamp_ns), 1_000_000_000)
    cov = np.zeros(9)
    ocov = np.zeros(9)
    ocov[0] = -1.0  # orientation not provided
    msg = T["sensor_msgs/msg/Imu"](
        header=T["std_msgs/msg/Header"](stamp=T["builtin_interfaces/msg/Time"](sec=sec, nanosec=nsec), frame_id=frame_id),
        orientation=T["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0), orientation_covariance=ocov,
        angular_velocity=T["geometry_msgs/msg/Vector3"](x=float(gyro[0]), y=float(gyro[1]), z=float(gyro[2])),
        angular_velocity_covariance=cov,
        linear_acceleration=T["geometry_msgs/msg/Vector3"](x=float(acc[0]), y=float(acc[1]), z=float(acc[2])),
        linear_acceleration_covariance=cov)
    return bytes(ts.serialize_cdr(msg, "sensor_msgs/msg/Imu"))


def convert_sequence(seq_name: str, raw: dict, source: dict, out_dir: Path, force: bool = False, log=print) -> dict:
    """raw: {bag: path, gt: path?}; source: {lidar: {...}, imu: {...}, gt: {...}}."""
    from rosbags.highlevel import AnyReader
    from rosbags.rosbag2 import Writer

    from ..config import expand_path

    src_bag = expand_path(raw["bag"])
    src_gt = expand_path(raw.get("gt"))
    fp = fingerprint({"bag": str(src_bag), "gt": str(src_gt), "source": source})
    meta_path = out_dir / "meta.yaml"
    if meta_path.exists() and not force:
        meta = yaml.safe_load(meta_path.read_text())
        if meta.get("fingerprint") == fp:
            gt_missing = src_gt and src_gt.exists() and not (out_dir / "gt.tum").exists()
            if not gt_missing:
                log(f"{seq_name}：已是最新（{out_dir}）")
                return meta
            log(f"{seq_name}：发现新的真值文件，只转换真值")
            meta["gt"] = convert_gt(src_gt, source.get("gt") or {}, out_dir / "gt.tum")
            meta_path.write_text(yaml.safe_dump(meta, sort_keys=False))
            return meta
    if not src_bag.exists():
        raise FileNotFoundError(f"{seq_name}：找不到原始数据包：{src_bag}")

    lcfg, icfg = source["lidar"], source["imu"]
    ldec = LIDAR.create(lcfg["decoder"], **_options(lcfg))
    idec = IMU.create(icfg.get("decoder", "imu"), **_options(icfg))
    l_off = int(round(float(lcfg.get("time_offset", 0.0)) * 1e9))  # added to LiDAR times
    i_off = int(round(float(icfg.get("time_offset", 0.0)) * 1e9))

    tmp = out_dir.with_name(out_dir.name + ".tmp")
    if tmp.exists():
        shutil.rmtree(tmp)
    tmp.mkdir(parents=True)
    ts = _typestore()
    t0 = time.monotonic()
    stats = {"scans": 0, "imu": 0, "points": 0, "empty_scans": 0}
    starts, durations, max_line = [], [], 0
    with AnyReader([src_bag], default_typestore=_typestore()) as reader, Writer(tmp / "bag", version=8) as writer:
        conns = {c.topic: c for c in reader.connections}
        for topic in (lcfg["topic"], icfg["topic"]):
            if topic not in conns:
                raise KeyError(f"{seq_name}：数据包里没有话题 {topic}（现有：{', '.join(sorted(conns))}）")
        lc = writer.add_connection(LIDAR_TOPIC, LIDAR_MSG, typestore=ts)
        ic = writer.add_connection(IMU_TOPIC, "sensor_msgs/msg/Imu", typestore=ts)
        is_ros1 = not reader.is2
        sel = [conns[lcfg["topic"]], conns[icfg["topic"]]]
        total = sum(c.msgcount for c in sel)
        n = 0
        for conn, _t, data in reader.messages(connections=sel):
            n += 1
            de = (lambda d=data, c=conn: reader.deserialize(d, c.msgtype))
            if conn.topic == lcfg["topic"]:
                scan = ldec.decode(data, is_ros1, de)
                if len(scan) == 0:
                    stats["empty_scans"] += 1
                    continue
                scan.stamp_ns += l_off
                writer.write(lc, scan.end_ns, encode_custom_cdr(scan))
                stats["scans"] += 1
                stats["points"] += len(scan)
                starts.append(scan.stamp_ns)
                durations.append(scan.end_ns - scan.stamp_ns)
                max_line = max(max_line, int(scan.line.max()))
            else:
                stamp, gyro, acc = idec.decode(data, is_ros1, de)
                stamp += i_off
                writer.write(ic, stamp, _imu_cdr(ts, stamp, gyro, acc))
                stats["imu"] += 1
            if n % 20000 == 0:
                log(f"  {seq_name}：{100 * n / total:.0f}%")
    if stats["scans"] < 2:
        raise ValueError(f"{seq_name}：只解出 {stats['scans']} 帧非空点云")

    starts = np.array(starts)
    period = float(np.median(np.diff(np.sort(starts)))) / 1e9
    meta = {
        "fingerprint": fp, "converter_version": VERSION,
        "converted": time.strftime("%Y-%m-%d %H:%M:%S"),
        "source": {"bag": str(src_bag), "gt": str(src_gt) if src_gt else None, **source},
        "sensor": {**CANONICAL_SENSOR, "scan_lines": max_line + 1, "scan_rate": round(1.0 / period, 3)},
        "stats": {**stats, "points_per_scan": round(stats["points"] / stats["scans"], 1),
                  "scan_duration_ms": round(float(np.median(durations)) / 1e6, 2),
                  "duration_s": round(float(starts[-1] - starts[0]) / 1e9, 1),
                  "convert_s": round(time.monotonic() - t0, 1)},
    }
    if src_gt and src_gt.exists():
        meta["gt"] = convert_gt(src_gt, source.get("gt") or {}, tmp / "gt.tum")
    (tmp / "meta.yaml").write_text(yaml.safe_dump(meta, sort_keys=False))
    if out_dir.exists():
        shutil.rmtree(out_dir)
    tmp.rename(out_dir)
    log(f"{seq_name}：{stats['scans']} 帧点云，{stats['imu']} 条 IMU，每帧 {meta['stats']['points_per_scan']} 点，"
        f"{meta['sensor']['scan_lines']} 线，{meta['sensor']['scan_rate']} Hz -> {out_dir}"
        f"（用时 {meta['stats']['convert_s']} s）")
    return meta


def convert_gt(src: Path, cfg: dict, dst: Path) -> dict:
    reader = GT.create(cfg.get("format", "tum"), **_options(cfg))
    gt = imu_gt.gt_to_imu(reader.read(src), cfg.get("imu_T_gt"))
    gt[:, 0] += float(cfg.get("time_offset", 0.0))
    gt = gt[np.argsort(gt[:, 0])]
    with open(dst, "w") as f:
        for r in gt:
            f.write(f"{r[0]:.9f} " + " ".join(f"{v:.9f}" for v in r[1:8]) + "\n")
    return {"poses": len(gt), "start": float(gt[0, 0]), "end": float(gt[-1, 0])}
