"""LiDAR decoders: source message -> canonical Scan.

Each decoder gets the raw serialized bytes, whether the bag is ROS1, and a
`deserialize()` callable for message types that are easier to read deserialized.
Options come from the dataset yaml (`source.lidar`), e.g.

    lidar: {topic: /velodyne_points, decoder: velodyne}
    lidar: {topic: /points, decoder: pointcloud2, time_field: t, time_unit: ns, time_ref: relative}
"""
from __future__ import annotations

import numpy as np

from .registry import LIDAR
from .scan import Scan, decode_custom_cdr, decode_custom_ros1

TIME_UNITS = {"s": 1e9, "ms": 1e6, "us": 1e3, "ns": 1.0}

# sensor_msgs/PointField datatype -> numpy
PF_TYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


@LIDAR.register("livox_custom")
class LivoxCustom:
    """Livox CustomMsg（livox_ros_driver / livox_ros_driver2）。time_base: header | timebase"""

    def __init__(self, time_base: str = "header"):
        # header: point time = header.stamp + offset_time (Point-LIO convention; needed when
        #         timebase holds device uptime). timebase: point time = timebase + offset_time.
        if time_base not in ("header", "timebase"):
            raise ValueError("time_base must be 'header' or 'timebase'")
        self.time_base = time_base

    def decode(self, raw: bytes, is_ros1: bool, deserialize) -> Scan:
        stamp, _fid, timebase, pts = (decode_custom_ros1 if is_ros1 else decode_custom_cdr)(raw)
        base = stamp if self.time_base == "header" else timebase
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1)
        return Scan.from_times(base, pts["offset_time"], xyz, pts["reflectivity"], pts["line"], pts["tag"])


@LIDAR.register("pointcloud2")
class PointCloud2:
    """通用 sensor_msgs/PointCloud2：自己指定 time_field、time_unit、time_ref、ring_field 等字段。"""

    def __init__(self, time_field: str = "time", time_unit: str = "s", time_ref: str = "relative",
                 ring_field: str | None = "ring", intensity_field: str | None = "intensity",
                 tag_field: str | None = None):
        if time_unit not in TIME_UNITS and time_unit != "auto":
            raise ValueError(f"time_unit must be one of {list(TIME_UNITS)} or auto")
        if time_ref not in ("relative", "absolute"):
            raise ValueError("time_ref must be 'relative' (to header.stamp) or 'absolute'")
        self.time_field, self.time_unit, self.time_ref = time_field, time_unit, time_ref
        self.ring_field, self.intensity_field, self.tag_field = ring_field, intensity_field, tag_field

    @staticmethod
    def structured(msg) -> np.ndarray:
        endian = ">" if msg.is_bigendian else "<"
        dt = np.dtype({"names": [f.name for f in msg.fields],
                       "formats": [endian + PF_TYPES[f.datatype] for f in msg.fields],
                       "offsets": [f.offset for f in msg.fields], "itemsize": msg.point_step})
        data = np.asarray(msg.data, np.uint8)
        n = msg.width * msg.height
        if msg.row_step != msg.width * msg.point_step:  # rows with padding
            rows = data.reshape(msg.height, msg.row_step)[:, :msg.width * msg.point_step]
            data = np.ascontiguousarray(rows).reshape(-1)
        return np.frombuffer(data.tobytes(), dt, n)

    def unit_scale(self, t: np.ndarray) -> float:
        if self.time_unit != "auto":
            return TIME_UNITS[self.time_unit]
        m = float(np.nanmax(np.abs(t))) if len(t) else 0.0
        if self.time_ref == "absolute":   # epoch time: pick the unit from the magnitude
            return 1.0 if m > 1e17 else 1e3 if m > 1e14 else 1e6 if m > 1e11 else 1e9
        return 1.0 if m > 1e6 else 1e3 if m > 1e3 else 1e6 if m > 1.0 else 1e9

    def decode(self, raw: bytes, is_ros1: bool, deserialize) -> Scan:
        msg = deserialize()
        pts = self.structured(msg)
        names = pts.dtype.names
        if self.time_field not in names:
            raise KeyError(f"PointCloud2 has no time field '{self.time_field}' (fields: {names})")
        t = pts[self.time_field].astype(np.float64)
        scale = self.unit_scale(t)
        if self.time_ref == "relative":
            st = msg.header.stamp
            base, rel = st.sec * 1_000_000_000 + st.nanosec, t * scale
        else:  # absolute: subtract the first finite value before scaling to keep precision
            fin = t[np.isfinite(t)]
            t_ref = float(fin.min()) if len(fin) else 0.0
            base, rel = int(round(t_ref * scale)), (t - t_ref) * scale
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1)
        inten = pts[self.intensity_field] if self.intensity_field in names else np.zeros(len(pts))
        ring = pts[self.ring_field] if self.ring_field in names else np.zeros(len(pts))
        tag = pts[self.tag_field] if self.tag_field and self.tag_field in names else None
        return Scan.from_times(base, rel, xyz, inten, ring, tag)


def _preset(name: str, doc: str, **defaults):
    """Register a PointCloud2 decoder with driver-specific defaults (still overridable)."""
    def init(self, **options):
        PointCloud2.__init__(self, **{**defaults, **options})
    cls = type(name, (PointCloud2,), {"__init__": init, "__doc__": doc})
    LIDAR.register(name)(cls)


_preset("velodyne", "Velodyne PointCloud2（velodyne_pointcloud）：float32 `time`，相对 header 的秒数；`ring`。",
        time_field="time", time_unit="s", time_ref="relative")
_preset("ouster", "Ouster PointCloud2（ouster-ros）：uint32 `t`，相对 header 的纳秒数；`ring`。",
        time_field="t", time_unit="ns", time_ref="relative")
_preset("hesai", "Hesai PointCloud2（HesaiLidar_ROS_2.0）：float64 `timestamp`，绝对时间（秒）；`ring`。",
        time_field="timestamp", time_unit="s", time_ref="absolute")
_preset("robosense", "RoboSense PointCloud2（rslidar_sdk XYZIRT）：float64 `timestamp`，绝对时间（秒）；`ring`。",
        time_field="timestamp", time_unit="s", time_ref="absolute")
_preset("livox_pointcloud2", "Livox PointCloud2（livox_ros_driver2，xfer_format 0）：float64 `timestamp`，绝对时间；`line`、`tag`。",
        time_field="timestamp", time_unit="auto", time_ref="absolute", ring_field="line", tag_field="tag")
