"""IMU decoders and ground-truth readers."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .registry import GT, IMU

G = 9.81   # same value the algorithms use for acc_norm / gravity
ACC_UNITS = {"m/s2": 1.0, "g": G}


@IMU.register("imu")
class SensorImu:
    """sensor_msgs/Imu，加速度统一换算成 m/s²。acc_unit: m/s2 | g；gyro_unit: rad/s | deg/s"""

    def __init__(self, acc_unit: str = "m/s2", gyro_unit: str = "rad/s"):
        if acc_unit not in ACC_UNITS:
            raise ValueError(f"acc_unit must be one of {list(ACC_UNITS)}")
        if gyro_unit not in ("rad/s", "deg/s"):
            raise ValueError("gyro_unit must be rad/s or deg/s")
        self.acc_scale = ACC_UNITS[acc_unit]
        self.gyro_scale = np.pi / 180 if gyro_unit == "deg/s" else 1.0

    def decode(self, raw: bytes, is_ros1: bool, deserialize):
        """-> (stamp_ns, gyro[3] rad/s, acc[3] m/s^2)."""
        m = deserialize()
        st = m.header.stamp
        g = m.angular_velocity
        a = m.linear_acceleration
        return (st.sec * 1_000_000_000 + st.nanosec,
                np.array([g.x, g.y, g.z]) * self.gyro_scale,
                np.array([a.x, a.y, a.z]) * self.acc_scale)


# ---------------------------------------------------------------- ground truth
# Readers return N x 8 arrays: t[s] x y z qx qy qz qw (pose of the GT frame in its world frame).


def _rows(path: Path, delimiter=None, skip_header: int = 0) -> np.ndarray:
    rows = []
    with open(path) as f:
        for i, line in enumerate(f):
            s = line.strip()
            if i < skip_header or not s or s.startswith("#") or s.startswith("%"):
                continue
            parts = s.split(delimiter) if delimiter else s.replace(",", " ").split()
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                continue  # textual header line
    return np.array(rows)


@GT.register("tum")
class TumGt:
    """TUM 文本：t x y z qx qy qz qw（空格或逗号分隔）。"""

    def __init__(self, time_unit: str = "s"):
        self.scale = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}[time_unit]

    def read(self, path: Path) -> np.ndarray:
        d = _rows(path)[:, :8]
        d[:, 0] *= self.scale
        return d


@GT.register("csv")
class CsvGt:
    """带表头的 CSV，用 columns 指定列名，如 {t: '#timestamp', x: p_x, ..., qw: q_w}。"""

    def __init__(self, columns: dict, time_unit: str = "s", delimiter: str = ","):
        self.columns = columns
        self.scale = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}[time_unit]
        self.delimiter = delimiter

    def read(self, path: Path) -> np.ndarray:
        import csv
        with open(path) as f:
            rows = list(csv.reader(f, delimiter=self.delimiter))
        header = [h.strip() for h in rows[0]]
        idx = [header.index(self.columns[k]) for k in ["t", "x", "y", "z", "qx", "qy", "qz", "qw"]]
        d = np.array([[float(r[i]) for i in idx] for r in rows[1:] if len(r) == len(header)])
        d[:, 0] *= self.scale
        return d


@GT.register("kitti")
class KittiGt:
    """KITTI 位姿：每行 12 个数（3x4 行优先），另需 times 文件（每行一个时间）。"""

    def __init__(self, times: str):
        self.times = Path(times).expanduser()

    def read(self, path: Path) -> np.ndarray:
        m = _rows(path)[:, :12].reshape(-1, 3, 4)
        t = _rows(self.times)[:, 0]
        if len(t) != len(m):
            raise ValueError(f"{len(m)} poses but {len(t)} times")
        q = rot_to_quat(m[:, :, :3])
        return np.concatenate([t[:, None], m[:, :, 3], q], axis=1)


# ---------------------------------------------------------------- SE(3) helpers (quaternions x y z w)


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    x, y, z, w = (q / np.linalg.norm(q, axis=-1, keepdims=True)).T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, float).reshape(-1, 3, 3)
    q = np.empty((len(R), 4))
    for i, m in enumerate(R):
        tr = np.trace(m)
        if tr > 0:
            s = 2 * np.sqrt(tr + 1)
            q[i] = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4]
        else:
            j = int(np.argmax(np.diag(m)))
            k, l = (j + 1) % 3, (j + 2) % 3
            s = 2 * np.sqrt(1 + m[j, j] - m[k, k] - m[l, l])
            v = np.empty(3)
            v[j] = s / 4
            v[k] = (m[k, j] + m[j, k]) / s
            v[l] = (m[l, j] + m[j, l]) / s
            q[i] = [*v, (m[l, k] - m[k, l]) / s]
    return q * np.sign(q[:, 3:4] + 1e-12)


def gt_to_imu(gt: np.ndarray, imu_T_gt: dict | None) -> np.ndarray:
    """Re-express GT poses as IMU poses: T_w_imu = T_w_gt * inv(T_imu_gt).
    imu_T_gt: {t: [3], r: [9 row-major]} or {t: [3], q: [x y z w]}; None = identity."""
    if not imu_T_gt:
        return gt
    t_ig = np.asarray(imu_T_gt.get("t", [0, 0, 0]), float)
    if "q" in imu_T_gt:
        R_ig = quat_to_rot(np.asarray(imu_T_gt["q"], float)[None])[0]
    else:
        R_ig = np.asarray(imu_T_gt.get("r", [1, 0, 0, 0, 1, 0, 0, 0, 1]), float).reshape(3, 3)
    R_gi, t_gi = R_ig.T, -R_ig.T @ t_ig
    R_wg = quat_to_rot(gt[:, 4:8])
    out = gt.copy()
    out[:, 1:4] = gt[:, 1:4] + np.einsum("nij,j->ni", R_wg, t_gi)
    out[:, 4:8] = rot_to_quat(R_wg @ R_gi)
    return out
