from pathlib import Path

import numpy as np
import pytest
import yaml

from liobench.convert import LIDAR, _imu_cdr, _typestore, convert_sequence, list_converters
from liobench.convert.imu_gt import gt_to_imu, quat_to_rot, rot_to_quat
from liobench.convert.scan import Scan, decode_custom_cdr, encode_custom_cdr


def make_scan(n=7, stamp=1_700_000_000_123_456_789):
    rng = np.random.default_rng(0)
    return Scan(stamp, rng.normal(size=(n, 3)).astype(np.float32), np.arange(n, dtype=np.float32),
                np.sort(rng.integers(0, 100_000_000, n)).astype(np.uint32),
                (np.arange(n) % 4).astype(np.uint8), np.full(n, 0x10, np.uint8))


@pytest.mark.parametrize("frame_id", ["", "l", "lidar", "livox_frame_xyz"])  # exercises 8-byte alignment
def test_custom_cdr_matches_rosbags(frame_id):
    ts = _typestore()
    scan = make_scan()
    raw = encode_custom_cdr(scan, frame_id)
    m = ts.deserialize_cdr(raw, "livox_ros_driver2/msg/CustomMsg")
    assert m.header.frame_id == frame_id
    assert m.header.stamp.sec * 10**9 + m.header.stamp.nanosec == scan.stamp_ns
    assert m.timebase == scan.stamp_ns and m.point_num == len(scan)
    assert [p.offset_time for p in m.points] == scan.offset_ns.tolist()
    assert np.allclose([[p.x, p.y, p.z] for p in m.points], scan.xyz)
    assert [p.line for p in m.points] == scan.line.tolist()
    # and our decoder reads what rosbags writes
    stamp, fid, tb, pts = decode_custom_cdr(bytes(ts.serialize_cdr(m, "livox_ros_driver2/msg/CustomMsg")))
    assert (stamp, fid, tb) == (scan.stamp_ns, frame_id, scan.stamp_ns)
    assert pts["offset_time"].tolist() == scan.offset_ns.tolist()


def test_custom_cdr_empty():
    s = make_scan(0)
    stamp, _, _, pts = decode_custom_cdr(encode_custom_cdr(s))
    assert len(pts) == 0


def test_livox_time_base_header_vs_timebase():
    ts = _typestore()
    scan = make_scan()
    m = ts.deserialize_cdr(encode_custom_cdr(scan), "livox_ros_driver2/msg/CustomMsg")
    m.timebase = 3_408_000_000_000  # device uptime, like NarrowWide
    raw = bytes(ts.serialize_cdr(m, "livox_ros_driver2/msg/CustomMsg"))
    h = LIDAR.create("livox_custom").decode(raw, False, None)
    assert h.stamp_ns == scan.stamp_ns + int(scan.offset_ns[0])
    assert h.end_ns == scan.stamp_ns + int(scan.offset_ns[-1])
    t = LIDAR.create("livox_custom", time_base="timebase").decode(raw, False, None)
    assert t.stamp_ns == 3_408_000_000_000 + int(scan.offset_ns[0])


class FakeField:
    def __init__(self, name, offset, datatype):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, 1


class FakeStamp:
    def __init__(self, ns):
        self.sec, self.nanosec = divmod(ns, 10**9)


class FakeCloud:
    """Minimal PointCloud2 stand-in: x y z intensity (f4), ring (u2), time field."""

    def __init__(self, header_ns, xyz, time_vals, time_dtype, time_name, ring):
        tcode = {"f4": 7, "f8": 8, "u4": 6}[time_dtype]
        tsize = np.dtype(time_dtype).itemsize
        self.fields = [FakeField("x", 0, 7), FakeField("y", 4, 7), FakeField("z", 8, 7),
                       FakeField("intensity", 12, 7), FakeField("ring", 16, 4), FakeField(time_name, 20, tcode)]
        self.point_step = 20 + tsize + (8 - (20 + tsize) % 8) % 8
        dt = np.dtype({"names": ["x", "y", "z", "intensity", "ring", time_name],
                       "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<" + time_dtype],
                       "offsets": [0, 4, 8, 12, 16, 20], "itemsize": self.point_step})
        a = np.zeros(len(xyz), dt)
        a["x"], a["y"], a["z"] = xyz.T
        a["intensity"] = 7
        a["ring"] = ring
        a[time_name] = time_vals
        self.data = np.frombuffer(a.tobytes(), np.uint8)
        self.width, self.height = len(xyz), 1
        self.row_step = self.point_step * self.width
        self.is_bigendian = False

        class H:
            pass
        self.header = H()
        self.header.stamp = FakeStamp(header_ns)


@pytest.mark.parametrize("decoder,dtype,name,vals,scale,absolute", [
    ("velodyne", "f4", "time", np.array([-0.05, -0.02, 0.0]), 1e9, False),   # header at scan end
    ("ouster", "u4", "t", np.array([0, 30_000_000, 60_000_000]), 1.0, False),
    ("hesai", "f8", "timestamp", None, 1e9, True),
])
def test_pointcloud2_presets(decoder, dtype, name, vals, scale, absolute):
    header = 1_700_000_000_000_000_000
    if absolute:
        vals = header / 1e9 + np.array([0.0, 0.03, 0.06])
    xyz = np.array([[1, 0, 0], [0, 2, 0], [np.nan, 0, 0]], np.float32) if decoder == "ouster" else \
        np.array([[1, 0, 0], [0, 2, 0], [0, 0, 3]], np.float32)
    cloud = FakeCloud(header, xyz, vals, dtype, name, ring=[0, 5, 9])
    s = LIDAR.create(decoder).decode(b"", False, lambda: cloud)
    t_abs = (vals.astype(np.float64) * scale) + (0 if absolute else header)
    ok = np.isfinite(xyz).all(1)
    assert len(s) == ok.sum()
    assert s.stamp_ns == pytest.approx(t_abs[ok].min(), abs=2e3 if absolute else 300)  # reference is float64 epoch ns
    assert s.end_ns == pytest.approx(t_abs[ok].max(), abs=2e3 if absolute else 300)  # reference is float64 epoch ns
    assert s.line.tolist() == np.array([0, 5, 9])[ok].tolist()
    assert np.all(s.tag == 0x10)


def test_registry_lists_everything():
    c = list_converters()
    names = [n for n, _ in c["lidar"]]
    for n in ["livox_custom", "pointcloud2", "velodyne", "ouster", "hesai", "robosense", "livox_pointcloud2"]:
        assert n in names
    assert [n for n, _ in c["gt"]] == ["csv", "kitti", "tum"]
    with pytest.raises(KeyError):
        LIDAR.create("nope")


def test_quat_roundtrip():
    rng = np.random.default_rng(1)
    q = rng.normal(size=(50, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    q *= np.sign(q[:, 3:4])
    assert np.allclose(rot_to_quat(quat_to_rot(q)), q, atol=1e-9)


def test_gt_to_imu():
    # GT frame is 1 m ahead (x) of the IMU, no rotation: T_imu_gt = translation (1, 0, 0).
    # A GT pose at yaw 90 deg, position (5, 5, 0) means the IMU is 1 m "behind" along the GT heading.
    yaw = np.pi / 2
    gt = np.array([[0.0, 5, 5, 0, 0, 0, np.sin(yaw / 2), np.cos(yaw / 2)]])
    imu = gt_to_imu(gt, {"t": [1, 0, 0]})
    assert np.allclose(imu[0, 1:4], [5, 4, 0])
    assert np.allclose(imu[0, 4:8], gt[0, 4:8])


def write_ros1_bag(path: Path, n_scans=5, n_imu=100):
    """Synthetic ROS1 bag: Velodyne-style PointCloud2 + IMU in g units."""
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS1_NOETIC)
    T = ts.types
    PF = T["sensor_msgs/msg/PointField"]
    t0 = 1_700_000_000_000_000_000
    with Writer(path) as w:
        lc = w.add_connection("/velodyne_points", "sensor_msgs/msg/PointCloud2", typestore=ts)
        ic = w.add_connection("/imu", "sensor_msgs/msg/Imu", typestore=ts)
        msgs = []
        for i in range(n_imu):
            st = t0 + i * 5_000_000
            sec, ns = divmod(st, 10**9)
            hdr = T["std_msgs/msg/Header"](seq=i, stamp=T["builtin_interfaces/msg/Time"](sec=sec, nanosec=ns), frame_id="imu")
            v = T["geometry_msgs/msg/Vector3"]
            m = T["sensor_msgs/msg/Imu"](header=hdr, orientation=T["geometry_msgs/msg/Quaternion"](x=0, y=0, z=0, w=1),
                                         orientation_covariance=np.zeros(9), angular_velocity=v(x=0.1, y=0, z=0),
                                         angular_velocity_covariance=np.zeros(9), linear_acceleration=v(x=0, y=0, z=1.0),
                                         linear_acceleration_covariance=np.zeros(9))
            msgs.append((st, ic, ts.serialize_ros1(m, "sensor_msgs/msg/Imu")))
        for k in range(n_scans):
            end = t0 + (k + 1) * 100_000_000          # velodyne driver stamps at scan end
            n = 50
            dt = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "time"],
                           "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
                           "offsets": [0, 4, 8, 12, 16, 20], "itemsize": 24})
            a = np.zeros(n, dt)
            a["x"] = np.linspace(1, 10, n)
            a["ring"] = np.arange(n) % 16
            a["time"] = np.linspace(-0.1, 0.0, n)
            sec, ns = divmod(end, 10**9)
            hdr = T["std_msgs/msg/Header"](seq=k, stamp=T["builtin_interfaces/msg/Time"](sec=sec, nanosec=ns), frame_id="velo")
            fields = [PF(name=nm, offset=o, datatype=d, count=1) for nm, o, d in
                      [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("ring", 16, 4), ("time", 20, 7)]]
            m = T["sensor_msgs/msg/PointCloud2"](header=hdr, height=1, width=n, fields=fields, is_bigendian=False,
                                                 point_step=24, row_step=24 * n,
                                                 data=np.frombuffer(a.tobytes(), np.uint8), is_dense=True)
            msgs.append((end, lc, ts.serialize_ros1(m, "sensor_msgs/msg/PointCloud2")))
        for st, c, raw in sorted(msgs, key=lambda x: x[0]):
            w.write(c, st, raw)
    return t0


def test_convert_ros1_velodyne_end_to_end(tmp_path):
    from rosbags.highlevel import AnyReader
    bag = tmp_path / "src.bag"
    t0 = write_ros1_bag(bag)
    gt = tmp_path / "gt.txt"
    gt.write_text("".join(f"{(t0 + i * 10**8) / 1e9:.9f} {i} 0 0 0 0 0 1\n" for i in range(6)))
    out = tmp_path / "out" / "seq"
    source = {"lidar": {"topic": "/velodyne_points", "decoder": "velodyne"},
              "imu": {"topic": "/imu", "acc_unit": "g"}, "gt": {"format": "tum", "time_offset": 0.5}}
    meta = convert_sequence("seq", {"bag": str(bag), "gt": str(gt)}, source, out, log=lambda *_: None)
    assert meta["stats"]["scans"] == 5 and meta["stats"]["imu"] == 100
    assert meta["sensor"]["scan_lines"] == 16
    assert meta["sensor"]["scan_rate"] == pytest.approx(10.0)
    assert meta["stats"]["scan_duration_ms"] == pytest.approx(100.0, abs=0.01)
    assert (out / "gt.tum").exists() and not out.with_name("seq.tmp").exists()
    g = np.loadtxt(out / "gt.tum")
    assert g[0, 0] == pytest.approx(t0 / 1e9 + 0.5)

    ts = _typestore()
    with AnyReader([out / "bag"], default_typestore=ts) as r:
        topics = {c.topic: c for c in r.connections}
        assert set(topics) == {"/liobench/lidar", "/liobench/imu"}
        first_lidar = None
        for c, t, raw in r.messages():
            m = ts.deserialize_cdr(raw, c.msgtype)
            if c.topic == "/liobench/imu":
                assert m.linear_acceleration.z == pytest.approx(9.81)
            elif first_lidar is None:
                first_lidar = (t, m)
        t, m = first_lidar
        start = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        assert start == m.timebase
        assert abs(start - t0) < 1000                         # scan start = header - 0.1 s
        assert t == start + m.points[-1].offset_time           # written at scan end
        assert [p.offset_time for p in m.points] == sorted(p.offset_time for p in m.points)

    # second call is a no-op; adding nothing changes nothing
    before = (out / "meta.yaml").read_text()
    convert_sequence("seq", {"bag": str(bag), "gt": str(gt)}, source, out, log=lambda *_: None)
    assert (out / "meta.yaml").read_text() == before


def test_imu_cdr_roundtrip():
    ts = _typestore()
    raw = _imu_cdr(ts, 1_700_000_000_500_000_000, [1, 2, 3], [4, 5, 6])
    m = ts.deserialize_cdr(raw, "sensor_msgs/msg/Imu")
    assert (m.header.stamp.sec, m.header.stamp.nanosec) == (1_700_000_000, 500_000_000)
    assert (m.angular_velocity.z, m.linear_acceleration.x) == (3, 4)
