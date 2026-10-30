"""Canonical scan representation and the Livox CustomMsg wire format (ROS2 CDR and ROS1)."""
from __future__ import annotations

import struct
from dataclasses import dataclass

import numpy as np

LIVOX_TAG_NORMAL = 0x10   # "normal point" tag; Point-LIO keeps tags 0x00 and 0x10 only

CUSTOM_MSG_DEF = """std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
"""
CUSTOM_POINT_DEF = """uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""

_FIELDS = dict(names=["offset_time", "x", "y", "z", "reflectivity", "tag", "line"],
               formats=["<u4", "<f4", "<f4", "<f4", "u1", "u1", "u1"],
               offsets=[0, 4, 8, 12, 16, 17, 18])
CDR_POINT = np.dtype({**_FIELDS, "itemsize": 20})   # CDR: each element re-aligned to 4 bytes
ROS1_POINT = np.dtype({**_FIELDS, "itemsize": 19})  # ROS1: packed


@dataclass
class Scan:
    """One LiDAR scan. stamp_ns is the time of the earliest point; offset_ns >= 0 per point."""
    stamp_ns: int
    xyz: np.ndarray          # (N, 3) float32
    intensity: np.ndarray    # (N,) float32
    offset_ns: np.ndarray    # (N,) uint32
    line: np.ndarray         # (N,) uint8
    tag: np.ndarray          # (N,) uint8

    def __len__(self):
        return len(self.offset_ns)

    @property
    def end_ns(self) -> int:
        return self.stamp_ns + (int(self.offset_ns.max()) if len(self) else 0)

    @staticmethod
    def from_times(base_ns: int, rel_ns: np.ndarray, xyz, intensity, line, tag=None) -> "Scan":
        """Build from per-point times base_ns + rel_ns (rel_ns may be negative or float).
        Absolute epoch nanoseconds do not fit float64 exactly, hence the integer base.
        Drops non-finite points and sorts by time; stamp_ns becomes the earliest point."""
        xyz = np.asarray(xyz, np.float64)
        rel = np.asarray(rel_ns, np.float64)
        ok = np.all(np.isfinite(xyz), axis=1) & np.isfinite(rel)
        rel = rel[ok]
        if len(rel) == 0:
            return Scan(int(base_ns), np.zeros((0, 3), np.float32), np.zeros(0, np.float32),
                        np.zeros(0, np.uint32), np.zeros(0, np.uint8), np.zeros(0, np.uint8))
        order = np.argsort(rel, kind="stable")
        rel = rel[order]
        shift = int(np.floor(rel[0]))
        tag = np.full(len(ok), LIVOX_TAG_NORMAL, np.uint8) if tag is None else np.asarray(tag, np.uint8)
        return Scan(int(base_ns) + shift, xyz[ok][order].astype(np.float32),
                    np.asarray(intensity, np.float32)[ok][order],
                    np.round(rel - shift).astype(np.uint32),
                    np.clip(np.asarray(line), 0, 255).astype(np.uint8)[ok][order],
                    tag[ok][order])


def _align(pos: int, n: int) -> int:
    return (pos + n - 1) & ~(n - 1)


def decode_custom_cdr(raw: bytes) -> tuple[int, str, int, np.ndarray]:
    """-> (header_stamp_ns, frame_id, timebase, points structured array)."""
    if raw[1] != 1:
        raise ValueError("big-endian CDR not supported")
    p = memoryview(raw)[4:]                       # payload after the encapsulation header
    sec, nsec, n = struct.unpack_from("<iII", p, 0)
    frame_id = bytes(p[12:12 + n - 1]).decode()
    pos = _align(12 + n, 8)
    timebase, _point_num = struct.unpack_from("<QI", p, pos)
    pos += 8 + 4 + 1 + 3
    (count,) = struct.unpack_from("<I", p, pos)
    pos += 4
    buf = bytes(p[pos:pos + count * 20])
    if count:
        buf += b"\0" * (count * 20 - len(buf))    # the last element has no trailing padding
    pts = np.frombuffer(buf, CDR_POINT, count)
    return sec * 1_000_000_000 + nsec, frame_id, timebase, pts


def decode_custom_ros1(raw: bytes) -> tuple[int, str, int, np.ndarray]:
    _seq, sec, nsec, n = struct.unpack_from("<IIII", raw, 0)
    frame_id = raw[16:16 + n].decode()
    pos = 16 + n
    timebase, _point_num = struct.unpack_from("<QI", raw, pos)
    pos += 8 + 4 + 1 + 3
    (count,) = struct.unpack_from("<I", raw, pos)
    pos += 4
    pts = np.frombuffer(raw[pos:pos + count * 19], ROS1_POINT, count)
    return sec * 1_000_000_000 + nsec, frame_id, timebase, pts


def encode_custom_cdr(scan: Scan, frame_id: str = "lidar") -> bytes:
    """Serialize as livox_ros_driver2/msg/CustomMsg (CDR, little endian).
    header.stamp = timebase = scan start; offset_time relative to it."""
    sec, nsec = divmod(int(scan.stamp_ns), 1_000_000_000)
    fid = frame_id.encode() + b"\0"
    out = bytearray(b"\x00\x01\x00\x00")
    out += struct.pack("<iII", sec, nsec, len(fid)) + fid
    out += b"\0" * (_align(len(out) - 4, 8) - (len(out) - 4))
    n = len(scan)
    out += struct.pack("<QIB3x", int(scan.stamp_ns), n, 0)
    out += struct.pack("<I", n)
    if n:
        pts = np.zeros(n, CDR_POINT)
        pts["offset_time"] = scan.offset_ns
        pts["x"], pts["y"], pts["z"] = scan.xyz[:, 0], scan.xyz[:, 1], scan.xyz[:, 2]
        pts["reflectivity"] = np.clip(scan.intensity, 0, 255).astype(np.uint8)
        pts["tag"] = scan.tag
        pts["line"] = scan.line
        out += pts.tobytes()[:-1]                 # drop trailing padding of the last element
    return bytes(out)
