"""Recorder node: logs LiDAR arrivals and odometry output of the algorithm under test.

Runs as its own process (so its CPU is not attributed to the algorithm):

    python -m liobench.recorder --lidar-topic /livox/lidar --lidar-type livox_ros_driver2/msg/CustomMsg \
        --imu-topic /vectornav/IMU --odom-topic /aft_mapped_to_init --out runs/x

Writes, line-buffered so a killed run still leaves usable data:
    lidar.csv   recv_time,stamp          (recv_time = time.monotonic())
    odom.csv    recv_time,stamp,x,y,z,qx,qy,qz,qw
    ready       created once the algorithm has subscribed to the LiDAR and IMU topics
"""
from __future__ import annotations

import argparse
import importlib
import signal
import struct
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def msg_class(type_str: str):
    pkg, _, name = type_str.split("/")
    return getattr(importlib.import_module(f"{pkg}.msg"), name)


def header_stamp(raw: bytes) -> float:
    """std_msgs/Header is the first field: 4-byte CDR encapsulation, int32 sec, uint32 nanosec."""
    sec, nsec = struct.unpack_from("<iI", raw, 4) if raw[1] == 1 else struct.unpack_from(">iI", raw, 4)
    return sec + nsec * 1e-9


class Recorder(Node):
    def __init__(self, a):
        super().__init__("liobench_recorder")
        out = Path(a.out)
        self.lidar_f = open(out / "lidar.csv", "w", buffering=1)
        self.odom_f = open(out / "odom.csv", "w", buffering=1)
        self.lidar_f.write("recv_time,stamp\n")
        self.odom_f.write("recv_time,stamp,x,y,z,qx,qy,qz,qw\n")
        self.ready_path = out / "ready"
        self.a = a
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(msg_class(a.lidar_type), a.lidar_topic, self.on_lidar, qos, raw=True)
        self.create_subscription(msg_class(a.odom_type), a.odom_topic, self.on_odom, qos)
        self.create_timer(0.2, self.check_ready)

    def on_lidar(self, raw: bytes):
        self.lidar_f.write(f"{time.monotonic():.6f},{header_stamp(raw):.9f}\n")

    def on_odom(self, msg):
        t = time.monotonic()
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        q = p.orientation
        self.odom_f.write(f"{t:.6f},{st:.9f},{p.position.x:.9f},{p.position.y:.9f},{p.position.z:.9f},"
                          f"{q.x:.9f},{q.y:.9f},{q.z:.9f},{q.w:.9f}\n")

    def check_ready(self):
        if self.ready_path.exists():
            return
        # The recorder itself is one subscriber of the LiDAR topic.
        if self.count_subscribers(self.a.lidar_topic) >= 2 and self.count_subscribers(self.a.imu_topic) >= 1:
            self.ready_path.write_text(f"{time.monotonic():.6f}\n")

    def close(self):
        self.lidar_f.close()
        self.odom_f.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--lidar-topic", required=True)
    p.add_argument("--lidar-type", required=True)
    p.add_argument("--imu-topic", required=True)
    p.add_argument("--odom-topic", required=True)
    p.add_argument("--odom-type", default="nav_msgs/msg/Odometry")
    p.add_argument("--out", required=True)
    a = p.parse_args()

    rclpy.init()
    node = Recorder(a)
    stop = False

    def handler(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    try:
        while not stop and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
