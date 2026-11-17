import math
import os
from pathlib import Path
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Vector3
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def test_height_projection_and_cached_updates():
    rclpy.init()
    node = rclpy.create_node('projection_probe')
    prefix = '/projection_test_' + str(os.getpid())
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cloud_pub = node.create_publisher(PointCloud2, prefix + '/cloud', qos)
    range_pub = node.create_publisher(Vector3, prefix + '/set_range', 1)
    maps = []
    ranges = []
    markers = []
    node.create_subscription(OccupancyGrid, prefix + '/map', maps.append, qos)
    node.create_subscription(MarkerArray, prefix + '/map/markers', markers.append, qos)
    node.create_subscription(Vector3, prefix + '/range', ranges.append, qos)
    executable = Path(get_package_prefix('map_3d_to_2d')) / 'lib/map_3d_to_2d/mapper'
    process = subprocess.Popen([
        str(executable), '--ros-args',
        '-p', 'input_topic:=' + prefix + '/cloud',
        '-p', 'output_topic:=' + prefix + '/map',
        '-p', 'ground_z:=-1.0', '-p', 'z_min:=0.25', '-p', 'z_max:=1.5',
        '-p', 'resolution:=0.25', '-p', 'padding:=0.0',
        '-r', '/map_3d_to_2d/set_range:=' + prefix + '/set_range',
        '-r', '/map_3d_to_2d/range:=' + prefix + '/range',
        '-r', '/map_3d_to_2d/status:=' + prefix + '/status',
        '-r', '/map_3d_to_2d/z_min:=' + prefix + '/z_min',
        '-r', '/map_3d_to_2d/z_max:=' + prefix + '/z_max',
    ])

    def wait_for(predicate):
        deadline = time.monotonic() + 12.0
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            assert process.poll() is None, 'mapper exited unexpectedly'
        assert predicate(), 'timed out waiting for mapper output'

    def set_range(minimum, maximum):
        previous = len(maps)
        range_pub.publish(Vector3(x=minimum, y=maximum))
        wait_for(lambda: len(maps) > previous)

    try:
        wait_for(lambda: cloud_pub.get_subscription_count() > 0 and
                 range_pub.get_subscription_count() > 0 and len(ranges) > 0)
        assert (ranges[-1].x, ranges[-1].y, ranges[-1].z) == (0.25, 1.5, -1.0)
        header = Header(frame_id='lio_odom')
        header.stamp.sec = 123
        cloud_pub.publish(create_cloud_xyz32(header, [
            (0.0, 0.0, -0.75), (1.0, 0.0, 0.5), (2.0, 0.0, -0.8),
            (3.0, 0.0, 0.6), (81.0, 0.0, 0.0), (math.nan, 0.0, 0.0),
        ]))
        wait_for(lambda: len(maps) > 0)
        initial = maps[-1]
        assert initial.data.count(100) == 3
        assert initial.info.height == 1
        assert initial.info.origin.position.z == -1.0
        assert initial.header.frame_id == 'lio_odom'
        assert initial.header.stamp.sec == 123
        wait_for(lambda: len(markers) > 0)
        assert len(markers[-1].markers[1].points) == 3
        assert markers[-1].markers[1].color.r == markers[-1].markers[1].color.g
        assert markers[-1].markers[0].color.r == markers[-1].markers[0].color.g
        set_range(2.0, 3.0)
        assert maps[-1].data.count(100) == 0
        assert set(maps[-1].data) == {-1}
        assert maps[-1].info == initial.info
        wait_for(lambda: markers[-1].markers[1].action == Marker.DELETE)
        set_range(1.5, 2.0)
        assert maps[-1].data.count(100) == 2
        set_range(0.25, 1.5)
        assert maps[-1].data == initial.data
        late_maps = []
        node.create_subscription(OccupancyGrid, prefix + '/map', late_maps.append, qos)
        wait_for(lambda: len(late_maps) > 0)
        assert late_maps[-1].data == initial.data
        previous = len(maps)
        cloud_pub.publish(create_cloud_xyz32(header, []))
        wait_for(lambda: len(maps) > previous)
        assert maps[-1].data.count(100) == 0
    finally:
        process.terminate()
        process.wait(timeout=8)
        node.destroy_node()
        rclpy.shutdown()
