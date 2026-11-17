import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster


def cloud_message(seconds):
    cloud = PointCloud2()
    cloud.header.frame_id = 'test_lidar'
    cloud.header.stamp.sec = int(seconds)
    cloud.header.stamp.nanosec = round((seconds - int(seconds)) * 1e9)
    cloud.height = 2
    cloud.width = 25
    cloud.point_step = 32
    cloud.row_step = cloud.width * cloud.point_step + 8
    cloud.is_dense = True
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='timestamp', offset=16, datatype=PointField.FLOAT64, count=1),
        PointField(name='label', offset=24, datatype=PointField.UINT32, count=1),
    ]
    records = []
    raw = bytearray()
    for row in range(cloud.height):
        for column in range(cloud.width):
            index = row * cloud.width + column
            record = struct.pack('<ffffdII', 4.05, column * 0.1 - 1.2, row * 0.2,
                                 10.0 + index, seconds + index * 0.001, index, 0xABCDEF01)
            records.append(record)
            raw.extend(record)
        raw.extend(b'ROWPAD00')
    cloud.data = bytes(raw)
    return cloud, b''.join(records)


@pytest.mark.parametrize('with_tf', [True, False])
def test_raw_record_partition_and_missing_tf(tmp_path, with_tf):
    executable = Path(get_package_prefix('dynablox_ros2')) / 'lib/dynablox_ros2/dynablox_node'
    rclpy.init()
    node = rclpy.create_node('dynablox_adapter_test')
    outputs = {'kept': [], 'dynamic': [], 'pending': []}
    statuses = []
    subscriptions = [node.create_subscription(PointCloud2, '/dynablox/' + name, values.append,
                                              qos_profile_sensor_data) for name, values in outputs.items()]
    subscriptions.append(node.create_subscription(String, '/dynablox/status', statuses.append, 10))
    broadcaster = StaticTransformBroadcaster(node)
    if with_tf:
        transform = TransformStamped()
        transform.header.frame_id = 'test_world'
        transform.child_frame_id = 'test_lidar'
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
    publisher = node.create_publisher(PointCloud2, '/test/dynablox_input', qos_profile_sensor_data)
    log = (tmp_path / 'node.log').open('w')
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p', 'input_topic:=/test/dynablox_input',
        '-p', 'world_frame:=test_world', '-p', 'lidar_frame:=test_lidar', '-p', 'num_threads:=1',
        '-p', 'use_sim_time:=false', '-p', 'tf_wait:=0.2',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def wait(predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path / 'node.log').read_text()
            rclpy.spin_once(node, timeout_sec=0.01)
        assert predicate(), (tmp_path / 'node.log').read_text()

    try:
        wait(lambda: publisher.get_subscription_count() == 1)
        time.sleep(0.3)
        cloud, expected_records = cloud_message(10.0)
        publisher.publish(cloud)
        wait(lambda: all(outputs.values()) and statuses)
        if with_tf:
            assert bytes(outputs['kept'][-1].data) == expected_records
            assert outputs['kept'][-1].header == cloud.header
            assert outputs['kept'][-1].fields == cloud.fields
            assert outputs['kept'][-1].point_step == cloud.point_step
            assert outputs['dynamic'][-1].width == 0
            assert outputs['pending'][-1].width == 0
            assert 'reason=ok' in statuses[-1].data
            invalid, _ = cloud_message(10.1)
            data = bytearray(invalid.data)
            struct.pack_into('<d', data, 16, 999.0)
            invalid.data = bytes(data)
            publisher.publish(invalid)
            wait(lambda: len(statuses) >= 2 and len(outputs['pending']) >= 2)
            assert 'invalid_scan:' in statuses[-1].data
            assert outputs['pending'][-1] == invalid
        else:
            assert outputs['pending'][-1] == cloud
            assert outputs['kept'][-1].width == 0
            assert outputs['dynamic'][-1].width == 0
            assert 'reason=tf_timeout' in statuses[-1].data
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
