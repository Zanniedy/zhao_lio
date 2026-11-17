import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def set_stamp(header, seconds):
    nanoseconds = round(seconds * 1e9)
    header.stamp.sec, header.stamp.nanosec = divmod(nanoseconds, 1_000_000_000)


def make_cloud(seconds):
    message = PointCloud2()
    message.header.frame_id = 'test_lidar'
    set_stamp(message.header, seconds)
    message.height = 1
    message.width = 41 * 25
    message.point_step = 24
    message.row_step = message.width * message.point_step
    message.is_dense = True
    message.fields = [PointField(name=name, offset=index * 4, count=1,
                                 datatype=PointField.FLOAT64 if index == 4 else PointField.FLOAT32)
                      for index, name in enumerate(('x', 'y', 'z', 'intensity', 'timestamp'))]
    data = bytearray()
    for horizontal in range(-20, 21):
        for vertical in range(-12, 13):
            point_time = seconds + 0.08 * (len(data) // 24) / (message.width - 1)
            angle = point_time - 10.0
            world_x = 8.0 + 0.15 * math.sin(horizontal * 0.7)
            world_y = horizontal * 0.12
            local_x = math.cos(angle) * world_x + math.sin(angle) * world_y
            local_y = -math.sin(angle) * world_x + math.cos(angle) * world_y
            data.extend(struct.pack('<ffffd', local_x, local_y, vertical * 0.12, 1.0, point_time))
    message.data = bytes(data)
    return message


def test_tf_direction_imu_waiting_and_timeout(tmp_path):
    package = 'zhao_lio_pointcloud_preprocessor'
    executable = Path(get_package_prefix(package)) / 'lib' / package / f'{package}_node'
    parameters = {
        'input_topic': '/test/cloud', 'output_topic': '/test/optimized', 'imu_topic': '/test/imu',
        'lidar_frame': 'test_lidar', 'body_frame': 'test_imu', 'voxel_size': 0.0,
        'publish_calibrated_tf': True, 'dynamic.imu_wait_timeout': 0.5,
        'extrinsic_R': [1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0],
        'extrinsic_T': [0.0, 0.0, 0.0],
    }
    config = tmp_path / 'params.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
    rclpy.init()
    node = rclpy.create_node('dynamic_filter_test')
    statuses, outputs, removed = [], [], []
    subscriptions = [
        node.create_subscription(String, '/zhao_lio_x5/dynamic_filter/status', statuses.append, 10),
        node.create_subscription(PointCloud2, '/test/optimized', outputs.append, qos_profile_sensor_data),
        node.create_subscription(PointCloud2, '/zhao_lio_x5/points/dynamic', removed.append, qos_profile_sensor_data),
    ]
    imu_publisher = node.create_publisher(Imu, '/test/imu', qos_profile_sensor_data)
    cloud_publisher = node.create_publisher(PointCloud2, '/test/cloud', qos_profile_sensor_data)
    log = (tmp_path / 'node.log').open('w')
    process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(config)],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def wait_for(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path / 'node.log').read_text()
            rclpy.spin_once(node, timeout_sec=0.01)
        assert predicate(), (tmp_path / 'node.log').read_text()

    def publish_imu(seconds, frame='test_imu'):
        message = Imu()
        message.header.frame_id = frame
        set_stamp(message.header, seconds)
        message.angular_velocity.y = -1.0
        imu_publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.002)

    try:
        wait_for(lambda: cloud_publisher.get_subscription_count() > 0 and imu_publisher.get_subscription_count() > 0)
        time.sleep(0.2)
        for sample in range(21):
            publish_imu(9.9 + sample * 0.005)
        for frame in range(4):
            input_cloud = make_cloud(10.0 + frame * 0.1)
            cloud_publisher.publish(input_cloud)
            if frame == 0:
                rclpy.spin_once(node, timeout_sec=0.03)
                assert not outputs
            for sample in range(1, 21):
                publish_imu(10.0 + frame * 0.1 + sample * 0.005)
            wait_for(lambda: len(outputs) == frame + 1 and len(statuses) == frame + 1 and len(removed) == frame + 1)
            assert outputs[-1].data == input_cloud.data
            assert outputs[-1].header == input_cloud.header
            assert removed[-1].width == 0
            assert f'reason={"warmup" if frame == 0 else "matched"}' in statuses[-1].data
        input_cloud = make_cloud(11.0)
        cloud_publisher.publish(input_cloud)
        wait_for(lambda: len(outputs) == 5 and len(statuses) == 5)
        assert 'reason=imu_wait_timeout' in statuses[-1].data
        assert outputs[-1].data == input_cloud.data
        publish_imu(11.1, 'missing_tf_frame')
        cloud_publisher.publish(make_cloud(11.1))
        wait_for(lambda: len(outputs) == 6 and len(statuses) == 6)
        assert 'reason=imu_wait_timeout' in statuses[-1].data
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def test_whole_cluster_removal_and_markers(tmp_path):
    package = 'zhao_lio_pointcloud_preprocessor'
    executable = Path(get_package_prefix(package)) / 'lib' / package / f'{package}_node'
    parameters = {
        'input_topic': '/test/cluster_input', 'output_topic': '/test/cluster_output',
        'imu_topic': '/test/cluster_imu', 'lidar_frame': 'test_lidar',
        'body_frame': 'test_base', 'height_frame': 'test_base',
        'voxel_size': 0.0, 'dynamic.cluster_tracking_enabled': True,
        'publish_calibrated_tf': True,
        'extrinsic_R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        'extrinsic_T': [0.0, 0.0, 1.2],
    }
    config = tmp_path / 'cluster_params.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
    rclpy.init()
    node = rclpy.create_node('cluster_filter_test')
    outputs, removed, markers = [], [], []
    subscriptions = [
        node.create_subscription(PointCloud2, '/test/cluster_output', outputs.append, qos_profile_sensor_data),
        node.create_subscription(PointCloud2, '/zhao_lio_x5/points/dynamic', removed.append, qos_profile_sensor_data),
        node.create_subscription(MarkerArray, '/zhao_lio_x5/dynamic_filter/clusters', markers.append, 10),
    ]
    publisher = node.create_publisher(PointCloud2, '/test/cluster_input', qos_profile_sensor_data)
    imu_publisher = node.create_publisher(Imu, '/test/cluster_imu', qos_profile_sensor_data)
    log_path = tmp_path / 'cluster_node.log'
    log = log_path.open('w')
    process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(config)],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def wait_for(predicate):
        deadline = time.monotonic() + 5.0
        while not predicate() and time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=0.01)
        assert predicate(), log_path.read_text()

    try:
        wait_for(lambda: publisher.get_subscription_count() > 0 and imu_publisher.get_subscription_count() > 0)
        time.sleep(0.2)
        for frame in range(9):
            seconds = 10.0 + frame * 0.1
            for sample in range(21):
                message = Imu()
                message.header.frame_id = 'test_base'
                set_stamp(message.header, seconds - 0.1 + sample * 0.005)
                imu_publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.002)
            background = [(8.0 + 0.15 * math.sin(horizontal * 0.7), horizontal * 0.12,
                           vertical * 0.12 - 1.2)
                          for horizontal in range(-25, 26) for vertical in range(20)]
            body = [(3.0, horizontal * 0.09 + frame * 0.07, 0.06 + vertical * 0.1 - 1.2)
                    for horizontal in range(-2, 3) for vertical in range(18)]
            records = [struct.pack('<ffffd', *point, float(index), seconds)
                       for index, point in enumerate(background + body)]
            message = make_cloud(seconds)
            message.width = len(records)
            message.row_step = message.width * message.point_step
            message.data = b''.join(records)
            publisher.publish(message)
            wait_for(lambda: len(outputs) == frame + 1 and len(removed) == frame + 1 and len(markers) == frame + 1)
            if frame < 4:
                assert removed[-1].width == 0
            else:
                assert removed[-1].width == len(body)
                assert bytes(outputs[-1].data) == b''.join(records[:len(background)])
                assert bytes(removed[-1].data) == b''.join(records[len(background):])
                boxes = [marker for marker in markers[-1].markers
                         if marker.action == Marker.ADD and marker.type == Marker.CUBE]
                assert len(boxes) == 1 and boxes[0].color.r == 1.0
                assert boxes[0].header == message.header
                assert any('moving' in marker.text and 'n=90' in marker.text
                           for marker in markers[-1].markers)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def test_height_and_intensity_with_nonidentity_tf(tmp_path):
    package = 'zhao_lio_pointcloud_preprocessor'
    executable = Path(get_package_prefix(package)) / 'lib' / package / f'{package}_node'
    parameters = {
        'input_topic': '/test/basic_input', 'output_topic': '/test/basic_output',
        'lidar_frame': 'test_lidar', 'body_frame': 'test_base', 'height_frame': 'test_base',
        'voxel_size': 0.0, 'dynamic.enabled': False, 'height_filter_enabled': True,
        'min_z': -0.125, 'max_z': 4.0, 'min_intensity': 5.0, 'publish_calibrated_tf': True,
        'extrinsic_R': [1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0],
        'extrinsic_T': [0.0, 0.0, 1.0],
    }
    config = tmp_path / 'basic_params.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
    rclpy.init()
    node = rclpy.create_node('basic_filter_test')
    outputs, statuses = [], []
    output_subscription = node.create_subscription(PointCloud2, '/test/basic_output', outputs.append,
                                                   qos_profile_sensor_data)
    status_subscription = node.create_subscription(String, '/zhao_lio_x5/point_filter/status', statuses.append, 10)
    publisher = node.create_publisher(PointCloud2, '/test/basic_input', qos_profile_sensor_data)
    log = (tmp_path / 'basic_node.log').open('w')
    process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(config)],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def wait_for(predicate):
        deadline = time.monotonic() + 5.0
        while not predicate() and time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path / 'basic_node.log').read_text()
            rclpy.spin_once(node, timeout_sec=0.01)
        assert predicate(), (tmp_path / 'basic_node.log').read_text()

    try:
        wait_for(lambda: publisher.get_subscription_count() > 0 and node.count_publishers('/test/basic_output') > 0)
        time.sleep(0.2)
        message = make_cloud(10.0)
        records = [struct.pack('<ffffd', 2.0, height, 3.0, intensity, 10.0 + index * 0.01)
                   for index, (height, intensity) in enumerate(
                       [(-1.25, 20.0), (-1.125, 5.0), (0.0, 4.0), (0.0, 20.0)])]
        message.width = 4
        message.row_step = message.width * message.point_step
        message.data = b''.join(records)
        publisher.publish(message)
        wait_for(lambda: len(outputs) == 1 and len(statuses) == 1)
        assert bytes(outputs[-1].data) == records[1] + records[3]
        assert outputs[-1].header == message.header
        assert 'low_height=1 low_intensity=1 height_applied=1 intensity_applied=1' in statuses[-1].data
        message.header.frame_id = 'missing_height_tf'
        publisher.publish(message)
        wait_for(lambda: len(outputs) == 2 and len(statuses) == 2)
        assert bytes(outputs[-1].data) == records[0] + records[1] + records[3]
        assert 'height_applied=0 intensity_applied=1' in statuses[-1].data
        message.header.frame_id = 'test_lidar'
        message.fields = [field for field in message.fields if field.name != 'intensity']
        publisher.publish(message)
        wait_for(lambda: len(outputs) == 3 and len(statuses) == 3)
        assert bytes(outputs[-1].data) == records[1] + records[2] + records[3]
        assert 'height_applied=1 intensity_applied=0' in statuses[-1].data
        records = [struct.pack('<ffffd', 2.0, height, 3.0, 20.0, 10.0)
                   for height in (3.0, 3.25)]
        message.width = 2
        message.row_step = message.width * message.point_step
        message.data = b''.join(records)
        publisher.publish(message)
        wait_for(lambda: len(outputs) == 4 and len(statuses) == 4)
        assert bytes(outputs[-1].data) == records[0]
        assert 'high_height=1' in statuses[-1].data
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        node.destroy_subscription(output_subscription)
        node.destroy_subscription(status_subscription)
        node.destroy_node()
        rclpy.shutdown()
