import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('zhao_lio_pointcloud_preprocessor')
    config = os.path.join(share, 'config', 'pointcloud_optimizer.yaml')
    return LaunchDescription([
        Node(
            package='zhao_lio_pointcloud_preprocessor',
            executable='zhao_lio_pointcloud_preprocessor_node',
            name='zhao_lio_pointcloud_preprocessor',
            output='screen',
            parameters=[config],
        ),
    ])
