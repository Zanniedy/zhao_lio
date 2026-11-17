import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    preprocessor_share = get_package_share_directory('zhao_lio_pointcloud_preprocessor')
    lio_share = get_package_share_directory('zhao_lio_ros2')
    optimizer_config = os.path.join(preprocessor_share, 'config', 'pointcloud_optimizer.yaml')
    lio_config = os.path.join(lio_share, 'config', 'mid70_x5_visual.yaml')
    rviz_config = os.path.join(preprocessor_share, 'rviz', 'zhao_lio_x5_mapping.rviz')

    bag = LaunchConfiguration('bag')
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Path to the MID70 rosbag2 directory'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('lio_config', default_value=lio_config),
        DeclareLaunchArgument('optimizer_config', default_value=optimizer_config),
        DeclareLaunchArgument('dynamic_filter', default_value='true'),
        DeclareLaunchArgument('publish_calibrated_tf', default_value='true'),
        DeclareLaunchArgument('publish_base_tf', default_value='true',
                              description='Use the URDF zero-offset imu_link/base_link relationship for replay'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='replay_imu_to_base', output='screen',
            arguments=['--frame-id', 'imu_link', '--child-frame-id', 'base_link',
                       '--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0'],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('publish_base_tf')),
        ),
        Node(
            package='zhao_lio_pointcloud_preprocessor',
            executable='zhao_lio_pointcloud_preprocessor_node',
            name='zhao_lio_pointcloud_preprocessor',
            output='screen',
            parameters=[LaunchConfiguration('lio_config'), LaunchConfiguration('optimizer_config'),
                        {'use_sim_time': True,
                         'dynamic.enabled': ParameterValue(LaunchConfiguration('dynamic_filter'), value_type=bool),
                         'publish_calibrated_tf': ParameterValue(
                             LaunchConfiguration('publish_calibrated_tf'), value_type=bool)}],
        ),
        Node(
            package='zhao_lio_ros2',
            executable='zhao_lio_node',
            name='zhao_lio_x5',
            output='screen',
            parameters=[LaunchConfiguration('lio_config'), {'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_zhao_lio_x5',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play', bag,
                '--clock',
                '--rate', LaunchConfiguration('rate'),
                '--topics', '/input/imu', '/input/lidar',
            ],
            output='screen',
        ),
    ])
