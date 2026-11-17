from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('dynablox_ros2'))
    replay = Path(get_package_share_directory('zhao_lio_pointcloud_preprocessor'))
    lio = Path(get_package_share_directory('zhao_lio_ros2'))
    mapper = Path(get_package_share_directory('map_3d_to_2d'))
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Path to the MID70 rosbag2 directory'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map2d', default_value='true'),
        DeclareLaunchArgument('config', default_value=str(share / 'config' / 'mid70.yaml')),
        DeclareLaunchArgument('lio_config', default_value=str(lio / 'config' / 'mid70_x5_visual.yaml')),
        DeclareLaunchArgument('optimizer_config', default_value=str(replay / 'config' / 'pointcloud_optimizer.yaml')),
        DeclareLaunchArgument('map2d_config', default_value=str(mapper / 'config' / 'dynablox.yaml')),
        DeclareLaunchArgument('publish_calibrated_tf', default_value='true'),
        DeclareLaunchArgument('publish_base_tf', default_value='true'),
        Node(
            package='dynablox_ros2', executable='dynablox_node', name='dynablox',
            parameters=[LaunchConfiguration('config'), {'use_sim_time': True}], output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz_dynablox',
            arguments=['-d', str(share / 'rviz' / 'dynablox.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')), output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(mapper / 'launch' / 'mapper.launch.py')),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz'), 'use_sim_time': 'true',
                'config': LaunchConfiguration('map2d_config'),
                'rviz_config': str(mapper / 'rviz' / 'dynablox.rviz'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('map2d')),
        ),
        TimerAction(period=5.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(replay / 'launch' / 'replay_mapping.launch.py')),
            launch_arguments={'bag': LaunchConfiguration('bag'), 'rate': LaunchConfiguration('rate'),
                              'rviz': 'false', 'dynamic_filter': 'false',
                              'lio_config': LaunchConfiguration('lio_config'),
                              'optimizer_config': LaunchConfiguration('optimizer_config'),
                              'publish_calibrated_tf': LaunchConfiguration('publish_calibrated_tf'),
                              'publish_base_tf': LaunchConfiguration('publish_base_tf')}.items(),
        )]),
    ])
