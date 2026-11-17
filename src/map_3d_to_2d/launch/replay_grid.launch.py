import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('map_3d_to_2d')
    lio_share = get_package_share_directory('zhao_lio_pointcloud_preprocessor')
    return LaunchDescription([
        DeclareLaunchArgument('bag'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'params.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'mapper.launch.py')),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz'), 'use_sim_time': 'true',
                'config': LaunchConfiguration('config'),
            }.items(),
        ),
        TimerAction(period=3.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(lio_share, 'launch', 'replay_mapping.launch.py')),
            launch_arguments={
                'bag': LaunchConfiguration('bag'), 'rate': LaunchConfiguration('rate'),
                'rviz': 'false',
            }.items(),
        )]),
    ])
