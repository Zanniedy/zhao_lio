import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_nodes(context):
    share = get_package_share_directory('zhao_lio_ros2')
    cfg = os.path.join(share, 'config', LaunchConfiguration('config').perform(context) + '.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    overrides = {
        'update_strategy': LaunchConfiguration('update_strategy').perform(context),
        'use_sim_time': use_sim_time,
    }
    nodes = [Node(package='zhao_lio_ros2', executable='zhao_lio_node', name='zhao_lio',
                  output='screen', parameters=[cfg, overrides])]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                          arguments=['-d', os.path.join(share, 'rviz', 'zhao_lio.rviz')],
                          parameters=[{'use_sim_time': use_sim_time}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='mid360', description='mid360 | avia | tracked'),
        DeclareLaunchArgument('update_strategy', default_value='pointwise_prematch'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when playing a bag with --clock'),
        OpaqueFunction(function=make_nodes),
    ])
