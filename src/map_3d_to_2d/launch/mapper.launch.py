from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory("map_3d_to_2d")
    params_file = os.path.join(pkg_path, "config", "params.yaml")
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}

    return LaunchDescription([
            DeclareLaunchArgument('rviz', default_value='false'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('config', default_value=params_file),
            DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg_path, 'rviz', 'map_3d_to_2d.rviz')),
            Node(
                package="map_3d_to_2d",
                executable="mapper",
                name="map_3d_to_2d",
                parameters=[LaunchConfiguration('config'), clock],
                output='screen',
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="map_3d_to_2d_rviz",
                arguments=['-d', LaunchConfiguration('rviz_config')],
                parameters=[clock],
                condition=IfCondition(LaunchConfiguration('rviz')),
            ),
        ])
