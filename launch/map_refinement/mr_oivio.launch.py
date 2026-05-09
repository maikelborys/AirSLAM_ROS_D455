"""Map refinement on OIVIO."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('air_slam')

    config_path = LaunchConfiguration('config_path')
    map_root = LaunchConfiguration('map_root')
    model_dir = LaunchConfiguration('model_dir')
    voc_path = LaunchConfiguration('voc_path')
    breakpoint_arg = LaunchConfiguration('breakpoint')
    visualization = LaunchConfiguration('visualization')

    return LaunchDescription([
        DeclareLaunchArgument('config_path',
            default_value=os.path.join(pkg_share, 'configs', 'map_refinement', 'mr_oivio.yaml')),
        DeclareLaunchArgument('map_root', default_value='/tmp/airslam'),
        DeclareLaunchArgument('model_dir', default_value=os.path.join(pkg_share, 'output')),
        DeclareLaunchArgument('voc_path',
            default_value=os.path.join(pkg_share, 'voc', 'point_voc_L4.bin')),
        DeclareLaunchArgument('breakpoint', default_value='0'),
        DeclareLaunchArgument('visualization', default_value='true'),
        Node(
            package='air_slam', executable='map_refinement',
            name='map_refinement', output='screen',
            parameters=[{'config_path': config_path, 'map_root': map_root,
                         'model_dir': model_dir, 'voc_path': voc_path,
                         'breakpoint': breakpoint_arg}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'map_optimization.rviz')],
            output='screen', condition=IfCondition(visualization),
        ),
    ])
