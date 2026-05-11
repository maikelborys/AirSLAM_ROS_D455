"""Relocalization on TartanAir sequences."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('air_slam_xfeat')

    config_path = LaunchConfiguration('config_path')
    dataroot = LaunchConfiguration('dataroot')
    camera_config_path = LaunchConfiguration('camera_config_path')
    model_dir = LaunchConfiguration('model_dir')
    map_root = LaunchConfiguration('map_root')
    voc_path = LaunchConfiguration('voc_path')
    traj_path = LaunchConfiguration('traj_path')
    visualization = LaunchConfiguration('visualization')

    return LaunchDescription([
        DeclareLaunchArgument('config_path',
            default_value=os.path.join(pkg_share, 'configs', 'relocalization', 'reloc_tartanair.yaml')),
        DeclareLaunchArgument('dataroot', default_value='/media/bssd/datasets/tartanair/mapping_relocalization/relocalization/abandonedfactory/sequences/P000'),
        DeclareLaunchArgument('camera_config_path',
            default_value=os.path.join(pkg_share, 'configs', 'camera', 'tartanair.yaml')),
        DeclareLaunchArgument('model_dir', default_value=os.path.join(pkg_share, 'output')),
        DeclareLaunchArgument('map_root', default_value='/tmp/airslam'),
        DeclareLaunchArgument('voc_path',
            default_value=os.path.join(pkg_share, 'voc', 'point_voc_L4.bin')),
        DeclareLaunchArgument('traj_path', default_value='/tmp/airslam/relocalization.txt'),
        DeclareLaunchArgument('visualization', default_value='true'),
        Node(
            package='air_slam_xfeat', executable='relocalization',
            name='relocalization', output='screen',
            parameters=[{'config_path': config_path, 'dataroot': dataroot,
                         'camera_config_path': camera_config_path,
                         'model_dir': model_dir, 'map_root': map_root,
                         'voc_path': voc_path, 'traj_path': traj_path}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'vo_jazzy.rviz')],
            output='screen', condition=IfCondition(visualization),
        ),
    ])
