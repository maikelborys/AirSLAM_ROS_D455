"""Visual odometry on TartanAir sequences."""
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
    dataroot = LaunchConfiguration('dataroot')
    camera_config_path = LaunchConfiguration('camera_config_path')
    model_dir = LaunchConfiguration('model_dir')
    saving_dir = LaunchConfiguration('saving_dir')
    visualization = LaunchConfiguration('visualization')

    return LaunchDescription([
        DeclareLaunchArgument('config_path',
            default_value=os.path.join(pkg_share, 'configs', 'visual_odometry', 'configs_tartanair.yaml')),
        DeclareLaunchArgument('dataroot', default_value='/media/bssd/datasets/tartanair/euroc_style/with_time/abandonedfactory/P009'),
        DeclareLaunchArgument('camera_config_path',
            default_value=os.path.join(pkg_share, 'configs', 'camera', 'tartanair.yaml')),
        DeclareLaunchArgument('model_dir', default_value=os.path.join(pkg_share, 'output')),
        DeclareLaunchArgument('saving_dir', default_value='/tmp/airslam'),
        DeclareLaunchArgument('visualization', default_value='true'),
        Node(
            package='air_slam', executable='visual_odometry',
            name='visual_odometry', output='screen',
            parameters=[{'config_path': config_path, 'dataroot': dataroot,
                         'camera_config_path': camera_config_path,
                         'model_dir': model_dir, 'saving_dir': saving_dir}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'vo_jazzy.rviz')],
            output='screen', condition=IfCondition(visualization),
        ),
    ])
