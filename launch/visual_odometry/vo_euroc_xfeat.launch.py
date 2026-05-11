"""Visual odometry on EuRoC stereo+IMU sequences — XFeat 64-dim + MNN matcher.

This is the sibling of vo_euroc.launch.py with one difference: it points
at configs/visual_odometry/vo_euroc_xfeat.yaml, which selects XFeat as the
feature extractor and MNN as the matcher. Everything else (camera intrinsics,
RViz config, ROS publishers) is shared with the SuperPoint launch so the
two pipelines can be A/B compared on the same sequence.
"""
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
    saving_dir = LaunchConfiguration('saving_dir')
    visualization = LaunchConfiguration('visualization')

    return LaunchDescription([
        DeclareLaunchArgument('config_path',
            default_value=os.path.join(pkg_share, 'configs', 'visual_odometry', 'vo_euroc_xfeat.yaml')),
        DeclareLaunchArgument('dataroot', default_value='/home/maikel/datasets/euroc/MH_03_medium'),
        DeclareLaunchArgument('camera_config_path',
            default_value=os.path.join(pkg_share, 'configs', 'camera', 'euroc.yaml')),
        DeclareLaunchArgument('model_dir', default_value=os.path.join(pkg_share, 'output')),
        DeclareLaunchArgument('saving_dir', default_value='/tmp/airslam_xfeat'),
        DeclareLaunchArgument('visualization', default_value='true'),
        Node(
            package='air_slam_xfeat', executable='visual_odometry',
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
